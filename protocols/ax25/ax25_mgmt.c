/**
 * @file ax25_mgmt.c
 * @brief AX.25 v2.2 Protocol Management Layer - XID Parameter Negotiation
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3.0
 * @date 2026
 */

#include <string.h>
#include <stdlib.h>

#include "ax25_mgmt.h"
#include "ax25.h"

// Initialize negotiation context
uint8_t ax25_mgmt_init(ax25_mgmt_context_t *ctx) {
    if (!ctx)
        return 1;

    memset(ctx, 0, sizeof(ax25_mgmt_context_t));
    ctx->state = AX25_MGMT_IDLE;

    // Set local defaults (AX.25 v2.2)
    ctx->local_params.full_duplex = false;
    ctx->local_params.selective_reject = true;
    ctx->local_params.implicit_reject = true;
    ctx->local_params.modulo128 = true;
    ctx->local_params.ifield_length = 256;
    // AX.25 v2.2 Appendix C Table C1: default window is 32 for modulo-128,
    // 7 for modulo-8. Using 7 here with modulo-128 caps throughput unnecessarily.
    ctx->local_params.window_size = 32;
    ctx->local_params.ack_timer = 3000;
    ctx->local_params.retries = 10;
    ctx->local_params.response_delay_timer = 500;

    // initialize new MDL-ERROR fields
    ctx->max_retries = 3;      // NM201 default per AX.25 v2.2 Appendix C5
    ctx->on_mdl_error = NULL;   // caller sets after init if MDL-ERROR reporting needed
    ctx->user_data = NULL;   // caller sets after init
    ctx->transmit = NULL;   // set by ax25_mgmt_start_negotiation
    // initialize on_mdl_negotiated callback to NULL
    // Caller sets after init if MDL-NEGOTIATE confirm notification is needed.
    ctx->on_mdl_negotiated = NULL;

    return 0;
}

// Returns true if XID was successfully encoded and passed to transmit callback,
// false if encoding failed (e.g. malloc failure on embedded target).
static bool ax25_mgmt_send_xid_command(ax25_mgmt_context_t *ctx) {
    if (!ctx || !ctx->transmit)
        return false;

    ax25_exchange_identification_frame_t xid;
    xid.base.base.header.destination = ctx->peer;
    xid.base.base.header.source = ctx->local;
    xid.base.base.header.cr = true;
    xid.base.base.type = AX25_FRAME_UNNUMBERED_XID;
    xid.base.pf = true;
    xid.fi = 0x82;
    xid.gi = 0x80;

    uint8_t num_params = 0;
    ax25_xid_parameter_t *params[8];
    uint8_t err;

    params[num_params++] = ax25_xid_class_of_procedures_new(
    true, ctx->local_params.full_duplex,
    false, false, false, false, false, 0, &err);

    params[num_params++] = ax25_xid_hdlc_optional_functions_new(
    true, ctx->local_params.implicit_reject, ctx->local_params.selective_reject,
    true, ctx->local_params.modulo128,
    true, true, true, true, true, true, true,
    true, ctx->local_params.modulo128,
    false, false, false, false, false, false, false, 0,
    false, &err);

    // ifield_length is stored in bytes internally; AX.25 v2.2 §4.3.3.7
    // requires the PI=6 wire value to be in bits, so multiply by 8.
    params[num_params++] = ax25_xid_big_endian_new(
    XID_PI_IFIELD_LENGTH_RX, (uint32_t) ctx->local_params.ifield_length * 8u, 2, &err);

    params[num_params++] = ax25_xid_big_endian_new(
    XID_PI_WINDOW_SIZE_RX, ctx->local_params.window_size, 1, &err);

    params[num_params++] = ax25_xid_big_endian_new(
    XID_PI_ACK_TIMER, ctx->local_params.ack_timer, 2, &err);

    params[num_params++] = ax25_xid_big_endian_new(
    XID_PI_RETRIES, ctx->local_params.retries, 1, &err);

    params[num_params++] = ax25_xid_big_endian_new(
    XID_PI_RESP_DELAY_TIMER, ctx->local_params.response_delay_timer, 2, &err);

    xid.parameters = params;
    xid.param_count = num_params;

    size_t len;
    bool sent = false;
    uint8_t *encoded = ax25_exchange_identification_frame_encode(&xid, &len, &err);
    if (encoded) {
        ctx->transmit(encoded, len);
        if (encoded != NULL) {
            free(encoded);
            encoded = NULL;
        }

        sent = true;
    }

    for (uint8_t i = 0; i < num_params; i++) {
        if (params[i]) {
            if (params[i]->free) {
                params[i]->free(params[i], &err);
            }
        }
    }

    return sent;
}

uint8_t ax25_mgmt_start_negotiation(ax25_mgmt_context_t *ctx, ax25_address_t *dest, ax25_address_t *src, void (*transmit)(uint8_t*, size_t)) {
    if (!ctx || !dest || !src || !transmit)
        return 1;
    if (ctx->state != AX25_MGMT_IDLE)
        return 2;  // Already negotiating

    ctx->peer = *dest;
    ctx->local = *src;
    ctx->transmit = transmit;
    // Use UINT32_MAX as sentinel: TM201 is armed on the first ax25_mgmt_tick call,
    // not from epoch zero. This prevents immediate TM201 expiry on running systems.
    ctx->timeout_tick = UINT32_MAX;
    ctx->state = AX25_MGMT_AWAITING_RESPONSE;
    ctx->retry_count = 0;

    // If initial XID cannot be encoded or transmitted, abort immediately.
    // Do not stay in AWAITING_RESPONSE with no XID sent.
    if (!ax25_mgmt_send_xid_command(ctx)) {
        ctx->state = AX25_MGMT_IDLE;
        return 3;
    }

    return 0;
}

// Process received XID (command or response)
uint8_t ax25_mgmt_process_xid(ax25_mgmt_context_t *ctx, ax25_exchange_identification_frame_t *xid, void (*transmit)(uint8_t*, size_t)) {
    if (!ctx || !xid || !transmit)
        return 1;

    // Parse received parameters
    ax25_negotiated_params_t remote;
    memset(&remote, 0, sizeof(remote));

    // Defaults
    remote.full_duplex = false;
    remote.selective_reject = false;
    remote.implicit_reject = false;
    remote.modulo128 = false;
    remote.ifield_length = 256;
    remote.window_size = 4;  // AX.25 v2.0 default
    remote.ack_timer = 3000;
    remote.retries = 10;
    remote.response_delay_timer = 500;  // T2 default

    // Parse each parameter
    for (size_t i = 0; i < xid->param_count; i++) {
        ax25_xid_parameter_t *param = xid->parameters[i];
        if (!param || !param->data)
            continue;

        // Extract PV data from parameter
        ax25_raw_parameter_t *raw = (ax25_raw_parameter_t*) param->data;
        if (!raw || !raw->pv)
            continue;

        switch (param->pi) {
            case XID_PI_CLASS_OF_PROCEDURES:  // 2
                if (raw->pv_len >= 2) {
                    remote.full_duplex = (raw->pv[0] & XID_COP_FULL_DUPLEX) != 0;
                }
            break;

            case XID_PI_HDLC_OPTIONAL_FUNCTIONS:  // 3
                if (raw->pv_len >= 3) {
                    remote.implicit_reject = (raw->pv[0] & XID_HDLC_REJ) != 0;
                    remote.selective_reject = (raw->pv[0] & XID_HDLC_SREJ) != 0;
                    remote.modulo128 = (raw->pv[1] & XID_HDLC_MOD128) != 0;
                }
            break;

            case XID_PI_IFIELD_LENGTH_RX:  // 6
                // Wire value is in bits per AX.25 v2.2 §4.3.3.7; convert to bytes.
                // Guard against zero to avoid a zero ifield_length in agreed params.
                if (raw->pv_len == 2) {
                    uint16_t bits = (uint16_t) ((raw->pv[0] << 8) | raw->pv[1]);
                    remote.ifield_length = (bits > 0u) ? (bits / 8u) : 256u;
                }
            break;

            case XID_PI_WINDOW_SIZE_RX:  // 8
                if (raw->pv_len == 1) {
                    remote.window_size = raw->pv[0];
                }
            break;

            case XID_PI_ACK_TIMER:  // 9
                if (raw->pv_len == 2) {
                    remote.ack_timer = (raw->pv[0] << 8) | raw->pv[1];
                }
            break;

            case XID_PI_RETRIES:  // 10
                if (raw->pv_len == 1) {
                    remote.retries = raw->pv[0];
                }
            break;

            case XID_PI_RESP_DELAY_TIMER:  // 11
                if (raw->pv_len == 2) {
                    remote.response_delay_timer = (raw->pv[0] << 8) | raw->pv[1];
                }
            break;
        }
    }

    ctx->remote_params = remote;

    // Negotiate parameters (take minimum/intersection)
    ctx->agreed_params.full_duplex = ctx->local_params.full_duplex && remote.full_duplex;
    ctx->agreed_params.selective_reject = ctx->local_params.selective_reject && remote.selective_reject;
    ctx->agreed_params.implicit_reject = ctx->local_params.implicit_reject && remote.implicit_reject;
    ctx->agreed_params.modulo128 = ctx->local_params.modulo128 && remote.modulo128;
    ctx->agreed_params.ifield_length = (ctx->local_params.ifield_length < remote.ifield_length) ? ctx->local_params.ifield_length : remote.ifield_length;
    // Negotiate window: take minimum of the two offered values, then cap to the
    // protocol maximum for the agreed modulo (127 for mod-128, 7 for mod-8).
    // modulo128 is already agreed above so the cap uses the final negotiated value.
    ctx->agreed_params.window_size = (ctx->local_params.window_size < remote.window_size) ? ctx->local_params.window_size : remote.window_size;
    uint8_t max_window = ctx->agreed_params.modulo128 ? 127u : 7u;
    if (ctx->agreed_params.window_size > max_window)
        ctx->agreed_params.window_size = max_window;
    ctx->agreed_params.ack_timer = (ctx->local_params.ack_timer > remote.ack_timer) ? ctx->local_params.ack_timer : remote.ack_timer;
    // AX.25 v2.2 Section 4.3.3.7 PI=10: retries negotiation uses the GREATER value.
    // Taking the minimum would cause premature link failure under noisy conditions.
    ctx->agreed_params.retries = (ctx->local_params.retries > remote.retries) ? ctx->local_params.retries : remote.retries;

    // Negotiate T2: use maximum (safer) or match local policy. Similar to T1 logic here.
    ctx->agreed_params.response_delay_timer =
            (ctx->local_params.response_delay_timer > remote.response_delay_timer) ? ctx->local_params.response_delay_timer : remote.response_delay_timer;

    // If this was a command, send response
    if (xid->base.base.header.cr) {
        // Send XID response with agreed parameters
        ax25_exchange_identification_frame_t response;
        response.base.base.header.destination = xid->base.base.header.source;
        response.base.base.header.source = xid->base.base.header.destination;
        response.base.base.header.cr = false;  // Response
        response.base.base.type = AX25_FRAME_UNNUMBERED_XID;
        response.base.pf = true;
        response.fi = 0x82;
        response.gi = 0x80;

        uint8_t num_params = 0;
        ax25_xid_parameter_t *params[8];
        uint8_t err;

        // Send agreed parameters back
        // Class of Procedures (PI=2)
        params[num_params++] = ax25_xid_class_of_procedures_new(
        true,  // a_flag: Half-duplex
                ctx->agreed_params.full_duplex,  // b_flag: Full-duplex
                false, false, false, false, false,  // c-g flags: unused
                0,  // reserved
                &err);

        // HDLC Optional Functions (PI=3)
        params[num_params++] = ax25_xid_hdlc_optional_functions_new(
        true,  // RNR: always supported
                ctx->agreed_params.implicit_reject,  // REJ
                ctx->agreed_params.selective_reject,  // SREJ
                true,  // SABM: always supported
                ctx->agreed_params.modulo128,  // SABME
                true,  // DM: always supported
                true,  // DISC: always supported
                true,  // UA: always supported
                true,  // FRMR: always supported
                true,  // UI: always supported
                true,  // XID: always supported
                true,  // TEST: always supported
                true,  // Modulo 8: always supported
                ctx->agreed_params.modulo128,  // Modulo 128
                false, false, false, false, false, false, false,  // reserved flags
                0,  // reserved byte
                false,  // extension bit
                &err);

        // I-field length Rx (PI=6)
        // agreed_params.ifield_length is in bytes; AX.25 v2.2 §4.3.3.7 wire value is bits.
        params[num_params++] = ax25_xid_big_endian_new(
        XID_PI_IFIELD_LENGTH_RX, (uint32_t) ctx->agreed_params.ifield_length * 8u, 2,  // 2 bytes
                &err);

        // Window size Rx (PI=8)
        params[num_params++] = ax25_xid_big_endian_new(
        XID_PI_WINDOW_SIZE_RX, ctx->agreed_params.window_size, 1,  // 1 byte
                &err);

        // Ack timer (PI=9)
        params[num_params++] = ax25_xid_big_endian_new(
        XID_PI_ACK_TIMER, ctx->agreed_params.ack_timer, 2,  // 2 bytes
                &err);

        // Retries (PI=10)
        params[num_params++] = ax25_xid_big_endian_new(
        XID_PI_RETRIES, ctx->agreed_params.retries, 1,  // 1 byte
                &err);

        // Response Delay Timer (PI=11)
        params[num_params++] = ax25_xid_big_endian_new(
        XID_PI_RESP_DELAY_TIMER, ctx->agreed_params.response_delay_timer, 2,  // 2 bytes
                &err);

        response.parameters = params;
        response.param_count = num_params;

        // Encode and send
        size_t len;
        uint8_t *encoded = ax25_exchange_identification_frame_encode(&response, &len, &err);
        if (encoded) {
            transmit(encoded, len);

            if (encoded != NULL) {
                free(encoded);
                encoded = NULL;
            }
        }

        // Free parameter structures
        for (uint8_t i = 0; i < num_params; i++) {
            if (params[i]) {
                if (params[i]->free) {
                    params[i]->free(params[i], &err);
                }
            }
        }
    }

    ctx->state = AX25_MGMT_NEGOTIATED;
    // deliver MDL-NEGOTIATE confirm to upper layer per AX.25 v2.2 Appendix C5.
    // agreed_params are fully populated at this point; upper layer may call
    // ax25_apply_negotiated_params() safely from within this callback.
    if (ctx->on_mdl_negotiated) {
        ctx->on_mdl_negotiated(ctx, ctx->user_data);
    }

    return 0;
}

// Timer tick handler for timeout management
void ax25_mgmt_tick(ax25_mgmt_context_t *ctx, uint32_t current_tick) {
    if (!ctx)
        return;

    if (ctx->state != AX25_MGMT_AWAITING_RESPONSE)
        return;

    // UINT32_MAX sentinel: TM201 not yet armed (first tick after start_negotiation).
    // Arm it here to prevent immediate expiry on systems where current_tick >> 0.
    if (ctx->timeout_tick == UINT32_MAX) {
        ctx->timeout_tick = current_tick;
        return;
    }

    // use T1 value as TM201, minimum 3000 ms per AX.25 v2.2 Appendix C5
    uint32_t tm201 = (ctx->local_params.ack_timer > 3000u) ? (uint32_t) ctx->local_params.ack_timer : 3000u;

    if ((current_tick - ctx->timeout_tick) < tm201)
        return;

    // TM201 has expired
    ctx->retry_count++;
    if (ctx->retry_count >= ctx->max_retries) {
        // NM201 retries exhausted - MDL-ERROR indication code K per AX.25 v2.2 Appendix C5
        ctx->state = AX25_MGMT_IDLE;
        // Clear agreed_params: no negotiation result is valid after timeout.
        // Layer 3 must not read stale partial negotiation data after MDL-ERROR K.
        memset(&ctx->agreed_params, 0, sizeof(ctx->agreed_params));
        if (ctx->on_mdl_error)
            ctx->on_mdl_error(AX25_MDL_ERROR_K, ctx->user_data);
    } else {
        ctx->timeout_tick = current_tick;
        // If retransmit encoding fails on embedded target (e.g. malloc failure),
        // issue MDL-ERROR K immediately rather than waiting NM201 more timeouts.
        if (!ax25_mgmt_send_xid_command(ctx)) {
            ctx->state = AX25_MGMT_IDLE;
            memset(&ctx->agreed_params, 0, sizeof(ctx->agreed_params));
            if (ctx->on_mdl_error)
                ctx->on_mdl_error(AX25_MDL_ERROR_K, ctx->user_data);
        }
    }
}

// bridge FRMR event into MDL state machine
void ax25_mgmt_notify_frmr_received(ax25_mgmt_context_t *ctx) {
    if (!ctx)
        return;
    // Only meaningful while waiting for an XID response
    if (ctx->state != AX25_MGMT_AWAITING_RESPONSE)
        return;
    ctx->state = AX25_MGMT_IDLE;
    // Clear agreed_params: FRMR means peer is AX.25 v2.0, XID not supported.
    // No agreed parameters are valid; Layer 3 must use defaults.
    memset(&ctx->agreed_params, 0, sizeof(ctx->agreed_params));
    // MDL-ERROR code B: FRMR received, peer is AX.25 v2.0 and does not support XID
    if (ctx->on_mdl_error)
        ctx->on_mdl_error(AX25_MDL_ERROR_B, ctx->user_data);
}
