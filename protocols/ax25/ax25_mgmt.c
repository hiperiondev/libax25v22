/**
 * @file ax25_mgmt.c
 * @brief AX.25 v2.2 Protocol Management Layer - XID Parameter Negotiation
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3.0
 * @date 2026
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#include <string.h>
#include <stdlib.h>
#include "hal.h"

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
    // ack_timer and response_delay_timer set to Linux AX25_DEF_T1/T2 values
    // so XID negotiation defaults match the Linux peer's expectations.
    ctx->local_params.ack_timer = 10000;           // Linux AX25_DEF_T1 = 10000 ms
    ctx->local_params.retries = 10;
    ctx->local_params.response_delay_timer = 3000;  // Linux AX25_DEF_T2 = 3000 ms

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
            hal_mem_free(encoded);
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
    // Remote defaults match Linux AX25_DEF_T* so a silent Linux peer is
    // assumed to use its own defaults, preventing timer mismatch.
    remote.ack_timer = 10000;            // Linux AX25_DEF_T1 = 10000 ms
    remote.retries = 10;
    remote.response_delay_timer = 3000;  // Linux AX25_DEF_T2 = 3000 ms

    // Parse each parameter
    for (size_t i = 0; i < xid->param_count; i++) {
        ax25_xid_parameter_t *param = xid->parameters[i];
        if (!param || !param->data)
            continue;

        // ax25_xid_raw_parameter_new() allocates param->data as ax25_raw_param_data_t:
        //   { size_t pv_len; uint8_t pv[]; }   (flexible array, pv_len FIRST)
        // The previous code cast to ax25_raw_parameter_t:
        //   { uint8_t *pv; size_t pv_len; }    (pointer FIRST)
        // On a 64-bit host size_t is 8 bytes, so raw->pv read the 8-byte pv_len
        // field as a pointer value, e.g. pv_len==2 gave raw->pv=(uint8_t*)2.
        // The !raw->pv guard passed (non-NULL), then raw->pv[0] dereferenced
        // address 0x0000000000000002 - instant segfault on every real XID frame.
        // cast to the correct type ax25_raw_param_data_t and access pv[]
        // directly as the flexible array member (no separate pointer needed).
        ax25_raw_param_data_t *raw = (ax25_raw_param_data_t*) param->data;
        if (!raw)
            continue;

        switch (param->pi) {
            case XID_PI_CLASS_OF_PROCEDURES:  // 2
                if (raw->pv_len >= 1u) {
                    remote.full_duplex = (raw->pv[0] & XID_COP_FULL_DUPLEX) != 0;
                }
            break;

            case XID_PI_HDLC_OPTIONAL_FUNCTIONS:  // 3
                if (raw->pv_len >= 1u) {
                    remote.implicit_reject = (raw->pv[0] & XID_HDLC_REJ) != 0;
                    remote.selective_reject = (raw->pv[0] & XID_HDLC_SREJ) != 0;
                }
                if (raw->pv_len >= 2u) {
                    remote.modulo128 = (raw->pv[1] & XID_HDLC_MOD128) != 0;
                }
            break;

            case XID_PI_IFIELD_LENGTH_RX:  // 6
                // Wire value is in bits per AX.25 v2.2 §4.3.3.7; convert to bytes.
                // Guard against zero to avoid a zero ifield_length in agreed params.
                // Use >= so a peer that sends extra padding bytes in PV is tolerated.
                if (raw->pv_len >= 2u) {
                    uint16_t bits = (uint16_t) (((uint16_t) raw->pv[0] << 8) | (uint16_t) raw->pv[1]);
                    remote.ifield_length = (bits > 0u) ? (uint16_t) (bits / 8u) : 256u;
                }
            break;

            case XID_PI_WINDOW_SIZE_RX:  // 8
                if (raw->pv_len >= 1u) {
                    remote.window_size = raw->pv[0];
                }
            break;

            case XID_PI_ACK_TIMER:  // 9
                if (raw->pv_len >= 2u) {
                    remote.ack_timer = (uint16_t) (((uint16_t) raw->pv[0] << 8) | (uint16_t) raw->pv[1]);
                }
            break;

            case XID_PI_RETRIES:  // 10
                if (raw->pv_len >= 1u) {
                    remote.retries = raw->pv[0];
                }
            break;

            case XID_PI_RESP_DELAY_TIMER:  // 11
                if (raw->pv_len >= 2u) {
                    remote.response_delay_timer = (uint16_t) (((uint16_t) raw->pv[0] << 8) | (uint16_t) raw->pv[1]);
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
    // protocol maximum for the agreed modulo.
    // AX25_K_MAX_MOD128=63 (NOT 127) per PE1CHL §5: values 64..127 create
    // N(S) resequencing ambiguity on mod-128 links. AX25_K_MAX_MOD8=7 per AX.25 v2.2.
    // modulo128 is already agreed above so the cap uses the final negotiated value.
    ctx->agreed_params.window_size = (ctx->local_params.window_size < remote.window_size) ? ctx->local_params.window_size : remote.window_size;
    uint8_t max_window = ctx->agreed_params.modulo128 ? AX25_K_MAX_MOD128 : AX25_K_MAX_MOD8;
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
                hal_mem_free(encoded);
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

// ax25_encode_xid: allocation-free XID info-field encoder.
// start modified part
// Produces FI + GI + GL + PI/PL/PV parameter list into buf[buf_size].
// AX.25 v2.2 §4.3.3.7 / ISO 8885 §4.4.1 XID info-field layout:
//   FI(1) = 0x82 Format Identifier
//   GI(1) = 0x80 Group Identifier (was missing in previous version)
//   GL(2) = Group Length big-endian (bytes of PI/PL/PV only, excludes FI+GI+GL)
//   PI/PL/PV... = parameters
// Uses the correct PI values from ax25_mgmt.h (2,3,6,8,9,10,11).
// Returns number of bytes written, or 0 on buffer overflow.
uint16_t ax25_encode_xid(uint8_t *buf, uint16_t buf_size, uint16_t n1_rx, uint8_t k_rx, uint16_t t1_ms, uint8_t n2, uint16_t t2_ms,
bool mod128, bool full_duplex) {
    uint16_t pos = 0;

    // Validate arguments
    if (!buf || buf_size < XID_INFO_MAX_LEN)
        return 0;

    // FI: Format Identifier = 0x82 (parameter negotiation per ISO 8885)
    buf[pos++] = XID_FI_GFI;

    // GI: Group Identifier = 0x80 (general group per ISO 8885 §4.4.1)
    // Previously missing: ax25_exchange_identification_frame_encode() in ax25.c
    // writes bytes[2]=gi and ax25_exchange_identification_frame_decode() reads
    // data[1]=gi. Without GI the output is one byte short and incompatible with
    // Linux ax25.ko and every compliant AX.25 v2.2 station.
    buf[pos++] = XID_GID_PARAMS;
    // end modified part

    // GL is 2 bytes big-endian per ISO 8885 §4.4.1
    // Reserve two bytes for GL and back-fill both after writing all parameters
    uint16_t gl_pos = pos;
    buf[pos++] = 0x00u;   // GL high byte placeholder
    buf[pos++] = 0x00u;   // GL low byte placeholder

    // PI=2: Class of Procedures (2 bytes PV)
    // Byte 0: bit0 = half-duplex (always offered), bit1 = full-duplex
    // Byte 1: reserved (0x00)
    if (pos + 4u > buf_size)
        return 0;
    buf[pos++] = (uint8_t) XID_PI_CLASS_OF_PROCEDURES;
    buf[pos++] = 2u;
    buf[pos++] = (uint8_t) (XID_COP_HALF_DUPLEX | (full_duplex ? XID_COP_FULL_DUPLEX : 0u));
    buf[pos++] = 0x00u;

    // PI=3: HDLC Optional Functions (3 bytes PV per AX.25 v2.2 §4.3.3.7)
    // Byte 0: RNR(0x01) | REJ(0x02) | SREJ(0x04) | SABM(0x08) | SABME(0x10) | DM(0x20) | DISC(0x40) | UA(0x80)
    // Byte 1: FRMR(0x01) | UI(0x02) | XID(0x04) | TEST(0x08) | MOD8(0x10) | MOD128(0x20)
    // Byte 2: reserved
    if (pos + 5u > buf_size)
        return 0;
    buf[pos++] = (uint8_t) XID_PI_HDLC_OPTIONAL_FUNCTIONS;
    buf[pos++] = 3u;
    buf[pos++] = (uint8_t) (XID_HDLC_RNR | XID_HDLC_REJ | XID_HDLC_SREJ | XID_HDLC_SABM | (mod128 ? XID_HDLC_SABME : 0u) | XID_HDLC_DM | XID_HDLC_DISC
            | XID_HDLC_UA);
    buf[pos++] = (uint8_t) (XID_HDLC_FRMR | XID_HDLC_UI | XID_HDLC_XID | XID_HDLC_TEST | XID_HDLC_MOD8 | (mod128 ? XID_HDLC_MOD128 : 0u));
    buf[pos++] = 0x00u;

    // PI=6: I-Field Length Receive (2 bytes PV, value in bits per spec)
    // Convert octets to bits for the wire encoding
    uint16_t n1_bits = (uint16_t) (n1_rx * 8u);
    if (pos + 4u > buf_size)
        return 0;
    buf[pos++] = (uint8_t) XID_PI_IFIELD_LENGTH_RX;
    buf[pos++] = 2u;
    buf[pos++] = (uint8_t) (n1_bits >> 8);
    buf[pos++] = (uint8_t) (n1_bits & 0xFFu);

    // PI=8: Window Size Receive (1 byte PV)
    if (pos + 3u > buf_size)
        return 0;
    buf[pos++] = (uint8_t) XID_PI_WINDOW_SIZE_RX;
    buf[pos++] = 1u;
    buf[pos++] = k_rx;

    // PI=9: Acknowledge Timer T1 (2 bytes PV, value in milliseconds)
    if (pos + 4u > buf_size)
        return 0;
    buf[pos++] = (uint8_t) XID_PI_ACK_TIMER;
    buf[pos++] = 2u;
    buf[pos++] = (uint8_t) (t1_ms >> 8);
    buf[pos++] = (uint8_t) (t1_ms & 0xFFu);

    // PI=10: Retries N2 (1 byte PV)
    if (pos + 3u > buf_size)
        return 0;
    buf[pos++] = (uint8_t) XID_PI_RETRIES;
    buf[pos++] = 1u;
    buf[pos++] = n2;

    // PI=11: Response Delay Timer T2 (2 bytes PV, value in milliseconds)
    if (pos + 4u > buf_size)
        return 0;
    buf[pos++] = (uint8_t) XID_PI_RESP_DELAY_TIMER;
    buf[pos++] = 2u;
    buf[pos++] = (uint8_t) (t2_ms >> 8);
    buf[pos++] = (uint8_t) (t2_ms & 0xFFu);

    // back-fill GL as 2-byte big-endian per ISO 8885 §4.4.1
    // GL value = bytes written after both GL bytes = pos - gl_pos - 2
    uint16_t gl_val = (uint16_t) (pos - gl_pos - 2u);
    buf[gl_pos] = (uint8_t) (gl_val >> 8);     // GL high byte (always 0 for our payload)
    buf[gl_pos + 1u] = (uint8_t) (gl_val & 0xFFu);  // GL low byte

    return pos;
}

// ax25_decode_xid: allocation-free XID info-field decoder.
// Parses a buffer produced by ax25_encode_xid or any AX.25 v2.2 compliant station.
// Fills *params with decoded values; defaults are applied for missing parameters.
// Unknown PI values are silently skipped per AX.25 v2.2 §4.3.3.7.
// Returns: 0 on success
//          1 if buf or params is NULL, or len < 3
//          2 if FI byte is not XID_FI_GFI (not a parameter-negotiation frame)
uint8_t ax25_decode_xid(const uint8_t *buf, uint16_t len, ax25_negotiated_params_t *params) {
    // start modified part
    // minimum 4 bytes required: FI(1) + GI(1) + GL_hi(1) + GL_lo(1)
    // per ISO 8885 §4.4.1 / AX.25 v2.2 §4.3.3.7
    // Previous check was < 3u which skipped the GI byte, causing GL to be
    // read from buf[1..2] instead of the correct buf[2..3].  When a
    // Linux-produced XID arrived with GI=0x80 at buf[1], the parser computed
    // gl = 0x8000 | buf[2], far exceeding the buffer length, clamped
    // group_end to len, and discarded every parameter silently.
    if (!buf || !params || len < 4u)
        return 1;
    // end modified part

    // Check FI (Format Identifier): must be 0x82 for parameter negotiation
    if (buf[0] != XID_FI_GFI)
        return 2;

    // Apply defaults before parsing so missing parameters keep v2.2 defaults
    params->full_duplex = false;
    params->selective_reject = false;
    params->implicit_reject = false;
    params->modulo128 = false;
    params->ifield_length = 256u;
    params->window_size = 4u;
    // Defaults match Linux AX25_DEF_T* so missing XID parameters assume
    // Linux peer values rather than stale AX.25 v2.2 spec minimums.
    params->ack_timer = 10000u;            // Linux AX25_DEF_T1 = 10000 ms
    params->retries = 10u;
    params->response_delay_timer = 3000u;  // Linux AX25_DEF_T2 = 3000 ms

    // start modified part
    // buf[1] = GI (Group Identifier = 0x80); skip it, do not validate strictly
    // so that future group identifiers are tolerated (unknown GI -> no params).
    // GL is a 2-byte big-endian field at buf[2..3] per ISO 8885 §4.4.1.
    // Previously read from buf[1..2] which consumed the GI byte as GL_hi
    // (= 0x80 = 128), making every valid XID produce a wildly wrong group_end.
    uint16_t gl = (uint16_t) (((uint16_t) buf[2] << 8) | (uint16_t) buf[3]);
    uint16_t group_end = (uint16_t) (4u + gl);
    if (group_end > len)
        group_end = len;
    uint16_t pos = 4u;
    // end modified part

    // Iterate over PI/PL/PV triplets
    while (pos + 2u <= group_end) {
        uint8_t pi = buf[pos];
        uint8_t pl = buf[pos + 1u];
        pos += 2u;

        // Guard: skip if PV extends beyond the group boundary
        if ((uint16_t) (pos + pl) > group_end) {
            pos = group_end;
            break;
        }

        // Decode known parameters; unknown PI silently skipped
        switch (pi) {
            case XID_PI_CLASS_OF_PROCEDURES:
                // PV byte 0: bit1 = full-duplex, bit0 = half-duplex
                if (pl >= 1u)
                    params->full_duplex = (buf[pos] & XID_COP_FULL_DUPLEX) != 0u;
            break;

            case XID_PI_HDLC_OPTIONAL_FUNCTIONS:
                // PV byte 0: REJ(0x02), SREJ(0x04)
                // PV byte 1: MOD128(0x20)
                if (pl >= 1u) {
                    params->implicit_reject = (buf[pos] & XID_HDLC_REJ) != 0u;
                    params->selective_reject = (buf[pos] & XID_HDLC_SREJ) != 0u;
                }
                if (pl >= 2u)
                    params->modulo128 = (buf[pos + 1u] & XID_HDLC_MOD128) != 0u;
            break;

            case XID_PI_IFIELD_LENGTH_RX:
                // Wire value is in bits; convert to octets; guard against zero
                if (pl == 2u) {
                    // UB fix -- uint8_t left-shifted 8 bits is UB when bit 7 is set
                    uint16_t bits = (uint16_t) (((uint16_t) buf[pos] << 8) | (uint16_t) buf[pos + 1u]);
                    params->ifield_length = (bits > 0u) ? (uint16_t) (bits / 8u) : 256u;
                }
            break;

            case XID_PI_WINDOW_SIZE_RX:
                if (pl == 1u)
                    params->window_size = buf[pos];
            break;

            case XID_PI_ACK_TIMER:
                if (pl == 2u) {
                    params->ack_timer = (uint16_t) (((uint16_t) buf[pos] << 8) | (uint16_t) buf[pos + 1u]);
                }
            break;

            case XID_PI_RETRIES:
                if (pl == 1u)
                    params->retries = buf[pos];
            break;

            case XID_PI_RESP_DELAY_TIMER:
                if (pl == 2u) {
                    params->response_delay_timer = (uint16_t) (((uint16_t) buf[pos] << 8) | (uint16_t) buf[pos + 1u]);
                }
            break;

            default:
                // Unknown PI: skip silently per AX.25 v2.2 §4.3.3.7
            break;
        }

        pos += (uint16_t) pl;
    }

    return 0;
}
