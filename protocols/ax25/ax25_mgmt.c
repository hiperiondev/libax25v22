/*
 * Copyright 2026 Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * * Project Site: https://github.com/hiperiondev/libax25v22 *
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
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
    ctx->local_params.selective_reject = true;   // Support SREJ
    ctx->local_params.implicit_reject = true;    // Support REJ
    ctx->local_params.modulo128 = true;          // Support extended
    ctx->local_params.ifield_length = 256;       // N1 default
    ctx->local_params.window_size = 7;           // k default (modulo-8)
    ctx->local_params.ack_timer = 3000;          // T1 = 3 seconds
    ctx->local_params.retries = 10;              // N2 default
    ctx->local_params.response_delay_timer = 500;  // T2 default = 500ms

    return 0;
}

// Start negotiation by sending XID command
uint8_t ax25_mgmt_start_negotiation(ax25_mgmt_context_t *ctx, ax25_address_t *dest, ax25_address_t *src, void (*transmit)(uint8_t*, size_t)) {
    if (!ctx || !dest || !src || !transmit)
        return 1;
    if (ctx->state != AX25_MGMT_IDLE)
        return 2;  // Already negotiating

    ctx->peer = *dest;
    ctx->state = AX25_MGMT_AWAITING_RESPONSE;
    ctx->retry_count = 0;

    // Build XID command frame
    ax25_exchange_identification_frame_t xid;
    xid.base.base.header.destination = *dest;
    xid.base.base.header.source = *src;
    xid.base.base.header.cr = true;  // Command
    xid.base.base.type = AX25_FRAME_UNNUMBERED_XID;
    xid.base.pf = true;
    xid.fi = 0x82;  // Parameter negotiation
    xid.gi = 0x80;  // General group

    // Add parameters
    uint8_t num_params = 0;
    ax25_xid_parameter_t *params[8];
    uint8_t err;

    // Class of Procedures (PI=2) - use specific function
    params[num_params++] = ax25_xid_class_of_procedures_new(
    true,  // a_flag: Half-duplex
            ctx->local_params.full_duplex,  // b_flag: Full-duplex
            false, false, false, false, false,  // c-g flags: unused
            0,  // reserved
            &err);

    // HDLC Optional Functions (PI=3) - use specific function
    params[num_params++] = ax25_xid_hdlc_optional_functions_new(
    true,  // RNR: always supported
            ctx->local_params.implicit_reject,  // REJ
            ctx->local_params.selective_reject,  // SREJ
            true,  // SABM: always supported
            ctx->local_params.modulo128,  // SABME
            true,  // DM: always supported
            true,  // DISC: always supported
            true,  // UA: always supported
            true,  // FRMR: always supported
            true,  // UI: always supported
            true,  // XID: always supported
            true,  // TEST: always supported
            true,  // Modulo 8: always supported
            ctx->local_params.modulo128,  // Modulo 128
            false, false, false, false, false, false, false,  // reserved flags
            0,  // reserved byte
            false,  // extension bit
            &err);

    // I-field length Rx (PI=6) - use big-endian function
    params[num_params++] = ax25_xid_big_endian_new(
    XID_PI_IFIELD_LENGTH_RX, ctx->local_params.ifield_length, 2,  // 2 bytes
            &err);

    // Window size Rx (PI=8) - use big-endian function
    params[num_params++] = ax25_xid_big_endian_new(
    XID_PI_WINDOW_SIZE_RX, ctx->local_params.window_size, 1,  // 1 byte
            &err);

    // Ack timer (PI=9) - use big-endian function
    params[num_params++] = ax25_xid_big_endian_new(
    XID_PI_ACK_TIMER, ctx->local_params.ack_timer, 2,  // 2 bytes
            &err);

    // Retries (PI=10) - use big-endian function
    params[num_params++] = ax25_xid_big_endian_new(
    XID_PI_RETRIES, ctx->local_params.retries, 1,  // 1 byte
            &err);

    // Response Delay Timer (PI=11) - use big-endian function
    params[num_params++] = ax25_xid_big_endian_new(
    XID_PI_RESP_DELAY_TIMER, ctx->local_params.response_delay_timer, 2,  // 2 bytes
            &err);

    xid.parameters = params;
    xid.param_count = num_params;

    // Encode and send
    size_t len;
    uint8_t *encoded = ax25_exchange_identification_frame_encode(&xid, &len, &err);
    if (encoded) {
        transmit(encoded, len);
        free(encoded);
    }

    // Free parameter structures
    for (uint8_t i = 0; i < num_params; i++) {
        if (params[i]) {
            if (params[i]->free) {
                params[i]->free(params[i], &err);
            }
        }
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
                if (raw->pv_len == 2) {
                    remote.ifield_length = (raw->pv[0] << 8) | raw->pv[1];
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
    ctx->agreed_params.window_size = (ctx->local_params.window_size < remote.window_size) ? ctx->local_params.window_size : remote.window_size;
    ctx->agreed_params.ack_timer = (ctx->local_params.ack_timer > remote.ack_timer) ? ctx->local_params.ack_timer : remote.ack_timer;
    ctx->agreed_params.retries = (ctx->local_params.retries < remote.retries) ? ctx->local_params.retries : remote.retries;
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
        params[num_params++] = ax25_xid_big_endian_new(
        XID_PI_IFIELD_LENGTH_RX, ctx->agreed_params.ifield_length, 2,  // 2 bytes
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
            free(encoded);
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
    return 0;
}

// Timer tick handler for timeout management
void ax25_mgmt_tick(ax25_mgmt_context_t *ctx, uint32_t current_tick) {
    if (!ctx)
        return;

    // Check for timeout in awaiting response state
    if (ctx->state == AX25_MGMT_AWAITING_RESPONSE) {
        if ((current_tick - ctx->timeout_tick) > 3000) {  // 3 second timeout
            ctx->retry_count++;
            if (ctx->retry_count >= 3) {
                // Negotiation failed, return to idle
                ctx->state = AX25_MGMT_IDLE;
            } else {
                // Retransmit XID command would go here
                ctx->timeout_tick = current_tick;
            }
        }
    }
}
