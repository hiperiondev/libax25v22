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

#ifndef AX25_MGMT_H_
#define AX25_MGMT_H_

#include <stdint.h>
#include <stdbool.h>

#include "ax25.h"

// XID Parameter Identifiers - AX.25 v2.2 Section 6.7.2
#define XID_PI_CLASS_OF_PROCEDURES     2  //
#define XID_PI_HDLC_OPTIONAL_FUNCTIONS 3  //
#define XID_PI_IFIELD_LENGTH_RX        6  //
#define XID_PI_WINDOW_SIZE_RX          8  //
#define XID_PI_ACK_TIMER               9  //
#define XID_PI_RETRIES                 10 //
#define XID_PI_RESP_DELAY_TIMER        11 // Response Delay Timer (T2)

// Class of Procedures bits
#define XID_COP_HALF_DUPLEX  0x01 //
#define XID_COP_FULL_DUPLEX  0x02 //
#define XID_COP_NORMAL_RESP  0x40 //
#define XID_COP_ASYNC_RESP   0x80 //

// HDLC Optional Functions bits - byte 0
#define XID_HDLC_RNR    0x01 //
#define XID_HDLC_REJ    0x02 //
#define XID_HDLC_SREJ   0x04 //
#define XID_HDLC_SABM   0x08 //
#define XID_HDLC_SABME  0x10 //
#define XID_HDLC_DM     0x20 //
#define XID_HDLC_DISC   0x40 //
#define XID_HDLC_UA     0x80 //

// HDLC Optional Functions bits - byte 1
#define XID_HDLC_FRMR     0x01 //
#define XID_HDLC_UI       0x02 //
#define XID_HDLC_XID      0x04 //
#define XID_HDLC_TEST     0x08 //
#define XID_HDLC_MOD8     0x10 //
#define XID_HDLC_MOD128   0x20 //

// Negotiated parameters result
typedef struct {
    bool full_duplex;        //
    bool selective_reject;   // SREJ supported
    bool implicit_reject;    // REJ supported
    bool modulo128;          // Extended sequence numbers
    uint16_t ifield_length;  // N1 - max I-field we can receive
    uint8_t window_size;     // k - max outstanding I-frames
    uint16_t ack_timer;      // T1 in milliseconds
    uint8_t retries;         // N2
    uint16_t response_delay_timer;  // T2 in milliseconds
} ax25_negotiated_params_t;

// Management state machine states
typedef enum {
    AX25_MGMT_IDLE = 0,           //
    AX25_MGMT_AWAITING_RESPONSE,  //
    AX25_MGMT_NEGOTIATED          //
} ax25_mgmt_state_t;

typedef struct {
    ax25_mgmt_state_t state;                 //
    ax25_negotiated_params_t local_params;   //
    ax25_negotiated_params_t remote_params;  //
    ax25_negotiated_params_t agreed_params;  //
    uint32_t timeout_tick;                   //
    uint8_t retry_count;                     //
    ax25_address_t peer;                     //
} ax25_mgmt_context_t;

// API
uint8_t ax25_mgmt_init(ax25_mgmt_context_t *ctx);
uint8_t ax25_mgmt_start_negotiation(ax25_mgmt_context_t *ctx, ax25_address_t *dest, ax25_address_t *src, void (*transmit)(uint8_t*, size_t));
uint8_t ax25_mgmt_process_xid(ax25_mgmt_context_t *ctx, ax25_exchange_identification_frame_t *xid, void (*transmit)(uint8_t*, size_t));
void ax25_mgmt_tick(ax25_mgmt_context_t *ctx, uint32_t current_tick);

#endif /* AX25_MGMT_H_ */
