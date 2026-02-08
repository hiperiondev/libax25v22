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

#ifndef AX25_STATE_MACHINE_H_
#define AX25_STATE_MACHINE_H_

#include <stdint.h>
#include <stdbool.h>

#include "ax25.h"

// FRMR reason codes per AX.25 v2.2 Section 4.3.3.6
#define FRMR_W  0x01  // Invalid control field or not implemented
#define FRMR_X  0x02  // Frame with info field not permitted (U/S frame with wrong length)
#define FRMR_Y  0x04  // Info field exceeded max length (N1)
#define FRMR_Z  0x08  // Invalid N(R) received

// AX.25 v2.2 Section 2.5 - Data Link State Machine States
typedef enum {
    AX25_STATE_DISCONNECTED = 0,     //
    AX25_STATE_AWAITING_CONNECTION,  // Sent SABM, waiting UA
    AX25_STATE_AWAITING_RELEASE,     // Sent DISC, waiting UA/DM
    AX25_STATE_CONNECTED,            // Information transfer phase
    AX25_STATE_TIMER_RECOVERY,       // T1 expired, retransmitting
    AX25_STATE_AWAITING_SABM,        // Received SABM, sending UA
    AX25_STATE_AWAITING_DISC,        // Received DISC, sending UA
    AX25_STATE_FRAME_REJECT          // FRMR sent, waiting SABM/DISC
} ax25_link_state_t;

// State variables per AX.25 v2.2 Section 4.2.2
typedef struct {
    uint8_t vs;   // Send state variable (modulo 8 or 128)
    uint8_t vr;   // Receive state variable
    uint8_t va;   // Acknowledge state variable
    uint8_t mod;  // Modulus (8 or 128)
} ax25_state_vars_t;

// Timer configuration - AX.25 v2.2 Section 6.7.1
typedef struct {
    uint16_t t1;  // Acknowledgment timer (1/10 seconds)
    uint16_t t2;  // Response delay timer (1/10 seconds)
    uint16_t t3;  // Inactive link timer (1/10 seconds)
    uint8_t n2;   // Maximum retries (default 10)
    uint8_t k;    // Window size (default 7, max 127)
    uint16_t n1;  // Maximum I-field length (default 256)
} ax25_timers_t;

// Frame queue for retransmission
#define AX25_MAX_QUEUE_SIZE 16
typedef struct {
    uint8_t *frames[AX25_MAX_QUEUE_SIZE];  //
    size_t lengths[AX25_MAX_QUEUE_SIZE];   //
    uint8_t ns[AX25_MAX_QUEUE_SIZE];       // Sequence numbers for tracking
    uint8_t head;                          //
    uint8_t tail;                          //
    uint8_t count;                         //
} ax25_frame_queue_t;

// Callback interface for upper layer
typedef struct {
    void (*on_connect)(void *user_data);                            // Link established
    void (*on_disconnect)(void *user_data, uint8_t reason);         // Link released
    void (*on_data)(void *user_data, uint8_t *data, size_t len);    // I-frame received
    void (*on_busy)(void *user_data, bool busy);                    // Remote busy state
    void (*transmit)(void *user_data, uint8_t *frame, size_t len);  // Send to HDLC
} ax25_callbacks_t;

// SREJ mode configuration - AX.25 v2.2 Section 6.4.4
typedef enum {
    AX25_REJ_MODE_NONE = 0,    // No reject mode negotiated yet
    AX25_REJ_MODE_REJ,         // Implicit reject only
    AX25_REJ_MODE_SREJ,        // Selective reject only
    AX25_REJ_MODE_SREJ_REJ     // Selective reject-reject (SREJ/REJ) - default per spec
} ax25_rej_mode_t;

// Main connection context - modified fields only
typedef struct {
    ax25_link_state_t state;        //
    ax25_state_vars_t vars;         //
    ax25_timers_t timers;           //
    ax25_frame_header_t peer_addr;  // Connected station address
    ax25_frame_queue_t tx_queue;    // Unacknowledged I-frames
    ax25_callbacks_t callbacks;     //
    void *user_data;                //

    // Timer tracking - use uint32_t tick count, not floating point
    uint32_t t1_start_tick;  //
    uint32_t t2_start_tick;  //
    uint32_t t3_start_tick;  //
    uint8_t retry_count;     //
    bool peer_busy;          //
    bool local_busy;         //
    uint32_t rnr_start_tick;  // When peer busy state entered (for T3 polling)

    // SREJ state tracking - AX.25 v2.2 Section 4.4.4 and 6.4.4
    ax25_rej_mode_t rej_mode;       // Negotiated reject mode
    bool srej_exception;            // Currently in SREJ exception state
    uint8_t srej_first_missing;     // First missing frame N(S) that triggered exception
    uint8_t srej_count;             // Number of pending SREJ conditions
    uint8_t srej_max;               // Maximum simultaneous SREJ (typically 1 per spec)

    // Bitmap for tracking which frames we've sent SREJ for
    // For modulo-8: 8 bits, for modulo-128: 128 bits = 16 bytes
    uint8_t srej_bitmap[16];        // Bit set = SREJ sent for this N(S)

    // REJ exception state - mutually exclusive with SREJ per Section 4.4.4
    bool rej_exception;             // REJ condition pending

    // Selective reject buffering - AX.25 v2.2 Section 6.4.4.2
    uint8_t srej_buffer[AX25_MAX_QUEUE_SIZE][256];  // Buffered out-of-seq frames
    uint8_t srej_buffer_len[AX25_MAX_QUEUE_SIZE];   //
    uint8_t srej_buffer_ns[AX25_MAX_QUEUE_SIZE];    // N(S) of buffered frame
    uint8_t srej_buffer_count;                      // Number of buffered frames

    // FRMR state tracking - AX.25 v2.2 Section 4.4.5
    bool frmr_pending;              // FRMR frame needs to be retransmitted
    uint8_t frmr_info[5];           // Stored FRMR info field for retransmission (max 5 bytes for modulo-128)
    uint8_t frmr_info_len;          // Length of stored FRMR info (3 for modulo-8, 5 for modulo-128)
    uint8_t frmr_retry_count;       // FRMR retransmission counter
} ax25_connection_t;

// API functions
uint8_t ax25_connection_init(ax25_connection_t *conn, ax25_callbacks_t *cb, void *user_data);
uint8_t ax25_connect(ax25_connection_t *conn, ax25_address_t *dest, ax25_address_t *src);
uint8_t ax25_disconnect(ax25_connection_t *conn);
uint8_t ax25_send_data(ax25_connection_t *conn, uint8_t *data, size_t len, uint8_t pid);
void ax25_process_frame(ax25_connection_t *conn, ax25_frame_t *frame);
void ax25_tick(ax25_connection_t *conn, uint32_t current_tick_10ms);  // Call every 10ms
// Send RNR when local buffers full - AX.25 v2.2 Section 6.4.10
uint8_t ax25_send_rnr(ax25_connection_t *conn);
// Clear local busy condition - AX.25 v2.2 Section 6.4.10
uint8_t ax25_clear_local_busy(ax25_connection_t *conn);

#endif /* AX25_STATE_MACHINE_H_ */
