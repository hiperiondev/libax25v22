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

#include "ax25_state_machine.h"

// Modulo arithmetic macros - avoid 64-bit, use 32-bit for safety
#define INC_MOD8(x) (((x) + 1) & 0x07)
#define INC_MOD128(x) (((x) + 1) & 0x7F)
#define INC_MOD(x, m) ((m) == 8 ? INC_MOD8(x) : INC_MOD128(x))

#define MOD_DIFF(a, b, m) (((a) - (b)) & ((m) == 8 ? 0x07 : 0x7F))
#define MOD_LT(a, b, m) (MOD_DIFF(a, b, m) > 0 && MOD_DIFF(a, b, m) < ((m) == 8 ? 4 : 64))

uint8_t ax25_connection_init(ax25_connection_t *conn, ax25_callbacks_t *cb, void *user_data) {
    if (!conn || !cb)
        return 1;

    memset(conn, 0, sizeof(ax25_connection_t));
    conn->state = AX25_STATE_DISCONNECTED;  //
    conn->vars.mod = 8;                    // Default to modulo 8
    conn->timers.t1 = 30;                  // 3 seconds default
    conn->timers.t2 = 15;                  // 1.5 seconds default
    conn->timers.t3 = 300;                 // 30 seconds default
    conn->timers.n2 = 10;                  //
    conn->timers.k = 7;                    //
    conn->timers.n1 = 256;                 //
    conn->callbacks = *cb;                 //
    conn->user_data = user_data;           //

    return 0;
}

// Internal: Send SABM/SABME command
static void send_sabm(ax25_connection_t *conn, bool extended) {
    ax25_unnumbered_frame_t sabm;
    sabm.base.header = conn->peer_addr;
    sabm.base.header.destination.ch = true;   // Command
    sabm.base.header.source.ch = false;
    sabm.base.type = extended ? AX25_FRAME_UNNUMBERED_SABME : AX25_FRAME_UNNUMBERED_SABM;
    sabm.pf = true;
    sabm.modifier = extended ? 0x6F : 0x2F;

    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_unnumbered_frame_encode(&sabm, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);
    }
    free(encoded);
}

// Internal: Send RR supervisory frame
static void send_rr(ax25_connection_t *conn, bool pf) {
    ax25_supervisory_frame_t rr;
    rr.base.header = conn->peer_addr;
    rr.base.type = (conn->vars.mod == 128) ? AX25_FRAME_SUPERVISORY_RR_16BIT : AX25_FRAME_SUPERVISORY_RR_8BIT;
    rr.nr = conn->vars.vr;
    rr.pf = pf;
    rr.code = 0;  // RR

    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_supervisory_frame_encode(&rr, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);
    }
    free(encoded);
}

// Internal: Send REJ frame
static void send_rej(ax25_connection_t *conn, bool pf) {
    ax25_supervisory_frame_t rej;
    rej.base.header = conn->peer_addr;
    rej.base.type = (conn->vars.mod == 128) ? AX25_FRAME_SUPERVISORY_REJ_16BIT : AX25_FRAME_SUPERVISORY_REJ_8BIT;
    rej.nr = conn->vars.vr;
    rej.pf = pf;
    rej.code = 2;  // REJ

    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_supervisory_frame_encode(&rej, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);
    }
    free(encoded);
}

// Internal: Send SREJ frame - AX.25 v2.2 Section 6.4.4.2
static void send_srej(ax25_connection_t *conn, uint8_t missing_ns, bool pf) {
    ax25_supervisory_frame_t srej;
    srej.base.header = conn->peer_addr;
    srej.base.type = (conn->vars.mod == 128) ? AX25_FRAME_SUPERVISORY_SREJ_16BIT : AX25_FRAME_SUPERVISORY_SREJ_8BIT;
    srej.nr = missing_ns;
    srej.pf = pf;
    srej.code = 3;  // SREJ

    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_supervisory_frame_encode(&srej, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);
    }
    free(encoded);

    // Track pending SREJ
    if (conn->srej_count < AX25_MAX_QUEUE_SIZE) {
        conn->srej_pending |= (1U << missing_ns);
    }
}

// Process received I-frame
void ax25_process_iframe(ax25_connection_t *conn, ax25_information_frame_t *iframe) {
    if (conn->state != AX25_STATE_CONNECTED && conn->state != AX25_STATE_TIMER_RECOVERY) {
        return;  // Ignore I-frames when not connected
    }

    uint8_t ns = iframe->ns;
    uint8_t nr = iframe->nr;
    bool pf = iframe->pf;

    // Check if this acknowledges our sent frames
    if (MOD_LT(conn->vars.va, nr, conn->vars.mod) || nr == conn->vars.va) {
        // Valid acknowledgment - remove acknowledged frames from queue
        while (conn->tx_queue.count > 0 && MOD_LT(conn->tx_queue.ns[conn->tx_queue.head], nr, conn->vars.mod)) {
            free(conn->tx_queue.frames[conn->tx_queue.head]);
            conn->tx_queue.head = (conn->tx_queue.head + 1) % AX25_MAX_QUEUE_SIZE;
            conn->tx_queue.count--;
        }
        conn->vars.va = nr;

        // Cancel T1 if all frames acknowledged
        if (conn->tx_queue.count == 0) {
            conn->t1_start_tick = 0;
        } else {
            // Restart T1 for remaining frames
            conn->t1_start_tick = conn->t3_start_tick;  // Use current tick
        }
    }

    // Process received sequence number
    if (ns == conn->vars.vr) {
        // Expected frame - deliver to upper layer
        if (conn->callbacks.on_data) {
            conn->callbacks.on_data(conn->user_data, iframe->payload, iframe->payload_len);
        }
        conn->vars.vr = INC_MOD(conn->vars.vr, conn->vars.mod);

        // Check if we have buffered out-of-sequence frames
        while (conn->srej_count > 0) {
            bool found = false;
            for (uint8_t i = 0; i < conn->srej_count; i++) {
                if (conn->srej_buffer_ns[i] == conn->vars.vr) {
                    // Deliver buffered frame
                    if (conn->callbacks.on_data) {
                        conn->callbacks.on_data(conn->user_data, conn->srej_buffer[i], conn->srej_buffer_len[i]);
                    }
                    conn->vars.vr = INC_MOD(conn->vars.vr, conn->vars.mod);

                    // Remove from buffer
                    memmove(&conn->srej_buffer_ns[i], &conn->srej_buffer_ns[i + 1], (conn->srej_count - i - 1) * sizeof(uint8_t));
                    memmove(&conn->srej_buffer_len[i], &conn->srej_buffer_len[i + 1], (conn->srej_count - i - 1) * sizeof(uint8_t));
                    // Note: buffer contents are overwritten, no need to memmove
                    conn->srej_count--;
                    found = true;
                    break;
                }
            }
            if (!found)
                break;
        }

        // Send RR acknowledgment
        send_rr(conn, pf);

    } else if (MOD_LT(conn->vars.vr, ns, conn->vars.mod)) {
        // Future frame - out of sequence
        if (conn->srej_count < AX25_MAX_QUEUE_SIZE && (conn->srej_pending & (1U << ns)) == 0) {
            // Buffer for later reassemble
            size_t copy_len = iframe->payload_len > 255 ? 255 : iframe->payload_len;
            memcpy(conn->srej_buffer[conn->srej_count], iframe->payload, copy_len);
            conn->srej_buffer_len[conn->srej_count] = copy_len;
            conn->srej_buffer_ns[conn->srej_count] = ns;
            conn->srej_count++;

            // Send SREJ for missing frame
            send_srej(conn, conn->vars.vr, true);
        }
    } else {
        // Duplicate or old frame - ignore but ack
        send_rr(conn, pf);
    }
}

// Timer tick handler - call every 10ms
void ax25_tick(ax25_connection_t *conn, uint32_t current_tick_10ms) {
    if (!conn)
        return;

    // Handle T1 - Acknowledgment timer
    if (conn->t1_start_tick != 0 && (uint16_t) (current_tick_10ms - conn->t1_start_tick) >= conn->timers.t1) {

        if (conn->retry_count >= conn->timers.n2) {
            // Max retries exceeded - disconnect
            conn->state = AX25_STATE_DISCONNECTED;
            if (conn->callbacks.on_disconnect) {
                conn->callbacks.on_disconnect(conn->user_data, 1);  // Timeout
            }
            return;
        }

        // Retransmit all unacknowledged frames
        uint8_t idx = conn->tx_queue.head;
        for (uint8_t i = 0; i < conn->tx_queue.count; i++) {
            if (conn->callbacks.transmit) {
                conn->callbacks.transmit(conn->user_data, conn->tx_queue.frames[idx], conn->tx_queue.lengths[idx]);
            }
            idx = (idx + 1) % AX25_MAX_QUEUE_SIZE;
        }

        conn->retry_count++;
        conn->t1_start_tick = current_tick_10ms;
    }

    // Handle T3 - Inactive link timer
    if (conn->state == AX25_STATE_CONNECTED && conn->t1_start_tick == 0 && conn->t3_start_tick != 0
            && (uint16_t) (current_tick_10ms - conn->t3_start_tick) >= conn->timers.t3) {

        // Send RR with P=1 to poll remote
        send_rr(conn, true);
        conn->t1_start_tick = current_tick_10ms;  // Start T1 for response
    }
}

// Main entry point for received frames
void ax25_process_frame(ax25_connection_t *conn, ax25_frame_t *frame) {
    if (!conn || !frame)
        return;

    switch (frame->type) {
        case AX25_FRAME_UNNUMBERED_SABM:
        case AX25_FRAME_UNNUMBERED_SABME: {
            // Connection request
            conn->peer_addr = frame->header;
            conn->vars.vs = conn->vars.vr = conn->vars.va = 0;
            conn->vars.mod = (frame->type == AX25_FRAME_UNNUMBERED_SABME) ? 128 : 8;
            conn->state = AX25_STATE_CONNECTED;

            // Send UA response
            ax25_unnumbered_frame_t ua;
            ua.base.header = conn->peer_addr;
            ua.base.type = AX25_FRAME_UNNUMBERED_UA;
            ua.pf = ((ax25_unnumbered_frame_t*) frame)->pf;
            ua.modifier = 0x63;

            size_t len;
            uint8_t err;
            uint8_t *encoded = ax25_unnumbered_frame_encode(&ua, &len, &err);
            if (encoded && conn->callbacks.transmit) {
                conn->callbacks.transmit(conn->user_data, encoded, len);
            }
            free(encoded);

            if (conn->callbacks.on_connect) {
                conn->callbacks.on_connect(conn->user_data);
            }
            break;
        }

        case AX25_FRAME_INFORMATION_8BIT:
        case AX25_FRAME_INFORMATION_16BIT:
            ax25_process_iframe(conn, (ax25_information_frame_t*) frame);
        break;

        case AX25_FRAME_SUPERVISORY_RR_8BIT:
        case AX25_FRAME_SUPERVISORY_RR_16BIT: {
            ax25_supervisory_frame_t *sframe = (ax25_supervisory_frame_t*) frame;
            // Process acknowledgment
            uint8_t nr = sframe->nr;
            if (MOD_LT(conn->vars.va, nr, conn->vars.mod) || nr == conn->vars.va) {
                while (conn->tx_queue.count > 0 && MOD_LT(conn->tx_queue.ns[conn->tx_queue.head], nr, conn->vars.mod)) {
                    free(conn->tx_queue.frames[conn->tx_queue.head]);
                    conn->tx_queue.head = (conn->tx_queue.head + 1) % AX25_MAX_QUEUE_SIZE;
                    conn->tx_queue.count--;
                }
                conn->vars.va = nr;

                if (conn->tx_queue.count == 0) {
                    conn->t1_start_tick = 0;
                    conn->retry_count = 0;
                }
            }

            if (sframe->pf) {
                // Response to our poll - restart T3
                conn->t3_start_tick = 0;  // Will be set when we send next data
            }
            break;
        }

        case AX25_FRAME_SUPERVISORY_REJ_8BIT:
        case AX25_FRAME_SUPERVISORY_REJ_16BIT: {
            ax25_supervisory_frame_t *sframe = (ax25_supervisory_frame_t*) frame;
            // Retransmit from N(R) onwards
            uint8_t target_ns = sframe->nr;
            uint8_t idx = conn->tx_queue.head;
            for (uint8_t i = 0; i < conn->tx_queue.count; i++) {
                if (MOD_LT(target_ns, conn->tx_queue.ns[idx], conn->vars.mod) || target_ns == conn->tx_queue.ns[idx]) {
                    if (conn->callbacks.transmit) {
                        conn->callbacks.transmit(conn->user_data, conn->tx_queue.frames[idx], conn->tx_queue.lengths[idx]);
                    }
                }
                idx = (idx + 1) % AX25_MAX_QUEUE_SIZE;
            }
            conn->t1_start_tick = 0;  // Reset T1, will restart on next tick
            break;
        }

        case AX25_FRAME_UNNUMBERED_DISC: {
            // Disconnect request
            conn->state = AX25_STATE_DISCONNECTED;

            // Send UA
            ax25_unnumbered_frame_t ua;
            ua.base.header = conn->peer_addr;
            ua.base.type = AX25_FRAME_UNNUMBERED_UA;
            ua.pf = ((ax25_unnumbered_frame_t*) frame)->pf;
            ua.modifier = 0x63;

            uint8_t err;
            size_t len;
            uint8_t *encoded = ax25_unnumbered_frame_encode(&ua, &len, &err);
            if (encoded && conn->callbacks.transmit) {
                conn->callbacks.transmit(conn->user_data, encoded, len);
            }
            free(encoded);

            if (conn->callbacks.on_disconnect) {
                conn->callbacks.on_disconnect(conn->user_data, 0);  // Normal
            }
            break;
        }

        default:
        break;
    }
}

uint8_t ax25_connect(ax25_connection_t *conn, ax25_address_t *dest, ax25_address_t *src) {
    if (!conn || !dest || !src)
        return 1;
    if (conn->state != AX25_STATE_DISCONNECTED)
        return 2;

    conn->peer_addr.destination = *dest;
    conn->peer_addr.source = *src;
    conn->peer_addr.cr = true;

    send_sabm(conn, false);  // Start with modulo 8, can upgrade via XID
    conn->state = AX25_STATE_AWAITING_CONNECTION;
    conn->retry_count = 0;
    conn->t1_start_tick = 0;  // Will be set by tick handler

    return 0;
}

uint8_t ax25_send_data(ax25_connection_t *conn, uint8_t *data, size_t len, uint8_t pid) {
    if (!conn || !data)
        return 1;
    if (conn->state != AX25_STATE_CONNECTED)
        return 2;

    // Check window not exceeded
    if (conn->tx_queue.count >= conn->timers.k)
        return 3;  // Window closed

    // Create I-frame
    ax25_information_frame_t iframe;
    iframe.base.header = conn->peer_addr;
    iframe.base.type = (conn->vars.mod == 128) ? AX25_FRAME_INFORMATION_16BIT : AX25_FRAME_INFORMATION_8BIT;
    iframe.ns = conn->vars.vs;
    iframe.nr = conn->vars.vr;
    iframe.pf = false;
    iframe.pid = pid;
    iframe.payload_len = len > conn->timers.n1 ? conn->timers.n1 : len;
    iframe.payload = data;  // Note: user must keep data valid until acked

    size_t frame_len;
    uint8_t err;
    uint8_t *encoded = ax25_information_frame_encode(&iframe, &frame_len, &err);
    if (!encoded)
        return 4;

    // Queue for retransmission
    uint8_t tail = conn->tx_queue.tail;
    conn->tx_queue.frames[tail] = encoded;
    conn->tx_queue.lengths[tail] = frame_len;
    conn->tx_queue.ns[tail] = conn->vars.vs;
    conn->tx_queue.tail = (tail + 1) % AX25_MAX_QUEUE_SIZE;
    conn->tx_queue.count++;

    // Send immediately
    if (conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, frame_len);
    }

    conn->vars.vs = INC_MOD(conn->vars.vs, conn->vars.mod);

    // Start/restart T1
    conn->t1_start_tick = 0;  // Set by tick handler on next call
    conn->retry_count = 0;

    return 0;
}
