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

// EST frame handler - AX.25 v2.2 Section 4.3.3.8 and 6.4.13
#define TEST_FRAME_MAX_PAYLOAD 256  // Maximum test payload for static buffer

// Modulo arithmetic macros - avoid 64-bit, use 32-bit for safety
#define INC_MOD8(x) (((x) + 1) & 0x07)
#define INC_MOD128(x) (((x) + 1) & 0x7F)
#define INC_MOD(x, m) ((m) == 8 ? INC_MOD8(x) : INC_MOD128(x))

#define MOD_DIFF(a, b, m) (((a) - (b)) & ((m) == 8 ? 0x07 : 0x7F))
#define MOD_LT(a, b, m) (MOD_DIFF(a, b, m) > 0 && MOD_DIFF(a, b, m) < ((m) == 8 ? 4 : 64))

// Start T2 timer instead of sending immediate RR response - AX.25 v2.2 Section 6.7.1.2
// This allows piggybacking acknowledgments on outgoing I-frames
static void start_t2_response(ax25_connection_t *conn, uint32_t current_tick) {
    conn->t2_start_tick = current_tick;
    conn->t2_running = true;
    conn->t2_ack_pending = true;
    conn->t2_pending_nr = conn->vars.vr;
}

// Cancel T2 timer when sending I-frame (ACK is piggybacked)
// This is called whenever we send an I-frame that includes N(R)
static void cancel_t2(ax25_connection_t *conn) {
    conn->t2_running = false;
    conn->t2_ack_pending = false;
}

// Handle UI frame - AX.25 v2.2 Section 6.4.12: UI frames accepted in any state
static void handle_ui_frame(ax25_connection_t *conn, ax25_unnumbered_information_frame_t *ui) {
    // AX.25 v2.2 Section 6.4.12: UI frames can be received in any state
    // No acknowledgment required, no connection needed
    // Passed directly to upper layer if callback exists

    if (conn->callbacks.on_data && ui->payload_len > 0 && ui->payload) {
        // Deliver payload to upper layer
        conn->callbacks.on_data(conn->user_data, ui->payload, ui->payload_len);
    }

    // UI frames can also update peer address if not connected
    // This allows connectionless communication
    if (conn->state == AX25_STATE_DISCONNECTED) {
        conn->peer_addr = ui->base.base.header;
    }
}

static void handle_test_frame(ax25_connection_t *conn, ax25_test_frame_t *test) {
    // AX.25 v2.2 Section 6.4.13: Respond to TEST command with TEST response
    // TEST frames work in any state - no connection required
    if (test->base.base.header.cr) {
        // Received TEST command - send TEST response
        static uint8_t test_response_buffer[TEST_FRAME_MAX_PAYLOAD];

        ax25_test_frame_t response;
        response.base.base.header.destination = test->base.base.header.source;
        response.base.base.header.source = test->base.base.header.destination;
        response.base.base.header.cr = false;  // Response
        response.base.base.type = AX25_FRAME_UNNUMBERED_TEST;
        response.base.pf = test->base.pf;
        response.base.modifier = 0xE3;  // TEST modifier

        // Echo payload back - truncate if exceeds buffer size
        response.payload_len = (test->payload_len > TEST_FRAME_MAX_PAYLOAD) ?
        TEST_FRAME_MAX_PAYLOAD :
                                                                              test->payload_len;
        response.payload = test_response_buffer;
        if (response.payload_len > 0 && test->payload) {
            memcpy(response.payload, test->payload, response.payload_len);
        }

        size_t len;
        uint8_t err;
        uint8_t *encoded = ax25_test_frame_encode(&response, &len, &err);
        if (encoded && conn->callbacks.transmit) {
            conn->callbacks.transmit(conn->user_data, encoded, len);
            free(encoded);
        }
    }
    // TEST response received - application could be notified if callback exists
    // For now, we silently accept TEST responses
}

// Send DM response when receiving frames in disconnected state
static void send_dm(ax25_connection_t *conn, bool pf) {
    ax25_unnumbered_frame_t dm;
    dm.base.header = conn->peer_addr;
    dm.base.header.cr = false;  // Response
    dm.base.type = AX25_FRAME_UNNUMBERED_DM;
    dm.pf = pf;
    dm.modifier = 0x0F;

    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_unnumbered_frame_encode(&dm, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);
        free(encoded);
    }
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

    // Mark this N(S) in bitmap - calculate byte and bit position
    uint8_t byte_idx = missing_ns >> 3;  // Divide by 8
    uint8_t bit_idx = missing_ns & 0x07;  // Modulo 8
    if (byte_idx < 16) {
        conn->srej_bitmap[byte_idx] |= (1U << bit_idx);
    }
}

static bool is_srej_pending(ax25_connection_t *conn, uint8_t ns) {
    uint8_t byte_idx = ns >> 3;
    uint8_t bit_idx = ns & 0x07;
    if (byte_idx < 16) {
        return (conn->srej_bitmap[byte_idx] & (1U << bit_idx)) != 0;
    }
    return false;
}

// Clear SREJ pending for a specific N(S)
static void clear_srej_pending(ax25_connection_t *conn, uint8_t ns) {
    uint8_t byte_idx = ns >> 3;
    uint8_t bit_idx = ns & 0x07;
    if (byte_idx < 16) {
        conn->srej_bitmap[byte_idx] &= ~(1U << bit_idx);
    }
}

// Clear all SREJ state
static void clear_srej_state(ax25_connection_t *conn) {
    conn->srej_exception = false;
    conn->srej_count = 0;
    conn->srej_first_missing = 0;
    memset(conn->srej_bitmap, 0, sizeof(conn->srej_bitmap));
    conn->srej_buffer_count = 0;
}

// Check if we can use SREJ for this connection
static bool can_use_srej(ax25_connection_t *conn) {
    return (conn->rej_mode == AX25_REJ_MODE_SREJ || conn->rej_mode == AX25_REJ_MODE_SREJ_REJ);
}

// Check if we can use REJ for this connection
static bool can_use_rej(ax25_connection_t *conn) {
    return (conn->rej_mode == AX25_REJ_MODE_REJ || conn->rej_mode == AX25_REJ_MODE_SREJ_REJ);
}

// Calculate number of missing frames between V(R) and received N(S)
static uint8_t count_missing_frames(ax25_connection_t *conn, uint8_t received_ns) {
    uint8_t vr = conn->vars.vr;
    uint8_t mod = conn->vars.mod;
    uint8_t mask = (mod == 8) ? 0x07 : 0x7F;

    // Calculate (received_ns - vr) modulo mod
    uint8_t diff = (received_ns - vr) & mask;
    return diff;
}

// Handle out-of-sequence I-frame per AX.25 v2.2 Section 6.4.4
static void handle_out_of_sequence_iframe(ax25_connection_t *conn, ax25_information_frame_t *iframe) {
    uint8_t ns = iframe->ns;
    uint8_t expected = conn->vars.vr;
    uint8_t missing_count = count_missing_frames(conn, ns);

    // Section 6.4.4.3: If REJ exception already pending, don't send SREJ
    if (conn->rej_exception) {
        // REJ already pending - discard frame per Section 6.4.4.1
        return;
    }

    // Section 6.4.4: Check if we should use SREJ or REJ
    bool use_srej = false;

    if (can_use_srej(conn) && !conn->srej_exception && missing_count == 1) {
        // Single missing frame and no SREJ pending - use SREJ
        use_srej = true;
    } else if (can_use_srej(conn) && conn->srej_exception && conn->srej_count < conn->srej_max) {
        // Already in SREJ exception, can send another SREJ if limit not reached
        // Check if we haven't already sent SREJ for this missing frame
        if (!is_srej_pending(conn, expected)) {
            use_srej = true;
        }
    }

    if (use_srej) {
        // Section 6.4.4.2: SREJ recovery

        // Buffer this out-of-sequence frame
        if (conn->srej_buffer_count < AX25_MAX_QUEUE_SIZE) {
            uint8_t buf_idx = conn->srej_buffer_count;
            size_t copy_len = iframe->payload_len;
            if (copy_len > 255)
                copy_len = 255;
            if (copy_len > 0 && iframe->payload) {
                memcpy(conn->srej_buffer[buf_idx], iframe->payload, copy_len);
            }
            conn->srej_buffer_len[buf_idx] = (uint8_t) copy_len;
            conn->srej_buffer_ns[buf_idx] = ns;
            conn->srej_buffer_count++;
        }

        // Set SREJ exception state
        if (!conn->srej_exception) {
            conn->srej_exception = true;
            conn->srej_first_missing = expected;
        }
        conn->srej_count++;

        // Send SREJ - P=1 if first SREJ, P=0 if additional SREJ per Section 6.4.4.2
        bool pf = (conn->srej_count == 1) ? true : false;
        send_srej(conn, expected, pf);

    } else if (can_use_rej(conn) && !conn->rej_exception) {
        // Section 6.4.4.1 or 6.4.4.3: Use REJ (multiple missing frames or SREJ limit reached)

        // If we were in SREJ mode, clear it and fall back to REJ per Section 6.4.4.3
        if (conn->srej_exception) {
            clear_srej_state(conn);
        }

        conn->rej_exception = true;
        send_rej(conn, iframe->pf);

        // Discard all buffered SREJ frames - will be retransmitted
        conn->srej_buffer_count = 0;
    }
    // If neither SREJ nor REJ can be used, just discard the frame
}

// Deliver buffered SREJ frames in sequence
static void deliver_buffered_srej_frames(ax25_connection_t *conn) {
    bool delivered = true;

    while (delivered && conn->srej_buffer_count > 0) {
        delivered = false;

        // Look for frame with N(S) = V(R)
        for (uint8_t i = 0; i < conn->srej_buffer_count; i++) {
            if (conn->srej_buffer_ns[i] == conn->vars.vr) {
                // Deliver this frame to upper layer
                if (conn->callbacks.on_data) {
                    conn->callbacks.on_data(conn->user_data, conn->srej_buffer[i], conn->srej_buffer_len[i]);
                }

                // Advance V(R)
                conn->vars.vr = INC_MOD(conn->vars.vr, conn->vars.mod);

                // Clear SREJ pending for this N(S)
                clear_srej_pending(conn, conn->srej_buffer_ns[i]);

                // Remove from buffer by shifting remaining entries
                for (uint8_t j = i; j < conn->srej_buffer_count - 1; j++) {
                    conn->srej_buffer_ns[j] = conn->srej_buffer_ns[j + 1];
                    conn->srej_buffer_len[j] = conn->srej_buffer_len[j + 1];
                    memcpy(conn->srej_buffer[j], conn->srej_buffer[j + 1], conn->srej_buffer_len[j]);
                }
                conn->srej_buffer_count--;
                conn->srej_count--;

                delivered = true;
                break;  // Restart search from beginning
            }
        }
    }

    // If all buffered frames delivered, clear SREJ exception
    if (conn->srej_buffer_count == 0) {
        conn->srej_exception = false;
        conn->srej_count = 0;
        memset(conn->srej_bitmap, 0, sizeof(conn->srej_bitmap));
    }
}

// Process expected I-frame (N(S) == V(R))
static void process_expected_iframe(ax25_connection_t *conn, ax25_information_frame_t *iframe) {
    // Deliver frame to upper layer
    if (conn->callbacks.on_data) {
        conn->callbacks.on_data(conn->user_data, iframe->payload, iframe->payload_len);
    }

    // Advance V(R)
    conn->vars.vr = INC_MOD(conn->vars.vr, conn->vars.mod);

    // Section 6.4.4.2: Check if this clears SREJ exception
    if (conn->srej_exception) {
        // Check if we received the frame that was SREJ'd
        uint8_t prev_ns = (conn->vars.vr == 0) ? (conn->vars.mod == 8 ? 7 : 127) : (conn->vars.vr - 1);

        // Clear SREJ pending for this N(S)
        clear_srej_pending(conn, prev_ns);

        // Deliver any buffered frames that are now in sequence
        deliver_buffered_srej_frames(conn);
    }

    // Clear REJ exception if we received the expected frame
    if (conn->rej_exception) {
        conn->rej_exception = false;
    }

    // Send acknowledgment - use T2 delay unless P/F bit is set
    if (iframe->pf) {
        send_rr(conn, true);  // Immediate response required for P/F bit
    }
    // For non-P/F frames, T2 timer will send RR after delay (started in tick handler)
}

// Handle received SREJ frame - AX.25 v2.2 Section 6.4.8
static void handle_received_srej(ax25_connection_t *conn, ax25_supervisory_frame_t *sframe) {
    uint8_t nr = sframe->nr;

    // Retransmit the specific frame with N(S) = N(R) of SREJ
    uint8_t idx = conn->tx_queue.head;
    for (uint8_t i = 0; i < conn->tx_queue.count; i++) {
        if (conn->tx_queue.ns[idx] == nr) {
            if (conn->callbacks.transmit) {
                conn->callbacks.transmit(conn->user_data, conn->tx_queue.frames[idx], conn->tx_queue.lengths[idx]);
            }
            break;  // Only retransmit the specific frame
        }
        idx = (idx + 1) % AX25_MAX_QUEUE_SIZE;
    }

    // Reset T1 to prevent timeout while waiting for acknowledgment
    conn->t1_start_tick = 0;
}

// Handle received REJ frame - AX.25 v2.2 Section 6.4.7
static void handle_received_rej(ax25_connection_t *conn, ax25_supervisory_frame_t *sframe) {
    uint8_t nr = sframe->nr;

    // Section 6.4.7: Retransmit from N(R) onwards
    uint8_t idx = conn->tx_queue.head;
    for (uint8_t i = 0; i < conn->tx_queue.count; i++) {
        if (MOD_LT(nr, conn->tx_queue.ns[idx], conn->vars.mod) || nr == conn->tx_queue.ns[idx]) {
            if (conn->callbacks.transmit) {
                conn->callbacks.transmit(conn->user_data, conn->tx_queue.frames[idx], conn->tx_queue.lengths[idx]);
            }
        }
        idx = (idx + 1) % AX25_MAX_QUEUE_SIZE;
    }

    conn->t1_start_tick = 0;
    conn->retry_count = 0;
}

// Internal: Send RNR supervisory frame
static void send_rnr(ax25_connection_t *conn, bool pf) {
    ax25_supervisory_frame_t rnr;
    rnr.base.header = conn->peer_addr;
    rnr.base.type = (conn->vars.mod == 128) ? AX25_FRAME_SUPERVISORY_RNR_16BIT : AX25_FRAME_SUPERVISORY_RNR_8BIT;
    rnr.nr = conn->vars.vr;
    rnr.pf = pf;
    rnr.code = 1;  // RNR code is 01 in bits 2-3

    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_supervisory_frame_encode(&rnr, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);
    }
    free(encoded);
}

// Handle received RNR frame - AX.25 v2.2 Section 6.4.9
static void handle_received_rnr(ax25_connection_t *conn, ax25_supervisory_frame_t *rnr) {
    uint8_t nr = rnr->nr;

    // Acknowledge frames up to N(R)-1 (same as RR processing)
    // Per AX.25 v2.2 Section 6.4.9: frames up to N(R)-1 are acknowledged
    while (conn->tx_queue.count > 0) {
        uint8_t head_ns = conn->tx_queue.ns[conn->tx_queue.head];
        // Check if this frame is acknowledged (N(S) < N(R) modulo arithmetic)
        uint8_t diff = (nr - head_ns) & (conn->vars.mod == 8 ? 0x07 : 0x7F);
        if (diff > 0 && diff < (conn->vars.mod == 8 ? 4 : 64)) {
            free(conn->tx_queue.frames[conn->tx_queue.head]);
            conn->tx_queue.head = (conn->tx_queue.head + 1) % AX25_MAX_QUEUE_SIZE;
            conn->tx_queue.count--;
        } else {
            break;
        }
    }

    // Update V(A) to N(R)
    conn->vars.va = nr;

    // Enter peer busy state per Section 6.4.9
    conn->peer_busy = true;
    conn->rnr_start_tick = conn->t3_start_tick;  // Track when busy started

    // Stop T1 timer (don't retransmit while peer is busy)
    // Per spec: stop transmitting I frames until busy condition clears
    conn->t1_start_tick = 0;

    // Notify upper layer of busy condition
    if (conn->callbacks.on_busy) {
        conn->callbacks.on_busy(conn->user_data, true);
    }

    // If P bit set, need to respond with RNR or RR with F=1
    // Per Section 6.2: response to S frame with P=1 is RR, RNR, or REJ with F=1
    if (rnr->pf) {
        // Respond with appropriate frame based on our busy state
        if (conn->local_busy) {
            send_rnr(conn, true);  // F=1
        } else {
            send_rr(conn, true);   // F=1
        }
    }
}

// Send FRMR frame - AX.25 v2.2 Section 4.4.5
// reason: FRMR_W | FRMR_X | FRMR_Y | FRMR_Z bits
// is_response: true if rejected frame was a response, false if command
static void send_frmr(ax25_connection_t *conn, uint16_t bad_control, uint8_t reason, bool is_response) {
    ax25_frame_reject_frame_t frmr;

    // Initialize base frame
    frmr.base.base.header = conn->peer_addr;
    frmr.base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
    frmr.base.pf = true;  // F bit set to 1 per Section 4.4.5
    frmr.base.modifier = 0x87;  // FRMR control field

    // Set FRMR specific fields
    frmr.is_modulo128 = (conn->vars.mod == 128);
    frmr.frmr_control = bad_control;
    frmr.vs = conn->vars.vs;
    frmr.vr = conn->vars.vr;
    frmr.frmr_cr = is_response;
    frmr.w = (reason & FRMR_W) != 0;
    frmr.x = (reason & FRMR_X) != 0;
    frmr.y = (reason & FRMR_Y) != 0;
    frmr.z = (reason & FRMR_Z) != 0;

    // Encode and send
    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_frame_reject_frame_encode(&frmr, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);
    }
    free(encoded);

    // Store FRMR info for potential retransmission per Section 4.4.5
    // Info field format per AX.25 v2.2 Section 4.3.3.6:
    // Modulo-8 (3 bytes): [control][V(R)|C/R|V(S)|0][0|0|0|0|Z|Y|X|W]
    // Modulo-128 (5 bytes): [control-low][control-high][N(S)|0][N(R)|C/R][0|0|0|0|Z|Y|X|W]
    if (conn->vars.mod == 128) {
        conn->frmr_info[0] = bad_control & 0xFF;           // Control low byte
        conn->frmr_info[1] = (bad_control >> 8) & 0xFF;    // Control high byte
        conn->frmr_info[2] = (conn->vars.vs & 0x7F) << 1;  // N(S) in bits 1-7
        conn->frmr_info[3] = ((conn->vars.vr & 0x7F) << 1) | (is_response ? 0x01 : 0x00);  // N(R) in bits 1-7, C/R in bit 0
        conn->frmr_info[4] = reason & 0x0F;                // W,X,Y,Z in bits 0-3
        conn->frmr_info_len = 5;
    } else {
        conn->frmr_info[0] = bad_control & 0xFF;           // Control byte
        // Byte 1: V(R) in bits 5-7, C/R in bit 4, V(S) in bits 1-3, bit 0 = 0
        conn->frmr_info[1] = ((conn->vars.vr & 0x07) << 5) | (is_response ? 0x10 : 0x00) | ((conn->vars.vs & 0x07) << 1);
        conn->frmr_info[2] = reason & 0x0F;                // W,X,Y,Z in bits 0-3
        conn->frmr_info_len = 3;
    }

    conn->frmr_pending = true;
    conn->frmr_retry_count = 0;

    // Enter frame reject state per Section 4.4.5
    conn->state = AX25_STATE_FRAME_REJECT;

    // Start T1 timer for FRMR retransmission
    conn->t1_start_tick = 0;  // Will be set by tick handler
}

// Handle received FRMR frame - AX.25 v2.2 Section 4.4.5
// When we receive FRMR, we must reset the link by sending SABM/SABME
static void handle_received_frmr(ax25_connection_t *conn, ax25_frame_reject_frame_t *frmr) {
    // FRMR received - link reset required per Section 4.4.5
    // The station receiving FRMR should reset the link by sending SABM/SABME

    // Notify upper layer of the error condition
    if (conn->callbacks.on_disconnect) {
        conn->callbacks.on_disconnect(conn->user_data, 2);  // 2 = FRMR received (link reset required)
    }

    // Reset connection state
    conn->state = AX25_STATE_DISCONNECTED;
    conn->vars.vs = conn->vars.vr = conn->vars.va = 0;
    conn->peer_busy = false;
    conn->local_busy = false;
    conn->frmr_pending = false;
    conn->frmr_retry_count = 0;

    // Clear all pending frames
    while (conn->tx_queue.count > 0) {
        free(conn->tx_queue.frames[conn->tx_queue.head]);
        conn->tx_queue.head = (conn->tx_queue.head + 1) % AX25_MAX_QUEUE_SIZE;
        conn->tx_queue.count--;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    conn->peer_busy = false;               // Initialize peer busy state
    conn->local_busy = false;              // Initialize local busy state
    conn->rnr_start_tick = 0;              // Initialize RNR tracking
    conn->frmr_pending = false;            // No FRMR pending
    conn->frmr_retry_count = 0;            // Clear FRMR retry counter

    // Initialize SREJ state per AX.25 v2.2 defaults
    conn->rej_mode = AX25_REJ_MODE_SREJ_REJ;  // Default per Section 6.3.2
    conn->srej_max = 1;                       // Typically 1 per specification
    conn->srej_exception = false;
    conn->rej_exception = false;
    conn->srej_count = 0;
    conn->srej_buffer_count = 0;
    memset(conn->srej_bitmap, 0, sizeof(conn->srej_bitmap));

    // T2 timer state initialization
    conn->t2_running = false;              // T2 timer not running initially
    conn->t2_ack_pending = false;          // No pending ACK
    conn->t2_pending_nr = 0;               // Clear pending N(R)

    return 0;
}

// Process received I-frame - AX.25 v2.2 Section 6.4.4
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
            conn->t1_start_tick = conn->t3_start_tick;
        }
    }

    // Process received sequence number
    if (ns == conn->vars.vr) {
        // Expected frame - process normally
        process_expected_iframe(conn, iframe);

    } else if (MOD_LT(conn->vars.vr, ns, conn->vars.mod)) {
        // Future frame - out of sequence, handle per Section 6.4.4
        handle_out_of_sequence_iframe(conn, iframe);

    } else {
        // Duplicate or old frame - ignore but acknowledge per Section 6.4.4
        // Use T2 delay for duplicate frame ACK unless P/F set
        if (pf) {
            send_rr(conn, true);  // Immediate response for P/F bit
        }
        // For non-P/F, T2 timer will send RR after delay
    }

    // Start T2 timer if not already running and ACK not pending
    // This is called after processing the I-frame to start the response delay
    if (!conn->t2_running && !conn->t2_ack_pending && !iframe->pf) {
        start_t2_response(conn, conn->t3_start_tick);  // Use current tick from last T3 update
    }
}

// Timer tick handler - call every 10ms
void ax25_tick(ax25_connection_t *conn, uint32_t current_tick_10ms) {
    if (!conn)
        return;

    // T2 Response Delay Timer - AX.25 v2.2 Section 6.7.1.2
    // Allows piggybacking of acknowledgments on I-frames to improve efficiency
    if (conn->t2_running && conn->state == AX25_STATE_CONNECTED) {
        // T2 value is already in 1/10 second units (same as tick resolution)
        if ((current_tick_10ms - conn->t2_start_tick) >= conn->timers.t2) {
            // T2 expired - send pending acknowledgment
            if (conn->t2_ack_pending) {
                send_rr(conn, false);
                conn->t2_ack_pending = false;
            }
            conn->t2_running = false;
        }
    }

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

    // Handle FRMR retransmission per AX.25 v2.2 Section 4.4.5
    // FRMR is retransmitted N2 times if no SABM/DISC/DM received
    if (conn->frmr_pending && conn->state == AX25_STATE_FRAME_REJECT) {
        // Check if T1 expired for FRMR retransmission
        if (conn->t1_start_tick != 0 && (uint16_t) (current_tick_10ms - conn->t1_start_tick) >= conn->timers.t1) {
            if (conn->frmr_retry_count >= conn->timers.n2) {
                // Max FRMR retries exceeded - reset link
                conn->state = AX25_STATE_DISCONNECTED;
                conn->frmr_pending = false;
                if (conn->callbacks.on_disconnect) {
                    conn->callbacks.on_disconnect(conn->user_data, 3);  // 3 = FRMR retry exceeded
                }
                return;
            }

            // Retransmit FRMR with same info field per Section 4.4.5
            ax25_frame_reject_frame_t frmr;
            frmr.base.base.header = conn->peer_addr;
            frmr.base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
            frmr.base.pf = true;
            frmr.base.modifier = 0x87;
            frmr.is_modulo128 = (conn->frmr_info_len == 5);

            // Reconstruct FRMR fields from stored info
            if (conn->frmr_info_len == 5) {
                frmr.frmr_control = conn->frmr_info[0] | (conn->frmr_info[1] << 8);
                frmr.vs = (conn->frmr_info[2] >> 1) & 0x7F;
                frmr.vr = (conn->frmr_info[3] >> 1) & 0x7F;
                frmr.frmr_cr = (conn->frmr_info[3] & 0x01) != 0;
            } else {
                frmr.frmr_control = conn->frmr_info[0];
                frmr.vs = (conn->frmr_info[1] >> 1) & 0x07;
                frmr.vr = (conn->frmr_info[1] >> 5) & 0x07;
                frmr.frmr_cr = (conn->frmr_info[1] & 0x10) != 0;
            }
            frmr.w = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_W) != 0;
            frmr.x = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_X) != 0;
            frmr.y = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_Y) != 0;
            frmr.z = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_Z) != 0;

            size_t len;
            uint8_t err;
            uint8_t *encoded = ax25_frame_reject_frame_encode(&frmr, &len, &err);
            if (encoded && conn->callbacks.transmit) {
                conn->callbacks.transmit(conn->user_data, encoded, len);
            }
            free(encoded);

            conn->frmr_retry_count++;
            conn->t1_start_tick = current_tick_10ms;
        }
    }

    // Handle T3 - Inactive link timer
    // Modified per AX.25 v2.2 Section 6.4.9: poll with RR when peer is busy
    if (conn->state == AX25_STATE_CONNECTED && conn->t1_start_tick == 0 && conn->t3_start_tick != 0
            && (uint16_t) (current_tick_10ms - conn->t3_start_tick) >= conn->timers.t3) {

        if (conn->peer_busy) {
            // Peer is busy - poll with RR command P=1 to check status
            // Per Section 6.4.9: send RR or RNR with P bit set
            send_rr(conn, true);
            conn->t1_start_tick = current_tick_10ms;  // Start T1 for response
            // Keep peer_busy true until we get RR response
        } else {
            // Normal T3 operation - poll remote with RR P=1
            send_rr(conn, true);
            conn->t1_start_tick = current_tick_10ms;  // Start T1 for response
        }
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

            // Clear SREJ/REJ state on new connection
            clear_srej_state(conn);
            conn->rej_exception = false;

            // Clear peer busy on SABM/SABME per AX.25 v2.2 Section 4.3.2.2
            if (conn->peer_busy) {
                conn->peer_busy = false;
                conn->rnr_start_tick = 0;
                if (conn->callbacks.on_busy) {
                    conn->callbacks.on_busy(conn->user_data, false);
                }
            }

            // Clear FRMR state on SABM/SABME per Section 4.4.5
            conn->frmr_pending = false;
            conn->frmr_retry_count = 0;

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
            // In frame reject state, discard I frames per Section 4.4.5
            if (conn->state != AX25_STATE_FRAME_REJECT) {
                ax25_process_iframe(conn, (ax25_information_frame_t*) frame);
            }
            // Note: P/F bit is still examined in frame reject state
        break;

        case AX25_FRAME_SUPERVISORY_RR_8BIT:
        case AX25_FRAME_SUPERVISORY_RR_16BIT: {
            ax25_supervisory_frame_t *sframe = (ax25_supervisory_frame_t*) frame;

            // In frame reject state, discard S frames per Section 4.4.5
            // Except for examining P/F bit
            if (conn->state == AX25_STATE_FRAME_REJECT) {
                // If P=1, respond with FRMR per Section 4.4.5
                if (sframe->pf) {
                    // Resend FRMR with same info field
                    if (conn->frmr_pending) {
                        ax25_frame_reject_frame_t frmr;
                        frmr.base.base.header = conn->peer_addr;
                        frmr.base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
                        frmr.base.pf = true;
                        frmr.base.modifier = 0x87;
                        frmr.is_modulo128 = (conn->frmr_info_len == 5);

                        if (conn->frmr_info_len == 5) {
                            frmr.frmr_control = conn->frmr_info[0] | (conn->frmr_info[1] << 8);
                        } else {
                            frmr.frmr_control = conn->frmr_info[0];
                        }
                        // Set other fields from stored info...
                        frmr.w = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_W) != 0;
                        frmr.x = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_X) != 0;
                        frmr.y = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_Y) != 0;
                        frmr.z = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_Z) != 0;

                        size_t len;
                        uint8_t err;
                        uint8_t *encoded = ax25_frame_reject_frame_encode(&frmr, &len, &err);
                        if (encoded && conn->callbacks.transmit) {
                            conn->callbacks.transmit(conn->user_data, encoded, len);
                        }
                        free(encoded);
                    }
                }
                break;
            }

            // Process acknowledgment
            uint8_t nr = sframe->nr;
            while (conn->tx_queue.count > 0 && MOD_LT(conn->tx_queue.ns[conn->tx_queue.head], nr, conn->vars.mod)) {
                free(conn->tx_queue.frames[conn->tx_queue.head]);
                conn->tx_queue.head = (conn->tx_queue.head + 1) % AX25_MAX_QUEUE_SIZE;
                conn->tx_queue.count--;
            }
            conn->vars.va = nr;

            // Clear peer busy condition on RR per AX.25 v2.2 Section 4.3.2.2
            if (conn->peer_busy) {
                conn->peer_busy = false;
                conn->rnr_start_tick = 0;
                if (conn->callbacks.on_busy) {
                    conn->callbacks.on_busy(conn->user_data, false);
                }
            }

            if (sframe->pf) {
                // Response to our poll - restart T3
                conn->t3_start_tick = 0;
            }

            // Cancel T1 if all frames acknowledged
            if (conn->tx_queue.count == 0) {
                conn->t1_start_tick = 0;
                conn->retry_count = 0;
            }
            break;
        }

        case AX25_FRAME_SUPERVISORY_RNR_8BIT:
        case AX25_FRAME_SUPERVISORY_RNR_16BIT: {
            ax25_supervisory_frame_t *sframe = (ax25_supervisory_frame_t*) frame;
            // In frame reject state, discard but check P bit
            if (conn->state == AX25_STATE_FRAME_REJECT) {
                if (sframe->pf && conn->frmr_pending) {
                    // Resend FRMR
                    ax25_frame_reject_frame_t frmr;
                    frmr.base.base.header = conn->peer_addr;
                    frmr.base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
                    frmr.base.pf = true;
                    frmr.base.modifier = 0x87;
                    frmr.is_modulo128 = (conn->frmr_info_len == 5);
                    if (conn->frmr_info_len == 5) {
                        frmr.frmr_control = conn->frmr_info[0] | (conn->frmr_info[1] << 8);
                    } else {
                        frmr.frmr_control = conn->frmr_info[0];
                    }
                    frmr.w = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_W) != 0;
                    frmr.x = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_X) != 0;
                    frmr.y = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_Y) != 0;
                    frmr.z = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_Z) != 0;

                    size_t len;
                    uint8_t err;
                    uint8_t *encoded = ax25_frame_reject_frame_encode(&frmr, &len, &err);
                    if (encoded && conn->callbacks.transmit) {
                        conn->callbacks.transmit(conn->user_data, encoded, len);
                    }
                    free(encoded);
                }
                break;
            }
            handle_received_rnr(conn, sframe);
            break;
        }

        case AX25_FRAME_SUPERVISORY_REJ_8BIT:
        case AX25_FRAME_SUPERVISORY_REJ_16BIT: {
            ax25_supervisory_frame_t *sframe = (ax25_supervisory_frame_t*) frame;
            // In frame reject state, discard but check P bit
            if (conn->state == AX25_STATE_FRAME_REJECT) {
                if (sframe->pf && conn->frmr_pending) {
                    // Resend FRMR
                    ax25_frame_reject_frame_t frmr;
                    frmr.base.base.header = conn->peer_addr;
                    frmr.base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
                    frmr.base.pf = true;
                    frmr.base.modifier = 0x87;
                    frmr.is_modulo128 = (conn->frmr_info_len == 5);
                    if (conn->frmr_info_len == 5) {
                        frmr.frmr_control = conn->frmr_info[0] | (conn->frmr_info[1] << 8);
                    } else {
                        frmr.frmr_control = conn->frmr_info[0];
                    }
                    frmr.w = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_W) != 0;
                    frmr.x = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_X) != 0;
                    frmr.y = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_Y) != 0;
                    frmr.z = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_Z) != 0;

                    size_t len;
                    uint8_t err;
                    uint8_t *encoded = ax25_frame_reject_frame_encode(&frmr, &len, &err);
                    if (encoded && conn->callbacks.transmit) {
                        conn->callbacks.transmit(conn->user_data, encoded, len);
                    }
                    free(encoded);
                }
                break;
            }
            handle_received_rej(conn, sframe);

            // Clear peer busy condition on REJ per AX.25 v2.2 Section 4.3.2.2
            if (conn->peer_busy) {
                conn->peer_busy = false;
                conn->rnr_start_tick = 0;
                if (conn->callbacks.on_busy) {
                    conn->callbacks.on_busy(conn->user_data, false);
                }
            }
            break;
        }

        case AX25_FRAME_SUPERVISORY_SREJ_8BIT:
        case AX25_FRAME_SUPERVISORY_SREJ_16BIT: {
            ax25_supervisory_frame_t *sframe = (ax25_supervisory_frame_t*) frame;
            // In frame reject state, discard but check P bit
            if (conn->state == AX25_STATE_FRAME_REJECT) {
                if (sframe->pf && conn->frmr_pending) {
                    // Resend FRMR
                    ax25_frame_reject_frame_t frmr;
                    frmr.base.base.header = conn->peer_addr;
                    frmr.base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
                    frmr.base.pf = true;
                    frmr.base.modifier = 0x87;
                    frmr.is_modulo128 = (conn->frmr_info_len == 5);
                    if (conn->frmr_info_len == 5) {
                        frmr.frmr_control = conn->frmr_info[0] | (conn->frmr_info[1] << 8);
                    } else {
                        frmr.frmr_control = conn->frmr_info[0];
                    }
                    frmr.w = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_W) != 0;
                    frmr.x = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_X) != 0;
                    frmr.y = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_Y) != 0;
                    frmr.z = (conn->frmr_info[conn->frmr_info_len - 1] & FRMR_Z) != 0;

                    size_t len;
                    uint8_t err;
                    uint8_t *encoded = ax25_frame_reject_frame_encode(&frmr, &len, &err);
                    if (encoded && conn->callbacks.transmit) {
                        conn->callbacks.transmit(conn->user_data, encoded, len);
                    }
                    free(encoded);
                }
                break;
            }
            handle_received_srej(conn, sframe);
            break;
        }

        case AX25_FRAME_UNNUMBERED_FRMR: {
            // Handle received FRMR
            ax25_frame_reject_frame_t *frmr = (ax25_frame_reject_frame_t*) frame;
            handle_received_frmr(conn, frmr);
            break;
        }

        case AX25_FRAME_UNNUMBERED_DISC: {
            // Disconnect request
            conn->state = AX25_STATE_DISCONNECTED;

            // Clear SREJ/REJ state on disconnect
            clear_srej_state(conn);
            conn->rej_exception = false;

            // Clear peer busy on disconnect
            conn->peer_busy = false;
            conn->local_busy = false;
            conn->rnr_start_tick = 0;

            // Clear FRMR state on DISC per Section 4.4.5
            conn->frmr_pending = false;
            conn->frmr_retry_count = 0;

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

        case AX25_FRAME_UNNUMBERED_TEST: {
            // Handle TEST frame - works in any state per AX.25 v2.2 Section 6.4.13
            ax25_test_frame_t *test = (ax25_test_frame_t*) frame;
            handle_test_frame(conn, test);
            break;
        }

        case AX25_FRAME_UNNUMBERED_INFORMATION: {
            // Handle UI frame - AX.25 v2.2 Section 6.4.12
            ax25_unnumbered_information_frame_t *ui = (ax25_unnumbered_information_frame_t*) frame;
            handle_ui_frame(conn, ui);
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

    // Check if peer is busy - cannot send I frames when peer sent RNR
    // Per AX.25 v2.2 Section 6.4.9: stop transmission of I frames until busy clears
    if (conn->peer_busy)
        return 5;  // Peer busy

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

    // Cancel T2 timer when sending I-frame (ACK is piggybacked in N(R))
    cancel_t2(conn);

    conn->vars.vs = INC_MOD(conn->vars.vs, conn->vars.mod);

    // Start/restart T1
    conn->t1_start_tick = 0;
    conn->retry_count = 0;

    return 0;
}

// Send RNR when local buffers full - AX.25 v2.2 Section 6.4.10
uint8_t ax25_send_rnr(ax25_connection_t *conn) {
    if (!conn || conn->state != AX25_STATE_CONNECTED)
        return 1;

    conn->local_busy = true;

    ax25_supervisory_frame_t rnr;
    rnr.base.header = conn->peer_addr;
    rnr.base.type = (conn->vars.mod == 128) ? AX25_FRAME_SUPERVISORY_RNR_16BIT : AX25_FRAME_SUPERVISORY_RNR_8BIT;
    rnr.nr = conn->vars.vr;
    rnr.pf = false;
    rnr.code = 1;  // RNR code is 01 in bits 2-3

    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_supervisory_frame_encode(&rnr, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);
    }
    free(encoded);

    return 0;
}

// Clear local busy condition - AX.25 v2.2 Section 6.4.10
uint8_t ax25_clear_local_busy(ax25_connection_t *conn) {
    if (!conn || !conn->local_busy)
        return 1;

    conn->local_busy = false;

    // Send RR to indicate we're ready - per Section 6.4.10
    // Use REJ if last frame was not properly received, RR if it was
    // For simplicity, we use RR assuming frames were received properly
    // A more complete implementation would track if REJ is needed
    send_rr(conn, false);

    return 0;
}

// Send UI frame without connection - AX.25 v2.2 Section 6.4.12
// UI frames are connectionless and can be sent/received in any state
uint8_t ax25_send_ui(ax25_address_t *dest, ax25_address_t *src, uint8_t *data, size_t len, uint8_t pid, void (*transmit)(uint8_t*, size_t)) {
    if (!dest || !src || !data || !transmit) {
        return 1;  // Invalid parameters
    }

    if (len == 0) {
        return 2;  // No data to send
    }

    // Build UI frame
    ax25_unnumbered_information_frame_t ui;
    ui.base.base.header.destination = *dest;
    ui.base.base.header.source = *src;
    ui.base.base.header.cr = true;  // Command
    ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
    ui.base.pf = false;
    ui.base.modifier = 0x03;  // UI modifier bits
    ui.pid = pid;
    ui.payload = data;
    ui.payload_len = len;

    size_t encoded_len;
    uint8_t err;
    uint8_t *encoded = ax25_unnumbered_information_frame_encode(&ui, &encoded_len, &err);
    if (!encoded) {
        return 3;  // Encoding failed
    }

    // Transmit the frame
    transmit(encoded, encoded_len);
    free(encoded);

    return 0;  // Success
}
