/**
 * @file ax25_state_machine.c
 * @brief AX.25 v2.2 Data Link State Machine Implementation
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 */

#include <string.h>
#include <stdlib.h>

#include "fx25.h"
#include "ax25_state_machine.h"
#include "ax25_segmenter.h"

// EST frame handler - AX.25 v2.2 Section 4.3.3.8 and 6.4.13
#define TEST_FRAME_MAX_PAYLOAD 256  // Maximum test payload for static buffer
// Modulo arithmetic macros - avoid 64-bit, use 32-bit for safety
#define AX25_MASK(mod)         ((uint8_t)(((mod) == 128) ? 0x7Fu : 0x07u))
#define INC_MOD(x, mod)        ((uint8_t)(((uint8_t)(x) + 1u) & AX25_MASK(mod)))
#define MOD_DIFF(a, b, mod)    ((uint8_t)(((uint8_t)(a) - (uint8_t)(b)) & AX25_MASK(mod)))
// AX25_OUTSTANDING: outstanding unacknowledged frames = (V(S)-V(A)) mod modulo
#define AX25_OUTSTANDING(vs, va, mod) \
    ((uint8_t)(((uint8_t)(vs) - (uint8_t)(va)) & AX25_MASK(mod)))

#define FIRE_DL_ERROR(conn, code) \
    do { \
        if ((conn)->callbacks.on_dl_error) \
            (conn)->callbacks.on_dl_error((conn)->user_data, (code)); \
    } while (0)

static inline bool ax25_in_window(uint8_t ns, uint8_t vr, uint8_t k, uint8_t mod) {
    uint8_t diff = (uint8_t) (((uint8_t) ns - (uint8_t) vr) & AX25_MASK(mod));
    return (diff > 0u) && (diff <= k);
}

// Restart T3 timer on link activity (frames sent or received)
// Per AX.25 v2.2 Section 6.7.1.3: T3 maintains link integrity during idle periods
static void restart_t3_on_activity(ax25_connection_t *conn, uint32_t current_tick) {
    if (conn->state == AX25_STATE_CONNECTED) {
        // Only restart T3 if T1 is not running (T3 only runs when no outstanding frames)
        if (conn->t1_start_tick == 0) {
            conn->t3_start_tick = current_tick;
        }
    }
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
    // use ax25_frame_encode to include address header in transmitted frame
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &rr, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);

        // Update statistics - S-frame sent
        conn->stats.sframe_sent++;
        if (conn->stats.sframe_sent == 0) {
            conn->stats.sframe_sent = 1;  // Prevent overflow
        }
    }

    if (encoded != NULL) {
        free(encoded);
        encoded = NULL;
    }
}

// Start T2 timer instead of sending immediate RR response - AX.25 v2.2 Section 6.7.1.2
// This allows piggybacking acknowledgments on outgoing I-frames
static void start_t2_response(ax25_connection_t *conn, uint32_t current_tick) {
    // In full-duplex mode both stations can transmit simultaneously so there is no
    // benefit in delaying the acknowledgment to piggyback it on an outgoing I-frame.
    // Send RR immediately and return without starting the T2 timer.
    if (conn->full_duplex) {
        send_rr(conn, false);
        return;
    }

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

// Dispatch received I-frame data to appropriate protocol handler
// AX.25 v2.2 Section 6.5 - Layer 3 Protocol Multiplexing
static void dispatch_to_protocol(ax25_connection_t *conn, uint8_t *data, size_t len, uint8_t pid) {
    if (!conn || !data || len == 0) {
        return;
    }

    // Search for registered handler for this PID
    for (uint8_t i = 0; i < AX25_MAX_PROTOCOL_HANDLERS; i++) {
        if (conn->protocols[i].active && conn->protocols[i].pid == pid) {
            // Found specific handler - dispatch to it
            conn->protocols[i].handler(conn->protocols[i].user_data, data, len, pid);
            return;
        }
    }

    // No specific handler found - try default handler
    if (conn->default_handler) {
        conn->default_handler(conn->default_user_data, data, len, pid);
        return;
    }

    // Fall back to legacy on_data callback for backward compatibility
    if (conn->callbacks.on_data) {
        // pid forwarded per AX.25 v2.2 Appendix D.4 DL-DATA indication.
        // Layer 3 needs PID to demultiplex protocols (IP=0xCC, NET/ROM=0xCF, etc.).
        conn->callbacks.on_data(conn->user_data, data, len, pid);
    }
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

// Deliver buffered SREJ frames in sequence
static void deliver_buffered_srej_frames(ax25_connection_t *conn, uint32_t current_tick) {
    bool delivered = true;

    while (delivered && conn->srej_buffer_count > 0) {
        delivered = false;

        // Look for frame with N(S) = V(R)
        for (uint8_t i = 0; i < conn->srej_buffer_count; i++) {
            if (conn->srej_buffer_ns[i] == conn->vars.vr) {
                // Route segment fragments to reassembler; all others to L3 directly.
                // PID stored at buffer time is authoritative per DL-DATA indication.
                if (conn->srej_buffer_pid[i] == AX25_PID_SEGMENT_FRAGMENT) {
                    ax25_segmenter_receive(&conn->segmenter, conn->srej_buffer[i], conn->srej_buffer_len[i], conn->srej_buffer_pid[i], current_tick);
                } else {
                    dispatch_to_protocol(conn, conn->srej_buffer[i], conn->srej_buffer_len[i], conn->srej_buffer_pid[i]);
                }

                // Advance V(R)
                conn->vars.vr = INC_MOD(conn->vars.vr, conn->vars.mod);

                // Clear SREJ pending for this N(S)
                clear_srej_pending(conn, conn->srej_buffer_ns[i]);

                // Remove from buffer by shifting remaining entries
                for (uint8_t j = i; j < conn->srej_buffer_count - 1; j++) {
                    conn->srej_buffer_ns[j] = conn->srej_buffer_ns[j + 1];
                    conn->srej_buffer_len[j] = conn->srej_buffer_len[j + 1];
                    // Shift PID alongside the other per-frame metadata.
                    conn->srej_buffer_pid[j] = conn->srej_buffer_pid[j + 1];
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

// Handle UI frame - AX.25 v2.2 Section 6.4.12: UI frames accepted in any state
static void handle_ui_frame(ax25_connection_t *conn, ax25_unnumbered_information_frame_t *ui) {
    // AX.25 v2.2 Section 6.4.12: UI frames can be received in any state
    // No acknowledgment required, no connection needed
    // Count UI frames received per connection statistics, even in DISCONNECTED
    // state, since UI is valid in all states per AX.25 v2.2 Section 6.3.7
    conn->stats.uframe_received++;
    if (conn->stats.uframe_received == 0) {
        conn->stats.uframe_received = 1;
    }

    // UI frames can also update peer address if not connected
    // This allows connectionless communication
    if (conn->state == AX25_STATE_DISCONNECTED) {
        conn->peer_addr = ui->base.base.header;
    }

    // DL-UNIT-DATA indication: per AX.25 v2.2 Appendix D.4 and Section 6.5,
    // UI frames must be dispatched through the PID-based protocol multiplexer
    // so registered handlers receive them. Falling back to on_data only when
    // no registered handler exists maintains backward compatibility.
    if (ui->payload_len > 0 && ui->payload) {
        dispatch_to_protocol(conn, ui->payload, ui->payload_len, ui->pid);

    }
}

static void handle_test_frame(ax25_connection_t *conn, ax25_test_frame_t *test, uint32_t current_tick) {
    // AX.25 v2.2 Section 6.4.13: Handle TEST command or response
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

            if (encoded != NULL) {
                free(encoded);
                encoded = NULL;
            }
        }
    } else {
        // Received TEST response - update statistics
        if (conn->test_stats.last_test_tick != 0) {
            // Calculate RTT - avoid overflow
            uint32_t rtt = current_tick - conn->test_stats.last_test_tick;

            // Update statistics
            conn->test_stats.rtt_sum += rtt;
            conn->test_stats.rtt_count++;
            conn->test_stats.test_received++;

            // Clear pending test
            conn->test_stats.last_test_tick = 0;
        }
    }
}

// Internal: Send SABM/SABME command
static void send_sabm(ax25_connection_t *conn, bool extended) {
    ax25_unnumbered_frame_t sabm;
    sabm.base.header = conn->peer_addr;
    // Only set the frame-level CR bit, let header encoder handle ch bits
    sabm.base.header.cr = true;  // This is a command frame
    sabm.base.type = extended ? AX25_FRAME_UNNUMBERED_SABME : AX25_FRAME_UNNUMBERED_SABM;
    sabm.pf = true;
    sabm.modifier = extended ? 0x6F : 0x2F;

    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &sabm, &len, &err);

    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);

        // Update statistics - U-frame sent
        conn->stats.uframe_sent++;
        if (conn->stats.uframe_sent == 0) {
            conn->stats.uframe_sent = 1;  // Prevent overflow
        }
    }
    if (encoded != NULL) {
        free(encoded);
        encoded = NULL;
    }
}

// send DISC command for retransmit
static void send_disc(ax25_connection_t *conn) {
    ax25_unnumbered_frame_t disc;
    disc.base.header = conn->peer_addr;
    disc.base.header.cr = true;   // command frame
    disc.base.type = AX25_FRAME_UNNUMBERED_DISC;
    disc.pf = true;               // P bit set per AX.25 v2.2 Section 4.3.3.3
    disc.modifier = 0x43;         // DISC modifier per AX.25 v2.2 Table 3

    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &disc, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);
        conn->stats.uframe_sent++;
        if (conn->stats.uframe_sent == 0) {
            conn->stats.uframe_sent = 1;
        }
    }
    if (encoded != NULL) {
        free(encoded);
        encoded = NULL;
    }
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
    // use ax25_frame_encode to include address header in transmitted frame
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &rej, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);

        // Update statistics - S-frame sent
        conn->stats.sframe_sent++;
        if (conn->stats.sframe_sent == 0) {
            conn->stats.sframe_sent = 1;  // Prevent overflow
        }
    }

    if (encoded != NULL) {
        free(encoded);
        encoded = NULL;
    }
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
    // use ax25_frame_encode to include address header in transmitted frame
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &srej, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);

        // Update statistics - S-frame sent
        conn->stats.sframe_sent++;
        if (conn->stats.sframe_sent == 0) {
            conn->stats.sframe_sent = 1;  // Prevent overflow
        }
    }

    if (encoded != NULL) {
        free(encoded);
        encoded = NULL;
    }

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

// Calculate number of missing frames between V(R) and received N(S)
static uint8_t count_missing_frames(ax25_connection_t *conn, uint8_t received_ns) {
    uint8_t vr = conn->vars.vr;
    uint8_t mod = conn->vars.mod;
    uint8_t mask = (mod == 8) ? 0x07 : 0x7F;

    uint8_t diff = (uint8_t) ((received_ns - vr) & mask);
    // Clamp diff to window k: frames beyond window are not in-window gaps.
    // Return 0 as sentinel so handle_out_of_sequence_iframe discards the frame.
    if (diff > conn->timers.k) {
        diff = 0u;
    }

    return diff;
}

// Handle out-of-sequence I-frame per AX.25 v2.2 Section 6.4.4
static void handle_out_of_sequence_iframe(ax25_connection_t *conn, ax25_information_frame_t *iframe) {
    uint8_t ns = iframe->ns;
    uint8_t expected = conn->vars.vr;
    uint8_t missing_count = count_missing_frames(conn, ns);
    uint8_t mask = (conn->vars.mod == 8) ? 0x07 : 0x7F;

    // If count_missing_frames returned 0 for ns != vr the frame is outside
    // the receive window - treat identically to the duplicate/discard path.
    if (missing_count == 0u) {
        if (iframe->pf)
            send_rr(conn, false);
        return;
    }

    // Section 6.4.4.3: If REJ exception already pending, don't send SREJ
    if (conn->rej_exception) {
        // REJ already pending - discard frame per Section 6.4.4.1
        return;
    }

    // Three distinct cases replace the original two-flag approach:
    //   (a) !srej_exception && single gap       -> start SREJ
    //   (b) srej_exception && consecutive frame -> buffer only, keep SREJ state
    //   (c) srej_exception && new gap           -> send SREJ for new missing or fall to REJ
    if (can_use_srej(conn) && !conn->srej_exception && missing_count == 1) {
        // Case (a): Single missing frame, no prior SREJ - start SREJ mode
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
            conn->srej_buffer_pid[buf_idx] = iframe->pid;  // store PID for DL-DATA indication on delivery
            conn->srej_buffer_count++;
        }

        conn->srej_exception = true;
        conn->srej_first_missing = expected;
        conn->srej_count = 1;
        // P=1 for first SREJ per Section 6.4.4.2
        send_srej(conn, expected, true);

    } else if (can_use_srej(conn) && conn->srej_exception) {
        // Cases (b) and (c): already in SREJ exception

        // Compute highest N(S) seen so far: last in-order received plus buffered frames
        uint8_t max_ns_seen = (conn->vars.vr == 0) ? ((conn->vars.mod - 1) & mask) : ((conn->vars.vr - 1) & mask);
        for (uint8_t bi = 0; bi < conn->srej_buffer_count; bi++) {
            uint8_t diff = (conn->srej_buffer_ns[bi] - max_ns_seen) & mask;
            if (diff > 0 && diff < ((conn->vars.mod == 8) ? 4 : 64)) {
                max_ns_seen = conn->srej_buffer_ns[bi];
            }
        }
        // Next sequence number we expect after everything seen so far
        uint8_t expected_next = (max_ns_seen + 1) & mask;

        // Silently discard duplicates already in the buffer
        for (uint8_t bi = 0; bi < conn->srej_buffer_count; bi++) {
            if (conn->srej_buffer_ns[bi] == ns) {
                return;
            }
        }

        if (ns == expected_next) {
            // Case (b): consecutive frame after the highest seen - just buffer it.
            // No new gap exists; the pending SREJ for the original missing frame
            // is still valid - do NOT touch the bitmap or srej_exception.
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
                conn->srej_buffer_pid[buf_idx] = iframe->pid;  // store PID for DL-DATA indication on delivery
                conn->srej_buffer_count++;
            }
            // SREJ bitmap unchanged - existing SREJ for missing frame stays pending
        } else {
            // Case (c): new gap at expected_next - decide SREJ or REJ
            uint8_t new_missing = expected_next;
            if (conn->srej_count < conn->srej_max && !is_srej_pending(conn, new_missing)) {
                // SREJ capacity available: send SREJ for the newly missing frame
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
                    conn->srej_buffer_pid[buf_idx] = iframe->pid;  // store PID for DL-DATA indication on delivery
                    conn->srej_buffer_count++;
                }
                conn->srej_count++;
                // P=0 for additional SREJs per Section 6.4.4.2
                send_srej(conn, new_missing, false);
            } else {
                // SREJ capacity exhausted: fall back to REJ per Section 6.4.4.3
                clear_srej_state(conn);
                conn->rej_exception = true;
                send_rej(conn, iframe->pf);
                conn->srej_buffer_count = 0;
            }
        }

    } else if (can_use_rej(conn) && !conn->rej_exception) {
        // Section 6.4.4.1 or 6.4.4.3: no SREJ support or initial multi-frame gap
        conn->rej_exception = true;
        send_rej(conn, iframe->pf);
    }
}

// Process expected I-frame (N(S) == V(R)) - routes PID=0x08 to reassembler
static void process_expected_iframe(ax25_connection_t *conn, ax25_information_frame_t *iframe, uint32_t current_tick) {
    // Route segment fragments to the built-in reassembler per AX.25 v2.2 Appendix C6.
    // All other PIDs go directly to the L3 protocol multiplexer (Section 6.5).
    if (iframe->payload_len > 0 && iframe->payload) {
        if (iframe->pid == AX25_PID_SEGMENT_FRAGMENT) {
            ax25_segmenter_receive(&conn->segmenter, iframe->payload, (uint16_t) iframe->payload_len, iframe->pid, current_tick);
        } else {
            dispatch_to_protocol(conn, iframe->payload, iframe->payload_len, iframe->pid);
        }
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
        deliver_buffered_srej_frames(conn, current_tick);
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

// ax25_nr_is_valid: check N(R) is in [V(A)..V(S)] per AX.25 v2.2 Section 4.2.2
static bool ax25_nr_is_valid(ax25_connection_t *conn, uint8_t nr) {
    uint8_t mask = AX25_MASK(conn->vars.mod);
    uint8_t dist_nr = (uint8_t) ((nr - conn->vars.va) & mask);
    uint8_t dist_vs = (uint8_t) ((conn->vars.vs - conn->vars.va) & mask);
    // N(R)==V(A): no new ack (valid); N(R)==V(S): all frames acked (valid)
    return dist_nr <= dist_vs;
}

// send_frmr_generic: encode and transmit a FRMR frame with arbitrary W/X/Y/Z reason bits.
// Correctly initialises all ax25_frame_reject_frame_t fields (vs, vr, frmr_cr)
// and stores frmr_info in a layout that resend_stored_frmr() can reproduce exactly.
// frmr_info layout per AX.25 v2.2 §4.3.3.6:
//   mod-8  (3 bytes): [ctrl][V(R)<<5 | C/R<<4 | V(S)<<1][W|X|Y|Z]
//   mod-128 (5 bytes): [ctrl_lo][ctrl_hi][V(S)<<1 | C/R][V(R)<<1][W|X|Y|Z]
// Per AX.25 v2.2 Section 4.4.5 - transitions link to AX25_STATE_FRAME_REJECT
static void send_frmr_generic(ax25_connection_t *conn, uint16_t bad_ctrl,
bool w, bool x, bool y, bool z) {
    ax25_frame_reject_frame_t frmr;
    memset(&frmr, 0, sizeof(frmr));
    frmr.base.base.header = conn->peer_addr;
    frmr.base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
    frmr.base.pf = true;       // F=1 per §4.4.5
    frmr.base.modifier = 0x87u;      // FRMR modifier
    frmr.is_modulo128 = (conn->vars.mod == 128);
    frmr.frmr_control = bad_ctrl;
    frmr.vs = conn->vars.vs;
    frmr.vr = conn->vars.vr;
    frmr.frmr_cr = conn->peer_addr.cr;  // C/R bit of the rejected frame
    frmr.w = w;
    frmr.x = x;
    frmr.y = y;
    frmr.z = z;
    // Store FRMR info for accurate retransmission by resend_stored_frmr()
    uint8_t wxyz = (uint8_t) ((w ? FRMR_W : 0u) | (x ? FRMR_X : 0u) | (y ? FRMR_Y : 0u) | (z ? FRMR_Z : 0u));
    if (conn->vars.mod == 128) {
        conn->frmr_info_len = 5;
        conn->frmr_info[0] = (uint8_t) (bad_ctrl & 0xFFu);
        conn->frmr_info[1] = (uint8_t) ((bad_ctrl >> 8) & 0xFFu);
        // byte 2: V(S) in bits 7-1, C/R in bit 0 - mirrors encode byte[3]
        conn->frmr_info[2] = (uint8_t) (((conn->vars.vs & 0x7Fu) << 1) | (conn->peer_addr.cr ? 1u : 0u));
        // byte 3: V(R) in bits 7-1 - mirrors encode byte[4]
        conn->frmr_info[3] = (uint8_t) ((conn->vars.vr & 0x7Fu) << 1);
        conn->frmr_info[4] = wxyz;
    } else {
        conn->frmr_info_len = 3;
        conn->frmr_info[0] = (uint8_t) (bad_ctrl & 0xFFu);
        // byte 1: V(R) in bits 7-5, C/R in bit 4, V(S) in bits 3-1 - mirrors encode byte[2]
        conn->frmr_info[1] = (uint8_t) (((conn->vars.vr & 0x07u) << 5) | (conn->peer_addr.cr ? 0x10u : 0u) | ((conn->vars.vs & 0x07u) << 1));
        conn->frmr_info[2] = wxyz;
    }
    conn->frmr_pending = true;
    conn->frmr_retry_count = 0;
    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_frame_reject_frame_encode(&frmr, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);
        conn->stats.frmr_sent++;
        if (conn->stats.frmr_sent == 0)
            conn->stats.frmr_sent = 1;
    }
    if (encoded != NULL) {
        free(encoded);
    }
    conn->state = AX25_STATE_FRAME_REJECT;
}

// send_frmr_z: FRMR with Z-bit (invalid N(R) received) per §4.3.3.6
static void send_frmr_z(ax25_connection_t *conn, uint8_t bad_ctrl) {
    send_frmr_generic(conn, (uint16_t) bad_ctrl, false, false, false, true);
    // DL-ERROR J: N(R) sequence error
    FIRE_DL_ERROR(conn, AX25_DL_ERROR_J);
}

// send_frmr_y: FRMR with Y-bit (I-field length exceeds N1) per §4.3.3.6
static void send_frmr_y(ax25_connection_t *conn, uint8_t bad_ctrl) {
    send_frmr_generic(conn, (uint16_t) bad_ctrl, false, false, true, false);
    // DL-ERROR G: I-field exceeded maximum length N1
    FIRE_DL_ERROR(conn, AX25_DL_ERROR_G);
}

// send_frmr_w: FRMR with W-bit (invalid/unimplemented control field) per §4.3.3.6
// Forward declaration with unused attribute suppresses -Wunused-function warning.
// Function is kept for AX.25 v2.2 spec completeness (W-bit FRMR per §4.3.3.6).
static void send_frmr_w(ax25_connection_t *conn, uint8_t bad_ctrl) __attribute__((unused));
static void send_frmr_w(ax25_connection_t *conn, uint8_t bad_ctrl) {
    send_frmr_generic(conn, (uint16_t) bad_ctrl, true, false, false, false);
    // DL-ERROR L: control field invalid or not implemented
    FIRE_DL_ERROR(conn, AX25_DL_ERROR_L);
}

// resend_stored_frmr: retransmit the FRMR stored in conn->frmr_info
// Called when peer polls (P=1) while we are in FRAME_REJECT state per §4.4.5
// Correctly reconstructs vs, vr, frmr_cr from the stored byte layout.
static void resend_stored_frmr(ax25_connection_t *conn) {
    if (!conn->frmr_pending)
        return;
    ax25_frame_reject_frame_t frmr;
    memset(&frmr, 0, sizeof(frmr));
    frmr.base.base.header = conn->peer_addr;
    frmr.base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
    frmr.base.pf = true;
    frmr.base.modifier = 0x87u;
    frmr.is_modulo128 = (conn->frmr_info_len == 5);
    if (conn->frmr_info_len == 5) {
        // mod-128 layout: [ctrl_lo][ctrl_hi][V(S)<<1|C/R][V(R)<<1][wxyz]
        frmr.frmr_control = (uint16_t) (conn->frmr_info[0]) | ((uint16_t) (conn->frmr_info[1]) << 8);
        frmr.vs = (conn->frmr_info[2] >> 1) & 0x7F;
        frmr.frmr_cr = (conn->frmr_info[2] & 0x01u) != 0u;
        frmr.vr = (conn->frmr_info[3] >> 1) & 0x7F;
        frmr.w = (conn->frmr_info[4] & FRMR_W) != 0;
        frmr.x = (conn->frmr_info[4] & FRMR_X) != 0;
        frmr.y = (conn->frmr_info[4] & FRMR_Y) != 0;
        frmr.z = (conn->frmr_info[4] & FRMR_Z) != 0;
    } else {
        // mod-8 layout: [ctrl][V(R)<<5|C/R<<4|V(S)<<1][wxyz]
        frmr.frmr_control = conn->frmr_info[0];
        frmr.vr = (conn->frmr_info[1] >> 5) & 0x07u;
        frmr.frmr_cr = (conn->frmr_info[1] & 0x10u) != 0u;
        frmr.vs = (conn->frmr_info[1] >> 1) & 0x07u;
        frmr.w = (conn->frmr_info[2] & FRMR_W) != 0;
        frmr.x = (conn->frmr_info[2] & FRMR_X) != 0;
        frmr.y = (conn->frmr_info[2] & FRMR_Y) != 0;
        frmr.z = (conn->frmr_info[2] & FRMR_Z) != 0;
    }
    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_frame_reject_frame_encode(&frmr, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);
    }
    if (encoded != NULL) {
        free(encoded);
    }
}

// validate N(R), dequeue acked frames, advance V(A)
// Returns false if N(R) invalid (FRMR Z sent); true on success
// Dequeue loop stops when head_ns==N(R) - correct for all k and modulo values
static bool ax25_process_nr(ax25_connection_t *conn, uint8_t nr, uint8_t raw_ctrl) {
    // When the tx queue is empty (V(A)==V(S), no outstanding unacknowledged frames),
    // the remote may piggyback a stale or zero-initialized N(R) in their I-frame.
    // Per AX.25 v2.2 Section 4.4.5, the FRMR Z condition is only meaningful when
    // there are outstanding frames whose acknowledgment range can be violated.
    // With an empty queue there is nothing to dequeue and nothing to validate;
    // attempting to advance V(A) to a stale N(R) would corrupt the send state.
    // Return true immediately so frame processing (e.g., I-frame receive path)
    // continues normally without touching V(A).
    if (conn->tx_queue.count == 0) {
        return true;
    }

    // Validate N(R) in [V(A)..V(S)]
    if (!ax25_nr_is_valid(conn, nr)) {
        send_frmr_z(conn, raw_ctrl);
        return false;
    }
    // Dequeue all frames with N(S) in [V(A), N(R))
    // N(R) is exclusive: acknowledges up to but NOT including N(R)
    while (conn->tx_queue.count > 0) {
        uint8_t head_ns = conn->tx_queue.ns[conn->tx_queue.head];
        if (head_ns == nr) {
            break;  // This frame not yet acknowledged
        }
        free(conn->tx_queue.frames[conn->tx_queue.head]);
        conn->tx_queue.frames[conn->tx_queue.head] = NULL;
        conn->tx_queue.head = (conn->tx_queue.head + 1) % AX25_MAX_QUEUE_SIZE;
        conn->tx_queue.count--;
    }
    // Advance V(A) to N(R)
    conn->vars.va = nr;
    // Stop T1 if all frames acknowledged
    if (conn->tx_queue.count == 0) {
        conn->t1_start_tick = 0;
        conn->retry_count = 0;
    }
    return true;
}

// Handle received SREJ frame - AX.25 v2.2 Section 6.4.8
static void handle_received_srej(ax25_connection_t *conn, ax25_supervisory_frame_t *sframe) {
    uint8_t nr = sframe->nr;

    // Update statistics - S-frame received
    conn->stats.sframe_received++;
    if (conn->stats.sframe_received == 0) {
        conn->stats.sframe_received = 1;  // Prevent overflow
    }

    // Retransmit the specific frame with N(S) = N(R) of SREJ
    uint8_t idx = conn->tx_queue.head;
    for (uint8_t i = 0; i < conn->tx_queue.count; i++) {
        if (conn->tx_queue.ns[idx] == nr) {
            if (conn->callbacks.transmit) {
                conn->callbacks.transmit(conn->user_data, conn->tx_queue.frames[idx], conn->tx_queue.lengths[idx]);

                // Update statistics - I-frame retransmitted
                conn->stats.iframe_retransmitted++;
                if (conn->stats.iframe_retransmitted == 0) {
                    conn->stats.iframe_retransmitted = 1;
                }
            }
            break;  // Only retransmit the specific frame
        }
        idx = (idx + 1) % AX25_MAX_QUEUE_SIZE;
    }

    // In full-duplex mode restart T1 immediately using the sentinel value so the
    // tick handler arms it on the very next call.  In half-duplex mode clear T1
    // and let the tick handler restart it once the channel is free.
    if (conn->full_duplex) {
        // Use AX25_T1_PENDING (UINT32_MAX) not 1; value 1 collides with tick==1
        // on MCUs booting near zero and causes an off-by-one T1 expiry at T1<20ms.
        conn->t1_start_tick = AX25_T1_PENDING;
    } else {
        conn->t1_start_tick = 0;
    }
}

// Handle received REJ frame - AX.25 v2.2 Section 6.4.7
static void handle_received_rej(ax25_connection_t *conn, ax25_supervisory_frame_t *sframe) {
    uint8_t nr = sframe->nr;

    // Update statistics - S-frame received
    conn->stats.sframe_received++;
    if (conn->stats.sframe_received == 0) {
        conn->stats.sframe_received = 1;  // Prevent overflow
    }

    // Validate N(R) before using it - must lie in [V(A)..V(S)] per AX.25 v2.2 Section 4.2.2
    if (!ax25_nr_is_valid(conn, nr)) {
        send_frmr_z(conn, (uint8_t) sframe->base.type);
        return;
    }

    // range [N(R), old_V(S)) can be computed correctly without any half-window limit;
    // the MOD_LT macro that was used here had an unsafe threshold of modulo/2 which
    // fails for modulo-128 when the window k exceeds 64 frames
    uint8_t old_vs = conn->vars.vs;

    // AX.25 v2.2 Section 6.4.7: roll back V(S) to V(A) before retransmission.
    // This ensures the sequence space is consistent before any frames are sent.
    // Required in both half-duplex and full-duplex paths.
    conn->vars.vs = conn->vars.va;

    // AX.25 v2.2 Section 6.4.5.3: in full-duplex mode the device MAY abort the
    // frame it was currently sending and start retransmission immediately.
    // TX and RX use separate frequencies so there is no channel contention;
    // waiting for the next TX opportunity would delay recovery unnecessarily.
    // Retransmit every unacknowledged frame in the queue right now using only
    // uint8_t arithmetic and % AX25_MAX_QUEUE_SIZE - no 64-bit types needed.
    if (conn->full_duplex && conn->tx_queue.count > 0) {
        // Abort the in-progress frame before re-queuing retransmits.
        // Without this, retransmits queue behind a frame that may take up to
        // 1.7 s to finish at 1200 bps, delaying recovery unnecessarily.
        // If abort_tx is NULL the behaviour is identical to before this fix.
        if (conn->callbacks.abort_tx) {
            conn->callbacks.abort_tx(conn->user_data);
        }

        uint8_t idx = conn->tx_queue.head;
        uint8_t n = conn->tx_queue.count;
        for (uint8_t i = 0; i < n; i++) {
            uint8_t s = (uint8_t) ((idx + i) % AX25_MAX_QUEUE_SIZE);
            if (conn->callbacks.transmit) {
                conn->callbacks.transmit(conn->user_data, conn->tx_queue.frames[s], conn->tx_queue.lengths[s]);

                // Update statistics - I-frame retransmitted
                conn->stats.iframe_retransmitted++;
                if (conn->stats.iframe_retransmitted == 0) {
                    conn->stats.iframe_retransmitted = 1;
                }
            }
        }
    } else {
        // Half-duplex path: Section 6.4.7 retransmit from N(R) onwards.
        // V(S) has already been rolled back to V(A) above; the normal send
        // loop in ax25_send_data / ax25_tick will re-send queued frames.
        // Retransmit here only the frames the remote explicitly rejected.
        // Compute the total span [N(R), old_V(S)) once, then test each frame's
        // offset from N(R) against it; this is correct for any window size and any
        // modulo value including modulo-128 with k > 64 where MOD_LT would fail
        uint8_t total_range = AX25_OUTSTANDING(old_vs, nr, conn->vars.mod);
        uint8_t idx = conn->tx_queue.head;
        for (uint8_t i = 0; i < conn->tx_queue.count; i++) {
            uint8_t ns_offset = (uint8_t) ((conn->tx_queue.ns[idx] - nr) & AX25_MASK(conn->vars.mod));
            if (ns_offset < total_range) {
                if (conn->callbacks.transmit) {
                    conn->callbacks.transmit(conn->user_data, conn->tx_queue.frames[idx], conn->tx_queue.lengths[idx]);

                    // Update statistics - I-frame retransmitted
                    conn->stats.iframe_retransmitted++;
                    if (conn->stats.iframe_retransmitted == 0) {
                        conn->stats.iframe_retransmitted = 1;
                    }
                }
            }
            idx = (idx + 1) % AX25_MAX_QUEUE_SIZE;
        }
    }

    // In full-duplex mode restart T1 immediately using the sentinel value so the
    // tick handler arms it on the very next call.  In half-duplex mode clear T1
    // and let the tick handler restart it once the channel is free.
    if (conn->full_duplex) {
        // Use AX25_T1_PENDING (UINT32_MAX) not 1; same rationale as SREJ handler.
        conn->t1_start_tick = AX25_T1_PENDING;
    } else {
        conn->t1_start_tick = 0;
    }

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
    // use ax25_frame_encode to include address header in transmitted frame
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &rnr, &len, &err);
    if (encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, encoded, len);

        // Update statistics - S-frame sent
        conn->stats.sframe_sent++;
        if (conn->stats.sframe_sent == 0) {
            conn->stats.sframe_sent = 1;  // Prevent overflow
        }
    }

    if (encoded != NULL) {
        free(encoded);
        encoded = NULL;
    }
}

// Handle received RNR frame - AX.25 v2.2 Section 6.4.9
static void handle_received_rnr(ax25_connection_t *conn, ax25_supervisory_frame_t *rnr) {
    uint8_t nr = rnr->nr;

    // Update statistics - S-frame received
    conn->stats.sframe_received++;
    if (conn->stats.sframe_received == 0) {
        conn->stats.sframe_received = 1;  // Prevent overflow
    }

    // Validate N(R) and dequeue acknowledged frames using spec-correct loop
    if (!ax25_process_nr(conn, nr, (uint8_t) rnr->base.type)) {
        return;  // FRMR Z sent; do not continue
    }

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

static void handle_received_rr(ax25_connection_t *conn, ax25_supervisory_frame_t *rr) {
    uint8_t nr = rr->nr;
    bool pf = rr->pf;

    // Update statistics - S-frame received
    conn->stats.sframe_received++;
    if (conn->stats.sframe_received == 0) {
        conn->stats.sframe_received = 1;  // Prevent overflow
    }

    // Validate N(R) and dequeue acknowledged frames using spec-correct loop
    if (!ax25_process_nr(conn, nr, (uint8_t) rr->base.type)) {
        return;  // FRMR Z sent; do not continue
    }

    if (conn->peer_busy) {
        conn->peer_busy = false;
        conn->rnr_start_tick = 0;
        if (conn->callbacks.on_busy) {
            conn->callbacks.on_busy(conn->user_data, false);
        }
    }

    // Stop T1 here; ax25_tick will restart it if needed
    conn->retry_count = 0;
    conn->t1_start_tick = 0;  // Stop T1 - will be restarted by ax25_tick if frames pending

    if (pf) {
        if (conn->local_busy) {
            send_rnr(conn, true);
        } else {
            send_rr(conn, true);
        }
    }
}

// Handle received FRMR frame - AX.25 v2.2 Section 4.4.5
// When we receive FRMR, we must reset the link by sending SABM/SABME
static void handle_received_frmr(ax25_connection_t *conn, ax25_frame_reject_frame_t *frmr) {
    // notify MDL state machine (MDL-ERROR B) then fire DL-ERROR B
    if (conn->mgmt_ctx)
        ax25_mgmt_notify_frmr_received(conn->mgmt_ctx);
    FIRE_DL_ERROR(conn, AX25_DL_ERROR_B);

    // FRMR received - link reset required per Section 4.4.5
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

// Raw single-I-frame sender - used internally by the segmenter callback and by
// ax25_send_data for payloads that already fit within N1. Does NOT invoke the
// segmenter, preventing infinite recursion. All window/state checks are kept so
// each segment from the segmenter is properly gated before transmission.
static uint8_t ax25_send_data_raw(ax25_connection_t *conn, uint8_t *data, size_t len, uint8_t pid) {
    if (!conn || !data)
        return 1;

    if (conn->state == AX25_STATE_TIMER_RECOVERY)
        return 6;

    if (conn->state != AX25_STATE_CONNECTED)
        return 2;

    if (conn->peer_busy)
        return 5;

    // Check window not exceeded
    uint8_t outstanding = AX25_OUTSTANDING(conn->vars.vs, conn->vars.va, conn->vars.mod);
    if (outstanding >= conn->timers.k)
        return 3;

    if (conn->tx_queue.count >= AX25_MAX_QUEUE_SIZE)
        return 3;

    // Build I-frame; clamp to N1 as safety guard (callers ensure len <= n1)
    ax25_information_frame_t iframe;
    iframe.base.header = conn->peer_addr;
    iframe.base.type = (conn->vars.mod == 128) ? AX25_FRAME_INFORMATION_16BIT : AX25_FRAME_INFORMATION_8BIT;
    iframe.ns = conn->vars.vs;
    iframe.nr = conn->vars.vr;
    iframe.pf = false;
    iframe.pid = pid;
    iframe.payload_len = len > conn->timers.n1 ? conn->timers.n1 : len;
    iframe.payload = data;

    size_t frame_len;
    uint8_t err;
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &iframe, &frame_len, &err);
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
    if (conn->callbacks.transmit)
        conn->callbacks.transmit(conn->user_data, encoded, frame_len);

    // Update statistics - S-frame sent
    conn->stats.sframe_sent++;
    if (conn->stats.sframe_sent == 0)
        conn->stats.sframe_sent = 1;

    // Update statistics - I-frame sent
    conn->stats.iframe_sent++;
    if (conn->stats.iframe_sent == 0)
        conn->stats.iframe_sent = 1;

    // Update bytes sent
    conn->stats.bytes_sent += iframe.payload_len;
    if (conn->stats.bytes_sent < iframe.payload_len)
        conn->stats.bytes_sent = iframe.payload_len;

    // Cancel T2 timer - ACK is piggybacked in N(R) of this I-frame
    cancel_t2(conn);

    conn->vars.vs = INC_MOD(conn->vars.vs, conn->vars.mod);

    // Start T1 timer for acknowledgment
    if (conn->t1_start_tick == 0)
        conn->t1_start_tick = AX25_T1_PENDING;
    conn->retry_count = 0;

    // Stop T3 while T1 is running
    conn->t3_start_tick = 0;

    return 0;
}

// Segmenter transmit callback - invoked by ax25_segmenter_send for each produced
// segment frame. user_data is the owning ax25_connection_t (set in ax25_connection_init).
// Errors (window full, peer busy) are silently tolerated: the peer's TR210 timer
// detects missing segments and drives SREJ/REJ recovery per Appendix C6.
static void seg_transmit_cb(uint8_t *data, uint16_t len, uint8_t pid, void *user_data) {
    ax25_connection_t *conn = (ax25_connection_t*) user_data;
    if (!conn)
        return;
    ax25_send_data_raw(conn, data, (size_t) len, pid);
}

// Segmenter reassembly-error callback - maps ax25_seg_error_t to DL-ERROR indications.
// Wired automatically in ax25_connection_init so TR210 timeouts, sequence errors and
// overflow are propagated to the upper layer without application intervention.
static void seg_reassembly_error_cb(ax25_seg_error_t error, void *user_data) {
    ax25_connection_t *conn = (ax25_connection_t*) user_data;
    if (!conn)
        return;
    switch (error) {
        case AX25_SEG_ERROR_TIMEOUT:
            FIRE_DL_ERROR(conn, AX25_DL_ERROR_A);
        break;
        case AX25_SEG_ERROR_OVERFLOW:
            FIRE_DL_ERROR(conn, AX25_DL_ERROR_G);
        break;
        case AX25_SEG_ERROR_SEQUENCE:
            FIRE_DL_ERROR(conn, AX25_DL_ERROR_A);
        break;
        case AX25_SEG_ERROR_INVALID:
            FIRE_DL_ERROR(conn, AX25_DL_ERROR_L);
        break;
        default:
        break;
    }
}

// Segmenter retransmit-request callback - fires DL-ERROR-A when a segment gap is
// detected so the upper layer is informed; actual L2 frame recovery is handled by SREJ/REJ.
static void seg_request_retransmit_cb(uint8_t sequence, void *user_data) {
    ax25_connection_t *conn = (ax25_connection_t*) user_data;
    if (!conn)
        return;
    (void) sequence;
    FIRE_DL_ERROR(conn, AX25_DL_ERROR_A);
}

// Segmenter reassembly-complete callback - invoked when all segments have arrived
// and the original payload is fully rebuilt. Delivers to L3 via PID multiplexer.
// user_data is the owning ax25_connection_t (set in ax25_connection_init).
static void seg_reassembly_complete_cb(uint8_t *data, uint16_t len, uint8_t pid, void *user_data) {
    ax25_connection_t *conn = (ax25_connection_t*) user_data;
    if (!conn)
        return;
    dispatch_to_protocol(conn, data, (size_t) len, pid);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t ax25_connection_init(ax25_connection_t *conn, ax25_callbacks_t *cb, void *user_data) {
    if (!conn || !cb)
        return 1;

    memset(conn, 0, sizeof(ax25_connection_t));
    conn->state = AX25_STATE_DISCONNECTED;  //
    conn->vars.mod = 8;                    // Default to modulo 8
    conn->timers.t1 = 300;                 // 3 seconds default (300 * 10ms = 3000ms)
    conn->timers.t2 = 150;                 // 1.5 seconds default (150 * 10ms = 1500ms)
    conn->timers.t3 = 3000;                // 30 seconds default (3000 * 10ms = 30000ms)
    conn->timers.n2 = 10;                  //
    conn->timers.k = 7;                    //
    conn->timers.n1 = 256;                 //
    conn->t3_timeout = 18000;              // 30 minutes default (18000 * 10ms = 180000ms)
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

    // Initialize TEST statistics
    memset(&conn->test_stats, 0, sizeof(ax25_test_stats_t));

    // Initialize full-duplex mode
    // Default to half-duplex - will be updated if XID negotiation enables full-duplex
    conn->full_duplex = false;

    // Initialize protocol multiplexing - all handlers inactive by default
    for (uint8_t i = 0; i < AX25_MAX_PROTOCOL_HANDLERS; i++) {
        conn->protocols[i].active = false;
    }
    conn->default_handler = NULL;
    conn->default_user_data = NULL;

    // Initialize statistics
    memset(&conn->stats, 0, sizeof(ax25_statistics_t));

    // initialize MDL context pointer
    conn->mgmt_ctx = NULL;  // caller sets if MDL error bridging is needed

    // Initialize SABME/SABM modulo preference to mod-8 default
    // Caller sets conn->want_mod128 = 1 before ax25_connect() to request SABME
    conn->want_mod128 = 0;

    // Initialize segmenter/reassembler per AX.25 v2.2 Section 2.4 / Appendix C6.
    // segment_size = N1 - 2 (1-byte segment header + 1-byte original-PID overhead).
    // Callbacks wire automatic segmentation on TX and reassembly delivery on RX.
    ax25_segmenter_init(&conn->segmenter, conn->timers.n1);
    conn->segmenter.transmit_iframe = seg_transmit_cb;
    conn->segmenter.on_reassembly_complete = seg_reassembly_complete_cb;
    conn->segmenter.user_data = conn;

    // Wire error/retransmit callbacks: completes automatic Appendix C6 integration
    conn->segmenter.on_reassembly_error = seg_reassembly_error_cb;
    conn->segmenter.on_request_retransmit = seg_request_retransmit_cb;

    return 0;
}

// Process received I-frame - AX.25 v2.2 Section 6.4.4
void ax25_process_iframe(ax25_connection_t *conn, ax25_information_frame_t *iframe, uint32_t current_tick_10ms) {
    // DL-ERROR M: I-frame received while not in information-transfer state.
    // Per AX.25 v2.2 Appendix C4 SDL, issue error indication and discard.
    if (conn->state != AX25_STATE_CONNECTED && conn->state != AX25_STATE_TIMER_RECOVERY) {
        FIRE_DL_ERROR(conn, AX25_DL_ERROR_M);
        return;
    }

    // Check I-field length against negotiated N1 - Y-bit condition per §4.3.3.6
    // If the received payload exceeds our N1 parameter, reject with FRMR Y=1
    if (iframe->payload_len > (size_t) conn->timers.n1) {
        send_frmr_y(conn, (uint8_t) iframe->base.type);
        return;
    }

    // Extract N(R), N(S), and P/F from the decoded I-frame structure.
    // These are set by ax25_information_frame_decode() during frame parsing.
    uint8_t nr = (uint8_t) iframe->nr;
    uint8_t ns = (uint8_t) iframe->ns;
    bool pf = iframe->pf;

    // Restart T3 using the authoritative tick supplied by the caller
    restart_t3_on_activity(conn, current_tick_10ms);

    // Update statistics - I-frame received
    conn->stats.iframe_received++;
    // Prevent overflow on 32-bit counter
    if (conn->stats.iframe_received == 0) {
        conn->stats.iframe_received = 1;
    }
    // Update bytes received (approximate - includes payload)
    conn->stats.bytes_received += iframe->payload_len;
    if (conn->stats.bytes_received < iframe->payload_len) {
        // Overflow occurred
        conn->stats.bytes_received = iframe->payload_len;
    }

    // Validate N(R) and dequeue acknowledged frames using spec-correct loop
    // ax25_process_nr validates [V(A)..V(S)] range, dequeues by head_ns==N(R),
    // advances V(A), and stops T1 if queue drained
    if (!ax25_process_nr(conn, nr, (uint8_t) iframe->base.type)) {
        return;  // FRMR Z sent; do not continue
    }
    // Restart T1 for remaining unacknowledged frames using authoritative tick
    if (conn->tx_queue.count > 0 && conn->t1_start_tick == 0) {
        conn->t1_start_tick = current_tick_10ms ? current_tick_10ms : 1;
    }
    // Restart T3 if T1 was just stopped (no outstanding frames) using authoritative tick
    if (conn->tx_queue.count == 0 && conn->t3_start_tick == 0) {
        conn->t3_start_tick = current_tick_10ms ? current_tick_10ms : 1;
    }

    // Process received N(S): classify as in-order, out-of-order, or duplicate
    if (ns == conn->vars.vr) {
        // In-order: expected sequence number - process normally
        // Pass tick so segment fragments reach the TR210 reassembly timer
        process_expected_iframe(conn, iframe, current_tick_10ms);

    } else if (ax25_in_window(ns, conn->vars.vr, conn->timers.k, conn->vars.mod)) {
        // Out-of-order: N(S) is ahead of V(R) and within receive window k.
        handle_out_of_sequence_iframe(conn, iframe);

    } else {
        // Outside the window: N(S) > k ahead of V(R) or behind V(R).
        // Per AX.25 v2.2 Section 4.2.4 discard silently; acknowledge if P/F set
        if (pf) {
            send_rr(conn, true);  // Immediate response for P/F bit
        }
        // For non-P/F, T2 timer will send RR after delay
    }

    // Start T2 timer if not already running and ACK not pending
    // This is called after processing the I-frame to start the response delay
    // Pass the authoritative tick instead of the stale t3_start_tick proxy
    if (!conn->t2_running && !conn->t2_ack_pending && !iframe->pf) {
        start_t2_response(conn, current_tick_10ms);
    }
}

// Timer tick handler - call every 10ms
void ax25_tick(ax25_connection_t *conn, uint32_t current_tick_10ms) {
    if (!conn)
        return;

    // Tick the TR210 reassembly timer per AX.25 v2.2 Section 6.7.1.13 / Appendix C6.3.
    // ax25_segmenter_tick is a no-op when no reassembly is in progress.
    ax25_segmenter_tick(&conn->segmenter, current_tick_10ms);

    // Initialize T1 timer with actual tick value if using sentinel
    // Never set to 0 since 0 means "timer not active"
    // Sentinel is now AX25_T1_PENDING (UINT32_MAX) instead of 1.
    // Resolution runs before the T1 expiry check, so UINT32_MAX is never
    // evaluated as a start time and cannot cause a spurious expiry.
    if (conn->t1_start_tick == AX25_T1_PENDING) {
        conn->t1_start_tick = (current_tick_10ms != 0u) ? current_tick_10ms : 1u;
    }

    // Start T3 if connected, no outstanding frames (T1 not running), and T3 not active
    if (conn->state == AX25_STATE_CONNECTED && conn->t1_start_tick == 0 && conn->t3_start_tick == 0) {
        conn->t3_start_tick = current_tick_10ms;
    }

    // T1: Acknowledgment timer expiration
    if (conn->t1_start_tick != 0 && (current_tick_10ms - conn->t1_start_tick) >= conn->timers.t1) {
        conn->retry_count++;
        if (conn->retry_count > conn->timers.n2) {
            // Max retries: Disconnect
            conn->state = AX25_STATE_DISCONNECTED;
            conn->t1_start_tick = 0;
            conn->t3_start_tick = 0;  // Reset T3 as well

            // Free any I-frames queued for retransmission
            while (conn->tx_queue.count > 0) {
                free(conn->tx_queue.frames[conn->tx_queue.head]);
                conn->tx_queue.frames[conn->tx_queue.head] = NULL;
                conn->tx_queue.head = (conn->tx_queue.head + 1) % AX25_MAX_QUEUE_SIZE;
                conn->tx_queue.count--;
            }

            // DL-ERROR indication code N: N2 retries exceeded
            FIRE_DL_ERROR(conn, AX25_DL_ERROR_N);

            // Notify upper layer if callback exists
            if (conn->callbacks.on_disconnect) {
                conn->callbacks.on_disconnect(conn->user_data, 3);  // 3 = timeout
            }
        } else {
            // state-aware T1 retry handling
            if (conn->state == AX25_STATE_AWAITING_CONNECTION) {
                // Retransmit SABM/SABME per AX.25 v2.2 Section 6.3.1 and C4 SDL.
                // Do NOT transition to TIMER_RECOVERY: stay in AWAITING_CONNECTION.
                bool use_sabme = (conn->vars.mod == 128);
                send_sabm(conn, use_sabme);
                conn->t1_start_tick = current_tick_10ms ? current_tick_10ms : 1u;
            } else if (conn->state == AX25_STATE_AWAITING_RELEASE) {
                // Retransmit DISC per AX.25 v2.2 Section 6.3.4 and C4 SDL.
                // Do NOT transition to TIMER_RECOVERY: stay in AWAITING_RELEASE.
                send_disc(conn);
                conn->t1_start_tick = current_tick_10ms ? current_tick_10ms : 1u;
            } else {
                // Enter timer recovery and retransmit per AX.25 v2.2 Section 6.7.1.1
                conn->state = AX25_STATE_TIMER_RECOVERY;

                // Retransmit all unacknowledged I-frames from V(A) to V(S)-1
                if (conn->tx_queue.count > 0) {
                    uint8_t idx = conn->tx_queue.head;
                    for (uint8_t i = 0; i < conn->tx_queue.count; i++) {
                        if (conn->callbacks.transmit) {
                            // Retransmit frame
                            conn->callbacks.transmit(conn->user_data, conn->tx_queue.frames[idx], conn->tx_queue.lengths[idx]);

                            // Update statistics - I-frame retransmitted
                            conn->stats.iframe_retransmitted++;
                            if (conn->stats.iframe_retransmitted == 0) {
                                conn->stats.iframe_retransmitted = 1;
                            }
                            conn->stats.retries++;
                            if (conn->stats.retries == 0) {
                                conn->stats.retries = 1;
                            }
                        }
                        idx = (idx + 1) % AX25_MAX_QUEUE_SIZE;
                    }
                } else {
                    // No I-frames to retransmit, send RR with P=1 to poll
                    send_rr(conn, true);
                }

                conn->t1_start_tick = current_tick_10ms ? current_tick_10ms : 1;  // Restart T1, avoid 0
            }
        }
        conn->stats.t1_expirations++;
    }

    // T2: Response delay timer expiration
    // Full-duplex never arms T2; skip evaluation if flag is set
    // to prevent a stale t2_running (from a prior half-duplex phase) from
    // firing a spurious RR after mode transition. Ref: Section 6.7.1.2.
    if (conn->t2_running && !conn->full_duplex && (current_tick_10ms - conn->t2_start_tick) >= conn->timers.t2) {
        conn->t2_running = false;
        if (conn->t2_ack_pending) {
            send_rr(conn, false);  // Send delayed ACK
            conn->t2_ack_pending = false;
        }
    }

    // T3: Inactive link timer expiration
    if (conn->t3_start_tick != 0 && (current_tick_10ms - conn->t3_start_tick) >= conn->timers.t3) {
        // Send poll to check link
        send_rr(conn, true);  // RR with P=1
        conn->t1_start_tick = current_tick_10ms ? current_tick_10ms : 1;  // Start T1, avoid 0
        conn->t3_start_tick = current_tick_10ms;  // Restart T3
    }
}

// Main entry point for received frames
void ax25_process_frame(ax25_connection_t *conn, ax25_frame_t *frame, uint32_t current_tick_10ms) {
    if (!conn || !frame)
        return;

    // Restart T3 using the authoritative tick supplied by the caller.
    // Previously a stale proxy (t3_start_tick or t1_start_tick) was computed
    // here; that caused T3 to appear already-expired on first use and T2 to
    // fire immediately when t3_start_tick was many ticks old.
    restart_t3_on_activity(conn, current_tick_10ms);

    switch (frame->type) {
        case AX25_FRAME_UNNUMBERED_UA: {
            // Update statistics - U-frame received
            conn->stats.uframe_received++;
            if (conn->stats.uframe_received == 0) {
                conn->stats.uframe_received = 1;
            }

            if (conn->state == AX25_STATE_AWAITING_CONNECTION) {
                // AX.25 v2.2 Appendix C4 SDL: UA received in AWAITING_CONNECTION must
                // have F=1 (responding to our SABM P=1). UA with F=0 is a protocol
                // error - discard and issue DL-ERROR C per Section 17.2.
                ax25_unnumbered_frame_t *ua_frame = (ax25_unnumbered_frame_t*) frame;
                if (!ua_frame->pf) {
                    FIRE_DL_ERROR(conn, AX25_DL_ERROR_C);
                    break;  // Discard; remain in AWAITING_CONNECTION
                }

                // Connection established - transition to CONNECTED state
                conn->vars.vs = conn->vars.vr = conn->vars.va = 0;
                conn->state = AX25_STATE_CONNECTED;
                // clear want_mod128 on successful connection
                // conn->vars.mod already holds the negotiated modulo (8 or 128)
                conn->want_mod128 = 0;

                // Clear SREJ/REJ state on new connection
                clear_srej_state(conn);
                conn->rej_exception = false;

                // Clear peer busy on connection establishment
                conn->peer_busy = false;
                conn->local_busy = false;
                conn->rnr_start_tick = 0;

                // Clear FRMR state
                conn->frmr_pending = false;
                conn->frmr_retry_count = 0;

                // Cancel T1 timer
                conn->t1_start_tick = 0;
                conn->retry_count = 0;

                // Start T3 using the authoritative current tick
                conn->t3_start_tick = current_tick_10ms;

                // Notify upper layer of connection establishment
                // DL-CONNECT confirm: local station initiated (sent SABM), peer replied UA.
                // initiated_locally = true per AX.25 v2.2 Section 5.3 / Appendix D.3.
                if (conn->callbacks.on_connect) {
                    conn->callbacks.on_connect(conn->user_data, true);
                }
            } else if (conn->state == AX25_STATE_AWAITING_RELEASE) {
                // Disconnect acknowledged - transition to DISCONNECTED
                conn->state = AX25_STATE_DISCONNECTED;

                // Clear all state
                clear_srej_state(conn);
                conn->rej_exception = false;
                conn->peer_busy = false;
                conn->local_busy = false;
                conn->rnr_start_tick = 0;
                conn->frmr_pending = false;
                conn->frmr_retry_count = 0;

                // Notify upper layer
                if (conn->callbacks.on_disconnect) {
                    conn->callbacks.on_disconnect(conn->user_data, 0);
                }
            } else if (conn->state == AX25_STATE_FRAME_REJECT) {
                // UA received in FRMR state - return to DISCONNECTED per Section 4.4.5
                conn->state = AX25_STATE_DISCONNECTED;

                // Clear all state
                clear_srej_state(conn);
                conn->rej_exception = false;
                conn->peer_busy = false;
                conn->local_busy = false;
                conn->rnr_start_tick = 0;
                conn->frmr_pending = false;
                conn->frmr_retry_count = 0;

                // Notify upper layer
                // DL-CONNECT confirm: local station initiated (sent SABM), peer replied UA.
                // initiated_locally = true per AX.25 v2.2 Section 5.3 / Appendix D.3.
                if (conn->callbacks.on_connect) {
                    conn->callbacks.on_connect(conn->user_data, true);
                }
            }
            // UA frames in other states are ignored per AX.25 v2.2
            break;
        }

        case AX25_FRAME_UNNUMBERED_SABM:
        case AX25_FRAME_UNNUMBERED_SABME: {
            // Update statistics - U-frame received
            conn->stats.uframe_received++;
            if (conn->stats.uframe_received == 0) {
                conn->stats.uframe_received = 1;
            }

            // SABM/SABME collision handling per §6.3.6.2
            // When both stations transmit SABM simultaneously each receives the
            // other's SABM while in AWAITING_CONNECTION. Cancel T1 so the pending
            // retransmit loop does not fire again; the code below sends UA and
            // transitions to CONNECTED immediately.
            if (conn->state == AX25_STATE_AWAITING_CONNECTION) {
                conn->t1_start_tick = 0;
                conn->retry_count = 0;
            }

            // Connection request
            conn->peer_addr = frame->header;
            conn->peer_addr.cr = false;  // Response will have opposite CR bit
            conn->vars.vs = conn->vars.vr = conn->vars.va = 0;
            conn->vars.mod = (frame->type == AX25_FRAME_UNNUMBERED_SABME) ? 128u : 8u;

            // For modulo-8: max k = 7 (M-1). For modulo-128: max k = 127,
            // further bounded by queue capacity.
            uint8_t proto_max_k = (conn->vars.mod == 128u) ? 127u : 7u;
            uint8_t queue_max_k = (uint8_t) (AX25_MAX_QUEUE_SIZE - 1u);
            uint8_t effective_max = (proto_max_k < queue_max_k) ? proto_max_k : queue_max_k;
            // Preserve XID-negotiated k if still within range for the new modulus.
            // Clamp if too large (e.g. mod-128 k=15 -> mod-8 clamped to 7).
            // Fill spec default if k is zero (invalid).
            if (conn->timers.k == 0u || conn->timers.k > effective_max) {
                uint8_t spec_default = (conn->vars.mod == 128u) ? 32u : 7u;
                conn->timers.k = (spec_default <= effective_max) ? spec_default : effective_max;
            }

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
            // set CR bits for response
            ua.base.header.destination.ch = false;   // Response: dest CH = 0
            ua.base.header.source.ch = true;         // Response: src CH = 1
            ua.base.header.cr = false;               // This is a response
            ua.base.type = AX25_FRAME_UNNUMBERED_UA;
            ua.pf = ((ax25_unnumbered_frame_t*) frame)->pf;
            ua.modifier = 0x63;

            size_t len;
            uint8_t err;
            uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &ua, &len, &err);
            if (encoded && conn->callbacks.transmit) {
                conn->callbacks.transmit(conn->user_data, encoded, len);
            }

            if (encoded != NULL) {
                free(encoded);
                encoded = NULL;
            }

            // Start T3 for idle link monitoring using the authoritative current tick
            conn->t3_start_tick = current_tick_10ms;

            // DL-CONNECT indication: remote station initiated (sent SABM), we replied UA.
            // initiated_locally = false per AX.25 v2.2 Section 5.3 / Appendix D.3.
            if (conn->callbacks.on_connect) {
                conn->callbacks.on_connect(conn->user_data, false);
            }
            break;
        }

        case AX25_FRAME_INFORMATION_8BIT:
        case AX25_FRAME_INFORMATION_16BIT:
            // In FRAME_REJECT state discard I-frames but resend FRMR if P=1
            // per AX.25 v2.2 Section 4.4.5
            if (conn->state == AX25_STATE_FRAME_REJECT) {
                if (((ax25_information_frame_t*) frame)->pf) {
                    resend_stored_frmr(conn);
                }
            } else {
                // Pass authoritative tick through to ax25_process_iframe
                ax25_process_iframe(conn, (ax25_information_frame_t*) frame, current_tick_10ms);

            }
        break;

        case AX25_FRAME_SUPERVISORY_RR_8BIT:
        case AX25_FRAME_SUPERVISORY_RR_16BIT: {
            ax25_supervisory_frame_t *sframe = (ax25_supervisory_frame_t*) frame;

            // In frame reject state, discard S frames per Section 4.4.5
            // Except for examining P/F bit
            if (conn->state == AX25_STATE_FRAME_REJECT) {
                // If P=1, respond with FRMR per Section 4.4.5
                if (sframe->pf) {
                    resend_stored_frmr(conn);
                }
                break;
            }

            handle_received_rr(conn, sframe);
            break;
        }

        case AX25_FRAME_SUPERVISORY_RNR_8BIT:
        case AX25_FRAME_SUPERVISORY_RNR_16BIT: {
            ax25_supervisory_frame_t *sframe = (ax25_supervisory_frame_t*) frame;
            // In frame reject state, discard but check P bit
            if (conn->state == AX25_STATE_FRAME_REJECT) {
                if (sframe->pf) {
                    resend_stored_frmr(conn);
                }
                break;
            }
            handle_received_rnr(conn, sframe);

            // Continue T3 during peer busy (T3 keeps running to detect stuck links)
            // Per Section 6.4.9: T3 expiry causes poll with RR/RNR P=1
            conn->t3_start_tick = current_tick_10ms;
            break;
        }

        case AX25_FRAME_SUPERVISORY_REJ_8BIT:
        case AX25_FRAME_SUPERVISORY_REJ_16BIT: {
            ax25_supervisory_frame_t *sframe = (ax25_supervisory_frame_t*) frame;
            // In frame reject state, discard but check P bit
            if (conn->state == AX25_STATE_FRAME_REJECT) {
                if (sframe->pf) {
                    resend_stored_frmr(conn);
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

            // Restart T3 on activity using the authoritative current tick
            conn->t3_start_tick = current_tick_10ms;
            break;
        }

        case AX25_FRAME_SUPERVISORY_SREJ_8BIT:
        case AX25_FRAME_SUPERVISORY_SREJ_16BIT: {
            ax25_supervisory_frame_t *sframe = (ax25_supervisory_frame_t*) frame;
            // In frame reject state, discard but check P bit
            if (conn->state == AX25_STATE_FRAME_REJECT) {
                if (sframe->pf) {
                    resend_stored_frmr(conn);
                }
                break;
            }
            handle_received_srej(conn, sframe);

            // Restart T3 on activity using the authoritative current tick
            conn->t3_start_tick = current_tick_10ms;

            break;
        }

        case AX25_FRAME_UNNUMBERED_FRMR: {
            // Handle received FRMR
            ax25_frame_reject_frame_t *frmr = (ax25_frame_reject_frame_t*) frame;

            // SABME fallback when FRMR received during AWAITING_CONNECTION
            // Per PE1CHL §6: some stations reply FRMR to SABME instead of UA.
            // Fall back to mod-8 SABM, reset retry counter, and try again.
            if (conn->state == AX25_STATE_AWAITING_CONNECTION && conn->want_mod128) {
                conn->want_mod128 = 0;
                conn->vars.mod = 8;
                conn->timers.k = 7;
                conn->retry_count = 0;
                send_sabm(conn, false);
                conn->t1_start_tick = AX25_T1_PENDING;
                break;
            }

            handle_received_frmr(conn, frmr);
            break;
        }

        case AX25_FRAME_UNNUMBERED_DISC: {
            // Update statistics - U-frame received
            conn->stats.uframe_received++;
            if (conn->stats.uframe_received == 0) {
                conn->stats.uframe_received = 1;
            }

            // AX.25 v2.2 Appendix C4 SDL: DISC received in DISCONNECTED state must be
            // answered with DM (not UA). UA would falsely imply an active connection
            // was released. Per SDL state D0 (DISCONNECTED), the correct response to
            // any command is DM with F=P bit mirrored from the received DISC.
            if (conn->state == AX25_STATE_DISCONNECTED) {
                ax25_unnumbered_frame_t dm;
                dm.base.header = conn->peer_addr;
                dm.base.header.cr = false;  // DM is a response frame
                dm.base.type = AX25_FRAME_UNNUMBERED_DM;
                dm.pf = ((ax25_unnumbered_frame_t*) frame)->pf;  // F = P from received DISC
                dm.modifier = 0x0F;  // DM modifier per AX.25 v2.2 Table 3
                uint8_t err;
                size_t len;
                // ax25_frame_encode includes address header + control byte for transmit
                uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &dm, &len, &err);
                if (encoded && conn->callbacks.transmit) {
                    conn->callbacks.transmit(conn->user_data, encoded, len);
                }
                if (encoded != NULL) {
                    free(encoded);
                }
                // Do NOT fire on_disconnect: Layer 3 was never in a connected state
                break;
            }

            // DISC received while awaiting UA to our own SABM: remote refused or
            // pre-empted the connection. Respond with DM, cancel T1, return to
            // DISCONNECTED, notify Layer 3 with reason 1 (remote disconnect).
            if (conn->state == AX25_STATE_AWAITING_CONNECTION) {
                ax25_unnumbered_frame_t dm;
                dm.base.header = conn->peer_addr;
                dm.base.header.cr = false;  // DM is a response frame
                dm.base.type = AX25_FRAME_UNNUMBERED_DM;
                dm.pf = ((ax25_unnumbered_frame_t*) frame)->pf;
                dm.modifier = 0x0F;  // DM modifier per AX.25 v2.2 Table 3
                uint8_t err;
                size_t len;
                // ax25_frame_encode includes address header + control byte for transmit
                uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &dm, &len, &err);
                if (encoded && conn->callbacks.transmit) {
                    conn->callbacks.transmit(conn->user_data, encoded, len);
                }
                if (encoded != NULL) {
                    free(encoded);
                }
                conn->state = AX25_STATE_DISCONNECTED;
                conn->t1_start_tick = 0;
                conn->retry_count = 0;
                // Report reason=1: remote refused or pre-empted connection
                if (conn->callbacks.on_disconnect) {
                    conn->callbacks.on_disconnect(conn->user_data, 1);
                }
                break;
            }

            // Normal DISC received while CONNECTED, TIMER_RECOVERY, AWAITING_RELEASE,
            // or FRAME_REJECT: send UA and transition to DISCONNECTED.
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
            uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &ua, &len, &err);
            if (encoded && conn->callbacks.transmit) {
                conn->callbacks.transmit(conn->user_data, encoded, len);
            }

            if (encoded != NULL) {
                free(encoded);
                encoded = NULL;
            }

            // Use reason=1 (remote disconnect) to distinguish from locally-initiated
            // disconnect (reason=0 from the AWAITING_RELEASE/UA-received path).
            if (conn->callbacks.on_disconnect) {
                conn->callbacks.on_disconnect(conn->user_data, 1);
            }

            break;
        }

        case AX25_FRAME_UNNUMBERED_TEST: {
            // Update statistics - U-frame received
            conn->stats.uframe_received++;
            if (conn->stats.uframe_received == 0) {
                conn->stats.uframe_received = 1;
            }

            // Handle TEST frame - works in any state per AX.25 v2.2 Section 6.4.13
            ax25_test_frame_t *test = (ax25_test_frame_t*) frame;
            // Pass the authoritative tick for accurate RTT measurement
            handle_test_frame(conn, test, current_tick_10ms);

            break;
        }

        case AX25_FRAME_UNNUMBERED_INFORMATION: {
            // Update statistics - U-frame received
            conn->stats.uframe_received++;
            if (conn->stats.uframe_received == 0) {
                conn->stats.uframe_received = 1;
            }

            // Handle UI frame - AX.25 v2.2 Section 6.4.12
            ax25_unnumbered_information_frame_t *ui = (ax25_unnumbered_information_frame_t*) frame;
            handle_ui_frame(conn, ui);
            break;
        }

        case AX25_FRAME_UNNUMBERED_DM: {
            // AX.25 v2.2 Appendix C4 SDL: DM received means the remote is in disconnected
            // mode. Per Section 4.3.3.5, handle behavior depends on current state.
            conn->stats.uframe_received++;
            if (conn->stats.uframe_received == 0) {
                conn->stats.uframe_received = 1;
            }

            if (conn->state == AX25_STATE_DISCONNECTED) {
                break;  // Already disconnected, ignore DM
            }

            if (conn->state == AX25_STATE_AWAITING_CONNECTION) {
                // SABME fallback when DM received during AWAITING_CONNECTION
                // Per PE1CHL §6: some stations reply DM to SABME instead of UA.
                // If we were trying mod-128, fall back to mod-8 SABM and retry.
                if (conn->want_mod128) {
                    conn->want_mod128 = 0;
                    conn->vars.mod = 8;
                    conn->timers.k = 7;
                    conn->retry_count = 0;
                    send_sabm(conn, false);
                    conn->t1_start_tick = AX25_T1_PENDING;
                    break;
                }

                // Remote refused connection - cancel T1, notify Layer 3
                conn->state = AX25_STATE_DISCONNECTED;
                conn->t1_start_tick = 0;
                conn->retry_count = 0;
                if (conn->callbacks.on_disconnect) {
                    conn->callbacks.on_disconnect(conn->user_data, 1);
                }
                break;
            }

            if (conn->state == AX25_STATE_CONNECTED || conn->state == AX25_STATE_TIMER_RECOVERY) {
                // DL-ERROR D: DM received while connected - remote reset the link
                FIRE_DL_ERROR(conn, AX25_DL_ERROR_D);
                conn->state = AX25_STATE_DISCONNECTED;
                conn->t1_start_tick = 0;
                conn->t3_start_tick = 0;
                conn->retry_count = 0;
                conn->peer_busy = false;
                conn->local_busy = false;
                // Flush tx_queue on forced disconnect
                while (conn->tx_queue.count > 0) {
                    free(conn->tx_queue.frames[conn->tx_queue.head]);
                    conn->tx_queue.frames[conn->tx_queue.head] = NULL;
                    conn->tx_queue.head = (conn->tx_queue.head + 1) % AX25_MAX_QUEUE_SIZE;
                    conn->tx_queue.count--;
                }
                if (conn->callbacks.on_disconnect) {
                    conn->callbacks.on_disconnect(conn->user_data, 1);
                }
                break;
            }

            // AWAITING_RELEASE, FRAME_REJECT or other: DM confirms disconnected state
            conn->state = AX25_STATE_DISCONNECTED;
            conn->t1_start_tick = 0;
            conn->retry_count = 0;
            clear_srej_state(conn);
            if (conn->callbacks.on_disconnect) {
                conn->callbacks.on_disconnect(conn->user_data, 0);
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
    conn->peer_addr.destination.res0 = true;
    conn->peer_addr.destination.res1 = true;
    conn->peer_addr.destination.extension = false;

    conn->peer_addr.source = *src;
    conn->peer_addr.source.res0 = true;
    conn->peer_addr.source.res1 = true;
    conn->peer_addr.source.extension = true;

    conn->peer_addr.cr = true;
    conn->peer_addr.repeaters.num_repeaters = 0;

    // send SABME when want_mod128 is set, SABM otherwise
    // Per PE1CHL §6: if want_mod128 is set we prefer mod-128 (SABME); the state
    // machine falls back to SABM automatically on DM/FRMR response from peer.
    if (conn->want_mod128) {
        // Request mod-128 extended sequence numbering via SABME
        conn->vars.mod = 128;
        // Apply spec default window size for mod-128 unless caller pre-configured a
        // larger value; clamp to queue capacity so the queue never overflows.
        if (conn->timers.k <= 7u) {
            uint8_t queue_max = (uint8_t) (AX25_MAX_QUEUE_SIZE - 1u);
            conn->timers.k = (32u <= queue_max) ? 32u : queue_max;
        }
        send_sabm(conn, true);  // true = SABME
    } else {
        // Standard mod-8 connection via SABM
        conn->vars.mod = 8;
        send_sabm(conn, false);  // false = SABM
    }

    conn->state = AX25_STATE_AWAITING_CONNECTION;
    conn->retry_count = 0;
    // T1 MUST be started after sending SABM per AX.25 v2.2 Section 6.3.1 and C4 SDL.
    // Use AX25_T1_PENDING sentinel so ax25_tick() arms it on the first call,
    // avoiding a spurious immediate expiry when current_tick is near zero.
    // Without this, AWAITING_CONNECTION never times out: no DL-ERROR N and no
    // DL-DISCONNECT indication reach Layer 3 when the peer never responds.
    conn->t1_start_tick = AX25_T1_PENDING;

    return 0;
}

// sends DISC frame, flushes and frees tx_queue, transitions to AWAITING_RELEASE
uint8_t ax25_disconnect(ax25_connection_t *conn) {
    if (!conn)
        return 1;
    // allow disconnect from CONNECTED or TIMER_RECOVERY states
    if (conn->state != AX25_STATE_CONNECTED && conn->state != AX25_STATE_TIMER_RECOVERY)
        return 2;

    // build and send DISC command frame per AX.25 v2.2 Section 4.3.3.3
    ax25_unnumbered_frame_t disc;
    disc.base.header = conn->peer_addr;
    disc.base.header.cr = true;   // command frame
    disc.base.type = AX25_FRAME_UNNUMBERED_DISC;
    disc.pf = true;               // P bit set per AX.25 v2.2 Section 4.3.3.3
    disc.modifier = 0x43;         // DISC modifier per AX.25 v2.2 Table 3

    size_t disc_len;
    uint8_t disc_err;
    uint8_t *disc_encoded = ax25_frame_encode((ax25_frame_t*) &disc, &disc_len, &disc_err);

    if (disc_encoded && conn->callbacks.transmit) {
        conn->callbacks.transmit(conn->user_data, disc_encoded, disc_len);

        // update statistics - U-frame sent
        conn->stats.uframe_sent++;
        if (conn->stats.uframe_sent == 0) {
            conn->stats.uframe_sent = 1;
        }
    }

    if (disc_encoded != NULL) {
        free(disc_encoded);
        disc_encoded = NULL;
    }

    // free all frames queued for retransmission - they will not be sent after DISC
    while (conn->tx_queue.count > 0) {
        free(conn->tx_queue.frames[conn->tx_queue.head]);
        conn->tx_queue.frames[conn->tx_queue.head] = NULL;
        conn->tx_queue.frames[conn->tx_queue.head] = NULL;
        conn->tx_queue.head = (conn->tx_queue.head + 1) % AX25_MAX_QUEUE_SIZE;
        conn->tx_queue.count--;
    }

    // transition to awaiting release and arm T1 for UA timeout
    conn->state = AX25_STATE_AWAITING_RELEASE;
    conn->retry_count = 0;

    // T1 MUST be started after sending DISC per AX.25 v2.2 Section 6.3.4 and
    // C4 SDL. If UA or DM is never received, T1 expiry drives N2 retries and
    // ultimately issues DL-ERROR N + DL-DISCONNECT indication to Layer 3.
    // Use AX25_T1_PENDING sentinel to let ax25_tick() arm it safely on first call.
    conn->t1_start_tick = AX25_T1_PENDING;

    return 0;
}

uint8_t ax25_send_data(ax25_connection_t *conn, uint8_t *data, size_t len, uint8_t pid) {
    if (!conn || !data)
        return 1;

    // DL-DATA request: per AX.25 v2.2 Section 6.4.11 and C4 SDL, new I-frames
    // must not be sent in TIMER_RECOVERY state (retransmission in progress).
    if (conn->state == AX25_STATE_TIMER_RECOVERY)
        return 6;  // Recovery in progress - retry later

    if (conn->state != AX25_STATE_CONNECTED)
        return 2;

    // Per AX.25 v2.2 Section 6.4.9: stop I-frame transmission while peer is busy
    if (conn->peer_busy)
        return 5;  // Peer busy

    // Segment automatically when payload exceeds negotiated N1 per Section 6.6 / Appendix C6.
    // Previously the payload was silently truncated to N1 (data loss); now the segmenter
    // splits the data into multiple I-frames and the peer reassembles them transparently.
    if (len > conn->timers.n1) {
        // Require at least one open window slot before starting segmentation.
        // If the window fills mid-burst, the peer's TR210 timer detects the gap
        // and drives SREJ/REJ recovery per AX.25 v2.2 Appendix C6.
        uint8_t outstanding = AX25_OUTSTANDING(conn->vars.vs, conn->vars.va, conn->vars.mod);
        if (outstanding >= conn->timers.k || conn->tx_queue.count >= AX25_MAX_QUEUE_SIZE)
            return 3;  // Window closed - caller must retry

        // Clamp to segmenter maximum of 4096 bytes (64 segments x 64-byte data max)
        uint16_t seg_len = (uint16_t) (len > 4096u ? 4096u : len);
        uint8_t result = ax25_segmenter_send(&conn->segmenter, data, seg_len, pid);
        // Map segmenter codes to ax25_send_data conventions:
        //   0=success, 1=invalid param, 2=too large, 3=no tx callback, 4=busy
        if (result == 4)
            return 3;  // Segmenter busy -> window full equivalent
        if (result != 0)
            return 1;  // Invalid parameters or oversized data
        return 0;
    }

    // Payload fits within N1: use raw single-frame path (no segmentation)
    return ax25_send_data_raw(conn, data, len, pid);
}

// Send RNR when local buffers full - AX.25 v2.2 Section 6.4.10
uint8_t ax25_send_rnr(ax25_connection_t *conn) {
    if (!conn || conn->state != AX25_STATE_CONNECTED)
        return 1;

    // DL-FLOW-OFF: permitted in CONNECTED and TIMER_RECOVERY per AX.25 v2.2
    // Section 6.4.10. Both are information-transfer states where local busy
    // can occur. Reject any other state.
    if (!conn)
        return 1;
    if (conn->state != AX25_STATE_CONNECTED && conn->state != AX25_STATE_TIMER_RECOVERY)
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

    if (encoded != NULL) {
        free(encoded);
        encoded = NULL;
    }

    return 0;
}

// Clear local busy condition - AX.25 v2.2 Section 6.4.10
uint8_t ax25_clear_local_busy(ax25_connection_t *conn) {
    // DL-FLOW-ON: guard against invalid states and choose correct response
    // frame per AX.25 v2.2 Section 6.4.10:
    //   - REJ if an implicit-reject exception is outstanding (last I-frame
    //     was out of sequence and REJ was not yet sent or not yet answered)
    //   - RR otherwise (all received I-frames were in order)
    if (!conn || !conn->local_busy)
        return 1;
    if (conn->state != AX25_STATE_CONNECTED && conn->state != AX25_STATE_TIMER_RECOVERY)
        return 1;

    conn->local_busy = false;

    if (conn->rej_exception && can_use_rej(conn)) {
        // REJ required: missing frames need retransmission from V(R)
        send_rej(conn, false);
    } else {
        // RR: all frames in order, peer may resume sending
        send_rr(conn, false);
    }

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

    if (encoded != NULL) {
        free(encoded);
        encoded = NULL;
    }

    return 0;  // Success
}

// ax25_send_ui_conn: DL-UNIT-DATA request through connection context.
// Preferred over ax25_send_ui() because it updates statistics and uses
// the connection's own transmit callback and user_data context.
// Per AX.25 v2.2 Appendix D.4: UI frames valid in all connection states.
uint8_t ax25_send_ui_conn(ax25_connection_t *conn, uint8_t *data, size_t len, uint8_t pid) {
    if (!conn || !data || len == 0)
        return 1;
    if (!conn->callbacks.transmit)
        return 1;

    ax25_unnumbered_information_frame_t ui;
    ui.base.base.header = conn->peer_addr;
    ui.base.base.header.cr = true;
    ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
    ui.base.pf = false;
    ui.base.modifier = 0x03;
    ui.pid = pid;
    ui.payload = data;
    ui.payload_len = len;

    size_t encoded_len;
    uint8_t err;
    uint8_t *encoded = ax25_unnumbered_information_frame_encode(&ui, &encoded_len, &err);
    if (!encoded)
        return 3;

    conn->callbacks.transmit(conn->user_data, encoded, encoded_len);

    // Update statistics for UI frames sent
    conn->stats.uframe_sent++;
    if (conn->stats.uframe_sent == 0) {
        conn->stats.uframe_sent = 1;
    }

    if (encoded != NULL) {
        free(encoded);
        encoded = NULL;
    }

    return 0;
}

// Send TEST command - AX.25 v2.2 Section 6.4.13
uint8_t ax25_send_test_command(ax25_connection_t *conn, uint8_t *payload, size_t payload_len) {
    if (!conn || !conn->callbacks.transmit) {
        return 1;  // Invalid parameters
    }

    if (payload_len > 256) {
        return 2;  // Payload too large
    }

    // Build TEST command frame
    ax25_test_frame_t test;
    test.base.base.header = conn->peer_addr;
    test.base.base.header.cr = true;  // Command
    test.base.base.type = AX25_FRAME_UNNUMBERED_TEST;
    test.base.pf = true;  // Poll bit set
    test.base.modifier = 0xE3;  // TEST modifier
    test.payload = payload;
    test.payload_len = payload_len;

    size_t len;
    uint8_t err;
    uint8_t *encoded = ax25_test_frame_encode(&test, &len, &err);
    if (!encoded) {
        return 3;  // Encoding failed
    }

    // Transmit the frame
    conn->callbacks.transmit(conn->user_data, encoded, len);

    if (encoded != NULL) {
        free(encoded);
        encoded = NULL;
    }

    // Update statistics - mark test as sent and waiting for response
    conn->test_stats.test_sent++;
    conn->test_stats.test_sequence++;
    // Note: last_test_tick should be set by caller when they call this function
    // We can't set it here because we don't have access to current_tick
    // Caller should call: conn->test_stats.last_test_tick = current_tick after this

    return 0;  // Success
}

// Get average round-trip time from TEST frames
uint32_t ax25_get_average_rtt_ms(ax25_connection_t *conn) {
    if (!conn || conn->test_stats.rtt_count == 0) {
        return 0;
    }

    // Calculate average RTT
    // Use 32-bit arithmetic only, avoid division if possible
    uint32_t sum = conn->test_stats.rtt_sum;
    uint16_t count = conn->test_stats.rtt_count;

    // Simple division for average
    uint32_t avg_ticks = sum / count;

    // Convert from 10ms ticks to milliseconds
    return avg_ticks * 10;
}

// Apply negotiated parameters to connection - AX.25 v2.2 Section 6.7.2
uint8_t ax25_apply_negotiated_params(ax25_mgmt_context_t *mgmt_ctx, ax25_connection_t *conn, ax25_physical_t *phys) {
    if (!mgmt_ctx || !conn) {
        return 1;  // Invalid parameters
    }

    // Only apply if negotiation completed successfully
    if (mgmt_ctx->state != AX25_MGMT_NEGOTIATED) {
        return 1;  // Negotiation not complete
    }

    // Apply full-duplex setting
    // Per AX.25 v2.2 Section 6.7.2: both stations must agree to full-duplex
    conn->full_duplex = mgmt_ctx->agreed_params.full_duplex;

    // Propagate full-duplex flag to physical layer so CSMA is bypassed (C2b path).
    // phys may be NULL in unit-test environments that have no physical layer instance.
    if (phys) {
        ax25_physical_set_duplex(phys, conn->full_duplex);
    }

    // If switching to full-duplex, cancel any T2 ACK timer that may have been
    // armed during a prior half-duplex exchange.  In full-duplex mode,
    // start_t2_response() sends RR immediately and never sets t2_running, so a
    // leftover armed T2 would fire a spurious RR on the next ax25_tick() call,
    // violating AX.25 v2.2 Section 6.7.1.2.
    if (conn->full_duplex) {
        conn->t2_running = false;
        conn->t2_ack_pending = false;
        // t2_pending_nr is irrelevant when t2_running is false — leave unchanged
    }

    // Apply modulo (affects sequence number range)
    if (mgmt_ctx->agreed_params.modulo128) {
        conn->vars.mod = 128;
    } else {
        conn->vars.mod = 8;
    }

    // Apply reject mode (SREJ/REJ support)
    if (mgmt_ctx->agreed_params.selective_reject && mgmt_ctx->agreed_params.implicit_reject) {
        conn->rej_mode = AX25_REJ_MODE_SREJ_REJ;  // Both SREJ and REJ
    } else if (mgmt_ctx->agreed_params.selective_reject) {
        conn->rej_mode = AX25_REJ_MODE_SREJ;  // SREJ only
    } else if (mgmt_ctx->agreed_params.implicit_reject) {
        conn->rej_mode = AX25_REJ_MODE_REJ;  // REJ only
    } else {
        conn->rej_mode = AX25_REJ_MODE_NONE;  // No reject support
    }

    // Apply timer values (convert from milliseconds to 10ms ticks)
    conn->timers.t1 = mgmt_ctx->agreed_params.ack_timer / 10;

    // Full-duplex T1 reduction.
    // The negotiated ack_timer was derived with "take the max" logic, which is
    // conservative for half-duplex (CSMA backoff + peer key-up + TX/RX contention).
    // In full-duplex both transmitters are permanently keyed; there is no CSMA,
    // no peer key-up delay and no channel contention.  Using 50% of the negotiated
    // value is a safe initial estimate consistent with AX.25 v2.2 Section 6.7.1.1
    // ("T1 should be at least 2x the measured round-trip time").
    // ax25_adjust_t1_adaptive() will refine T1 further once TEST-frame RTT
    // samples are available.  Minimum floor: 10 ticks (100 ms).
    // All arithmetic is 32-bit integer only -- no floating point.
    if (conn->full_duplex && conn->timers.t1 > 0u) {
        uint16_t fd_t1 = (uint16_t) (conn->timers.t1 / 2u);
        if (fd_t1 < 10u) {
            fd_t1 = 10u;   // floor: 100 ms
        }
        conn->timers.t1 = fd_t1;
    }

    // Apply retry count
    conn->timers.n2 = mgmt_ctx->agreed_params.retries;

    // Apply window size
    // include PI=8 (Window Size Rx) in its XID, agreed_params.window_size stays at 7
    // (the modulo-8 XID offer default); applying 7/127 would run the link at ~5.5%
    // window efficiency; per AX.25 v2.2 the spec default for modulo-128 is 32, so
    // promote any modulo-8-default value to 32 when we are in modulo-128 mode;
    // always clamp to the smaller of the protocol maximum and the queue array limit
    uint8_t max_proto_k = (conn->vars.mod == 128) ? 127u : 7u;
    uint8_t max_queue_k = (uint8_t) (AX25_MAX_QUEUE_SIZE - 1);
    uint8_t max_k = (max_proto_k < max_queue_k) ? max_proto_k : max_queue_k;
    uint8_t spec_def_k = (conn->vars.mod == 128) ? 32u : 7u;
    uint8_t req_k = mgmt_ctx->agreed_params.window_size;
    // promote modulo-8 default to spec default for modulo-128
    if (conn->vars.mod == 128 && req_k <= 7u) {
        req_k = spec_def_k;
    }
    conn->timers.k = (req_k <= max_k) ? req_k : max_k;

    // Apply maximum I-field length
    conn->timers.n1 = mgmt_ctx->agreed_params.ifield_length;

    // Reinitialize segmenter with the newly negotiated N1 value.
    // segment_size = N1 - 2 (1-byte segment header + 1-byte original-PID overhead).
    // Re-wires callbacks because ax25_segmenter_init zeroes the entire struct.
    ax25_segmenter_init(&conn->segmenter, conn->timers.n1);
    conn->segmenter.transmit_iframe = seg_transmit_cb;
    conn->segmenter.on_reassembly_complete = seg_reassembly_complete_cb;
    conn->segmenter.user_data = conn;

    // Restore error/retransmit callbacks after re-init (ax25_segmenter_init zeroes struct)
    conn->segmenter.on_reassembly_error = seg_reassembly_error_cb;
    conn->segmenter.on_request_retransmit = seg_request_retransmit_cb;

    return 0;  // Success
}

// Check if full-duplex operation is enabled
bool ax25_is_full_duplex(ax25_connection_t *conn) {
    if (!conn) {
        return false;  // Default to half-duplex on error
    }

    return conn->full_duplex;
}

// API: Register protocol handler for specific PID
uint8_t ax25_register_protocol(ax25_connection_t *conn, uint8_t pid, ax25_protocol_handler_t handler, void *user_data) {
    if (!conn || !handler) {
        return 1;  // Invalid parameters
    }

    // Check if already registered - update existing entry
    for (uint8_t i = 0; i < AX25_MAX_PROTOCOL_HANDLERS; i++) {
        if (conn->protocols[i].active && conn->protocols[i].pid == pid) {
            // Update existing handler
            conn->protocols[i].handler = handler;
            conn->protocols[i].user_data = user_data;
            return 0;  // Success (updated)
        }
    }

    // Find free slot for new registration
    for (uint8_t i = 0; i < AX25_MAX_PROTOCOL_HANDLERS; i++) {
        if (!conn->protocols[i].active) {
            conn->protocols[i].pid = pid;
            conn->protocols[i].handler = handler;
            conn->protocols[i].user_data = user_data;
            conn->protocols[i].active = true;
            return 0;  // Success (new)
        }
    }

    return 2;  // No free slots available
}

// API: Unregister protocol handler for specific PID
void ax25_unregister_protocol(ax25_connection_t *conn, uint8_t pid) {
    if (!conn) {
        return;
    }

    for (uint8_t i = 0; i < AX25_MAX_PROTOCOL_HANDLERS; i++) {
        if (conn->protocols[i].active && conn->protocols[i].pid == pid) {
            conn->protocols[i].active = false;
            conn->protocols[i].handler = NULL;
            conn->protocols[i].user_data = NULL;
            return;
        }
    }
}

// API: Set default protocol handler for unregistered PIDs
void ax25_set_default_protocol_handler(ax25_connection_t *conn, ax25_protocol_handler_t handler, void *user_data) {
    if (!conn) {
        return;
    }

    conn->default_handler = handler;
    conn->default_user_data = user_data;
}

// Adaptive T1 adjustment based on measured RTT from TEST frames
// Per AX.25 v2.2 Section 6.7.1.1: T1 should be at least 2x round-trip time
void ax25_adjust_t1_adaptive(ax25_connection_t *conn) {
    if (!conn) {
        return;
    }

    // Use TEST frame RTT measurements to adjust T1
    if (conn->test_stats.rtt_count > 0) {
        // Calculate average RTT (already in 10ms ticks from test_stats)
        uint32_t avg_rtt = conn->test_stats.rtt_sum / conn->test_stats.rtt_count;

        // Safety margin: FD has no CSMA/key-up delay so 100 ms is sufficient;
        // HD requires 300 ms to cover worst-case CSMA backoff + T103 key-up.
        // NOTE: call ax25_adjust_t1_adaptive() after each received TEST response
        // to progressively refine T1 based on actual link RTT.
        uint32_t margin = conn->full_duplex ? 10u : 30u;  // FD: 100ms, HD: 300ms

        uint32_t new_t1 = avg_rtt * 2 + margin;

        // Clamp to reasonable range per AX.25 v2.2
        // Minimum: 100ms (10 ticks), Maximum: 30s (3000 ticks)
        if (new_t1 < 10) {
            new_t1 = 10;    // Min 100ms
        }
        if (new_t1 > 3000) {
            new_t1 = 3000;  // Max 30s
        }

        conn->timers.t1 = (uint16_t) new_t1;
    }
}

// Get statistics structure (read-only access)
// conn: Connection context
// Returns: Pointer to statistics structure or NULL on error
const ax25_statistics_t* ax25_get_statistics(ax25_connection_t *conn) {
    if (!conn) {
        return NULL;
    }

    // Update current state variables
    conn->stats.current_vs = conn->vars.vs;
    conn->stats.current_vr = conn->vars.vr;
    conn->stats.current_va = conn->vars.va;
    conn->stats.tx_queue_depth = conn->tx_queue.count;

    return &conn->stats;
}

// Reset all statistics counters to zero
// conn: Connection context
void ax25_reset_statistics(ax25_connection_t *conn) {
    if (!conn) {
        return;
    }

    memset(&conn->stats, 0, sizeof(ax25_statistics_t));
}

// Send data with optional FX.25 Forward Error Correction
// This function wraps ax25_send_data() with FX.25 encoding when requested
uint8_t ax25_send_data_with_fec(ax25_connection_t *conn, uint8_t *data, size_t len, uint8_t pid, bool use_fx25, uint8_t channel_quality) {
    if (!conn || !data || len == 0) {
        return 1;
    }

    // If FX.25 not requested, use standard AX.25
    if (!use_fx25) {
        return ax25_send_data(conn, data, len, pid);
    }

    // DL-DATA request: per AX.25 v2.2 Section 6.4.11 and C4 SDL, new I-frames
    // must not be sent in TIMER_RECOVERY state (retransmission in progress).
    // Return code 6 ("recovery in progress") instead of allowing transmission so
    // Layer 3 knows the link is alive but temporarily flow-controlled, not down.
    // This check is required here because the FX.25 path bypasses ax25_send_data().
    if (conn->state == AX25_STATE_TIMER_RECOVERY)
        return 6;  // Recovery in progress - retry later

    // Gate 1: enforce send window BEFORE any encoding or transmission.
    // Previously these checks appeared after transmit, allowing frames to be
    // sent when the window was already full (Violation A) and V(S) to be
    // incremented after a potential ACK arrival (Violation B).
    uint8_t outstanding = AX25_OUTSTANDING(conn->vars.vs, conn->vars.va, conn->vars.mod);
    if (outstanding >= conn->timers.k || conn->tx_queue.count >= AX25_MAX_QUEUE_SIZE) {
        return 3;  // Window full - mirrors ax25_send_data() return code
    }
    if (conn->peer_busy) {
        return 5;  // Peer sent RNR - cannot send I-frames
    }

    // Calculate frame size: address(14) + control(1-2) + pid(1) + data(len)
    bool extended = (conn->vars.mod == 128);
    uint16_t ctrl_len = extended ? 2 : 1;
    uint16_t ax25_frame_len = 14 + ctrl_len + 1 + len;

    // Allocate buffer for AX.25 frame (static allocation to avoid malloc on MCU)
    uint8_t ax25_frame[256];  // Max reasonable frame size
    if (ax25_frame_len > sizeof(ax25_frame)) {
        return 2;  // Frame too large
    }

    uint16_t offset = 0;

    // Build address field (destination + source, 7 bytes each)
    memcpy(&ax25_frame[offset], &conn->peer_addr.destination, 7);
    offset += 7;
    memcpy(&ax25_frame[offset], &conn->peer_addr.source, 7);
    offset += 7;

    // Snapshot V(S) and V(R) before any state changes so the control field
    // is encoded with consistent values even if a rollback is needed later.
    uint8_t snap_vs = conn->vars.vs;
    uint8_t snap_nr = conn->vars.vr;

    if (extended) {
        // Modulo-128: 2 bytes
        ax25_frame[offset++] = (snap_vs << 1);  // N(S) in bits 1-7, bit 0 = 0
        ax25_frame[offset++] = (snap_nr << 1);  // N(R) in bits 1-7, bit 0 = P
    } else {
        // Modulo-8: 1 byte
        ax25_frame[offset++] = (snap_nr << 5) | (snap_vs << 1);  // N(R)|N(S)|0
    }

    // Add PID
    ax25_frame[offset++] = pid;

    // Add data
    memcpy(&ax25_frame[offset], data, len);
    offset += len;

    // Advance V(S) NOW, before fx25_encode() or transmit(), so that any ACK
    // arriving during or after transmission sees a valid V(S) >= N(R).
    conn->vars.vs = INC_MOD(conn->vars.vs, conn->vars.mod);

    // Cancel T2 - ACK is piggybacked in the N(R) of this outgoing I-frame
    cancel_t2(conn);

    // Start T1 acknowledgment timer if not already running
    if (conn->t1_start_tick == 0) {
        // Use AX25_T1_PENDING (UINT32_MAX) not 1; avoids off-by-one on MCUs
        // that boot with tick counter near zero (see fix 1.3).
        conn->t1_start_tick = AX25_T1_PENDING;
    }
    // T3 only runs when there are no outstanding frames
    conn->t3_start_tick = 0;

    // Now wrap with FX.25
    fx25_frame_t fx25;
    uint8_t mode = fx25_select_mode_for_conditions(ax25_frame_len, channel_quality);
    uint8_t err = fx25_encode(ax25_frame, ax25_frame_len, mode, &fx25);

    if (err != 0) {
        // Roll back V(S) so the window accounting stays consistent
        conn->vars.vs = snap_vs;
        return 3;  // FX.25 encoding failed
    }

    // Transmit FX.25 frame: correlation tag + RS codeword
    if (conn->callbacks.transmit) {
        // Allocate temporary buffer for transmission
        uint16_t tx_len = 8 + fx25.codeword_len;
        uint8_t *tx_buffer = malloc(tx_len);

        if (!tx_buffer) {
            fx25_frame_free(&fx25);
            conn->vars.vs = snap_vs;  // Roll back V(S) on alloc failure
            return 4;  // Memory allocation failed
        }

        // Copy correlation tag (8 bytes)
        memcpy(tx_buffer, fx25.correlation_tag, 8);

        // Copy RS codeword (data + parity)
        memcpy(tx_buffer + 8, fx25.rs_codeword, fx25.codeword_len);

        // Transmit
        conn->callbacks.transmit(conn->user_data, tx_buffer, tx_len);

        // Cleanup
        if (tx_buffer != NULL) {
            free(tx_buffer);
            tx_buffer = NULL;
        }
    }

    // Free FX.25 frame resources
    fx25_frame_free(&fx25);

    // Update statistics
    conn->stats.iframe_sent++;
    if (conn->stats.iframe_sent == 0) {
        conn->stats.iframe_sent = 1;  // Prevent overflow
    }
    conn->stats.bytes_sent += len;
    if (conn->stats.bytes_sent < len) {
        conn->stats.bytes_sent = len;  // Prevent overflow
    }

    conn->stats.bytes_sent += len;

    return 0;
}

void ax25_connection_cleanup(ax25_connection_t *conn) {
    if (!conn)
        return;
    // Drain and free every frame still queued for retransmission
    while (conn->tx_queue.count > 0) {
        free(conn->tx_queue.frames[conn->tx_queue.head]);
        conn->tx_queue.frames[conn->tx_queue.head] = NULL;
        conn->tx_queue.head = (conn->tx_queue.head + 1) % AX25_MAX_QUEUE_SIZE;
        conn->tx_queue.count--;
    }
}
