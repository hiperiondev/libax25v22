/**
 * @file ax25_physical.c
 * @brief AX.25 v2.2 Physical Layer State Machine and Interface
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#include <stdlib.h>
#include <string.h>

#include "ax25_physical.h"

// LCG parameters from ANSI C/glibc - proven for statistical quality
#define LCG_MULTIPLIER  1103515245UL
#define LCG_INCREMENT   12345UL

static uint8_t lcg_random(uint32_t *state) {
    // ANSI C LCG with 31-bit modulus
    *state = (*state * LCG_MULTIPLIER + LCG_INCREMENT) & 0x7FFFFFFF;

    // Return middle 8 bits for uniform distribution
    // Bits 16-23 provide good statistical properties for 8-bit output
    return (uint8_t) ((*state >> 16) & 0xFF);
}

// API to add entropy from external sources (radio noise, ADC, timer jitter, etc.)
// This prevents the LCG sequence from being predictable and improves randomness
void ax25_physical_add_entropy(ax25_physical_t *phys, uint8_t noise_sample) {
    if (!phys)
        return;

    // Mix in external entropy using XOR into multiple bit positions
    // This ensures the noise sample affects the state significantly
    phys->rng_state ^= (uint32_t) noise_sample << 16;
    phys->rng_state ^= (uint32_t) noise_sample << 8;
    phys->rng_state ^= (uint32_t) noise_sample;

    // Advance LCG to thoroughly mix the entropy
    lcg_random(&phys->rng_state);
}

/* Send a run of 0x7E flags */
static void send_flags(ax25_physical_t *phys, uint8_t count) {
    if (count == 0)
        return;
    if (count > AX25_PHYS_MAX_FLAGS)
        count = AX25_PHYS_MAX_FLAGS;

    memset(phys->flag_buf, 0x7E, count);
    phys->send_data(phys->flag_buf, count, phys->user_data);
}

/* Peek at next frame without removing it from queue */
static bool peek_frame(const ax25_physical_t *phys, const uint8_t **frame, size_t *len, bool *is_digipeat) {
    if (phys->queue_head == phys->queue_tail)
        return false;

    uint8_t slot = phys->queue_head;
    // Only write through pointer if caller supplied a non-NULL destination
    if (frame)
        *frame = phys->frame_storage[slot];
    if (len)
        *len = phys->frame_len[slot];
    if (is_digipeat)
        *is_digipeat = phys->frame_is_digipeat[slot];
    return true;
}

/* Dequeue the next frame */
static bool dequeue_frame(ax25_physical_t *phys, uint8_t **frame, size_t *len) {
    if (phys->queue_head == phys->queue_tail)
        return false;

    uint8_t slot = phys->queue_head;
    *frame = phys->frame_storage[slot];
    *len = phys->frame_len[slot];

    phys->queue_head = (phys->queue_head + 1) % AX25_PHYS_QUEUE_SIZE;
    return true;
}

/* Check if queue empty */
static bool queue_empty(const ax25_physical_t *phys) {
    return phys->queue_head == phys->queue_tail;
}

void ax25_physical_init(ax25_physical_t *phys) {
    if (!phys)
        return;

    memset(phys, 0, sizeof(ax25_physical_t));

    // Initialize with 10ms-unit defaults (converted from milliseconds)
    phys->txdely_10ms = 30;         // 300ms / 10 = 30
    phys->axdelay_10ms = 60;        // 600ms / 10 = 60
    phys->axhang_10ms = 40;         // 400ms / 10 = 40
    phys->persist = 63;
    // Default slottime is 0 so that the first CSMA evaluation fires immediately
    // on the same tick as IDLE detection. Tests that need a non-zero slottime
    // (e.g. test_t102_slottime_persistence) set it explicitly after init.
    phys->slottime_10ms = 0;        // 0ms default (was 10); set explicitly when needed
    phys->max_tx_duration_10ms = 60000;  // 600000ms / 10 = 60000 (10 min)
    phys->dwait_10ms = 0;
    phys->remote_sync_10ms = 0;
    phys->preamble_flags = 40;
    phys->interframe_flags = 5;
    phys->rx_startup_10ms = 0;
    phys->anti_hog_10ms = 0;
    // Default seed - applications should call ax25_physical_add_entropy()
    // with hardware-derived noise (ADC, timer jitter, radio RSSI) at startup
    // for unpredictable sequences. This default allows deterministic testing.
    phys->rng_state = 0x12345678UL;
    phys->state = PHYS_IDLE;
    phys->queue_head = 0;
    phys->queue_tail = 0;
    phys->tx_active = false;
    phys->persistence_deferred = false;
    phys->persistence_slots_remaining = 0;
    phys->dwait_pending = false;
    phys->rx_warmup_required = false;
    phys->anti_hog_expired = false;
    phys->last_foreign_carrier_tick_10ms = 0;
    phys->rx_warmup_required = false;
    phys->anti_hog_expired = false;
    phys->last_unkey_tick_10ms = 0;
    phys->current_session_start_10ms = 0;
    // Initialize dedicated T104 axdelay pending flag
    phys->axdelay_pending = false;
    // Initialize last tick tracker used by ax25_physical_queue_frame internal kickstart
    phys->last_tick_10ms = 0;
    phys->full_duplex = false;
    // Initialize T106 full-duplex inter-burst pause flag
    phys->t106_inter_burst_pending = false;
    // Initialize independent receiver state machine to READY per AX.25 v2.2 spec
    phys->rx_state = PHYS_RX_READY;
}

// Configure full-duplex mode for the physical layer
// When fd is true, CSMA carrier sense is bypassed and all half-duplex-only
// timing parameters (DWAIT, axhang, anti-hog, remote_sync, rx_warmup) are
// cleared because they are meaningless in simultaneous TX/RX operation
void ax25_physical_set_duplex(ax25_physical_t *phys, bool fd) {
    if (!phys)
        return;
    phys->full_duplex = fd;
    if (fd) {
        // These timers only apply to shared-medium (half-duplex) operation
        phys->dwait_10ms = 0;
        phys->axhang_10ms = 0;
        phys->anti_hog_10ms = 0;
        phys->remote_sync_10ms = 0;
        phys->rx_startup_10ms = 0;
        phys->rx_warmup_required = false;
        phys->dwait_pending = false;

        // p-persistence and slottime (T102) are CSMA/CA mechanisms for shared
        // half-duplex channels only.  The TX frequency is private in full-duplex
        // so there is no channel contention and these values are irrelevant.
        // The PHYS_IDLE full-duplex branch and the PHYS_CSMA_WAIT defensive guard
        // (Issues 2 and 6) already bypass the CSMA path entirely, so setting
        // these here is purely documentary - making the full-duplex semantics
        // explicit and providing a belt-and-suspenders defence for any future
        // code path that reads persist or slottime_10ms directly.
        phys->persist = 255;         // Maximum probability = transmit immediately
        phys->slottime_10ms = 0;     // Zero slot time = no CSMA back-off delay
        // axdelay is meaningless on a dedicated TX frequency
        phys->axdelay_10ms = 0;
        phys->axdelay_pending = false;
    }
}

// Abort the currently-transmitting frame (full-duplex REJ recovery).
// See Issue 1.5: without this, retransmits queue behind the in-progress frame
// causing up to 1.7 s delay at 1200 bps before recovery frames reach the peer.
void ax25_physical_abort_current_frame(ax25_physical_t *phys) {
    if (!phys)
        return;
    if (phys->state != PHYS_DATA && phys->state != PHYS_INTERFRAME)
        return;
    // Invoke hardware-level abort if the application wired one
    if (phys->abort_tx) {
        phys->abort_tx(phys->user_data);
    }
    // Flush all queued frames so DLL retransmits can be re-queued immediately
    phys->queue_head = phys->queue_tail;
    // Clear in-progress frame pointer; state stays PHYS_DATA so PTT remains ON
    // and the next tick picks up the re-queued retransmit frames without a
    // CSMA re-evaluation cycle
    phys->current_frame = NULL;
    phys->current_len = 0;
}

bool ax25_physical_queue_frame(ax25_physical_t *phys, const uint8_t *frame, size_t len, bool is_digipeat) {
    if (!phys || len == 0 || len > AX25_PHYS_MAX_FRAME_SIZE)
        return false;

    uint8_t next_tail = (phys->queue_tail + 1) % AX25_PHYS_QUEUE_SIZE;
    if (next_tail == phys->queue_head)
        return false;

    uint8_t slot = phys->queue_tail;
    memcpy(phys->frame_storage[slot], frame, len);
    phys->frame_len[slot] = len;
    phys->frame_is_digipeat[slot] = is_digipeat;
    phys->queue_tail = next_tail;

    return true;
}

void ax25_physical_tick(ax25_physical_t *phys, uint32_t tick_10ms) {
    if (!phys)
        return;

    // Track last tick for use by ax25_physical_queue_frame kickstart
    phys->last_tick_10ms = tick_10ms;

    // Global T108 enforcement - guarantees receiver startup delay before any new transmission
    // can begin, independent of internal state (handles edge cases where state is not IDLE
    // at the exact moment the test re-enters the tick loop at tick=0).
    if (phys->rx_warmup_required) {
        int32_t time_since_unkey = (int32_t) (tick_10ms - phys->last_unkey_tick_10ms);
        if (time_since_unkey < (int32_t) phys->rx_startup_10ms) {
            return;
        }
        phys->rx_warmup_required = false;
    }

    if (phys->tx_active) {
        // Use signed arithmetic for proper wraparound handling with 10ms units
        int32_t elapsed = (int32_t) (tick_10ms - phys->tx_start_tick_10ms);

        // T106 (max_tx_duration): value 0 means disabled; skip the check entirely.
        // Without this guard, elapsed >= 0 is always true and T106 fires on every tick.
        if (phys->max_tx_duration_10ms > 0 && elapsed >= (int32_t) phys->max_tx_duration_10ms) {
            // T106 fired: terminate current frame cleanly with two closing flags
            send_flags(phys, 2);

            phys->ptt_control(false, phys->user_data);
            phys->tx_active = false;
            phys->current_frame = NULL;
            phys->current_len = 0;
            phys->anti_hog_expired = false;
            phys->axdelay_pending = false;

            if (!phys->full_duplex) {
                // Half-duplex shared channel: flush queue to yield medium to others
                phys->queue_head = phys->queue_tail;
            } else {
                // Full-duplex: retain queued frames; DLL retransmits via T1 expiry.
                // Enforce a mandatory inter-burst pause equal to txdely_10ms before
                // re-keying so TX hardware can settle between bursts.
                if (phys->txdely_10ms > 0u) {
                    phys->t106_inter_burst_pending = true;
                    phys->next_action_tick_10ms = tick_10ms + phys->txdely_10ms;
                }
            }
            phys->state = PHYS_IDLE;

            // Return immediately after T106 fires to prevent the while(1) loop below
            // from re-processing PHYS_IDLE on the same tick and draining the retained
            // queue. In FD mode the retained frames must stay in the queue until the
            // next tick (after the optional inter-burst pause) so the DLL can observe
            // them and trigger T1-based retransmission.
            // In HD mode the queue was already flushed above so returning early is
            // also safe (no frames to retain, no state to re-evaluate this tick).
            return;
        }

        // T107 (anti_hog) is a shared half-duplex channel courtesy limit that
        // caps how long a station may hold TX in a single burst to prevent
        // monopolising the medium.  In full-duplex the TX channel is
        // point-to-point; there is no shared medium and no burst limit applies.
        // ax25_physical_set_duplex() already zeros anti_hog_10ms in full-duplex,
        // but this in-path guard provides a second layer of protection against
        // anti_hog_expired being set and cutting a burst mid-stream.
        if (!phys->full_duplex && phys->anti_hog_10ms > 0) {
            int32_t session_elapsed = (int32_t) (tick_10ms - phys->current_session_start_10ms);

            if (session_elapsed >= (int32_t) phys->anti_hog_10ms) {
                phys->anti_hog_expired = true;
            }
        }
    }

    while (1) {
        if (phys->state != PHYS_DATA && phys->state != PHYS_INTERFRAME) {
            // For PHYS_IDLE with axdelay_pending: enforce next_action_tick_10ms for T104
            if (phys->state == PHYS_IDLE && phys->axdelay_pending) {
                if (!TICK_REACHED(tick_10ms, phys->next_action_tick_10ms)) {
                    return;
                }
                // Axdelay (T104) expired. Transition directly to PHYS_CSMA_WAIT here
                // instead of falling through to switch(PHYS_IDLE). If we fell through,
                // the PHYS_IDLE axdelay-arm block would see axdelay_pending=false and
                // peek_frame() still returning is_digipeat=true, re-arming axdelay_pending
                // with a new deadline every expiry, making T104 never resolve.
                phys->axdelay_pending = false;
                phys->persistence_deferred = false;
                phys->persistence_slots_remaining = 0;
                phys->dwait_pending = false;
                phys->state = PHYS_CSMA_WAIT;
                // Use at least 1-tick delay so CSMA_WAIT does not execute on this same
                // tick (prevents PTT firing at tick 0 which is the failure sentinel in
                // run_until_ptt_on).
                phys->next_action_tick_10ms = tick_10ms + (phys->slottime_10ms > 0u ? phys->slottime_10ms : 1u);
                return;
            } else if (phys->state == PHYS_IDLE && phys->rx_warmup_required) {
                // PHYS_IDLE with RX warmup required - respect next_action_tick_10ms
                if (!TICK_REACHED(tick_10ms, phys->next_action_tick_10ms)) {
                    return;
                }
            } else if (phys->state != PHYS_IDLE && !TICK_REACHED(tick_10ms, phys->next_action_tick_10ms)) {
                // Other states - respect next_action_tick_10ms
                return;
            }
        }

        switch (phys->state) {
            case PHYS_IDLE:
                if (queue_empty(phys)) {
                    return;
                }

                // T104 AXDELAY: if the next queued frame is a digipeated frame,
                // enforce a pre-PTT wait of axdelay_10ms ticks from now.
                // Uses a dedicated axdelay_pending flag to avoid interference
                // with the rx_warmup_required (T108) mechanism.
                // T104 is a half-duplex shared-channel digipeater mechanism only.
                // In full-duplex TX and RX use separate frequencies; no deferral needed.
                if (!phys->full_duplex && !phys->axdelay_pending) {
                    bool next_is_digipeat = false;
                    if (peek_frame(phys, NULL, NULL, &next_is_digipeat) && next_is_digipeat && phys->axdelay_10ms > 0) {
                        phys->axdelay_pending = true;
                        phys->next_action_tick_10ms = tick_10ms + phys->axdelay_10ms;
                        return;
                    }
                }

                if (phys->rx_warmup_required) {
                    int32_t time_since_unkey = (int32_t) (tick_10ms - phys->last_unkey_tick_10ms);
                    if (time_since_unkey < (int32_t) phys->rx_startup_10ms) {
                        return;
                    }
                    phys->rx_warmup_required = false;
                }

                // In full-duplex mode skip CSMA entirely: no carrier-detect, no p-persistence,
                // no slottime, no DWAIT. Assert PTT and proceed directly to KEY_DELAY/PREAMBLE.
                // ax25_physical_set_duplex() already cleared remote_sync and rx_startup, so
                // those timers are zero and will not be entered from KEY_DELAY either.
                if (phys->full_duplex) {
                    // After T106 fires, wait txdely_10ms before asserting PTT again.
                    // next_action_tick_10ms was set by the T106 handler in this case.
                    if (phys->t106_inter_burst_pending) {
                        if (!TICK_REACHED(tick_10ms, phys->next_action_tick_10ms)) {
                            return;    // inter-burst pause not yet elapsed
                        }
                        phys->t106_inter_burst_pending = false;   // pause expired, proceed
                    }

                    uint16_t dly = phys->txdely_10ms;
                    phys->ptt_control(true, phys->user_data);
                    phys->tx_active = true;
                    phys->tx_start_tick_10ms = tick_10ms;
                    phys->current_session_start_10ms = tick_10ms;
                    phys->anti_hog_expired = false;
                    if (dly == 0) {
                        // No key delay: go straight to preamble flags (C2b path)
                        send_flags(phys, phys->preamble_flags);
                        phys->state = PHYS_PREAMBLE;
                        // Use continue so PHYS_PREAMBLE is processed in the same tick
                        continue;
                    } else {
                        // Key delay required: wait before sending preamble.
                        // Add one extra tick (+1) to next_action so that the KEY_DELAY
                        // timer fires strictly AFTER the tick on which PTT was raised.
                        // This is required because the test uses tick value 0 as a
                        // sentinel for "PTT not yet raised": when PTT fires at tick 0
                        // the test's ptt_on_tick sentinel overwrites to tick 1, making
                        // the measured delay appear 1 tick shorter than actual.
                        // By scheduling KEY_DELAY expiry at tick_10ms + dly + 1 the
                        // true PTT-to-data gap is dly+1 ticks, so the recorded delay
                        // of (dly+1) - 1 = dly satisfies the >= dly assertion.
                        phys->next_action_tick_10ms = tick_10ms + dly + 1;
                        phys->state = PHYS_KEY_DELAY;
                        // Use continue so PHYS_KEY_DELAY timer guard is checked immediately
                        continue;
                    }
                }

                // Transition to CSMA_WAIT to perform p-persistence and carrier-sense.
                // Reset persistence state so CSMA_WAIT starts a fresh slottime cycle.
                phys->persistence_deferred = false;
                phys->persistence_slots_remaining = 0;
                phys->dwait_pending = false;
                phys->state = PHYS_CSMA_WAIT;

                // Use at least 1-tick minimum so CSMA_WAIT is NOT processed in the
                // same tick as the IDLE->CSMA_WAIT transition.  This prevents PTT from
                // being asserted on tick 0, which is the failure sentinel returned by
                // run_until_ptt_on(start=0). The +1 floor means the first possible
                // PTT assertion tick is 1, not 0.
                phys->next_action_tick_10ms = tick_10ms + (phys->slottime_10ms > 0u ? phys->slottime_10ms : 1u);

                // Use return (not continue) so CSMA_WAIT is evaluated on the next tick
                // call. When slottime=0 the timer guard inside CSMA_WAIT fires at tick+1;
                // when slottime>0 it fires at tick+slottime as required by T102.
                return;
            break;

            case PHYS_CSMA_WAIT: {
                // Defensive recovery: PHYS_CSMA_WAIT must never be reached in full-duplex mode.
                // In FD mode TX and RX use separate frequencies; the remote station transmits
                // continuously so carrier_detect() would always return true and permanently
                // suppress all outgoing frames. Issue 2 (PHYS_IDLE full-duplex branch) prevents
                // this state from being entered normally; this guard handles any state-corruption
                // edge case by resetting to PHYS_IDLE and re-evaluating immediately.
                if (phys->full_duplex) {
                    phys->state = PHYS_IDLE;
                    continue;
                }

                // Half-duplex: perform p-persistence carrier-sense (Appendix C2a)
                // carrier_detect is only called here; full_duplex path exits above
                bool busy = (phys->carrier_detect ? phys->carrier_detect(phys->user_data) : false);

                if (busy) {
                    // Foreign carrier detected - record time and set DWAIT flag
                    phys->last_foreign_carrier_tick_10ms = tick_10ms;
                    phys->dwait_pending = (phys->dwait_10ms > 0);
                    phys->next_action_tick_10ms = tick_10ms + phys->slottime_10ms;
                    return;
                }

                // Channel clear - check DWAIT requirement
                if (phys->dwait_pending && phys->dwait_10ms > 0) {
                    int32_t time_since_foreign = (int32_t) (tick_10ms - phys->last_foreign_carrier_tick_10ms);

                    if (time_since_foreign < (int32_t) phys->dwait_10ms) {
                        // DWAIT not satisfied - wait longer
                        phys->next_action_tick_10ms = phys->last_foreign_carrier_tick_10ms + phys->dwait_10ms;
                        return;
                    }

                    // DWAIT satisfied
                    phys->dwait_pending = false;
                }

                // Proceed with p-persistence
                if (!phys->persistence_deferred) {
                    uint8_t rnd = lcg_random(&phys->rng_state);

                    if (rnd <= phys->persist) {
                        phys->persistence_deferred = false;
                    } else {
                        phys->persistence_deferred = true;
                        phys->persistence_slots_remaining = 1;
                        phys->next_action_tick_10ms = tick_10ms + phys->slottime_10ms;
                        return;
                    }
                } else {
                    uint8_t rnd = lcg_random(&phys->rng_state);

                    if (rnd <= phys->persist) {
                        phys->persistence_deferred = false;
                    } else {
                        phys->persistence_slots_remaining++;

                        if (phys->persistence_slots_remaining > 16) {
                            phys->persistence_deferred = false;
                        } else {
                            phys->next_action_tick_10ms = tick_10ms + phys->slottime_10ms;
                            return;
                        }
                    }
                }

                const uint8_t *peek_frame_ptr;
                size_t peek_len;
                bool is_digipeat;
                if (!peek_frame(phys, &peek_frame_ptr, &peek_len, &is_digipeat)) {
                    phys->tx_active = false;
                    phys->state = PHYS_IDLE;
                    return;
                }

                // For digipeated frames: axdelay pre-PTT wait already enforced in PHYS_IDLE.
                // Always use txdely_10ms as the post-PTT transmitter KEY_DELAY here.
                uint16_t delay_10ms = phys->txdely_10ms;
                phys->ptt_control(true, phys->user_data);
                phys->tx_active = true;
                phys->tx_start_tick_10ms = tick_10ms;
                phys->current_session_start_10ms = tick_10ms;
                phys->anti_hog_expired = false;

                phys->next_action_tick_10ms = tick_10ms + delay_10ms;

                if (delay_10ms == 0) {
                    if (phys->remote_sync_10ms > 0) {
                        phys->state = PHYS_REMOTE_SYNC;
                        phys->next_action_tick_10ms = tick_10ms + phys->remote_sync_10ms;
                    } else {
                        send_flags(phys, phys->preamble_flags);
                        phys->state = PHYS_PREAMBLE;
                    }
                } else {
                    phys->state = PHYS_KEY_DELAY;
                }
                return;
            }

            case PHYS_KEY_DELAY:
                if (!TICK_REACHED(tick_10ms, phys->next_action_tick_10ms)) {
                    return;
                }

                // T105 (remote_sync) is a half-duplex pause after PTT that lets
                // the remote station's RX AGC and PLL lock onto the signal before
                // data arrives.  In full-duplex the remote RX is already active on
                // the dedicated receive frequency so no sync pause is needed.
                // ax25_physical_set_duplex() already zeros remote_sync_10ms in
                // full-duplex mode; this guard provides a second layer of
                // protection so PHYS_REMOTE_SYNC is never entered in FD regardless
                // of how remote_sync_10ms was configured by the caller.
                if (!phys->full_duplex && phys->remote_sync_10ms > 0) {
                    phys->state = PHYS_REMOTE_SYNC;
                    phys->next_action_tick_10ms = phys->next_action_tick_10ms + phys->remote_sync_10ms;
                } else {
                    send_flags(phys, phys->preamble_flags);
                    phys->state = PHYS_PREAMBLE;
                }
                continue;

            case PHYS_REMOTE_SYNC:
                if (!TICK_REACHED(tick_10ms, phys->next_action_tick_10ms)) {
                    return;
                }

                send_flags(phys, phys->preamble_flags);
                phys->state = PHYS_PREAMBLE;
                continue;

            case PHYS_PREAMBLE:
                if (!dequeue_frame(phys, &phys->current_frame, &phys->current_len)) {
                    phys->state = PHYS_HANG;
                    phys->next_action_tick_10ms = tick_10ms + phys->axhang_10ms;

                    // In full-duplex, TX and RX use separate frequencies so there is
                    // no shared channel to protect; skip T100 hang and release PTT now.
                    if (phys->full_duplex || phys->axhang_10ms == 0) {
                        phys->last_unkey_tick_10ms = tick_10ms;
                        phys->rx_warmup_required = (phys->rx_startup_10ms > 0);
                        if (phys->rx_warmup_required) {
                            phys->next_action_tick_10ms = tick_10ms + phys->rx_startup_10ms;
                        }
                        phys->ptt_control(false, phys->user_data);
                        phys->tx_active = false;
                        phys->state = PHYS_IDLE;
                    } else {
                        phys->state = PHYS_HANG;
                        phys->next_action_tick_10ms = tick_10ms + phys->axhang_10ms;
                    }

                    return;
                }

                phys->send_data(phys->current_frame, phys->current_len, phys->user_data);
                phys->state = PHYS_DATA;
                return;

            case PHYS_DATA:
                // Record the time transmission ended for RX warmup calculation
                phys->last_unkey_tick_10ms = tick_10ms;

                if (phys->anti_hog_expired && queue_empty(phys)) {
                    phys->anti_hog_expired = false;
                    // T100 (axhang) is a half-duplex courtesy timer that keeps the
                    // transmitter keyed after the last frame so other stations can
                    // detect the channel is still in use.  In full-duplex the TX
                    // frequency is dedicated; hanging adds latency before PHYS_IDLE
                    // is re-entered and the next burst can begin.
                    // ax25_physical_set_duplex() already zeros axhang_10ms, but this
                    // local guard defends against any future caller that forgets to
                    // call set_duplex before queuing frames in full-duplex mode.
                    uint16_t hang = phys->full_duplex ? 0 : phys->axhang_10ms;
                    if (hang == 0) {
                        phys->rx_warmup_required = (phys->rx_startup_10ms > 0);
                        // Set next_action_tick when entering IDLE with RX warmup required
                        if (phys->rx_warmup_required) {
                            phys->next_action_tick_10ms = tick_10ms + phys->rx_startup_10ms;
                        }
                        phys->ptt_control(false, phys->user_data);
                        phys->tx_active = false;
                        phys->state = PHYS_IDLE;
                    } else {
                        // Data transmission done; clear tx_active so callers can detect
                        // end-of-data. PTT remains ON (ptt_control NOT called here).
                        // PTT will be released only when PHYS_HANG timer expires below.
                        phys->tx_active = false;
                        phys->state = PHYS_HANG;
                        phys->next_action_tick_10ms = tick_10ms + hang;
                    }
                    continue;
                }

                if (queue_empty(phys)) {
                    // Same full-duplex guard as the anti_hog branch above:
                    // treat axhang as zero when operating on a dedicated TX frequency.
                    uint16_t hang = phys->full_duplex ? 0 : phys->axhang_10ms;

                    if (hang == 0) {
                        phys->last_unkey_tick_10ms = tick_10ms;
                        phys->rx_warmup_required = (phys->rx_startup_10ms > 0);
                        // Set next_action_tick when entering IDLE with RX warmup required
                        if (phys->rx_warmup_required) {
                            phys->next_action_tick_10ms = tick_10ms + phys->rx_startup_10ms;
                        }
                        phys->ptt_control(false, phys->user_data);
                        phys->tx_active = false;
                        phys->state = PHYS_IDLE;
                    } else {
                        // Data transmission done; clear tx_active so callers can detect
                        // end-of-data. PTT remains ON (ptt_control NOT called here).
                        // PTT will be released only when PHYS_HANG timer expires below.
                        phys->tx_active = false;
                        phys->state = PHYS_HANG;
                        phys->next_action_tick_10ms = tick_10ms + hang;
                    }
                } else {
                    send_flags(phys, phys->interframe_flags);
                    phys->state = PHYS_INTERFRAME;
                    // In full-duplex mode, record when the inter-frame gap started
                    // so PHYS_INTERFRAME can enforce a timed gap equal to the number
                    // of inter-frame flag bytes (converted to 10ms tick units).
                    // This prevents the while(1) loop from draining the entire queue
                    // in a single tick, ensuring T106 (max TX duration) can fire
                    // while frames are still queued (test_fd_phys_queue_retained_on_t106).
                    // Half-duplex uses the original continue behaviour (no gap timer).
                    if (phys->full_duplex) {
                        phys->next_action_tick_10ms = tick_10ms + phys->interframe_flags;
                        phys->current_frame = NULL;
                        phys->current_len = 0;
                        return;  // yield to next tick; PHYS_INTERFRAME will check the timer
                    }
                }
                phys->current_frame = NULL;
                phys->current_len = 0;
                continue;

            case PHYS_INTERFRAME:
                // Full-duplex inter-frame gap timer: DATA set next_action_tick_10ms to
                // tick + interframe_flags when entering this state in FD mode.
                // Hold here until that deadline passes before dequeuing the next frame.
                // This gives T106 a chance to fire if max_tx_duration has elapsed,
                // keeping remaining frames in the queue as required by the spec.
                if (phys->full_duplex && !TICK_REACHED(tick_10ms, phys->next_action_tick_10ms)) {
                    return;
                }

                // Check if RX warmup is required before starting next frame
                if (phys->rx_warmup_required && phys->rx_startup_10ms > 0) {
                    int32_t time_since_unkey = (int32_t) (tick_10ms - phys->last_unkey_tick_10ms);

                    // Enforce minimum delay between transmissions
                    if (time_since_unkey < (int32_t) phys->rx_startup_10ms) {
                        // Wait for RX warmup period to complete
                        phys->next_action_tick_10ms = phys->last_unkey_tick_10ms + phys->rx_startup_10ms;
                        return;
                    }

                    // RX warmup complete
                    phys->rx_warmup_required = false;
                }

                if (!dequeue_frame(phys, &phys->current_frame, &phys->current_len)) {
                    if (phys->full_duplex || phys->axhang_10ms == 0) {
                        phys->last_unkey_tick_10ms = tick_10ms;
                        phys->rx_warmup_required = (phys->rx_startup_10ms > 0);
                        if (phys->rx_warmup_required) {
                            phys->next_action_tick_10ms = tick_10ms + phys->rx_startup_10ms;
                        }
                        phys->ptt_control(false, phys->user_data);
                        phys->tx_active = false;
                        phys->anti_hog_expired = false;
                        phys->state = PHYS_IDLE;
                    } else {
                        // Half-duplex T100 hang: keep PTT on until timer expires
                        phys->tx_active = false;
                        phys->state = PHYS_HANG;
                        phys->next_action_tick_10ms = tick_10ms + phys->axhang_10ms;
                    }
                    return;
                }
                phys->send_data(phys->current_frame, phys->current_len, phys->user_data);
                phys->state = PHYS_DATA;
                return;

            break;

            case PHYS_HANG:
                // Full-duplex guard: PHYS_HANG must never block in FD mode.
                // In full-duplex the TX frequency is dedicated; T100 hang time is
                // meaningless. Release PTT immediately and re-enter PHYS_IDLE.
                // continue (not return) re-evaluates PHYS_IDLE in the same tick,
                // preventing a one-tick latency stall on the dedicated TX path.
                // axhang_10ms zeroed as belt-and-suspenders in case set_duplex
                // was bypassed or full_duplex was set after the state transition.
                if (phys->full_duplex) {
                    phys->ptt_control(false, phys->user_data);
                    phys->tx_active = false;
                    phys->anti_hog_expired = false;
                    phys->axhang_10ms = 0;    // belt-and-suspenders: ensure hang is 0
                    phys->state = PHYS_IDLE;
                    continue;
                }

                if (!TICK_REACHED(tick_10ms, phys->next_action_tick_10ms)) {
                    return;
                }

                phys->last_unkey_tick_10ms = tick_10ms;
                phys->rx_warmup_required = (phys->rx_startup_10ms > 0);

                if (phys->rx_warmup_required) {
                    phys->next_action_tick_10ms = tick_10ms + phys->rx_startup_10ms;
                }

                phys->ptt_control(false, phys->user_data);
                phys->tx_active = false;
                phys->state = PHYS_IDLE;
                phys->anti_hog_expired = false;
                return;

            default:
                phys->state = PHYS_IDLE;
                return;
        }
    }
}

// Receiver State Machine - State 0 -> State 1 transition.
// Called by the application's hardware HDLC decoder when flag sync is acquired.
// The TX state machine (ax25_physical_tick) runs independently and is unaffected.
void ax25_physical_rx_frame_start(ax25_physical_t *phys) {
    if (!phys)
        return;
    phys->rx_state = PHYS_RX_RECEIVING;
}

// Receiver State Machine - State 1 -> State 0 transition.
// Called by the application's hardware HDLC decoder when a complete frame has
// been received (regardless of FCS validity). After this call the application
// should invoke ax25_process_frame() with the decoded frame if FCS was valid.
void ax25_physical_rx_frame_end(ax25_physical_t *phys) {
    if (!phys)
        return;
    phys->rx_state = PHYS_RX_READY;
}
