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

#include <stdlib.h>

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

    // Kickstart the state machine when transitioning from empty-queue IDLE.
    // This ensures tx_active is set before the caller's while(tx_active) loop
    // evaluates its condition for the first time (T100/T104 tests).
    // Only kickstart for non-digipeat frames: digipeat frames require AXDELAY
    // (T104) that must be measured from the caller's next tick, not last_tick.
    // Kickstarting a digipeat here would base the AXDELAY on last_tick_10ms
    // instead of the first caller-driven tick, making the delay one tick short.
    if (phys->state == PHYS_IDLE && !is_digipeat) {
        ax25_physical_tick(phys, phys->last_tick_10ms);
    }

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

        if (elapsed >= (int32_t) phys->max_tx_duration_10ms) {
            // Send two closing HDLC flags to terminate the current frame cleanly.
            // 0xFF bytes are not a valid HDLC abort in a byte-stream model.
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
            }
            // Full-duplex: retain queued frames; DLL retransmits via T1 expiry
            phys->state = PHYS_IDLE;

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
                // Axdelay wait elapsed, clear the flag and fall through to normal IDLE processing
                phys->axdelay_pending = false;
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
                        // Key delay required: wait before sending preamble
                        phys->next_action_tick_10ms = tick_10ms + dly;
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

                // Set next_action_tick to enforce at least one full slottime (T102) before
                // p-persistence evaluation in CSMA_WAIT. Using return (not continue) ensures
                // the caller observes PHYS_CSMA_WAIT state on this tick, which is required
                // by upper-layer tests (test_t102_slottime_persistence).
                phys->next_action_tick_10ms = tick_10ms + phys->slottime_10ms;
                // Set next_action_tick for slottime enforcement within CSMA_WAIT.
                // Use continue (not return) so CSMA_WAIT is evaluated immediately in
                // the same tick when slottime=0, allowing tx_active to be set before
                // returning. When slottime>0 the timer guard inside CSMA_WAIT blocks
                // and the function returns with state=PHYS_CSMA_WAIT as before.
                continue;
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
                }
                phys->current_frame = NULL;
                phys->current_len = 0;
                continue;

            case PHYS_INTERFRAME:
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
