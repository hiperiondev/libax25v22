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
    phys->slottime_10ms = 10;       // 100ms / 10 = 10
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

    if (phys->tx_active) {
        // Use signed arithmetic for proper wraparound handling with 10ms units
        int32_t elapsed = (int32_t) (tick_10ms - phys->tx_start_tick_10ms);

        if (elapsed > (int32_t) phys->max_tx_duration_10ms) {
            memset(phys->flag_buf, 0xFF, 20);
            phys->send_data(phys->flag_buf, 20, phys->user_data);

            phys->ptt_control(false, phys->user_data);
            phys->tx_active = false;
            phys->state = PHYS_IDLE;
            phys->queue_head = phys->queue_tail;
            phys->current_frame = NULL;
            phys->current_len = 0;
            phys->anti_hog_expired = false;
            return;
        }

        if (phys->anti_hog_10ms > 0) {
            int32_t session_elapsed = (int32_t) (tick_10ms - phys->current_session_start_10ms);

            if (session_elapsed > (int32_t) phys->anti_hog_10ms) {
                phys->anti_hog_expired = true;
            }
        }
    }

    while (1) {
        if (phys->state != PHYS_DATA && phys->state != PHYS_INTERFRAME) {
            // For PHYS_IDLE, only skip tick check if RX warmup is NOT required
            if (phys->state == PHYS_IDLE && phys->rx_warmup_required) {
                // PHYS_IDLE with RX warmup required - respect next_action_tick_10ms
                if (tick_10ms < phys->next_action_tick_10ms) {
                    return;
                }
            } else if (phys->state != PHYS_IDLE && tick_10ms < phys->next_action_tick_10ms) {
                // Other states - respect next_action_tick_10ms
                return;
            }
        }

        switch (phys->state) {
            case PHYS_IDLE:
                if (queue_empty(phys)) {
                    return;
                }

                // Enforce receiver startup delay (T108) after previous burst ended
                if (phys->rx_warmup_required) {
                    int32_t time_since_unkey = (int32_t) (tick_10ms - phys->last_unkey_tick_10ms);
                    if (time_since_unkey < (int32_t) phys->rx_startup_10ms) {
                        return;
                    }
                    phys->rx_warmup_required = false;
                }

                // Transition to CSMA_WAIT to perform p-persistence and carrier-sense.
                // Doing the full CSMA logic inline in IDLE prevented the state machine
                // from ever entering PHYS_CSMA_WAIT, which the upper layer tests rely on.
                // Reset persistence state so CSMA_WAIT starts a fresh slottime cycle.
                phys->persistence_deferred = false;
                phys->persistence_slots_remaining = 0;
                phys->dwait_pending = false;
                phys->next_action_tick_10ms = tick_10ms + phys->slottime_10ms;
                phys->state = PHYS_CSMA_WAIT;
                return;
            break;

            case PHYS_CSMA_WAIT: {
                bool busy = phys->carrier_detect ? phys->carrier_detect(phys->user_data) : false;

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
                    phys->ptt_control(false, phys->user_data);
                    phys->tx_active = false;
                    phys->state = PHYS_IDLE;
                    return;
                }

                uint16_t delay_10ms = is_digipeat ? phys->axdelay_10ms : phys->txdely_10ms;
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
                if (tick_10ms < phys->next_action_tick_10ms) {
                    return;
                }

                if (phys->remote_sync_10ms > 0) {
                    phys->state = PHYS_REMOTE_SYNC;
                    phys->next_action_tick_10ms = phys->next_action_tick_10ms + phys->remote_sync_10ms;
                } else {
                    send_flags(phys, phys->preamble_flags);
                    phys->state = PHYS_PREAMBLE;
                }
                continue;

            case PHYS_REMOTE_SYNC:
                if (tick_10ms < phys->next_action_tick_10ms) {
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
                    if (phys->axhang_10ms == 0) {
                        phys->rx_warmup_required = (phys->rx_startup_10ms > 0);
                        // Set next_action_tick when entering IDLE with RX warmup required
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
                    continue;
                }

                if (queue_empty(phys)) {
                    if (phys->axhang_10ms == 0) {
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
                        phys->state = PHYS_HANG;
                        phys->next_action_tick_10ms = tick_10ms + phys->axhang_10ms;
                    }
                } else {
                    // More frames queued - enforce RX warmup before next frame if configured
                    if (phys->rx_startup_10ms > 0) {
                        phys->rx_warmup_required = true;
                    }
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
                    phys->state = PHYS_HANG;
                    phys->next_action_tick_10ms = tick_10ms + phys->axhang_10ms;
                    return;
                }
                phys->send_data(phys->current_frame, phys->current_len, phys->user_data);
                phys->state = PHYS_DATA;
                return;

            break;

            case PHYS_HANG:
                if (tick_10ms < phys->next_action_tick_10ms) {
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
