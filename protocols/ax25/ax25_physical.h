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

#ifndef AX25_PHYSICAL_H_
#define AX25_PHYSICAL_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define AX25_PHYS_MAX_FRAME_SIZE      512  // Safe for FX.25 + headers
#define AX25_PHYS_QUEUE_SIZE          8    // Supports full window + some margin
#define AX25_PHYS_MAX_FLAGS           64   // Buffer for preamble/interframe

typedef enum {
    PHYS_IDLE,         //
    PHYS_CSMA_WAIT,    //
    PHYS_KEY_DELAY,    //
    PHYS_REMOTE_SYNC,  //
    PHYS_PREAMBLE,     //
    PHYS_DATA,         //
    PHYS_INTERFRAME,   //
    PHYS_HANG          //
} ax25_phys_state_t;

typedef struct {
    bool full_duplex;               // Full-duplex mode flag; disables CSMA when true
    uint16_t txdely_10ms;           // T103 - normal TX startup delay (default 30 = 300ms)
    uint16_t axdelay_10ms;          // T104 - delay for digipeated frames (default 60 = 600ms)
    uint16_t axhang_10ms;           // T100 - repeater hang time (default 40 = 400ms)
    uint8_t persist;                // p-persistence (0-255, default 63 ≈ 0.25)
    uint16_t slottime_10ms;         // T102 slot time (default 10 = 100ms)
    uint32_t max_tx_duration_10ms;  // T106 limit (default 60000 = 10 min)
    uint8_t preamble_flags;         // Number of 0x7E flags before first frame (default 40)
    uint8_t interframe_flags;       // Flags between frames in burst (default 5)
    uint16_t dwait_10ms;            // Optional DWAIT - politeness delay after foreign carrier (default 0)
    uint16_t remote_sync_10ms;      // T105 remote sync delay (default 0)
    uint16_t rx_startup_10ms;       // T108 receiver startup time (default 0)
    uint16_t anti_hog_10ms;         // T107 anti-hogging limit (default 0 = disabled)

    /* Callbacks (required) */
    void (*ptt_control)(bool on, void *user_data);
    bool (*carrier_detect)(void *user_data);
    void (*send_data)(const uint8_t *data, size_t len, void *user_data);

    void *user_data;

    /* Internal state - do not touch */
    uint8_t frame_storage[AX25_PHYS_QUEUE_SIZE][AX25_PHYS_MAX_FRAME_SIZE];
    size_t frame_len[AX25_PHYS_QUEUE_SIZE];
    bool frame_is_digipeat[AX25_PHYS_QUEUE_SIZE];

    uint8_t queue_head;
    uint8_t queue_tail;

    uint8_t *current_frame;
    size_t current_len;

    uint32_t rng_state;

    ax25_phys_state_t state;
    uint32_t next_action_tick_10ms;     // Now in 10ms units
    uint32_t tx_start_tick_10ms;        // Now in 10ms units
    bool tx_active;

    uint8_t flag_buf[AX25_PHYS_MAX_FLAGS];

    bool persistence_deferred;
    uint8_t persistence_slots_remaining;
    uint32_t last_foreign_carrier_tick_10ms;  // Now in 10ms units
    bool dwait_pending;
    uint32_t last_unkey_tick_10ms;      // Now in 10ms units
    bool rx_warmup_required;
    bool anti_hog_expired;
    uint32_t current_session_start_10ms;  // Now in 10ms units
    bool axdelay_pending;               // T104 pre-PTT wait active
    uint32_t last_tick_10ms;            // Last tick value processed, used internally by ax25_physical_queue_frame kickstart
} ax25_physical_t;

/* Initialize context with sensible defaults */
void ax25_physical_init(ax25_physical_t *phys);

/* Queue a fully-encoded frame for transmission
 * Returns true on success, false if queue full */
bool ax25_physical_queue_frame(ax25_physical_t *phys, const uint8_t *frame, size_t len, bool is_digipeat);

// API to add entropy from external sources (radio noise, ADC, timer jitter, etc.)
// This prevents the LCG sequence from being predictable and improves randomness
void ax25_physical_add_entropy(ax25_physical_t *phys, uint8_t noise_sample);

/* Tick handler - call periodically (every 10ms recommended) */
void ax25_physical_tick(ax25_physical_t *phys, uint32_t tick_10ms);

// Configure full-duplex mode after XID negotiation
// In full-duplex mode CSMA is bypassed and all half-duplex-only timers are cleared
void ax25_physical_set_duplex(ax25_physical_t *phys, bool fd);

#endif /* AX25_PHYSICAL_H_ */
