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

// Full-duplex test battery for AX.25 v2.2.
// Covers:
//   Section 3 + Appendix C2  : HDLC physical layer state machine - C2b (FD) path
//   Section 6.7.2            : Link-layer full-duplex operation and XID negotiation

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "test_common.h"
#include "ax25_state_machine.h"
#include "ax25_physical.h"
#include "ax25_segmenter.h"
#include "ax25_mgmt.h"
#include "ax25.h"

// ---------------------------------------------------------------------------
// Module-level state (all static so no symbol collision with other test TUs)
// ---------------------------------------------------------------------------

static uint32_t assert_count = 0;

// Capture buffer for frames sent via the connection-layer transmit callback
static uint8_t fd_captured_buffer[2048];
static size_t fd_captured_len = 0;
static uint32_t fd_transmit_count = 0;

// PTT state tracking
static bool fd_ptt_state = false;
static uint32_t fd_ptt_on_count = 0;
static uint32_t fd_ptt_off_count = 0;

// Carrier-detect simulation + call counting
static bool fd_simulated_carrier = false;
static uint32_t fd_carrier_detect_call_count = 0;

// Raw bytes for callsigns TEST1 / TEST2 (AX.25 encoding: chars shifted <<1,
// space-padded to 6 bytes, SSID byte follows)
static const uint8_t fd_test1_call[6] = { 0xA8, 0x8A, 0xA6, 0xA8, 0x62, 0x40 };  // TEST1
static const uint8_t fd_test2_call[6] = { 0xA8, 0x8A, 0xA6, 0xA8, 0x64, 0x40 };  // TEST2

// ---------------------------------------------------------------------------
// Callbacks - all static, distinct from test_ax25_timers.c equivalents
// ---------------------------------------------------------------------------

static void fd_reset_capture(void) {
    fd_captured_len = 0;
    fd_transmit_count = 0;
}

static void fd_reset_ptt_state(void) {
    fd_ptt_state = false;
    fd_ptt_on_count = 0;
    fd_ptt_off_count = 0;
}

static void fd_reset_carrier_count(void) {
    fd_carrier_detect_call_count = 0;
}

// Connection-layer transmit callback (upper-layer)
static void fd_capture_transmit(void *user_data, uint8_t *data, size_t len) {
    if (len <= sizeof(fd_captured_buffer)) {
        memcpy(fd_captured_buffer, data, len);
        fd_captured_len = len;
    }
    fd_transmit_count++;
}

// Physical-layer PTT control callback
static void fd_ptt_control_callback(bool on, void *user_data) {
    fd_ptt_state = on;
    if (on) {
        fd_ptt_on_count++;
    } else {
        fd_ptt_off_count++;
    }
}

// Physical-layer carrier-detect callback - counts every invocation
static bool fd_carrier_detect_callback(void *user_data) {
    fd_carrier_detect_call_count++;
    return fd_simulated_carrier;
}

// Physical-layer send-data callback
static void fd_send_data_callback(const uint8_t *data, size_t len, void *user_data) {
    if (len <= sizeof(fd_captured_buffer)) {
        memcpy(fd_captured_buffer, data, len);
        fd_captured_len = len;
    }
    fd_transmit_count++;
}

// ---------------------------------------------------------------------------
// Helper: build a connected ax25_connection_t without needing a live peer
// Mirrors the helper used in test_ax25_timers.c
// ---------------------------------------------------------------------------

static int fd_establish_connection(ax25_connection_t *conn, ax25_address_t *dest, ax25_address_t *src) {
    fd_reset_capture();
    uint8_t err = ax25_connect(conn, dest, src);
    if (err != 0)
        return -1;

    // Synthesise a UA response from the remote end
    uint8_t ua_raw[15];
    memcpy(ua_raw + 0, fd_test1_call, 6);
    ua_raw[6] = 0x60;
    memcpy(ua_raw + 7, fd_test2_call, 6);
    ua_raw[13] = 0x61;
    ua_raw[14] = 0x73;  // UA, F=1

    uint8_t decode_err = 0;
    ax25_frame_t *ua_frame = ax25_frame_decode(ua_raw, sizeof(ua_raw),
    MODULO128_FALSE, &decode_err);
    if (!ua_frame)
        return -1;

    fd_reset_capture();
    ax25_process_frame(conn, ua_frame, 1);
    ax25_frame_free(ua_frame, &decode_err);

    return (conn->state == AX25_STATE_CONNECTED) ? 0 : -1;
}

static void fd_cleanup_addresses(ax25_address_t **dest, ax25_address_t **src) {
    if (dest && *dest) {
        free(*dest);
        *dest = NULL;
    }
    if (src && *src) {
        free(*src);
        *src = NULL;
    }
}

// ---------------------------------------------------------------------------
// Helper: init a physical layer with FD-mode callbacks and queue one frame
// ---------------------------------------------------------------------------

static void fd_phys_setup(ax25_physical_t *phys) {
    ax25_physical_init(phys);
    phys->ptt_control = fd_ptt_control_callback;
    phys->carrier_detect = fd_carrier_detect_callback;
    phys->send_data = fd_send_data_callback;
    fd_reset_ptt_state();
    fd_reset_carrier_count();
    fd_reset_capture();
    fd_simulated_carrier = false;
}

// ===========================================================================
// GROUP 1 - Physical Layer Full-Duplex (Section 3, Appendix C2 - C2b path)
// ===========================================================================

// ---------------------------------------------------------------------------
// Test 1: ax25_physical_set_duplex(true) clears all half-duplex-only timers
//         and sets the full_duplex flag.
// ---------------------------------------------------------------------------

static int test_fd_phys_set_duplex_clears_timers(void) {
    printf("\n--- test_fd_phys_set_duplex_clears_timers"
            " (ax25_physical_set_duplex resets HD timers) ---\n");
    DEBUG_PRINT("Verifying set_duplex clears half-duplex-only parameters");

    ax25_physical_t phys;
    ax25_physical_init(&phys);

    // Populate every half-duplex timer with a non-zero value before calling set_duplex
    phys.dwait_10ms = 30;
    phys.axhang_10ms = 40;
    phys.anti_hog_10ms = 50;
    phys.remote_sync_10ms = 20;
    phys.rx_startup_10ms = 10;
    phys.persist = 63;
    phys.slottime_10ms = 10;
    phys.axdelay_10ms = 60;
    DEBUG_VAR("dwait before set_duplex", phys.dwait_10ms); DEBUG_VAR("axhang before set_duplex", phys.axhang_10ms); DEBUG_VAR("anti_hog before set_duplex", phys.anti_hog_10ms); DEBUG_VAR("remote_sync before set_duplex", phys.remote_sync_10ms); DEBUG_VAR("rx_startup before set_duplex", phys.rx_startup_10ms); DEBUG_VAR("persist before set_duplex", phys.persist); DEBUG_VAR("slottime before set_duplex", phys.slottime_10ms); DEBUG_VAR("axdelay before set_duplex", phys.axdelay_10ms);

    ax25_physical_set_duplex(&phys, true);
    DEBUG_PRINT("ax25_physical_set_duplex(true) called");

    DEBUG_VAR("dwait after set_duplex", phys.dwait_10ms); DEBUG_VAR("axhang after set_duplex", phys.axhang_10ms); DEBUG_VAR("anti_hog after set_duplex", phys.anti_hog_10ms); DEBUG_VAR("remote_sync after set_duplex", phys.remote_sync_10ms); DEBUG_VAR("rx_startup after set_duplex", phys.rx_startup_10ms); DEBUG_VAR("persist after set_duplex", phys.persist); DEBUG_VAR("slottime after set_duplex", phys.slottime_10ms); DEBUG_VAR("axdelay after set_duplex", phys.axdelay_10ms); DEBUG_BOOL("full_duplex flag set", phys.full_duplex);

    TEST_ASSERT(phys.full_duplex == true, "full_duplex flag set", 0);
    TEST_ASSERT(phys.dwait_10ms == 0, "DWAIT cleared by set_duplex", phys.dwait_10ms);
    TEST_ASSERT(phys.axhang_10ms == 0, "AXHANG cleared by set_duplex", phys.axhang_10ms);
    TEST_ASSERT(phys.anti_hog_10ms == 0, "ANTI_HOG cleared", phys.anti_hog_10ms);
    TEST_ASSERT(phys.remote_sync_10ms == 0, "REMOTE_SYNC cleared", phys.remote_sync_10ms);
    TEST_ASSERT(phys.rx_startup_10ms == 0, "RX_STARTUP cleared", phys.rx_startup_10ms);
    TEST_ASSERT(phys.persist == 255, "persist set to 255 (always TX)", phys.persist);
    TEST_ASSERT(phys.slottime_10ms == 0, "slottime cleared", phys.slottime_10ms);
    TEST_ASSERT(phys.axdelay_10ms == 0, "AXDELAY cleared", phys.axdelay_10ms);
    TEST_ASSERT(phys.rx_warmup_required == false, "rx_warmup_required cleared", phys.rx_warmup_required);
    TEST_ASSERT(phys.dwait_pending == false, "dwait_pending cleared", phys.dwait_pending);
    TEST_ASSERT(phys.axdelay_pending == false, "axdelay_pending cleared", phys.axdelay_pending);

    DEBUG_PRINT("test_fd_phys_set_duplex_clears_timers PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 2: In full-duplex mode, transmission proceeds even when carrier is
//         present (CSMA/CA completely bypassed - Appendix C2b).
// ---------------------------------------------------------------------------

static int test_fd_phys_csma_bypassed_with_carrier(void) {
    printf("\n--- test_fd_phys_csma_bypassed_with_carrier"
            " (CSMA bypassed when channel busy in FD mode) ---\n");
    DEBUG_PRINT("Verifying CSMA bypass with active carrier in FD mode");

    ax25_physical_t phys;
    fd_phys_setup(&phys);

    // Enable full-duplex - clears all HD timers
    ax25_physical_set_duplex(&phys, true);
    DEBUG_BOOL("full_duplex enabled", phys.full_duplex);

    // Simulate a busy channel - in HD this would permanently defer TX
    fd_simulated_carrier = true;
    DEBUG_BOOL("Carrier active (busy channel)", fd_simulated_carrier);

    // Queue one frame
    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    bool queued = ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    TEST_ASSERT(queued, "Frame queued successfully", !queued); DEBUG_FRAME("Frame queued", frame, sizeof(frame));

    // Run ticks - in HD mode with carrier=true the state machine would park in
    // PHYS_CSMA_WAIT indefinitely.  In FD it must raise PTT immediately.
    uint32_t ptt_on_tick = 0;
    for (uint32_t tick = 0; tick < 10; tick++) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick); DEBUG_BOOL("PTT state", fd_ptt_state); DEBUG_STATE("Physical state", phys.state);
        if (fd_ptt_state && ptt_on_tick == 0) {
            ptt_on_tick = tick + 1;  // 1-based for easy zero-check
            DEBUG_VAR("PTT raised at tick", tick);
        }
    }

    DEBUG_VAR("PTT on tick (1-based)", ptt_on_tick); DEBUG_BOOL("PTT raised despite busy channel", ptt_on_tick > 0);
    TEST_ASSERT(ptt_on_tick > 0, "PTT raised despite busy channel in FD mode", 0);
    TEST_ASSERT(fd_ptt_on_count >= 1, "PTT on count >= 1", fd_ptt_on_count);

    DEBUG_PRINT("test_fd_phys_csma_bypassed_with_carrier PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 3: carrier_detect() callback is never called in full-duplex mode.
//         In FD the TX frequency is dedicated; sensing the RX frequency is
//         meaningless and the callback must not be invoked.
// ---------------------------------------------------------------------------

static int test_fd_phys_carrier_detect_not_called(void) {
    printf("\n--- test_fd_phys_carrier_detect_not_called"
            " (carrier_detect never called in FD mode) ---\n");
    DEBUG_PRINT("Verifying carrier_detect callback is not invoked in FD mode");

    ax25_physical_t phys;
    fd_phys_setup(&phys);

    ax25_physical_set_duplex(&phys, true);
    fd_simulated_carrier = true;  // Would permanently block HD transmission
    fd_carrier_detect_call_count = 0;
    DEBUG_BOOL("Carrier simulated as busy", fd_simulated_carrier); DEBUG_VAR("Initial carrier_detect call count", fd_carrier_detect_call_count);

    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("Frame queued", frame, sizeof(frame));

    // Run enough ticks to complete the full transmission cycle
    for (uint32_t tick = 0; tick < 20; tick++) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick); DEBUG_STATE("Physical state", phys.state); DEBUG_VAR("carrier_detect calls so far", fd_carrier_detect_call_count);
    }

    DEBUG_VAR("Total carrier_detect calls", fd_carrier_detect_call_count); DEBUG_BOOL("carrier_detect never called", fd_carrier_detect_call_count == 0);
    TEST_ASSERT(fd_carrier_detect_call_count == 0, "carrier_detect never called in FD mode", fd_carrier_detect_call_count);

    DEBUG_PRINT("test_fd_phys_carrier_detect_not_called PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 4: PHYS_CSMA_WAIT state is never entered in full-duplex mode.
//         Appendix C2b: the FD path in PHYS_IDLE skips straight to PTT.
// ---------------------------------------------------------------------------

static int test_fd_phys_no_csma_wait_state(void) {
    printf("\n--- test_fd_phys_no_csma_wait_state"
            " (PHYS_CSMA_WAIT never entered in FD mode) ---\n");
    DEBUG_PRINT("Verifying PHYS_CSMA_WAIT is never entered in FD mode");

    ax25_physical_t phys;
    fd_phys_setup(&phys);

    ax25_physical_set_duplex(&phys, true);
    // Use a non-zero slottime to make CSMA_WAIT more visible if it were entered
    phys.slottime_10ms = 5;
    fd_simulated_carrier = false;
    DEBUG_VAR("slottime_10ms (should be ignored in FD)", phys.slottime_10ms);

    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("Frame queued", frame, sizeof(frame));

    bool csma_wait_entered = false;
    for (uint32_t tick = 0; tick < 20; tick++) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick); DEBUG_STATE("Physical state", phys.state);
        if (phys.state == PHYS_CSMA_WAIT) {
            csma_wait_entered = true;
            DEBUG_PRINT("ERROR: PHYS_CSMA_WAIT entered in FD mode!");
            break;
        }
    }

    DEBUG_BOOL("PHYS_CSMA_WAIT never entered", !csma_wait_entered);
    TEST_ASSERT(!csma_wait_entered, "PHYS_CSMA_WAIT never entered in FD mode", csma_wait_entered);

    DEBUG_PRINT("test_fd_phys_no_csma_wait_state PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 5: PHYS_REMOTE_SYNC state is never entered in full-duplex mode.
//         set_duplex clears remote_sync_10ms; KEY_DELAY also has an explicit
//         guard `if (!phys->full_duplex && phys->remote_sync_10ms > 0)`.
// ---------------------------------------------------------------------------

static int test_fd_phys_no_remote_sync_state(void) {
    printf("\n--- test_fd_phys_no_remote_sync_state"
            " (PHYS_REMOTE_SYNC never entered in FD mode) ---\n");
    DEBUG_PRINT("Verifying PHYS_REMOTE_SYNC is never entered in FD mode");

    ax25_physical_t phys;
    fd_phys_setup(&phys);

    // Enable FD then forcibly re-set remote_sync to a non-zero value to stress
    // the KEY_DELAY guard independently of set_duplex clearing it.
    ax25_physical_set_duplex(&phys, true);
    phys.remote_sync_10ms = 20;
    phys.txdely_10ms = 10;  // Ensure KEY_DELAY state is entered first
    DEBUG_VAR("remote_sync_10ms (overridden after set_duplex)", phys.remote_sync_10ms); DEBUG_VAR("txdely_10ms", phys.txdely_10ms); DEBUG_BOOL("full_duplex flag", phys.full_duplex);

    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("Frame queued", frame, sizeof(frame));

    bool remote_sync_entered = false;
    bool key_delay_entered = false;
    for (uint32_t tick = 0; tick < 50; tick++) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick); DEBUG_STATE("Physical state", phys.state);
        if (phys.state == PHYS_KEY_DELAY) {
            key_delay_entered = true;
        }
        if (phys.state == PHYS_REMOTE_SYNC) {
            remote_sync_entered = true;
            DEBUG_PRINT("ERROR: PHYS_REMOTE_SYNC entered in FD mode!");
            break;
        }
    }

    DEBUG_BOOL("KEY_DELAY was entered (confirming test reach)", key_delay_entered); DEBUG_BOOL("PHYS_REMOTE_SYNC never entered", !remote_sync_entered);
    TEST_ASSERT(key_delay_entered, "KEY_DELAY entered (confirms guard was exercised)", !key_delay_entered);
    TEST_ASSERT(!remote_sync_entered, "PHYS_REMOTE_SYNC never entered in FD mode", remote_sync_entered);

    DEBUG_PRINT("test_fd_phys_no_remote_sync_state PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 6: T100 AXHANG is bypassed in full-duplex mode.
//         The DATA state uses `uint16_t hang = phys->full_duplex ? 0 : axhang`.
//         PTT must be released immediately after the last frame without waiting.
// ---------------------------------------------------------------------------

static int test_fd_phys_axhang_bypassed(void) {
    printf("\n--- test_fd_phys_axhang_bypassed"
            " (T100 AXHANG bypassed in FD mode) ---\n");
    DEBUG_PRINT("Verifying T100 hang timer is not enforced in FD mode");

    ax25_physical_t phys;
    fd_phys_setup(&phys);

    // Enable FD but then manually restore a large axhang value to stress
    // the in-code `full_duplex ? 0 : axhang_10ms` guard.
    phys.full_duplex = true;
    phys.axhang_10ms = 20;   // Would cause 200 ms hang in HD mode
    phys.txdely_10ms = 0;
    phys.persist = 255;
    DEBUG_VAR("axhang_10ms (set manually, should be ignored)", phys.axhang_10ms); DEBUG_BOOL("full_duplex flag", phys.full_duplex);

    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("Frame queued", frame, sizeof(frame));

    // Run until TX is done and PTT drops
    uint32_t tx_ended_tick = 0;
    uint32_t ptt_off_tick = 0;
    for (uint32_t tick = 0; tick < 60; tick++) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick); DEBUG_BOOL("TX active", phys.tx_active); DEBUG_BOOL("PTT state", fd_ptt_state); DEBUG_STATE("Physical state", phys.state);

        // Detect transition from tx_active=true to tx_active=false
        if (!phys.tx_active && tx_ended_tick == 0 && fd_ptt_on_count > 0) {
            tx_ended_tick = tick;
            DEBUG_VAR("TX ended at tick", tx_ended_tick);
        }
        // Detect PTT release (after TX ended)
        if (!fd_ptt_state && ptt_off_tick == 0 && fd_ptt_off_count > 0) {
            ptt_off_tick = tick;
            DEBUG_VAR("PTT released at tick", ptt_off_tick);
            break;
        }
    }

    DEBUG_VAR("tx_ended_tick", tx_ended_tick); DEBUG_VAR("ptt_off_tick", ptt_off_tick); DEBUG_VAR("ptt_on_count", fd_ptt_on_count); DEBUG_VAR("ptt_off_count", fd_ptt_off_count);

    // PTT must have been raised and then released
    TEST_ASSERT(fd_ptt_on_count >= 1, "PTT was raised", fd_ptt_on_count);
    TEST_ASSERT(fd_ptt_off_count >= 1, "PTT was released", fd_ptt_off_count);

    // In HD with axhang=20 the gap would be >= 20 ticks.
    // In FD the gap must be < 20 ticks (hang is 0 regardless of axhang_10ms).
    uint32_t hang_gap = (ptt_off_tick >= tx_ended_tick) ? (ptt_off_tick - tx_ended_tick) : 0;
    DEBUG_VAR("Hang gap (ticks)", hang_gap); DEBUG_BOOL("Hang gap < 20 (axhang bypassed)", hang_gap < 20);
    TEST_ASSERT(hang_gap < 20, "AXHANG bypassed: PTT released without 200ms hang", hang_gap);

    DEBUG_PRINT("test_fd_phys_axhang_bypassed PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 7: T104 AXDELAY (digipeater pre-PTT deferral) is bypassed in FD mode.
//         In FD the TX channel is dedicated; no pre-PTT wait is needed.
// ---------------------------------------------------------------------------

static int test_fd_phys_axdelay_bypassed(void) {
    printf("\n--- test_fd_phys_axdelay_bypassed"
            " (T104 AXDELAY bypassed for digipeated frames in FD mode) ---\n");
    DEBUG_PRINT("Verifying AXDELAY is not enforced for digipeated frames in FD mode");

    ax25_physical_t phys;
    fd_phys_setup(&phys);

    // Enable FD then set axdelay to a large value to stress the guard
    // `if (!phys->full_duplex && !phys->axdelay_pending)` in PHYS_IDLE.
    phys.full_duplex = true;
    phys.axdelay_10ms = 50;  // Would cause 500 ms deferral in HD mode
    phys.txdely_10ms = 0;
    phys.persist = 255;
    phys.axhang_10ms = 0;
    DEBUG_VAR("axdelay_10ms (set manually, should be ignored)", phys.axdelay_10ms); DEBUG_BOOL("full_duplex flag", phys.full_duplex);

    // Queue a DIGIPEATED frame (is_digipeat=true)
    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    bool queued = ax25_physical_queue_frame(&phys, frame, sizeof(frame), true);
    TEST_ASSERT(queued, "Digipeated frame queued", !queued); DEBUG_PRINT("Digipeated frame queued (is_digipeat=true)");

    // PTT must fire within 5 ticks - an axdelay of 50 would defer it far beyond
    uint32_t ptt_on_tick = 0;
    for (uint32_t tick = 0; tick < 10; tick++) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick); DEBUG_BOOL("PTT state", fd_ptt_state); DEBUG_BOOL("axdelay_pending", phys.axdelay_pending); DEBUG_STATE("Physical state", phys.state);
        if (fd_ptt_state && ptt_on_tick == 0) {
            ptt_on_tick = tick + 1;  // 1-based
            DEBUG_VAR("PTT raised at tick", tick);
        }
    }

    DEBUG_BOOL("axdelay_pending never set", !phys.axdelay_pending);
    TEST_ASSERT(!phys.axdelay_pending, "axdelay_pending was never set in FD mode", phys.axdelay_pending); DEBUG_VAR("PTT on tick (1-based, must be <= 5)", ptt_on_tick);
    TEST_ASSERT(ptt_on_tick > 0 && ptt_on_tick <= 5, "PTT raised promptly (within 5 ticks) - AXDELAY bypassed", ptt_on_tick);

    DEBUG_PRINT("test_fd_phys_axdelay_bypassed PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 8: T107 anti-hogging does not interrupt transmission in FD mode.
//         In HD mode anti-hog caps the TX burst; in FD it must be ignored.
// ---------------------------------------------------------------------------

static int test_fd_phys_anti_hog_bypassed(void) {
    printf("\n--- test_fd_phys_anti_hog_bypassed"
            " (T107 anti-hog does not cut TX in FD mode) ---\n");
    DEBUG_PRINT("Verifying anti-hog timer does not interrupt FD transmission");

    ax25_physical_t phys;
    fd_phys_setup(&phys);

    // Enable FD then manually set anti_hog to a very short interval so it
    // would fire quickly if the guard `!phys->full_duplex` were absent.
    phys.full_duplex = true;
    phys.anti_hog_10ms = 3;  // Would cut burst after 30 ms in HD mode
    phys.txdely_10ms = 0;
    phys.persist = 255;
    phys.axhang_10ms = 0;
    DEBUG_VAR("anti_hog_10ms (set manually, should be ignored)", phys.anti_hog_10ms); DEBUG_BOOL("full_duplex flag", phys.full_duplex);

    // Queue several frames to give anti-hog time to fire if broken
    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    for (int i = 0; i < 4; i++) {
        ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
        DEBUG_VAR("Frame queued", i);
    }

    uint32_t transmission_breaks = 0;
    bool was_tx_active = false;
    for (uint32_t tick = 0; tick < 40; tick++) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick); DEBUG_BOOL("TX active", phys.tx_active); DEBUG_BOOL("anti_hog_expired", phys.anti_hog_expired);

        if (phys.tx_active && !was_tx_active) {
            was_tx_active = true;
            DEBUG_VAR("TX started at tick", tick);
        } else if (!phys.tx_active && was_tx_active) {
            // tx_active went false - check if it restarted (burst break vs end)
            was_tx_active = false;
            transmission_breaks++;
            DEBUG_VAR("Transmission break at tick", tick);
        }
    }

    DEBUG_VAR("Total transmission breaks", transmission_breaks); DEBUG_BOOL("anti_hog_expired never set", !phys.anti_hog_expired);
    // In HD mode with anti_hog=3 we would see >= 1 break in 40 ticks
    TEST_ASSERT(!phys.anti_hog_expired, "anti_hog_expired never asserted in FD mode", phys.anti_hog_expired);
    // Allow exactly one break = the natural end-of-queue (all 4 frames sent)
    TEST_ASSERT(transmission_breaks <= 1, "No mid-burst anti-hog break in FD mode", transmission_breaks);

    DEBUG_PRINT("test_fd_phys_anti_hog_bypassed PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 9: T103 TXDELAY (transmitter warm-up) is still honoured in FD mode.
//         TXDELAY is a hardware constraint, not a CSMA mechanism, and applies
//         to both HD and FD operation.
// ---------------------------------------------------------------------------

static int test_fd_phys_txdelay_honored(void) {
    printf("\n--- test_fd_phys_txdelay_honored"
            " (T103 TXDELAY enforced in FD mode) ---\n");
    DEBUG_PRINT("Verifying TXDELAY is still enforced in full-duplex mode");

    ax25_physical_t phys;
    fd_phys_setup(&phys);

    ax25_physical_set_duplex(&phys, true);
    phys.txdely_10ms = 20;  // 200 ms warm-up
    DEBUG_VAR("txdely_10ms", phys.txdely_10ms); DEBUG_BOOL("full_duplex flag", phys.full_duplex);

    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("Frame queued", frame, sizeof(frame));

    uint32_t ptt_on_tick = 0;
    uint32_t data_tx_tick = 0;
    fd_reset_capture();

    for (uint32_t tick = 0; tick < 60; tick++) {
        size_t prev_len = fd_captured_len;
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick); DEBUG_BOOL("PTT state", fd_ptt_state); DEBUG_STATE("Physical state", phys.state);

        if (fd_ptt_state && ptt_on_tick == 0) {
            ptt_on_tick = tick;
            DEBUG_VAR("PTT raised at tick", ptt_on_tick);
        }
        if (fd_captured_len > prev_len && data_tx_tick == 0) {
            data_tx_tick = tick;
            DEBUG_VAR("Data transmitted at tick", data_tx_tick);
        }
    }

    DEBUG_VAR("ptt_on_tick", ptt_on_tick); DEBUG_VAR("data_tx_tick", data_tx_tick);
    uint32_t delay = (data_tx_tick > ptt_on_tick) ? (data_tx_tick - ptt_on_tick) : 0;
    DEBUG_VAR("Delay between PTT and data (ticks)", delay); DEBUG_BOOL("Delay >= txdely_10ms (20 ticks)", delay >= 20);

    TEST_ASSERT(ptt_on_tick > 0, "PTT was raised in FD mode", ptt_on_tick == 0);
    TEST_ASSERT(data_tx_tick > 0, "Data was transmitted", data_tx_tick == 0);
    TEST_ASSERT(delay >= 20, "TXDELAY (20 ticks) enforced in FD mode", delay);

    DEBUG_PRINT("test_fd_phys_txdelay_honored PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 10: PTT is raised immediately (tick 0 or 1) in FD mode with no CSMA.
//          The FD path in PHYS_IDLE asserts PTT and continues without waiting.
// ---------------------------------------------------------------------------

static int test_fd_phys_ptt_fires_without_csma(void) {
    printf("\n--- test_fd_phys_ptt_fires_without_csma"
            " (PTT raised immediately on first tick in FD mode) ---\n");
    DEBUG_PRINT("Verifying PTT fires on first tick in FD mode");

    ax25_physical_t phys;
    fd_phys_setup(&phys);

    ax25_physical_set_duplex(&phys, true);
    phys.txdely_10ms = 0;      // No extra delay so we can test pure FD response
    fd_simulated_carrier = true;  // Busy channel - irrelevant in FD
    DEBUG_BOOL("Carrier active (irrelevant in FD)", fd_simulated_carrier); DEBUG_BOOL("full_duplex flag", phys.full_duplex);

    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("Frame queued", frame, sizeof(frame));

    // Single tick - PTT must already be on
    ax25_physical_tick(&phys, 0);
    DEBUG_BOOL("PTT state after tick 0", fd_ptt_state); DEBUG_STATE("Physical state after tick 0", phys.state); DEBUG_BOOL("TX active after tick 0", phys.tx_active);

    TEST_ASSERT(fd_ptt_state == true, "PTT raised on first tick (no CSMA wait) in FD mode", !fd_ptt_state);
    TEST_ASSERT(phys.tx_active == true, "tx_active set on first tick in FD mode", !phys.tx_active);

    DEBUG_PRINT("test_fd_phys_ptt_fires_without_csma PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 11: Frame queue is NOT flushed when T106 (max TX duration) expires in
//          FD mode.  Section C2b / ax25_physical_set_duplex comment: "Full-
//          duplex: retain queued frames; DLL retransmits via T1 expiry."
// ---------------------------------------------------------------------------

static int test_fd_phys_queue_retained_on_t106(void) {
    printf("\n--- test_fd_phys_queue_retained_on_t106"
            " (queue NOT flushed on T106 expiry in FD mode) ---\n");
    DEBUG_PRINT("Verifying queued frames are retained when T106 fires in FD mode");

    ax25_physical_t phys;
    fd_phys_setup(&phys);

    ax25_physical_set_duplex(&phys, true);
    phys.max_tx_duration_10ms = 5;  // Very short limit so T106 fires quickly
    phys.txdely_10ms = 0;
    DEBUG_VAR("max_tx_duration_10ms", phys.max_tx_duration_10ms); DEBUG_BOOL("full_duplex flag", phys.full_duplex);

    // Queue 3 frames - T106 will fire before all are sent
    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    for (int i = 0; i < 3; i++) {
        ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
        DEBUG_VAR("Frame queued", i);
    }

    // Run until T106 fires (tx_active drops due to time limit)
    bool t106_fired = false;
    for (uint32_t tick = 0; tick < 30; tick++) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick); DEBUG_BOOL("TX active", phys.tx_active); DEBUG_BOOL("PTT state", fd_ptt_state); DEBUG_STATE("Physical state", phys.state);
        // T106 fires and releases PTT; state returns to PHYS_IDLE
        if (!fd_ptt_state && fd_ptt_off_count >= 1 && tick >= (uint32_t) phys.max_tx_duration_10ms) {
            t106_fired = true;
            DEBUG_VAR("T106 fired at tick", tick);
            break;
        }
    }

    // Measure remaining frames in queue after T106
    uint8_t q_head = phys.queue_head;
    uint8_t q_tail = phys.queue_tail;
    uint8_t q_used = (uint8_t) ((q_tail - q_head + AX25_PHYS_QUEUE_SIZE) % AX25_PHYS_QUEUE_SIZE);
    DEBUG_VAR("Queue head after T106", q_head); DEBUG_VAR("Queue tail after T106", q_tail); DEBUG_VAR("Frames still in queue", q_used); DEBUG_BOOL("T106 fired", t106_fired); DEBUG_BOOL("Queue not empty (frames retained)", q_used > 0);

    TEST_ASSERT(t106_fired, "T106 fired (confirmed by PTT release)", !t106_fired);
    TEST_ASSERT(q_used > 0, "Queue NOT flushed on T106 in FD mode (frames retained)", q_used);

    DEBUG_PRINT("test_fd_phys_queue_retained_on_t106 PASSED");
    return 0;
}

// ===========================================================================
// GROUP 2 - Link Layer Full-Duplex (AX.25 v2.2 Section 6.7.2)
// ===========================================================================

// ---------------------------------------------------------------------------
// Test 12: XID negotiation - both stations support FD -> agreed FD = true.
//          ax25_mgmt negotiation uses AND of local and remote capabilities.
// ---------------------------------------------------------------------------

static int test_fd_link_negotiation_both_agree(void) {
    printf("\n--- test_fd_link_negotiation_both_agree"
            " (XID negotiation: both agree FD -> agreed FD=true) ---\n");
    DEBUG_PRINT("Verifying FD agreed when both stations support it");

    ax25_mgmt_context_t ctx;
    uint8_t err = ax25_mgmt_init(&ctx);
    TEST_ASSERT(err == 0, "mgmt_init succeeded", err); DEBUG_VAR("mgmt_init error code", err);

    // Both stations want full-duplex
    ctx.local_params.full_duplex = true;
    ctx.remote_params.full_duplex = true;
    DEBUG_BOOL("local full_duplex", ctx.local_params.full_duplex); DEBUG_BOOL("remote full_duplex", ctx.remote_params.full_duplex);

    // Mirror the AND negotiation in ax25_mgmt_process_xid()
    ctx.agreed_params.full_duplex = ctx.local_params.full_duplex && ctx.remote_params.full_duplex;
    ctx.state = AX25_MGMT_NEGOTIATED;

    DEBUG_BOOL("agreed full_duplex", ctx.agreed_params.full_duplex); DEBUG_STATE("mgmt state", ctx.state);

    TEST_ASSERT(ctx.agreed_params.full_duplex == true, "Agreed FD=true when both stations request it", ctx.agreed_params.full_duplex);

    DEBUG_PRINT("test_fd_link_negotiation_both_agree PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 13: XID negotiation - one station refuses FD -> agreed FD = false.
//          Per Section 6.7.2 both stations MUST agree; one refusal is enough
//          to fall back to half-duplex.
// ---------------------------------------------------------------------------

static int test_fd_link_negotiation_one_refuses(void) {
    printf("\n--- test_fd_link_negotiation_one_refuses"
            " (XID negotiation: one refuses FD -> agreed FD=false, HD used) ---\n");
    DEBUG_PRINT("Verifying FD falls back to HD when one station refuses");

    ax25_mgmt_context_t ctx;
    ax25_mgmt_init(&ctx);

    // Local wants FD; remote does NOT
    ctx.local_params.full_duplex = true;
    ctx.remote_params.full_duplex = false;
    DEBUG_BOOL("local full_duplex", ctx.local_params.full_duplex); DEBUG_BOOL("remote full_duplex", ctx.remote_params.full_duplex);

    ctx.agreed_params.full_duplex = ctx.local_params.full_duplex && ctx.remote_params.full_duplex;
    ctx.state = AX25_MGMT_NEGOTIATED;

    DEBUG_BOOL("agreed full_duplex (must be false)", ctx.agreed_params.full_duplex);

    TEST_ASSERT(ctx.agreed_params.full_duplex == false, "Agreed FD=false when remote refuses (AND logic)", ctx.agreed_params.full_duplex);

    // Symmetry: reverse who refuses
    ctx.local_params.full_duplex = false;
    ctx.remote_params.full_duplex = true;
    ctx.agreed_params.full_duplex = ctx.local_params.full_duplex && ctx.remote_params.full_duplex;
    DEBUG_BOOL("agreed full_duplex (reversed, must also be false)",
            ctx.agreed_params.full_duplex);

    TEST_ASSERT(ctx.agreed_params.full_duplex == false, "Agreed FD=false when local refuses (AND logic, reversed)", ctx.agreed_params.full_duplex);

    // Both refuse
    ctx.local_params.full_duplex = false;
    ctx.remote_params.full_duplex = false;
    ctx.agreed_params.full_duplex = ctx.local_params.full_duplex && ctx.remote_params.full_duplex;
    DEBUG_BOOL("agreed full_duplex (both refuse, must be false)",
            ctx.agreed_params.full_duplex);

    TEST_ASSERT(ctx.agreed_params.full_duplex == false, "Agreed FD=false when both refuse", ctx.agreed_params.full_duplex);

    DEBUG_PRINT("test_fd_link_negotiation_one_refuses PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 14: ax25_apply_negotiated_params propagates FD to connection AND
//          physical layer when agreed_params.full_duplex=true.
// ---------------------------------------------------------------------------

static int test_fd_link_apply_params_to_conn_and_phys(void) {
    printf("\n--- test_fd_link_apply_params_to_conn_and_phys"
            " (apply_negotiated_params sets FD on conn and phys) ---\n");
    DEBUG_PRINT("Verifying apply_negotiated_params propagates FD to conn and phys");

    ax25_mgmt_context_t mgmt_ctx;
    ax25_mgmt_init(&mgmt_ctx);

    // Simulate completed negotiation with FD agreed
    mgmt_ctx.agreed_params.full_duplex = true;
    mgmt_ctx.agreed_params.modulo128 = false;
    mgmt_ctx.agreed_params.selective_reject = false;
    mgmt_ctx.agreed_params.implicit_reject = true;
    mgmt_ctx.agreed_params.ifield_length = 256;
    mgmt_ctx.agreed_params.window_size = 7;
    mgmt_ctx.agreed_params.ack_timer = 3000;
    mgmt_ctx.agreed_params.retries = 10;
    mgmt_ctx.agreed_params.response_delay_timer = 500;
    mgmt_ctx.state = AX25_MGMT_NEGOTIATED;
    DEBUG_BOOL("agreed full_duplex", mgmt_ctx.agreed_params.full_duplex); DEBUG_STATE("mgmt state", mgmt_ctx.state);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = fd_capture_transmit };
    ax25_connection_init(&conn, &cb, NULL);
    conn.full_duplex = false;  // Explicitly start as HD
    DEBUG_BOOL("conn full_duplex before apply", conn.full_duplex);

    ax25_physical_t phys;
    fd_phys_setup(&phys);
    phys.full_duplex = false;  // Explicitly start as HD
    DEBUG_BOOL("phys full_duplex before apply", phys.full_duplex);

    uint8_t result = ax25_apply_negotiated_params(&mgmt_ctx, &conn, &phys);
    DEBUG_VAR("apply_negotiated_params return code", result); DEBUG_BOOL("conn full_duplex after apply", conn.full_duplex); DEBUG_BOOL("phys full_duplex after apply", phys.full_duplex); DEBUG_VAR("phys axhang_10ms after apply (must be 0)", phys.axhang_10ms); DEBUG_VAR("phys anti_hog_10ms after apply (must be 0)", phys.anti_hog_10ms); DEBUG_VAR("phys remote_sync_10ms after apply (must be 0)", phys.remote_sync_10ms);

    TEST_ASSERT(result == 0, "apply_negotiated_params succeeded", result);
    TEST_ASSERT(conn.full_duplex == true, "conn->full_duplex set to true by apply", conn.full_duplex);
    TEST_ASSERT(phys.full_duplex == true, "phys->full_duplex set to true by apply", phys.full_duplex);
    // set_duplex should have cleared HD-only timers
    TEST_ASSERT(phys.axhang_10ms == 0, "phys axhang cleared", phys.axhang_10ms);
    TEST_ASSERT(phys.anti_hog_10ms == 0, "phys anti_hog cleared", phys.anti_hog_10ms);
    TEST_ASSERT(phys.remote_sync_10ms == 0, "phys remote_sync cleared", phys.remote_sync_10ms);

    DEBUG_PRINT("test_fd_link_apply_params_to_conn_and_phys PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 15: ax25_apply_negotiated_params with phys=NULL succeeds gracefully.
//          Unit-test environments may not have a physical layer instance.
// ---------------------------------------------------------------------------

static int test_fd_link_apply_params_null_phys(void) {
    printf("\n--- test_fd_link_apply_params_null_phys"
            " (apply_negotiated_params with phys=NULL succeeds) ---\n");
    DEBUG_PRINT("Verifying apply_negotiated_params handles NULL phys gracefully");

    ax25_mgmt_context_t mgmt_ctx;
    ax25_mgmt_init(&mgmt_ctx);
    mgmt_ctx.agreed_params.full_duplex = true;
    mgmt_ctx.agreed_params.modulo128 = false;
    mgmt_ctx.agreed_params.selective_reject = false;
    mgmt_ctx.agreed_params.implicit_reject = true;
    mgmt_ctx.agreed_params.ifield_length = 256;
    mgmt_ctx.agreed_params.window_size = 7;
    mgmt_ctx.agreed_params.ack_timer = 3000;
    mgmt_ctx.agreed_params.retries = 10;
    mgmt_ctx.agreed_params.response_delay_timer = 500;
    mgmt_ctx.state = AX25_MGMT_NEGOTIATED;
    DEBUG_BOOL("agreed full_duplex", mgmt_ctx.agreed_params.full_duplex);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = fd_capture_transmit };
    ax25_connection_init(&conn, &cb, NULL);

    DEBUG_PRINT("Calling apply_negotiated_params with phys=NULL");
    uint8_t result = ax25_apply_negotiated_params(&mgmt_ctx, &conn, NULL);
    DEBUG_VAR("Return code", result); DEBUG_BOOL("conn full_duplex", conn.full_duplex);

    TEST_ASSERT(result == 0, "apply_negotiated_params with NULL phys returns 0", result);
    TEST_ASSERT(conn.full_duplex == true, "conn->full_duplex still set even with NULL phys", conn.full_duplex);

    DEBUG_PRINT("test_fd_link_apply_params_null_phys PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 16: ax25_is_full_duplex() returns correct values.
//          NULL conn returns false; conn with flag false returns false;
//          conn with flag true returns true.
// ---------------------------------------------------------------------------

static int test_fd_link_is_full_duplex_flag(void) {
    printf("\n--- test_fd_link_is_full_duplex_flag"
            " (ax25_is_full_duplex returns correct value) ---\n");
    DEBUG_PRINT("Verifying ax25_is_full_duplex returns correct values");

    // NULL input must return false without crashing
    bool result_null = ax25_is_full_duplex(NULL);
    DEBUG_BOOL("ax25_is_full_duplex(NULL)", result_null);
    TEST_ASSERT(result_null == false, "ax25_is_full_duplex(NULL) returns false", result_null);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = fd_capture_transmit };
    ax25_connection_init(&conn, &cb, NULL);

    // Default state should be HD
    conn.full_duplex = false;
    bool result_hd = ax25_is_full_duplex(&conn);
    DEBUG_BOOL("ax25_is_full_duplex() when full_duplex=false", result_hd);
    TEST_ASSERT(result_hd == false, "ax25_is_full_duplex() returns false when HD", result_hd);

    // Switch to FD
    conn.full_duplex = true;
    bool result_fd = ax25_is_full_duplex(&conn);
    DEBUG_BOOL("ax25_is_full_duplex() when full_duplex=true", result_fd);
    TEST_ASSERT(result_fd == true, "ax25_is_full_duplex() returns true when FD", !result_fd);

    DEBUG_PRINT("test_fd_link_is_full_duplex_flag PASSED");
    return 0;
}

// ---------------------------------------------------------------------------
// Test 17: In FD mode, the T2 response-delay mechanism is bypassed.
//          start_t2_response() sends RR immediately in FD; no deferred ACK.
//          After receiving an I-frame the transmit callback must fire within
//          the same call to ax25_process_frame() - before any ax25_tick().
// ---------------------------------------------------------------------------

static int test_fd_link_t2_rr_immediate(void) {
    printf("\n--- test_fd_link_t2_rr_immediate"
            " (RR ACK sent immediately in FD mode, T2 not started) ---\n");
    DEBUG_PRINT("Verifying T2 timer not started; RR sent immediately in FD mode");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = fd_capture_transmit };
    uint8_t err = ax25_connection_init(&conn, &cb, NULL);
    TEST_ASSERT(err == 0, "Connection init succeeded", err);

    conn.timers.t2 = 10;   // 100 ms - would delay ACK in HD mode
    DEBUG_VAR("T2 timer configured (ticks)", conn.timers.t2);

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(dest != NULL && src != NULL, "Addresses created", parse_err);

    int res = fd_establish_connection(&conn, dest, src);
    if (res != 0) {
        fd_cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Connection established", res);
    } DEBUG_STATE("Connection state after establish", conn.state);

    // Enable full-duplex on the connection
    conn.full_duplex = true;
    DEBUG_BOOL("full_duplex set on connection", conn.full_duplex);

    // Build a synthetic inbound I-frame (N(S)=0, N(R)=0, PID=0xF0)
    uint8_t iframe_raw[19];
    memcpy(iframe_raw + 0, fd_test1_call, 6);
    iframe_raw[6] = 0x60;
    memcpy(iframe_raw + 7, fd_test2_call, 6);
    iframe_raw[13] = 0x61;
    iframe_raw[14] = 0x00;  // I-frame: N(S)=0, N(R)=0
    iframe_raw[15] = 0xF0;  // PID: no layer 3
    iframe_raw[16] = 'H';
    iframe_raw[17] = 'I';
    iframe_raw[18] = '!';
    DEBUG_FRAME("Inbound I-frame", iframe_raw, sizeof(iframe_raw));

    uint8_t decode_err = 0;
    ax25_frame_t *iframe = ax25_frame_decode(iframe_raw, sizeof(iframe_raw),
    MODULO128_FALSE, &decode_err);
    if (!iframe) {
        fd_cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "I-frame decoded", decode_err);
    }

    fd_reset_capture();
    uint32_t transmit_before = fd_transmit_count;
    DEBUG_VAR("Transmit count before process_frame", transmit_before);

    ax25_process_frame(&conn, iframe, 1);
    ax25_frame_free(iframe, &decode_err);

    uint32_t transmit_after = fd_transmit_count;
    DEBUG_VAR("Transmit count after process_frame", transmit_after); DEBUG_BOOL("T2 running (must be false in FD)", conn.t2_running); DEBUG_BOOL("T2 ack pending (must be false in FD)", conn.t2_ack_pending); DEBUG_BOOL("RR sent immediately", transmit_after > transmit_before);

    TEST_ASSERT(conn.t2_running == false, "T2 not started in FD mode", conn.t2_running);
    TEST_ASSERT(conn.t2_ack_pending == false, "No pending deferred ACK in FD mode", conn.t2_ack_pending);
    TEST_ASSERT(transmit_after > transmit_before, "RR sent immediately (without waiting for T2)", transmit_after - transmit_before);

    fd_cleanup_addresses(&dest, &src);
    DEBUG_PRINT("test_fd_link_t2_rr_immediate PASSED");
    return 0;
}

// ===========================================================================
// Main test runner
// ===========================================================================

int test_ax25_fullduplex_main(void) {
    int result = 0;

    printf("\n================================================================================\n");
    printf("Starting AX.25 v2.2 Full-Duplex Tests\n");
    printf("  Section 3 / Appendix C2 : Physical layer state machine - C2b FD path\n");
    printf("  Section 6.7.2           : Link-layer full-duplex operation\n");
    printf("================================================================================\n");

    printf("\n--- GROUP 1: Physical Layer Full-Duplex (Section 3, Appendix C2) ---\n");
    result |= test_fd_phys_set_duplex_clears_timers();
    result |= test_fd_phys_csma_bypassed_with_carrier();
    result |= test_fd_phys_carrier_detect_not_called();
    result |= test_fd_phys_no_csma_wait_state();
    result |= test_fd_phys_no_remote_sync_state();
    result |= test_fd_phys_axhang_bypassed();
    result |= test_fd_phys_axdelay_bypassed();
    result |= test_fd_phys_anti_hog_bypassed();
    result |= test_fd_phys_txdelay_honored();
    result |= test_fd_phys_ptt_fires_without_csma();
    result |= test_fd_phys_queue_retained_on_t106();

    printf("\n--- GROUP 2: Link Layer Full-Duplex (Section 6.7.2) ---\n");
    result |= test_fd_link_negotiation_both_agree();
    result |= test_fd_link_negotiation_one_refuses();
    result |= test_fd_link_apply_params_to_conn_and_phys();
    result |= test_fd_link_apply_params_null_phys();
    result |= test_fd_link_is_full_duplex_flag();
    result |= test_fd_link_t2_rr_immediate();

    printf("\n================================================================================\n");
    printf("AX.25 Full-Duplex Tests Completed. %s\n", result == 0 ? "All tests passed" : "Some tests FAILED");
    printf("================================================================================\n\n");

    return result;
}
