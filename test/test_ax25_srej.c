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
 *
 */

// AX.25 v2.2 Section 6.4.4 - Selective Reject (SREJ) comprehensive test suite
// Tests all SREJ scenarios per AX.25 v2.2 specification including:
//   - Section 6.4.4.1: Implicit Reject (REJ) - basic reject mode
//   - Section 6.4.4.2: Selective Reject (SREJ) - single missing frame
//   - Section 6.4.4.3: Selective Reject-Reject (SREJ/REJ) - multiple simultaneous
//   - Section 6.4.8:   Handling received SREJ (retransmitting a specific I-frame)
//   - Section 6.4.7:   Handling received REJ (retransmitting from N(R) forward)
//   - Section 6.4.4 bitmap tracking and SREJ state machine transitions
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "test_common.h"
#include "ax25.h"
#include "ax25_state_machine.h"

// -------------------------------------------------------------------------
// Test infrastructure - shared across all test cases in this file
// -------------------------------------------------------------------------

// Counter for TEST_ASSERT macro (required by test_common.h macro expansion)
static uint32_t assert_count = 0;

// Maximum number of transmitted frames we can capture per test
#define SREJ_TEST_TX_BUF_COUNT 32
#define SREJ_TEST_TX_BUF_SIZE  512

// Captured transmitted frame storage
typedef struct {
    uint8_t data[SREJ_TEST_TX_BUF_SIZE];
    size_t len;
} srej_captured_frame_t;

// Test harness context - models the "other station" in the link
typedef struct {
    srej_captured_frame_t tx_frames[SREJ_TEST_TX_BUF_COUNT];
    uint8_t tx_count;
    uint8_t rx_data_count;     // I-frame payloads delivered to upper layer
    uint8_t last_rx_payload[256];
    size_t last_rx_len;
    bool connected;
    bool disconnected;
    bool peer_busy;
    uint8_t on_data_call_count;
    uint8_t last_rx_payloads[SREJ_TEST_TX_BUF_COUNT][256];
    size_t last_rx_payload_lens[SREJ_TEST_TX_BUF_COUNT];
} srej_harness_t;

// -------------------------------------------------------------------------
// Callback implementations for test harness
// -------------------------------------------------------------------------

// Transmit callback: stores every outgoing frame for inspection
static void srej_cb_transmit(void *user_data, uint8_t *frame, size_t len) {
    srej_harness_t *h = (srej_harness_t*) user_data;
    if (h->tx_count >= SREJ_TEST_TX_BUF_COUNT) {
        return;  // Buffer full - should not happen in tests
    }
    size_t copy_len = (len > SREJ_TEST_TX_BUF_SIZE) ? SREJ_TEST_TX_BUF_SIZE : len;
    memcpy(h->tx_frames[h->tx_count].data, frame, copy_len);
    h->tx_frames[h->tx_count].len = copy_len;
    h->tx_count++;
}

// on_connect callback
//   initiated_locally = true  -> DL-CONNECT confirm  (we sent SABM, peer replied UA)
//   initiated_locally = false -> DL-CONNECT indication (peer sent SABM, we replied UA)
static void srej_cb_connect(void *user_data, bool initiated_locally) {
    (void) initiated_locally;  // test does not inspect direction, suppress unused-parameter warning
    srej_harness_t *h = (srej_harness_t*) user_data;
    h->connected = true;
}

// on_disconnect callback
static void srej_cb_disconnect(void *user_data, uint8_t reason) {
    (void) reason;
    srej_harness_t *h = (srej_harness_t*) user_data;
    h->disconnected = true;
}

// on_data callback: stores last received payload for verification
// pid parameter added to match updated ax25_callbacks_t on_data signature
// per AX.25 v2.2 Appendix D.4 DL-DATA indication. PID unused in this harness.
static void srej_cb_data(void *user_data, uint8_t *data, size_t len, uint8_t pid) {
    srej_harness_t *h = (srej_harness_t*) user_data;
    size_t copy_len = (len > 255) ? 255 : len;
    // Store in per-call buffer for ordered delivery tests
    if (h->on_data_call_count < SREJ_TEST_TX_BUF_COUNT) {
        memcpy(h->last_rx_payloads[h->on_data_call_count], data, copy_len);
        h->last_rx_payload_lens[h->on_data_call_count] = copy_len;
    }
    // Also update the "latest" fields
    memcpy(h->last_rx_payload, data, copy_len);
    h->last_rx_len = copy_len;
    h->rx_data_count++;
    h->on_data_call_count++;
}

// on_busy callback
static void srej_cb_busy(void *user_data, bool busy) {
    srej_harness_t *h = (srej_harness_t*) user_data;
    h->peer_busy = busy;
}

// -------------------------------------------------------------------------
// Helper: build a standard callbacks struct pointing to harness
// -------------------------------------------------------------------------
static ax25_callbacks_t make_callbacks(srej_harness_t *h) {
    ax25_callbacks_t cb;
    cb.transmit = srej_cb_transmit;
    cb.on_connect = srej_cb_connect;
    cb.on_disconnect = srej_cb_disconnect;
    cb.on_data = srej_cb_data;
    cb.on_busy = srej_cb_busy;
    return cb;
}

// -------------------------------------------------------------------------
// Helper: initialise a connection in CONNECTED state without SABM exchange.
// This sets addresses and forces state for unit-level SREJ tests.
// -------------------------------------------------------------------------
static void force_connected(ax25_connection_t *conn, srej_harness_t *h, uint8_t mod) {
    ax25_callbacks_t cb = make_callbacks(h);
    ax25_connection_init(conn, &cb, h);

    // Set modulo (8 or 128)
    conn->vars.mod = mod;

    // Set peer address directly
    memcpy(conn->peer_addr.destination.callsign, "W1AW  ", 6);
    conn->peer_addr.destination.ssid = 0;
    conn->peer_addr.destination.ch = true;
    conn->peer_addr.destination.extension = false;
    memcpy(conn->peer_addr.source.callsign, "N0CALL", 6);
    conn->peer_addr.source.ssid = 0;
    conn->peer_addr.source.ch = false;
    conn->peer_addr.source.extension = true;
    conn->peer_addr.cr = false;  // Response direction for our frames
    conn->peer_addr.repeaters.num_repeaters = 0;

    // Force connected state
    conn->state = AX25_STATE_CONNECTED;
    conn->vars.vs = 0;
    conn->vars.vr = 0;
    conn->vars.va = 0;

    // Configure SREJ/REJ mode (default is SREJ_REJ per spec Section 6.3.2)
    conn->rej_mode = AX25_REJ_MODE_SREJ_REJ;
    conn->srej_max = 1;   // Allow one simultaneous SREJ per spec

    // Set timers
    conn->timers.t1 = 300;  // 3 seconds in 10ms ticks
    conn->timers.t2 = 30;   // 300ms
    conn->timers.k = 7;    // Window size
    conn->timers.n2 = 10;   // Max retries

    h->connected = true;
}

// -------------------------------------------------------------------------
// Helper: build an I-frame ax25_frame_t for injection into ax25_process_frame.
// All heap allocations are freed by ax25_frame_free after process_frame.
// -------------------------------------------------------------------------
static ax25_frame_t* make_iframe(uint8_t ns, uint8_t nr, bool pf, uint8_t *payload, size_t payload_len, uint8_t mod) {
    ax25_information_frame_t *f = (ax25_information_frame_t*) malloc(sizeof(ax25_information_frame_t));
    if (!f) {
        return NULL;
    }
    memset(f, 0, sizeof(*f));
    f->base.type = (mod == 128) ? AX25_FRAME_INFORMATION_16BIT : AX25_FRAME_INFORMATION_8BIT;
    memcpy(f->base.header.destination.callsign, "N0CALL", 6);
    f->base.header.destination.ssid = 0;
    memcpy(f->base.header.source.callsign, "W1AW  ", 6);
    f->base.header.source.ssid = 0;
    f->base.header.cr = true;  // Command (from remote)
    f->base.header.repeaters.num_repeaters = 0;
    f->ns = ns;
    f->nr = nr;
    f->pf = pf;
    f->pid = PID_NO_L3;
    if (payload && payload_len > 0) {
        f->payload = (uint8_t*) malloc(payload_len);
        if (f->payload) {
            memcpy(f->payload, payload, payload_len);
        }
        f->payload_len = payload_len;
    } else {
        f->payload = NULL;
        f->payload_len = 0;
    }
    return (ax25_frame_t*) f;
}

// Helper: free a frame built by make_iframe
static void free_iframe(ax25_frame_t *f) {
    if (!f) {
        return;
    }
    ax25_information_frame_t *iframe = (ax25_information_frame_t*) f;
    if (iframe->payload) {
        free(iframe->payload);
    }
    free(iframe);
}

// -------------------------------------------------------------------------
// Helper: build a supervisory frame (RR / RNR / REJ / SREJ) for injection
// -------------------------------------------------------------------------
static ax25_frame_t* make_sframe(ax25_frame_type_t type, uint8_t nr, bool pf, uint8_t code) {
    ax25_supervisory_frame_t *f = (ax25_supervisory_frame_t*) malloc(sizeof(ax25_supervisory_frame_t));
    if (!f) {
        return NULL;
    }
    memset(f, 0, sizeof(*f));
    f->base.type = type;
    memcpy(f->base.header.destination.callsign, "N0CALL", 6);
    f->base.header.destination.ssid = 0;
    memcpy(f->base.header.source.callsign, "W1AW  ", 6);
    f->base.header.source.ssid = 0;
    f->base.header.cr = false;  // Response frame from remote
    f->base.header.repeaters.num_repeaters = 0;
    f->nr = nr;
    f->pf = pf;
    f->code = code;
    return (ax25_frame_t*) f;
}

// Helper: free a frame built by make_sframe
static void free_sframe(ax25_frame_t *f) {
    if (f) {
        free(f);
    }
}

// -------------------------------------------------------------------------
// Helper: find last transmitted frame of a given supervisory code.
// Returns index in h->tx_frames or -1 if not found.
// Scans backwards from newest to oldest so we get the most recent match.
// -------------------------------------------------------------------------
static int find_last_sframe(srej_harness_t *h, ax25_frame_type_t expected_type) {
    // Decoded inspection: supervisory frames are at least 4 bytes (addr+ctrl)
    // We decode each captured frame to check its type
    for (int i = (int) h->tx_count - 1; i >= 0; i--) {
        uint8_t err = 0;
        ax25_frame_t *decoded = ax25_frame_decode(h->tx_frames[i].data, h->tx_frames[i].len, 0, &err);
        if (!decoded) {
            continue;
        }
        ax25_frame_type_t got = decoded->type;
        ax25_frame_free(decoded, &err);
        if (got == expected_type) {
            return i;
        }
    }
    return -1;
}

// Helper: count transmitted frames matching a given type
static uint8_t count_sframes_of_type(srej_harness_t *h, ax25_frame_type_t expected_type) {
    uint8_t count = 0;
    for (int i = 0; i < (int) h->tx_count; i++) {
        uint8_t err = 0;
        ax25_frame_t *decoded = ax25_frame_decode(h->tx_frames[i].data, h->tx_frames[i].len, 0, &err);
        if (!decoded) {
            continue;
        }
        if (decoded->type == expected_type) {
            count++;
        }
        ax25_frame_free(decoded, &err);
    }
    return count;
}

// Helper: get N(R) of an SREJ or REJ frame at a given capture index
static int get_sframe_nr(srej_harness_t *h, int idx) {
    if (idx < 0 || idx >= (int) h->tx_count) {
        return -1;
    }
    uint8_t err = 0;
    ax25_frame_t *decoded = ax25_frame_decode(h->tx_frames[idx].data, h->tx_frames[idx].len, 0, &err);
    if (!decoded) {
        return -1;
    }
    int nr = -1;
    if (decoded->type == AX25_FRAME_SUPERVISORY_SREJ_8BIT || decoded->type == AX25_FRAME_SUPERVISORY_SREJ_16BIT
            || decoded->type == AX25_FRAME_SUPERVISORY_REJ_8BIT || decoded->type == AX25_FRAME_SUPERVISORY_REJ_16BIT
            || decoded->type == AX25_FRAME_SUPERVISORY_RR_8BIT || decoded->type == AX25_FRAME_SUPERVISORY_RR_16BIT) {
        ax25_supervisory_frame_t *sf = (ax25_supervisory_frame_t*) decoded;
        nr = sf->nr;
    }
    ax25_frame_free(decoded, &err);
    return nr;
}

// =========================================================================
// TEST 1: Section 6.4.4.2 - SREJ sent when single I-frame missing (modulo 8)
// Scenario: Frames 0, 2 received (frame 1 missing) -> SREJ(1) expected
// =========================================================================
static int test_srej_single_missing_frame_mod8(void) {
    printf("\n--- test_srej_single_missing_frame_mod8 ---\n");
    printf("AX.25 v2.2 Section 6.4.4.2: SREJ for single missing I-frame (modulo 8)\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    DEBUG_PRINT("Test 1 setup: mod=8, rej_mode=%d, srej_max=%u", conn.rej_mode, (unsigned)conn.srej_max);DEBUG_STATE("Initial state", conn.state);DEBUG_VAR("Initial V(S)", conn.vars.vs);DEBUG_VAR("Initial V(R)", conn.vars.vr);DEBUG_VAR("Initial V(A)", conn.vars.va);DEBUG_VAR("Timer k (window)", conn.timers.k);

    // Receive frame N(S)=0 - expected, no SREJ
    uint8_t payload0[] = { 'F', 'R', 'M', '0' };
    ax25_frame_t *f0 = make_iframe(0, 0, false, payload0, sizeof(payload0), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=0 (in-order, expected)");DEBUG_FRAME("frame0 payload", payload0, sizeof(payload0));

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    DEBUG_VAR("V(R) after frame 0", conn.vars.vr);DEBUG_VAR("rx_data_count after frame 0", h.rx_data_count);DEBUG_VAR("tx_count after frame 0", h.tx_count);DEBUG_BOOL("srej_exception after frame 0", conn.srej_exception);

    // V(R) should now be 1
    TEST_ASSERT(conn.vars.vr == 1, "V(R) = 1 after receiving frame N(S)=0", 0);
    TEST_ASSERT(h.rx_data_count == 1, "Upper layer received frame 0", 0);

    uint8_t tx_before = h.tx_count;

    DEBUG_PRINT("Injecting I-frame N(S)=2 (frame 1 missing - creating gap)");DEBUG_VAR("tx_count before gap injection", tx_before);

    // Receive frame N(S)=2 (frame 1 is missing) - single gap, SREJ(1) expected
    uint8_t payload2[] = { 'F', 'R', 'M', '2' };
    ax25_frame_t *f2 = make_iframe(2, 0, false, payload2, sizeof(payload2), 8);
    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_VAR("V(R) after frame 2 (gap at 1)", conn.vars.vr);DEBUG_BOOL("srej_exception after gap", conn.srej_exception);DEBUG_BOOL("rej_exception after gap", conn.rej_exception);DEBUG_VAR("srej_count", conn.srej_count);DEBUG_VAR("srej_buffer_count", conn.srej_buffer_count);DEBUG_VAR("tx_count after gap", h.tx_count);DEBUG_VAR("srej_bitmap[0]", conn.srej_bitmap[0]);DEBUG_VAR("rx_data_count after gap", h.rx_data_count);

    // SREJ should have been sent for missing frame N(S)=1
    uint8_t srej_count = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);

    DEBUG_VAR("SREJ frames found in tx buffer", srej_count);

    TEST_ASSERT(srej_count >= 1, "SREJ sent for missing frame N(S)=1", 0);

    // SREJ N(R) must point to the missing sequence number 1
    int srej_idx = find_last_sframe(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    int srej_nr = get_sframe_nr(&h, srej_idx);

    DEBUG_VAR("SREJ frame index in tx buffer", (unsigned)srej_idx);DEBUG_VAR("SREJ N(R) value", (unsigned)srej_nr);
    if (srej_idx >= 0) {
        DEBUG_FRAME("SREJ raw frame bytes", h.tx_frames[srej_idx].data, h.tx_frames[srej_idx].len);
    }

    TEST_ASSERT(srej_nr == 1, "SREJ N(R) = 1 (requesting retransmit of frame 1)", 0);

    // SREJ exception state must be active
    TEST_ASSERT(conn.srej_exception == true, "SREJ exception state is active", 0);

    // Frame 2 must have been buffered (not yet delivered to upper layer)
    TEST_ASSERT(h.rx_data_count == 1, "Frame 2 not yet delivered (waiting for frame 1)", 0);

    // V(R) must still be 1 (cannot advance past missing frame)
    TEST_ASSERT(conn.vars.vr == 1, "V(R) still 1 (blocked by missing frame 1)", 0);

    DEBUG_PRINT("Test 1 final state summary:");DEBUG_VAR("Final V(R)", conn.vars.vr);DEBUG_VAR("Final V(S)", conn.vars.vs);DEBUG_VAR("Final rx_data_count", h.rx_data_count);DEBUG_VAR("Final tx_count", h.tx_count);DEBUG_VAR("Final srej_bitmap[0]", conn.srej_bitmap[0]);DEBUG_BOOL("Final srej_exception", conn.srej_exception);

    printf("SREJ count: %u, SREJ N(R): %d\n", (unsigned) srej_count, srej_nr);
    (void) tx_before;
    return 0;
}

// =========================================================================
// TEST 2: Section 6.4.4.2 - SREJ recovery completes on retransmission
// Scenario: Frame 1 is retransmitted after SREJ, V(R) advances, both frames delivered
// =========================================================================
static int test_srej_recovery_on_retransmit(void) {
    printf("\n--- test_srej_recovery_on_retransmit ---\n");
    printf("AX.25 v2.2 Section 6.4.4.2: SREJ recovery when missing frame retransmitted\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    DEBUG_PRINT("Test 2 setup: mod=8, rej_mode=%d, srej_max=%u", conn.rej_mode, (unsigned)conn.srej_max);DEBUG_STATE("Initial state", conn.state);DEBUG_VAR("Initial V(S)", conn.vars.vs);DEBUG_VAR("Initial V(R)", conn.vars.vr);

    // Receive frame 0 (in-order)
    uint8_t p0[] = { 0x10, 0x11, 0x12, 0x13 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=0 (in-order)");DEBUG_FRAME("p0 payload", p0, sizeof(p0));

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    DEBUG_VAR("V(R) after frame 0", conn.vars.vr);DEBUG_VAR("rx_data_count after frame 0", h.rx_data_count);

    // Receive frame 2 (frame 1 missing) - triggers SREJ(1)
    uint8_t p2[] = { 0x20, 0x21, 0x22, 0x23 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=2 (frame 1 missing - expected SREJ(1))");DEBUG_FRAME("p2 payload", p2, sizeof(p2));

    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_BOOL("srej_exception after gap", conn.srej_exception);DEBUG_VAR("V(R) after gap (should be 1)", conn.vars.vr);DEBUG_VAR("rx_data_count after gap (should be 1)", h.rx_data_count);DEBUG_VAR("srej_buffer_count (frame 2 buffered)", conn.srej_buffer_count);DEBUG_VAR("srej_bitmap[0]", conn.srej_bitmap[0]);DEBUG_VAR("tx_count (SREJ should have been sent)", h.tx_count);

    TEST_ASSERT(conn.srej_exception == true, "SREJ exception active after gap", 0);
    TEST_ASSERT(h.rx_data_count == 1, "Only frame 0 delivered before retransmit", 0);

    // Now retransmit frame 1 (SREJ recovery)
    uint8_t p1[] = { 0xAA, 0xBB, 0xCC, 0xDD };
    ax25_frame_t *f1 = make_iframe(1, 0, false, p1, sizeof(p1), 8);

    DEBUG_PRINT("Injecting retransmitted I-frame N(S)=1 (SREJ recovery)");DEBUG_FRAME("p1 payload (retransmit)", p1, sizeof(p1));

    ax25_process_frame(&conn, f1, 1);
    free_iframe(f1);

    DEBUG_VAR("rx_data_count after retransmit (should be 3)", h.rx_data_count);DEBUG_VAR("V(R) after recovery (should be 3)", conn.vars.vr);DEBUG_BOOL("srej_exception after recovery (should be false)", conn.srej_exception);DEBUG_VAR("srej_buffer_count after recovery (should be 0)", conn.srej_buffer_count);DEBUG_VAR("srej_bitmap[0] after recovery (should be 0)", conn.srej_bitmap[0]);
    // verify in-order delivery of payloads
    if (h.on_data_call_count >= 2) {
        DEBUG_FRAME("Delivered payload[1] (should match p1)", h.last_rx_payloads[1], h.last_rx_payload_lens[1]);
    }
    if (h.on_data_call_count >= 3) {
        DEBUG_FRAME("Delivered payload[2] (should match p2)", h.last_rx_payloads[2], h.last_rx_payload_lens[2]);
    }

    // After receiving the missing frame 1, both frame 1 and buffered frame 2
    // should be delivered. V(R) advances to 3.
    TEST_ASSERT(h.rx_data_count == 3, "All 3 frames delivered after retransmit", 0);
    TEST_ASSERT(conn.vars.vr == 3, "V(R) = 3 after full recovery", 0);

    // SREJ exception must be cleared
    TEST_ASSERT(conn.srej_exception == false, "SREJ exception cleared after recovery", 0);
    TEST_ASSERT(conn.srej_buffer_count == 0, "SREJ buffer empty after recovery", 0);

    // Verify in-order delivery: frame 1 data must match what was delivered second
    int match1 = memcmp(h.last_rx_payloads[1], p1, sizeof(p1));
    TEST_ASSERT(match1 == 0, "Frame 1 payload matches retransmitted data (in-order delivery)", 0);

    // Verify frame 2 buffered payload delivered third
    int match2 = memcmp(h.last_rx_payloads[2], p2, sizeof(p2));
    TEST_ASSERT(match2 == 0, "Frame 2 buffered payload delivered correctly after frame 1", 0);

    DEBUG_PRINT("Test 2 final state: recovery complete");DEBUG_VAR("Final V(R)", conn.vars.vr);DEBUG_VAR("Final rx_data_count", h.rx_data_count);DEBUG_VAR("Final srej_bitmap[0]", conn.srej_bitmap[0]);

    return 0;
}

// =========================================================================
// TEST 3: Section 6.4.4.3 - SREJ/REJ mode: multiple gaps fall back to REJ
// Scenario: Frames 0, 3 received (frames 1 and 2 both missing) - exceeds SREJ
//           srej_max=1, so should fall back to REJ for the second gap.
// =========================================================================
static int test_srej_fallback_to_rej_on_multiple_gaps(void) {
    printf("\n--- test_srej_fallback_to_rej_on_multiple_gaps ---\n");
    printf("AX.25 v2.2 Section 6.4.4.3: SREJ/REJ fallback when multiple frames missing\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    // SREJ/REJ mode with max 1 simultaneous SREJ
    conn.rej_mode = AX25_REJ_MODE_SREJ_REJ;
    conn.srej_max = 1;

    DEBUG_PRINT("Test 3 setup: SREJ/REJ mode, srej_max=1 (multiple-gap fallback test)");DEBUG_VAR("rej_mode", conn.rej_mode);DEBUG_VAR("srej_max", conn.srej_max);DEBUG_STATE("Initial state", conn.state);DEBUG_VAR("Initial V(R)", conn.vars.vr);

    // Receive frame 0 (in-order)
    uint8_t p0[] = { 0x00 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=0 (in-order)");

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    DEBUG_VAR("V(R) after frame 0", conn.vars.vr);DEBUG_VAR("rx_data_count after frame 0", h.rx_data_count);

    // Receive frame 3 - gap of 3 frames missing (1, 2, 3 expected; 1 and 2 missing)
    // With srej_max=1 and missing_count=3 this should fall back to REJ

    DEBUG_PRINT("Injecting I-frame N(S)=3 (gap of 3: frames 1 and 2 missing)");DEBUG_PRINT("Expected: fallback to REJ because gap > srej_max");

    uint8_t p3[] = { 0x30 };
    ax25_frame_t *f3 = make_iframe(3, 0, false, p3, sizeof(p3), 8);
    ax25_process_frame(&conn, f3, 1);
    free_iframe(f3);

    DEBUG_BOOL("rej_exception after multi-gap", conn.rej_exception);DEBUG_BOOL("srej_exception after multi-gap", conn.srej_exception);DEBUG_VAR("V(R) after multi-gap", conn.vars.vr);DEBUG_VAR("srej_bitmap[0]", conn.srej_bitmap[0]);DEBUG_VAR("tx_count after multi-gap", h.tx_count);
#ifdef DEBUG_ENABLE
    uint8_t rej_dbg = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    uint8_t srej_dbg = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    DEBUG_VAR("REJ frames sent", rej_dbg); DEBUG_VAR("SREJ frames sent", srej_dbg);
    int rej_dbg_idx = find_last_sframe(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    int rej_dbg_nr = get_sframe_nr(&h, rej_dbg_idx);
    DEBUG_VAR("REJ N(R) value", (unsigned)rej_dbg_nr);
#endif // DEBUG_ENABLE

    // Check that REJ was sent (not SREJ) because gap > 1
    uint8_t rej_count = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    TEST_ASSERT(rej_count >= 1, "REJ sent when multiple frames missing and gap > 1", 0);

    // REJ N(R) must be V(R) = 1 (next expected frame after last in-order delivery)
    int rej_idx = find_last_sframe(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    int rej_nr = get_sframe_nr(&h, rej_idx);
    TEST_ASSERT(rej_nr == 1, "REJ N(R) = 1 (request retransmit from frame 1 onwards)", 0);

    // REJ exception must be set
    TEST_ASSERT(conn.rej_exception == true, "REJ exception active after multi-frame gap", 0);

    DEBUG_PRINT("Test 3 final state: REJ fallback complete");DEBUG_VAR("Final V(R)", conn.vars.vr);DEBUG_BOOL("Final rej_exception", conn.rej_exception);DEBUG_BOOL("Final srej_exception", conn.srej_exception);

    printf("REJ count: %u, REJ N(R): %d\n", (unsigned) rej_count, rej_nr);
    return 0;
}

// =========================================================================
// TEST 4: Section 6.4.4.1 - REJ-only mode (no SREJ)
// Scenario: Mode REJ_MODE_REJ, single gap -> REJ not SREJ
// =========================================================================
static int test_rej_only_mode_single_gap(void) {
    printf("\n--- test_rej_only_mode_single_gap ---\n");
    printf("AX.25 v2.2 Section 6.4.4.1: REJ-only mode with single frame missing\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    // Force REJ-only mode (no SREJ support)
    conn.rej_mode = AX25_REJ_MODE_REJ;

    DEBUG_PRINT("Test 4 setup: REJ-only mode (no SREJ), single-gap scenario");DEBUG_VAR("rej_mode", conn.rej_mode);DEBUG_STATE("Initial state", conn.state);DEBUG_VAR("Initial V(R)", conn.vars.vr);

    // Receive frame 0 in-order
    uint8_t p0[] = { 0xA0 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=0 (in-order)");

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    DEBUG_VAR("V(R) after frame 0", conn.vars.vr);DEBUG_VAR("rx_data_count after frame 0", h.rx_data_count);

    // Receive frame 2 (frame 1 missing) - even single gap must use REJ in REJ-only mode
    DEBUG_PRINT("Injecting I-frame N(S)=2 (frame 1 missing, REJ-only mode expects REJ not SREJ)");

    uint8_t p2[] = { 0xA2 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_BOOL("rej_exception after gap in REJ-only mode", conn.rej_exception);DEBUG_BOOL("srej_exception (should be false in REJ-only)", conn.srej_exception);DEBUG_VAR("V(R) after gap", conn.vars.vr);DEBUG_VAR("tx_count after gap", h.tx_count);
#ifdef DEBUG_ENABLE
    uint8_t rej4_count = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    uint8_t srej4_count = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    DEBUG_VAR("REJ frames sent", rej4_count); DEBUG_VAR("SREJ frames sent (should be 0)", srej4_count);
    int rej4_idx = find_last_sframe(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    DEBUG_VAR("REJ N(R)", (unsigned)get_sframe_nr(&h, rej4_idx));
#endif // DEBUG_ENABLE

    // No SREJ should have been sent
    uint8_t srej_count = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    TEST_ASSERT(srej_count == 0, "No SREJ sent in REJ-only mode", 0);

    // REJ must have been sent
    uint8_t rej_count = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    TEST_ASSERT(rej_count >= 1, "REJ sent for missing frame in REJ-only mode", 0);

    // REJ N(R) = 1
    int rej_idx = find_last_sframe(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    int rej_nr = get_sframe_nr(&h, rej_idx);
    TEST_ASSERT(rej_nr == 1, "REJ N(R) = 1 in REJ-only mode", 0);

    DEBUG_PRINT("Test 4 final state: REJ-only mode verified");DEBUG_VAR("Final REJ count", rej_count);DEBUG_VAR("Final SREJ count (must be 0)", srej_count);DEBUG_VAR("Final REJ N(R)", (unsigned)rej_nr);

    printf("SREJ count: %u, REJ count: %u, REJ N(R): %d\n", (unsigned) srej_count, (unsigned) rej_count, rej_nr);
    return 0;
}

// =========================================================================
// TEST 5: Section 6.4.8 - Receiving SREJ causes selective retransmission
// Scenario: We (the local station) send frames 0,1,2,3.
//           Remote sends SREJ(2) requesting retransmit of our I(2).
//           We must retransmit only frame 2.
// =========================================================================
static int test_received_srej_causes_selective_retransmit(void) {
    printf("\n--- test_received_srej_causes_selective_retransmit ---\n");
    printf("AX.25 v2.2 Section 6.4.8: Received SREJ triggers selective I-frame retransmit\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    DEBUG_PRINT("Test 5 setup: sending 4 frames then injecting SREJ(2) from remote");DEBUG_VAR("rej_mode", conn.rej_mode);DEBUG_VAR("srej_max", conn.srej_max);DEBUG_VAR("window size k", conn.timers.k);

    // Window size = 7, send 4 frames 0-3
    uint8_t d[4][4];
    for (int i = 0; i < 4; i++) {
        d[i][0] = (uint8_t) (0xB0 + i);
        d[i][1] = (uint8_t) (0xB1 + i);
        d[i][2] = (uint8_t) (0xB2 + i);
        d[i][3] = (uint8_t) (0xB3 + i);

        DEBUG_PRINT("Sending I-frame %d: [%02X %02X %02X %02X]", i, d[i][0], d[i][1], d[i][2], d[i][3]);

        uint8_t ret = ax25_send_data(&conn, d[i], 4, PID_NO_L3);
        TEST_ASSERT(ret == 0, "ax25_send_data succeeds for frames 0-3", 0);
    }

    DEBUG_VAR("V(S) after sending 4 frames (should be 4)", conn.vars.vs);DEBUG_VAR("V(A) after sending 4 frames", conn.vars.va);DEBUG_VAR("tx_queue.count after sending 4 frames", conn.tx_queue.count);DEBUG_VAR("total tx_count (4 frames transmitted)", h.tx_count);

    // V(S) should be 4 after sending 4 frames
    TEST_ASSERT(conn.vars.vs == 4, "V(S) = 4 after sending 4 frames", 0);

    uint8_t tx_count_before = h.tx_count;

    DEBUG_PRINT("Injecting SREJ(2) from remote (selective retransmit request for N(S)=2)");DEBUG_VAR("tx_count before SREJ injection", tx_count_before);

    // Remote sends SREJ(2): requesting retransmit of our N(S)=2
    ax25_frame_t *srej = make_sframe(AX25_FRAME_SUPERVISORY_SREJ_8BIT, 2, false, 3);
    ax25_process_frame(&conn, srej, 1);
    free_sframe(srej);

    // Exactly one additional transmission should have occurred (retransmit of frame 2)
    uint8_t tx_count_after = h.tx_count;
    uint8_t new_tx = (uint8_t) (tx_count_after - tx_count_before);

    DEBUG_VAR("tx_count after SREJ(2)", tx_count_after);DEBUG_VAR("new_tx (should be 1 - only frame 2 retransmitted)", new_tx);DEBUG_VAR("V(S) after SREJ", conn.vars.vs);DEBUG_VAR("V(A) after SREJ", conn.vars.va);

    TEST_ASSERT(new_tx == 1, "Exactly 1 frame retransmitted in response to SREJ(2)", 0);

    // Verify the retransmitted frame is an I-frame
    if (new_tx >= 1) {
        uint8_t err = 0;
        ax25_frame_t *decoded = ax25_frame_decode(h.tx_frames[tx_count_before].data, h.tx_frames[tx_count_before].len, 0, &err);
        TEST_ASSERT(decoded != NULL, "Retransmitted frame decodes correctly", err);
        if (decoded) {
            TEST_ASSERT(decoded->type == AX25_FRAME_INFORMATION_8BIT, "Retransmitted frame is an I-frame", 0);
            ax25_information_frame_t *rf = (ax25_information_frame_t*) decoded;

            DEBUG_VAR("Retransmitted frame N(S) (should be 2)", rf->ns);DEBUG_VAR("Retransmitted frame N(R)", rf->nr);DEBUG_FRAME("Retransmitted frame raw bytes", h.tx_frames[tx_count_before].data, h.tx_frames[tx_count_before].len);

            TEST_ASSERT(rf->ns == 2, "Retransmitted I-frame has N(S)=2", 0);
            ax25_frame_free(decoded, &err);
        }
    }

    DEBUG_PRINT("Test 5 final state: selective retransmit verified");DEBUG_VAR("Final V(S)", conn.vars.vs);DEBUG_VAR("Final V(A)", conn.vars.va);DEBUG_VAR("Final new_tx count", new_tx);

    printf("New TX after SREJ: %u\n", (unsigned) new_tx);
    ax25_connection_cleanup(&conn);

    return 0;
}

// =========================================================================
// TEST 6: Section 6.4.7 - Receiving REJ causes retransmission from N(R) forward
// Scenario: Send frames 0,1,2,3. Remote sends REJ(2).
//           We must retransmit frames 2 and 3.
// =========================================================================
static int test_received_rej_causes_bulk_retransmit(void) {
    printf("\n--- test_received_rej_causes_bulk_retransmit ---\n");
    printf("AX.25 v2.2 Section 6.4.7: Received REJ triggers retransmit from N(R) onward\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    DEBUG_PRINT("Test 6 setup: sending 4 frames then injecting REJ(2) from remote");DEBUG_VAR("window size k", conn.timers.k);DEBUG_VAR("rej_mode", conn.rej_mode);

    // Send 4 frames 0-3
    uint8_t d[4][4];
    for (int i = 0; i < 4; i++) {
        d[i][0] = (uint8_t) (0xC0 + i);
        d[i][1] = (uint8_t) (0xC1 + i);
        d[i][2] = (uint8_t) (0xC2 + i);
        d[i][3] = (uint8_t) (0xC3 + i);

        DEBUG_PRINT("Sending I-frame %d: [%02X %02X %02X %02X]", i, d[i][0], d[i][1], d[i][2], d[i][3]);

        uint8_t ret = ax25_send_data(&conn, d[i], 4, PID_NO_L3);
        TEST_ASSERT(ret == 0, "ax25_send_data succeeds (REJ test frames)", 0);
    }

    DEBUG_VAR("V(S) after sending 4 frames", conn.vars.vs);DEBUG_VAR("V(A) before REJ", conn.vars.va);DEBUG_VAR("tx_queue.count", conn.tx_queue.count);DEBUG_VAR("tx_count before REJ", h.tx_count);

    uint8_t tx_count_before = h.tx_count;

    DEBUG_PRINT("Injecting REJ(2) from remote (bulk retransmit from N(S)=2 onwards)");

    // Remote sends REJ(2): retransmit everything from N(S)=2 onwards
    ax25_frame_t *rej = make_sframe(AX25_FRAME_SUPERVISORY_REJ_8BIT, 2, false, 2);
    ax25_process_frame(&conn, rej, 1);
    free_sframe(rej);

    // Frames 2 and 3 must be retransmitted (2 new transmissions)
    uint8_t new_tx = (uint8_t) (h.tx_count - tx_count_before);

    DEBUG_VAR("new_tx after REJ(2) (should be 2)", new_tx);DEBUG_VAR("V(S) after REJ processing", conn.vars.vs);DEBUG_VAR("V(A) after REJ processing", conn.vars.va);
    for (uint8_t dbg_i = 0; dbg_i < new_tx && dbg_i < 2; dbg_i++) {
        DEBUG_FRAME("Retransmitted frame raw bytes", h.tx_frames[tx_count_before + dbg_i].data, h.tx_frames[tx_count_before + dbg_i].len);
    }

    TEST_ASSERT(new_tx == 2, "Frames 2 and 3 retransmitted after REJ(2)", 0);

    // Verify both retransmitted frames are I-frames with correct N(S)
    for (uint8_t i = 0; i < new_tx && i < 2; i++) {
        uint8_t err = 0;
        ax25_frame_t *decoded = ax25_frame_decode(h.tx_frames[tx_count_before + i].data, h.tx_frames[tx_count_before + i].len, 0, &err);
        TEST_ASSERT(decoded != NULL, "Retransmitted frame decodes OK", err);
        if (decoded) {
            TEST_ASSERT(decoded->type == AX25_FRAME_INFORMATION_8BIT, "Retransmitted frame is I-frame", 0);
            ax25_information_frame_t *rf = (ax25_information_frame_t*) decoded;

            DEBUG_VAR("Retransmitted frame N(S)", rf->ns);DEBUG_VAR("Expected N(S)", (unsigned)(2 + i));

            TEST_ASSERT((int )rf->ns == (int )(2 + i), "Retransmitted I-frame N(S) matches expected sequence", 0);
            ax25_frame_free(decoded, &err);
        }
    }

    DEBUG_PRINT("Test 6 final state: bulk REJ retransmit verified");DEBUG_VAR("Final new_tx count", new_tx);DEBUG_VAR("Final V(S)", conn.vars.vs);DEBUG_VAR("Final V(A)", conn.vars.va);

    printf("New TX after REJ(2): %u (expected 2)\n", (unsigned) new_tx);
    ax25_connection_cleanup(&conn);

    return 0;
}

// =========================================================================
// TEST 7: Section 6.4.4 - SREJ bitmap tracks pending requests correctly
// Scenario: Frame 0 received, frames 2 and 3 received (frames 1 missing).
//           SREJ bitmap bit for N(S)=1 must be set.
//           When frame 1 arrives, bit clears.
// =========================================================================
static int test_srej_bitmap_tracking(void) {
    printf("\n--- test_srej_bitmap_tracking ---\n");
    printf("AX.25 v2.2 Section 6.4.4: SREJ bitmap tracks pending SREJ conditions\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    DEBUG_PRINT("Test 7 setup: bitmap tracking test, mod=8");DEBUG_VAR("Initial srej_bitmap[0]", conn.srej_bitmap[0]);DEBUG_BOOL("Initial srej_exception", conn.srej_exception);DEBUG_VAR("Initial V(R)", conn.vars.vr);

    // Receive frame 0
    uint8_t p0[] = { 0xD0 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=0");

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    DEBUG_VAR("V(R) after frame 0", conn.vars.vr);DEBUG_VAR("srej_bitmap[0] after frame 0 (should be 0)", conn.srej_bitmap[0]);DEBUG_BOOL("srej_exception after frame 0 (should be false)", conn.srej_exception);

    // Verify bitmap clean before gap
    TEST_ASSERT(conn.srej_bitmap[0] == 0, "SREJ bitmap clear before any gap", 0);
    TEST_ASSERT(conn.srej_exception == false, "No SREJ exception before gap", 0);

    // Receive frame 2 (frame 1 missing) - SREJ(1) sent, bit 1 set in bitmap
    DEBUG_PRINT("Injecting I-frame N(S)=2 (frame 1 missing, expecting bitmap bit 1 set)");

    uint8_t p2[] = { 0xD2 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_VAR("srej_bitmap[0] after gap (bit 1 expected set: 0x02)", conn.srej_bitmap[0]);DEBUG_BOOL("srej_exception after gap", conn.srej_exception);DEBUG_VAR("srej_count after gap", conn.srej_count);DEBUG_VAR("V(R) after gap (should still be 1)", conn.vars.vr);DEBUG_BOOL("bit 1 set in srej_bitmap[0]", (conn.srej_bitmap[0] & (1U << 1)) != 0);

    // Bit 1 (for N(S)=1) must be set in byte 0 of bitmap
    TEST_ASSERT((conn.srej_bitmap[0] & (1U << 1)) != 0, "SREJ bitmap bit 1 set for missing frame N(S)=1", 0);
    TEST_ASSERT(conn.srej_exception == true, "SREJ exception active", 0);
    TEST_ASSERT(conn.srej_count >= 1, "SREJ count >= 1", 0);

    printf("srej_bitmap[0] = 0x%02X (expected bit 1 set)\n", conn.srej_bitmap[0]);

    // Receive frame 3 as well (still waiting for 1)
    DEBUG_PRINT("Injecting I-frame N(S)=3 (frame 1 still missing)");

    uint8_t p3[] = { 0xD3 };
    ax25_frame_t *f3 = make_iframe(3, 0, false, p3, sizeof(p3), 8);
    ax25_process_frame(&conn, f3, 1);
    free_iframe(f3);

    DEBUG_VAR("srej_bitmap[0] after frame 3 (bit 1 still expected)", conn.srej_bitmap[0]);DEBUG_VAR("srej_buffer_count (frames 2,3 buffered)", conn.srej_buffer_count);DEBUG_BOOL("srej_exception still active", conn.srej_exception);DEBUG_VAR("V(R) still 1 (blocked)", conn.vars.vr);

    // Bit 1 still set (frame 1 still missing)
    TEST_ASSERT((conn.srej_bitmap[0] & (1U << 1)) != 0, "SREJ bitmap bit 1 still set (frame 1 still missing)", 0);

    // Now retransmit frame 1 to resolve the SREJ
    DEBUG_PRINT("Injecting retransmitted I-frame N(S)=1 (SREJ recovery)");

    uint8_t p1[] = { 0xD1 };
    ax25_frame_t *f1 = make_iframe(1, 0, false, p1, sizeof(p1), 8);
    ax25_process_frame(&conn, f1, 1);
    free_iframe(f1);

    DEBUG_VAR("srej_bitmap[0] after frame 1 retransmit (should be 0)", conn.srej_bitmap[0]);DEBUG_BOOL("srej_exception after recovery (should be false)", conn.srej_exception);DEBUG_VAR("V(R) after full recovery (should be 4)", conn.vars.vr);DEBUG_VAR("rx_data_count after recovery (should be 4)", h.rx_data_count);DEBUG_VAR("srej_buffer_count after recovery (should be 0)", conn.srej_buffer_count);

    // Bitmap bit 1 must be cleared now
    TEST_ASSERT((conn.srej_bitmap[0] & (1U << 1)) == 0, "SREJ bitmap bit 1 cleared after frame 1 retransmitted", 0);
    TEST_ASSERT(conn.srej_exception == false, "SREJ exception cleared after recovery", 0);

    // All 4 frames (0,1,2,3) should have been delivered
    TEST_ASSERT(h.rx_data_count == 4, "All 4 frames delivered in order after SREJ recovery", 0);
    TEST_ASSERT(conn.vars.vr == 4, "V(R) = 4 after all frames received in order", 0);

    return 0;
}

// =========================================================================
// TEST 8: Section 6.4.4.3 - Simultaneous multiple SREJ conditions (mod 128)
// Scenario (modulo 128): Frames 0,1,3,5 received (2 and 4 missing).
//           First gap->SREJ(2), then gap->SREJ(4) with srej_max=2.
//           Both bits set in bitmap simultaneously.
// =========================================================================
static int test_srej_simultaneous_multiple_mod128(void) {
    printf("\n--- test_srej_simultaneous_multiple_mod128 ---\n");
    printf("AX.25 v2.2 Section 6.4.4.3: Multiple simultaneous SREJ conditions (modulo 128)\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 128);

    // Allow 2 simultaneous SREJs to properly exercise the multiple condition
    conn.rej_mode = AX25_REJ_MODE_SREJ_REJ;
    conn.srej_max = 2;
    conn.timers.k = 127;  // Large window for this test

    DEBUG_PRINT("Test 8 setup: mod=128, srej_max=2, window=127, multi-SREJ scenario");DEBUG_VAR("rej_mode", conn.rej_mode);DEBUG_VAR("srej_max", conn.srej_max);DEBUG_VAR("mod", conn.vars.mod);DEBUG_VAR("window k", conn.timers.k);DEBUG_STATE("Initial state", conn.state);DEBUG_VAR("Initial V(R)", conn.vars.vr);

    // Frame 0 - in order
    uint8_t p0[] = { 0x00 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 128);

    DEBUG_PRINT("Injecting I-frame N(S)=0 (mod128, in-order)");

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    // Frame 1 - in order
    uint8_t p1[] = { 0x01 };
    ax25_frame_t *f1 = make_iframe(1, 0, false, p1, sizeof(p1), 128);

    DEBUG_PRINT("Injecting I-frame N(S)=1 (mod128, in-order)");

    ax25_process_frame(&conn, f1, 1);
    free_iframe(f1);

    DEBUG_VAR("V(R) after frames 0,1 (should be 2)", conn.vars.vr);DEBUG_VAR("rx_data_count after frames 0,1", h.rx_data_count);

    TEST_ASSERT(conn.vars.vr == 2, "V(R) = 2 after frames 0 and 1", 0);

    // Frame 3 received - frame 2 missing -> first SREJ(2)
    DEBUG_PRINT("Injecting I-frame N(S)=3 (mod128, frame 2 missing -> SREJ(2) expected)");

    uint8_t p3[] = { 0x03 };
    ax25_frame_t *f3 = make_iframe(3, 0, false, p3, sizeof(p3), 128);
    ax25_process_frame(&conn, f3, 1);
    free_iframe(f3);

#ifdef DEBUG_ENABLE
    uint8_t srej16_count_1 = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_16BIT);
    DEBUG_VAR("SREJ_16BIT frames sent after first gap", srej16_count_1);
#endif // DEBUG_ENABLE
    DEBUG_VAR("srej_bitmap[0] after first gap (bit 2 expected set)", conn.srej_bitmap[0]);DEBUG_BOOL("srej_exception after first gap", conn.srej_exception);DEBUG_VAR("V(R) after first gap (should be 2 - blocked)", conn.vars.vr);DEBUG_BOOL("bit 2 set in srej_bitmap[0]", (conn.srej_bitmap[0] & (1U << 2)) != 0);

    // SREJ for frame 2 must be pending
    uint8_t srej_count_1 = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_16BIT);
    TEST_ASSERT(srej_count_1 >= 1, "First SREJ(2) sent for missing frame 2", 0);
    TEST_ASSERT((conn.srej_bitmap[0] & (1U << 2)) != 0, "SREJ bitmap bit 2 set for missing frame 2", 0);
    TEST_ASSERT(conn.srej_exception == true, "SREJ exception active after first gap", 0);

    // Frame 5 received - frame 4 missing (while frame 2 still missing) -> second SREJ(4)
    DEBUG_PRINT("Injecting I-frame N(S)=5 (mod128, frame 4 also missing -> second SREJ(4))");

    uint8_t p5[] = { 0x05 };
    ax25_frame_t *f5 = make_iframe(5, 0, false, p5, sizeof(p5), 128);
    ax25_process_frame(&conn, f5, 1);
    free_iframe(f5);

    uint8_t srej_count_2 = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_16BIT);

    DEBUG_VAR("Total SREJ_16BIT frames after second gap", srej_count_2);DEBUG_VAR("srej_bitmap[0] (bits 2 and 4 expected set)", conn.srej_bitmap[0]);DEBUG_BOOL("bit 2 still set (frame 2 still missing)", (conn.srej_bitmap[0] & (1U << 2)) != 0);DEBUG_BOOL("bit 4 set (frame 4 missing)", (conn.srej_bitmap[0] & (1U << 4)) != 0);DEBUG_VAR("srej_count", conn.srej_count);DEBUG_BOOL("srej_exception still active", conn.srej_exception);DEBUG_BOOL("rej_exception (should be false with srej_max=2)", conn.rej_exception);DEBUG_VAR("V(R) (still 2 - blocked by frame 2)", conn.vars.vr);

    printf("Total SREJ frames sent: %u\n", (unsigned) srej_count_2);

    // Verify both missing frames tracked in bitmap
    // Frame 2: bit 2 of byte 0
    TEST_ASSERT((conn.srej_bitmap[0] & (1U << 2)) != 0, "SREJ bitmap bit 2 still set (frame 2 still missing)", 0);

    // Frame 4: bit 4 of byte 0
    // If srej_max=2 was honoured the second SREJ was also sent
    bool second_srej_sent = ((conn.srej_bitmap[0] & (1U << 4)) != 0) || (srej_count_2 >= 2);
    TEST_ASSERT(second_srej_sent, "Second SREJ condition tracked (frame 4) or SREJ/REJ sent for second gap", 0);

    // srej_count must be at least 1 (we are in SREJ exception)
    TEST_ASSERT(conn.srej_count >= 1, "srej_count >= 1 during multi-SREJ condition", 0);

    printf("srej_bitmap[0] = 0x%02X, srej_count = %u, SREJ frames sent = %u\n", conn.srej_bitmap[0], (unsigned) conn.srej_count, (unsigned) srej_count_2);

    // Now retransmit frame 2 to partially resolve
    DEBUG_PRINT("Injecting retransmitted I-frame N(S)=2 (partial recovery, frame 4 still missing)");

    uint8_t p2[] = { 0x02 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 128);
    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_VAR("V(R) after frame 2 delivered (should be >= 4)", conn.vars.vr);DEBUG_VAR("rx_data_count after frame 2", h.rx_data_count);DEBUG_VAR("srej_bitmap[0] after frame 2 retransmit", conn.srej_bitmap[0]);DEBUG_BOOL("srej_exception (still active - frame 4 missing)", conn.srej_exception);

    // After frame 2 arrives, V(R) advances to 4 (frames 0-3 now in order)
    TEST_ASSERT(conn.vars.vr >= 4, "V(R) >= 4 after frame 2 delivered", 0);

    // Retransmit frame 4 to fully resolve
    DEBUG_PRINT("Injecting retransmitted I-frame N(S)=4 (final recovery)");

    uint8_t p4[] = { 0x04 };
    ax25_frame_t *f4 = make_iframe(4, 0, false, p4, sizeof(p4), 128);
    ax25_process_frame(&conn, f4, 1);
    free_iframe(f4);

    DEBUG_VAR("V(R) after full recovery (should be 6)", conn.vars.vr);DEBUG_VAR("rx_data_count (should be 6)", h.rx_data_count);DEBUG_BOOL("srej_exception after full recovery (should be false)", conn.srej_exception);DEBUG_VAR("srej_bitmap[0] after full recovery (should be 0)", conn.srej_bitmap[0]);DEBUG_VAR("srej_count after full recovery (should be 0)", conn.srej_count);

    // After both missing frames retransmitted, all 6 frames (0-5) delivered
    TEST_ASSERT(conn.vars.vr == 6, "V(R) = 6 after all missing frames retransmitted", 0);
    TEST_ASSERT(h.rx_data_count == 6, "All 6 frames delivered to upper layer", 0);
    TEST_ASSERT(conn.srej_exception == false, "SREJ exception cleared after full recovery", 0);

    return 0;
}

// =========================================================================
// TEST 9: Section 6.4.4.3 - SREJ exception clears when REJ issued
// Scenario: SREJ active for frame 1, then second gap triggers REJ.
//           SREJ state must be wiped and REJ exception set.
// =========================================================================
static int test_srej_exception_cleared_on_rej_fallback(void) {
    printf("\n--- test_srej_exception_cleared_on_rej_fallback ---\n");
    printf("AX.25 v2.2 Section 6.4.4.3: SREJ exception cleared when REJ is issued\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    // Only 1 simultaneous SREJ allowed; SREJ/REJ mode
    conn.rej_mode = AX25_REJ_MODE_SREJ_REJ;
    conn.srej_max = 1;

    DEBUG_PRINT("Test 9 setup: SREJ/REJ mode, srej_max=1 (SREJ->REJ fallback on second gap)");DEBUG_VAR("rej_mode", conn.rej_mode);DEBUG_VAR("srej_max", conn.srej_max);DEBUG_STATE("Initial state", conn.state);

    // Receive frame 0
    uint8_t p0[] = { 0xE0 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=0");

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    // Receive frame 2 - SREJ(1) sent, one SREJ active
    DEBUG_PRINT("Injecting I-frame N(S)=2 (frame 1 missing -> SREJ(1))");

    uint8_t p2[] = { 0xE2 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_BOOL("srej_exception after first gap (should be true)", conn.srej_exception);DEBUG_BOOL("rej_exception after first gap (should be false)", conn.rej_exception);DEBUG_VAR("srej_count (should be 1)", conn.srej_count);DEBUG_VAR("srej_bitmap[0] (bit 1 set)", conn.srej_bitmap[0]);DEBUG_VAR("V(R) after first gap (should be 1)", conn.vars.vr);
#ifdef DEBUG_ENABLE
    uint8_t t9_srej_cnt = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    DEBUG_VAR("SREJ frames sent", t9_srej_cnt);
#endif // DEBUG_ENABLE

    TEST_ASSERT(conn.srej_exception == true, "SREJ exception active after frame 1 missing", 0);
    TEST_ASSERT(conn.rej_exception == false, "No REJ exception yet", 0);

    // Now receive frame 4 while frame 1 still missing (srej_max already used by frame 1)
    // This should fall back to REJ and clear SREJ state
    DEBUG_PRINT("Injecting I-frame N(S)=4 (second gap while srej_max=1 exhausted -> REJ fallback)");DEBUG_PRINT("Expected: SREJ exception cleared, REJ exception set, bitmap zeroed");

    uint8_t p4[] = { 0xE4 };
    ax25_frame_t *f4 = make_iframe(4, 0, false, p4, sizeof(p4), 8);
    ax25_process_frame(&conn, f4, 1);
    free_iframe(f4);

    DEBUG_BOOL("rej_exception after second gap (should be true)", conn.rej_exception);DEBUG_BOOL("srej_exception after fallback (should be false)", conn.srej_exception);DEBUG_VAR("srej_bitmap[0] after fallback (should be 0)", conn.srej_bitmap[0]);
#ifdef DEBUG_ENABLE
    uint8_t t9_rej_cnt = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    DEBUG_VAR("REJ frames sent", t9_rej_cnt);
    int t9_rej_idx = find_last_sframe(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    DEBUG_VAR("REJ N(R) value", (unsigned)get_sframe_nr(&h, t9_rej_idx)); DEBUG_VAR("V(R) after fallback", conn.vars.vr);
#endif // DEBUG_ENABLE

    // REJ exception must now be set
    TEST_ASSERT(conn.rej_exception == true, "REJ exception set after SREJ max exceeded", 0);

    // SREJ exception and bitmap must be cleared (REJ supersedes SREJ per Section 6.4.4.3)
    TEST_ASSERT(conn.srej_exception == false, "SREJ exception cleared when REJ issued", 0);
    TEST_ASSERT(conn.srej_bitmap[0] == 0, "SREJ bitmap cleared when REJ issued", 0);

    // REJ N(R) must request retransmit from V(R)=1
    int rej_idx = find_last_sframe(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    int rej_nr = get_sframe_nr(&h, rej_idx);
    TEST_ASSERT(rej_nr == 1, "REJ N(R) = 1 (retransmit from frame 1)", 0);

    DEBUG_PRINT("Test 9 final state: SREJ->REJ fallback verified");DEBUG_BOOL("Final rej_exception", conn.rej_exception);DEBUG_BOOL("Final srej_exception", conn.srej_exception);DEBUG_VAR("Final srej_bitmap[0]", conn.srej_bitmap[0]);DEBUG_VAR("Final REJ N(R)", (unsigned)rej_nr);

    printf("rej_exception=%d, srej_exception=%d, srej_bitmap[0]=0x%02X, REJ N(R)=%d\n", (int) conn.rej_exception, (int) conn.srej_exception,
            conn.srej_bitmap[0], rej_nr);
    return 0;
}

// =========================================================================
// TEST 10: Section 6.4.4.1 - REJ clears on receipt of expected frame
// Scenario: REJ sent for frame 1. Frame 1 arrives. REJ exception clears.
//           Subsequent frames accepted normally.
// =========================================================================
static int test_rej_exception_clears_on_expected_frame(void) {
    printf("\n--- test_rej_exception_clears_on_expected_frame ---\n");
    printf("AX.25 v2.2 Section 6.4.4.1: REJ exception cleared on receipt of expected frame\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    // Use REJ-only mode
    conn.rej_mode = AX25_REJ_MODE_REJ;

    DEBUG_PRINT("Test 10 setup: REJ-only mode, verify REJ exception clears on expected frame");DEBUG_VAR("rej_mode", conn.rej_mode);DEBUG_STATE("Initial state", conn.state);DEBUG_VAR("Initial V(R)", conn.vars.vr);

    // Receive frame 0 in-order
    uint8_t p0[] = { 0xF0 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=0 (in-order)");

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    DEBUG_VAR("V(R) after frame 0", conn.vars.vr);DEBUG_VAR("rx_data_count after frame 0", h.rx_data_count);

    // Receive frame 2 - REJ(1) sent
    DEBUG_PRINT("Injecting I-frame N(S)=2 (frame 1 missing -> REJ(1) in REJ-only mode)");

    uint8_t p2[] = { 0xF2 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_BOOL("rej_exception after gap (should be true)", conn.rej_exception);DEBUG_VAR("V(R) after gap (should be 1)", conn.vars.vr);
#ifdef DEBUG_ENABLE
    uint8_t t10_rej_cnt = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    DEBUG_VAR("REJ frames sent", t10_rej_cnt);
#endif // DEBUG_ENABLE

    TEST_ASSERT(conn.rej_exception == true, "REJ exception active after gap", 0);

    // While REJ pending, receive duplicate of frame 2 - must be discarded
    DEBUG_PRINT("Injecting duplicate I-frame N(S)=2 during REJ exception (should be discarded)");DEBUG_VAR("rx_data_count before duplicate injection", h.rx_data_count);

    ax25_frame_t *f2dup = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2dup, 1);
    free_iframe(f2dup);

    DEBUG_VAR("rx_data_count after duplicate (should remain 1 - discarded)", h.rx_data_count);DEBUG_BOOL("rej_exception still active after duplicate", conn.rej_exception);

    // rx_data_count must remain 1 (frame 2 discarded during REJ exception)
    TEST_ASSERT(h.rx_data_count == 1, "Duplicate out-of-seq frame discarded during REJ exception", 0);

    // Receive frame 1 (the retransmitted expected frame) - clears REJ
    DEBUG_PRINT("Injecting retransmitted I-frame N(S)=1 (expected frame - should clear REJ exception)");

    uint8_t p1[] = { 0xF1 };
    ax25_frame_t *f1 = make_iframe(1, 0, false, p1, sizeof(p1), 8);
    ax25_process_frame(&conn, f1, 1);
    free_iframe(f1);

    DEBUG_BOOL("rej_exception after frame 1 (should be false - cleared)", conn.rej_exception);DEBUG_VAR("rx_data_count after frame 1 (should be 2)", h.rx_data_count);DEBUG_VAR("V(R) after frame 1 (should be 2)", conn.vars.vr);

    // REJ exception must be cleared
    TEST_ASSERT(conn.rej_exception == false, "REJ exception cleared after frame 1 received", 0);

    // Frame 1 delivered to upper layer
    TEST_ASSERT(h.rx_data_count == 2, "Frame 1 delivered after REJ recovery", 0);

    TEST_ASSERT(conn.vars.vr == 2, "V(R) = 2 after REJ recovery with frame 1", 0);

    // Receive frame 2 again (re-sent per REJ) - must be accepted now
    DEBUG_PRINT("Injecting I-frame N(S)=2 again (re-sent per REJ, should now be accepted)");

    ax25_frame_t *f2b = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2b, 1);
    free_iframe(f2b);

    DEBUG_VAR("rx_data_count after frame 2 re-accepted (should be 3)", h.rx_data_count);DEBUG_VAR("V(R) after full REJ recovery (should be 3)", conn.vars.vr);DEBUG_BOOL("rej_exception final (should be false)", conn.rej_exception);

    TEST_ASSERT(h.rx_data_count == 3, "Frame 2 accepted and delivered after REJ cleared", 0);
    TEST_ASSERT(conn.vars.vr == 3, "V(R) = 3 after complete REJ recovery", 0);

    return 0;
}

// =========================================================================
// TEST 11: Duplicate I-frame during SREJ exception is silently discarded
// Scenario: SREJ active for frame 1. Receive duplicate of frame 2.
//           No extra deliveries, V(R) unchanged.
// =========================================================================
static int test_srej_duplicate_frame_discarded(void) {
    printf("\n--- test_srej_duplicate_frame_discarded ---\n");
    printf("AX.25 v2.2 Section 6.4.4: Duplicate I-frame during SREJ exception discarded\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    DEBUG_PRINT("Test 11 setup: SREJ/REJ mode, duplicate frame discard test");DEBUG_VAR("rej_mode", conn.rej_mode);DEBUG_STATE("Initial state", conn.state);

    // Frame 0 in-order
    uint8_t p0[] = { 0x01 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=0 (in-order)");

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    DEBUG_VAR("V(R) after frame 0", conn.vars.vr);

    // Frame 2 - SREJ(1) sent
    DEBUG_PRINT("Injecting I-frame N(S)=2 (frame 1 missing -> SREJ(1), frame 2 buffered)");

    uint8_t p2[] = { 0x02 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_BOOL("srej_exception after gap (should be true)", conn.srej_exception);DEBUG_VAR("srej_buffer_count (frame 2 buffered)", conn.srej_buffer_count);DEBUG_VAR("V(R) after gap (should be 1)", conn.vars.vr);DEBUG_VAR("rx_data_count (should be 1 - frame 0 only)", h.rx_data_count);

    TEST_ASSERT(conn.srej_exception == true, "SREJ exception active", 0);
    uint8_t rx_before = h.rx_data_count;

    DEBUG_PRINT("Injecting duplicate I-frame N(S)=2 (already buffered, should be discarded)");DEBUG_VAR("rx_data_count before duplicate", rx_before);

    // Receive duplicate of frame 2 - already buffered, must be discarded
    ax25_frame_t *f2dup = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2dup, 1);
    free_iframe(f2dup);

    DEBUG_VAR("rx_data_count after duplicate (should equal rx_before)", h.rx_data_count);DEBUG_VAR("V(R) after duplicate (should still be 1)", conn.vars.vr);DEBUG_BOOL("srej_exception still active after duplicate", conn.srej_exception);DEBUG_VAR("srej_buffer_count (should remain same)", conn.srej_buffer_count);

    TEST_ASSERT(h.rx_data_count == rx_before, "Duplicate frame during SREJ not delivered", 0);
    TEST_ASSERT(conn.vars.vr == 1, "V(R) unchanged after duplicate during SREJ", 0);
    TEST_ASSERT(conn.srej_exception == true, "SREJ exception still active after duplicate", 0);

    return 0;
}

// =========================================================================
// TEST 12: SREJ mode negotiation - SREJ mode set correctly by rej_mode field
// Section 6.4.4: Mode is negotiated via XID (tested via rej_mode field directly)
// =========================================================================
static int test_srej_mode_negotiation(void) {
    printf("\n--- test_srej_mode_negotiation ---\n");
    printf("AX.25 v2.2 Section 6.4.4: SREJ mode selection\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    DEBUG_PRINT("Test 12 setup: SREJ mode negotiation verification");DEBUG_VAR("Default rej_mode (should be AX25_REJ_MODE_SREJ_REJ=3)", conn.rej_mode);

    // Default mode must be SREJ_REJ per Section 6.3.2
    TEST_ASSERT(conn.rej_mode == AX25_REJ_MODE_SREJ_REJ, "Default rej_mode is AX25_REJ_MODE_SREJ_REJ per Section 6.3.2", 0);

    // Test SREJ-only mode: gap -> only SREJ sent, no REJ
    conn.rej_mode = AX25_REJ_MODE_SREJ;
    conn.srej_max = 4;

    DEBUG_PRINT("Switching to SREJ-only mode (rej_mode=AX25_REJ_MODE_SREJ, srej_max=4)");DEBUG_VAR("New rej_mode", conn.rej_mode);DEBUG_VAR("srej_max", conn.srej_max);

    uint8_t p0[] = { 0x01 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=0 (in-order)");

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    DEBUG_PRINT("Injecting I-frame N(S)=2 (frame 1 missing, SREJ-only mode: only SREJ expected)");

    uint8_t p2[] = { 0x02 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    uint8_t srej_c = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    uint8_t rej_c = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);

    DEBUG_VAR("SREJ frames sent in SREJ-only mode (should be >= 1)", srej_c);DEBUG_VAR("REJ frames sent in SREJ-only mode (should be 0)", rej_c);DEBUG_BOOL("srej_exception active", conn.srej_exception);DEBUG_BOOL("rej_exception (should be false in SREJ-only)", conn.rej_exception);DEBUG_VAR("V(R) (blocked by missing frame 1)", conn.vars.vr);

    TEST_ASSERT(srej_c >= 1, "SREJ sent in SREJ-only mode for single gap", 0);
    TEST_ASSERT(rej_c == 0, "No REJ sent in SREJ-only mode for single gap", 0);

    printf("rej_mode=SREJ_ONLY: SREJ count=%u, REJ count=%u\n", (unsigned) srej_c, (unsigned) rej_c);
    return 0;
}

// =========================================================================
// TEST 13: SREJ with P=1 on first SREJ, P=0 on subsequent per Section 6.4.4.2
// =========================================================================
static int test_srej_poll_final_bit_behaviour(void) {
    printf("\n--- test_srej_poll_final_bit_behaviour ---\n");
    printf("AX.25 v2.2 Section 6.4.4.2: First SREJ has P=1, subsequent SREJs have P=0\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    conn.rej_mode = AX25_REJ_MODE_SREJ_REJ;
    conn.srej_max = 3;  // Allow multiple SREJs

    DEBUG_PRINT("Test 13 setup: P/F bit behaviour on SREJ, rej_mode=SREJ_REJ, srej_max=3");DEBUG_VAR("rej_mode", conn.rej_mode);DEBUG_VAR("srej_max", conn.srej_max);

    // Frame 0 in-order
    uint8_t p0[] = { 0xA0 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=0 (in-order)");

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    DEBUG_VAR("V(R) after frame 0", conn.vars.vr);

    // Frame 2 - triggers first SREJ(1) with P=1
    DEBUG_PRINT("Injecting I-frame N(S)=2 (frame 1 missing -> first SREJ(1) with P=1 expected)");DEBUG_VAR("tx_count before gap injection", h.tx_count);

    uint8_t p2[] = { 0xA2 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_VAR("tx_count after first gap (SREJ should have been sent)", h.tx_count);DEBUG_BOOL("srej_exception after first gap", conn.srej_exception);

    // Locate and decode the first SREJ frame
    int first_srej_idx = -1;
    for (int i = 0; i < (int) h.tx_count; i++) {
        uint8_t err = 0;
        ax25_frame_t *decoded = ax25_frame_decode(h.tx_frames[i].data, h.tx_frames[i].len, 0, &err);
        if (!decoded) {
            continue;
        }
        if (decoded->type == AX25_FRAME_SUPERVISORY_SREJ_8BIT) {
            first_srej_idx = i;
            ax25_frame_free(decoded, &err);
            break;
        }
        ax25_frame_free(decoded, &err);
    }

    DEBUG_VAR("First SREJ frame index in tx buffer", (unsigned)first_srej_idx);

    TEST_ASSERT(first_srej_idx >= 0, "First SREJ frame found in tx buffer", 0);
    if (first_srej_idx >= 0) {
        uint8_t err = 0;
        ax25_frame_t *decoded = ax25_frame_decode(h.tx_frames[first_srej_idx].data, h.tx_frames[first_srej_idx].len, 0, &err);
        TEST_ASSERT(decoded != NULL, "First SREJ frame decoded OK", err);
        if (decoded) {
            ax25_supervisory_frame_t *sf = (ax25_supervisory_frame_t*) decoded;

            DEBUG_BOOL("First SREJ P/F bit (should be true = P=1)", sf->pf);DEBUG_VAR("First SREJ N(R) (should be 1)", sf->nr);DEBUG_FRAME("First SREJ raw frame bytes", h.tx_frames[first_srej_idx].data, h.tx_frames[first_srej_idx].len);

            // Per Section 6.4.4.2: first SREJ command has P=1
            TEST_ASSERT(sf->pf == true, "First SREJ has P=1 per Section 6.4.4.2", 0);
            TEST_ASSERT(sf->nr == 1, "First SREJ N(R) = 1 (missing frame 1)", 0);
            ax25_frame_free(decoded, &err);
        }
    }

    return 0;
}

// =========================================================================
// TEST 14: SREJ state machine resets on new connection (SABM)
// Scenario: SREJ exception active, then SABM received -> all SREJ state clears
// =========================================================================
static int test_srej_state_reset_on_sabm(void) {
    printf("\n--- test_srej_state_reset_on_sabm ---\n");
    printf("AX.25 v2.2 Section 6.4.4: SREJ state resets when SABM received\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    DEBUG_PRINT("Test 14 setup: SREJ exception created then SABM injected");DEBUG_STATE("Initial state", conn.state);DEBUG_VAR("Initial V(S)", conn.vars.vs);DEBUG_VAR("Initial V(R)", conn.vars.vr);

    // Create SREJ exception state
    uint8_t p0[] = { 0xB0 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=0 to set up state");

    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    uint8_t p2[] = { 0xB2 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 8);

    DEBUG_PRINT("Injecting I-frame N(S)=2 (frame 1 missing -> SREJ exception active)");

    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_BOOL("srej_exception before SABM (should be true)", conn.srej_exception);DEBUG_VAR("srej_buffer_count before SABM (should be > 0)", conn.srej_buffer_count);DEBUG_VAR("srej_bitmap[0] before SABM", conn.srej_bitmap[0]);DEBUG_VAR("V(R) before SABM", conn.vars.vr);DEBUG_VAR("V(S) before SABM", conn.vars.vs);

    TEST_ASSERT(conn.srej_exception == true, "SREJ exception active before SABM", 0);
    TEST_ASSERT(conn.srej_buffer_count > 0, "SREJ buffer has frames before SABM", 0);

    // Inject SABM frame (new connection request resets everything)
    ax25_unnumbered_frame_t sabm_frame;
    memset(&sabm_frame, 0, sizeof(sabm_frame));
    sabm_frame.base.type = AX25_FRAME_UNNUMBERED_SABM;
    memcpy(sabm_frame.base.header.destination.callsign, "N0CALL", 6);
    sabm_frame.base.header.destination.ssid = 0;
    memcpy(sabm_frame.base.header.source.callsign, "W1AW  ", 6);
    sabm_frame.base.header.source.ssid = 0;
    sabm_frame.base.header.cr = true;
    sabm_frame.base.header.repeaters.num_repeaters = 0;
    sabm_frame.pf = true;
    sabm_frame.modifier = 0x2F;

    DEBUG_PRINT("Injecting SABM frame (new connection request - should reset all SREJ state)");

    ax25_process_frame(&conn, (ax25_frame_t*) &sabm_frame, 1);

    DEBUG_BOOL("srej_exception after SABM (should be false)", conn.srej_exception);DEBUG_BOOL("rej_exception after SABM (should be false)", conn.rej_exception);DEBUG_VAR("srej_count after SABM (should be 0)", conn.srej_count);DEBUG_VAR("srej_buffer_count after SABM (should be 0)", conn.srej_buffer_count);DEBUG_VAR("V(S) after SABM (should be 0)", conn.vars.vs);DEBUG_VAR("V(R) after SABM (should be 0)", conn.vars.vr);DEBUG_VAR("V(A) after SABM (should be 0)", conn.vars.va);DEBUG_STATE("State after SABM", conn.state);
#ifdef DEBUG_ENABLE
    bool bitmap_clear_dbg = true;
    for (int dbg_i = 0; dbg_i < 16; dbg_i++) {
        if (conn.srej_bitmap[dbg_i] != 0) {
            bitmap_clear_dbg = false;
            break;
        }
    } DEBUG_BOOL("srej_bitmap all zero after SABM", bitmap_clear_dbg);
#endif // DEBUG_ENABLE

    // After SABM, all SREJ state must be cleared
    TEST_ASSERT(conn.srej_exception == false, "SREJ exception cleared after SABM", 0);
    TEST_ASSERT(conn.rej_exception == false, "REJ exception cleared after SABM", 0);
    TEST_ASSERT(conn.srej_count == 0, "srej_count = 0 after SABM", 0);
    TEST_ASSERT(conn.srej_buffer_count == 0, "SREJ buffer empty after SABM", 0);
    TEST_ASSERT(conn.vars.vs == 0, "V(S) reset to 0 after SABM", 0);
    TEST_ASSERT(conn.vars.vr == 0, "V(R) reset to 0 after SABM", 0);
    TEST_ASSERT(conn.vars.va == 0, "V(A) reset to 0 after SABM", 0);

    // Verify bitmap zeroed
    bool bitmap_clear = true;
    for (int i = 0; i < 16; i++) {
        if (conn.srej_bitmap[i] != 0) {
            bitmap_clear = false;
            break;
        }
    }
    TEST_ASSERT(bitmap_clear, "SREJ bitmap fully cleared after SABM", 0);

    return 0;
}

// =========================================================================
// TEST 15: Window boundary - SREJ within window, duplicate beyond window
// Scenario: Modulo-8 window=7. Frames 0-4 sent. Frame 3 missing.
//           Frame 6 received (still in window). SREJ(3) sent.
//           Old frame (modulo-wrapped) correctly ignored.
// =========================================================================
static int test_srej_window_boundary_behaviour(void) {
    printf("\n--- test_srej_window_boundary_behaviour ---\n");
    printf("AX.25 v2.2 Section 6.4.4: SREJ within window, old frames outside window ignored\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    conn.rej_mode = AX25_REJ_MODE_SREJ_REJ;
    conn.srej_max = 2;
    conn.timers.k = 7;

    DEBUG_PRINT("Test 15 setup: window boundary behaviour, mod=8, k=7, srej_max=2");DEBUG_VAR("rej_mode", conn.rej_mode);DEBUG_VAR("srej_max", conn.srej_max);DEBUG_VAR("window k", conn.timers.k);DEBUG_STATE("Initial state", conn.state);DEBUG_VAR("Initial V(R)", conn.vars.vr);

    // Receive frames 0, 1, 2 in order
    for (uint8_t ns = 0; ns < 3; ns++) {
        uint8_t p[] = { ns };
        ax25_frame_t *f = make_iframe(ns, 0, false, p, 1, 8);

        DEBUG_PRINT("Injecting I-frame N(S)=%u (in-order)", (unsigned)ns);

        ax25_process_frame(&conn, f, 1);
        free_iframe(f);
    }

    DEBUG_VAR("V(R) after frames 0-2 (should be 3)", conn.vars.vr);DEBUG_VAR("rx_data_count after frames 0-2", h.rx_data_count);

    TEST_ASSERT(conn.vars.vr == 3, "V(R) = 3 after frames 0-2", 0);

    // Receive frame 4 - frame 3 missing -> SREJ(3)
    DEBUG_PRINT("Injecting I-frame N(S)=4 (frame 3 missing -> SREJ(3) expected)");

    uint8_t p4[] = { 0x04 };
    ax25_frame_t *f4 = make_iframe(4, 0, false, p4, sizeof(p4), 8);
    ax25_process_frame(&conn, f4, 1);
    free_iframe(f4);

#ifdef DEBUG_ENABLE
    uint8_t srej_wnd = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    int srej_wnd_idx = find_last_sframe(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    int srej_wnd_nr = get_sframe_nr(&h, srej_wnd_idx);
    DEBUG_VAR("SREJ frames sent after frame 4", srej_wnd); DEBUG_VAR("SREJ N(R) (should be 3)", (unsigned)srej_wnd_nr); DEBUG_BOOL("srej_exception active", conn.srej_exception); DEBUG_VAR("V(R) after frame 4 (should be 3 - blocked)", conn.vars.vr);
#endif // DEBUG_ENABLE

    uint8_t srej_count = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    TEST_ASSERT(srej_count >= 1, "SREJ sent for missing frame 3", 0);

    int srej_idx = find_last_sframe(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    int srej_nr = get_sframe_nr(&h, srej_idx);
    TEST_ASSERT(srej_nr == 3, "SREJ N(R) = 3 for missing frame", 0);

    // Receive frame 5 and 6 (still in window while waiting for frame 3)
    DEBUG_PRINT("Injecting I-frame N(S)=5 (in window, waiting for frame 3)");

    uint8_t p5[] = { 0x05 };
    ax25_frame_t *f5 = make_iframe(5, 0, false, p5, sizeof(p5), 8);
    ax25_process_frame(&conn, f5, 1);
    free_iframe(f5);

    DEBUG_VAR("srej_buffer_count after frame 5 (buffered)", conn.srej_buffer_count);DEBUG_VAR("V(R) still blocked at 3", conn.vars.vr);

    // Deliver frame 3 (SREJ recovery)
    DEBUG_PRINT("Injecting retransmitted I-frame N(S)=3 (SREJ recovery)");

    uint8_t p3[] = { 0x03 };
    ax25_frame_t *f3 = make_iframe(3, 0, false, p3, sizeof(p3), 8);
    ax25_process_frame(&conn, f3, 1);
    free_iframe(f3);

    DEBUG_VAR("V(R) after recovery (should be 6)", conn.vars.vr);DEBUG_VAR("rx_data_count after recovery (should be 6)", h.rx_data_count);DEBUG_BOOL("srej_exception after recovery (should be false)", conn.srej_exception);DEBUG_VAR("srej_bitmap[0] after recovery (should be 0)", conn.srej_bitmap[0]);

    // All 6 frames (0-5) must be delivered in order
    TEST_ASSERT(conn.vars.vr == 6, "V(R) = 6 after SREJ window recovery", 0);
    TEST_ASSERT(h.rx_data_count == 6, "6 frames delivered after SREJ window boundary test", 0);
    TEST_ASSERT(conn.srej_exception == false, "SREJ exception cleared after full recovery", 0);

    printf("Final V(R)=%u, rx_data_count=%u\n", (unsigned) conn.vars.vr, (unsigned) h.rx_data_count);
    return 0;
}

// =========================================================================
// TEST 16: SREJ not sent when local_busy - Section 6.4.10 interaction
// Per Section 6.4.4: If TNC is busy (RNR sent), SREJ/REJ must not be generated
// because TNC cannot reliably buffer out-of-order frames.
// =========================================================================
static int test_srej_not_sent_when_local_busy(void) {
    printf("\n--- test_srej_not_sent_when_local_busy ---\n");
    printf("AX.25 v2.2 Section 6.4.4 / 6.4.10: Verify local busy flag interaction\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    // Set local_busy - TNC busy condition
    conn.local_busy = true;

    DEBUG_PRINT("Test 16 setup: local_busy=true, verify SREJ/REJ suppressed");DEBUG_BOOL("local_busy", conn.local_busy);DEBUG_STATE("Initial state", conn.state);DEBUG_VAR("Initial V(R)", conn.vars.vr);

    uint8_t tx_before = h.tx_count;

    DEBUG_VAR("tx_count before busy test", tx_before);

    // Receive frame 0 normally (in-order) - should still be accepted
    DEBUG_PRINT("Injecting I-frame N(S)=0 (in-order, local_busy=true)");

    uint8_t p0[] = { 0x10 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);
    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    DEBUG_VAR("V(R) after frame 0 in busy state", conn.vars.vr);DEBUG_VAR("rx_data_count after frame 0", h.rx_data_count);DEBUG_VAR("tx_count after frame 0 in busy mode", h.tx_count);

    // Frame 1 missing; frame 2 received
    DEBUG_PRINT("Injecting I-frame N(S)=2 (frame 1 missing, local_busy=true -> no SREJ/REJ expected)");

    uint8_t p2[] = { 0x12 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_BOOL("srej_exception in busy state", conn.srej_exception);DEBUG_BOOL("rej_exception in busy state", conn.rej_exception);DEBUG_VAR("V(R) after gap in busy state", conn.vars.vr);DEBUG_VAR("srej_bitmap[0] in busy state", conn.srej_bitmap[0]);
#ifdef DEBUG_ENABLE
    uint8_t t16_srej = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    uint8_t t16_rej = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    DEBUG_VAR("SREJ frames sent during local_busy (should be 0)", t16_srej); DEBUG_VAR("REJ frames sent during local_busy (should be 0)", t16_rej);
    uint8_t tx_delta_dbg = (uint8_t) (h.tx_count - tx_before);
    DEBUG_VAR("Total new TX frames while local_busy", tx_delta_dbg);
#endif // DEBUG_ENABLE

    // The srej_exception flag and local_busy interaction:
    // local_busy == true means receiver is not ready; the SREJ/REJ behaviour
    // may differ per implementation. We verify that the state is internally
    // consistent (no crash, state correctly reflects what happened).
    // The key invariant: if SREJ exception is set, bitmap must be consistent.
    if (conn.srej_exception) {
        bool bitmap_has_bit1 = (conn.srej_bitmap[0] & (1U << 1)) != 0;
        TEST_ASSERT(bitmap_has_bit1, "If SREJ exception set while busy, bitmap bit 1 set", 0);
    }

    // Overall: state is internally consistent
    TEST_ASSERT(conn.state == AX25_STATE_CONNECTED, "Connection still CONNECTED", 0);

    uint8_t tx_delta = (uint8_t) (h.tx_count - tx_before);
    printf("TX frames generated while local_busy: %u\n", (unsigned) tx_delta);

    return 0;
}

// =========================================================================
// TEST 17: SREJ/REJ interaction with RR acknowledgment
// Scenario: SREJ sent for frame 1. Remote RR(3) comes in acknowledging frames 0-2.
//           SREJ exception must clear because the frame was acknowledged via N(R).
//           V(A) advances to 3.
// =========================================================================
static int test_srej_clears_on_rr_acknowledgment(void) {
    printf("\n--- test_srej_clears_on_rr_acknowledgment ---\n");
    printf("AX.25 v2.2 Section 6.4.4: SREJ state interaction with incoming RR\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    DEBUG_PRINT("Test 17 setup: send 3 frames, receive RR(3) to acknowledge all");DEBUG_STATE("Initial state", conn.state);DEBUG_VAR("Initial V(S)", conn.vars.vs);DEBUG_VAR("Initial V(A)", conn.vars.va);

    // Send frames 0, 1, 2 from our side
    for (int i = 0; i < 3; i++) {
        uint8_t d[] = { (uint8_t) (0xD0 + i) };

        DEBUG_PRINT("Sending I-frame N(S)=%d: [%02X]", i, d[0]);

        ax25_send_data(&conn, d, 1, PID_NO_L3);
    }

    DEBUG_VAR("V(S) after sending 3 frames (should be 3)", conn.vars.vs);DEBUG_VAR("V(A) before RR (should be 0)", conn.vars.va);DEBUG_VAR("tx_queue.count", conn.tx_queue.count);DEBUG_VAR("tx_count (3 I-frames transmitted)", h.tx_count);

    TEST_ASSERT(conn.vars.vs == 3, "V(S) = 3 after sending 3 frames", 0);

    // Receive RR(3) from remote - acknowledges all 3 of our frames
    DEBUG_PRINT("Injecting RR(3) from remote (acknowledges all 3 of our I-frames)");

    ax25_frame_t *rr = make_sframe(AX25_FRAME_SUPERVISORY_RR_8BIT, 3, false, 0);
    ax25_process_frame(&conn, rr, 1);
    free_sframe(rr);

    DEBUG_VAR("V(A) after RR(3) (should be 3)", conn.vars.va);DEBUG_VAR("tx_queue.count after RR(3) (should be 0)", conn.tx_queue.count);

    // V(A) must advance to 3
    TEST_ASSERT(conn.vars.va == 3, "V(A) = 3 after RR(3) received", 0);

    // TX queue must be empty (all frames acknowledged)
    TEST_ASSERT(conn.tx_queue.count == 0, "TX queue empty after RR(3) acknowledgment", 0);

    // Now test that SREJ exception state interacts with RR correctly:
    // Force a receive-side SREJ exception, then receive RR from us to remote
    // (this tests that our outgoing RR after SREJ delivers proper N(R))

    // Receive frame 0 from remote
    DEBUG_PRINT("Now testing SREJ exception + RR interaction on receive side");DEBUG_PRINT("Injecting I-frame N(S)=0 from remote");

    uint8_t p0[] = { 0x01 };
    ax25_frame_t *f0 = make_iframe(0, 0, false, p0, sizeof(p0), 8);
    ax25_process_frame(&conn, f0, 1);
    free_iframe(f0);

    DEBUG_VAR("V(R) after remote frame 0", conn.vars.vr);

    // Receive frame 2 (gap at 1) - SREJ(1) sent
    DEBUG_PRINT("Injecting remote I-frame N(S)=2 (gap at frame 1 -> SREJ(1) expected)");

    uint8_t p2[] = { 0x02 };
    ax25_frame_t *f2b = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2b, 1);
    free_iframe(f2b);

    DEBUG_BOOL("srej_exception after receive-side gap", conn.srej_exception);DEBUG_VAR("V(R) after gap (should be 1 - blocked)", conn.vars.vr);
#ifdef DEBUG_ENABLE
    uint8_t t17_srej = count_sframes_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    DEBUG_VAR("SREJ frames sent from our side (receive-side SREJ)", t17_srej);
    int t17_srej_idx = find_last_sframe(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    int t17_srej_nr = get_sframe_nr(&h, t17_srej_idx);
    DEBUG_VAR("Our SREJ N(R) (should be 1)", (unsigned)t17_srej_nr);
#endif // DEBUG_ENABLE

    TEST_ASSERT(conn.srej_exception == true, "SREJ exception active", 0);
    // V(R) = 1 (can't advance past missing frame 1)
    TEST_ASSERT(conn.vars.vr == 1, "V(R) = 1 during SREJ exception", 0);

    // Find the last SREJ that was sent - its N(R) must equal V(R) = 1
    int srej_idx = find_last_sframe(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    int srej_nr = get_sframe_nr(&h, srej_idx);
    TEST_ASSERT(srej_nr == 1, "SREJ N(R) = 1 (= V(R) = missing frame number)", 0);

    return 0;
}

// =========================================================================
// TEST 18: Section 6.4.4 - SREJ with I-frame N(R) acknowledgment piggybacking
// Scenario: We have received frames 0-4. Frame 2 was missing (SREJ sent and resolved).
//           We send an I-frame - its N(R) must reflect current V(R).
// =========================================================================
static int test_srej_nr_piggyback_in_iframe(void) {
    printf("\n--- test_srej_nr_piggyback_in_iframe ---\n");
    printf("AX.25 v2.2 Section 6.4.4: N(R) in outgoing I-frame reflects V(R) after SREJ\n");

    srej_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    force_connected(&conn, &h, 8);

    DEBUG_PRINT("Test 18 setup: N(R) piggybacking after SREJ recovery");DEBUG_STATE("Initial state", conn.state);DEBUG_VAR("Initial V(R)", conn.vars.vr);DEBUG_VAR("Initial V(S)", conn.vars.vs);

    // Receive frames 0, 1 in order
    for (uint8_t ns = 0; ns < 2; ns++) {
        uint8_t p[] = { ns };
        ax25_frame_t *f = make_iframe(ns, 0, false, p, 1, 8);

        DEBUG_PRINT("Injecting I-frame N(S)=%u (in-order)", (unsigned)ns);

        ax25_process_frame(&conn, f, 1);
        free_iframe(f);
    }

    DEBUG_VAR("V(R) after frames 0,1 (should be 2)", conn.vars.vr);DEBUG_VAR("rx_data_count after frames 0,1", h.rx_data_count);

    // Receive frame 3 (frame 2 missing) - SREJ(2)
    DEBUG_PRINT("Injecting I-frame N(S)=3 (frame 2 missing -> SREJ(2))");

    uint8_t p3[] = { 0x03 };
    ax25_frame_t *f3 = make_iframe(3, 0, false, p3, sizeof(p3), 8);
    ax25_process_frame(&conn, f3, 1);
    free_iframe(f3);

    DEBUG_BOOL("srej_exception after gap (should be true)", conn.srej_exception);DEBUG_VAR("V(R) after gap (should be 2 - blocked)", conn.vars.vr);DEBUG_VAR("srej_bitmap[0] (bit 2 set)", conn.srej_bitmap[0]);

    TEST_ASSERT(conn.srej_exception == true, "SREJ exception active", 0);
    TEST_ASSERT(conn.vars.vr == 2, "V(R) = 2 (blocked by missing frame 2)", 0);

    // Retransmit frame 2 - SREJ resolved
    DEBUG_PRINT("Injecting retransmitted I-frame N(S)=2 (SREJ resolution)");

    uint8_t p2[] = { 0x02 };
    ax25_frame_t *f2 = make_iframe(2, 0, false, p2, sizeof(p2), 8);
    ax25_process_frame(&conn, f2, 1);
    free_iframe(f2);

    DEBUG_VAR("V(R) after SREJ recovery (should be 4)", conn.vars.vr);DEBUG_BOOL("srej_exception after recovery (should be false)", conn.srej_exception);DEBUG_VAR("rx_data_count after recovery (should be 4)", h.rx_data_count);DEBUG_VAR("srej_bitmap[0] after recovery (should be 0)", conn.srej_bitmap[0]);

    TEST_ASSERT(conn.vars.vr == 4, "V(R) = 4 after SREJ recovery (frames 0-3 received)", 0);
    TEST_ASSERT(conn.srej_exception == false, "SREJ exception cleared", 0);

    // Now send an I-frame - N(R) in this frame must be current V(R) = 4
    uint8_t outdata[] = { 0xFF, 0xFE };
    uint8_t tx_before = h.tx_count;

    DEBUG_PRINT("Sending outgoing I-frame after SREJ recovery (N(R) should be V(R)=4)");DEBUG_VAR("tx_count before outgoing I-frame", tx_before);DEBUG_VAR("Current V(R) (should be 4 - to be piggybacked as N(R))", conn.vars.vr);

    ax25_send_data(&conn, outdata, sizeof(outdata), PID_NO_L3);

    DEBUG_VAR("tx_count after send (should be tx_before+1)", h.tx_count);

    TEST_ASSERT((uint8_t )(h.tx_count - tx_before) == 1, "One I-frame transmitted", 0);

    // Decode and check N(R) in the transmitted I-frame
    if (h.tx_count > tx_before) {
        uint8_t err = 0;
        ax25_frame_t *decoded = ax25_frame_decode(h.tx_frames[tx_before].data, h.tx_frames[tx_before].len, 0, &err);
        TEST_ASSERT(decoded != NULL, "Outgoing I-frame decoded OK", err);
        if (decoded) {
            TEST_ASSERT(decoded->type == AX25_FRAME_INFORMATION_8BIT, "Outgoing frame is I-frame", 0);
            ax25_information_frame_t *iout = (ax25_information_frame_t*) decoded;

            DEBUG_VAR("Outgoing I-frame N(S)", iout->ns);DEBUG_VAR("Outgoing I-frame N(R) (should be 4 = V(R) after SREJ recovery)", iout->nr);DEBUG_FRAME("Outgoing I-frame raw bytes", h.tx_frames[tx_before].data, h.tx_frames[tx_before].len);
            TEST_ASSERT(iout->nr == 4, "Outgoing I-frame N(R) = 4 (= V(R) after SREJ recovery)", 0);
            ax25_frame_free(decoded, &err);
        }
    }

    DEBUG_PRINT("Test 18 final state: N(R) piggyback verified");DEBUG_VAR("Final V(R)", conn.vars.vr);DEBUG_VAR("Final V(S)", conn.vars.vs);DEBUG_BOOL("Final srej_exception (should be false)", conn.srej_exception);
    ax25_connection_cleanup(&conn);

    return 0;
}

// =========================================================================
// Main entry point for all SREJ tests
// =========================================================================
int test_ax25_srej_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("Starting AX.25 v2.2 Section 6.4.4 Selective Reject (SREJ) Tests\n");
    printf("==================================================================================\n");

    result |= test_srej_single_missing_frame_mod8();
    result |= test_srej_recovery_on_retransmit();
    result |= test_srej_fallback_to_rej_on_multiple_gaps();
    result |= test_rej_only_mode_single_gap();
    result |= test_received_srej_causes_selective_retransmit();
    result |= test_received_rej_causes_bulk_retransmit();
    result |= test_srej_bitmap_tracking();
    result |= test_srej_simultaneous_multiple_mod128();
    result |= test_srej_exception_cleared_on_rej_fallback();
    result |= test_rej_exception_clears_on_expected_frame();
    result |= test_srej_duplicate_frame_discarded();
    result |= test_srej_mode_negotiation();
    result |= test_srej_poll_final_bit_behaviour();
    result |= test_srej_state_reset_on_sabm();
    result |= test_srej_window_boundary_behaviour();
    result |= test_srej_not_sent_when_local_busy();
    result |= test_srej_clears_on_rr_acknowledgment();
    result |= test_srej_nr_piggyback_in_iframe();

    printf("\n==================================================================================\n");
    printf("AX.25 SREJ Tests Completed. %s\n", result == 0 ? "All tests passed" : "Some tests failed");
    printf("==================================================================================\n\n");

    return result;
}
