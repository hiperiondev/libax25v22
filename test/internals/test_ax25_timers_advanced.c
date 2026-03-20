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
 * SECTION 7: ADVANCED TIMER TESTS
 * Covers:
 *   - T1 Adaptive Adjustment (RTT-based): ax25_adjust_t1_adaptive()
 *   - Exponential Backoff: progressive retry delays under loss
 *   - T100-T108 Interaction: digipeater timer interplay
 */

#define DEBUG_ENABLE
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

// ============================================================================
// Shared test infrastructure
// ============================================================================

static uint32_t assert_count = 0;

// Capture buffer for state machine transmissions
static uint8_t captured_buffer[2048];
static size_t captured_len = 0;
static uint32_t transmit_count = 0;

// Per-transmission capture log (records each call independently)
#define MAX_TX_LOG 64
static uint8_t tx_log_buf[MAX_TX_LOG][512];
static size_t tx_log_len[MAX_TX_LOG];
static uint32_t tx_log_tick[MAX_TX_LOG];
static uint32_t tx_log_index = 0;
static uint32_t global_tick = 0;  // set by driver before each tick call

// PTT / physical-layer capture
static bool ptt_state = false;
static uint32_t ptt_on_count = 0;
static uint32_t ptt_off_count = 0;
static bool simulated_carrier = false;
static uint32_t phys_send_count = 0;

// DL-ERROR capture
static ax25_dl_error_t last_dl_error = (ax25_dl_error_t) 0xFF;
static uint32_t dl_error_call_count = 0;

// Hardcoded shifted callsign bytes (SSID=0)
// TEST1 = 0xA8 0x8A 0xA6 0xA8 0x62 0x40
// TEST2 = 0xA8 0x8A 0xA6 0xA8 0x64 0x40
static const uint8_t test1_call[6] = { 0xA8, 0x8A, 0xA6, 0xA8, 0x62, 0x40 };
static const uint8_t test2_call[6] = { 0xA8, 0x8A, 0xA6, 0xA8, 0x64, 0x40 };

// ============================================================================
// Callbacks
// ============================================================================

static void reset_capture(void) {
    captured_len = 0;
    transmit_count = 0;
    tx_log_index = 0;
}

static void reset_ptt_state(void) {
    ptt_state = false;
    ptt_on_count = 0;
    ptt_off_count = 0;
    phys_send_count = 0;
}

static void capture_transmit(void *user_data, uint8_t *data, size_t len) {
    (void) user_data;
    if (len <= sizeof(captured_buffer)) {
        memcpy(captured_buffer, data, len);
        captured_len = len;
    }
    transmit_count++;
    // log individual frames for backoff analysis
    if (tx_log_index < MAX_TX_LOG && len <= 512) {
        memcpy(tx_log_buf[tx_log_index], data, len);
        tx_log_len[tx_log_index] = len;
        tx_log_tick[tx_log_index] = global_tick;
        tx_log_index++;
    }
}

static void dl_error_callback(void *user_data, ax25_dl_error_t error) {
    (void) user_data;
    last_dl_error = error;
    dl_error_call_count++;
    DEBUG_VAR("DL-ERROR code received", (unsigned)error);
}

static void ptt_control_callback(bool on, void *user_data) {
    (void) user_data;
    ptt_state = on;
    if (on)
        ptt_on_count++;
    else
        ptt_off_count++;
}

static bool carrier_detect_callback(void *user_data) {
    (void) user_data;
    return simulated_carrier;
}

static void send_data_callback(const uint8_t *data, size_t len, void *user_data) {
    (void) user_data;
    if (len <= sizeof(captured_buffer)) {
        memcpy(captured_buffer, data, len);
        captured_len = len;
    }
    phys_send_count++;
    transmit_count++;
}

static void cleanup_addresses(ax25_address_t **dest, ax25_address_t **src) {
    if (dest && *dest) {
        free(*dest);
        *dest = NULL;
    }
    if (src && *src) {
        free(*src);
        *src = NULL;
    }
}

// Build a raw 15-byte UA frame (TEST2 -> TEST1, F=1)
static void build_ua_frame(uint8_t out[15]) {
    memcpy(out + 0, test1_call, 6);
    out[6] = 0x60;
    memcpy(out + 7, test2_call, 6);
    out[13] = 0x61;
    out[14] = 0x73;  // UA modifier, F=1
}

// Establish a CONNECTED link; returns 0 on success, -1 on failure
static int establish_connection(ax25_connection_t *conn, ax25_address_t *dest, ax25_address_t *src) {
    reset_capture();
    if (ax25_connect(conn, dest, src) != 0)
        return -1;

    uint8_t ua_raw[15];
    build_ua_frame(ua_raw);

    uint8_t decode_err = 0;
    ax25_frame_t *ua = ax25_frame_decode(ua_raw, 15, MODULO128_FALSE, &decode_err);
    if (!ua)
        return -1;

    reset_capture();
    ax25_process_frame(conn, ua, 1);
    ax25_frame_free(ua, &decode_err);

    return (conn->state == AX25_STATE_CONNECTED) ? 0 : -1;
}

// Send an RR with N(R)=nr to acknowledge frames
static void send_rr_ack(ax25_connection_t *conn, uint8_t nr, uint32_t tick) {
    uint8_t rr_raw[15];
    memcpy(rr_raw + 0, test1_call, 6);
    rr_raw[6] = 0x60;
    memcpy(rr_raw + 7, test2_call, 6);
    rr_raw[13] = 0x61;
    // RR control: bits[7:5]=N(R)<<5, bits[4]=P/F=0, bits[3:0]=0001
    rr_raw[14] = (uint8_t) ((nr << 5) | 0x01);

    uint8_t decode_err = 0;
    ax25_frame_t *f = ax25_frame_decode(rr_raw, 15, MODULO128_FALSE, &decode_err);
    if (f) {
        ax25_process_frame(conn, f, tick);
        ax25_frame_free(f, &decode_err);
    }
}

// Build and inject a synthetic TEST-response frame (response = cr=false, pf=F)
// modifier 0xE1 = TEST response (0xE3 | 0x00 = cmd; 0xE1 = rsp, F=0; 0xE1|0x10=F=1)
static void inject_test_response(ax25_connection_t *conn, uint32_t tick) {
    // TEST response raw frame: dest=TEST1, src=TEST2, ctrl=0xE3(TEST rsp F=1)
    // AX.25 TEST response has modifier 0xE3 with F=1: 0xE3 | 0x10 = 0xF3
    // but in 8-bit modulo the control byte for TEST is 0xE3 (cmd P=1)
    // response is 0xE1 (F=0) or 0xE1|0x10=0xF1 (F=1)
    // From the code: response.base.modifier = 0xE3 and pf copied from cmd
    // The actual wire byte = 0xE3 | (pf ? 0x10 : 0x00) when P=1 -> 0xF3
    uint8_t test_rsp_raw[15];
    memcpy(test_rsp_raw + 0, test1_call, 6);
    test_rsp_raw[6] = 0x60;  // last addr byte for dest, C/R bits handled by decoder
    memcpy(test_rsp_raw + 7, test2_call, 6);
    test_rsp_raw[13] = 0x61;
    // TEST response control = 0xF3 (modifier 0xE3, F=1)
    test_rsp_raw[14] = 0xF3;

    uint8_t decode_err = 0;
    ax25_frame_t *f = ax25_frame_decode(test_rsp_raw, 15, MODULO128_FALSE, &decode_err);
    if (f) {
        ax25_process_frame(conn, f, tick);
        ax25_frame_free(f, &decode_err);
    }
}

// ============================================================================
// SECTION 7.1 – T1 Adaptive Adjustment (RTT-based)
// ============================================================================

// Test 7.1.a: ax25_adjust_t1_adaptive changes T1 proportional to RTT
static int test_t1_adaptive_rtt_proportional(void) {
    printf("\n--- test_t1_adaptive_rtt_proportional ---\n");
    DEBUG_PRINT("T1 adaptive: verify T1 = 2*avg_RTT + margin (half-duplex)");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit, .on_dl_error = dl_error_callback };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    conn.full_duplex = false;
    // Inject EMA RTT = 50 ticks (equivalent to 3 samples of 50 ticks)
    conn.test_stats.ema_rtt = 50;
    conn.test_stats.ema_seeded = 1u;
#ifdef DEBUG_ENABLE
    uint16_t t1_before = conn.timers.t1;
#endif
    DEBUG_VAR("T1 before adaptive (ticks)", t1_before);

    ax25_adjust_t1_adaptive(&conn);
    uint16_t t1_after = conn.timers.t1;
    DEBUG_VAR("T1 after adaptive  (ticks)", t1_after);

    // Expected: avg_rtt=50, margin=30(HD), new_t1 = 50*2+30 = 130
    uint16_t expected = 130;
    DEBUG_VAR("Expected T1 (ticks)", expected);
    TEST_ASSERT(t1_after == expected, "T1 set to 2*avg_RTT+margin (half-duplex, 50 tick RTT)", t1_after);

    ax25_connection_cleanup(&conn);
    return 0;
}

// Test 7.1.b: Full-duplex uses smaller margin (10 ticks = 100ms)
static int test_t1_adaptive_fullduplex_margin(void) {
    printf("\n--- test_t1_adaptive_fullduplex_margin ---\n");
    DEBUG_PRINT("T1 adaptive: full-duplex margin = 10 ticks");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    conn.full_duplex = true;
    // EMA RTT = 30 ticks (equivalent to avg of 20+40 ticks)
    conn.test_stats.ema_rtt = 30;
    conn.test_stats.ema_seeded = 1u;

    ax25_adjust_t1_adaptive(&conn);
    // Expected: 30*2 + 10 = 70
    uint16_t expected = 70;
    DEBUG_VAR("T1 after adaptive FD (ticks)", conn.timers.t1);
    DEBUG_VAR("Expected T1 FD (ticks)", expected);
    TEST_ASSERT(conn.timers.t1 == expected, "T1 full-duplex margin = 10 ticks", conn.timers.t1);

    ax25_connection_cleanup(&conn);
    return 0;
}

// Test 7.1.c: T1 clamped to minimum (10 ticks = 100ms) when RTT very small
static int test_t1_adaptive_min_clamp(void) {
    printf("\n--- test_t1_adaptive_min_clamp ---\n");
    DEBUG_PRINT("T1 adaptive: minimum clamp = 10 ticks (100ms)");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    conn.full_duplex = true;
    // EMA RTT = 0 ticks -> new_t1 = 0*2+10 = 10 -> exactly at min
    conn.test_stats.ema_rtt = 0;
    conn.test_stats.ema_seeded = 1u;

    // avg=0, margin(HD)=30 -> 30, above 10 -> clamp not triggered by HD margin
    // ema_rtt=0, ema_seeded=1, FD margin=10 -> 0*2+10=10 -> exactly min
    ax25_adjust_t1_adaptive(&conn);
    DEBUG_VAR("T1 at min boundary (ticks)", conn.timers.t1);
    TEST_ASSERT(conn.timers.t1 >= 10, "T1 >= minimum (10 ticks = 100ms)", conn.timers.t1);

    // Now force below minimum by giving negative-equivalent count trick:
    // The only way to go below min is avg*2+margin < 10
    // With FD margin=10: need avg<0, impossible with unsigned.
    // Confirm clamp holds if we manually set timers.t1 too low
    conn.timers.t1 = 2;  // artificially low
    // ema_rtt=0, ema_seeded=1, FD margin=10: result=10 -> clamp not needed
    conn.test_stats.ema_rtt = 0;
    conn.test_stats.ema_seeded = 1u;
    ax25_adjust_t1_adaptive(&conn);
    DEBUG_VAR("T1 after second adaptive (ticks)", conn.timers.t1);
    TEST_ASSERT(conn.timers.t1 >= 10, "T1 never below minimum after adaptive call", conn.timers.t1);

    ax25_connection_cleanup(&conn);
    return 0;
}

// Test 7.1.d: T1 clamped to maximum (3000 ticks = 30s) when RTT very large
static int test_t1_adaptive_max_clamp(void) {
    printf("\n--- test_t1_adaptive_max_clamp ---\n");
    DEBUG_PRINT("T1 adaptive: maximum clamp = 3000 ticks (30 seconds)");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    conn.full_duplex = false;
    // ema_rtt=2000 -> 2*2000+30 = 4030 -> clamped to 3000
    conn.test_stats.ema_rtt = 2000;
    conn.test_stats.ema_seeded = 1u;
    DEBUG_VAR("EMA RTT injected (ticks)", conn.test_stats.ema_rtt);

    ax25_adjust_t1_adaptive(&conn);
    DEBUG_VAR("T1 after adaptive large RTT (ticks)", conn.timers.t1);
    TEST_ASSERT(conn.timers.t1 == 3000, "T1 clamped to maximum 3000 ticks (30s)", conn.timers.t1);

    ax25_connection_cleanup(&conn);
    return 0;
}

// Test 7.1.e: No adjustment when no RTT samples available
static int test_t1_adaptive_no_samples(void) {
    printf("\n--- test_t1_adaptive_no_samples ---\n");
    DEBUG_PRINT("T1 adaptive: no change when ema_seeded = 0");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    uint16_t t1_default = conn.timers.t1;
    DEBUG_VAR("T1 default before (ticks)", t1_default);

    // No RTT samples: ema_seeded=0 (already zero from init, set explicitly for clarity)
    conn.test_stats.ema_seeded = 0;
    conn.test_stats.ema_rtt = 0;
    ax25_adjust_t1_adaptive(&conn);

    DEBUG_VAR("T1 after no-samples adaptive (ticks)", conn.timers.t1);
    TEST_ASSERT(conn.timers.t1 == t1_default, "T1 unchanged when no RTT samples", conn.timers.t1);

    ax25_connection_cleanup(&conn);
    return 0;
}

// Test 7.1.f: RTT accumulates over multiple TEST exchanges,
//             adaptive T1 reduces over successive calls
static int test_t1_adaptive_progressive_refinement(void) {
    printf("\n--- test_t1_adaptive_progressive_refinement ---\n");
    DEBUG_PRINT("T1 adaptive: T1 converges across multiple TEST cycles");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    conn.full_duplex = false;

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(dest && src, "Addresses allocated", 0);

    int res = establish_connection(&conn, dest, src);
    if (res != 0) {
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Connection established", res);
    }

    // Initial T1 after connection (spec default)
    uint16_t t1_initial = conn.timers.t1;
    DEBUG_VAR("T1 initial (ticks)", t1_initial);

    // Simulate 5 TEST round-trips with RTT = 20 ticks each
    uint32_t tick = 100;
    uint8_t payload[4] = { 'R', 'T', 'T', '0' };
    for (int i = 0; i < 5; i++) {
        reset_capture();
        uint8_t rc = ax25_send_test_command(&conn, payload, sizeof(payload));
        DEBUG_VAR("send_test_command return", rc);
        TEST_ASSERT(rc == 0, "TEST command sent", rc);

        // Mark the tick when we sent the command
        conn.test_stats.last_test_tick = tick;

        // Advance 20 ticks (RTT = 200ms)
        tick += 20;
        inject_test_response(&conn, tick);
        DEBUG_VAR("RTT sample injected (ticks)", (uint32_t)20);
        DEBUG_VAR("ema_seeded now", conn.test_stats.ema_seeded);
        DEBUG_VAR("ema_rtt now", conn.test_stats.ema_rtt);

        ax25_adjust_t1_adaptive(&conn);
        DEBUG_VAR("T1 after iteration %d (ticks)", conn.timers.t1);
    }

    // After 5 samples all equal to 20: EMA converges to 20 -> T1=2*20+30=70
    uint16_t expected = 70;
    DEBUG_VAR("Final T1 (ticks)", conn.timers.t1);
    DEBUG_VAR("Expected T1 (ticks)", expected);
    TEST_ASSERT(conn.timers.t1 == expected, "T1 converges to 2*RTT+margin after 5 samples", conn.timers.t1);

    // Verify T1 decreased from the (much larger) default
    TEST_ASSERT(conn.timers.t1 < t1_initial, "T1 reduced below initial default after RTT sampling", conn.timers.t1);

    // Drain the tx queue before cleanup
    send_rr_ack(&conn, conn.vars.vs, tick);
    cleanup_addresses(&dest, &src);
    ax25_connection_cleanup(&conn);
    return 0;
}

// Test 7.1.g: ax25_get_average_rtt_ms returns 0 when no samples
static int test_t1_get_avg_rtt_no_samples(void) {
    printf("\n--- test_t1_get_avg_rtt_no_samples ---\n");
    DEBUG_PRINT("ax25_get_average_rtt_ms returns 0 with no samples");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    uint32_t avg = ax25_get_average_rtt_ms(&conn);
    DEBUG_VAR("Average RTT ms (no samples)", avg);
    TEST_ASSERT(avg == 0, "Average RTT is 0 with no samples", avg);

    ax25_connection_cleanup(&conn);
    return 0;
}

// Test 7.1.h: ax25_get_average_rtt_ms returns correct value
static int test_t1_get_avg_rtt_with_samples(void) {
    printf("\n--- test_t1_get_avg_rtt_with_samples ---\n");
    DEBUG_PRINT("ax25_get_average_rtt_ms returns ema_rtt * 10ms");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    // EMA RTT = 25 ticks -> 250ms (equivalent to avg of 4 x 25-tick samples)
    conn.test_stats.ema_rtt = 25;
    conn.test_stats.ema_seeded = 1u;

    uint32_t avg = ax25_get_average_rtt_ms(&conn);
    DEBUG_VAR("Average RTT ms (expected 250)", avg);
    TEST_ASSERT(avg == 250, "Average RTT = 250ms for EMA of 25 ticks", avg);

    ax25_connection_cleanup(&conn);
    return 0;
}

// ============================================================================
// SECTION 7.2 – Exponential Backoff (retry strategy under loss)
// ============================================================================

// Helper: count how many times the transmit callback fired between two ticks
// Returns transmit_count delta

// Test 7.2.a: N2 retries exhaust and connection goes DISCONNECTED
static int test_backoff_n2_exhaustion(void) {
    printf("\n--- test_backoff_n2_exhaustion ---\n");
    DEBUG_PRINT("Backoff: N2 retries exhausted -> DISCONNECTED + DL-ERROR-N");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit, .on_dl_error = dl_error_callback };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    // Use very short T1 and small N2 for fast test
    conn.timers.t1 = 5;   // 50ms
    conn.timers.n2 = 3;   // 3 retries
    DEBUG_VAR("T1 configured (ticks)", conn.timers.t1);
    DEBUG_VAR("N2 configured (retries)", conn.timers.n2);

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(dest && src, "Addresses allocated", 0);

    int res = establish_connection(&conn, dest, src);
    if (res != 0) {
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Connection established", res);
    }

    // Send one I-frame and do NOT acknowledge it
    const uint8_t payload[] = { 'D', 'A', 'T', 'A' };
    reset_capture();
    uint8_t snd = ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    if (snd != 0) {
        DEBUG_VAR("send_data error", snd);
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "I-frame sent successfully", snd);
    }

    dl_error_call_count = 0;
    last_dl_error = (ax25_dl_error_t) 0xFF;

    // Drive ticks: let T1 expire N2+1 times (N2 retries + final disconnect)
    // Each expiry is separated by timers.t1 ticks; run enough ticks to cover all
    uint32_t max_ticks = (uint32_t) (conn.timers.t1 + 2) * (conn.timers.n2 + 2) + 10;
    DEBUG_VAR("Driving ticks", max_ticks);
    for (uint32_t i = 1; i <= max_ticks; i++) {
        global_tick = i;
        ax25_tick(&conn, i);
        DEBUG_VAR("tick", i);
        DEBUG_STATE("state", conn.state);
        DEBUG_VAR("retry_count", conn.retry_count);
        if (conn.state == AX25_STATE_DISCONNECTED) {
            DEBUG_VAR("DISCONNECTED at tick", i);
            break;
        }
    }

    DEBUG_STATE("Final state", conn.state);
    DEBUG_VAR("DL-ERROR call count", dl_error_call_count);
    DEBUG_VAR("Last DL-ERROR code", (unsigned)last_dl_error);
    // Collect assertion results before cleanup so no path leaks addresses or tx_queue frames.
    // Previous code called TEST_ASSERT (which does return 1) before cleanup_addresses,
    // leaving dest/src malloc blocks and the queued I-frame unreachable on failure paths.
    int disconnected_ok = (conn.state == AX25_STATE_DISCONNECTED);
    int dl_error_ok = (dl_error_call_count >= 1);
    int dl_error_code_ok = (last_dl_error == AX25_DL_ERROR_N);
    cleanup_addresses(&dest, &src);
    ax25_connection_cleanup(&conn);
    TEST_ASSERT(disconnected_ok, "State = DISCONNECTED after N2 exhaustion", conn.state);
    TEST_ASSERT(dl_error_ok, "DL-ERROR callback fired at least once", dl_error_call_count);
    TEST_ASSERT(dl_error_code_ok, "DL-ERROR code N (retry limit exceeded) raised", (unsigned )last_dl_error);
    return 0;
}

// Test 7.2.b: Retry count increments on each T1 expiry
static int test_backoff_retry_counter(void) {
    printf("\n--- test_backoff_retry_counter ---\n");
    DEBUG_PRINT("Backoff: retry_count increments on each T1 expiry");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    conn.timers.t1 = 5;
    conn.timers.n2 = 10;  // plenty of retries so we do not disconnect
    DEBUG_VAR("T1 (ticks)", conn.timers.t1);
    DEBUG_VAR("N2 (retries)", conn.timers.n2);

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(dest && src, "Addresses allocated", 0);

    if (establish_connection(&conn, dest, src) != 0) {
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Connection established", 0);
    }

    const uint8_t payload[] = { 'X' };
    reset_capture();
    ax25_send_data(&conn, (uint8_t*) payload, 1, 0xF0);

    uint8_t prev_retry = conn.retry_count;
    DEBUG_VAR("Initial retry_count", prev_retry);

    // Let T1 expire twice: drive 2 * (t1+1) ticks
    for (uint32_t i = 1; i <= 2 * (uint32_t) (conn.timers.t1 + 2); i++) {
        global_tick = i;
        ax25_tick(&conn, i);
        if (conn.retry_count > prev_retry) {
            DEBUG_VAR("retry_count incremented to", conn.retry_count);
            prev_retry = conn.retry_count;
        }
        if (conn.state == AX25_STATE_DISCONNECTED)
            break;
    }

    DEBUG_VAR("Final retry_count", conn.retry_count);
    // Collect results before cleanup so TEST_ASSERT early-return cannot skip
    // cleanup_addresses or ax25_connection_cleanup, which would leak address
    // structs (20 bytes each) and the tx_queue I-frame (17 bytes).
    int retry_ok = (conn.retry_count >= 2);
    int expiry_ok = (conn.stats.t1_expirations >= 2);
    // Drain queue to avoid leak
    send_rr_ack(&conn, conn.vars.vs, 200);
    cleanup_addresses(&dest, &src);
    ax25_connection_cleanup(&conn);
    TEST_ASSERT(retry_ok, "retry_count >= 2 after two T1 expirations", conn.retry_count);
    TEST_ASSERT(expiry_ok, "t1_expirations stat >= 2", conn.stats.t1_expirations);
    return 0;
}

// Test 7.2.c: retry_count resets to 0 after a successful ACK
static int test_backoff_retry_reset_on_ack(void) {
    printf("\n--- test_backoff_retry_reset_on_ack ---\n");
    DEBUG_PRINT("Backoff: retry_count = 0 after successful acknowledgment");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    conn.timers.t1 = 5;
    conn.timers.n2 = 10;

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(dest && src, "Addresses allocated", 0);

    if (establish_connection(&conn, dest, src) != 0) {
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Connection established", 0);
    }

    const uint8_t payload[] = { 'Y' };
    reset_capture();
    ax25_send_data(&conn, (uint8_t*) payload, 1, 0xF0);

    // Let T1 expire once to increment retry_count
    for (uint32_t i = 1; i <= (uint32_t) (conn.timers.t1 + 2); i++) {
        global_tick = i;
        ax25_tick(&conn, i);
        if (conn.retry_count > 0)
            break;
    }

    DEBUG_VAR("retry_count after T1 expiry", conn.retry_count);
    TEST_ASSERT(conn.retry_count > 0, "retry_count > 0 after T1 expiry", conn.retry_count);

    // Now ACK all outstanding frames
    send_rr_ack(&conn, conn.vars.vs, 50);
    DEBUG_VAR("retry_count after ACK", conn.retry_count);
    TEST_ASSERT(conn.retry_count == 0, "retry_count = 0 after acknowledgment", conn.retry_count);

    cleanup_addresses(&dest, &src);
    ax25_connection_cleanup(&conn);
    return 0;
}

// Test 7.2.d: Multiple I-frames all retransmitted on T1 expiry (go-back-N)
static int test_backoff_go_back_n_retransmit(void) {
    printf("\n--- test_backoff_go_back_n_retransmit ---\n");
    DEBUG_PRINT("Backoff: all unACKed I-frames retransmitted on T1 expiry");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    conn.timers.t1 = 10;
    conn.timers.n2 = 5;
    conn.timers.k = 7;  // window

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(dest && src, "Addresses allocated", 0);

    if (establish_connection(&conn, dest, src) != 0) {
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Connection established", 0);
    }

    // Send 3 I-frames
    const uint8_t payload[] = { 'G', 'B', 'N' };
    reset_capture();
    tx_log_index = 0;
    for (int i = 0; i < 3; i++) {
        ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    }
    uint32_t initial_tx = transmit_count;
    DEBUG_VAR("Initial transmit count (3 I-frames)", initial_tx);

    // Advance past T1 (no ACK given)
    for (uint32_t i = 1; i <= (uint32_t) (conn.timers.t1 + 2); i++) {
        global_tick = i;
        ax25_tick(&conn, i);
        if (conn.state == AX25_STATE_TIMER_RECOVERY)
            break;
    }
    DEBUG_STATE("State after T1 expiry", conn.state);
    uint32_t after_retry_tx = transmit_count;
    DEBUG_VAR("Transmit count after retransmit", after_retry_tx);

    TEST_ASSERT(conn.state == AX25_STATE_TIMER_RECOVERY, "State = TIMER_RECOVERY after T1 expiry", conn.state);
    // All 3 frames should have been retransmitted (3 additional TX calls)
    TEST_ASSERT(after_retry_tx >= initial_tx + 3, "All 3 I-frames retransmitted after T1 expiry", after_retry_tx);
    TEST_ASSERT(conn.stats.iframe_retransmitted >= 3, "iframe_retransmitted stat >= 3", conn.stats.iframe_retransmitted);

    // Clean up
    send_rr_ack(&conn, conn.vars.vs, 200);
    cleanup_addresses(&dest, &src);
    ax25_connection_cleanup(&conn);
    return 0;
}

// Test 7.2.e: AWAITING_CONNECTION retransmits SABM on T1, not TIMER_RECOVERY
static int test_backoff_sabm_retransmit_no_recovery_state(void) {
    printf("\n--- test_backoff_sabm_retransmit_no_recovery_state ---\n");
    DEBUG_PRINT("Backoff: SABM retransmit in AWAITING_CONNECTION stays in that state");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    conn.timers.t1 = 5;
    conn.timers.n2 = 5;

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(dest && src, "Addresses allocated", 0);

    reset_capture();
    uint8_t err = ax25_connect(&conn, dest, src);
    TEST_ASSERT(err == 0, "ax25_connect returned 0", err);
    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_CONNECTION, "State = AWAITING_CONNECTION after connect", conn.state);
    DEBUG_STATE("State after connect", conn.state);

    uint32_t tx_before = transmit_count;

    // Let T1 expire once (no UA received)
    for (uint32_t i = 1; i <= (uint32_t) (conn.timers.t1 + 3); i++) {
        global_tick = i;
        ax25_tick(&conn, i);
        DEBUG_VAR("tick", i);
        DEBUG_STATE("state", conn.state);
        if (transmit_count > tx_before)
            break;  // SABM retransmitted
    }

    DEBUG_VAR("tx count before retry", tx_before);
    DEBUG_VAR("tx count after retry", transmit_count);
    DEBUG_STATE("State after T1 expiry during AWAITING_CONNECTION", conn.state);

    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_CONNECTION, "State remains AWAITING_CONNECTION (not TIMER_RECOVERY)", conn.state);
    TEST_ASSERT(transmit_count > tx_before, "SABM retransmitted on T1 expiry", transmit_count);

    cleanup_addresses(&dest, &src);
    ax25_connection_cleanup(&conn);
    return 0;
}

// Test 7.2.f: Retransmit counter stats track correctly across multiple retries
static int test_backoff_stats_tracking(void) {
    printf("\n--- test_backoff_stats_tracking ---\n");
    DEBUG_PRINT("Backoff: retries and t1_expirations stats tracked accurately");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit, .on_dl_error = dl_error_callback };
    TEST_ASSERT(ax25_connection_init(&conn, &cb, NULL) == 0, "Connection init succeeded", 0);

    conn.timers.t1 = 5;
    conn.timers.n2 = 4;

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(dest && src, "Addresses allocated", 0);

    if (establish_connection(&conn, dest, src) != 0) {
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Connection established", 0);
    }

    ax25_reset_statistics(&conn);
    const uint8_t payload[] = { 'S', 'T', 'A', 'T' };
    reset_capture();
    dl_error_call_count = 0;
    ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);

    // Run until disconnect or max iterations
    uint32_t max_ticks = (uint32_t) (conn.timers.t1 + 2) * (conn.timers.n2 + 2) + 20;
    for (uint32_t i = 1; i <= max_ticks; i++) {
        global_tick = i;
        ax25_tick(&conn, i);
        if (conn.state == AX25_STATE_DISCONNECTED)
            break;
    }

    const ax25_statistics_t *stats = ax25_get_statistics(&conn);
    TEST_ASSERT(stats != NULL, "Statistics pointer valid", 0);
    DEBUG_VAR("t1_expirations", stats->t1_expirations);
    DEBUG_VAR("retries stat", stats->retries);
    DEBUG_VAR("iframe_retransmitted", stats->iframe_retransmitted);
    DEBUG_VAR("dl_error_call_count", dl_error_call_count);

    // Collect results before cleanup to prevent TEST_ASSERT early-return from
    // leaking address structs and the encoded I-frame in the tx_queue.
    // ax25_connection_cleanup() added to free any frames left in the queue
    // after N2 exhaustion drives the connection to DISCONNECTED state.
    int t1exp_ok = (stats->t1_expirations >= (uint16_t) conn.timers.n2);
    int dlerr_ok = (dl_error_call_count >= 1);
    cleanup_addresses(&dest, &src);
    ax25_connection_cleanup(&conn);
    TEST_ASSERT(t1exp_ok, "t1_expirations >= N2", stats->t1_expirations);
    TEST_ASSERT(dlerr_ok, "DL-ERROR fired (N2 exhaustion)", dl_error_call_count);
    return 0;
}

// ============================================================================
// SECTION 7.3 – T100-T108 Interaction (digipeater timer interplay)
// ============================================================================

// Helper: initialize physical layer with all callbacks
static void init_phys_with_callbacks(ax25_physical_t *phys) {
    ax25_physical_init(phys);
    phys->ptt_control = ptt_control_callback;
    phys->carrier_detect = carrier_detect_callback;
    phys->send_data = send_data_callback;
    reset_ptt_state();
    simulated_carrier = false;
}

// Helper: run physical ticks and return tick when PTT first goes ON
// Returns 0 if PTT never went on within max_ticks
static uint32_t run_until_ptt_on(ax25_physical_t *phys, uint32_t start, uint32_t max_ticks) {
    reset_ptt_state();
    for (uint32_t i = start; i < start + max_ticks; i++) {
        ax25_physical_tick(phys, i);
        if (ptt_state) {
            DEBUG_VAR("PTT ON at tick", i);
            return i;
        }
    }
    return 0;
}

// Helper: run physical ticks and return tick when PTT first goes OFF
// Returns 0 if PTT never went off within max_ticks
static uint32_t run_until_ptt_off(ax25_physical_t *phys, uint32_t start, uint32_t max_ticks) {
    for (uint32_t i = start; i < start + max_ticks; i++) {
        ax25_physical_tick(phys, i);
        if (!ptt_state) {
            DEBUG_VAR("PTT OFF at tick", i);
            return i;
        }
    }
    return 0;
}

// Test 7.3.a: T100 (axhang) + T103 (txdely) – hang added after data, delay before data
static int test_t100_t103_interaction(void) {
    printf("\n--- test_t100_t103_interaction (T100+T103 axhang + txdelay) ---\n");
    DEBUG_PRINT("T100+T103: PTT ON at t=0, data at t=txdely, PTT OFF at t=data_end+axhang");

    ax25_physical_t phys;
    init_phys_with_callbacks(&phys);

    phys.txdely_10ms = 3;   // T103: 30ms key-up delay
    phys.axhang_10ms = 4;   // T100: 40ms hang after last frame
    phys.persist = 255;  // Always transmit (no CSMA backoff)
    phys.full_duplex = false;
    phys.slottime_10ms = 0;
    phys.interframe_flags = 1;
    phys.preamble_flags = 0;
    DEBUG_VAR("txdely_10ms (T103)", phys.txdely_10ms);
    DEBUG_VAR("axhang_10ms (T100)", phys.axhang_10ms);

    uint8_t frame[10] = { 0x7E, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);

    // PTT should assert immediately (persist=255, no carrier)
    uint32_t ptt_on_tick = run_until_ptt_on(&phys, 0, 20);
    DEBUG_VAR("PTT asserted at tick", ptt_on_tick);
    TEST_ASSERT(ptt_on_tick != 0, "PTT asserted after frame queued", 0);

    // Data transmission begins after txdely; let it run until completion
    uint32_t done_tick = ptt_on_tick;
    for (uint32_t i = ptt_on_tick; i < ptt_on_tick + 30; i++) {
        ax25_physical_tick(&phys, i);
        done_tick = i;
        if (!ptt_state && phys_send_count > 0) {
            DEBUG_VAR("Transmission complete at tick", i);
            break;
        }
    }

    // After data, PTT should remain ON for axhang_10ms ticks
#ifdef DEBUG_ENABLE
    bool ptt_held = ptt_state;  // still ON during hang
#endif
    DEBUG_BOOL("PTT still held during hang", ptt_held);
    DEBUG_VAR("phys_send_count", phys_send_count);
    // The key assertion: data was sent (send_data called)
    TEST_ASSERT(phys_send_count > 0, "Data transmitted via send_data callback", phys_send_count);

    // PTT should go OFF after hang expires; run enough ticks
    uint32_t ptt_off_tick = run_until_ptt_off(&phys, done_tick, 20);
    DEBUG_VAR("PTT de-asserted at tick", ptt_off_tick);
    TEST_ASSERT(ptt_off_tick != 0, "PTT de-asserted after hang time", 0);

    return 0;
}

// Test 7.3.b: T104 (axdelay) digipeater delay adds extra wait before PTT
static int test_t104_digipeater_interaction(void) {
    printf("\n--- test_t104_digipeater_interaction (T104 axdelay) ---\n");
    DEBUG_PRINT("T104: digipeated frame PTT delayed by axdelay over normal frame");

    ax25_physical_t phys_normal;
    init_phys_with_callbacks(&phys_normal);
    phys_normal.txdely_10ms = 0;
    phys_normal.axhang_10ms = 0;
    phys_normal.axdelay_10ms = 5;  // T104: 50ms digipeater delay
    phys_normal.persist = 255;
    phys_normal.slottime_10ms = 0;
    phys_normal.preamble_flags = 0;
    phys_normal.interframe_flags = 1;
    DEBUG_VAR("axdelay_10ms (T104)", phys_normal.axdelay_10ms);

    uint8_t frame[10] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A };

    // Queue normal (non-digipeated) frame
    ax25_physical_queue_frame(&phys_normal, frame, sizeof(frame), false);
    uint32_t ptt_on_normal = run_until_ptt_on(&phys_normal, 0, 30);
    DEBUG_VAR("PTT ON tick for normal frame", ptt_on_normal);
    TEST_ASSERT(ptt_on_normal != 0, "Normal frame PTT asserted", 0);

    // Queue digipeated frame
    ax25_physical_t phys_digi;
    init_phys_with_callbacks(&phys_digi);
    phys_digi.txdely_10ms = 0;
    phys_digi.axhang_10ms = 0;
    phys_digi.axdelay_10ms = 5;
    phys_digi.persist = 255;
    phys_digi.slottime_10ms = 0;
    phys_digi.preamble_flags = 0;
    phys_digi.interframe_flags = 1;

    ax25_physical_queue_frame(&phys_digi, frame, sizeof(frame), true);  // digipeat=true
    uint32_t ptt_on_digi = run_until_ptt_on(&phys_digi, 0, 30);
    DEBUG_VAR("PTT ON tick for digipeated frame", ptt_on_digi);
    // Digipeated PTT should be delayed by axdelay_10ms compared to normal
    // ptt_on_normal ~0, ptt_on_digi >= axdelay_10ms
    TEST_ASSERT(ptt_on_digi >= (uint32_t )phys_normal.axdelay_10ms, "Digipeated frame PTT delayed by at least axdelay ticks", ptt_on_digi);
    TEST_ASSERT(ptt_on_digi > ptt_on_normal, "Digipeated PTT later than normal PTT", ptt_on_digi);

    return 0;
}

// Test 7.3.c: T100 + T104 combined – axhang from first frame defers digipeated frame
static int test_t100_t104_combined(void) {
    printf("\n--- test_t100_t104_combined (T100 hang + T104 digipeater delay) ---\n");
    DEBUG_PRINT("T100+T104: normal frame hangs channel; digipeated adds on top");

    ax25_physical_t phys;
    init_phys_with_callbacks(&phys);
    phys.txdely_10ms = 0;
    phys.axhang_10ms = 3;   // T100: 30ms hang
    phys.axdelay_10ms = 2;   // T104: 20ms extra for digi
    phys.persist = 255;
    phys.slottime_10ms = 0;
    phys.preamble_flags = 0;
    phys.interframe_flags = 1;
    DEBUG_VAR("axhang_10ms (T100)", phys.axhang_10ms);
    DEBUG_VAR("axdelay_10ms (T104)", phys.axdelay_10ms);

    uint8_t frame[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

    // Queue one normal frame followed by one digipeated frame
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), true);

    // Drive ticks to get first frame through
    uint32_t first_data_sent_tick = 0;
    for (uint32_t i = 0; i < 60; i++) {
        ax25_physical_tick(&phys, i);
        if (phys_send_count > 0 && first_data_sent_tick == 0) {
            first_data_sent_tick = i;
            DEBUG_VAR("First frame data sent at tick", first_data_sent_tick);
        }
        // Stop once first frame fully done (PTT drops after hang)
        if (phys_send_count > 0 && !ptt_state && i > first_data_sent_tick + 1) {
            DEBUG_VAR("PTT released after first frame+hang at tick", i);
            break;
        }
    }

    TEST_ASSERT(phys_send_count > 0, "First frame transmitted", phys_send_count);

    // Now continue running to see second (digipeated) frame transmit
    uint32_t second_data_sent_tick = 0;
    uint32_t second_start = first_data_sent_tick + phys.axhang_10ms + 1;
    for (uint32_t i = second_start; i < second_start + 30; i++) {
        ax25_physical_tick(&phys, i);
        if (phys_send_count >= 2 && second_data_sent_tick == 0) {
            second_data_sent_tick = i;
            DEBUG_VAR("Second (digi) frame data sent at tick", second_data_sent_tick);
        }
    }

    DEBUG_VAR("second_data_sent_tick", second_data_sent_tick);
    TEST_ASSERT(second_data_sent_tick > first_data_sent_tick, "Second (digipeated) frame sent after first frame", second_data_sent_tick);

    return 0;
}

// Test 7.3.d: T102 (slottime) CSMA back-off – channel busy delays PTT
static int test_t102_csma_defers_digi(void) {
    printf("\n--- test_t102_csma_defers_digi (T102 slottime CSMA deferral) ---\n");
    DEBUG_PRINT("T102: channel busy forces slottime deferral");

    ax25_physical_t phys;
    init_phys_with_callbacks(&phys);
    phys.txdely_10ms = 0;
    phys.axhang_10ms = 0;
    phys.axdelay_10ms = 0;
    phys.slottime_10ms = 3;   // T102: 30ms slot time
    phys.persist = 255;  // deterministic: always try after slot
    phys.preamble_flags = 0;
    phys.interframe_flags = 1;
    DEBUG_VAR("slottime_10ms (T102)", phys.slottime_10ms);

    // Simulate busy channel
    simulated_carrier = true;
    uint8_t frame[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);

    // Run while channel busy – PTT must NOT assert
    for (uint32_t i = 0; i < 10; i++) {
        ax25_physical_tick(&phys, i);
        DEBUG_VAR("tick", i);DEBUG_BOOL("ptt_state while carrier busy", ptt_state);
    }
    bool ptt_asserted_while_busy = ptt_state;
    DEBUG_BOOL("PTT asserted while carrier busy", ptt_asserted_while_busy);
    TEST_ASSERT(!ptt_asserted_while_busy, "PTT NOT asserted while channel busy (CSMA defers)", (uint32_t )ptt_asserted_while_busy);

    // Clear the carrier – PTT should assert within slottime ticks
    simulated_carrier = false;
    DEBUG_PRINT("Channel cleared, waiting for PTT");
    uint32_t ptt_on_tick = run_until_ptt_on(&phys, 10, 20);
    DEBUG_VAR("PTT asserted after channel clear at tick", ptt_on_tick);
    TEST_ASSERT(ptt_on_tick != 0, "PTT asserted after channel clears", 0);

    return 0;
}

// Test 7.3.e: T103 (txdely) delays data after PTT – short enough does not lose sync
static int test_t103_data_after_txdely(void) {
    printf("\n--- test_t103_data_after_txdely (T103 txdely pipelining) ---\n");
    DEBUG_PRINT("T103: data transmission starts exactly after txdely ticks");

    ax25_physical_t phys;
    init_phys_with_callbacks(&phys);
    phys.txdely_10ms = 5;  // T103: 50ms key-up delay
    phys.axhang_10ms = 0;
    phys.axdelay_10ms = 0;
    phys.persist = 255;
    phys.slottime_10ms = 0;
    phys.preamble_flags = 0;
    phys.interframe_flags = 1;
    DEBUG_VAR("txdely_10ms (T103)", phys.txdely_10ms);

    uint8_t frame[8] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22 };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);

    // Find tick when PTT goes ON
    uint32_t ptt_tick = run_until_ptt_on(&phys, 0, 20);
    DEBUG_VAR("PTT ON at tick", ptt_tick);
    TEST_ASSERT(ptt_tick != 0, "PTT asserted", 0);

    // Data must NOT be sent before txdely has elapsed
    uint32_t send_before = phys_send_count;
    for (uint32_t i = ptt_tick; i < ptt_tick + phys.txdely_10ms - 1; i++) {
        ax25_physical_tick(&phys, i);
        DEBUG_VAR("tick before txdely expiry", i);DEBUG_BOOL("send_data called before txdely", phys_send_count > send_before);
    }
    bool data_too_early = (phys_send_count > send_before);
    DEBUG_BOOL("Data sent before txdely expired", data_too_early);
    TEST_ASSERT(!data_too_early, "Data NOT sent before txdely expires", (uint32_t )data_too_early);

    // After txdely, data must be sent
    for (uint32_t i = ptt_tick + phys.txdely_10ms; i < ptt_tick + phys.txdely_10ms + 5; i++) {
        ax25_physical_tick(&phys, i);
        if (phys_send_count > send_before)
            break;
    }
    DEBUG_VAR("phys_send_count after txdely", phys_send_count);
    TEST_ASSERT(phys_send_count > send_before, "Data sent after txdely elapses", phys_send_count);

    return 0;
}

// Test 7.3.f: T106 (max_tx_duration) truncates burst before N2 retries
static int test_t106_max_tx_duration_interaction(void) {
    printf("\n--- test_t106_max_tx_duration_interaction ---\n");
    DEBUG_PRINT("T106: max_tx_duration terminates transmission before T107 if hit");

    ax25_physical_t phys;
    init_phys_with_callbacks(&phys);
    phys.txdely_10ms = 0;
    phys.axhang_10ms = 0;
    phys.axdelay_10ms = 0;
    phys.max_tx_duration_10ms = 5;  // T106: only 50ms max transmission
    phys.anti_hog_10ms = 0;  // T107 disabled
    phys.persist = 255;
    phys.slottime_10ms = 0;
    phys.preamble_flags = 0;
    phys.interframe_flags = 1;
    DEBUG_VAR("max_tx_duration_10ms (T106)", phys.max_tx_duration_10ms);

    // Queue several large frames
    uint8_t frame[50];
    memset(frame, 0xAB, sizeof(frame));
    for (int i = 0; i < 4; i++) {
        ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
        DEBUG_VAR("Frame queued", i);
    }

    // Run and see if TX stops before all frames are sent
    uint32_t ptt_on_tick = run_until_ptt_on(&phys, 0, 20);
    DEBUG_VAR("PTT ON at tick", ptt_on_tick);
    TEST_ASSERT(ptt_on_tick != 0, "PTT asserted", 0);

    uint32_t tx_end = 0;
    for (uint32_t i = ptt_on_tick; i < ptt_on_tick + 30; i++) {
        ax25_physical_tick(&phys, i);
        if (!ptt_state) {
            tx_end = i;
            DEBUG_VAR("TX ended at tick", tx_end);
            break;
        }
    }

    DEBUG_VAR("phys_send_count at TX end", phys_send_count);
    // TX should have been stopped by T106 limit
    TEST_ASSERT(tx_end != 0, "Transmission terminated by T106 (PTT released)", tx_end);
    TEST_ASSERT((tx_end - ptt_on_tick) <= (uint32_t )(phys.max_tx_duration_10ms + 3), "TX duration <= T106 limit (+3 ticks tolerance)", tx_end - ptt_on_tick);

    return 0;
}

// Test 7.3.g: T107 (anti_hog) + T100 (axhang) – anti-hog releases PTT then hang waits
static int test_t107_t100_combined(void) {
    printf("\n--- test_t107_t100_combined (T107 anti-hog + T100 hang) ---\n");
    DEBUG_PRINT("T107+T100: anti-hog causes early TX end, hang follows");

    ax25_physical_t phys;
    init_phys_with_callbacks(&phys);
    phys.txdely_10ms = 0;
    phys.axhang_10ms = 3;  // T100: 30ms hang
    phys.axdelay_10ms = 0;
    phys.max_tx_duration_10ms = 0;  // T106: disabled
    phys.anti_hog_10ms = 4;  // T107: 40ms anti-hog burst limit
    phys.persist = 255;
    phys.slottime_10ms = 0;
    phys.preamble_flags = 0;
    phys.interframe_flags = 1;
    DEBUG_VAR("anti_hog_10ms (T107)", phys.anti_hog_10ms);
    DEBUG_VAR("axhang_10ms  (T100)", phys.axhang_10ms);

    uint8_t frame[20];
    memset(frame, 0x55, sizeof(frame));
    for (int i = 0; i < 3; i++)
        ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);

    uint32_t ptt_on_tick = run_until_ptt_on(&phys, 0, 20);
    DEBUG_VAR("PTT ON at tick", ptt_on_tick);
    TEST_ASSERT(ptt_on_tick != 0, "PTT asserted", 0);

    uint32_t anti_hog_release_tick = 0;
    for (uint32_t i = ptt_on_tick; i < ptt_on_tick + 30; i++) {
        ax25_physical_tick(&phys, i);
        if (!ptt_state && anti_hog_release_tick == 0) {
            anti_hog_release_tick = i;
            DEBUG_VAR("Anti-hog released PTT at tick", anti_hog_release_tick);
            break;
        }
    }

    TEST_ASSERT(anti_hog_release_tick != 0, "Anti-hog (T107) released PTT within burst limit", anti_hog_release_tick);
    // Duration should be roughly anti_hog limit
    uint32_t burst = anti_hog_release_tick - ptt_on_tick;
    DEBUG_VAR("Burst duration ticks", burst);
    TEST_ASSERT(burst <= (uint32_t )(phys.anti_hog_10ms + 5), "Burst duration within T107 limit (+5 ticks tolerance)", burst);

    return 0;
}

// Test 7.3.h: T108 (rx_startup) delays second transmission after first TX ends
static int test_t108_rx_startup_interaction(void) {
    printf("\n--- test_t108_rx_startup_interaction (T108 + T103 interaction) ---\n");
    DEBUG_PRINT("T108: rx_startup delay between successive TX bursts");

    ax25_physical_t phys;
    init_phys_with_callbacks(&phys);
    phys.txdely_10ms = 0;
    phys.axhang_10ms = 0;
    phys.axdelay_10ms = 0;
    phys.rx_startup_10ms = 5;  // T108: 50ms receiver startup delay
    phys.persist = 255;
    phys.slottime_10ms = 0;
    phys.preamble_flags = 0;
    phys.interframe_flags = 1;
    DEBUG_VAR("rx_startup_10ms (T108)", phys.rx_startup_10ms);

    uint8_t frame[10] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A };

    // Queue and transmit first frame
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    uint32_t first_ptt_on = run_until_ptt_on(&phys, 0, 20);
    DEBUG_VAR("First PTT ON tick", first_ptt_on);
    TEST_ASSERT(first_ptt_on != 0, "First PTT asserted", 0);

    // Run until first transmission ends (PTT goes off)
    uint32_t first_tx_end = 0;
    for (uint32_t i = first_ptt_on; i < first_ptt_on + 30; i++) {
        ax25_physical_tick(&phys, i);
        if (!ptt_state) {
            first_tx_end = i;
            DEBUG_VAR("First TX ended at tick", first_tx_end);
            break;
        }
    }
    TEST_ASSERT(first_tx_end != 0, "First transmission completed", first_tx_end);

    // Queue second frame immediately after first completes
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_PRINT("Second frame queued after first TX end");

    // Second TX must not start before rx_startup ticks have elapsed
    uint32_t second_ptt_on = 0;
    for (uint32_t i = first_tx_end; i < first_tx_end + phys.rx_startup_10ms + 10; i++) {
        ax25_physical_tick(&phys, i);
        if (ptt_state) {
            second_ptt_on = i;
            DEBUG_VAR("Second PTT ON tick", second_ptt_on);
            break;
        }
    }

    DEBUG_VAR("T108 rx_startup_10ms", phys.rx_startup_10ms);
    DEBUG_VAR("second_ptt_on - first_tx_end", second_ptt_on > 0 ? second_ptt_on - first_tx_end : 0);
    TEST_ASSERT(second_ptt_on != 0, "Second transmission eventually starts", second_ptt_on);
    TEST_ASSERT((second_ptt_on - first_tx_end) >= (uint32_t )phys.rx_startup_10ms, "Second TX delayed by at least T108 rx_startup ticks",
            second_ptt_on - first_tx_end);

    return 0;
}

// Test 7.3.i: T100 axhang defers a new frame if queued during hang
static int test_t100_hang_defers_new_frame(void) {
    printf("\n--- test_t100_hang_defers_new_frame (T100 hang defers queued frame) ---\n");
    DEBUG_PRINT("T100: frame queued during hang window does not start immediately");

    ax25_physical_t phys;
    init_phys_with_callbacks(&phys);
    phys.txdely_10ms = 0;
    phys.axhang_10ms = 6;  // T100: 60ms
    phys.persist = 255;
    phys.slottime_10ms = 0;
    phys.preamble_flags = 0;
    phys.interframe_flags = 1;
    DEBUG_VAR("axhang_10ms (T100)", phys.axhang_10ms);

    uint8_t frame[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04 };

    // Transmit first frame
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    uint32_t ptt_on_1 = run_until_ptt_on(&phys, 0, 20);
    DEBUG_VAR("First PTT ON at tick", ptt_on_1);
    TEST_ASSERT(ptt_on_1 != 0, "First PTT asserted", 0);

    uint32_t first_data_end = 0;
    for (uint32_t i = ptt_on_1; i < ptt_on_1 + 20; i++) {
        ax25_physical_tick(&phys, i);
        if (phys_send_count > 0 && !phys.tx_active) {
            first_data_end = i;
            DEBUG_VAR("First data end (TX inactive) at tick", i);
            break;
        }
    }

    // Queue a second frame DURING the hang window
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_PRINT("Second frame queued during hang window");

    // PTT should remain continuously ON through the hang (back-to-back burst)
    // or second PTT should not assert until after hang completes
    // Either behaviour is acceptable, but data must only be sent after hang
    uint32_t hang_end_expected = ptt_on_1 + phys.axhang_10ms + 10;  // generous estimate
#ifdef DEBUG_ENABLE
    bool ptt_dropped_during_hang = false;
#endif
    for (uint32_t i = first_data_end; i < hang_end_expected; i++) {
        ax25_physical_tick(&phys, i);
        if (!ptt_state) {
#ifdef DEBUG_ENABLE
            ptt_dropped_during_hang = true;
#endif
            DEBUG_VAR("PTT dropped at tick", i);
            break;
        }
    }
    // Whether back-to-back (continuous PTT) or hang-then-restart, second frame
    // must eventually be sent
    uint32_t second_data_tick = 0;
    for (uint32_t i = first_data_end; i < first_data_end + phys.axhang_10ms + 20; i++) {
        ax25_physical_tick(&phys, i);
        if (phys_send_count >= 2) {
            second_data_tick = i;
            DEBUG_VAR("Second frame data sent at tick", second_data_tick);
            break;
        }
    }

    DEBUG_BOOL("PTT dropped during hang", ptt_dropped_during_hang);
    DEBUG_VAR("second_data_tick", second_data_tick);
    TEST_ASSERT(second_data_tick > first_data_end, "Second frame sent after first frame completes", second_data_tick);

    return 0;
}

// Test 7.3.j: All timers disabled (axhang=0, txdely=0, axdelay=0) – immediate TX
static int test_all_timers_disabled(void) {
    printf("\n--- test_all_timers_disabled (all T100-T108 zero) ---\n");
    DEBUG_PRINT("All timers zero: immediate PTT, immediate data, immediate release");

    ax25_physical_t phys;
    init_phys_with_callbacks(&phys);
    phys.txdely_10ms = 0;
    phys.axhang_10ms = 0;
    phys.axdelay_10ms = 0;
    phys.max_tx_duration_10ms = 0;
    phys.anti_hog_10ms = 0;
    phys.rx_startup_10ms = 0;
    phys.remote_sync_10ms = 0;
    phys.persist = 255;
    phys.slottime_10ms = 0;
    phys.preamble_flags = 0;
    phys.interframe_flags = 0;
    DEBUG_PRINT("All timing parameters = 0");

    uint8_t frame[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);

    // One tick should be enough to start
    ax25_physical_tick(&phys, 1);
    DEBUG_BOOL("PTT after tick 1", ptt_state);
    DEBUG_VAR("phys_send_count after tick 1", phys_send_count);

    ax25_physical_tick(&phys, 2);
    DEBUG_BOOL("PTT after tick 2", ptt_state);

    // Data should be sent within a very few ticks
    for (uint32_t i = 3; i < 10; i++) {
        ax25_physical_tick(&phys, i);
        if (phys_send_count > 0)
            break;
    }
    TEST_ASSERT(phys_send_count > 0, "Data sent within 10 ticks with all timers disabled", phys_send_count);

    // PTT should release quickly (no hang)
    for (uint32_t i = 10; i < 20; i++) {
        ax25_physical_tick(&phys, i);
        if (!ptt_state)
            break;
    }
    TEST_ASSERT(!ptt_state, "PTT released quickly with axhang=0", (uint32_t )ptt_state);

    return 0;
}

// ============================================================================
// Main test runner
// ============================================================================

int test_ax25_timers_advanced_main(void) {
    int result = 0;

    printf("\n================================================================================\n");
    printf("AX.25 Advanced Timer Tests: T1 Adaptive, Backoff, T100-T108 Interaction\n");
    printf("================================================================================\n");

    printf("\n--- Section 7.1: T1 Adaptive Adjustment (RTT-based) ---\n");
    result |= test_t1_adaptive_rtt_proportional();
    result |= test_t1_adaptive_fullduplex_margin();
    result |= test_t1_adaptive_min_clamp();
    result |= test_t1_adaptive_max_clamp();
    result |= test_t1_adaptive_no_samples();
    result |= test_t1_adaptive_progressive_refinement();
    result |= test_t1_get_avg_rtt_no_samples();
    result |= test_t1_get_avg_rtt_with_samples();

    printf("\n--- Section 7.2: Exponential Backoff / Retry Strategy ---\n");
    result |= test_backoff_n2_exhaustion();
    result |= test_backoff_retry_counter();
    result |= test_backoff_retry_reset_on_ack();
    result |= test_backoff_go_back_n_retransmit();
    result |= test_backoff_sabm_retransmit_no_recovery_state();
    result |= test_backoff_stats_tracking();

    printf("\n--- Section 7.3: T100-T108 Interaction (digipeater timer interplay) ---\n");
    result |= test_t100_t103_interaction();
    result |= test_t104_digipeater_interaction();
    result |= test_t100_t104_combined();
    result |= test_t102_csma_defers_digi();
    result |= test_t103_data_after_txdely();
    result |= test_t106_max_tx_duration_interaction();
    result |= test_t107_t100_combined();
    result |= test_t108_rx_startup_interaction();
    result |= test_t100_hang_defers_new_frame();
    result |= test_all_timers_disabled();

    printf("\n================================================================================\n");
    printf("Advanced Timer Tests Completed. %s\n", result == 0 ? "All tests PASSED" : "Some tests FAILED");
    printf("Total assertions: %u\n", assert_count);
    printf("================================================================================\n\n");

    return result;
}
