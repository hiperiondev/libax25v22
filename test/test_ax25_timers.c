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

static uint32_t assert_count = 0;

// Capture buffers for transmitted frames
static uint8_t captured_buffer[2048];
static size_t captured_len = 0;
static uint32_t transmit_count = 0;

// PTT state tracking
static bool ptt_state = false;
static uint32_t ptt_on_count = 0;
static uint32_t ptt_off_count = 0;

// Carrier detect simulation
static bool simulated_carrier = false;

// Hardcoded callsign bytes (shifted left by 1, space padded)
static const uint8_t test1_call[6] = { 0xA8, 0x8A, 0xA6, 0xA8, 0x62, 0x40 };  // TEST1
static const uint8_t test2_call[6] = { 0xA8, 0x8A, 0xA6, 0xA8, 0x64, 0x40 };  // TEST2

// Reset capture state
static void reset_capture(void) {
    captured_len = 0;
    transmit_count = 0;
}

// Reset PTT state
static void reset_ptt_state(void) {
    ptt_state = false;
    ptt_on_count = 0;
    ptt_off_count = 0;
}

static void capture_transmit(void *user_data, uint8_t *data, size_t len) {
    if (len <= sizeof(captured_buffer)) {
        memcpy(captured_buffer, data, len);
        captured_len = len;
    }
    transmit_count++;
}

// PTT control callback for physical layer
static void ptt_control_callback(bool on, void *user_data) {
    ptt_state = on;
    if (on) {
        ptt_on_count++;
    } else {
        ptt_off_count++;
    }
}

// Carrier detect callback for physical layer
static bool carrier_detect_callback(void *user_data) {
    return simulated_carrier;
}

// Send data callback for physical layer
static void send_data_callback(const uint8_t *data, size_t len, void *user_data) {
    if (len <= sizeof(captured_buffer)) {
        memcpy(captured_buffer, data, len);
        captured_len = len;
    }
    transmit_count++;
}

// Helper to establish connection for state machine tests
static int establish_connection(ax25_connection_t *conn, ax25_address_t *dest, ax25_address_t *src) {
    reset_capture();
    uint8_t err = ax25_connect(conn, dest, src);
    if (err != 0)
        return -1;

    // Simulate UA response
    uint8_t ua_raw[15];
    memcpy(ua_raw + 0, test1_call, 6);
    ua_raw[6] = 0x60;
    memcpy(ua_raw + 7, test2_call, 6);
    ua_raw[13] = 0x61;
    ua_raw[14] = 0x73;  // UA with F=1

    uint8_t decode_err = 0;
    ax25_frame_t *ua_frame = ax25_frame_decode(ua_raw, sizeof(ua_raw), MODULO128_FALSE, &decode_err);
    if (!ua_frame)
        return -1;

    reset_capture();
    ax25_process_frame(conn, ua_frame);
    ax25_frame_free(ua_frame, &decode_err);

    return (conn->state == AX25_STATE_CONNECTED) ? 0 : -1;
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

// ============================================================================
// T1 - Acknowledgment Timer Tests
// ============================================================================

static int test_t1_timer_expiration(void) {
    printf("\n--- test_t1_timer_expiration (T1 acknowledgment timer) ---\n");
    DEBUG_PRINT("Starting T1 timer expiration test");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    uint8_t err = ax25_connection_init(&conn, &cb, NULL);
    TEST_ASSERT(err == 0, "Connection init succeeded", err);DEBUG_VAR("Connection init error code", err);

    // Set T1 to a short value for testing (100ms = 10 ticks)
    conn.timers.t1 = 10;
    DEBUG_VAR("T1 timer configured (ticks)", conn.timers.t1);

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    DEBUG_PRINT("Addresses created: dest=TEST2-0, src=TEST1-0");

    int result = establish_connection(&conn, dest, src);
    if (result != 0) {
        DEBUG_PRINT("Connection establishment failed");
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Connection established", result);
    }DEBUG_STATE("Connection state after establish", conn.state);

    // Send data but don't acknowledge
    const uint8_t payload[] = { 'T', 'E', 'S', 'T' };
    reset_capture();
    DEBUG_PRINT("Sending data frame");
    err = ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    if (err != 0) {
        DEBUG_VAR("Send data error code", err);
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Data sent successfully", err);
    }DEBUG_FRAME("Payload sent", payload, sizeof(payload));

    if (transmit_count != 1) {
        DEBUG_VAR("Unexpected transmit count", transmit_count);
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Frame transmitted once", transmit_count);
    }DEBUG_VAR("Initial transmit count", transmit_count);

    uint32_t initial_transmit_count = transmit_count;

    // Advance time past T1 expiration (11 ticks)
    DEBUG_PRINT("Advancing time past T1 expiration");
    for (uint32_t i = 0; i < 11; i++) {
        ax25_tick(&conn, i);
        DEBUG_VAR("Tick", i);DEBUG_STATE("Connection state", conn.state);DEBUG_VAR("Transmit count", transmit_count);
    }

    if (transmit_count <= initial_transmit_count) {
        DEBUG_VAR("Transmit count after timeout", transmit_count);
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Frame retransmitted after T1 expiration", transmit_count);
    }

    if (conn.state != AX25_STATE_TIMER_RECOVERY) {
        DEBUG_STATE("Final connection state", conn.state);
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "State changed to TIMER_RECOVERY", conn.state);
    }

    DEBUG_VAR("Final transmit count", transmit_count);DEBUG_STATE("Final state", conn.state);
    TEST_ASSERT(transmit_count > initial_transmit_count, "Frame retransmitted after T1 expiration", transmit_count);
    TEST_ASSERT(conn.state == AX25_STATE_TIMER_RECOVERY, "State changed to TIMER_RECOVERY", conn.state);

    // the I-frame sent above (N(S)=0) is stored in tx_queue for retransmission
    // send a synthetic RR with N(R)=1 to acknowledge and free that queued frame
    {
        uint8_t rr_ack_raw[15];
        uint8_t decode_err_drain = 0;
        memcpy(rr_ack_raw + 0, test1_call, 6);
        rr_ack_raw[6] = 0x60;
        memcpy(rr_ack_raw + 7, test2_call, 6);
        rr_ack_raw[13] = 0x61;
        rr_ack_raw[14] = 0x21;  // RR with N(R)=1, acknowledges frame N(S)=0
        ax25_frame_t *rr_drain = ax25_frame_decode(rr_ack_raw, sizeof(rr_ack_raw), MODULO128_FALSE, &decode_err_drain);
        if (rr_drain) {
            ax25_process_frame(&conn, rr_drain);
            ax25_frame_free(rr_drain, &decode_err_drain);
        }
    }

    cleanup_addresses(&dest, &src);
    DEBUG_PRINT("T1 timer expiration test completed");
    return 0;
}

static int test_t1_timer_reset_on_ack(void) {
    printf("\n--- test_t1_timer_reset_on_ack (T1 reset when ACK received) ---\n");
    DEBUG_PRINT("Starting T1 timer reset on ACK test");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    uint8_t err = ax25_connection_init(&conn, &cb, NULL);
    TEST_ASSERT(err == 0, "Connection init succeeded", err);DEBUG_VAR("Connection init error code", err);

    conn.timers.t1 = 20;  // 200ms
    DEBUG_VAR("T1 timer configured (ticks)", conn.timers.t1);

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    DEBUG_PRINT("Addresses created: dest=TEST2-0, src=TEST1-0");

    int result = establish_connection(&conn, dest, src);
    if (result != 0) {
        DEBUG_PRINT("Connection establishment failed");
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Connection established", result);
    }DEBUG_STATE("Connection state after establish", conn.state);

    // Send data
    const uint8_t payload[] = { 'T', 'E', 'S', 'T' };
    reset_capture();
    DEBUG_PRINT("Sending data frame");
    err = ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    if (err != 0) {
        DEBUG_VAR("Send data error code", err);
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Data sent successfully", err);
    }DEBUG_FRAME("Payload sent", payload, sizeof(payload));

    uint32_t t1_start = conn.t1_start_tick;
    DEBUG_VAR64("T1 start tick", t1_start);

    // Advance time but send ACK before expiration
    DEBUG_PRINT("Advancing time (5 ticks)");
    for (uint32_t i = 0; i < 5; i++) {
        ax25_tick(&conn, i);
        DEBUG_VAR("Tick", i);
    }

    // Simulate RR (ACK) response
    DEBUG_PRINT("Simulating RR (ACK) response");
    uint8_t rr_raw[15];
    memcpy(rr_raw + 0, test1_call, 6);
    rr_raw[6] = 0x60;
    memcpy(rr_raw + 7, test2_call, 6);
    rr_raw[13] = 0x61;
    rr_raw[14] = 0x01;  // RR with N(R)=0, P/F=0
    DEBUG_FRAME("RR frame", rr_raw, sizeof(rr_raw));

    uint8_t decode_err = 0;
    ax25_frame_t *rr_frame = ax25_frame_decode(rr_raw, sizeof(rr_raw), MODULO128_FALSE, &decode_err);
    if (!rr_frame) {
        DEBUG_PRINT("RR frame decode failed");
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "RR frame decoded", decode_err);
    }

    reset_capture();
    ax25_process_frame(&conn, rr_frame);
    ax25_frame_free(rr_frame, &decode_err);
    DEBUG_STATE("Connection state after RR", conn.state);

    uint32_t t1_after_ack = conn.t1_start_tick;
    DEBUG_VAR64("T1 start tick after ACK", t1_after_ack);DEBUG_BOOL("T1 timer reset", t1_after_ack != t1_start);

    TEST_ASSERT(t1_after_ack != t1_start, "T1 timer reset after ACK", t1_after_ack == t1_start);

    // the I-frame sent above (N(S)=0) is stored in tx_queue for retransmission
    // the earlier RR had N(R)=0 which does not acknowledge frame N(S)=0 in modulo-8
    // send a synthetic RR with N(R)=1 to acknowledge and free that queued frame
    {
        uint8_t rr_ack_raw[15];
        uint8_t decode_err_drain = 0;
        memcpy(rr_ack_raw + 0, test1_call, 6);
        rr_ack_raw[6] = 0x60;
        memcpy(rr_ack_raw + 7, test2_call, 6);
        rr_ack_raw[13] = 0x61;
        rr_ack_raw[14] = 0x21;  // RR with N(R)=1, acknowledges frame N(S)=0
        ax25_frame_t *rr_drain = ax25_frame_decode(rr_ack_raw, sizeof(rr_ack_raw), MODULO128_FALSE, &decode_err_drain);
        if (rr_drain) {
            ax25_process_frame(&conn, rr_drain);
            ax25_frame_free(rr_drain, &decode_err_drain);
        }
    }

    cleanup_addresses(&dest, &src);
    DEBUG_PRINT("T1 timer reset on ACK test completed");
    return 0;
}

static int test_t2_response_delay(void) {
    printf("\n--- test_t2_response_delay (T2 response delay timer) ---\n");
    DEBUG_PRINT("Starting T2 response delay test");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    uint8_t err = ax25_connection_init(&conn, &cb, NULL);
    TEST_ASSERT(err == 0, "Connection init succeeded", err);DEBUG_VAR("Connection init error code", err);

    conn.timers.t2 = 5;  // 50ms
    DEBUG_VAR("T2 timer configured (ticks)", conn.timers.t2);

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    DEBUG_PRINT("Addresses created: dest=TEST2-0, src=TEST1-0");

    int result = establish_connection(&conn, dest, src);
    if (result != 0) {
        DEBUG_PRINT("Connection establishment failed");
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Connection established", result);
    }DEBUG_STATE("Connection state after establish", conn.state);

    // Simulate receiving I-frame
    DEBUG_PRINT("Simulating received I-frame");
    uint8_t iframe_raw[19];
    memcpy(iframe_raw + 0, test1_call, 6);
    iframe_raw[6] = 0x60;
    memcpy(iframe_raw + 7, test2_call, 6);
    iframe_raw[13] = 0x61;
    iframe_raw[14] = 0x00;  // I-frame with N(S)=0, N(R)=0
    iframe_raw[15] = 0xF0;  // PID
    iframe_raw[16] = 'T';
    iframe_raw[17] = 'E';
    iframe_raw[18] = 'S';
    DEBUG_FRAME("I-frame", iframe_raw, sizeof(iframe_raw));

    uint8_t decode_err = 0;
    ax25_frame_t *iframe = ax25_frame_decode(iframe_raw, sizeof(iframe_raw), MODULO128_FALSE, &decode_err);
    if (!iframe) {
        DEBUG_PRINT("I-frame decode failed");
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "I-frame decoded", decode_err);
    }

    reset_capture();
    ax25_process_frame(&conn, iframe);
    ax25_frame_free(iframe, &decode_err);
    DEBUG_STATE("Connection state after I-frame", conn.state);DEBUG_BOOL("T2 running", conn.t2_running);DEBUG_VAR("Transmit count immediately", transmit_count);

    uint32_t immediate_transmit_count = transmit_count;

    // Advance time past T2
    DEBUG_PRINT("Advancing time past T2");
    for (uint32_t i = 0; i < 6; i++) {
        ax25_tick(&conn, i);
        DEBUG_VAR("Tick", i);DEBUG_VAR("Transmit count", transmit_count);
    }

    DEBUG_VAR("Final transmit count", transmit_count);DEBUG_BOOL("ACK delayed", transmit_count > immediate_transmit_count);
    TEST_ASSERT(transmit_count > immediate_transmit_count, "ACK delayed by T2", transmit_count <= immediate_transmit_count);

    cleanup_addresses(&dest, &src);
    DEBUG_PRINT("T2 response delay test completed");
    return 0;
}

static int test_t3_inactive_link_polling(void) {
    printf("\n--- test_t3_inactive_link_polling (T3 inactive link timer) ---\n");
    DEBUG_PRINT("Starting T3 inactive link polling test");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    uint8_t err = ax25_connection_init(&conn, &cb, NULL);
    TEST_ASSERT(err == 0, "Connection init succeeded", err);DEBUG_VAR("Connection init error code", err);

    conn.timers.t3 = 10;  // 100ms
    conn.t3_timeout = 10;
    DEBUG_VAR("T3 timer configured (ticks)", conn.timers.t3);

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    DEBUG_PRINT("Addresses created: dest=TEST2-0, src=TEST1-0");

    int result = establish_connection(&conn, dest, src);
    if (result != 0) {
        DEBUG_PRINT("Connection establishment failed");
        cleanup_addresses(&dest, &src);
        TEST_ASSERT(false, "Connection established", result);
    }DEBUG_STATE("Connection state after establish", conn.state);

    reset_capture();

    // Advance time past T3 with no activity
    DEBUG_PRINT("Advancing time past T3 with no activity");
    for (uint32_t i = 0; i < 15; i++) {
        ax25_tick(&conn, i);
        DEBUG_VAR("Tick", i);DEBUG_VAR("Transmit count", transmit_count);
    }

    DEBUG_VAR("Final transmit count", transmit_count);DEBUG_BOOL("Poll sent", transmit_count > 0);
    TEST_ASSERT(transmit_count > 0, "Poll sent after T3 expiration", transmit_count == 0);

    cleanup_addresses(&dest, &src);
    DEBUG_PRINT("T3 inactive link polling test completed");
    return 0;
}

static int test_t100_axhang_timer(void) {
    printf("\n--- test_t100_axhang_timer (T100 repeater hang time) ---\n");
    DEBUG_PRINT("Starting T100 AXHANG timer test");

    ax25_physical_t phys;
    ax25_physical_init(&phys);
    DEBUG_PRINT("Physical layer initialized");

    phys.ptt_control = ptt_control_callback;
    phys.carrier_detect = carrier_detect_callback;
    phys.send_data = send_data_callback;

    phys.axhang_10ms = 10;  // 100ms hang time
    phys.txdely_10ms = 0;
    phys.persist = 255;
    DEBUG_VAR("AXHANG configured (ticks)", phys.axhang_10ms);

    reset_ptt_state();
    simulated_carrier = false;
    DEBUG_BOOL("Initial carrier state", simulated_carrier);

    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("Frame queued", frame, sizeof(frame));

    uint32_t tick = 0;

    // Transmit frame
    DEBUG_PRINT("Transmitting frame");
    while (phys.tx_active && tick < 50) {
        ax25_physical_tick(&phys, tick++);
        DEBUG_VAR("Tick", tick);DEBUG_BOOL("PTT state", ptt_state);DEBUG_STATE("Physical state", phys.state);
    }

    uint32_t tx_end_tick = tick;
    DEBUG_VAR("Transmission ended at tick", tx_end_tick);DEBUG_BOOL("PTT still on", ptt_state);

    // Continue ticking and check PTT stays on during hang
    DEBUG_PRINT("Checking PTT hang time");
    while (ptt_state && tick < tx_end_tick + 20) {
        ax25_physical_tick(&phys, tick++);
        DEBUG_VAR("Tick", tick);DEBUG_BOOL("PTT state", ptt_state);DEBUG_STATE("Physical state", phys.state);
    }

    uint32_t ptt_off_tick = tick;
    uint32_t hang_duration = ptt_off_tick - tx_end_tick;
    DEBUG_VAR("PTT released at tick", ptt_off_tick);DEBUG_VAR("Hang duration (ticks)", hang_duration);DEBUG_BOOL("Hang time >= configured", hang_duration >= 10);

    TEST_ASSERT(hang_duration >= 10, "PTT held for AXHANG duration", hang_duration);

    DEBUG_PRINT("T100 AXHANG timer test completed");
    return 0;
}

static int test_t102_slottime_persistence(void) {
    printf("\n--- test_t102_slottime_persistence (T102 slot time and p-persistence) ---\n");
    DEBUG_PRINT("Starting T102 slottime persistence test");

    ax25_physical_t phys;
    ax25_physical_init(&phys);
    DEBUG_PRINT("Physical layer initialized");

    phys.ptt_control = ptt_control_callback;
    phys.carrier_detect = carrier_detect_callback;
    phys.send_data = send_data_callback;

    phys.slottime_10ms = 10;  // 100ms slot time
    phys.persist = 128;        // p = 0.5
    phys.txdely_10ms = 0;
    phys.axhang_10ms = 0;
    DEBUG_VAR("Slottime configured (ticks)", phys.slottime_10ms);DEBUG_VAR("Persist value", phys.persist);

    reset_ptt_state();
    simulated_carrier = false;
    DEBUG_BOOL("Initial carrier state", simulated_carrier);

    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("Frame queued", frame, sizeof(frame));

    uint32_t tick = 0;
    uint32_t csma_wait_start = 0;
    bool entered_csma = false;

    // Wait for CSMA state
    DEBUG_PRINT("Waiting for CSMA wait state");
    while (tick < 50) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick);DEBUG_STATE("Physical state", phys.state);
        if (phys.state == PHYS_CSMA_WAIT && !entered_csma) {
            csma_wait_start = tick;
            entered_csma = true;
            DEBUG_VAR("Entered CSMA at tick", csma_wait_start);
            break;
        }
        tick++;
    }

    TEST_ASSERT(entered_csma, "Entered CSMA wait state", !entered_csma);

    // Continue until transmission starts
    DEBUG_PRINT("Waiting for transmission start");
    while (tick < csma_wait_start + 50 && !phys.tx_active) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick);DEBUG_BOOL("TX active", phys.tx_active);
        tick++;
    }

    uint32_t csma_duration = tick - csma_wait_start;
    DEBUG_VAR("CSMA duration (ticks)", csma_duration);DEBUG_BOOL("CSMA applied slottime", csma_duration >= 10);

    TEST_ASSERT(csma_duration >= 10, "CSMA applied slottime delay", csma_duration);

    DEBUG_PRINT("T102 slottime persistence test completed");
    return 0;
}

static int test_t103_txdelay_timer(void) {
    printf("\n--- test_t103_txdelay_timer (T103 TX startup delay) ---\n");
    DEBUG_PRINT("Starting T103 TXDELAY timer test");

    ax25_physical_t phys;
    ax25_physical_init(&phys);
    DEBUG_PRINT("Physical layer initialized");

    phys.ptt_control = ptt_control_callback;
    phys.carrier_detect = carrier_detect_callback;
    phys.send_data = send_data_callback;

    phys.txdely_10ms = 30;  // 300ms TX delay
    phys.persist = 255;      // Always transmit
    phys.axhang_10ms = 0;
    DEBUG_VAR("TXDELAY configured (ticks)", phys.txdely_10ms);

    reset_ptt_state();
    simulated_carrier = false;
    DEBUG_BOOL("Initial carrier state", simulated_carrier);

    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("Frame queued", frame, sizeof(frame));

    uint32_t tick = 0;
    uint32_t ptt_on_tick = 0;
    uint32_t data_start_tick = 0;

    // Find when PTT turns on
    DEBUG_PRINT("Waiting for PTT on");
    while (tick < 100 && ptt_on_tick == 0) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick);DEBUG_BOOL("PTT state", ptt_state);
        if (ptt_state && ptt_on_tick == 0) {
            ptt_on_tick = tick;
            DEBUG_VAR("PTT on at tick", ptt_on_tick);
        }
        tick++;
    }

    TEST_ASSERT(ptt_on_tick > 0, "PTT activated", ptt_on_tick == 0);

    // Find when data transmission starts
    DEBUG_PRINT("Waiting for data transmission");
    while (tick < ptt_on_tick + 100 && data_start_tick == 0) {
        size_t prev_len = captured_len;
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick);DEBUG_VAR("Captured length", captured_len);
        if (captured_len > prev_len && data_start_tick == 0) {
            data_start_tick = tick;
            DEBUG_VAR("Data transmission started at tick", data_start_tick);
        }
        tick++;
    }

    uint32_t delay = data_start_tick - ptt_on_tick;
    DEBUG_VAR("Delay between PTT and data (ticks)", delay);DEBUG_BOOL("Delay >= configured TXDELAY", delay >= 30);

    TEST_ASSERT(delay >= 30, "TXDELAY enforced between PTT and data", delay);

    DEBUG_PRINT("T103 TXDELAY timer test completed");
    return 0;
}

static int test_t104_axdelay_timer(void) {
    printf("\n--- test_t104_axdelay_timer (T104 digipeater delay) ---\n");
    DEBUG_PRINT("Starting T104 AXDELAY timer test");

    ax25_physical_t phys;
    ax25_physical_init(&phys);
    DEBUG_PRINT("Physical layer initialized");

    phys.ptt_control = ptt_control_callback;
    phys.carrier_detect = carrier_detect_callback;
    phys.send_data = send_data_callback;

    phys.txdely_10ms = 10;   // Normal delay
    phys.axdelay_10ms = 50;  // 500ms digipeat delay
    phys.persist = 255;
    phys.axhang_10ms = 0;
    DEBUG_VAR("TXDELAY configured (ticks)", phys.txdely_10ms);DEBUG_VAR("AXDELAY configured (ticks)", phys.axdelay_10ms);

    reset_ptt_state();
    simulated_carrier = false;
    DEBUG_BOOL("Initial carrier state", simulated_carrier);

    // Queue normal frame first
    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("Normal frame queued", frame, sizeof(frame));

    uint32_t tick = 0;

    // Transmit normal frame
    DEBUG_PRINT("Transmitting normal frame");
    while (phys.tx_active && tick < 100) {
        ax25_physical_tick(&phys, tick++);
        DEBUG_VAR("Tick", tick);DEBUG_STATE("Physical state", phys.state);
    }

    uint32_t first_tx_end = tick;
    DEBUG_VAR("First transmission ended at tick", first_tx_end);

    // Queue digipeated frame
    reset_ptt_state();
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), true);  // is_digipeat=true
    DEBUG_PRINT("Digipeated frame queued");

    // Find when second transmission starts
    uint32_t second_ptt_on = 0;
    DEBUG_PRINT("Waiting for second transmission");
    while (tick < first_tx_end + 100 && second_ptt_on == 0) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick);DEBUG_BOOL("PTT state", ptt_state);
        if (ptt_state && second_ptt_on == 0) {
            second_ptt_on = tick;
            DEBUG_VAR("Second PTT on at tick", second_ptt_on);
        }
        tick++;
    }

    uint32_t digipeat_delay = second_ptt_on - first_tx_end;
    DEBUG_VAR("Digipeat delay (ticks)", digipeat_delay);DEBUG_BOOL("Delay >= AXDELAY", digipeat_delay >= 50);

    TEST_ASSERT(digipeat_delay >= 50, "AXDELAY enforced for digipeated frame", digipeat_delay);

    DEBUG_PRINT("T104 AXDELAY timer test completed");
    return 0;
}

static int test_t105_remote_sync_timer(void) {
    printf("\n--- test_t105_remote_sync_timer (T105 remote sync delay) ---\n");
    DEBUG_PRINT("Starting T105 remote sync timer test");

    ax25_physical_t phys;
    ax25_physical_init(&phys);
    DEBUG_PRINT("Physical layer initialized");

    phys.ptt_control = ptt_control_callback;
    phys.carrier_detect = carrier_detect_callback;
    phys.send_data = send_data_callback;

    phys.remote_sync_10ms = 20;  // 200ms remote sync
    phys.txdely_10ms = 0;
    phys.persist = 255;
    phys.axhang_10ms = 0;
    DEBUG_VAR("Remote sync configured (ticks)", phys.remote_sync_10ms);

    reset_ptt_state();
    simulated_carrier = false;
    DEBUG_BOOL("Initial carrier state", simulated_carrier);

    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("Frame queued", frame, sizeof(frame));

    uint32_t tick = 0;
    bool entered_remote_sync = false;

    // Check if REMOTE_SYNC state is entered
    DEBUG_PRINT("Checking for REMOTE_SYNC state");
    while (tick < 50) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick);DEBUG_STATE("Physical state", phys.state);
        if (phys.state == PHYS_REMOTE_SYNC) {
            entered_remote_sync = true;
            DEBUG_VAR("Entered REMOTE_SYNC at tick", tick);
            break;
        }
        tick++;
    }

    DEBUG_BOOL("Entered REMOTE_SYNC", entered_remote_sync);
    TEST_ASSERT(entered_remote_sync, "Entered REMOTE_SYNC state", !entered_remote_sync);

    DEBUG_PRINT("T105 remote sync timer test completed");
    return 0;
}

static int test_t106_transmission_limit(void) {
    printf("\n--- test_t106_transmission_limit (T106 10-minute limit) ---\n");
    DEBUG_PRINT("Starting T106 transmission limit test");

    ax25_physical_t phys;
    ax25_physical_init(&phys);
    DEBUG_PRINT("Physical layer initialized");

    phys.ptt_control = ptt_control_callback;
    phys.carrier_detect = carrier_detect_callback;
    phys.send_data = send_data_callback;

    // Set a very short limit for testing (100ms = 10 ticks)
    phys.max_tx_duration_10ms = 10;
    phys.txdely_10ms = 0;
    phys.persist = 255;
    phys.axhang_10ms = 0;
    DEBUG_VAR("Max TX duration configured (ticks)", phys.max_tx_duration_10ms);

    reset_ptt_state();
    simulated_carrier = false;
    DEBUG_BOOL("Initial carrier state", simulated_carrier);

    // Queue multiple frames
    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    for (int i = 0; i < 5; i++) {
        ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
        DEBUG_VAR("Frame queued", i);
    }

    uint32_t tick = 0;

    // Run until limit or timeout
    DEBUG_PRINT("Running transmission until limit");
    while (phys.tx_active && tick < 50) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick);DEBUG_BOOL("TX active", phys.tx_active);DEBUG_STATE("Physical state", phys.state);
        tick++;
    }

    DEBUG_BOOL("Transmission stopped", !phys.tx_active);DEBUG_BOOL("PTT released", ptt_state == false);DEBUG_STATE("Final state", phys.state);

    TEST_ASSERT(!phys.tx_active, "Transmission stopped after T106 limit", phys.tx_active);
    TEST_ASSERT(ptt_state == false, "PTT released after limit", ptt_state);
    TEST_ASSERT(phys.state == PHYS_IDLE, "Returned to IDLE state", phys.state);

    DEBUG_PRINT("T106 transmission limit test completed");
    return 0;
}

static int test_t107_anti_hogging(void) {
    printf("\n--- test_t107_anti_hogging (T107 anti-hogging limit) ---\n");
    DEBUG_PRINT("Starting T107 anti-hogging test");

    ax25_physical_t phys;
    ax25_physical_init(&phys);
    DEBUG_PRINT("Physical layer initialized");

    phys.ptt_control = ptt_control_callback;
    phys.carrier_detect = carrier_detect_callback;
    phys.send_data = send_data_callback;

    // Set anti-hog limit to 50ms (5 ticks)
    phys.anti_hog_10ms = 5;
    phys.txdely_10ms = 0;
    phys.persist = 255;
    phys.axhang_10ms = 2;  // Short hang time
    DEBUG_VAR("Anti-hog limit configured (ticks)", phys.anti_hog_10ms);

    reset_ptt_state();
    simulated_carrier = false;
    DEBUG_BOOL("Initial carrier state", simulated_carrier);

    // Queue multiple frames
    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    for (int i = 0; i < 3; i++) {
        ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
        DEBUG_VAR("Frame queued", i);
    }

    uint32_t tick = 0;
    uint32_t transmission_breaks = 0;

    // Monitor for transmission breaks due to anti-hog
    bool was_transmitting = false;
    DEBUG_PRINT("Monitoring for transmission breaks");
    while (tick < 50) {
        ax25_physical_tick(&phys, tick);
        DEBUG_VAR("Tick", tick);DEBUG_BOOL("TX active", phys.tx_active);

        if (phys.tx_active && !was_transmitting) {
            was_transmitting = true;
            DEBUG_PRINT("Transmission started");
        } else if (!phys.tx_active && was_transmitting) {
            transmission_breaks++;
            was_transmitting = false;
            DEBUG_VAR("Transmission break count", transmission_breaks);
        }
        tick++;
    }

    DEBUG_VAR("Total transmission breaks", transmission_breaks);DEBUG_BOOL("Anti-hogging triggered", transmission_breaks > 0);

    TEST_ASSERT(transmission_breaks > 0, "Anti-hogging caused transmission break", transmission_breaks);

    DEBUG_PRINT("T107 anti-hogging test completed");
    return 0;
}

// ============================================================================
// FILE: test_ax25_timers.c - MODIFIED WITH ENHANCED DEBUG
// ============================================================================

static int test_t108_receiver_startup(void) {
    printf("\n--- test_t108_receiver_startup (T108 receiver startup time) ---\n");
    DEBUG_PRINT("Starting T108 receiver startup test");

    ax25_physical_t phys;
    ax25_physical_init(&phys);
    DEBUG_PRINT("Physical layer initialized");

    phys.ptt_control = ptt_control_callback;
    phys.carrier_detect = carrier_detect_callback;
    phys.send_data = send_data_callback;

    // Set receiver startup time to 40ms (4 ticks)
    phys.rx_startup_10ms = 4;
    phys.txdely_10ms = 0;
    phys.persist = 255;
    phys.axhang_10ms = 0;
    phys.interframe_flags = 1;  // Minimal interframe delay
    DEBUG_VAR("RX startup time configured (ticks)", phys.rx_startup_10ms);DEBUG_VAR("TX delay (ticks)", phys.txdely_10ms);DEBUG_VAR("AX hang (ticks)", phys.axhang_10ms);DEBUG_VAR("Interframe flags", phys.interframe_flags);

    reset_ptt_state();
    simulated_carrier = false;
    DEBUG_BOOL("Initial carrier state", simulated_carrier);

    // Queue and transmit first frame
    uint8_t frame[20] = { 0x7E, 0x01, 0x02, 0x03, 0x7E };
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_FRAME("First frame queued", frame, sizeof(frame));

    uint32_t tick = 0;
    DEBUG_PRINT("Transmitting first frame");DEBUG_PRINT("=== Starting transmission loop ===");
    while (tick < 50) {
        DEBUG_VAR("Loop tick", tick);DEBUG_STATE("State before tick", phys.state);DEBUG_BOOL("TX active before tick", phys.tx_active);DEBUG_BOOL("RX warmup required before tick", phys.rx_warmup_required);DEBUG_VAR("Next action tick", phys.next_action_tick_10ms);DEBUG_VAR("Last unkey tick", phys.last_unkey_tick_10ms);

        ax25_physical_tick(&phys, tick);

        DEBUG_STATE("State after tick", phys.state);DEBUG_BOOL("TX active after tick", phys.tx_active);DEBUG_BOOL("RX warmup required after tick", phys.rx_warmup_required);DEBUG_VAR("Queue head", phys.queue_head);DEBUG_VAR("Queue tail", phys.queue_tail);

        if (!phys.tx_active) {
            DEBUG_PRINT("TX became inactive, breaking loop");
            break;
        }
        tick++;
    }

    uint32_t first_tx_end = tick;
    DEBUG_VAR("First transmission ended at tick", first_tx_end);DEBUG_STATE("State after first TX", phys.state);DEBUG_BOOL("RX warmup required after first TX", phys.rx_warmup_required);DEBUG_VAR("Last unkey tick after first TX", phys.last_unkey_tick_10ms);
    TEST_ASSERT(!phys.tx_active, "First transmission completed", phys.tx_active);

    // Queue second frame immediately
    ax25_physical_queue_frame(&phys, frame, sizeof(frame), false);
    DEBUG_PRINT("Second frame queued");DEBUG_VAR("Queue head after 2nd queue", phys.queue_head);DEBUG_VAR("Queue tail after 2nd queue", phys.queue_tail);

    // Advance time and check when second transmission starts
    uint32_t second_tx_start = 0;
    DEBUG_PRINT("Waiting for second transmission");DEBUG_PRINT("=== Starting second transmission detection loop ===");
    while (tick < first_tx_end + 20) {
        DEBUG_VAR("Loop tick", tick);DEBUG_STATE("State before tick", phys.state);DEBUG_BOOL("TX active before tick", phys.tx_active);DEBUG_BOOL("RX warmup required before tick", phys.rx_warmup_required);DEBUG_VAR("Next action tick", phys.next_action_tick_10ms);DEBUG_VAR("Last unkey tick", phys.last_unkey_tick_10ms);

        ax25_physical_tick(&phys, tick);

        DEBUG_STATE("State after tick", phys.state);DEBUG_BOOL("TX active after tick", phys.tx_active);DEBUG_VAR("Transmit count", transmit_count);

        if (phys.tx_active && second_tx_start == 0) {
            second_tx_start = tick;
            DEBUG_VAR("Second transmission started at tick", second_tx_start);
            break;
        }
        tick++;
    }

    if (second_tx_start == 0) {
        DEBUG_PRINT("ERROR: Second transmission never started!");DEBUG_STATE("Final state", phys.state);DEBUG_VAR("Final tick", tick);
        TEST_ASSERT(false, "Second transmission should have started", 0);
    }

    uint32_t rx_warmup_delay = second_tx_start - first_tx_end;
    DEBUG_VAR("RX warmup delay (ticks)", rx_warmup_delay);DEBUG_VAR("Expected minimum delay (ticks)", phys.rx_startup_10ms);DEBUG_BOOL("Delay >= configured RX startup", rx_warmup_delay >= 4);

    TEST_ASSERT(rx_warmup_delay >= 4, "RX startup time enforced between transmissions", rx_warmup_delay);

    DEBUG_PRINT("T108 receiver startup test completed");
    return 0;
}

static int test_tr210_segmentation_timeout(void) {
    printf("\n--- test_tr210_segmentation_timeout (TR210 segment reassembly timeout) ---\n");
    DEBUG_PRINT("Starting TR210 segmentation timeout test");

    ax25_segmenter_t seg;
    uint8_t err = ax25_segmenter_init(&seg, 256);
    TEST_ASSERT(err == 0, "Segmenter init succeeded", err);DEBUG_VAR("Segmenter init error code", err);

    seg.on_reassembly_error = NULL;  // We'll check state directly
    DEBUG_PRINT("Segmenter initialized without error callback");

    // Simulate first segment
    uint8_t first_seg[10];
    first_seg[0] = 0x80;  // First segment flag + sequence 0
    memcpy(first_seg + 1, "TEST", 4);
    DEBUG_FRAME("First segment", first_seg, 5);

    ax25_segmenter_receive(&seg, first_seg, 5, AX25_PID_SEGMENT_FRAGMENT, 0);
    DEBUG_STATE("Segmenter state after first segment", seg.state);
    TEST_ASSERT(seg.state == SEG_STATE_REASSEMBLING, "Started reassembly", seg.state);

    // Advance time past TR210 timeout (3000 ticks)
    DEBUG_PRINT("Advancing time past TR210 timeout");
    for (uint32_t tick = 0; tick <= 3100; tick++) {
        ax25_segmenter_tick(&seg, tick);
        if (tick % 500 == 0) {
            DEBUG_VAR("Tick", tick);DEBUG_STATE("Segmenter state", seg.state);DEBUG_VAR("Buffer used", seg.rx_buffer_used);
        }
    }

    DEBUG_STATE("Final segmenter state", seg.state);DEBUG_VAR("Final buffer used", seg.rx_buffer_used);

    TEST_ASSERT(seg.state == SEG_STATE_IDLE, "Timeout caused return to IDLE", seg.state);
    TEST_ASSERT(seg.rx_buffer_used == 0, "Buffer cleared on timeout", seg.rx_buffer_used);

    DEBUG_PRINT("TR210 segmentation timeout test completed");
    return 0;
}

// ============================================================================
// Main Test Runner
// ============================================================================

int test_ax25_timers_main(void) {
    int result = 0;

    printf("\n================================================================================\n");
    printf("Starting AX.25 v2.2 Timer Tests\n");
    printf("================================================================================\n");

    printf("\n--- State Machine Timers (T1, T2, T3) ---\n");
    result |= test_t1_timer_expiration();
    result |= test_t1_timer_reset_on_ack();
    result |= test_t2_response_delay();
    result |= test_t3_inactive_link_polling();

    printf("\n--- Physical Layer Timers (T100-T108) ---\n");
    result |= test_t100_axhang_timer();
    result |= test_t102_slottime_persistence();
    result |= test_t103_txdelay_timer();
    result |= test_t104_axdelay_timer();
    result |= test_t105_remote_sync_timer();
    result |= test_t106_transmission_limit();
    result |= test_t107_anti_hogging();
    result |= test_t108_receiver_startup();

    printf("\n--- Segmentation Timer (TR210) ---\n");
    result |= test_tr210_segmentation_timeout();

    printf("\n================================================================================\n");
    printf("AX.25 Timer Tests Completed. %s\n", result == 0 ? "All tests passed" : "Some tests failed");
    printf("\n================================================================================\n\n");

    return result;
}
