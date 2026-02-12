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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "test_common.h"
#include "ax25_segmenter.h"

static uint32_t assert_count = 0;

// est helper structures and globals
typedef struct {
    uint8_t segments[64][260];
    uint16_t segment_lengths[64];
    uint8_t segment_pids[64];
    uint8_t segment_count;
} test_segment_capture_t;

typedef struct {
    uint8_t data[2048];
    uint16_t length;
    uint8_t pid;
    bool called;
} test_reassembly_result_t;

typedef struct {
    ax25_seg_error_t error;
    bool called;
} test_error_result_t;

static test_segment_capture_t g_captured_segments;
static test_reassembly_result_t g_reassembly_result;
static test_error_result_t g_error_result;

// Test callbacks
static void test_transmit_iframe(uint8_t *data, uint16_t len, uint8_t pid, void *user_data) {
    if (g_captured_segments.segment_count < 64) {
        memcpy(g_captured_segments.segments[g_captured_segments.segment_count], data, len);
        g_captured_segments.segment_lengths[g_captured_segments.segment_count] = len;
        g_captured_segments.segment_pids[g_captured_segments.segment_count] = pid;
        g_captured_segments.segment_count++;
    }
}

static void test_on_reassembly_complete(uint8_t *data, uint16_t len, uint8_t pid, void *user_data) {
    memcpy(g_reassembly_result.data, data, len);
    g_reassembly_result.length = len;
    g_reassembly_result.pid = pid;
    g_reassembly_result.called = true;
}

static void test_on_reassembly_error(ax25_seg_error_t error, void *user_data) {
    g_error_result.error = error;
    g_error_result.called = true;
}

// Reset test globals
static void reset_test_globals(void) {
    memset(&g_captured_segments, 0, sizeof(g_captured_segments));
    memset(&g_reassembly_result, 0, sizeof(g_reassembly_result));
    memset(&g_error_result, 0, sizeof(g_error_result));
}

// Test 1: Initialization
static int test_segmenter_init(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_init ---\n");
    reset_test_globals();

    ax25_segmenter_t seg;

    // Test successful init
    uint8_t err = ax25_segmenter_init(&seg, 256);
    TEST_ASSERT(err == 0, "Segmenter init succeeds", err);
    TEST_ASSERT(seg.state == SEG_STATE_IDLE, "Initial state is IDLE", 0);
    TEST_ASSERT(seg.segment_size == 254, "Segment size is N1-2 (256-2=254)", 0);
    TEST_ASSERT(seg.rx_gap_threshold == 8, "Gap threshold initialized to 8", 0);

    // Test with small N1
    err = ax25_segmenter_init(&seg, 2);
    TEST_ASSERT(err == 0, "Init with N1=2 succeeds", err);
    TEST_ASSERT(seg.segment_size == 254, "Segment size falls back to 254", 0);

    // Test NULL pointer
    err = ax25_segmenter_init(NULL, 256);
    TEST_ASSERT(err == 1, "Init with NULL returns error 1", err);

    return 0;
}

// Test 2: Small data (no segmentation)
static int test_segmenter_small_data(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_small_data ---\n");
    reset_test_globals();

    ax25_segmenter_t seg;
    ax25_segmenter_init(&seg, 256);

    seg.transmit_iframe = test_transmit_iframe;
    seg.user_data = NULL;

    // Small data that fits in one segment
    uint8_t test_data[100];
    for (int i = 0; i < 100; i++) {
        test_data[i] = (uint8_t) i;
    }

    uint8_t err = ax25_segmenter_send(&seg, test_data, 100, 0xF0);
    TEST_ASSERT(err == 0, "Send small data succeeds", err);
    TEST_ASSERT(g_captured_segments.segment_count == 1, "One segment transmitted", 0);
    TEST_ASSERT(g_captured_segments.segment_pids[0] == AX25_PID_SEGMENT_FRAGMENT, "Segment has PID 0x08", 0);

    // Check segment format: [header][original_pid][data]
    uint8_t header = g_captured_segments.segments[0][0];
    TEST_ASSERT((header & 0x80) != 0, "First segment flag set", 0);
    TEST_ASSERT((header & 0x40) != 0, "Last segment flag set", 0);
    TEST_ASSERT((header & 0x3F) == 0, "Sequence number is 0", 0);
    TEST_ASSERT(g_captured_segments.segments[0][1] == 0xF0, "Original PID preserved", 0);

    return 0;
}

// Test 3: Medium data segmentation
static int test_segmenter_medium_data(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_medium_data ---\n");
    reset_test_globals();

    ax25_segmenter_t seg;
    ax25_segmenter_init(&seg, 256);

    seg.transmit_iframe = test_transmit_iframe;
    seg.user_data = NULL;

    // Data that requires 3 segments (254*2 < 600 < 254*3)
    uint8_t test_data[600];
    for (int i = 0; i < 600; i++) {
        test_data[i] = (uint8_t) (i & 0xFF);
    }

    uint8_t err = ax25_segmenter_send(&seg, test_data, 600, 0xF0);
    TEST_ASSERT(err == 0, "Send medium data succeeds", err);
    TEST_ASSERT(g_captured_segments.segment_count == 3, "Three segments transmitted", 0);

    // Check first segment
    uint8_t header0 = g_captured_segments.segments[0][0];
    TEST_ASSERT((header0 & 0x80) != 0, "Segment 0: First flag set", 0);
    TEST_ASSERT((header0 & 0x40) == 0, "Segment 0: Last flag not set", 0);
    TEST_ASSERT((header0 & 0x3F) == 0, "Segment 0: Sequence is 0", 0);

    // Check middle segment
    uint8_t header1 = g_captured_segments.segments[1][0];
    TEST_ASSERT((header1 & 0x80) == 0, "Segment 1: First flag not set", 0);
    TEST_ASSERT((header1 & 0x40) == 0, "Segment 1: Last flag not set", 0);
    TEST_ASSERT((header1 & 0x3F) == 1, "Segment 1: Sequence is 1", 0);

    // Check last segment
    uint8_t header2 = g_captured_segments.segments[2][0];
    TEST_ASSERT((header2 & 0x80) == 0, "Segment 2: First flag not set", 0);
    TEST_ASSERT((header2 & 0x40) != 0, "Segment 2: Last flag set", 0);
    TEST_ASSERT((header2 & 0x3F) == 2, "Segment 2: Sequence is 2", 0);

    return 0;
}

// Test 4: In-order reassembly
static int test_segmenter_inorder_reassembly(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_inorder_reassembly ---\n");
    reset_test_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    ax25_segmenter_init(&tx_seg, 256);
    ax25_segmenter_init(&rx_seg, 256);

    tx_seg.transmit_iframe = test_transmit_iframe;
    rx_seg.on_reassembly_complete = test_on_reassembly_complete;
    rx_seg.user_data = NULL;

    // Original data
    uint8_t original_data[600];
    for (int i = 0; i < 600; i++) {
        original_data[i] = (uint8_t) (i & 0xFF);
    }

    // Segment and send
    uint8_t err = ax25_segmenter_send(&tx_seg, original_data, 600, 0xF0);
    TEST_ASSERT(err == 0, "Segmentation succeeds", err);

    // Receive segments in order
    uint32_t current_tick = 0;
    for (uint8_t i = 0; i < g_captured_segments.segment_count; i++) {
        ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[i], g_captured_segments.segment_lengths[i], g_captured_segments.segment_pids[i],
                current_tick);
    }

    // Check reassembly completed
    TEST_ASSERT(g_reassembly_result.called == true, "Reassembly complete callback called", 0);
    TEST_ASSERT(g_reassembly_result.length == 600, "Reassembled length matches original", 0);
    TEST_ASSERT(g_reassembly_result.pid == 0xF0, "Original PID preserved", 0);

    int data_match = memcmp(g_reassembly_result.data, original_data, 600);
    TEST_ASSERT(data_match == 0, "Reassembled data matches original", 0);
    TEST_ASSERT(rx_seg.state == SEG_STATE_IDLE, "State returned to IDLE", 0);

    return 0;
}

// Test 5: Out-of-order reassembly
static int test_segmenter_out_of_order_reassembly(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_out_of_order_reassembly ---\n");
    reset_test_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    ax25_segmenter_init(&tx_seg, 256);
    ax25_segmenter_init(&rx_seg, 256);

    tx_seg.transmit_iframe = test_transmit_iframe;
    rx_seg.on_reassembly_complete = test_on_reassembly_complete;
    rx_seg.user_data = NULL;

    // Original data requiring 4 segments
    uint8_t original_data[800];
    for (int i = 0; i < 800; i++) {
        original_data[i] = (uint8_t) (i & 0xFF);
    }

    uint8_t err = ax25_segmenter_send(&tx_seg, original_data, 800, 0xF0);
    TEST_ASSERT(err == 0, "Segmentation succeeds", err);
    TEST_ASSERT(g_captured_segments.segment_count == 4, "Four segments created", 0);

    // Receive out of order: 0, 2, 1, 3
    uint32_t current_tick = 0;
    uint8_t receive_order[] = { 0, 2, 1, 3 };

    for (int i = 0; i < 4; i++) {
        uint8_t idx = receive_order[i];
        ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[idx], g_captured_segments.segment_lengths[idx], g_captured_segments.segment_pids[idx],
                current_tick);
    }

    // Check reassembly completed after all segments received
    TEST_ASSERT(g_reassembly_result.called == true, "Reassembly complete after out-of-order", 0);
    TEST_ASSERT(g_reassembly_result.length == 800, "Reassembled length correct", 0);

    int data_match = memcmp(g_reassembly_result.data, original_data, 800);
    TEST_ASSERT(data_match == 0, "Out-of-order reassembled data matches", 0);

    return 0;
}

// Test 6: Duplicate segment handling
static int test_segmenter_duplicate_segments(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_duplicate_segments ---\n");
    reset_test_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    ax25_segmenter_init(&tx_seg, 256);
    ax25_segmenter_init(&rx_seg, 256);

    tx_seg.transmit_iframe = test_transmit_iframe;
    rx_seg.on_reassembly_complete = test_on_reassembly_complete;
    rx_seg.on_reassembly_error = test_on_reassembly_error;
    rx_seg.user_data = NULL;

    uint8_t original_data[400];
    for (int i = 0; i < 400; i++) {
        original_data[i] = (uint8_t) (i & 0xFF);
    }

    uint8_t err = ax25_segmenter_send(&tx_seg, original_data, 400, 0xF0);
    TEST_ASSERT(err == 0, "Segmentation succeeds", err);

    uint32_t current_tick = 0;
    // Receive first segment twice
    ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[0], g_captured_segments.segment_lengths[0], g_captured_segments.segment_pids[0], current_tick);

    ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[0], g_captured_segments.segment_lengths[0], g_captured_segments.segment_pids[0], current_tick);

    // Complete with remaining segments
    for (uint8_t i = 1; i < g_captured_segments.segment_count; i++) {
        ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[i], g_captured_segments.segment_lengths[i], g_captured_segments.segment_pids[i],
                current_tick);
    }

    TEST_ASSERT(g_reassembly_result.called == true, "Reassembly completes with duplicates", 0);
    TEST_ASSERT(g_error_result.called == false, "No error from duplicate", 0);
    TEST_ASSERT(g_reassembly_result.length == 400, "Length correct despite duplicate", 0);

    return 0;
}

// Test 7: Timeout handling (TR210)
static int test_segmenter_timeout(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_timeout ---\n");
    reset_test_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    ax25_segmenter_init(&tx_seg, 256);
    ax25_segmenter_init(&rx_seg, 256);

    tx_seg.transmit_iframe = test_transmit_iframe;
    rx_seg.on_reassembly_complete = test_on_reassembly_complete;
    rx_seg.on_reassembly_error = test_on_reassembly_error;
    rx_seg.user_data = NULL;

    uint8_t original_data[400];
    for (int i = 0; i < 400; i++) {
        original_data[i] = (uint8_t) (i & 0xFF);
    }

    uint8_t err = ax25_segmenter_send(&tx_seg, original_data, 400, 0xF0);
    TEST_ASSERT(err == 0, "Segmentation succeeds", err);

    // Receive only first segment
    uint32_t current_tick = 0;
    ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[0], g_captured_segments.segment_lengths[0], g_captured_segments.segment_pids[0], current_tick);

    TEST_ASSERT(rx_seg.state == SEG_STATE_REASSEMBLING, "State is REASSEMBLING", 0);

    // Advance time beyond TR210 timeout (30 seconds = 3000 ticks)
    current_tick += 3100;
    ax25_segmenter_tick(&rx_seg, current_tick);

    // Check timeout triggered
    TEST_ASSERT(g_error_result.called == true, "Timeout error callback called", 0);
    TEST_ASSERT(g_error_result.error == AX25_SEG_ERROR_TIMEOUT, "Error is TIMEOUT", 0);
    TEST_ASSERT(rx_seg.state == SEG_STATE_IDLE, "State reset to IDLE after timeout", 0);
    TEST_ASSERT(g_reassembly_result.called == false, "Reassembly not completed on timeout", 0);

    return 0;
}

// Test 8: Buffer overflow detection
static int test_segmenter_buffer_overflow(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_buffer_overflow ---\n");
    reset_test_globals();

    ax25_segmenter_t rx_seg;
    ax25_segmenter_init(&rx_seg, 256);

    rx_seg.on_reassembly_complete = test_on_reassembly_complete;
    rx_seg.on_reassembly_error = test_on_reassembly_error;
    rx_seg.user_data = NULL;

    // Create segments that would overflow the 2048-byte buffer
    uint8_t large_segment[260];

    // First segment - header byte
    large_segment[0] = 0x80;  // First segment, seq 0
    large_segment[1] = 0xF0;  // PID
    memset(&large_segment[2], 0xAA, 258);

    uint32_t current_tick = 0;
    ax25_segmenter_receive(&rx_seg, large_segment, 260, AX25_PID_SEGMENT_FRAGMENT, current_tick);
    TEST_ASSERT(rx_seg.state == SEG_STATE_REASSEMBLING, "Started reassembly", 0);

    // Send segments until overflow (2048 / 258 ≈ 8 segments)
    for (uint8_t i = 1; i < 10; i++) {
        large_segment[0] = i;  // Sequence only
        if (i == 9) {
            large_segment[0] |= 0x40;  // Last segment flag
        }
        large_segment[1] = 0xF0;

        ax25_segmenter_receive(&rx_seg, large_segment, 260, AX25_PID_SEGMENT_FRAGMENT, current_tick);

        if (g_error_result.called) {
            break;
        }
    }

    TEST_ASSERT(g_error_result.called == true, "Overflow error detected", 0);
    TEST_ASSERT(g_error_result.error == AX25_SEG_ERROR_OVERFLOW, "Error is OVERFLOW", 0);
    TEST_ASSERT(rx_seg.state == SEG_STATE_IDLE, "State reset after overflow", 0);

    return 0;
}

// Test 9: Gap detection threshold
static int test_segmenter_gap_threshold(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_gap_threshold ---\n");
    reset_test_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    ax25_segmenter_init(&tx_seg, 256);
    ax25_segmenter_init(&rx_seg, 256);

    tx_seg.transmit_iframe = test_transmit_iframe;
    rx_seg.on_reassembly_complete = test_on_reassembly_complete;
    rx_seg.on_reassembly_error = test_on_reassembly_error;
    rx_seg.user_data = NULL;

    // Create data requiring 12 segments
    uint8_t original_data[3000];
    for (int i = 0; i < 3000; i++) {
        original_data[i] = (uint8_t) (i & 0xFF);
    }

    uint8_t err = ax25_segmenter_send(&tx_seg, original_data, 3000, 0xF0);
    TEST_ASSERT(err == 0, "Segmentation succeeds", err);

    uint32_t current_tick = 0;
    // Receive first segment (seq 0)
    ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[0], g_captured_segments.segment_lengths[0], g_captured_segments.segment_pids[0], current_tick);

    // Skip segment 1, receive segments 2-10 (9 segments beyond gap)
    // This exceeds gap threshold of 8
    for (uint8_t i = 2; i < 11 && i < g_captured_segments.segment_count; i++) {
        ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[i], g_captured_segments.segment_lengths[i], g_captured_segments.segment_pids[i],
                current_tick);
    }

    // Gap threshold exceeded
    TEST_ASSERT(g_error_result.called == true, "Gap threshold error triggered", 0);
    TEST_ASSERT(g_error_result.error == AX25_SEG_ERROR_SEQUENCE, "Error is SEQUENCE", 0);
    TEST_ASSERT(rx_seg.state == SEG_STATE_IDLE, "State reset after gap threshold", 0);

    return 0;
}

// Test 10: Large data segmentation
static int test_segmenter_large_data(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_large_data ---\n");
    reset_test_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    ax25_segmenter_init(&tx_seg, 256);
    ax25_segmenter_init(&rx_seg, 256);

    tx_seg.transmit_iframe = test_transmit_iframe;
    rx_seg.on_reassembly_complete = test_on_reassembly_complete;
    rx_seg.user_data = NULL;

    // Max reassembly buffer is 2048 bytes
    uint8_t original_data[2000];
    for (int i = 0; i < 2000; i++) {
        original_data[i] = (uint8_t) (i & 0xFF);
    }

    uint8_t err = ax25_segmenter_send(&tx_seg, original_data, 2000, 0xCC);
    TEST_ASSERT(err == 0, "Large data segmentation succeeds", err);

    uint8_t expected_segments = (2000 + 253) / 254;  // Ceiling division
    TEST_ASSERT(g_captured_segments.segment_count == expected_segments, "Correct number of segments", 0);

    // Receive all segments
    uint32_t current_tick = 0;
    for (uint8_t i = 0; i < g_captured_segments.segment_count; i++) {
        ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[i], g_captured_segments.segment_lengths[i], g_captured_segments.segment_pids[i],
                current_tick);
    }

    TEST_ASSERT(g_reassembly_result.called == true, "Large data reassembly complete", 0);
    TEST_ASSERT(g_reassembly_result.length == 2000, "Reassembled length correct", 0);

    int data_match = memcmp(g_reassembly_result.data, original_data, 2000);
    TEST_ASSERT(data_match == 0, "Large data matches", 0);

    return 0;
}

// Test 11: Invalid inputs
static int test_segmenter_invalid_inputs(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_invalid_inputs ---\n");
    reset_test_globals();

    ax25_segmenter_t seg;
    ax25_segmenter_init(&seg, 256);

    seg.transmit_iframe = test_transmit_iframe;
    seg.user_data = NULL;

    uint8_t test_data[100];

    // NULL segmenter pointer
    uint8_t err = ax25_segmenter_send(NULL, test_data, 100, 0xF0);
    TEST_ASSERT(err == 1, "Send with NULL segmenter returns error 1", err);

    // NULL data pointer
    err = ax25_segmenter_send(&seg, NULL, 100, 0xF0);
    TEST_ASSERT(err == 1, "Send with NULL data returns error 1", err);

    // Zero length
    err = ax25_segmenter_send(&seg, test_data, 0, 0xF0);
    TEST_ASSERT(err == 1, "Send with zero length returns error 1", err);

    // Data too large (>2048 bytes)
    err = ax25_segmenter_send(&seg, test_data, 2049, 0xF0);
    TEST_ASSERT(err == 2, "Send with oversized data returns error 2", err);

    // NULL transmit callback
    seg.transmit_iframe = NULL;
    err = ax25_segmenter_send(&seg, test_data, 100, 0xF0);
    TEST_ASSERT(err == 3, "Send without transmit callback returns error 3", err);

    return 0;
}

// Test 12: Segment header encoding/decoding
static int test_segmenter_header_format(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_header_format ---\n");
    reset_test_globals();

    ax25_segmenter_t tx_seg;
    ax25_segmenter_init(&tx_seg, 256);
    tx_seg.transmit_iframe = test_transmit_iframe;

    uint8_t test_data[100];
    for (int i = 0; i < 100; i++) {
        test_data[i] = (uint8_t) i;
    }

    ax25_segmenter_send(&tx_seg, test_data, 100, 0xF0);

    // Single segment: should have both first and last flags
    uint8_t header = g_captured_segments.segments[0][0];
    TEST_ASSERT((header & 0x80) != 0, "First segment bit (bit 7) set", 0);
    TEST_ASSERT((header & 0x40) != 0, "Last segment bit (bit 6) set", 0);
    TEST_ASSERT((header & 0x3F) == 0, "Sequence number (bits 5-0) is 0", 0);

    // Multi-segment test
    reset_test_globals();
    ax25_segmenter_init(&tx_seg, 256);
    tx_seg.transmit_iframe = test_transmit_iframe;

    uint8_t large_data[600];
    ax25_segmenter_send(&tx_seg, large_data, 600, 0xF0);

    // First segment
    header = g_captured_segments.segments[0][0];
    TEST_ASSERT((header & 0x80) != 0, "Multi: First segment has first bit", 0);
    TEST_ASSERT((header & 0x40) == 0, "Multi: First segment no last bit", 0);

    // Last segment
    uint8_t last_idx = g_captured_segments.segment_count - 1;
    header = g_captured_segments.segments[last_idx][0];
    TEST_ASSERT((header & 0x80) == 0, "Multi: Last segment no first bit", 0);
    TEST_ASSERT((header & 0x40) != 0, "Multi: Last segment has last bit", 0);
    TEST_ASSERT((header & 0x3F) == last_idx, "Multi: Last segment sequence correct", 0);

    return 0;
}

// Test 13: Out-of-order buffer management
static int test_segmenter_ooo_buffer(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_ooo_buffer ---\n");
    reset_test_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    ax25_segmenter_init(&tx_seg, 256);
    ax25_segmenter_init(&rx_seg, 256);

    tx_seg.transmit_iframe = test_transmit_iframe;
    rx_seg.on_reassembly_complete = test_on_reassembly_complete;
    rx_seg.user_data = NULL;

    // Create data with 10 segments to test buffer limits
    uint8_t original_data[2500];
    for (int i = 0; i < 2500; i++) {
        original_data[i] = (uint8_t) (i & 0xFF);
    }

    ax25_segmenter_send(&tx_seg, original_data, 2500, 0xF0);
    uint8_t total_segments = g_captured_segments.segment_count;

    uint32_t current_tick = 0;

    // Receive first segment
    ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[0], g_captured_segments.segment_lengths[0], g_captured_segments.segment_pids[0], current_tick);

    // Receive segments 2-9 out of order (missing segment 1)
    // This tests out-of-order buffering
    for (uint8_t i = 2; i < 9 && i < total_segments; i++) {
        ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[i], g_captured_segments.segment_lengths[i], g_captured_segments.segment_pids[i],
                current_tick);
    }

    TEST_ASSERT(rx_seg.ooo_count > 0, "Out-of-order segments buffered", 0);
    TEST_ASSERT(rx_seg.state == SEG_STATE_REASSEMBLING, "Still reassembling", 0);

    // Receive missing segment 1
    ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[1], g_captured_segments.segment_lengths[1], g_captured_segments.segment_pids[1], current_tick);

    // Buffer should be processed
    TEST_ASSERT(rx_seg.rx_expected_segment == 9, "Expected segment advanced", 0);

    return 0;
}

// Test 14: TR210 timer restart on segment receipt
static int test_segmenter_timer_restart(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_timer_restart ---\n");
    reset_test_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    ax25_segmenter_init(&tx_seg, 256);
    ax25_segmenter_init(&rx_seg, 256);

    tx_seg.transmit_iframe = test_transmit_iframe;
    rx_seg.on_reassembly_error = test_on_reassembly_error;
    rx_seg.on_reassembly_complete = test_on_reassembly_complete;
    rx_seg.user_data = NULL;

    uint8_t original_data[800];
    for (int i = 0; i < 800; i++) {
        original_data[i] = (uint8_t) (i & 0xFF);
    }

    ax25_segmenter_send(&tx_seg, original_data, 800, 0xF0);

    uint32_t current_tick = 0;

    // Receive first segment
    ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[0], g_captured_segments.segment_lengths[0], g_captured_segments.segment_pids[0], current_tick);

    uint32_t first_timeout = rx_seg.rx_timeout_tick;
    TEST_ASSERT(first_timeout > 0, "Timer initialized", 0);

    // Advance time but not past timeout
    current_tick += 1000;

    // Receive another segment
    ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[1], g_captured_segments.segment_lengths[1], g_captured_segments.segment_pids[1], current_tick);

    uint32_t second_timeout = rx_seg.rx_timeout_tick;
    TEST_ASSERT(second_timeout > first_timeout, "Timer restarted on segment receipt", 0);

    // Complete reassembly
    for (uint8_t i = 2; i < g_captured_segments.segment_count; i++) {
        ax25_segmenter_receive(&rx_seg, g_captured_segments.segments[i], g_captured_segments.segment_lengths[i], g_captured_segments.segment_pids[i],
                current_tick);
    }

    TEST_ASSERT(g_reassembly_result.called == true, "Reassembly completed", 0);
    TEST_ASSERT(g_error_result.called == false, "No timeout error", 0);

    return 0;
}

// Test 15: Sequence number wraparound (edge case)
static int test_segmenter_sequence_wraparound(void) {
    assert_count = 0;
    printf("\n--- test_segmenter_sequence_wraparound ---\n");
    reset_test_globals();

    ax25_segmenter_t tx_seg;
    ax25_segmenter_init(&tx_seg, 256);
    tx_seg.transmit_iframe = test_transmit_iframe;

    // Create data requiring 64 segments (max sequence number)
    // 64 * 254 = 16256 bytes, but our buffer is 2048
    // So we'll just test that sequence numbers stay within 6-bit range
    uint8_t test_data[2000];
    for (int i = 0; i < 2000; i++) {
        test_data[i] = (uint8_t) (i & 0xFF);
    }

    ax25_segmenter_send(&tx_seg, test_data, 2000, 0xF0);

    // Check all sequence numbers are within 0-63
    for (uint8_t i = 0; i < g_captured_segments.segment_count; i++) {
        uint8_t header = g_captured_segments.segments[i][0];
        uint8_t sequence = header & 0x3F;
        TEST_ASSERT(sequence == i, "Sequence numbers in correct order", 0);
        TEST_ASSERT(sequence < 64, "Sequence number within 6-bit range", 0);
    }

    return 0;
}

// Main test function
int test_ax25_segmenter_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("Starting AX.25 Segmenter Tests\n");
    printf("==================================================================================\n\n");

    result |= test_segmenter_init();
    result |= test_segmenter_small_data();
    result |= test_segmenter_medium_data();
    result |= test_segmenter_inorder_reassembly();
    result |= test_segmenter_out_of_order_reassembly();
    result |= test_segmenter_duplicate_segments();
    result |= test_segmenter_timeout();
    result |= test_segmenter_buffer_overflow();
    result |= test_segmenter_gap_threshold();
    result |= test_segmenter_large_data();
    result |= test_segmenter_invalid_inputs();
    result |= test_segmenter_header_format();
    result |= test_segmenter_ooo_buffer();
    result |= test_segmenter_timer_restart();
    result |= test_segmenter_sequence_wraparound();

    printf("\n==================================================================================\n");
    printf("AX.25 Segmenter Tests Completed. %s\n", result == 0 ? "All tests passed" : "Some tests failed");
    printf("==================================================================================\n\n");

    return result;
}
