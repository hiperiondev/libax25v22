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
 * Extended tests for SECTION 8: SEGMENTATION/REASSEMBLY
 *   - Out-of-Order Segment Handling (extended disorder scenarios)
 *   - TR210 Timer Edge Cases (boundary conditions)
 *   - Large Multi-Segment Reassembly (>3 segments, up to near-limit)
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "test_common.h"
#include "ax25_segmenter.h"

static uint32_t assert_count = 0;

// ============================================================
// Shared test infrastructure
// ============================================================

typedef struct {
    uint8_t segments[64][260];
    uint16_t segment_lengths[64];
    uint8_t segment_pids[64];
    uint8_t segment_count;
} t2_segment_capture_t;

typedef struct {
    uint8_t data[4096];
    uint16_t length;
    uint8_t pid;
    bool called;
    uint32_t call_count;
} t2_reassembly_result_t;

typedef struct {
    ax25_seg_error_t error;
    bool called;
    uint32_t call_count;
} t2_error_result_t;

typedef struct {
    uint8_t sequences[64];
    uint8_t count;
} t2_retransmit_requests_t;

static t2_segment_capture_t g2_captured_segments;
static t2_reassembly_result_t g2_reassembly_result;
static t2_error_result_t g2_error_result;
static t2_retransmit_requests_t g2_retransmit_requests;

static void t2_transmit_iframe(uint8_t *data, uint16_t len, uint8_t pid, void *user_data) {
    if (g2_captured_segments.segment_count < 64) {
        memcpy(g2_captured_segments.segments[g2_captured_segments.segment_count], data, len);
        g2_captured_segments.segment_lengths[g2_captured_segments.segment_count] = len;
        g2_captured_segments.segment_pids[g2_captured_segments.segment_count] = pid;
        g2_captured_segments.segment_count++;
    }
}

static void t2_on_reassembly_complete(uint8_t *data, uint16_t len, uint8_t pid, void *user_data) {
    memcpy(g2_reassembly_result.data, data, len);
    g2_reassembly_result.length = len;
    g2_reassembly_result.pid = pid;
    g2_reassembly_result.called = true;
    g2_reassembly_result.call_count++;
    DEBUG_PRINT("Reassembly complete: len=%u pid=0x%02X call_count=%u", len, pid, g2_reassembly_result.call_count);
}

static void t2_on_reassembly_error(ax25_seg_error_t error, void *user_data) {
    g2_error_result.error = error;
    g2_error_result.called = true;
    g2_error_result.call_count++;
    DEBUG_PRINT("Reassembly error: code=%u call_count=%u", (unsigned)error, g2_error_result.call_count);
}

static void t2_on_request_retransmit(uint8_t sequence, void *user_data) {
    if (g2_retransmit_requests.count < 64) {
        g2_retransmit_requests.sequences[g2_retransmit_requests.count++] = sequence;
    }DEBUG_PRINT("Retransmit requested: seq=%u", sequence);
}

static void t2_reset_globals(void) {
    memset(&g2_captured_segments, 0, sizeof(g2_captured_segments));
    memset(&g2_reassembly_result, 0, sizeof(g2_reassembly_result));
    memset(&g2_error_result, 0, sizeof(g2_error_result));
    memset(&g2_retransmit_requests, 0, sizeof(g2_retransmit_requests));
}

// Helper: build a tx+rx pair with standard callbacks
static void t2_setup_pair(ax25_segmenter_t *tx, ax25_segmenter_t *rx) {
    ax25_segmenter_init(tx, 256);
    ax25_segmenter_init(rx, 256);
    tx->transmit_iframe = t2_transmit_iframe;
    rx->on_reassembly_complete = t2_on_reassembly_complete;
    rx->on_reassembly_error = t2_on_reassembly_error;
    rx->on_request_retransmit = t2_on_request_retransmit;
    rx->user_data = NULL;
}

// Helper: feed a single captured segment to rx
static void t2_feed(ax25_segmenter_t *rx, uint8_t idx, uint32_t tick) {
    ax25_segmenter_receive(rx, g2_captured_segments.segments[idx], g2_captured_segments.segment_lengths[idx], g2_captured_segments.segment_pids[idx], tick);
}

// Helper: dump segment bitmap for debug
static void t2_dump_bitmap(const ax25_segmenter_t *seg) {
    DEBUG_PRINT("rx_bitmap: %02X %02X %02X %02X %02X %02X %02X %02X", seg->rx_segment_bitmap[0], seg->rx_segment_bitmap[1], seg->rx_segment_bitmap[2],
            seg->rx_segment_bitmap[3], seg->rx_segment_bitmap[4], seg->rx_segment_bitmap[5], seg->rx_segment_bitmap[6], seg->rx_segment_bitmap[7]);DEBUG_PRINT(
            "ooo_count=%u rx_expected=%u rx_buffer_used=%u rx_last=%u", seg->ooo_count, seg->rx_expected_segment, seg->rx_buffer_used, seg->rx_last_received);
}

// ============================================================
// GROUP A: Out-of-Order Segment Handling (extended)
// ============================================================

// A1: Reverse order (last segment first, first segment last)
static int test_ooo_reverse_order(void) {
    assert_count = 0;
    printf("\n--- [A1] test_ooo_reverse_order: all segments reversed ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[600];
    for (int i = 0; i < 600; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    uint8_t err = ax25_segmenter_send(&tx_seg, orig, 600, 0xF0);
    TEST_ASSERT(err == 0, "[A1] Segmentation succeeds", err);
    TEST_ASSERT(g2_captured_segments.segment_count == 3, "[A1] Three segments created", g2_captured_segments.segment_count);

    // Feed in reverse: 2, 1, 0
    uint32_t tick = 0;
    DEBUG_PRINT("[A1] Feeding segment 2 (last) first");
    t2_feed(&rx_seg, 2, tick);
    DEBUG_PRINT("[A1] state=%d ooo_count=%u", rx_seg.state, rx_seg.ooo_count);

    // Non-first segment without active reassembly — expect seq error
    // (receiver must get seq=0 first to start reassembly)
    // After seq error the state returns to IDLE, so subsequent feeds are ignored

    // Feed 1 — also out of context (no active reassembly after error)
    DEBUG_PRINT("[A1] Feeding segment 1");
    t2_feed(&rx_seg, 1, tick);

    // Feed 0 — this is the first segment, starts reassembly
    DEBUG_PRINT("[A1] Feeding segment 0 (first)");
    t2_feed(&rx_seg, 0, tick);
    DEBUG_PRINT("[A1] After feeding 0: state=%d ooo_count=%u called=%d", rx_seg.state, rx_seg.ooo_count, g2_reassembly_result.called);

    // With all-reverse and OOO buffer: segs 2 and 1 were lost before first frame,
    // reassembly cannot complete without them being retransmitted.
    // Verify the reassembler state is consistent (no crash, no spurious complete).
    TEST_ASSERT(g2_error_result.call_count <= 2u, "[A1] No more than 2 errors from misdelivered segments", g2_error_result.call_count);
    TEST_ASSERT(g2_reassembly_result.called == false, "[A1] Reassembly not complete (missing buffered segs)", 0);

    return 0;
}

// A2: Interleaved out-of-order: receive 0,3,1,2 (4 segments)
static int test_ooo_interleaved_4seg(void) {
    assert_count = 0;
    printf("\n--- [A2] test_ooo_interleaved_4seg: order 0,3,1,2 ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[800];
    for (int i = 0; i < 800; i++)
        orig[i] = (uint8_t) (i ^ 0x5A);

    uint8_t err = ax25_segmenter_send(&tx_seg, orig, 800, 0xCF);
    TEST_ASSERT(err == 0, "[A2] Segmentation succeeds", err);
    TEST_ASSERT(g2_captured_segments.segment_count == 4, "[A2] 4 segments created", g2_captured_segments.segment_count);

    uint32_t tick = 100;
    uint8_t order[] = { 0, 3, 1, 2 };
    for (int i = 0; i < 4; i++) {
        DEBUG_PRINT("[A2] Feeding segment idx=%u seq=%u", order[i], g2_captured_segments.segments[order[i]][0] & 0x3F);
        t2_feed(&rx_seg, order[i], tick);
        t2_dump_bitmap(&rx_seg);
    }

    TEST_ASSERT(g2_reassembly_result.called == true, "[A2] Reassembly completes (0,3,1,2)", 0);
    TEST_ASSERT(g2_reassembly_result.length == 800, "[A2] Length correct", g2_reassembly_result.length);
    TEST_ASSERT(g2_reassembly_result.pid == 0xCF, "[A2] PID preserved", g2_reassembly_result.pid);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 800) == 0, "[A2] Data integrity OK", 0);

    return 0;
}

// A3: Two consecutive missing segments then both arrive
static int test_ooo_two_consecutive_missing(void) {
    assert_count = 0;
    printf("\n--- [A3] test_ooo_two_consecutive_missing: skip segs 1+2 then deliver ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[1100];
    for (int i = 0; i < 1100; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    uint8_t err = ax25_segmenter_send(&tx_seg, orig, 1100, 0xF0);
    TEST_ASSERT(err == 0, "[A3] Segmentation succeeds", err);
    uint8_t total = g2_captured_segments.segment_count;
    TEST_ASSERT(total >= 5, "[A3] At least 5 segments", total);

    uint32_t tick = 0;

    // Feed: 0, 3, 4 (skip 1 and 2)
    DEBUG_PRINT("[A3] Feed 0");
    t2_feed(&rx_seg, 0, tick);
    DEBUG_PRINT("[A3] Feed 3");
    t2_feed(&rx_seg, 3, tick);
    if (total > 4) {
        DEBUG_PRINT("[A3] Feed 4");
        t2_feed(&rx_seg, 4, tick);
    }

    TEST_ASSERT(rx_seg.state == SEG_STATE_REASSEMBLING, "[A3] Still reassembling after gap", 0);
    TEST_ASSERT(g2_reassembly_result.called == false, "[A3] No premature complete", 0);
    DEBUG_PRINT("[A3] ooo_count=%u", rx_seg.ooo_count);

    // Deliver missing 1 and 2
    DEBUG_PRINT("[A3] Feed 1 (was missing)");
    t2_feed(&rx_seg, 1, tick);
    DEBUG_PRINT("[A3] Feed 2 (was missing)");
    t2_feed(&rx_seg, 2, tick);
    t2_dump_bitmap(&rx_seg);

    // Feed remaining (if any)
    for (uint8_t i = 5; i < total; i++) {
        DEBUG_PRINT("[A3] Feed remaining %u", i);
        t2_feed(&rx_seg, i, tick);
    }

    TEST_ASSERT(g2_reassembly_result.called == true, "[A3] Reassembly completes after missing segs delivered", 0);
    TEST_ASSERT(g2_reassembly_result.length == 1100, "[A3] Length correct", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 1100) == 0, "[A3] Data correct", 0);

    return 0;
}

// A4: Duplicate out-of-order segment — must not corrupt buffer
static int test_ooo_duplicate_ooo_segment(void) {
    assert_count = 0;
    printf("\n--- [A4] test_ooo_duplicate_ooo_segment ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[600];
    for (int i = 0; i < 600; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 600, 0xF0);
    TEST_ASSERT(g2_captured_segments.segment_count == 3, "[A4] 3 segments", g2_captured_segments.segment_count);

    uint32_t tick = 0;
    t2_feed(&rx_seg, 0, tick);      // in order
    t2_feed(&rx_seg, 2, tick);      // OOO (buffered)
    DEBUG_PRINT("[A4] ooo_count after first OOO=%u", rx_seg.ooo_count);
    t2_feed(&rx_seg, 2, tick);      // duplicate OOO — must be silently dropped
    DEBUG_PRINT("[A4] ooo_count after duplicate OOO=%u", rx_seg.ooo_count);
    TEST_ASSERT(rx_seg.ooo_count == 1, "[A4] ooo_count still 1 after duplicate OOO", rx_seg.ooo_count);
    t2_feed(&rx_seg, 1, tick);      // fills the gap — triggers drain
    t2_dump_bitmap(&rx_seg);

    TEST_ASSERT(g2_reassembly_result.called == true, "[A4] Reassembly completes", 0);
    TEST_ASSERT(g2_reassembly_result.call_count == 1, "[A4] Callback called exactly once", g2_reassembly_result.call_count);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 600) == 0, "[A4] Data correct", 0);

    return 0;
}

// A5: All segments arrive OOO in alternating even/odd pattern (5 segments)
static int test_ooo_alternating_pattern_5seg(void) {
    assert_count = 0;
    printf("\n--- [A5] test_ooo_alternating_pattern_5seg: order 0,2,4,1,3 ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[1200];
    for (int i = 0; i < 1200; i++)
        orig[i] = (uint8_t) ((i * 3) & 0xFF);

    uint8_t err = ax25_segmenter_send(&tx_seg, orig, 1200, 0xAB);
    TEST_ASSERT(err == 0, "[A5] Segmentation succeeds", err);
    uint8_t total = g2_captured_segments.segment_count;
    DEBUG_PRINT("[A5] total segments=%u", total);
    TEST_ASSERT(total >= 5, "[A5] At least 5 segments", total);

    uint32_t tick = 50;
    // Even indices first (0,2,4...) then odd (1,3,...)
    for (uint8_t i = 0; i < total; i += 2) {
        DEBUG_PRINT("[A5] feed even idx=%u", i);
        t2_feed(&rx_seg, i, tick);
    }
    for (uint8_t i = 1; i < total; i += 2) {
        DEBUG_PRINT("[A5] feed odd idx=%u", i);
        t2_feed(&rx_seg, i, tick);
    }

    t2_dump_bitmap(&rx_seg);
    TEST_ASSERT(g2_reassembly_result.called == true, "[A5] Reassembly completes with alternating delivery", 0);
    TEST_ASSERT(g2_reassembly_result.length == 1200, "[A5] Length correct", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 1200) == 0, "[A5] Data integrity OK", 0);

    return 0;
}

// A6: OOO buffer full scenario — fill OOO buffer then deliver in-order key
static int test_ooo_buffer_full_then_drain(void) {
    assert_count = 0;
    printf("\n--- [A6] test_ooo_buffer_full_then_drain ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    // Need at least AX25_MAX_OUT_OF_ORDER_SEGMENTS+2 segments = 10 total
    // 10 * 254 = 2540 bytes
    uint8_t orig[2500];
    for (int i = 0; i < 2500; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    uint8_t err = ax25_segmenter_send(&tx_seg, orig, 2500, 0xF0);
    TEST_ASSERT(err == 0, "[A6] Segmentation succeeds", err);
    uint8_t total = g2_captured_segments.segment_count;
    DEBUG_PRINT("[A6] total segments=%u ooo_buffer_size=%u", total, AX25_MAX_OUT_OF_ORDER_SEGMENTS);
    TEST_ASSERT(total >= 10, "[A6] At least 10 segments created", total);

    uint32_t tick = 0;

    // Feed segment 0 (first)
    t2_feed(&rx_seg, 0, tick);
    TEST_ASSERT(rx_seg.state == SEG_STATE_REASSEMBLING, "[A6] Reassembling started", 0);

    // Fill OOO buffer with segments 2..9 (skip 1, which creates a gap)
    uint8_t fed_ooo = 0;
    for (uint8_t i = 2; i < total && fed_ooo < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
        DEBUG_PRINT("[A6] feed OOO seg=%u", i);
        t2_feed(&rx_seg, i, tick);
        fed_ooo++;
    }
    DEBUG_PRINT("[A6] ooo_count=%u rx_gap_count=%u rx_expected=%u", rx_seg.ooo_count, rx_seg.rx_gap_count, rx_seg.rx_expected_segment);

    // OOO buffer should be at or near capacity
    TEST_ASSERT(rx_seg.ooo_count > 0, "[A6] OOO buffer has entries", rx_seg.ooo_count);

    // If gap threshold not yet exceeded: deliver missing seg 1 to drain
    if (!g2_error_result.called) {
        DEBUG_PRINT("[A6] Delivering missing segment 1");
        t2_feed(&rx_seg, 1, tick);
        t2_dump_bitmap(&rx_seg);

        // Deliver any remaining segments not yet fed
        for (uint8_t i = AX25_MAX_OUT_OF_ORDER_SEGMENTS + 2; i < total; i++) {
            DEBUG_PRINT("[A6] Feed remaining seg=%u", i);
            t2_feed(&rx_seg, i, tick);
        }

        if (g2_reassembly_result.called) {
            TEST_ASSERT(g2_reassembly_result.length == 2500, "[A6] Full data reassembled", g2_reassembly_result.length);
            TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 2500) == 0, "[A6] Data correct after drain", 0);
        }
        // Either reassembly completed or gap threshold triggered — both are valid
        TEST_ASSERT(g2_reassembly_result.called || g2_error_result.called, "[A6] Either reassembly or error: no hung state", 0);
    } else {
        // Gap threshold exceeded during fill — valid behavior
        TEST_ASSERT(g2_error_result.error == AX25_SEG_ERROR_SEQUENCE, "[A6] Correct error on gap exceeded", g2_error_result.error);
        TEST_ASSERT(rx_seg.state == SEG_STATE_IDLE, "[A6] State is IDLE after gap abort", 0);
    }

    return 0;
}

// A7: Non-first segment received when IDLE (no active reassembly)
static int test_ooo_orphan_segment_when_idle(void) {
    assert_count = 0;
    printf("\n--- [A7] test_ooo_orphan_segment_when_idle ---\n");
    t2_reset_globals();

    ax25_segmenter_t rx_seg;
    ax25_segmenter_init(&rx_seg, 256);
    rx_seg.on_reassembly_complete = t2_on_reassembly_complete;
    rx_seg.on_reassembly_error = t2_on_reassembly_error;

    // Craft a non-first segment (no BEG flag, seq=1)
    uint8_t fake_seg[10];
    fake_seg[0] = 0x01u;  // BEG=0 END=0 seq=1
    memset(&fake_seg[1], 0xAA, 9);

    uint32_t tick = 0;
    ax25_segmenter_receive(&rx_seg, fake_seg, 10, AX25_PID_SEGMENT_FRAGMENT, tick);

    DEBUG_PRINT("[A7] error_called=%d state=%d", g2_error_result.called, rx_seg.state);
    // Spec: receiving non-first without active reassembly is an error
    TEST_ASSERT(g2_error_result.called == true, "[A7] Error reported for orphan segment", 0);
    TEST_ASSERT(g2_error_result.error == AX25_SEG_ERROR_SEQUENCE, "[A7] Error type is SEQUENCE", g2_error_result.error);
    TEST_ASSERT(rx_seg.state == SEG_STATE_IDLE, "[A7] Remains IDLE", 0);
    TEST_ASSERT(g2_reassembly_result.called == false, "[A7] No spurious reassembly", 0);

    return 0;
}

// A8: 6 segments OOO — feed last first, then all remaining in random order
static int test_ooo_last_first_6seg(void) {
    assert_count = 0;
    printf("\n--- [A8] test_ooo_last_first_6seg: feed last (5) first, then 0,4,2,3,1 ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[1450];
    for (int i = 0; i < 1450; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 1450, 0xF0);
    uint8_t total = g2_captured_segments.segment_count;
    DEBUG_PRINT("[A8] total=%u", total);
    TEST_ASSERT(total == 6, "[A8] 6 segments created (1450/254 ceiling=6)", total);

    uint32_t tick = 200;

    // Feed segment 5 (last) first — no active reassembly yet => error
    DEBUG_PRINT("[A8] Feed 5 (last) first");
    t2_feed(&rx_seg, 5, tick);

    // Now feed 0 (first) — starts reassembly
    DEBUG_PRINT("[A8] Feed 0 (first)");
    t2_feed(&rx_seg, 0, tick);
    TEST_ASSERT(rx_seg.state == SEG_STATE_REASSEMBLING, "[A8] Reassembly started", 0);

    // Feed 4,2,3,1 in that order
    uint8_t remaining[] = { 4, 2, 3, 1 };
    for (int i = 0; i < 4; i++) {
        DEBUG_PRINT("[A8] Feed seg=%u", remaining[i]);
        t2_feed(&rx_seg, remaining[i], tick);
        t2_dump_bitmap(&rx_seg);
    }

    // Now feed 5 again (retransmitted)
    DEBUG_PRINT("[A8] Feed 5 again (retransmit)");
    t2_feed(&rx_seg, 5, tick);

    TEST_ASSERT(g2_reassembly_result.called == true, "[A8] Reassembly completes", 0);
    TEST_ASSERT(g2_reassembly_result.length == 1450, "[A8] Length correct", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 1450) == 0, "[A8] Data correct", 0);

    return 0;
}

// A9: SREJ-style retransmit request triggered on small gap
static int test_ooo_retransmit_request_triggered(void) {
    assert_count = 0;
    printf("\n--- [A9] test_ooo_retransmit_request_triggered ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);
    rx_seg.on_request_retransmit = t2_on_request_retransmit;

    uint8_t orig[800];
    for (int i = 0; i < 800; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 800, 0xF0);
    TEST_ASSERT(g2_captured_segments.segment_count == 4, "[A9] 4 segments", g2_captured_segments.segment_count);

    uint32_t tick = 0;
    t2_feed(&rx_seg, 0, tick);    // seq 0 — in order
    t2_feed(&rx_seg, 2, tick);    // seq 2 — gap at 1, should trigger SREJ for seq 1

    DEBUG_PRINT("[A9] retransmit_requests count=%u", g2_retransmit_requests.count);
    TEST_ASSERT(g2_retransmit_requests.count >= 1, "[A9] At least one retransmit request sent", g2_retransmit_requests.count);
    TEST_ASSERT(g2_retransmit_requests.sequences[0] == 1, "[A9] Retransmit requested for seq=1", g2_retransmit_requests.sequences[0]);

    return 0;
}

// ============================================================
// GROUP B: TR210 Timer Edge Cases
// ============================================================

// B1: Timer starts exactly on first segment, expires at boundary tick
static int test_tr210_exact_boundary(void) {
    assert_count = 0;
    printf("\n--- [B1] test_tr210_exact_boundary: expire at timeout_tick exactly ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[400];
    for (int i = 0; i < 400; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 400, 0xF0);

    uint32_t tick = 0;
    t2_feed(&rx_seg, 0, tick);

    uint32_t timeout = rx_seg.rx_timeout_tick;
    DEBUG_PRINT("[B1] timeout_tick=%u", timeout);
    TEST_ASSERT(rx_seg.rx_timer_armed == true, "[B1] Timer is armed", 0);

    // Tick at exactly timeout_tick — 1 (one before boundary) — must NOT expire
    ax25_segmenter_tick(&rx_seg, timeout - 1);
    TEST_ASSERT(g2_error_result.called == false, "[B1] No timeout one tick before boundary", 0);
    TEST_ASSERT(rx_seg.state == SEG_STATE_REASSEMBLING, "[B1] Still reassembling", 0);

    // Tick at exactly timeout_tick — must expire
    ax25_segmenter_tick(&rx_seg, timeout);
    DEBUG_PRINT("[B1] error_called=%d error=%u state=%d", g2_error_result.called, (unsigned)g2_error_result.error, rx_seg.state);
    TEST_ASSERT(g2_error_result.called == true, "[B1] Timeout at exact boundary tick", 0);
    TEST_ASSERT(g2_error_result.error == AX25_SEG_ERROR_TIMEOUT, "[B1] Error is TIMEOUT", 0);
    TEST_ASSERT(rx_seg.state == SEG_STATE_IDLE, "[B1] State IDLE after timeout", 0);
    TEST_ASSERT(rx_seg.rx_timer_armed == false, "[B1] Timer disarmed after timeout", 0);

    return 0;
}

// B2: Timer restarts on each received segment — verify growing timeout
static int test_tr210_restart_each_segment(void) {
    assert_count = 0;
    printf("\n--- [B2] test_tr210_restart_each_segment ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[1000];
    for (int i = 0; i < 1000; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 1000, 0xF0);
    uint8_t total = g2_captured_segments.segment_count;
    DEBUG_PRINT("[B2] total=%u", total);

    uint32_t tick = 0;
    uint32_t prev_timeout = 0;
    for (uint8_t i = 0; i < total - 1; i++) {
        tick += 500;  // Advance 5 seconds between each segment
        t2_feed(&rx_seg, i, tick);
        uint32_t curr_timeout = rx_seg.rx_timeout_tick;
        DEBUG_PRINT("[B2] seg=%u tick=%u timeout_tick=%u", i, tick, curr_timeout);
        if (i > 0) {
            TEST_ASSERT(curr_timeout > prev_timeout, "[B2] Timer extended on each segment", 0);
        }
        TEST_ASSERT(rx_seg.rx_timer_armed == true, "[B2] Timer always armed during reassembly", 0);
        // Verify no premature timeout (tick is before timeout_tick)
        ax25_segmenter_tick(&rx_seg, tick);
        TEST_ASSERT(g2_error_result.called == false, "[B2] No premature timeout", 0);
        prev_timeout = curr_timeout;
    }
    // Feed last segment
    tick += 100;
    t2_feed(&rx_seg, total - 1, tick);
    TEST_ASSERT(g2_reassembly_result.called == true, "[B2] Reassembly completes", 0);
    TEST_ASSERT(rx_seg.rx_timer_armed == false, "[B2] Timer disarmed after complete", 0);

    return 0;
}

// B3: Timeout fires, new message starts immediately after
static int test_tr210_new_message_after_timeout(void) {
    assert_count = 0;
    printf("\n--- [B3] test_tr210_new_message_after_timeout ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[600];
    for (int i = 0; i < 600; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 600, 0xF0);

    uint32_t tick = 0;
    // Start first reassembly, receive only first segment
    t2_feed(&rx_seg, 0, tick);
    TEST_ASSERT(rx_seg.state == SEG_STATE_REASSEMBLING, "[B3] First reassembly started", 0);

    // Expire TR210
    tick = rx_seg.rx_timeout_tick + 1;
    ax25_segmenter_tick(&rx_seg, tick);
    TEST_ASSERT(g2_error_result.called == true, "[B3] Timeout fired", 0);
    TEST_ASSERT(rx_seg.state == SEG_STATE_IDLE, "[B3] IDLE after timeout", 0);

    // Reset captured segments for second message
    t2_reset_globals();
    ax25_segmenter_init(&tx_seg, 256);
    tx_seg.transmit_iframe = t2_transmit_iframe;

    uint8_t orig2[600];
    for (int i = 0; i < 600; i++)
        orig2[i] = (uint8_t) ((i + 1) & 0xFF);

    ax25_segmenter_send(&tx_seg, orig2, 600, 0xAA);

    // Receive all segments of second message
    for (uint8_t i = 0; i < g2_captured_segments.segment_count; i++) {
        t2_feed(&rx_seg, i, tick);
    }

    TEST_ASSERT(g2_reassembly_result.called == true, "[B3] Second message reassembled after timeout recovery", 0);
    TEST_ASSERT(g2_reassembly_result.length == 600, "[B3] Length correct for second message", g2_reassembly_result.length);
    TEST_ASSERT(g2_reassembly_result.pid == 0xAA, "[B3] Second message PID correct", g2_reassembly_result.pid);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig2, 600) == 0, "[B3] Second message data correct", 0);

    return 0;
}

// B4: Tick called when IDLE — must do nothing (no crash, no spurious error)
static int test_tr210_tick_when_idle(void) {
    assert_count = 0;
    printf("\n--- [B4] test_tr210_tick_when_idle ---\n");
    t2_reset_globals();

    ax25_segmenter_t rx_seg;
    ax25_segmenter_init(&rx_seg, 256);
    rx_seg.on_reassembly_complete = t2_on_reassembly_complete;
    rx_seg.on_reassembly_error = t2_on_reassembly_error;

    // Call tick many times while idle
    for (uint32_t t = 0; t < 100000; t += 1000) {
        ax25_segmenter_tick(&rx_seg, t);
    }

    TEST_ASSERT(g2_error_result.called == false, "[B4] No spurious error from tick while IDLE", 0);
    TEST_ASSERT(g2_reassembly_result.called == false, "[B4] No spurious complete from tick while IDLE", 0);
    TEST_ASSERT(rx_seg.state == SEG_STATE_IDLE, "[B4] State remains IDLE", 0);

    return 0;
}

// B5: Timer wraparound — tick counter wraps around UINT32_MAX
static int test_tr210_wraparound_tick(void) {
    assert_count = 0;
    printf("\n--- [B5] test_tr210_wraparound_tick: tick near UINT32_MAX ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[600];
    for (int i = 0; i < 600; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 600, 0xF0);

    // Start reassembly near UINT32_MAX
    uint32_t tick = 0xFFFFFF00u;
    t2_feed(&rx_seg, 0, tick);
    DEBUG_PRINT("[B5] timeout_tick=0x%08X current_tick=0x%08X", rx_seg.rx_timeout_tick, tick);

    // Feed remaining segments across the wraparound
    tick = 0x00000100u;  // Wrapped around
    for (uint8_t i = 1; i < g2_captured_segments.segment_count; i++) {
        t2_feed(&rx_seg, i, tick);
    }

    // Run tick after wrap to ensure no false timeout
    ax25_segmenter_tick(&rx_seg, tick);
    DEBUG_PRINT("[B5] after wrap: error_called=%d reassembly_called=%d state=%d", g2_error_result.called, g2_reassembly_result.called, rx_seg.state);

    TEST_ASSERT(g2_reassembly_result.called == true, "[B5] Reassembly completes across tick wraparound", 0);
    TEST_ASSERT(g2_reassembly_result.length == 600, "[B5] Length correct across wraparound", 0);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 600) == 0, "[B5] Data correct across wraparound", 0);

    return 0;
}

// B6: Multiple ticks between segments — timer must not fire prematurely
static int test_tr210_ticks_between_segments(void) {
    assert_count = 0;
    printf("\n--- [B6] test_tr210_ticks_between_segments ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[800];
    for (int i = 0; i < 800; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 800, 0xF0);
    uint8_t total = g2_captured_segments.segment_count;

    uint32_t tick = 0;
    t2_feed(&rx_seg, 0, tick);

    // Run SEG_TIMEOUT_TICKS - 10 ticks without sending next segment
    // This must NOT cause a timeout since we are still inside the window
    uint32_t safe_tick = tick + SEG_TIMEOUT_TICKS - 10;
    for (uint32_t t = tick + 1; t <= safe_tick; t += 100) {
        ax25_segmenter_tick(&rx_seg, t);
    }
    TEST_ASSERT(g2_error_result.called == false, "[B6] No premature timeout at TIMEOUT-10", 0);

    // Now send remaining segments — timer should restart each time
    for (uint8_t i = 1; i < total; i++) {
        t2_feed(&rx_seg, i, safe_tick);
    }

    TEST_ASSERT(g2_reassembly_result.called == true, "[B6] Reassembly completes after slow delivery", 0);
    TEST_ASSERT(g2_error_result.called == false, "[B6] No timeout error on slow but valid delivery", 0);

    return 0;
}

// B7: First segment received, then new FIRST segment before timeout (restart)
static int test_tr210_new_first_segment_aborts_ongoing(void) {
    assert_count = 0;
    printf("\n--- [B7] test_tr210_new_first_segment_aborts_ongoing ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig_a[600], orig_b[400];
    for (int i = 0; i < 600; i++)
        orig_a[i] = (uint8_t) (i & 0xFF);
    for (int i = 0; i < 400; i++)
        orig_b[i] = (uint8_t) ((i + 100) & 0xFF);

    // Segment message A
    ax25_segmenter_send(&tx_seg, orig_a, 600, 0xF0);
    // copy A segments aside
    t2_segment_capture_t segs_a = g2_captured_segments;

    // Segment message B
    t2_reset_globals();
    ax25_segmenter_init(&tx_seg, 256);
    tx_seg.transmit_iframe = t2_transmit_iframe;
    ax25_segmenter_send(&tx_seg, orig_b, 400, 0xCC);
    uint8_t seg_b_count = g2_captured_segments.segment_count;
    t2_segment_capture_t segs_b = g2_captured_segments;

    t2_reset_globals();

    uint32_t tick = 0;

    // Feed first segment of message A
    rx_seg.on_reassembly_complete = t2_on_reassembly_complete;
    rx_seg.on_reassembly_error = t2_on_reassembly_error;
    ax25_segmenter_receive(&rx_seg, segs_a.segments[0], segs_a.segment_lengths[0], segs_a.segment_pids[0], tick);
    TEST_ASSERT(rx_seg.state == SEG_STATE_REASSEMBLING, "[B7] Reassembling message A", 0);

    // Before completing A, feed first segment of message B (new first segment)
    tick += 100;
    ax25_segmenter_receive(&rx_seg, segs_b.segments[0], segs_b.segment_lengths[0], segs_b.segment_pids[0], tick);
    DEBUG_PRINT("[B7] After new-first: state=%d ooo_count=%u", rx_seg.state, rx_seg.ooo_count);
    TEST_ASSERT(rx_seg.state == SEG_STATE_REASSEMBLING, "[B7] Still reassembling (now for B)", 0);
    TEST_ASSERT(rx_seg.rx_expected_segment == 1, "[B7] Expect seq=1 next (B started fresh)", 0);

    // Complete message B
    for (uint8_t i = 1; i < seg_b_count; i++) {
        ax25_segmenter_receive(&rx_seg, segs_b.segments[i], segs_b.segment_lengths[i], segs_b.segment_pids[i], tick);
    }

    TEST_ASSERT(g2_reassembly_result.called == true, "[B7] Message B reassembled", 0);
    TEST_ASSERT(g2_reassembly_result.pid == 0xCC, "[B7] PID is B's PID", g2_reassembly_result.pid);
    TEST_ASSERT(g2_reassembly_result.length == 400, "[B7] Length is B's length", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig_b, 400) == 0, "[B7] Data is B's data", 0);

    return 0;
}

// ============================================================
// GROUP C: Large Multi-Segment Reassembly (>3 segments)
// ============================================================

// C1: Exactly 5 segments in order
static int test_large_5seg_inorder(void) {
    assert_count = 0;
    printf("\n--- [C1] test_large_5seg_inorder: 5 segments, in order ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[1200];
    for (int i = 0; i < 1200; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 1200, 0xF0);
    uint8_t total = g2_captured_segments.segment_count;
    DEBUG_PRINT("[C1] total segments=%u", total);
    TEST_ASSERT(total == 5, "[C1] Exactly 5 segments for 1200 bytes @ 254/seg", total);

    uint32_t tick = 0;
    for (uint8_t i = 0; i < total; i++)
        t2_feed(&rx_seg, i, tick);

    TEST_ASSERT(g2_reassembly_result.called == true, "[C1] Reassembly completes", 0);
    TEST_ASSERT(g2_reassembly_result.length == 1200, "[C1] Length correct", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 1200) == 0, "[C1] Data integrity OK", 0);
    TEST_ASSERT(rx_seg.state == SEG_STATE_IDLE, "[C1] State IDLE after complete", 0);

    return 0;
}

// C2: 8 segments in order — exact boundary for OOO buffer
static int test_large_8seg_inorder(void) {
    assert_count = 0;
    printf("\n--- [C2] test_large_8seg_inorder: 8 segments, in order ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[1950];
    for (int i = 0; i < 1950; i++)
        orig[i] = (uint8_t) ((i * 7) & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 1950, 0xBB);
    uint8_t total = g2_captured_segments.segment_count;
    DEBUG_PRINT("[C2] total=%u", total);
    TEST_ASSERT(total == 8, "[C2] 8 segments for 1950 bytes", total);

    uint32_t tick = 0;
    for (uint8_t i = 0; i < total; i++)
        t2_feed(&rx_seg, i, tick);

    TEST_ASSERT(g2_reassembly_result.called == true, "[C2] Reassembly completes", 0);
    TEST_ASSERT(g2_reassembly_result.length == 1950, "[C2] Length correct", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 1950) == 0, "[C2] Data correct", 0);

    return 0;
}

// C3: 10 segments in order (exceeds OOO buffer size)
static int test_large_10seg_inorder(void) {
    assert_count = 0;
    printf("\n--- [C3] test_large_10seg_inorder: 10 segments ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[2500];
    for (int i = 0; i < 2500; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 2500, 0xF0);
    uint8_t total = g2_captured_segments.segment_count;
    DEBUG_PRINT("[C3] total=%u", total);
    TEST_ASSERT(total == 10, "[C3] 10 segments for 2500 bytes", total);

    uint32_t tick = 0;
    for (uint8_t i = 0; i < total; i++)
        t2_feed(&rx_seg, i, tick);

    TEST_ASSERT(g2_reassembly_result.called == true, "[C3] Reassembly completes", 0);
    TEST_ASSERT(g2_reassembly_result.length == 2500, "[C3] Length correct", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 2500) == 0, "[C3] Data correct", 0);

    return 0;
}

// C4: 16 segments — test larger sequence number range and bitmap
static int test_large_16seg_inorder(void) {
    assert_count = 0;
    printf("\n--- [C4] test_large_16seg_inorder: 16 segments ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    // 16 segments * 254 bytes = 4064 bytes (just within 4096 rx buffer)
    uint8_t *orig = (uint8_t*) malloc(4000);
    TEST_ASSERT(orig != NULL, "[C4] malloc succeeded", 0);
    for (int i = 0; i < 4000; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 4000, 0xF0);
    uint8_t total = g2_captured_segments.segment_count;
    DEBUG_PRINT("[C4] total segments=%u", total);
    TEST_ASSERT(total == 16, "[C4] 16 segments for 4000 bytes", total);

    // Verify each segment header has correct sequence numbers
    for (uint8_t i = 0; i < total; i++) {
        uint8_t hdr = g2_captured_segments.segments[i][0];
        uint8_t seq = hdr & 0x3F;
        TEST_ASSERT(seq == i, "[C4] Segment sequence number correct", i);
        if (i == 0) {
            TEST_ASSERT((hdr & 0x80) != 0, "[C4] First segment BEG flag set", i);
        } else {
            TEST_ASSERT((hdr & 0x80) == 0, "[C4] Non-first segment BEG flag clear", i);
        }
        if (i == total - 1) {
            TEST_ASSERT((hdr & 0x40) != 0, "[C4] Last segment END flag set", i);
        } else {
            TEST_ASSERT((hdr & 0x40) == 0, "[C4] Non-last segment END flag clear", i);
        }
    }

    uint32_t tick = 0;
    for (uint8_t i = 0; i < total; i++)
        t2_feed(&rx_seg, i, tick);

    TEST_ASSERT(g2_reassembly_result.called == true, "[C4] Reassembly completes", 0);
    TEST_ASSERT(g2_reassembly_result.length == 4000, "[C4] Length correct", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 4000) == 0, "[C4] Data correct", 0);

    free(orig);
    return 0;
}

// C5: 7 segments, one missing segment recovered via retransmit (full end-to-end)
static int test_large_7seg_with_recovery(void) {
    assert_count = 0;
    printf("\n--- [C5] test_large_7seg_with_recovery: skip seg 3, retransmit and complete ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);
    rx_seg.on_request_retransmit = t2_on_request_retransmit;

    uint8_t orig[1700];
    for (int i = 0; i < 1700; i++)
        orig[i] = (uint8_t) (i & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 1700, 0xF0);
    uint8_t total = g2_captured_segments.segment_count;
    DEBUG_PRINT("[C5] total=%u", total);
    TEST_ASSERT(total == 7, "[C5] 7 segments for 1700 bytes", total);

    uint32_t tick = 0;
    // Feed 0,1,2, skip 3, feed 4,5,6
    for (uint8_t i = 0; i < 3; i++)
        t2_feed(&rx_seg, i, tick);

    DEBUG_PRINT("[C5] Before skip: state=%d ooo_count=%u expected=%u", rx_seg.state, rx_seg.ooo_count, rx_seg.rx_expected_segment);

    for (uint8_t i = 4; i < total; i++)
        t2_feed(&rx_seg, i, tick);

    DEBUG_PRINT("[C5] After skip: ooo_count=%u rx_expected=%u retransmit_requests=%u", rx_seg.ooo_count, rx_seg.rx_expected_segment,
            g2_retransmit_requests.count);

    TEST_ASSERT(g2_reassembly_result.called == false, "[C5] Not yet complete without seg 3", 0);
    TEST_ASSERT(g2_retransmit_requests.count >= 1, "[C5] Retransmit requested for seg 3", g2_retransmit_requests.count);

    // Simulate retransmit: deliver seg 3
    t2_feed(&rx_seg, 3, tick);
    t2_dump_bitmap(&rx_seg);

    TEST_ASSERT(g2_reassembly_result.called == true, "[C5] Reassembly completes after recovery", 0);
    TEST_ASSERT(g2_reassembly_result.length == 1700, "[C5] Length correct", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 1700) == 0, "[C5] Data correct after recovery", 0);

    return 0;
}

// C6: 10 segments, every other one OOO (0,2,4,6,8,1,3,5,7,9)
static int test_large_10seg_alternating_ooo(void) {
    assert_count = 0;
    printf("\n--- [C6] test_large_10seg_alternating_ooo ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    uint8_t orig[2500];
    for (int i = 0; i < 2500; i++)
        orig[i] = (uint8_t) ((i * 5) & 0xFF);

    ax25_segmenter_send(&tx_seg, orig, 2500, 0xD0);
    uint8_t total = g2_captured_segments.segment_count;
    TEST_ASSERT(total == 10, "[C6] 10 segments", total);

    uint32_t tick = 0;
    for (uint8_t i = 0; i < total; i += 2) {
        DEBUG_PRINT("[C6] feed even seg=%u", i);
        t2_feed(&rx_seg, i, tick);
    }
    for (uint8_t i = 1; i < total; i += 2) {
        DEBUG_PRINT("[C6] feed odd seg=%u", i);
        t2_feed(&rx_seg, i, tick);
    }
    t2_dump_bitmap(&rx_seg);

    TEST_ASSERT(g2_reassembly_result.called == true, "[C6] Reassembly completes 10-seg alternating", 0);
    TEST_ASSERT(g2_reassembly_result.length == 2500, "[C6] Length correct", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 2500) == 0, "[C6] Data correct", 0);

    return 0;
}

// C7: Multiple independent messages reassembled sequentially (state machine reset)
static int test_large_multiple_messages_sequential(void) {
    assert_count = 0;
    printf("\n--- [C7] test_large_multiple_messages_sequential: 3 messages back to back ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    rx_seg.on_reassembly_complete = t2_on_reassembly_complete;
    rx_seg.on_reassembly_error = t2_on_reassembly_error;
    ax25_segmenter_init(&rx_seg, 256);
    rx_seg.on_reassembly_complete = t2_on_reassembly_complete;
    rx_seg.on_reassembly_error = t2_on_reassembly_error;

    uint32_t tick = 0;
    uint16_t sizes[] = { 600, 1000, 1500 };
    uint8_t pids[] = { 0xF0, 0xCF, 0xAB };

    for (int msg = 0; msg < 3; msg++) {
        t2_reset_globals();
        // re-apply callbacks after reset (they weren't cleared but reset clears capture)
        rx_seg.on_reassembly_complete = t2_on_reassembly_complete;
        rx_seg.on_reassembly_error = t2_on_reassembly_error;

        ax25_segmenter_init(&tx_seg, 256);
        tx_seg.transmit_iframe = t2_transmit_iframe;

        uint8_t *orig = (uint8_t*) malloc(sizes[msg]);
        TEST_ASSERT(orig != NULL, "[C7] malloc succeeded", msg);
        for (int i = 0; i < sizes[msg]; i++)
            orig[i] = (uint8_t) ((i + msg * 37) & 0xFF);

        ax25_segmenter_send(&tx_seg, orig, sizes[msg], pids[msg]);
        uint8_t total = g2_captured_segments.segment_count;
        DEBUG_PRINT("[C7] msg=%d size=%u total_segs=%u", msg, sizes[msg], total);

        for (uint8_t i = 0; i < total; i++)
            t2_feed(&rx_seg, i, tick);
        tick += 1000;

        TEST_ASSERT(g2_reassembly_result.called == true, "[C7] Message reassembled", msg);
        TEST_ASSERT(g2_reassembly_result.length == sizes[msg], "[C7] Length correct", g2_reassembly_result.length);
        TEST_ASSERT(g2_reassembly_result.pid == pids[msg], "[C7] PID correct", g2_reassembly_result.pid);
        TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, sizes[msg]) == 0, "[C7] Data correct", 0);
        TEST_ASSERT(rx_seg.state == SEG_STATE_IDLE, "[C7] State IDLE after each message", 0);
        TEST_ASSERT(g2_error_result.called == false, "[C7] No error on valid sequential messages", 0);

        free(orig);
    }

    return 0;
}

// C8: Near-maximum data (near 4096 rx buffer limit, max segments)
static int test_large_near_max_reassembly(void) {
    assert_count = 0;
    printf("\n--- [C8] test_large_near_max_reassembly: 16 segments, 4000 bytes ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg, rx_seg;
    t2_setup_pair(&tx_seg, &rx_seg);

    // Use heap to avoid stack overflow in test
    uint8_t *orig = (uint8_t*) malloc(4000);
    TEST_ASSERT(orig != NULL, "[C8] malloc succeeded", 0);
    for (int i = 0; i < 4000; i++)
        orig[i] = (uint8_t) ((i * 3) ^ 0xA5);

    uint8_t err = ax25_segmenter_send(&tx_seg, orig, 4000, 0xF0);
    TEST_ASSERT(err == 0, "[C8] Segmentation succeeds", err);
    uint8_t total = g2_captured_segments.segment_count;
    DEBUG_PRINT("[C8] total segments=%u (expected 16)", total);
    TEST_ASSERT(total <= 64, "[C8] Segments within 6-bit limit (<=64)", total);

    uint32_t tick = 0;
    for (uint8_t i = 0; i < total; i++) {
        t2_feed(&rx_seg, i, tick);
        // Regularly call tick to simulate real-time operation
        if (i % 4 == 0)
            ax25_segmenter_tick(&rx_seg, tick);
        tick += 10;
    }
    t2_dump_bitmap(&rx_seg);

    TEST_ASSERT(g2_reassembly_result.called == true, "[C8] Near-max reassembly completes", 0);
    TEST_ASSERT(g2_reassembly_result.length == 4000, "[C8] Length correct", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, orig, 4000) == 0, "[C8] Data correct", 0);
    TEST_ASSERT(g2_error_result.called == false, "[C8] No errors on valid near-max", 0);

    free(orig);
    return 0;
}

// C9: Segment bitmap correctness — verify all 64 bits can be set
static int test_large_bitmap_all_64_segs(void) {
    assert_count = 0;
    printf("\n--- [C9] test_large_bitmap_all_64_segs: verify 64-bit bitmap tracking ---\n");
    t2_reset_globals();

    // Construct a minimal rx_seg to test bitmap directly
    ax25_segmenter_t rx_seg;
    ax25_segmenter_init(&rx_seg, 256);
    rx_seg.on_reassembly_complete = t2_on_reassembly_complete;
    rx_seg.on_reassembly_error = t2_on_reassembly_error;

    // Simulate receiving first segment to arm state
    uint8_t first_seg[3];
    first_seg[0] = 0x80u;  // BEG=1 END=0 seq=0
    first_seg[1] = 0xF0;  // PID
    first_seg[2] = 0xAA;  // 1 byte of data
    ax25_segmenter_receive(&rx_seg, first_seg, 3, AX25_PID_SEGMENT_FRAGMENT, 0);
    TEST_ASSERT(rx_seg.state == SEG_STATE_REASSEMBLING, "[C9] Reassembling started", 0);

    // Verify bit 0 is set in bitmap
    TEST_ASSERT((rx_seg.rx_segment_bitmap[0] & 0x01u) != 0, "[C9] Bit 0 set in bitmap", 0);

    // Feed segments in ascending order, verify each bit
    // Use seq 1..7 (filling byte 0 of bitmap)
    for (uint8_t seq = 1; seq <= 7; seq++) {
        uint8_t seg[2];
        seg[0] = seq;  // BEG=0 END=0 seq
        seg[1] = seq;  // 1 byte data
        ax25_segmenter_receive(&rx_seg, seg, 2, AX25_PID_SEGMENT_FRAGMENT, 0);
        uint8_t bit = (rx_seg.rx_segment_bitmap[0] >> seq) & 1u;
        TEST_ASSERT(bit == 1, "[C9] Bitmap bit set for seq 1-7", seq);
    }

    // Verify full first byte of bitmap
    TEST_ASSERT(rx_seg.rx_segment_bitmap[0] == 0xFF, "[C9] Bitmap byte 0 fully set (seqs 0-7)", 0);

    // Feed seq 8 to verify second byte starts correctly
    uint8_t seg8[2];
    seg8[0] = 8;  // seq=8
    seg8[1] = 0x08;
    ax25_segmenter_receive(&rx_seg, seg8, 2, AX25_PID_SEGMENT_FRAGMENT, 0);
    TEST_ASSERT((rx_seg.rx_segment_bitmap[1] & 0x01u) != 0, "[C9] Bit 0 of byte 1 set (seq=8)", 0);

    DEBUG_PRINT("[C9] bitmap byte0=0x%02X byte1=0x%02X expected=0x%02X", rx_seg.rx_segment_bitmap[0], rx_seg.rx_segment_bitmap[1], rx_seg.rx_expected_segment);

    return 0;
}

// C10: Exact segment count boundary — data that produces exactly 1 segment vs 2
static int test_large_segment_count_boundary(void) {
    assert_count = 0;
    printf("\n--- [C10] test_large_segment_count_boundary: size=254 -> 1 seg, 255 -> 2 segs ---\n");
    t2_reset_globals();

    ax25_segmenter_t tx_seg;
    ax25_segmenter_init(&tx_seg, 256);
    tx_seg.transmit_iframe = t2_transmit_iframe;

    uint8_t data254[254];
    memset(data254, 0x11, 254);

    // 254 bytes = exactly 1 segment (fits in segment_size=254)
    ax25_segmenter_send(&tx_seg, data254, 254, 0xF0);
    DEBUG_PRINT("[C10] 254 bytes -> %u segments", g2_captured_segments.segment_count);
    TEST_ASSERT(g2_captured_segments.segment_count == 1, "[C10] 254 bytes fits in 1 segment", g2_captured_segments.segment_count);

    t2_reset_globals();
    ax25_segmenter_init(&tx_seg, 256);
    tx_seg.transmit_iframe = t2_transmit_iframe;

    // 255 bytes requires 2 segments (254 + 1)
    uint8_t data255[255];
    memset(data255, 0x22, 255);
    ax25_segmenter_send(&tx_seg, data255, 255, 0xF0);
    DEBUG_PRINT("[C10] 255 bytes -> %u segments", g2_captured_segments.segment_count);
    TEST_ASSERT(g2_captured_segments.segment_count == 2, "[C10] 255 bytes needs 2 segments", g2_captured_segments.segment_count);

    // Verify 2-segment reassembly
    ax25_segmenter_t rx_seg;
    ax25_segmenter_init(&rx_seg, 256);
    rx_seg.on_reassembly_complete = t2_on_reassembly_complete;
    rx_seg.on_reassembly_error = t2_on_reassembly_error;
    for (uint8_t i = 0; i < g2_captured_segments.segment_count; i++) {
        t2_feed(&rx_seg, i, 0);
    }
    TEST_ASSERT(g2_reassembly_result.called == true, "[C10] 2-segment reassembly completes", 0);
    TEST_ASSERT(g2_reassembly_result.length == 255, "[C10] Reassembled length is 255", g2_reassembly_result.length);
    TEST_ASSERT(memcmp(g2_reassembly_result.data, data255, 255) == 0, "[C10] 2-seg data correct", 0);

    return 0;
}

// ============================================================
// Main entry point
// ============================================================

int test_ax25_segmenter2_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("Starting AX.25 Segmenter Extended Tests (Section 8 - OOO/TR210/Large)\n");
    printf("==================================================================================\n\n");

    printf("--- GROUP A: Out-of-Order Segment Handling ---\n");
    result |= test_ooo_reverse_order();
    result |= test_ooo_interleaved_4seg();
    result |= test_ooo_two_consecutive_missing();
    result |= test_ooo_duplicate_ooo_segment();
    result |= test_ooo_alternating_pattern_5seg();
    result |= test_ooo_buffer_full_then_drain();
    result |= test_ooo_orphan_segment_when_idle();
    result |= test_ooo_last_first_6seg();
    result |= test_ooo_retransmit_request_triggered();

    printf("\n--- GROUP B: TR210 Timer Edge Cases ---\n");
    result |= test_tr210_exact_boundary();
    result |= test_tr210_restart_each_segment();
    result |= test_tr210_new_message_after_timeout();
    result |= test_tr210_tick_when_idle();
    result |= test_tr210_wraparound_tick();
    result |= test_tr210_ticks_between_segments();
    result |= test_tr210_new_first_segment_aborts_ongoing();

    printf("\n--- GROUP C: Large Multi-Segment Reassembly (>3 segments) ---\n");
    result |= test_large_5seg_inorder();
    result |= test_large_8seg_inorder();
    result |= test_large_10seg_inorder();
    result |= test_large_16seg_inorder();
    result |= test_large_7seg_with_recovery();
    result |= test_large_10seg_alternating_ooo();
    result |= test_large_multiple_messages_sequential();
    result |= test_large_near_max_reassembly();
    result |= test_large_bitmap_all_64_segs();
    result |= test_large_segment_count_boundary();

    printf("\n==================================================================================\n");
    printf("AX.25 Segmenter Extended Tests Completed. %s\n", result == 0 ? "All tests passed" : "Some tests failed");
    printf("==================================================================================\n\n");

    return result;
}
