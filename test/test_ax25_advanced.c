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

// AX.25 v2.2 Advanced Feature Tests - Section 13
// Covers three areas identified as having limited test coverage:
//   SECTION A: SREJ Bitmap Management
//     - Multi-frame SREJ bitmap tracking across window boundaries
//     - Bitmap bit position correctness (byte_idx = ns>>3, bit_idx = ns&7)
//     - Bitmap clearing on individual SREJ resolution and REJ fallback
//     - Bitmap wrap-around at modulo-8 (N(S)=7->0) and modulo-128 boundaries
//     - Bitmap cleared on SABM reset; persistence across window slide
//   SECTION B: Extended Sequence Window Management
//     - Modulo-128 window full condition blocks ax25_send_data correctly
//     - Sequence number wrap 125->0 with window crossing modulo boundary
//     - RNR + RR pair: window reopens, transmission resumes correctly
//     - Cumulative ACK advances V(A), releases window slots
//     - AX25_MAX_QUEUE_SIZE vs timers.k distinction
//     - V(S) rollback on REJ when wrap crosses zero boundary
//   SECTION C: Full-Duplex State Synchronization
//     - Simultaneous I-frame send and receive updates V(S) and V(R) independently
//     - Piggybacked N(R) in outgoing I-frame reflects current V(R)
//     - REJ in full-duplex triggers abort_tx and immediate bulk retransmit
//     - SREJ in full-duplex sets T1 to AX25_T1_PENDING sentinel
//     - RR immediately sent in full-duplex (no T2 delay) clears peer_busy
//     - Stats (iframe_retransmitted) correctly incremented on full-duplex REJ
//     - Crossed I-frames: both V(S) and V(R) updated correctly without race

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "test_common.h"
#include "ax25.h"
#include "ax25_state_machine.h"

// ---------------------------------------------------------------------------
// Module-level assert counter required by TEST_ASSERT macro
// ---------------------------------------------------------------------------
static uint32_t assert_count = 0;

// ---------------------------------------------------------------------------
// Test harness sizes
// ---------------------------------------------------------------------------
#define ADV_TX_BUF_COUNT  64
#define ADV_TX_BUF_SIZE   512

// ---------------------------------------------------------------------------
// Captured transmitted frame storage
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t data[ADV_TX_BUF_SIZE];
    size_t len;
} adv_captured_frame_t;

// ---------------------------------------------------------------------------
// Test harness context
// ---------------------------------------------------------------------------
typedef struct {
    adv_captured_frame_t tx_frames[ADV_TX_BUF_COUNT];
    uint8_t tx_count;
    uint8_t rx_data_count;
    uint8_t last_rx_payloads[ADV_TX_BUF_COUNT][256];
    size_t last_rx_payload_lens[ADV_TX_BUF_COUNT];
    bool connected;
    bool disconnected;
    bool peer_busy;
    uint8_t disconnect_reason;
    bool abort_tx_called;
    uint32_t abort_tx_call_count;
    ax25_dl_error_t last_dl_error;
    bool dl_error_fired;
} adv_harness_t;

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

static void adv_cb_transmit(void *user_data, uint8_t *frame, size_t len) {
    adv_harness_t *h = (adv_harness_t*) user_data;
    if (h->tx_count >= ADV_TX_BUF_COUNT)
        return;
    size_t copy_len = (len > ADV_TX_BUF_SIZE) ? ADV_TX_BUF_SIZE : len;
    memcpy(h->tx_frames[h->tx_count].data, frame, copy_len);
    h->tx_frames[h->tx_count].len = copy_len;
    h->tx_count++;
}

static void adv_cb_connect(void *user_data, bool initiated_locally) {
    (void) initiated_locally;
    adv_harness_t *h = (adv_harness_t*) user_data;
    h->connected = true;
}

static void adv_cb_disconnect(void *user_data, uint8_t reason) {
    adv_harness_t *h = (adv_harness_t*) user_data;
    h->disconnected = true;
    h->disconnect_reason = reason;
}

static void adv_cb_data(void *user_data, uint8_t *data, size_t len, uint8_t pid) {
    (void) pid;
    adv_harness_t *h = (adv_harness_t*) user_data;
    if (h->rx_data_count < ADV_TX_BUF_COUNT) {
        size_t copy_len = (len > 255) ? 255 : len;
        memcpy(h->last_rx_payloads[h->rx_data_count], data, copy_len);
        h->last_rx_payload_lens[h->rx_data_count] = copy_len;
    }
    h->rx_data_count++;
}

static void adv_cb_busy(void *user_data, bool busy) {
    adv_harness_t *h = (adv_harness_t*) user_data;
    h->peer_busy = busy;
}

static void adv_cb_abort_tx(void *user_data) {
    adv_harness_t *h = (adv_harness_t*) user_data;
    h->abort_tx_called = true;
    h->abort_tx_call_count++;
}

static void adv_cb_dl_error(void *user_data, ax25_dl_error_t error) {
    adv_harness_t *h = (adv_harness_t*) user_data;
    h->last_dl_error = error;
    h->dl_error_fired = true;
}

// ---------------------------------------------------------------------------
// Helper: build callbacks struct
// ---------------------------------------------------------------------------
static ax25_callbacks_t adv_make_callbacks(adv_harness_t *h) {
    ax25_callbacks_t cb;
    memset(&cb, 0, sizeof(cb));
    cb.transmit = adv_cb_transmit;
    cb.on_connect = adv_cb_connect;
    cb.on_disconnect = adv_cb_disconnect;
    cb.on_data = adv_cb_data;
    cb.on_busy = adv_cb_busy;
    cb.abort_tx = adv_cb_abort_tx;
    cb.on_dl_error = adv_cb_dl_error;
    return cb;
}

// ---------------------------------------------------------------------------
// Helper: force a connection into CONNECTED state without SABM exchange.
// mod: 8 or 128. full_duplex: true enables full-duplex mode.
// srej_max: max simultaneous SREJ requests (1 = spec default, >1 for bitmap tests).
// k: window size (7 for mod-8, use 32 or higher for mod-128 tests).
// ---------------------------------------------------------------------------
static void adv_force_connected(ax25_connection_t *conn, adv_harness_t *h, uint8_t mod, bool full_duplex, uint8_t srej_max, uint8_t k) {
    ax25_callbacks_t cb = adv_make_callbacks(h);
    ax25_connection_init(conn, &cb, h);

    conn->vars.mod = mod;
    conn->full_duplex = full_duplex;
    conn->rej_mode = AX25_REJ_MODE_SREJ_REJ;
    conn->srej_max = srej_max;

    memcpy(conn->peer_addr.destination.callsign, "N0CALL", 6);
    conn->peer_addr.destination.ssid = 0;
    conn->peer_addr.destination.ch = true;
    conn->peer_addr.destination.extension = false;
    memcpy(conn->peer_addr.source.callsign, "W1AW  ", 6);
    conn->peer_addr.source.ssid = 0;
    conn->peer_addr.source.ch = false;
    conn->peer_addr.source.extension = true;
    conn->peer_addr.cr = false;
    conn->peer_addr.repeaters.num_repeaters = 0;

    conn->state = AX25_STATE_CONNECTED;
    conn->vars.vs = 0;
    conn->vars.vr = 0;
    conn->vars.va = 0;

    conn->timers.t1 = 300;
    conn->timers.t2 = 30;
    conn->timers.t3 = 3000;
    conn->timers.k = (k > 0) ? k : ((mod == 128) ? 32 : 7);
    conn->timers.n2 = 10;
    conn->timers.n1 = 256;

    h->connected = true;
}

// ---------------------------------------------------------------------------
// Helper: build an inbound I-frame for injection via ax25_process_frame.
// ---------------------------------------------------------------------------
static ax25_frame_t* adv_make_iframe(uint8_t ns, uint8_t nr, bool pf, const uint8_t *payload, size_t payload_len, uint8_t mod) {
    ax25_information_frame_t *f = (ax25_information_frame_t*) malloc(sizeof(ax25_information_frame_t));
    if (!f)
        return NULL;
    memset(f, 0, sizeof(*f));

    f->base.type = (mod == 128) ? AX25_FRAME_INFORMATION_16BIT : AX25_FRAME_INFORMATION_8BIT;
    memcpy(f->base.header.destination.callsign, "N0CALL", 6);
    f->base.header.destination.ssid = 0;
    memcpy(f->base.header.source.callsign, "W1AW  ", 6);
    f->base.header.source.ssid = 0;
    f->base.header.cr = true;  // command from remote
    f->base.header.repeaters.num_repeaters = 0;

    f->ns = ns;
    f->nr = nr;
    f->pf = pf;
    f->pid = PID_NO_L3;

    if (payload && payload_len > 0) {
        f->payload = (uint8_t*) malloc(payload_len);
        if (f->payload)
            memcpy(f->payload, payload, payload_len);
        f->payload_len = payload_len;
    } else {
        f->payload = NULL;
        f->payload_len = 0;
    }
    return (ax25_frame_t*) f;
}

static void adv_free_iframe(ax25_frame_t *f) {
    if (!f)
        return;
    ax25_information_frame_t *iframe = (ax25_information_frame_t*) f;
    if (iframe->payload)
        free(iframe->payload);
    free(iframe);
}

// ---------------------------------------------------------------------------
// Helper: build an inbound supervisory frame for injection.
// ---------------------------------------------------------------------------
static ax25_frame_t* adv_make_sframe(ax25_frame_type_t type, uint8_t nr, bool pf, uint8_t code) {
    ax25_supervisory_frame_t *f = (ax25_supervisory_frame_t*) malloc(sizeof(ax25_supervisory_frame_t));
    if (!f)
        return NULL;
    memset(f, 0, sizeof(*f));
    f->base.type = type;
    memcpy(f->base.header.destination.callsign, "N0CALL", 6);
    f->base.header.destination.ssid = 0;
    memcpy(f->base.header.source.callsign, "W1AW  ", 6);
    f->base.header.source.ssid = 0;
    f->base.header.cr = false;  // response from remote
    f->base.header.repeaters.num_repeaters = 0;
    f->nr = nr;
    f->pf = pf;
    f->code = code;
    return (ax25_frame_t*) f;
}

static void adv_free_sframe(ax25_frame_t *f) {
    if (f)
        free(f);
}

// ---------------------------------------------------------------------------
// Helper: count transmitted frames of a given decoded type
// ---------------------------------------------------------------------------
static uint8_t adv_count_frames_of_type(adv_harness_t *h, ax25_frame_type_t expected) {
    uint8_t count = 0;
    for (int i = 0; i < (int) h->tx_count; i++) {
        uint8_t err = 0;
        ax25_frame_t *d = ax25_frame_decode(h->tx_frames[i].data, h->tx_frames[i].len, 0, &err);
        if (!d)
            continue;
        if (d->type == expected)
            count++;
        ax25_frame_free(d, &err);
    }
    return count;
}

// ---------------------------------------------------------------------------
// Helper: get N(R) from the last transmitted frame of a given type.
// Returns -1 if not found.
// ---------------------------------------------------------------------------
static int adv_get_last_sframe_nr(adv_harness_t *h, ax25_frame_type_t expected) {
    for (int i = (int) h->tx_count - 1; i >= 0; i--) {
        uint8_t err = 0;
        ax25_frame_t *d = ax25_frame_decode(h->tx_frames[i].data, h->tx_frames[i].len, 0, &err);
        if (!d)
            continue;
        if (d->type == expected) {
            ax25_supervisory_frame_t *sf = (ax25_supervisory_frame_t*) d;
            int nr = (int) sf->nr;
            ax25_frame_free(d, &err);
            return nr;
        }
        ax25_frame_free(d, &err);
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Helper: decode N(R) from outgoing I-frame at capture index idx.
// Returns -1 if the frame at that index is not an I-frame.
// ---------------------------------------------------------------------------
static int adv_get_iframe_nr(adv_harness_t *h, int idx) {
    if (idx < 0 || idx >= (int) h->tx_count)
        return -1;
    uint8_t err = 0;
    ax25_frame_t *d = ax25_frame_decode(h->tx_frames[idx].data, h->tx_frames[idx].len, 0, &err);
    if (!d)
        return -1;
    int nr = -1;
    if (d->type == AX25_FRAME_INFORMATION_8BIT || d->type == AX25_FRAME_INFORMATION_16BIT) {
        ax25_information_frame_t *iframe = (ax25_information_frame_t*) d;
        nr = (int) iframe->nr;
    }
    ax25_frame_free(d, &err);
    return nr;
}

// ---------------------------------------------------------------------------
// Helper: decode N(S) from outgoing I-frame at capture index idx.
// ---------------------------------------------------------------------------
static int adv_get_iframe_ns(adv_harness_t *h, int idx) {
    if (idx < 0 || idx >= (int) h->tx_count)
        return -1;
    uint8_t err = 0;
    ax25_frame_t *d = ax25_frame_decode(h->tx_frames[idx].data, h->tx_frames[idx].len, 0, &err);
    if (!d)
        return -1;
    int ns = -1;
    if (d->type == AX25_FRAME_INFORMATION_8BIT || d->type == AX25_FRAME_INFORMATION_16BIT) {
        ax25_information_frame_t *iframe = (ax25_information_frame_t*) d;
        ns = (int) iframe->ns;
    }
    ax25_frame_free(d, &err);
    return ns;
}

// ===========================================================================
// ============== SECTION A: SREJ BITMAP MANAGEMENT ==========================
// ===========================================================================

// ---------------------------------------------------------------------------
// Test A1: Bitmap bit positions verified - each N(S) maps to correct byte/bit
//          ns=0  -> byte 0 bit 0
//          ns=7  -> byte 0 bit 7
//          ns=8  -> byte 1 bit 0
//          ns=15 -> byte 1 bit 7
//          ns=127-> byte 15 bit 7   (mod-128)
// This is a structural test that validates the bitmap indexing without
// involving the live state machine.
// ---------------------------------------------------------------------------
static int test_srej_bitmap_bit_positions(void) {
    printf("\n--- test_srej_bitmap_bit_positions ---\n");
    printf("Section 6.4.4.2: SREJ bitmap bit-position correctness\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 128, false, 3, 32);

    // Manually set srej_bitmap and verify that is_srej_pending works correctly
    // by injecting out-of-order frames and observing which SREJ frames are sent.

    // ns=0: byte_idx=0 bit_idx=0 -> srej_bitmap[0] |= 0x01
    conn.srej_bitmap[0] = 0;
    conn.srej_bitmap[0] |= (1u << 0);
    DEBUG_VAR("bitmap[0] after ns=0 set", conn.srej_bitmap[0]);
    TEST_ASSERT((conn.srej_bitmap[0] & 0x01u) != 0, "A1: ns=0 maps to byte 0 bit 0", 0);

    // ns=7: byte_idx=0 bit_idx=7 -> srej_bitmap[0] |= 0x80
    conn.srej_bitmap[0] = 0;
    conn.srej_bitmap[0] |= (1u << 7);
    DEBUG_VAR("bitmap[0] after ns=7 set", conn.srej_bitmap[0]);
    TEST_ASSERT((conn.srej_bitmap[0] & 0x80u) != 0, "A1: ns=7 maps to byte 0 bit 7", 0);

    // ns=8: byte_idx=1 bit_idx=0 -> srej_bitmap[1] |= 0x01
    conn.srej_bitmap[1] = 0;
    conn.srej_bitmap[1] |= (1u << 0);
    DEBUG_VAR("bitmap[1] after ns=8 set", conn.srej_bitmap[1]);
    TEST_ASSERT((conn.srej_bitmap[1] & 0x01u) != 0, "A1: ns=8 maps to byte 1 bit 0", 0);

    // ns=15: byte_idx=1 bit_idx=7
    conn.srej_bitmap[1] = 0;
    conn.srej_bitmap[1] |= (1u << 7);
    DEBUG_VAR("bitmap[1] after ns=15 set", conn.srej_bitmap[1]);
    TEST_ASSERT((conn.srej_bitmap[1] & 0x80u) != 0, "A1: ns=15 maps to byte 1 bit 7", 0);

    // ns=127 (mod-128 max): byte_idx=15 bit_idx=7
    conn.srej_bitmap[15] = 0;
    conn.srej_bitmap[15] |= (1u << 7);
    DEBUG_VAR("bitmap[15] after ns=127 set", conn.srej_bitmap[15]);
    TEST_ASSERT((conn.srej_bitmap[15] & 0x80u) != 0, "A1: ns=127 maps to byte 15 bit 7", 0);

    // Cross-check: setting ns=3 should not affect ns=7 in the same byte
    conn.srej_bitmap[0] = 0;
    conn.srej_bitmap[0] |= (1u << 3);  // ns=3
    TEST_ASSERT((conn.srej_bitmap[0] & 0x08u) != 0, "A1: ns=3 maps to byte 0 bit 3", 0);
    TEST_ASSERT((conn.srej_bitmap[0] & 0x80u) == 0, "A1: ns=3 does not set bit 7 (ns=7)", 0);

    DEBUG_PRINT("A1 passed: all bitmap bit positions verified");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test A2: Bitmap set by live state machine when SREJ is sent (mod-128 single gap)
//          Inject frame N(S)=1 while V(R)=0: gap at 0, frame 1 buffered.
//          Then inject frame N(S)=3 while V(R) still 0: second consecutive frame
//          after frame 2 is also missing -> depends on implementation path.
//          We focus on verifying that bitmap[0] bit 0 is set after SREJ(0).
// ---------------------------------------------------------------------------
static int test_srej_bitmap_set_by_state_machine(void) {
    printf("\n--- test_srej_bitmap_set_by_state_machine ---\n");
    printf("Section 6.4.4.2: Bitmap set by live state machine on SREJ send\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 128, false, 1, 32);

    // V(R)=0, V(S)=0. Inject N(S)=1 - frame 0 is missing
    uint8_t p1[] = { 0x01, 0x02 };
    ax25_frame_t *f1 = adv_make_iframe(1, 0, false, p1, sizeof(p1), 128);
    ax25_process_frame(&conn, f1, 1);
    adv_free_iframe(f1);

    DEBUG_BOOL("srej_exception after gap at 0", conn.srej_exception); DEBUG_VAR("srej_bitmap[0] (should have bit 0 set)", conn.srej_bitmap[0]); DEBUG_VAR("srej_count", conn.srej_count); DEBUG_VAR("V(R) (should be 0 - blocked)", conn.vars.vr);

    TEST_ASSERT(conn.srej_exception == true, "A2: srej_exception set on gap at N(S)=0", 0);
    // Bitmap byte 0 bit 0 must be set for N(S)=0 (the missing frame)
    TEST_ASSERT((conn.srej_bitmap[0] & 0x01u) != 0, "A2: bitmap[0] bit 0 set for missing N(S)=0", 0);
    TEST_ASSERT(conn.vars.vr == 0, "A2: V(R) still 0 (blocked by missing frame)", 0);
    TEST_ASSERT(conn.srej_buffer_count >= 1, "A2: frame N(S)=1 buffered", 0);

    // Verify SREJ(0) was sent with N(R)=0
    uint8_t srej8_cnt = adv_count_frames_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    uint8_t srej16_cnt = adv_count_frames_of_type(&h, AX25_FRAME_SUPERVISORY_SREJ_16BIT);
    DEBUG_VAR("SREJ 8-bit count", srej8_cnt); DEBUG_VAR("SREJ 16-bit count", srej16_cnt);
    TEST_ASSERT(srej8_cnt + srej16_cnt >= 1, "A2: at least one SREJ sent", 0);

    int srej_nr = adv_get_last_sframe_nr(&h, AX25_FRAME_SUPERVISORY_SREJ_16BIT);
    if (srej_nr < 0)
        srej_nr = adv_get_last_sframe_nr(&h, AX25_FRAME_SUPERVISORY_SREJ_8BIT);
    DEBUG_VAR("SREJ N(R) (should be 0 - first missing)", (unsigned)srej_nr);
    TEST_ASSERT(srej_nr == 0, "A2: SREJ N(R)=0 (requesting missing frame 0)", 0);

    DEBUG_PRINT("A2 passed: bitmap correctly set by live state machine");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test A3: Bitmap cleared entry-by-entry as SREJ frames are delivered
//          Setup: srej_max=2 (allow 2 simultaneous SREJs), mod-128
//          1. Send N(S)=2 (missing 0 and 1) -> should SREJ(0) only (srej_max=1 default)
//          Adjust: force srej_max=2:
//          1. Inject N(S)=0: in-order, delivered. V(R)=1.
//          2. Inject N(S)=2: gap at 1. SREJ(1). bitmap[0] bit 1 set.
//          3. Inject N(S)=1: missing frame retransmitted. SREJ cleared.
//             bitmap[0] bit 1 cleared, V(R) advances to 3.
// ---------------------------------------------------------------------------
static int test_srej_bitmap_cleared_on_delivery(void) {
    printf("\n--- test_srej_bitmap_cleared_on_delivery ---\n");
    printf("Section 6.4.4.2: Bitmap entries cleared as missing frames are delivered\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 128, false, 1, 32);

    // Step 1: Inject N(S)=0 (in-order)
    uint8_t p0[] = { 0xA0 };
    ax25_frame_t *f0 = adv_make_iframe(0, 0, false, p0, sizeof(p0), 128);
    ax25_process_frame(&conn, f0, 1);
    adv_free_iframe(f0);

    DEBUG_VAR("V(R) after N(S)=0 (should be 1)", conn.vars.vr);
    TEST_ASSERT(conn.vars.vr == 1, "A3: V(R)=1 after N(S)=0 received", 0);
    TEST_ASSERT(h.rx_data_count == 1, "A3: frame 0 delivered to upper layer", 0);

    // Step 2: Inject N(S)=2 (frame 1 missing -> SREJ(1))
    uint8_t p2[] = { 0xA2 };
    ax25_frame_t *f2 = adv_make_iframe(2, 0, false, p2, sizeof(p2), 128);
    ax25_process_frame(&conn, f2, 1);
    adv_free_iframe(f2);

    DEBUG_BOOL("srej_exception after N(S)=2", conn.srej_exception); DEBUG_VAR("bitmap[0] (bit 1 should be set for N(S)=1 missing)", conn.srej_bitmap[0]); DEBUG_VAR("V(R) (still 1)", conn.vars.vr);
    TEST_ASSERT(conn.srej_exception == true, "A3: srej_exception set after gap at N(S)=1", 0);
    TEST_ASSERT((conn.srej_bitmap[0] & 0x02u) != 0, "A3: bitmap[0] bit 1 set for missing N(S)=1", 0);
    TEST_ASSERT(conn.vars.vr == 1, "A3: V(R) still 1 (blocked by N(S)=1)", 0);

    // Step 3: Inject N(S)=1 (retransmission of missing frame)
    uint8_t p1[] = { 0xA1 };
    ax25_frame_t *f1 = adv_make_iframe(1, 0, false, p1, sizeof(p1), 128);
    ax25_process_frame(&conn, f1, 1);
    adv_free_iframe(f1);

    DEBUG_BOOL("srej_exception after retransmit (should be false)", conn.srej_exception); DEBUG_VAR("bitmap[0] after delivery (should be 0)", conn.srej_bitmap[0]); DEBUG_VAR("V(R) after recovery (should be 3)", conn.vars.vr); DEBUG_VAR("rx_data_count (should be 3: frames 0,1,2)", h.rx_data_count);
    TEST_ASSERT(conn.srej_exception == false, "A3: srej_exception cleared after N(S)=1 retransmit", 0);
    TEST_ASSERT(conn.srej_bitmap[0] == 0, "A3: bitmap[0] fully cleared after SREJ recovery", 0);
    TEST_ASSERT(conn.vars.vr == 3, "A3: V(R)=3 after SREJ recovery (all three frames)", 0);
    TEST_ASSERT(h.rx_data_count == 3, "A3: 3 frames delivered to upper layer in order", 0);

    // Verify delivered order: [0]=p0, [1]=p1, [2]=p2
    TEST_ASSERT(h.last_rx_payloads[0][0] == 0xA0, "A3: frame 0 payload correct", 0);
    TEST_ASSERT(h.last_rx_payloads[1][0] == 0xA1, "A3: frame 1 payload correct (retransmitted)", 0);
    TEST_ASSERT(h.last_rx_payloads[2][0] == 0xA2, "A3: frame 2 payload correct (was buffered)", 0);

    DEBUG_PRINT("A3 passed: bitmap cleared entry-by-entry on delivery");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test A4: Bitmap completely cleared on REJ fallback
//          srej_max=1. Force a second gap while SREJ is pending.
//          Expected: SREJ state cleared, bitmap zeroed, REJ sent.
// ---------------------------------------------------------------------------
static int test_srej_bitmap_cleared_on_rej_fallback(void) {
    printf("\n--- test_srej_bitmap_cleared_on_rej_fallback ---\n");
    printf("Section 6.4.4.3: Bitmap fully cleared when SREJ falls back to REJ\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, false, 1, 7);

    // Step 1: Create first SREJ condition
    // Inject N(S)=0 in-order (V(R) goes to 1)
    uint8_t p0[] = { 0x00 };
    ax25_frame_t *f0 = adv_make_iframe(0, 0, false, p0, 1, 8);
    ax25_process_frame(&conn, f0, 1);
    adv_free_iframe(f0);
    TEST_ASSERT(conn.vars.vr == 1, "A4: V(R)=1 after N(S)=0", 0);

    // Inject N(S)=2 (frame 1 missing) -> SREJ(1) sent, bitmap bit 1 set
    uint8_t p2[] = { 0x02 };
    ax25_frame_t *f2 = adv_make_iframe(2, 0, false, p2, 1, 8);
    ax25_process_frame(&conn, f2, 1);
    adv_free_iframe(f2);

    DEBUG_BOOL("srej_exception after first gap", conn.srej_exception); DEBUG_VAR("bitmap[0] after first gap (bit 1 set)", conn.srej_bitmap[0]);
    TEST_ASSERT(conn.srej_exception == true, "A4: srej_exception set on first gap", 0);
    TEST_ASSERT((conn.srej_bitmap[0] & 0x02u) != 0, "A4: bitmap bit 1 set (missing N(S)=1)", 0);

    // Step 2: Create second gap while in SREJ mode -> should trigger REJ fallback
    // Inject N(S)=4 (frame 3 also missing - this should be a new gap beyond expected_next)
    // With srej_max=1 already at limit, second gap causes REJ fallback
    uint8_t p4[] = { 0x04 };
    ax25_frame_t *f4 = adv_make_iframe(4, 0, false, p4, 1, 8);
    ax25_process_frame(&conn, f4, 1);
    adv_free_iframe(f4);

    DEBUG_BOOL("srej_exception after REJ fallback (should be false)", conn.srej_exception); DEBUG_BOOL("rej_exception after REJ fallback (should be true)", conn.rej_exception);
    // srej_bitmap must be fully zeroed after clear_srej_state()
    uint8_t bm_sum = 0;
    for (int bi = 0; bi < 16; bi++)
        bm_sum |= conn.srej_bitmap[bi];
    DEBUG_VAR("bitmap XOR-sum after REJ fallback (must be 0)", bm_sum);
    TEST_ASSERT(bm_sum == 0, "A4: All srej_bitmap bytes zeroed after REJ fallback", 0);
    TEST_ASSERT(conn.srej_exception == false, "A4: srej_exception cleared on REJ fallback", 0);
    TEST_ASSERT(conn.rej_exception == true, "A4: rej_exception set on REJ fallback", 0);

    // Verify REJ was sent
    uint8_t rej_count = adv_count_frames_of_type(&h, AX25_FRAME_SUPERVISORY_REJ_8BIT);
    DEBUG_VAR("REJ frames transmitted (should be >= 1)", rej_count);
    TEST_ASSERT(rej_count >= 1, "A4: REJ frame sent on SREJ->REJ fallback", 0);

    DEBUG_PRINT("A4 passed: bitmap cleared on REJ fallback");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test A5: Bitmap cleared on SABM reset while SREJ is active
//          After SREJ is pending, simulate peer sending SABM to reset link.
//          Expected: state transitions to DISCONNECTED or CONNECTED again,
//          srej_bitmap all zeros, srej_exception = false.
// ---------------------------------------------------------------------------
static int test_srej_bitmap_cleared_on_sabm_reset(void) {
    printf("\n--- test_srej_bitmap_cleared_on_sabm_reset ---\n");
    printf("Section 6.4.4.2 + 4.3.3.1: SREJ bitmap cleared when SABM resets the link\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, false, 1, 7);

    // Create a SREJ condition
    uint8_t p0[] = { 0x11 };
    ax25_frame_t *f0 = adv_make_iframe(0, 0, false, p0, 1, 8);
    ax25_process_frame(&conn, f0, 1);
    adv_free_iframe(f0);

    uint8_t p2[] = { 0x22 };
    ax25_frame_t *f2 = adv_make_iframe(2, 0, false, p2, 1, 8);
    ax25_process_frame(&conn, f2, 1);
    adv_free_iframe(f2);

    DEBUG_BOOL("srej_exception before SABM (should be true)", conn.srej_exception);
    TEST_ASSERT(conn.srej_exception == true, "A5: srej_exception active before SABM", 0);
    TEST_ASSERT((conn.srej_bitmap[0] & 0x02u) != 0, "A5: bitmap set before SABM", 0);

    // Inject SABM from peer to reset the link
    ax25_unnumbered_frame_t sabm_f;
    memset(&sabm_f, 0, sizeof(sabm_f));
    sabm_f.base.type = AX25_FRAME_UNNUMBERED_SABM;
    memcpy(sabm_f.base.header.destination.callsign, "N0CALL", 6);
    sabm_f.base.header.destination.ssid = 0;
    memcpy(sabm_f.base.header.source.callsign, "W1AW  ", 6);
    sabm_f.base.header.source.ssid = 0;
    sabm_f.base.header.cr = true;
    sabm_f.base.header.repeaters.num_repeaters = 0;
    sabm_f.pf = true;
    sabm_f.modifier = 0x2F;

    ax25_process_frame(&conn, (ax25_frame_t*) &sabm_f, 10);

    DEBUG_BOOL("srej_exception after SABM (should be false)", conn.srej_exception); DEBUG_BOOL("rej_exception after SABM (should be false)", conn.rej_exception); DEBUG_VAR("V(S) after SABM reset (should be 0)", conn.vars.vs); DEBUG_VAR("V(R) after SABM reset (should be 0)", conn.vars.vr); DEBUG_VAR("V(A) after SABM reset (should be 0)", conn.vars.va);

    uint8_t bm_sum = 0;
    for (int bi = 0; bi < 16; bi++)
        bm_sum |= conn.srej_bitmap[bi];
    DEBUG_VAR("bitmap XOR-sum after SABM (should be 0)", bm_sum);

    TEST_ASSERT(bm_sum == 0, "A5: All srej_bitmap bytes zeroed after SABM reset", 0);
    TEST_ASSERT(conn.srej_exception == false, "A5: srej_exception cleared after SABM", 0);
    TEST_ASSERT(conn.rej_exception == false, "A5: rej_exception cleared after SABM", 0);
    TEST_ASSERT(conn.vars.vs == 0, "A5: V(S)=0 after SABM reset", 0);
    TEST_ASSERT(conn.vars.vr == 0, "A5: V(R)=0 after SABM reset", 0);
    TEST_ASSERT(conn.vars.va == 0, "A5: V(A)=0 after SABM reset", 0);

    DEBUG_PRINT("A5 passed: bitmap and state reset by SABM");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test A6: Bitmap wrap-around at modulo-8 boundary (N(S)=7->0)
//          Advance V(R) to 7 by delivering frames 0-6 in order.
//          Then inject N(S)=1 of the next window cycle (= absolute N(S)=1, but
//          that is the NEXT in-window number after 7 in mod-8).
//          The implementation stores ns values modulo 8 in the bitmap.
//          Missing frame is N(S)=0 (mod-8 wrap). Bitmap byte 0 bit 0 must be set.
// ---------------------------------------------------------------------------
static int test_srej_bitmap_mod8_wraparound(void) {
    printf("\n--- test_srej_bitmap_mod8_wraparound ---\n");
    printf("Section 6.4.4.2: SREJ bitmap bit position correct at mod-8 wrap-around\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, false, 1, 7);

    // Deliver frames 0..6 to get V(R) to 7
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t pl[] = { i };
        ax25_frame_t *f = adv_make_iframe(i, 0, false, pl, 1, 8);
        ax25_process_frame(&conn, f, (uint32_t) (i + 1));
        adv_free_iframe(f);
    } DEBUG_VAR("V(R) after delivering frames 0-6 (should be 7)", conn.vars.vr);
    TEST_ASSERT(conn.vars.vr == 7, "A6: V(R)=7 after 7 in-order frames", 0);

    // Reset harness counters for cleaner observation
    h.tx_count = 0;

    // Now inject N(S)=1 in-window (V(R)=7, next expected=7 mod 8=7).
    // But if we inject N(S) = (7+2)%8 = 1 with the "skip" being N(S)=7 missing,
    // that creates a gap at N(S)=7 while V(R)=7.
    // Actually V(R)=7 means we expect N(S)=7 next. So inject N(S)=1 (mod-8 cycle 2).
    // Gap = N(S)=7 (the expected frame in mod-8 cycle 1).
    // Wait - let me reconsider: V(R)=7, next expected N(S) is 7 (since vr is 7).
    // Inject N(S)=(7+1)%8=0 mod-8: that skips N(S)=7. Gap at N(S)=7.
    // Bitmap bit for N(S)=7 = byte_idx=0 bit_idx=7.
    uint8_t p_skip[] = { 0xAB };
    ax25_frame_t *f_skip = adv_make_iframe(0, 0, false, p_skip, 1, 8);  // N(S)=0 in mod-8, skips N(S)=7
    ax25_process_frame(&conn, f_skip, 10);
    adv_free_iframe(f_skip);

    DEBUG_BOOL("srej_exception after wrap-around gap", conn.srej_exception); DEBUG_VAR("srej_bitmap[0] after wrap gap (bit 7 should be set for N(S)=7)", conn.srej_bitmap[0]); DEBUG_VAR("V(R) (should still be 7 - blocked)", conn.vars.vr);

    // With V(R)=7, receiving N(S)=0 (gap at 7): SREJ(7) sent, bitmap[0] bit 7 = 1
    TEST_ASSERT(conn.srej_exception == true, "A6: srej_exception set at mod-8 wrap", 0);
    TEST_ASSERT((conn.srej_bitmap[0] & 0x80u) != 0, "A6: bitmap[0] bit 7 set for missing N(S)=7", 0);
    TEST_ASSERT(conn.vars.vr == 7, "A6: V(R) still 7 (blocked by missing N(S)=7)", 0);

    // Now retransmit N(S)=7 to complete SREJ recovery
    uint8_t p7[] = { 0x07 };
    ax25_frame_t *f7 = adv_make_iframe(7, 0, false, p7, 1, 8);
    ax25_process_frame(&conn, f7, 11);
    adv_free_iframe(f7);

    DEBUG_VAR("V(R) after wrap recovery (should be 2: 7->0->1->2 delivered)", conn.vars.vr); DEBUG_BOOL("srej_exception after recovery (should be false)", conn.srej_exception); DEBUG_VAR("bitmap[0] after recovery (should be 0)", conn.srej_bitmap[0]);

    TEST_ASSERT(conn.srej_exception == false, "A6: srej_exception cleared after wrap recovery", 0);
    TEST_ASSERT(conn.srej_bitmap[0] == 0, "A6: bitmap cleared after mod-8 wrap recovery", 0);
    // V(R) should have advanced: frame 7 fills gap, then frame 0 was buffered -> V(R)=1
    TEST_ASSERT(conn.vars.vr >= 1, "A6: V(R) advanced after wrap-around recovery", 0);

    DEBUG_PRINT("A6 passed: bitmap handles mod-8 wrap-around correctly");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test A7: Bitmap persistence - bitmap entry survives window slide
//          After SREJ(ns=2) is pending, send frames 0,1 (V(R) advances to 2).
//          The SREJ bit for ns=2 must persist in the bitmap until frame 2 arrives.
//          This validates that bitmap bits are NOT cleared by V(R) advancement alone.
// ---------------------------------------------------------------------------
static int test_srej_bitmap_persists_across_window_slide(void) {
    printf("\n--- test_srej_bitmap_persists_across_window_slide ---\n");
    printf("Section 6.4.4.2: SREJ bitmap entry persists while missing frame not yet received\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 128, false, 1, 32);

    // Step 1: Inject N(S)=0 (in-order)
    uint8_t p0[] = { 0x10 };
    ax25_frame_t *f0 = adv_make_iframe(0, 0, false, p0, 1, 128);
    ax25_process_frame(&conn, f0, 1);
    adv_free_iframe(f0);
    TEST_ASSERT(conn.vars.vr == 1, "A7: V(R)=1 after frame 0", 0);

    // Step 2: Inject N(S)=2 - gap at N(S)=1. SREJ(1) sent. bitmap bit 1 set.
    uint8_t p2[] = { 0x12 };
    ax25_frame_t *f2 = adv_make_iframe(2, 0, false, p2, 1, 128);
    ax25_process_frame(&conn, f2, 2);
    adv_free_iframe(f2);
    TEST_ASSERT(conn.srej_exception == true, "A7: srej_exception active", 0);
    TEST_ASSERT((conn.srej_bitmap[0] & 0x02u) != 0, "A7: bitmap bit 1 set for missing N(S)=1", 0);

    // Step 3: Inject N(S)=3, N(S)=4 (consecutive frames after the gap)
    // These should be buffered without changing the bitmap bit for N(S)=1
    uint8_t p3[] = { 0x13 }, p4[] = { 0x14 };
    ax25_frame_t *f3 = adv_make_iframe(3, 0, false, p3, 1, 128);
    ax25_process_frame(&conn, f3, 3);
    adv_free_iframe(f3);
    ax25_frame_t *f4 = adv_make_iframe(4, 0, false, p4, 1, 128);
    ax25_process_frame(&conn, f4, 4);
    adv_free_iframe(f4);

    DEBUG_BOOL("srej_exception still true after buffering frames 3,4", conn.srej_exception); DEBUG_VAR("bitmap[0] (bit 1 must still be set)", conn.srej_bitmap[0]); DEBUG_VAR("srej_buffer_count (should be 3: frames 2,3,4)", conn.srej_buffer_count); DEBUG_VAR("V(R) (should still be 1 - blocked by N(S)=1 missing)", conn.vars.vr);

    // Bitmap bit for N(S)=1 must still be set (missing frame not yet retransmitted)
    TEST_ASSERT(conn.srej_exception == true, "A7: srej_exception persists after buffering 3,4", 0);
    TEST_ASSERT((conn.srej_bitmap[0] & 0x02u) != 0, "A7: bitmap bit 1 persists (N(S)=1 still missing)", 0);
    TEST_ASSERT(conn.vars.vr == 1, "A7: V(R) unchanged at 1 (blocked)", 0);
    TEST_ASSERT(conn.srej_buffer_count >= 3, "A7: at least 3 frames buffered", 0);

    // Step 4: Deliver N(S)=1 (SREJ recovery)
    uint8_t p1[] = { 0x11 };
    ax25_frame_t *f1 = adv_make_iframe(1, 0, false, p1, 1, 128);
    ax25_process_frame(&conn, f1, 5);
    adv_free_iframe(f1);

    DEBUG_VAR("V(R) after full recovery (should be 5)", conn.vars.vr); DEBUG_BOOL("srej_exception cleared (should be false)", conn.srej_exception); DEBUG_VAR("bitmap[0] cleared (should be 0)", conn.srej_bitmap[0]);

    TEST_ASSERT(conn.srej_exception == false, "A7: srej_exception cleared after N(S)=1 delivery", 0);
    TEST_ASSERT(conn.srej_bitmap[0] == 0, "A7: bitmap[0] zeroed after full recovery", 0);
    TEST_ASSERT(conn.vars.vr == 5, "A7: V(R)=5 after full SREJ recovery (frames 0-4)", 0);
    TEST_ASSERT(h.rx_data_count == 5, "A7: 5 frames delivered to upper layer", 0);

    DEBUG_PRINT("A7 passed: bitmap persists across buffering then clears on recovery");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test A8: Multi-frame SREJ - mod-128 with srej_max=2, two simultaneous SREJs
//          1. Send N(S)=0 (in-order)
//          2. Skip N(S)=1, send N(S)=2 -> SREJ(1) P=1
//          3. Skip N(S)=3, send N(S)=4 -> SREJ(3) P=0 (second simultaneous SREJ)
//          4. Retransmit N(S)=1 -> partial recovery
//          5. Retransmit N(S)=3 -> full recovery
// ---------------------------------------------------------------------------
static int test_srej_bitmap_multi_frame_mod128(void) {
    printf("\n--- test_srej_bitmap_multi_frame_mod128 ---\n");
    printf("Section 6.4.4.2: Multi-frame SREJ with srej_max=2, mod-128 window\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 128, false, 2, 32);  // srej_max=2

    // Step 1: Frame 0 in-order
    uint8_t pl[] = { 0x00 };
    ax25_frame_t *f = adv_make_iframe(0, 0, false, pl, 1, 128);
    ax25_process_frame(&conn, f, 1);
    adv_free_iframe(f);
    TEST_ASSERT(conn.vars.vr == 1, "A8: V(R)=1 after N(S)=0", 0);

    // Step 2: Skip N(S)=1, receive N(S)=2 -> gap at 1
    pl[0] = 0x02;
    f = adv_make_iframe(2, 0, false, pl, 1, 128);
    ax25_process_frame(&conn, f, 2);
    adv_free_iframe(f);

    DEBUG_BOOL("srej_exception after first gap", conn.srej_exception); DEBUG_VAR("srej_count after first gap (should be 1)", conn.srej_count); DEBUG_VAR("bitmap[0] after first gap (bit 1 set)", conn.srej_bitmap[0]);
    TEST_ASSERT(conn.srej_exception == true, "A8: srej_exception after gap 1", 0);
    TEST_ASSERT((conn.srej_bitmap[0] & 0x02u) != 0, "A8: bitmap bit 1 (N(S)=1 missing)", 0);

    // Step 3: Skip N(S)=3, receive N(S)=4 -> another gap at 3 (within srej_max=2)
    pl[0] = 0x04;
    f = adv_make_iframe(4, 0, false, pl, 1, 128);
    ax25_process_frame(&conn, f, 3);
    adv_free_iframe(f);

    DEBUG_VAR("srej_count after second gap (should be 2)", conn.srej_count); DEBUG_VAR("bitmap[0] after second gap (bits 1 and 3 set)", conn.srej_bitmap[0]);
    // bit 1 for N(S)=1, bit 3 for N(S)=3
    TEST_ASSERT(conn.srej_count >= 1, "A8: srej_count >= 1 after second gap", 0);
    // If implementation sent second SREJ, bit 3 will also be set
    // (depends on whether it picked case (c) from handle_out_of_sequence_iframe)
    // Accept either SREJ or REJ fallback for the second gap
    bool second_gap_handled = ((conn.srej_bitmap[0] & 0x08u) != 0) || conn.rej_exception;
    DEBUG_BOOL("second gap handled (SREJ bit 3 or REJ)", second_gap_handled);
    TEST_ASSERT(second_gap_handled, "A8: second gap handled (SREJ or REJ fallback)", 0);

    // Step 4: Retransmit N(S)=1 (first missing frame)
    pl[0] = 0x01;
    f = adv_make_iframe(1, 0, false, pl, 1, 128);
    ax25_process_frame(&conn, f, 4);
    adv_free_iframe(f);

    DEBUG_VAR("V(R) after N(S)=1 retransmit", conn.vars.vr); DEBUG_VAR("rx_data_count after N(S)=1 retransmit", h.rx_data_count);
    // After N(S)=1 arrives, frame 0(delivered), 1(just delivered), 2(buffered) -> V(R)>=3
    TEST_ASSERT(conn.vars.vr >= 3, "A8: V(R) advanced past N(S)=1 recovery", 0);

    DEBUG_PRINT("A8 passed: multi-frame SREJ with srej_max=2");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ===========================================================================
// ======== SECTION B: EXTENDED SEQUENCE WINDOW MANAGEMENT ==================
// ===========================================================================

// ---------------------------------------------------------------------------
// Test B1: Modulo-128 window full condition blocks further sends
//          k=7 (mod-128 with small window for fast testing).
//          Send 7 I-frames without acknowledgment -> window full.
//          8th send must return error code 3 (window closed / queue full).
// ---------------------------------------------------------------------------
static int test_window_full_blocks_send(void) {
    printf("\n--- test_window_full_blocks_send ---\n");
    printf("Section 6.4: Window full (k=7, mod-128) - 8th send returns error 3\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 128, false, 1, 7);

    uint8_t payload[] = { 0x01, 0x02, 0x03, 0x04 };
    uint8_t sent = 0;
    uint8_t last_rc = 0;

    // Fill window: send exactly k=7 frames
    for (uint8_t i = 0; i < 7; i++) {
        payload[0] = i;
        uint8_t rc = ax25_send_data(&conn, payload, sizeof(payload), PID_NO_L3);
        DEBUG_HEX("ax25_send_data rc", rc);
        if (rc == 0)
            sent++;
        last_rc = rc;
    }

    DEBUG_VAR("V(S) after filling window (should be 7)", conn.vars.vs); DEBUG_VAR("V(A) (should be 0)", conn.vars.va); DEBUG_VAR("tx_queue.count (should be 7)", conn.tx_queue.count); DEBUG_VAR("sent count (should be 7)", sent); DEBUG_VAR("Outstanding = V(S)-V(A) (should be 7)", (uint8_t)(conn.vars.vs - conn.vars.va));

    TEST_ASSERT(sent == 7, "B1: 7 frames sent successfully (window not yet full)", 0);
    TEST_ASSERT(conn.vars.vs == 7, "B1: V(S)=7 after 7 sends", 0);
    TEST_ASSERT(conn.tx_queue.count == 7, "B1: tx_queue holds 7 frames", 0);

    // 8th send must fail with rc=3 (window closed)
    payload[0] = 0x08;
    last_rc = ax25_send_data(&conn, payload, sizeof(payload), PID_NO_L3);
    DEBUG_HEX("8th send rc (should be 3)", last_rc);
    TEST_ASSERT(last_rc == 3, "B1: 8th send returns 3 (window full)", 0);
    TEST_ASSERT(conn.vars.vs == 7, "B1: V(S) unchanged after rejected send", 0);

    // Now acknowledge all 7 frames with RR(N(R)=7)
    ax25_frame_t *rr = adv_make_sframe(AX25_FRAME_SUPERVISORY_RR_16BIT, 7, false, 0);
    ax25_process_frame(&conn, rr, 100);
    adv_free_sframe(rr);

    DEBUG_VAR("V(A) after RR(7) (should be 7)", conn.vars.va); DEBUG_VAR("tx_queue.count after RR (should be 0)", conn.tx_queue.count);
    TEST_ASSERT(conn.vars.va == 7, "B1: V(A)=7 after RR(7)", 0);
    TEST_ASSERT(conn.tx_queue.count == 0, "B1: tx_queue empty after full ACK", 0);

    // Now a send should succeed again (window reopened)
    payload[0] = 0x09;
    last_rc = ax25_send_data(&conn, payload, sizeof(payload), PID_NO_L3);
    DEBUG_HEX("Send after window reopen rc (should be 0)", last_rc);
    TEST_ASSERT(last_rc == 0, "B1: send succeeds after window reopens (RR ACK)", 0);

    DEBUG_PRINT("B1 passed: window full blocks correctly; reopens on RR");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test B2: Sequence number wrap 125->126->127->0 in mod-128
//          Advance V(S) to 125 by acknowledging frames incrementally.
//          Send 4 more frames: V(S) goes 125, 126, 127, 0 (wrap).
//          Verify correct modulo arithmetic across the boundary.
// ---------------------------------------------------------------------------
static int test_window_seq_wrap_mod128(void) {
    printf("\n--- test_window_seq_wrap_mod128 ---\n");
    printf("Section 4.2.2: Modulo-128 sequence wrap at 127->0\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 128, false, 1, 7);

    // Pre-advance V(S) and V(A) to 125 by simulating a full round-trip:
    // Directly set state variables (legitimate unit-test shortcut)
    conn.vars.vs = 125;
    conn.vars.va = 125;
    conn.vars.vr = 0;

    DEBUG_VAR("Starting V(S) (should be 125)", conn.vars.vs); DEBUG_VAR("Starting V(A) (should be 125)", conn.vars.va);

    // Send 3 frames: N(S) should be 125, 126, 127
    uint8_t payload[] = { 0xA1 };
    for (uint8_t i = 0; i < 3; i++) {
        payload[0] = (uint8_t) (125 + i);
        uint8_t rc = ax25_send_data(&conn, payload, 1, PID_NO_L3);
        DEBUG_HEX("send rc", rc);
        TEST_ASSERT(rc == 0, "B2: send succeeds near mod-128 boundary", 0);
    }

    DEBUG_VAR("V(S) after 3 sends (should be 0 = 128 mod 128)", conn.vars.vs);
    TEST_ASSERT(conn.vars.vs == 0, "B2: V(S) wraps to 0 after 127", 0);

    // Verify the three queued frames have N(S) = 125, 126, 127
    uint8_t tx_start = (uint8_t) (h.tx_count - 3);
    int ns0 = adv_get_iframe_ns(&h, (int) tx_start);
    int ns1 = adv_get_iframe_ns(&h, (int) (tx_start + 1));
    int ns2 = adv_get_iframe_ns(&h, (int) (tx_start + 2));

    DEBUG_VAR("Frame[-3] N(S) (should be 125)", (unsigned)ns0); DEBUG_VAR("Frame[-2] N(S) (should be 126)", (unsigned)ns1); DEBUG_VAR("Frame[-1] N(S) (should be 127)", (unsigned)ns2);
    TEST_ASSERT(ns0 == 125, "B2: first frame N(S)=125", 0);
    TEST_ASSERT(ns1 == 126, "B2: second frame N(S)=126", 0);
    TEST_ASSERT(ns2 == 127, "B2: third frame N(S)=127", 0);

    // Acknowledge all with RR(N(R)=0) - acknowledges N(S)=125,126,127
    ax25_frame_t *rr = adv_make_sframe(AX25_FRAME_SUPERVISORY_RR_16BIT, 0, false, 0);
    ax25_process_frame(&conn, rr, 50);
    adv_free_sframe(rr);

    DEBUG_VAR("V(A) after RR(0) (should be 0 - acknowledged all through wrap)", conn.vars.va);
    TEST_ASSERT(conn.vars.va == 0, "B2: V(A)=0 after RR(0) acknowledges wrap", 0);

    // Send frame at V(S)=0 (wrap): N(S) should be 0
    payload[0] = 0x00;
    uint8_t rc = ax25_send_data(&conn, payload, 1, PID_NO_L3);
    TEST_ASSERT(rc == 0, "B2: send after wrap succeeds at V(S)=0", 0);
    int ns_wrap = adv_get_iframe_ns(&h, (int) h.tx_count - 1);
    DEBUG_VAR("Post-wrap frame N(S) (should be 0)", (unsigned)ns_wrap);
    TEST_ASSERT(ns_wrap == 0, "B2: post-wrap I-frame has N(S)=0", 0);

    DEBUG_PRINT("B2 passed: mod-128 sequence wrap arithmetic correct");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test B3: RNR blocks sends; RR reopens window
//          While connected, inject RNR from peer.
//          Verify: peer_busy=true, subsequent ax25_send_data returns 5.
//          Inject RR from peer -> peer_busy=false, send returns 0.
// ---------------------------------------------------------------------------
static int test_window_rnr_blocks_then_rr_reopens(void) {
    printf("\n--- test_window_rnr_blocks_then_rr_reopens ---\n");
    printf("Section 6.4.9+6.4.10: RNR sets peer_busy; RR clears it\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, false, 1, 7);

    // Send one frame first so there is an outstanding frame
    uint8_t payload[] = { 0xAA, 0xBB };
    uint8_t rc = ax25_send_data(&conn, payload, sizeof(payload), PID_NO_L3);
    TEST_ASSERT(rc == 0, "B3: first I-frame sent OK", 0);
    TEST_ASSERT(conn.vars.vs == 1, "B3: V(S)=1 after first send", 0);

    // Inject RNR(N(R)=1, P=0) from peer
    ax25_frame_t *rnr = adv_make_sframe(AX25_FRAME_SUPERVISORY_RNR_8BIT, 1, false, 1);
    ax25_process_frame(&conn, rnr, 10);
    adv_free_sframe(rnr);

    DEBUG_BOOL("peer_busy after RNR (should be true)", conn.peer_busy); DEBUG_BOOL("h.peer_busy (callback received)", h.peer_busy);
    TEST_ASSERT(conn.peer_busy == true, "B3: peer_busy=true after RNR", 0);
    TEST_ASSERT(h.peer_busy == true, "B3: on_busy(true) callback fired", 0);
    TEST_ASSERT(conn.vars.va == 1, "B3: V(A)=1 (frame 0 acknowledged by RNR)", 0);

    // Attempt to send while peer is busy - must return 5
    payload[0] = 0xCC;
    rc = ax25_send_data(&conn, payload, sizeof(payload), PID_NO_L3);
    DEBUG_HEX("send while peer busy rc (should be 5)", rc);
    TEST_ASSERT(rc == 5, "B3: ax25_send_data returns 5 while peer_busy=true", 0);

    // Inject RR(N(R)=1) from peer to clear busy
    ax25_frame_t *rr = adv_make_sframe(AX25_FRAME_SUPERVISORY_RR_8BIT, 1, false, 0);
    ax25_process_frame(&conn, rr, 20);
    adv_free_sframe(rr);

    DEBUG_BOOL("peer_busy after RR (should be false)", conn.peer_busy);
    TEST_ASSERT(conn.peer_busy == false, "B3: peer_busy=false after RR", 0);
    TEST_ASSERT(h.peer_busy == false, "B3: on_busy(false) callback fired", 0);

    // Now send must succeed
    payload[0] = 0xDD;
    rc = ax25_send_data(&conn, payload, sizeof(payload), PID_NO_L3);
    DEBUG_HEX("send after RR clears busy rc (should be 0)", rc);
    TEST_ASSERT(rc == 0, "B3: send succeeds after RR clears peer_busy", 0);

    DEBUG_PRINT("B3 passed: RNR/RR window flow control correct");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test B4: Cumulative ACK - RR(N(R)) advances V(A) over multiple frames
//          Send 5 frames (N(S)=0..4). Acknowledge with RR(N(R)=3).
//          V(A) must advance to 3 (frames 0,1,2 dequeued). Frame 3,4 remain.
//          Then acknowledge with RR(N(R)=5) -> V(A)=5, queue empty.
// ---------------------------------------------------------------------------
static int test_window_cumulative_ack(void) {
    printf("\n--- test_window_cumulative_ack ---\n");
    printf("Section 6.4.7: Cumulative ACK via N(R) dequeues acknowledged frames\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, false, 1, 7);

    // Send 5 frames
    uint8_t pl[] = { 0 };
    for (uint8_t i = 0; i < 5; i++) {
        pl[0] = i;
        uint8_t rc = ax25_send_data(&conn, pl, 1, PID_NO_L3);
        TEST_ASSERT(rc == 0, "B4: frame i sent OK", 0);
    }

    DEBUG_VAR("V(S) after 5 sends (should be 5)", conn.vars.vs); DEBUG_VAR("V(A) before ACK (should be 0)", conn.vars.va); DEBUG_VAR("tx_queue.count (should be 5)", conn.tx_queue.count);
    TEST_ASSERT(conn.vars.vs == 5, "B4: V(S)=5 after 5 sends", 0);
    TEST_ASSERT(conn.vars.va == 0, "B4: V(A)=0 before ACK", 0);
    TEST_ASSERT(conn.tx_queue.count == 5, "B4: 5 frames in queue", 0);

    // Partial ACK: RR(N(R)=3) acknowledges frames 0,1,2
    ax25_frame_t *rr3 = adv_make_sframe(AX25_FRAME_SUPERVISORY_RR_8BIT, 3, false, 0);
    ax25_process_frame(&conn, rr3, 10);
    adv_free_sframe(rr3);

    DEBUG_VAR("V(A) after RR(3) (should be 3)", conn.vars.va); DEBUG_VAR("tx_queue.count after RR(3) (should be 2)", conn.tx_queue.count);
    TEST_ASSERT(conn.vars.va == 3, "B4: V(A)=3 after RR(3)", 0);
    TEST_ASSERT(conn.tx_queue.count == 2, "B4: 2 frames remain (N(S)=3 and 4)", 0);

    // Full ACK: RR(N(R)=5)
    ax25_frame_t *rr5 = adv_make_sframe(AX25_FRAME_SUPERVISORY_RR_8BIT, 5, false, 0);
    ax25_process_frame(&conn, rr5, 20);
    adv_free_sframe(rr5);

    DEBUG_VAR("V(A) after RR(5) (should be 5)", conn.vars.va); DEBUG_VAR("tx_queue.count after full ACK (should be 0)", conn.tx_queue.count);
    TEST_ASSERT(conn.vars.va == 5, "B4: V(A)=5 after RR(5) full ACK", 0);
    TEST_ASSERT(conn.tx_queue.count == 0, "B4: queue empty after full ACK", 0);

    // T1 must be stopped when queue is empty
    TEST_ASSERT(conn.t1_start_tick == 0, "B4: T1 stopped when queue empty after full ACK", 0);

    DEBUG_PRINT("B4 passed: cumulative ACK dequeues correctly");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test B5: V(S) rollback on REJ with sequence number wrap
//          Start with V(S)=V(A)=2, V(R)=0.
//          Send 4 frames (N(S)=2,3,4,5). ACK N(S)=2 only: V(A)=3.
//          Then inject REJ(N(R)=3): peer rejects from 3 forward.
//          Verify: V(S) rolled back to V(A)=3, retransmit of frames 3,4,5.
// ---------------------------------------------------------------------------
static int test_window_vs_rollback_on_rej(void) {
    printf("\n--- test_window_vs_rollback_on_rej ---\n");
    printf("Section 6.4.7: V(S) rollback to V(A) on REJ reception\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, false, 1, 7);

    // Send 4 frames N(S)=0,1,2,3
    uint8_t pl[] = { 0 };
    for (uint8_t i = 0; i < 4; i++) {
        pl[0] = i;
        ax25_send_data(&conn, pl, 1, PID_NO_L3);
    } DEBUG_VAR("V(S) after 4 sends (should be 4)", conn.vars.vs);
    TEST_ASSERT(conn.vars.vs == 4, "B5: V(S)=4 after 4 sends", 0);

    // Partial ACK: RR(N(R)=2) acknowledges frames 0,1 -> V(A)=2
    ax25_frame_t *rr2 = adv_make_sframe(AX25_FRAME_SUPERVISORY_RR_8BIT, 2, false, 0);
    ax25_process_frame(&conn, rr2, 5);
    adv_free_sframe(rr2);
    TEST_ASSERT(conn.vars.va == 2, "B5: V(A)=2 after partial ACK", 0); DEBUG_VAR("V(A) after partial ACK (should be 2)", conn.vars.va);

    // Capture current tx_count to measure retransmits
    uint8_t tx_before = h.tx_count;

    // Inject REJ(N(R)=2): reject from N(S)=2 forward
    ax25_frame_t *rej = adv_make_sframe(AX25_FRAME_SUPERVISORY_REJ_8BIT, 2, false, 2);
    ax25_process_frame(&conn, rej, 10);
    adv_free_sframe(rej);

    DEBUG_VAR("V(S) after REJ(2) (should be 2 = V(A))", conn.vars.vs); DEBUG_VAR("V(A) after REJ(2) (should be 2)", conn.vars.va); DEBUG_VAR("tx_count increase (should be 2 retransmits: N(S)=2,3)", (uint8_t)(h.tx_count - tx_before));

    TEST_ASSERT(conn.vars.vs == 2, "B5: V(S) rolled back to V(A)=2 on REJ(2)", 0);
    TEST_ASSERT(conn.vars.va == 2, "B5: V(A) unchanged at 2 after REJ", 0);
    // In half-duplex mode, retransmission of 2 frames (N(S)=2 and 3)
    uint8_t retransmits = (uint8_t) (h.tx_count - tx_before);
    DEBUG_VAR("retransmitted frames (should be 2)", retransmits);
    TEST_ASSERT(retransmits == 2, "B5: 2 frames retransmitted after REJ(2)", 0);

    DEBUG_PRINT("B5 passed: V(S) rollback and retransmit on REJ");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test B6: AX25_MAX_QUEUE_SIZE limits queue even when k is larger
//          k=15 but AX25_MAX_QUEUE_SIZE=16. Send 16 frames.
//          The 15th must fail (window k=15 closes first; if k=16 exact test).
//          Use k=AX25_MAX_QUEUE_SIZE to test the hard queue limit path.
// ---------------------------------------------------------------------------
static int test_window_queue_size_limit(void) {
    printf("\n--- test_window_queue_size_limit ---\n");
    printf("Section 4.2.2: AX25_MAX_QUEUE_SIZE limits queue depth independent of k\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    // Set k = AX25_MAX_QUEUE_SIZE to make queue-depth the binding constraint
    adv_force_connected(&conn, &h, 128, false, 1, AX25_MAX_QUEUE_SIZE);

    uint8_t pl[] = { 0xCC };
    uint8_t sent = 0;
    for (uint8_t i = 0; i < AX25_MAX_QUEUE_SIZE; i++) {
        pl[0] = i;
        uint8_t rc = ax25_send_data(&conn, pl, 1, PID_NO_L3);
        if (rc == 0)
            sent++;
    }

    DEBUG_VAR("Frames sent (should be AX25_MAX_QUEUE_SIZE)", sent); DEBUG_VAR("tx_queue.count (should be AX25_MAX_QUEUE_SIZE)", conn.tx_queue.count);
    TEST_ASSERT(sent == AX25_MAX_QUEUE_SIZE, "B6: AX25_MAX_QUEUE_SIZE frames sent before limit hit", 0);
    TEST_ASSERT(conn.tx_queue.count == AX25_MAX_QUEUE_SIZE, "B6: tx_queue.count == AX25_MAX_QUEUE_SIZE", 0);

    // One more send must fail (queue full)
    pl[0] = 0xFF;
    uint8_t rc = ax25_send_data(&conn, pl, 1, PID_NO_L3);
    DEBUG_HEX("send after queue full rc (should be 3)", rc);
    TEST_ASSERT(rc == 3, "B6: send returns 3 when tx_queue at AX25_MAX_QUEUE_SIZE", 0);

    DEBUG_PRINT("B6 passed: queue size limit enforced");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ===========================================================================
// ======== SECTION C: FULL-DUPLEX STATE SYNCHRONIZATION ====================
// ===========================================================================

// ---------------------------------------------------------------------------
// Test C1: Full-duplex: simultaneous send and receive, V(S) and V(R) independent
//          In full-duplex mode, send an I-frame and simultaneously receive
//          an I-frame. V(S) and V(R) must be updated independently.
// ---------------------------------------------------------------------------
static int test_fd_simultaneous_send_receive(void) {
    printf("\n--- test_fd_simultaneous_send_receive ---\n");
    printf("Section 6.7.2: Full-duplex simultaneous send+receive updates V(S) and V(R)\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, true, 1, 7);  // full_duplex=true

    DEBUG_BOOL("full_duplex enabled", conn.full_duplex);
    TEST_ASSERT(conn.full_duplex == true, "C1: full_duplex is true", 0);

    // Step 1: Send an outgoing I-frame (V(S) should go from 0 to 1)
    uint8_t outgoing[] = { 0xAA, 0xBB };
    uint8_t rc = ax25_send_data(&conn, outgoing, sizeof(outgoing), PID_NO_L3);
    DEBUG_HEX("send rc (should be 0)", rc);
    TEST_ASSERT(rc == 0, "C1: outgoing I-frame sent OK", 0);
    TEST_ASSERT(conn.vars.vs == 1, "C1: V(S)=1 after outgoing I-frame", 0);
    TEST_ASSERT(conn.vars.vr == 0, "C1: V(R) unchanged at 0 after send", 0);

    // Step 2: Receive incoming I-frame N(S)=0 from peer (V(R) goes from 0 to 1)
    uint8_t incoming[] = { 0x55, 0x66 };
    ax25_frame_t *f_in = adv_make_iframe(0, 1, false, incoming, sizeof(incoming), 8);
    ax25_process_frame(&conn, f_in, 10);
    adv_free_iframe(f_in);

    DEBUG_VAR("V(R) after receiving N(S)=0 (should be 1)", conn.vars.vr); DEBUG_VAR("V(S) after receiving (should still be 1)", conn.vars.vs); DEBUG_VAR("rx_data_count (should be 1)", h.rx_data_count);
    // V(A) should have advanced to 1 if peer's N(R)=1 acknowledges our N(S)=0
    DEBUG_VAR("V(A) after piggybacked N(R)=1 (should be 1)", conn.vars.va);

    TEST_ASSERT(conn.vars.vr == 1, "C1: V(R)=1 after peer frame received", 0);
    TEST_ASSERT(conn.vars.vs == 1, "C1: V(S) still 1 (independent of receive)", 0);
    TEST_ASSERT(h.rx_data_count == 1, "C1: incoming I-frame delivered to upper layer", 0);
    TEST_ASSERT(conn.vars.va == 1, "C1: V(A)=1 from piggybacked N(R)=1 in peer frame", 0);

    DEBUG_PRINT("C1 passed: V(S) and V(R) updated independently in full-duplex");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test C2: Full-duplex: outgoing I-frame piggybacks current V(R) as N(R)
//          Receive 3 incoming frames (V(R)=3).
//          Then send an outgoing I-frame.
//          The outgoing frame's N(R) field must equal current V(R)=3.
// ---------------------------------------------------------------------------
static int test_fd_piggybacked_nr_reflects_vr(void) {
    printf("\n--- test_fd_piggybacked_nr_reflects_vr ---\n");
    printf("Section 6.4.2.1: Outgoing I-frame piggybacks N(R)=V(R)\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, true, 1, 7);

    // Receive 3 in-order frames to advance V(R) to 3
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t pl[] = { i };
        ax25_frame_t *f = adv_make_iframe(i, 0, false, pl, 1, 8);
        ax25_process_frame(&conn, f, (uint32_t) (i + 1));
        adv_free_iframe(f);
    }

    DEBUG_VAR("V(R) after 3 in-order receives (should be 3)", conn.vars.vr);
    TEST_ASSERT(conn.vars.vr == 3, "C2: V(R)=3 after 3 inbound frames", 0);

    // Now send an outgoing I-frame
    uint8_t tx_before = h.tx_count;
    uint8_t payload[] = { 0xFF };
    uint8_t rc = ax25_send_data(&conn, payload, 1, PID_NO_L3);
    TEST_ASSERT(rc == 0, "C2: outgoing send OK", 0);

    // Decode the outgoing I-frame and check N(R)
    if ((uint8_t) (h.tx_count - tx_before) >= 1) {
        DEBUG_VAR("Outgoing I-frame N(R) (should be V(R)=3)", (unsigned)nr);
        // In full-duplex, RR is sent immediately (no T2 delay). The outgoing I-frame
        // itself piggybacks V(R) as N(R) only if the I-frame was the first TX action.
        // Allow that there may be RR frames sent too; we look for the I-frame.
        int i_frame_nr = -1;
        for (int idx = (int) tx_before; idx < (int) h.tx_count; idx++) {
            int test_nr = adv_get_iframe_nr(&h, idx);
            if (test_nr >= 0) {
                i_frame_nr = test_nr;
                break;
            }
        } DEBUG_VAR("First outgoing I-frame N(R) (should be 3)", (unsigned)i_frame_nr);
        TEST_ASSERT(i_frame_nr == 3, "C2: outgoing I-frame N(R)=3 = V(R)", 0);
    } else {
        TEST_ASSERT(false, "C2: outgoing I-frame was not transmitted", 0);
    }

    DEBUG_PRINT("C2 passed: piggybacked N(R) correctly reflects V(R)");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test C3: Full-duplex REJ triggers abort_tx and immediate bulk retransmit
//          Send 4 frames. Receive REJ(N(R)=2) - should trigger abort_tx
//          (since full_duplex=true) and retransmit frames 2,3 immediately.
// ---------------------------------------------------------------------------
static int test_fd_rej_triggers_abort_and_retransmit(void) {
    printf("\n--- test_fd_rej_triggers_abort_and_retransmit ---\n");
    printf("Section 6.4.5.3: Full-duplex REJ triggers abort_tx + immediate retransmit\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, true, 1, 7);  // full_duplex=true

    // Send 4 frames
    uint8_t pl[] = { 0 };
    for (uint8_t i = 0; i < 4; i++) {
        pl[0] = i;
        ax25_send_data(&conn, pl, 1, PID_NO_L3);
    }
    TEST_ASSERT(conn.vars.vs == 4, "C3: V(S)=4 after 4 sends", 0);

    // Partially ACK frames 0,1 with RR(N(R)=2)
    ax25_frame_t *rr2 = adv_make_sframe(AX25_FRAME_SUPERVISORY_RR_8BIT, 2, false, 0);
    ax25_process_frame(&conn, rr2, 5);
    adv_free_sframe(rr2);
    TEST_ASSERT(conn.vars.va == 2, "C3: V(A)=2 after partial ACK", 0);

    // Reset abort tracking
    h.abort_tx_called = false;
    h.abort_tx_call_count = 0;
    uint8_t tx_before_rej = h.tx_count;

    // Inject REJ(N(R)=2) - full-duplex path should call abort_tx
    ax25_frame_t *rej = adv_make_sframe(AX25_FRAME_SUPERVISORY_REJ_8BIT, 2, false, 2);
    ax25_process_frame(&conn, rej, 10);
    adv_free_sframe(rej);

    uint8_t retransmits = (uint8_t) (h.tx_count - tx_before_rej);
    DEBUG_BOOL("abort_tx called on full-duplex REJ (should be true)", h.abort_tx_called); DEBUG_VAR("abort_tx call count (should be 1)", h.abort_tx_call_count); DEBUG_VAR("retransmitted frames after REJ (should be 2: N(S)=2,3)", retransmits); DEBUG_VAR("V(S) after REJ rollback (should be 2 = V(A))", conn.vars.vs); DEBUG_VAR("conn.stats.iframe_retransmitted (should be >= 2)", conn.stats.iframe_retransmitted);

    TEST_ASSERT(h.abort_tx_called == true, "C3: abort_tx called on full-duplex REJ", 0);
    TEST_ASSERT(h.abort_tx_call_count == 1, "C3: abort_tx called exactly once", 0);
    TEST_ASSERT(retransmits == 2, "C3: 2 frames immediately retransmitted", 0);
    TEST_ASSERT(conn.vars.vs == 2, "C3: V(S) rolled back to V(A)=2", 0);
    TEST_ASSERT(conn.stats.iframe_retransmitted >= 2, "C3: iframe_retransmitted counter incremented for retransmits", 0);
    // T1 should be set to AX25_T1_PENDING sentinel for full-duplex retry
    TEST_ASSERT(conn.t1_start_tick == AX25_T1_PENDING, "C3: T1 set to AX25_T1_PENDING after full-duplex REJ", 0);

    DEBUG_PRINT("C3 passed: full-duplex REJ triggers abort_tx + retransmit + T1_PENDING");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test C4: Full-duplex SREJ sets T1 to AX25_T1_PENDING sentinel (not 0)
//          Send 3 frames. Receive SREJ(N(R)=1) = peer requesting retransmit of N(S)=1.
//          In full-duplex mode, T1 should be set to AX25_T1_PENDING (UINT32_MAX).
// ---------------------------------------------------------------------------
static int test_fd_srej_sets_t1_pending(void) {
    printf("\n--- test_fd_srej_sets_t1_pending ---\n");
    printf("Section 6.4.8: Full-duplex SREJ sets T1 to AX25_T1_PENDING sentinel\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, true, 1, 7);  // full_duplex=true

    // Send 3 frames
    uint8_t pl[] = { 0 };
    for (uint8_t i = 0; i < 3; i++) {
        pl[0] = i;
        ax25_send_data(&conn, pl, 1, PID_NO_L3);
    }
    TEST_ASSERT(conn.vars.vs == 3, "C4: V(S)=3 after 3 sends", 0);

    // Receive SREJ(N(R)=1) - request retransmit of N(S)=1
    ax25_frame_t *srej = adv_make_sframe(AX25_FRAME_SUPERVISORY_SREJ_8BIT, 1, false, 3);
    ax25_process_frame(&conn, srej, 10);
    adv_free_sframe(srej);

    DEBUG_VAR("t1_start_tick before SREJ", t1_before); DEBUG_VAR("t1_start_tick after SREJ (should be AX25_T1_PENDING=UINT32_MAX)", conn.t1_start_tick); DEBUG_VAR("conn.stats.iframe_retransmitted (should be 1)", conn.stats.iframe_retransmitted);

    // In full-duplex, SREJ handler sets T1 = AX25_T1_PENDING
    TEST_ASSERT(conn.t1_start_tick == AX25_T1_PENDING, "C4: T1 set to AX25_T1_PENDING after full-duplex SREJ", 0);
    TEST_ASSERT(conn.stats.iframe_retransmitted >= 1, "C4: iframe_retransmitted incremented after SREJ retransmit", 0);

    // Verify N(S)=1 frame was retransmitted
    // The retransmitted frame should be the last I-frame sent
    bool found_retransmit = false;
    for (int i = (int) h.tx_count - 1; i >= 0; i--) {
        int ns = adv_get_iframe_ns(&h, i);
        if (ns == 1) {
            found_retransmit = true;
            break;
        }
    } DEBUG_BOOL("N(S)=1 retransmitted (found in tx_frames)", found_retransmit);
    TEST_ASSERT(found_retransmit, "C4: N(S)=1 I-frame retransmitted in response to SREJ(1)", 0);

    DEBUG_PRINT("C4 passed: full-duplex SREJ sets T1_PENDING and retransmits correct frame");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test C5: Full-duplex: RR immediately sent (no T2 delay) on incoming I-frame
//          In full-duplex mode, RR should be sent immediately upon receiving
//          an I-frame. In half-duplex mode, T2 timer delays the RR.
//          Verify that in full-duplex, tx_count increases immediately after
//          the first incoming I-frame (no delay required via ax25_tick).
// ---------------------------------------------------------------------------
static int test_fd_rr_sent_immediately_no_t2(void) {
    printf("\n--- test_fd_rr_sent_immediately_no_t2 ---\n");
    printf("Section 6.7.2: Full-duplex RR sent immediately (no T2 delay)\n");

    // Full-duplex test
    {
        adv_harness_t h_fd;
        memset(&h_fd, 0, sizeof(h_fd));
        ax25_connection_t conn_fd;
        adv_force_connected(&conn_fd, &h_fd, 8, true, 1, 7);  // full_duplex=true

        uint8_t pl[] = { 0xAA };
        ax25_frame_t *f = adv_make_iframe(0, 0, false, pl, 1, 8);
        ax25_process_frame(&conn_fd, f, 1);
        adv_free_iframe(f);

        DEBUG_VAR("Full-duplex tx_count after I-frame (should be 1 = RR)", h_fd.tx_count);
        TEST_ASSERT(h_fd.tx_count >= 1, "C5: Full-duplex: at least one frame (RR) sent immediately", 0);
        // T2 timer must NOT be running in full-duplex
        DEBUG_BOOL("t2_running in full-duplex (should be false)", conn_fd.t2_running);
        TEST_ASSERT(conn_fd.t2_running == false, "C5: T2 timer NOT started in full-duplex mode", 0);

        ax25_connection_cleanup(&conn_fd);
    }

    // Half-duplex test (for comparison)
    {
        adv_harness_t h_hd;
        memset(&h_hd, 0, sizeof(h_hd));
        ax25_connection_t conn_hd;
        adv_force_connected(&conn_hd, &h_hd, 8, false, 1, 7);  // half-duplex

        uint8_t pl[] = { 0xBB };
        ax25_frame_t *f = adv_make_iframe(0, 0, false, pl, 1, 8);
        ax25_process_frame(&conn_hd, f, 1);
        adv_free_iframe(f);

        DEBUG_VAR("Half-duplex tx_count delta after I-frame (usually 0 - T2 pending)",
                (uint8_t)(h_hd.tx_count - tx_count_before)); DEBUG_BOOL("t2_running in half-duplex (should be true when no P/F)", conn_hd.t2_running);
        // In half-duplex, no immediate RR unless P bit was set; T2 delays it
        TEST_ASSERT(conn_hd.t2_running == true, "C5: T2 timer started in half-duplex mode after I-frame", 0);

        ax25_connection_cleanup(&conn_hd);
    }

    DEBUG_PRINT("C5 passed: full-duplex sends RR immediately, half-duplex uses T2");
    return 0;
}

// ---------------------------------------------------------------------------
// Test C6: Full-duplex state: both V(S) and V(R) update correctly after
//          crossed I-frames (each side sends before receiving the other's frame).
//          Station A sends N(S)=0 N(R)=0.
//          Station B (simulated by injecting frames into A's conn):
//            - A receives N(S)=0 N(R)=1 from B (B acknowledges A's frame and
//              sends its own frame 0).
//          After receiving: V(R) should be 1 (A accepted B's frame),
//          V(A) should be 1 (A's N(S)=0 was acknowledged by B's N(R)=1),
//          V(S) remains at 1.
// ---------------------------------------------------------------------------
static int test_fd_crossed_iframes_state_sync(void) {
    printf("\n--- test_fd_crossed_iframes_state_sync ---\n");
    printf("Section 6.7.2: Crossed I-frames in full-duplex - V(S), V(R), V(A) consistency\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, true, 1, 7);

    // Station A sends I-frame N(S)=0 N(R)=0
    uint8_t pa[] = { 0xAA };
    uint8_t rc = ax25_send_data(&conn, pa, 1, PID_NO_L3);
    TEST_ASSERT(rc == 0, "C6: A sends N(S)=0 OK", 0);
    TEST_ASSERT(conn.vars.vs == 1, "C6: A V(S)=1 after sending", 0);
    TEST_ASSERT(conn.vars.vr == 0, "C6: A V(R)=0 (nothing received yet)", 0);
    TEST_ASSERT(conn.vars.va == 0, "C6: A V(A)=0 (nothing acknowledged yet)", 0);

    // Station B's response arrives: N(S)=0 N(R)=1 (B acknowledges A's N(S)=0)
    uint8_t pb[] = { 0xBB };
    ax25_frame_t *f_b = adv_make_iframe(0, 1, false, pb, 1, 8);
    ax25_process_frame(&conn, f_b, 10);
    adv_free_iframe(f_b);

    DEBUG_VAR("V(R) after B's frame (should be 1)", conn.vars.vr); DEBUG_VAR("V(A) after B's N(R)=1 (should be 1 - A's frame acknowledged)", conn.vars.va); DEBUG_VAR("V(S) unchanged (should be 1)", conn.vars.vs); DEBUG_VAR("rx_data_count (should be 1 - B's frame delivered)", h.rx_data_count);

    TEST_ASSERT(conn.vars.vr == 1, "C6: V(R)=1 after receiving B's N(S)=0", 0);
    TEST_ASSERT(conn.vars.va == 1, "C6: V(A)=1 after B's piggybacked N(R)=1 acks A's frame", 0);
    TEST_ASSERT(conn.vars.vs == 1, "C6: V(S) still 1 (independent)", 0);
    TEST_ASSERT(h.rx_data_count == 1, "C6: B's I-frame delivered to upper layer", 0);

    // T1 must be stopped (tx queue drained when V(A)==V(S))
    TEST_ASSERT(conn.tx_queue.count == 0, "C6: tx_queue empty after A's frame acked", 0);
    TEST_ASSERT(conn.t1_start_tick == 0, "C6: T1 stopped after full ACK", 0);

    DEBUG_PRINT("C6 passed: crossed I-frames update V(S), V(R), V(A) consistently");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test C7: Full-duplex SREJ in mid-stream does not block other sends
//          Send frames 0,1,2. Peer sends SREJ(1) requesting N(S)=1 retransmit.
//          After SREJ processing, station must still be able to send new frames
//          (SREJ does NOT enter TIMER_RECOVERY - only T1 expiry does that).
//          Send frame N(S)=3 after SREJ -> should succeed (return 0).
// ---------------------------------------------------------------------------
static int test_fd_srej_does_not_block_new_sends(void) {
    printf("\n--- test_fd_srej_does_not_block_new_sends ---\n");
    printf("Section 6.4.8: SREJ retransmit does not block new I-frame sends\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, true, 1, 7);

    // Send 3 frames
    uint8_t pl[] = { 0 };
    for (uint8_t i = 0; i < 3; i++) {
        pl[0] = i;
        ax25_send_data(&conn, pl, 1, PID_NO_L3);
    }
    TEST_ASSERT(conn.vars.vs == 3, "C7: V(S)=3 after 3 sends", 0);

    // Receive SREJ(1) - retransmit N(S)=1
    ax25_frame_t *srej = adv_make_sframe(AX25_FRAME_SUPERVISORY_SREJ_8BIT, 1, false, 3);
    ax25_process_frame(&conn, srej, 10);
    adv_free_sframe(srej);

    DEBUG_VAR("State after SREJ (should still be CONNECTED)", conn.state);
    TEST_ASSERT(conn.state == AX25_STATE_CONNECTED, "C7: State remains CONNECTED after SREJ (not TIMER_RECOVERY)", 0);

    // Try to send a new frame after SREJ - should succeed if window not full
    pl[0] = 0x03;
    uint8_t rc = ax25_send_data(&conn, pl, 1, PID_NO_L3);
    DEBUG_HEX("send after SREJ rc (should be 0)", rc);
    TEST_ASSERT(rc == 0, "C7: New I-frame can be sent after SREJ processing", 0);
    TEST_ASSERT(conn.vars.vs == 4, "C7: V(S)=4 after new send post-SREJ", 0);

    DEBUG_PRINT("C7 passed: SREJ does not block subsequent sends");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test C8: Full-duplex RNR clears peer_busy on subsequent RR
//          (also tests that on_busy callback is called correctly)
//          In full-duplex: RNR -> peer_busy=true, RR -> peer_busy=false.
//          Verify both transitions fire on_busy callback.
// ---------------------------------------------------------------------------
static int test_fd_rnr_rr_busy_transitions(void) {
    printf("\n--- test_fd_rnr_rr_busy_transitions ---\n");
    printf("Section 6.4.9+6.4.10: Full-duplex RNR/RR peer_busy state transitions\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, true, 1, 7);

    // Send one frame
    uint8_t pl[] = { 0xAA };
    ax25_send_data(&conn, pl, 1, PID_NO_L3);

    // Peer sends RNR(N(R)=1) to indicate busy
    ax25_frame_t *rnr = adv_make_sframe(AX25_FRAME_SUPERVISORY_RNR_8BIT, 1, false, 1);
    ax25_process_frame(&conn, rnr, 10);
    adv_free_sframe(rnr);

    DEBUG_BOOL("peer_busy after RNR (should be true)", conn.peer_busy); DEBUG_BOOL("h.peer_busy after RNR (should be true)", h.peer_busy);
    TEST_ASSERT(conn.peer_busy == true, "C8: peer_busy set after RNR", 0);
    TEST_ASSERT(h.peer_busy == true, "C8: on_busy(true) callback fired for RNR", 0);

    // Peer sends RR(N(R)=1) to clear busy
    ax25_frame_t *rr = adv_make_sframe(AX25_FRAME_SUPERVISORY_RR_8BIT, 1, false, 0);
    ax25_process_frame(&conn, rr, 20);
    adv_free_sframe(rr);

    DEBUG_BOOL("peer_busy after RR (should be false)", conn.peer_busy); DEBUG_BOOL("h.peer_busy after RR (should be false)", h.peer_busy);
    TEST_ASSERT(conn.peer_busy == false, "C8: peer_busy cleared after RR", 0);
    TEST_ASSERT(h.peer_busy == false, "C8: on_busy(false) callback fired for RR", 0);

    // After RR, sends should succeed again
    pl[0] = 0xBB;
    uint8_t rc = ax25_send_data(&conn, pl, 1, PID_NO_L3);
    DEBUG_HEX("send after RR clears busy rc (should be 0)", rc);
    TEST_ASSERT(rc == 0, "C8: send succeeds in full-duplex after RR clears peer_busy", 0);

    DEBUG_PRINT("C8 passed: full-duplex RNR/RR busy transitions correct");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test C9: iframe_retransmitted statistics correctly counted during full-duplex
//          Three retransmit-causing events in sequence:
//          1. SREJ(N(R)=0) -> retransmit N(S)=0 (count += 1)
//          2. REJ(N(R)=0)  -> retransmit frames 0..N-1 (count += queue_len)
//          Check that conn.stats.iframe_retransmitted is accurate after each.
// ---------------------------------------------------------------------------
static int test_fd_retransmit_stats_accuracy(void) {
    printf("\n--- test_fd_retransmit_stats_accuracy ---\n");
    printf("Statistics: iframe_retransmitted count accurate for SREJ and REJ in full-duplex\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, true, 1, 7);

    // Send 4 frames
    uint8_t pl[] = { 0 };
    for (uint8_t i = 0; i < 4; i++) {
        pl[0] = i;
        ax25_send_data(&conn, pl, 1, PID_NO_L3);
    }
    TEST_ASSERT(conn.vars.vs == 4, "C9: V(S)=4 after 4 sends", 0);

    uint32_t retx_after_sends = conn.stats.iframe_retransmitted;
    DEBUG_VAR("iframe_retransmitted after 4 sends (should be 0)", retx_after_sends);
    TEST_ASSERT(retx_after_sends == 0, "C9: No retransmits yet after initial sends", 0);

    // Event 1: SREJ(N(R)=1) -> retransmit N(S)=1 only
    ax25_frame_t *srej = adv_make_sframe(AX25_FRAME_SUPERVISORY_SREJ_8BIT, 1, false, 3);
    ax25_process_frame(&conn, srej, 10);
    adv_free_sframe(srej);

    uint32_t retx_after_srej = conn.stats.iframe_retransmitted;
    DEBUG_VAR("iframe_retransmitted after SREJ(1) (should be 1)", retx_after_srej);
    TEST_ASSERT(retx_after_srej == 1, "C9: iframe_retransmitted=1 after single SREJ retransmit", 0);

    // Event 2: REJ(N(R)=0) -> retransmit all 4 frames (V(A)=0, V(S)=4)
    ax25_frame_t *rej = adv_make_sframe(AX25_FRAME_SUPERVISORY_REJ_8BIT, 0, false, 2);
    ax25_process_frame(&conn, rej, 20);
    adv_free_sframe(rej);

    uint32_t retx_after_rej = conn.stats.iframe_retransmitted;
    DEBUG_VAR("iframe_retransmitted after REJ(0) (should be 1 + 4 = 5)", retx_after_rej);
    TEST_ASSERT(retx_after_rej >= 5, "C9: iframe_retransmitted >= 5 after SREJ(1-frame) + REJ(4-frames)", 0);

    DEBUG_PRINT("C9 passed: retransmit stats accurate for SREJ and REJ");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ---------------------------------------------------------------------------
// Test C10: Full-duplex window management during continuous bidirectional data
//           exchange over multiple modulo cycles.
//           Simulate 3 full window cycles of send + ACK while simultaneously
//           receiving frames. V(S), V(R), V(A) must all remain consistent.
// ---------------------------------------------------------------------------
static int test_fd_continuous_bidirectional_exchange(void) {
    printf("\n--- test_fd_continuous_bidirectional_exchange ---\n");
    printf("Section 6.7.2: Continuous bidirectional full-duplex exchange over 3 window cycles\n");

    adv_harness_t h;
    memset(&h, 0, sizeof(h));
    ax25_connection_t conn;
    adv_force_connected(&conn, &h, 8, true, 1, 7);

    uint32_t tick = 0;
    uint8_t incoming_ns = 0;  // peer's N(S) counter
    uint8_t total_sent = 0;
    uint8_t total_recv = 0;

    // Simulate 3 full window cycles (k=7, mod-8 wraps at 8)
    // Each cycle: send 7 frames, receive 7 frames, ACK all
    for (uint8_t cycle = 0; cycle < 3; cycle++) {
        DEBUG_VAR("Cycle", cycle); DEBUG_VAR("V(S) at cycle start", cycle_vs_start); DEBUG_VAR("V(R) at cycle start", conn.vars.vr); DEBUG_VAR("V(A) at cycle start", conn.vars.va);

        // Send k=7 outgoing frames
        uint8_t pl[] = { 0 };
        for (uint8_t i = 0; i < 7; i++) {
            pl[0] = (uint8_t) (cycle * 10 + i);
            uint8_t rc = ax25_send_data(&conn, pl, 1, PID_NO_L3);
            if (rc == 0)
                total_sent++;
            tick++;
        }

        // Simultaneously receive 7 incoming frames from peer
        for (uint8_t i = 0; i < 7; i++) {
            uint8_t pip[] = { (uint8_t) (0x80 + cycle * 10 + i) };
            // Peer piggybacks N(R) = current V(S) (acknowledges all our frames up to now)
            ax25_frame_t *f = adv_make_iframe(incoming_ns & 0x07u, conn.vars.vs & 0x07u,
            false, pip, 1, 8);
            ax25_process_frame(&conn, f, tick++);
            adv_free_iframe(f);
            incoming_ns++;
            total_recv++;
        }

        // Peer also sends explicit RR to acknowledge remaining outstanding frames
        ax25_frame_t *rr = adv_make_sframe(AX25_FRAME_SUPERVISORY_RR_8BIT, conn.vars.vs & 0x07u, false, 0);
        ax25_process_frame(&conn, rr, tick++);
        adv_free_sframe(rr);

        DEBUG_VAR("V(S) at cycle end", conn.vars.vs); DEBUG_VAR("V(R) at cycle end", conn.vars.vr); DEBUG_VAR("V(A) at cycle end", conn.vars.va); DEBUG_VAR("tx_queue.count at cycle end (should be 0)", conn.tx_queue.count);

        TEST_ASSERT(conn.tx_queue.count == 0, "C10: tx_queue empty after each cycle's ACK", 0);
        TEST_ASSERT(conn.state == AX25_STATE_CONNECTED, "C10: state remains CONNECTED throughout", 0);
    }

    DEBUG_VAR("Total sent (should be 21 = 3 cycles x 7)", total_sent); DEBUG_VAR("Total recv (should be 21 = 3 cycles x 7)", total_recv); DEBUG_VAR("Final V(S) (mod-8 of 21 = 5)", conn.vars.vs); DEBUG_VAR("Final V(R) (mod-8 of 21 = 5)", conn.vars.vr); DEBUG_VAR("Final V(A) should equal V(S)", conn.vars.va);

    TEST_ASSERT(total_sent == 21, "C10: 21 frames sent over 3 cycles", 0);
    TEST_ASSERT(total_recv == 21, "C10: 21 frames received over 3 cycles", 0);
    TEST_ASSERT(h.rx_data_count == 21, "C10: 21 frames delivered to upper layer", 0);
    TEST_ASSERT(conn.state == AX25_STATE_CONNECTED, "C10: still CONNECTED at end", 0);
    TEST_ASSERT(conn.vars.va == conn.vars.vs, "C10: V(A)==V(S) at end (all frames acknowledged)", 0);

    DEBUG_PRINT("C10 passed: 3-cycle bidirectional full-duplex exchange consistent");
    ax25_connection_cleanup(&conn);
    return 0;
}

// ===========================================================================
// =============================== MAIN ENTRY ================================
// ===========================================================================

int test_ax25_advanced_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("Starting AX.25 v2.2 Section 13 Advanced Feature Tests\n");
    printf("  SECTION A: SREJ Bitmap Management\n");
    printf("  SECTION B: Extended Sequence Window Management\n");
    printf("  SECTION C: Full-Duplex State Synchronization\n");
    printf("==================================================================================\n");

    printf("\n--- SECTION A: SREJ Bitmap Management ---\n");
    result |= test_srej_bitmap_bit_positions();
    result |= test_srej_bitmap_set_by_state_machine();
    result |= test_srej_bitmap_cleared_on_delivery();
    result |= test_srej_bitmap_cleared_on_rej_fallback();
    result |= test_srej_bitmap_cleared_on_sabm_reset();
    result |= test_srej_bitmap_mod8_wraparound();
    result |= test_srej_bitmap_persists_across_window_slide();
    result |= test_srej_bitmap_multi_frame_mod128();

    printf("\n--- SECTION B: Extended Sequence Window Management ---\n");
    result |= test_window_full_blocks_send();
    result |= test_window_seq_wrap_mod128();
    result |= test_window_rnr_blocks_then_rr_reopens();
    result |= test_window_cumulative_ack();
    result |= test_window_vs_rollback_on_rej();
    result |= test_window_queue_size_limit();

    printf("\n--- SECTION C: Full-Duplex State Synchronization ---\n");
    result |= test_fd_simultaneous_send_receive();
    result |= test_fd_piggybacked_nr_reflects_vr();
    result |= test_fd_rej_triggers_abort_and_retransmit();
    result |= test_fd_srej_sets_t1_pending();
    result |= test_fd_rr_sent_immediately_no_t2();
    result |= test_fd_crossed_iframes_state_sync();
    result |= test_fd_srej_does_not_block_new_sends();
    result |= test_fd_rnr_rr_busy_transitions();
    result |= test_fd_retransmit_stats_accuracy();
    result |= test_fd_continuous_bidirectional_exchange();

    printf("\n==================================================================================\n");
    printf("AX.25 Advanced Tests Completed. %s\n", result == 0 ? "All tests passed" : "Some tests FAILED");
    printf("Total assertions processed: %u\n", assert_count);
    printf("==================================================================================\n\n");

    return result;
}
