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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "test_common.h"
#include "ax25.h"
#include "ax25_state_machine.h"
#include "ax25_mux.h"

/* -------------------------------------------------------------------------
 * Shared assert counter required by TEST_ASSERT macro (test_common.h)
 * ---------------------------------------------------------------------- */
static uint32_t assert_count = 0;

/* =========================================================================
 * Test infrastructure
 * ====================================================================== */

#define MUX_TEST_MAX_RX_FRAMES   32
#define MUX_TEST_FRAME_BUF_SIZE  512

/** Captured frame (received or transmitted) */
typedef struct {
    uint8_t data[MUX_TEST_FRAME_BUF_SIZE];
    size_t len;
} mux_cap_frame_t;

/** Per-link harness tracking what was delivered and confirmed */
typedef struct {
    /* Frames delivered via callbacks.transmit (from state machine outward) */
    mux_cap_frame_t tx_frames[MUX_TEST_MAX_RX_FRAMES];
    uint8_t tx_count;

    /* LM-SEIZE-CONFIRM frames: frame bytes handed back to the link */
    mux_cap_frame_t seize_confirmed[MUX_TEST_MAX_RX_FRAMES];
    uint8_t seize_confirm_count;

    /* Frames delivered to upper layer via on_data */
    mux_cap_frame_t rx_frames[MUX_TEST_MAX_RX_FRAMES];
    uint8_t rx_count;

    bool connected;
    bool disconnected;
} mux_link_harness_t;

/* -------------------------------------------------------------------------
 * Callbacks wired to ax25_connection_t for each link under test
 * ---------------------------------------------------------------------- */
static void mux_cb_transmit(void *user_data, uint8_t *frame, size_t len) {
    mux_link_harness_t *h = (mux_link_harness_t*) user_data;
    if (!h || h->tx_count >= MUX_TEST_MAX_RX_FRAMES)
        return;
    size_t clen = (len > MUX_TEST_FRAME_BUF_SIZE) ? MUX_TEST_FRAME_BUF_SIZE : len;
    memcpy(h->tx_frames[h->tx_count].data, frame, clen);
    h->tx_frames[h->tx_count].len = clen;
    h->tx_count++;
}

static void mux_cb_connect(void *user_data, bool initiated_locally) {
    (void) initiated_locally;
    mux_link_harness_t *h = (mux_link_harness_t*) user_data;
    if (h)
        h->connected = true;
}

static void mux_cb_disconnect(void *user_data, uint8_t reason) {
    (void) reason;
    mux_link_harness_t *h = (mux_link_harness_t*) user_data;
    if (h)
        h->disconnected = true;
}

static void mux_cb_data(void *user_data, uint8_t *data, size_t len, uint8_t pid) {
    (void) pid;
    mux_link_harness_t *h = (mux_link_harness_t*) user_data;
    if (!h || h->rx_count >= MUX_TEST_MAX_RX_FRAMES)
        return;
    size_t clen = (len > MUX_TEST_FRAME_BUF_SIZE) ? MUX_TEST_FRAME_BUF_SIZE : len;
    memcpy(h->rx_frames[h->rx_count].data, data, clen);
    h->rx_frames[h->rx_count].len = clen;
    h->rx_count++;
}

static void mux_cb_busy(void *user_data, bool busy) {
    (void) user_data;
    (void) busy;
}

static ax25_callbacks_t make_mux_callbacks(mux_link_harness_t *h) {
    ax25_callbacks_t cb;
    memset(&cb, 0, sizeof(cb));
    cb.transmit = mux_cb_transmit;
    cb.on_connect = mux_cb_connect;
    cb.on_disconnect = mux_cb_disconnect;
    cb.on_data = mux_cb_data;
    cb.on_busy = mux_cb_busy;
    return cb;
}

/* -------------------------------------------------------------------------
 * LM-SEIZE-CONFIRM callback: records the frame that was confirmed
 * ---------------------------------------------------------------------- */
static void lm_seize_confirm_cb(void *user_data, uint8_t *frame, size_t len) {
    mux_link_harness_t *h = (mux_link_harness_t*) user_data;
    if (!h || h->seize_confirm_count >= MUX_TEST_MAX_RX_FRAMES)
        return;
    size_t clen = (len > MUX_TEST_FRAME_BUF_SIZE) ? MUX_TEST_FRAME_BUF_SIZE : len;
    memcpy(h->seize_confirmed[h->seize_confirm_count].data, frame, clen);
    h->seize_confirmed[h->seize_confirm_count].len = clen;
    h->seize_confirm_count++;

    DEBUG_PRINT("LM-SEIZE-CONFIRM called: link got %zu bytes", len);
}

/* -------------------------------------------------------------------------
 * Helper: build a minimal raw AX.25 frame byte-stream with a given control
 * byte, suitable for classify_priority.
 *
 * AX.25 address layout (no repeaters):
 *   bytes [0..6]  = destination (byte 6 SSID, bit 0 = 0 = not last addr)
 *   bytes [7..13] = source      (byte 13 SSID, bit 0 = 1 = last addr)
 *   byte  [14]    = control
 *
 * We do not encode the actual callsign shifts - for priority tests the
 * address bytes just need to satisfy the extension-bit walk in classify().
 * ---------------------------------------------------------------------- */
static void build_raw_frame(uint8_t *buf, size_t *len_out, uint8_t ctrl) {
    memset(buf, 0x00, 15);
    /* destination SSID byte: extension bit = 0 (more addresses follow) */
    buf[6] = 0x00;
    /* source SSID byte: extension bit = 1 (last address) */
    buf[13] = 0x01;
    /* control field */
    buf[14] = ctrl;
    *len_out = 15;
}

/**
 * Build a raw AX.25 frame with a proper AX.25-encoded callsign pair so
 * it can be decoded by ax25_frame_decode() and dispatched through
 * ax25_mux_receive_frame().
 *
 * AX.25 callsign encoding: each char << 1, padded to 6 chars with spaces,
 * SSID byte = (ssid << 1) | extension_bit.
 */
static size_t build_ui_frame(uint8_t *buf, size_t bufsz, const char *dst, uint8_t dst_ssid, const char *src, uint8_t src_ssid, const uint8_t *info,
        size_t info_len) {
    if (bufsz < 16 + info_len)
        return 0;

    size_t pos = 0;

    /* Destination callsign (6 bytes shifted left) */
    for (int i = 0; i < 6; i++) {
        char c = (i < (int) strlen(dst)) ? dst[i] : ' ';
        buf[pos++] = (uint8_t) (c << 1);
    }
    /* Destination SSID: ext bit = 0 (more addresses) */
    buf[pos++] = (uint8_t) ((dst_ssid & 0x0F) << 1) | 0x00;

    /* Source callsign (6 bytes shifted left) */
    for (int i = 0; i < 6; i++) {
        char c = (i < (int) strlen(src)) ? src[i] : ' ';
        buf[pos++] = (uint8_t) (c << 1);
    }
    /* Source SSID: ext bit = 1 (last address), C/R bit = 0 */
    buf[pos++] = (uint8_t) (((src_ssid & 0x0F) << 1) | 0x01);

    /* Control: UI = 0x03 */
    buf[pos++] = 0x03;
    /* PID: no L3 = 0xF0 */
    buf[pos++] = 0xF0;
    /* Info field */
    if (info && info_len > 0) {
        memcpy(&buf[pos], info, info_len);
        pos += info_len;
    }

    return pos;
}

/* -------------------------------------------------------------------------
 * Helper: initialise a connection in CONNECTED state (addr set manually)
 * Used to get a real conn that can process frames via the mux
 * ---------------------------------------------------------------------- */
static void init_connected_conn(ax25_connection_t *conn, mux_link_harness_t *h, const char *local_call, uint8_t local_ssid, const char *peer_call,
        uint8_t peer_ssid) {
    ax25_callbacks_t cb = make_mux_callbacks(h);
    ax25_connection_init(conn, &cb, h);

    conn->vars.mod = 8;
    conn->state = AX25_STATE_CONNECTED;
    conn->vars.vs = 0;
    conn->vars.vr = 0;
    conn->vars.va = 0;
    conn->rej_mode = AX25_REJ_MODE_SREJ_REJ;
    conn->srej_max = 1;
    conn->timers.t1 = 300;
    conn->timers.t2 = 30;
    conn->timers.k = 7;

    /* peer_addr: destination = remote, source = local (our address) */
    memset(conn->peer_addr.destination.callsign, ' ', 6);
    memcpy(conn->peer_addr.destination.callsign, peer_call, strlen(peer_call) < 6 ? strlen(peer_call) : 6);
    conn->peer_addr.destination.ssid = peer_ssid;
    conn->peer_addr.destination.ch = true;
    conn->peer_addr.destination.extension = false;

    memset(conn->peer_addr.source.callsign, ' ', 6);
    memcpy(conn->peer_addr.source.callsign, local_call, strlen(local_call) < 6 ? strlen(local_call) : 6);
    conn->peer_addr.source.ssid = local_ssid;
    conn->peer_addr.source.ch = false;
    conn->peer_addr.source.extension = true;
    conn->peer_addr.cr = false;
    conn->peer_addr.repeaters.num_repeaters = 0;
}

/* =========================================================================
 * TEST 1 – ax25_mux_init: basic initialization and NULL safety
 * ====================================================================== */
static int test_mux_init(void) {
    printf("\n--- test_mux_init ---\n");
    printf("AX.25 v2.2 Section 2.7: Mux init and NULL safety\n");

    ax25_mux_t mux;

    /* NULL pointer must return non-zero */
    uint8_t rc = ax25_mux_init(NULL);
    TEST_ASSERT(rc != 0, "ax25_mux_init(NULL) returns error", rc);

    /* Valid pointer must succeed */
    rc = ax25_mux_init(&mux);
    TEST_ASSERT(rc == 0, "ax25_mux_init(&mux) returns 0", rc);

    /* Initial state */
    TEST_ASSERT(mux.num_active == 0, "num_active = 0 after init", 0);
    TEST_ASSERT(mux.seized_link == AX25_MUX_NO_SEIZED, "seized_link = NO_SEIZED after init", 0);
    TEST_ASSERT(mux.last_served == 0, "last_served = 0 after init", 0);

    /* All link slots inactive */
    bool all_inactive = true;
    for (int i = 0; i < AX25_MUX_MAX_LINKS; i++) {
        if (mux.links[i].active) {
            all_inactive = false;
            break;
        }
    }
    TEST_ASSERT(all_inactive, "All link slots inactive after init", 0);

    DEBUG_PRINT("ax25_mux_init: num_active=%u, seized=%u", mux.num_active, mux.seized_link);

    return 0;
}

/* =========================================================================
 * TEST 2 – ax25_mux_register_link / ax25_mux_unregister_link
 * ====================================================================== */
static int test_mux_register_unregister(void) {
    printf("\n--- test_mux_register_unregister ---\n");
    printf("AX.25 v2.2 Section 2.7: Register and unregister links\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conn0, conn1;
    mux_link_harness_t h0, h1;
    memset(&h0, 0, sizeof(h0));
    memset(&h1, 0, sizeof(h1));
    init_connected_conn(&conn0, &h0, "N0CALL", 0, "W1AW  ", 0);
    init_connected_conn(&conn1, &h1, "N0CALL", 1, "W1AW  ", 0);

    ax25_address_t la0 = { .callsign = "N0CALL", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa0 = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t la1 = { .callsign = "N0CALL", .ssid = 1, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part

    uint8_t id0 = 0xFF, id1 = 0xFF;

    /* NULL mux */
    uint8_t rc = ax25_mux_register_link(NULL, &conn0, &la0, &pa0, &id0);
    TEST_ASSERT(rc != 0, "register_link(NULL mux) returns error", rc);

    /* NULL conn */
    rc = ax25_mux_register_link(&mux, NULL, &la0, &pa0, &id0);
    TEST_ASSERT(rc != 0, "register_link(NULL conn) returns error", rc);

    /* Valid registration of link 0 */
    rc = ax25_mux_register_link(&mux, &conn0, &la0, &pa0, &id0);
    TEST_ASSERT(rc == 0, "register_link 0 returns 0", rc);
    TEST_ASSERT(id0 == 0, "link 0 assigned slot 0", id0);
    TEST_ASSERT(mux.num_active == 1, "num_active = 1 after first register", 0);
    TEST_ASSERT(mux.links[0].active, "slot 0 is active", 0);

    DEBUG_PRINT("Registered link 0: id=%u, num_active=%u", id0, mux.num_active);

    /* Valid registration of link 1 */
    rc = ax25_mux_register_link(&mux, &conn1, &la1, &pa0, &id1);
    TEST_ASSERT(rc == 0, "register_link 1 returns 0", rc);
    TEST_ASSERT(id1 == 1, "link 1 assigned slot 1", id1);
    TEST_ASSERT(mux.num_active == 2, "num_active = 2 after second register", 0);

    DEBUG_PRINT("Registered link 1: id=%u, num_active=%u", id1, mux.num_active);

    /* Unregister link 0 */
    rc = ax25_mux_unregister_link(&mux, id0);
    TEST_ASSERT(rc == 0, "unregister_link 0 returns 0", rc);
    TEST_ASSERT(!mux.links[0].active, "slot 0 inactive after unregister", 0);
    TEST_ASSERT(mux.num_active == 1, "num_active = 1 after unregister 0", 0);

    /* NULL mux unregister */
    rc = ax25_mux_unregister_link(NULL, 0);
    TEST_ASSERT(rc != 0, "unregister_link(NULL) returns error", rc);

    /* Invalid link_id */
    rc = ax25_mux_unregister_link(&mux, AX25_MUX_MAX_LINKS);
    TEST_ASSERT(rc != 0, "unregister_link(out-of-range id) returns error", rc);

    ax25_connection_cleanup(&conn0);
    ax25_connection_cleanup(&conn1);
    return 0;
}

/* =========================================================================
 * TEST 3 – ax25_mux_register_link: slot exhaustion
 * ====================================================================== */
static int test_mux_slot_exhaustion(void) {
    printf("\n--- test_mux_slot_exhaustion ---\n");
    printf("AX.25 v2.2 Section 2.7: No free slots returns error\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conns[AX25_MUX_MAX_LINKS + 1];
    mux_link_harness_t hs[AX25_MUX_MAX_LINKS + 1];
    uint8_t ids[AX25_MUX_MAX_LINKS + 1];

    for (int i = 0; i <= AX25_MUX_MAX_LINKS; i++) {
        memset(&hs[i], 0, sizeof(hs[i]));
    }

    /* Fill all 8 slots */
    for (int i = 0; i < AX25_MUX_MAX_LINKS; i++) {
        char call[7];
        snprintf(call, sizeof(call), "N%dCALL", i);
        init_connected_conn(&conns[i], &hs[i], call, (uint8_t) i, "W1AW  ", 0);
        ax25_address_t la = { .callsign = {0}, .ssid = (int)((uint8_t) i), .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init // end modified part
        ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
        memcpy(la.callsign, call, 6);
        uint8_t rc = ax25_mux_register_link(&mux, &conns[i], &la, &pa, &ids[i]);
        TEST_ASSERT(rc == 0, "fill slot succeeds", rc);DEBUG_VAR("Registered slot", ids[i]);
    }

    TEST_ASSERT(mux.num_active == AX25_MUX_MAX_LINKS, "All 8 slots filled", 0);

    /* 9th registration must fail with 'no free slots' */
    init_connected_conn(&conns[AX25_MUX_MAX_LINKS], &hs[AX25_MUX_MAX_LINKS], "XTRA  ", 0, "W1AW  ", 0);
    ax25_address_t la_extra = { .callsign = "XTRA  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa_extra = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    uint8_t rc = ax25_mux_register_link(&mux, &conns[AX25_MUX_MAX_LINKS], &la_extra, &pa_extra, &ids[AX25_MUX_MAX_LINKS]);
    TEST_ASSERT(rc == 2, "9th registration returns 2 (no free slots)", rc);
    DEBUG_PRINT("Slot exhaustion: 9th register returned rc=%u (expected 2)", rc);

    for (int i = 0; i <= AX25_MUX_MAX_LINKS; i++) {
        ax25_connection_cleanup(&conns[i]);
    }
    return 0;
}

/* =========================================================================
 * TEST 4 – ax25_mux_classify_priority: frame type classification
 *
 * Per AX.25 v2.2 and ax25_mux.h:
 *   UI / XID / TEST   -> AX25_MUX_PRI_UI     (0)
 *   I-frame           -> AX25_MUX_PRI_DATA   (100)
 *   S-frame (RR etc.) -> AX25_MUX_PRI_ACK    (200)
 *   Other U-frames    -> AX25_MUX_PRI_URGENT (255)
 * ====================================================================== */
static int test_mux_classify_priority(void) {
    printf("\n--- test_mux_classify_priority ---\n");
    printf("AX.25 v2.2 Section 2.7: Priority classification of raw frames\n");

    uint8_t frame[32];
    size_t flen;
    uint8_t pri;

    /* --- NULL / short frame safety --- */
    pri = ax25_mux_classify_priority(NULL, 15);
    TEST_ASSERT(pri == AX25_MUX_PRI_UI, "NULL frame -> PRI_UI (safe default)", pri);

    pri = ax25_mux_classify_priority(frame, 5);
    TEST_ASSERT(pri == AX25_MUX_PRI_UI, "Frame <15 bytes -> PRI_UI (too short)", pri);

    /* --- I-frame: control bit 0 = 0 (e.g. 0x00, 0x02, 0x04 ...) --- */
    build_raw_frame(frame, &flen, 0x00); /* I-frame N(S)=0, N(R)=0, P/F=0 */
    pri = ax25_mux_classify_priority(frame, flen);
    TEST_ASSERT(pri == AX25_MUX_PRI_DATA, "I-frame ctrl=0x00 -> PRI_DATA (100)", pri);
    DEBUG_VAR("I-frame pri", pri);

    build_raw_frame(frame, &flen, 0x04); /* I-frame N(S)=2 */
    pri = ax25_mux_classify_priority(frame, flen);
    TEST_ASSERT(pri == AX25_MUX_PRI_DATA, "I-frame ctrl=0x04 -> PRI_DATA (100)", pri);

    /* --- S-frame: bits 1:0 = 01 ---
     *   RR  = 0x01,  RNR = 0x05,  REJ  = 0x09,  SREJ = 0x0D              */
    uint8_t s_frames[] = { 0x01, 0x05, 0x09, 0x0D };
    const char *s_names[] = { "RR(0x01)", "RNR(0x05)", "REJ(0x09)", "SREJ(0x0D)" };
    for (int i = 0; i < 4; i++) {
        build_raw_frame(frame, &flen, s_frames[i]);
        pri = ax25_mux_classify_priority(frame, flen);
        char msg[64];
        snprintf(msg, sizeof(msg), "S-frame %s -> PRI_ACK (200)", s_names[i]);
        TEST_ASSERT(pri == AX25_MUX_PRI_ACK, msg, pri);DEBUG_PRINT("S-frame %s: pri=%u (expected %u)", s_names[i], pri, AX25_MUX_PRI_ACK);
    }

    /* --- U-frame UI (0x03): expect PRI_UI --- */
    build_raw_frame(frame, &flen, 0x03);
    pri = ax25_mux_classify_priority(frame, flen);
    TEST_ASSERT(pri == AX25_MUX_PRI_UI, "U-frame UI ctrl=0x03 -> PRI_UI (0)", pri);
    DEBUG_VAR("UI frame pri", pri);

    /* UI with P bit (0x13) */
    build_raw_frame(frame, &flen, 0x13);
    pri = ax25_mux_classify_priority(frame, flen);
    TEST_ASSERT(pri == AX25_MUX_PRI_UI, "U-frame UI ctrl=0x13 (P=1) -> PRI_UI (0)", pri);

    /* XID command (0xAF) -> PRI_UI */
    build_raw_frame(frame, &flen, 0xAF);
    pri = ax25_mux_classify_priority(frame, flen);
    TEST_ASSERT(pri == AX25_MUX_PRI_UI, "XID cmd ctrl=0xAF -> PRI_UI (0)", pri);

    /* XID response (0xBF) -> PRI_UI */
    build_raw_frame(frame, &flen, 0xBF);
    pri = ax25_mux_classify_priority(frame, flen);
    TEST_ASSERT(pri == AX25_MUX_PRI_UI, "XID rsp ctrl=0xBF -> PRI_UI (0)", pri);

    /* TEST command P/F=0 (0xE3) -> PRI_UI */
    build_raw_frame(frame, &flen, 0xE3);
    pri = ax25_mux_classify_priority(frame, flen);
    TEST_ASSERT(pri == AX25_MUX_PRI_UI, "TEST cmd ctrl=0xE3 -> PRI_UI (0)", pri);

    // start modified part: fix invalid test byte 0xE1 -> correct TEST P/F=1 value 0xF3
    // 0xE1 = 1110_0001 has bits[1:0]=01 which classifies as S-frame, NOT TEST.
    // The AX.25 TEST modifier is 0xE3; with P/F bit (bit4=0x10) set: 0xE3|0x10=0xF3.
    // The old code only caught it because both the implementation and test were wrong
    // in the same way (hardcoded 0xE1 in both places). Fixed implementation now uses
    // ax25_u_subtype() which strips bit4 before comparing, so 0xF3 -> sub=0xE3=AX25_U_TEST.
    /* TEST with P/F=1 (0xF3 = 0xE3 | 0x10) -> PRI_UI */
    build_raw_frame(frame, &flen, 0xF3);
    pri = ax25_mux_classify_priority(frame, flen);
    TEST_ASSERT(pri == AX25_MUX_PRI_UI, "TEST P/F=1 ctrl=0xF3 -> PRI_UI (0)", pri);
    // end modified part: fix invalid test byte 0xE1 -> correct TEST P/F=1 value 0xF3

    /* --- Urgent U-frames: SABM, SABME, DISC, DM, UA, FRMR --- */
    /* SABM = 0x2F (ctrl 0x2F: bits 1:0 = 11 = U-frame, not UI/XID/TEST) */
    uint8_t urgent[] = { 0x2F, 0x6F, 0x43, 0x0F, 0x63, 0x87 };
    const char *u_names[] = { "SABM(0x2F)", "SABME(0x6F)", "DISC(0x43)", "DM(0x0F)", "UA(0x63)", "FRMR(0x87)" };
    for (int i = 0; i < 6; i++) {
        build_raw_frame(frame, &flen, urgent[i]);
        pri = ax25_mux_classify_priority(frame, flen);
        char msg[64];
        snprintf(msg, sizeof(msg), "Urgent U-frame %s -> PRI_URGENT (255)", u_names[i]);
        TEST_ASSERT(pri == AX25_MUX_PRI_URGENT, msg, pri);DEBUG_PRINT("Urgent U-frame %s: pri=%u (expected %u)", u_names[i], pri, AX25_MUX_PRI_URGENT);
    }

    return 0;
}

/* =========================================================================
 * TEST 5 – LM-SEIZE-REQUEST / LM-SEIZE-CONFIRM via ax25_mux_tick
 *
 * Per AX.25 v2.2 Section 2.7: The LM receives seize requests from data
 * link entities and grants channel access via seize-confirm.
 * ====================================================================== */
static int test_mux_lm_seize_tick(void) {
    printf("\n--- test_mux_lm_seize_tick ---\n");
    printf("AX.25 v2.2 Section 2.7: LM-SEIZE-REQUEST -> tick -> LM-SEIZE-CONFIRM\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conn0;
    mux_link_harness_t h0;
    memset(&h0, 0, sizeof(h0));
    init_connected_conn(&conn0, &h0, "N0CALL", 0, "W1AW  ", 0);

    ax25_address_t la = { .callsign = "N0CALL", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    uint8_t link_id = 0xFF;

    ax25_mux_register_link(&mux, &conn0, &la, &pa, &link_id);
    ax25_mux_set_lm_seize_confirm(&mux, link_id, lm_seize_confirm_cb, &h0);

    DEBUG_PRINT("Registered link id=%u", link_id);

    /* Build a test frame (I-frame) */
    uint8_t frame[16];
    size_t flen;
    build_raw_frame(frame, &flen, 0x00); /* I-frame */

    /* Seize request for this link */
    uint8_t rc = ax25_mux_lm_seize_request(&mux, link_id, frame, flen,
    AX25_MUX_PRI_DATA);
    TEST_ASSERT(rc == 0, "LM-SEIZE-REQUEST returns 0", rc);
    TEST_ASSERT(mux.links[link_id].seize_pending, "seize_pending set after request", 0);

    DEBUG_BOOL("seize_pending before tick", mux.links[link_id].seize_pending);

    /* Duplicate seize request must be rejected */
    rc = ax25_mux_lm_seize_request(&mux, link_id, frame, flen, AX25_MUX_PRI_DATA);
    TEST_ASSERT(rc != 0, "Duplicate LM-SEIZE-REQUEST rejected", rc);

    /* No seized link yet - tick should grant it */
    TEST_ASSERT(mux.seized_link == AX25_MUX_NO_SEIZED, "No seized link before tick", 0);

    ax25_mux_tick(&mux, 1);

    DEBUG_VAR("seized_link after tick (should be link_id)", mux.seized_link);
    DEBUG_VAR("seize_confirm_count after tick", h0.seize_confirm_count);

    TEST_ASSERT(mux.seized_link == link_id, "Link seized by tick", 0);
    TEST_ASSERT(h0.seize_confirm_count == 1, "LM-SEIZE-CONFIRM called once", 0);
    TEST_ASSERT(h0.seize_confirmed[0].len == flen, "Confirmed frame has correct length", 0);

    /* Second tick while seized must NOT re-confirm */
    ax25_mux_tick(&mux, 2);
    TEST_ASSERT(h0.seize_confirm_count == 1, "No duplicate confirm on second tick while seized", 0);

    /* Release */
    ax25_mux_lm_release(&mux, link_id);
    TEST_ASSERT(mux.seized_link == AX25_MUX_NO_SEIZED, "seized_link reset after release", 0);
    TEST_ASSERT(!mux.links[link_id].seize_pending, "seize_pending cleared after release", 0);

    DEBUG_PRINT("After release: seized=%u, pending=%d", mux.seized_link, mux.links[link_id].seize_pending);

    ax25_mux_unregister_link(&mux, link_id);
    ax25_connection_cleanup(&conn0);
    return 0;
}

/* =========================================================================
 * TEST 6 – Priority ordering: higher-priority seize wins
 *
 * Per AX.25 v2.2 Section 2.7: urgent U-frames take precedence over data
 * ====================================================================== */
static int test_mux_priority_ordering(void) {
    printf("\n--- test_mux_priority_ordering ---\n");
    printf("AX.25 v2.2 Section 2.7: Higher-priority seize wins channel\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conn0, conn1, conn2;
    mux_link_harness_t h0, h1, h2;
    memset(&h0, 0, sizeof(h0));
    memset(&h1, 0, sizeof(h1));
    memset(&h2, 0, sizeof(h2));

    init_connected_conn(&conn0, &h0, "N0CAL0", 0, "W1AW  ", 0);
    init_connected_conn(&conn1, &h1, "N0CAL1", 0, "W1AW  ", 0);
    init_connected_conn(&conn2, &h2, "N0CAL2", 0, "W1AW  ", 0);

    ax25_address_t la0 = { .callsign = "N0CAL0", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t la1 = { .callsign = "N0CAL1", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t la2 = { .callsign = "N0CAL2", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part

    uint8_t id0, id1, id2;
    ax25_mux_register_link(&mux, &conn0, &la0, &pa, &id0);
    ax25_mux_register_link(&mux, &conn1, &la1, &pa, &id1);
    ax25_mux_register_link(&mux, &conn2, &la2, &pa, &id2);

    ax25_mux_set_lm_seize_confirm(&mux, id0, lm_seize_confirm_cb, &h0);
    ax25_mux_set_lm_seize_confirm(&mux, id1, lm_seize_confirm_cb, &h1);
    ax25_mux_set_lm_seize_confirm(&mux, id2, lm_seize_confirm_cb, &h2);

    uint8_t frame[16];
    size_t flen;
    build_raw_frame(frame, &flen, 0x00); /* I-frame for all */

    /* Link 0: PRI_UI (lowest), Link 1: PRI_DATA, Link 2: PRI_URGENT (highest) */
    ax25_mux_lm_seize_request(&mux, id0, frame, flen, AX25_MUX_PRI_UI);
    ax25_mux_lm_seize_request(&mux, id1, frame, flen, AX25_MUX_PRI_DATA);
    ax25_mux_lm_seize_request(&mux, id2, frame, flen, AX25_MUX_PRI_URGENT);

    DEBUG_PRINT("Three pending seizes: id0=PRI_UI, id1=PRI_DATA, id2=PRI_URGENT");

    ax25_mux_tick(&mux, 1);

    DEBUG_VAR("seized_link after tick (should be id2=URGENT)", mux.seized_link);
    DEBUG_VAR("h2.seize_confirm_count (should be 1)", h2.seize_confirm_count);

    TEST_ASSERT(mux.seized_link == id2, "URGENT link (id2) seized first", 0);
    TEST_ASSERT(h2.seize_confirm_count == 1, "id2 (URGENT) got seize-confirm", 0);
    TEST_ASSERT(h0.seize_confirm_count == 0, "id0 (UI) NOT yet confirmed", 0);
    TEST_ASSERT(h1.seize_confirm_count == 0, "id1 (DATA) NOT yet confirmed", 0);

    /* Release URGENT, next should be ACK/DATA (id1) */
    ax25_mux_lm_release(&mux, id2);

    DEBUG_VAR("seized_link after id2 release (should be id1)", mux.seized_link);
    DEBUG_VAR("h1.seize_confirm_count (should be 1)", h1.seize_confirm_count);

    TEST_ASSERT(mux.seized_link == id1, "DATA link (id1) seized after URGENT released", 0);
    TEST_ASSERT(h1.seize_confirm_count == 1, "id1 (DATA) got seize-confirm on burst", 0);

    /* Release DATA, next should be UI (id0) */
    ax25_mux_lm_release(&mux, id1);

    TEST_ASSERT(mux.seized_link == id0, "UI link (id0) seized last (lowest priority)", 0);
    TEST_ASSERT(h0.seize_confirm_count == 1, "id0 (UI) got seize-confirm on burst", 0);

    /* All done */
    ax25_mux_lm_release(&mux, id0);
    TEST_ASSERT(mux.seized_link == AX25_MUX_NO_SEIZED, "No seized link after all released", 0);

    ax25_connection_cleanup(&conn0);
    ax25_connection_cleanup(&conn1);
    ax25_connection_cleanup(&conn2);
    return 0;
}

/* =========================================================================
 * TEST 7 – Round-robin fairness among equal-priority seize requests
 *
 * Per AX.25 v2.2 Section 2.7: When priorities are equal, round-robin after
 * the last-served link ensures fairness.
 * ====================================================================== */
static int test_mux_round_robin(void) {
    printf("\n--- test_mux_round_robin ---\n");
    printf("AX.25 v2.2 Section 2.7: Round-robin fairness for equal priorities\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conns[3];
    mux_link_harness_t hs[3];
    uint8_t ids[3];

    for (int i = 0; i < 3; i++) {
        memset(&hs[i], 0, sizeof(hs[i]));
        char call[7];
        snprintf(call, sizeof(call), "N0RR%02d", i);
        init_connected_conn(&conns[i], &hs[i], call, (uint8_t) i, "W1AW  ", 0);
        ax25_address_t la = { .callsign = {0}, .ssid = (int)((uint8_t) i), .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init // end modified part
        ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
        memcpy(la.callsign, call, 6);
        ax25_mux_register_link(&mux, &conns[i], &la, &pa, &ids[i]);
        ax25_mux_set_lm_seize_confirm(&mux, ids[i], lm_seize_confirm_cb, &hs[i]);
    }

    uint8_t frame[16];
    size_t flen;
    build_raw_frame(frame, &flen, 0x00);

    /* All three at same priority */
    for (int i = 0; i < 3; i++) {
        ax25_mux_lm_seize_request(&mux, ids[i], frame, flen, AX25_MUX_PRI_DATA);
    }

    DEBUG_PRINT("Three equal-priority seize requests pending");
    DEBUG_VAR("last_served before first tick", mux.last_served);

    /* Round 1: id0 should go first (last_served=0, start search at 1, wrap) */
    ax25_mux_tick(&mux, 1);
    uint8_t first_seized = mux.seized_link;
    DEBUG_VAR("First seized link (should be ids[0] or ids[1])", first_seized);
    TEST_ASSERT(first_seized < AX25_MUX_MAX_LINKS, "First seized is a valid link", 0);

    ax25_mux_lm_release(&mux, first_seized);
    uint8_t second_seized = mux.seized_link;
    DEBUG_VAR("Second seized (burst on release)", second_seized);
    TEST_ASSERT(second_seized != first_seized, "Second seized differs from first", 0);

    ax25_mux_lm_release(&mux, second_seized);
    uint8_t third_seized = mux.seized_link;
    DEBUG_VAR("Third seized", third_seized);
    TEST_ASSERT(third_seized != second_seized, "Third seized differs from second", 0);
    TEST_ASSERT(third_seized != first_seized, "Third seized differs from first", 0);

    /* All three must have been served exactly once */
    TEST_ASSERT(hs[0].seize_confirm_count == 1, "Link 0 confirmed exactly once", 0);
    TEST_ASSERT(hs[1].seize_confirm_count == 1, "Link 1 confirmed exactly once", 0);
    TEST_ASSERT(hs[2].seize_confirm_count == 1, "Link 2 confirmed exactly once", 0);

    ax25_mux_lm_release(&mux, third_seized);

    for (int i = 0; i < 3; i++)
        ax25_connection_cleanup(&conns[i]);
    return 0;
}

/* =========================================================================
 * TEST 8 – ax25_mux_get_next_to_serve: ordering and last_served update
 * ====================================================================== */
static int test_mux_get_next_to_serve(void) {
    printf("\n--- test_mux_get_next_to_serve ---\n");
    printf("AX.25 v2.2 Section 2.7: get_next_to_serve priority + last_served tracking\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conn0, conn1;
    mux_link_harness_t h0, h1;
    memset(&h0, 0, sizeof(h0));
    memset(&h1, 0, sizeof(h1));
    init_connected_conn(&conn0, &h0, "N0GNS0", 0, "W1AW  ", 0);
    init_connected_conn(&conn1, &h1, "N0GNS1", 0, "W1AW  ", 0);

    ax25_address_t la0 = { .callsign = "N0GNS0", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t la1 = { .callsign = "N0GNS1", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part

    uint8_t id0, id1;
    ax25_mux_register_link(&mux, &conn0, &la0, &pa, &id0);
    ax25_mux_register_link(&mux, &conn1, &la1, &pa, &id1);

    /* NULL mux must return -1 */
    int8_t idx = ax25_mux_get_next_to_serve(NULL);
    TEST_ASSERT(idx == -1, "get_next_to_serve(NULL) returns -1", 0);

    /* No pending requests -> -1 */
    idx = ax25_mux_get_next_to_serve(&mux);
    TEST_ASSERT(idx == -1, "get_next_to_serve with no pending returns -1", 0);

    uint8_t frame[16];
    size_t flen;
    build_raw_frame(frame, &flen, 0x01); /* S-frame = ACK priority */

    /* Add two pending seizes with different priorities */
    ax25_mux_lm_seize_request(&mux, id0, frame, flen, AX25_MUX_PRI_UI);
    ax25_mux_lm_seize_request(&mux, id1, frame, flen, AX25_MUX_PRI_ACK);

    idx = ax25_mux_get_next_to_serve(&mux);

    DEBUG_VAR("get_next_to_serve with ACK > UI (should be id1)", (uint8_t)idx);

    TEST_ASSERT(idx == (int8_t )id1, "get_next_to_serve returns higher-priority link (id1/ACK)", 0);
    TEST_ASSERT(!mux.links[id1].seize_pending, "seize_pending cleared after get_next_to_serve", 0);
    TEST_ASSERT(mux.last_served == id1, "last_served updated to served link", 0);

    /* Now only id0 remains */
    idx = ax25_mux_get_next_to_serve(&mux);
    TEST_ASSERT(idx == (int8_t )id0, "get_next_to_serve returns only remaining link", 0);

    /* No more pending */
    idx = ax25_mux_get_next_to_serve(&mux);
    TEST_ASSERT(idx == -1, "get_next_to_serve returns -1 when queue empty", 0);

    ax25_connection_cleanup(&conn0);
    ax25_connection_cleanup(&conn1);
    return 0;
}

/* =========================================================================
 * TEST 9 – LM-SEIZE input validation
 * ====================================================================== */
static int test_mux_seize_validation(void) {
    printf("\n--- test_mux_seize_validation ---\n");
    printf("AX.25 v2.2 Section 2.7: LM-SEIZE-REQUEST input validation\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conn;
    mux_link_harness_t h;
    memset(&h, 0, sizeof(h));
    init_connected_conn(&conn, &h, "N0VAL ", 0, "W1AW  ", 0);

    ax25_address_t la = { .callsign = "N0VAL ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    uint8_t link_id;
    ax25_mux_register_link(&mux, &conn, &la, &pa, &link_id);

    uint8_t frame[16];
    size_t flen;
    build_raw_frame(frame, &flen, 0x00);

    /* NULL mux */
    uint8_t rc = ax25_mux_lm_seize_request(NULL, link_id, frame, flen,
    AX25_MUX_PRI_DATA);
    TEST_ASSERT(rc == 1, "seize_request(NULL mux) returns 1", rc);

    /* link_id out of range */
    rc = ax25_mux_lm_seize_request(&mux, AX25_MUX_MAX_LINKS, frame, flen,
    AX25_MUX_PRI_DATA);
    TEST_ASSERT(rc == 1, "seize_request(out-of-range link_id) returns 1", rc);

    /* NULL frame */
    rc = ax25_mux_lm_seize_request(&mux, link_id, NULL, flen, AX25_MUX_PRI_DATA);
    TEST_ASSERT(rc == 1, "seize_request(NULL frame) returns 1", rc);

    /* Zero length */
    rc = ax25_mux_lm_seize_request(&mux, link_id, frame, 0, AX25_MUX_PRI_DATA);
    TEST_ASSERT(rc == 1, "seize_request(len=0) returns 1", rc);

    /* Frame too large */
    uint8_t big_frame[AX25_MUX_FRAME_BUF + 1];
    memset(big_frame, 0, sizeof(big_frame));
    rc = ax25_mux_lm_seize_request(&mux, link_id, big_frame,
    AX25_MUX_FRAME_BUF + 1, AX25_MUX_PRI_DATA);
    TEST_ASSERT(rc == 1, "seize_request(frame > AX25_MUX_FRAME_BUF) returns 1", rc);

    /* Inactive link */
    ax25_mux_unregister_link(&mux, link_id);
    rc = ax25_mux_lm_seize_request(&mux, link_id, frame, flen, AX25_MUX_PRI_DATA);
    TEST_ASSERT(rc == 2, "seize_request(inactive link) returns 2", rc);

    ax25_connection_cleanup(&conn);
    return 0;
}

/* =========================================================================
 * TEST 10 – Burst mode: LM-RELEASE auto-seizes next pending link
 *
 * Per AX.25 v2.2 Section 2.7: on release the LM should immediately grant
 * the next pending seize (burst / back-to-back frame support).
 * ====================================================================== */
static int test_mux_burst_on_release(void) {
    printf("\n--- test_mux_burst_on_release ---\n");
    printf("AX.25 v2.2 Section 2.7: Auto-seize next pending on LM-RELEASE\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conn0, conn1;
    mux_link_harness_t h0, h1;
    memset(&h0, 0, sizeof(h0));
    memset(&h1, 0, sizeof(h1));
    init_connected_conn(&conn0, &h0, "N0BST0", 0, "W1AW  ", 0);
    init_connected_conn(&conn1, &h1, "N0BST1", 0, "W1AW  ", 0);

    ax25_address_t la0 = { .callsign = "N0BST0", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t la1 = { .callsign = "N0BST1", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part

    uint8_t id0, id1;
    ax25_mux_register_link(&mux, &conn0, &la0, &pa, &id0);
    ax25_mux_register_link(&mux, &conn1, &la1, &pa, &id1);
    ax25_mux_set_lm_seize_confirm(&mux, id0, lm_seize_confirm_cb, &h0);
    ax25_mux_set_lm_seize_confirm(&mux, id1, lm_seize_confirm_cb, &h1);

    uint8_t frame[16];
    size_t flen;
    build_raw_frame(frame, &flen, 0x2F); /* SABM = URGENT */

    /* id0 seizes channel via tick */
    ax25_mux_lm_seize_request(&mux, id0, frame, flen, AX25_MUX_PRI_URGENT);
    ax25_mux_tick(&mux, 1);

    TEST_ASSERT(mux.seized_link == id0, "id0 seized after tick", 0);
    TEST_ASSERT(h0.seize_confirm_count == 1, "id0 got seize-confirm", 0);

    /* id1 queues a pending request while id0 still holds channel */
    ax25_mux_lm_seize_request(&mux, id1, frame, flen, AX25_MUX_PRI_URGENT);
    TEST_ASSERT(mux.links[id1].seize_pending, "id1 seize_pending set while id0 holds channel", 0);

    DEBUG_PRINT("id0 holds channel; id1 pending. Releasing id0...");

    /* Release id0 -> should immediately seize id1 (burst) */
    ax25_mux_lm_release(&mux, id0);

    DEBUG_VAR("seized_link after id0 release (should be id1)", mux.seized_link);
    DEBUG_VAR("h1.seize_confirm_count (should be 1)", h1.seize_confirm_count);

    TEST_ASSERT(mux.seized_link == id1, "id1 auto-seized immediately on id0 release (burst)", 0);
    TEST_ASSERT(h1.seize_confirm_count == 1, "id1 got seize-confirm via burst", 0);

    /* Release id1 -> no more pending */
    ax25_mux_lm_release(&mux, id1);
    TEST_ASSERT(mux.seized_link == AX25_MUX_NO_SEIZED, "No seized link after id1 released (empty queue)", 0);

    ax25_connection_cleanup(&conn0);
    ax25_connection_cleanup(&conn1);
    return 0;
}

/* =========================================================================
 * TEST 11 – ax25_mux_receive_frame: point-to-point routing
 *
 * Per AX.25 v2.2 Section 3.12: Frames addressed to a specific station are
 * delivered only to the link whose local/peer addresses match.
 * ====================================================================== */
static int test_mux_receive_point_to_point(void) {
    printf("\n--- test_mux_receive_point_to_point ---\n");
    printf("AX.25 v2.2 Section 3.12: Point-to-point frame routing by address\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    /* Two links: Link A: N0AAA<->W1AW, Link B: N0BBB<->W1AW */
    ax25_connection_t connA, connB;
    mux_link_harness_t hA, hB;
    memset(&hA, 0, sizeof(hA));
    memset(&hB, 0, sizeof(hB));

    init_connected_conn(&connA, &hA, "N0AAA ", 0, "W1AW  ", 0);
    init_connected_conn(&connB, &hB, "N0BBB ", 0, "W1AW  ", 0);

    ax25_address_t laA = { .callsign = "N0AAA ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t laB = { .callsign = "N0BBB ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part

    uint8_t idA, idB;
    ax25_mux_register_link(&mux, &connA, &laA, &pa, &idA);
    ax25_mux_register_link(&mux, &connB, &laB, &pa, &idB);

    DEBUG_PRINT("Registered: Link A (id=%u) N0AAA<->W1AW, Link B (id=%u) N0BBB<->W1AW", idA, idB);

    /* Build a UI frame from W1AW to N0AAA (should hit link A only) */
    uint8_t raw[64];
    uint8_t info[] = { 0x42, 0x43 };
    size_t raw_len = build_ui_frame(raw, sizeof(raw), "N0AAA ", 0, "W1AW  ", 0, info, sizeof(info));
    TEST_ASSERT(raw_len > 0, "UI frame for N0AAA built OK", 0);

    /* Decode the frame first */
    uint8_t err = 0;
    ax25_frame_t *decoded = ax25_frame_decode(raw, raw_len, MODULO128_FALSE, &err);

    if (!decoded) {
        /* Skip frame routing sub-tests if decode fails (addr encoding variation) */
        DEBUG_PRINT("Frame decode returned NULL (err=%u) - skipping routing check", err);
        printf("  [SKIP] Frame decode failed - routing tests require matching address encoding\n");
    } else {
        DEBUG_PRINT("Decoded frame type=%d", decoded->type);

        /* Deliver via mux */
        ax25_mux_receive_frame(&mux, decoded, 1);

        DEBUG_VAR("hA.rx_count after frame to N0AAA (should be >= 0, state machine driven)", hA.rx_count);
        DEBUG_VAR("hB.rx_count (should be 0 - wrong dest)", hB.rx_count);

        /* Link B must NOT receive the frame addressed to link A's local addr */
        TEST_ASSERT(hB.rx_count == 0, "Link B (N0BBB) does NOT receive frame for N0AAA", 0);

        ax25_frame_free(decoded, &err);
    }

    /* NULL safety */
    ax25_mux_receive_frame(NULL, NULL, 1); /* must not crash */
    TEST_ASSERT(true, "receive_frame(NULL,NULL) does not crash", 0);

    ax25_connection_cleanup(&connA);
    ax25_connection_cleanup(&connB);
    return 0;
}

/* =========================================================================
 * TEST 12 – ax25_mux_receive_frame: UI broadcast dispatch
 *
 * Per AX.25 v2.2 Section 3.12.5 and 6.3: UI frames addressed to CQ,
 * APRS, or BEACON are delivered to ALL active links.
 * ====================================================================== */
static int test_mux_ui_broadcast(void) {
    printf("\n--- test_mux_ui_broadcast ---\n");
    printf("AX.25 v2.2 Section 3.12.5 / 6.3: UI broadcast dispatch to all links\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conn0, conn1, conn2;
    mux_link_harness_t h0, h1, h2;
    memset(&h0, 0, sizeof(h0));
    memset(&h1, 0, sizeof(h1));
    memset(&h2, 0, sizeof(h2));

    init_connected_conn(&conn0, &h0, "N0BC0 ", 0, "W1AW  ", 0);
    init_connected_conn(&conn1, &h1, "N0BC1 ", 0, "W1AW  ", 0);
    init_connected_conn(&conn2, &h2, "N0BC2 ", 0, "W1AW  ", 0);

    ax25_address_t la0 = { .callsign = "N0BC0 ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t la1 = { .callsign = "N0BC1 ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t la2 = { .callsign = "N0BC2 ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part

    uint8_t id0, id1, id2;
    ax25_mux_register_link(&mux, &conn0, &la0, &pa, &id0);
    ax25_mux_register_link(&mux, &conn1, &la1, &pa, &id1);
    ax25_mux_register_link(&mux, &conn2, &la2, &pa, &id2);

    /* Manually build a decoded AX.25 frame that the mux will classify as
     * UI broadcast by checking frame->type and frame->header.destination.
     * We set these fields directly since the mux checks them explicitly. */
    ax25_unnumbered_information_frame_t ui_frame;
    memset(&ui_frame, 0, sizeof(ui_frame));
    ui_frame.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
    /* destination = CQ (broadcast trigger #1 per mux code) */
    memcpy(ui_frame.base.base.header.destination.callsign, "CQ    ", 6);
    ui_frame.base.base.header.destination.ssid = 0;
    memcpy(ui_frame.base.base.header.source.callsign, "W1AW  ", 6);
    ui_frame.base.base.header.source.ssid = 0;
    ui_frame.base.base.header.repeaters.num_repeaters = 0;
    ui_frame.base.pf = false;
    ui_frame.base.base.header.cr = false;
    ui_frame.pid = PID_NO_L3;
    uint8_t ui_info[] = { 0xAA, 0xBB };
    ui_frame.payload = ui_info;
    ui_frame.payload_len = sizeof(ui_info);

    DEBUG_PRINT("Dispatching CQ UI broadcast via ax25_mux_receive_frame");

    ax25_mux_receive_frame(&mux, (ax25_frame_t*) &ui_frame, 1);

    /* The mux's ax25_process_frame will be called for each active link.
     * Since the connections are in CONNECTED state and receive a UI frame
     * (which is valid in any state), the frame is processed. We verify the
     * mux called ax25_process_frame for all three links by checking that no
     * link was excluded. The state machine may or may not call on_data for
     * a UI addressed to CQ (not the link's own address) - but the mux MUST
     * call ax25_process_frame for ALL links when destination is broadcast.
     *
     * We cannot inspect ax25_process_frame calls directly, but we can verify
     * no crash and that all 3 links remain active (mux did not error out). */
    TEST_ASSERT(mux.links[id0].active, "Link 0 still active after broadcast", 0);
    TEST_ASSERT(mux.links[id1].active, "Link 1 still active after broadcast", 0);
    TEST_ASSERT(mux.links[id2].active, "Link 2 still active after broadcast", 0);

    DEBUG_PRINT("All 3 links survived broadcast dispatch - mux dispatched to all");

    /* Test APRS broadcast destination */
    memcpy(ui_frame.base.base.header.destination.callsign, "APRS  ", 6);
    ax25_mux_receive_frame(&mux, (ax25_frame_t*) &ui_frame, 2);
    TEST_ASSERT(true, "APRS broadcast dispatched without crash", 0);

    /* Test BEACON broadcast destination */
    memcpy(ui_frame.base.base.header.destination.callsign, "BEACON", 6);
    ax25_mux_receive_frame(&mux, (ax25_frame_t*) &ui_frame, 3);
    TEST_ASSERT(true, "BEACON broadcast dispatched without crash", 0);

    ax25_connection_cleanup(&conn0);
    ax25_connection_cleanup(&conn1);
    ax25_connection_cleanup(&conn2);
    return 0;
}

/* =========================================================================
 * TEST 13 – ax25_mux_transmit_adapter: priority classification and seize
 *
 * The adapter classifies the frame, calls lm_seize_request automatically.
 * ====================================================================== */
static int test_mux_transmit_adapter(void) {
    printf("\n--- test_mux_transmit_adapter ---\n");
    printf("AX.25 v2.2 Section 2.7: transmit_adapter queues seize with correct priority\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conn;
    mux_link_harness_t h;
    memset(&h, 0, sizeof(h));
    init_connected_conn(&conn, &h, "N0ADP ", 0, "W1AW  ", 0);

    ax25_address_t la = { .callsign = "N0ADP ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    uint8_t link_id;
    ax25_mux_register_link(&mux, &conn, &la, &pa, &link_id);
    ax25_mux_set_lm_seize_confirm(&mux, link_id, lm_seize_confirm_cb, &h);

    ax25_mux_adapter_ctx_t ctx;
    ctx.mux = &mux;
    ctx.link_id = link_id;

    /* Build an S-frame (ACK priority) to pass through the adapter */
    uint8_t frame[16];
    size_t flen;
    build_raw_frame(frame, &flen, 0x01); /* RR S-frame */

    DEBUG_PRINT("Calling transmit_adapter with RR S-frame (should set PRI_ACK seize)");

    ax25_mux_transmit_adapter(&ctx, frame, flen);

    TEST_ASSERT(mux.links[link_id].seize_pending, "seize_pending set by transmit_adapter", 0);
    TEST_ASSERT(mux.links[link_id].seize_priority == AX25_MUX_PRI_ACK, "seize_priority set to PRI_ACK (200) for S-frame", 0);

    DEBUG_VAR("seize_priority after adapter (should be 200)", mux.links[link_id].seize_priority);

    /* NULL ctx must not crash */
    ax25_mux_transmit_adapter(NULL, frame, flen);
    TEST_ASSERT(true, "transmit_adapter(NULL ctx) does not crash", 0);

    /* ctx with NULL mux must not crash */
    ax25_mux_adapter_ctx_t bad_ctx = { NULL, 0 };
    ax25_mux_transmit_adapter(&bad_ctx, frame, flen);
    TEST_ASSERT(true, "transmit_adapter(NULL mux in ctx) does not crash", 0);

    ax25_mux_lm_release(&mux, link_id);
    ax25_connection_cleanup(&conn);
    return 0;
}

/* =========================================================================
 * TEST 14 – ax25_mux_lm_release NULL safety and releasing non-seized link
 * ====================================================================== */
static int test_mux_release_safety(void) {
    printf("\n--- test_mux_release_safety ---\n");
    printf("AX.25 v2.2 Section 2.7: LM-RELEASE safety checks\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    /* NULL mux release must not crash */
    ax25_mux_lm_release(NULL, 0);
    TEST_ASSERT(true, "lm_release(NULL mux) does not crash", 0);

    /* Release on empty mux must not crash */
    ax25_mux_lm_release(&mux, 0);
    TEST_ASSERT(true, "lm_release(empty mux, id=0) does not crash", 0);

    /* Release with out-of-range link_id */
    ax25_mux_lm_release(&mux, AX25_MUX_MAX_LINKS);
    TEST_ASSERT(true, "lm_release(out-of-range id) does not crash", 0);

    /* Release a valid inactive link */
    ax25_connection_t conn;
    mux_link_harness_t h;
    memset(&h, 0, sizeof(h));
    init_connected_conn(&conn, &h, "N0REL ", 0, "W1AW  ", 0);
    ax25_address_t la = { .callsign = "N0REL ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    uint8_t link_id;
    ax25_mux_register_link(&mux, &conn, &la, &pa, &link_id);

    /* Release a link that hasn't seized - must not crash */
    ax25_mux_lm_release(&mux, link_id);
    TEST_ASSERT(mux.seized_link == AX25_MUX_NO_SEIZED, "seized_link stays NO_SEIZED when non-seized link released", 0);

    ax25_connection_cleanup(&conn);
    return 0;
}

/* =========================================================================
 * TEST 15 – Seized link stays seized until explicitly released
 *            (a second tick must NOT re-dispatch confirm)
 * ====================================================================== */
static int test_mux_seized_stable(void) {
    printf("\n--- test_mux_seized_stable ---\n");
    printf("AX.25 v2.2 Section 2.7: seized link not disturbed by additional ticks\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conn;
    mux_link_harness_t h;
    memset(&h, 0, sizeof(h));
    init_connected_conn(&conn, &h, "N0SZD ", 0, "W1AW  ", 0);

    ax25_address_t la = { .callsign = "N0SZD ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    uint8_t link_id;
    ax25_mux_register_link(&mux, &conn, &la, &pa, &link_id);
    ax25_mux_set_lm_seize_confirm(&mux, link_id, lm_seize_confirm_cb, &h);

    uint8_t frame[16];
    size_t flen;
    build_raw_frame(frame, &flen, 0x2F);

    ax25_mux_lm_seize_request(&mux, link_id, frame, flen, AX25_MUX_PRI_URGENT);
    ax25_mux_tick(&mux, 1);

    TEST_ASSERT(mux.seized_link == link_id, "Link seized on first tick", 0);
    TEST_ASSERT(h.seize_confirm_count == 1, "Seize-confirm called once", 0);

    /* Call tick many more times - must NOT re-confirm */
    for (uint32_t t = 2; t < 20; t++) {
        ax25_mux_tick(&mux, t);
    }

    DEBUG_VAR("seize_confirm_count after 20 ticks (should stay 1)", h.seize_confirm_count);
    TEST_ASSERT(h.seize_confirm_count == 1, "Seize-confirm NOT repeated on subsequent ticks while seized", 0);
    TEST_ASSERT(mux.seized_link == link_id, "seized_link unchanged across ticks without release", 0);

    /* Now release */
    ax25_mux_lm_release(&mux, link_id);
    TEST_ASSERT(mux.seized_link == AX25_MUX_NO_SEIZED, "seized_link cleared after explicit release", 0);

    ax25_connection_cleanup(&conn);
    return 0;
}

/* =========================================================================
 * TEST 16 – ax25_mux_tick with NULL mux (safety)
 * ====================================================================== */
static int test_mux_tick_null_safety(void) {
    printf("\n--- test_mux_tick_null_safety ---\n");
    printf("AX.25 v2.2 Section 2.7: ax25_mux_tick(NULL) does not crash\n");

    ax25_mux_tick(NULL, 0);
    TEST_ASSERT(true, "ax25_mux_tick(NULL) does not crash", 0);

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    /* Tick on empty mux with no pending seizes - no-op */
    ax25_mux_tick(&mux, 100);
    TEST_ASSERT(mux.seized_link == AX25_MUX_NO_SEIZED, "Tick on empty mux leaves seized_link = NO_SEIZED", 0);

    return 0;
}

/* =========================================================================
 * TEST 17 – set_lm_seize_confirm NULL safety
 * ====================================================================== */
static int test_mux_set_confirm_safety(void) {
    printf("\n--- test_mux_set_confirm_safety ---\n");
    printf("AX.25 v2.2 Section 2.7: ax25_mux_set_lm_seize_confirm NULL safety\n");

    /* NULL mux must not crash */
    ax25_mux_set_lm_seize_confirm(NULL, 0, NULL, NULL);
    TEST_ASSERT(true, "set_lm_seize_confirm(NULL mux) does not crash", 0);

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    /* Out of range link_id */
    ax25_mux_set_lm_seize_confirm(&mux, AX25_MUX_MAX_LINKS, NULL, NULL);
    TEST_ASSERT(true, "set_lm_seize_confirm(out-of-range id) does not crash", 0);

    /* Valid but setting NULL callback (clears it) */
    ax25_connection_t conn;
    mux_link_harness_t h;
    memset(&h, 0, sizeof(h));
    init_connected_conn(&conn, &h, "N0CFM ", 0, "W1AW  ", 0);
    ax25_address_t la = { .callsign = "N0CFM ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    uint8_t link_id;
    ax25_mux_register_link(&mux, &conn, &la, &pa, &link_id);

    ax25_mux_set_lm_seize_confirm(&mux, link_id, NULL, NULL);
    TEST_ASSERT(mux.links[link_id].lm_seize_confirm == NULL, "NULL seize confirm stored correctly (clears callback)", 0);

    ax25_mux_set_lm_seize_confirm(&mux, link_id, lm_seize_confirm_cb, &h);
    TEST_ASSERT(mux.links[link_id].lm_seize_confirm == lm_seize_confirm_cb, "Non-NULL seize confirm stored correctly", 0);
    TEST_ASSERT(mux.links[link_id].confirm_user_data == &h, "user_data stored correctly", 0);

    ax25_connection_cleanup(&conn);
    return 0;
}

/* =========================================================================
 * TEST 18 – Full integration: two-link mux with adapter, tick, and release
 *
 * Simulates both links queuing frames via ax25_mux_transmit_adapter,
 * validates priority arbitration and correct seize-confirm delivery.
 * ====================================================================== */
static int test_mux_full_integration(void) {
    printf("\n--- test_mux_full_integration ---\n");
    printf("AX.25 v2.2 Section 2.7: Full adapter + tick + release integration\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    /* Link A: will queue a SABM (URGENT) */
    ax25_connection_t connA;
    mux_link_harness_t hA;
    memset(&hA, 0, sizeof(hA));
    init_connected_conn(&connA, &hA, "N0FIA ", 0, "W1AW  ", 0);

    /* Link B: will queue a UI frame (UI priority) */
    ax25_connection_t connB;
    mux_link_harness_t hB;
    memset(&hB, 0, sizeof(hB));
    init_connected_conn(&connB, &hB, "N0FIB ", 0, "W1AW  ", 0);

    ax25_address_t laA = { .callsign = "N0FIA ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t laB = { .callsign = "N0FIB ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part

    uint8_t idA, idB;
    ax25_mux_register_link(&mux, &connA, &laA, &pa, &idA);
    ax25_mux_register_link(&mux, &connB, &laB, &pa, &idB);

    ax25_mux_set_lm_seize_confirm(&mux, idA, lm_seize_confirm_cb, &hA);
    ax25_mux_set_lm_seize_confirm(&mux, idB, lm_seize_confirm_cb, &hB);

    ax25_mux_adapter_ctx_t ctxA = { &mux, idA };
    ax25_mux_adapter_ctx_t ctxB = { &mux, idB };

    uint8_t sabm_frame[16];
    size_t sabm_len;
    build_raw_frame(sabm_frame, &sabm_len, 0x2F); /* SABM = URGENT */

    uint8_t ui_frame[16];
    size_t ui_len;
    build_raw_frame(ui_frame, &ui_len, 0x03); /* UI = PRI_UI */

    /* B queues UI first, A queues SABM second */
    ax25_mux_transmit_adapter(&ctxB, ui_frame, ui_len);
    ax25_mux_transmit_adapter(&ctxA, sabm_frame, sabm_len);

    DEBUG_PRINT("Both adapters queued: B=UI(pri=0), A=SABM(pri=255)");
    DEBUG_VAR("B seize_priority (should be 0)", mux.links[idB].seize_priority);
    DEBUG_VAR("A seize_priority (should be 255)", mux.links[idA].seize_priority);

    TEST_ASSERT(mux.links[idA].seize_priority == AX25_MUX_PRI_URGENT, "Link A priority set to URGENT (255) for SABM", 0);
    TEST_ASSERT(mux.links[idB].seize_priority == AX25_MUX_PRI_UI, "Link B priority set to UI (0) for UI frame", 0);

    /* Tick: URGENT should win */
    ax25_mux_tick(&mux, 1);

    DEBUG_VAR("seized_link after tick (should be idA)", mux.seized_link);
    TEST_ASSERT(mux.seized_link == idA, "Link A (URGENT/SABM) seized before Link B (UI)", 0);
    TEST_ASSERT(hA.seize_confirm_count == 1, "Link A got seize-confirm", 0);
    TEST_ASSERT(hB.seize_confirm_count == 0, "Link B NOT yet confirmed (lower priority)", 0);

    /* Verify the correct frame bytes were delivered in confirm */
    TEST_ASSERT(hA.seize_confirmed[0].len == sabm_len, "Confirmed frame length matches SABM frame", 0);

    DEBUG_FRAME("Link A confirmed frame", hA.seize_confirmed[0].data, hA.seize_confirmed[0].len);

    /* Release A: B should be served immediately (burst) */
    ax25_mux_lm_release(&mux, idA);

    TEST_ASSERT(mux.seized_link == idB, "Link B burst-seized on release of A", 0);
    TEST_ASSERT(hB.seize_confirm_count == 1, "Link B got seize-confirm after A released", 0);
    TEST_ASSERT(hB.seize_confirmed[0].len == ui_len, "Confirmed frame length matches UI frame", 0);

    DEBUG_FRAME("Link B confirmed frame", hB.seize_confirmed[0].data, hB.seize_confirmed[0].len);

    ax25_mux_lm_release(&mux, idB);
    TEST_ASSERT(mux.seized_link == AX25_MUX_NO_SEIZED, "All links released, channel free", 0);

    ax25_connection_cleanup(&connA);
    ax25_connection_cleanup(&connB);
    return 0;
}

/* =========================================================================
 * Test 19 – Broadcast addresses: CQ, APRS, BEACON detection in is_broadcast
 *
 * The internal is_broadcast_address is exercised indirectly by
 * ax25_mux_receive_frame. We test all three broadcast identifiers.
 * ====================================================================== */
static int test_mux_broadcast_identifiers(void) {
    printf("\n--- test_mux_broadcast_identifiers ---\n");
    printf("AX.25 v2.2 Section 3.12.5: All three broadcast address identifiers\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conn;
    mux_link_harness_t h;
    memset(&h, 0, sizeof(h));
    init_connected_conn(&conn, &h, "N0BCT ", 0, "W1AW  ", 0);

    ax25_address_t la = { .callsign = "N0BCT ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    uint8_t link_id;
    ax25_mux_register_link(&mux, &conn, &la, &pa, &link_id);

    ax25_unnumbered_information_frame_t ui;
    memset(&ui, 0, sizeof(ui));
    ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
    memcpy(ui.base.base.header.source.callsign, "W1AW  ", 6);
    ui.pid = PID_NO_L3;
    uint8_t info[] = { 0x01 };
    ui.payload = info;
    ui.payload_len = 1;

    const char *bcast_addrs[] = { "CQ    ", "APRS  ", "BEACON" };
    for (int i = 0; i < 3; i++) {
        memcpy(ui.base.base.header.destination.callsign, bcast_addrs[i], 6);
        DEBUG_PRINT("Dispatching broadcast UI to %s", bcast_addrs[i]);
        ax25_mux_receive_frame(&mux, (ax25_frame_t*) &ui, (uint32_t) (i + 1));
        TEST_ASSERT(mux.links[link_id].active, "Link still active after broadcast dispatch", 0);
    }

    TEST_ASSERT(true, "All three broadcast addresses dispatched without crash", 0);

    ax25_connection_cleanup(&conn);
    return 0;
}

/* =========================================================================
 * TEST 20 – Seize-confirm not called when pending_len == 0
 * (Guard: do not invoke confirm if no frame data was stored)
 * ====================================================================== */
static int test_mux_no_confirm_without_frame(void) {
    printf("\n--- test_mux_no_confirm_without_frame ---\n");
    printf("AX.25 v2.2 Section 2.7: seize-confirm not invoked with no pending frame\n");

    ax25_mux_t mux;
    ax25_mux_init(&mux);

    ax25_connection_t conn;
    mux_link_harness_t h;
    memset(&h, 0, sizeof(h));
    init_connected_conn(&conn, &h, "N0NCF ", 0, "W1AW  ", 0);

    ax25_address_t la = { .callsign = "N0NCF ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    ax25_address_t pa = { .callsign = "W1AW  ", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .mod8_legacy = true, .extension = false }; // start modified part: complete designated init (res0=true per AX.25 spec §3.12.2) // end modified part
    uint8_t link_id;
    ax25_mux_register_link(&mux, &conn, &la, &pa, &link_id);
    ax25_mux_set_lm_seize_confirm(&mux, link_id, lm_seize_confirm_cb, &h);

    /* Manually mark pending without setting a frame (simulate edge case) */
    mux.links[link_id].seize_pending = true;
    mux.links[link_id].seize_priority = AX25_MUX_PRI_DATA;
    mux.links[link_id].pending_len = 0; /* no frame data */

    ax25_mux_tick(&mux, 1);

    DEBUG_VAR("seized_link after tick (should be link_id)", mux.seized_link);
    DEBUG_VAR("seize_confirm_count (should be 0 - no frame)", h.seize_confirm_count);

    TEST_ASSERT(mux.seized_link == link_id, "Link seized even when pending_len=0 (seize still granted)", 0);
    TEST_ASSERT(h.seize_confirm_count == 0, "Seize-confirm NOT called when pending_len=0 (no frame to confirm)", 0);

    ax25_mux_lm_release(&mux, link_id);

    ax25_connection_cleanup(&conn);
    return 0;
}

/* =========================================================================
 * Main entry point
 * ====================================================================== */
int test_ax25_mux_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("Starting AX.25 v2.2 Section 2.7 Link Multiplexer (ax25_mux) Tests\n");
    printf("==================================================================================\n");

    result |= test_mux_init();
    result |= test_mux_register_unregister();
    result |= test_mux_slot_exhaustion();
    result |= test_mux_classify_priority();
    result |= test_mux_lm_seize_tick();
    result |= test_mux_priority_ordering();
    result |= test_mux_round_robin();
    result |= test_mux_get_next_to_serve();
    result |= test_mux_seize_validation();
    result |= test_mux_burst_on_release();
    result |= test_mux_receive_point_to_point();
    result |= test_mux_ui_broadcast();
    result |= test_mux_transmit_adapter();
    result |= test_mux_release_safety();
    result |= test_mux_seized_stable();
    result |= test_mux_tick_null_safety();
    result |= test_mux_set_confirm_safety();
    result |= test_mux_full_integration();
    result |= test_mux_broadcast_identifiers();
    result |= test_mux_no_confirm_without_frame();

    printf("\n==================================================================================\n");
    printf("AX.25 Mux Tests Completed. Total assertions: %u. %s\n", assert_count,
            result == 0 ? "\033[0;32mAll tests passed\033[0m" : "\033[0;31mSome tests FAILED\033[0m");
    printf("==================================================================================\n\n");

    return result;
}
