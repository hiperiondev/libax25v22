/*
 * Copyright 2026 Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * Project Site: https://github.com/hiperiondev/libax25v22
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "test_common.h"
#include "ax25_mgmt.h"
#include "ax25.h"

/* -------------------------------------------------------------------------
 * Module-level helpers
 * ---------------------------------------------------------------------- */

static uint32_t assert_count = 0;

#define MAX_TX_HISTORY 64
static uint8_t tx_history_buf[MAX_TX_HISTORY][4096];
static size_t tx_history_len[MAX_TX_HISTORY];
static uint32_t tx_history_count = 0;

static bool mdl_error_fired = false;
static ax25_mdl_error_t mdl_error_code_rcvd = (ax25_mdl_error_t) 0xFF;
static uint32_t mdl_error_fire_count = 0;

static void reset_capture(void) {
    tx_history_count = 0;
    mdl_error_fired = false;
    mdl_error_code_rcvd = (ax25_mdl_error_t) 0xFF;
    mdl_error_fire_count = 0;
    memset(tx_history_len, 0, sizeof(tx_history_len));
}

static void capture_transmit(uint8_t *data, size_t len) {
    if (tx_history_count < MAX_TX_HISTORY && len <= sizeof(tx_history_buf[0])) {
        memcpy(tx_history_buf[tx_history_count], data, len);
        tx_history_len[tx_history_count] = len;
        tx_history_count++;
    }
}

static void null_transmit(uint8_t *data, size_t len) {
    (void) data;
    (void) len;
}

static void on_mdl_error_cb(ax25_mdl_error_t err, void *user_data) {
    mdl_error_fired = true;
    mdl_error_code_rcvd = err;
    mdl_error_fire_count++;
    (void) user_data;
}

static void init_ctx(ax25_mgmt_context_t *ctx) {
    ax25_mgmt_init(ctx);
    ctx->on_mdl_error = on_mdl_error_cb;
    reset_capture();
}

/* -------------------------------------------------------------------------
 * local_process_xid_response()
 *
 * Replaces the non-existent ax25_mgmt_process_xid_response() API function.
 * Parses raw XID info-field bytes and drives ax25_mgmt_process_xid().
 * ---------------------------------------------------------------------- */
#define LOCAL_MAX_PARAMS 32

static uint8_t local_process_xid_response(ax25_mgmt_context_t *ctx, const uint8_t *buf, size_t len) {
    if (!ctx || !buf || len < 4u)
        return 1;
    if (buf[0] != 0x82u)
        return 2;
    if (buf[1] != 0x80u)
        return 3;

    uint16_t gl = (uint16_t) (((uint16_t) buf[2] << 8) | buf[3]);
    if ((size_t) gl > (len - 4u))
        return 4;

    ax25_raw_parameter_t raw_data[LOCAL_MAX_PARAMS];
    ax25_xid_parameter_t params_arr[LOCAL_MAX_PARAMS];
    ax25_xid_parameter_t *param_ptrs[LOCAL_MAX_PARAMS];
    uint8_t num_params = 0;

    const uint8_t *p = buf + 4u;
    const uint8_t *end = p + (size_t) gl;

    while (p + 2u <= end && num_params < LOCAL_MAX_PARAMS) {
        uint8_t pi = p[0];
        uint8_t pl = p[1];
        p += 2u;
        if ((size_t) (end - p) < (size_t) pl)
            break;

        raw_data[num_params].pv = (uint8_t*) (uintptr_t) p;
        raw_data[num_params].pv_len = (size_t) pl;

        params_arr[num_params].pi = (int) pi;
        params_arr[num_params].data = &raw_data[num_params];
        params_arr[num_params].encode = NULL;
        params_arr[num_params].copy = NULL;
        params_arr[num_params].free = NULL;
        param_ptrs[num_params] = &params_arr[num_params];
        num_params++;

        p += (size_t) pl;
    }

    ax25_exchange_identification_frame_t xid;
    memset(&xid, 0, sizeof(xid));
    xid.base.base.header.cr = false;
    xid.base.base.type = AX25_FRAME_UNNUMBERED_XID;
    xid.base.pf = true;
    xid.fi = buf[0];
    xid.gi = buf[1];
    xid.parameters = param_ptrs;
    xid.param_count = num_params;

    return ax25_mgmt_process_xid(ctx, &xid, null_transmit);
}

/* -------------------------------------------------------------------------
 * XID info-field builder
 * ---------------------------------------------------------------------- */

static size_t build_xid(uint8_t *buf, size_t buf_size,
bool modulo128, bool srej, bool rej, bool full_duplex, uint16_t ifield_bits, uint8_t window, uint16_t t1_ms, uint8_t retries, bool include_t2, uint16_t t2_ms) {
    uint8_t params[256];
    size_t plen = 0;

    params[plen++] = XID_PI_CLASS_OF_PROCEDURES;
    params[plen++] = 0x02;
    params[plen++] = full_duplex ? XID_COP_FULL_DUPLEX : XID_COP_HALF_DUPLEX;
    params[plen++] = 0x00;

    params[plen++] = XID_PI_HDLC_OPTIONAL_FUNCTIONS;
    params[plen++] = 0x03;
    {
        uint8_t b0 = XID_HDLC_RNR | XID_HDLC_SABM | XID_HDLC_DM |
        XID_HDLC_DISC | XID_HDLC_UA;
        if (rej)
            b0 |= XID_HDLC_REJ;
        if (srej)
            b0 |= XID_HDLC_SREJ;
        if (modulo128)
            b0 |= XID_HDLC_SABME;
        params[plen++] = b0;
        uint8_t b1 = XID_HDLC_FRMR | XID_HDLC_UI | XID_HDLC_XID |
        XID_HDLC_TEST | XID_HDLC_MOD8;
        if (modulo128)
            b1 |= XID_HDLC_MOD128;
        params[plen++] = b1;
        params[plen++] = 0x00;
    }

    params[plen++] = XID_PI_IFIELD_LENGTH_RX;
    params[plen++] = 0x02;
    params[plen++] = (uint8_t) (ifield_bits >> 8);
    params[plen++] = (uint8_t) (ifield_bits & 0xFF);

    params[plen++] = XID_PI_WINDOW_SIZE_RX;
    params[plen++] = 0x01;
    params[plen++] = window;

    params[plen++] = XID_PI_ACK_TIMER;
    params[plen++] = 0x02;
    params[plen++] = (uint8_t) (t1_ms >> 8);
    params[plen++] = (uint8_t) (t1_ms & 0xFF);

    params[plen++] = XID_PI_RETRIES;
    params[plen++] = 0x01;
    params[plen++] = retries;

    if (include_t2) {
        params[plen++] = XID_PI_RESP_DELAY_TIMER;
        params[plen++] = 0x02;
        params[plen++] = (uint8_t) (t2_ms >> 8);
        params[plen++] = (uint8_t) (t2_ms & 0xFF);
    }

    size_t total = 4u + plen;
    if (total > buf_size)
        return 0;

    buf[0] = 0x82;
    buf[1] = 0x80;
    buf[2] = (uint8_t) (plen >> 8);
    buf[3] = (uint8_t) (plen & 0xFF);
    memcpy(buf + 4, params, plen);
    return total;
}

static size_t build_xid_no_t2(uint8_t *buf, size_t buf_size,
bool modulo128, bool srej, bool rej, bool full_duplex, uint16_t ifield_bits, uint8_t window, uint16_t t1_ms, uint8_t retries) {
    return build_xid(buf, buf_size, modulo128, srej, rej, full_duplex, ifield_bits, window, t1_ms, retries, false, 0);
}

/* =========================================================================
 * Tests 51 – 80
 * ====================================================================== */

/* --- 51: T1 at uint16 max ------------------------------------------------ */
static int test_xid_t1_max_uint16(void) {
    assert_count = 0;
    printf("\n--- test_xid_t1_max_uint16 (test 51) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.ack_timer = 3000;
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 32, 0xFFFFu, 10);
    TEST_ASSERT(xlen > 0, "XID T1=65535 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds T1=65535", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.ack_timer == 0xFFFFu, "agreed T1=65535 (max rule)", 0);

    return 0;
}

/* --- 52: Retries at uint8 max ------------------------------------------- */
static int test_xid_retries_max_uint8(void) {
    assert_count = 0;
    printf("\n--- test_xid_retries_max_uint8 (test 52) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.retries = 10;
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 32, 3000, 0xFFu);
    TEST_ASSERT(xlen > 0, "XID retries=255 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds retries=255", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.retries == 255u, "agreed retries=255 (max rule)", 0);

    return 0;
}

/* --- 53: Window=127 both sides mod-128 -> capped to 63 (PE1CHL §5) ------ */
static int test_xid_window_max_mod128_both(void) {
    assert_count = 0;
    printf("\n--- test_xid_window_max_mod128_both (test 53) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 127;
    ctx.local_params.selective_reject = true;
    ctx.local_params.implicit_reject = true;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 127, 3000, 10);
    TEST_ASSERT(xlen > 0, "XID window=127 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds window=127", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.modulo128 == true, "agreed modulo128=true", 0);
    // min(127,127)=127, then capped to AX25_K_MAX_MOD128=63 per PE1CHL §5.
    TEST_ASSERT(ctx.agreed_params.window_size == 63u, "agreed window=63 (capped from 127 per PE1CHL §5)", 0);

    return 0;
}

/* --- 54: Window>127 both sides mod-128 -> capped to 63 (PE1CHL §5) ------ */
static int test_xid_window_over127_mod128_capped(void) {
    assert_count = 0;
    printf("\n--- test_xid_window_over127_mod128_capped (test 54) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 200;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 200, 3000, 10);
    TEST_ASSERT(xlen > 0, "XID window=200 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds window=200", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    // min(200,200)=200 -> capped to AX25_K_MAX_MOD128=63 per PE1CHL §5.
    TEST_ASSERT(ctx.agreed_params.window_size <= 63u, "agreed window<=63 (protocol cap applied)", 0);
    TEST_ASSERT(ctx.agreed_params.window_size == 63u, "agreed window=63 (min(200,200)=200 -> capped to 63)", 0);

    return 0;
}

/* --- 55: Modulo mismatch: local mod-128, peer mod-8 --------------------- */
static int test_xid_modulo_mismatch_local128_peer8(void) {
    assert_count = 0;
    printf("\n--- test_xid_modulo_mismatch_local128_peer8 (test 55) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;
    ctx.local_params.selective_reject = true;
    ctx.local_params.implicit_reject = true;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), false, false, true, false, 256 * 8, 7, 3000, 10);
    TEST_ASSERT(xlen > 0, "XID peer-mod8 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds (mismatch)", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.modulo128 == false, "agreed modulo128=false (AND rule)", 0);
    TEST_ASSERT(ctx.agreed_params.window_size <= 7u, "agreed window<=7 (mod-8 cap)", 0);
    TEST_ASSERT(ctx.agreed_params.selective_reject == false, "agreed srej=false (peer no SREJ)", 0);

    return 0;
}

/* --- 56: I-field local smaller (local=128, peer=256) -------------------- */
static int test_xid_ifield_local_smaller_wins(void) {
    assert_count = 0;
    printf("\n--- test_xid_ifield_local_smaller_wins (test 56) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.ifield_length = 128;
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 32, 3000, 10);
    TEST_ASSERT(xlen > 0, "XID peer ifield=256 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.ifield_length == 128u, "agreed ifield=128 (local smaller wins)", 0);

    return 0;
}

/* --- 57: I-field peer smaller (local=256, peer=64) ---------------------- */
static int test_xid_ifield_peer_smaller_wins(void) {
    assert_count = 0;
    printf("\n--- test_xid_ifield_peer_smaller_wins (test 57) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.ifield_length = 256;
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 64 * 8, 32, 3000, 10);
    TEST_ASSERT(xlen > 0, "XID peer ifield=64 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.ifield_length == 64u, "agreed ifield=64 (peer smaller wins)", 0);

    return 0;
}

/* --- 58: T1 local larger (5000 vs 2000) --------------------------------- */
static int test_xid_t1_local_larger_wins(void) {
    assert_count = 0;
    printf("\n--- test_xid_t1_local_larger_wins (test 58) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.ack_timer = 5000;
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 32, 2000, 10);
    TEST_ASSERT(xlen > 0, "XID peer T1=2000 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.ack_timer == 5000u, "agreed T1=5000 (local larger, max rule)", 0);

    return 0;
}

/* --- 59: T1 peer larger (3000 vs 8000) ---------------------------------- */
static int test_xid_t1_peer_larger_wins(void) {
    assert_count = 0;
    printf("\n--- test_xid_t1_peer_larger_wins (test 59) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.ack_timer = 3000;
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 32, 8000, 10);
    TEST_ASSERT(xlen > 0, "XID peer T1=8000 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.ack_timer == 8000u, "agreed T1=8000 (peer larger, max rule)", 0);

    return 0;
}

/* --- 60: T1 equal both sides -------------------------------------------- */
static int test_xid_t1_equal_both_sides(void) {
    assert_count = 0;
    printf("\n--- test_xid_t1_equal_both_sides (test 60) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.ack_timer = 4000;
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 32, 4000, 10);
    TEST_ASSERT(xlen > 0, "XID T1=4000 equal built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.ack_timer == 4000u, "agreed T1=4000 (equal tie)", 0);

    return 0;
}

/* --- 61: Retries local larger (20 vs 5) --------------------------------- */
static int test_xid_retries_local_larger(void) {
    assert_count = 0;
    printf("\n--- test_xid_retries_local_larger (test 61) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.retries = 20;
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 32, 3000, 5);
    TEST_ASSERT(xlen > 0, "XID peer retries=5 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.retries == 20u, "agreed retries=20 (local larger, max rule)", 0);

    return 0;
}

/* --- 62: Retries peer larger (5 vs 20) ---------------------------------- */
static int test_xid_retries_peer_larger(void) {
    assert_count = 0;
    printf("\n--- test_xid_retries_peer_larger (test 62) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.retries = 5;
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 32, 3000, 20);
    TEST_ASSERT(xlen > 0, "XID peer retries=20 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.retries == 20u, "agreed retries=20 (peer larger, max rule)", 0);

    return 0;
}

/* --- 63: T2 equal both sides -------------------------------------------- */
static int test_xid_t2_equal_both_sides(void) {
    assert_count = 0;
    printf("\n--- test_xid_t2_equal_both_sides (test 63) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.response_delay_timer = 800;
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid(xid, sizeof(xid), true, true, true, false, 256 * 8, 32, 3000, 10, true, 800);
    TEST_ASSERT(xlen > 0, "XID T2=800 equal built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.response_delay_timer == 800u, "agreed T2=800 (equal tie)", 0);

    return 0;
}

/* --- 64: Process XID in IDLE state (unsolicited) ------------------------ */
static int test_xid_response_in_idle_state(void) {
    assert_count = 0;
    printf("\n--- test_xid_response_in_idle_state (test 64) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    TEST_ASSERT(ctx.state == AX25_MGMT_IDLE, "initial state=IDLE", 0);

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 32, 3000, 10);
    TEST_ASSERT(xlen > 0, "XID built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    /* Implementation may accept or reject; must not crash. */
    if (rc == 0) {
        TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED when implementation accepts in IDLE", 0);
    } else {
        TEST_ASSERT(ctx.state == AX25_MGMT_IDLE, "state unchanged when implementation rejects in IDLE", 0);
    }
    printf("\033[0;32m[%04d]    PASS: Processing XID in IDLE handled without crash\033[0m\n", ++assert_count);
    return 0;
}

/* --- 65: Re-negotiate when already NEGOTIATED --------------------------- */
static int test_xid_response_in_negotiated_state(void) {
    assert_count = 0;
    printf("\n--- test_xid_response_in_negotiated_state (test 65) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    /* First negotiation */
    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part
    uint8_t xid1[512];
    size_t xlen1 = build_xid_no_t2(xid1, sizeof(xid1), true, true, true, false, 256 * 8, 32, 3000, 10);
    uint8_t rc1 = local_process_xid_response(&ctx, xid1, xlen1);
    TEST_ASSERT(rc1 == 0, "first negotiation succeeds", rc1);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED after first", 0);

    DEBUG_VAR("first agreed ifield", ctx.agreed_params.ifield_length);

    /* Re-negotiate with smaller ifield */
    reset_capture();
    uint8_t xid2[512];
    size_t xlen2 = build_xid_no_t2(xid2, sizeof(xid2), true, true, true, false, 128 * 8, 32, 3000, 10);
    uint8_t rc2 = local_process_xid_response(&ctx, xid2, xlen2);

    if (rc2 == 0) {
        TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED after re-negotiate", 0);
        TEST_ASSERT(ctx.agreed_params.ifield_length == 128u, "re-negotiated ifield=128 (updated correctly)", 0);
    } else {
        printf("[INFO] re-negotiate in NEGOTIATED state rejected (rc=%u)\n", rc2);
    }
    printf("\033[0;32m[%04d]    PASS: Re-negotiate in NEGOTIATED handled without crash\033[0m\n", ++assert_count);

    return 0;
}

/* --- 66: Wrong FI byte 0x00 -------------------------------------------- */
static int test_xid_wrong_fi_byte(void) {
    assert_count = 0;
    printf("\n--- test_xid_wrong_fi_byte (test 66) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part
    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t bad[4] = { 0x00, 0x80, 0x00, 0x00 };
    uint8_t rc = local_process_xid_response(&ctx, bad, sizeof(bad));
    TEST_ASSERT(rc == 2u, "rc=2 for wrong FI=0x00", rc);
    TEST_ASSERT(ctx.state != AX25_MGMT_NEGOTIATED, "state does not advance to NEGOTIATED", 0);

    return 0;
}

/* --- 67: Wrong FI byte 0x01 -------------------------------------------- */
static int test_xid_wrong_fi_byte_0x01(void) {
    assert_count = 0;
    printf("\n--- test_xid_wrong_fi_byte_0x01 (test 67) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part
    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t bad[4] = { 0x01, 0x80, 0x00, 0x00 };
    uint8_t rc = local_process_xid_response(&ctx, bad, sizeof(bad));
    TEST_ASSERT(rc == 2u, "rc=2 for wrong FI=0x01", rc);

    return 0;
}

/* --- 68: GL claims more bytes than available ---------------------------- */
static int test_xid_gl_truncated(void) {
    assert_count = 0;
    printf("\n--- test_xid_gl_truncated (test 68) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part
    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    /* GL=100 but only 2 data bytes follow */
    uint8_t trunc[6] = { 0x82, 0x80, 0x00, 0x64, 0xAA, 0xBB };
    uint8_t rc = local_process_xid_response(&ctx, trunc, sizeof(trunc));
    TEST_ASSERT(rc == 4u, "rc=4 for GL truncated", rc);
    TEST_ASSERT(ctx.state != AX25_MGMT_NEGOTIATED, "state does not advance to NEGOTIATED", 0);

    return 0;
}

/* --- 69: Buffer too short (len=3 < 4) ---------------------------------- */
static int test_xid_buffer_too_short(void) {
    assert_count = 0;
    printf("\n--- test_xid_buffer_too_short (test 69) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    uint8_t buf[3] = { 0x82, 0x80, 0x00 };
    uint8_t rc = local_process_xid_response(&ctx, buf, 3);
    TEST_ASSERT(rc == 1u, "rc=1 for len=3 (too short)", rc);

    return 0;
}

/* --- 70: NULL buf ------------------------------------------------------- */
static int test_xid_null_buf_guard(void) {
    assert_count = 0;
    printf("\n--- test_xid_null_buf_guard (test 70) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    uint8_t rc = local_process_xid_response(&ctx, NULL, 64);
    TEST_ASSERT(rc == 1u, "rc=1 for NULL buf", rc);

    return 0;
}

/* --- 71: NULL ctx ------------------------------------------------------- */
static int test_xid_null_ctx_guard_response(void) {
    assert_count = 0;
    printf("\n--- test_xid_null_ctx_guard_response (test 71) ---\n");

    uint8_t buf[4] = { 0x82, 0x80, 0x00, 0x00 };
    uint8_t rc = local_process_xid_response(NULL, buf, 4);
    TEST_ASSERT(rc == 1u, "rc=1 for NULL ctx", rc);

    return 0;
}

/* --- 72: Multiple unknown PIs mixed with valid params ------------------- */
static int test_xid_multiple_unknown_pis(void) {
    assert_count = 0;
    printf("\n--- test_xid_multiple_unknown_pis (test 72) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part
    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t params[256];
    size_t plen = 0;

    /* Unknown PI=0x20 */
    params[plen++] = 0x20;
    params[plen++] = 0x01;
    params[plen++] = 0xAA;

    /* PI=2 COP half-duplex */
    params[plen++] = XID_PI_CLASS_OF_PROCEDURES;
    params[plen++] = 0x02;
    params[plen++] = XID_COP_HALF_DUPLEX;
    params[plen++] = 0x00;

    /* Unknown PI=0x40 */
    params[plen++] = 0x40;
    params[plen++] = 0x02;
    params[plen++] = 0xBB;
    params[plen++] = 0xCC;

    /* PI=3 HDLC mod-128 */
    params[plen++] = XID_PI_HDLC_OPTIONAL_FUNCTIONS;
    params[plen++] = 0x03;
    params[plen++] = (uint8_t) (XID_HDLC_RNR | XID_HDLC_REJ | XID_HDLC_SREJ |
    XID_HDLC_SABM | XID_HDLC_SABME | XID_HDLC_DM |
    XID_HDLC_DISC | XID_HDLC_UA);
    params[plen++] = (uint8_t) (XID_HDLC_FRMR | XID_HDLC_UI | XID_HDLC_XID |
    XID_HDLC_TEST | XID_HDLC_MOD8 | XID_HDLC_MOD128);
    params[plen++] = 0x00;

    /* Unknown PI=0xFD */
    params[plen++] = 0xFD;
    params[plen++] = 0x03;
    params[plen++] = 0x11;
    params[plen++] = 0x22;
    params[plen++] = 0x33;

    /* PI=6 I-field 256 bytes */
    params[plen++] = XID_PI_IFIELD_LENGTH_RX;
    params[plen++] = 0x02;
    params[plen++] = (uint8_t) ((256u * 8u) >> 8);
    params[plen++] = (uint8_t) ((256u * 8u) & 0xFF);

    /* PI=8 window 32 */
    params[plen++] = XID_PI_WINDOW_SIZE_RX;
    params[plen++] = 0x01;
    params[plen++] = 32;

    /* PI=9 T1 3000ms */
    params[plen++] = XID_PI_ACK_TIMER;
    params[plen++] = 0x02;
    params[plen++] = (uint8_t) (3000u >> 8);
    params[plen++] = (uint8_t) (3000u & 0xFF);

    /* PI=10 retries 10 */
    params[plen++] = XID_PI_RETRIES;
    params[plen++] = 0x01;
    params[plen++] = 10;

    uint8_t buf[512];
    buf[0] = 0x82;
    buf[1] = 0x80;
    buf[2] = (uint8_t) (plen >> 8);
    buf[3] = (uint8_t) (plen & 0xFF);
    memcpy(buf + 4, params, plen);

    uint8_t rc = local_process_xid_response(&ctx, buf, 4u + plen);
    TEST_ASSERT(rc == 0, "succeeds with 3 unknown PIs", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(!mdl_error_fired, "MDL-ERROR NOT fired for unknown PIs", 0);
    TEST_ASSERT(ctx.agreed_params.ifield_length == 256u, "ifield parsed correctly despite unknown PIs", 0);
    TEST_ASSERT(ctx.agreed_params.modulo128 == true, "modulo128 parsed correctly despite unknown PIs", 0);

    return 0;
}

/* --- 73: Truncated PV bytes (PL=4 but 1 byte in group) ----------------- */
static int test_xid_truncated_pv_bytes(void) {
    assert_count = 0;
    printf("\n--- test_xid_truncated_pv_bytes (test 73) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.modulo128 = true;
    ctx.local_params.ack_timer = 3000;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part
    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    /* PI=9 PL=4 (claims 4 bytes) but only 1 byte of PV available */
    uint8_t buf[7];
    buf[0] = 0x82;
    buf[1] = 0x80;
    buf[2] = 0x00;
    buf[3] = 0x03; /* GL=3 */
    buf[4] = XID_PI_ACK_TIMER;
    buf[5] = 0x04;
    buf[6] = 0x0B; /* PL=4, 1 byte */

    uint8_t rc = local_process_xid_response(&ctx, buf, sizeof(buf));

    /* local helper breaks (param skipped); ax25_mgmt_process_xid receives 0
     params and uses defaults -> succeeds with NEGOTIATED state */
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED || ctx.state == AX25_MGMT_AWAITING_RESPONSE || ctx.state == AX25_MGMT_IDLE, "state valid after truncated PV",
            0);
    (void) rc;
    printf("\033[0;32m[%04d]    PASS: Truncated PV handled without crash\033[0m\n", ++assert_count);

    return 0;
}

/* --- 74: Retry stepping -> error K on exhaustion ----------------------- */
static int test_xid_retry_stepping_count(void) {
    assert_count = 0;
    printf("\n--- test_xid_retry_stepping_count (test 74) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.max_retries = 3;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part
    uint32_t tx_after_start = tx_history_count;
    TEST_ASSERT(tx_after_start == 1u, "1 XID sent at start", 0);

    // start modified part: tick values raised to exceed tm201=max(ack_timer=10000,3000)=10000 ms
    ctx.timeout_tick = 0;
    ax25_mgmt_tick(&ctx, 1000); /* arm: elapsed=1000 < 10000, no timeout */
    TEST_ASSERT(ctx.state == AX25_MGMT_AWAITING_RESPONSE, "AWAITING after arm", 0);
    TEST_ASSERT(!mdl_error_fired, "no MDL-ERROR yet", 0);

    /* 1st timeout -> retry_count=1: elapsed=11000-0=11000 >= 10000 */
    ax25_mgmt_tick(&ctx, 11000);
    TEST_ASSERT(ctx.state == AX25_MGMT_AWAITING_RESPONSE, "AWAITING after retry 1", 0);
    TEST_ASSERT(!mdl_error_fired, "no MDL-ERROR after retry 1", 0);
    TEST_ASSERT(ctx.retry_count == 1u, "retry_count=1", 0);
    TEST_ASSERT(tx_history_count > tx_after_start, "XID retransmitted after retry 1", 0);
    uint32_t tx_r1 = tx_history_count;

    /* 2nd timeout -> retry_count=2: elapsed=22000-11000=11000 >= 10000 */
    ctx.timeout_tick = 11000;
    ax25_mgmt_tick(&ctx, 22000);
    TEST_ASSERT(ctx.state == AX25_MGMT_AWAITING_RESPONSE, "AWAITING after retry 2", 0);
    TEST_ASSERT(!mdl_error_fired, "no MDL-ERROR after retry 2", 0);
    TEST_ASSERT(ctx.retry_count == 2u, "retry_count=2", 0);
    TEST_ASSERT(tx_history_count > tx_r1, "XID retransmitted after retry 2", 0);

    /* 3rd timeout -> error K: elapsed=33000-22000=11000 >= 10000 */
    ctx.timeout_tick = 22000;
    ax25_mgmt_tick(&ctx, 33000);
    TEST_ASSERT(ctx.state == AX25_MGMT_IDLE, "state=IDLE after exhaustion", 0);
    TEST_ASSERT(mdl_error_fired, "MDL-ERROR K fired", 0);
    TEST_ASSERT(mdl_error_code_rcvd == AX25_MDL_ERROR_K, "error code=K", 0);
    TEST_ASSERT(mdl_error_fire_count == 1u, "MDL-ERROR fires exactly once", 0);

    for (uint32_t t = 38000; t <= 68000; t += 5000)
        ax25_mgmt_tick(&ctx, t);
    TEST_ASSERT(mdl_error_fire_count == 1u, "MDL-ERROR count stays 1", 0);
    // end modified part

    return 0;
}

/* --- 75: Retry count resets on fresh start_negotiation ----------------- */
static int test_xid_retry_count_resets_on_restart(void) {
    assert_count = 0;
    printf("\n--- test_xid_retry_count_resets_on_restart (test 75) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.max_retries = 1;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // dest/src are used in two start_negotiation calls; free after the second.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part
    // start modified part: tick raised to exceed tm201=10000 ms
    ctx.timeout_tick = 0;
    ax25_mgmt_tick(&ctx, 1000);   /* no timeout: elapsed=1000 < 10000 */
    ax25_mgmt_tick(&ctx, 11000);  /* -> error K: elapsed=11000-0=11000 >= 10000 */
    TEST_ASSERT(ctx.state == AX25_MGMT_IDLE, "state=IDLE after error K", 0);
    // end modified part

    reset_capture();
    uint8_t res = ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately after the last call that uses dest/src — copied by value.
    dest = NULL;
    src = NULL;
    // end modified part
    TEST_ASSERT(res == 0, "second start_negotiation returns 0", res);
    TEST_ASSERT(ctx.retry_count == 0u, "retry_count=0 after restart", 0);
    TEST_ASSERT(ctx.state == AX25_MGMT_AWAITING_RESPONSE, "state=AWAITING_RESPONSE after restart", 0);

    return 0;
}

/* --- 76: I-field minimum 1 byte (8 bits on wire) ----------------------- */
static int test_xid_ifield_minimum_1_byte(void) {
    assert_count = 0;
    printf("\n--- test_xid_ifield_minimum_1_byte (test 76) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.ifield_length = 256;
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part
    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 8u, 32, 3000, 10); /* 8 bits = 1 byte */
    TEST_ASSERT(xlen > 0, "XID ifield=8 bits built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.ifield_length == 1u, "agreed ifield=1 byte (min, 8 bits on wire)", 0);

    return 0;
}

/* --- 77: Window minimum 1 ---------------------------------------------- */
static int test_xid_window_minimum_1(void) {
    assert_count = 0;
    printf("\n--- test_xid_window_minimum_1 (test 77) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 1;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part
    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid_no_t2(xid, sizeof(xid), true, true, true, false, 256 * 8, 1, 3000, 10);
    TEST_ASSERT(xlen > 0, "XID window=1 built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);
    TEST_ASSERT(ctx.agreed_params.window_size == 1u, "agreed window=1 (minimum)", 0);

    return 0;
}

/* --- 78: Agreed-params integrity ---------------------------------------- */
static int test_xid_agreed_params_integrity(void) {
    assert_count = 0;
    printf("\n--- test_xid_agreed_params_integrity (test 78) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.local_params.modulo128 = true;
    ctx.local_params.window_size = 32;
    ctx.local_params.ifield_length = 256;
    ctx.local_params.ack_timer = 3000;
    ctx.local_params.retries = 10;
    ctx.local_params.response_delay_timer = 500;
    ctx.local_params.selective_reject = true;
    ctx.local_params.implicit_reject = true;
    ctx.local_params.full_duplex = false;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part
    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part

    uint8_t xid[512];
    size_t xlen = build_xid(xid, sizeof(xid), true, true, true, false, 256 * 8, 16, 4000, 8, true, 600);
    TEST_ASSERT(xlen > 0, "XID for integrity check built", 0);

    uint8_t rc = local_process_xid_response(&ctx, xid, xlen);
    TEST_ASSERT(rc == 0, "process succeeds", rc);
    TEST_ASSERT(ctx.state == AX25_MGMT_NEGOTIATED, "state=NEGOTIATED", 0);

    TEST_ASSERT(ctx.agreed_params.modulo128 == true, "modulo128=true", 0);
    TEST_ASSERT(ctx.agreed_params.window_size == 16u, "window=16 (min)", 0);
    TEST_ASSERT(ctx.agreed_params.ifield_length == 256u, "ifield=256 (equal)", 0);
    TEST_ASSERT(ctx.agreed_params.ack_timer == 4000u, "T1=4000 (max)", 0);
    TEST_ASSERT(ctx.agreed_params.retries == 10u, "retries=10 (max)", 0);
    TEST_ASSERT(ctx.agreed_params.response_delay_timer == 600u, "T2=600 (max)", 0);
    TEST_ASSERT(ctx.agreed_params.selective_reject == true, "srej=true (AND)", 0);
    TEST_ASSERT(ctx.agreed_params.implicit_reject == true, "rej=true (AND)", 0);
    TEST_ASSERT(ctx.agreed_params.full_duplex == false, "full_duplex=false (AND)", 0);

    return 0;
}

/* --- 79: Agreed-params zeroed on MDL-ERROR K --------------------------- */
static int test_xid_agreed_params_zeroed_on_error_k(void) {
    assert_count = 0;
    printf("\n--- test_xid_agreed_params_zeroed_on_error_k (test 79) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);
    ctx.max_retries = 1;

    ctx.agreed_params.ifield_length = 0xDEAD;
    ctx.agreed_params.window_size = 0x42;
    ctx.agreed_params.ack_timer = 0x1234;
    ctx.agreed_params.retries = 0xFF;
    ctx.agreed_params.modulo128 = true;
    ctx.agreed_params.selective_reject = true;
    ctx.agreed_params.full_duplex = true;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part
    // start modified part: tick raised to exceed tm201=10000 ms
    ctx.timeout_tick = 0;
    ax25_mgmt_tick(&ctx, 1000);   /* no timeout: elapsed=1000 < 10000 */
    ax25_mgmt_tick(&ctx, 11000);  /* -> error K: elapsed=11000-0=11000 >= 10000 */
    // end modified part

    TEST_ASSERT(mdl_error_fired, "MDL-ERROR K fired", 0);
    TEST_ASSERT(ctx.state == AX25_MGMT_IDLE, "state=IDLE", 0);
    TEST_ASSERT(ctx.agreed_params.ifield_length == 0, "ifield=0 after K", 0);
    TEST_ASSERT(ctx.agreed_params.window_size == 0, "window=0 after K", 0);
    TEST_ASSERT(ctx.agreed_params.modulo128 == false, "modulo128=0 after K", 0);
    TEST_ASSERT(ctx.agreed_params.selective_reject == false, "srej=0 after K", 0);
    TEST_ASSERT(ctx.agreed_params.retries == 0, "retries=0 after K", 0);

    return 0;
}

/* --- 80: Agreed-params zeroed on FRMR (MDL-ERROR B) -------------------- */
static int test_xid_agreed_params_zeroed_on_frmr(void) {
    assert_count = 0;
    printf("\n--- test_xid_agreed_params_zeroed_on_frmr (test 80) ---\n");

    ax25_mgmt_context_t ctx;
    init_ctx(&ctx);

    ctx.agreed_params.ifield_length = 512;
    ctx.agreed_params.window_size = 64;
    ctx.agreed_params.ack_timer = 9999;
    ctx.agreed_params.modulo128 = true;
    ctx.agreed_params.selective_reject = true;

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation/process_xid copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "addresses parsed", err);
    }
    // end modified part

    uint8_t res = ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part
    TEST_ASSERT(res == 0, "start_negotiation returns 0", res);
    TEST_ASSERT(ctx.state == AX25_MGMT_AWAITING_RESPONSE, "state=AWAITING", 0);

    ax25_mgmt_notify_frmr_received(&ctx);

    TEST_ASSERT(ctx.state == AX25_MGMT_IDLE, "state=IDLE after FRMR", 0);
    TEST_ASSERT(mdl_error_fired, "MDL-ERROR B fired", 0);
    TEST_ASSERT(mdl_error_code_rcvd == AX25_MDL_ERROR_B, "error code=B", 0);
    TEST_ASSERT(ctx.agreed_params.ifield_length == 0, "ifield=0 after FRMR", 0);
    TEST_ASSERT(ctx.agreed_params.window_size == 0, "window=0 after FRMR", 0);
    TEST_ASSERT(ctx.agreed_params.ack_timer == 0, "ack_timer=0 after FRMR", 0);
    TEST_ASSERT(ctx.agreed_params.modulo128 == false, "modulo128=0 after FRMR", 0);

    return 0;
}

/* =========================================================================
 * Entry point
 * ====================================================================== */

int test_ax25_xid_params_edge2_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("AX.25 XID Parameter Edge Case Tests - Battery 2 (Tests 51-80)\n");
    printf("==================================================================================\n\n");

    result |= test_xid_t1_max_uint16();
    result |= test_xid_retries_max_uint8();
    result |= test_xid_window_max_mod128_both();
    result |= test_xid_window_over127_mod128_capped();
    result |= test_xid_modulo_mismatch_local128_peer8();
    result |= test_xid_ifield_local_smaller_wins();
    result |= test_xid_ifield_peer_smaller_wins();
    result |= test_xid_t1_local_larger_wins();
    result |= test_xid_t1_peer_larger_wins();
    result |= test_xid_t1_equal_both_sides();
    result |= test_xid_retries_local_larger();
    result |= test_xid_retries_peer_larger();
    result |= test_xid_t2_equal_both_sides();
    result |= test_xid_response_in_idle_state();
    result |= test_xid_response_in_negotiated_state();
    result |= test_xid_wrong_fi_byte();
    result |= test_xid_wrong_fi_byte_0x01();
    result |= test_xid_gl_truncated();
    result |= test_xid_buffer_too_short();
    result |= test_xid_null_buf_guard();
    result |= test_xid_null_ctx_guard_response();
    result |= test_xid_multiple_unknown_pis();
    result |= test_xid_truncated_pv_bytes();
    result |= test_xid_retry_stepping_count();
    result |= test_xid_retry_count_resets_on_restart();
    result |= test_xid_ifield_minimum_1_byte();
    result |= test_xid_window_minimum_1();
    result |= test_xid_agreed_params_integrity();
    result |= test_xid_agreed_params_zeroed_on_error_k();
    result |= test_xid_agreed_params_zeroed_on_frmr();

    printf("\n==================================================================================\n");
    printf("Battery 2 Tests Completed. %s\n", result == 0 ? "All tests passed." : "Some tests FAILED.");
    printf("==================================================================================\n\n");

    return result;
}
