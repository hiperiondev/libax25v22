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
 * ============================================================================
 * Section 6 Data Link State Machine - Advanced / Previously Untested Areas
 * ============================================================================
 * Covers:
 *   A) Simultaneous Bidirectional Connection Attempts (Section 6.3.1 collision)
 *   B) Collision Recovery Scenarios (UA F=0, DM, DISC during AWAITING_CONNECTION)
 *   C) Timer Expiration Edge Cases (T1 boundary, N2 exhaustion per state)
 *   D) State Transition Validation (DL-ERROR codes, FRMR, cross-state frames)
 *
 * References:
 *   AX.25 v2.2 Sections 4.4.5, 6.3, 6.4, 6.7; Appendix C4 SDL diagrams.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "test_common.h"
#include "ax25_state_machine.h"
#include "ax25.h"

// ============================================================================
// Test-local assert counter (separate namespace from other test files)
// ============================================================================
static uint32_t assert_count = 0;

// ============================================================================
// Transmit capture state
// ============================================================================
static uint8_t captured_buffer[2048];
static size_t captured_len = 0;
static uint32_t transmit_count = 0;

// Last control byte extracted from captured frame (offset 14 for simple frames)
static uint8_t last_ctrl_byte = 0;

// ============================================================================
// Callback state capture structure
// Records what the state machine told the upper layer
// ============================================================================
typedef struct {
    bool connect_called;
    bool connect_initiated_locally;
    uint32_t connect_call_count;

    bool disconnect_called;
    uint8_t disconnect_reason;
    uint32_t disconnect_call_count;

    bool error_called;
    ax25_dl_error_t last_error;
    uint32_t error_call_count;

    bool busy_called;
    bool busy_state;

    bool data_received;
    uint8_t last_pid;
    uint8_t last_payload[256];
    size_t last_payload_len;
} test_ctx_t;

// ============================================================================
// Hardcoded callsign bytes (AX.25 on-air: ASCII shifted left 1 bit, space-padded)
// ============================================================================
static const uint8_t TEST1_CALL[6] = { 0xA8, 0x8A, 0xA6, 0xA8, 0x62, 0x40 };  // "TEST1 "
static const uint8_t TEST2_CALL[6] = { 0xA8, 0x8A, 0xA6, 0xA8, 0x64, 0x40 };  // "TEST2 "

// ============================================================================
// Helper macros for building raw frame bytes (modulo-8, no digipeaters)
//   dest_call[6] : 6 callsign bytes
//   dest_ssid    : SSID byte (0x60 = not-last; 0xE0 = not-last + CH)
//   src_call[6]  : 6 callsign bytes
//   src_ssid     : SSID byte (0x61 = last; 0xE1 = last + CH)
// ============================================================================

// Build a 15-byte unnumbered frame header + 1-byte control into dst[]
static void build_u_frame(uint8_t *dst, const uint8_t dest_call[6], uint8_t dest_ssid, const uint8_t src_call[6], uint8_t src_ssid, uint8_t ctrl) {
    memcpy(dst, dest_call, 6);
    dst[6] = dest_ssid;
    memcpy(dst + 7, src_call, 6);
    dst[13] = src_ssid;
    dst[14] = ctrl;
}

// Build I-frame (15-byte header + ctrl + PID + payload) into dst[].
// Returns total length written.
static size_t build_i_frame(uint8_t *dst, const uint8_t dest_call[6], uint8_t dest_ssid, const uint8_t src_call[6], uint8_t src_ssid, uint8_t ctrl, uint8_t pid,
        const uint8_t *payload, size_t payload_len) {
    memcpy(dst, dest_call, 6);
    dst[6] = dest_ssid;
    memcpy(dst + 7, src_call, 6);
    dst[13] = src_ssid;
    dst[14] = ctrl;
    dst[15] = pid;
    if (payload && payload_len > 0) {
        memcpy(dst + 16, payload, payload_len);
    }
    return 16 + payload_len;
}

// ============================================================================
// Callback implementations
// ============================================================================
static void cb_transmit(void *user_data, uint8_t *data, size_t len) {
    // capture transmit for frame inspection
    if (len > 0 && len <= sizeof(captured_buffer)) {
        memcpy(captured_buffer, data, len);
        captured_len = len;
        // control byte is always at offset 14 for simple (no-digipeater) frames
        if (len >= 15) {
            last_ctrl_byte = data[14];
        }
    }
    transmit_count++;
}

static void cb_on_connect(void *user_data, bool initiated_locally) {
    test_ctx_t *ctx = (test_ctx_t*) user_data;
    if (!ctx)
        return;

    ctx->connect_called = true;
    ctx->connect_initiated_locally = initiated_locally;
    ctx->connect_call_count++;
    DEBUG_PRINT("on_connect fired: initiated_locally=%s count=%u", initiated_locally ? "true" : "false", ctx->connect_call_count);
}

static void cb_on_disconnect(void *user_data, uint8_t reason) {
    test_ctx_t *ctx = (test_ctx_t*) user_data;
    if (!ctx)
        return;

    ctx->disconnect_called = true;
    ctx->disconnect_reason = reason;
    ctx->disconnect_call_count++;
    DEBUG_PRINT("on_disconnect fired: reason=%u count=%u", reason, ctx->disconnect_call_count);
}

static void cb_on_dl_error(void *user_data, ax25_dl_error_t error) {
    test_ctx_t *ctx = (test_ctx_t*) user_data;
    if (!ctx)
        return;

    ctx->error_called = true;
    ctx->last_error = error;
    ctx->error_call_count++;
    DEBUG_PRINT("on_dl_error fired: error=%d count=%u", (int)error, ctx->error_call_count);
}

static void cb_on_busy(void *user_data, bool busy) {
    test_ctx_t *ctx = (test_ctx_t*) user_data;
    if (!ctx)
        return;

    ctx->busy_called = true;
    ctx->busy_state = busy;
    DEBUG_PRINT("on_busy fired: busy=%s", busy ? "true" : "false");
}

static void cb_on_data(void *user_data, uint8_t *data, size_t len, uint8_t pid) {
    test_ctx_t *ctx = (test_ctx_t*) user_data;
    if (!ctx)
        return;

    ctx->data_received = true;
    ctx->last_pid = pid;
    ctx->last_payload_len = (len > sizeof(ctx->last_payload)) ? sizeof(ctx->last_payload) : len;
    if (data && ctx->last_payload_len > 0) {
        memcpy(ctx->last_payload, data, ctx->last_payload_len);
    }DEBUG_PRINT("on_data fired: pid=0x%02X len=%zu", pid, len);
}

// ============================================================================
// Utility helpers
// ============================================================================

static void reset_capture(void) {
    captured_len = 0;
    transmit_count = 0;
    last_ctrl_byte = 0;
    memset(captured_buffer, 0, sizeof(captured_buffer));
}

static void reset_ctx(test_ctx_t *ctx) {
    memset(ctx, 0, sizeof(test_ctx_t));
}

// Establish a standard modulo-8 connection between TEST1 and TEST2.
// On success conn->state == AX25_STATE_CONNECTED.  Returns 0 on success.
static int helper_establish_connection(ax25_connection_t *conn) {
    // shared helper for connection setup
    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    if (!dest || parse_err != 0)
        return -1;
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    if (!src || parse_err != 0) {
        free(dest);
        return -1;
    }

    reset_capture();
    uint8_t err = ax25_connect(conn, dest, src);
    free(dest);
    free(src);
    if (err != 0)
        return -1;

    // Inject UA F=1 from TEST2 (dest=TEST1, src=TEST2, ctrl=0x73)
    uint8_t ua_raw[15];
    build_u_frame(ua_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x73);
    uint8_t decode_err = 0;
    ax25_frame_t *ua = ax25_frame_decode(ua_raw, sizeof(ua_raw), MODULO128_FALSE, &decode_err);
    if (!ua || decode_err != 0)
        return -1;

    reset_capture();
    ax25_process_frame(conn, ua, 10);
    ax25_frame_free(ua, &decode_err);

    return (conn->state == AX25_STATE_CONNECTED) ? 0 : -1;
}

// ============================================================================
// A) Simultaneous Bidirectional Connection Attempts
// ============================================================================

// test_simultaneous_connection:
//   Station A (TEST1) sends SABM to TEST2 -> AWAITING_CONNECTION.
//   Before any UA arrives, station B (TEST2) sends its own SABM to TEST1.
//   Per AX.25 v2.2 Section 6.3.1 and Appendix C4 SDL, receiving a SABM in
//   AWAITING_CONNECTION resolves the collision: send UA, go CONNECTED,
//   and report DL-CONNECT indication (initiated_locally=false).
static int test_simultaneous_connection(void) {
    printf("\n--- test_simultaneous_connection ---\n");
    DEBUG_PRINT("Testing simultaneous SABM collision resolution");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };
    cb.on_busy = cb_on_busy;
    cb.on_data = cb_on_data;

    uint8_t err = ax25_connection_init(&conn, &cb, &ctx);
    TEST_ASSERT(err == 0, "Connection init succeeded", err);

    // send SABM from TEST1 to TEST2
    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(dest && src && parse_err == 0, "Addresses parsed", parse_err);

    reset_capture();
    err = ax25_connect(&conn, dest, src);
    free(dest);
    free(src);
    TEST_ASSERT(err == 0, "ax25_connect succeeded", err);
    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_CONNECTION, "State is AWAITING_CONNECTION after ax25_connect", conn.state);
    TEST_ASSERT(transmit_count == 1, "SABM was transmitted", transmit_count);
    DEBUG_STATE("State after ax25_connect", conn.state);
    DEBUG_VAR("Transmit count", transmit_count);

    // inject SABM from TEST2 while we are in AWAITING_CONNECTION
    // This simulates TEST2 also initiating a connection simultaneously.
    // dest=TEST1 (us), src=TEST2, ctrl=0x3F (SABM P=1)
    uint8_t sabm_raw[15];
    build_u_frame(sabm_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x3F);
    uint8_t decode_err = 0;
    ax25_frame_t *sabm_in = ax25_frame_decode(sabm_raw, sizeof(sabm_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(sabm_in != NULL && decode_err == 0, "Incoming SABM decoded successfully", decode_err);
    DEBUG_FRAME("Injected SABM raw bytes", sabm_raw, sizeof(sabm_raw));
    DEBUG_VAR("Frame type after decode", sabm_in->type);

    // process the incoming SABM - should resolve collision
    reset_capture();
    ax25_process_frame(&conn, sabm_in, 20);
    ax25_frame_free(sabm_in, &decode_err);

    DEBUG_STATE("State after receiving SABM while AWAITING_CONNECTION", conn.state);
    DEBUG_VAR("Transmit count after SABM", transmit_count);
    DEBUG_HEX("Last transmitted control byte", last_ctrl_byte);
    DEBUG_BOOL("on_connect called", ctx.connect_called);
    DEBUG_BOOL("on_connect initiated_locally", ctx.connect_initiated_locally);

    TEST_ASSERT(conn.state == AX25_STATE_CONNECTED, "Simultaneous SABM: state resolved to CONNECTED", conn.state);
    TEST_ASSERT(transmit_count == 1, "Simultaneous SABM: UA transmitted as collision response", transmit_count);
    // UA control byte: 0x63 (F=P bit from received SABM P=1 -> F=1 -> 0x63|0x10=0x73)
    TEST_ASSERT(last_ctrl_byte == 0x73 || last_ctrl_byte == 0x63, "Transmitted frame is UA (0x63 or 0x73)", last_ctrl_byte);
    TEST_ASSERT(ctx.connect_called, "on_connect callback fired on collision resolution", 0);
    TEST_ASSERT(!ctx.connect_initiated_locally, "on_connect: initiated_locally=false (we received SABM)", 0);

    ax25_connection_cleanup(&conn);
    return 0;
}

// ============================================================================
// B) Collision Recovery Scenarios (half-duplex failures during AWAITING_CONNECTION)
// ============================================================================

// test_ua_f0_dl_error_c:
//   Per AX.25 v2.2 Appendix C4 SDL, UA with F=0 received in AWAITING_CONNECTION
//   must trigger DL-ERROR C and NOT transition to CONNECTED.
//   The spec states only UA with F=1 confirms the connection.
static int test_ua_f0_dl_error_c(void) {
    printf("\n--- test_ua_f0_dl_error_c ---\n");
    DEBUG_PRINT("Testing UA with F=0 causes DL-ERROR C in AWAITING_CONNECTION");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };

    ax25_connection_init(&conn, &cb, &ctx);

    // put conn into AWAITING_CONNECTION
    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    ax25_connect(&conn, dest, src);
    free(dest);
    free(src);
    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_CONNECTION, "State is AWAITING_CONNECTION", conn.state);

    // inject UA with F=0 (control byte 0x63, not 0x73)
    uint8_t ua_f0_raw[15];
    build_u_frame(ua_f0_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x63);  // 0x63 = UA F=0
    uint8_t decode_err = 0;
    ax25_frame_t *ua_f0 = ax25_frame_decode(ua_f0_raw, sizeof(ua_f0_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(ua_f0 != NULL && decode_err == 0, "UA F=0 frame decoded", decode_err);
    DEBUG_FRAME("UA F=0 raw bytes", ua_f0_raw, sizeof(ua_f0_raw));
    DEBUG_HEX("UA frame type", ua_f0->type);

    reset_capture();
    ax25_process_frame(&conn, ua_f0, 30);
    ax25_frame_free(ua_f0, &decode_err);

    DEBUG_STATE("State after UA F=0", conn.state);
    DEBUG_BOOL("DL-ERROR called", ctx.error_called);
    DEBUG_VAR("DL-ERROR code", ctx.last_error);
    DEBUG_BOOL("on_connect called", ctx.connect_called);

    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_CONNECTION, "UA F=0: state remains AWAITING_CONNECTION (not connected)", conn.state);
    TEST_ASSERT(ctx.error_called, "UA F=0: DL-ERROR callback fired", 0);
    TEST_ASSERT(ctx.last_error == AX25_DL_ERROR_C, "UA F=0: error code is DL-ERROR C", ctx.last_error);
    TEST_ASSERT(!ctx.connect_called, "UA F=0: on_connect NOT called (no false connection)", 0);

    ax25_connection_cleanup(&conn);
    return 0;
}

// test_dm_in_awaiting_connection:
//   Per AX.25 v2.2 Section 4.3.3.5, receiving DM while AWAITING_CONNECTION
//   means the remote refuses or cannot accept the connection.
//   Expected: DISCONNECTED, on_disconnect(reason=1).
static int test_dm_in_awaiting_connection(void) {
    printf("\n--- test_dm_in_awaiting_connection ---\n");
    DEBUG_PRINT("Testing DM received during AWAITING_CONNECTION");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };

    ax25_connection_init(&conn, &cb, &ctx);

    // enter AWAITING_CONNECTION
    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    ax25_connect(&conn, dest, src);
    free(dest);
    free(src);
    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_CONNECTION, "State is AWAITING_CONNECTION", conn.state);

    // Inject DM from TEST2 (control byte 0x0F)
    uint8_t dm_raw[15];
    build_u_frame(dm_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x0F);
    uint8_t decode_err = 0;
    ax25_frame_t *dm = ax25_frame_decode(dm_raw, sizeof(dm_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(dm != NULL && decode_err == 0, "DM frame decoded", decode_err);
    DEBUG_FRAME("DM raw bytes", dm_raw, sizeof(dm_raw));
    DEBUG_HEX("DM frame type", dm->type);

    reset_capture();
    ax25_process_frame(&conn, dm, 40);
    ax25_frame_free(dm, &decode_err);

    DEBUG_STATE("State after DM in AWAITING_CONNECTION", conn.state);
    DEBUG_BOOL("on_disconnect called", ctx.disconnect_called);
    DEBUG_VAR("disconnect reason", ctx.disconnect_reason);

    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "DM in AWAITING_CONNECTION: state = DISCONNECTED", conn.state);
    TEST_ASSERT(ctx.disconnect_called, "DM in AWAITING_CONNECTION: on_disconnect called", 0);
    TEST_ASSERT(ctx.disconnect_reason == 1, "DM in AWAITING_CONNECTION: disconnect reason=1 (remote refused)", ctx.disconnect_reason);
    TEST_ASSERT(conn.t1.running == 0, "DM in AWAITING_CONNECTION: T1 stopped", 0);
    TEST_ASSERT(!ctx.connect_called, "DM in AWAITING_CONNECTION: on_connect NOT called", 0);

    return 0;
}

// test_disc_in_awaiting_connection:
//   Per Appendix C4 SDL: DISC received while AWAITING_CONNECTION causes
//   DM response, T1 cancel, transition to DISCONNECTED with reason=1.
static int test_disc_in_awaiting_connection(void) {
    printf("\n--- test_disc_in_awaiting_connection ---\n");
    DEBUG_PRINT("Testing DISC received during AWAITING_CONNECTION");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };

    ax25_connection_init(&conn, &cb, &ctx);

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    ax25_connect(&conn, dest, src);
    free(dest);
    free(src);
    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_CONNECTION, "State is AWAITING_CONNECTION", conn.state);

    // Inject DISC from TEST2 (control byte 0x43, P=1)
    uint8_t disc_raw[15];
    build_u_frame(disc_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x43);
    uint8_t decode_err = 0;
    ax25_frame_t *disc = ax25_frame_decode(disc_raw, sizeof(disc_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(disc != NULL && decode_err == 0, "DISC frame decoded", decode_err);
    DEBUG_FRAME("DISC raw bytes", disc_raw, sizeof(disc_raw));

    reset_capture();
    ax25_process_frame(&conn, disc, 50);
    ax25_frame_free(disc, &decode_err);

    DEBUG_STATE("State after DISC in AWAITING_CONNECTION", conn.state);
    DEBUG_VAR("Transmit count (DM expected)", transmit_count);
    DEBUG_HEX("Last ctrl byte (expect DM=0x0F)", last_ctrl_byte);
    DEBUG_BOOL("on_disconnect called", ctx.disconnect_called);
    DEBUG_VAR("disconnect reason", ctx.disconnect_reason);

    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "DISC in AWAITING_CONNECTION: state = DISCONNECTED", conn.state);
    // DM must be sent in response to DISC
    TEST_ASSERT(transmit_count == 1, "DISC in AWAITING_CONNECTION: DM response transmitted", transmit_count);
    TEST_ASSERT(last_ctrl_byte == 0x0F || last_ctrl_byte == 0x1F, "DISC in AWAITING_CONNECTION: transmitted DM (0x0F or 0x1F with F=P)", last_ctrl_byte);
    TEST_ASSERT(ctx.disconnect_called, "DISC in AWAITING_CONNECTION: on_disconnect fired", 0);
    TEST_ASSERT(ctx.disconnect_reason == 1, "DISC in AWAITING_CONNECTION: reason=1 (remote pre-empted)", ctx.disconnect_reason);
    TEST_ASSERT(conn.t1.running == 0, "DISC in AWAITING_CONNECTION: T1 stopped", 0);
    return 0;
}

// ============================================================================
// C) Timer Expiration Edge Cases
// ============================================================================

// test_t1_boundary_exact:
//   T1-1 ticks after the start tick: timer must NOT fire.
//   T1 ticks after start tick: timer MUST fire (entry to TIMER_RECOVERY or SABM retry).
static int test_t1_boundary_exact(void) {
    printf("\n--- test_t1_boundary_exact ---\n");
    DEBUG_PRINT("Testing T1 fires exactly at boundary, not before");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error };
    ax25_connection_init(&conn, &cb, &ctx);

    conn.timers.t1 = 5;
    conn.timers.n2 = 10;
    DEBUG_VAR("T1 configured (ticks)", conn.timers.t1);

    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established for T1 boundary test", rc);
    TEST_ASSERT(conn.state == AX25_STATE_CONNECTED, "State is CONNECTED", conn.state);

    // Send one I-frame to arm T1
    const uint8_t payload[] = { 0x01 };
    reset_capture();
    uint8_t send_err = ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    TEST_ASSERT(send_err == 0, "I-frame sent to arm T1", send_err);
    // capture T1 start_ms immediately after send (Issue 9 fix).
    // ax25_send_data arms T1 using last_tick_10ms; read start_ms before any ax25_tick
    // so we get the exact arm-time and can compute the expiry tick without
    // triggering spurious expirations.
    // Expiry tick = ceil((start_ms + duration_ms) / 10)  because
    //   ax25_timer_expired: (tick*10 - start_ms) >= duration_ms.
    uint32_t t1_expire_tick = (conn.t1.start_ms + conn.t1.duration_ms + 9u) / 10u;
    DEBUG_VAR("t1 expiry tick (ceil)", t1_expire_tick);

    // One tick before expiry: no change
    uint32_t tick_before = t1_expire_tick - 1;
    ax25_tick(&conn, tick_before);
    DEBUG_STATE("State at T1-1 ticks", conn.state);
    DEBUG_VAR("retry_count at T1-1", conn.retry_count);

    TEST_ASSERT(conn.state == AX25_STATE_CONNECTED, "T1-1 ticks: still CONNECTED (timer not yet fired)", conn.state);
    TEST_ASSERT(conn.retry_count == 0, "T1-1 ticks: retry_count still 0 (no expiry)", conn.retry_count);

    // Exactly at expiry tick
    reset_capture();
    uint32_t tick_expire = t1_expire_tick;
    ax25_tick(&conn, tick_expire);
    DEBUG_STATE("State at T1 exact expiry", conn.state);
    DEBUG_VAR("retry_count after exact expiry", conn.retry_count);
    DEBUG_VAR("Transmit count (retransmit expected)", transmit_count);

    TEST_ASSERT(conn.state == AX25_STATE_TIMER_RECOVERY, "T1 exact: entered TIMER_RECOVERY", conn.state);
    TEST_ASSERT(conn.retry_count == 1, "T1 exact: retry_count incremented to 1", conn.retry_count);
    TEST_ASSERT(transmit_count >= 1, "T1 exact: frame retransmitted", transmit_count);

    ax25_connection_cleanup(&conn);
    return 0;
}

// test_n2_exhaustion_awaiting_connection:
//   Per AX.25 v2.2 Section 6.3.1 and Appendix C4 SDL:
//   When T1 expires N2+1 times without receiving UA in AWAITING_CONNECTION,
//   the state machine must issue DL-ERROR N and on_disconnect(reason=3).
static int test_n2_exhaustion_awaiting_connection(void) {
    printf("\n--- test_n2_exhaustion_awaiting_connection ---\n");
    DEBUG_PRINT("Testing N2 retry exhaustion from AWAITING_CONNECTION");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    // configure very short T1 and small N2 for fast testing
    conn.timers.t1 = 3;  // 3 ticks = 30ms
    conn.timers.n2 = 3;  // only 3 retries allowed
    DEBUG_VAR("T1 configured (ticks)", conn.timers.t1);
    DEBUG_VAR("N2 configured (retries)", conn.timers.n2);

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(dest && src && parse_err == 0, "Addresses parsed", parse_err);

    reset_capture();
    ax25_connect(&conn, dest, src);
    free(dest);
    free(src);
    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_CONNECTION, "State is AWAITING_CONNECTION", conn.state);
    DEBUG_STATE("Initial state", conn.state);

    // drive ticks until N2 exhaustion
    // arm T1 via first tick
    ax25_tick(&conn, 1);
    uint32_t t1_start = conn.t1.start_ms / 10u;
    DEBUG_VAR("t1.start_ms/10 after first tick", t1_start);

    uint32_t tick = t1_start;
    // run up to (N2+2) T1 cycles to ensure exhaustion
    uint32_t max_ticks = (uint32_t) (conn.timers.n2 + 3) * (conn.timers.t1 + 2);
    for (uint32_t i = 0; i <= max_ticks && conn.state == AX25_STATE_AWAITING_CONNECTION; i++) {
        tick++;
        ax25_tick(&conn, tick);
        if (i % conn.timers.t1 == 0) {
            DEBUG_STATE("  state at tick", conn.state);
            DEBUG_VAR("  retry_count", conn.retry_count);DEBUG_BOOL("  DL-ERROR called", ctx.error_called);
        }
    }

    DEBUG_STATE("Final state after N2 exhaustion", conn.state);
    DEBUG_VAR("retry_count", conn.retry_count);
    DEBUG_BOOL("on_disconnect called", ctx.disconnect_called);
    DEBUG_VAR("disconnect reason", ctx.disconnect_reason);
    DEBUG_BOOL("DL-ERROR N called", ctx.error_called);
    DEBUG_VAR("DL-ERROR code", ctx.last_error);

    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "N2 exhaustion from AWAITING_CONNECTION: DISCONNECTED", conn.state);
    TEST_ASSERT(ctx.disconnect_called, "N2 exhaustion: on_disconnect callback fired", 0);
    TEST_ASSERT(ctx.disconnect_reason == 3, "N2 exhaustion: reason=3 (timeout)", ctx.disconnect_reason);
    TEST_ASSERT(ctx.error_called, "N2 exhaustion: DL-ERROR N fired", 0);
    TEST_ASSERT(ctx.last_error == AX25_DL_ERROR_N, "N2 exhaustion: error code is DL-ERROR N", ctx.last_error);
    TEST_ASSERT(conn.t1.running == 0, "N2 exhaustion: T1 stopped", 0);

    return 0;
}

// test_n2_exhaustion_awaiting_release:
//   Same as above but connection is first established, then DISC is sent
//   (AWAITING_RELEASE state) and no UA/DM is ever received.
static int test_n2_exhaustion_awaiting_release(void) {
    printf("\n--- test_n2_exhaustion_awaiting_release ---\n");
    DEBUG_PRINT("Testing N2 retry exhaustion from AWAITING_RELEASE");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    // set T1 and N2 before establishing
    conn.timers.t1 = 3;
    conn.timers.n2 = 2;
    DEBUG_VAR("T1 (ticks)", conn.timers.t1);
    DEBUG_VAR("N2 (retries)", conn.timers.n2);

    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established", rc);
    TEST_ASSERT(conn.state == AX25_STATE_CONNECTED, "State is CONNECTED", conn.state);
    reset_ctx(&ctx);  // reset ctx after connect callbacks

    // Initiate disconnect - enters AWAITING_RELEASE
    reset_capture();
    uint8_t disc_err = ax25_disconnect(&conn);
    TEST_ASSERT(disc_err == 0, "ax25_disconnect succeeded", disc_err);
    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_RELEASE, "State is AWAITING_RELEASE", conn.state);
    DEBUG_STATE("State after ax25_disconnect", conn.state);

    // drive ticks until N2 exhaustion (no UA response)
    ax25_tick(&conn, 200);
    uint32_t t1_start = conn.t1.start_ms / 10u;
    DEBUG_VAR("t1.start_ms/10 (tick equivalent)", t1_start);

    uint32_t tick = t1_start;
    uint32_t max_ticks = (uint32_t) (conn.timers.n2 + 3) * (conn.timers.t1 + 2);
    for (uint32_t i = 0; i <= max_ticks && conn.state == AX25_STATE_AWAITING_RELEASE; i++) {
        tick++;
        ax25_tick(&conn, tick);
        if (i % conn.timers.t1 == 0) {
            DEBUG_STATE("  state at tick", conn.state);DEBUG_VAR("  retry_count", conn.retry_count);
        }
    }

    DEBUG_STATE("Final state", conn.state);
    DEBUG_BOOL("on_disconnect called", ctx.disconnect_called);
    DEBUG_VAR("disconnect reason", ctx.disconnect_reason);
    DEBUG_BOOL("DL-ERROR N called", ctx.error_called);

    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "N2 exhaustion from AWAITING_RELEASE: DISCONNECTED", conn.state);
    TEST_ASSERT(ctx.disconnect_called, "N2 exhaustion from AWAITING_RELEASE: on_disconnect fired", 0);
    TEST_ASSERT(ctx.disconnect_reason == 3, "N2 exhaustion from AWAITING_RELEASE: reason=3 (timeout)", ctx.disconnect_reason);
    TEST_ASSERT(ctx.error_called, "N2 exhaustion from AWAITING_RELEASE: DL-ERROR N fired", 0);
    TEST_ASSERT(ctx.last_error == AX25_DL_ERROR_N, "N2 exhaustion from AWAITING_RELEASE: error = DL-ERROR N", ctx.last_error);

    return 0;
}

// test_n2_exhaustion_timer_recovery:
//   T1 expires while in CONNECTED (outstanding I-frame) -> TIMER_RECOVERY.
//   T1 continues expiring in TIMER_RECOVERY until N2 exceeded -> DISCONNECTED.
static int test_n2_exhaustion_timer_recovery(void) {
    printf("\n--- test_n2_exhaustion_timer_recovery ---\n");
    DEBUG_PRINT("Testing N2 exhaustion from TIMER_RECOVERY (unacked I-frames)");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    // small timers for fast test
    conn.timers.t1 = 3;
    conn.timers.n2 = 2;
    DEBUG_VAR("T1 (ticks)", conn.timers.t1);
    DEBUG_VAR("N2 (retries)", conn.timers.n2);

    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established", rc);
    reset_ctx(&ctx);

    // Send an I-frame that will go unacknowledged
    const uint8_t payload[] = { 'N', '2' };
    reset_capture();
    uint8_t send_err = ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    TEST_ASSERT(send_err == 0, "I-frame sent", send_err);
    TEST_ASSERT(conn.tx_queue.count == 1, "Frame in retransmit queue", conn.tx_queue.count);
    DEBUG_VAR("tx_queue.count", conn.tx_queue.count);

    // Arm T1
    ax25_tick(&conn, 500);
    uint32_t t1_start = conn.t1.start_ms / 10u;
    DEBUG_VAR("t1.start_ms/10 (tick equivalent)", t1_start);

    uint32_t tick = t1_start;
    uint32_t max_ticks = (uint32_t) (conn.timers.n2 + 3) * (conn.timers.t1 + 2);
    for (uint32_t i = 0; i <= max_ticks; i++) {
        tick++;
        ax25_tick(&conn, tick);
        if (i % conn.timers.t1 == 0) {
            DEBUG_STATE("  state", conn.state);DEBUG_VAR("  retry_count", conn.retry_count);
        }
        if (conn.state == AX25_STATE_DISCONNECTED)
            break;
    }

    DEBUG_STATE("Final state after N2 exhaustion in TIMER_RECOVERY", conn.state);
    DEBUG_VAR("retry_count", conn.retry_count);
    DEBUG_BOOL("on_disconnect called", ctx.disconnect_called);
    DEBUG_VAR("disconnect reason", ctx.disconnect_reason);
    DEBUG_BOOL("DL-ERROR N called", ctx.error_called);

    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "N2 exhaustion from TIMER_RECOVERY: DISCONNECTED", conn.state);
    TEST_ASSERT(ctx.disconnect_called, "N2 exhaustion from TIMER_RECOVERY: on_disconnect fired", 0);
    TEST_ASSERT(ctx.disconnect_reason == 3, "N2 exhaustion from TIMER_RECOVERY: reason=3 (timeout)", ctx.disconnect_reason);
    TEST_ASSERT(ctx.error_called && ctx.last_error == AX25_DL_ERROR_N, "N2 exhaustion from TIMER_RECOVERY: DL-ERROR N fired", ctx.last_error);

    return 0;
}

// ============================================================================
// D) State Transition Validation
// ============================================================================

// test_dm_in_connected:
//   Per AX.25 v2.2 Section 4.3.3.5 / Appendix C4 SDL: receiving DM while
//   CONNECTED is a protocol violation (remote unilaterally reset).
//   Must fire DL-ERROR D, flush tx queue, go DISCONNECTED, on_disconnect(1).
static int test_dm_in_connected(void) {
    printf("\n--- test_dm_in_connected ---\n");
    DEBUG_PRINT("Testing DM received while CONNECTED triggers DL-ERROR D");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established", rc);

    // queue an I-frame so we can verify it is flushed
    const uint8_t payload[] = { 'D', 'M', 'K', 'I', 'L' };
    ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    TEST_ASSERT(conn.tx_queue.count == 1, "Frame queued before DM", conn.tx_queue.count);

    reset_ctx(&ctx);

    uint8_t dm_raw[15];
    build_u_frame(dm_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x0F);
    uint8_t decode_err = 0;
    ax25_frame_t *dm = ax25_frame_decode(dm_raw, sizeof(dm_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(dm != NULL && decode_err == 0, "DM frame decoded", decode_err);
    DEBUG_FRAME("DM injected while CONNECTED", dm_raw, sizeof(dm_raw));

    reset_capture();
    ax25_process_frame(&conn, dm, 600);
    ax25_frame_free(dm, &decode_err);

    DEBUG_STATE("State after DM in CONNECTED", conn.state);
    DEBUG_BOOL("DL-ERROR D called", ctx.error_called);
    DEBUG_VAR("Error code", ctx.last_error);
    DEBUG_BOOL("on_disconnect called", ctx.disconnect_called);
    DEBUG_VAR("disconnect reason", ctx.disconnect_reason);
    DEBUG_VAR("tx_queue.count (should be 0)", conn.tx_queue.count);

    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "DM in CONNECTED: state = DISCONNECTED", conn.state);
    TEST_ASSERT(ctx.error_called && ctx.last_error == AX25_DL_ERROR_D, "DM in CONNECTED: DL-ERROR D fired", ctx.last_error);
    TEST_ASSERT(ctx.disconnect_called, "DM in CONNECTED: on_disconnect called", 0);
    TEST_ASSERT(ctx.disconnect_reason == 1, "DM in CONNECTED: reason=1 (remote reset)", ctx.disconnect_reason);
    TEST_ASSERT(conn.tx_queue.count == 0, "DM in CONNECTED: tx_queue flushed", conn.tx_queue.count);

    return 0;
}

// test_iframe_in_disconnected:
//   Per AX.25 v2.2 Appendix C4 SDL, I-frames received while DISCONNECTED
//   must trigger DL-ERROR M (I-frame not in information-transfer state).
static int test_iframe_in_disconnected(void) {
    printf("\n--- test_iframe_in_disconnected ---\n");
    DEBUG_PRINT("Testing I-frame in DISCONNECTED fires DL-ERROR M");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    // conn stays DISCONNECTED (no ax25_connect)
    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "State is DISCONNECTED", conn.state);

    const uint8_t raw_payload[] = { 'H', 'E', 'L', 'O' };
    uint8_t iframe_raw[20];
    size_t iframe_len = build_i_frame(iframe_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x00, 0xF0, raw_payload, sizeof(raw_payload));
    uint8_t decode_err = 0;
    ax25_frame_t *iframe = ax25_frame_decode(iframe_raw, iframe_len, MODULO128_FALSE, &decode_err);
    TEST_ASSERT(iframe != NULL && decode_err == 0, "I-frame decoded", decode_err);
    DEBUG_FRAME("I-frame injected while DISCONNECTED", iframe_raw, iframe_len);
    DEBUG_HEX("Frame type", iframe->type);

    reset_capture();
    ax25_process_frame(&conn, iframe, 700);
    ax25_frame_free(iframe, &decode_err);

    DEBUG_BOOL("DL-ERROR M fired", ctx.error_called);
    DEBUG_VAR("Error code", ctx.last_error);
    DEBUG_STATE("State (should still be DISCONNECTED)", conn.state);

    TEST_ASSERT(ctx.error_called, "I-frame in DISCONNECTED: DL-ERROR callback fired", 0);
    TEST_ASSERT(ctx.last_error == AX25_DL_ERROR_M, "I-frame in DISCONNECTED: error = DL-ERROR M", ctx.last_error);
    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "I-frame in DISCONNECTED: state unchanged", conn.state);

    return 0;
}

// test_iframe_in_awaiting_connection:
//   I-frames received while AWAITING_CONNECTION also trigger DL-ERROR M.
static int test_iframe_in_awaiting_connection(void) {
    printf("\n--- test_iframe_in_awaiting_connection ---\n");
    DEBUG_PRINT("Testing I-frame in AWAITING_CONNECTION fires DL-ERROR M");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_dl_error = cb_on_dl_error, .on_disconnect = cb_on_disconnect, };
    ax25_connection_init(&conn, &cb, &ctx);

    // enter AWAITING_CONNECTION
    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    ax25_connect(&conn, dest, src);
    free(dest);
    free(src);
    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_CONNECTION, "State is AWAITING_CONNECTION", conn.state);

    const uint8_t raw_payload[] = { 'E', 'A', 'R', 'L', 'Y' };
    uint8_t iframe_raw[21];
    size_t iframe_len = build_i_frame(iframe_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x00, 0xF0, raw_payload, sizeof(raw_payload));
    uint8_t decode_err = 0;
    ax25_frame_t *iframe = ax25_frame_decode(iframe_raw, iframe_len, MODULO128_FALSE, &decode_err);
    TEST_ASSERT(iframe != NULL && decode_err == 0, "I-frame decoded", decode_err);
    DEBUG_FRAME("I-frame injected while AWAITING_CONNECTION", iframe_raw, iframe_len);

    reset_capture();
    ax25_process_frame(&conn, iframe, 800);
    ax25_frame_free(iframe, &decode_err);

    DEBUG_BOOL("DL-ERROR M fired", ctx.error_called);
    DEBUG_VAR("Error code", ctx.last_error);
    DEBUG_STATE("State (should still be AWAITING_CONNECTION)", conn.state);

    TEST_ASSERT(ctx.error_called, "I-frame in AWAITING_CONNECTION: DL-ERROR callback fired", 0);
    TEST_ASSERT(ctx.last_error == AX25_DL_ERROR_M, "I-frame in AWAITING_CONNECTION: error = DL-ERROR M", ctx.last_error);
    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_CONNECTION, "I-frame in AWAITING_CONNECTION: state unchanged", conn.state);

    ax25_connection_cleanup(&conn);
    return 0;
}

// test_disc_received_while_connected:
//   Per AX.25 v2.2 Appendix C4 SDL: DISC received while CONNECTED causes
//   UA response, transition to DISCONNECTED, on_disconnect(reason=1).
static int test_disc_received_while_connected(void) {
    printf("\n--- test_disc_received_while_connected ---\n");
    DEBUG_PRINT("Testing DISC received while CONNECTED -> UA + DISCONNECTED");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established", rc);
    reset_ctx(&ctx);

    // inject DISC from TEST2 while CONNECTED
    uint8_t disc_raw[15];
    build_u_frame(disc_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x43);  // DISC P=1
    uint8_t decode_err = 0;
    ax25_frame_t *disc = ax25_frame_decode(disc_raw, sizeof(disc_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(disc != NULL && decode_err == 0, "DISC frame decoded", decode_err);
    DEBUG_FRAME("DISC injected while CONNECTED", disc_raw, sizeof(disc_raw));

    reset_capture();
    ax25_process_frame(&conn, disc, 900);
    ax25_frame_free(disc, &decode_err);

    DEBUG_STATE("State after DISC in CONNECTED", conn.state);
    DEBUG_VAR("Transmit count (UA expected)", transmit_count);
    DEBUG_HEX("Transmitted ctrl byte (UA=0x63 or 0x73)", last_ctrl_byte);
    DEBUG_BOOL("on_disconnect called", ctx.disconnect_called);
    DEBUG_VAR("disconnect reason", ctx.disconnect_reason);

    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "DISC in CONNECTED: state = DISCONNECTED", conn.state);
    TEST_ASSERT(transmit_count == 1, "DISC in CONNECTED: UA transmitted", transmit_count);
    // UA control: 0x63 (F matches P of received DISC: P=1 -> F=1 -> 0x73)
    TEST_ASSERT(last_ctrl_byte == 0x63 || last_ctrl_byte == 0x73, "DISC in CONNECTED: UA control byte (0x63 or 0x73)", last_ctrl_byte);
    TEST_ASSERT(ctx.disconnect_called, "DISC in CONNECTED: on_disconnect fired", 0);
    TEST_ASSERT(ctx.disconnect_reason == 1, "DISC in CONNECTED: reason=1 (remote disconnect)", ctx.disconnect_reason);
    TEST_ASSERT(!ctx.error_called, "DISC in CONNECTED: no DL-ERROR (clean disconnect)", 0);

    return 0;
}

// test_sabm_received_while_connected:
//   Per AX.25 v2.2 Section 4.3.3.1 / Appendix C4: receiving SABM while CONNECTED
//   resets the link: sequence numbers reset to zero, UA sent, state stays CONNECTED,
//   on_connect(initiated_locally=false) re-fired (DL-CONNECT indication).
static int test_sabm_received_while_connected(void) {
    printf("\n--- test_sabm_received_while_connected ---\n");
    DEBUG_PRINT("Testing SABM received while CONNECTED: link resync");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established", rc);

    // Advance sequence numbers by sending some I-frames
    const uint8_t payload[] = { 'R', 'E', 'S', 'E', 'T' };
    for (int i = 0; i < 3; i++) {
        ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    }
    DEBUG_VAR("V(S) before SABM resync", conn.vars.vs);
    TEST_ASSERT(conn.vars.vs == 3, "V(S) = 3 after 3 I-frames", conn.vars.vs);

    reset_ctx(&ctx);

    // inject SABM from TEST2 while already CONNECTED
    uint8_t sabm_raw[15];
    build_u_frame(sabm_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x3F);
    uint8_t decode_err = 0;
    ax25_frame_t *sabm = ax25_frame_decode(sabm_raw, sizeof(sabm_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(sabm != NULL && decode_err == 0, "SABM frame decoded", decode_err);
    DEBUG_FRAME("SABM injected while CONNECTED", sabm_raw, sizeof(sabm_raw));

    reset_capture();
    ax25_process_frame(&conn, sabm, 1000);
    ax25_frame_free(sabm, &decode_err);

    DEBUG_STATE("State after SABM in CONNECTED", conn.state);
    DEBUG_VAR("V(S) after SABM resync (should be 0)", conn.vars.vs);
    DEBUG_VAR("V(R) after SABM resync (should be 0)", conn.vars.vr);
    DEBUG_VAR("V(A) after SABM resync (should be 0)", conn.vars.va);
    DEBUG_VAR("Transmit count (UA expected)", transmit_count);
    DEBUG_BOOL("on_connect called", ctx.connect_called);
    DEBUG_BOOL("on_connect initiated_locally", ctx.connect_initiated_locally);

    TEST_ASSERT(conn.state == AX25_STATE_CONNECTED, "SABM in CONNECTED: state remains CONNECTED (resync)", conn.state);
    TEST_ASSERT(conn.vars.vs == 0 && conn.vars.vr == 0 && conn.vars.va == 0, "SABM in CONNECTED: sequence numbers reset to 0", conn.vars.vs);
    TEST_ASSERT(transmit_count == 1, "SABM in CONNECTED: UA transmitted", transmit_count);
    TEST_ASSERT(last_ctrl_byte == 0x63 || last_ctrl_byte == 0x73, "SABM in CONNECTED: UA ctrl (0x63 or 0x73)", last_ctrl_byte);
    TEST_ASSERT(ctx.connect_called, "SABM in CONNECTED: on_connect re-fired", 0);
    TEST_ASSERT(!ctx.connect_initiated_locally, "SABM in CONNECTED: initiated_locally=false (peer sent SABM)", 0);

    ax25_connection_cleanup(&conn);
    return 0;
}

// test_rnr_blocks_send_and_rr_clears:
//   Per AX.25 v2.2 Section 6.4.9: RNR received sets peer_busy; ax25_send_data
//   must return 5 while busy. After RR is received, peer_busy clears and
//   send succeeds again.
static int test_rnr_blocks_send_and_rr_clears(void) {
    printf("\n--- test_rnr_blocks_send_and_rr_clears ---\n");
    DEBUG_PRINT("Testing RNR sets peer_busy, RR clears it");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_busy = cb_on_busy, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established", rc);

    // send one I-frame first (V(S)=1, V(A)=0)
    const uint8_t payload[] = { 'F', 'L', 'O', 'W' };
    uint8_t send_err = ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    TEST_ASSERT(send_err == 0, "First I-frame sent", send_err);
    DEBUG_VAR("V(S) after first I-frame", conn.vars.vs);
    DEBUG_BOOL("peer_busy before RNR", conn.peer_busy);

    // Inject RNR N(R)=1 from TEST2 (acknowledges frame 0, busy for new frames)
    // RNR 8-bit: [NR=1][P=0][01][01] = 0010 0101 = 0x25
    uint8_t rnr_raw[15];
    build_u_frame(rnr_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x25);  // RNR NR=1
    uint8_t decode_err = 0;
    ax25_frame_t *rnr = ax25_frame_decode(rnr_raw, sizeof(rnr_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(rnr != NULL && decode_err == 0, "RNR frame decoded", decode_err);
    DEBUG_FRAME("RNR injected", rnr_raw, sizeof(rnr_raw));

    reset_ctx(&ctx);
    ax25_process_frame(&conn, rnr, 1100);
    ax25_frame_free(rnr, &decode_err);

    DEBUG_BOOL("peer_busy after RNR", conn.peer_busy);
    DEBUG_BOOL("on_busy(true) called", ctx.busy_called);
    DEBUG_BOOL("busy_state value", ctx.busy_state);

    TEST_ASSERT(conn.peer_busy, "RNR: peer_busy set", 0);
    TEST_ASSERT(ctx.busy_called && ctx.busy_state, "RNR: on_busy(true) fired", 0);
    TEST_ASSERT(conn.vars.va == 1, "RNR N(R)=1: V(A) advanced to 1 (frame 0 acked)", conn.vars.va);

    // Attempt to send while peer is busy -> must return 5
    uint8_t blocked_err = ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    DEBUG_VAR("ax25_send_data return while busy (expect 5)", blocked_err);
    TEST_ASSERT(blocked_err == 5, "ax25_send_data returns 5 (peer busy)", blocked_err);

    // Inject RR N(R)=1 to clear busy (acknowledges frame, clears busy condition)
    // RR 8-bit: [NR=1][P=0][00][01] = 0010 0001 = 0x21
    uint8_t rr_raw[15];
    build_u_frame(rr_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x21);  // RR NR=1
    ax25_frame_t *rr = ax25_frame_decode(rr_raw, sizeof(rr_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(rr != NULL && decode_err == 0, "RR frame decoded", decode_err);
    DEBUG_FRAME("RR injected to clear busy", rr_raw, sizeof(rr_raw));

    reset_ctx(&ctx);
    ax25_process_frame(&conn, rr, 1200);
    ax25_frame_free(rr, &decode_err);

    DEBUG_BOOL("peer_busy after RR", conn.peer_busy);
    DEBUG_BOOL("on_busy(false) called", ctx.busy_called);

    TEST_ASSERT(!conn.peer_busy, "RR: peer_busy cleared", 0);
    TEST_ASSERT(ctx.busy_called && !ctx.busy_state, "RR: on_busy(false) fired", 0);

    // Send should succeed now
    uint8_t after_err = ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    DEBUG_VAR("ax25_send_data return after RR clears busy (expect 0)", after_err);
    TEST_ASSERT(after_err == 0, "ax25_send_data succeeds after peer busy cleared", after_err);

    ax25_connection_cleanup(&conn);
    return 0;
}

// test_frmr_received_while_connected:
//   Per AX.25 v2.2 Section 4.4.5: receiving FRMR while CONNECTED must
//   trigger DL-ERROR B, on_disconnect(reason=2), flush tx_queue, DISCONNECTED.
static int test_frmr_received_while_connected(void) {
    printf("\n--- test_frmr_received_while_connected ---\n");
    DEBUG_PRINT("Testing FRMR received while CONNECTED -> DL-ERROR B + DISCONNECTED");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established", rc);

    // Send I-frame so queue is non-empty before FRMR
    const uint8_t payload[] = { 'F', 'R', 'M', 'R' };
    ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    TEST_ASSERT(conn.tx_queue.count == 1, "I-frame queued before FRMR", conn.tx_queue.count);

    reset_ctx(&ctx);

    // build FRMR frame from TEST2 to TEST1
    // FRMR 8-bit: control=0x87, info=[rejected_ctrl, VS/VR/CR, flags]
    // 15 header + 1 control + 3 info = 18 bytes
    uint8_t frmr_raw[18];
    memcpy(frmr_raw, TEST1_CALL, 6);
    frmr_raw[6] = 0x60;
    memcpy(frmr_raw + 7, TEST2_CALL, 6);
    frmr_raw[13] = 0x61;
    frmr_raw[14] = 0x87;  // FRMR modifier
    frmr_raw[15] = 0x3F;  // rejected control = SABM
    frmr_raw[16] = 0x00;  // V(S)=0, V(R)=0, CR=0
    frmr_raw[17] = 0x01;  // FRMR_W = invalid control field

    uint8_t decode_err = 0;
    ax25_frame_t *frmr = ax25_frame_decode(frmr_raw, sizeof(frmr_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(frmr != NULL && decode_err == 0, "FRMR frame decoded", decode_err);
    DEBUG_FRAME("FRMR injected while CONNECTED", frmr_raw, sizeof(frmr_raw));
    DEBUG_HEX("FRMR frame type", frmr->type);

    reset_capture();
    ax25_process_frame(&conn, frmr, 1300);
    ax25_frame_free(frmr, &decode_err);

    DEBUG_STATE("State after FRMR in CONNECTED", conn.state);
    DEBUG_BOOL("DL-ERROR B called", ctx.error_called);
    DEBUG_VAR("Error code (expect AX25_DL_ERROR_B=1)", ctx.last_error);
    DEBUG_BOOL("on_disconnect called", ctx.disconnect_called);
    DEBUG_VAR("disconnect reason (expect 2)", ctx.disconnect_reason);
    DEBUG_VAR("tx_queue.count (should be 0)", conn.tx_queue.count);

    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "FRMR in CONNECTED: state = DISCONNECTED", conn.state);
    TEST_ASSERT(ctx.error_called && ctx.last_error == AX25_DL_ERROR_B, "FRMR in CONNECTED: DL-ERROR B fired", ctx.last_error);
    TEST_ASSERT(ctx.disconnect_called, "FRMR in CONNECTED: on_disconnect fired", 0);
    TEST_ASSERT(ctx.disconnect_reason == 2, "FRMR in CONNECTED: reason=2 (FRMR received)", ctx.disconnect_reason);
    TEST_ASSERT(conn.tx_queue.count == 0, "FRMR in CONNECTED: tx_queue flushed", conn.tx_queue.count);

    return 0;
}

// test_timer_recovery_transition:
//   CONNECTED + T1 expires with outstanding frame -> TIMER_RECOVERY.
//   While in TIMER_RECOVERY, an RR(F=1) from the peer acknowledges all frames
//   and stops T1. Verifies transition dynamics.
static int test_timer_recovery_transition(void) {
    printf("\n--- test_timer_recovery_transition ---\n");
    DEBUG_PRINT("Testing CONNECTED->TIMER_RECOVERY on T1 expiry");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    // short T1 for quick TIMER_RECOVERY
    conn.timers.t1 = 4;
    conn.timers.n2 = 10;

    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established", rc);
    reset_ctx(&ctx);

    // Send an I-frame (V(S)=1, V(A)=0 -> unacked)
    const uint8_t payload[] = { 'R', 'E', 'C', 'V' };
    ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    TEST_ASSERT(conn.tx_queue.count == 1, "I-frame queued", conn.tx_queue.count);

    // T1 is armed inside ax25_send_data using last_tick_10ms; read start_ms now
    // before calling ax25_tick so no spurious expiry fires first.
    uint32_t t1_expire_tick = (conn.t1.start_ms + conn.t1.duration_ms + 9u) / 10u;
    DEBUG_VAR("t1 expiry tick", t1_expire_tick);

    // Force T1 expiry
    reset_capture();
    ax25_tick(&conn, t1_expire_tick);

    DEBUG_STATE("State after T1 expiry in CONNECTED", conn.state);
    DEBUG_VAR("retry_count after T1 expiry", conn.retry_count);
    DEBUG_VAR("transmit_count (retransmit expected)", transmit_count);

    TEST_ASSERT(conn.state == AX25_STATE_TIMER_RECOVERY, "T1 expiry: state = TIMER_RECOVERY", conn.state);
    TEST_ASSERT(conn.retry_count == 1, "T1 expiry: retry_count = 1", conn.retry_count);
    TEST_ASSERT(transmit_count >= 1, "T1 expiry: frame retransmitted in TIMER_RECOVERY", transmit_count);

    // inject RR N(R)=1 from peer to ack the outstanding frame
    // RR NR=1, F=1 (response to our poll): [1][0][0][P=1][00][01] = 0011 0001 = 0x31
    // Actually RR NR=1, F=0: 0010 0001 = 0x21
    uint8_t rr_raw[15];
    build_u_frame(rr_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x21);  // RR NR=1 F=0
    uint8_t decode_err = 0;
    ax25_frame_t *rr = ax25_frame_decode(rr_raw, sizeof(rr_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(rr != NULL && decode_err == 0, "RR frame decoded", decode_err);

    reset_ctx(&ctx);
    ax25_process_frame(&conn, rr, t1_expire_tick + 1);
    ax25_frame_free(rr, &decode_err);

    DEBUG_STATE("State after RR in TIMER_RECOVERY", conn.state);
    DEBUG_VAR("V(A) after RR NR=1 (should be 1)", conn.vars.va);
    DEBUG_VAR("tx_queue.count (should be 0)", conn.tx_queue.count);
    DEBUG_VAR("t1.running (should be 0 - stopped)", conn.t1.running);

    TEST_ASSERT(conn.vars.va == 1, "RR N(R)=1 in TIMER_RECOVERY: V(A) = 1", conn.vars.va);
    TEST_ASSERT(conn.tx_queue.count == 0, "RR in TIMER_RECOVERY: tx_queue empty (all acked)", conn.tx_queue.count);
    TEST_ASSERT(conn.t1.running == 0, "RR in TIMER_RECOVERY: T1 stopped (all acked)", conn.t1.running);

    ax25_connection_cleanup(&conn);
    return 0;
}

// test_sabm_received_while_timer_recovery:
//   SABM received while in TIMER_RECOVERY also resets the link per spec.
//   Expected: tx_queue flushed, sequence variables reset, UA sent, CONNECTED,
//   on_connect(initiated_locally=false).
static int test_sabm_received_while_timer_recovery(void) {
    printf("\n--- test_sabm_received_while_timer_recovery ---\n");
    DEBUG_PRINT("Testing SABM received while in TIMER_RECOVERY resets link");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    // put conn in TIMER_RECOVERY with an unacked I-frame
    conn.timers.t1 = 4;
    conn.timers.n2 = 10;

    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established", rc);
    reset_ctx(&ctx);

    const uint8_t payload[] = { 'T', 'R', 'E', 'C' };
    ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    ax25_tick(&conn, 3000);
    ax25_tick(&conn, 3000 + conn.timers.t1);
    TEST_ASSERT(conn.state == AX25_STATE_TIMER_RECOVERY, "Pre-condition: in TIMER_RECOVERY", conn.state);
    TEST_ASSERT(conn.tx_queue.count == 1, "Pre-condition: I-frame in queue", conn.tx_queue.count);

    reset_ctx(&ctx);

    // Inject SABM from TEST2 while in TIMER_RECOVERY
    uint8_t sabm_raw[15];
    build_u_frame(sabm_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x3F);
    uint8_t decode_err = 0;
    ax25_frame_t *sabm = ax25_frame_decode(sabm_raw, sizeof(sabm_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(sabm != NULL && decode_err == 0, "SABM decoded", decode_err);

    reset_capture();
    ax25_process_frame(&conn, sabm, 3010);
    ax25_frame_free(sabm, &decode_err);

    DEBUG_STATE("State after SABM in TIMER_RECOVERY", conn.state);
    DEBUG_VAR("V(S) (should be 0 after reset)", conn.vars.vs);
    DEBUG_VAR("tx_queue.count (should be 0 after SABM)", conn.tx_queue.count);
    DEBUG_BOOL("on_connect called", ctx.connect_called);
    DEBUG_BOOL("initiated_locally", ctx.connect_initiated_locally);

    TEST_ASSERT(conn.state == AX25_STATE_CONNECTED, "SABM in TIMER_RECOVERY: state = CONNECTED", conn.state);
    TEST_ASSERT(conn.vars.vs == 0 && conn.vars.vr == 0, "SABM in TIMER_RECOVERY: sequence numbers reset", conn.vars.vs);
    TEST_ASSERT(ctx.connect_called, "SABM in TIMER_RECOVERY: on_connect fired", 0);
    TEST_ASSERT(!ctx.connect_initiated_locally, "SABM in TIMER_RECOVERY: initiated_locally=false", 0);

    ax25_connection_cleanup(&conn);
    return 0;
}

// test_stats_tracking:
//   Verifies that statistics counters update correctly across frame operations.
static int test_stats_tracking(void) {
    printf("\n--- test_stats_tracking ---\n");
    DEBUG_PRINT("Testing statistics counter accuracy");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_dl_error = cb_on_dl_error, .on_data = cb_on_data, };
    ax25_connection_init(&conn, &cb, &ctx);

    // establish and track stats
    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established", rc);

    ax25_reset_statistics(&conn);
    DEBUG_PRINT("Statistics reset");

    // Send 2 I-frames
    const uint8_t payload[] = { 'S', 'T', 'A', 'T' };
    ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    DEBUG_VAR("iframe_sent (expect 2)", conn.stats.iframe_sent);
    TEST_ASSERT(conn.stats.iframe_sent == 2, "Stats: iframe_sent = 2 after 2 sends", conn.stats.iframe_sent);
    TEST_ASSERT(conn.stats.bytes_sent == sizeof(payload) * 2, "Stats: bytes_sent = 8", conn.stats.bytes_sent);

    // Inject an I-frame from TEST2 (N(S)=0, N(R)=2 - acks our 2 frames)
    // I ctrl: [NR=2][P=0][NS=0][0] = 0100 0000 = 0x40
    const uint8_t rx_payload[] = { 'R', 'X' };
    uint8_t iframe_raw[18];
    size_t iframe_len = build_i_frame(iframe_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x40, 0xF0, rx_payload, sizeof(rx_payload));
    uint8_t decode_err = 0;
    ax25_frame_t *iframe = ax25_frame_decode(iframe_raw, iframe_len, MODULO128_FALSE, &decode_err);
    TEST_ASSERT(iframe != NULL && decode_err == 0, "I-frame from peer decoded", decode_err);

    ax25_process_frame(&conn, iframe, 4000);
    ax25_frame_free(iframe, &decode_err);

    DEBUG_VAR("iframe_received (expect 1)", conn.stats.iframe_received);
    DEBUG_VAR("bytes_received", conn.stats.bytes_received);
    DEBUG_VAR("V(A) after peer I-frame (expect 2)", conn.vars.va);
    DEBUG_BOOL("Data callback fired", ctx.data_received);
    DEBUG_VAR("Received PID", ctx.last_pid);

    TEST_ASSERT(conn.stats.iframe_received == 1, "Stats: iframe_received = 1", conn.stats.iframe_received);
    TEST_ASSERT(conn.vars.va == 2, "Stats: V(A)=2 (peer acked both our frames via N(R)=2)", conn.vars.va);
    TEST_ASSERT(ctx.data_received, "Stats: on_data callback fired for received I-frame", 0);
    TEST_ASSERT(ctx.last_pid == 0xF0, "Stats: received PID = 0xF0", ctx.last_pid);

    ax25_connection_cleanup(&conn);
    return 0;
}

// test_disc_while_awaiting_release:
//   Verifies that receiving DISC while in AWAITING_RELEASE (we sent DISC, peer
//   sends DISC back before UA) also transitions to DISCONNECTED cleanly.
static int test_disc_while_awaiting_release(void) {
    printf("\n--- test_disc_while_awaiting_release ---\n");
    DEBUG_PRINT("Testing DISC received while in AWAITING_RELEASE");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit, .on_connect = cb_on_connect, .on_disconnect = cb_on_disconnect, .on_dl_error = cb_on_dl_error, };
    ax25_connection_init(&conn, &cb, &ctx);

    int rc = helper_establish_connection(&conn);
    TEST_ASSERT(rc == 0, "Connection established", rc);

    // initiate our disconnect
    reset_ctx(&ctx);
    reset_capture();
    uint8_t disc_err = ax25_disconnect(&conn);
    TEST_ASSERT(disc_err == 0, "ax25_disconnect succeeded", disc_err);
    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_RELEASE, "In AWAITING_RELEASE", conn.state);
    DEBUG_STATE("State after ax25_disconnect", conn.state);

    // Inject DISC from peer (simultaneous disconnect)
    uint8_t disc_raw[15];
    build_u_frame(disc_raw, TEST1_CALL, 0x60, TEST2_CALL, 0x61, 0x43);
    uint8_t decode_err = 0;
    ax25_frame_t *disc = ax25_frame_decode(disc_raw, sizeof(disc_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(disc != NULL && decode_err == 0, "DISC frame decoded", decode_err);

    reset_capture();
    ax25_process_frame(&conn, disc, 5000);
    ax25_frame_free(disc, &decode_err);

    DEBUG_STATE("State after peer DISC in AWAITING_RELEASE", conn.state);
    DEBUG_BOOL("on_disconnect called", ctx.disconnect_called);
    DEBUG_VAR("transmit_count (UA expected)", transmit_count);

    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "DISC in AWAITING_RELEASE: state = DISCONNECTED", conn.state);
    TEST_ASSERT(transmit_count == 1, "DISC in AWAITING_RELEASE: UA or DM transmitted", transmit_count);
    TEST_ASSERT(ctx.disconnect_called, "DISC in AWAITING_RELEASE: on_disconnect fired", 0);

    return 0;
}

// test_send_data_returns_2_when_not_connected:
//   Verifies ax25_send_data returns error 2 when not in CONNECTED state.
static int test_send_data_invalid_states(void) {
    printf("\n--- test_send_data_invalid_states ---\n");
    DEBUG_PRINT("Testing ax25_send_data error codes for non-CONNECTED states");

    test_ctx_t ctx;
    reset_ctx(&ctx);

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = cb_transmit };
    ax25_connection_init(&conn, &cb, &ctx);

    // DISCONNECTED state -> should return 2
    const uint8_t payload[] = { 'N', 'O' };
    uint8_t err_disc = ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    DEBUG_VAR("send_data in DISCONNECTED (expect 2)", err_disc);
    TEST_ASSERT(err_disc == 2, "DISCONNECTED: ax25_send_data returns 2", err_disc);

    // AWAITING_CONNECTION -> should return 2
    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    ax25_connect(&conn, dest, src);
    free(dest);
    free(src);
    TEST_ASSERT(conn.state == AX25_STATE_AWAITING_CONNECTION, "In AWAITING_CONNECTION", conn.state);
    uint8_t err_await = ax25_send_data(&conn, (uint8_t*) payload, sizeof(payload), 0xF0);
    DEBUG_VAR("send_data in AWAITING_CONNECTION (expect 2)", err_await);
    TEST_ASSERT(err_await == 2, "AWAITING_CONNECTION: ax25_send_data returns 2", err_await);

    ax25_connection_cleanup(&conn);
    return 0;
}

// ============================================================================
// Main Test Entry Point
// ============================================================================

int test_ax25_dl_state_machine_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("Starting AX.25 Section 6 Data Link State Machine Advanced Tests\n");
    printf("Covers: simultaneous connections, collision recovery, timer edge cases,\n");
    printf("        state transition cross-validation, DL-ERROR code verification.\n");
    printf("==================================================================================\n\n");

    // --- A) Simultaneous Bidirectional Connection ---
    printf("--- A) Simultaneous Bidirectional Connection Attempts ---\n");
    result |= test_simultaneous_connection();

    // --- B) Collision Recovery ---
    printf("\n--- B) Collision Recovery Scenarios ---\n");
    result |= test_ua_f0_dl_error_c();
    result |= test_dm_in_awaiting_connection();
    result |= test_disc_in_awaiting_connection();

    // --- C) Timer Edge Cases ---
    printf("\n--- C) Timer Expiration Edge Cases ---\n");
    result |= test_t1_boundary_exact();
    result |= test_n2_exhaustion_awaiting_connection();
    result |= test_n2_exhaustion_awaiting_release();
    result |= test_n2_exhaustion_timer_recovery();

    // --- D) State Transition Validation ---
    printf("\n--- D) State Transition Validation ---\n");
    result |= test_dm_in_connected();
    result |= test_iframe_in_disconnected();
    result |= test_iframe_in_awaiting_connection();
    result |= test_disc_received_while_connected();
    result |= test_sabm_received_while_connected();
    result |= test_rnr_blocks_send_and_rr_clears();
    result |= test_frmr_received_while_connected();
    result |= test_timer_recovery_transition();
    result |= test_sabm_received_while_timer_recovery();
    result |= test_stats_tracking();
    result |= test_disc_while_awaiting_release();
    result |= test_send_data_invalid_states();

    printf("\n==================================================================================\n");
    printf("AX.25 DL State Machine Advanced Tests Completed. %s\n", result == 0 ? "All tests passed." : "*** SOME TESTS FAILED ***");
    printf("==================================================================================\n\n");

    return result;
}
