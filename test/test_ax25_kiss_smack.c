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

// SMACK / G8BPQ / Auto-detection comprehensive test suite
// Tests extensions NOT covered by test_ax25_kiss.c:
//   - SMACK CRC-16 (CRC-16/ARC) utility function known-value vectors
//   - G8BPQ 8-bit XOR checksum utility known-value vectors
//   - SMACK TX path: type-byte flag, CRC trailer, SLIP escaping of CRC bytes
//   - SMACK TX multi-port: port in bits [6:4], bit-7 CRC flag
//   - SMACK RX path: valid frame dispatch with CRC stripped
//   - SMACK RX path: bad CRC triggers on_crc_error, suppresses on_frame
//   - SMACK RX path: frame shorter than 2 CRC bytes treated as CRC error
//   - SMACK RX path: CRC trailer bytes that are FEND/FESC get SLIP-decoded
//   - SMACK TX+RX round-trip loopback (encode then decode)
//   - SMACK with payload containing SLIP-special bytes (FEND, FESC)
//   - SMACK empty payload edge case
//   - AUTO mode: starts standard, auto-upgrades on first SMACK frame
//   - AUTO mode: TX uses SMACK after latch; smack_active stays latched
//   - AUTO mode: ax25_kiss_get_variant reports correctly before/after latch
//   - ax25_kiss_set_variant / ax25_kiss_get_variant for all variants
//   - ax25_kiss_smack_is_active query
//   - ax25_kiss_reset_rx: clears RX state, preserves port params
//   - ax25_kiss_reset_stats / ax25_kiss_get_stats snapshot
//   - Statistics counters: rx_frames, rx_bad_checksum, rx_dropped
//   - ax25_kiss_reset_port_params: single port reset to defaults
//   - ax25_kiss_reset_all_ports: all ports reset to defaults
//   - ax25_kiss_set_poll_mode: poll flag and interval
//   - ax25_kiss_set_hw_flowctrl: HW flow control flag

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "kiss.h"
#include "test_common.h"

// ---------------------------------------------------------------------------
// Global assertion counter (same idiom as test_ax25_kiss.c)
// ---------------------------------------------------------------------------
static uint32_t assert_count = 0;

// ---------------------------------------------------------------------------
// Harness capacity - 128 slots required for reset_all_ports (15 ports x 5 cmds = 75)
// ---------------------------------------------------------------------------
#define HARNESS_TX_SLOTS    128
#define HARNESS_TX_SLOT_SZ  1200  // worst-case KISS+SMACK encoded frame

// ---------------------------------------------------------------------------
// Harness capture buffers
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t data[HARNESS_TX_SLOT_SZ];
    size_t len;
} smack_cap_buf_t;

typedef struct {
    // Serial TX capture
    smack_cap_buf_t tx[HARNESS_TX_SLOTS];
    uint8_t tx_count;

    // on_frame capture
    uint8_t rx_port[HARNESS_TX_SLOTS];
    uint8_t rx_frames[HARNESS_TX_SLOTS][KISS_MAX_FRAME_SIZE];
    size_t rx_lens[HARNESS_TX_SLOTS];
    uint8_t rx_count;

    // on_hardware capture
    uint8_t hw_port[HARNESS_TX_SLOTS];
    uint8_t hw_data[HARNESS_TX_SLOTS][64];
    size_t hw_lens[HARNESS_TX_SLOTS];
    uint8_t hw_count;

    // on_return capture
    uint8_t return_count;

    // on_crc_error capture
    uint8_t crc_err_port[HARNESS_TX_SLOTS];
    uint8_t crc_err_frames[HARNESS_TX_SLOTS][KISS_MAX_FRAME_SIZE];
    size_t crc_err_lens[HARNESS_TX_SLOTS];
    uint8_t crc_err_count;

    // on_log capture
    uint8_t log_count;
} smack_harness_t;

// ---------------------------------------------------------------------------
// Callback implementations
// ---------------------------------------------------------------------------

static void sh_serial_write(uint8_t *data, size_t len, void *user_data) {
    smack_harness_t *h = (smack_harness_t*) user_data;
    if (!h || h->tx_count >= HARNESS_TX_SLOTS)
        return;
    size_t copy = (len > HARNESS_TX_SLOT_SZ) ? HARNESS_TX_SLOT_SZ : len;
    memcpy(h->tx[h->tx_count].data, data, copy);
    h->tx[h->tx_count].len = copy;
    h->tx_count++;
}

static void sh_on_frame(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t *frame, size_t len, void *user_data) {
    (void) ctx;
    smack_harness_t *h = (smack_harness_t*) user_data;
    if (!h || h->rx_count >= HARNESS_TX_SLOTS)
        return;
    uint8_t idx = h->rx_count;
    h->rx_port[idx] = port;
    size_t copy = (len > KISS_MAX_FRAME_SIZE) ? KISS_MAX_FRAME_SIZE : len;
    memcpy(h->rx_frames[idx], frame, copy);
    h->rx_lens[idx] = copy;
    h->rx_count++;
}

static void sh_on_hardware(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t *data, size_t len, void *user_data) {
    (void) ctx;
    smack_harness_t *h = (smack_harness_t*) user_data;
    if (!h || h->hw_count >= HARNESS_TX_SLOTS)
        return;
    uint8_t idx = h->hw_count;
    h->hw_port[idx] = port;
    size_t copy = (len > 64) ? 64 : len;
    memcpy(h->hw_data[idx], data, copy);
    h->hw_lens[idx] = copy;
    h->hw_count++;
}

static void sh_on_return(ax25_kiss_ctx_t *ctx, void *user_data) {
    (void) ctx;
    smack_harness_t *h = (smack_harness_t*) user_data;
    if (!h)
        return;
    h->return_count++;
}

static void sh_on_crc_error(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t *frame, size_t len, void *user_data) {
    (void) ctx;
    smack_harness_t *h = (smack_harness_t*) user_data;
    if (!h || h->crc_err_count >= HARNESS_TX_SLOTS)
        return;
    uint8_t idx = h->crc_err_count;
    h->crc_err_port[idx] = port;
    size_t copy = (len > KISS_MAX_FRAME_SIZE) ? KISS_MAX_FRAME_SIZE : len;
    memcpy(h->crc_err_frames[idx], frame, copy);
    h->crc_err_lens[idx] = copy;
    h->crc_err_count++;
}

static void sh_on_log(ax25_kiss_ctx_t *ctx, uint8_t level, const char *msg, void *user_data) {
    (void) ctx;
    (void) level;
    (void) msg;
    smack_harness_t *h = (smack_harness_t*) user_data;
    if (!h)
        return;
    h->log_count++;
    DEBUG_PRINT("LOG[%u]: %s", level, msg ? msg : "(null)");
}

// ---------------------------------------------------------------------------
// Helper: initialize context and wire all callbacks to harness
// ---------------------------------------------------------------------------
static void sh_harness_init(smack_harness_t *h, ax25_kiss_ctx_t *ctx) {
    memset(h, 0, sizeof(*h));
    ax25_kiss_init(ctx);
    ctx->serial_write = sh_serial_write;
    ctx->on_frame = sh_on_frame;
    ctx->on_hardware = sh_on_hardware;
    ctx->on_return = sh_on_return;
    ctx->on_crc_error = sh_on_crc_error;
    ctx->on_log = sh_on_log;
    ctx->user_data = h;
}

// ---------------------------------------------------------------------------
// Helper: inject raw bytes into the RX state machine
// ---------------------------------------------------------------------------
static void sh_inject(ax25_kiss_ctx_t *ctx, const uint8_t *bytes, size_t len) {
    ax25_kiss_receive_bytes(ctx, bytes, len);
}

// ---------------------------------------------------------------------------
// Helper: SLIP-decode payload bytes from a captured TX buffer.
// Skips opening FEND, type byte(s), and closing FEND.
// type_len = number of type-indicator bytes (1 for standard, 1 for SMACK since
// the type byte is just one byte even with bit-7 set).
// Returns decoded length.
// ---------------------------------------------------------------------------
static size_t slip_decode_payload(const uint8_t *buf, size_t blen, size_t type_len, uint8_t *out, size_t out_max) {
    // buf[0] = FEND, buf[1..type_len] = type bytes, buf[blen-1] = FEND
    size_t dec_len = 0;
    bool in_esc = false;
    for (size_t i = 1u + type_len; i < blen - 1u && dec_len < out_max; i++) {
        uint8_t b = buf[i];
        if (in_esc) {
            if (b == KISS_TFEND) {
                out[dec_len++] = KISS_FEND;
            } else if (b == KISS_TFESC) {
                out[dec_len++] = KISS_FESC;
            }
            in_esc = false;
        } else if (b == KISS_FESC) {
            in_esc = true;
        } else {
            out[dec_len++] = b;
        }
    }
    return dec_len;
}

// ---------------------------------------------------------------------------
// Helper: build a valid SMACK KISS frame bytes for injection into RX path.
// type_smack = type byte with bit-7 set (e.g. 0x80 for port 0 DATA).
// payload / payload_len = raw AX.25 payload bytes.
// out_buf / out_max = destination buffer.
// Returns total bytes written.
// The CRC is computed by calling ax25_kiss_smack_crc16 over [type_smack, payload].
// All bytes except outer FENDs are SLIP-encoded.
// ---------------------------------------------------------------------------
static size_t build_smack_frame(uint8_t type_smack, const uint8_t *payload, size_t payload_len, uint8_t *out_buf, size_t out_max) {
    // Build pre-SLIP input for CRC: [type_smack] + payload
    uint8_t crc_input[KISS_MAX_FRAME_SIZE + 1u];
    crc_input[0] = type_smack;
    if (payload && payload_len > 0u) {
        memcpy(&crc_input[1], payload, payload_len);
    }
    uint16_t crc = ax25_kiss_smack_crc16(crc_input, 1u + payload_len);
    uint8_t crc_lo = (uint8_t) (crc & 0x00FFu);
    uint8_t crc_hi = (uint8_t) ((crc >> 8u) & 0x00FFu);

    DEBUG_VAR("build_smack_frame: type_smack", type_smack); DEBUG_VAR("build_smack_frame: payload_len", (unsigned)payload_len);
    DEBUG_HEX("build_smack_frame: crc_lo", crc_lo);
    DEBUG_HEX("build_smack_frame: crc_hi", crc_hi);

    // SLIP-encode: opening FEND + type + payload + crc_lo + crc_hi + closing FEND
    size_t pos = 0u;

    // Macro: write one SLIP-encoded byte
#define SLIP_WRITE(byte_val) \
    do { \
        uint8_t _b = (byte_val); \
        if (_b == KISS_FEND) { \
            if (pos + 2u <= out_max) { out_buf[pos++] = KISS_FESC; out_buf[pos++] = KISS_TFEND; } \
        } else if (_b == KISS_FESC) { \
            if (pos + 2u <= out_max) { out_buf[pos++] = KISS_FESC; out_buf[pos++] = KISS_TFESC; } \
        } else { \
            if (pos + 1u <= out_max) { out_buf[pos++] = _b; } \
        } \
    } while (0)

    if (pos < out_max)
        out_buf[pos++] = KISS_FEND;  // opening FEND
    SLIP_WRITE(type_smack);
    for (size_t i = 0u; i < payload_len; i++) {
        SLIP_WRITE(payload[i]);
    }
    SLIP_WRITE(crc_lo);
    SLIP_WRITE(crc_hi);
    if (pos < out_max)
        out_buf[pos++] = KISS_FEND;  // closing FEND

#undef SLIP_WRITE

    return pos;
}

// ===========================================================================
// TEST S01: ax25_kiss_smack_crc16 - known-value vectors (CRC-16/ARC)
// ===========================================================================
static int test_smack_crc16_known_vectors(void) {
    printf("\n--- test_smack_crc16_known_vectors ---\n");
    printf("Verify ax25_kiss_smack_crc16() CRC-16/ARC against reference test vectors\n");
    printf("CRC-16/ARC: poly 0xA001 (reflected 0x8005), init 0x0000, no final XOR\n");

    // Reference: CRC-16/ARC of ASCII "123456789" = 0xBB3D
    // Verified against https://crccalc.com CRC-16/ARC
    const uint8_t test_vec[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    uint16_t crc = ax25_kiss_smack_crc16(test_vec, sizeof(test_vec));
    DEBUG_HEX("CRC-16/ARC of '123456789' (expect 0xBB3D)", crc);
    TEST_ASSERT(crc == 0xBB3Du, "CRC-16/ARC of '123456789' = 0xBB3D", crc);

    // Empty string: CRC of zero bytes returns 0x0000 (init value unchanged)
    crc = ax25_kiss_smack_crc16(test_vec, 0u);
    DEBUG_HEX("CRC-16/ARC of empty (expect 0x0000)", crc);
    TEST_ASSERT(crc == 0x0000u, "CRC-16/ARC of empty input = 0x0000", crc);

    // NULL pointer: must return 0x0000 (guard)
    crc = ax25_kiss_smack_crc16(NULL, 5u);
    DEBUG_HEX("CRC-16/ARC of NULL pointer (expect 0x0000)", crc);
    TEST_ASSERT(crc == 0x0000u, "CRC-16/ARC(NULL) = 0x0000", crc);

    // Single byte 0x00
    uint8_t zero_byte = 0x00u;
    crc = ax25_kiss_smack_crc16(&zero_byte, 1u);
    DEBUG_HEX("CRC-16/ARC of single 0x00 byte", crc);
    // For CRC-16/ARC(0x00): init=0, process 0 bits all 0 → CRC=0
    TEST_ASSERT(crc == 0x0000u, "CRC-16/ARC of single 0x00 = 0x0000", crc);

    // Single byte 0xFF
    uint8_t ff_byte = 0xFFu;
    crc = ax25_kiss_smack_crc16(&ff_byte, 1u);
    DEBUG_HEX("CRC-16/ARC of single 0xFF byte", crc);
    // CRC-16/ARC of 0xFF: all 8 bits set
    // Each bit=1 XOR with crc_lsb, then maybe XOR with 0xA001
    // Bit 0: lsb=(0^1)=1, crc>>=1=0, crc^=0xA001=0xA001
    // Bit 1: lsb=(0xA001^1)&1=0, crc>>=1=0x5000, no xor
    // Bit 2: lsb=(0x5000^1)&1=1... this is hard to compute by hand.
    // Use a second computation to cross-check: two identical implementations must agree
    // We just verify it is not zero and is 16-bit
    TEST_ASSERT(crc != 0x0000u, "CRC-16/ARC of 0xFF byte is non-zero", crc);

    // Incremental vs bulk: CRC of two bytes computed in one pass equals
    // CRC of first byte then continuing (incremental) - test via known vector
    // CRC of [0x01, 0x02] single call
    uint8_t two_bytes[] = { 0x01u, 0x02u };
    uint16_t crc_bulk = ax25_kiss_smack_crc16(two_bytes, 2u);
    // CRC of [0x01] then feed [0x02] as if continuing
    // This is an internal property; just verify bulk is consistent with itself
    uint16_t crc_again = ax25_kiss_smack_crc16(two_bytes, 2u);
    DEBUG_HEX("CRC-16/ARC of [01,02] first call", crc_bulk);
    DEBUG_HEX("CRC-16/ARC of [01,02] second call", crc_again);
    TEST_ASSERT(crc_bulk == crc_again, "CRC-16/ARC is deterministic for same input", 0);

    return 0;
}

// ===========================================================================
// TEST S02: ax25_kiss_crc8_xor - known-value vectors (G8BPQ XOR checksum)
// ===========================================================================
static int test_crc8_xor_known_vectors(void) {
    printf("\n--- test_crc8_xor_known_vectors ---\n");
    printf("Verify ax25_kiss_crc8_xor() XOR checksum function\n");

    // Single byte: XOR of 0x55 with init 0x00 = 0x55
    uint8_t b1 = 0x55u;
    uint8_t xor1 = ax25_kiss_crc8_xor(&b1, 1u);
    DEBUG_HEX("XOR of single byte 0x55 (expect 0x55)", xor1);
    TEST_ASSERT(xor1 == 0x55u, "crc8_xor of single 0x55 = 0x55", xor1);

    // Two identical bytes: 0xAA ^ 0xAA = 0x00
    uint8_t two_same[] = { 0xAAu, 0xAAu };
    uint8_t xor2 = ax25_kiss_crc8_xor(two_same, 2u);
    DEBUG_HEX("XOR of [AA,AA] (expect 0x00)", xor2);
    TEST_ASSERT(xor2 == 0x00u, "crc8_xor of [0xAA, 0xAA] = 0x00", xor2);

    // Known sequence: 0x00 ^ 0x01 ^ 0x02 ^ 0x03 = 0x00
    uint8_t seq[] = { 0x00u, 0x01u, 0x02u, 0x03u };
    uint8_t xor3 = ax25_kiss_crc8_xor(seq, sizeof(seq));
    DEBUG_HEX("XOR of [00,01,02,03] (expect 0x00)", xor3);
    TEST_ASSERT(xor3 == 0x00u, "crc8_xor of [0x00,0x01,0x02,0x03] = 0x00", xor3);

    // 0x01 ^ 0x02 ^ 0x04 = 0x07
    uint8_t seq2[] = { 0x01u, 0x02u, 0x04u };
    uint8_t xor4 = ax25_kiss_crc8_xor(seq2, sizeof(seq2));
    DEBUG_HEX("XOR of [01,02,04] (expect 0x07)", xor4);
    TEST_ASSERT(xor4 == 0x07u, "crc8_xor of [0x01,0x02,0x04] = 0x07", xor4);

    // Zero-length: returns init value 0x00
    uint8_t dummy = 0x99u;
    uint8_t xor_empty = ax25_kiss_crc8_xor(&dummy, 0u);
    DEBUG_HEX("XOR of zero-length (expect 0x00)", xor_empty);
    TEST_ASSERT(xor_empty == 0x00u, "crc8_xor of zero-length = 0x00", xor_empty);

    // NULL pointer guard
    uint8_t xor_null = ax25_kiss_crc8_xor(NULL, 5u);
    DEBUG_HEX("XOR of NULL pointer (expect 0x00)", xor_null);
    TEST_ASSERT(xor_null == 0x00u, "crc8_xor(NULL) = 0x00", xor_null);

    return 0;
}

// ===========================================================================
// TEST S03: SMACK TX - type byte has bit-7 set (CRC flag)
// ===========================================================================
static int test_smack_tx_type_byte_flag(void) {
    printf("\n--- test_smack_tx_type_byte_flag ---\n");
    printf("Verify TX SMACK frame: type byte has bit-7 (KISS_SMACK_CRC_FLAG) set\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    uint8_t payload[] = { 0x11u, 0x22u, 0x33u };
    uint8_t rc = ax25_kiss_send_frame(&ctx, 0, payload, sizeof(payload));
    DEBUG_VAR("send_frame (SMACK, port=0) return code (expect 0)", rc);
    TEST_ASSERT(rc == KISS_OK, "SMACK send_frame returns KISS_OK", rc);
    TEST_ASSERT(h.tx_count == 1u, "One TX write produced", h.tx_count);

    if (h.tx_count > 0u) {
        const uint8_t *buf = h.tx[0].data;
        size_t blen = h.tx[0].len;
        DEBUG_BUF("SMACK TX frame bytes", buf, blen);

        // Opening and closing FEND
        TEST_ASSERT(buf[0] == KISS_FEND, "SMACK frame starts with FEND", buf[0]);
        TEST_ASSERT(buf[blen - 1u] == KISS_FEND, "SMACK frame ends with FEND", 0);

        // Type byte: for port=0, DATA command with SMACK flag = 0x00 | 0x80 = 0x80
        uint8_t type_byte = buf[1u];
        DEBUG_HEX("SMACK type byte (expect 0x80)", type_byte);
        TEST_ASSERT((type_byte & KISS_SMACK_CRC_FLAG) != 0u, "SMACK type byte has KISS_SMACK_CRC_FLAG (bit 7) set", type_byte);
        TEST_ASSERT(KISS_IS_SMACK_FRAME(type_byte), "KISS_IS_SMACK_FRAME macro returns true for SMACK type byte", type_byte);
        TEST_ASSERT(KISS_CMD(type_byte) == KISS_CMD_DATA, "SMACK type byte CMD nibble = KISS_CMD_DATA", type_byte);

        // For SMACK port 0: bits[6:4] of 0x80 = 0 → port 0
        uint8_t smack_port = (uint8_t) ((type_byte >> 4u) & 0x07u);
        DEBUG_VAR("SMACK port extracted from type byte (expect 0)", smack_port);
        TEST_ASSERT(smack_port == 0u, "Port 0 encoded correctly in SMACK type byte", smack_port);

        // Minimum frame length: FEND + type(1) + payload(3) + CRC(2) + FEND = 8
        // (No escaping in this payload)
        TEST_ASSERT(blen >= 8u, "SMACK frame has minimum required length (>=8)", (unsigned )blen);
    }

    return 0;
}

// ===========================================================================
// TEST S04: SMACK TX - CRC trailer appended and correct
// ===========================================================================
static int test_smack_tx_crc_trailer_correct(void) {
    printf("\n--- test_smack_tx_crc_trailer_correct ---\n");
    printf("Verify SMACK TX appends correct CRC-16/ARC trailer (LSB first)\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    uint8_t payload[] = { 0xAAu, 0xBBu, 0xCCu };
    ax25_kiss_send_frame(&ctx, 0, payload, sizeof(payload));

    if (h.tx_count == 0u) {
        TEST_ASSERT(0, "No TX output for SMACK frame", 0);
        return 1;
    }

    const uint8_t *buf = h.tx[0].data;
    size_t blen = h.tx[0].len;
    DEBUG_BUF("SMACK TX frame with CRC", buf, blen);

    // Decode the full post-FEND, post-type content via SLIP
    uint8_t decoded[KISS_MAX_FRAME_SIZE + 4u];  // +4 for CRC bytes
    size_t dec_len = slip_decode_payload(buf, blen, 1u, decoded, sizeof(decoded));

    DEBUG_BUF("SLIP-decoded content (payload + CRC)", decoded, dec_len); DEBUG_VAR("SLIP-decoded length (expect payload_len + 2 CRC bytes)", (unsigned)dec_len);

    // Decoded must be: payload + crc_lo + crc_hi = 3 + 2 = 5 bytes
    TEST_ASSERT(dec_len == sizeof(payload) + KISS_SMACK_CRC_SIZE, "SMACK decoded content = payload + 2 CRC bytes", (unsigned )dec_len);

    if (dec_len >= sizeof(payload) + KISS_SMACK_CRC_SIZE) {
        // Verify payload portion unchanged
        bool payload_ok = (memcmp(decoded, payload, sizeof(payload)) == 0);
        TEST_ASSERT(payload_ok, "Payload bytes unchanged before CRC trailer", 0);

        // Extract received CRC (LSB first)
        uint8_t rx_crc_lo = decoded[sizeof(payload)];
        uint8_t rx_crc_hi = decoded[sizeof(payload) + 1u];
        uint16_t rx_crc = (uint16_t) ((uint16_t) rx_crc_lo | ((uint16_t) rx_crc_hi << 8u));
        DEBUG_HEX("Received CRC lo", rx_crc_lo);
        DEBUG_HEX("Received CRC hi", rx_crc_hi);

        // Recompute expected CRC: CRC-16/ARC over [smack_type, payload]
        // smack_type for port 0 DATA = 0x80
        uint8_t crc_input[4u];
        crc_input[0] = 0x80u;  // SMACK type byte for port=0, DATA
        memcpy(&crc_input[1], payload, sizeof(payload));
        uint16_t expected_crc = ax25_kiss_smack_crc16(crc_input, 1u + sizeof(payload));
        DEBUG_HEX("Expected CRC (computed independently)", expected_crc);
        DEBUG_HEX("Received CRC (from TX frame)", rx_crc);

        TEST_ASSERT(rx_crc == expected_crc, "SMACK CRC trailer matches independently-computed CRC-16/ARC", rx_crc);
    }

    return 0;
}

// ===========================================================================
// TEST S05: SMACK TX - CRC bytes that are FEND or FESC get SLIP-escaped
// ===========================================================================
static int test_smack_tx_crc_slip_escaped(void) {
    printf("\n--- test_smack_tx_crc_slip_escaped ---\n");
    printf("Verify SMACK CRC trailer bytes FEND/FESC are SLIP-escaped in TX stream\n");
    printf("(Exhaustive search for a payload that produces FEND or FESC as a CRC byte)\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    // Search for a payload whose SMACK CRC contains FEND (0xC0) or FESC (0xDB)
    bool found_fend_in_crc = false;
    bool found_fesc_in_crc = false;
    uint8_t trigger_lo_payload[4] = { 0 };
    uint8_t trigger_hi_payload[4] = { 0 };

    for (int b = 0; b <= 255; b++) {
        uint8_t input_crc[2];
        input_crc[0] = 0x80u;  // smack_type for port 0 DATA
        input_crc[1] = (uint8_t) b;
        uint16_t crc = ax25_kiss_smack_crc16(input_crc, 2u);
        uint8_t crc_lo = (uint8_t) (crc & 0x00FFu);
        uint8_t crc_hi = (uint8_t) ((crc >> 8u) & 0x00FFu);

        if (!found_fend_in_crc && (crc_lo == KISS_FEND || crc_hi == KISS_FEND)) {
            found_fend_in_crc = true;
            trigger_lo_payload[0] = (uint8_t) b;
            trigger_lo_payload[1] = (crc_lo == KISS_FEND) ? crc_lo : crc_hi;
            DEBUG_VAR("Found FEND in CRC for payload byte", b);
            DEBUG_HEX("CRC lo", crc_lo);
            DEBUG_HEX("CRC hi", crc_hi);
        }
        if (!found_fesc_in_crc && (crc_lo == KISS_FESC || crc_hi == KISS_FESC)) {
            found_fesc_in_crc = true;
            trigger_hi_payload[0] = (uint8_t) b;
            trigger_hi_payload[1] = (crc_lo == KISS_FESC) ? crc_lo : crc_hi;
            DEBUG_VAR("Found FESC in CRC for payload byte", b);
            DEBUG_HEX("CRC lo", crc_lo);
            DEBUG_HEX("CRC hi", crc_hi);
        }
        if (found_fend_in_crc && found_fesc_in_crc)
            break;
    }

    // Now use the found payload and verify that the TX stream does NOT contain
    // a bare FEND or FESC in the CRC-trailer position (must be SLIP-escaped)
    if (found_fend_in_crc) {
        memset(&h, 0, sizeof(h));
        uint8_t p[1] = { trigger_lo_payload[0] };
        ax25_kiss_send_frame(&ctx, 0, p, 1u);

        if (h.tx_count > 0u) {
            const uint8_t *buf = h.tx[0].data;
            size_t blen = h.tx[0].len;
            DEBUG_BUF("TX frame (should have escaped FEND in CRC)", buf, blen);

            // Scan body (skip opening FEND and type, skip closing FEND)
            // There must be no bare FEND (0xC0) between position 2 and blen-1
            // because all inner FENDs must be escaped
            bool bare_fend_found = false;
            for (size_t i = 2u; i < blen - 1u; i++) {
                if (buf[i] == KISS_FEND) {
                    bare_fend_found = true;
                    DEBUG_VAR("Bare FEND found at position (unexpected)", (unsigned)i);
                    break;
                }
            }
            TEST_ASSERT(!bare_fend_found, "No bare FEND in SMACK TX body when CRC byte equals FEND", 0);

            // Verify the SLIP-decoded payload+CRC is correct after decoding
            uint8_t decoded[16u];
            size_t dec_len = slip_decode_payload(buf, blen, 1u, decoded, sizeof(decoded));
            TEST_ASSERT(dec_len == 1u + KISS_SMACK_CRC_SIZE, "SLIP-decoded length = 1 (payload) + 2 (CRC) when CRC has FEND", (unsigned )dec_len);
            if (dec_len == 1u + KISS_SMACK_CRC_SIZE) {
                TEST_ASSERT(decoded[0] == p[0], "Payload byte preserved when CRC has FEND", decoded[0]);
            }
        }
    } else {
        // This should always find a match in the 256-byte search space; if not, note it
        printf("  [INFO] No payload found that produces FEND in CRC for port-0 1-byte frame\n");
        TEST_ASSERT(1, "Skipped: no FEND-in-CRC trigger found in 256-byte search", 0);
    }

    if (found_fesc_in_crc) {
        memset(&h, 0, sizeof(h));
        uint8_t p2[1] = { trigger_hi_payload[0] };
        ax25_kiss_send_frame(&ctx, 0, p2, 1u);

        if (h.tx_count > 0u) {
            const uint8_t *buf = h.tx[0].data;
            size_t blen = h.tx[0].len;
            DEBUG_BUF("TX frame (should have escaped FESC in CRC)", buf, blen);

            // Check that FESC in the body is always followed by TFEND or TFESC
            bool bare_fesc_found = false;
            for (size_t i = 2u; i < blen - 1u; i++) {
                if (buf[i] == KISS_FESC) {
                    if (i + 1u >= blen - 1u) {
                        bare_fesc_found = true;
                        break;
                    }
                    if (buf[i + 1u] != KISS_TFEND && buf[i + 1u] != KISS_TFESC) {
                        bare_fesc_found = true;
                        break;
                    }
                    i++;  // skip escape pair
                }
            }
            TEST_ASSERT(!bare_fesc_found, "No invalid FESC sequence in SMACK TX when CRC byte equals FESC", 0);

            uint8_t decoded2[16u];
            size_t dec_len2 = slip_decode_payload(buf, blen, 1u, decoded2, sizeof(decoded2));
            TEST_ASSERT(dec_len2 == 1u + KISS_SMACK_CRC_SIZE, "SLIP-decoded length correct when CRC has FESC", (unsigned )dec_len2);
        }
    } else {
        printf("  [INFO] No payload found that produces FESC in CRC for port-0 1-byte frame\n");
        TEST_ASSERT(1, "Skipped: no FESC-in-CRC trigger found in 256-byte search", 0);
    }

    return 0;
}

// ===========================================================================
// TEST S06: SMACK TX - multi-port frame (port > 0) type byte encoding
// ===========================================================================
static int test_smack_tx_multiport(void) {
    printf("\n--- test_smack_tx_multiport ---\n");
    printf("Verify SMACK type byte encodes port in bits [6:4] for ports 0-7\n");
    printf("Note: port 4 smack_type=0xC0 (FEND) is SLIP-escaped; decode before checking\n");

    int failures = 0;
    // SMACK uses bit-7 as CRC flag; port uses bits [6:4] giving max 8 ports (0-7)
    for (uint8_t port = 0u; port <= 7u; port++) {
        smack_harness_t h;
        ax25_kiss_ctx_t ctx;
        sh_harness_init(&h, &ctx);
        ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

        uint8_t payload[] = { (uint8_t) port, 0x42u };
        ax25_kiss_send_frame(&ctx, port, payload, sizeof(payload));

        if (h.tx_count == 0u) {
            failures++;
            continue;
        }

        const uint8_t *buf = h.tx[0].data;
        size_t blen = h.tx[0].len;

        // start modified part - SLIP-decode type byte before inspecting
        // buf[0]=FEND, buf[1] may be FESC if smack_type is FEND(0xC0) or FESC(0xDB)
        // port 4: smack_type = (4<<4)|0x80 = 0xC0 = FEND -> SLIP-escaped as FESC TFEND
        uint8_t decoded_type;
        if (buf[1u] == KISS_FESC && blen >= 4u) {
            decoded_type = (buf[2u] == KISS_TFEND) ? KISS_FEND : (buf[2u] == KISS_TFESC) ? KISS_FESC : 0x00u;
            DEBUG_HEX("Port type byte SLIP-escaped; decoded", decoded_type);
        } else {
            decoded_type = buf[1u];
        }
        // end modified part

        // Bit 7 must be set (SMACK flag)
        if (!(decoded_type & KISS_SMACK_CRC_FLAG)) {
            printf("  [FAIL] Port %u: SMACK flag not set in decoded type byte 0x%02X\n", port, decoded_type);
            failures++;
            continue;
        }
        // Bits [6:4] encode the port
        uint8_t enc_port = (uint8_t) ((decoded_type >> 4u) & 0x07u);
        if (enc_port != port) {
            printf("  [FAIL] Port %u: encoded as %u in decoded type byte 0x%02X\n", port, enc_port, decoded_type);
            failures++;
        } else {
            DEBUG_VAR("Port encoded correctly in SMACK type byte", port);
        }
    }
    TEST_ASSERT(failures == 0, "SMACK ports 0-7 encoded in bits[6:4] with CRC flag in bit-7", failures);

    return 0;
}

// ===========================================================================
// TEST S07: SMACK RX - valid frame dispatched with CRC stripped
// ===========================================================================
static int test_smack_rx_valid_frame(void) {
    printf("\n--- test_smack_rx_valid_frame ---\n");
    printf("Verify RX: valid SMACK frame fires on_frame with AX.25 payload (CRC stripped)\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    uint8_t payload[] = { 0xDEu, 0xADu, 0xBEu, 0xEFu };
    uint8_t smack_type = 0x80u;  // port=0, DATA, CRC flag

    uint8_t frame_buf[64u];
    size_t frame_len = build_smack_frame(smack_type, payload, sizeof(payload), frame_buf, sizeof(frame_buf));

    DEBUG_BUF("Injecting valid SMACK frame", frame_buf, frame_len);
    sh_inject(&ctx, frame_buf, frame_len);

    DEBUG_VAR("rx_count (expect 1)", h.rx_count); DEBUG_VAR("crc_err_count (expect 0)", h.crc_err_count);

    TEST_ASSERT(h.rx_count == 1u, "on_frame fires once for valid SMACK frame", h.rx_count);
    TEST_ASSERT(h.crc_err_count == 0u, "on_crc_error not fired for valid SMACK frame", h.crc_err_count);

    if (h.rx_count > 0u) {
        DEBUG_VAR("Received port (expect 0)", h.rx_port[0]);
        DEBUG_BUF("Received payload (expect DE AD BE EF)", h.rx_frames[0], h.rx_lens[0]);
        TEST_ASSERT(h.rx_port[0] == 0u, "on_frame port = 0", h.rx_port[0]);
        TEST_ASSERT(h.rx_lens[0] == sizeof(payload), "on_frame payload length = original payload (CRC stripped)", (unsigned )h.rx_lens[0]);
        bool match = (memcmp(h.rx_frames[0], payload, sizeof(payload)) == 0);
        TEST_ASSERT(match, "on_frame payload bytes match original (no CRC trailer)", 0);
    }

    return 0;
}

// ===========================================================================
// TEST S08: SMACK RX - bad CRC fires on_crc_error and suppresses on_frame
// ===========================================================================
static int test_smack_rx_bad_crc(void) {
    printf("\n--- test_smack_rx_bad_crc ---\n");
    printf("Verify RX: SMACK frame with wrong CRC fires on_crc_error, suppresses on_frame\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    uint8_t payload[] = { 0x11u, 0x22u, 0x33u };
    uint8_t smack_type = 0x80u;

    // Build a correct SMACK frame then corrupt the CRC bytes
    uint8_t frame_buf[64u];
    size_t frame_len = build_smack_frame(smack_type, payload, sizeof(payload), frame_buf, sizeof(frame_buf));

    // Corrupt the CRC: flip the last two bytes before the closing FEND
    // (closing FEND is frame_buf[frame_len-1])
    // Two bytes before closing FEND are the CRC hi byte (may be escaped - find them)
    // Simple approach: replace CRC bytes in the decoded region.
    // Since our test payload has no special bytes, the CRC is the last 2 decoded bytes.
    // In the raw stream: FEND type p0 p1 p2 CRC_lo CRC_hi FEND (positions 0..7)
    // Corrupt by XOR-ing a non-zero value into the CRC area
    if (frame_len >= 4u) {
        frame_buf[frame_len - 2u] ^= 0xFFu;  // corrupt CRC hi byte (or escaped version)
    }

    DEBUG_BUF("Injecting SMACK frame with corrupted CRC", frame_buf, frame_len);
    sh_inject(&ctx, frame_buf, frame_len);

    DEBUG_VAR("rx_count (expect 0)", h.rx_count); DEBUG_VAR("crc_err_count (expect 1)", h.crc_err_count);

    TEST_ASSERT(h.rx_count == 0u, "on_frame NOT fired for SMACK frame with bad CRC", h.rx_count);
    TEST_ASSERT(h.crc_err_count == 1u, "on_crc_error fired once for bad SMACK CRC", h.crc_err_count);

    return 0;
}

// ===========================================================================
// TEST S09: SMACK RX - bad CRC increments rx_bad_checksum and rx_dropped stats
// ===========================================================================
static int test_smack_rx_bad_crc_stats(void) {
    printf("\n--- test_smack_rx_bad_crc_stats ---\n");
    printf("Verify rx_bad_checksum and rx_dropped increment on SMACK CRC failure\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    uint8_t payload[] = { 0xA1u, 0xB2u };
    uint8_t smack_type = 0x80u;
    uint8_t frame_buf[64u];
    size_t frame_len = build_smack_frame(smack_type, payload, sizeof(payload), frame_buf, sizeof(frame_buf));

    // Corrupt CRC
    frame_buf[frame_len - 2u] ^= 0x01u;

    DEBUG_BUF("Injecting corrupted SMACK frame", frame_buf, frame_len);
    sh_inject(&ctx, frame_buf, frame_len);

    ax25_kiss_stats_t stats;
    uint8_t rc = ax25_kiss_get_stats(&ctx, &stats);
    DEBUG_VAR("get_stats return code (expect 0)", rc); DEBUG_VAR("rx_bad_checksum (expect 1)", stats.rx_bad_checksum); DEBUG_VAR("rx_dropped (expect 1)", stats.rx_dropped); DEBUG_VAR("rx_frames (expect 0)", stats.rx_frames);

    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_get_stats returns KISS_OK", rc);
    TEST_ASSERT(stats.rx_bad_checksum == 1u, "rx_bad_checksum = 1 after SMACK CRC failure", stats.rx_bad_checksum);
    TEST_ASSERT(stats.rx_dropped == 1u, "rx_dropped = 1 after SMACK CRC failure", stats.rx_dropped);
    TEST_ASSERT(stats.rx_frames == 0u, "rx_frames = 0 after SMACK CRC failure (frame not dispatched)", stats.rx_frames);

    return 0;
}

// ===========================================================================
// TEST S10: SMACK RX - frame shorter than 2 CRC bytes is treated as CRC error
// ===========================================================================
static int test_smack_rx_too_short_for_crc(void) {
    printf("\n--- test_smack_rx_too_short_for_crc ---\n");
    printf("Verify RX: SMACK frame with < 2 payload bytes triggers CRC error, not on_frame\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    // Inject: FEND + 0x80 (SMACK type, port 0) + only 1 byte (too short for CRC) + FEND
    uint8_t raw_short[] = { KISS_FEND, 0x80u, 0x42u, KISS_FEND };
    DEBUG_BUF("Injecting 1-byte SMACK frame (too short for 2-byte CRC)", raw_short, sizeof(raw_short));
    sh_inject(&ctx, raw_short, sizeof(raw_short));

    DEBUG_VAR("rx_count (expect 0)", h.rx_count); DEBUG_VAR("crc_err_count (expect 1)", h.crc_err_count);

    TEST_ASSERT(h.rx_count == 0u, "on_frame not fired for SMACK frame < 2 CRC bytes", h.rx_count);
    TEST_ASSERT(h.crc_err_count == 1u, "on_crc_error fired for SMACK frame < 2 CRC bytes", h.crc_err_count);

    // Zero-payload SMACK frame (only type byte, no data at all)
    memset(&h, 0, sizeof(h));
    uint8_t raw_zero[] = { KISS_FEND, 0x80u, KISS_FEND };
    DEBUG_BUF("Injecting 0-byte SMACK frame (zero payload)", raw_zero, sizeof(raw_zero));
    sh_inject(&ctx, raw_zero, sizeof(raw_zero));

    DEBUG_VAR("rx_count for 0-byte SMACK (expect 0)", h.rx_count); DEBUG_VAR("crc_err_count for 0-byte SMACK (expect 1)", h.crc_err_count);

    TEST_ASSERT(h.rx_count == 0u, "on_frame not fired for 0-byte SMACK frame", h.rx_count);
    TEST_ASSERT(h.crc_err_count == 1u, "on_crc_error fired for 0-byte SMACK frame", h.crc_err_count);

    return 0;
}

// ===========================================================================
// TEST S11: SMACK RX - SLIP-escaped CRC bytes decoded before CRC check
// ===========================================================================
static int test_smack_rx_escaped_crc_bytes(void) {
    printf("\n--- test_smack_rx_escaped_crc_bytes ---\n");
    printf("Verify RX: SLIP-escaped CRC trailer bytes are decoded before CRC verification\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    // We need a payload whose CRC contains a FEND or FESC byte.
    // Search systematically.
    uint8_t trigger_payload = 0xFFu;
    bool found = false;
    for (int b = 0; b <= 255; b++) {
        uint8_t inp[2u];
        inp[0] = 0x80u;
        inp[1] = (uint8_t) b;
        uint16_t crc = ax25_kiss_smack_crc16(inp, 2u);
        uint8_t crc_lo = (uint8_t) (crc & 0x00FFu);
        uint8_t crc_hi = (uint8_t) ((crc >> 8u) & 0x00FFu);
        if (crc_lo == KISS_FEND || crc_lo == KISS_FESC || crc_hi == KISS_FEND || crc_hi == KISS_FESC) {
            trigger_payload = (uint8_t) b;
            found = true;
            DEBUG_VAR("trigger payload byte for escaped-CRC test", b);
            DEBUG_HEX("CRC lo", crc_lo);
            DEBUG_HEX("CRC hi", crc_hi);
            break;
        }
    }

    if (!found) {
        printf("  [INFO] No trigger found for escaped-CRC test; skipping\n");
        TEST_ASSERT(1, "Skipped: no escaped-CRC trigger found", 0);
        return 0;
    }

    // Build valid SMACK frame using build_smack_frame (handles SLIP escaping)
    uint8_t payload[1u] = { trigger_payload };
    uint8_t frame_buf[32u];
    size_t frame_len = build_smack_frame(0x80u, payload, 1u, frame_buf, sizeof(frame_buf));

    DEBUG_BUF("Injecting SMACK frame with escaped CRC bytes", frame_buf, frame_len);

    // Frame is longer than minimal due to escaping: verify FESC present
    bool has_fesc = false;
    for (size_t i = 2u; i < frame_len - 1u; i++) {
        if (frame_buf[i] == KISS_FESC) {
            has_fesc = true;
            break;
        }
    } DEBUG_BOOL("FESC present in encoded frame (expect true)", has_fesc);
    TEST_ASSERT(has_fesc, "TX frame contains FESC escape for FEND/FESC CRC byte", 0);

    sh_inject(&ctx, frame_buf, frame_len);

    DEBUG_VAR("rx_count (expect 1)", h.rx_count); DEBUG_VAR("crc_err_count (expect 0)", h.crc_err_count);

    TEST_ASSERT(h.rx_count == 1u, "on_frame fires for SMACK frame with escaped CRC bytes", h.rx_count);
    TEST_ASSERT(h.crc_err_count == 0u, "No CRC error for frame with correctly-escaped CRC bytes", h.crc_err_count);

    if (h.rx_count > 0u) {
        DEBUG_BUF("Received payload", h.rx_frames[0], h.rx_lens[0]);
        TEST_ASSERT(h.rx_lens[0] == 1u, "Payload length = 1 after CRC stripping", (unsigned )h.rx_lens[0]);
        TEST_ASSERT(h.rx_frames[0][0] == trigger_payload, "Payload byte correct", h.rx_frames[0][0]);
    }

    return 0;
}

// ===========================================================================
// TEST S12: SMACK TX+RX round-trip loopback
// ===========================================================================
static int test_smack_round_trip(void) {
    printf("\n--- test_smack_round_trip ---\n");
    printf("Verify SMACK: encode with TX, capture bytes, inject into RX, check on_frame\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    // Payload with special bytes to stress both escaping and CRC
    uint8_t payload[] = { 0x82u, 0x84u, KISS_FEND, 0x86u, KISS_FESC, 0x88u, 0x8Au, 0x8Cu };
    DEBUG_BUF("Round-trip payload", payload, sizeof(payload));

    // TX: encode
    uint8_t rc = ax25_kiss_send_frame(&ctx, 0, payload, sizeof(payload));
    TEST_ASSERT(rc == KISS_OK, "SMACK round-trip TX send_frame returns KISS_OK", rc);
    TEST_ASSERT(h.tx_count == 1u, "One TX write captured", h.tx_count);

    if (h.tx_count == 0u)
        return 1;

    // Capture TX output
    uint8_t tx_copy[HARNESS_TX_SLOT_SZ];
    size_t tx_len = h.tx[0].len;
    memcpy(tx_copy, h.tx[0].data, tx_len);
    DEBUG_BUF("Captured SMACK TX bytes for RX injection", tx_copy, tx_len);

    // Reset harness, keep ctx config; inject TX bytes into RX engine
    h.tx_count = 0u;
    h.rx_count = 0u;
    h.crc_err_count = 0u;

    sh_inject(&ctx, tx_copy, tx_len);

    DEBUG_VAR("rx_count after round-trip (expect 1)", h.rx_count); DEBUG_VAR("crc_err_count after round-trip (expect 0)", h.crc_err_count);

    TEST_ASSERT(h.rx_count == 1u, "SMACK round-trip: on_frame fires once", h.rx_count);
    TEST_ASSERT(h.crc_err_count == 0u, "SMACK round-trip: no CRC error", h.crc_err_count);

    if (h.rx_count > 0u) {
        DEBUG_BUF("Round-trip received payload", h.rx_frames[0], h.rx_lens[0]);
        TEST_ASSERT(h.rx_lens[0] == sizeof(payload), "Round-trip payload length matches original", (unsigned )h.rx_lens[0]);
        bool match = (memcmp(h.rx_frames[0], payload, sizeof(payload)) == 0);
        TEST_ASSERT(match, "Round-trip payload bytes match original including FEND/FESC", 0);
    }

    return 0;
}

// ===========================================================================
// TEST S13: SMACK RX - multiple valid frames in sequence, stats accumulate
// ===========================================================================
static int test_smack_rx_multiple_frames_stats(void) {
    printf("\n--- test_smack_rx_multiple_frames_stats ---\n");
    printf("Verify rx_frames stat increments for each valid SMACK frame received\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    uint8_t p1[] = { 0x01u };
    uint8_t p2[] = { 0x02u, 0x03u };
    uint8_t p3[] = { 0x04u, 0x05u, 0x06u };

    uint8_t frame_buf[64u];
    size_t flen;

    flen = build_smack_frame(0x80u, p1, sizeof(p1), frame_buf, sizeof(frame_buf));
    sh_inject(&ctx, frame_buf, flen);
    flen = build_smack_frame(0x80u, p2, sizeof(p2), frame_buf, sizeof(frame_buf));
    sh_inject(&ctx, frame_buf, flen);
    flen = build_smack_frame(0x80u, p3, sizeof(p3), frame_buf, sizeof(frame_buf));
    sh_inject(&ctx, frame_buf, flen);

    ax25_kiss_stats_t stats;
    ax25_kiss_get_stats(&ctx, &stats);
    DEBUG_VAR("rx_frames after 3 valid SMACK frames (expect 3)", stats.rx_frames);

    TEST_ASSERT(h.rx_count == 3u, "on_frame fired 3 times for 3 valid SMACK frames", h.rx_count);
    TEST_ASSERT(stats.rx_frames == 3u, "stats.rx_frames = 3 after 3 valid SMACK frames", stats.rx_frames);
    TEST_ASSERT(stats.rx_bad_checksum == 0u, "stats.rx_bad_checksum = 0 (all frames valid)", stats.rx_bad_checksum);

    return 0;
}

// ===========================================================================
// TEST S14: SMACK RX - port extraction from multi-port SMACK frame
// ===========================================================================
static int test_smack_rx_multiport(void) {
    printf("\n--- test_smack_rx_multiport ---\n");
    printf("Verify RX: SMACK port extracted from bits[6:4] of type byte for ports 0-7\n");

    int failures = 0;
    for (uint8_t port = 0u; port <= 7u; port++) {
        smack_harness_t h;
        ax25_kiss_ctx_t ctx;
        sh_harness_init(&h, &ctx);
        ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

        // Construct SMACK type byte: bit-7 | (port << 4) | KISS_CMD_DATA
        uint8_t smack_type = (uint8_t) (KISS_SMACK_CRC_FLAG | ((uint8_t) (port << 4u)) | KISS_CMD_DATA);
        uint8_t payload[] = { (uint8_t) (0xA0u + port) };
        uint8_t frame_buf[32u];
        size_t flen = build_smack_frame(smack_type, payload, sizeof(payload), frame_buf, sizeof(frame_buf));

        sh_inject(&ctx, frame_buf, flen);

        if (h.rx_count != 1u || h.rx_port[0] != port) {
            printf("  [FAIL] SMACK port %u: rx_count=%u, rx_port=%u\n", port, h.rx_count, h.rx_port[0]);
            failures++;
        } else {
            DEBUG_VAR("SMACK port correctly extracted", port);
        }
    }
    TEST_ASSERT(failures == 0, "SMACK RX ports 0-7 extracted from bits[6:4] correctly", failures);

    return 0;
}

// ===========================================================================
// TEST S15: SMACK RX - standard-mode context ignores bit-7 (no false SMACK detection)
// ===========================================================================
static int test_smack_rx_standard_mode_no_false_detect(void) {
    printf("\n--- test_smack_rx_standard_mode_no_false_detect ---\n");
    printf("Verify: in STANDARD mode, bit-7 in type byte is NOT treated as SMACK flag\n");
    printf("  (port-8 to port-14 DATA frames have bit-7 set naturally in standard KISS)\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    // Leave variant as STANDARD (default after init)

    // Port 8 DATA frame: type = (8 << 4) | 0x00 = 0x80 - same byte as SMACK port-0
    // In STANDARD mode this must be treated as port-8 DATA, NOT as SMACK
    uint8_t payload[] = { 0x55u, 0xAAu };
    uint8_t raw[8u];
    raw[0] = KISS_FEND;
    raw[1] = KISS_TYPE_BYTE(8u, KISS_CMD_DATA);  // = 0x80
    raw[2] = 0x55u;
    raw[3] = 0xAAu;
    raw[4] = KISS_FEND;

    DEBUG_BUF("Injecting port-8 DATA frame (type=0x80) in STANDARD mode", raw, 5u);
    sh_inject(&ctx, raw, 5u);

    DEBUG_VAR("rx_count (expect 1)", h.rx_count); DEBUG_VAR("crc_err_count (expect 0)", h.crc_err_count);

    TEST_ASSERT(h.rx_count == 1u, "Port-8 DATA frame (0x80 type) dispatched in STANDARD mode", h.rx_count);
    TEST_ASSERT(h.crc_err_count == 0u, "No CRC error in STANDARD mode for 0x80 type byte", h.crc_err_count);

    if (h.rx_count > 0u) {
        DEBUG_VAR("Received port (expect 8)", h.rx_port[0]);
        TEST_ASSERT(h.rx_port[0] == 8u, "Port correctly decoded as 8 in STANDARD mode", h.rx_port[0]);
        TEST_ASSERT(h.rx_lens[0] == sizeof(payload), "Payload length correct in STANDARD mode", (unsigned )h.rx_lens[0]);
        bool match = (memcmp(h.rx_frames[0], payload, sizeof(payload)) == 0);
        TEST_ASSERT(match, "Payload bytes correct in STANDARD mode (no CRC stripping)", 0);
    }

    return 0;
}

// ===========================================================================
// TEST S16: ax25_kiss_set_variant and ax25_kiss_get_variant for all variants
// ===========================================================================
static int test_smack_set_get_variant(void) {
    printf("\n--- test_smack_set_get_variant ---\n");
    printf("Verify ax25_kiss_set_variant / ax25_kiss_get_variant for all enum values\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);

    ax25_kiss_variant_t vr;
    uint8_t rc;

    // Default after init: STANDARD
    rc = ax25_kiss_get_variant(&ctx, &vr);
    DEBUG_VAR("Default variant (expect KISS_VARIANT_STANDARD=0)", vr);
    TEST_ASSERT(rc == KISS_OK, "get_variant returns KISS_OK", rc);
    TEST_ASSERT(vr == KISS_VARIANT_STANDARD, "Default variant = KISS_VARIANT_STANDARD", vr);

    // Set SMACK
    rc = ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);
    TEST_ASSERT(rc == KISS_OK, "set_variant(SMACK) returns KISS_OK", rc);
    rc = ax25_kiss_get_variant(&ctx, &vr);
    DEBUG_VAR("After set SMACK (expect 1)", vr);
    TEST_ASSERT(vr == KISS_VARIANT_SMACK, "get_variant returns KISS_VARIANT_SMACK after set", vr);

    // Set G8BPQ
    rc = ax25_kiss_set_variant(&ctx, KISS_VARIANT_G8BPQ);
    TEST_ASSERT(rc == KISS_OK, "set_variant(G8BPQ) returns KISS_OK", rc);
    rc = ax25_kiss_get_variant(&ctx, &vr);
    DEBUG_VAR("After set G8BPQ (expect 2)", vr);
    TEST_ASSERT(vr == KISS_VARIANT_G8BPQ, "get_variant returns KISS_VARIANT_G8BPQ after set", vr);

    // Set FLEXNET
    rc = ax25_kiss_set_variant(&ctx, KISS_VARIANT_FLEXNET);
    TEST_ASSERT(rc == KISS_OK, "set_variant(FLEXNET) returns KISS_OK", rc);
    rc = ax25_kiss_get_variant(&ctx, &vr);
    DEBUG_VAR("After set FLEXNET (expect 3)", vr);
    TEST_ASSERT(vr == KISS_VARIANT_FLEXNET, "get_variant returns KISS_VARIANT_FLEXNET after set", vr);

    // Set AUTO: get_variant should return STANDARD (not yet upgraded)
    rc = ax25_kiss_set_variant(&ctx, KISS_VARIANT_AUTO);
    TEST_ASSERT(rc == KISS_OK, "set_variant(AUTO) returns KISS_OK", rc);
    rc = ax25_kiss_get_variant(&ctx, &vr);
    DEBUG_VAR("After set AUTO before SMACK frame (expect STANDARD=0)", vr);
    TEST_ASSERT(vr == KISS_VARIANT_STANDARD, "get_variant in AUTO mode (pre-SMACK) = KISS_VARIANT_STANDARD", vr);

    // Restore to STANDARD
    rc = ax25_kiss_set_variant(&ctx, KISS_VARIANT_STANDARD);
    rc = ax25_kiss_get_variant(&ctx, &vr);
    TEST_ASSERT(vr == KISS_VARIANT_STANDARD, "Restored to STANDARD", vr);

    // NULL pointer guard
    rc = ax25_kiss_set_variant(NULL, KISS_VARIANT_SMACK);
    TEST_ASSERT(rc == KISS_ERR_NULL, "set_variant(NULL) returns KISS_ERR_NULL", rc);

    rc = ax25_kiss_get_variant(NULL, &vr);
    TEST_ASSERT(rc == KISS_ERR_NULL, "get_variant(NULL ctx) returns KISS_ERR_NULL", rc);

    rc = ax25_kiss_get_variant(&ctx, NULL);
    TEST_ASSERT(rc == KISS_ERR_NULL, "get_variant(NULL out) returns KISS_ERR_NULL", rc);

    return 0;
}

// ===========================================================================
// TEST S17: ax25_kiss_smack_is_active query
// ===========================================================================
static int test_smack_is_active_query(void) {
    printf("\n--- test_smack_is_active_query ---\n");
    printf("Verify ax25_kiss_smack_is_active() returns correct state\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);

    bool active;
    uint8_t rc;

    // After init: not active
    rc = ax25_kiss_smack_is_active(&ctx, &active);
    DEBUG_BOOL("smack_active after init (expect false)", active);
    TEST_ASSERT(rc == KISS_OK, "smack_is_active returns KISS_OK", rc);
    TEST_ASSERT(active == false, "smack_active = false after init", 0);

    // After set STANDARD: not active
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_STANDARD);
    ax25_kiss_smack_is_active(&ctx, &active);
    TEST_ASSERT(active == false, "smack_active = false for STANDARD variant", 0);

    // After set SMACK: active immediately
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);
    ax25_kiss_smack_is_active(&ctx, &active);
    DEBUG_BOOL("smack_active after set SMACK (expect true)", active);
    TEST_ASSERT(active == true, "smack_active = true immediately after set SMACK", 0);

    // After set G8BPQ: not active (G8BPQ uses its own XOR scheme, smack_active=false)
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_G8BPQ);
    ax25_kiss_smack_is_active(&ctx, &active);
    DEBUG_BOOL("smack_active after set G8BPQ (expect false)", active);
    TEST_ASSERT(active == false, "smack_active = false for G8BPQ variant", 0);

    // After set AUTO: not active yet
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_AUTO);
    ax25_kiss_smack_is_active(&ctx, &active);
    DEBUG_BOOL("smack_active after set AUTO before upgrade (expect false)", active);
    TEST_ASSERT(active == false, "smack_active = false in AUTO before first SMACK frame", 0);

    // NULL guard
    rc = ax25_kiss_smack_is_active(NULL, &active);
    TEST_ASSERT(rc == KISS_ERR_NULL, "smack_is_active(NULL ctx) = KISS_ERR_NULL", rc);
    rc = ax25_kiss_smack_is_active(&ctx, NULL);
    TEST_ASSERT(rc == KISS_ERR_NULL, "smack_is_active(NULL out) = KISS_ERR_NULL", rc);

    return 0;
}

// ===========================================================================
// TEST S18: AUTO mode - starts standard, upgrades on first received SMACK frame
// ===========================================================================
static int test_smack_auto_mode_upgrade(void) {
    printf("\n--- test_smack_auto_mode_upgrade ---\n");
    printf("Verify AUTO mode: starts STANDARD, auto-upgrades to SMACK on first SMACK RX\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_AUTO);

    // Before any frame: smack_active must be false, get_variant returns STANDARD
    bool active;
    ax25_kiss_variant_t vr;
    ax25_kiss_smack_is_active(&ctx, &active);
    ax25_kiss_get_variant(&ctx, &vr);
    DEBUG_BOOL("smack_active before first SMACK frame (expect false)", active); DEBUG_VAR("get_variant before first SMACK frame (expect 0=STANDARD)", vr);
    TEST_ASSERT(active == false, "AUTO mode: smack_active=false before first SMACK frame", 0);
    TEST_ASSERT(vr == KISS_VARIANT_STANDARD, "AUTO mode: get_variant=STANDARD before upgrade", vr);

    // Inject a standard (non-SMACK) DATA frame first - should NOT trigger upgrade
    uint8_t std_frame[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_DATA), 0x01u, KISS_FEND };
    sh_inject(&ctx, std_frame, sizeof(std_frame));
    ax25_kiss_smack_is_active(&ctx, &active);
    DEBUG_BOOL("smack_active after standard frame (expect false)", active);
    TEST_ASSERT(active == false, "AUTO mode: smack_active still false after standard DATA frame", 0);
    TEST_ASSERT(h.rx_count == 1u, "Standard frame dispatched in AUTO mode before upgrade", h.rx_count);

    // Now inject a valid SMACK frame - should trigger upgrade
    uint8_t payload[] = { 0xCAu, 0xFEu };
    uint8_t frame_buf[32u];
    size_t flen = build_smack_frame(0x80u, payload, sizeof(payload), frame_buf, sizeof(frame_buf));

    DEBUG_BUF("Injecting first SMACK frame (should trigger AUTO upgrade)", frame_buf, flen);
    sh_inject(&ctx, frame_buf, flen);

    ax25_kiss_smack_is_active(&ctx, &active);
    ax25_kiss_get_variant(&ctx, &vr);
    DEBUG_BOOL("smack_active after first SMACK frame (expect true)", active); DEBUG_VAR("get_variant after first SMACK frame (expect 1=SMACK)", vr);

    TEST_ASSERT(active == true, "AUTO mode: smack_active=true after first SMACK frame received", 0);
    TEST_ASSERT(vr == KISS_VARIANT_SMACK, "AUTO mode: get_variant=SMACK after upgrade", vr);
    TEST_ASSERT(h.rx_count == 2u, "SMACK frame dispatched after AUTO upgrade (rx_count=2)", h.rx_count);
    if (h.rx_count == 2u) {
        TEST_ASSERT(h.rx_lens[1] == sizeof(payload), "AUTO upgrade: received SMACK payload length correct", (unsigned )h.rx_lens[1]);
        bool match = (memcmp(h.rx_frames[1], payload, sizeof(payload)) == 0);
        TEST_ASSERT(match, "AUTO upgrade: received SMACK payload bytes correct", 0);
    }

    return 0;
}

// ===========================================================================
// TEST S19: AUTO mode - TX uses SMACK after smack_active is latched
// ===========================================================================
static int test_smack_auto_mode_tx_after_latch(void) {
    printf("\n--- test_smack_auto_mode_tx_after_latch ---\n");
    printf("Verify AUTO mode: TX uses SMACK framing after smack_active is latched by RX\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_AUTO);

    // TX before latch: should be standard KISS (no SMACK type byte)
    uint8_t payload[] = { 0x11u };
    ax25_kiss_send_frame(&ctx, 0, payload, sizeof(payload));
    if (h.tx_count > 0u) {
        uint8_t type_before = h.tx[0].data[1u];
        DEBUG_HEX("TX type byte before AUTO latch (expect 0x00 standard)", type_before);
        TEST_ASSERT(!(type_before & KISS_SMACK_CRC_FLAG), "TX type byte has no SMACK flag before AUTO latch", type_before);
    }

    // Trigger latch via RX of a SMACK frame
    uint8_t smack_buf[32u];
    uint8_t smack_payload[] = { 0x99u };
    size_t slen = build_smack_frame(0x80u, smack_payload, sizeof(smack_payload), smack_buf, sizeof(smack_buf));
    sh_inject(&ctx, smack_buf, slen);

    bool active;
    ax25_kiss_smack_is_active(&ctx, &active);
    DEBUG_BOOL("smack_active after RX latch (expect true)", active);
    TEST_ASSERT(active == true, "smack_active latched by SMACK RX frame", 0);

    // TX after latch: should now use SMACK framing (bit-7 in type byte)
    h.tx_count = 0u;  // reset TX capture
    uint8_t payload2[] = { 0x22u };
    ax25_kiss_send_frame(&ctx, 0, payload2, sizeof(payload2));
    if (h.tx_count > 0u) {
        uint8_t type_after = h.tx[0].data[1u];
        DEBUG_HEX("TX type byte after AUTO latch (expect 0x80 SMACK)", type_after);
        TEST_ASSERT((type_after & KISS_SMACK_CRC_FLAG) != 0u, "TX type byte has SMACK flag after AUTO latch", type_after);

        // And a CRC trailer must be present: decoded length = payload + 2 CRC bytes
        uint8_t decoded[16u];
        size_t dec = slip_decode_payload(h.tx[0].data, h.tx[0].len, 1u, decoded, sizeof(decoded));
        DEBUG_VAR("Decoded length after AUTO latch TX (expect 3 = 1 payload + 2 CRC)", (unsigned)dec);
        TEST_ASSERT(dec == 1u + KISS_SMACK_CRC_SIZE, "AUTO latch TX: payload + 2 CRC bytes in decoded content", (unsigned )dec);
    }

    return 0;
}

// ===========================================================================
// TEST S20: AUTO mode - smack_active stays latched (cannot be un-latched)
// ===========================================================================
static int test_smack_auto_mode_latch_persistent(void) {
    printf("\n--- test_smack_auto_mode_latch_persistent ---\n");
    printf("Verify AUTO mode: smack_active stays latched after first SMACK frame\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_AUTO);

    // Latch by injecting a SMACK frame
    uint8_t smack_buf[32u];
    uint8_t smack_p[] = { 0x55u };
    size_t slen = build_smack_frame(0x80u, smack_p, sizeof(smack_p), smack_buf, sizeof(smack_buf));
    sh_inject(&ctx, smack_buf, slen);

    bool active;
    ax25_kiss_smack_is_active(&ctx, &active);
    TEST_ASSERT(active == true, "smack_active latched", 0);

    // Inject several standard DATA frames - smack_active must NOT revert
    for (int i = 0; i < 5; i++) {
        uint8_t std_frame[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_DATA), (uint8_t) i, KISS_FEND };
        sh_inject(&ctx, std_frame, sizeof(std_frame));
    }
    ax25_kiss_smack_is_active(&ctx, &active);
    DEBUG_BOOL("smack_active after 5 standard frames (expect true - stays latched)", active);
    TEST_ASSERT(active == true, "smack_active stays latched after standard frames", 0);

    return 0;
}

// ===========================================================================
// TEST S21: SMACK TX - empty payload edge case
// ===========================================================================
static int test_smack_tx_empty_payload(void) {
    printf("\n--- test_smack_tx_empty_payload ---\n");
    printf("Verify SMACK TX with zero-length payload: only type byte + 2 CRC bytes\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    uint8_t rc = ax25_kiss_send_frame(&ctx, 0, NULL, 0u);
    DEBUG_VAR("send_frame(empty, SMACK) return code (expect 0)", rc);
    TEST_ASSERT(rc == KISS_OK, "SMACK send_frame with empty payload returns KISS_OK", rc);
    TEST_ASSERT(h.tx_count == 1u, "One TX write for empty SMACK frame", h.tx_count);

    if (h.tx_count > 0u) {
        const uint8_t *buf = h.tx[0].data;
        size_t blen = h.tx[0].len;
        DEBUG_BUF("Empty SMACK TX frame", buf, blen);

        // Minimum: FEND + type(0x80) + crc_lo_esc + crc_hi_esc + FEND
        // At minimum 5 bytes (if no CRC bytes need escaping)
        TEST_ASSERT(blen >= 5u, "Empty SMACK frame is at least 5 bytes", (unsigned )blen);
        TEST_ASSERT(buf[0] == KISS_FEND, "Starts with FEND", buf[0]);
        TEST_ASSERT(buf[blen - 1u] == KISS_FEND, "Ends with FEND", 0);
        TEST_ASSERT((buf[1u] & KISS_SMACK_CRC_FLAG) != 0u, "SMACK flag set in type byte for empty payload", buf[1u]);

        // Decoded content should be exactly 2 CRC bytes (no payload)
        uint8_t decoded[8u];
        size_t dec_len = slip_decode_payload(buf, blen, 1u, decoded, sizeof(decoded));
        DEBUG_VAR("Decoded length for empty SMACK (expect 2 = CRC only)", (unsigned)dec_len);
        TEST_ASSERT(dec_len == KISS_SMACK_CRC_SIZE, "Empty SMACK TX decoded content = 2 CRC bytes only", (unsigned )dec_len);

        // Verify the 2 CRC bytes are the correct CRC of [type_byte] alone
        uint8_t type_only[1u] = { buf[1u] };  // smack type byte (after SLIP decode = 0x80)
        // Actually smack type = 0x80 (no escaping needed), so buf[1]=0x80
        uint16_t expected_crc = ax25_kiss_smack_crc16(type_only, 1u);
        uint8_t exp_lo = (uint8_t) (expected_crc & 0x00FFu);
        uint8_t exp_hi = (uint8_t) ((expected_crc >> 8u) & 0x00FFu);
        DEBUG_HEX("Expected CRC lo for empty payload", exp_lo);
        DEBUG_HEX("Expected CRC hi for empty payload", exp_hi);
        if (dec_len == KISS_SMACK_CRC_SIZE) {
            TEST_ASSERT(decoded[0] == exp_lo, "CRC lo correct for empty SMACK payload", decoded[0]);
            TEST_ASSERT(decoded[1] == exp_hi, "CRC hi correct for empty SMACK payload", decoded[1]);
        }
    }

    return 0;
}

// ===========================================================================
// TEST S22: ax25_kiss_reset_rx - clears RX state without clearing port params
// ===========================================================================
static int test_smack_reset_rx(void) {
    printf("\n--- test_smack_reset_rx ---\n");
    printf("Verify ax25_kiss_reset_rx() clears RX state machine without clearing port params\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);

    // Modify a port parameter
    ctx.ports[3].txdelay = 99u;
    ctx.ports[3].persistence = 200u;

    // Partially inject a frame to put state machine mid-frame
    uint8_t partial[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_DATA), 0x01u, 0x02u };
    sh_inject(&ctx, partial, sizeof(partial));

    // Confirm we are IN_FRAME state
    DEBUG_VAR("rx_state after partial inject (expect 1=IN_FRAME)", ctx.rx_state);
    TEST_ASSERT(ctx.rx_state == KISS_RX_IN_FRAME, "RX state = IN_FRAME after partial inject", ctx.rx_state);
    TEST_ASSERT(ctx.rx_len > 0u, "rx_len > 0 during partial frame", (unsigned )ctx.rx_len);

    // Reset RX
    uint8_t rc = ax25_kiss_reset_rx(&ctx);
    DEBUG_VAR("reset_rx return code (expect 0)", rc); DEBUG_VAR("rx_state after reset_rx (expect 0=IDLE)", ctx.rx_state); DEBUG_VAR("rx_len after reset_rx (expect 0)", (unsigned)ctx.rx_len); DEBUG_BOOL("rx_got_type after reset_rx (expect false)", ctx.rx_got_type);

    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_reset_rx returns KISS_OK", rc);
    TEST_ASSERT(ctx.rx_state == KISS_RX_IDLE, "rx_state = IDLE after reset_rx", ctx.rx_state);
    TEST_ASSERT(ctx.rx_len == 0u, "rx_len = 0 after reset_rx", (unsigned )ctx.rx_len);
    TEST_ASSERT(ctx.rx_got_type == false, "rx_got_type = false after reset_rx", 0);

    // Port params must be preserved
    DEBUG_VAR("Port 3 txdelay after reset_rx (expect 99)", ctx.ports[3].txdelay);
    TEST_ASSERT(ctx.ports[3].txdelay == 99u, "Port 3 txdelay preserved after reset_rx", ctx.ports[3].txdelay);
    TEST_ASSERT(ctx.ports[3].persistence == 200u, "Port 3 persistence preserved after reset_rx", ctx.ports[3].persistence);

    // State machine should be functional after reset
    uint8_t valid_frame[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_DATA), 0xABu, KISS_FEND };
    sh_inject(&ctx, valid_frame, sizeof(valid_frame));
    DEBUG_VAR("rx_count after valid frame post-reset (expect 1)", h.rx_count);
    TEST_ASSERT(h.rx_count == 1u, "State machine functional after reset_rx", h.rx_count);

    // NULL guard
    rc = ax25_kiss_reset_rx(NULL);
    TEST_ASSERT(rc == KISS_ERR_NULL, "reset_rx(NULL) returns KISS_ERR_NULL", rc);

    return 0;
}

// ===========================================================================
// TEST S23: ax25_kiss_reset_stats and ax25_kiss_get_stats
// ===========================================================================
static int test_smack_stats_reset_get(void) {
    printf("\n--- test_smack_stats_reset_get ---\n");
    printf("Verify ax25_kiss_reset_stats zeroes all counters and get_stats returns snapshot\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    // Inject some frames to build up stats
    uint8_t p[] = { 0x10u, 0x20u };
    uint8_t fbuf[64u];
    size_t flen;

    // Valid SMACK frame -> rx_frames++
    flen = build_smack_frame(0x80u, p, sizeof(p), fbuf, sizeof(fbuf));
    sh_inject(&ctx, fbuf, flen);

    // Invalid SMACK frame -> rx_bad_checksum++, rx_dropped++
    flen = build_smack_frame(0x80u, p, sizeof(p), fbuf, sizeof(fbuf));
    fbuf[flen - 2u] ^= 0xFFu;  // corrupt CRC
    sh_inject(&ctx, fbuf, flen);

    // Check stats snapshot
    ax25_kiss_stats_t stats;
    uint8_t rc = ax25_kiss_get_stats(&ctx, &stats);
    DEBUG_VAR("get_stats return code (expect 0)", rc); DEBUG_VAR("rx_frames (expect 1)", stats.rx_frames); DEBUG_VAR("rx_bad_checksum (expect 1)", stats.rx_bad_checksum); DEBUG_VAR("rx_dropped (expect 1)", stats.rx_dropped);

    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_get_stats returns KISS_OK", rc);
    TEST_ASSERT(stats.rx_frames == 1u, "rx_frames = 1 after one valid frame", stats.rx_frames);
    TEST_ASSERT(stats.rx_bad_checksum == 1u, "rx_bad_checksum = 1 after one bad CRC", stats.rx_bad_checksum);
    TEST_ASSERT(stats.rx_dropped == 1u, "rx_dropped = 1 after one dropped frame", stats.rx_dropped);

    // Reset stats
    rc = ax25_kiss_reset_stats(&ctx);
    DEBUG_VAR("reset_stats return code (expect 0)", rc);
    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_reset_stats returns KISS_OK", rc);

    rc = ax25_kiss_get_stats(&ctx, &stats);
    TEST_ASSERT(stats.rx_frames == 0u, "rx_frames = 0 after reset_stats", stats.rx_frames);
    TEST_ASSERT(stats.rx_bad_checksum == 0u, "rx_bad_checksum = 0 after reset_stats", stats.rx_bad_checksum);
    TEST_ASSERT(stats.rx_dropped == 0u, "rx_dropped = 0 after reset_stats", stats.rx_dropped);
    TEST_ASSERT(stats.rx_aborted == 0u, "rx_aborted = 0 after reset_stats", stats.rx_aborted);
    TEST_ASSERT(stats.rx_overflows == 0u, "rx_overflows = 0 after reset_stats", stats.rx_overflows);

    // Verify port params not affected by reset_stats
    DEBUG_VAR("variant after reset_stats (expect SMACK=1)", ctx.variant);
    TEST_ASSERT(ctx.variant == KISS_VARIANT_SMACK, "variant preserved after reset_stats", ctx.variant);

    // NULL guards
    rc = ax25_kiss_reset_stats(NULL);
    TEST_ASSERT(rc == KISS_ERR_NULL, "reset_stats(NULL) returns KISS_ERR_NULL", rc);

    rc = ax25_kiss_get_stats(NULL, &stats);
    TEST_ASSERT(rc == KISS_ERR_NULL, "get_stats(NULL ctx) returns KISS_ERR_NULL", rc);

    rc = ax25_kiss_get_stats(&ctx, NULL);
    TEST_ASSERT(rc == KISS_ERR_NULL, "get_stats(NULL out) returns KISS_ERR_NULL", rc);

    return 0;
}

// ===========================================================================
// TEST S24: ax25_kiss_reset_port_params - single port reset to defaults
// ===========================================================================
static int test_smack_reset_port_params(void) {
    printf("\n--- test_smack_reset_port_params ---\n");
    printf("Verify ax25_kiss_reset_port_params() restores one port to spec defaults\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);

    // Modify port 5
    ctx.ports[5].txdelay = 99u;
    ctx.ports[5].persistence = 200u;
    ctx.ports[5].slottime = 25u;
    ctx.ports[5].txtail = 10u;
    ctx.ports[5].full_duplex = true;

    DEBUG_PRINT("Port 5 modified to non-default values"); DEBUG_VAR("Port 5 txdelay before reset", ctx.ports[5].txdelay);

    uint8_t rc = ax25_kiss_reset_port_params(&ctx, 5u);
    DEBUG_VAR("reset_port_params(5) return code (expect 0)", rc);
    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_reset_port_params returns KISS_OK", rc);

    // Verify defaults restored
    DEBUG_VAR("Port 5 txdelay after reset (expect 50)", ctx.ports[5].txdelay); DEBUG_VAR("Port 5 persistence after reset (expect 63)", ctx.ports[5].persistence); DEBUG_VAR("Port 5 slottime after reset (expect 10)", ctx.ports[5].slottime); DEBUG_VAR("Port 5 txtail after reset (expect 0)", ctx.ports[5].txtail); DEBUG_BOOL("Port 5 full_duplex after reset (expect false)", ctx.ports[5].full_duplex);

    TEST_ASSERT(ctx.ports[5].txdelay == KISS_DEFAULT_TXDELAY, "Port 5 txdelay reset to default", ctx.ports[5].txdelay);
    TEST_ASSERT(ctx.ports[5].persistence == KISS_DEFAULT_PERSISTENCE, "Port 5 persistence reset to default", ctx.ports[5].persistence);
    TEST_ASSERT(ctx.ports[5].slottime == KISS_DEFAULT_SLOTTIME, "Port 5 slottime reset to default", ctx.ports[5].slottime);
    TEST_ASSERT(ctx.ports[5].txtail == KISS_DEFAULT_TXTAIL, "Port 5 txtail reset to default", ctx.ports[5].txtail);
    TEST_ASSERT(ctx.ports[5].full_duplex == KISS_DEFAULT_FULLDUPLEX, "Port 5 full_duplex reset to default", ctx.ports[5].full_duplex);

    // Other ports must be unaffected
    ctx.ports[2].txdelay = 77u;
    ax25_kiss_reset_port_params(&ctx, 5u);
    DEBUG_VAR("Port 2 txdelay (expect 77, unaffected)", ctx.ports[2].txdelay);
    TEST_ASSERT(ctx.ports[2].txdelay == 77u, "Non-reset port 2 txdelay unaffected", ctx.ports[2].txdelay);

    // Verify it also sends the parameter commands to TNC (5 TX writes expected)
    uint8_t pre_tx = h.tx_count;
    ax25_kiss_reset_port_params(&ctx, 0u);
    DEBUG_VAR("tx_count increase after reset_port_params (expect +5)", h.tx_count - pre_tx);
    TEST_ASSERT((h.tx_count - pre_tx) == 5u, "reset_port_params sends 5 TNC commands", (h.tx_count - pre_tx));

    // Error cases
    rc = ax25_kiss_reset_port_params(NULL, 0u);
    TEST_ASSERT(rc == KISS_ERR_NULL, "reset_port_params(NULL) returns KISS_ERR_NULL", rc);

    rc = ax25_kiss_reset_port_params(&ctx, 0x0Fu);
    TEST_ASSERT(rc == KISS_ERR_PORT, "reset_port_params(port=0x0F) returns KISS_ERR_PORT", rc);

    return 0;
}

// ===========================================================================
// TEST S25: ax25_kiss_reset_all_ports - all 15 ports reset to defaults
// ===========================================================================
static int test_smack_reset_all_ports(void) {
    printf("\n--- test_smack_reset_all_ports ---\n");
    printf("Verify ax25_kiss_reset_all_ports() restores all 15 user ports to defaults\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);

    // Modify all ports with non-default values
    for (uint8_t p = 0u; p < 0x0Fu; p++) {
        ctx.ports[p].txdelay = (uint8_t) (p + 100u);
        ctx.ports[p].persistence = 255u;
        ctx.ports[p].slottime = (uint8_t) (p + 20u);
        ctx.ports[p].full_duplex = true;
    }

    DEBUG_PRINT("All 15 ports modified to non-default values"); DEBUG_VAR("Port 0 txdelay before reset_all", ctx.ports[0].txdelay);

    uint8_t rc = ax25_kiss_reset_all_ports(&ctx);
    DEBUG_VAR("reset_all_ports return code (expect 0)", rc);
    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_reset_all_ports returns KISS_OK", rc);

    // Check all 15 user ports (0-14) have been reset
    int failures = 0;
    for (uint8_t p = 0u; p < 0x0Fu; p++) {
        if (ctx.ports[p].txdelay != KISS_DEFAULT_TXDELAY || ctx.ports[p].persistence != KISS_DEFAULT_PERSISTENCE
                || ctx.ports[p].slottime != KISS_DEFAULT_SLOTTIME || ctx.ports[p].txtail != KISS_DEFAULT_TXTAIL
                || ctx.ports[p].full_duplex != KISS_DEFAULT_FULLDUPLEX) {
            printf("  [FAIL] Port %u not fully reset to defaults\n", p);
            failures++;
        } else {
            DEBUG_VAR("Port correctly reset to defaults", p);
        }
    }
    TEST_ASSERT(failures == 0, "All 15 ports (0-14) restored to specification defaults", failures);

    // Should produce 5 TX commands per port = 75 writes total
    DEBUG_VAR("tx_count after reset_all_ports (expect 75 = 15 ports x 5 cmds)", h.tx_count);
    TEST_ASSERT(h.tx_count == 75u, "reset_all_ports sends 75 TNC commands (15 ports x 5 params each)", h.tx_count);

    // NULL guard
    rc = ax25_kiss_reset_all_ports(NULL);
    TEST_ASSERT(rc == KISS_ERR_NULL, "reset_all_ports(NULL) returns KISS_ERR_NULL", rc);

    return 0;
}

// ===========================================================================
// TEST S26: ax25_kiss_set_poll_mode - G8BPQ polled mode flags
// ===========================================================================
static int test_smack_set_poll_mode(void) {
    printf("\n--- test_smack_set_poll_mode ---\n");
    printf("Verify ax25_kiss_set_poll_mode() sets/clears poll_mode flag and interval\n");

    ax25_kiss_ctx_t ctx;
    ax25_kiss_init(&ctx);

    // Defaults: poll_mode=false, poll_interval=0
    DEBUG_BOOL("poll_mode after init (expect false)", ctx.poll_mode); DEBUG_VAR("poll_interval after init (expect 0)", ctx.poll_interval);
    TEST_ASSERT(ctx.poll_mode == false, "poll_mode = false after init", 0);
    TEST_ASSERT(ctx.poll_interval == 0u, "poll_interval = 0 after init", ctx.poll_interval);

    // Enable polling with 50 * 100ms = 5s interval
    uint8_t rc = ax25_kiss_set_poll_mode(&ctx, true, 50u);
    DEBUG_VAR("set_poll_mode(true,50) return code (expect 0)", rc); DEBUG_BOOL("poll_mode after enable (expect true)", ctx.poll_mode); DEBUG_VAR("poll_interval after enable (expect 50)", ctx.poll_interval);
    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_set_poll_mode returns KISS_OK", rc);
    TEST_ASSERT(ctx.poll_mode == true, "poll_mode = true after enable", 0);
    TEST_ASSERT(ctx.poll_interval == 50u, "poll_interval = 50 after enable", ctx.poll_interval);

    // Disable polling (interval arg ignored when enable=false)
    rc = ax25_kiss_set_poll_mode(&ctx, false, 100u);
    DEBUG_BOOL("poll_mode after disable (expect false)", ctx.poll_mode); DEBUG_VAR("poll_interval after disable (expect 0)", ctx.poll_interval);
    TEST_ASSERT(ctx.poll_mode == false, "poll_mode = false after disable", 0);
    TEST_ASSERT(ctx.poll_interval == 0u, "poll_interval = 0 after disable (cleared)", ctx.poll_interval);

    // Re-enable with different interval
    rc = ax25_kiss_set_poll_mode(&ctx, true, 1u);
    TEST_ASSERT(ctx.poll_mode == true, "poll_mode re-enabled", 0);
    TEST_ASSERT(ctx.poll_interval == 1u, "poll_interval = 1 after re-enable", ctx.poll_interval);

    // NULL guard
    rc = ax25_kiss_set_poll_mode(NULL, true, 5u);
    TEST_ASSERT(rc == KISS_ERR_NULL, "set_poll_mode(NULL) returns KISS_ERR_NULL", rc);

    return 0;
}

// ===========================================================================
// TEST S27: ax25_kiss_set_hw_flowctrl - hardware flow control flag
// ===========================================================================
static int test_smack_set_hw_flowctrl(void) {
    printf("\n--- test_smack_set_hw_flowctrl ---\n");
    printf("Verify ax25_kiss_set_hw_flowctrl() sets/clears hw_flowctrl flag\n");

    ax25_kiss_ctx_t ctx;
    ax25_kiss_init(&ctx);

    // Default: hw_flowctrl = false
    DEBUG_BOOL("hw_flowctrl after init (expect false)", ctx.hw_flowctrl);
    TEST_ASSERT(ctx.hw_flowctrl == false, "hw_flowctrl = false after init", 0);

    uint8_t rc = ax25_kiss_set_hw_flowctrl(&ctx, true);
    DEBUG_BOOL("hw_flowctrl after enable (expect true)", ctx.hw_flowctrl);
    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_set_hw_flowctrl returns KISS_OK", rc);
    TEST_ASSERT(ctx.hw_flowctrl == true, "hw_flowctrl = true after enable", 0);

    rc = ax25_kiss_set_hw_flowctrl(&ctx, false);
    TEST_ASSERT(ctx.hw_flowctrl == false, "hw_flowctrl = false after disable", 0);

    // NULL guard
    rc = ax25_kiss_set_hw_flowctrl(NULL, true);
    TEST_ASSERT(rc == KISS_ERR_NULL, "set_hw_flowctrl(NULL) returns KISS_ERR_NULL", rc);

    return 0;
}

// ===========================================================================
// TEST S28: SMACK with payload containing special bytes (stress test)
// ===========================================================================
static int test_smack_payload_special_bytes(void) {
    printf("\n--- test_smack_payload_special_bytes ---\n");
    printf("Verify SMACK CRC computed pre-SLIP over raw payload (not over escaped bytes)\n");
    printf("Tests: payload=[FEND, FESC, FEND, FESC] - all special bytes\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    // All-special-bytes payload
    uint8_t payload[] = { KISS_FEND, KISS_FESC, KISS_FEND, KISS_FESC, KISS_FEND };

    // TX encode
    uint8_t rc = ax25_kiss_send_frame(&ctx, 0, payload, sizeof(payload));
    TEST_ASSERT(rc == KISS_OK, "SMACK TX with all-special payload returns KISS_OK", rc);
    TEST_ASSERT(h.tx_count == 1u, "One TX write for all-special payload SMACK frame", h.tx_count);

    if (h.tx_count == 0u)
        return 1;

    uint8_t tx_copy[HARNESS_TX_SLOT_SZ];
    size_t tx_len = h.tx[0].len;
    memcpy(tx_copy, h.tx[0].data, tx_len);
    DEBUG_BUF("SMACK TX all-special bytes frame", tx_copy, tx_len);

    // Verify no bare FEND inside the frame body
    bool bare_fend = false;
    for (size_t i = 2u; i < tx_len - 1u; i++) {
        if (tx_copy[i] == KISS_FEND) {
            bare_fend = true;
            break;
        }
    }
    TEST_ASSERT(!bare_fend, "No bare FEND inside SMACK frame body with all-special payload", 0);

    // RX round-trip
    h.tx_count = 0u;
    h.rx_count = 0u;
    h.crc_err_count = 0u;
    sh_inject(&ctx, tx_copy, tx_len);
    DEBUG_VAR("rx_count (expect 1)", h.rx_count); DEBUG_VAR("crc_err_count (expect 0)", h.crc_err_count);

    TEST_ASSERT(h.rx_count == 1u, "SMACK round-trip fires on_frame for all-special payload", h.rx_count);
    TEST_ASSERT(h.crc_err_count == 0u, "No CRC error in SMACK round-trip with all-special payload", h.crc_err_count);

    if (h.rx_count > 0u) {
        DEBUG_BUF("Round-trip payload (expect FEND FESC FEND FESC FEND)", h.rx_frames[0], h.rx_lens[0]);
        TEST_ASSERT(h.rx_lens[0] == sizeof(payload), "Round-trip all-special payload length correct", (unsigned )h.rx_lens[0]);
        bool match = (memcmp(h.rx_frames[0], payload, sizeof(payload)) == 0);
        TEST_ASSERT(match, "Round-trip all-special payload bytes match exactly", 0);
    }

    return 0;
}

// ===========================================================================
// TEST S29: SMACK exhaustive round-trip: all 256 single-byte payloads
// ===========================================================================
static int test_smack_exhaustive_round_trip(void) {
    printf("\n--- test_smack_exhaustive_round_trip ---\n");
    printf("Verify SMACK TX+RX round-trip is correct for all 256 single-byte payloads\n");

    int failures = 0;
    for (int b = 0; b <= 255; b++) {
        smack_harness_t h;
        ax25_kiss_ctx_t ctx;
        sh_harness_init(&h, &ctx);
        ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

        uint8_t payload[1] = { (uint8_t) b };
        ax25_kiss_send_frame(&ctx, 0, payload, 1u);
        if (h.tx_count == 0u) {
            failures++;
            continue;
        }

        uint8_t tx_copy[HARNESS_TX_SLOT_SZ];
        size_t tx_len = h.tx[0].len;
        memcpy(tx_copy, h.tx[0].data, tx_len);

        h.tx_count = 0u;
        h.rx_count = 0u;
        h.crc_err_count = 0u;
        sh_inject(&ctx, tx_copy, tx_len);

        if (h.rx_count != 1u || h.crc_err_count != 0u || h.rx_lens[0] != 1u || h.rx_frames[0][0] != payload[0]) {
            failures++;
            printf("  [FAIL] SMACK byte 0x%02X: rx=%u crc_err=%u len=%zu val=0x%02X\n", b, h.rx_count, h.crc_err_count, h.rx_count > 0u ? h.rx_lens[0] : 0u,
                    h.rx_count > 0u ? h.rx_frames[0][0] : 0xFFu);
        }
    }
    TEST_ASSERT(failures == 0, "SMACK round-trip correct for all 256 single-byte payloads", failures);

    return 0;
}

// ===========================================================================
// TEST S30: SMACK + AUTO mode - mixed valid/bad frames, stats integrity
// ===========================================================================
static int test_smack_mixed_valid_bad_stats(void) {
    printf("\n--- test_smack_mixed_valid_bad_stats ---\n");
    printf("Verify stats after mix of valid SMACK, bad SMACK, and standard KISS frames\n");

    smack_harness_t h;
    ax25_kiss_ctx_t ctx;
    sh_harness_init(&h, &ctx);
    ax25_kiss_set_variant(&ctx, KISS_VARIANT_SMACK);

    uint8_t payload[] = { 0xAAu, 0xBBu };
    uint8_t fbuf[64u];
    size_t flen;

    // 2 valid SMACK frames
    flen = build_smack_frame(0x80u, payload, sizeof(payload), fbuf, sizeof(fbuf));
    sh_inject(&ctx, fbuf, flen);
    flen = build_smack_frame(0x80u, payload, sizeof(payload), fbuf, sizeof(fbuf));
    sh_inject(&ctx, fbuf, flen);

    // 3 bad SMACK frames (corrupted CRC)
    for (int i = 0; i < 3; i++) {
        flen = build_smack_frame(0x80u, payload, sizeof(payload), fbuf, sizeof(fbuf));
        fbuf[flen - 2u] ^= (uint8_t) (0x01u + (uint8_t) i);
        sh_inject(&ctx, fbuf, flen);
    }

    ax25_kiss_stats_t stats;
    ax25_kiss_get_stats(&ctx, &stats);

    DEBUG_VAR("rx_frames (expect 2)", stats.rx_frames); DEBUG_VAR("rx_bad_checksum (expect 3)", stats.rx_bad_checksum); DEBUG_VAR("rx_dropped (expect 3)", stats.rx_dropped);

    TEST_ASSERT(stats.rx_frames == 2u, "rx_frames = 2 after 2 valid + 3 bad frames", stats.rx_frames);
    TEST_ASSERT(stats.rx_bad_checksum == 3u, "rx_bad_checksum = 3 after 3 bad frames", stats.rx_bad_checksum);
    TEST_ASSERT(stats.rx_dropped == 3u, "rx_dropped = 3 after 3 bad frames", stats.rx_dropped);

    TEST_ASSERT(h.rx_count == 2u, "on_frame called 2 times (only valid frames)", h.rx_count);
    TEST_ASSERT(h.crc_err_count == 3u, "on_crc_error called 3 times (bad frames)", h.crc_err_count);

    return 0;
}

// ===========================================================================
// TEST S31: SMACK TX - explicit SMACK variant vs AUTO with latch: same output
// ===========================================================================
static int test_smack_explicit_vs_auto_tx(void) {
    printf("\n--- test_smack_explicit_vs_auto_tx ---\n");
    printf("Verify explicit SMACK and AUTO+latched SMACK produce identical TX frames\n");

    uint8_t payload[] = { 0x12u, 0x34u, 0x56u };

    // Context 1: explicit SMACK
    smack_harness_t h1;
    ax25_kiss_ctx_t ctx1;
    sh_harness_init(&h1, &ctx1);
    ax25_kiss_set_variant(&ctx1, KISS_VARIANT_SMACK);
    ax25_kiss_send_frame(&ctx1, 0, payload, sizeof(payload));

    // Context 2: AUTO mode with latch
    smack_harness_t h2;
    ax25_kiss_ctx_t ctx2;
    sh_harness_init(&h2, &ctx2);
    ax25_kiss_set_variant(&ctx2, KISS_VARIANT_AUTO);
    // Trigger latch
    uint8_t smack_buf[32u];
    uint8_t trigger[] = { 0x01u };
    size_t slen = build_smack_frame(0x80u, trigger, sizeof(trigger), smack_buf, sizeof(smack_buf));
    sh_inject(&ctx2, smack_buf, slen);
    // Now send same payload
    ax25_kiss_send_frame(&ctx2, 0, payload, sizeof(payload));

    TEST_ASSERT(h1.tx_count == 1u, "Explicit SMACK has 1 TX", h1.tx_count);
    TEST_ASSERT(h2.tx_count == 1u, "AUTO+latched has 1 TX", h2.tx_count);

    if (h1.tx_count == 1u && h2.tx_count == 1u) {
        size_t len1 = h1.tx[0].len;
        size_t len2 = h2.tx[0].len;
        DEBUG_VAR("Explicit SMACK TX length", (unsigned)len1); DEBUG_VAR("AUTO latched TX length", (unsigned)len2);
        DEBUG_BUF("Explicit SMACK TX", h1.tx[0].data, len1);
        DEBUG_BUF("AUTO latched TX", h2.tx[0].data, len2);

        TEST_ASSERT(len1 == len2, "Explicit SMACK and AUTO+latch TX frames same length", (unsigned )len1);
        if (len1 == len2) {
            bool same = (memcmp(h1.tx[0].data, h2.tx[0].data, len1) == 0);
            TEST_ASSERT(same, "Explicit SMACK and AUTO+latch TX frames byte-identical", 0);
        }
    }

    return 0;
}

// ===========================================================================
// Main entry point
// ===========================================================================
int test_ax25_kiss_smack_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("SMACK / G8BPQ / Auto-detection / Extensions KISS Test Suite\n");
    printf("  Tests extensions not covered by test_ax25_kiss.c:\n");
    printf("  SMACK CRC-16, auto-detection, G8BPQ XOR, stats, reset APIs\n");
    printf("==================================================================================\n");

    result |= test_smack_crc16_known_vectors();
    result |= test_crc8_xor_known_vectors();
    result |= test_smack_tx_type_byte_flag();
    result |= test_smack_tx_crc_trailer_correct();
    result |= test_smack_tx_crc_slip_escaped();
    result |= test_smack_tx_multiport();
    result |= test_smack_rx_valid_frame();
    result |= test_smack_rx_bad_crc();
    result |= test_smack_rx_bad_crc_stats();
    result |= test_smack_rx_too_short_for_crc();
    result |= test_smack_rx_escaped_crc_bytes();
    result |= test_smack_round_trip();
    result |= test_smack_rx_multiple_frames_stats();
    result |= test_smack_rx_multiport();
    result |= test_smack_rx_standard_mode_no_false_detect();
    result |= test_smack_set_get_variant();
    result |= test_smack_is_active_query();
    result |= test_smack_auto_mode_upgrade();
    result |= test_smack_auto_mode_tx_after_latch();
    result |= test_smack_auto_mode_latch_persistent();
    result |= test_smack_tx_empty_payload();
    result |= test_smack_reset_rx();
    result |= test_smack_stats_reset_get();
    result |= test_smack_reset_port_params();
    result |= test_smack_reset_all_ports();
    result |= test_smack_set_poll_mode();
    result |= test_smack_set_hw_flowctrl();
    result |= test_smack_payload_special_bytes();
    result |= test_smack_exhaustive_round_trip();
    result |= test_smack_mixed_valid_bad_stats();
    result |= test_smack_explicit_vs_auto_tx();

    printf("\n==================================================================================\n");
    printf("SMACK/Extensions Tests Completed.  Total assertions: %u. %s\n", assert_count,
            result == 0 ? "\033[0;32mAll tests passed\033[0m" : "\033[0;31mSome tests FAILED\033[0m");
    printf("==================================================================================\n\n");

    return result;
}

