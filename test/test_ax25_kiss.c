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

// KISS TNC Interface Protocol comprehensive test suite
// Tests all KISS scenarios per the KISS specification (Chepponis/Karn 1987):
//   - Section: Init and enter KISS mode
//   - Section: TX path - raw data frame with KISS framing (FEND wrapping)
//   - Section: TX path - byte stuffing of FEND (0xC0) in payload
//   - Section: TX path - byte stuffing of FESC (0xDB) in payload
//   - Section: TX path - consecutive special bytes in payload
//   - Section: TX path - all 256 byte values in payload (exhaustive stuffing)
//   - Section: RX path - state machine IDLE -> IN_FRAME -> dispatch
//   - Section: RX path - escape sequence decoding FESC+TFEND -> FEND
//   - Section: RX path - escape sequence decoding FESC+TFESC -> FESC
//   - Section: RX path - invalid escape sequence handling
//   - Section: RX path - consecutive FEND (empty frame discarded)
//   - Section: RX path - partial / fragmented byte delivery
//   - Section: RX path - buffer overflow protection
//   - Section: Commands - TXDELAY, PERSISTENCE, SLOTTIME, TXTAIL, FULLDUPLEX
//   - Section: Commands - SETHARDWARE with callback
//   - Section: Commands - RETURN command exits KISS mode
//   - Section: Multi-port - port nibble extraction and dispatch
//   - Section: Port parameters - get/set round-trip
//   - Section: Error codes - NULL, no-serial, port range, frame size
//   - Section: Kiss mode flag - transitions on enter/return
//   - Section: Round-trip - encode then decode a frame

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "kiss.h"
#include "test_common.h"

// -------------------------------------------------------------------------
// Debug infrastructure
// -------------------------------------------------------------------------

#define DEBUG_ENABLE

// -------------------------------------------------------------------------
// Test assertion macro
// -------------------------------------------------------------------------

static uint32_t assert_count = 0;

// -------------------------------------------------------------------------
// Test harness - captures serial output and callback events
// -------------------------------------------------------------------------

#define HARNESS_TX_SLOTS  64
#define HARNESS_TX_SLOT_SZ 1200   /* worst-case KISS encoded: 512*2+4 = 1028 */

typedef struct {
    uint8_t data[HARNESS_TX_SLOT_SZ];
    size_t len;
} kiss_captured_buf_t;

typedef struct {
    // Serial output capture
    kiss_captured_buf_t tx[HARNESS_TX_SLOTS];
    uint8_t tx_count;

    // on_frame callback captures
    uint8_t rx_port[HARNESS_TX_SLOTS];
    uint8_t rx_frames[HARNESS_TX_SLOTS][KISS_MAX_FRAME_SIZE];
    size_t rx_lens[HARNESS_TX_SLOTS];
    uint8_t rx_count;

    // on_hardware callback captures
    uint8_t hw_port[HARNESS_TX_SLOTS];
    uint8_t hw_data[HARNESS_TX_SLOTS][64];
    size_t hw_lens[HARNESS_TX_SLOTS];
    uint8_t hw_count;

    // on_return callback
    uint8_t return_count;
} kiss_harness_t;

// -------------------------------------------------------------------------
// Callback implementations
// -------------------------------------------------------------------------

static void harness_serial_write(uint8_t *data, size_t len, void *user_data) {
    kiss_harness_t *h = (kiss_harness_t*) user_data;
    if (!h || h->tx_count >= HARNESS_TX_SLOTS)
        return;
    size_t copy = (len > HARNESS_TX_SLOT_SZ) ? HARNESS_TX_SLOT_SZ : len;
    memcpy(h->tx[h->tx_count].data, data, copy);
    h->tx[h->tx_count].len = copy;
    h->tx_count++;
}

static void harness_on_frame(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t *frame, size_t len, void *user_data) {
    (void) ctx;
    kiss_harness_t *h = (kiss_harness_t*) user_data;
    if (!h || h->rx_count >= HARNESS_TX_SLOTS)
        return;
    uint8_t idx = h->rx_count;
    h->rx_port[idx] = port;
    size_t copy = (len > KISS_MAX_FRAME_SIZE) ? KISS_MAX_FRAME_SIZE : len;
    memcpy(h->rx_frames[idx], frame, copy);
    h->rx_lens[idx] = copy;
    h->rx_count++;
}

static void harness_on_hardware(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t *data, size_t len, void *user_data) {
    (void) ctx;
    kiss_harness_t *h = (kiss_harness_t*) user_data;
    if (!h || h->hw_count >= HARNESS_TX_SLOTS)
        return;
    uint8_t idx = h->hw_count;
    h->hw_port[idx] = port;
    size_t copy = (len > 64) ? 64 : len;
    memcpy(h->hw_data[idx], data, copy);
    h->hw_lens[idx] = copy;
    h->hw_count++;
}

static void harness_on_return(ax25_kiss_ctx_t *ctx, void *user_data) {
    (void) ctx;
    kiss_harness_t *h = (kiss_harness_t*) user_data;
    if (!h)
        return;
    h->return_count++;
}

// -------------------------------------------------------------------------
// Helper: initialize a context with all callbacks wired to a harness
// -------------------------------------------------------------------------

static void harness_init(kiss_harness_t *h, ax25_kiss_ctx_t *ctx) {
    memset(h, 0, sizeof(*h));
    uint8_t rc = ax25_kiss_init(ctx);
    (void) rc; /* checked by caller if needed */
    ctx->serial_write = harness_serial_write;
    ctx->on_frame = harness_on_frame;
    ctx->on_hardware = harness_on_hardware;
    ctx->on_return = harness_on_return;
    ctx->user_data = h;
}

// -------------------------------------------------------------------------
// Helper: inject a fully-formed raw KISS byte stream into the RX engine
// -------------------------------------------------------------------------
static void inject(ax25_kiss_ctx_t *ctx, const uint8_t *bytes, size_t len) {
    ax25_kiss_receive_bytes(ctx, bytes, len);
}

// -------------------------------------------------------------------------
// Helper: verify last TX output has the expected KISS structure for a DATA frame
// Returns true on success.
// -------------------------------------------------------------------------
static bool verify_kiss_data_frame(const kiss_harness_t *h, uint8_t expected_port, const uint8_t *expected_payload, size_t expected_len, uint8_t tx_slot) {
    if (tx_slot >= h->tx_count) {
        printf("  [DBG] verify_kiss_data_frame: slot %u not found (tx_count=%u)\n", tx_slot, h->tx_count);
        return false;
    }
    const uint8_t *buf = h->tx[tx_slot].data;
    size_t blen = h->tx[tx_slot].len;

    DEBUG_BUF("verify_kiss_data_frame TX bytes", buf, blen);

    // Must start and end with FEND
    if (blen < 3) {
        DEBUG_PRINT("TX buf too short");
        return false;
    }
    if (buf[0] != KISS_FEND) {
        DEBUG_PRINT("Missing opening FEND");
        return false;
    }
    if (buf[blen - 1] != KISS_FEND) {
        DEBUG_PRINT("Missing closing FEND");
        return false;
    }

    // Type byte
    uint8_t type_byte = buf[1];
    DEBUG_HEX("type_byte in KISS frame", type_byte);
    if (KISS_PORT(type_byte) != expected_port) {
        DEBUG_PRINT("Port mismatch");
        return false;
    }
    if (KISS_CMD(type_byte) != KISS_CMD_DATA) {
        DEBUG_PRINT("CMD not DATA");
        return false;
    }

    // Decode stuffed payload
    uint8_t decoded[KISS_MAX_FRAME_SIZE];
    size_t dec_len = 0;
    bool in_esc = false;
    for (size_t i = 2; i < blen - 1; i++) {
        uint8_t b = buf[i];
        if (in_esc) {
            if (b == KISS_TFEND) {
                decoded[dec_len++] = KISS_FEND;
            } else if (b == KISS_TFESC) {
                decoded[dec_len++] = KISS_FESC;
            } else {
                return false; /* invalid escape */
            }
            in_esc = false;
        } else if (b == KISS_FESC) {
            in_esc = true;
        } else {
            decoded[dec_len++] = b;
        }
    }
    if (dec_len != expected_len) {
        DEBUG_VAR("decoded length", dec_len);DEBUG_VAR("expected length", expected_len);
        return false;
    }
    return memcmp(decoded, expected_payload, expected_len) == 0;
}

// ==========================================================================
// TEST 1: ax25_kiss_init - context zeroed and defaults applied
// ==========================================================================
static int test_kiss_init_defaults(void) {
    printf("\n--- test_kiss_init_defaults ---\n");
    printf("Verify ax25_kiss_init() sets specification default parameters on all ports\n");

    ax25_kiss_ctx_t ctx;
    uint8_t rc = ax25_kiss_init(&ctx);

    DEBUG_VAR("ax25_kiss_init return code (expect KISS_OK=0)", rc);DEBUG_BOOL("kiss_mode after init (expect false)", ctx.kiss_mode);DEBUG_VAR("rx_state after init (expect KISS_RX_IDLE=0)", ctx.rx_state);DEBUG_VAR("rx_len after init (expect 0)", ctx.rx_len);DEBUG_BOOL("rx_got_type after init (expect false)", ctx.rx_got_type);

    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_init returns KISS_OK", rc);
    TEST_ASSERT(ctx.kiss_mode == false, "kiss_mode is false after init", 0);
    TEST_ASSERT(ctx.rx_state == KISS_RX_IDLE, "rx_state is KISS_RX_IDLE", ctx.rx_state);
    TEST_ASSERT(ctx.rx_len == 0, "rx_len is 0", 0);
    TEST_ASSERT(ctx.rx_got_type == false, "rx_got_type is false", 0);
    TEST_ASSERT(ctx.serial_write == NULL, "serial_write is NULL", 0);
    TEST_ASSERT(ctx.on_frame == NULL, "on_frame is NULL", 0);
    TEST_ASSERT(ctx.on_hardware == NULL, "on_hardware is NULL", 0);
    TEST_ASSERT(ctx.on_return == NULL, "on_return is NULL", 0);
    TEST_ASSERT(ctx.user_data == NULL, "user_data is NULL", 0);

    for (uint8_t i = 0; i < KISS_MAX_PORTS; i++) {
        DEBUG_VAR("Port txdelay", ctx.ports[i].txdelay);DEBUG_VAR("Port persistence", ctx.ports[i].persistence);DEBUG_VAR("Port slottime", ctx.ports[i].slottime);DEBUG_VAR("Port txtail", ctx.ports[i].txtail);DEBUG_BOOL("Port full_duplex", ctx.ports[i].full_duplex);
        TEST_ASSERT(ctx.ports[i].txdelay == KISS_DEFAULT_TXDELAY, "Default txdelay on port", i);
        TEST_ASSERT(ctx.ports[i].persistence == KISS_DEFAULT_PERSISTENCE, "Default persistence on port", i);
        TEST_ASSERT(ctx.ports[i].slottime == KISS_DEFAULT_SLOTTIME, "Default slottime on port", i);
        TEST_ASSERT(ctx.ports[i].txtail == KISS_DEFAULT_TXTAIL, "Default txtail on port", i);
        TEST_ASSERT(ctx.ports[i].full_duplex == KISS_DEFAULT_FULLDUPLEX, "Default full_duplex on port", i);
    }

    // NULL init must return error
    rc = ax25_kiss_init(NULL);
    DEBUG_VAR("ax25_kiss_init(NULL) return code (expect KISS_ERR_NULL=1)", rc);
    TEST_ASSERT(rc == KISS_ERR_NULL, "ax25_kiss_init(NULL) returns KISS_ERR_NULL", rc);

    return 0;
}

// ==========================================================================
// TEST 2: ax25_kiss_enter - FEND sent, kiss_mode set
// ==========================================================================
static int test_kiss_enter(void) {
    printf("\n--- test_kiss_enter ---\n");
    printf("Verify ax25_kiss_enter() transmits a leading FEND and sets kiss_mode=true\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    DEBUG_BOOL("kiss_mode before enter", ctx.kiss_mode);

    uint8_t rc = ax25_kiss_enter(&ctx);

    DEBUG_VAR("ax25_kiss_enter return code (expect 0)", rc);DEBUG_BOOL("kiss_mode after enter (expect true)", ctx.kiss_mode);DEBUG_VAR("tx_count after enter (expect 1)", h.tx_count);

    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_enter returns KISS_OK", rc);
    TEST_ASSERT(ctx.kiss_mode == true, "kiss_mode is true after enter", 0);
    TEST_ASSERT(h.tx_count == 1, "One serial write performed for FEND flush", 0);
    if (h.tx_count > 0) {
        DEBUG_BUF("enter TX bytes", h.tx[0].data, h.tx[0].len);
        TEST_ASSERT(h.tx[0].len == 1, "Enter FEND is 1 byte", (unsigned int )h.tx[0].len);
        TEST_ASSERT(h.tx[0].data[0] == KISS_FEND, "Enter byte is FEND (0xC0)", (unsigned int )h.tx[0].data[0]);
    }

    // NULL and no-serial error cases
    rc = ax25_kiss_enter(NULL);
    TEST_ASSERT(rc == KISS_ERR_NULL, "ax25_kiss_enter(NULL) returns KISS_ERR_NULL", rc);

    ax25_kiss_ctx_t ctx2;
    ax25_kiss_init(&ctx2); /* serial_write intentionally left NULL */
    rc = ax25_kiss_enter(&ctx2);
    DEBUG_VAR("ax25_kiss_enter without serial_write (expect KISS_ERR_NO_SERIAL=4)", rc);
    TEST_ASSERT(rc == KISS_ERR_NO_SERIAL, "ax25_kiss_enter without serial_write returns KISS_ERR_NO_SERIAL", rc);
    TEST_ASSERT(ctx2.kiss_mode == false, "kiss_mode stays false when no serial_write", 0);

    return 0;
}

// ==========================================================================
// TEST 3: ax25_kiss_send_frame - basic DATA frame framing
// ==========================================================================
static int test_kiss_send_frame_basic(void) {
    printf("\n--- test_kiss_send_frame_basic ---\n");
    printf("Verify ax25_kiss_send_frame() produces: FEND type_byte payload FEND\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    // A short "AX.25 frame" payload (no special bytes)
    uint8_t payload[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xE0,   // Dest addr
            0x96, 0x60, 0x86, 0x82, 0x82, 0x40, 0x61,   // Src addr
            0x3F };                                       // Control

    DEBUG_BUF("Payload to send", payload, sizeof(payload));

    uint8_t rc = ax25_kiss_send_frame(&ctx, 0, payload, sizeof(payload));

    DEBUG_VAR("send_frame return code (expect 0)", rc);DEBUG_VAR("tx_count (expect 1)", h.tx_count);

    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_send_frame returns KISS_OK", rc);
    TEST_ASSERT(h.tx_count == 1, "One serial write for send_frame", 0);

    if (h.tx_count > 0) {
        DEBUG_BUF("Encoded KISS frame", h.tx[0].data, h.tx[0].len);

        TEST_ASSERT(h.tx[0].data[0] == KISS_FEND, "Frame starts with FEND", h.tx[0].data[0]);
        TEST_ASSERT(h.tx[0].data[h.tx[0].len-1] == KISS_FEND, "Frame ends with FEND", 0);
        TEST_ASSERT(h.tx[0].data[1] == KISS_TYPE_BYTE(0, KISS_CMD_DATA), "Type byte is port=0,cmd=DATA (0x00)", h.tx[0].data[1]);
        // No special bytes so encoded length = 2 (FENDs) + 1 (type) + payload
        TEST_ASSERT(h.tx[0].len == 2 + 1 + sizeof(payload), "Encoded length = 2+1+payload for no-special-bytes payload", (int )h.tx[0].len);

        bool ok = verify_kiss_data_frame(&h, 0, payload, sizeof(payload), 0);
        TEST_ASSERT(ok, "Decoded payload matches original", 0);
    }

    return 0;
}

// ==========================================================================
// TEST 4: TX byte stuffing - FEND (0xC0) in payload
// ==========================================================================
static int test_kiss_tx_stuff_fend(void) {
    printf("\n--- test_kiss_tx_stuff_fend ---\n");
    printf("Verify FEND (0xC0) in payload is escaped as FESC(0xDB) TFEND(0xDC)\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t payload[] = { 0x01, KISS_FEND, 0x02 }; /* FEND in the middle */
    DEBUG_BUF("Payload with FEND inside", payload, sizeof(payload));

    ax25_kiss_send_frame(&ctx, 0, payload, sizeof(payload));

    if (h.tx_count > 0) {
        DEBUG_BUF("Encoded KISS frame", h.tx[0].data, h.tx[0].len);

        // Length = 2 (FENDs) + 1 (type) + 1 (0x01) + 2 (escaped FEND) + 1 (0x02) = 7
        TEST_ASSERT(h.tx[0].len == 7, "Encoded len = 7 for one FEND in payload", (unsigned int )h.tx[0].len);

        // Find FESC TFEND pair
        bool found = false;
        for (size_t i = 1; i < h.tx[0].len - 1; i++) {
            if (h.tx[0].data[i] == KISS_FESC && h.tx[0].data[i + 1] == KISS_TFEND) {
                found = true;
                DEBUG_VAR("FESC TFEND pair found at offset", (unsigned )i);
                break;
            }
        }
        TEST_ASSERT(found, "FESC TFEND escape sequence present in encoded frame", 0);

        // Round-trip decode must recover original
        bool ok = verify_kiss_data_frame(&h, 0, payload, sizeof(payload), 0);
        TEST_ASSERT(ok, "Round-trip: FEND in payload recovered correctly", 0);
    }

    return 0;
}

// ==========================================================================
// TEST 5: TX byte stuffing - FESC (0xDB) in payload
// ==========================================================================
static int test_kiss_tx_stuff_fesc(void) {
    printf("\n--- test_kiss_tx_stuff_fesc ---\n");
    printf("Verify FESC (0xDB) in payload is escaped as FESC(0xDB) TFESC(0xDD)\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t payload[] = { 0x10, KISS_FESC, 0x20 };
    DEBUG_BUF("Payload with FESC inside", payload, sizeof(payload));

    ax25_kiss_send_frame(&ctx, 0, payload, sizeof(payload));

    if (h.tx_count > 0) {
        DEBUG_BUF("Encoded KISS frame", h.tx[0].data, h.tx[0].len);

        TEST_ASSERT(h.tx[0].len == 7, "Encoded len = 7 for one FESC in payload", (unsigned int )h.tx[0].len);

        bool found = false;
        for (size_t i = 1; i < h.tx[0].len - 1; i++) {
            if (h.tx[0].data[i] == KISS_FESC && h.tx[0].data[i + 1] == KISS_TFESC) {
                found = true;
                DEBUG_VAR("FESC TFESC pair found at offset", (unsigned )i);
                break;
            }
        }
        TEST_ASSERT(found, "FESC TFESC escape sequence present in encoded frame", 0);

        bool ok = verify_kiss_data_frame(&h, 0, payload, sizeof(payload), 0);
        TEST_ASSERT(ok, "Round-trip: FESC in payload recovered correctly", 0);
    }

    return 0;
}

// ==========================================================================
// TEST 6: TX byte stuffing - consecutive special bytes
// ==========================================================================
static int test_kiss_tx_stuff_consecutive(void) {
    printf("\n--- test_kiss_tx_stuff_consecutive ---\n");
    printf("Verify consecutive FEND/FESC bytes are each independently escaped\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    // Payload: FEND FESC FEND FESC - four special bytes back to back
    uint8_t payload[] = { KISS_FEND, KISS_FESC, KISS_FEND, KISS_FESC };
    DEBUG_BUF("Payload of 4 consecutive special bytes", payload, sizeof(payload));

    ax25_kiss_send_frame(&ctx, 0, payload, sizeof(payload));

    if (h.tx_count > 0) {
        DEBUG_BUF("Encoded KISS frame", h.tx[0].data, h.tx[0].len);

        // Each special byte expands to 2 bytes: 4 * 2 + 1 (type) + 2 (FENDs) = 11
        DEBUG_VAR("Encoded length (expect 11)", h.tx[0].len);
        TEST_ASSERT(h.tx[0].len == 11, "Encoded length = 11 for 4 consecutive special bytes", (unsigned int )h.tx[0].len);

        bool ok = verify_kiss_data_frame(&h, 0, payload, sizeof(payload), 0);
        TEST_ASSERT(ok, "Round-trip: consecutive special bytes recovered", 0);
    }

    return 0;
}

// ==========================================================================
// TEST 7: TX stuffing - exhaustive single-byte payload (all 256 values)
// ==========================================================================
static int test_kiss_tx_stuff_exhaustive(void) {
    printf("\n--- test_kiss_tx_stuff_exhaustive ---\n");
    printf("Verify every possible byte value is encoded and decoded correctly\n");

    int failures = 0;
    for (int b = 0; b <= 255; b++) {
        kiss_harness_t h;
        ax25_kiss_ctx_t ctx;
        harness_init(&h, &ctx);

        uint8_t payload[1] = { (uint8_t) b };
        ax25_kiss_send_frame(&ctx, 0, payload, 1);

        if (h.tx_count == 0) {
            failures++;
            printf("  [FAIL] byte 0x%02X: no TX output\n", b);
            continue;
        }

        bool ok = verify_kiss_data_frame(&h, 0, payload, 1, 0);
        if (!ok) {
            failures++;
            printf("  [FAIL] byte 0x%02X: round-trip mismatch\n", b);
            DEBUG_BUF("Encoded", h.tx[0].data, h.tx[0].len);
        }
    }
    TEST_ASSERT(failures == 0, "All 256 byte values encode and decode correctly", failures);

    return 0;
}

// ==========================================================================
// TEST 8: RX state machine - well-formed DATA frame
// ==========================================================================
static int test_kiss_rx_data_frame(void) {
    printf("\n--- test_kiss_rx_data_frame ---\n");
    printf("Verify RX state machine assembles a DATA frame and fires on_frame callback\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    // Hand-craft: FEND + type(port=0,cmd=DATA) + payload + FEND
    uint8_t payload[] = { 0xAA, 0xBB, 0xCC };
    uint8_t frame[8];
    frame[0] = KISS_FEND;
    frame[1] = KISS_TYPE_BYTE(0, KISS_CMD_DATA);
    frame[2] = 0xAA;
    frame[3] = 0xBB;
    frame[4] = 0xCC;
    frame[5] = KISS_FEND;

    DEBUG_BUF("Injecting raw KISS frame", frame, 6);

    inject(&ctx, frame, 6);

    DEBUG_VAR("rx_count (expect 1)", h.rx_count);DEBUG_BOOL("kiss_mode after receiving FEND (expect true)", ctx.kiss_mode);

    TEST_ASSERT(h.rx_count == 1, "on_frame fired once", h.rx_count);
    if (h.rx_count > 0) {
        DEBUG_VAR("Received port (expect 0)", h.rx_port[0]);
        DEBUG_BUF("Received payload", h.rx_frames[0], h.rx_lens[0]);
        TEST_ASSERT(h.rx_port[0] == 0, "Received on port 0", h.rx_port[0]);
        TEST_ASSERT(h.rx_lens[0] == 3, "Received payload length = 3", (unsigned int )h.rx_lens[0]);
        TEST_ASSERT(memcmp(h.rx_frames[0], payload, 3) == 0, "Received payload matches original", 0);
    }
    TEST_ASSERT(ctx.kiss_mode == true, "kiss_mode set to true on first FEND seen", 0);

    return 0;
}

// ==========================================================================
// TEST 9: RX escape decoding - FESC TFEND -> FEND
// ==========================================================================
static int test_kiss_rx_escape_fend(void) {
    printf("\n--- test_kiss_rx_escape_fend ---\n");
    printf("Verify FESC(0xDB) TFEND(0xDC) in RX stream decodes to FEND(0xC0)\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    // Frame with escaped FEND in payload: FEND type 0x01 FESC TFEND 0x02 FEND
    uint8_t raw[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_DATA), 0x01, KISS_FESC, KISS_TFEND, 0x02, KISS_FEND };
    DEBUG_BUF("Injecting frame with FESC TFEND", raw, sizeof(raw));

    inject(&ctx, raw, sizeof(raw));

    TEST_ASSERT(h.rx_count == 1, "on_frame fired once", h.rx_count);
    if (h.rx_count > 0) {
        DEBUG_BUF("Decoded payload (expect 01 C0 02)", h.rx_frames[0], h.rx_lens[0]);
        uint8_t expected[] = { 0x01, KISS_FEND, 0x02 };
        TEST_ASSERT(h.rx_lens[0] == 3, "Decoded length = 3", (unsigned int )h.rx_lens[0]);
        TEST_ASSERT(memcmp(h.rx_frames[0], expected, 3) == 0, "FESC TFEND decoded to FEND correctly", 0);
    }

    return 0;
}

// ==========================================================================
// TEST 10: RX escape decoding - FESC TFESC -> FESC
// ==========================================================================
static int test_kiss_rx_escape_fesc(void) {
    printf("\n--- test_kiss_rx_escape_fesc ---\n");
    printf("Verify FESC(0xDB) TFESC(0xDD) in RX stream decodes to FESC(0xDB)\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t raw[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_DATA), 0x10, KISS_FESC, KISS_TFESC, 0x20, KISS_FEND };
    DEBUG_BUF("Injecting frame with FESC TFESC", raw, sizeof(raw));

    inject(&ctx, raw, sizeof(raw));

    TEST_ASSERT(h.rx_count == 1, "on_frame fired once", h.rx_count);
    if (h.rx_count > 0) {
        DEBUG_BUF("Decoded payload (expect 10 DB 20)", h.rx_frames[0], h.rx_lens[0]);
        uint8_t expected[] = { 0x10, KISS_FESC, 0x20 };
        TEST_ASSERT(h.rx_lens[0] == 3, "Decoded length = 3", (unsigned int )h.rx_lens[0]);
        TEST_ASSERT(memcmp(h.rx_frames[0], expected, 3) == 0, "FESC TFESC decoded to FESC correctly", 0);
    }

    return 0;
}

// ==========================================================================
// TEST 11: RX - invalid escape byte (not TFEND or TFESC)
// ==========================================================================
static int test_kiss_rx_invalid_escape(void) {
    printf("\n--- test_kiss_rx_invalid_escape ---\n");
    printf("Verify invalid byte after FESC is silently ignored; frame assembly continues\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    // Per spec: byte after FESC that is neither TFEND nor TFESC -> ignore, continue
    // Frame: FEND type 0xAA FESC 0x00(invalid) 0xBB FEND
    uint8_t raw[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_DATA), 0xAA, KISS_FESC, 0x00, 0xBB, KISS_FEND };
    DEBUG_BUF("Injecting frame with invalid escape byte", raw, sizeof(raw));

    inject(&ctx, raw, sizeof(raw));

    // Implementation returns immediately on invalid escape (state set back to IN_FRAME)
    // so the invalid byte is dropped; 0xBB still arrives
    TEST_ASSERT(h.rx_count == 1, "on_frame still fires after invalid escape", h.rx_count);
    if (h.rx_count > 0) {
        DEBUG_BUF("Received payload after invalid escape", h.rx_frames[0], h.rx_lens[0]);DEBUG_VAR("Received payload length (expect 2: 0xAA 0xBB)", h.rx_lens[0]);
        // 0xAA was stored before the escape, invalid byte dropped, 0xBB stored after
        TEST_ASSERT(h.rx_lens[0] == 2, "Payload length = 2 (invalid escape byte dropped)", (unsigned int )h.rx_lens[0]);
        TEST_ASSERT(h.rx_frames[0][0] == 0xAA, "First byte = 0xAA", h.rx_frames[0][0]);
        TEST_ASSERT(h.rx_frames[0][1] == 0xBB, "Second byte = 0xBB", h.rx_frames[0][1]);
    }

    return 0;
}

// ==========================================================================
// TEST 12: RX - consecutive FEND (empty / padding frame discarded)
// ==========================================================================
static int test_kiss_rx_consecutive_fend(void) {
    printf("\n--- test_kiss_rx_consecutive_fend ---\n");
    printf("Verify consecutive FENDs (empty frames) do not trigger on_frame callback\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    // Five consecutive FENDs = four empty inter-frame gaps
    uint8_t raw[] = { KISS_FEND, KISS_FEND, KISS_FEND, KISS_FEND, KISS_FEND };
    DEBUG_BUF("Injecting 5 consecutive FENDs", raw, sizeof(raw));

    inject(&ctx, raw, sizeof(raw));

    DEBUG_VAR("rx_count after consecutive FENDs (expect 0)", h.rx_count);
    TEST_ASSERT(h.rx_count == 0, "No on_frame fired for consecutive FEND stream", h.rx_count);

    return 0;
}

// ==========================================================================
// TEST 13: RX - bytes before first FEND are discarded (IDLE state)
// ==========================================================================
static int test_kiss_rx_idle_discard(void) {
    printf("\n--- test_kiss_rx_idle_discard ---\n");
    printf("Verify bytes arriving before first FEND are silently discarded\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    // Junk before FEND, then a valid frame
    uint8_t payload[] = { 0x42 };
    uint8_t raw[] = { 0x01, 0x02, 0x03,                               // noise
            KISS_FEND,                                       // frame start
            KISS_TYPE_BYTE(0, KISS_CMD_DATA), 0x42,          // type + payload
            KISS_FEND                                        // frame end
            };
    DEBUG_BUF("Injecting noise + valid frame", raw, sizeof(raw));

    inject(&ctx, raw, sizeof(raw));

    DEBUG_VAR("rx_count (expect 1)", h.rx_count);
    TEST_ASSERT(h.rx_count == 1, "on_frame fires once (noise discarded)", h.rx_count);
    if (h.rx_count > 0) {
        DEBUG_BUF("Payload received", h.rx_frames[0], h.rx_lens[0]);
        TEST_ASSERT(h.rx_lens[0] == 1, "Payload length = 1", (unsigned int )h.rx_lens[0]);
        TEST_ASSERT(h.rx_frames[0][0] == payload[0], "Payload byte = 0x42", h.rx_frames[0][0]);
    }

    return 0;
}

// ==========================================================================
// TEST 14: RX - fragmented delivery (byte by byte)
// ==========================================================================
static int test_kiss_rx_fragmented(void) {
    printf("\n--- test_kiss_rx_fragmented ---\n");
    printf("Verify frame assembly works when bytes arrive one at a time\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t raw[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_DATA), 0x11, 0x22, 0x33, 0x44, KISS_FEND };

    DEBUG_PRINT("Injecting frame one byte at a time");
    for (size_t i = 0; i < sizeof(raw); i++) {
        DEBUG_HEX("Byte injected", raw[i]);
        ax25_kiss_receive_byte(&ctx, raw[i]);
    }

    DEBUG_VAR("rx_count (expect 1)", h.rx_count);
    TEST_ASSERT(h.rx_count == 1, "Frame assembled from single-byte injections", h.rx_count);
    if (h.rx_count > 0) {
        DEBUG_BUF("Payload received", h.rx_frames[0], h.rx_lens[0]);
        uint8_t expected[] = { 0x11, 0x22, 0x33, 0x44 };
        TEST_ASSERT(h.rx_lens[0] == 4, "Payload length = 4", (unsigned int )h.rx_lens[0]);
        TEST_ASSERT(memcmp(h.rx_frames[0], expected, 4) == 0, "Fragmented frame payload matches", 0);
    }

    return 0;
}

// ==========================================================================
// TEST 15: RX - multiple frames in a single stream
// ==========================================================================
static int test_kiss_rx_multiple_frames(void) {
    printf("\n--- test_kiss_rx_multiple_frames ---\n");
    printf("Verify back-to-back frames in a single byte stream are each dispatched\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    // Three frames concatenated; shared FEND between them per spec
    uint8_t raw[] = {
    KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_DATA), 0x01, KISS_FEND, KISS_TYPE_BYTE(1, KISS_CMD_DATA), 0x02, 0x03, KISS_FEND, KISS_TYPE_BYTE(2, KISS_CMD_DATA),
            0x04, 0x05, 0x06, KISS_FEND };
    DEBUG_BUF("Injecting 3 concatenated frames", raw, sizeof(raw));

    inject(&ctx, raw, sizeof(raw));

    DEBUG_VAR("rx_count (expect 3)", h.rx_count);
    TEST_ASSERT(h.rx_count == 3, "Three on_frame callbacks fired", h.rx_count);
    if (h.rx_count >= 3) {
        DEBUG_VAR("Frame 0 port (expect 0)", h.rx_port[0]);DEBUG_VAR("Frame 1 port (expect 1)", h.rx_port[1]);DEBUG_VAR("Frame 2 port (expect 2)", h.rx_port[2]);
        TEST_ASSERT(h.rx_port[0] == 0, "Frame 0 port = 0", h.rx_port[0]);
        TEST_ASSERT(h.rx_port[1] == 1, "Frame 1 port = 1", h.rx_port[1]);
        TEST_ASSERT(h.rx_port[2] == 2, "Frame 2 port = 2", h.rx_port[2]);
        TEST_ASSERT(h.rx_lens[0] == 1, "Frame 0 length = 1", (unsigned int )h.rx_lens[0]);
        TEST_ASSERT(h.rx_lens[1] == 2, "Frame 1 length = 2", (unsigned int )h.rx_lens[1]);
        TEST_ASSERT(h.rx_lens[2] == 3, "Frame 2 length = 3", (unsigned int )h.rx_lens[2]);
    }

    return 0;
}

// ==========================================================================
// TEST 16: RX command - TXDELAY parameter
// ==========================================================================
static int test_kiss_rx_cmd_txdelay(void) {
    printf("\n--- test_kiss_rx_cmd_txdelay ---\n");
    printf("Verify received TXDELAY command updates ctx->ports[port].txdelay\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t new_txdelay = 120u;
    uint8_t raw[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_TXDELAY), new_txdelay,
    KISS_FEND };
    DEBUG_BUF("Injecting TXDELAY command frame", raw, sizeof(raw));DEBUG_VAR("Port 0 txdelay before (expect default=50)", ctx.ports[0].txdelay);

    inject(&ctx, raw, sizeof(raw));

    DEBUG_VAR("Port 0 txdelay after (expect 120)", ctx.ports[0].txdelay);
    TEST_ASSERT(ctx.ports[0].txdelay == new_txdelay, "txdelay updated to 120", ctx.ports[0].txdelay);
    TEST_ASSERT(h.rx_count == 0, "TXDELAY does not fire on_frame", h.rx_count);

    return 0;
}

// ==========================================================================
// TEST 17: RX command - PERSISTENCE parameter
// ==========================================================================
static int test_kiss_rx_cmd_persistence(void) {
    printf("\n--- test_kiss_rx_cmd_persistence ---\n");
    printf("Verify received PERSISTENCE command updates ctx->ports[port].persistence\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t new_p = 200u;
    uint8_t raw[] = { KISS_FEND, KISS_TYPE_BYTE(3, KISS_CMD_PERSISTENCE), new_p,
    KISS_FEND };
    DEBUG_VAR("Port 3 persistence before (expect default=63)", ctx.ports[3].persistence);

    inject(&ctx, raw, sizeof(raw));

    DEBUG_VAR("Port 3 persistence after (expect 200)", ctx.ports[3].persistence);
    TEST_ASSERT(ctx.ports[3].persistence == new_p, "persistence updated to 200 on port 3", ctx.ports[3].persistence);

    return 0;
}

// ==========================================================================
// TEST 18: RX command - SLOTTIME parameter
// ==========================================================================
static int test_kiss_rx_cmd_slottime(void) {
    printf("\n--- test_kiss_rx_cmd_slottime ---\n");
    printf("Verify received SLOTTIME command updates ctx->ports[port].slottime\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t new_slot = 20u;
    uint8_t raw[] = { KISS_FEND, KISS_TYPE_BYTE(1, KISS_CMD_SLOTTIME), new_slot,
    KISS_FEND };
    DEBUG_VAR("Port 1 slottime before (expect 10)", ctx.ports[1].slottime);

    inject(&ctx, raw, sizeof(raw));

    DEBUG_VAR("Port 1 slottime after (expect 20)", ctx.ports[1].slottime);
    TEST_ASSERT(ctx.ports[1].slottime == new_slot, "slottime updated to 20 on port 1", ctx.ports[1].slottime);

    return 0;
}

// ==========================================================================
// TEST 19: RX command - TXTAIL parameter
// ==========================================================================
static int test_kiss_rx_cmd_txtail(void) {
    printf("\n--- test_kiss_rx_cmd_txtail ---\n");
    printf("Verify received TXTAIL command updates ctx->ports[port].txtail\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t new_tail = 5u;
    uint8_t raw[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_TXTAIL), new_tail,
    KISS_FEND };

    inject(&ctx, raw, sizeof(raw));

    DEBUG_VAR("Port 0 txtail after (expect 5)", ctx.ports[0].txtail);
    TEST_ASSERT(ctx.ports[0].txtail == new_tail, "txtail updated to 5", ctx.ports[0].txtail);

    return 0;
}

// ==========================================================================
// TEST 20: RX command - FULLDUPLEX parameter
// ==========================================================================
static int test_kiss_rx_cmd_fullduplex(void) {
    printf("\n--- test_kiss_rx_cmd_fullduplex ---\n");
    printf("Verify received FULLDUPLEX command toggles ctx->ports[port].full_duplex\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    DEBUG_BOOL("Port 0 full_duplex before (expect false)", ctx.ports[0].full_duplex);

    // Set full duplex ON (non-zero)
    uint8_t raw_on[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_FULLDUPLEX), 0x01, KISS_FEND };
    inject(&ctx, raw_on, sizeof(raw_on));
    DEBUG_BOOL("Port 0 full_duplex after ON (expect true)", ctx.ports[0].full_duplex);
    TEST_ASSERT(ctx.ports[0].full_duplex == true, "full_duplex = true after cmd with 0x01", 0);

    // Set full duplex OFF (zero)
    uint8_t raw_off[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_FULLDUPLEX), 0x00, KISS_FEND };
    inject(&ctx, raw_off, sizeof(raw_off));
    DEBUG_BOOL("Port 0 full_duplex after OFF (expect false)", ctx.ports[0].full_duplex);
    TEST_ASSERT(ctx.ports[0].full_duplex == false, "full_duplex = false after cmd with 0x00", 0);

    // Non-zero (e.g. 0xFF) also means full duplex
    uint8_t raw_ff[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_FULLDUPLEX), 0xFF, KISS_FEND };
    inject(&ctx, raw_ff, sizeof(raw_ff));
    DEBUG_BOOL("Port 0 full_duplex after 0xFF (expect true)", ctx.ports[0].full_duplex);
    TEST_ASSERT(ctx.ports[0].full_duplex == true, "full_duplex = true after cmd with 0xFF", 0);

    return 0;
}

// ==========================================================================
// TEST 21: RX command - SETHARDWARE fires on_hardware callback
// ==========================================================================
static int test_kiss_rx_cmd_sethardware(void) {
    printf("\n--- test_kiss_rx_cmd_sethardware ---\n");
    printf("Verify received SETHARDWARE command fires on_hardware and stores bytes\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t hw_data[] = { 0xDE, 0xAD, 0xBE, 0xEF };
    uint8_t raw[32];
    size_t pos = 0;
    raw[pos++] = KISS_FEND;
    raw[pos++] = KISS_TYPE_BYTE(0, KISS_CMD_SETHARDWARE);
    for (size_t i = 0; i < sizeof(hw_data); i++)
        raw[pos++] = hw_data[i];
    raw[pos++] = KISS_FEND;
    DEBUG_BUF("Injecting SETHARDWARE frame", raw, pos);

    inject(&ctx, raw, pos);

    DEBUG_VAR("hw_count (expect 1)", h.hw_count);
    TEST_ASSERT(h.hw_count == 1, "on_hardware fired once", h.hw_count);
    if (h.hw_count > 0) {
        DEBUG_VAR("Hardware port (expect 0)", h.hw_port[0]);
        DEBUG_BUF("Hardware data received", h.hw_data[0], h.hw_lens[0]);
        TEST_ASSERT(h.hw_port[0] == 0, "Hardware command on port 0", h.hw_port[0]);
        TEST_ASSERT(h.hw_lens[0] == sizeof(hw_data), "Hardware data length = 4", (unsigned int )h.hw_lens[0]);
        TEST_ASSERT(memcmp(h.hw_data[0], hw_data, sizeof(hw_data)) == 0, "Hardware data bytes match", 0);
    }
    // Stored in port params too
    TEST_ASSERT(ctx.ports[0].hardware_len == sizeof(hw_data), "port hardware_len = 4", (unsigned int )ctx.ports[0].hardware_len);
    TEST_ASSERT(memcmp(ctx.ports[0].hardware, hw_data, sizeof(hw_data)) == 0, "port hardware[] bytes match", 0);

    return 0;
}

// ==========================================================================
// TEST 22: RX command - RETURN exits KISS mode and fires on_return
// ==========================================================================
static int test_kiss_rx_cmd_return(void) {
    printf("\n--- test_kiss_rx_cmd_return ---\n");
    printf("Verify RETURN command (0xFF) exits KISS mode and calls on_return callback\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);
    ax25_kiss_enter(&ctx); /* put into KISS mode */

    DEBUG_BOOL("kiss_mode before RETURN (expect true)", ctx.kiss_mode);

    uint8_t raw[] = { KISS_FEND, KISS_RETURN_TYPE_BYTE, KISS_FEND };
    DEBUG_BUF("Injecting RETURN frame", raw, sizeof(raw));

    inject(&ctx, raw, sizeof(raw));

    DEBUG_BOOL("kiss_mode after RETURN (expect false)", ctx.kiss_mode);DEBUG_VAR("return_count (expect 1)", h.return_count);

    TEST_ASSERT(ctx.kiss_mode == false, "kiss_mode is false after RETURN received", 0);
    TEST_ASSERT(h.return_count == 1, "on_return fired once", h.return_count);

    return 0;
}

// ==========================================================================
// TEST 23: ax25_kiss_send_return - transmits 0xFF frame, clears kiss_mode
// ==========================================================================
static int test_kiss_send_return(void) {
    printf("\n--- test_kiss_send_return ---\n");
    printf("Verify ax25_kiss_send_return() sends FEND 0xFF FEND and sets kiss_mode=false\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);
    ax25_kiss_enter(&ctx);

    DEBUG_BOOL("kiss_mode before send_return (expect true)", ctx.kiss_mode);

    uint8_t rc = ax25_kiss_send_return(&ctx);

    DEBUG_VAR("send_return return code (expect 0)", rc);DEBUG_BOOL("kiss_mode after send_return (expect false)", ctx.kiss_mode);

    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_send_return returns KISS_OK", rc);
    TEST_ASSERT(ctx.kiss_mode == false, "kiss_mode is false after send_return", 0);

    // Find the RETURN frame (the enter() also wrote a FEND, so tx_count >= 2)
    bool found = false;
    for (uint8_t i = 0; i < h.tx_count; i++) {
        DEBUG_BUF("TX slot", h.tx[i].data, h.tx[i].len);
        if (h.tx[i].len == 3 && h.tx[i].data[0] == KISS_FEND && h.tx[i].data[1] == KISS_RETURN_TYPE_BYTE && h.tx[i].data[2] == KISS_FEND) {
            found = true;
            DEBUG_VAR("RETURN frame found at TX slot", i);
            break;
        }
    }
    TEST_ASSERT(found, "RETURN frame (FEND 0xFF FEND) found in TX output", 0);

    return 0;
}

// ==========================================================================
// TEST 24: Multi-port DATA frame - port nibble routing
// ==========================================================================
static int test_kiss_multiport_data(void) {
    printf("\n--- test_kiss_multiport_data ---\n");
    printf("Verify frames on ports 0-14 are routed to on_frame with correct port number\n");

    int failures = 0;
    for (uint8_t port = 0; port <= 14; port++) {
        kiss_harness_t h;
        ax25_kiss_ctx_t ctx;
        harness_init(&h, &ctx);

        uint8_t payload[] = { port }; /* payload is just the port number as data */
        uint8_t raw[] = { KISS_FEND, KISS_TYPE_BYTE(port, KISS_CMD_DATA), port, KISS_FEND };

        inject(&ctx, raw, sizeof(raw));

        if (h.rx_count != 1 || h.rx_port[0] != port) {
            failures++;
            printf("  [FAIL] Port %u: rx_count=%u, rx_port=%u\n", port, h.rx_count, h.rx_port[0]);
        } else {
            DEBUG_VAR("Port routed correctly", port);
        }
        (void) payload;
    }
    TEST_ASSERT(failures == 0, "All 15 ports (0-14) route data frames correctly", failures);
    // Port 0x0F (15) is reserved - frames on it must be silently discarded

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);
    uint8_t raw[] = { KISS_FEND, KISS_TYPE_BYTE(0x0F, KISS_CMD_DATA), 0x99, KISS_FEND };
    DEBUG_BUF("Injecting frame on reserved port 0x0F", raw, sizeof(raw));
    inject(&ctx, raw, sizeof(raw));
    DEBUG_VAR("rx_count for port 0x0F (expect 0)", h.rx_count);
    TEST_ASSERT(h.rx_count == 0, "Reserved port 0x0F frames are silently discarded", h.rx_count);

    return 0;
}

// ==========================================================================
// TEST 25: ax25_kiss_send_command - parameter commands TX path
// ==========================================================================
static int test_kiss_send_command_params(void) {
    printf("\n--- test_kiss_send_command_params ---\n");
    printf("Verify ax25_kiss_send_command() correctly frames parameter commands\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t param = 75u;
    uint8_t rc = ax25_kiss_send_command(&ctx, 0, KISS_CMD_TXDELAY, &param, 1);

    DEBUG_VAR("send_command(TXDELAY) return code (expect 0)", rc);
    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_send_command returns KISS_OK", rc);
    TEST_ASSERT(h.tx_count == 1, "One TX write for parameter command", h.tx_count);

    if (h.tx_count > 0) {
        DEBUG_BUF("Encoded TXDELAY command frame", h.tx[0].data, h.tx[0].len);
        // FEND + type_byte(port=0,cmd=TXDELAY) + param + FEND = 4 bytes min
        TEST_ASSERT(h.tx[0].data[0] == KISS_FEND, "Starts with FEND", 0);
        TEST_ASSERT(h.tx[0].data[1] == KISS_TYPE_BYTE(0, KISS_CMD_TXDELAY), "Type byte = port0 cmd TXDELAY", h.tx[0].data[1]);
        TEST_ASSERT(h.tx[0].data[h.tx[0].len-1] == KISS_FEND, "Ends with FEND", 0);
    }

    // Attempting to send DATA via send_command must fail
    uint8_t data_byte = 0x00;
    rc = ax25_kiss_send_command(&ctx, 0, KISS_CMD_DATA, &data_byte, 1);
    DEBUG_VAR("send_command(DATA) return code (expect error)", rc);
    TEST_ASSERT(rc != KISS_OK, "ax25_kiss_send_command rejects KISS_CMD_DATA", rc);

    // Attempting to send RETURN via send_command must fail
    rc = ax25_kiss_send_command(&ctx, 0, KISS_CMD_RETURN, &data_byte, 1);
    DEBUG_VAR("send_command(RETURN) return code (expect error)", rc);
    TEST_ASSERT(rc != KISS_OK, "ax25_kiss_send_command rejects KISS_CMD_RETURN", rc);

    return 0;
}

// ==========================================================================
// TEST 26: ax25_kiss_set_port_params / ax25_kiss_get_port_params round-trip
// ==========================================================================
static int test_kiss_set_get_port_params(void) {
    printf("\n--- test_kiss_set_get_port_params ---\n");
    printf("Verify set_port_params sends all 5 commands and get_port_params retrieves them\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    ax25_kiss_port_params_t set_p;
    set_p.txdelay = 80u;
    set_p.persistence = 128u;
    set_p.slottime = 15u;
    set_p.txtail = 3u;
    set_p.full_duplex = true;
    set_p.hardware_len = 0u;

    DEBUG_VAR("Setting txdelay", set_p.txdelay);DEBUG_VAR("Setting persistence", set_p.persistence);DEBUG_VAR("Setting slottime", set_p.slottime);DEBUG_VAR("Setting txtail", set_p.txtail);DEBUG_BOOL("Setting full_duplex", set_p.full_duplex);

    uint8_t rc = ax25_kiss_set_port_params(&ctx, 2, &set_p);

    DEBUG_VAR("set_port_params return code (expect 0)", rc);
    // Expect 5 commands sent (TXDELAY, PERSISTENCE, SLOTTIME, TXTAIL, FULLDUPLEX)
    DEBUG_VAR("tx_count after set_port_params (expect 5)", h.tx_count);
    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_set_port_params returns KISS_OK", rc);
    TEST_ASSERT(h.tx_count == 5, "5 command frames transmitted for all params", h.tx_count);

    // Verify local mirror updated
    TEST_ASSERT(ctx.ports[2].txdelay == set_p.txdelay, "Local txdelay mirrored", ctx.ports[2].txdelay);
    TEST_ASSERT(ctx.ports[2].persistence == set_p.persistence, "Local persistence mirrored", ctx.ports[2].persistence);
    TEST_ASSERT(ctx.ports[2].slottime == set_p.slottime, "Local slottime mirrored", ctx.ports[2].slottime);
    TEST_ASSERT(ctx.ports[2].txtail == set_p.txtail, "Local txtail mirrored", ctx.ports[2].txtail);
    TEST_ASSERT(ctx.ports[2].full_duplex == set_p.full_duplex, "Local full_duplex mirrored", ctx.ports[2].full_duplex);

    // Get round-trip
    ax25_kiss_port_params_t get_p;
    memset(&get_p, 0xFF, sizeof(get_p));
    rc = ax25_kiss_get_port_params(&ctx, 2, &get_p);

    DEBUG_VAR("get_port_params return code (expect 0)", rc);DEBUG_VAR("Got txdelay (expect 80)", get_p.txdelay);DEBUG_VAR("Got persistence (expect 128)", get_p.persistence);DEBUG_VAR("Got slottime (expect 15)", get_p.slottime);DEBUG_VAR("Got txtail (expect 3)", get_p.txtail);DEBUG_BOOL("Got full_duplex (expect true)", get_p.full_duplex);

    TEST_ASSERT(rc == KISS_OK, "ax25_kiss_get_port_params returns KISS_OK", rc);
    TEST_ASSERT(get_p.txdelay == set_p.txdelay, "Get txdelay = set txdelay", get_p.txdelay);
    TEST_ASSERT(get_p.persistence == set_p.persistence, "Get persistence = set persistence", get_p.persistence);
    TEST_ASSERT(get_p.slottime == set_p.slottime, "Get slottime = set slottime", get_p.slottime);
    TEST_ASSERT(get_p.txtail == set_p.txtail, "Get txtail = set txtail", get_p.txtail);
    TEST_ASSERT(get_p.full_duplex == set_p.full_duplex, "Get full_duplex = set full_duplex", 0);

    return 0;
}

// ==========================================================================
// TEST 27: set_port_params sends SETHARDWARE command when hardware_len > 0
// ==========================================================================
static int test_kiss_set_port_params_hardware(void) {
    printf("\n--- test_kiss_set_port_params_hardware ---\n");
    printf("Verify set_port_params sends SETHARDWARE when hardware_len > 0\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    ax25_kiss_port_params_t p;
    memset(&p, 0, sizeof(p));
    p.txdelay = KISS_DEFAULT_TXDELAY;
    p.persistence = KISS_DEFAULT_PERSISTENCE;
    p.slottime = KISS_DEFAULT_SLOTTIME;
    p.txtail = KISS_DEFAULT_TXTAIL;
    p.full_duplex = false;
    p.hardware[0] = 0xCA;
    p.hardware[1] = 0xFE;
    p.hardware_len = 2u;

    DEBUG_BUF("Hardware bytes to send", p.hardware, p.hardware_len);

    uint8_t rc = ax25_kiss_set_port_params(&ctx, 0, &p);

    DEBUG_VAR("set_port_params return code (expect 0)", rc);
    // Should be 5 (standard params) + 1 (SETHARDWARE) = 6 writes
    DEBUG_VAR("tx_count (expect 6 = 5 params + 1 hardware)", h.tx_count);
    TEST_ASSERT(rc == KISS_OK, "set_port_params with hardware returns KISS_OK", rc);
    TEST_ASSERT(h.tx_count == 6, "6 command frames transmitted (5 params + sethardware)", h.tx_count);

    // Check the last TX slot contains a SETHARDWARE type byte
    if (h.tx_count == 6) {
        DEBUG_BUF("SETHARDWARE TX frame", h.tx[5].data, h.tx[5].len);
        TEST_ASSERT(h.tx[5].data[1] == KISS_TYPE_BYTE(0, KISS_CMD_SETHARDWARE), "Slot 5 type byte = SETHARDWARE", h.tx[5].data[1]);
    }

    return 0;
}

// ==========================================================================
// TEST 28: Error codes - invalid port, NULL, frame too large
// ==========================================================================
static int test_kiss_error_codes(void) {
    printf("\n--- test_kiss_error_codes ---\n");
    printf("Verify error return codes for invalid inputs\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t frame[1] = { 0x00 };
    uint8_t rc;

    // --- ax25_kiss_send_frame ---

    // NULL context
    rc = ax25_kiss_send_frame(NULL, 0, frame, 1);
    DEBUG_VAR("send_frame(NULL ctx) (expect KISS_ERR_NULL=1)", rc);
    TEST_ASSERT(rc == KISS_ERR_NULL, "send_frame(NULL) returns KISS_ERR_NULL", rc);

    // Reserved port 0x0F
    rc = ax25_kiss_send_frame(&ctx, 0x0F, frame, 1);
    DEBUG_VAR("send_frame(port=0x0F) (expect KISS_ERR_PORT=2)", rc);
    TEST_ASSERT(rc == KISS_ERR_PORT, "send_frame(port=0x0F) returns KISS_ERR_PORT", rc);

    // Port 15 (same as 0x0F)
    rc = ax25_kiss_send_frame(&ctx, 15, frame, 1);
    TEST_ASSERT(rc == KISS_ERR_PORT, "send_frame(port=15) returns KISS_ERR_PORT", rc);

    // Frame too large
    uint8_t big_frame[KISS_MAX_FRAME_SIZE + 1];
    memset(big_frame, 0xAB, sizeof(big_frame));
    rc = ax25_kiss_send_frame(&ctx, 0, big_frame, sizeof(big_frame));
    DEBUG_VAR("send_frame(oversized) (expect KISS_ERR_FRAME_SIZE=3)", rc);
    TEST_ASSERT(rc == KISS_ERR_FRAME_SIZE, "send_frame(oversized) returns KISS_ERR_FRAME_SIZE", rc);

    // No serial_write
    ax25_kiss_ctx_t ctx_noserial;
    ax25_kiss_init(&ctx_noserial);
    rc = ax25_kiss_send_frame(&ctx_noserial, 0, frame, 1);
    DEBUG_VAR("send_frame(no serial_write) (expect KISS_ERR_NO_SERIAL=4)", rc);
    TEST_ASSERT(rc == KISS_ERR_NO_SERIAL, "send_frame without serial_write returns KISS_ERR_NO_SERIAL", rc);

    // --- ax25_kiss_send_command ---

    rc = ax25_kiss_send_command(NULL, 0, KISS_CMD_TXDELAY, frame, 1);
    TEST_ASSERT(rc == KISS_ERR_NULL, "send_command(NULL) returns KISS_ERR_NULL", rc);

    rc = ax25_kiss_send_command(&ctx, 0x0F, KISS_CMD_TXDELAY, frame, 1);
    TEST_ASSERT(rc == KISS_ERR_PORT, "send_command(port=0x0F) returns KISS_ERR_PORT", rc);

    // --- ax25_kiss_send_return ---

    rc = ax25_kiss_send_return(NULL);
    TEST_ASSERT(rc == KISS_ERR_NULL, "send_return(NULL) returns KISS_ERR_NULL", rc);

    rc = ax25_kiss_send_return(&ctx_noserial);
    TEST_ASSERT(rc == KISS_ERR_NO_SERIAL, "send_return without serial_write returns KISS_ERR_NO_SERIAL", rc);

    // --- ax25_kiss_get_port_params ---

    rc = ax25_kiss_get_port_params(NULL, 0, NULL);
    TEST_ASSERT(rc == KISS_ERR_NULL, "get_port_params(NULL ctx) returns KISS_ERR_NULL", rc);

    ax25_kiss_port_params_t pp;
    rc = ax25_kiss_get_port_params(&ctx, KISS_MAX_PORTS, &pp);
    DEBUG_VAR("get_port_params(port=KISS_MAX_PORTS) (expect KISS_ERR_PORT=2)", rc);
    TEST_ASSERT(rc == KISS_ERR_PORT, "get_port_params(port >= KISS_MAX_PORTS) returns KISS_ERR_PORT", rc);

    // --- ax25_kiss_set_port_params ---

    rc = ax25_kiss_set_port_params(NULL, 0, NULL);
    TEST_ASSERT(rc == KISS_ERR_NULL, "set_port_params(NULL ctx) returns KISS_ERR_NULL", rc);

    ax25_kiss_port_params_t default_pp;
    memset(&default_pp, 0, sizeof(default_pp));
    rc = ax25_kiss_set_port_params(&ctx, 0x0F, &default_pp);
    TEST_ASSERT(rc == KISS_ERR_PORT, "set_port_params(port=0x0F) returns KISS_ERR_PORT", rc);

    return 0;
}

// ==========================================================================
// TEST 29: kiss_mode flag transitions
// ==========================================================================
static int test_kiss_mode_transitions(void) {
    printf("\n--- test_kiss_mode_transitions ---\n");
    printf("Verify kiss_mode transitions: init=false, enter=true, send_return=false, rx_FEND=true\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    DEBUG_BOOL("kiss_mode after init (expect false)", ctx.kiss_mode);
    TEST_ASSERT(ctx.kiss_mode == false, "kiss_mode false after init", 0);

    ax25_kiss_enter(&ctx);
    DEBUG_BOOL("kiss_mode after enter (expect true)", ctx.kiss_mode);
    TEST_ASSERT(ctx.kiss_mode == true, "kiss_mode true after ax25_kiss_enter", 0);

    ax25_kiss_send_return(&ctx);
    DEBUG_BOOL("kiss_mode after send_return (expect false)", ctx.kiss_mode);
    TEST_ASSERT(ctx.kiss_mode == false, "kiss_mode false after ax25_kiss_send_return", 0);

    // Receiving a FEND from TNC also sets kiss_mode to true (auto-enter on receive)
    uint8_t fend = KISS_FEND;
    ax25_kiss_receive_byte(&ctx, fend);
    DEBUG_BOOL("kiss_mode after receiving FEND (expect true)", ctx.kiss_mode);
    TEST_ASSERT(ctx.kiss_mode == true, "kiss_mode true after receiving FEND", 0);

    // Receiving RETURN frame resets it
    uint8_t return_frame[] = { KISS_RETURN_TYPE_BYTE, KISS_FEND };
    inject(&ctx, return_frame, sizeof(return_frame));
    DEBUG_BOOL("kiss_mode after receiving RETURN (expect false)", ctx.kiss_mode);
    TEST_ASSERT(ctx.kiss_mode == false, "kiss_mode false after receiving RETURN frame", 0);

    return 0;
}

// ==========================================================================
// TEST 30: Round-trip - full TX then RX loopback
// ==========================================================================
static int test_kiss_round_trip_loopback(void) {
    printf("\n--- test_kiss_round_trip_loopback ---\n");
    printf("Encode a frame via send_frame, feed TX bytes back into RX engine, verify payload\n");

    kiss_harness_t h_tx, h_rx;
    ax25_kiss_ctx_t ctx_tx, ctx_rx;
    harness_init(&h_tx, &ctx_tx);
    harness_init(&h_rx, &ctx_rx);

    // Payload with both special bytes to exercise stuffing/unstuffing
    uint8_t payload[] = { 0x82, 0x88, KISS_FEND, 0x61, KISS_FESC, 0x3F, 0x00, 0xFF };
    DEBUG_BUF("Original payload (with FEND and FESC)", payload, sizeof(payload));

    // Encode on ctx_tx
    uint8_t rc = ax25_kiss_send_frame(&ctx_tx, 5, payload, sizeof(payload));
    TEST_ASSERT(rc == KISS_OK, "TX frame encoded without error", rc);
    TEST_ASSERT(h_tx.tx_count == 1, "One TX write produced", h_tx.tx_count);

    if (h_tx.tx_count > 0) {
        DEBUG_BUF("Encoded KISS stream", h_tx.tx[0].data, h_tx.tx[0].len);

        // Feed encoded stream into RX engine
        inject(&ctx_rx, h_tx.tx[0].data, h_tx.tx[0].len);

        DEBUG_VAR("rx_count after loopback (expect 1)", h_rx.rx_count);
        TEST_ASSERT(h_rx.rx_count == 1, "RX engine dispatched one frame from loopback", h_rx.rx_count);

        if (h_rx.rx_count > 0) {
            DEBUG_VAR("Received port (expect 5)", h_rx.rx_port[0]);
            DEBUG_BUF("Received payload", h_rx.rx_frames[0], h_rx.rx_lens[0]);

            TEST_ASSERT(h_rx.rx_port[0] == 5, "Port 5 preserved through loopback", h_rx.rx_port[0]);
            TEST_ASSERT(h_rx.rx_lens[0] == sizeof(payload), "Payload length preserved through loopback", (unsigned int )h_rx.rx_lens[0]);
            TEST_ASSERT(memcmp(h_rx.rx_frames[0], payload, sizeof(payload)) == 0, "Payload bytes identical after TX->RX loopback", 0);
        }
    }

    return 0;
}

// ==========================================================================
// TEST 31: RX - maximum frame size boundary
// ==========================================================================
static int test_kiss_rx_max_frame_size(void) {
    printf("\n--- test_kiss_rx_max_frame_size ---\n");
    printf("Verify KISS_MAX_FRAME_SIZE boundary: max frame accepted, overflow truncated safely\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    // Build a frame with exactly KISS_MAX_FRAME_SIZE payload bytes
    size_t max_sz = KISS_MAX_FRAME_SIZE;
    uint8_t *raw = (uint8_t*) malloc(max_sz + 3);  // FEND + type + max payload + FEND
    if (!raw) {
        printf("  [SKIP] malloc failed\n");
        return 0;
    }

    raw[0] = KISS_FEND;
    raw[1] = KISS_TYPE_BYTE(0, KISS_CMD_DATA);
    for (size_t i = 0; i < max_sz; i++)
        raw[2 + i] = (uint8_t) (i & 0xFF);
    raw[max_sz + 2] = KISS_FEND;

    DEBUG_VAR("Injecting frame of exactly KISS_MAX_FRAME_SIZE payload bytes", max_sz);

    inject(&ctx, raw, max_sz + 3);

    DEBUG_VAR("rx_count (expect 1)", h.rx_count);
    TEST_ASSERT(h.rx_count == 1, "Max-size frame accepted and dispatched", h.rx_count);
    if (h.rx_count > 0) {
        DEBUG_VAR("Received payload length (expect KISS_MAX_FRAME_SIZE)", h.rx_lens[0]);
        TEST_ASSERT(h.rx_lens[0] == max_sz, "Received length = KISS_MAX_FRAME_SIZE", (unsigned int )h.rx_lens[0]);
    }
    free(raw);

    // Now one byte OVER: extra bytes should be silently dropped (no crash)
    memset(&h, 0, sizeof(h));
    ax25_kiss_init(&ctx);
    ctx.serial_write = harness_serial_write;
    ctx.on_frame = harness_on_frame;
    ctx.on_hardware = harness_on_hardware;
    ctx.on_return = harness_on_return;
    ctx.user_data = &h;

    size_t over_sz = max_sz + 10;
    uint8_t *raw2 = (uint8_t*) malloc(over_sz + 3);
    if (!raw2) {
        printf("  [SKIP] malloc failed\n");
        return 0;
    }
    raw2[0] = KISS_FEND;
    raw2[1] = KISS_TYPE_BYTE(0, KISS_CMD_DATA);
    for (size_t i = 0; i < over_sz; i++)
        raw2[2 + i] = (uint8_t) (i & 0xFF);
    raw2[over_sz + 2] = KISS_FEND;

    DEBUG_VAR("Injecting frame over KISS_MAX_FRAME_SIZE (expect truncation)", over_sz);

    inject(&ctx, raw2, over_sz + 3);

    TEST_ASSERT(h.rx_count == 1, "Oversized frame still dispatches (truncated)", h.rx_count);
    if (h.rx_count > 0) {
        DEBUG_VAR("Truncated receive length (expect KISS_MAX_FRAME_SIZE)", h.rx_lens[0]);
        TEST_ASSERT(h.rx_lens[0] == max_sz, "Oversized payload truncated at KISS_MAX_FRAME_SIZE", (unsigned int )h.rx_lens[0]);
    }
    free(raw2);

    return 0;
}

// ==========================================================================
// TEST 32: TX - empty payload (zero-length data frame)
// ==========================================================================
static int test_kiss_tx_empty_frame(void) {
    printf("\n--- test_kiss_tx_empty_frame ---\n");
    printf("Verify a zero-length payload produces: FEND type_byte FEND\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    uint8_t rc = ax25_kiss_send_frame(&ctx, 0, NULL, 0);

    DEBUG_VAR("send_frame(len=0) return code (expect 0)", rc);DEBUG_VAR("tx_count (expect 1)", h.tx_count);
    TEST_ASSERT(rc == KISS_OK, "send_frame(empty) returns KISS_OK", rc);
    TEST_ASSERT(h.tx_count == 1, "One TX write for empty frame", h.tx_count);

    if (h.tx_count > 0) {
        DEBUG_BUF("Encoded empty frame", h.tx[0].data, h.tx[0].len);
        // Must be: FEND + type_byte + FEND = 3 bytes
        TEST_ASSERT(h.tx[0].len == 3, "Empty frame encoded as 3 bytes", (unsigned int )h.tx[0].len);
        TEST_ASSERT(h.tx[0].data[0] == KISS_FEND, "Starts with FEND", 0);
        TEST_ASSERT(h.tx[0].data[1] == KISS_TYPE_BYTE(0, KISS_CMD_DATA), "Type byte correct", 0);
        TEST_ASSERT(h.tx[0].data[2] == KISS_FEND, "Ends with FEND", 0);
    }

    return 0;
}

// ==========================================================================
// TEST 33: Type byte construction and deconstruction macros
// ==========================================================================
static int test_kiss_type_byte_macros(void) {
    printf("\n--- test_kiss_type_byte_macros ---\n");
    printf("Verify KISS_TYPE_BYTE, KISS_PORT, KISS_CMD macros for all valid combinations\n");

    int failures = 0;
    for (uint8_t port = 0; port <= 15; port++) {
        for (uint8_t cmd = 0; cmd <= 15; cmd++) {
            uint8_t tb = KISS_TYPE_BYTE(port, cmd);
            uint8_t dp = KISS_PORT(tb);
            uint8_t dc = KISS_CMD(tb);
            if (dp != port || dc != cmd) {
                failures++;
                printf("  [FAIL] port=%u cmd=%u -> type=0x%02X -> port=%u cmd=%u\n", port, cmd, tb, dp, dc);
            }
        }
    }
    TEST_ASSERT(failures == 0, "KISS_TYPE_BYTE / KISS_PORT / KISS_CMD round-trip for all 256 combinations", failures);

    // Special: RETURN type byte
    DEBUG_HEX("KISS_RETURN_TYPE_BYTE (expect 0xFF)", KISS_RETURN_TYPE_BYTE);
    TEST_ASSERT(KISS_RETURN_TYPE_BYTE == 0xFFu, "KISS_RETURN_TYPE_BYTE = 0xFF", KISS_RETURN_TYPE_BYTE);

    return 0;
}

// ==========================================================================
// TEST 34: SETHARDWARE with max payload in set_port_params (hardware 16 bytes)
// ==========================================================================
static int test_kiss_sethardware_max(void) {
    printf("\n--- test_kiss_sethardware_max ---\n");
    printf("Verify SETHARDWARE command with maximum (16) hardware bytes in set_port_params\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    ax25_kiss_port_params_t p;
    memset(&p, 0, sizeof(p));
    p.txdelay = KISS_DEFAULT_TXDELAY;
    p.persistence = KISS_DEFAULT_PERSISTENCE;
    p.slottime = KISS_DEFAULT_SLOTTIME;
    p.txtail = KISS_DEFAULT_TXTAIL;
    p.full_duplex = false;
    p.hardware_len = 16u;
    for (uint8_t i = 0; i < 16; i++)
        p.hardware[i] = i;

    DEBUG_BUF("Hardware payload (16 bytes)", p.hardware, 16);

    uint8_t rc = ax25_kiss_set_port_params(&ctx, 0, &p);
    TEST_ASSERT(rc == KISS_OK, "set_port_params with 16 hw bytes returns KISS_OK", rc);
    TEST_ASSERT(h.tx_count == 6, "6 TX writes (5 params + SETHARDWARE)", h.tx_count);

    // Verify hardware bytes stored in port
    TEST_ASSERT(ctx.ports[0].hardware_len == 16u, "hardware_len = 16 stored", ctx.ports[0].hardware_len);
    bool hw_match = (memcmp(ctx.ports[0].hardware, p.hardware, 16) == 0);
    TEST_ASSERT(hw_match, "hardware[] bytes stored correctly", 0);

    return 0;
}

// ==========================================================================
// TEST 35: Unknown command silently ignored
// ==========================================================================
static int test_kiss_rx_unknown_cmd(void) {
    printf("\n--- test_kiss_rx_unknown_cmd ---\n");
    printf("Verify an unknown command (not 0x00-0x06 or 0x0F) is silently discarded\n");

    kiss_harness_t h;
    ax25_kiss_ctx_t ctx;
    harness_init(&h, &ctx);

    // Command 0x09 is not defined in the spec; must be silently ignored
    uint8_t raw[] = { KISS_FEND, KISS_TYPE_BYTE(0, 0x09), 0xAB, 0xCD, KISS_FEND };
    DEBUG_BUF("Injecting unknown command frame (cmd=0x09)", raw, sizeof(raw));

    inject(&ctx, raw, sizeof(raw));

    DEBUG_VAR("rx_count (expect 0 - not a DATA frame)", h.rx_count);DEBUG_VAR("hw_count (expect 0 - not SETHARDWARE)", h.hw_count);DEBUG_VAR("return_count (expect 0 - not RETURN)", h.return_count);
    TEST_ASSERT(h.rx_count == 0, "on_frame not fired for unknown command", h.rx_count);
    TEST_ASSERT(h.hw_count == 0, "on_hardware not fired for unknown command", h.hw_count);
    TEST_ASSERT(h.return_count == 0, "on_return not fired for unknown command", h.return_count);
    // No crash = success (state machine remains operational)
    // Inject a valid DATA frame after to confirm state machine still works
    uint8_t valid[] = { KISS_FEND, KISS_TYPE_BYTE(0, KISS_CMD_DATA), 0x55, KISS_FEND };
    inject(&ctx, valid, sizeof(valid));
    TEST_ASSERT(h.rx_count == 1, "State machine still works after unknown command", h.rx_count);

    return 0;
}

// ==========================================================================
// Main entry point
// ==========================================================================
int test_ax25_kiss_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("Starting KISS TNC Interface Protocol Tests\n");
    printf("  Per: Chepponis & Karn, ARRL 6th Computer Networking Conference, 1987\n");
    printf("  Reference: http://www.ka9q.net/papers/kiss.html\n");
    printf("==================================================================================\n");

    result |= test_kiss_init_defaults();
    result |= test_kiss_enter();
    result |= test_kiss_send_frame_basic();
    result |= test_kiss_tx_stuff_fend();
    result |= test_kiss_tx_stuff_fesc();
    result |= test_kiss_tx_stuff_consecutive();
    result |= test_kiss_tx_stuff_exhaustive();
    result |= test_kiss_rx_data_frame();
    result |= test_kiss_rx_escape_fend();
    result |= test_kiss_rx_escape_fesc();
    result |= test_kiss_rx_invalid_escape();
    result |= test_kiss_rx_consecutive_fend();
    result |= test_kiss_rx_idle_discard();
    result |= test_kiss_rx_fragmented();
    result |= test_kiss_rx_multiple_frames();
    result |= test_kiss_rx_cmd_txdelay();
    result |= test_kiss_rx_cmd_persistence();
    result |= test_kiss_rx_cmd_slottime();
    result |= test_kiss_rx_cmd_txtail();
    result |= test_kiss_rx_cmd_fullduplex();
    result |= test_kiss_rx_cmd_sethardware();
    result |= test_kiss_rx_cmd_return();
    result |= test_kiss_send_return();
    result |= test_kiss_multiport_data();
    result |= test_kiss_send_command_params();
    result |= test_kiss_set_get_port_params();
    result |= test_kiss_set_port_params_hardware();
    result |= test_kiss_error_codes();
    result |= test_kiss_mode_transitions();
    result |= test_kiss_round_trip_loopback();
    result |= test_kiss_rx_max_frame_size();
    result |= test_kiss_tx_empty_frame();
    result |= test_kiss_type_byte_macros();
    result |= test_kiss_sethardware_max();
    result |= test_kiss_rx_unknown_cmd();

    printf("\n==================================================================================\n");
    printf("KISS Tests Completed.  Total assertions: %u. %s\n", assert_count,
            result == 0 ? "\033[0;32mAll tests passed\033[0m" : "\033[0;31mSome tests FAILED\033[0m");
    printf("==================================================================================\n\n");

    return result;
}
