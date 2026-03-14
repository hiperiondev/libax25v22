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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * test_fx25_channel.c - FX.25 Noisy Channel Simulation Tests
 *
 * Tests NOT covered by test_fx25.c:
 *   - Real noisy channel simulation (random byte errors, burst errors)
 *   - Error correction at capacity boundary (exactly T and T+1 errors)
 *   - All 11 FX.25 modes under simulated noise
 *   - HDLC round-trip with injected RS-level byte errors
 *   - Correlation tag with 6-bit errors (tolerance boundary)
 *   - HDLC frame with errors in parity-only region vs data region
 *   - Zero-error clean channel baseline across all modes
 *   - Multi-pattern data (all-zero, all-0xFF, alternating, ramp)
 *   - Error injection at first, last, and middle byte positions
 *   - Burst error (consecutive bytes) correction
 *   - Verify corrected_errors count accuracy
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "test_common.h"
#include "fx25.h"
#include "fx25_rs.h"
#include "fx25_gf256.h"
#include "fx25_hdlc.h"

static uint32_t assert_count = 0;

// ============================================================================
// Pseudo-random number generator (LCG, deterministic seed for reproducibility)
// ============================================================================

static uint32_t prng_state = 0xDEADBEEFu;

// deterministic PRNG for reproducible noise injection
static uint32_t prng_next(void) {
    prng_state = prng_state * 1664525u + 1013904223u;
    return prng_state;
}

static void prng_seed(uint32_t seed) {
    prng_state = seed;
}

// ============================================================================
// Noise injection helpers
// ============================================================================

// inject exactly 'count' distinct byte errors at random positions
static void inject_byte_errors(uint8_t *buf, size_t buf_len, int count, uint32_t seed) {
    prng_seed(seed);
    // Track which positions have already been corrupted to avoid double-hit
    uint8_t used[512];
    memset(used, 0, sizeof(used));
    int injected = 0;
    int attempts = 0;
    while (injected < count && attempts < 10000) {
        attempts++;
        size_t pos = (size_t) (prng_next() % (uint32_t) buf_len);
        if (used[pos]) {
            continue;
        }
        used[pos] = 1;
        // XOR with non-zero random byte to guarantee corruption
        uint8_t corrupt_val = (uint8_t) ((prng_next() & 0xFE) | 0x01);
        buf[pos] ^= corrupt_val;
        DEBUG_HEX("  injected byte error at pos", (unsigned)pos);
        DEBUG_HEX("  corrupt_val", corrupt_val);
        injected++;
    }
}

// inject exactly 'burst_len' consecutive byte errors at 'start_pos'
static void inject_burst_errors(uint8_t *buf, size_t buf_len, size_t start_pos, int burst_len, uint32_t seed) {
    prng_seed(seed);
    for (int i = 0; i < burst_len; i++) {
        size_t pos = start_pos + (size_t) i;
        if (pos >= buf_len) {
            break;
        }
        uint8_t corrupt_val = (uint8_t) ((prng_next() & 0xFE) | 0x01);
        buf[pos] ^= corrupt_val;
        DEBUG_HEX("  injected burst error at pos", (unsigned)pos);
    }
}

// flip exactly 'bit_count' distinct bits inside tag[0..7]
static void inject_tag_bit_errors(uint8_t *tag, int bit_count, uint32_t seed) {
    prng_seed(seed);
    uint8_t used_bits[64];
    memset(used_bits, 0, sizeof(used_bits));
    int injected = 0;
    int attempts = 0;
    while (injected < bit_count && attempts < 10000) {
        attempts++;
        uint8_t bit_pos = (uint8_t) (prng_next() % 64u);
        if (used_bits[bit_pos]) {
            continue;
        }
        used_bits[bit_pos] = 1;
        uint8_t byte_idx = bit_pos / 8u;
        uint8_t bit_idx = bit_pos % 8u;
        tag[byte_idx] ^= (uint8_t) (1u << bit_idx);
        DEBUG_HEX("  flipped tag bit", bit_pos);
        injected++;
    }
}

// ============================================================================
// Helper: encode, optionally inject errors, decode, compare
// Returns 0 on test pass, 1 on failure.
// 'num_errors' errors are injected into the codeword region (after tag).
// 'expect_success': 1 if decode should succeed, 0 if it should fail.
// ============================================================================

// unified encode-inject-decode-verify helper
static int channel_round_trip(const char *test_name, const uint8_t *original_data, size_t data_len, uint8_t mode_id, int num_errors, uint32_t noise_seed,
        int expect_success) {
    char msg[128];

    // Encode
    fx25_frame_t tx_frame;
    memset(&tx_frame, 0, sizeof(tx_frame));
    uint8_t enc_err = fx25_encode(original_data, data_len, mode_id, &tx_frame);
    snprintf(msg, sizeof(msg), "[%s] encode succeeds (mode=0x%02X, data=%zu, errs=%d)", test_name, mode_id, data_len, num_errors);
    TEST_ASSERT(enc_err == 0, msg, enc_err);

    // Build rx buffer: tag + codeword
    size_t rx_buf_len = 8u + tx_frame.codeword_len;
    uint8_t *rx_buf = (uint8_t*) malloc(rx_buf_len);
    if (!rx_buf) {
        printf("\033[0;31m[%04d] FAIL: malloc failed in channel_round_trip\033[0m\n", ++assert_count);
        fx25_frame_free(&tx_frame);
        return 1;
    }
    memcpy(rx_buf, tx_frame.correlation_tag, 8u);
    memcpy(rx_buf + 8u, tx_frame.rs_codeword, tx_frame.codeword_len);
    fx25_frame_free(&tx_frame);

    // Inject errors into codeword region only (skip tag bytes 0..7)
    if (num_errors > 0) {
        inject_byte_errors(rx_buf + 8u, rx_buf_len - 8u, num_errors, noise_seed);
        DEBUG_PRINT("  %s: injected %d byte errors into codeword", test_name, num_errors);
    }

    // Decode
    fx25_frame_t rx_frame;
    memset(&rx_frame, 0, sizeof(rx_frame));
    uint8_t corrected_errors = 0;
    uint8_t dec_err = fx25_decode(rx_buf, rx_buf_len, &rx_frame, &corrected_errors);
    free(rx_buf);

    if (expect_success) {
        snprintf(msg, sizeof(msg), "[%s] decode succeeds (mode=0x%02X, errs=%d)", test_name, mode_id, num_errors);
        TEST_ASSERT(dec_err == 0, msg, dec_err);

        if (dec_err == 0) {
            snprintf(msg, sizeof(msg), "[%s] corrected_errors=%u (injected=%d)", test_name, corrected_errors, num_errors);
            // corrected_errors should equal injected count when within capacity
            TEST_ASSERT((int )corrected_errors == num_errors, msg, corrected_errors);

            // Verify recovered data matches original
            snprintf(msg, sizeof(msg), "[%s] recovered data matches original", test_name);
            int data_ok = memcmp(rx_frame.rs_codeword, original_data, data_len);
            TEST_ASSERT(data_ok == 0, msg, data_ok);
        }
    } else {
        snprintf(msg, sizeof(msg), "[%s] decode fails as expected (mode=0x%02X, errs=%d)", test_name, mode_id, num_errors);
        TEST_ASSERT(dec_err != 0, msg, dec_err);
    }

    fx25_frame_free(&rx_frame);
    return 0;
}

// ============================================================================
// Test 1: Clean channel baseline - all 11 modes, multiple data patterns
// ============================================================================

static int test_clean_channel_all_modes(void) {
    assert_count = 0;
    printf("\n--- test_clean_channel_all_modes ---\n");
    printf("  Verifies encode->decode round-trip with zero errors for all 11 modes\n");
    printf("  Data patterns: ramp, all-zero, all-0xFF, alternating 0xAA/0x55\n\n");

    // test all 11 modes with 4 data patterns, zero injected errors
    struct {
        uint8_t mode_id;
        uint16_t data_bytes;
        const char *name;
    } modes[] = { { FX25_MODE_239_16, 239, "239,16" }, { FX25_MODE_128_16, 128, "128,16" }, { FX25_MODE_64_16, 64, "64,16" }, { FX25_MODE_32_16, 32, "32,16" },
            { FX25_MODE_223_32, 223, "223,32" }, { FX25_MODE_128_32, 128, "128,32" }, { FX25_MODE_64_32, 64, "64,32" }, { FX25_MODE_32_32, 32, "32,32" }, {
            FX25_MODE_191_64, 191, "191,64" }, { FX25_MODE_128_64, 128, "128,64" }, { FX25_MODE_64_64, 64, "64,64" }, };

    uint8_t data_buf[239];

    for (int m = 0; m < 11; m++) {
        // Pattern A: ramp 0x00..0xEF
        for (int i = 0; i < modes[m].data_bytes; i++) {
            data_buf[i] = (uint8_t) (i & 0xFF);
        }
        char name[64];
        snprintf(name, sizeof(name), "clean/%s/ramp", modes[m].name);
        if (channel_round_trip(name, data_buf, modes[m].data_bytes, modes[m].mode_id, 0, 0, 1)) {
            return 1;
        }

        // Pattern B: all zeros
        memset(data_buf, 0x00, modes[m].data_bytes);
        snprintf(name, sizeof(name), "clean/%s/zeros", modes[m].name);
        if (channel_round_trip(name, data_buf, modes[m].data_bytes, modes[m].mode_id, 0, 0, 1)) {
            return 1;
        }

        // Pattern C: all 0xFF
        memset(data_buf, 0xFF, modes[m].data_bytes);
        snprintf(name, sizeof(name), "clean/%s/0xFF", modes[m].name);
        if (channel_round_trip(name, data_buf, modes[m].data_bytes, modes[m].mode_id, 0, 0, 1)) {
            return 1;
        }

        // Pattern D: alternating 0xAA/0x55
        for (int i = 0; i < modes[m].data_bytes; i++) {
            data_buf[i] = (i & 1) ? 0x55u : 0xAAu;
        }
        snprintf(name, sizeof(name), "clean/%s/alt", modes[m].name);
        if (channel_round_trip(name, data_buf, modes[m].data_bytes, modes[m].mode_id, 0, 0, 1)) {
            return 1;
        }
    }

    return 0;
}

// ============================================================================
// Test 2: Exactly T errors (at capacity) - must succeed for all modes
// ============================================================================

static int test_exactly_T_errors_all_modes(void) {
    assert_count = 0;
    printf("\n--- test_exactly_T_errors_all_modes ---\n");
    printf("  Injects exactly T byte errors (correction capacity) into each mode\n");
    printf("  Expects successful decode and corrected_errors == T\n\n");

    // boundary correction test: exactly T errors per mode
    struct {
        uint8_t mode_id;
        uint16_t data_bytes;
        int t;          // error correction capacity
        const char *name;
    } modes[] =
            { { FX25_MODE_239_16, 239, 8, "239,16" }, { FX25_MODE_128_16, 128, 8, "128,16" }, { FX25_MODE_64_16, 64, 8, "64,16" }, { FX25_MODE_32_16, 32, 8,
                    "32,16" }, { FX25_MODE_223_32, 223, 16, "223,32" }, { FX25_MODE_128_32, 128, 16, "128,32" }, { FX25_MODE_64_32, 64, 16, "64,32" }, {
            FX25_MODE_32_32, 32, 16, "32,32" }, { FX25_MODE_191_64, 191, 32, "191,64" }, { FX25_MODE_128_64, 128, 32, "128,64" }, { FX25_MODE_64_64, 64, 32,
                    "64,64" }, };

    uint8_t data_buf[239];

    for (int m = 0; m < 11; m++) {
        // Use ramp data
        for (int i = 0; i < modes[m].data_bytes; i++) {
            data_buf[i] = (uint8_t) (i & 0xFF);
        }
        char name[64];
        snprintf(name, sizeof(name), "T_exact/%s/T=%d", modes[m].name, modes[m].t);
        printf("  Testing mode %s: injecting %d errors (T=%d)\n", modes[m].name, modes[m].t, modes[m].t);
        // seed differs per mode to ensure independent error positions
        uint32_t seed = 0x1234u + (uint32_t) m * 0x1111u;
        if (channel_round_trip(name, data_buf, modes[m].data_bytes, modes[m].mode_id, modes[m].t, seed, 1)) {
            return 1;
        }
    }

    return 0;
}

// ============================================================================
// Test 3: T+1 errors (one beyond capacity) - must fail to decode
// ============================================================================

static int test_beyond_capacity_errors(void) {
    assert_count = 0;
    printf("\n--- test_beyond_capacity_errors ---\n");
    printf("  Injects T+1 byte errors (one beyond capacity) into selected modes\n");
    printf("  Expects decode to fail (return non-zero)\n\n");

    // test T+1 errors cause decode failure
    struct {
        uint8_t mode_id;
        uint16_t data_bytes;
        int t;
        const char *name;
    } modes[] = { { FX25_MODE_32_16, 32, 8, "32,16" }, { FX25_MODE_64_32, 64, 16, "64,32" }, { FX25_MODE_64_64, 64, 32, "64,64" }, };

    uint8_t data_buf[239];

    for (int m = 0; m < 3; m++) {
        for (int i = 0; i < modes[m].data_bytes; i++) {
            data_buf[i] = (uint8_t) (i & 0xFF);
        }
        int over_capacity = modes[m].t + 1;
        char name[64];
        snprintf(name, sizeof(name), "T+1/%s/errs=%d", modes[m].name, over_capacity);
        printf("  Testing mode %s: injecting %d errors (T+1=%d) - expect failure\n", modes[m].name, over_capacity, over_capacity);
        uint32_t seed = 0xABCDu + (uint32_t) m * 0x5555u;
        // expect_success=0: decode should fail
        if (channel_round_trip(name, data_buf, modes[m].data_bytes, modes[m].mode_id, over_capacity, seed, 0)) {
            return 1;
        }
    }

    return 0;
}

// ============================================================================
// Test 4: Errors at boundary positions (first, last, middle byte)
// ============================================================================

static int test_errors_at_boundary_positions(void) {
    assert_count = 0;
    printf("\n--- test_errors_at_boundary_positions ---\n");
    printf("  Injects 1 byte error at first, middle, and last byte of codeword\n");
    printf("  Expects successful decode for each position\n\n");

    // positional error injection: first/middle/last byte
    uint8_t mode_id = FX25_MODE_64_32;
    uint16_t data_len = 64;
    int parity_len = 32;
    size_t codeword_len = (size_t) (data_len + parity_len);
    char msg[128];

    uint8_t data_buf[64];
    for (int i = 0; i < data_len; i++) {
        data_buf[i] = (uint8_t) (i & 0xFF);
    }

    uint8_t positions_label[3][16] = { "first", "middle", "last" };
    size_t positions[3] = { 0, codeword_len / 2, codeword_len - 1 };

    for (int p = 0; p < 3; p++) {
        fx25_frame_t tx_frame;
        memset(&tx_frame, 0, sizeof(tx_frame));
        uint8_t enc_err = fx25_encode(data_buf, data_len, mode_id, &tx_frame);
        snprintf(msg, sizeof(msg), "[pos/%s] encode succeeds", positions_label[p]);
        TEST_ASSERT(enc_err == 0, msg, enc_err);

        size_t rx_buf_len = 8u + tx_frame.codeword_len;
        uint8_t *rx_buf = (uint8_t*) malloc(rx_buf_len);
        if (!rx_buf) {
            fx25_frame_free(&tx_frame);
            return 1;
        }
        memcpy(rx_buf, tx_frame.correlation_tag, 8u);
        memcpy(rx_buf + 8u, tx_frame.rs_codeword, tx_frame.codeword_len);
        fx25_frame_free(&tx_frame);

        // Corrupt exactly one byte at specified position in codeword
        rx_buf[8u + positions[p]] ^= 0x5A;
        DEBUG_PRINT("  Corrupted codeword[%zu] (codeword position: %s)",
                positions[p], positions_label[p]);

        fx25_frame_t rx_frame;
        memset(&rx_frame, 0, sizeof(rx_frame));
        uint8_t corrected = 0;
        uint8_t dec_err = fx25_decode(rx_buf, rx_buf_len, &rx_frame, &corrected);
        free(rx_buf);

        snprintf(msg, sizeof(msg), "[pos/%s] decode succeeds", positions_label[p]);
        TEST_ASSERT(dec_err == 0, msg, dec_err);

        if (dec_err == 0) {
            snprintf(msg, sizeof(msg), "[pos/%s] corrected_errors == 1 (got %u)", positions_label[p], corrected);
            TEST_ASSERT(corrected == 1, msg, corrected);

            snprintf(msg, sizeof(msg), "[pos/%s] data matches original", positions_label[p]);
            int data_ok = memcmp(rx_frame.rs_codeword, data_buf, data_len);
            TEST_ASSERT(data_ok == 0, msg, data_ok);
        }

        fx25_frame_free(&rx_frame);
    }

    return 0;
}

// ============================================================================
// Test 5: Burst errors (consecutive bytes corrupted)
// ============================================================================

static int test_burst_errors(void) {
    assert_count = 0;
    printf("\n--- test_burst_errors ---\n");
    printf("  Injects consecutive byte burst errors of various lengths\n");
    printf("  Uses mode 64,64 (T=32) to allow large bursts\n\n");

    // burst error simulation
    uint8_t mode_id = FX25_MODE_64_64;
    uint16_t data_len = 64;

    uint8_t data_buf[64];
    for (int i = 0; i < data_len; i++) {
        data_buf[i] = (uint8_t) (i & 0xFF);
    }

    // Test burst lengths: 1, T/4, T/2, T, T+1 (last should fail)
    int burst_lengths[] = { 1, 8, 16, 32, 33 };
    int expect_success[] = { 1, 1, 1, 1, 0 };
    size_t start_pos = 5;  // inject burst starting at codeword byte 5

    for (int b = 0; b < 5; b++) {
        int blen = burst_lengths[b];
        char msg[128];

        fx25_frame_t tx_frame;
        memset(&tx_frame, 0, sizeof(tx_frame));
        uint8_t enc_err = fx25_encode(data_buf, data_len, mode_id, &tx_frame);
        snprintf(msg, sizeof(msg), "[burst/%d] encode succeeds", blen);
        TEST_ASSERT(enc_err == 0, msg, enc_err);

        size_t rx_buf_len = 8u + tx_frame.codeword_len;
        uint8_t *rx_buf = (uint8_t*) malloc(rx_buf_len);
        if (!rx_buf) {
            fx25_frame_free(&tx_frame);
            return 1;
        }
        memcpy(rx_buf, tx_frame.correlation_tag, 8u);
        memcpy(rx_buf + 8u, tx_frame.rs_codeword, tx_frame.codeword_len);
        fx25_frame_free(&tx_frame);

        // Inject burst into codeword region
        inject_burst_errors(rx_buf + 8u, rx_buf_len - 8u, start_pos, blen, 0xBEEFu + (uint32_t) b);
        DEBUG_PRINT("  burst of %d bytes starting at codeword[%zu]", blen, start_pos);

        fx25_frame_t rx_frame;
        memset(&rx_frame, 0, sizeof(rx_frame));
        uint8_t corrected = 0;
        uint8_t dec_err = fx25_decode(rx_buf, rx_buf_len, &rx_frame, &corrected);
        free(rx_buf);

        if (expect_success[b]) {
            snprintf(msg, sizeof(msg), "[burst/%d] decode succeeds", blen);
            TEST_ASSERT(dec_err == 0, msg, dec_err);
            if (dec_err == 0) {
                snprintf(msg, sizeof(msg), "[burst/%d] corrected=%u == burst_len=%d", blen, corrected, blen);
                TEST_ASSERT((int )corrected == blen, msg, corrected);
                snprintf(msg, sizeof(msg), "[burst/%d] data matches original", blen);
                int data_ok = memcmp(rx_frame.rs_codeword, data_buf, data_len);
                TEST_ASSERT(data_ok == 0, msg, data_ok);
            }
        } else {
            snprintf(msg, sizeof(msg), "[burst/%d] decode fails as expected (T+1 burst)", blen);
            TEST_ASSERT(dec_err != 0, msg, dec_err);
        }

        fx25_frame_free(&rx_frame);
    }

    return 0;
}

// ============================================================================
// Test 6: Correlation tag bit-error tolerance boundary
// ============================================================================

static int test_tag_bit_error_tolerance(void) {
    assert_count = 0;
    printf("\n--- test_tag_bit_error_tolerance ---\n");
    printf("  Injects 0..6 bit errors into correlation tag (FX.25 tolerates up to 6)\n");
    printf("  Injects 7 bit errors to confirm decode failure\n\n");

    // correlation tag Hamming distance boundary tests
    uint8_t mode_id = FX25_MODE_64_32;
    uint16_t data_len = 64;

    uint8_t data_buf[64];
    for (int i = 0; i < data_len; i++) {
        data_buf[i] = (uint8_t) (i & 0xFF);
    }

    // FX.25 spec section 2.2: tag tolerance is 6 bit errors
    // Test: 0,1,2,3,4,5,6 bit errors -> success; 7 bit errors -> fail
    int bit_error_counts[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    int expect_success[] = { 1, 1, 1, 1, 1, 1, 1, 0 };

    for (int t = 0; t < 8; t++) {
        int nbits = bit_error_counts[t];
        char msg[128];

        fx25_frame_t tx_frame;
        memset(&tx_frame, 0, sizeof(tx_frame));
        uint8_t enc_err = fx25_encode(data_buf, data_len, mode_id, &tx_frame);
        snprintf(msg, sizeof(msg), "[tag_bits/%d] encode succeeds", nbits);
        TEST_ASSERT(enc_err == 0, msg, enc_err);

        size_t rx_buf_len = 8u + tx_frame.codeword_len;
        uint8_t *rx_buf = (uint8_t*) malloc(rx_buf_len);
        if (!rx_buf) {
            fx25_frame_free(&tx_frame);
            return 1;
        }
        memcpy(rx_buf, tx_frame.correlation_tag, 8u);
        memcpy(rx_buf + 8u, tx_frame.rs_codeword, tx_frame.codeword_len);
        fx25_frame_free(&tx_frame);

        // Inject bit errors into the tag (first 8 bytes of rx_buf)
        if (nbits > 0) {
            inject_tag_bit_errors(rx_buf, nbits, 0xC0DE0u + (uint32_t) t);
        }

        fx25_frame_t rx_frame;
        memset(&rx_frame, 0, sizeof(rx_frame));
        uint8_t corrected = 0;
        uint8_t dec_err = fx25_decode(rx_buf, rx_buf_len, &rx_frame, &corrected);
        free(rx_buf);

        if (expect_success[t]) {
            snprintf(msg, sizeof(msg), "[tag_bits/%d] decode succeeds (Hamming dist %d <= 6)", nbits, nbits);
            TEST_ASSERT(dec_err == 0, msg, dec_err);
            if (dec_err == 0) {
                snprintf(msg, sizeof(msg), "[tag_bits/%d] mode correctly identified", nbits);
                TEST_ASSERT(rx_frame.mode_id == mode_id, msg, rx_frame.mode_id);
            }
        } else {
            snprintf(msg, sizeof(msg), "[tag_bits/%d] decode fails (Hamming dist %d > 6)", nbits, nbits);
            TEST_ASSERT(dec_err != 0, msg, dec_err);
        }

        fx25_frame_free(&rx_frame);
    }

    return 0;
}

// ============================================================================
// Test 7: Errors in parity-only region vs data region
// ============================================================================

static int test_errors_parity_vs_data_region(void) {
    assert_count = 0;
    printf("\n--- test_errors_parity_vs_data_region ---\n");
    printf("  Injects errors exclusively in parity region vs data region\n");
    printf("  Both should recover correctly within T limit\n\n");

    // region-specific error injection
    uint8_t mode_id = FX25_MODE_64_32;
    uint16_t data_len = 64;
    int parity_len = 32;
    char msg[128];

    uint8_t data_buf[64];
    for (int i = 0; i < data_len; i++) {
        data_buf[i] = (uint8_t) (i & 0xFF);
    }

    // ---- Errors in DATA region only ----
    {
        fx25_frame_t tx_frame;
        memset(&tx_frame, 0, sizeof(tx_frame));
        uint8_t enc_err = fx25_encode(data_buf, data_len, mode_id, &tx_frame);
        TEST_ASSERT(enc_err == 0, "[region/data] encode succeeds", enc_err);

        size_t rx_buf_len = 8u + tx_frame.codeword_len;
        uint8_t *rx_buf = (uint8_t*) malloc(rx_buf_len);
        memcpy(rx_buf, tx_frame.correlation_tag, 8u);
        memcpy(rx_buf + 8u, tx_frame.rs_codeword, tx_frame.codeword_len);
        fx25_frame_free(&tx_frame);

        // Inject 8 errors in data bytes only [codeword pos 0 .. data_len-1]
        int n = 8;
        prng_seed(0x1111u);
        uint8_t used[256];
        memset(used, 0, sizeof(used));
        int injected = 0;
        int attempts = 0;
        while (injected < n && attempts < 10000) {
            attempts++;
            size_t pos = (size_t) (prng_next() % (uint32_t) data_len);
            if (used[pos]) {
                continue;
            }
            used[pos] = 1;
            uint8_t cv = (uint8_t) ((prng_next() & 0xFE) | 0x01);
            rx_buf[8u + pos] ^= cv;
            injected++;
        }DEBUG_PRINT("  Injected %d errors in data region", injected);

        fx25_frame_t rx_frame;
        memset(&rx_frame, 0, sizeof(rx_frame));
        uint8_t corrected = 0;
        uint8_t dec_err = fx25_decode(rx_buf, rx_buf_len, &rx_frame, &corrected);
        free(rx_buf);

        TEST_ASSERT(dec_err == 0, "[region/data] decode succeeds", dec_err);
        if (dec_err == 0) {
            snprintf(msg, sizeof(msg), "[region/data] corrected=%u == %d", corrected, n);
            TEST_ASSERT((int )corrected == n, msg, corrected);
            int data_ok = memcmp(rx_frame.rs_codeword, data_buf, data_len);
            TEST_ASSERT(data_ok == 0, "[region/data] data matches original", data_ok);
        }
        fx25_frame_free(&rx_frame);
    }

    // ---- Errors in PARITY region only ----
    {
        fx25_frame_t tx_frame;
        memset(&tx_frame, 0, sizeof(tx_frame));
        uint8_t enc_err = fx25_encode(data_buf, data_len, mode_id, &tx_frame);
        TEST_ASSERT(enc_err == 0, "[region/parity] encode succeeds", enc_err);

        size_t rx_buf_len = 8u + tx_frame.codeword_len;
        uint8_t *rx_buf = (uint8_t*) malloc(rx_buf_len);
        memcpy(rx_buf, tx_frame.correlation_tag, 8u);
        memcpy(rx_buf + 8u, tx_frame.rs_codeword, tx_frame.codeword_len);
        fx25_frame_free(&tx_frame);

        // Inject 8 errors in parity bytes only [codeword pos data_len .. data_len+parity_len-1]
        int n = 8;
        prng_seed(0x2222u);
        uint8_t used[64];
        memset(used, 0, sizeof(used));
        int injected = 0;
        int attempts = 0;
        while (injected < n && attempts < 10000) {
            attempts++;
            size_t pos = (size_t) (data_len + prng_next() % (uint32_t) parity_len);
            size_t rel = pos - data_len;
            if (used[rel]) {
                continue;
            }
            used[rel] = 1;
            uint8_t cv = (uint8_t) ((prng_next() & 0xFE) | 0x01);
            rx_buf[8u + pos] ^= cv;
            injected++;
        }DEBUG_PRINT("  Injected %d errors in parity region", injected);

        fx25_frame_t rx_frame;
        memset(&rx_frame, 0, sizeof(rx_frame));
        uint8_t corrected = 0;
        uint8_t dec_err = fx25_decode(rx_buf, rx_buf_len, &rx_frame, &corrected);
        free(rx_buf);

        TEST_ASSERT(dec_err == 0, "[region/parity] decode succeeds", dec_err);
        if (dec_err == 0) {
            snprintf(msg, sizeof(msg), "[region/parity] corrected=%u == %d", corrected, n);
            TEST_ASSERT((int )corrected == n, msg, corrected);
            int data_ok = memcmp(rx_frame.rs_codeword, data_buf, data_len);
            TEST_ASSERT(data_ok == 0, "[region/parity] data matches original", data_ok);
        }
        fx25_frame_free(&rx_frame);
    }

    // ---- Mixed errors spanning both regions ----
    {
        fx25_frame_t tx_frame;
        memset(&tx_frame, 0, sizeof(tx_frame));
        uint8_t enc_err = fx25_encode(data_buf, data_len, mode_id, &tx_frame);
        TEST_ASSERT(enc_err == 0, "[region/mixed] encode succeeds", enc_err);

        size_t rx_buf_len = 8u + tx_frame.codeword_len;
        uint8_t *rx_buf = (uint8_t*) malloc(rx_buf_len);
        memcpy(rx_buf, tx_frame.correlation_tag, 8u);
        memcpy(rx_buf + 8u, tx_frame.rs_codeword, tx_frame.codeword_len);
        fx25_frame_free(&tx_frame);

        // 5 errors in data + 5 errors in parity = 10 total (T=16)
        int n_data = 5, n_parity = 5;
        prng_seed(0x3333u);

        // data errors
        uint8_t ud[256];
        memset(ud, 0, sizeof(ud));
        int inj = 0, att = 0;
        while (inj < n_data && att < 10000) {
            att++;
            size_t pos = (size_t) (prng_next() % (uint32_t) data_len);
            if (ud[pos]) {
                continue;
            }
            ud[pos] = 1;
            rx_buf[8u + pos] ^= (uint8_t) ((prng_next() & 0xFE) | 0x01);
            inj++;
        }
        // parity errors
        uint8_t up[64];
        memset(up, 0, sizeof(up));
        inj = 0;
        att = 0;
        while (inj < n_parity && att < 10000) {
            att++;
            size_t rel = (size_t) (prng_next() % (uint32_t) parity_len);
            if (up[rel]) {
                continue;
            }
            up[rel] = 1;
            rx_buf[8u + data_len + rel] ^= (uint8_t) ((prng_next() & 0xFE) | 0x01);
            inj++;
        }DEBUG_PRINT("  Injected %d+%d=10 mixed region errors", n_data, n_parity);

        fx25_frame_t rx_frame;
        memset(&rx_frame, 0, sizeof(rx_frame));
        uint8_t corrected = 0;
        uint8_t dec_err = fx25_decode(rx_buf, rx_buf_len, &rx_frame, &corrected);
        free(rx_buf);

        TEST_ASSERT(dec_err == 0, "[region/mixed] decode succeeds", dec_err);
        if (dec_err == 0) {
            snprintf(msg, sizeof(msg), "[region/mixed] corrected=%u == %d", corrected, n_data + n_parity);
            TEST_ASSERT((int )corrected == n_data + n_parity, msg, corrected);
            int data_ok = memcmp(rx_frame.rs_codeword, data_buf, data_len);
            TEST_ASSERT(data_ok == 0, "[region/mixed] data matches original", data_ok);
        }
        fx25_frame_free(&rx_frame);
    }

    return 0;
}

// ============================================================================
// Test 8: HDLC layer round-trip with simulated RS-level byte errors
// ============================================================================

static int test_hdlc_noisy_channel(void) {
    assert_count = 0;
    printf("\n--- test_hdlc_noisy_channel ---\n");
    printf("  Tests fx25_hdlc_encode -> noise injection -> fx25_hdlc_decode\n");
    printf("  Uses multiple error counts within correction capacity\n\n");

    // full HDLC layer noisy channel simulation
    // Construct a minimal valid-looking AX.25 frame (14 bytes addr + 1 ctrl + 2 FCS)
    // We test the HDLC layer so we pass raw bytes and verify round-trip.
    uint8_t ax25_frame[20];
    for (int i = 0; i < 20; i++) {
        ax25_frame[i] = (uint8_t) (0x41 + i);  // 'A'..'T'
    }

    // Use mode 64,32 (T=16) for HDLC test
    uint8_t mode_id = FX25_MODE_64_32;

    // Error counts to test: 0, 1, 4, 8 (all within T=16 after HDLC overhead)
    int err_counts[] = { 0, 1, 4, 8 };
    char msg[128];

    for (int e = 0; e < 4; e++) {
        int n_err = err_counts[e];

        // Encode via HDLC layer
        uint8_t tx_buf[512];
        size_t tx_len = 0;
        fx25_hdlc_err_t enc_err = fx25_hdlc_encode(ax25_frame, 20, mode_id, 80, tx_buf, &tx_len);
        snprintf(msg, sizeof(msg), "[hdlc_noisy/errs=%d] hdlc_encode succeeds", n_err);
        TEST_ASSERT(enc_err == FX25_HDLC_OK, msg, (int )enc_err);

        if (enc_err != FX25_HDLC_OK) {
            continue;
        }

        // tx_buf layout: [preamble 4][tag 8][codeword N][postamble 4]
        // Inject errors into codeword region only (skip preamble+tag = 4+8=12 bytes)
        if (n_err > 0) {
            size_t preamble_tag_len = 4u + 8u;  // 4 preamble + 8 tag
            size_t codeword_region = tx_len - preamble_tag_len - 4u;  // minus postamble
            if (codeword_region > 0) {
                inject_byte_errors(tx_buf + preamble_tag_len, codeword_region, n_err, 0xF00Du + (uint32_t) e);
                DEBUG_PRINT("  [hdlc_noisy/errs=%d] injected %d errors into codeword region (%zu bytes)",
                        n_err, n_err, codeword_region);
            }
        }

        // Decode via HDLC layer
        uint8_t rx_frame[256];
        size_t rx_len = 0;
        uint8_t corrected = 0;
        fx25_hdlc_err_t dec_err = fx25_hdlc_decode(tx_buf, tx_len, rx_frame, &rx_len, &corrected);

        snprintf(msg, sizeof(msg), "[hdlc_noisy/errs=%d] hdlc_decode succeeds", n_err);
        TEST_ASSERT(dec_err == FX25_HDLC_OK, msg, (int )dec_err);

        if (dec_err == FX25_HDLC_OK) {
            snprintf(msg, sizeof(msg), "[hdlc_noisy/errs=%d] decoded length == 20 (got %zu)", n_err, rx_len);
            TEST_ASSERT(rx_len == 20u, msg, (int )rx_len);

            if (rx_len == 20u) {
                int data_ok = memcmp(rx_frame, ax25_frame, 20u);
                snprintf(msg, sizeof(msg), "[hdlc_noisy/errs=%d] data matches original", n_err);
                TEST_ASSERT(data_ok == 0, msg, data_ok);
            }
        }
    }

    return 0;
}

// ============================================================================
// Test 9: Noise sweep - simulate increasing BER across all modes
// ============================================================================

static int test_noise_sweep_all_modes(void) {
    assert_count = 0;
    printf("\n--- test_noise_sweep_all_modes ---\n");
    printf("  Sweeps 0..T errors in steps of T/4 for all 11 modes\n");
    printf("  Verifies that correction succeeds at each step\n\n");

    // BER sweep simulation across all modes
    struct {
        uint8_t mode_id;
        uint16_t data_bytes;
        int t;
        const char *name;
    } modes[] =
            { { FX25_MODE_239_16, 239, 8, "239,16" }, { FX25_MODE_128_16, 128, 8, "128,16" }, { FX25_MODE_64_16, 64, 8, "64,16" }, { FX25_MODE_32_16, 32, 8,
                    "32,16" }, { FX25_MODE_223_32, 223, 16, "223,32" }, { FX25_MODE_128_32, 128, 16, "128,32" }, { FX25_MODE_64_32, 64, 16, "64,32" }, {
            FX25_MODE_32_32, 32, 16, "32,32" }, { FX25_MODE_191_64, 191, 32, "191,64" }, { FX25_MODE_128_64, 128, 32, "128,64" }, { FX25_MODE_64_64, 64, 32,
                    "64,64" }, };

    uint8_t data_buf[239];

    for (int m = 0; m < 11; m++) {
        for (int i = 0; i < modes[m].data_bytes; i++) {
            data_buf[i] = (uint8_t) (i & 0xFF);
        }

        // Steps: 0, T/4, T/2, 3T/4, T
        int steps[5];
        steps[0] = 0;
        steps[1] = modes[m].t / 4;
        steps[2] = modes[m].t / 2;
        steps[3] = (modes[m].t * 3) / 4;
        steps[4] = modes[m].t;

        for (int s = 0; s < 5; s++) {
            int n_err = steps[s];
            char name[64];
            snprintf(name, sizeof(name), "sweep/%s/errs=%d", modes[m].name, n_err);
            uint32_t seed = 0xFACEu + (uint32_t) (m * 100 + s);
            if (channel_round_trip(name, data_buf, modes[m].data_bytes, modes[m].mode_id, n_err, seed, 1)) {
                return 1;
            }
        }
    }

    return 0;
}

// ============================================================================
// Test 10: corrected_errors count accuracy
// ============================================================================

static int test_corrected_errors_count_accuracy(void) {
    assert_count = 0;
    printf("\n--- test_corrected_errors_count_accuracy ---\n");
    printf("  Injects 1..T errors one by one and verifies corrected_errors matches\n");
    printf("  Uses mode 32,32 (T=16) for comprehensive sweep\n\n");

    // verify corrected_errors count accuracy for each error count
    uint8_t mode_id = FX25_MODE_32_32;
    uint16_t data_len = 32;
    int T = 16;
    char msg[128];

    uint8_t data_buf[32];
    for (int i = 0; i < data_len; i++) {
        data_buf[i] = (uint8_t) (i & 0xFF);
    }

    for (int n = 0; n <= T; n++) {
        fx25_frame_t tx_frame;
        memset(&tx_frame, 0, sizeof(tx_frame));
        uint8_t enc_err = fx25_encode(data_buf, data_len, mode_id, &tx_frame);
        snprintf(msg, sizeof(msg), "[count_acc/n=%d] encode succeeds", n);
        TEST_ASSERT(enc_err == 0, msg, enc_err);

        size_t rx_buf_len = 8u + tx_frame.codeword_len;
        uint8_t *rx_buf = (uint8_t*) malloc(rx_buf_len);
        memcpy(rx_buf, tx_frame.correlation_tag, 8u);
        memcpy(rx_buf + 8u, tx_frame.rs_codeword, tx_frame.codeword_len);
        fx25_frame_free(&tx_frame);

        if (n > 0) {
            inject_byte_errors(rx_buf + 8u, tx_frame.codeword_len, n, 0xCAFEu + (uint32_t) n);
        }

        fx25_frame_t rx_frame;
        memset(&rx_frame, 0, sizeof(rx_frame));
        uint8_t corrected = 0;
        uint8_t dec_err = fx25_decode(rx_buf, rx_buf_len, &rx_frame, &corrected);
        free(rx_buf);

        snprintf(msg, sizeof(msg), "[count_acc/n=%d] decode succeeds", n);
        TEST_ASSERT(dec_err == 0, msg, dec_err);

        if (dec_err == 0) {
            snprintf(msg, sizeof(msg), "[count_acc/n=%d] corrected_errors=%u == %d", n, corrected, n);
            TEST_ASSERT((int )corrected == n, msg, corrected);

            int data_ok = memcmp(rx_frame.rs_codeword, data_buf, data_len);
            snprintf(msg, sizeof(msg), "[count_acc/n=%d] data matches original", n);
            TEST_ASSERT(data_ok == 0, msg, data_ok);
        }

        fx25_frame_free(&rx_frame);
    }

    return 0;
}

// ============================================================================
// Main entry point
// ============================================================================

int test_fx25_channel_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("Starting FX.25 Noisy Channel Simulation Tests\n");
    printf("==================================================================================\n\n");

    // register and run all channel simulation tests
    result |= test_clean_channel_all_modes();
    result |= test_exactly_T_errors_all_modes();
    result |= test_beyond_capacity_errors();
    result |= test_errors_at_boundary_positions();
    result |= test_burst_errors();
    result |= test_tag_bit_error_tolerance();
    result |= test_errors_parity_vs_data_region();
    result |= test_hdlc_noisy_channel();
    result |= test_noise_sweep_all_modes();
    result |= test_corrected_errors_count_accuracy();

    printf("\n==================================================================================\n");
    printf("FX.25 Noisy Channel Tests Completed. %s\n", result == 0 ? "All tests passed" : "Some tests FAILED");
    printf("==================================================================================\n\n");

    return result;
}
