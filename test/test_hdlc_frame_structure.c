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

// HDLC Frame Structure Tests - Section 1 (FULLY INSTRUMENTED - DEBUG VERSION)
// Covers: Abort Sequence, NRZI Encoding/Decoding, Idle Pattern Generation
// AX.25 v2.2 Sections 3.7, 3.8, 3.10
//
// === MASSIVE DEBUG ADDED (March 2026) ===
//   - debug_bitstream() on EVERY encode/decode
//   - Per-bit NRZI logging
//   - Transition counting
//   - State machine dumps for abort / stuffing
//   - Every test starts with "=== ENTERING TEST: xxx ==="
//   - All error paths show exact bytes/bits
#define DEBUG_ENABLE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "test_common.h"
#include "hdlc.h"
#include "common.h"
#include "hal.h"

// Counter required by TEST_ASSERT
static uint32_t assert_count = 0;

// ---------------------------------------------------------------------------
// NEW: Ultra-detailed bitstream dump (LSB-first = HDLC order)
// ---------------------------------------------------------------------------
static void debug_bitstream(const char *label, const unsigned char *buf, int len) {
    DEBUG_PRINT("=== BITSTREAM DUMP: %s (%d bytes) ===", label, len);
    for (int i = 0; i < len; i++) {
        printf("  [%02d] 0x%02X = ", i, buf[i]);
        for (int b = 0; b < 8; b++)
            printf("%d", (buf[i] >> b) & 1);
        printf("  (LSB-first)\n");
    }DEBUG_FRAME("Raw hex", buf, len);
}

// ---------------------------------------------------------------------------
// NEW: Count NRZI transitions
// ---------------------------------------------------------------------------
static int count_nrzi_transitions(const uint8_t *levels, int bit_count) {
    if (!levels || bit_count <= 0)
        return 0;
    int t = 0;
    uint8_t prev = 1u;

    for (int i = 0; i < bit_count; i++) {
        if (levels[i] != prev)
            t++;
        prev = levels[i];
    }
    return t;
}

// ---------------------------------------------------------------------------
// Utilities (kept + logged)
// ---------------------------------------------------------------------------
static int max_consecutive_ones_lsb(const unsigned char *buf, int byte_len) {
    int max_run = 0, run = 0;
    for (int i = 0; i < byte_len; i++) {
        for (int bit = 0; bit < 8; bit++) {
            if ((buf[i] >> bit) & 0x01) {
                run++;
                if (run > max_run)
                    max_run = run;
            } else
                run = 0;
        }
    }
    return max_run;
}

static int max_consecutive_ones_msb(const unsigned char *buf, int byte_len) {
    int max_run = 0, run = 0;
    for (int i = 0; i < byte_len; i++) {
        for (int bit = 7; bit >= 0; bit--) {
            if ((buf[i] >> bit) & 0x01) {
                run++;
                if (run > max_run)
                    max_run = run;
            } else
                run = 0;
        }
    }
    return max_run;
}

// ===========================================================================
// NRZI helpers (now with per-bit debug)
// ===========================================================================

static uint8_t* nrzi_encode(const unsigned char *nrz_bytes, int byte_len, int *out_bit_count) {
    DEBUG_PRINT("=== NRZI ENCODE START (%d bytes) ===", byte_len);
    debug_bitstream("NRZ input", nrz_bytes, byte_len);

    int total_bits = byte_len * 8;
    uint8_t *levels = (uint8_t*) malloc(total_bits);
    if (!levels) {
        DEBUG_PRINT("!!! MALLOC FAILED !!!");
        return NULL;
    }

    uint8_t current_level = 1;
    int idx = 0;
    for (int i = 0; i < byte_len; i++) {
        for (int bit = 0; bit < 8; bit++) {
            uint8_t nrz_bit = (nrz_bytes[i] >> bit) & 0x01;
            if (nrz_bit == 0) {
                current_level ^= 1;
                DEBUG_PRINT("  byte %d bit %d: 0 → TRANSITION → level=%d", i, bit, current_level);
            } else {
                DEBUG_PRINT("  byte %d bit %d: 1 → no change → level=%d", i, bit, current_level);
            }
            levels[idx++] = current_level;
        }
    }
    *out_bit_count = total_bits;
    DEBUG_PRINT("NRZI levels (first 64 + last 64 shown):");
    for (int i = 0; i < total_bits; i++) {
        if (i < 64 || i > total_bits - 65)
            printf("%d", levels[i]);
        if ((i + 1) % 64 == 0)
            printf("\n");
    }
    printf("\n");
    DEBUG_VAR("Total transitions", count_nrzi_transitions(levels, total_bits));DEBUG_PRINT("=== NRZI ENCODE END ===");
    return levels;
}

static unsigned char* nrzi_decode(const uint8_t *levels, int bit_count, int *out_byte_len) {
    DEBUG_PRINT("=== NRZI DECODE START (%d bits) ===", bit_count);
    if (bit_count <= 0) {
        *out_byte_len = 0;
        return NULL;
    }

    size_t byte_len = ((size_t) bit_count + 7u) / 8u;
    unsigned char *nrz = (unsigned char*) calloc(byte_len, 1);
    if (!nrz) {
        DEBUG_PRINT("!!! MALLOC FAILED !!!");
        return NULL;
    }

    uint8_t prev_level = 1;
    for (int i = 0; i < bit_count; i++) {
        uint8_t nrz_bit = (levels[i] != prev_level) ? 0 : 1;
        prev_level = levels[i];
        nrz[i / 8u] |= (unsigned char) (nrz_bit << (i % 8));
    }
    *out_byte_len = (int) byte_len;
    debug_bitstream("NRZ recovered", nrz, *out_byte_len);
    DEBUG_PRINT("=== NRZI DECODE END ===");
    return nrz;
}

// ===========================================================================
// ALL TEST FUNCTIONS (fully implemented + heavy debug)
// ===========================================================================

static int test_abort_output_format(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_abort_output_format ===");
    printf("\n--- test_abort_output_format ---\n");
    unsigned char abort_buf[16];
    memset(abort_buf, 0, sizeof(abort_buf));
    int abort_len = 0;

    hdlc_frame_abort(abort_buf, &abort_len);

    DEBUG_VAR("abort_len", abort_len);
    debug_bitstream("abort bytes", abort_buf, abort_len);

    TEST_ASSERT(abort_len == 2, "Abort length == 2", 0);
    TEST_ASSERT(abort_buf[0] == 0xFF && abort_buf[1] == 0xFF, "Abort bytes == 0xFF 0xFF", 0);

    int max_ones = max_consecutive_ones_msb(abort_buf, abort_len);
    DEBUG_VAR("Max consecutive 1s (MSB)", max_ones);
    TEST_ASSERT(max_ones >= HDLC_ABORT_MIN_ONES, ">=15 ones", max_ones);
    return 0;
}

static int test_abort_null_safety(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_abort_null_safety ===");
    printf("\n--- test_abort_null_safety ---\n");
    unsigned char buf[4];
    int len = 99;

    hdlc_frame_abort(buf, NULL);           // should not crash
    hdlc_frame_abort(NULL, &len);
    DEBUG_VAR("len after NULL seq", len);
    TEST_ASSERT(len == 0, "abortLen=0 when seq=NULL", len);
    return 0;
}

static int test_abort_detection_in_decoder(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_abort_detection_in_decoder ===");
    printf("\n--- test_abort_detection_in_decoder ---\n");
    unsigned char stream[] = { 0x7E, 0xFF, 0xFF };
    unsigned char dec[256];
    int dlen = 0;
    hdlc_error_t err = hdlc_frame_decode(stream, 3, dec, &dlen);

    DEBUG_VAR("decode result", (unsigned)err);
    TEST_ASSERT(err == HDLC_ERR_ABORT, "Detects abort", (unsigned )err);
    return 0;
}

static int test_abort_at_seven_ones(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_abort_at_seven_ones ===");
    printf("\n--- test_abort_at_seven_ones ---\n");
    unsigned char stream[] = { 0x7E, 0xFE };
    unsigned char dec[256];
    int dlen = 0;
    hdlc_error_t err = hdlc_frame_decode(stream, 2, dec, &dlen);

    DEBUG_VAR("decode result", (unsigned)err);
    TEST_ASSERT(err == HDLC_ERR_ABORT, "7 ones = abort", (unsigned )err);
    return 0;
}

static int test_abort_injected_mid_frame(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_abort_injected_mid_frame ===");
    printf("\n--- test_abort_injected_mid_frame ---\n");
    unsigned char payload[15];
    memset(payload, 0xAA, sizeof(payload));
    int enc_len = 0;
    unsigned char *enc = malloc(512);
    hdlc_frame_encode(payload, 15, enc, &enc_len);

    if (enc_len >= 4) {
        enc[2] = 0xFF;
        enc[3] = 0xFF;
    }
    debug_bitstream("tampered frame", enc, enc_len);

    unsigned char dec[512];
    int dlen = 0;
    hdlc_error_t err = hdlc_frame_decode(enc, enc_len, dec, &dlen);

    DEBUG_VAR("decode result", (unsigned)err);
    TEST_ASSERT(err == HDLC_ERR_ABORT, "Mid-frame abort detected", (unsigned )err);
    free(enc);
    return 0;
}

static int test_abort_bit_count_lsb(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_abort_bit_count_lsb ===");
    printf("\n--- test_abort_bit_count_lsb ---\n");
    unsigned char abort_buf[4];
    int len = 0;
    hdlc_frame_abort(abort_buf, &len);

    int ones = max_consecutive_ones_lsb(abort_buf, len);
    DEBUG_VAR("ones (LSB-first)", ones);
    TEST_ASSERT(ones == 16, "Exactly 16 ones", ones);
    return 0;
}

// ---------------------------------------------------------------------------
// GROUP 2: NRZI
// ---------------------------------------------------------------------------

static int test_nrzi_roundtrip_flag(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_nrzi_roundtrip_flag ===");
    printf("\n--- test_nrzi_roundtrip_flag ---\n");
    unsigned char input[] = { 0x7E };
    int bc = 0;
    uint8_t *levels = nrzi_encode(input, 1, &bc);
    TEST_ASSERT(levels != NULL, "encode ok", 0);

    int olen = 0;
    unsigned char *out = nrzi_decode(levels, bc, &olen);
    TEST_ASSERT(out != NULL && olen == 1 && out[0] == 0x7E, "roundtrip 0x7E", 0);

    free(levels);
    free(out);
    return 0;
}

static int test_nrzi_roundtrip_abort(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_nrzi_roundtrip_abort ===");
    printf("\n--- test_nrzi_roundtrip_abort ---\n");
    unsigned char ab[4];
    int al = 0;
    hdlc_frame_abort(ab, &al);
    int bc = 0;
    uint8_t *levels = nrzi_encode(ab, al, &bc);
    int olen = 0;
    unsigned char *out = nrzi_decode(levels, bc, &olen);
    TEST_ASSERT(olen == al && memcmp(out, ab, al) == 0, "abort roundtrip", 0);
    free(levels);
    free(out);
    return 0;
}

static int test_nrzi_roundtrip_full_frame(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_nrzi_roundtrip_full_frame ===");
    printf("\n--- test_nrzi_roundtrip_full_frame ---\n");
    unsigned char payload[15];
    for (int i = 0; i < 15; i++)
        payload[i] = (unsigned char) (i + 0x41);
    int el = 0;
    unsigned char *enc = malloc(512);
    hdlc_frame_encode(payload, 15, enc, &el);

    int bc = 0;
    uint8_t *levels = nrzi_encode(enc, el, &bc);
    int rl = 0;
    unsigned char *rec = nrzi_decode(levels, bc, &rl);

    TEST_ASSERT(rl == el && memcmp(rec, enc, el) == 0, "full frame NRZI roundtrip", 0);
    free(enc);
    free(levels);
    free(rec);
    return 0;
}

static int test_nrzi_differential_transitions(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_nrzi_differential_transitions ===");
    printf("\n--- test_nrzi_differential_transitions ---\n");
    unsigned char zeros[] = { 0x00 };
    int bc = 0;
    uint8_t *levels = nrzi_encode(zeros, 1, &bc);
    int trans = count_nrzi_transitions(levels, bc);
    DEBUG_VAR("transitions (8 zeros)", trans);
    TEST_ASSERT(trans == 8, "8 zeros = 8 transitions", trans);
    free(levels);
    return 0;
}

static int test_nrzi_flag_transitions(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_nrzi_flag_transitions ===");
    printf("\n--- test_nrzi_flag_transitions ---\n");
    unsigned char flag[] = { 0x7E };
    int bc = 0;
    uint8_t *levels = nrzi_encode(flag, 1, &bc);
    int trans = count_nrzi_transitions(levels, bc);
    DEBUG_VAR("transitions for 0x7E", trans);
    TEST_ASSERT(trans == 2, "0x7E produces exactly 2 transitions", trans);
    free(levels);
    return 0;
}

static int test_bitstuffing_flag_pattern_debug(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_bitstuffing_flag_pattern_debug (EXTREME) ===");
    printf("\n--- TEST 3.1 DIAGNOSIS: Bit-stuffing with flag pattern (0x7E) in data ---\n");
    unsigned char payload[] = { 0x7E, 0x7E, 0x7E, 0x7E };
    int plen = 4;
    debug_bitstream("ORIGINAL PAYLOAD (contains 0x7E)", payload, plen);

    int el = 0;
    unsigned char enc[256];
    hdlc_frame_encode(payload, plen, enc, &el);
    debug_bitstream("ENCODED (after stuffing)", enc, el);

    unsigned char dec[256];
    int dl = 0;
    hdlc_error_t err = hdlc_frame_decode(enc, el, dec, &dl);
    DEBUG_VAR("decode result", (int)err);
    debug_bitstream("DECODED", dec, dl);

    TEST_ASSERT(err == HDLC_OK && dl == plen && memcmp(dec, payload, plen) == 0, "0x7E payload survives encode/decode", (int )err);
    return 0;
}

// ---------------------------------------------------------------------------
// GROUP 3: Idle patterns
// ---------------------------------------------------------------------------

static int generate_idle_flags(unsigned char *buf, int buf_size, int count) {
    if (!buf || count <= 0 || count > buf_size)
        return 0;
    for (int i = 0; i < count; i++)
        buf[i] = HDLC_FLAG_BYTE;
    return count;
}

static int generate_idle_mark(unsigned char *buf, int buf_size, int count) {
    if (!buf || count <= 0 || count > buf_size)
        return 0;
    memset(buf, 0xFF, count);
    return count;
}

static int test_idle_flags_pattern(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_idle_flags_pattern ===");
    printf("\n--- test_idle_flags_pattern ---\n");
    unsigned char idle[16];
    int g = generate_idle_flags(idle, 16, 8);
    debug_bitstream("idle flags (8x0x7E)", idle, g);
    TEST_ASSERT(g == 8 && idle[0] == 0x7E, "all flags", g);
    return 0;
}

static int test_idle_flags_not_data(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_idle_flags_not_data ===");
    printf("\n--- test_idle_flags_not_data ---\n");
    unsigned char idle[8];
    generate_idle_flags(idle, 8, 4);
    unsigned char dec[256];
    int dl = 0;
    hdlc_error_t err = hdlc_frame_decode(idle, 4, dec, &dl);
    DEBUG_VAR("decode err", (unsigned)err);
    TEST_ASSERT(err != HDLC_OK, "idle flags not valid frame", (unsigned )err);
    return 0;
}

static int test_idle_mark_pattern(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_idle_mark_pattern ===");
    printf("\n--- test_idle_mark_pattern ---\n");
    unsigned char mark[8];
    int g = generate_idle_mark(mark, 8, 6);
    debug_bitstream("mark-hold (6x0xFF)", mark, g);
    TEST_ASSERT(g == 6 && mark[0] == 0xFF, "all 0xFF", g);
    return 0;
}

static int test_idle_mark_triggers_abort(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_idle_mark_triggers_abort ===");
    printf("\n--- test_idle_mark_triggers_abort ---\n");
    unsigned char s[3] = { 0x7E, 0xFF, 0xFF };
    unsigned char dec[256];
    int dl = 0;
    hdlc_error_t err = hdlc_frame_decode(s, 3, dec, &dl);
    DEBUG_VAR("decode", (unsigned)err);
    TEST_ASSERT(err == HDLC_ERR_ABORT, "mark-hold = abort", (unsigned )err);
    return 0;
}

static int test_idle_flags_nrzi_pattern(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_idle_flags_nrzi_pattern ===");
    printf("\n--- test_idle_flags_nrzi_pattern ---\n");
    unsigned char flags[4];
    generate_idle_flags(flags, 4, 4);
    int bc = 0;
    uint8_t *levels = nrzi_encode(flags, 4, &bc);
    TEST_ASSERT(levels != NULL, "nrzi ok", 0);
    free(levels);
    return 0;
}

static int test_preamble_postamble(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_preamble_postamble ===");
    printf("\n--- test_preamble_postamble ---\n");
    unsigned char payload[15];
    memset(payload, 0x61, 15);
    int el = 0;
    unsigned char *enc = malloc(512);
    hdlc_frame_encode(payload, 15, enc, &el);

    unsigned char *stream = malloc(el + 16);
    stream[0] = stream[1] = stream[2] = 0x7E;
    memcpy(stream + 3, enc, el);
    int total = el + 3;

    unsigned char dec[512];
    int dl = 0;
    hdlc_error_t err = hdlc_frame_decode(stream, total, dec, &dl);
    DEBUG_VAR("preamble decode", (unsigned)err);
    TEST_ASSERT(err != HDLC_OK, "preamble handled (not empty frame)", (unsigned )err);

    free(stream);
    free(enc);
    return 0;
}

// ---------------------------------------------------------------------------
// GROUP 4
// ---------------------------------------------------------------------------

static int test_abort_nrzi_is_dc_mark(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_abort_nrzi_is_dc_mark ===");
    printf("\n--- test_abort_nrzi_is_dc_mark ---\n");
    unsigned char ab[4];
    int al = 0;
    hdlc_frame_abort(ab, &al);
    int bc = 0;
    uint8_t *levels = nrzi_encode(ab, al, &bc);
    int trans = count_nrzi_transitions(levels, bc);
    DEBUG_VAR("transitions (should be 0)", trans);
    TEST_ASSERT(trans == 0, "abort = DC mark", trans);
    free(levels);
    return 0;
}

static int test_crc_embed_and_verify(void) {
    DEBUG_PRINT("=== ENTERING TEST: test_crc_embed_and_verify ===");
    printf("\n--- test_crc_embed_and_verify ---\n");
    unsigned char data[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    uint16_t crc = hal_crc16_buf(data, 9);
    DEBUG_VAR("CRC(123456789)", crc);
    TEST_ASSERT(crc == 0x906E, "CRC check value", crc);

    unsigned char frame[11];
    memcpy(frame, data, 9);
    frame[9] = (unsigned char) (crc & 0xFF);
    frame[10] = (unsigned char) (crc >> 8);
    // CRC_verify replaced: hal_crc16_buf over data+FCS residual must equal 0x0F47
    bool valid = (hal_crc16_buf(frame, 11) == 0x0F47u);
    DEBUG_BOOL("CRC_verify", valid);
    TEST_ASSERT(valid, "CRC_verify true (0x0F47 residue)", 0);
    return 0;
}

// ===========================================================================
// MAIN
// ===========================================================================
int test_hdlc_frame_structure_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("HDLC Frame Structure Tests - Section 1 (MASSIVE DEBUG ENABLED)\n");
    printf("AX.25 v2.2 Sections 3.7, 3.8, 3.10\n");
    printf("==================================================================================\n");

    printf("\n== GROUP 1: HDLC Abort Sequence ==\n");
    result |= test_abort_output_format();
    result |= test_abort_null_safety();
    result |= test_abort_detection_in_decoder();
    result |= test_abort_at_seven_ones();
    result |= test_abort_injected_mid_frame();
    result |= test_abort_bit_count_lsb();

    printf("\n== GROUP 2: NRZI Encoding / Decoding ==\n");
    result |= test_nrzi_roundtrip_flag();
    result |= test_nrzi_roundtrip_abort();
    result |= test_nrzi_roundtrip_full_frame();
    result |= test_nrzi_differential_transitions();
    result |= test_nrzi_flag_transitions();
    result |= test_bitstuffing_flag_pattern_debug();

    printf("\n== GROUP 3: Idle Pattern Generation ==\n");
    result |= test_idle_flags_pattern();
    result |= test_idle_flags_not_data();
    result |= test_idle_mark_pattern();
    result |= test_idle_mark_triggers_abort();
    result |= test_idle_flags_nrzi_pattern();
    result |= test_preamble_postamble();

    printf("\n== GROUP 4: Abort + NRZI Interactions ==\n");
    result |= test_abort_nrzi_is_dc_mark();
    result |= test_crc_embed_and_verify();

    printf("\n==================================================================================\n");
    printf("HDLC Frame Structure Tests Completed. %s\n", result == 0 ? "All tests PASSED" : "Some tests FAILED");
    printf("Total asserts: %u\n", assert_count);
    printf("==================================================================================\n\n");

    return result;
}
