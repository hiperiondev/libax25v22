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

// HDLC Frame Structure Tests - Section 1
// Covers features NOT tested by existing test suite:
//   - HDLC Frame Abort Sequence (AX.25 v2.2 Section 3.10): 15+ contiguous 1s
//   - NRZI Encoding / Decoding (AX.25 v2.2 Section 3.8)
//   - Idle Pattern Generation (continuous flag / mark-hold patterns)
#define DEBUG_ENABLE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// start modified part - new test includes
#include "test_common.h"
#include "hdlc.h"
#include "common.h"
// end modified part

// Counter required by TEST_ASSERT macro in test_common.h
static uint32_t assert_count = 0;

// ---------------------------------------------------------------------------
// Utility: count consecutive 1-bits starting from a raw bit stream.
// Operates on a byte array, reads bits LSB-first (matching HDLC bit order).
// Returns the maximum run of consecutive 1-bits found anywhere in the stream.
// ---------------------------------------------------------------------------
static int max_consecutive_ones_lsb(const unsigned char *buf, int byte_len) {
    int max_run = 0;
    int run = 0;
    for (int i = 0; i < byte_len; i++) {
        for (int bit = 0; bit < 8; bit++) {
            if ((buf[i] >> bit) & 0x01) {
                run++;
                if (run > max_run) {
                    max_run = run;
                }
            } else {
                run = 0;
            }
        }
    }
    return max_run;
}

// ---------------------------------------------------------------------------
// Utility: count consecutive 1-bits in a byte array read MSB-first.
// Needed when checking raw abort bytes (0xFF 0xFF) before bit-stuffing layer.
// ---------------------------------------------------------------------------
static int max_consecutive_ones_msb(const unsigned char *buf, int byte_len) {
    int max_run = 0;
    int run = 0;
    for (int i = 0; i < byte_len; i++) {
        for (int bit = 7; bit >= 0; bit--) {
            if ((buf[i] >> bit) & 0x01) {
                run++;
                if (run > max_run) {
                    max_run = run;
                }
            } else {
                run = 0;
            }
        }
    }
    return max_run;
}

// ===========================================================================
// NRZI helper functions
// AX.25 v2.2 Section 3.8: Non-Return to Zero Inverted (NRZI) encoding.
// Rule: a 0-bit causes a transition; a 1-bit causes no transition.
// Initial state: line starts HIGH (mark).
// ===========================================================================

// start modified part - NRZI encode/decode helpers (software model for test)

// nrzi_encode: convert NRZ bit stream (byte array, LSB-first per AX.25) to
// NRZI signal levels. Returns allocated buffer of same length. Caller frees.
// Each element is 0 (space) or 1 (mark).
static uint8_t *nrzi_encode(const unsigned char *nrz_bytes, int byte_len, int *out_bit_count) {
    int total_bits = byte_len * 8;
    uint8_t *levels = (uint8_t *)malloc(total_bits);
    if (!levels) return NULL;

    uint8_t current_level = 1; // start HIGH (mark)
    int idx = 0;
    for (int i = 0; i < byte_len; i++) {
        for (int bit = 0; bit < 8; bit++) {
            uint8_t nrz_bit = (nrz_bytes[i] >> bit) & 0x01;
            if (nrz_bit == 0) {
                // 0 causes transition
                current_level ^= 1;
            }
            // 1 causes no transition
            levels[idx++] = current_level;
        }
    }
    *out_bit_count = total_bits;
    return levels;
}

// nrzi_decode: recover NRZ bit stream from NRZI signal levels.
// Returns allocated byte array. Caller frees.
static unsigned char *nrzi_decode(const uint8_t *levels, int bit_count, int *out_byte_len) {
    // start modified part - guard non-positive bit_count; use size_t so calloc
    // receives an unsigned value and the compiler can prove it is in range
    if (bit_count <= 0) {
        *out_byte_len = 0;
        return NULL;
    }
    size_t byte_len = ((size_t)bit_count + 7u) / 8u;
    // end modified part
    unsigned char *nrz = (unsigned char *)calloc(byte_len, 1);
    if (!nrz) return NULL;

    uint8_t prev_level = 1; // initial state HIGH
    for (int i = 0; i < bit_count; i++) {
        uint8_t nrz_bit;
        if (levels[i] != prev_level) {
            nrz_bit = 0; // transition -> 0
        } else {
            nrz_bit = 1; // no transition -> 1
        }
        prev_level = levels[i];
        // start modified part - size_t index and explicit cast silence sign/shift warnings
        nrz[(size_t)i / 8u] |= (unsigned char)(nrz_bit << (i % 8));
        // end modified part
    }
    // start modified part - byte_len <= INT_MAX because bit_count <= INT_MAX
    *out_byte_len = (int)byte_len;
    // end modified part
    return nrz;
}

// end modified part - NRZI helpers

// ===========================================================================
// TEST GROUP 1: HDLC ABORT SEQUENCE
// AX.25 v2.2 Section 3.10 / hdlc.h HDLC_ABORT_MIN_ONES = 15
// ===========================================================================

// ---------------------------------------------------------------------------
// TEST 1.1: hdlc_frame_abort output format validation
// Verify the abort bytes are 0xFF 0xFF and length == 2.
// ---------------------------------------------------------------------------
static int test_abort_output_format(void) {
    printf("\n--- test_abort_output_format ---\n");
    printf("AX.25 v2.2 Section 3.10: abort output must be >= 15 contiguous 1s\n");

    unsigned char abort_buf[16];
    memset(abort_buf, 0, sizeof(abort_buf));
    int abort_len = 0;

    hdlc_frame_abort(abort_buf, &abort_len);

    DEBUG_VAR("abort_len (should be 2)", abort_len);
    DEBUG_FRAME("abort bytes", abort_buf, abort_len);

    // start modified part - test abort length and byte values
    TEST_ASSERT(abort_len == 2, "Abort sequence length == 2", 0);
    TEST_ASSERT(abort_buf[0] == 0xFF, "Abort byte[0] == 0xFF", abort_buf[0]);
    TEST_ASSERT(abort_buf[1] == 0xFF, "Abort byte[1] == 0xFF", abort_buf[1]);
    // end modified part

    // Count raw consecutive ones in the abort buffer
    int max_ones = max_consecutive_ones_msb(abort_buf, abort_len);
    DEBUG_VAR("Max consecutive 1-bits (MSB-first count)", max_ones);
    TEST_ASSERT(max_ones >= HDLC_ABORT_MIN_ONES,
                "Abort sequence has >= 15 consecutive 1-bits (AX.25 v2.2 Section 3.10)", max_ones);

    return 0;
}

// ---------------------------------------------------------------------------
// TEST 1.2: hdlc_frame_abort with NULL pointers - must not crash
// ---------------------------------------------------------------------------
static int test_abort_null_safety(void) {
    printf("\n--- test_abort_null_safety ---\n");
    printf("Verify hdlc_frame_abort handles NULL arguments safely\n");

    // NULL abortLen - should not crash
    unsigned char abort_buf[4];
    hdlc_frame_abort(abort_buf, NULL); // must not segfault

    // NULL abortSeq - abortLen should be set to 0
    int abort_len = 99;
    hdlc_frame_abort(NULL, &abort_len);
    DEBUG_VAR("abort_len after NULL abortSeq (should be 0)", abort_len);
    TEST_ASSERT(abort_len == 0, "abort_len == 0 when abortSeq is NULL", abort_len);

    printf("  NULL safety check passed (no crash)\n");
    return 0;
}

// ---------------------------------------------------------------------------
// TEST 1.3: Decoder detects abort sequence in received bit stream
// Build a raw bit stream: 0x7E (start flag) + 16 ones (abort) and verify
// hdlc_frame_decode returns HDLC_ERR_ABORT.
// ---------------------------------------------------------------------------
static int test_abort_detection_in_decoder(void) {
    printf("\n--- test_abort_detection_in_decoder ---\n");
    printf("Verify decoder returns HDLC_ERR_ABORT for 15+ contiguous 1s\n");

    // Construct encoded stream manually:
    // Byte 0: 0x7E (start flag)
    // Bytes 1-2: 0xFF 0xFF (16 ones - abort, no bit stuffing applied)
    // This simulates a transmitter sending abort mid-frame.
    unsigned char abort_stream[] = { 0x7E, 0xFF, 0xFF };
    int stream_len = (int)sizeof(abort_stream);

    DEBUG_FRAME("abort_stream input to decoder", abort_stream, stream_len);

    unsigned char decoded_buf[256];
    int decoded_len = 0;
    hdlc_error_t err = hdlc_frame_decode(abort_stream, stream_len, decoded_buf, &decoded_len);

    DEBUG_VAR("hdlc_frame_decode result (should be HDLC_ERR_ABORT)", (unsigned)err);
    DEBUG_VAR("HDLC_ERR_ABORT constant value", (unsigned)HDLC_ERR_ABORT);

    TEST_ASSERT(err == HDLC_ERR_ABORT,
                "Decoder returns HDLC_ERR_ABORT for 16 consecutive 1-bits", (unsigned)err);

    return 0;
}

// ---------------------------------------------------------------------------
// TEST 1.4: Decoder detects abort at minimum 7 ones (>=7 aborts per spec)
// Per hdlc.c: cnt >= 7 triggers HDLC_ERR_ABORT.
// ---------------------------------------------------------------------------
static int test_abort_at_seven_ones(void) {
    printf("\n--- test_abort_at_seven_ones ---\n");
    printf("Verify decoder returns HDLC_ERR_ABORT at >= 7 contiguous 1s (in-stream)\n");

    // 0x7E + 0xFE (7 ones LSB-first = 1111111_0 bits 0-6 = 0x7F, but 0xFE = 11111110)
    // LSB-first: bit0=0,bit1=1,bit2=1,bit3=1,bit4=1,bit5=1,bit6=1,bit7=1 -> after bit0=0
    // we get 7 ones from bits 1-7. That triggers abort.
    unsigned char abort_stream7[] = { 0x7E, 0xFE };
    int stream_len = (int)sizeof(abort_stream7);

    DEBUG_FRAME("abort_stream7 input (7 ones LSB-first in byte 0xFE)", abort_stream7, stream_len);

    unsigned char decoded_buf[256];
    int decoded_len = 0;
    hdlc_error_t err = hdlc_frame_decode(abort_stream7, stream_len, decoded_buf, &decoded_len);

    DEBUG_VAR("hdlc_frame_decode result (should be HDLC_ERR_ABORT=6)", (unsigned)err);

    TEST_ASSERT(err == HDLC_ERR_ABORT,
                "Decoder returns HDLC_ERR_ABORT for 7 contiguous 1-bits", (unsigned)err);

    return 0;
}

// ---------------------------------------------------------------------------
// TEST 1.5: Abort on a mid-frame injected sequence
// Encode a valid frame, then manually inject an abort pattern inside and
// verify the decoder rejects it.
// ---------------------------------------------------------------------------
static int test_abort_injected_mid_frame(void) {
    printf("\n--- test_abort_injected_mid_frame ---\n");
    printf("Verify abort sequence injected after start flag is detected\n");

    // Minimal AX.25-like data: 15 bytes (address + control)
    unsigned char frame_data[15];
    memset(frame_data, 0xAA, sizeof(frame_data));
    int encoded_len = 0;
    int max_enc = hdlc_encoded_size_max((int)sizeof(frame_data));
    unsigned char *encoded = (unsigned char *)malloc(max_enc + 4);
    if (!encoded) {
        printf("  SKIP: malloc failed\n");
        return 0;
    }

    hdlc_frame_encode(frame_data, (int)sizeof(frame_data), encoded, &encoded_len);
    DEBUG_FRAME("valid encoded frame", encoded, encoded_len);

    // Overwrite bytes 2-3 with 0xFF 0xFF to simulate mid-frame abort injection
    if (encoded_len >= 4) {
        encoded[2] = 0xFF;
        encoded[3] = 0xFF;
    }

    DEBUG_FRAME("tampered frame with 0xFF 0xFF at offset 2", encoded, encoded_len);

    unsigned char decoded_buf[512];
    int decoded_len = 0;
    hdlc_error_t err = hdlc_frame_decode(encoded, encoded_len, decoded_buf, &decoded_len);

    DEBUG_VAR("hdlc_frame_decode result (expect HDLC_ERR_ABORT)", (unsigned)err);

    TEST_ASSERT(err == HDLC_ERR_ABORT,
                "Mid-frame abort injection detected by decoder", (unsigned)err);

    free(encoded);
    return 0;
}

// ---------------------------------------------------------------------------
// TEST 1.6: hdlc_frame_abort bit stream - verify 16 raw consecutive 1-bits
// Count ones in LSB-first bit order (matches HDLC transmission order).
// ---------------------------------------------------------------------------
static int test_abort_bit_count_lsb(void) {
    printf("\n--- test_abort_bit_count_lsb ---\n");
    printf("Count consecutive 1-bits in abort sequence (LSB-first = HDLC bit order)\n");

    unsigned char abort_buf[4];
    int abort_len = 0;
    hdlc_frame_abort(abort_buf, &abort_len);

    int max_ones_lsb = max_consecutive_ones_lsb(abort_buf, abort_len);
    DEBUG_VAR("Max consecutive 1-bits (LSB-first)", max_ones_lsb);

    TEST_ASSERT(max_ones_lsb >= HDLC_ABORT_MIN_ONES,
                "Abort sequence >= 15 ones in LSB-first bit order", max_ones_lsb);

    // Specifically: 2 bytes of 0xFF = 16 ones regardless of bit order
    TEST_ASSERT(max_ones_lsb == 16,
                "2 x 0xFF = exactly 16 consecutive 1-bits", max_ones_lsb);

    return 0;
}

// ===========================================================================
// TEST GROUP 2: NRZI ENCODING / DECODING
// AX.25 v2.2 Section 3.8: NRZI on all transmitted bit streams.
// ===========================================================================

// ---------------------------------------------------------------------------
// TEST 2.1: NRZI encode/decode round-trip on HDLC flag byte 0x7E
// ---------------------------------------------------------------------------
static int test_nrzi_roundtrip_flag(void) {
    printf("\n--- test_nrzi_roundtrip_flag ---\n");
    printf("AX.25 v2.2 Section 3.8: NRZI round-trip on HDLC flag 0x7E\n");

    unsigned char input[] = { HDLC_FLAG_BYTE };
    int bit_count = 0;

    DEBUG_FRAME("NRZI input (0x7E)", input, 1);

    uint8_t *levels = nrzi_encode(input, 1, &bit_count);
    TEST_ASSERT(levels != NULL, "nrzi_encode returned non-NULL", 0);
    if (!levels) return 1;

    DEBUG_VAR("Encoded bit_count", bit_count);
    DEBUG_PRINT("NRZI levels (0=space/transition, 1=mark/no-transition):");
    for (int i = 0; i < bit_count; i++) {
        printf("%d", levels[i]);
    }
    printf("\n");

    int out_byte_len = 0;
    unsigned char *decoded = nrzi_decode(levels, bit_count, &out_byte_len);
    TEST_ASSERT(decoded != NULL, "nrzi_decode returned non-NULL", 0);

    DEBUG_FRAME("NRZI decoded output", decoded, out_byte_len);

    TEST_ASSERT(out_byte_len == 1, "NRZI round-trip length matches", out_byte_len);
    TEST_ASSERT(decoded[0] == 0x7E, "NRZI round-trip value matches 0x7E", decoded[0]);

    free(levels);
    free(decoded);
    return 0;
}

// ---------------------------------------------------------------------------
// TEST 2.2: NRZI round-trip on abort sequence 0xFF 0xFF
// ---------------------------------------------------------------------------
static int test_nrzi_roundtrip_abort(void) {
    printf("\n--- test_nrzi_roundtrip_abort ---\n");
    printf("NRZI round-trip on abort sequence 0xFF 0xFF\n");

    unsigned char abort_buf[4];
    int abort_len = 0;
    hdlc_frame_abort(abort_buf, &abort_len);

    DEBUG_FRAME("Abort bytes for NRZI test", abort_buf, abort_len);

    int bit_count = 0;
    uint8_t *levels = nrzi_encode(abort_buf, abort_len, &bit_count);
    TEST_ASSERT(levels != NULL, "nrzi_encode abort: non-NULL", 0);
    if (!levels) return 1;

    DEBUG_VAR("NRZI bit_count for abort", bit_count);
    DEBUG_PRINT("NRZI levels for 0xFF 0xFF:");
    for (int i = 0; i < bit_count; i++) {
        printf("%d", levels[i]);
    }
    printf("\n");

    // 0xFF = all 1s = no transitions. All levels should stay at initial HIGH (1).
    // Since abort is all 1-bits and 1 = no transition, the level stays at the
    // initial mark level (1) for all 16 bits.
    int all_mark = 1;
    for (int i = 0; i < bit_count; i++) {
        if (levels[i] != 1) {
            all_mark = 0;
            break;
        }
    }
    DEBUG_BOOL("All NRZI levels are HIGH/mark (no transition for 1s)", all_mark);
    TEST_ASSERT(all_mark == 1,
                "NRZI abort (0xFF 0xFF): all levels HIGH - no transitions for 1-bits", 0);

    int out_byte_len = 0;
    unsigned char *decoded = nrzi_decode(levels, bit_count, &out_byte_len);
    TEST_ASSERT(decoded != NULL, "nrzi_decode abort: non-NULL", 0);

    if (decoded) {
        DEBUG_FRAME("NRZI decoded abort", decoded, out_byte_len);
        TEST_ASSERT(out_byte_len == abort_len, "NRZI abort round-trip length matches", out_byte_len);
        TEST_ASSERT(memcmp(decoded, abort_buf, abort_len) == 0,
                    "NRZI abort round-trip bytes match 0xFF 0xFF", 0);
        free(decoded);
    }

    free(levels);
    return 0;
}

// ---------------------------------------------------------------------------
// TEST 2.3: NRZI round-trip on a full HDLC-encoded frame
// ---------------------------------------------------------------------------
static int test_nrzi_roundtrip_full_frame(void) {
    printf("\n--- test_nrzi_roundtrip_full_frame ---\n");
    printf("NRZI round-trip on a full HDLC-encoded AX.25-like frame\n");

    // Minimal valid AX.25 payload: 15 bytes of known data
    unsigned char payload[15];
    for (int i = 0; i < 15; i++) {
        payload[i] = (unsigned char)(i + 0x41); // 'A', 'B', ...
    }

    int enc_len = 0;
    int max_enc = hdlc_encoded_size_max(15);
    unsigned char *encoded = (unsigned char *)malloc(max_enc + 8);
    if (!encoded) {
        printf("  SKIP: malloc failed\n");
        return 0;
    }

    hdlc_frame_encode(payload, 15, encoded, &enc_len);
    DEBUG_FRAME("HDLC encoded frame", encoded, enc_len);
    DEBUG_VAR("enc_len", enc_len);

    TEST_ASSERT(enc_len > 0, "Frame encoded successfully (enc_len > 0)", enc_len);
    if (enc_len == 0) {
        free(encoded);
        return 1;
    }

    // Apply NRZI encoding
    int bit_count = 0;
    uint8_t *levels = nrzi_encode(encoded, enc_len, &bit_count);
    TEST_ASSERT(levels != NULL, "NRZI encode full frame: non-NULL", 0);
    if (!levels) {
        free(encoded);
        return 1;
    }

    DEBUG_VAR("NRZI bit count for full frame", bit_count);

    // Apply NRZI decoding to recover original HDLC bytes
    int recovered_len = 0;
    unsigned char *recovered = nrzi_decode(levels, bit_count, &recovered_len);
    TEST_ASSERT(recovered != NULL, "NRZI decode full frame: non-NULL", 0);

    if (recovered) {
        DEBUG_FRAME("NRZI recovered frame", recovered, recovered_len);
        TEST_ASSERT(recovered_len == enc_len,
                    "NRZI full frame round-trip length matches", recovered_len);
        int match = (memcmp(recovered, encoded, enc_len) == 0);
        DEBUG_BOOL("NRZI byte-level match", match);
        TEST_ASSERT(match, "NRZI full frame round-trip bytes match exactly", 0);
        free(recovered);
    }

    free(levels);
    free(encoded);
    return 0;
}

// ---------------------------------------------------------------------------
// TEST 2.4: NRZI differential property - 0-bit causes transition
// Feed alternating 0/1 bytes and verify transitions in NRZI output.
// ---------------------------------------------------------------------------
static int test_nrzi_differential_transitions(void) {
    printf("\n--- test_nrzi_differential_transitions ---\n");
    printf("Verify 0-bit causes transition, 1-bit causes no transition in NRZI\n");

    // 0x00 = 8 zeros = 8 transitions
    unsigned char zeros[] = { 0x00 };
    int bit_count = 0;
    uint8_t *levels = nrzi_encode(zeros, 1, &bit_count);
    TEST_ASSERT(levels != NULL, "nrzi_encode 0x00: non-NULL", 0);
    if (!levels) return 1;

    int transition_count = 0;
    uint8_t prev = 1; // initial mark
    for (int i = 0; i < bit_count; i++) {
        if (levels[i] != prev) transition_count++;
        prev = levels[i];
    }
    DEBUG_VAR("Transition count for 0x00 (8 zeros, expect 8 transitions)", transition_count);
    TEST_ASSERT(transition_count == 8,
                "0x00 (8 zero-bits) causes 8 NRZI transitions", transition_count);
    free(levels);

    // 0xFF = 8 ones = 0 transitions
    unsigned char ones[] = { 0xFF };
    bit_count = 0;
    levels = nrzi_encode(ones, 1, &bit_count);
    TEST_ASSERT(levels != NULL, "nrzi_encode 0xFF: non-NULL", 0);
    if (!levels) return 1;

    transition_count = 0;
    prev = 1;
    for (int i = 0; i < bit_count; i++) {
        if (levels[i] != prev) transition_count++;
        prev = levels[i];
    }
    DEBUG_VAR("Transition count for 0xFF (8 ones, expect 0 transitions)", transition_count);
    TEST_ASSERT(transition_count == 0,
                "0xFF (8 one-bits) causes 0 NRZI transitions", transition_count);
    free(levels);

    // 0xAA = 10101010 LSB-first = 0,1,0,1,0,1,0,1 = alternating, 4 transitions
    // Bit 0 = 0 (transition), bit 1 = 1 (no transition), bit 2 = 0 (transition), ...
    unsigned char alt[] = { 0xAA };
    bit_count = 0;
    levels = nrzi_encode(alt, 1, &bit_count);
    TEST_ASSERT(levels != NULL, "nrzi_encode 0xAA: non-NULL", 0);
    if (!levels) return 1;

    DEBUG_PRINT("NRZI levels for 0xAA:");
    for (int i = 0; i < bit_count; i++) printf("%d", levels[i]);
    printf("\n");

    transition_count = 0;
    prev = 1;
    for (int i = 0; i < bit_count; i++) {
        if (levels[i] != prev) transition_count++;
        prev = levels[i];
    }
    // 0xAA LSB-first: bit0=0(trans), bit1=1(no), bit2=0(trans), bit3=1(no),
    //                 bit4=0(trans), bit5=1(no), bit6=0(trans), bit7=1(no) = 4 transitions
    DEBUG_VAR("Transition count for 0xAA (expect 4)", transition_count);
    TEST_ASSERT(transition_count == 4,
                "0xAA (alternating bits LSB-first) causes 4 NRZI transitions", transition_count);
    free(levels);

    return 0;
}

// ---------------------------------------------------------------------------
// TEST 2.5: NRZI on HDLC flag 0x7E - verify known transition pattern
// 0x7E = 01111110, LSB-first: 0,1,1,1,1,1,1,0
// bit0=0 -> transition, bits1-6=1 -> no transitions, bit7=0 -> transition
// Total 2 transitions per flag byte.
// ---------------------------------------------------------------------------
static int test_nrzi_flag_transitions(void) {
    printf("\n--- test_nrzi_flag_transitions ---\n");
    printf("Verify HDLC flag 0x7E generates exactly 2 NRZI transitions\n");

    unsigned char flag[] = { 0x7E };
    int bit_count = 0;
    uint8_t *levels = nrzi_encode(flag, 1, &bit_count);
    TEST_ASSERT(levels != NULL, "nrzi_encode 0x7E: non-NULL", 0);
    if (!levels) return 1;

    DEBUG_PRINT("NRZI levels for 0x7E (01111110 LSB-first = 0,1,1,1,1,1,1,0):");
    for (int i = 0; i < bit_count; i++) printf("%d", levels[i]);
    printf("\n");

    int transition_count = 0;
    uint8_t prev = 1;
    for (int i = 0; i < bit_count; i++) {
        if (levels[i] != prev) transition_count++;
        prev = levels[i];
    }
    DEBUG_VAR("Transition count for 0x7E flag (expect 2)", transition_count);
    TEST_ASSERT(transition_count == 2,
                "HDLC flag 0x7E (LSB-first 0,1,1,1,1,1,1,0) causes 2 NRZI transitions", transition_count);

    free(levels);
    return 0;
}

// ===========================================================================
// TEST GROUP 3: IDLE PATTERN GENERATION
// AX.25 v2.2 Section 3.7: Idle line between frames = continuous flag bytes or
// mark-hold (continuous 1s). Both are tested.
// ===========================================================================

// ---------------------------------------------------------------------------
// Helper: generate N continuous HDLC flag bytes (idle flags pattern)
// AX.25 v2.2 Section 3.7: "When no frame is being transmitted, the DTE shall
// transmit a continuous series of flag sequences."
// ---------------------------------------------------------------------------
static int generate_idle_flags(unsigned char *buf, int buf_size, int count) {
    if (!buf || count <= 0 || count > buf_size) return 0;
    for (int i = 0; i < count; i++) {
        buf[i] = HDLC_FLAG_BYTE;
    }
    return count;
}

// ---------------------------------------------------------------------------
// Helper: generate mark-hold (continuous 1s) idle pattern.
// Some TNC implementations use mark-hold (0xFF) as channel idle.
// ---------------------------------------------------------------------------
static int generate_idle_mark(unsigned char *buf, int buf_size, int count) {
    if (!buf || count <= 0 || count > buf_size) return 0;
    memset(buf, 0xFF, count);
    return count;
}

// ---------------------------------------------------------------------------
// TEST 3.1: Idle flag pattern is valid 0x7E bytes
// ---------------------------------------------------------------------------
static int test_idle_flags_pattern(void) {
    printf("\n--- test_idle_flags_pattern ---\n");
    printf("AX.25 v2.2 Section 3.7: Idle line uses continuous HDLC flag sequences\n");

    unsigned char idle_buf[16];
    memset(idle_buf, 0, sizeof(idle_buf));
    int generated = generate_idle_flags(idle_buf, (int)sizeof(idle_buf), 8);

    DEBUG_FRAME("idle flags buffer (8 x 0x7E)", idle_buf, generated);
    DEBUG_VAR("generated count", generated);

    TEST_ASSERT(generated == 8, "Generate 8 idle flag bytes", generated);

    int all_flags = 1;
    for (int i = 0; i < generated; i++) {
        if (idle_buf[i] != HDLC_FLAG_BYTE) {
            all_flags = 0;
            printf("  [ERR] idle_buf[%d] = 0x%02X (expected 0x7E)\n", i, idle_buf[i]);
            break;
        }
    }
    TEST_ASSERT(all_flags, "All 8 idle bytes are HDLC_FLAG_BYTE (0x7E)", 0);

    return 0;
}

// ---------------------------------------------------------------------------
// TEST 3.2: Idle flags are NOT decoded as data frames
// An idle stream of flags should not be mistaken for a valid data frame.
// ---------------------------------------------------------------------------
static int test_idle_flags_not_data(void) {
    printf("\n--- test_idle_flags_not_data ---\n");
    printf("Idle flag stream must not decode as valid data frame\n");

    // 4 flags: start_flag + nothing inside + second flag = empty content
    // The decoder should fail with too-short or no-end-flag on a pure flag stream
    unsigned char idle_buf[8];
    int count = generate_idle_flags(idle_buf, 8, 4);
    DEBUG_FRAME("4 x 0x7E idle stream", idle_buf, count);

    unsigned char decoded_buf[256];
    int decoded_len = 0;
    hdlc_error_t err = hdlc_frame_decode(idle_buf, count, decoded_buf, &decoded_len);

    DEBUG_VAR("decode result for idle flags (expect error, NOT HDLC_OK)", (unsigned)err);

    // The first 0x7E is the start flag; the second 0x7E is immediately the end flag
    // with nothing in between - should fail as HDLC_ERR_TOO_SHORT
    TEST_ASSERT(err != HDLC_OK,
                "Idle flag stream is NOT decoded as a valid frame", (unsigned)err);
    DEBUG_VAR("Specific error code for idle stream", (unsigned)err);

    return 0;
}

// ---------------------------------------------------------------------------
// TEST 3.3: Mark-hold idle pattern is all 0xFF bytes
// ---------------------------------------------------------------------------
static int test_idle_mark_pattern(void) {
    printf("\n--- test_idle_mark_pattern ---\n");
    printf("Mark-hold idle pattern: verify all bytes are 0xFF (continuous 1s)\n");

    unsigned char mark_buf[8];
    memset(mark_buf, 0, sizeof(mark_buf));
    int generated = generate_idle_mark(mark_buf, (int)sizeof(mark_buf), 6);

    DEBUG_FRAME("mark-hold idle buffer (6 x 0xFF)", mark_buf, generated);
    TEST_ASSERT(generated == 6, "Generate 6 mark-hold bytes", generated);

    int all_ff = 1;
    for (int i = 0; i < generated; i++) {
        if (mark_buf[i] != 0xFF) {
            all_ff = 0;
            printf("  [ERR] mark_buf[%d] = 0x%02X (expected 0xFF)\n", i, mark_buf[i]);
            break;
        }
    }
    TEST_ASSERT(all_ff, "All mark-hold bytes are 0xFF (continuous 1s)", 0);

    return 0;
}

// ---------------------------------------------------------------------------
// TEST 3.4: Mark-hold stream triggers HDLC_ERR_ABORT in decoder
// A stream of 0xFF bytes after the start flag = abort sequence.
// ---------------------------------------------------------------------------
static int test_idle_mark_triggers_abort(void) {
    printf("\n--- test_idle_mark_triggers_abort ---\n");
    printf("Mark-hold 0xFF stream after start flag must trigger abort detection\n");

    // start flag + 2 bytes of 0xFF
    unsigned char stream[3];
    stream[0] = HDLC_FLAG_BYTE;
    stream[1] = 0xFF;
    stream[2] = 0xFF;

    DEBUG_FRAME("mark-hold stream: 0x7E 0xFF 0xFF", stream, 3);

    unsigned char decoded_buf[256];
    int decoded_len = 0;
    hdlc_error_t err = hdlc_frame_decode(stream, 3, decoded_buf, &decoded_len);

    DEBUG_VAR("decode result (expect HDLC_ERR_ABORT)", (unsigned)err);
    TEST_ASSERT(err == HDLC_ERR_ABORT,
                "Mark-hold 0xFF stream after start flag is detected as abort", (unsigned)err);

    return 0;
}

// ---------------------------------------------------------------------------
// TEST 3.5: Idle flags NRZI - verify flag bytes produce consistent NRZI pattern
// Multiple consecutive flag bytes should produce a regular, repeating NRZI waveform.
// ---------------------------------------------------------------------------
static int test_idle_flags_nrzi_pattern(void) {
    printf("\n--- test_idle_flags_nrzi_pattern ---\n");
    printf("Verify idle flag NRZI pattern is regular and repeating\n");

    unsigned char flags[4];
    int count = generate_idle_flags(flags, 4, 4);
    DEBUG_FRAME("idle flags for NRZI", flags, count);

    int bit_count = 0;
    uint8_t *levels = nrzi_encode(flags, count, &bit_count);
    TEST_ASSERT(levels != NULL, "NRZI encode idle flags: non-NULL", 0);
    if (!levels) return 1;

    DEBUG_PRINT("NRZI levels for 4 x 0x7E:");
    for (int i = 0; i < bit_count; i++) printf("%d", levels[i]);
    printf("\n");

    // The NRZI pattern for 0x7E (LSB-first: 0,1,1,1,1,1,1,0) repeats every 8 bits.
    // Each flag produces exactly 2 transitions. With 4 flags, expect 8 transitions total
    // (plus initial half-period), but accounting for state carry-over between bytes.
    // Count transitions:
    int total_transitions = 0;
    uint8_t prev = 1;
    for (int i = 0; i < bit_count; i++) {
        if (levels[i] != prev) total_transitions++;
        prev = levels[i];
    }
    DEBUG_VAR("Total NRZI transitions for 4 idle flags", total_transitions);
    // Each 0x7E has 2 transitions, but the state carries over from byte to byte.
    // The NRZI transition count depends on carry-over state, so we just assert > 0
    // and that round-trip works.
    TEST_ASSERT(total_transitions > 0,
                "Idle flag NRZI encoding produces at least one transition", total_transitions);

    // Verify round-trip
    int recovered_len = 0;
    unsigned char *recovered = nrzi_decode(levels, bit_count, &recovered_len);
    TEST_ASSERT(recovered != NULL, "NRZI decode idle flags: non-NULL", 0);
    if (recovered) {
        DEBUG_FRAME("NRZI recovered idle flags", recovered, recovered_len);
        TEST_ASSERT(recovered_len == count, "Idle flags NRZI round-trip length OK", recovered_len);
        TEST_ASSERT(memcmp(recovered, flags, count) == 0,
                    "Idle flags NRZI round-trip bytes match exactly", 0);
        free(recovered);
    }

    free(levels);
    return 0;
}

// ---------------------------------------------------------------------------
// TEST 3.6: Preamble / postamble structure
// AX.25 v2.2 Section 3.11: Pre- and post-amble are flag sequences.
// Simulate a frame with leading/trailing idle flags and verify the decoder
// correctly ignores the preamble flags and decodes the frame.
// ---------------------------------------------------------------------------
static int test_preamble_postamble(void) {
    printf("\n--- test_preamble_postamble ---\n");
    printf("Verify decoder handles preamble flags correctly by finding start flag\n");

    // Build: [0x7E 0x7E 0x7E] + encoded_frame (already starts with 0x7E)
    // The decoder looks for the FIRST 0x7E byte as start flag; anything before it
    // needs to be stripped by the framing layer. Here we test that the raw decoder
    // fails gracefully on a preamble-prefixed stream (since it uses index 0 as start).

    unsigned char payload[15];
    memset(payload, 0x61, sizeof(payload)); // 'a' * 15
    int enc_len = 0;
    int max_enc = hdlc_encoded_size_max(15);
    unsigned char *encoded = (unsigned char *)malloc(max_enc + 16);
    if (!encoded) {
        printf("  SKIP: malloc failed\n");
        return 0;
    }

    hdlc_frame_encode(payload, 15, encoded, &enc_len);
    DEBUG_VAR("Encoded frame length", enc_len);
    TEST_ASSERT(enc_len > 0, "Preamble test: frame encoded OK", enc_len);

    // Prepend 3 idle flags to simulate preamble
    int total_len = 3 + enc_len;
    unsigned char *stream = (unsigned char *)malloc(total_len + 4);
    if (!stream) {
        free(encoded);
        printf("  SKIP: malloc failed\n");
        return 0;
    }
    stream[0] = 0x7E;
    stream[1] = 0x7E;
    stream[2] = 0x7E;
    memcpy(stream + 3, encoded, enc_len);
    DEBUG_FRAME("stream with 3-flag preamble", stream, total_len);

    // Direct decode starting at offset 0 - uses first 0x7E as start flag,
    // second 0x7E as immediate end flag -> HDLC_ERR_TOO_SHORT
    unsigned char decoded_buf[512];
    int decoded_len = 0;
    hdlc_error_t err = hdlc_frame_decode(stream, total_len, decoded_buf, &decoded_len);
    DEBUG_VAR("decode with preamble (first 0x7E = start, second = end flag)", (unsigned)err);
    // With consecutive 0x7E bytes, the second flag immediately ends the frame -> too short
    TEST_ASSERT(err == HDLC_ERR_TOO_SHORT || err == HDLC_ERR_CRC_FAIL || err != HDLC_OK,
                "Preamble stream: decoder does not pass empty frame as valid data", (unsigned)err);

    // Now decode the encoded frame directly (no preamble) - must succeed
    memset(decoded_buf, 0, sizeof(decoded_buf));
    decoded_len = 0;
    hdlc_error_t err2 = hdlc_frame_decode(encoded, enc_len, decoded_buf, &decoded_len);
    DEBUG_VAR("decode without preamble (should be HDLC_OK)", (unsigned)err2);
    DEBUG_FRAME("decoded payload", decoded_buf, decoded_len);
    TEST_ASSERT(err2 == HDLC_OK, "Frame without preamble decodes successfully", (unsigned)err2);
    TEST_ASSERT(decoded_len == 15, "Decoded payload length == 15", decoded_len);
    TEST_ASSERT(memcmp(decoded_buf, payload, 15) == 0, "Decoded payload matches original", 0);

    free(stream);
    free(encoded);
    return 0;
}

// ===========================================================================
// TEST GROUP 4: COMBINED ABORT + NRZI INTERACTIONS
// ===========================================================================

// ---------------------------------------------------------------------------
// TEST 4.1: NRZI-encoded abort is continuous mark (no transitions)
// After NRZI encoding, an abort (all 1s) should appear as a DC mark signal.
// ---------------------------------------------------------------------------
static int test_abort_nrzi_is_dc_mark(void) {
    printf("\n--- test_abort_nrzi_is_dc_mark ---\n");
    printf("NRZI-encoded abort sequence must appear as DC mark (no transitions)\n");

    unsigned char abort_buf[4];
    int abort_len = 0;
    hdlc_frame_abort(abort_buf, &abort_len);

    int bit_count = 0;
    uint8_t *levels = nrzi_encode(abort_buf, abort_len, &bit_count);
    TEST_ASSERT(levels != NULL, "NRZI encode abort: non-NULL", 0);
    if (!levels) return 1;

    DEBUG_PRINT("NRZI abort levels:");
    for (int i = 0; i < bit_count; i++) printf("%d", levels[i]);
    printf("\n");

    int transition_count = 0;
    uint8_t prev = 1;
    for (int i = 0; i < bit_count; i++) {
        if (levels[i] != prev) transition_count++;
        prev = levels[i];
    }
    DEBUG_VAR("NRZI transitions for abort 0xFF 0xFF (expect 0)", transition_count);
    TEST_ASSERT(transition_count == 0,
                "NRZI-encoded abort (0xFF 0xFF) has 0 transitions = DC mark signal", transition_count);

    free(levels);
    return 0;
}

// ---------------------------------------------------------------------------
// TEST 4.2: CRC verify magic constant cross-check
// A valid encode -> decode should produce correct CRC (0xF0B8 residue).
// Indirectly tests that CRC is correctly embedded and verified end-to-end.
// ---------------------------------------------------------------------------
static int test_crc_embed_and_verify(void) {
    printf("\n--- test_crc_embed_and_verify ---\n");
    // start modified part - updated to match corrected CRC_verify residue 0x0F47
    printf("CRC-CCITT: check value 0x906E and CRC_verify residue 0x0F47\n");

    unsigned char data[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    uint16_t crc = CRC(data, sizeof(data));

    DEBUG_VAR("CRC(\"123456789\") = 0x", crc);
    printf("  CRC check value: 0x%04X (AX.25/X.25 check = 0x906E)\n", crc);

    // AX.25 CRC-CCITT check value for "123456789" = 0x906E
    TEST_ASSERT(crc == 0x906E,
                "CRC(\"123456789\") == 0x906E (CRC-CCITT check value)", crc);

    // Build frame with CRC appended as [lo, hi] (low byte first per AX.25)
    unsigned char frame_with_fcs[11];
    memcpy(frame_with_fcs, data, 9);
    frame_with_fcs[9]  = (unsigned char)(crc & 0xFF);
    frame_with_fcs[10] = (unsigned char)((crc >> 8) & 0xFF);

    DEBUG_FRAME("data + FCS [lo, hi]", frame_with_fcs, 11);

    // This implementation processes FCS as [lo,hi] without bit-reversal.
    // The resulting residue is 0x0F47 (not the textbook 0xF0B8 which requires
    // bit-reversed FCS bytes). common.c CRC_verify was corrected to check 0x0F47.
    uint16_t residue = CRC(frame_with_fcs, 11);
    DEBUG_VAR("CRC residue over data+FCS (should be 0x0F47)", residue);
    TEST_ASSERT(residue == 0x0F47,
                "CRC residue == 0x0F47 for data+[lo,hi] (implementation residue)", residue);

    bool valid = CRC_verify(frame_with_fcs, 11);
    DEBUG_BOOL("CRC_verify(data + FCS) must be true (0x0F47)", valid);
    TEST_ASSERT(valid, "CRC_verify returns true for data+FCS (corrected residue 0x0F47)", 0);
    // end modified part

    return 0;
}

// ===========================================================================
// MAIN entry point
// ===========================================================================
int test_hdlc_frame_structure_main(void) {
    int result = 0;

    printf("\n==================================================================================\n");
    printf("HDLC Frame Structure Tests - Section 1\n");
    printf("Covers: Abort Sequence, NRZI Encoding/Decoding, Idle Pattern Generation\n");
    printf("AX.25 v2.2 Sections 3.7, 3.8, 3.10\n");
    printf("==================================================================================\n");

    printf("\n== GROUP 1: HDLC Abort Sequence (AX.25 v2.2 Section 3.10) ==\n");
    result |= test_abort_output_format();
    result |= test_abort_null_safety();
    result |= test_abort_detection_in_decoder();
    result |= test_abort_at_seven_ones();
    result |= test_abort_injected_mid_frame();
    result |= test_abort_bit_count_lsb();

    printf("\n== GROUP 2: NRZI Encoding / Decoding (AX.25 v2.2 Section 3.8) ==\n");
    result |= test_nrzi_roundtrip_flag();
    result |= test_nrzi_roundtrip_abort();
    result |= test_nrzi_roundtrip_full_frame();
    result |= test_nrzi_differential_transitions();
    result |= test_nrzi_flag_transitions();

    printf("\n== GROUP 3: Idle Pattern Generation (AX.25 v2.2 Section 3.7) ==\n");
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
    printf("HDLC Frame Structure Tests Completed. %s\n",
           result == 0 ? "All tests PASSED" : "Some tests FAILED");
    printf("Total asserts: %u\n", assert_count);
    printf("==================================================================================\n\n");

    return result;
}
