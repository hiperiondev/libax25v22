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

/*
 * Comprehensive test suite for IL2P encode/decode pipeline.
 * Covers all items in the unchecked extensions list:
 *   - IL2P header mapping (Type 0 and Type 1)
 *   - SIXBIT callsign compression
 *   - Reed-Solomon payload blocks (encode + decode + error correction)
 *   - 24-bit sync word (write, search, bit-error tolerance)
 *   - Scrambling / descrambling (LFSR round-trip)
 *   - Type 0 transparent encapsulation
 *   - Type 1 translated encapsulation
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

// Library headers
#include "ax25.h"
#include "common.h"
#include "fx25_gf256.h"
#include "il2p_sync.h"
#include "il2p_lfsr.h"
#include "il2p_sixbit.h"
#include "il2p_rs.h"
#include "il2p_header.h"
#include "il2p.h"
#include "hal.h"

// Test infrastructure (mirrors test_common.h style)
#include "test_common.h"

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

static void fill_address(ax25_address_t *addr, const char *call, uint8_t ssid) {
    memset(addr->callsign, ' ', 6);
    addr->callsign[6] = '\0';
    size_t l = strlen(call);
    if (l > 6)
        l = 6;
    memcpy(addr->callsign, call, l);
    addr->ssid = ssid;
}

static void print_buf(const char *label, const uint8_t *buf, size_t len) {
    printf("  [DBG] %-40s (%3zu bytes): ", label, len);
    for (size_t i = 0; i < len; i++)
        printf("%02X ", buf[i]);
    printf("\n");
}

// -----------------------------------------------------------------------
// Test counter (used by TEST_ASSERT macro from test_common.h)
// -----------------------------------------------------------------------
static int assert_count = 0;

// -----------------------------------------------------------------------
// SECTION 1: GF(256) arithmetic sanity
// -----------------------------------------------------------------------
static int test_gf256(void) {
    printf("\n=== GF(256) arithmetic ===\n");

    // Addition is XOR
    TEST_ASSERT(gf_add(0x53, 0xCA) == (0x53 ^ 0xCA), "gf_add is XOR", 0);
    TEST_ASSERT(gf_add(0x00, 0xFF) == 0xFF, "gf_add identity", 0);
    TEST_ASSERT(gf_add(0xAB, 0xAB) == 0x00, "gf_add self-inverse", 0);

    // Subtraction identical to addition in GF(2^m)
    TEST_ASSERT(gf_sub(0x53, 0xCA) == gf_add(0x53, 0xCA), "gf_sub == gf_add", 0);

    // Multiply by 0
    TEST_ASSERT(gf_mul(0x00, 0xFF) == 0x00, "gf_mul by zero", 0);
    TEST_ASSERT(gf_mul(0xFF, 0x00) == 0x00, "gf_mul zero by x", 0);

    // Multiply by 1
    TEST_ASSERT(gf_mul(0x01, 0xAB) == 0xAB, "gf_mul identity", 0);
    TEST_ASSERT(gf_mul(0xAB, 0x01) == 0xAB, "gf_mul identity commutative", 0);

    // Commutativity
    uint8_t a = 0x53, b = 0xCA;
    TEST_ASSERT(gf_mul(a, b) == gf_mul(b, a), "gf_mul commutative", 0);

    // Associativity: (a*b)*c == a*(b*c)
    uint8_t c = 0x11;
    TEST_ASSERT(gf_mul(gf_mul(a, b), c) == gf_mul(a, gf_mul(b, c)), "gf_mul associative", 0);

    // Distributivity: a*(b+c) == a*b + a*c
    TEST_ASSERT(gf_mul(a, gf_add(b, c)) == gf_add(gf_mul(a, b), gf_mul(a, c)), "gf_mul distributive", 0);

    // Division: a / a == 1
    TEST_ASSERT(gf_div(0xAB, 0xAB) == 0x01, "gf_div self is 1", 0);

    // Division: (a*b)/b == a
    uint8_t prod = gf_mul(a, b);
    uint8_t quot = gf_div(prod, b);
    printf("  [DBG] a=0x%02X b=0x%02X a*b=0x%02X (a*b)/b=0x%02X\n", a, b, prod, quot);
    TEST_ASSERT(quot == a, "gf_div inverse of mul", 0);

    // Division by zero returns 0xFF and does not crash
    uint8_t dz = gf_div(0x01, 0x00);
    TEST_ASSERT(dz == 0xFF, "gf_div by zero returns 0xFF", 0);

    // Inverse: a * inv(a) == 1
    uint8_t inv_a = gf_inverse(a);
    printf("  [DBG] a=0x%02X inv=0x%02X product=0x%02X\n", a, inv_a, gf_mul(a, inv_a));
    TEST_ASSERT(gf_mul(a, inv_a) == 0x01, "gf_inverse", 0);
    TEST_ASSERT(gf_inverse(0x00) == 0x00, "gf_inverse(0)==0", 0);

    // Power: a^0 == 1
    TEST_ASSERT(gf_pow(a, 0) == 0x01, "gf_pow x^0==1", 0);
    // Power: 0^n == 0 for n>0
    TEST_ASSERT(gf_pow(0x00, 5) == 0x00, "gf_pow 0^n==0", 0);
    // Power: a^1 == a
    TEST_ASSERT(gf_pow(a, 1) == a, "gf_pow x^1==x", 0);
    // Fermat: a^255 == 1 for a != 0
    TEST_ASSERT(gf_pow(a, 255) == 0x01, "gf_pow Fermat a^255==1", 0);

    // gf_exp table: exp[0]==1, exp[1]==2 (generator alpha=2)
    TEST_ASSERT(gf_exp[0] == 0x01, "gf_exp[0]==1", 0);
    TEST_ASSERT(gf_exp[1] == 0x02, "gf_exp[1]==2 (alpha)", 0);
    TEST_ASSERT(gf_exp[255] == 0x01, "gf_exp[255]==1 (period)", 0);

    // gf_log: log[1]==0 (log of identity), log[2]==1 (log of generator)
    TEST_ASSERT(gf_log[1] == 0x00, "gf_log[1]==0", 0);
    TEST_ASSERT(gf_log[2] == 0x01, "gf_log[2]==1", 0);

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 2: CRC-CCITT
// -----------------------------------------------------------------------
static int test_crc(void) {
    printf("\n=== CRC-CCITT ===\n");

    // Known AX.25 test vector
    // Check value for "123456789" in CRC-16/X-25 = 0x906E
    uint8_t test_str[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    uint16_t crc_val = hal_crc16_buf(test_str, 9);
    printf("  [DBG] CRC('123456789') = 0x%04X (expected 0x906E)\n", crc_val);
    TEST_ASSERT(crc_val == 0x906E, "CRC('123456789') == 0x906E", crc_val);

    // Round-trip: calculate CRC, append as [lo, hi], verify via 0x0F47 residual
    uint8_t frame[16] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00 };
    size_t data_len = 4;
    uint16_t fcs = hal_crc16_buf(frame, (uint16_t) data_len);
    frame[data_len] = (uint8_t) (fcs & 0xFF);
    frame[data_len + 1] = (uint8_t) ((fcs >> 8) & 0xFF);
    printf("  [DBG] FCS=0x%04X appended as [%02X %02X]\n", fcs, frame[data_len], frame[data_len + 1]);
    // CRC_verify replaced: hal_crc16_buf over data+FCS must yield residual 0x0F47
    TEST_ASSERT(hal_crc16_buf(frame, (uint16_t )(data_len + 2)) == 0x0F47u, "CRC_verify round-trip", fcs);

    // Corrupt one byte and verify fails
    frame[0] ^= 0xFF;
    TEST_ASSERT(hal_crc16_buf(frame, (uint16_t )(data_len + 2)) != 0x0F47u, "CRC_verify detects corruption", 0);

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 3: SIXBIT callsign encoding
// -----------------------------------------------------------------------
static int test_sixbit(void) {
    printf("\n=== SIXBIT callsign ===\n");

    // Single character encode/decode round-trips
    uint8_t out;
    TEST_ASSERT(il2p_sixbit_encode_char('A', &out), "encode 'A'", 0);
    printf("  [DBG] 'A'(0x41) -> sixbit 0x%02X, decode -> 0x%02X ('%c')\n", out, il2p_sixbit_decode_char(out), il2p_sixbit_decode_char(out));
    TEST_ASSERT(il2p_sixbit_decode_char(out) == 'A', "decode 'A'", 0);

    TEST_ASSERT(il2p_sixbit_encode_char('0', &out), "encode '0'", 0);
    TEST_ASSERT(il2p_sixbit_decode_char(out) == '0', "decode '0'", 0);

    TEST_ASSERT(il2p_sixbit_encode_char(' ', &out), "encode ' '", 0);
    TEST_ASSERT(il2p_sixbit_decode_char(out) == ' ', "decode ' '", 0);

    // Character below valid range
    TEST_ASSERT(!il2p_sixbit_encode_char(0x1F, &out), "reject 0x1F", 0);
    // Character above valid range
    TEST_ASSERT(!il2p_sixbit_encode_char(0x60, &out), "reject 0x60 (lowercase)", 0);

    // Callsign round-trips
    struct {
        const char *call;
        bool expect_ok;
    } calls[] = { { "W1AW  ", true }, { "VE3XYZ", true }, { "N0CALL", true }, { "KA1ABC", true }, { "      ", true },  // all spaces
            { "ab1cde", false },  // lowercase -- not SIXBIT-encodable
            { "W1\x80Z  ", false },  // non-ASCII character
            };

    for (size_t i = 0; i < sizeof(calls) / sizeof(calls[0]); i++) {
        uint8_t sb[6];
        bool ok = il2p_sixbit_encode_callsign(calls[i].call, sb);
        printf("  [DBG] encode_callsign(\"%s\") -> %s\n", calls[i].call, ok ? "OK" : "FAIL");
        if (ok) {
            char decoded[7];
            il2p_sixbit_decode_callsign(sb, decoded);
            printf("         decoded -> \"%s\"\n", decoded);
        }
        TEST_ASSERT(ok == calls[i].expect_ok, calls[i].call, 0);

        if (ok) {
            uint8_t sb2[6];
            char decoded[7];
            il2p_sixbit_decode_callsign(sb, decoded);
            // Re-encode decoded to verify stability
            bool ok2 = il2p_sixbit_encode_callsign(decoded, sb2);
            TEST_ASSERT(ok2, "re-encode decoded callsign", 0);
            TEST_ASSERT(memcmp(sb, sb2, 6) == 0, "re-encode matches original", 0);
        }
    }

    // NULL safety
    // use explicit variable instead of compound literal for C99 portability
    {
        uint8_t tmp6[6] = { 0 };
        TEST_ASSERT(!il2p_sixbit_encode_callsign(NULL, tmp6), "NULL callsign rejected", 0);
    }
    TEST_ASSERT(!il2p_sixbit_encode_callsign("W1AW  ", NULL), "NULL output rejected", 0);

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 4: Reed-Solomon encode / decode
// -----------------------------------------------------------------------
static int test_rs(void) {
    printf("\n=== Reed-Solomon ===\n");

    // --- Header RS block: 13 data + 2 parity ---
    {
        uint8_t block[IL2P_RS_HDR_DATA + IL2P_RS_HDR_PARITY + 1];  // +1 safety
        uint8_t orig[IL2P_RS_HDR_DATA];
        for (uint8_t i = 0; i < IL2P_RS_HDR_DATA; i++)
            orig[i] = (uint8_t) (0xA0 + i);
        memcpy(block, orig, IL2P_RS_HDR_DATA);
        print_buf("RS header data in", block, IL2P_RS_HDR_DATA);

        bool enc_ok = il2p_rs_encode(block, IL2P_RS_HDR_DATA, IL2P_RS_HDR_PARITY);
        TEST_ASSERT(enc_ok, "RS header encode returns true", 0);
        print_buf("RS header encoded (data+parity)", block, IL2P_RS_HDR_DATA + IL2P_RS_HDR_PARITY);

        // Decode with no errors -- should return 0
        int8_t corr = il2p_rs_decode(block, IL2P_RS_HDR_DATA, IL2P_RS_HDR_PARITY);
        printf("  [DBG] RS decode corrections (no error): %d\n", corr);
        TEST_ASSERT(corr == 0, "RS header decode no error", corr);
        TEST_ASSERT(memcmp(block, orig, IL2P_RS_HDR_DATA) == 0, "RS header data unchanged", 0);

        // Introduce 1 error (1 error correctable with 2 parity bytes -> t=1)
        uint8_t block_err[IL2P_RS_HDR_DATA + IL2P_RS_HDR_PARITY];
        memcpy(block_err, block, sizeof(block_err));
        block_err[3] ^= 0xFF;
        printf("  [DBG] Injected error at byte 3: 0x%02X -> 0x%02X\n", block[3], block_err[3]);
        int8_t corr1 = il2p_rs_decode(block_err, IL2P_RS_HDR_DATA, IL2P_RS_HDR_PARITY);
        printf("  [DBG] RS decode corrections (1 error): %d\n", corr1);
        TEST_ASSERT(corr1 == 1, "RS header corrects 1 error", corr1);
        TEST_ASSERT(memcmp(block_err, orig, IL2P_RS_HDR_DATA) == 0, "RS header data restored after 1 error", 0);

        // Introduce 2 errors -- exceeds t=1, should fail
        uint8_t block_err2[IL2P_RS_HDR_DATA + IL2P_RS_HDR_PARITY];
        memcpy(block_err2, block, sizeof(block_err2));
        block_err2[0] ^= 0xFF;
        block_err2[7] ^= 0xFF;
        int8_t corr2 = il2p_rs_decode(block_err2, IL2P_RS_HDR_DATA, IL2P_RS_HDR_PARITY);
        printf("  [DBG] RS decode result (2 errors, t=1): %d (expect -1)\n", corr2);
        TEST_ASSERT(corr2 == -1, "RS header fails on 2 errors (t=1)", corr2);
    }

    // --- Payload RS block: variable data + 16 parity, t=8 ---
    {
        uint8_t data_len = 50;
        uint8_t block[IL2P_RS_PAY_MAX_DATA + IL2P_RS_PAY_PARITY];
        uint8_t orig[IL2P_RS_PAY_MAX_DATA];
        for (uint8_t i = 0; i < data_len; i++)
            orig[i] = (uint8_t) (i * 7 + 13);
        memcpy(block, orig, data_len);

        bool enc_ok = il2p_rs_encode(block, data_len, IL2P_RS_PAY_PARITY);
        TEST_ASSERT(enc_ok, "RS payload encode returns true", 0);
        print_buf("RS payload parity bytes", block + data_len, IL2P_RS_PAY_PARITY);

        // No error
        int8_t c0 = il2p_rs_decode(block, data_len, IL2P_RS_PAY_PARITY);
        printf("  [DBG] RS payload decode (no error): %d\n", c0);
        TEST_ASSERT(c0 == 0, "RS payload decode no error", c0);
        TEST_ASSERT(memcmp(block, orig, data_len) == 0, "RS payload data unchanged", 0);

        // 8 errors (at capacity t=8)
        uint8_t block8[IL2P_RS_PAY_MAX_DATA + IL2P_RS_PAY_PARITY];
        // Re-encode fresh
        memcpy(block8, orig, data_len);
        il2p_rs_encode(block8, data_len, IL2P_RS_PAY_PARITY);
        uint8_t err_positions[8] = { 0, 5, 10, 15, 20, 25, 30, 35 };
        for (int i = 0; i < 8; i++)
            block8[err_positions[i]] ^= 0xAA;
        printf("  [DBG] Injected 8 errors at positions: ");
        for (int i = 0; i < 8; i++)
            printf("%u ", err_positions[i]);
        printf("\n");
        int8_t c8 = il2p_rs_decode(block8, data_len, IL2P_RS_PAY_PARITY);
        printf("  [DBG] RS payload decode (8 errors): %d\n", c8);
        TEST_ASSERT(c8 == 8, "RS payload corrects 8 errors (t=8)", c8);
        TEST_ASSERT(memcmp(block8, orig, data_len) == 0, "RS payload data restored (8 errors)", 0);

        // 9 errors -- should fail
        uint8_t block9[IL2P_RS_PAY_MAX_DATA + IL2P_RS_PAY_PARITY];
        memcpy(block9, orig, data_len);
        il2p_rs_encode(block9, data_len, IL2P_RS_PAY_PARITY);
        for (int i = 0; i < 9; i++)
            block9[i] ^= 0x55;
        int8_t c9 = il2p_rs_decode(block9, data_len, IL2P_RS_PAY_PARITY);
        printf("  [DBG] RS payload decode result (9 errors, t=8): %d (expect -1)\n", c9);
        TEST_ASSERT(c9 == -1, "RS payload fails on 9 errors (t=8)", c9);

        // Errors in parity bytes only
        uint8_t blkp[IL2P_RS_PAY_MAX_DATA + IL2P_RS_PAY_PARITY];
        memcpy(blkp, orig, data_len);
        il2p_rs_encode(blkp, data_len, IL2P_RS_PAY_PARITY);
        blkp[data_len + 0] ^= 0xFF;  // corrupt first parity byte
        blkp[data_len + 1] ^= 0xFF;  // corrupt second parity byte
        int8_t cp = il2p_rs_decode(blkp, data_len, IL2P_RS_PAY_PARITY);
        printf("  [DBG] RS payload decode (2 parity errors): %d\n", cp);
        TEST_ASSERT(cp == 2, "RS corrects errors in parity bytes", cp);
        TEST_ASSERT(memcmp(blkp, orig, data_len) == 0, "RS data intact after parity error", 0);
    }

    // --- Max payload block (239 bytes) ---
    {
        uint8_t block[IL2P_RS_PAY_MAX_DATA + IL2P_RS_PAY_PARITY];
        uint8_t orig[IL2P_RS_PAY_MAX_DATA];
        for (unsigned int i = 0u; i < IL2P_RS_PAY_MAX_DATA; i++)
            orig[i] = (uint8_t) (i ^ 0x5A);
        memcpy(block, orig, IL2P_RS_PAY_MAX_DATA);
        bool enc = il2p_rs_encode(block, IL2P_RS_PAY_MAX_DATA, IL2P_RS_PAY_PARITY);
        TEST_ASSERT(enc, "RS max block encode", 0);
        int8_t dec = il2p_rs_decode(block, IL2P_RS_PAY_MAX_DATA, IL2P_RS_PAY_PARITY);
        TEST_ASSERT(dec == 0, "RS max block decode no error", dec);
        TEST_ASSERT(memcmp(block, orig, IL2P_RS_PAY_MAX_DATA) == 0, "RS max block data intact", 0);
    }

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 5: LFSR scrambler
// -----------------------------------------------------------------------
static int test_lfsr(void) {
    printf("\n=== LFSR scrambler ===\n");

    il2p_lfsr_t lfsr;

    // Reset test
    il2p_lfsr_reset(&lfsr);
    TEST_ASSERT(lfsr.state == IL2P_LFSR_INIT, "LFSR reset to 0x1FF", lfsr.state);

    // All-zero input does not produce all-zero output (scrambling)
    uint8_t zeros[16];
    memset(zeros, 0, sizeof(zeros));
    il2p_lfsr_reset(&lfsr);
    uint8_t scrambled[16];
    for (size_t i = 0; i < 16; i++) {
        scrambled[i] = il2p_lfsr_step_byte(&lfsr, zeros[i]);
    }
    bool all_zero = true;
    for (size_t i = 0; i < 16; i++)
        if (scrambled[i] != 0) {
            all_zero = false;
            break;
        }
    printf("  [DBG] scrambled all-zeros: ");
    for (int i = 0; i < 8; i++)
        printf("%02X ", scrambled[i]);
    printf("...\n");
    TEST_ASSERT(!all_zero, "LFSR scrambles zeros to non-zero", 0);

    // Round-trip: scramble then descramble (same LFSR XOR = inverse)
    uint8_t data[32];
    for (size_t i = 0; i < sizeof(data); i++)
        data[i] = (uint8_t) (i * 37 + 0xAB);
    uint8_t orig[32];
    memcpy(orig, data, sizeof(data));

    // Scramble
    il2p_lfsr_reset(&lfsr);
    uint8_t enc[32];
    for (size_t i = 0; i < 32; i++)
        enc[i] = il2p_lfsr_step_byte(&lfsr, data[i]);

    // Descramble (reset LFSR same initial state, XOR again)
    il2p_lfsr_reset(&lfsr);
    uint8_t dec[32];
    for (size_t i = 0; i < 32; i++)
        dec[i] = il2p_lfsr_step_byte(&lfsr, enc[i]);

    print_buf("LFSR original", orig, 16);
    print_buf("LFSR scrambled", enc, 16);
    print_buf("LFSR descrambled", dec, 16);
    TEST_ASSERT(memcmp(dec, orig, 32) == 0, "LFSR round-trip", 0);

    // Test using the convenience functions
    uint8_t buf1[20], buf2[20];
    for (size_t i = 0; i < 20; i++)
        buf1[i] = buf2[i] = (uint8_t) (0x55 ^ i);
    il2p_lfsr_scramble(&lfsr, buf1, 20);
    il2p_lfsr_descramble(&lfsr, buf1, 20);
    TEST_ASSERT(memcmp(buf1, buf2, 20) == 0, "LFSR scramble+descramble convenience round-trip", 0);

    // Different blocks get independent LFSR state (reset between blocks)
    uint8_t blkA[8], blkB[8];
    for (size_t i = 0; i < 8; i++)
        blkA[i] = blkB[i] = (uint8_t) i;
    // Block A
    il2p_lfsr_reset(&lfsr);
    for (size_t i = 0; i < 8; i++)
        blkA[i] = il2p_lfsr_step_byte(&lfsr, blkA[i]);
    // Block B (identical input, reset LFSR)
    il2p_lfsr_reset(&lfsr);
    for (size_t i = 0; i < 8; i++)
        blkB[i] = il2p_lfsr_step_byte(&lfsr, blkB[i]);
    // They must produce identical output since LFSR was reset
    TEST_ASSERT(memcmp(blkA, blkB, 8) == 0, "LFSR reset gives identical output for same input", 0);

    // Single-bit step test: feed 0x80 (only MSB set), verify output bit pattern
    il2p_lfsr_reset(&lfsr);
    uint8_t first_byte_out = il2p_lfsr_step_byte(&lfsr, 0x80);
    printf("  [DBG] step_byte(0x80) first output = 0x%02X\n", first_byte_out);
    // MSB of output depends on LFSR initial state MSB (bit 8 of 0x1FF = 1)
    // First step: out_bit = (0x1FF >> 8) & 1 = 1
    // In-bit for MSB of 0x80 = 1, so XOR: 1^1 = 0 for MSB of result
    TEST_ASSERT((first_byte_out & 0x80) == 0x00, "LFSR first output MSB is 0 for input 0x80", first_byte_out);

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 6: Sync word
// -----------------------------------------------------------------------
static int test_sync(void) {
    printf("\n=== Sync word ===\n");

    // Write and verify bytes
    uint8_t buf[8] = { 0 };
    il2p_sync_write(buf);
    printf("  [DBG] sync bytes: %02X %02X %02X\n", buf[0], buf[1], buf[2]);
    TEST_ASSERT(buf[0] == 0xF1, "sync byte 0 = 0xF1", buf[0]);
    TEST_ASSERT(buf[1] == 0x5E, "sync byte 1 = 0x5E", buf[1]);
    TEST_ASSERT(buf[2] == 0x48, "sync byte 2 = 0x48", buf[2]);
    TEST_ASSERT(buf[3] == 0x00, "sync no overflow", buf[3]);

    // Match: exact sync word (0 bit errors)
    TEST_ASSERT(il2p_sync_bit_errors(IL2P_SYNC_WORD) == 0, "sync match 0 errors", 0);
    TEST_ASSERT(il2p_sync_match(IL2P_SYNC_WORD), "sync_match exact", 0);

    // 1-bit error (should still match per IL2P_SYNC_MAX_ERRORS=1)
    uint32_t one_err = IL2P_SYNC_WORD ^ 0x000001UL;
    printf("  [DBG] 1-bit flip candidate: 0x%06lX, errors=%u\n", (unsigned long) one_err, il2p_sync_bit_errors(one_err));
    TEST_ASSERT(il2p_sync_bit_errors(one_err) == 1, "1-bit error count == 1", 0);
    TEST_ASSERT(il2p_sync_match(one_err), "sync_match 1 bit error", 0);

    // 2-bit error (should NOT match)
    uint32_t two_err = IL2P_SYNC_WORD ^ 0x000003UL;
    TEST_ASSERT(il2p_sync_bit_errors(two_err) == 2, "2-bit error count == 2", 0);
    TEST_ASSERT(!il2p_sync_match(two_err), "sync_match rejects 2 bit errors", 0);

    // All zeros -- many bit errors
    TEST_ASSERT(!il2p_sync_match(0x000000UL), "sync_match rejects all zeros", 0);
    // All ones (0xFFFFFF) -- many bit errors
    TEST_ASSERT(!il2p_sync_match(0xFFFFFFUL), "sync_match rejects all ones", 0);

    // Search: exact match at byte 0
    uint8_t frame[32] = { 0 };
    il2p_sync_write(frame);
    size_t byte_off;
    uint8_t bit_off;
    bool found = il2p_sync_search(frame, 32, &byte_off, &bit_off);
    printf("  [DBG] sync search (at offset 0): found=%d byte_off=%zu bit_off=%u\n", found, byte_off, bit_off);
    TEST_ASSERT(found, "sync search finds word at offset 0", 0);
    TEST_ASSERT(byte_off == 0, "sync search byte_off == 0", (int )byte_off);
    TEST_ASSERT(bit_off == 0, "sync search bit_off == 0", bit_off);

    // Search: sync word at a non-zero offset
    uint8_t frame2[32] = { 0 };
    frame2[5] = 0xF1;
    frame2[6] = 0x5E;
    frame2[7] = 0x48;
    found = il2p_sync_search(frame2, 32, &byte_off, &bit_off);
    printf("  [DBG] sync search (at offset 5): found=%d byte_off=%zu bit_off=%u\n", found, byte_off, bit_off);
    TEST_ASSERT(found, "sync search finds word at offset 5", 0);
    TEST_ASSERT(byte_off == 5, "sync search byte_off == 5", (int )byte_off);
    TEST_ASSERT(bit_off == 0, "sync search bit_off == 0 (byte-aligned)", bit_off);

    // Search: 1-bit corrupt sync word should still be found
    uint8_t frame3[32] = { 0xFF, 0x00 };  // noise prefix
    frame3[2] = 0xF1;
    frame3[3] = 0x5E;
    frame3[4] = 0x49;  // last byte: 0x48^0x01
    found = il2p_sync_search(frame3, 32, &byte_off, &bit_off);
    printf("  [DBG] sync search (1-bit error): found=%d byte_off=%zu bit_off=%u\n", found, byte_off, bit_off);
    TEST_ASSERT(found, "sync search finds word with 1-bit error", 0);

    // Search: no sync word in buffer
    uint8_t frame4[16];
    memset(frame4, 0x55, sizeof(frame4));
    found = il2p_sync_search(frame4, 16, &byte_off, &bit_off);
    TEST_ASSERT(!found, "sync search returns false when not found", 0);

    // Search: buffer too short
    found = il2p_sync_search(frame, 2, &byte_off, &bit_off);
    TEST_ASSERT(!found, "sync search fails on too-short buffer", 0);

    // NULL safety
    found = il2p_sync_search(NULL, 32, &byte_off, &bit_off);
    TEST_ASSERT(!found, "sync search NULL buf returns false", 0);

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 7: IL2P header encode/decode
// -----------------------------------------------------------------------
static int test_header(void) {
    printf("\n=== IL2P header encode/decode ===\n");

    // Build a Type 1 header manually
    il2p_header_t hdr_in, hdr_out;
    memset(&hdr_in, 0, sizeof(hdr_in));
    hdr_in.hdr_type = IL2P_HDR_TYPE_1_TRANSLATED;
    hdr_in.payload_byte_count = 42;
    hdr_in.pid = IL2P_PID_NO_L3;
    hdr_in.control = 0x7F;
    hdr_in.ui = 0;
    hdr_in.dest_ssid = 3;
    hdr_in.src_ssid = 0;

    // SIXBIT-encode callsigns
    TEST_ASSERT(il2p_sixbit_encode_callsign("W1AW  ", hdr_in.dest_callsign), "encode dest callsign", 0);
    TEST_ASSERT(il2p_sixbit_encode_callsign("VE3TKI", hdr_in.src_callsign), "encode src callsign", 0);

    // Encode header to 13 bytes
    uint8_t raw_hdr[IL2P_HEADER_SIZE];
    uint8_t n = il2p_header_encode(&hdr_in, raw_hdr);
    printf("  [DBG] il2p_header_encode returned %u\n", n);
    print_buf("Encoded header (13 bytes)", raw_hdr, IL2P_HEADER_SIZE);
    TEST_ASSERT(n == IL2P_HEADER_SIZE, "header encode returns 13", n);

    // Decode back
    bool dec_ok = il2p_header_decode(raw_hdr, &hdr_out);
    TEST_ASSERT(dec_ok, "header decode returns true", 0);

    printf("  [DBG] hdr_type: in=%u out=%u\n", hdr_in.hdr_type, hdr_out.hdr_type);
    printf("  [DBG] payload_byte_count: in=%u out=%u\n", hdr_in.payload_byte_count, hdr_out.payload_byte_count);
    printf("  [DBG] pid: in=%u out=%u\n", hdr_in.pid, hdr_out.pid);
    printf("  [DBG] control: in=0x%02X out=0x%02X\n", hdr_in.control, hdr_out.control);
    printf("  [DBG] dest_ssid: in=%u out=%u\n", hdr_in.dest_ssid, hdr_out.dest_ssid);
    printf("  [DBG] src_ssid: in=%u out=%u\n", hdr_in.src_ssid, hdr_out.src_ssid);

    TEST_ASSERT(hdr_out.hdr_type == hdr_in.hdr_type, "header hdr_type round-trip", hdr_out.hdr_type);
    TEST_ASSERT(hdr_out.payload_byte_count == hdr_in.payload_byte_count, "header payload_byte_count round-trip", hdr_out.payload_byte_count);
    TEST_ASSERT(hdr_out.pid == hdr_in.pid, "header pid round-trip", hdr_out.pid);
    TEST_ASSERT(hdr_out.control == hdr_in.control, "header control round-trip", hdr_out.control);
    TEST_ASSERT(hdr_out.dest_ssid == hdr_in.dest_ssid, "header dest_ssid round-trip", hdr_out.dest_ssid);
    TEST_ASSERT(hdr_out.src_ssid == hdr_in.src_ssid, "header src_ssid round-trip", hdr_out.src_ssid);
    TEST_ASSERT(hdr_out.ui == hdr_in.ui, "header ui round-trip", hdr_out.ui);
    TEST_ASSERT(memcmp(hdr_out.dest_callsign, hdr_in.dest_callsign, 6) == 0, "header dest_callsign round-trip", 0);
    TEST_ASSERT(memcmp(hdr_out.src_callsign, hdr_in.src_callsign, 6) == 0, "header src_callsign round-trip", 0);

    // Type 0 header: all address fields should be zero
    il2p_header_t hdr_t0_in, hdr_t0_out;
    memset(&hdr_t0_in, 0, sizeof(hdr_t0_in));
    hdr_t0_in.hdr_type = IL2P_HDR_TYPE_0_TRANSPARENT;
    hdr_t0_in.payload_byte_count = 1023;  // Max payload
    uint8_t raw_t0[IL2P_HEADER_SIZE];
    il2p_header_encode(&hdr_t0_in, raw_t0);
    il2p_header_decode(raw_t0, &hdr_t0_out);
    printf("  [DBG] Type 0: payload_byte_count in=%u out=%u\n", hdr_t0_in.payload_byte_count, hdr_t0_out.payload_byte_count);
    TEST_ASSERT(hdr_t0_out.hdr_type == IL2P_HDR_TYPE_0_TRANSPARENT, "header Type 0 round-trip", hdr_t0_out.hdr_type);
    TEST_ASSERT(hdr_t0_out.payload_byte_count == 1023, "header max payload_byte_count round-trip", hdr_t0_out.payload_byte_count);

    // Zero payload
    il2p_header_t hdr_z;
    memset(&hdr_z, 0, sizeof(hdr_z));
    hdr_z.hdr_type = IL2P_HDR_TYPE_0_TRANSPARENT;
    hdr_z.payload_byte_count = 0;
    uint8_t raw_z[IL2P_HEADER_SIZE];
    il2p_header_encode(&hdr_z, raw_z);
    il2p_header_t hdr_z_out;
    memset(&hdr_z_out, 0xFF, sizeof(hdr_z_out));
    il2p_header_decode(raw_z, &hdr_z_out);
    TEST_ASSERT(hdr_z_out.payload_byte_count == 0, "header zero payload count round-trip", 0);

    // NULL safety
    TEST_ASSERT(il2p_header_encode(NULL, raw_hdr) == 0, "header encode NULL hdr returns 0", 0);
    TEST_ASSERT(!il2p_header_decode(raw_hdr, NULL), "header decode NULL out returns false", 0);

    // PID mapping round-trip
    {
        struct {
            uint8_t ax25;
            uint8_t il2p;
        } pid_map[] = { { 0xF0, IL2P_PID_NO_L3 }, { 0xCC, IL2P_PID_ARPA_IP }, { 0xCD, IL2P_PID_ARPA_ARP }, { 0x01, IL2P_PID_X25_PLP },
                { 0x06, IL2P_PID_COMP_TCP }, { 0x07, IL2P_PID_UNCOMP_TCP }, { 0x08, IL2P_PID_SEGMENT }, };
        for (size_t i = 0; i < sizeof(pid_map) / sizeof(pid_map[0]); i++) {
            uint8_t il2p = il2p_pid_from_ax25(pid_map[i].ax25, false, false);
            uint8_t ax25 = il2p_pid_to_ax25(il2p);
            printf("  [DBG] PID ax25=0x%02X -> il2p=0x%02X -> ax25=0x%02X\n", pid_map[i].ax25, il2p, ax25);
            TEST_ASSERT(il2p == pid_map[i].il2p, "PID ax25->il2p mapping", il2p);
            TEST_ASSERT(ax25 == pid_map[i].ax25, "PID il2p->ax25 mapping", ax25);
        }
        // S-frame PID
        TEST_ASSERT(il2p_pid_from_ax25(0x00, true, false) == IL2P_PID_S_FRAME, "S-frame PID mapping", 0);
        // U-frame PID
        TEST_ASSERT(il2p_pid_from_ax25(0x00, false, true) == IL2P_PID_U_FRAME, "U-frame PID mapping", 0);
        // Unknown PID
        uint8_t unk = il2p_pid_from_ax25(0x99, false, false);
        printf("  [DBG] Unknown PID 0x99 -> il2p=0x%02X (expect 0xFF)\n", unk);
        TEST_ASSERT(unk == 0xFF, "Unknown PID returns 0xFF", unk);
    }

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 8: Payload block size computation
// -----------------------------------------------------------------------
static int test_payload_blocks(void) {
    printf("\n=== Payload block sizes ===\n");

    struct {
        uint16_t payload;
        uint16_t exp_num;  // ceil(payload / 239)
    } cases[] = { { 1, 1 },   // 1 byte -> 1 block
            { 100, 1 },   // 100 bytes -> 1 block
            { 239, 1 },   // exactly 1 max block
            { 240, 2 },   // ceil(240/239) = 2
            { 241, 2 },   // ceil(241/239) = 2
            { 478, 2 },   // ceil(478/239) = 2 exactly
            { 479, 3 },   // ceil(479/239) = 3
            { 1023, 5 },   // ceil(1023/239) = 5
            };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint16_t num, lc, sc;
        uint8_t ls, ss;
        il2p_payload_block_sizes(cases[i].payload, &num, &ls, &ss, &lc, &sc);
        printf("  [DBG] payload=%4u -> num=%u large(%u x %u) small(%u x %u)\n", cases[i].payload, num, lc, ls, sc, ss);
        // Verify total bytes matches
        uint32_t total = (uint32_t) lc * (uint32_t) ls + (uint32_t) sc * (uint32_t) ss;
        printf("         total bytes: %u (expect %u)\n", total, cases[i].payload);
        TEST_ASSERT(num == cases[i].exp_num, "block count", (int )(num - cases[i].exp_num));
        TEST_ASSERT(total == cases[i].payload, "block sizes sum to payload length", (int )(total - cases[i].payload));
        // large >= small
        if (num > 0) {
            TEST_ASSERT(ls >= ss, "large_block_size >= small_block_size", 0);
        }
    }

    // Zero payload
    uint16_t num;
    uint8_t ls, ss;
    uint16_t lc, sc;
    il2p_payload_block_sizes(0, &num, &ls, &ss, &lc, &sc);
    TEST_ASSERT(num == 0, "zero payload -> 0 blocks", 0);

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 9: IL2P header from AX.25 frame (il2p_header_from_ax25)
// -----------------------------------------------------------------------
static int test_header_from_ax25(void) {
    printf("\n=== il2p_header_from_ax25 ===\n");

    // --- I-frame (Type 1 should be selected) ---
    {
        ax25_information_frame_t iframe;
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
        fill_address(&iframe.base.header.destination, "W1AW  ", 0);
        fill_address(&iframe.base.header.source, "VE3TKI", 1);
        iframe.base.header.repeaters.num_repeaters = 0;
        iframe.pid = 0xF0;  // No Layer 3
        iframe.ns = 3;
        iframe.nr = 5;
        iframe.pf = 0;
        uint8_t payload_data[] = { 0x01, 0x02, 0x03 };
        iframe.payload = payload_data;
        iframe.payload_len = 3;

        il2p_header_t hdr;
        bool ok = il2p_header_from_ax25((const ax25_frame_t*) &iframe, 3, &hdr);
        printf("  [DBG] I-frame from_ax25: ok=%d hdr_type=%u pid=0x%02X ctrl=0x%02X\n", ok, hdr.hdr_type, hdr.pid, hdr.control);
        TEST_ASSERT(ok, "I-frame header_from_ax25 returns true", 0);
        TEST_ASSERT(hdr.hdr_type == IL2P_HDR_TYPE_1_TRANSLATED, "I-frame selects Type 1", hdr.hdr_type);
        TEST_ASSERT(hdr.pid == IL2P_PID_NO_L3, "I-frame PID mapped to NO_L3", hdr.pid);
        // control: P/F=0, NR=5, NS=3 -> 0b0_101_011 = 0x2B
        uint8_t exp_ctrl = (uint8_t) ((0 << 6) | (5 << 3) | 3);
        printf("  [DBG] expected control=0x%02X got=0x%02X\n", exp_ctrl, hdr.control);
        TEST_ASSERT(hdr.control == exp_ctrl, "I-frame control subfield", hdr.control);
        TEST_ASSERT(hdr.payload_byte_count == 3, "I-frame payload_byte_count", 0);
    }

    // --- S-frame RR (Type 1) ---
    {
        ax25_supervisory_frame_t sframe;
        memset(&sframe, 0, sizeof(sframe));
        sframe.base.type = AX25_FRAME_SUPERVISORY_RR_8BIT;
        fill_address(&sframe.base.header.destination, "W1AW  ", 0);
        fill_address(&sframe.base.header.source, "VE3TKI", 0);
        sframe.base.header.repeaters.num_repeaters = 0;
        sframe.nr = 2;
        sframe.pf = 1;
        sframe.code = 0;  // RR

        il2p_header_t hdr;
        bool ok = il2p_header_from_ax25((const ax25_frame_t*) &sframe, 0, &hdr);
        printf("  [DBG] S-frame RR from_ax25: ok=%d hdr_type=%u pid=0x%02X ctrl=0x%02X\n", ok, hdr.hdr_type, hdr.pid, hdr.control);
        TEST_ASSERT(ok, "S-frame header_from_ax25 returns true", 0);
        TEST_ASSERT(hdr.hdr_type == IL2P_HDR_TYPE_1_TRANSLATED, "S-frame selects Type 1", hdr.hdr_type);
        TEST_ASSERT(hdr.pid == IL2P_PID_S_FRAME, "S-frame PID", hdr.pid);
        // control: NR=2, C=1 (pf), OPCODE=0 (RR) -> 0b010_1_00 = bits: (2<<3)|(1<<2)|0 = 0x14
        uint8_t exp_ctrl = (uint8_t) ((2 << 3) | (1 << 2) | 0);
        printf("  [DBG] expected control=0x%02X got=0x%02X\n", exp_ctrl, hdr.control);
        TEST_ASSERT(hdr.control == exp_ctrl, "S-frame RR control subfield", hdr.control);
    }

    // --- U-frame SABM (Type 1) ---
    {
        ax25_unnumbered_frame_t uframe;
        memset(&uframe, 0, sizeof(uframe));
        uframe.base.type = AX25_FRAME_UNNUMBERED_SABM;
        fill_address(&uframe.base.header.destination, "W1AW  ", 0);
        fill_address(&uframe.base.header.source, "VE3TKI", 0);
        uframe.base.header.repeaters.num_repeaters = 0;
        uframe.base.header.cr = 1;
        uframe.pf = 1;

        il2p_header_t hdr;
        bool ok = il2p_header_from_ax25((const ax25_frame_t*) &uframe, 0, &hdr);
        printf("  [DBG] SABM from_ax25: ok=%d hdr_type=%u pid=0x%02X ctrl=0x%02X\n", ok, hdr.hdr_type, hdr.pid, hdr.control);
        TEST_ASSERT(ok, "SABM header_from_ax25 returns true", 0);
        TEST_ASSERT(hdr.hdr_type == IL2P_HDR_TYPE_1_TRANSLATED, "SABM selects Type 1", hdr.hdr_type);
        TEST_ASSERT(hdr.pid == IL2P_PID_U_FRAME, "SABM PID is U_FRAME", hdr.pid);
    }

    // --- SABME forces Type 0 ---
    {
        ax25_unnumbered_frame_t uframe;
        memset(&uframe, 0, sizeof(uframe));
        uframe.base.type = AX25_FRAME_UNNUMBERED_SABME;
        fill_address(&uframe.base.header.destination, "W1AW  ", 0);
        fill_address(&uframe.base.header.source, "VE3TKI", 0);
        uframe.base.header.repeaters.num_repeaters = 0;

        il2p_header_t hdr;
        bool ok = il2p_header_from_ax25((const ax25_frame_t*) &uframe, 0, &hdr);
        printf("  [DBG] SABME from_ax25: ok=%d hdr_type=%u (expect Type 0)\n", ok, hdr.hdr_type);
        TEST_ASSERT(ok, "SABME returns true", 0);
        TEST_ASSERT(hdr.hdr_type == IL2P_HDR_TYPE_0_TRANSPARENT, "SABME forces Type 0", hdr.hdr_type);
    }

    // --- Frame with repeaters forces Type 0 ---
    {
        ax25_information_frame_t iframe;
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
        fill_address(&iframe.base.header.destination, "W1AW  ", 0);
        fill_address(&iframe.base.header.source, "VE3TKI", 0);
        iframe.base.header.repeaters.num_repeaters = 1;

        // ax25_path_t uses 'repeaters[]' array (not 'addr[]') for the repeater addresses
        fill_address(&iframe.base.header.repeaters.repeaters[0], "KB1ABC", 0);
        iframe.pid = 0xF0;

        il2p_header_t hdr;
        bool ok = il2p_header_from_ax25((const ax25_frame_t*) &iframe, 10, &hdr);
        printf("  [DBG] repeater frame from_ax25: ok=%d hdr_type=%u (expect Type 0)\n", ok, hdr.hdr_type);
        TEST_ASSERT(ok, "repeater frame returns true", 0);
        TEST_ASSERT(hdr.hdr_type == IL2P_HDR_TYPE_0_TRANSPARENT, "repeater frame forces Type 0", hdr.hdr_type);
    }

    // --- UI frame ---
    {
        ax25_unnumbered_information_frame_t uiframe;
        memset(&uiframe, 0, sizeof(uiframe));
        // ax25_unnumbered_information_frame_t.base is ax25_unnumbered_frame_t (not ax25_frame_t).
        // ax25_frame_t fields (type, header) are in .base.base, not .base.
        uiframe.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        fill_address(&uiframe.base.base.header.destination, "W1AW  ", 0);
        fill_address(&uiframe.base.base.header.source, "VE3TKI", 0);
        uiframe.base.base.header.repeaters.num_repeaters = 0;
        uiframe.pid = 0xF0;
        uint8_t ui_data[] = { 0xAA, 0xBB };
        uiframe.payload = ui_data;
        uiframe.payload_len = 2;

        il2p_header_t hdr;
        bool ok = il2p_header_from_ax25((const ax25_frame_t*) &uiframe, 2, &hdr);
        printf("  [DBG] UI frame from_ax25: ok=%d hdr_type=%u ui=%u pid=0x%02X\n", ok, hdr.hdr_type, hdr.ui, hdr.pid);
        TEST_ASSERT(ok, "UI frame header_from_ax25 returns true", 0);
        TEST_ASSERT(hdr.hdr_type == IL2P_HDR_TYPE_1_TRANSLATED, "UI frame selects Type 1", hdr.hdr_type);
        TEST_ASSERT(hdr.ui == 1, "UI frame sets ui=1", hdr.ui);
    }

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 10: Full IL2P encode / decode round-trip (Type 0 + Type 1)
// -----------------------------------------------------------------------
static int test_il2p_roundtrip(void) {
    printf("\n=== IL2P encode/decode round-trip ===\n");

    uint8_t il2p_buf[IL2P_MAX_FRAME_BYTES];
    uint8_t ax25_recovered[512];
    size_t il2p_len, ax25_len;

    // --- Type 1: I-frame with payload ---
    {
        printf("  -- Type 1 I-frame --\n");
        ax25_information_frame_t iframe;
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
        fill_address(&iframe.base.header.destination, "W1AW  ", 0);
        fill_address(&iframe.base.header.source, "VE3TKI", 1);
        iframe.base.header.repeaters.num_repeaters = 0;
        iframe.pid = 0xF0;
        iframe.ns = 1;
        iframe.nr = 2;
        iframe.pf = 0;

        // Build a raw AX.25 I-frame byte sequence for the payload argument
        // AX.25 raw: [DEST 7B][SRC 7B][CTRL 1B][PID 1B][INFO...]
        uint8_t raw[64];
        size_t p = 0;
        // Destination (space-padded, shifted left 1)
        const char *dc = "W1AW  ";
        for (int i = 0; i < 6; i++)
            raw[p++] = (uint8_t) ((dc[i] & 0x7F) << 1);
        raw[p++] = (uint8_t) ((0 & 0x0F) << 1);  // dest SSID
        const char *sc2 = "VE3TKI";
        for (int i = 0; i < 6; i++)
            raw[p++] = (uint8_t) ((sc2[i] & 0x7F) << 1);
        raw[p++] = (uint8_t) (((1 & 0x0F) << 1) | 0x01);  // src SSID, last addr bit
        raw[p++] = (uint8_t) ((1 << 1) | (0 << 4) | (2 << 5));  // control
        raw[p++] = 0xF0;  // PID
        uint8_t info[] = "Hello IL2P!";
        memcpy(&raw[p], info, sizeof(info));
        p += sizeof(info);
        size_t raw_len = p;

        iframe.payload = &raw[16];  // point at info field
        iframe.payload_len = sizeof(info);

        print_buf("AX.25 raw frame", raw, raw_len);

        bool enc_ok = il2p_encode((const ax25_frame_t*) &iframe, raw, raw_len, il2p_buf, sizeof(il2p_buf), &il2p_len);
        printf("  [DBG] il2p_encode: ok=%d len=%zu\n", enc_ok, il2p_len);
        print_buf("IL2P encoded", il2p_buf, il2p_len);
        TEST_ASSERT(enc_ok, "Type 1 I-frame encode", 0);
        TEST_ASSERT(il2p_len >= 3u + IL2P_HEADER_RS_SIZE, "IL2P output minimum length", 0);

        // Verify sync word at start
        TEST_ASSERT(il2p_buf[0] == 0xF1 && il2p_buf[1] == 0x5E && il2p_buf[2] == 0x48, "IL2P sync word present at output start", 0);

        bool dec_ok = il2p_decode(il2p_buf, il2p_len, ax25_recovered, sizeof(ax25_recovered), &ax25_len);
        printf("  [DBG] il2p_decode: ok=%d ax25_len=%zu\n", dec_ok, ax25_len);
        print_buf("Recovered AX.25", ax25_recovered, ax25_len > 0 ? ax25_len : 0);
        TEST_ASSERT(dec_ok, "Type 1 I-frame decode", 0);
        // The recovered AX.25 frame must contain the info field bytes
        // (Type 1 decode reconstructs the header + payload)
        TEST_ASSERT(ax25_len > 0, "Recovered AX.25 non-empty", 0);

        // Search for "Hello IL2P!" in the recovered bytes
        bool found_info = false;
        for (size_t i = 0; i + sizeof(info) <= ax25_len; i++) {
            if (memcmp(&ax25_recovered[i], info, sizeof(info)) == 0) {
                found_info = true;
                printf("  [DBG] Info field found at offset %zu in recovered frame\n", i);
                break;
            }
        }
        TEST_ASSERT(found_info, "Info field survives Type 1 round-trip", 0);
    }

    // --- Type 0: frame with repeater (transparent encapsulation) ---
    {
        printf("  -- Type 0 transparent frame (with repeater) --\n");
        ax25_information_frame_t iframe;
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
        fill_address(&iframe.base.header.destination, "W1AW  ", 0);
        fill_address(&iframe.base.header.source, "VE3TKI", 0);
        iframe.base.header.repeaters.num_repeaters = 1;
        // ax25_path_t uses 'repeaters[]' array (not 'addr[]') for the repeater addresses
        fill_address(&iframe.base.header.repeaters.repeaters[0], "KB1EL ", 0);
        iframe.pid = 0xF0;
        iframe.ns = 0;
        iframe.nr = 0;
        iframe.pf = 0;

        // Build full raw AX.25 frame with repeater
        uint8_t raw[128];
        size_t p = 0;
        const char *dc = "W1AW  ";
        for (int i = 0; i < 6; i++)
            raw[p++] = (uint8_t) ((dc[i] & 0x7F) << 1);
        raw[p++] = (uint8_t) ((0 & 0x0F) << 1);
        const char *sc2 = "VE3TKI";
        for (int i = 0; i < 6; i++)
            raw[p++] = (uint8_t) ((sc2[i] & 0x7F) << 1);
        raw[p++] = (uint8_t) ((0 & 0x0F) << 1);  // no last-addr yet (repeater follows)
        const char *rc = "KB1EL ";
        for (int i = 0; i < 6; i++)
            raw[p++] = (uint8_t) ((rc[i] & 0x7F) << 1);
        raw[p++] = (uint8_t) (((0 & 0x0F) << 1) | 0x01);  // last addr bit
        raw[p++] = 0x00;  // control
        raw[p++] = 0xF0;  // PID
        uint8_t info[] = "Type0 Test";
        memcpy(&raw[p], info, sizeof(info));
        p += sizeof(info);
        size_t raw_len = p;

        iframe.payload = &raw[p - sizeof(info)];
        iframe.payload_len = sizeof(info);

        print_buf("AX.25 raw (Type 0)", raw, raw_len);

        bool enc_ok = il2p_encode((const ax25_frame_t*) &iframe, raw, raw_len, il2p_buf, sizeof(il2p_buf), &il2p_len);
        printf("  [DBG] Type 0 encode: ok=%d len=%zu\n", enc_ok, il2p_len);
        TEST_ASSERT(enc_ok, "Type 0 encode", 0);

        bool dec_ok = il2p_decode(il2p_buf, il2p_len, ax25_recovered, sizeof(ax25_recovered), &ax25_len);
        printf("  [DBG] Type 0 decode: ok=%d ax25_len=%zu (expected %zu)\n", dec_ok, ax25_len, raw_len);
        TEST_ASSERT(dec_ok, "Type 0 decode", 0);
        TEST_ASSERT(ax25_len == raw_len, "Type 0 recovered length matches original", 0);
        COMPARE_FRAME(ax25_recovered, ax25_len, raw, raw_len, "Type 0 exact AX.25 recovery");
    }

    // --- Type 1: S-frame (no payload) ---
    {
        printf("  -- Type 1 S-frame RNR --\n");
        ax25_supervisory_frame_t sframe;
        memset(&sframe, 0, sizeof(sframe));
        sframe.base.type = AX25_FRAME_SUPERVISORY_RNR_8BIT;
        fill_address(&sframe.base.header.destination, "N0CALL", 0);
        fill_address(&sframe.base.header.source, "W6XYZ ", 0);
        sframe.base.header.repeaters.num_repeaters = 0;
        sframe.nr = 4;
        sframe.pf = 0;
        sframe.code = 1;  // RNR

        uint8_t raw[32];
        size_t p = 0;
        const char *dc = "N0CALL";
        for (int i = 0; i < 6; i++)
            raw[p++] = (uint8_t) ((dc[i] & 0x7F) << 1);
        raw[p++] = (uint8_t) ((0 & 0x0F) << 1);
        const char *sc2 = "W6XYZ ";
        for (int i = 0; i < 6; i++)
            raw[p++] = (uint8_t) ((sc2[i] & 0x7F) << 1);
        raw[p++] = (uint8_t) (((0 & 0x0F) << 1) | 0x01);
        raw[p++] = (uint8_t) (0x01 | (1 << 2) | (4 << 5));  // S-frame ctrl: RNR NR=4
        size_t raw_len = p;

        bool enc_ok = il2p_encode((const ax25_frame_t*) &sframe, raw, raw_len, il2p_buf, sizeof(il2p_buf), &il2p_len);
        printf("  [DBG] S-frame encode: ok=%d len=%zu\n", enc_ok, il2p_len);
        TEST_ASSERT(enc_ok, "S-frame encode", 0);

        bool dec_ok = il2p_decode(il2p_buf, il2p_len, ax25_recovered, sizeof(ax25_recovered), &ax25_len);
        printf("  [DBG] S-frame decode: ok=%d ax25_len=%zu\n", dec_ok, ax25_len);
        TEST_ASSERT(dec_ok, "S-frame decode", 0);
        TEST_ASSERT(ax25_len > 0, "S-frame recovered non-empty", 0);
    }

    // --- Type 1: UI frame ---
    {
        printf("  -- Type 1 UI frame --\n");
        ax25_unnumbered_information_frame_t uiframe;
        memset(&uiframe, 0, sizeof(uiframe));
        // ax25_unnumbered_information_frame_t.base is ax25_unnumbered_frame_t (not ax25_frame_t).
        // ax25_frame_t fields (type, header) are in .base.base, not .base.
        uiframe.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        fill_address(&uiframe.base.base.header.destination, "APRS  ", 0);
        fill_address(&uiframe.base.base.header.source, "KG7BRD", 1);
        uiframe.base.base.header.repeaters.num_repeaters = 0;
        uiframe.pid = 0xF0;

        uint8_t ui_data[] = "!1234.56N/01234.56E#APRS comment";
        uiframe.payload = ui_data;
        uiframe.payload_len = sizeof(ui_data) - 1;

        uint8_t raw[128];
        size_t p = 0;
        const char *dc = "APRS  ";
        for (int i = 0; i < 6; i++)
            raw[p++] = (uint8_t) ((dc[i] & 0x7F) << 1);
        raw[p++] = (uint8_t) ((0 & 0x0F) << 1);
        const char *sc2 = "KG7BRD";
        for (int i = 0; i < 6; i++)
            raw[p++] = (uint8_t) ((sc2[i] & 0x7F) << 1);
        raw[p++] = (uint8_t) (((1 & 0x0F) << 1) | 0x01);
        raw[p++] = 0x03;  // UI control
        raw[p++] = 0xF0;  // No Layer 3
        memcpy(&raw[p], ui_data, uiframe.payload_len);
        p += uiframe.payload_len;
        size_t raw_len = p;

        bool enc_ok = il2p_encode((const ax25_frame_t*) &uiframe, raw, raw_len, il2p_buf, sizeof(il2p_buf), &il2p_len);
        printf("  [DBG] UI encode: ok=%d len=%zu\n", enc_ok, il2p_len);
        TEST_ASSERT(enc_ok, "UI frame encode", 0);

        bool dec_ok = il2p_decode(il2p_buf, il2p_len, ax25_recovered, sizeof(ax25_recovered), &ax25_len);
        printf("  [DBG] UI decode: ok=%d ax25_len=%zu\n", dec_ok, ax25_len);
        TEST_ASSERT(dec_ok, "UI frame decode", 0);
    }

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 11: IL2P error correction in full pipeline
// -----------------------------------------------------------------------
static int test_il2p_error_correction(void) {
    printf("\n=== IL2P error correction in pipeline ===\n");

    // Build a simple Type 0 frame and introduce errors into the IL2P stream
    ax25_information_frame_t iframe;
    memset(&iframe, 0, sizeof(iframe));
    iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
    fill_address(&iframe.base.header.destination, "W1AW  ", 0);
    fill_address(&iframe.base.header.source, "VE3TKI", 0);
    iframe.base.header.repeaters.num_repeaters = 1;  // Force Type 0
    // ax25_path_t uses 'repeaters[]' array (not 'addr[]') for the repeater addresses
    fill_address(&iframe.base.header.repeaters.repeaters[0], "KB1EL ", 0);
    iframe.pid = 0xF0;
    iframe.ns = 0;
    iframe.nr = 0;
    iframe.pf = 0;

    uint8_t raw[64];
    size_t p = 0;
    const char *dc = "W1AW  ";
    for (int i = 0; i < 6; i++)
        raw[p++] = (uint8_t) ((dc[i] & 0x7F) << 1);
    raw[p++] = 0x00;
    const char *sc2 = "VE3TKI";
    for (int i = 0; i < 6; i++)
        raw[p++] = (uint8_t) ((sc2[i] & 0x7F) << 1);
    raw[p++] = 0x00;
    const char *rc = "KB1EL ";
    for (int i = 0; i < 6; i++)
        raw[p++] = (uint8_t) ((rc[i] & 0x7F) << 1);
    raw[p++] = 0x01;
    raw[p++] = 0x00;
    raw[p++] = 0xF0;
    uint8_t info[] = "ErrorTest";
    memcpy(&raw[p], info, sizeof(info));
    p += sizeof(info);
    size_t raw_len = p;
    iframe.payload = &raw[p - sizeof(info)];
    iframe.payload_len = sizeof(info);

    uint8_t il2p_buf[IL2P_MAX_FRAME_BYTES];
    size_t il2p_len;
    bool enc_ok = il2p_encode((const ax25_frame_t*) &iframe, raw, raw_len, il2p_buf, sizeof(il2p_buf), &il2p_len);
    TEST_ASSERT(enc_ok, "encode for error test", 0);
    printf("  [DBG] encoded IL2P length: %zu\n", il2p_len);

    // Corrupt 1 byte in the payload RS block area (after sync + header)
    // Payload block starts at byte 3 + IL2P_HEADER_RS_SIZE = 3+15 = 18
    uint8_t il2p_corrupt[IL2P_MAX_FRAME_BYTES];
    memcpy(il2p_corrupt, il2p_buf, il2p_len);
    size_t payload_start = 3 + IL2P_HEADER_RS_SIZE;
    if (payload_start < il2p_len) {
        il2p_corrupt[payload_start] ^= 0xFF;
        printf("  [DBG] Corrupted byte at offset %zu: 0x%02X -> 0x%02X\n", payload_start, il2p_buf[payload_start], il2p_corrupt[payload_start]);
    }

    uint8_t ax25_out[512];
    size_t ax25_len;
    bool dec_ok = il2p_decode(il2p_corrupt, il2p_len, ax25_out, sizeof(ax25_out), &ax25_len);
    printf("  [DBG] decode with 1 payload error: ok=%d len=%zu\n", dec_ok, ax25_len);
    TEST_ASSERT(dec_ok, "decode survives 1 payload byte error (RS correction)", 0);
    TEST_ASSERT(ax25_len == raw_len, "recovered length correct after 1 error", 0);
    COMPARE_FRAME(ax25_out, ax25_len, raw, raw_len, "data intact after 1 byte error correction");

    // Corrupt 1 byte in the HEADER RS block area (after sync)
    memcpy(il2p_corrupt, il2p_buf, il2p_len);
    il2p_corrupt[3] ^= 0xFF;  // First header byte
    printf("  [DBG] Corrupted header byte at offset 3: 0x%02X -> 0x%02X\n", il2p_buf[3], il2p_corrupt[3]);
    dec_ok = il2p_decode(il2p_corrupt, il2p_len, ax25_out, sizeof(ax25_out), &ax25_len);
    printf("  [DBG] decode with 1 header error: ok=%d len=%zu\n", dec_ok, ax25_len);
    TEST_ASSERT(dec_ok, "decode survives 1 header byte error (RS correction)", 0);
    COMPARE_FRAME(ax25_out, ax25_len, raw, raw_len, "data intact after 1 header error");

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 12: Large payload (multiple RS blocks)
// -----------------------------------------------------------------------
static int test_il2p_large_payload(void) {
    printf("\n=== IL2P large payload (multi-block) ===\n");

    // Use Type 0 (repeater present) to keep things simple
    ax25_information_frame_t iframe;
    memset(&iframe, 0, sizeof(iframe));
    iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
    fill_address(&iframe.base.header.destination, "W1AW  ", 0);
    fill_address(&iframe.base.header.source, "VE3TKI", 0);
    iframe.base.header.repeaters.num_repeaters = 1;
    // ax25_path_t uses 'repeaters[]' array (not 'addr[]') for the repeater addresses
    fill_address(&iframe.base.header.repeaters.repeaters[0], "KB1EL ", 0);

    // Build a large raw frame (600 bytes of data)
    static uint8_t large_raw[700];
    size_t p = 0;
    const char *dc = "W1AW  ";
    for (int i = 0; i < 6; i++)
        large_raw[p++] = (uint8_t) ((dc[i] & 0x7F) << 1);
    large_raw[p++] = 0x00;
    const char *sc2 = "VE3TKI";
    for (int i = 0; i < 6; i++)
        large_raw[p++] = (uint8_t) ((sc2[i] & 0x7F) << 1);
    large_raw[p++] = 0x00;
    const char *rc = "KB1EL ";
    for (int i = 0; i < 6; i++)
        large_raw[p++] = (uint8_t) ((rc[i] & 0x7F) << 1);
    large_raw[p++] = 0x01;
    large_raw[p++] = 0x00;
    large_raw[p++] = 0xF0;
    // Fill info field with a recognisable pattern
    for (int i = 0; i < 600; i++)
        large_raw[p++] = (uint8_t) (i & 0xFF);
    size_t raw_len = p;
    iframe.payload = &large_raw[23];
    iframe.payload_len = 600;

    printf("  [DBG] Large raw frame: %zu bytes\n", raw_len);

    uint8_t il2p_buf[IL2P_MAX_FRAME_BYTES];
    size_t il2p_len;
    bool enc_ok = il2p_encode((const ax25_frame_t*) &iframe, large_raw, raw_len, il2p_buf, sizeof(il2p_buf), &il2p_len);
    printf("  [DBG] Large payload encode: ok=%d il2p_len=%zu\n", enc_ok, il2p_len);
    TEST_ASSERT(enc_ok, "large payload encode", 0);

    uint8_t ax25_out[1024];
    size_t ax25_len;
    bool dec_ok = il2p_decode(il2p_buf, il2p_len, ax25_out, sizeof(ax25_out), &ax25_len);
    printf("  [DBG] Large payload decode: ok=%d ax25_len=%zu (expected %zu)\n", dec_ok, ax25_len, raw_len);
    TEST_ASSERT(dec_ok, "large payload decode", 0);
    TEST_ASSERT(ax25_len == raw_len, "large payload recovered length", 0);
    COMPARE_FRAME(ax25_out, ax25_len, large_raw, raw_len, "large payload exact recovery");

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 13: Edge cases and NULL safety
// -----------------------------------------------------------------------
static int test_edge_cases(void) {
    printf("\n=== Edge cases / NULL safety ===\n");

    // il2p_encode NULL args
    // Initialize dummy to suppress -Wmaybe-uninitialized: dummy is passed as both
    // input and output buffer in NULL-safety checks; uninitialized content is irrelevant
    // but the compiler cannot prove it won't be read before the early-return guard fires.
    uint8_t dummy[8];
    memset(dummy, 0, sizeof(dummy));
    size_t len;
    ax25_information_frame_t iframe;
    memset(&iframe, 0, sizeof(iframe));

    TEST_ASSERT(!il2p_encode(NULL, dummy, 1, dummy, sizeof(dummy), &len), "il2p_encode NULL frame", 0);
    TEST_ASSERT(!il2p_encode((ax25_frame_t*)&iframe, NULL, 1, dummy, sizeof(dummy), &len), "il2p_encode NULL raw", 0);
    TEST_ASSERT(!il2p_encode((ax25_frame_t*)&iframe, dummy, 1, NULL, sizeof(dummy), &len), "il2p_encode NULL out", 0);
    TEST_ASSERT(!il2p_encode((ax25_frame_t*)&iframe, dummy, 1, dummy, sizeof(dummy), NULL), "il2p_encode NULL out_len", 0);

    // il2p_decode NULL args
    TEST_ASSERT(!il2p_decode(NULL, 8, dummy, sizeof(dummy), &len), "il2p_decode NULL in", 0);
    TEST_ASSERT(!il2p_decode(dummy, 8, NULL, sizeof(dummy), &len), "il2p_decode NULL out", 0);
    TEST_ASSERT(!il2p_decode(dummy, 8, dummy, sizeof(dummy), NULL), "il2p_decode NULL ax25_len", 0);

    // il2p_decode on random data (should fail gracefully without crash)
    uint8_t noise[64];
    for (size_t i = 0; i < 64; i++)
        noise[i] = (uint8_t) (i * 113 + 7);
    uint8_t ax25_out[512];
    size_t ax25_len;
    bool dec = il2p_decode(noise, 64, ax25_out, sizeof(ax25_out), &ax25_len);
    printf("  [DBG] decode random noise: %s\n", dec ? "decoded (sync false match)" : "failed (expected)");
    // We do not assert decode==false since a 1-bit tolerant sync search might
    // produce a false match, but it should not crash and if it does decode,
    // ax25_len must be reasonable
    if (dec) {
        TEST_ASSERT(ax25_len <= sizeof(ax25_out), "random noise decode length sane", 0);
    }

    // il2p_encode with raw_len > IL2P_MAX_PAYLOAD_BYTES
    uint8_t big[IL2P_MAX_PAYLOAD_BYTES + 2];
    memset(big, 0, sizeof(big));
    TEST_ASSERT(!il2p_encode((ax25_frame_t*)&iframe, big, IL2P_MAX_PAYLOAD_BYTES + 1, dummy, sizeof(dummy), &len), "il2p_encode rejects oversized payload", 0);

    // il2p_rs_encode NULL block
    TEST_ASSERT(!il2p_rs_encode(NULL, 10, 2), "rs_encode NULL block", 0);
    // il2p_rs_decode NULL block
    TEST_ASSERT(il2p_rs_decode(NULL, 10, 2) == -1, "rs_decode NULL block", 0);

    // il2p_sixbit NULL args
    TEST_ASSERT(!il2p_sixbit_encode_callsign(NULL, NULL), "sixbit_encode NULL/NULL", 0);

    // il2p_header_encode NULL
    uint8_t raw_hdr[IL2P_HEADER_SIZE];
    TEST_ASSERT(il2p_header_encode(NULL, raw_hdr) == 0, "header_encode NULL hdr", 0);

    return 0;
}

// -----------------------------------------------------------------------
// SECTION 14: Scramble + RS pipeline integrity
// -----------------------------------------------------------------------
static int test_scramble_rs_pipeline(void) {
    printf("\n=== Scramble + RS pipeline ===\n");

    // Build a block, scramble, RS encode, introduce errors, RS decode, descramble
    uint8_t data_len = 30;
    uint8_t orig[30];
    for (uint8_t i = 0; i < data_len; i++)
        orig[i] = (uint8_t) (i * 11 + 0x37);

    uint8_t block[30 + IL2P_RS_PAY_PARITY];
    memcpy(block, orig, data_len);

    il2p_lfsr_t lfsr;

    // Scramble
    il2p_lfsr_reset(&lfsr);
    for (uint8_t i = 0; i < data_len; i++) {
        block[i] = il2p_lfsr_step_byte(&lfsr, block[i]);
    }
    print_buf("After scramble", block, data_len);

    // RS encode
    bool enc = il2p_rs_encode(block, data_len, IL2P_RS_PAY_PARITY);
    TEST_ASSERT(enc, "scramble+RS encode", 0);
    print_buf("After RS encode (data+parity)", block, data_len + IL2P_RS_PAY_PARITY);

    // Introduce 3 errors
    block[5] ^= 0xAA;
    block[12] ^= 0x55;
    block[20] ^= 0xFF;
    printf("  [DBG] Injected 3 errors at bytes 5, 12, 20\n");

    // RS decode
    int8_t corr = il2p_rs_decode(block, data_len, IL2P_RS_PAY_PARITY);
    printf("  [DBG] RS decode corrections: %d\n", corr);
    TEST_ASSERT(corr == 3, "scramble+RS corrects 3 errors", corr);

    // Descramble
    il2p_lfsr_reset(&lfsr);
    for (uint8_t i = 0; i < data_len; i++) {
        block[i] = il2p_lfsr_step_byte(&lfsr, block[i]);
    }
    print_buf("After descramble", block, data_len);

    TEST_ASSERT(memcmp(block, orig, data_len) == 0, "data recovered through scramble+RS pipeline", 0);

    return 0;
}

// -----------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------
int test_il2p_main(void) {
    int failures = 0;

    printf("\n----------------------------------------------------------------------------------\n");
    printf("Starting IL2P Tests\n");
    printf("----------------------------------------------------------------------------------\n\n");

    failures += test_gf256();
    failures += test_crc();
    failures += test_sixbit();
    failures += test_rs();
    failures += test_lfsr();
    failures += test_sync();
    failures += test_header();
    failures += test_payload_blocks();
    failures += test_header_from_ax25();
    failures += test_il2p_roundtrip();
    failures += test_il2p_error_correction();
    failures += test_il2p_large_payload();
    failures += test_edge_cases();
    failures += test_scramble_rs_pipeline();

    printf("\n----------------------------------------------------------------------------------\n");
    printf("IL2P Tests Completed. %s\n", failures == 0 ? "All tests passed" : "Some tests failed");
    printf("----------------------------------------------------------------------------------\n\n");

    return failures;
}
