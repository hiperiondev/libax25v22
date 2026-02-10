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

#ifndef FX25_RS_H_
#define FX25_RS_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Galois Field tables and multiplication macro for RS operations
// GF(2^8) tables defined in fx25.c
extern const uint8_t fx25_gf_exp[512];
extern const uint8_t fx25_gf_log[256];

// Galois Field multiplication macro
// Returns 0 if either operand is 0, otherwise alpha^(log(a) + log(b))
#define GF_MUL(a, b) ((a) == 0 || (b) == 0 ? 0 : fx25_gf_exp[fx25_gf_log[a] + fx25_gf_log[b]])

// Reed-Solomon code parameters for FX.25
typedef struct {
    uint8_t n;          // Total codeword length (symbols)
    uint8_t k;          // Data symbols
    uint8_t t;          // Error correction capability (symbols)
    uint8_t nroots;     // Number of parity symbols (2*t)
    uint8_t fcr;        // First consecutive root (typically 1)
    uint8_t prim;       // Primitive element (typically 1 for alpha)
} rs_params_t;

// FX.25 correlation tag identifiers
typedef enum {
    FX25_TAG_01 = 0x01,  // RS(255,239) - 16 check bytes
    FX25_TAG_02 = 0x02,  // RS(144,128) - 16 check bytes
    FX25_TAG_03 = 0x03,  // RS(80,64) - 16 check bytes
    FX25_TAG_04 = 0x04,  // RS(48,32) - 16 check bytes
    FX25_TAG_05 = 0x05,  // RS(255,223) - 32 check bytes
    FX25_TAG_06 = 0x06,  // RS(160,128) - 32 check bytes
    FX25_TAG_07 = 0x07,  // RS(96,64) - 32 check bytes
    FX25_TAG_08 = 0x08,  // RS(64,32) - 32 check bytes
    FX25_TAG_09 = 0x09,  // RS(255,191) - 64 check bytes
    FX25_TAG_0A = 0x0A,  // RS(192,128) - 64 check bytes
    FX25_TAG_0B = 0x0B,  // RS(128,64) - 64 check bytes
} fx25_tag_id_t;

// Get RS parameters for a given FX.25 tag
bool fx25_get_rs_params(fx25_tag_id_t tag, rs_params_t *params);

void rs_init_params(rs_params_t *params, uint8_t parity_bytes);

// Reed-Solomon encoding: generate parity symbols
// Input: data[k] - information symbols
// Output: parity[nroots] - parity symbols to append
// Note: This is a systematic code - data is not modified
void rs_encode(const rs_params_t *params, const uint8_t *data, uint8_t *parity);

// Reed-Solomon decoding: detect and correct errors
// Input/Output: codeword[n] - received codeword (data + parity)
// Returns: number of corrected errors, or -1 if uncorrectable
int rs_decode(const rs_params_t *params, uint8_t *codeword);

// Generate generator polynomial for RS code
// Output: gen_poly[nroots+1] - generator polynomial coefficients
void rs_generate_genpoly(const rs_params_t *params, uint8_t *gen_poly);

#endif /* FX25_RS_H_ */
