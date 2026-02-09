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

#ifndef FX25_H_
#define FX25_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>

// FX.25 v01.06 - Forward Error Correction for AX.25
// Uses Reed-Solomon RS(255,255-2T) where T = correctable bytes

// Correlation tags - 64-bit identifiers for FX.25 modes
typedef struct {
    uint8_t tag_id;              //
    uint8_t correlation_tag[8];  // 8-byte correlation sequence
    uint8_t data_bytes;          // D: AX.25 frame size
    uint8_t parity_bytes;        // P: RS parity bytes
    uint8_t correctable_bytes;   // T = P/2
} fx25_mode_t;

// Predefined FX.25 modes per specification
#define FX25_MODE_239_16  0x01  // 239 data, 16 parity, correct 8
#define FX25_MODE_128_16  0x02  // 128 data, 16 parity, correct 8
#define FX25_MODE_64_16   0x03  // 64 data, 16 parity, correct 8
#define FX25_MODE_32_16   0x04  // 32 data, 16 parity, correct 8
#define FX25_MODE_223_32  0x05  // 223 data, 32 parity, correct 16
#define FX25_MODE_128_32  0x06  // 128 data, 32 parity, correct 16
#define FX25_MODE_64_32   0x07  // 64 data, 32 parity, correct 16
#define FX25_MODE_32_32   0x08  // 32 data, 32 parity, correct 16
#define FX25_MODE_191_64  0x09  // 191 data, 64 parity, correct 32
#define FX25_MODE_128_64  0x0A  // 128 data, 64 parity, correct 32
#define FX25_MODE_64_64   0x0B  // 64 data, 64 parity, correct 32

// Galois Field GF(2^8) operations - using lookup tables, no 64-bit
extern const uint8_t fx25_gf_exp[512];
extern const uint8_t fx25_gf_log[256];

// GF multiplication using log tables: 32-bit intermediate only
#define GF_MUL(a, b) ((a) == 0 || (b) == 0 ? 0 : fx25_gf_exp[fx25_gf_log[a] + fx25_gf_log[b]])
#define GF_DIV(a, b) ((a) == 0 ? 0 : fx25_gf_exp[fx25_gf_log[a] - fx25_gf_log[b] + 255])
#define GF_POW(a, n) (fx25_gf_exp[(fx25_gf_log[a] * (n)) % 255])

// FX.25 frame structure
typedef struct {
    uint8_t correlation_tag[8];  // 8 bytes
    uint8_t *rs_codeword;       // D + P bytes (AX.25 frame + parity)
    size_t codeword_len;        // D + P (max 255)
    uint8_t mode_id;            // Selected mode
} fx25_frame_t;

// API
uint8_t fx25_encode(const uint8_t *ax25_frame, size_t ax25_len, uint8_t mode_id, fx25_frame_t *fx25_frame);
uint8_t fx25_decode(const uint8_t *rx_data, size_t rx_len, fx25_frame_t *fx25_frame, uint8_t *corrected_errors);
void fx25_frame_free(fx25_frame_t *frame);
const fx25_mode_t* fx25_get_mode(uint8_t mode_id);
uint8_t fx25_select_mode(size_t ax25_len);  // Auto-select best mode

// Select optimal FX.25 mode based on AX.25 frame length and channel quality
// ax25_len: Length of AX.25 frame to be encoded
// channel_quality: 0-100 (0=worst, 100=perfect channel)
// Returns: mode_id suitable for fx25_encode()
uint8_t fx25_select_mode_for_conditions(size_t ax25_len, uint8_t channel_quality);

#endif /* FX25_H_ */
