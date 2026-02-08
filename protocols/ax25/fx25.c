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

#include <stdlib.h>
#include <string.h>
#include <stdlib.h>

#include "fx25.h"

typedef struct {
    uint8_t coeff[64];   // Max parity bytes
    uint8_t length;
} rs_poly_t;

// GF(2^8) with primitive polynomial x^8 + x^4 + x^3 + x^2 + 1 (0x11D)
const uint8_t fx25_gf_exp[512] = {
// Precomputed exponent table: exp[i] = alpha^i
        0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1d, 0x3a, 0x74, 0xe8, 0xcd, 0x87, 0x13, 0x26, 0x4c, 0x98, 0x2d, 0x5a, 0xb4, 0x75, 0xea, 0xc9, 0x8f,
        0x03, 0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x9d, 0x27, 0x4e, 0x9c, 0x25, 0x4a, 0x94, 0x35, 0x6a, 0xd4, 0xb5, 0x77, 0xee, 0xc1, 0x9f, 0x23, 0x46, 0x8c,
        0x05, 0x0a, 0x14, 0x28, 0x50, 0xa0, 0x5d, 0xba, 0x69, 0xd2, 0xb9, 0x6f, 0xde, 0xa1, 0x5f, 0xbe, 0x61, 0xc2, 0x99, 0x2f, 0x5e, 0xbc, 0x65, 0xca, 0x89,
        0x0f, 0x1e, 0x3c, 0x78, 0xf0, 0xfd, 0xe7, 0xd3, 0xbb, 0x6b, 0xd6, 0xb1, 0x7f, 0xfe, 0xe1, 0xdf, 0xa3, 0x5b, 0xb6, 0x71, 0xe2, 0xd9, 0xaf, 0x43, 0x86,
        0x11, 0x22, 0x44, 0x88, 0x0d, 0x1a, 0x34, 0x68, 0xd0, 0xbd, 0x67, 0xce, 0x81, 0x1f, 0x3e, 0x7c, 0xf8, 0xed, 0xc7, 0x93, 0x3b, 0x76, 0xec, 0xc5, 0x97,
        0x33, 0x66, 0xcc, 0x85, 0x17, 0x2e, 0x5c, 0xb8, 0x6d, 0xda, 0xa9, 0x4f, 0x9e, 0x21, 0x42, 0x84, 0x15, 0x2a, 0x54, 0xa8, 0x4d, 0x9a, 0x29, 0x52, 0xa4,
        0x55, 0xaa, 0x49, 0x92, 0x39, 0x72, 0xe4, 0xd5, 0xb7, 0x73, 0xe6, 0xd1, 0xbf, 0x63, 0xc6, 0x91, 0x3f, 0x7e, 0xfc, 0xe5, 0xd7, 0xb3, 0x7b, 0xf6, 0xf1,
        0xff, 0xe3, 0xdb, 0xab, 0x4b, 0x96, 0x31, 0x62, 0xc4, 0x95, 0x37, 0x6e, 0xdc, 0xa5, 0x57, 0xae, 0x41, 0x82, 0x19, 0x32, 0x64, 0xc8, 0x8d, 0x07, 0x0e,
        0x1c, 0x38, 0x70, 0xe0, 0xdd, 0xa7, 0x53, 0xa6, 0x51, 0xa2, 0x59, 0xb2, 0x79, 0xf2, 0xf9, 0xef, 0xc3, 0x9b, 0x2b, 0x56, 0xac, 0x45, 0x8a, 0x09, 0x12,
        0x24, 0x48, 0x90, 0x3d, 0x7a, 0xf4, 0xf5, 0xf7, 0xf3, 0xfb, 0xeb, 0xcb, 0x8b, 0x0b, 0x16, 0x2c, 0x58, 0xb0, 0x7d, 0xfa, 0xe9, 0xcf, 0x83, 0x1b, 0x36,
        0x6c, 0xd8, 0xad, 0x47, 0x8e, 0x01,
        // Repeat for overflow handling
        0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1d, 0x3a, 0x74, 0xe8, 0xcd, 0x87, 0x13, 0x26, 0x4c, 0x98, 0x2d, 0x5a, 0xb4, 0x75, 0xea, 0xc9, 0x8f, 0x03,
        0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x9d, 0x27, 0x4e, 0x9c, 0x25, 0x4a, 0x94, 0x35, 0x6a, 0xd4, 0xb5, 0x77, 0xee, 0xc1, 0x9f, 0x23, 0x46, 0x8c, 0x05,
        0x0a, 0x14, 0x28, 0x50, 0xa0, 0x5d, 0xba, 0x69, 0xd2, 0xb9, 0x6f, 0xde, 0xa1, 0x5f, 0xbe, 0x61, 0xc2, 0x99, 0x2f, 0x5e, 0xbc, 0x65, 0xca, 0x89, 0x0f,
        0x1e, 0x3c, 0x78, 0xf0, 0xfd, 0xe7, 0xd3, 0xbb, 0x6b, 0xd6, 0xb1, 0x7f, 0xfe, 0xe1, 0xdf, 0xa3, 0x5b, 0xb6, 0x71, 0xe2, 0xd9, 0xaf, 0x43, 0x86, 0x11,
        0x22, 0x44, 0x88, 0x0d, 0x1a, 0x34, 0x68, 0xd0, 0xbd, 0x67, 0xce, 0x81, 0x1f, 0x3e, 0x7c, 0xf8, 0xed, 0xc7, 0x93, 0x3b, 0x76, 0xec, 0xc5, 0x97, 0x33,
        0x66, 0xcc, 0x85, 0x17, 0x2e, 0x5c, 0xb8, 0x6d, 0xda, 0xa9, 0x4f, 0x9e, 0x21, 0x42, 0x84, 0x15, 0x2a, 0x54, 0xa8, 0x4d, 0x9a, 0x29, 0x52, 0xa4, 0x55,
        0xaa, 0x49, 0x92, 0x39, 0x72, 0xe4, 0xd5, 0xb7, 0x73, 0xe6, 0xd1, 0xbf, 0x63, 0xc6, 0x91, 0x3f, 0x7e, 0xfc, 0xe5, 0xd7, 0xb3, 0x7b, 0xf6, 0xf1, 0xff,
        0xe3, 0xdb, 0xab, 0x4b, 0x96, 0x31, 0x62, 0xc4, 0x95, 0x37, 0x6e, 0xdc, 0xa5, 0x57, 0xae, 0x41, 0x82, 0x19, 0x32, 0x64, 0xc8, 0x8d, 0x07, 0x0e, 0x1c,
        0x38, 0x70, 0xe0, 0xdd, 0xa7, 0x53, 0xa6, 0x51, 0xa2, 0x59, 0xb2, 0x79, 0xf2, 0xf9, 0xef, 0xc3, 0x9b, 0x2b, 0x56, 0xac, 0x45, 0x8a, 0x09, 0x12, 0x24,
        0x48, 0x90, 0x3d, 0x7a, 0xf4, 0xf5, 0xf7, 0xf3, 0xfb, 0xeb, 0xcb, 0x8b, 0x0b, 0x16, 0x2c, 0x58, 0xb0, 0x7d, 0xfa, 0xe9, 0xcf, 0x83, 0x1b, 0x36, 0x6c,
        0xd8, 0xad, 0x47, 0x8e, 0x01 };

const uint8_t fx25_gf_log[256] = { 0xff, 0x00, 0x01, 0x19, 0x02, 0x32, 0x1a, 0xc6, 0x03, 0xdf, 0x33, 0xee, 0x1b, 0x68, 0xc7, 0x4b, 0x04, 0x64, 0xe0, 0x0e, 0x34,
        0x8d, 0xef, 0x81, 0x1c, 0xc1, 0x69, 0xf8, 0xc8, 0x08, 0x4c, 0x71, 0x05, 0x8a, 0x65, 0x2f, 0xe1, 0x24, 0x0f, 0x21, 0x35, 0x93, 0x8e, 0xda, 0xf0, 0x12,
        0x82, 0x45, 0x1d, 0xb5, 0xc2, 0x7d, 0x6a, 0x27, 0xf9, 0xb9, 0xc9, 0x9a, 0x09, 0x78, 0x4d, 0xe4, 0x72, 0xa6, 0x06, 0xbf, 0x8b, 0x62, 0x66, 0xdd, 0x30,
        0xfd, 0xe2, 0x98, 0x25, 0xb3, 0x10, 0x91, 0x22, 0x88, 0x36, 0xd0, 0x94, 0xce, 0x8f, 0x96, 0xdb, 0xbd, 0xf1, 0xd2, 0x13, 0x5c, 0x83, 0x38, 0x46, 0x40,
        0x1e, 0x42, 0xb6, 0xa3, 0xc3, 0x48, 0x7e, 0x6e, 0x6b, 0x3a, 0x28, 0x54, 0xfa, 0x85, 0xba, 0x3d, 0xca, 0x5e, 0x9b, 0x9f, 0x0a, 0x15, 0x79, 0x2b, 0x4e,
        0xd4, 0xe5, 0xac, 0x73, 0xf3, 0xa7, 0x57, 0x07, 0x70, 0xc0, 0xf7, 0x8c, 0x80, 0x63, 0x0d, 0x67, 0x4a, 0xde, 0xed, 0x31, 0xc5, 0xfe, 0x18, 0xe3, 0xa5,
        0x99, 0x77, 0x26, 0xb8, 0xb4, 0x7c, 0x11, 0x44, 0x92, 0xd9, 0x23, 0x20, 0x89, 0x2e, 0x37, 0x3f, 0xd1, 0x5b, 0x95, 0xbc, 0xcf, 0xcd, 0x90, 0x87, 0x97,
        0xb2, 0xdc, 0xfc, 0xbe, 0x61, 0xf2, 0x56, 0xd3, 0xab, 0x14, 0x2a, 0x5d, 0x9e, 0x84, 0x3c, 0x39, 0x53, 0x47, 0x6d, 0x41, 0xa2, 0x1f, 0x2d, 0x43, 0xd8,
        0xb7, 0x7b, 0xa4, 0x76, 0xc4, 0x17, 0x49, 0xec, 0x7f, 0x0c, 0x6f, 0xf6, 0x6c, 0xa1, 0x3b, 0x52, 0x29, 0x9d, 0x55, 0xaa, 0xfb, 0x60, 0x86, 0xb1, 0xbb,
        0xcc, 0x3e, 0x5a, 0xcb, 0x59, 0x5f, 0xb0, 0x9c, 0xa9, 0xa0, 0x51, 0x0b, 0xf5, 0x16, 0xeb, 0x7a, 0x75, 0x2c, 0xd7, 0x4f, 0xae, 0xd5, 0xe9, 0xe6, 0xe7,
        0xad, 0xe8, 0x74, 0xd6, 0xf4, 0xea, 0xa8, 0x50, 0x58, 0xaf };

// Correlation tags per FX.25 v01.06 specification
// Stored as byte arrays in big-endian order to avoid 64-bit arithmetic
static const fx25_mode_t fx25_modes[] = {  //
        { 0x01, { 0xB7, 0x4D, 0xB7, 0xDF, 0x8A, 0x53, 0x2F, 0x3E }, 239, 16, 8 },  //
                { 0x02, { 0x26, 0xFF, 0x60, 0xA6, 0x00, 0xCC, 0x8F, 0xDE }, 128, 16, 8 },  //
                { 0x03, { 0xC7, 0xDC, 0x05, 0x08, 0xF3, 0xD9, 0xB0, 0x9E }, 64, 16, 8 },   //
                { 0x04, { 0x8F, 0x05, 0x6E, 0xB4, 0x36, 0x96, 0x60, 0xEE }, 32, 16, 8 },   //
                { 0x05, { 0x6E, 0x26, 0x0B, 0x1A, 0xC5, 0x83, 0x5F, 0xAE }, 223, 32, 16 },  //
                { 0x06, { 0xFF, 0x94, 0xDC, 0x63, 0x4F, 0x1C, 0xFF, 0x4E }, 128, 32, 16 },  //
                { 0x07, { 0x1E, 0xB7, 0xB9, 0xCD, 0xBC, 0x09, 0xC0, 0x0E }, 64, 32, 16 },  //
                { 0x08, { 0xDB, 0xF8, 0x69, 0xBD, 0x2D, 0xBB, 0x17, 0x76 }, 32, 32, 16 },  //
                { 0x09, { 0x3A, 0xDB, 0x0C, 0x13, 0xDE, 0xAE, 0x28, 0x36 }, 191, 64, 32 },  //
                { 0x0A, { 0xAB, 0x69, 0xDB, 0x6A, 0x54, 0x31, 0x88, 0xD6 }, 128, 64, 32 },  //
                { 0x0B, { 0x4A, 0x4A, 0xBE, 0xC4, 0xA7, 0x24, 0xB7, 0x96 }, 64, 64, 32 },  //
                { 0, { 0, 0, 0, 0, 0, 0, 0, 0 }, 0, 0, 0 }                                 //
        };

// Berlekamp-Massey algorithm for finding error locator polynomial
static void rs_find_error_locator(const uint8_t *syndromes, uint8_t nsyn, rs_poly_t *lambda) {
    rs_poly_t lambda_prev;
    rs_poly_t temp;
    uint8_t k = 0;
    uint8_t l = 0;
    uint8_t dm = 1;  // Discrepancy from previous iteration

    // Initialize
    memset(lambda, 0, sizeof(rs_poly_t));
    memset(&lambda_prev, 0, sizeof(rs_poly_t));
    lambda->coeff[0] = 1;
    lambda->length = 1;
    lambda_prev.coeff[0] = 1;
    lambda_prev.length = 1;

    for (k = 0; k < nsyn; k++) {
        // Calculate discrepancy
        uint8_t d = syndromes[k];
        for (uint8_t i = 1; i < lambda->length; i++) {
            d ^= GF_MUL(lambda->coeff[i], syndromes[k - i]);
        }

        if (d != 0) {
            // Update lambda
            temp = *lambda;

            uint8_t factor = GF_MUL(d, GF_DIV(1, dm));
            for (uint8_t i = 0; i < lambda_prev.length; i++) {
                uint8_t idx = i + k + 1 - (lambda_prev.length - 1);
                if (idx < 64) {
                    lambda->coeff[idx] ^= GF_MUL(factor, lambda_prev.coeff[i]);
                }
            }

            if (lambda->length < k + 1 - l + lambda_prev.length) {
                lambda->length = k + 1 - l + lambda_prev.length;
            }

            if (2 * l <= k) {
                l = k + 1 - l;
                lambda_prev = temp;
                dm = d;
            }
        }
    }
}

// Find error positions using Chien search
static uint8_t rs_find_error_positions(const rs_poly_t *lambda, uint8_t codeword_len, uint8_t *error_pos) {
    uint8_t num_errors = 0;

    // Chien search: evaluate lambda at each alpha^i
    for (uint8_t i = 0; i < codeword_len; i++) {
        uint8_t sum = lambda->coeff[0];

        for (uint8_t j = 1; j < lambda->length; j++) {
            sum ^= GF_MUL(lambda->coeff[j], fx25_gf_exp[(j * i) % 255]);
        }

        if (sum == 0) {
            // Error at position (codeword_len - 1 - i)
            if (num_errors < 32) {  // Max correctable
                error_pos[num_errors++] = codeword_len - 1 - i;
            }
        }
    }

    return num_errors;
}

// Calculate error magnitudes using Forney algorithm
static void rs_calculate_error_magnitudes(const uint8_t *syndromes, const rs_poly_t *lambda, const uint8_t *error_pos, uint8_t num_errors, uint8_t *error_mag) {
    for (uint8_t i = 0; i < num_errors; i++) {
        uint8_t pos = error_pos[i];
        uint8_t x_inv = fx25_gf_exp[255 - pos];  // alpha^(-pos)

        // Calculate omega(x_inv) - numerator
        uint8_t omega = syndromes[0];
        for (uint8_t j = 1; j < lambda->length; j++) {
            omega ^= GF_MUL(syndromes[j], GF_POW(x_inv, j));
        }

        // Calculate lambda'(x_inv) - denominator
        uint8_t lambda_prime = lambda->coeff[1];
        for (uint8_t j = 2; j < lambda->length; j += 2) {
            lambda_prime ^= GF_MUL(lambda->coeff[j + 1], GF_POW(x_inv, j));
        }

        // Error magnitude = omega / lambda'
        error_mag[i] = GF_DIV(omega, lambda_prime);
    }
}

// Helper function to compare two 8-byte correlation tags
// Returns true if tags match, false otherwise
static bool tag_matches(const uint8_t *tag1, const uint8_t *tag2) {
    for (int i = 0; i < 8; i++) {
        if (tag1[i] != tag2[i]) {
            return false;
        }
    }
    return true;
}

const fx25_mode_t* fx25_get_mode(uint8_t mode_id) {
    for (int i = 0; fx25_modes[i].tag_id != 0; i++) {
        if (fx25_modes[i].tag_id == mode_id) {
            return &fx25_modes[i];
        }
    }
    return NULL;
}

uint8_t fx25_select_mode(size_t ax25_len) {
    // Select smallest mode that fits the frame
    if (ax25_len <= 32)
        return FX25_MODE_32_16;  // Smallest overhead
    if (ax25_len <= 64)
        return FX25_MODE_64_16;
    if (ax25_len <= 128)
        return FX25_MODE_128_16;
    if (ax25_len <= 191)
        return FX25_MODE_191_64;  // Best protection for medium
    if (ax25_len <= 223)
        return FX25_MODE_223_32;
    if (ax25_len <= 239)
        return FX25_MODE_239_16;
    return 0;  // Too large for FX.25 single block
}

// Generate RS generator polynomial for given number of parity symbols
static void rs_generator_poly(uint8_t parity_bytes, uint8_t *g) {
    // g(x) = (x - alpha^0)(x - alpha^1)...(x - alpha^(parity_bytes-1))
    memset(g, 0, parity_bytes + 1);
    g[0] = 1;  // Start with g(x) = 1

    for (uint8_t i = 0; i < parity_bytes; i++) {
        // Multiply by (x - alpha^i)
        uint8_t alpha_i = fx25_gf_exp[i];
        for (int j = parity_bytes; j > 0; j--) {
            g[j] = g[j - 1] ^ GF_MUL(g[j], alpha_i);
        }
        g[0] = GF_MUL(g[0], alpha_i);
    }
}

// Encode data using Reed-Solomon
static void rs_encode(const uint8_t *data, uint8_t data_len, uint8_t parity_bytes, uint8_t *parity) {
    uint8_t g[65];  // Max 64 parity bytes
    rs_generator_poly(parity_bytes, g);

    memset(parity, 0, parity_bytes);

    for (uint8_t i = 0; i < data_len; i++) {
        uint8_t feedback = data[i] ^ parity[0];

        // Shift parity register
        for (uint8_t j = 0; j < parity_bytes - 1; j++) {
            parity[j] = parity[j + 1] ^ GF_MUL(g[parity_bytes - j], feedback);
        }
        parity[parity_bytes - 1] = GF_MUL(g[1], feedback);
    }
}

uint8_t fx25_encode(const uint8_t *ax25_frame, size_t ax25_len, uint8_t mode_id, fx25_frame_t *fx25_frame) {
    if (!ax25_frame || !fx25_frame || ax25_len == 0)
        return 1;

    const fx25_mode_t *mode = fx25_get_mode(mode_id);
    if (!mode)
        return 2;
    if (ax25_len > mode->data_bytes)
        return 3;  // Frame too large

    fx25_frame->mode_id = mode_id;
    // Copy correlation tag as byte array instead of uint64_t assignment
    for (int i = 0; i < 8; i++) {
        fx25_frame->correlation_tag[i] = mode->correlation_tag[i];
    }
    fx25_frame->codeword_len = mode->data_bytes + mode->parity_bytes;

    fx25_frame->rs_codeword = malloc(fx25_frame->codeword_len);
    if (!fx25_frame->rs_codeword)
        return 4;

    // Copy AX.25 frame to data portion (zero-padded)
    memcpy(fx25_frame->rs_codeword, ax25_frame, ax25_len);
    if (ax25_len < mode->data_bytes) {
        memset(fx25_frame->rs_codeword + ax25_len, 0, mode->data_bytes - ax25_len);
    }

    // Compute parity bytes
    uint8_t *parity = fx25_frame->rs_codeword + mode->data_bytes;
    rs_encode(fx25_frame->rs_codeword, mode->data_bytes, mode->parity_bytes, parity);

    return 0;
}

// Syndrome computation for decoding
static void rs_compute_syndromes(const uint8_t *codeword, uint8_t len, uint8_t parity_bytes, uint8_t *syndromes) {
    for (uint8_t i = 0; i < parity_bytes; i++) {
        syndromes[i] = 0;
        uint8_t alpha_i = fx25_gf_exp[i];  // alpha^i

        for (uint8_t j = 0; j < len; j++) {
            syndromes[i] = GF_MUL(syndromes[i], alpha_i) ^ codeword[j];
        }
    }
}

// Simplified decoder - returns number of errors corrected or 0xFF if uncorrectable
uint8_t fx25_decode(const uint8_t *rx_data, size_t rx_len, fx25_frame_t *fx25_frame, uint8_t *corrected_errors) {
    if (!rx_data || !fx25_frame || !corrected_errors)
        return 1;

    // Minimum size: correlation tag (8) + at least some data + parity
    if (rx_len < 8 + 32 + 16)
        return 2;

    // Extract correlation tag (first 8 bytes) - store as byte array
    uint8_t rx_tag[8];
    for (int i = 0; i < 8; i++) {
        rx_tag[i] = rx_data[i];
    }

    // Find matching mode by comparing byte arrays instead of uint64_t
    const fx25_mode_t *mode = NULL;
    for (int i = 0; fx25_modes[i].tag_id != 0; i++) {
        if (tag_matches(rx_tag, fx25_modes[i].correlation_tag)) {
            mode = &fx25_modes[i];
            break;
        }
    }

    if (!mode)
        return 3;  // Invalid correlation tag

    size_t expected_len = 8 + mode->data_bytes + mode->parity_bytes;
    if (rx_len < expected_len)
        return 4;  // Too short

    fx25_frame->mode_id = mode->tag_id;
    // Copy correlation tag as byte array instead of uint64_t assignment
    for (int i = 0; i < 8; i++) {
        fx25_frame->correlation_tag[i] = rx_tag[i];
    }
    fx25_frame->codeword_len = mode->data_bytes + mode->parity_bytes;

    fx25_frame->rs_codeword = malloc(fx25_frame->codeword_len);
    if (!fx25_frame->rs_codeword)
        return 5;

    // Copy RS codeword (skip correlation tag)
    memcpy(fx25_frame->rs_codeword, rx_data + 8, fx25_frame->codeword_len);

    // Compute syndromes
    uint8_t syndromes[64];
    rs_compute_syndromes(fx25_frame->rs_codeword, fx25_frame->codeword_len, mode->parity_bytes, syndromes);

    // Check if all syndromes are zero (no errors)
    bool all_zero = true;
    for (uint8_t i = 0; i < mode->parity_bytes; i++) {
        if (syndromes[i] != 0) {
            all_zero = false;
            break;
        }
    }

    if (all_zero) {
        *corrected_errors = 0;
        return 0;  // Success, no errors
    }

    // Find error locator polynomial using Berlekamp-Massey
    rs_poly_t lambda;
    rs_find_error_locator(syndromes, mode->parity_bytes, &lambda);

    // Find error positions using Chien search
    uint8_t error_pos[32];
    uint8_t num_errors = rs_find_error_positions(&lambda, fx25_frame->codeword_len, error_pos);

    if (num_errors == 0 || num_errors > mode->correctable_bytes) {
        *corrected_errors = 0xFF;  // Uncorrectable
        return 6;
    }

    // Calculate error magnitudes using Forney algorithm
    uint8_t error_mag[32];
    rs_calculate_error_magnitudes(syndromes, &lambda, error_pos, num_errors, error_mag);

    // Correct errors
    for (uint8_t i = 0; i < num_errors; i++) {
        fx25_frame->rs_codeword[error_pos[i]] ^= error_mag[i];
    }

    *corrected_errors = num_errors;
    return 0;
}

void fx25_frame_free(fx25_frame_t *frame) {
    if (frame && frame->rs_codeword) {
        free(frame->rs_codeword);
        frame->rs_codeword = NULL;
    }
}
