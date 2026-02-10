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

#ifndef FX25_GF256_H_
#define FX25_GF256_H_

#include <stdint.h>

// GF(256) error detection for embedded systems
// Define MCU_DEBUG to enable runtime error detection
#ifdef MCU_DEBUG
extern uint8_t gf_error_flag;
#define GF_SET_ERROR() do { gf_error_flag = 1; } while(0)
#else
#define GF_SET_ERROR() do { } while(0)
#endif

// Primitive polynomial: 0x11D (x^8 + x^4 + x^3 + x^2 + 1)
// This is the standard polynomial used in FX.25/AX.25 applications
#define GF_PRIMITIVE_POLY 0x11D

// GF(256) tables - placed in ROM/flash on microcontrollers
extern const uint8_t gf_exp[512];   // Anti-log table (extended to 512 for efficiency)
extern const uint8_t gf_log[256];   // Logarithm table

// Initialize GF tables (call once at startup)
void gf_init_tables(void);

// GF(256) addition (simple XOR)
static inline uint8_t gf_add(uint8_t a, uint8_t b) {
    return a ^ b;
}

// GF(256) subtraction (same as addition in GF(2^m))
static inline uint8_t gf_sub(uint8_t a, uint8_t b) {
    return a ^ b;
}

// GF(256) multiplication using log tables
static inline uint8_t gf_mul(uint8_t a, uint8_t b) {
    if (a == 0 || b == 0)
        return 0;

    // Using 16-bit intermediate to avoid overflow
    uint16_t log_sum = (uint16_t) gf_log[a] + (uint16_t) gf_log[b];

    // Modulo 255 using conditional subtraction (no division!)
    if (log_sum >= 255) {
        log_sum -= 255;
    }

    return gf_exp[log_sum];
}

// GF(256) division using log tables
static inline uint8_t gf_div(uint8_t a, uint8_t b) {
    if (a == 0)
        return 0;  // 0 divided by anything is 0 (valid in GF)

    if (b == 0) {
        // Division by zero is undefined and indicates algorithm error or corrupted data
        GF_SET_ERROR();
        return 0xFF;  // Return invalid field element to make error visible
    }

    int16_t log_diff = (int16_t)gf_log[a] - (int16_t)gf_log[b];

    // Modulo 255 for negative values
    if (log_diff < 0) {
        log_diff += 255;
    }

    return gf_exp[log_diff];
}

// GF(256) multiplicative inverse
static inline uint8_t gf_inverse(uint8_t a) {
    if (a == 0)
        return 0;
    return gf_exp[255 - gf_log[a]];
}

// GF(256) power: a^b (using repeated squaring - no 64-bit)
uint8_t gf_pow(uint8_t a, uint8_t b);

#endif /* FX25_GF256_H_ */
