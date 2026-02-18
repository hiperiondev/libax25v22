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

#ifndef TEST_COMMON_H_
#define TEST_COMMON_H_

#define TEST_ASSERT(condition, message, err) \
    do { \
        if (!(condition)) { \
            printf("\033[0;31m[%04d] FAIL(%u): %s\033[0m\n", ++assert_count, err, message); \
            return 1; \
        } else { \
            printf("\033[0;32m[%04d]    PASS: %s\033[0m\n", ++assert_count, message); \
        } \
    } while (0)

#define COMPARE_FRAME(encoded, encoded_len, expected, expected_len, msg) \
    do { \
        int cmp = memcmp(encoded, expected, (encoded_len < expected_len) ? encoded_len : expected_len); \
        if (cmp != 0 || encoded_len != expected_len) { \
            printf("\033[0;31m[%04d] FAIL: %s\nExpected (%zu bytes): ", ++assert_count, msg, expected_len); \
            for (size_t i = 0; i < expected_len; i++) printf("%02X ", expected[i]); \
            printf("\nGot (%zu bytes): ", encoded_len); \
            for (size_t i = 0; i < encoded_len; i++) printf("%02X ", encoded[i]); \
            printf("\033[0m\n"); \
            TEST_ASSERT(false, msg, cmp); \
        } else { \
            printf("\033[0;32m[%04d]    PASS: %s\033[0m\n", ++assert_count, msg); \
        } \
    } while (0)
#ifdef DEBUG_ENABLE
    #define DEBUG_PRINT(fmt, ...) printf("\033[0;36m[DEBUG] " fmt "\033[0m\n", ##__VA_ARGS__)
    #define DEBUG_FRAME(label, data, len) \
        do { \
            printf("\033[0;36m[DEBUG] %s (%zu bytes): ", label, len); \
            for (size_t i = 0; i < len; i++) printf("%02X ", data[i]); \
            printf("\033[0m\n"); \
        } while (0)
    #define DEBUG_STATE(label, state) printf("\033[0;36m[DEBUG] %s: %d\033[0m\n", label, state)
    #define DEBUG_VAR(label, var) printf("\033[0;36m[DEBUG] %s: %u\033[0m\n", label, (unsigned int)var)
    #define DEBUG_VAR64(label, var) printf("\033[0;36m[DEBUG] %s: %lu\033[0m\n", label, (unsigned long)var)
    #define DEBUG_BOOL(label, var) printf("\033[0;36m[DEBUG] %s: %s\033[0m\n", label, var ? "true" : "false")
#else
    #define DEBUG_PRINT(fmt, ...)
    #define DEBUG_FRAME(label, data, len)
    #define DEBUG_STATE(label, state)
    #define DEBUG_VAR(label, var)
    #define DEBUG_VAR64(label, var)
    #define DEBUG_BOOL(label, var)
#endif

#endif /* TEST_COMMON_H_ */
