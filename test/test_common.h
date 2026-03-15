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
            printf("[%04d] FAIL(%u): %s\n", \
                    ++assert_count, (unsigned)(err), (message)); \
            return 1; \
        } else { \
            printf("[%04d]    PASS: %s\n", \
                    ++assert_count, (message)); \
        } \
    } while (0)

#define COMPARE_FRAME(encoded, encoded_len, expected, expected_len, msg) \
    do { \
        int _cmp = memcmp((encoded), (expected), \
                          ((encoded_len) < (expected_len)) \
                              ? (size_t)(encoded_len) : (size_t)(expected_len)); \
        if (_cmp != 0 || (encoded_len) != (expected_len)) { \
            printf("[%04d] FAIL: %s\n", ++assert_count, (msg)); \
            printf("  Expected (%zu bytes): ", (size_t)(expected_len)); \
            for (size_t _i = 0; _i < (size_t)(expected_len); _i++) \
                printf("%02X ", (expected)[_i]); \
            printf("\n  Got      (%zu bytes): ", (size_t)(encoded_len)); \
            for (size_t _i = 0; _i < (size_t)(encoded_len); _i++) \
                printf("%02X ", (encoded)[_i]); \
            printf("\n"); \
            TEST_ASSERT(0, (msg), _cmp); \
        } else { \
            printf("[%04d]    PASS: %s\n", ++assert_count, (msg)); \
        } \
    } while (0)

#define TEST_SECTION(title) \
    printf("\n%s\n", (title))

#ifdef DEBUG_ENABLE
#define DEBUG_PRINT(fmt, ...) \
    printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#define DEBUG_FRAME(label, data, len) \
    do { \
        printf("[DEBUG] %s (%zu bytes): ", (label), (size_t)(len)); \
        for (size_t _i = 0; _i < (size_t)(len); _i++) \
            printf("%02X ", ((const uint8_t*)(data))[_i]); \
        printf("\n"); \
    } while (0)
#define DEBUG_STATE(label, state) \
    printf("[DEBUG] %s: %d\n",  (label), (state))
#define DEBUG_VAR(label, var) \
    printf("[DEBUG] %s: %u\n",  (label), (unsigned int)(var))
#define DEBUG_VAR64(label, var) \
    printf("[DEBUG] %s: %lu\n", (label), (unsigned long)(var))
#define DEBUG_BOOL(label, var) \
    printf("[DEBUG] %s: %s\n",  (label), (var) ? "true" : "false")
#define DEBUG_HEX(label, val) \
    printf("  [DBG] %-45s = 0x%02X\n", (label), (unsigned)(val))
#define DEBUG_BUF(label, buf, len) \
    do { \
        printf("  [DBG] %-45s (%zu bytes): ", (label), (size_t)(len)); \
        for (size_t _i = 0; _i < (size_t)(len); _i++) \
            printf("%02X ", ((const uint8_t*)(buf))[_i]); \
        printf("\n"); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...)          ((void)0)
#define DEBUG_FRAME(label, data, len)  ((void)0)
#define DEBUG_STATE(label, state)      ((void)0)
#define DEBUG_VAR(label, var)          ((void)0)
#define DEBUG_VAR64(label, var)        ((void)0)
#define DEBUG_BOOL(label, var)         ((void)0)
#define DEBUG_HEX(label, val)          ((void)0)
#define DEBUG_BUF(label, buf, len)     ((void)0)
#endif

#endif /* TEST_COMMON_H_ */
