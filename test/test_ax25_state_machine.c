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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "test_common.h"
#include "ax25_state_machine.h"
#include "ax25.h"

static uint32_t assert_count = 0;

static uint8_t captured_buffer[2048];
static size_t captured_len = 0;

static void reset_capture(void) {
    captured_len = 0;
}

static void capture_transmit(void *user_data, uint8_t *data, size_t len) {
    if (len <= sizeof(captured_buffer)) {
        memcpy(captured_buffer, data, len);
        captured_len = len;
    }
}

// Hardcoded callsign bytes (shifted left by 1, space padded)
static const uint8_t test1_call[6] = { 0xA8, 0x8A, 0xA6, 0xA8, 0x62, 0x40 };  // TEST1
static const uint8_t test2_call[6] = { 0xA8, 0x8A, 0xA6, 0xA8, 0x64, 0x40 };  // TEST2

static int establish_connection(ax25_connection_t *conn, ax25_address_t *dest, ax25_address_t *src) {
    // Expected SABM frame (modulo 8, no digi, SSID-0, P=1)
    uint8_t expected_sabm[15];
    memcpy(expected_sabm + 0, test2_call, 6);     // destination callsign
    expected_sabm[6] = 0x60;                      // destination SSID (not last, ch=0, res0=1, res1=1, ext=0, ssid=0)
    memcpy(expected_sabm + 7, test1_call, 6);     // source callsign
    // source for modulo-8 (res1=1)
    expected_sabm[13] = 0x61;                     // source SSID (last, ch=0, res0=1, res1=1, ext=1, ssid=0)
    expected_sabm[14] = 0x3F;                     // SABM with P=1

    reset_capture();
    uint8_t err = ax25_connect(conn, dest, src);
    TEST_ASSERT(err == 0, "ax25_connect succeeded", err);
    TEST_ASSERT(conn->state == AX25_STATE_AWAITING_CONNECTION, "State is AWAITING_CONNECTION", 0);
    COMPARE_FRAME(captured_buffer, captured_len, expected_sabm, sizeof(expected_sabm), "Transmitted SABM frame matches expected (modulo 8, P=1)");

    // Hardcoded UA response frame (addresses swapped, F=1)
    uint8_t ua_raw[15];
    memcpy(ua_raw + 0, test1_call, 6);            // destination = original source
    // destination for modulo-8
    ua_raw[6] = 0x60;                             // destination SSID (not last, ch=0, res0=1, res1=1, ext=0, ssid=0)
    memcpy(ua_raw + 7, test2_call, 6);            // source = original destination
    // source for modulo-8 with extension bit
    ua_raw[13] = 0x61;                            // source SSID (last, ch=1, res0=1, res1=1, ext=1, ssid=0)
    ua_raw[14] = 0x73;                            // UA with F=1

    uint8_t decode_err = 0;
    ax25_frame_t *ua_frame = ax25_frame_decode(ua_raw, sizeof(ua_raw), MODULO128_FALSE, &decode_err);
    TEST_ASSERT(ua_frame != NULL && decode_err == 0, "Hardcoded UA frame decoded successfully", decode_err);

    reset_capture();
    ax25_process_frame(conn, ua_frame);
    TEST_ASSERT(conn->state == AX25_STATE_CONNECTED, "State changed to CONNECTED after receiving UA", 0);
    TEST_ASSERT(captured_len == 0, "No frame transmitted when receiving UA", 0);

    ax25_frame_free(ua_frame, &decode_err);
    return 0;
}

static int test_connection_establishment(void) {
    printf("\n--- test_connection_establishment ---\n");

    ax25_connection_t conn;
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    uint8_t err = ax25_connection_init(&conn, &cb, NULL);
    TEST_ASSERT(err == 0, "Connection init succeeded", err);
    TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "Initial state is DISCONNECTED", 0);

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    TEST_ASSERT(dest != NULL && parse_err == 0, "Destination address parsed", parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(src != NULL && parse_err == 0, "Source address parsed", parse_err);

    int result = establish_connection(&conn, dest, src);

    free(dest);
    free(src);
    return result;
}

static int test_data_transfer(void) {
    printf("\n--- test_data_transfer ---\n");

    ax25_connection_t *conn = malloc(sizeof(ax25_connection_t));
    ax25_callbacks_t cb = { .transmit = capture_transmit };
    uint8_t err = ax25_connection_init(conn, &cb, NULL);
    TEST_ASSERT(err == 0, "Connection init succeeded", err);

    uint8_t parse_err = 0;
    ax25_address_t *dest = ax25_address_from_string("TEST2-0", &parse_err);
    TEST_ASSERT(dest != NULL && parse_err == 0, "Destination address parsed", parse_err);
    ax25_address_t *src = ax25_address_from_string("TEST1-0", &parse_err);
    TEST_ASSERT(src != NULL && parse_err == 0, "Source address parsed", parse_err);

    establish_connection(conn, dest, src);

    // Send data
    const uint8_t payload[] = { 'T', 'E', 'S', 'T' };
    const size_t payload_len = sizeof(payload);
    const uint8_t pid = 0xF0;

    // Expected I-frame (N(S)=0, N(R)=0, P/F=0, modulo 8)
    uint8_t expected_i[20];
    memcpy(expected_i + 0, test2_call, 6);        // destination callsign
    // modulo-8 SSID bytes
    expected_i[6] = 0x60;                         // destination SSID (ch=0, res0=1, res1=1, ext=0, ssid=0)
    memcpy(expected_i + 7, test1_call, 6);        // source callsign
    expected_i[13] = 0x61;                        // source SSID (ch=0, res0=1, res1=1, ext=1, ssid=0)
    expected_i[14] = 0x00;                        // I-frame control (NS=0, NR=0, PF=0)
    expected_i[15] = pid;                         // PID
    memcpy(expected_i + 16, payload, payload_len);  // payload

    reset_capture();
    err = ax25_send_data(conn, (uint8_t*) payload, payload_len, pid);
    TEST_ASSERT(err == 0, "ax25_send_data succeeded", err);

    COMPARE_FRAME(captured_buffer, captured_len, expected_i, sizeof(expected_i), "Transmitted I-frame matches expected (modulo 8)");

    free(dest);
    free(src);
    free(conn);

    return 0;
}

int test_ax25_state_machine_main(void) {
    int result = 0;

    printf("\n----------------------------------------------------------------------------------\n");
    printf("Starting AX.25 State Machine (Connected Mode) Tests\n");
    printf("----------------------------------------------------------------------------------\n\n");

    result |= test_connection_establishment();
    result |= test_data_transfer();

    printf("\n----------------------------------------------------------------------------------\n");
    printf("AX.25 State Machine Tests Completed. %s\n", result == 0 ? "All tests passed" : "Some tests failed");
    printf("----------------------------------------------------------------------------------\n\n");

    return result;
}
