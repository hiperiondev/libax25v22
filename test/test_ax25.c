/*
 * Copyright 2026 Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com))
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
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "test_common.h"
#include "ax25.h"
#include "utils.h"
#include "hdlc.h"

static uint32_t assert_count = 0;

#define PRINT_PACKET(packet, len)   \
    printf("         -- packet: "); \
    for (uint32_t n=0;n<len;n++)    \
        printf("%02x ", packet[n]); \
    printf("\n")

int test_address_functions() {
    printf("test_address_functions\n");
    uint8_t err = 0;

    // Test ax25_address_from_string with "NOCALL-7*"
    ax25_address_t *addr = ax25_address_from_string("NOCALL-7*", &err);
    TEST_ASSERT(addr != NULL, "ax25_address_from_string should return non-NULL for valid input", err);
    if (addr) {
        TEST_ASSERT(strcmp(addr->callsign, "NOCALL") == 0, "Callsign should be NOCALL", err);
        TEST_ASSERT(addr->ssid == 7, "SSID should be 7", err);
        TEST_ASSERT(addr->ch == true, "ch should be true due to '*'", err);
        TEST_ASSERT(addr->res0 == true, "res0 should be true", err);
        TEST_ASSERT(addr->res1 == true, "res1 should be true", err);
        TEST_ASSERT(addr->extension == false, "extension should be false", err);
        ax25_address_free(addr, &err);
    }

    // Test ax25_address_from_string with "ABC123-15"
    addr = ax25_address_from_string("ABC123-15", &err);
    TEST_ASSERT(addr != NULL, "ax25_address_from_string should return non-NULL for valid input", err);
    if (addr) {
        TEST_ASSERT(strcmp(addr->callsign, "ABC123") == 0, "Callsign should be ABC123", err);
        TEST_ASSERT(addr->ssid == 15, "SSID should be 15", err);
        TEST_ASSERT(addr->ch == false, "ch should be false", err);
        TEST_ASSERT(addr->res0 == true, "res0 should be true", err);
        TEST_ASSERT(addr->res1 == true, "res1 should be true", err);
        TEST_ASSERT(addr->extension == false, "extension should be false", err);
        ax25_address_free(addr, &err);
    }

    // Test ax25_address_from_string with "NOCALL-16" (SSID out of range)
    addr = ax25_address_from_string("NOCALL-16", &err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for invalid SSID", err);
    TEST_ASSERT(err == 4, "Error code should be 4 for invalid SSID", err);

    // Test ax25_address_from_string with "NOCALL-1A" (Non-numeric SSID)
    addr = ax25_address_from_string("NOCALL-1A", &err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for non-numeric SSID", err);
    TEST_ASSERT(err == 5, "Error code should be 5 for invalid character after SSID", err);

    // Test ax25_address_from_string with "NOCALL*7" (Misplaced asterisk)
    addr = ax25_address_from_string("NOCALL*7", &err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for misplaced asterisk", err);
    TEST_ASSERT(err == 6, "Error code should be 6 for '*' not at the end", err);

    // Test ax25_address_from_string with "TOOLONGADDR-1" (String too long)
    addr = ax25_address_from_string("TOOLONGADDR-1", &err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for string too long", err);
    TEST_ASSERT(err == 4, "Error code should be 4 for invalid callsign length", err);

    // Test ax25_address_from_string with "" (Empty string)
    addr = ax25_address_from_string("", &err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for empty string", err);
    TEST_ASSERT(err == 4, "Error code should be 4 for invalid callsign length", err);

    // Test ax25_address_from_string with NULL
    addr = ax25_address_from_string(NULL, &err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for NULL input", err);
    TEST_ASSERT(err == 2, "Error code should be 2 for invalid input", err);

    return 0;
}

int test_path_functions() {
    printf("test_path_functions\n");
    uint8_t err = 0;

    // Test 1: Single repeater
    {
        ax25_address_t *addr1 = ax25_address_from_string("REPEATER-1*", &err);
        TEST_ASSERT(addr1 != NULL, "Address creation should succeed", err);
        ax25_address_t *repeaters[] = { addr1 };
        ax25_path_t *path = ax25_path_new(repeaters, 1, &err);
        TEST_ASSERT(path != NULL, "Path creation with one repeater should succeed", err);
        TEST_ASSERT(path->num_repeaters == 1, "Path should have 1 repeater", err);
        TEST_ASSERT(strcmp(path->repeaters[0].callsign, "REPEAT") == 0, "Repeater callsign should be REPEAT", err);
        TEST_ASSERT(path->repeaters[0].ssid == 1, "Repeater SSID should be 1", err);
        TEST_ASSERT(path->repeaters[0].ch == true, "Repeater ch should be true", err);
        ax25_path_free(path, &err);
        ax25_address_free(addr1, &err);
    }

    // Test 2: Zero repeaters
    {
        ax25_address_t *repeaters[] = { };
        ax25_path_t *path = ax25_path_new(repeaters, 0, &err);
        TEST_ASSERT(path == NULL, "Path creation with zero repeaters should fail", err);
        TEST_ASSERT(err == 2, "Error should be 2 for invalid input", err);
    }

    // Test 3: Maximum repeaters (8)
    {
        ax25_address_t *repeaters[MAX_REPEATERS];
        for (int i = 0; i < MAX_REPEATERS; i++) {
            char callsign[12];
            sprintf(callsign, "RPT%d-%d*", i, i);
            repeaters[i] = ax25_address_from_string(callsign, &err);
            TEST_ASSERT(repeaters[i] != NULL, "Repeater address creation should succeed", err);
        }
        ax25_path_t *path = ax25_path_new(repeaters, MAX_REPEATERS, &err);
        TEST_ASSERT(path != NULL, "Path creation with max repeaters should succeed", err);
        TEST_ASSERT(path->num_repeaters == MAX_REPEATERS, "Path should have 8 repeaters", err);
        for (int i = 0; i < MAX_REPEATERS; i++) {
            char expected_callsign[7];
            sprintf(expected_callsign, "RPT%d", i);
            TEST_ASSERT(strcmp(path->repeaters[i].callsign, expected_callsign) == 0, "Repeater callsign should match", err);
            TEST_ASSERT(path->repeaters[i].ssid == i, "Repeater SSID should match index", err);
            TEST_ASSERT(path->repeaters[i].ch == true, "Repeater ch should be true", err);
        }
        ax25_path_free(path, &err);
        for (int i = 0; i < MAX_REPEATERS; i++) {
            ax25_address_free(repeaters[i], &err);
        }
    }

    // Test 4: Exceeding maximum repeaters (9)
    {
        ax25_address_t *repeaters[MAX_REPEATERS + 1];
        for (int i = 0; i < MAX_REPEATERS + 1; i++) {
            char callsign[12];
            sprintf(callsign, "RPT%d-%d*", i, i);
            repeaters[i] = ax25_address_from_string(callsign, &err);
            TEST_ASSERT(repeaters[i] != NULL, "Repeater address creation should succeed", err);
        }
        ax25_path_t *path = ax25_path_new(repeaters, MAX_REPEATERS + 1, &err);
        TEST_ASSERT(path == NULL, "Path creation exceeding max repeaters should fail", err);
        TEST_ASSERT(err == 2, "Error should be 2 for too many repeaters", err);
        for (int i = 0; i < MAX_REPEATERS + 1; i++) {
            ax25_address_free(repeaters[i], &err);
        }
    }

    // Test 5: NULL repeaters array
    {
        ax25_path_t *path = ax25_path_new(NULL, 1, &err);
        TEST_ASSERT(path == NULL, "Path creation with NULL repeaters should fail", err);
        TEST_ASSERT(err == 2, "Error should be 2 for NULL input", err);
    }

    // Test 6: NULL individual repeater
    {
        ax25_address_t *addr1 = ax25_address_from_string("REPEATER-1*", &err);
        TEST_ASSERT(addr1 != NULL, "Address creation should succeed", err);
        ax25_address_t *repeaters[] = { addr1, NULL };
        ax25_path_t *path = ax25_path_new(repeaters, 2, &err);
        TEST_ASSERT(path == NULL, "Path creation with NULL repeater should fail", err);
        TEST_ASSERT(err == 2, "Error should be 2 for NULL repeater", err);
        ax25_address_free(addr1, &err);
    }

    // Test 7: Realistic AX.25 path with 3 repeaters
    {
        ax25_address_t *addr1 = ax25_address_from_string("WIDE1-1*", &err);
        ax25_address_t *addr2 = ax25_address_from_string("WIDE2-2*", &err);
        ax25_address_t *addr3 = ax25_address_from_string("NOCALL-0", &err);
        TEST_ASSERT(addr1 != NULL && addr2 != NULL && addr3 != NULL, "Address creation should succeed", err);
        ax25_address_t *repeaters[] = { addr1, addr2, addr3 };
        ax25_path_t *path = ax25_path_new(repeaters, 3, &err);
        TEST_ASSERT(path != NULL, "Path creation with realistic repeaters should succeed", err);
        TEST_ASSERT(path->num_repeaters == 3, "Path should have 3 repeaters", err);
        TEST_ASSERT(strcmp(path->repeaters[0].callsign, "WIDE1") == 0, "First repeater callsign should be WIDE1", err);
        TEST_ASSERT(path->repeaters[0].ssid == 1, "First repeater SSID should be 1", err);
        TEST_ASSERT(path->repeaters[0].ch == true, "First repeater ch should be true", err);
        TEST_ASSERT(strcmp(path->repeaters[1].callsign, "WIDE2") == 0, "Second repeater callsign should be WIDE2", err);
        TEST_ASSERT(path->repeaters[1].ssid == 2, "Second repeater SSID should be 2", err);
        TEST_ASSERT(path->repeaters[1].ch == true, "Second repeater ch should be true", err);
        TEST_ASSERT(strcmp(path->repeaters[2].callsign, "NOCALL") == 0, "Third repeater callsign should be NOCALL", err);
        TEST_ASSERT(path->repeaters[2].ssid == 0, "Third repeater SSID should be 0", err);
        TEST_ASSERT(path->repeaters[2].ch == false, "Third repeater ch should be false", err);
        ax25_path_free(path, &err);
        ax25_address_free(addr1, &err);
        ax25_address_free(addr2, &err);
        ax25_address_free(addr3, &err);
    }

    return 0;
}

int test_modulo128_source_address() {
    printf("test_modulo128_source_address\n");
    uint8_t err = 0;

    // Create addresses
    ax25_address_t dest = { .callsign = "NOCALL", .ssid = 0, .ch = true, .res0 = true, .res1 = true, .extension = false };
    ax25_address_t src = { .callsign = "REPEAT", .ssid = 1, .ch = false, .res0 = true, .res1 = true, .extension = true };
    // Set callsign properly
    memset(dest.callsign, ' ', 6);
    memcpy(dest.callsign, "NOCALL", 6);
    memset(src.callsign, ' ', 6);
    memcpy(src.callsign, "REPEAT", 6);

    // Create a modulo-128 I-frame
    ax25_information_frame_t i_frame;
    i_frame.base.type = AX25_FRAME_INFORMATION_16BIT;
    i_frame.base.header.destination = dest;
    i_frame.base.header.source = src;
    i_frame.base.header.cr = true;
    i_frame.base.header.src_cr = false;
    i_frame.base.header.repeaters.num_repeaters = 0;
    i_frame.nr = 3;
    i_frame.pf = true;
    i_frame.ns = 5;
    i_frame.pid = 0xF0;
    i_frame.payload_len = 4;
    i_frame.payload = (uint8_t*) "TEST";

    // Encode the frame
    size_t len;
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &i_frame, &len, &err);
    TEST_ASSERT(encoded != NULL, "Frame encoding should succeed", err);
    if (encoded) {
        // Source address is bytes 7 to 13
        uint8_t source_ssid_byte = encoded[13];
        TEST_ASSERT((source_ssid_byte & 0x40) == 0, "Source SSID bit 6 should be 0 for modulo-128", err);
        // Expected SSID byte: 0x23
        TEST_ASSERT(source_ssid_byte == 0x23, "Source SSID byte should be 0x23", err);
        free(encoded);
    }
    return 0;
}

int test_modulo8_source_address() {
    printf("test_modulo8_source_address\n");
    uint8_t err = 0;

    // Create addresses
    ax25_address_t dest = { .callsign = "NOCALL", .ssid = 0, .ch = true, .res0 = true, .res1 = true, .extension = false };
    ax25_address_t src = { .callsign = "REPEAT", .ssid = 1, .ch = false, .res0 = true, .res1 = true, .extension = true };
    memset(dest.callsign, ' ', 6);
    memcpy(dest.callsign, "NOCALL", 6);
    memset(src.callsign, ' ', 6);
    memcpy(src.callsign, "REPEAT", 6);

    // Create a modulo-8 I-frame
    ax25_information_frame_t i_frame;
    i_frame.base.type = AX25_FRAME_INFORMATION_8BIT;
    i_frame.base.header.destination = dest;
    i_frame.base.header.source = src;
    i_frame.base.header.cr = true;
    i_frame.base.header.src_cr = false;
    i_frame.base.header.repeaters.num_repeaters = 0;
    i_frame.nr = 3;
    i_frame.pf = true;
    i_frame.ns = 5;
    i_frame.pid = 0xF0;
    i_frame.payload_len = 4;
    i_frame.payload = (uint8_t*) "TEST";

    // Encode the frame
    size_t len;
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) &i_frame, &len, &err);
    TEST_ASSERT(encoded != NULL, "Frame encoding should succeed", err);
    if (encoded) {
        // Source address is bytes 7 to 13
        uint8_t source_ssid_byte = encoded[13];
        TEST_ASSERT((source_ssid_byte & 0x40) == 0x40, "Source SSID bit 6 should be 1 for modulo-8", err);
        // Expected SSID byte: 0x63
        TEST_ASSERT(source_ssid_byte == 0x63, "Source SSID byte should be 0x63", err);
        free(encoded);
    }
    return 0;
}

int test_frame_header_functions() {
    printf("test_frame_header_functions\n");
    uint8_t err = 0;

    // Test data: Header with destination "ABCDEF-7" and source "GHIJKL-1*"
    // Dest: 'A'<<1=0x82, 'B'<<1=0x84, 'C'<<1=0x86, 'D'<<1=0x88, 'E'<<1=0x8A, 'F'<<1=0x8C, SSID=7, ch=1, res0=1, res1=1, ext=0: 0xEE
    // Src:  'G'<<1=0x8E, 'H'<<1=0x90, 'I'<<1=0x92, 'J'<<1=0x94, 'K'<<1=0x96, 'L'<<1=0x98, SSID=1, ch=0, res0=1, res1=1, ext=1: 0x63
    uint8_t header_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63 };
    header_decode_result_t result = ax25_frame_header_decode(header_data, sizeof(header_data), &err);
    TEST_ASSERT(result.header != NULL, "ax25_frame_header_decode should return non-NULL header", err);
    if (result.header) {
        // Verify all fields
        TEST_ASSERT(strcmp(result.header->destination.callsign, "ABCDEF") == 0, "Destination callsign should be ABCDEF", err);
        TEST_ASSERT(result.header->destination.ssid == 7, "Destination SSID should be 7", err);
        TEST_ASSERT(result.header->destination.ch == true, "Destination ch should be true", err);
        TEST_ASSERT(result.header->destination.res0 == true, "Destination res0 should be true", err);
        TEST_ASSERT(result.header->destination.res1 == true, "Destination res1 should be true", err);
        TEST_ASSERT(result.header->destination.extension == false, "Destination extension should be false", err);

        TEST_ASSERT(strcmp(result.header->source.callsign, "GHIJKL") == 0, "Source callsign should be GHIJKL", err);
        TEST_ASSERT(result.header->source.ssid == 1, "Source SSID should be 1", err);
        TEST_ASSERT(result.header->source.ch == false, "Source ch should be false", err);
        TEST_ASSERT(result.header->source.res0 == true, "Source res0 should be true", err);
        TEST_ASSERT(result.header->source.res1 == true, "Source res1 should be true", err);
        TEST_ASSERT(result.header->source.extension == true, "Source extension should be true", err);

        TEST_ASSERT(result.header->cr == true, "cr should be true (dest ch=1, src ch=0)", err);
        TEST_ASSERT(result.header->src_cr == false, "src_cr should be false", err);
        TEST_ASSERT(result.header->repeaters.num_repeaters == 0, "No repeaters expected", err);

        size_t len;
        uint8_t *encoded = ax25_frame_header_encode(result.header, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_frame_header_encode should return non-NULL", err);
        TEST_ASSERT(len == sizeof(header_data), "Encoded header length should match input", err);
        COMPARE_FRAME(encoded, len, header_data, sizeof(header_data), "Header re-encoding should match");
        free(encoded);
        ax25_frame_header_free(result.header, &err);
    }
    return 0;
}

int test_frame_functions() {
    printf("test_frame_functions\n");
    uint8_t err = 0;

    // Test data: UI frame with dest "ABCDEF-7", src "GHIJKL-1*", control=0x03, PID=0xF0, payload="TEST"
    uint8_t frame_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63, 0x03, 0xF0, 'T', 'E', 'S', 'T' };
    ax25_frame_t *frame = ax25_frame_decode(frame_data, sizeof(frame_data), 0, &err);
    TEST_ASSERT(frame != NULL, "ax25_frame_decode should return non-NULL", err);
    if (frame) {
        TEST_ASSERT(frame->type == AX25_FRAME_UNNUMBERED_INFORMATION, "Frame type should be UI", err);
        ax25_unnumbered_information_frame_t *ui_frame = (ax25_unnumbered_information_frame_t*) frame;
        TEST_ASSERT(strcmp(ui_frame->base.base.header.destination.callsign, "ABCDEF") == 0, "Destination callsign should be ABCDEF", err);
        TEST_ASSERT(ui_frame->base.base.header.destination.ssid == 7, "Destination SSID should be 7", err);
        TEST_ASSERT(ui_frame->base.base.header.source.ssid == 1, "Source SSID should be 1", err);
        TEST_ASSERT(ui_frame->base.pf == false, "Poll/Final should be false", err);
        TEST_ASSERT(ui_frame->base.modifier == 0x03, "Modifier should be 0x03", err);
        TEST_ASSERT(ui_frame->pid == 0xF0, "PID should be 0xF0", err);
        TEST_ASSERT(ui_frame->payload_len == 4, "Payload length should be 4", err);
        TEST_ASSERT(memcmp(ui_frame->payload, "TEST", 4) == 0, "Payload should be 'TEST'", err);

        size_t len;
        uint8_t *encoded = ax25_frame_encode(frame, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_frame_encode should return non-NULL", err);
        COMPARE_FRAME(encoded, len, frame_data, sizeof(frame_data), "Frame re-encoding should match");
        free(encoded);
        ax25_frame_free(frame, &err);
    }
    return 0;
}

int test_raw_frame_functions() {
    printf("test_raw_frame_functions\n");
    uint8_t err = 0;

    // Test data: Raw frame with header and payload, control byte set to 0x00 (I-frame)
    uint8_t frame_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63, 0x00, 0xF0, 'T', 'E', 'S', 'T' };
    ax25_frame_t *frame = ax25_frame_decode(frame_data, sizeof(frame_data), MODULO128_NONE, &err);
    TEST_ASSERT(frame != NULL, "ax25_frame_decode should return non-NULL", err);
    if (frame) {
        TEST_ASSERT(frame->type == AX25_FRAME_RAW, "Frame type should be RAW", err);
        ax25_raw_frame_t *raw_frame = (ax25_raw_frame_t*) frame;
        TEST_ASSERT(raw_frame->control == 0x00, "Control should be 0x00", err);
        TEST_ASSERT(raw_frame->payload_len == 5, "Payload length should be 5", err);
        TEST_ASSERT(memcmp(raw_frame->payload, "\xF0TEST", 5) == 0, "Payload should be 0xF0 followed by 'TEST'", err);

        size_t len;
        uint8_t *encoded = ax25_raw_frame_encode(raw_frame, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_raw_frame_encode should return non-NULL", err);
        TEST_ASSERT(len == 6, "Encoded length should be 6 (control + payload)", err);
        TEST_ASSERT(memcmp(encoded, "\x00\xF0TEST", 6) == 0, "Encoded raw frame should match control + payload", err);
        free(encoded);
        ax25_frame_free(frame, &err);
    }
    return 0;
}

int test_unnumbered_frame_functions() {
    printf("test_unnumbered_frame_functions\n");
    uint8_t err = 0;

    // Test data: Header for "ABCDEF-7" -> "GHIJKL-1*"
    uint8_t header_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63 };
    ax25_frame_header_t *header = ax25_frame_header_decode(header_data, sizeof(header_data), &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header) {
        // Test UI frame with PID=0xF0, payload="TEST"
        uint8_t dummy_info_field[] = { 0xF0, 'T', 'E', 'S', 'T' };
        ax25_unnumbered_frame_t *u_frame = ax25_unnumbered_frame_decode(header, 0x13, dummy_info_field, sizeof(dummy_info_field), &err);  // 0x13 = UI with P/F=1
        TEST_ASSERT(u_frame != NULL, "ax25_unnumbered_frame_decode should return non-NULL", err);
        if (u_frame) {
            TEST_ASSERT(u_frame->base.type == AX25_FRAME_UNNUMBERED_INFORMATION, "Frame type should be UI", err);
            ax25_unnumbered_information_frame_t *ui_frame = (ax25_unnumbered_information_frame_t*) u_frame;
            TEST_ASSERT(ui_frame->base.pf == true, "Poll/Final should be true", err);
            TEST_ASSERT(ui_frame->base.modifier == 0x03, "Modifier should be 0x03", err);
            TEST_ASSERT(ui_frame->pid == 0xF0, "PID should be 0xF0", err);
            TEST_ASSERT(ui_frame->payload_len == 4, "Payload length should be 4", err);
            TEST_ASSERT(memcmp(ui_frame->payload, "TEST", 4) == 0, "Payload should be 'TEST'", err);

            size_t len;
            uint8_t *encoded = ax25_unnumbered_information_frame_encode(ui_frame, &len, &err);
            TEST_ASSERT(encoded != NULL, "ax25_unnumbered_information_frame_encode should return non-NULL", err);
            uint8_t expected[] = { 0x13, 0xF0, 'T', 'E', 'S', 'T' };
            COMPARE_FRAME(encoded, len, expected, sizeof(expected), "Encoded UI frame should match");
            free(encoded);
            ax25_frame_free((ax25_frame_t*) u_frame, &err);
        }
        ax25_frame_header_free(header, &err);
    }
    return 0;
}

int test_unnumbered_information_frame_functions() {
    printf("test_unnumbered_information_frame_functions\n");
    uint8_t err = 0;

    // Test data: Header for "ABCDEF-7" -> "GHIJKL-1*"
    uint8_t header_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63 };
    ax25_frame_header_t *header = ax25_frame_header_decode(header_data, sizeof(header_data), &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header) {
        uint8_t info[] = { 0xF0, 'T', 'E', 'S', 'T' };
        ax25_unnumbered_information_frame_t *ui_frame = ax25_unnumbered_information_frame_decode(header, true, info, sizeof(info), &err);
        TEST_ASSERT(ui_frame != NULL, "ax25_unnumbered_information_frame_decode should return non-NULL", err);
        if (ui_frame) {
            TEST_ASSERT(ui_frame->base.pf == true, "Poll/Final should be true", err);
            TEST_ASSERT(ui_frame->base.modifier == 0x03, "Modifier should be 0x03", err);
            TEST_ASSERT(ui_frame->pid == 0xF0, "PID should be 0xF0", err);
            TEST_ASSERT(ui_frame->payload_len == 4, "Payload length should be 4", err);
            TEST_ASSERT(memcmp(ui_frame->payload, "TEST", 4) == 0, "Payload should be 'TEST'", err);

            size_t len;
            uint8_t *encoded = ax25_unnumbered_information_frame_encode(ui_frame, &len, &err);
            TEST_ASSERT(encoded != NULL, "ax25_unnumbered_information_frame_encode should return non-NULL", err);
            uint8_t expected[] = { 0x13, 0xF0, 'T', 'E', 'S', 'T' };
            COMPARE_FRAME(encoded, len, expected, sizeof(expected), "Encoded UI frame should match");
            free(encoded);
            ax25_frame_free((ax25_frame_t*) ui_frame, &err);
        }
        ax25_frame_header_free(header, &err);
    }
    return 0;
}

int test_frame_reject_frame_functions() {
    printf("test_frame_reject_frame_functions\n");
    uint8_t err = 0;

    // Test data: Header for "AAAAAA-0" -> "BBBBBB-0"
    uint8_t header_data[] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x60, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x61 };
    ax25_frame_header_t *header = ax25_frame_header_decode(header_data, sizeof(header_data), &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header) {
        // FRMR data: w=1, x=0, y=0, z=0, vr=0, frmr_cr=0, vs=2, frmr_control=0x0A
        uint8_t frmr_data[] = { 0x0A, 0x04, 0x01 };
        ax25_frame_reject_frame_t *frmr_frame = ax25_frame_reject_frame_decode(header, false, frmr_data, sizeof(frmr_data), &err);
        TEST_ASSERT(frmr_frame != NULL, "ax25_frame_reject_frame_decode should return non-NULL", err);
        if (frmr_frame) {
            TEST_ASSERT(frmr_frame->base.pf == false, "Poll/Final should be false", err);
            TEST_ASSERT(frmr_frame->base.modifier == 0x87, "Modifier should be 0x87", err);
            TEST_ASSERT(frmr_frame->w == true, "w should be true", err);
            TEST_ASSERT(frmr_frame->x == false, "x should be false", err);
            TEST_ASSERT(frmr_frame->y == false, "y should be false", err);
            TEST_ASSERT(frmr_frame->z == false, "z should be false", err);
            TEST_ASSERT(frmr_frame->vr == 0, "vr should be 0", err);
            TEST_ASSERT(frmr_frame->frmr_cr == false, "frmr_cr should be false", err);
            TEST_ASSERT(frmr_frame->vs == 2, "vs should be 2", err);
            TEST_ASSERT(frmr_frame->frmr_control == 0x0A, "frmr_control should be 0x0A", err);

            size_t len;
            uint8_t *encoded = ax25_frame_reject_frame_encode(frmr_frame, &len, &err);
            TEST_ASSERT(encoded != NULL, "ax25_frame_reject_frame_encode should return non-NULL", err);
            uint8_t expected[] = { 0x87, 0x0A, 0x04, 0x01 };
            COMPARE_FRAME(encoded, len, expected, sizeof(expected), "Encoded FRMR frame should match");
            free(encoded);
            ax25_frame_free((ax25_frame_t*) frmr_frame, &err);
        }
        ax25_frame_header_free(header, &err);
    }
    return 0;
}

int test_information_frame_functions() {
    printf("test_frame_reject_frame_functions\n");
    uint8_t err = 0;

    // Test data: Header for "ABCDEF-7" -> "GHIJKL-1*"
    uint8_t header_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63 };
    ax25_frame_header_t *header = ax25_frame_header_decode(header_data, sizeof(header_data), &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header) {
        // I-frame data: control=0x10 (nr=0, pf=1, ns=0), PID=0xF0, payload="TEST"
        uint8_t info[] = { 0xF0, 'T', 'E', 'S', 'T' };
        ax25_information_frame_t *i_frame = ax25_information_frame_decode(header, 0x10, info, sizeof(info), false, &err);
        TEST_ASSERT(i_frame != NULL, "ax25_information_frame_decode should return non-NULL", err);
        if (i_frame) {
            TEST_ASSERT(i_frame->base.type == AX25_FRAME_INFORMATION_8BIT, "Frame type should be 8-bit I-frame", err);
            TEST_ASSERT(i_frame->nr == 0, "nr should be 0", err);
            TEST_ASSERT(i_frame->pf == true, "Poll/Final should be true", err);
            TEST_ASSERT(i_frame->ns == 0, "ns should be 0", err);
            TEST_ASSERT(i_frame->pid == 0xF0, "PID should be 0xF0", err);
            TEST_ASSERT(i_frame->payload_len == 4, "Payload length should be 4", err);
            TEST_ASSERT(memcmp(i_frame->payload, "TEST", 4) == 0, "Payload should be 'TEST'", err);

            size_t len;
            uint8_t *encoded = ax25_information_frame_encode(i_frame, &len, &err);
            TEST_ASSERT(encoded != NULL, "ax25_information_frame_encode should return non-NULL", err);
            uint8_t expected[] = { 0x10, 0xF0, 'T', 'E', 'S', 'T' };
            COMPARE_FRAME(encoded, len, expected, sizeof(expected), "Encoded I-frame should match");
            free(encoded);
            ax25_frame_free((ax25_frame_t*) i_frame, &err);
        }
        ax25_frame_header_free(header, &err);
    }
    return 0;
}

int test_supervisory_frame_functions() {
    printf("test_supervisory_frame_functions\n");
    uint8_t err = 0;

    ax25_frame_header_t *header = ax25_frame_header_decode((uint8_t[] ) { 0x82, 0xA0, 0xA4, 0xA6, 0x40, 0x40, 0xE0, 0x9C, 0x9E, 0x86, 0x82, 0x98, 0x98, 0xE1 },
            14, &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header == NULL)
        return 1;

    ax25_supervisory_frame_t *s_frame = ax25_supervisory_frame_decode(header, 0x21, false, &err);  // RR with nr=1
    TEST_ASSERT(s_frame != NULL, "ax25_supervisory_frame_decode should return non-NULL", err);
    if (s_frame) {
        TEST_ASSERT(s_frame->nr == 1, "nr should be 1", err);  // Explicitly test nr
        TEST_ASSERT(s_frame->code == 0x00, "Supervisory code should be 0x00 (RR)", err);
        TEST_ASSERT(s_frame->pf == false, "Poll/Final bit should be false", err);
        TEST_ASSERT(s_frame->base.type == AX25_FRAME_SUPERVISORY_RR_8BIT, "Frame type should be RR_8BIT", err);

        size_t len;
        uint8_t *encoded = ax25_supervisory_frame_encode(s_frame, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_supervisory_frame_encode should return non-NULL", err);
        TEST_ASSERT(len == 1, "Encoded length should be 1 byte", err);
        TEST_ASSERT(encoded[0] == 0x21, "Encoded control byte should be 0x21", err);
        if (encoded)
            free(encoded);
        ax25_frame_free((ax25_frame_t*) s_frame, &err);
    }
    ax25_frame_header_free(header, &err);
    TEST_ASSERT(err == 0, "Freeing header", err);
    return err ? 1 : 0;
}

int test_xid_parameter_functions() {
    printf("test_xid_parameter_functions\n");
    uint8_t err = 0;

    uint8_t pv[] = { 0x01, 0x02 };
    ax25_xid_parameter_t *param = ax25_xid_raw_parameter_new(1, pv, 2, &err);
    TEST_ASSERT(param != NULL, "ax25_xid_raw_parameter_new should return non-NULL", err);
    if (param) {
        size_t len;
        uint8_t *encoded = ax25_xid_raw_parameter_encode(param, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_xid_raw_parameter_encode should return non-NULL", err);
        if (encoded) {
            size_t consumed;
            ax25_xid_parameter_t *decoded = ax25_xid_parameter_decode(encoded, len, &consumed, &err);
            TEST_ASSERT(decoded != NULL, "ax25_xid_parameter_decode should return non-NULL", err);
            if (decoded)
                ax25_xid_raw_parameter_free(decoded, &err);
            free(encoded);
        }
        ax25_xid_parameter_t *copy = ax25_xid_raw_parameter_copy(param, &err);
        TEST_ASSERT(copy != NULL, "ax25_xid_raw_parameter_copy should return non-NULL", err);
        if (copy)
            ax25_xid_raw_parameter_free(copy, &err);
        ax25_xid_raw_parameter_free(param, &err);
    }

    param = ax25_xid_class_of_procedures_new(true, false, true, false, false, true, false, 0, &err);
    TEST_ASSERT(param != NULL, "ax25_xid_class_of_procedures_new should return non-NULL", err);
    if (param) {
        size_t len;
        uint8_t *encoded = ax25_xid_raw_parameter_encode(param, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_xid_raw_parameter_encode should return non-NULL", err);
        if (encoded) {
            uint8_t expected[] = { 0x01, 0x02, 0x25, 0x00 };  // PI=1, PL=2, PV=0x25,0x00
            size_t expected_len = sizeof(expected);
            COMPARE_FRAME(encoded, len, expected, expected_len, "Class of Procedures parameter encoding");
            free(encoded);
        }
        ax25_xid_raw_parameter_free(param, &err);
    }

    param = ax25_xid_hdlc_optional_functions_new(true, false, true, false, true, false, true, false, true,
    false, false, false, false, false, false, false, false,
    false, false, false, false, 0, false, &err);
    TEST_ASSERT(param != NULL, "ax25_xid_hdlc_optional_functions_new should return non-NULL", err);
    if (param)
        ax25_xid_raw_parameter_free(param, &err);

    param = ax25_xid_big_endian_new(1, 0x12345678, 4, &err);
    TEST_ASSERT(param != NULL, "ax25_xid_big_endian_new should return non-NULL", err);
    if (param)
        ax25_xid_raw_parameter_free(param, &err);

    ax25_xid_init_defaults(&err);  // No return value to check
    ax25_xid_deinit_defaults(&err);
    printf("\033[0;32m[%04d]    PASS: ax25_xid_init_defaults executed\033[0m\n", ++assert_count);
    return 0;
}

int test_exchange_identification_frame_functions() {
    printf("test_exchange_identification_frame_functions\n");
    uint8_t err = 0;

    // Test data: Header for "ABCDEF-7" -> "GHIJKL-1*"
    uint8_t header_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63 };
    ax25_frame_header_t *header = ax25_frame_header_decode(header_data, sizeof(header_data), &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header) {
        // XID data: FI=0x82, GI=0x80, GL=0x0004, param PI=0x01, PL=0x02, PV={0x41, 0x00}
        uint8_t data[] = { 0x82, 0x80, 0x00, 0x04, 0x01, 0x02, 0x41, 0x00 };
        ax25_exchange_identification_frame_t *xid_frame = ax25_exchange_identification_frame_decode(header, true, data, sizeof(data), &err);
        TEST_ASSERT(xid_frame != NULL, "ax25_exchange_identification_frame_decode should return non-NULL", err);
        if (xid_frame) {
            TEST_ASSERT(xid_frame->base.pf == true, "Poll/Final should be true", err);
            TEST_ASSERT(xid_frame->base.modifier == 0xAF, "Modifier should be 0xAF", err);
            TEST_ASSERT(xid_frame->fi == 0x82, "FI should be 0x82", err);
            TEST_ASSERT(xid_frame->gi == 0x80, "GI should be 0x80", err);
            TEST_ASSERT(xid_frame->param_count == 1, "Should have 1 parameter", err);
            if (xid_frame->param_count > 0) {
                TEST_ASSERT(xid_frame->parameters[0]->pi == 0x01, "Parameter PI should be 0x01", err);
                ax25_raw_param_data_t *param_data = (ax25_raw_param_data_t*) xid_frame->parameters[0]->data;
                TEST_ASSERT(param_data->pv_len == 2, "Parameter PV length should be 2", err);
                TEST_ASSERT(memcmp(param_data->pv, "\x41\x00", 2) == 0, "Parameter PV should be {0x41, 0x00}", err);
            }

            size_t len;
            uint8_t *encoded = ax25_exchange_identification_frame_encode(xid_frame, &len, &err);
            TEST_ASSERT(encoded != NULL, "ax25_exchange_identification_frame_encode should return non-NULL", err);
            uint8_t expected[] = { 0xBF, 0x82, 0x80, 0x00, 0x04, 0x01, 0x02, 0x41, 0x00 };
            COMPARE_FRAME(encoded, len, expected, sizeof(expected), "Encoded XID frame should match");
            free(encoded);
            ax25_frame_free((ax25_frame_t*) xid_frame, &err);
        }
        ax25_frame_header_free(header, &err);
    }
    return 0;
}

int test_test_frame_functions() {
    printf("test_test_frame_functions\n");
    uint8_t err = 0;

    ax25_frame_header_t *header = ax25_frame_header_decode((uint8_t[] ) { 0x82, 0xA0, 0xA4, 0xA6, 0x40, 0x40, 0xE0, 0x9C, 0x9E, 0x86, 0x82, 0x98, 0x98, 0xE1 },
            14, &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header == NULL)
        return 1;

    uint8_t data[] = "TEST";
    ax25_test_frame_t *test_frame = ax25_test_frame_decode(header, true, data, 4, &err);
    TEST_ASSERT(test_frame != NULL, "ax25_test_frame_decode should return non-NULL", err);
    if (test_frame) {
        TEST_ASSERT(test_frame->payload_len == 4, "Payload length should be 4", err);
        TEST_ASSERT(memcmp(test_frame->payload, data, 4) == 0, "Payload should match 'TEST'", err);
        size_t len;
        uint8_t *encoded = ax25_test_frame_encode(test_frame, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_test_frame_encode should return non-NULL", err);
        if (encoded) {
            uint8_t expected[] = { 0xF3, 'T', 'E', 'S', 'T' };  // Control byte (0xE3 | POLL_FINAL_8BIT) + "TEST"
            size_t expected_len = sizeof(expected);
            COMPARE_FRAME(encoded, len, expected, expected_len, "Encoded TEST frame content should match");
            free(encoded);
        }
        ax25_frame_free((ax25_frame_t*) test_frame, &err);
        TEST_ASSERT(err == 0, "Freeing TEST frame", err);
    }
    ax25_frame_header_free(header, &err);
    TEST_ASSERT(err == 0, "Freeing header", err);
    return err ? 1 : 0;
}

int test_ax25_modulo128(void) {
    printf("test_ax25_modulo128\n");
    uint8_t err = 0;
    ax25_frame_t *frame;

    // Modulo-128 RR frame: N(R)=4, P/F=0
    uint8_t ax25_rr_frame_mod128[] = { 0x9C, 0x9E, 0x86, 0x82, 0x98, 0x98, 0xE0,  // Dest: NOCALL-0, ch=1
            0xA6, 0x8A, 0xA0, 0x8A, 0x82, 0xA2, 0x63,  // Src: REPEAT-1, ch=0, extension=1
            0x01, 0x08  // Control: First byte 0x01 (S=00, 01), Second byte 0x08 (N(R)=4, P/F=0)
            };
    size_t ax25_rr_frame_mod128_len = sizeof(ax25_rr_frame_mod128);

    frame = ax25_frame_decode(ax25_rr_frame_mod128, ax25_rr_frame_mod128_len, 1, &err);
    TEST_ASSERT(frame != NULL, "Decoding modulo-128 RR frame", err);

    if (frame) {
        TEST_ASSERT(frame->type == AX25_FRAME_SUPERVISORY_RR_16BIT, "Frame type should be RR 16-bit", err);
        ax25_supervisory_frame_t *s_frame = (ax25_supervisory_frame_t*) frame;
        TEST_ASSERT(s_frame->nr == 4, "nr should be 4", err);
        TEST_ASSERT(s_frame->pf == false, "Poll/Final should be false", err);
        TEST_ASSERT(s_frame->code == 0x00, "Code should be 0x00 (RR)", err);
        ax25_frame_free(frame, &err);
    }

    return 0;
}

int test_ax25_modulo128_encode() {
    printf("test_ax25_modulo128_encode\n");
    uint8_t err = 0;

    // Create addresses
    ax25_address_t *dest = ax25_address_from_string("NOCALL-0", &err);
    TEST_ASSERT(dest != NULL, "Destination address creation should succeed", err);
    ax25_address_t *src = ax25_address_from_string("REPEAT-1", &err);
    TEST_ASSERT(src != NULL, "Source address creation should succeed", err);
    if (!dest || !src)
        return 1;

    // Adjust source address for modulo-128: res1 = false
    src->res1 = false;

    // Create modulo-128 I-frame: N(S)=5, N(R)=3, P/F=1, PID=0xF0, Payload="TEST"
    ax25_information_frame_t *i_frame = malloc(sizeof(ax25_information_frame_t));
    TEST_ASSERT(i_frame != NULL, "I-frame allocation should succeed", err);
    if (!i_frame) {
        ax25_address_free(dest, &err);
        ax25_address_free(src, &err);
        return 1;
    }

    i_frame->base.type = AX25_FRAME_INFORMATION_16BIT;
    i_frame->base.header.destination = *dest;
    i_frame->base.header.source = *src;
    i_frame->base.header.cr = true;  // Command frame
    i_frame->base.header.src_cr = false;
    i_frame->base.header.repeaters.num_repeaters = 0;
    i_frame->nr = 3;
    i_frame->pf = true;
    i_frame->ns = 5;
    i_frame->pid = 0xF0;
    i_frame->payload_len = 4;
    i_frame->payload = malloc(4);
    TEST_ASSERT(i_frame->payload != NULL, "Payload allocation should succeed", err);
    if (!i_frame->payload) {
        free(i_frame);
        ax25_address_free(dest, &err);
        ax25_address_free(src, &err);
        return 1;
    }
    memcpy(i_frame->payload, "TEST", 4);

    // Encode the frame
    size_t len;
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) i_frame, &len, &err);
    TEST_ASSERT(encoded != NULL, "Frame encoding should succeed", err);
    if (encoded) {
        // Expected frame:
        // Dest: NOCALL-0, ch=1, res0=1, res1=1, ext=0: 0x9C, 0x9E, 0x86, 0x82, 0x98, 0x98, 0xE0
        // Src:  REPEAT-1, ch=0, res0=1, res1=0, ext=1: 0xA6, 0x8A, 0xA0, 0x8A, 0x82, 0xA2, 0x23
        // Control: 0x0A (N(S)=5, I=0), 0x07 (N(R)=3, P/F=1)
        // PID: 0xF0, Payload: "TEST"
        uint8_t expected[] = { 0x9C, 0x9E, 0x86, 0x82, 0x98, 0x98, 0xE0, 0xA6, 0x8A, 0xA0, 0x8A, 0x82, 0xA2, 0x23, 0x0A, 0x07, 0xF0, 'T', 'E', 'S', 'T' };
        size_t expected_len = sizeof(expected);
        COMPARE_FRAME(encoded, len, expected, expected_len, "Modulo-128 I-frame encoding should match expected bytes");
        TEST_ASSERT((encoded[13] & 0x40) == 0, "Source SSID bit 6 (res1) should be 0 for modulo-128", err);
        free(encoded);
    }

    // Clean up
    free(i_frame->payload);
    free(i_frame);
    ax25_address_free(dest, &err);
    ax25_address_free(src, &err);
    return 0;
}

int test_ax25_connection(void) {
    printf("test_ax25_connection\n");
    uint8_t err = 0;

    // AX.25 Connection Test Packets
    // 1. CONNECT Request (Station A -> Station B: SABM)
    // Dest: VA3BBB-7 (C=1, ext=0), Src: VA3AAA-1 (C=0, ext=1), Control: 0x3F (SABM, P=1)
    unsigned char ax25_sabm_packet[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x3F };
    size_t ax25_sabm_packet_len = sizeof(ax25_sabm_packet);

    // 2. CONNECT Acknowledgment (Station B -> Station A: UA)
    // Dest: VA3AAA-1 (C=0, ext=0), Src: VA3BBB-7 (C=1, ext=1), Control: 0x73 (UA, F=1)
    unsigned char ax25_ua_connect_packet[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x62, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEF, 0x73 };
    size_t ax25_ua_connect_packet_len = sizeof(ax25_ua_connect_packet);

    // 3. SEND Data (Station A -> Station B: I-Frame)
    // Dest: VA3BBB-7, Src: VA3AAA-1, Control: 0x00 (I, N(S)=0, N(R)=0), PID: 0xF0, Payload: "Hello, World!"
    unsigned char ax25_i_frame_packet[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x00, 0xF0, 0x48, 0x65, 0x6C,
            0x6C, 0x6F, 0x2C, 0x20, 0x57, 0x6F, 0x72, 0x6C, 0x64, 0x21 };
    size_t ax25_i_frame_packet_len = sizeof(ax25_i_frame_packet);

    // 4. RECEIVE Data Acknowledgment (Station B -> Station A: RR)
    // Dest: VA3AAA-1, Src: VA3BBB-7, Control: 0x31 (RR, N(R)=1, P/F=1)
    unsigned char ax25_rr_packet[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x62, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEF, 0x31 };
    size_t ax25_rr_packet_len = sizeof(ax25_rr_packet);

    // 5. DISCONNECT Request (Station A -> Station B: DISC)
    // Dest: VA3BBB-7, Src: VA3AAA-1, Control: 0x43 (DISC, P=0)
    unsigned char ax25_disc_packet[] = {  //
            0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x43 };
    size_t ax25_disc_packet_len = sizeof(ax25_disc_packet);

    // 6. DISCONNECT Acknowledgment (Station B -> Station A: UA)
    // Dest: VA3AAA-1, Src: VA3BBB-7, Control: 0x73 (UA, F=1)
    unsigned char ax25_ua_disconnect_packet[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x62, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEF, 0x73 };
    size_t ax25_ua_disconnect_packet_len = sizeof(ax25_ua_disconnect_packet);

    // Invalid packet for error testing
    unsigned char invalid_packet[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0xFF };
    size_t invalid_packet_len = sizeof(invalid_packet);

    unsigned char short_packet[] = { 0xAC, 0x82, 0x66 };
    size_t short_packet_len = sizeof(short_packet);

    // Initialize addresses
    uint8_t addr_err;
    ax25_address_t *station_a = ax25_address_from_string("VA3AAA-1", &addr_err);
    TEST_ASSERT(station_a != NULL && addr_err == 0, "Create VA3AAA-1 address", addr_err);
    ax25_address_t *station_b = ax25_address_from_string("VA3BBB-7", &addr_err);
    TEST_ASSERT(station_b != NULL && addr_err == 0, "Create VA3BBB-7 address", addr_err);

    // Buffer for encoded frames
    size_t encoded_len;
    ax25_frame_t *decoded_frame;
    uint8_t *encode_result;

    // 1. Test SABM frame
    decoded_frame = ax25_frame_decode(ax25_sabm_packet, ax25_sabm_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding SABM frame", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_SABM, "Frame type should be SABM", err);
        ax25_unnumbered_frame_t *u_frame = (ax25_unnumbered_frame_t*) decoded_frame;
        TEST_ASSERT(u_frame->pf == true, "Poll/Final should be true", err);
        TEST_ASSERT(u_frame->modifier == 0x2F, "Modifier should be 0x2F", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding SABM frame", err);
        COMPARE_FRAME(encode_result, encoded_len, ax25_sabm_packet, ax25_sabm_packet_len, "SABM frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 2. Test UA connect frame
    decoded_frame = ax25_frame_decode(ax25_ua_connect_packet, ax25_ua_connect_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding UA connect frame", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_UA, "Frame type should be UA", err);
        ax25_unnumbered_frame_t *u_frame = (ax25_unnumbered_frame_t*) decoded_frame;
        TEST_ASSERT(u_frame->pf == true, "Poll/Final should be true", err);
        TEST_ASSERT(u_frame->modifier == 0x63, "Modifier should be 0x63", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding UA connect frame", err);
        COMPARE_FRAME(encode_result, encoded_len, ax25_ua_connect_packet, ax25_ua_connect_packet_len, "UA connect frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 3. Test I-Frame
    decoded_frame = ax25_frame_decode(ax25_i_frame_packet, ax25_i_frame_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding I-Frame", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_INFORMATION_8BIT, "Frame type should be I-frame 8-bit", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) decoded_frame;
        TEST_ASSERT(i_frame->nr == 0, "nr should be 0", err);
        TEST_ASSERT(i_frame->ns == 0, "ns should be 0", err);
        TEST_ASSERT(i_frame->pf == false, "Poll/Final should be false", err);
        TEST_ASSERT(i_frame->pid == 0xF0, "PID should be 0xF0", err);
        TEST_ASSERT(i_frame->payload_len == 13, "Payload length should be 13", err);
        TEST_ASSERT(memcmp(i_frame->payload, "Hello, World!", 13) == 0, "Payload should be 'Hello, World!'", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding I-Frame", err);
        COMPARE_FRAME(encode_result, encoded_len, ax25_i_frame_packet, ax25_i_frame_packet_len, "I-Frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 4. Test RR frame
    decoded_frame = ax25_frame_decode(ax25_rr_packet, ax25_rr_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding RR frame", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_SUPERVISORY_RR_8BIT, "Frame type should be RR 8-bit", err);
        ax25_supervisory_frame_t *s_frame = (ax25_supervisory_frame_t*) decoded_frame;
        TEST_ASSERT(s_frame->nr == 1, "nr should be 1", err);
        TEST_ASSERT(s_frame->pf == true, "Poll/Final should be true", err);  // Changed from false to true
        TEST_ASSERT(s_frame->code == 0x00, "Code should be 0x00 (RR)", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding RR frame", err);
        COMPARE_FRAME(encode_result, encoded_len, ax25_rr_packet, ax25_rr_packet_len, "RR frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 5. Test DISC frame
    decoded_frame = ax25_frame_decode(ax25_disc_packet, ax25_disc_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding DISC frame", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_DISC, "Frame type should be DISC", err);
        ax25_unnumbered_frame_t *u_frame = (ax25_unnumbered_frame_t*) decoded_frame;
        TEST_ASSERT(u_frame->pf == false, "Poll/Final should be false", err);
        TEST_ASSERT(u_frame->modifier == 0x43, "Modifier should be 0x43", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding DISC frame", err);
        COMPARE_FRAME(encode_result, encoded_len, ax25_disc_packet, ax25_disc_packet_len, "DISC frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 6. Test UA disconnect frame
    decoded_frame = ax25_frame_decode(ax25_ua_disconnect_packet, ax25_ua_disconnect_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding UA disconnect frame", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_UA, "Frame type should be UA", err);
        ax25_unnumbered_frame_t *u_frame = (ax25_unnumbered_frame_t*) decoded_frame;
        TEST_ASSERT(u_frame->pf == true, "Poll/Final should be true", err);
        TEST_ASSERT(u_frame->modifier == 0x63, "Modifier should be 0x63", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding UA disconnect frame", err);
        COMPARE_FRAME(encode_result, encoded_len, ax25_ua_disconnect_packet, ax25_ua_disconnect_packet_len, "UA disconnect frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 7. Error Case: Invalid control byte
    decoded_frame = ax25_frame_decode(invalid_packet, invalid_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame == NULL && err != 0, "Decoding invalid control frame should fail", err);

    // 8. Error Case: Short frame
    decoded_frame = ax25_frame_decode(short_packet, short_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame == NULL && err != 0, "Decoding short frame should fail", err);

    // 9. Error Case: Null input
    decoded_frame = ax25_frame_decode(NULL, 0, 0, &err);
    TEST_ASSERT(decoded_frame == NULL && err != 0, "Decoding null input should fail", err);

    // Clean up addresses
    ax25_address_free(station_a, &addr_err);
    ax25_address_free(station_b, &addr_err);
    return 0;
}

int test_frmr_frame_functions() {
    printf("test_frmr_frame_functions\n");
    uint8_t err = 0;

    // Define the modulo-8 FRMR frame components
    uint8_t header_mod8[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE,  // Dest: ABCDEF-7
            0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63 };  // Src: GHIJKL-1, res1=1
    uint8_t control_byte = 0x87;  // FRMR control byte
    uint8_t frmr_data_mod8[] = { 0x10, 0x24, 0x01 };  // FRMR data

    // Calculate lengths
    size_t header_len = sizeof(header_mod8);       // 14 bytes
    size_t frmr_data_len = sizeof(frmr_data_mod8);  // 3 bytes
    size_t total_len = header_len + 1 + frmr_data_len;  // Header + control + data

    // Construct the frame
    uint8_t *frame_mod8 = malloc(total_len);
    memcpy(frame_mod8, header_mod8, header_len);
    frame_mod8[header_len] = control_byte;
    memcpy(frame_mod8 + header_len + 1, frmr_data_mod8, frmr_data_len);

    // Decode the frame
    ax25_frame_t *frame = ax25_frame_decode(frame_mod8, total_len, 0, &err);
    if (!frame) {
        printf("Decoding failed with error %d\n", err);
        free(frame_mod8);
        return 1;
    }

    // Verify the decoded FRMR frame
    ax25_frame_reject_frame_t *frmr = (ax25_frame_reject_frame_t*) frame;

    // Assertions
    if (frmr->base.base.type != AX25_FRAME_UNNUMBERED_FRMR || frmr->is_modulo128 || frmr->frmr_control != 0x10 || frmr->vr != 1 || frmr->vs != 2
            || frmr->frmr_cr || !frmr->w || frmr->x || frmr->y || frmr->z) {
        printf("Test failed: Expected control=0x10, vr=1, vs=2, cr=0, w=1, x=0, y=0, z=0\n");
        ax25_frame_free(frame, &err);
        free(frame_mod8);
        return 1;
    }

    // Clean up
    ax25_frame_free(frame, &err);
    free(frame_mod8);

    return 0;  // Success
}

int test_auto_modulo_detection() {
    printf("test_auto_modulo_detection\n");
    uint8_t err = 0;

    // Test modulo-8 I-frame
    uint8_t frame_mod8[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE,  // dest: ABCDEF-7, ch=1
            0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63,  // src: GHIJKL-1, ch=0, res1=1 (0x63)
            0x00,  // control: I-frame, nr=0, ns=0, pf=0
            0xF0, 'T', 'E', 'S', 'T'  // PID and payload
            };
    ax25_frame_t *frame = ax25_frame_decode(frame_mod8, sizeof(frame_mod8), MODULO128_AUTO, &err);
    TEST_ASSERT(frame != NULL, "Decoding modulo-8 I-frame with auto detection", err);
    if (frame) {
        TEST_ASSERT(frame->type == AX25_FRAME_INFORMATION_8BIT, "Should decode as 8-bit I-frame", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) frame;
        TEST_ASSERT(i_frame->nr == 0, "nr should be 0", err);
        TEST_ASSERT(i_frame->ns == 0, "ns should be 0", err);
        TEST_ASSERT(i_frame->pf == false, "pf should be false", err);
        ax25_frame_free(frame, &err);
    }

    // Test modulo-128 I-frame
    uint8_t frame_mod128[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE,  // dest: ABCDEF-7, ch=1
            0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x23,  // src: GHIJKL-1, ch=0, res1=0 (0x23)
            0x00, 0x00,  // control: 16-bit, nr=0, ns=0, pf=0
            0xF0, 'T', 'E', 'S', 'T'  // PID and payload
            };
    frame = ax25_frame_decode(frame_mod128, sizeof(frame_mod128), MODULO128_AUTO, &err);
    TEST_ASSERT(frame != NULL, "Decoding modulo-128 I-frame with auto detection", err);
    if (frame) {
        TEST_ASSERT(frame->type == AX25_FRAME_INFORMATION_16BIT, "Should decode as 16-bit I-frame", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) frame;
        TEST_ASSERT(i_frame->nr == 0, "nr should be 0", err);
        TEST_ASSERT(i_frame->ns == 0, "ns should be 0", err);
        TEST_ASSERT(i_frame->pf == false, "pf should be false", err);
        ax25_frame_free(frame, &err);
    }

    return 0;
}

int test_segmentation_reassembly() {
    printf("test_segmentation_reassembly\n");
    uint8_t err = 0;
    int result = 0;  // Track test result

    // Create a large payload (10000 bytes)
    size_t payload_len = 10000;
    uint8_t *payload = malloc(payload_len);
    if (!payload) {
        printf("\033[0;31m[%04d] FAIL(%u): Payload allocation should succeed\033[0m\n", ++assert_count, err);
        return 1;
    }
    for (size_t i = 0; i < payload_len; i++) {
        payload[i] = (uint8_t) (i % 256);  // Fill with 0x00, 0x01, ..., 0xFF, 0x00, ...
    }

    // Segment the payload with N1=256
    size_t n1 = 256;
    size_t num_segments;
    ax25_segmented_info_t *segments = ax25_segment_info_fields(payload, payload_len, n1, &err, &num_segments);
    if (!segments || err != 0) {
        result = 1;
        goto cleanup_payload;
    }

    // Check segment count
    if (num_segments != 40) {
        result = 1;
        goto cleanup_segments;
    }

    // Verify first segment
    if (segments[0].info_field_len != 256 || segments[0].info_field[0] != 0x08 || segments[0].info_field[1] != 0x80 || segments[0].info_field[2] != 0x27
            || segments[0].info_field[3] != 0x10 || memcmp(segments[0].info_field + 4, payload, 252) != 0) {
        result = 1;
        goto cleanup_segments;
    }

    // Verify second segment
    if (segments[1].info_field_len != 256 || segments[1].info_field[0] != 0x08 || segments[1].info_field[1] != 0x01
            || memcmp(segments[1].info_field + 2, payload + 252, 254) != 0) {
        result = 1;
        goto cleanup_segments;
    }

    // Verify last segment
    size_t last_seg = num_segments - 1;
    size_t last_data_len = payload_len - 252 - (num_segments - 2) * 254;  // 96 bytes
    size_t last_info_len = 2 + last_data_len;  // 98 bytes
    size_t offset = 252 + (last_seg - 1) * 254;
    if (segments[last_seg].info_field_len != last_info_len || segments[last_seg].info_field[0] != 0x08 || segments[last_seg].info_field[1] != 0x67
            || memcmp(segments[last_seg].info_field + 2, payload + offset, last_data_len) != 0) {
        result = 1;
        goto cleanup_segments;
    }

    // Calculate overhead
    size_t total_segment_bytes = 0;
    for (size_t i = 0; i < num_segments; i++) {
        total_segment_bytes += segments[i].info_field_len;
    }
    double overhead = (double) (total_segment_bytes - payload_len) / payload_len * 100;
    if (overhead >= 1.0) {
        result = 1;
        goto cleanup_segments;
    }

    // Reassemble segments
    size_t reassembled_len;
    uint8_t *reassembled = ax25_reassemble_info_fields(segments, num_segments, &reassembled_len, &err);
    if (!reassembled || err != 0 || reassembled_len != payload_len || memcmp(reassembled, payload, payload_len) != 0) {
        result = 1;
        free(reassembled);
        goto cleanup_segments;
    }

    free(reassembled);  // Free the reassembled buffer after use

    // Print final result only
    printf("\033[0;32m[%04d]    PASS: test_segmentation_reassembly completed successfully\033[0m\n", ++assert_count);

    cleanup_segments:
    ax25_free_segmented_info(segments, num_segments);
    cleanup_payload:
    free(payload);
    if (result != 0) {
        printf("\033[0;31m[%04d] FAIL(%u): test_segmentation_reassembly failed\033[0m\n", ++assert_count, err);
    }
    return result;
}

void test_ax25_frame_print() {
    printf("test_ax25_frame_print\n");
    // UI frame
    unsigned char ui_frame[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63, 0x03, 0xF0, 'T', 'E', 'S', 'T' };
    printf("UI Frame:\n");
    ax25_frame_print(ui_frame, sizeof(ui_frame));

    // I-frame
    unsigned char i_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x00, 0xF0, 'H', 'e', 'l', 'l', 'o', ',',
            ' ', 'W', 'o', 'r', 'l', 'd', '!' };
    printf("\nI-Frame:\n");
    ax25_frame_print(i_frame, sizeof(i_frame));

    // SABM frame
    unsigned char sabm_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x3F };
    printf("\nSABM Frame:\n");
    ax25_frame_print(sabm_frame, sizeof(sabm_frame));

    // UA frame
    unsigned char ua_frame[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x62, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEF, 0x73 };
    printf("\nUA Frame:\n");
    ax25_frame_print(ua_frame, sizeof(ua_frame));

    // RR frame
    unsigned char rr_frame[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x62, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEF, 0x31 };
    printf("\nRR Frame:\n");
    ax25_frame_print(rr_frame, sizeof(rr_frame));

    // DISC frame
    unsigned char disc_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x43 };
    printf("\nDISC Frame:\n");
    ax25_frame_print(disc_frame, sizeof(disc_frame));
}

void test_ax25_hdlc_frame_print() {
    printf("test_ax25_hdlc_frame_print\n");

    // UI frame
    unsigned char ui_frame[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63, 0x03, 0xF0, 'T', 'E', 'S', 'T' };
    int hdlc_len;
    unsigned char hdlc_ui_frame[512];
    printf("\n=== UI Frame Test ===\n");
    printf("  [DEBUG] Original AX.25 frame (%zu bytes): ", sizeof(ui_frame));
    for (size_t i = 0; i < sizeof(ui_frame); i++) {
        printf("%02X ", ui_frame[i]);
    }
    printf("\n");
    hdlc_frame_encode(ui_frame, sizeof(ui_frame), hdlc_ui_frame, &hdlc_len);
    printf("  [DEBUG] Encoded HDLC frame (%d bytes): ", hdlc_len);
    for (int i = 0; i < hdlc_len; i++) {
        printf("%02X ", hdlc_ui_frame[i]);
    }
    printf("\n");
    printf("HDLC UI Frame:\n");
    hdlc_frame_print(hdlc_ui_frame, hdlc_len);

    // I-frame
    unsigned char i_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x00, 0xF0, 'H', 'e', 'l', 'l', 'o', ',',
            ' ', 'W', 'o', 'r', 'l', 'd', '!' };
    unsigned char hdlc_i_frame[512];
    printf("\n=== I-Frame Test ===\n");
    printf("  [DEBUG] Original AX.25 frame (%zu bytes): ", sizeof(i_frame));
    for (size_t i = 0; i < sizeof(i_frame); i++) {
        printf("%02X ", i_frame[i]);
    }
    printf("\n");
    hdlc_frame_encode(i_frame, sizeof(i_frame), hdlc_i_frame, &hdlc_len);
    printf("  [DEBUG] Encoded HDLC frame (%d bytes): ", hdlc_len);
    for (int i = 0; i < hdlc_len; i++) {
        printf("%02X ", hdlc_i_frame[i]);
    }
    printf("\n");
    printf("\nHDLC I-Frame:\n");
    hdlc_frame_print(hdlc_i_frame, hdlc_len);

    // SABM frame
    unsigned char sabm_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x3F };
    unsigned char hdlc_sabm_frame[512];
    printf("\n=== SABM Frame Test ===\n");
    printf("  [DEBUG] Original AX.25 frame (%zu bytes): ", sizeof(sabm_frame));
    for (size_t i = 0; i < sizeof(sabm_frame); i++) {
        printf("%02X ", sabm_frame[i]);
    }
    printf("\n");
    hdlc_frame_encode(sabm_frame, sizeof(sabm_frame), hdlc_sabm_frame, &hdlc_len);
    printf("  [DEBUG] Encoded HDLC frame (%d bytes): ", hdlc_len);
    for (int i = 0; i < hdlc_len; i++) {
        printf("%02X ", hdlc_sabm_frame[i]);
    }
    printf("\n");
    printf("\nHDLC SABM Frame:\n");
    hdlc_frame_print(hdlc_sabm_frame, hdlc_len);

    // UA frame
    unsigned char ua_frame[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x62, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEF, 0x73 };
    unsigned char hdlc_ua_frame[512];
    printf("\n=== UA Frame Test ===\n");
    printf("  [DEBUG] Original AX.25 frame (%zu bytes): ", sizeof(ua_frame));
    for (size_t i = 0; i < sizeof(ua_frame); i++) {
        printf("%02X ", ua_frame[i]);
    }
    printf("\n");
    hdlc_frame_encode(ua_frame, sizeof(ua_frame), hdlc_ua_frame, &hdlc_len);
    printf("  [DEBUG] Encoded HDLC frame (%d bytes): ", hdlc_len);
    for (int i = 0; i < hdlc_len; i++) {
        printf("%02X ", hdlc_ua_frame[i]);
    }
    printf("\n");
    printf("\nHDLC UA Frame:\n");
    hdlc_frame_print(hdlc_ua_frame, hdlc_len);

    // RR frame
    unsigned char rr_frame[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x62, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEF, 0x31 };
    unsigned char hdlc_rr_frame[512];
    printf("\n=== RR Frame Test ===\n");
    printf("  [DEBUG] Original AX.25 frame (%zu bytes): ", sizeof(rr_frame));
    for (size_t i = 0; i < sizeof(rr_frame); i++) {
        printf("%02X ", rr_frame[i]);
    }
    printf("\n");
    hdlc_frame_encode(rr_frame, sizeof(rr_frame), hdlc_rr_frame, &hdlc_len);
    printf("  [DEBUG] Encoded HDLC frame (%d bytes): ", hdlc_len);
    for (int i = 0; i < hdlc_len; i++) {
        printf("%02X ", hdlc_rr_frame[i]);
    }
    printf("\n");
    printf("\nHDLC RR Frame:\n");
    hdlc_frame_print(hdlc_rr_frame, hdlc_len);

    // DISC frame
    unsigned char disc_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x43 };
    unsigned char hdlc_disc_frame[512];
    printf("\n=== DISC Frame Test ===\n");
    printf("  [DEBUG] Original AX.25 frame (%zu bytes): ", sizeof(disc_frame));
    for (size_t i = 0; i < sizeof(disc_frame); i++) {
        printf("%02X ", disc_frame[i]);
    }
    printf("\n");
    hdlc_frame_encode(disc_frame, sizeof(disc_frame), hdlc_disc_frame, &hdlc_len);
    printf("  [DEBUG] Encoded HDLC frame (%d bytes): ", hdlc_len);
    for (int i = 0; i < hdlc_len; i++) {
        printf("%02X ", hdlc_disc_frame[i]);
    }
    printf("\n");
    printf("\nHDLC DISC Frame:\n");
    hdlc_frame_print(hdlc_disc_frame, hdlc_len);
}

int test_extended_i_frame() {
    printf("test_extended_i_frame\n");
    uint8_t err = 0;
    // Extended I-frame: Dest: VA3BBB-7, Src: VA3AAA-1 (res1=0), Control: 0x0000 (N(S)=0, N(R)=0, P/F=0), PID: 0xF0, Payload: "Extended"
    unsigned char extended_i_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x23, 0x00, 0x00, 0xF0, 'E', 'x', 't',
            'e', 'n', 'd', 'e', 'd' };
    size_t extended_i_frame_len = sizeof(extended_i_frame);
    ax25_frame_t *decoded_frame = ax25_frame_decode(extended_i_frame, extended_i_frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding extended I-frame", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_INFORMATION_16BIT, "Frame type should be I-frame 16-bit", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) decoded_frame;
        TEST_ASSERT(i_frame->nr == 0, "nr should be 0", err);
        TEST_ASSERT(i_frame->ns == 0, "ns should be 0", err);
        TEST_ASSERT(i_frame->pf == false, "Poll/Final should be false", err);
        TEST_ASSERT(i_frame->pid == 0xF0, "PID should be 0xF0", err);
        TEST_ASSERT(i_frame->payload_len == 8, "Payload length should be 8", err);
        TEST_ASSERT(memcmp(i_frame->payload, "Extended", 8) == 0, "Payload should be 'Extended'", err);
        ax25_frame_free(decoded_frame, &err);
    }
    return 0;
}

int test_sabme_frame() {
    printf("test_sabme_frame\n");
    uint8_t err = 0;
    // SABME frame: Dest: VA3BBB-7, Src: VA3AAA-1, Control: 0x7F (SABME, P=1)
    unsigned char sabme_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x7F };
    size_t sabme_frame_len = sizeof(sabme_frame);
    ax25_frame_t *decoded_frame = ax25_frame_decode(sabme_frame, sabme_frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding SABME frame", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_SABME, "Frame type should be SABME", err);
        ax25_unnumbered_frame_t *u_frame = (ax25_unnumbered_frame_t*) decoded_frame;
        TEST_ASSERT(u_frame->pf == true, "Poll/Final should be true", err);
        TEST_ASSERT(u_frame->modifier == 0x6F, "Modifier should be 0x6F", err);
        ax25_frame_free(decoded_frame, &err);
    }
    return 0;
}

int test_extended_s_frame() {
    printf("test_extended_s_frame\n");
    uint8_t err = 0;
    // Extended RR frame: Dest: VA3AAA-1, Src: VA3BBB-1 (res1=0), Control: 0x0100 (RR, N(R)=0, P/F=0)
    unsigned char extended_rr_frame[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x62, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xA3, 0x01, 0x00 };
    size_t extended_rr_frame_len = sizeof(extended_rr_frame);
    ax25_frame_t *decoded_frame = ax25_frame_decode(extended_rr_frame, extended_rr_frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding extended RR frame", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_SUPERVISORY_RR_16BIT, "Frame type should be RR 16-bit", err);
        ax25_supervisory_frame_t *s_frame = (ax25_supervisory_frame_t*) decoded_frame;
        TEST_ASSERT(s_frame->nr == 0, "nr should be 0", err);
        TEST_ASSERT(s_frame->pf == false, "Poll/Final should be false", err);
        TEST_ASSERT(s_frame->code == 0x00, "Code should be 0x00 (RR)", err);
        ax25_frame_free(decoded_frame, &err);
    }
    return 0;
}

int test_max_repeaters() {
    printf("test_max_repeaters\n");
    uint8_t err = 0;
    // Create a frame with maximum repeaters (8)
    // UI frame with dummy repeaters: Dest: AAAAAA-0, Src: BBBBBB-0, 8 repeaters (CCCCCC-0 to JJJJJJ-0), Control: 0x03, PID: 0xF0
    unsigned char max_repeaters_frame[] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x60,  // Dest: AAAAAA-0, extension=0
            0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x60,  // Src: BBBBBB-0, extension=0
            // Repeaters: CCCCCC-0 to JJJJJJ-0
            0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x60,  // CCCCCC-0, extension=0
            0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x60,  // DDDDDD-0, extension=0
            0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x60,  // EEEEEE-0, extension=0
            0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x60,  // FFFFFF-0, extension=0
            0x8E, 0x8E, 0x8E, 0x8E, 0x8E, 0x8E, 0x60,  // GGGGGG-0, extension=0
            0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x60,  // HHHHHH-0, extension=0
            0x92, 0x92, 0x92, 0x92, 0x92, 0x92, 0x60,  // IIIIII-0, extension=0
            0x94, 0x94, 0x94, 0x94, 0x94, 0x94, 0x61,  // JJJJJJ-0, extension=1
            0x03, 0xF0  // Control: UI, PID: 0xF0
            };
    size_t frame_len = sizeof(max_repeaters_frame);
    ax25_frame_t *decoded_frame = ax25_frame_decode(max_repeaters_frame, frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding frame with maximum repeaters", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->header.repeaters.num_repeaters == 8, "Should have 8 repeaters", err);
        ax25_frame_free(decoded_frame, &err);
    }
    return 0;
}

int test_large_payload() {
    printf("test_large_payload\n");
    uint8_t err = 0;
    // Create a UI frame with 256-byte payload: Dest: AAAAAA-0, Src: BBBBBB-0, Control: 0x03, PID: 0xF0
    unsigned char large_payload_frame[14 + 1 + 1 + 256];  // Header + control + PID + payload
    // Header: AAAAAA-0 -> BBBBBB-0
    unsigned char header[] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x60, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x61 };
    memcpy(large_payload_frame, header, 14);
    large_payload_frame[14] = 0x03;  // Control byte for UI
    large_payload_frame[15] = 0xF0;  // PID
    for (int i = 0; i < 256; i++) {
        large_payload_frame[16 + i] = (uint8_t) i;
    }
    size_t frame_len = 14 + 1 + 1 + 256;
    ax25_frame_t *decoded_frame = ax25_frame_decode(large_payload_frame, frame_len, MODULO128_AUTO, &err);
    if (decoded_frame == NULL || err != 0) {
        printf("\033[0;31m[%04d] FAIL(%u): Decoding frame with large payload\033[0m\n", ++assert_count, err);
        return 1;
    }
    if (decoded_frame->type != AX25_FRAME_UNNUMBERED_INFORMATION) {
        printf("\033[0;31m[%04d] FAIL: Frame type should be UI\033[0m\n", ++assert_count);
        ax25_frame_free(decoded_frame, &err);
        return 1;
    }
    ax25_unnumbered_information_frame_t *ui_frame = (ax25_unnumbered_information_frame_t*) decoded_frame;
    if (ui_frame->payload_len != 256) {
        printf("\033[0;31m[%04d] FAIL: Payload length should be 256\033[0m\n", ++assert_count);
        ax25_frame_free(decoded_frame, &err);
        return 1;
    }
    // Check the entire payload at once
    int cmp_result = memcmp(ui_frame->payload, large_payload_frame + 16, 256);
    if (cmp_result == 0) {
        printf("\033[0;32m[%04d]    PASS: Payload data matches\033[0m\n", ++assert_count);
    } else {
        printf("\033[0;31m[%04d] FAIL: Payload data mismatch\033[0m\n", ++assert_count);
        // Optionally, print details about the mismatch
        for (int i = 0; i < 256; i++) {
            if (ui_frame->payload[i] != large_payload_frame[16 + i]) {
                printf("         -- Mismatch at byte %d: expected 0x%02X, got 0x%02X\n", i, large_payload_frame[16 + i], ui_frame->payload[i]);
                break;  // Only print the first mismatch
            }
        }
    }
    ax25_frame_free(decoded_frame, &err);
    return cmp_result != 0;
}

int test_ui_frame_no_payload() {
    printf("test_ui_frame_no_payload\n");
    uint8_t err = 0;
    // Create a UI frame with no payload: Dest: AAAAAA-0, Src: BBBBBB-0, Control: 0x03, PID: 0xF0
    unsigned char frame[16];  // 14 header + 1 control + 1 PID
    // Header: AAAAAA-0 -> BBBBBB-0
    unsigned char header[] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x60,  // dest: AAAAAA-0, C=0
            0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x61  // src: BBBBBB-0, C=1
            };
    memcpy(frame, header, 14);
    frame[14] = 0x03;  // Control byte for UI
    frame[15] = 0xF0;  // PID
    size_t frame_len = 16;
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding UI frame with no payload", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_INFORMATION, "Frame type should be UI", err);
        ax25_unnumbered_information_frame_t *ui_frame = (ax25_unnumbered_information_frame_t*) decoded_frame;
        TEST_ASSERT(ui_frame->payload_len == 0, "Payload length should be 0", err);
        ax25_frame_free(decoded_frame, &err);
    }
    return 0;
}

int test_i_frame_no_payload() {
    printf("test_i_frame_no_payload\n");
    uint8_t err = 0;
    // Create an I frame with no payload: Dest: AAAAAA-0, Src: BBBBBB-0, Control: 0x00 (I frame, N(S)=0, N(R)=0, P=0)
    unsigned char frame[15] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0xE0,  // Dest: AAAAAA-0, SSID=0xE0 (C=1, res1=1, res0=1, extension=0)
            0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x61,  // Src: BBBBBB-0, SSID=0x61 (C=0, res1=1, res0=1, extension=1)
            0x00  // Control byte
            };
    size_t frame_len = 15;
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_FALSE, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding I frame with no payload", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_INFORMATION_8BIT, "Frame type should be I frame (modulo 8)", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) decoded_frame;
        TEST_ASSERT(i_frame->payload_len == 0, "Payload length should be 0", err);
        TEST_ASSERT(i_frame->ns == 0, "N(S) should be 0", err);
        TEST_ASSERT(i_frame->nr == 0, "N(R) should be 0", err);
        TEST_ASSERT(i_frame->pf == false, "P/F should be 0", err);
        ax25_frame_free(decoded_frame, &err);
    }
    return 0;
}

int test_i_frame_no_payload_modulo128() {
    printf("test_i_frame_no_payload_modulo128\n");
    uint8_t err = 0;
    // Create an I frame with no payload, modulo 128: Dest: AAAAAA-0, Src: BBBBBB-0, Control: 0x00 0x00
    unsigned char frame[16] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0xE0,  // Dest: AAAAAA-0, SSID=0xE0 (C=1, res1=1, res0=1, extension=0)
            0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x21,  // Src: BBBBBB-0, SSID=0x21 (C=0, res1=0, res0=1, extension=1)
            0x00, 0x00  // Control bytes
            };
    size_t frame_len = 16;
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding I frame with no payload (modulo 128)", err);
    if (decoded_frame) {
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_INFORMATION_16BIT, "Frame type should be I frame (modulo 128)", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) decoded_frame;
        TEST_ASSERT(i_frame->payload_len == 0, "Payload length should be 0", err);
        TEST_ASSERT(i_frame->ns == 0, "N(S) should be 0", err);
        TEST_ASSERT(i_frame->nr == 0, "N(R) should be 0", err);
        TEST_ASSERT(i_frame->pf == false, "P/F should be 0", err);
        ax25_frame_free(decoded_frame, &err);
    }
    return 0;
}

int test_invalid_address_field() {
    printf("test_invalid_address_field\n");
    uint8_t err = 0;
    // Create a frame with destination (E=0) and source (E=0, invalid termination)
    unsigned char frame[15] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x60,  // Destination: AAAAAA-0, E=0
            0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x60,  // Source: BBBBBB-0, E=0
            0x03  // Control byte for UI
            };
    size_t frame_len = 15;
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame == NULL && err == 5, "Decoding frame with invalid address field", err);
    if (decoded_frame) {
        ax25_frame_free(decoded_frame, &err);
    }
    return 0;
}

int test_valid_address_field() {
    printf("test_valid_address_field\n");
    uint8_t err = 0;
    // Create a frame with destination (E=0, C=1) and source (E=1, C=0), with PID
    unsigned char frame[16] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0xE0,  // Destination: AAAAAA-0, C=1, E=0
            0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x61,  // Source: BBBBBB-0, C=0, E=1
            0x03,  // Control byte for UI
            0xF0   // PID: no layer 3 protocol
            };
    size_t frame_len = 16;
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding frame with valid address field", err);
    if (decoded_frame) {
        ax25_frame_free(decoded_frame, &err);
    }
    return 0;
}

int test_invalid_control_field() {
    printf("test_invalid_control_field\n");
    uint8_t err = 0;
    // Create a U frame with invalid control byte (0xFF)
    unsigned char frame[15];
    unsigned char dest[7] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x60 };  // AAAAAA-0, E=0
    unsigned char src[7] = { 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x61 };  // BBBBBB-0, E=1
    memcpy(frame, dest, 7);
    memcpy(frame + 7, src, 7);
    frame[14] = 0xFF;  // Invalid control byte for U frame
    size_t frame_len = 15;
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame == NULL && err == 6, "Decoding U frame with invalid control field", err);
    return 0;
}

int test_sabme_ua_negotiation() {
    printf("test_sabme_ua_negotiation\n");
    uint8_t err = 0;

    // Create SABME frame: Dest: AAAAAA-0, Src: BBBBBB-0, Control: 0x6F (SABME, P/F=0)
    ax25_unnumbered_frame_t *sabme_frame = malloc(sizeof(ax25_unnumbered_frame_t));
    sabme_frame->base.type = AX25_FRAME_UNNUMBERED_SABME;
    sabme_frame->base.header.destination = (ax25_address_t ) { .callsign = "AAAAAA", .ssid = 0, .ch = true, .res0 = true, .res1 = true, .extension = false };
    sabme_frame->base.header.source = (ax25_address_t ) { .callsign = "BBBBBB", .ssid = 0, .ch = false, .res0 = true, .res1 = false, .extension = true };
    sabme_frame->base.header.cr = true;
    sabme_frame->base.header.src_cr = false;
    sabme_frame->base.header.repeaters.num_repeaters = 0;
    sabme_frame->pf = false;
    sabme_frame->modifier = 0x6F;

    // Test 1: UA Response (modulo-128)
    ax25_unnumbered_frame_t *ua_response = malloc(sizeof(ax25_unnumbered_frame_t));
    ua_response->base.type = AX25_FRAME_UNNUMBERED_UA;
    ua_response->base.header = sabme_frame->base.header;  // Copy header
    ua_response->base.header.destination.ch = false;
    ua_response->base.header.source.ch = true;
    ua_response->base.header.cr = false;
    ua_response->base.header.src_cr = true;
    ua_response->pf = false;
    ua_response->modifier = 0x63;
    TEST_ASSERT(is_modulo128_used((ax25_frame_t*)sabme_frame, (ax25_frame_t*)ua_response) == true, "UA response should indicate modulo-128", err);

    // Test 2: DM Response (fallback to modulo-8)
    ax25_unnumbered_frame_t *dm_response = malloc(sizeof(ax25_unnumbered_frame_t));
    dm_response->base.type = AX25_FRAME_UNNUMBERED_DM;
    dm_response->base.header = sabme_frame->base.header;
    dm_response->base.header.destination.ch = false;
    dm_response->base.header.source.ch = true;
    dm_response->base.header.cr = false;
    dm_response->base.header.src_cr = true;
    dm_response->pf = false;
    dm_response->modifier = 0x0F;
    TEST_ASSERT(is_modulo128_used((ax25_frame_t*)sabme_frame, (ax25_frame_t*)dm_response) == false, "DM response should indicate modulo-8", err);

    // Test 3: FRMR Response (fallback to modulo-8)
    ax25_frame_reject_frame_t *frmr_response = malloc(sizeof(ax25_frame_reject_frame_t));
    frmr_response->base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
    frmr_response->base.base.header = sabme_frame->base.header;
    frmr_response->base.base.header.destination.ch = false;
    frmr_response->base.base.header.source.ch = true;
    frmr_response->base.base.header.cr = false;
    frmr_response->base.base.header.src_cr = true;
    frmr_response->base.pf = false;
    frmr_response->base.modifier = 0x87;
    frmr_response->is_modulo128 = false;
    frmr_response->frmr_control = 0x6F;
    frmr_response->vs = 0;
    frmr_response->vr = 0;
    frmr_response->frmr_cr = false;
    frmr_response->w = true;
    frmr_response->x = false;
    frmr_response->y = false;
    frmr_response->z = false;
    TEST_ASSERT(is_modulo128_used((ax25_frame_t*)sabme_frame, (ax25_frame_t*)frmr_response) == false, "FRMR response should indicate modulo-8", err);

    // Cleanup
    ax25_frame_free((ax25_frame_t*) sabme_frame, &err);
    ax25_frame_free((ax25_frame_t*) ua_response, &err);
    ax25_frame_free((ax25_frame_t*) dm_response, &err);
    ax25_frame_free((ax25_frame_t*) frmr_response, &err);

    return 0;
}

int test_sequence_number_wrap_around() {
    printf("test_sequence_number_wrap_around\n");
    uint8_t err = 0;

    // Create I-frame with ns=127: Dest: AAAAAA-0, Src: BBBBBB-0, Control: ns=127, nr=0, P/F=0
    ax25_information_frame_t *frame_127 = malloc(sizeof(ax25_information_frame_t));
    frame_127->base.type = AX25_FRAME_INFORMATION_16BIT;
    frame_127->base.header.destination = (ax25_address_t ) { .callsign = "AAAAAA", .ssid = 0, .ch = true, .res0 = true, .res1 = true, .extension = false };
    frame_127->base.header.source = (ax25_address_t ) { .callsign = "BBBBBB", .ssid = 0, .ch = false, .res0 = true, .res1 = false, .extension = true };
    frame_127->base.header.cr = true;
    frame_127->base.header.src_cr = false;
    frame_127->base.header.repeaters.num_repeaters = 0;
    frame_127->nr = 0;
    frame_127->pf = false;
    frame_127->ns = 127;
    frame_127->pid = 0xF0;
    frame_127->payload_len = 0;
    frame_127->payload = NULL;

    // Create I-frame with ns=0 (wrap-around)
    ax25_information_frame_t *frame_0 = malloc(sizeof(ax25_information_frame_t));
    *frame_0 = *frame_127;  // Copy all fields
    frame_0->ns = 0;

    // Encode both frames
    size_t len_127, len_0;
    uint8_t *encoded_127 = ax25_frame_encode((ax25_frame_t*) frame_127, &len_127, &err);
    uint8_t *encoded_0 = ax25_frame_encode((ax25_frame_t*) frame_0, &len_0, &err);
    TEST_ASSERT(encoded_127 != NULL && encoded_0 != NULL, "Encoding frames should succeed", err);

    // Decode and verify
    ax25_frame_t *decoded_127 = ax25_frame_decode(encoded_127, len_127, MODULO128_TRUE, &err);
    ax25_frame_t *decoded_0 = ax25_frame_decode(encoded_0, len_0, MODULO128_TRUE, &err);
    TEST_ASSERT(decoded_127 != NULL && decoded_0 != NULL, "Decoding frames should succeed", err);

    ax25_information_frame_t *i_frame_127 = (ax25_information_frame_t*) decoded_127;
    ax25_information_frame_t *i_frame_0 = (ax25_information_frame_t*) decoded_0;
    TEST_ASSERT(i_frame_127->ns == 127 && i_frame_0->ns == 0, "Sequence numbers should wrap from 127 to 0", err);

    // Cleanup
    free(encoded_127);
    free(encoded_0);
    ax25_frame_free((ax25_frame_t*) frame_127, &err);
    ax25_frame_free((ax25_frame_t*) frame_0, &err);
    ax25_frame_free(decoded_127, &err);
    ax25_frame_free(decoded_0, &err);

    return 0;
}

int test_large_payloads() {
    printf("test_large_payloads\n");
    uint8_t err = 0;

    // Create a 512-byte payload
    size_t payload_size = 512;
    uint8_t *payload = malloc(payload_size);
    if (!payload) {
        TEST_ASSERT(false, "Payload allocation should succeed", err);
        return 1;
    }
    for (size_t i = 0; i < payload_size; i++) {
        payload[i] = (uint8_t) (i % 256);
    }

    // Create UI frame: Dest: AAAAAA-0, Src: BBBBBB-0, Control: 0x03, PID: 0xF0
    ax25_unnumbered_information_frame_t *ui_frame = malloc(sizeof(ax25_unnumbered_information_frame_t));
    ui_frame->base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
    ui_frame->base.base.header.destination = (ax25_address_t ) { .callsign = "AAAAAA", .ssid = 0, .ch = true, .res0 = true, .res1 = true, .extension = false };
    ui_frame->base.base.header.source = (ax25_address_t ) { .callsign = "BBBBBB", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .extension = true };
    ui_frame->base.base.header.cr = true;
    ui_frame->base.base.header.src_cr = false;
    ui_frame->base.base.header.repeaters.num_repeaters = 0;
    ui_frame->base.pf = false;
    ui_frame->base.modifier = 0x03;
    ui_frame->pid = 0xF0;
    ui_frame->payload_len = payload_size;
    ui_frame->payload = malloc(payload_size);
    if (!ui_frame->payload) {
        TEST_ASSERT(false, "Payload allocation for UI frame should succeed", err);
        free(payload);
        ax25_frame_free((ax25_frame_t*) ui_frame, &err);
        return 1;
    }
    memcpy(ui_frame->payload, payload, payload_size);

    // Encode the frame
    size_t encoded_len;
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) ui_frame, &encoded_len, &err);
    TEST_ASSERT(encoded != NULL, "Encoding UI frame with large payload should succeed", err);

    // Decode the frame
    ax25_frame_t *decoded_frame = ax25_frame_decode(encoded, encoded_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL, "Decoding UI frame with large payload should succeed", err);

    ax25_unnumbered_information_frame_t *decoded_ui = (ax25_unnumbered_information_frame_t*) decoded_frame;
    TEST_ASSERT(decoded_ui->payload_len == payload_size, "Decoded payload size should match original", err);
    TEST_ASSERT(memcmp(decoded_ui->payload, payload, payload_size) == 0, "Decoded payload data should match original", err);

    // Cleanup
    free(payload);
    free(encoded);
    ax25_frame_free((ax25_frame_t*) ui_frame, &err);
    ax25_frame_free(decoded_frame, &err);

    return 0;
}

int test_srej_functionality() {
    uint8_t err = 0;

    // Create three I-frames: ns=0, ns=1, ns=2
    ax25_information_frame_t *frame_0 = malloc(sizeof(ax25_information_frame_t));
    frame_0->base.type = AX25_FRAME_INFORMATION_8BIT;
    frame_0->base.header.destination = (ax25_address_t ) { .callsign = "AAAAAA", .ssid = 0, .ch = true, .res0 = true, .res1 = true, .extension = false };
    frame_0->base.header.source = (ax25_address_t ) { .callsign = "BBBBBB", .ssid = 0, .ch = false, .res0 = true, .res1 = true, .extension = true };
    frame_0->base.header.cr = true;
    frame_0->base.header.src_cr = false;
    frame_0->base.header.repeaters.num_repeaters = 0;
    frame_0->nr = 0;
    frame_0->pf = false;
    frame_0->ns = 0;
    frame_0->pid = 0xF0;
    frame_0->payload_len = 1;
    frame_0->payload = malloc(1);
    frame_0->payload[0] = 'A';

    ax25_information_frame_t *frame_1 = malloc(sizeof(ax25_information_frame_t));
    *frame_1 = *frame_0;
    frame_1->ns = 1;
    frame_1->payload = malloc(1);
    frame_1->payload[0] = 'B';

    ax25_information_frame_t *frame_2 = malloc(sizeof(ax25_information_frame_t));
    *frame_2 = *frame_0;
    frame_2->ns = 2;
    frame_2->payload = malloc(1);
    frame_2->payload[0] = 'C';

    // Simulate packet loss: only frame_0 and frame_2 received
    // Generate SREJ for ns=1
    ax25_supervisory_frame_t *srej_frame = malloc(sizeof(ax25_supervisory_frame_t));
    srej_frame->base.type = AX25_FRAME_SUPERVISORY_SREJ_8BIT;
    srej_frame->base.header = frame_0->base.header;
    srej_frame->base.header.destination.ch = false;
    srej_frame->base.header.source.ch = true;
    srej_frame->base.header.cr = false;
    srej_frame->base.header.src_cr = true;
    srej_frame->nr = 1;  // Request retransmission of ns=1
    srej_frame->pf = false;
    srej_frame->code = 0x0C;

    // Encode SREJ
    size_t srej_len;
    uint8_t *srej_encoded = ax25_supervisory_frame_encode(srej_frame, &srej_len, &err);
    TEST_ASSERT(srej_encoded != NULL, "Encoding SREJ frame should succeed", err);

    // Decode SREJ
    ax25_frame_t *decoded_srej = ax25_frame_decode(srej_encoded, srej_len, MODULO128_FALSE, &err);
    TEST_ASSERT(decoded_srej != NULL, "Decoding SREJ frame should succeed", err);
    TEST_ASSERT(decoded_srej->type == AX25_FRAME_SUPERVISORY_SREJ_8BIT, "Decoded frame should be SREJ", err);
    ax25_supervisory_frame_t *decoded_srej_frame = (ax25_supervisory_frame_t*) decoded_srej;
    TEST_ASSERT(decoded_srej_frame->nr == 1, "SREJ should request ns=1", err);

    // Simulate retransmission of frame_1
    size_t retransmitted_len;
    uint8_t *retransmitted = ax25_frame_encode((ax25_frame_t*) frame_1, &retransmitted_len, &err);
    TEST_ASSERT(retransmitted != NULL, "Encoding retransmitted frame should succeed", err);

    ax25_frame_t *decoded_retransmitted = ax25_frame_decode(retransmitted, retransmitted_len, MODULO128_FALSE, &err);
    TEST_ASSERT(decoded_retransmitted != NULL, "Decoding retransmitted frame should succeed", err);
    TEST_ASSERT(decoded_retransmitted->type == AX25_FRAME_INFORMATION_8BIT, "Retransmitted frame should be I-frame", err);
    ax25_information_frame_t *retransmitted_frame = (ax25_information_frame_t*) decoded_retransmitted;
    TEST_ASSERT(retransmitted_frame->ns == 1, "Retransmitted frame should have ns=1", err);
    TEST_ASSERT(retransmitted_frame->payload_len == 1 && retransmitted_frame->payload[0] == 'B', "Retransmitted payload should be 'B'", err);

    // Cleanup
    free(frame_0->payload);
    free(frame_1->payload);
    free(frame_2->payload);
    ax25_frame_free((ax25_frame_t*) frame_0, &err);
    ax25_frame_free((ax25_frame_t*) frame_1, &err);
    ax25_frame_free((ax25_frame_t*) frame_2, &err);
    free(srej_encoded);
    ax25_frame_free(decoded_srej, &err);
    free(retransmitted);
    ax25_frame_free(decoded_retransmitted, &err);
    ax25_frame_free((ax25_frame_t*) srej_frame, &err);

    return 0;
}

int test_ax25_main() {
    int result = 0;
    printf("\n----------------------------------------------------------------------------------\n");
    printf("Starting AX.25 Tests\n");
    printf("----------------------------------------------------------------------------------\n\n");
    result |= test_address_functions();
    result |= test_path_functions();
    result |= test_frame_header_functions();
    result |= test_frame_functions();
    result |= test_raw_frame_functions();
    result |= test_unnumbered_frame_functions();
    result |= test_unnumbered_information_frame_functions();
    result |= test_frame_reject_frame_functions();
    result |= test_information_frame_functions();
    result |= test_supervisory_frame_functions();
    result |= test_xid_parameter_functions();
    result |= test_exchange_identification_frame_functions();
    result |= test_test_frame_functions();
    result |= test_ax25_connection();
    result |= test_ax25_modulo128();
    result |= test_frmr_frame_functions();
    result |= test_auto_modulo_detection();
    result |= test_segmentation_reassembly();
    result |= test_sabme_frame();
    result |= test_extended_i_frame();
    result |= test_extended_s_frame();
    result |= test_max_repeaters();
    result |= test_large_payload();
    result |= test_ui_frame_no_payload();
    result |= test_i_frame_no_payload();
    result |= test_i_frame_no_payload_modulo128();
    result |= test_invalid_address_field();
    result |= test_valid_address_field();
    result |= test_invalid_control_field();
    result |= test_sabme_ua_negotiation();
    result |= test_sequence_number_wrap_around();
    result |= test_large_payloads();
    result |= test_sabme_ua_negotiation();

    printf("\n----------------------------------------------------------------------------------\n\n");
    test_ax25_frame_print();
    printf("\n----------------------------------------------------------------------------------\n");
    test_ax25_hdlc_frame_print();
    printf("\n----------------------------------------------------------------------------------\n");
    printf("Tests AX.25 Completed. %s\n", result == 0 ? "All tests passed" : "Some tests failed");
    printf("----------------------------------------------------------------------------------\n\n");
    return result;
}
