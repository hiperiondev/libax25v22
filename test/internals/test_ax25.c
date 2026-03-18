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
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "test_common.h"
#include "utils.h"
#include "ax25.h"

static uint32_t assert_count = 0;

#define DEBUG_ENABLE

int test_address_functions() {
    printf("test_address_functions\n");
    uint8_t err = 0;

    // Test ax25_address_from_string with "NOCALL-7*"
    DEBUG_PRINT("Testing ax25_address_from_string: NOCALL-7*");
    ax25_address_t *addr = ax25_address_from_string("NOCALL-7*", &err);
    TEST_ASSERT(addr != NULL, "ax25_address_from_string should return non-NULL for valid input", err);
    DEBUG_VAR("Error code after valid parse", err);
    if (addr) {
        DEBUG_PRINT("Parsed address fields:");
        DEBUG_PRINT("  callsign expected=NOCALL");
        DEBUG_BOOL("  ch (has-been-repeated)", addr->ch);
        DEBUG_VAR("  ssid", addr->ssid);
        DEBUG_BOOL("  res0", addr->res0);
        DEBUG_BOOL("  res1", addr->res1);
        DEBUG_BOOL("  extension", addr->extension);
        TEST_ASSERT(strcmp(addr->callsign, "NOCALL") == 0, "Callsign should be NOCALL", err);
        TEST_ASSERT(addr->ssid == 7, "SSID should be 7", err);
        TEST_ASSERT(addr->ch == true, "ch should be true due to '*'", err);
        TEST_ASSERT(addr->res0 == true, "res0 should be true", err);
        TEST_ASSERT(addr->res1 == true, "res1 should be true", err);
        TEST_ASSERT(addr->extension == false, "extension should be false", err);
        ax25_address_free(addr, &err);
    }

    // Test ax25_address_from_string with "ABC123-15"
    DEBUG_PRINT("Testing ax25_address_from_string: ABC123-15");
    addr = ax25_address_from_string("ABC123-15", &err);
    TEST_ASSERT(addr != NULL, "ax25_address_from_string should return non-NULL for valid input", err);
    if (addr) {
        DEBUG_VAR("ssid", addr->ssid);
        DEBUG_BOOL("ch", addr->ch);
        TEST_ASSERT(strcmp(addr->callsign, "ABC123") == 0, "Callsign should be ABC123", err);
        TEST_ASSERT(addr->ssid == 15, "SSID should be 15", err);
        TEST_ASSERT(addr->ch == false, "ch should be false", err);
        TEST_ASSERT(addr->res0 == true, "res0 should be true", err);
        TEST_ASSERT(addr->res1 == true, "res1 should be true", err);
        TEST_ASSERT(addr->extension == false, "extension should be false", err);
        ax25_address_free(addr, &err);
    }

    // Test ax25_address_from_string with "NOCALL-16" (SSID out of range)
    DEBUG_PRINT("Testing ax25_address_from_string: NOCALL-16 (invalid SSID)");
    addr = ax25_address_from_string("NOCALL-16", &err);
    DEBUG_VAR("Error code (expected 4)", err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for invalid SSID", err);
    TEST_ASSERT(err == 4, "Error code should be 4 for invalid SSID", err);

    // Test ax25_address_from_string with "NOCALL-1A" (Non-numeric SSID)
    DEBUG_PRINT("Testing ax25_address_from_string: NOCALL-1A (non-numeric SSID)");
    addr = ax25_address_from_string("NOCALL-1A", &err);
    DEBUG_VAR("Error code (expected 5)", err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for non-numeric SSID", err);
    TEST_ASSERT(err == 5, "Error code should be 5 for invalid character after SSID", err);

    // Test ax25_address_from_string with "NOCALL*7" (Misplaced asterisk)
    DEBUG_PRINT("Testing ax25_address_from_string: NOCALL*7 (misplaced asterisk)");
    addr = ax25_address_from_string("NOCALL*7", &err);
    DEBUG_VAR("Error code (expected 6)", err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for misplaced asterisk", err);
    TEST_ASSERT(err == 6, "Error code should be 6 for '*' not at the end", err);

    // Test ax25_address_from_string with "TOOLONGADDR-1" (String too long)
    DEBUG_PRINT("Testing ax25_address_from_string: TOOLONGADDR-1 (too long)");
    addr = ax25_address_from_string("TOOLONGADDR-1", &err);
    DEBUG_VAR("Error code (expected 4)", err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for string too long", err);
    TEST_ASSERT(err == 4, "Error code should be 4 for invalid callsign length", err);

    // Test ax25_address_from_string with "" (Empty string)
    DEBUG_PRINT("Testing ax25_address_from_string: empty string");
    addr = ax25_address_from_string("", &err);
    DEBUG_VAR("Error code (expected 4)", err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for empty string", err);
    TEST_ASSERT(err == 4, "Error code should be 4 for invalid callsign length", err);

    // Test ax25_address_from_string with NULL
    DEBUG_PRINT("Testing ax25_address_from_string: NULL input");
    addr = ax25_address_from_string(NULL, &err);
    DEBUG_VAR("Error code (expected 2)", err);
    TEST_ASSERT(addr == NULL, "ax25_address_from_string should return NULL for NULL input", err);
    TEST_ASSERT(err == 2, "Error code should be 2 for invalid input", err);

    DEBUG_PRINT("Address function tests complete");

    return 0;
}

int test_path_functions() {
    printf("test_path_functions\n");

    uint8_t err = 0;

    // Test 1: Single repeater
    {
        DEBUG_PRINT("Test 1: single repeater REPEATER-1*");
        ax25_address_t *addr1 = ax25_address_from_string("REPEATER-1*", &err);
        TEST_ASSERT(addr1 != NULL, "Address creation should succeed", err);
        ax25_address_t *repeaters[] = { addr1 };
        ax25_path_t *path = ax25_path_new(repeaters, 1, &err);
        TEST_ASSERT(path != NULL, "Path creation with one repeater should succeed", err);
        if (path) {
            DEBUG_VAR("num_repeaters", path->num_repeaters);
            DEBUG_PRINT("repeater[0].callsign (expected REPEAT)");
            DEBUG_VAR("repeater[0].ssid", path->repeaters[0].ssid);
            DEBUG_BOOL("repeater[0].ch", path->repeaters[0].ch);
        }
        TEST_ASSERT(path->num_repeaters == 1, "Path should have 1 repeater", err);
        TEST_ASSERT(strcmp(path->repeaters[0].callsign, "REPEAT") == 0, "Repeater callsign should be REPEAT", err);
        TEST_ASSERT(path->repeaters[0].ssid == 1, "Repeater SSID should be 1", err);
        TEST_ASSERT(path->repeaters[0].ch == true, "Repeater ch should be true", err);
        ax25_path_free(path, &err);
        ax25_address_free(addr1, &err);
    }

    // Test 2: Zero repeaters
    {
        DEBUG_PRINT("Test 2: zero repeaters (should fail)");
        ax25_address_t *repeaters[] = { };
        ax25_path_t *path = ax25_path_new(repeaters, 0, &err);
        DEBUG_VAR("Error code (expected 2)", err);
        TEST_ASSERT(path == NULL, "Path creation with zero repeaters should fail", err);
        TEST_ASSERT(err == 2, "Error should be 2 for invalid input", err);
    }

    // Test 3: Maximum repeaters (8)
    {
        DEBUG_PRINT("Test 3: maximum repeaters (8)");
        ax25_address_t *repeaters[AX25_MAX_REPEATERS];
        for (int i = 0; i < AX25_MAX_REPEATERS; i++) {
            char callsign[12];
            sprintf(callsign, "RPT%d-%d*", i, i);
            repeaters[i] = ax25_address_from_string(callsign, &err);
            TEST_ASSERT(repeaters[i] != NULL, "Repeater address creation should succeed", err);
        }
        ax25_path_t *path = ax25_path_new(repeaters, AX25_MAX_REPEATERS, &err);
        TEST_ASSERT(path != NULL, "Path creation with max repeaters should succeed", err);
        if (path) {
            DEBUG_VAR("num_repeaters", path->num_repeaters);
        }
        TEST_ASSERT(path->num_repeaters == AX25_MAX_REPEATERS, "Path should have 8 repeaters", err);
        for (int i = 0; i < AX25_MAX_REPEATERS; i++) {
            char expected_callsign[7];
            sprintf(expected_callsign, "RPT%d", i);
            TEST_ASSERT(strcmp(path->repeaters[i].callsign, expected_callsign) == 0, "Repeater callsign should match", err);
            TEST_ASSERT(path->repeaters[i].ssid == i, "Repeater SSID should match index", err);
            TEST_ASSERT(path->repeaters[i].ch == true, "Repeater ch should be true", err);
        }
        ax25_path_free(path, &err);
        for (int i = 0; i < AX25_MAX_REPEATERS; i++) {
            ax25_address_free(repeaters[i], &err);
        }
    }

    // Test 4: Exceeding maximum repeaters (9)
    {
        DEBUG_PRINT("Test 4: exceeding maximum repeaters (9, should fail)");
        ax25_address_t *repeaters[AX25_MAX_REPEATERS + 1];
        for (int i = 0; i < AX25_MAX_REPEATERS + 1; i++) {
            char callsign[12];
            sprintf(callsign, "RPT%d-%d*", i, i);
            repeaters[i] = ax25_address_from_string(callsign, &err);
            TEST_ASSERT(repeaters[i] != NULL, "Repeater address creation should succeed", err);
        }
        ax25_path_t *path = ax25_path_new(repeaters, AX25_MAX_REPEATERS + 1, &err);
        DEBUG_VAR("Error code (expected 2)", err);
        TEST_ASSERT(path == NULL, "Path creation exceeding max repeaters should fail", err);
        TEST_ASSERT(err == 2, "Error should be 2 for too many repeaters", err);
        for (int i = 0; i < AX25_MAX_REPEATERS + 1; i++) {
            ax25_address_free(repeaters[i], &err);
        }
    }

    // Test 5: NULL repeaters array
    {
        DEBUG_PRINT("Test 5: NULL repeaters array (should fail)");
        ax25_path_t *path = ax25_path_new(NULL, 1, &err);
        DEBUG_VAR("Error code (expected 2)", err);
        TEST_ASSERT(path == NULL, "Path creation with NULL repeaters should fail", err);
        TEST_ASSERT(err == 2, "Error should be 2 for NULL input", err);
    }

    // Test 6: NULL individual repeater
    {
        DEBUG_PRINT("Test 6: NULL individual repeater in array (should fail)");
        ax25_address_t *addr1 = ax25_address_from_string("REPEATER-1*", &err);
        TEST_ASSERT(addr1 != NULL, "Address creation should succeed", err);
        ax25_address_t *repeaters[] = { addr1, NULL };
        ax25_path_t *path = ax25_path_new(repeaters, 2, &err);
        DEBUG_VAR("Error code (expected 2)", err);
        TEST_ASSERT(path == NULL, "Path creation with NULL repeater should fail", err);
        TEST_ASSERT(err == 2, "Error should be 2 for NULL repeater", err);
        ax25_address_free(addr1, &err);
    }

    // Test 7: Realistic AX.25 path with 3 repeaters
    {
        DEBUG_PRINT("Test 7: realistic 3-repeater path WIDE1-1* WIDE2-2* NOCALL-0");
        ax25_address_t *addr1 = ax25_address_from_string("WIDE1-1*", &err);
        ax25_address_t *addr2 = ax25_address_from_string("WIDE2-2*", &err);
        ax25_address_t *addr3 = ax25_address_from_string("NOCALL-0", &err);
        TEST_ASSERT(addr1 != NULL && addr2 != NULL && addr3 != NULL, "Address creation should succeed", err);
        ax25_address_t *repeaters[] = { addr1, addr2, addr3 };
        ax25_path_t *path = ax25_path_new(repeaters, 3, &err);
        TEST_ASSERT(path != NULL, "Path creation with realistic repeaters should succeed", err);
        if (path) {
            DEBUG_VAR("num_repeaters", path->num_repeaters);
            DEBUG_BOOL("repeater[0].ch (WIDE1-1*)", path->repeaters[0].ch);
            DEBUG_BOOL("repeater[1].ch (WIDE2-2*)", path->repeaters[1].ch);
            DEBUG_BOOL("repeater[2].ch (NOCALL-0)", path->repeaters[2].ch);
        }
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

    DEBUG_PRINT("Path function tests complete");

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
        DEBUG_FRAME("Encoded modulo-128 I-frame", encoded, len);
        // Source address is bytes 7 to 13
        uint8_t source_ssid_byte = encoded[13];
        DEBUG_VAR("Source SSID byte (expected 0x23)", source_ssid_byte);
        TEST_ASSERT((source_ssid_byte & 0x40) == 0, "Source SSID bit 6 should be 0 for modulo-128", err);
        // Expected SSID byte: 0x23
        TEST_ASSERT(source_ssid_byte == 0x23, "Source SSID byte should be 0x23", err);
        free(encoded);
    }
    DEBUG_PRINT("Modulo-128 source address test complete");

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
        DEBUG_FRAME("Encoded modulo-8 I-frame", encoded, len);
        // Source address is bytes 7 to 13
        uint8_t source_ssid_byte = encoded[13];
        DEBUG_VAR("Source SSID byte (expected 0x63)", source_ssid_byte);
        TEST_ASSERT((source_ssid_byte & 0x40) == 0x40, "Source SSID bit 6 should be 1 for modulo-8", err);
        // Expected SSID byte: 0x63
        TEST_ASSERT(source_ssid_byte == 0x63, "Source SSID byte should be 0x63", err);
        free(encoded);
    }
    DEBUG_PRINT("Modulo-8 source address test complete");

    return 0;
}

int test_frame_header_functions() {
    printf("test_frame_header_functions\n");

    uint8_t err = 0;

    // Test data: Header with destination "ABCDEF-7" and source "GHIJKL-1*"
    // Dest: 'A'<<1=0x82, 'B'<<1=0x84, 'C'<<1=0x86, 'D'<<1=0x88, 'E'<<1=0x8A, 'F'<<1=0x8C, SSID=7, ch=1, res0=1, res1=0, ext=0: 0xAE
    // Src:  'G'<<1=0x8E, 'H'<<1=0x90, 'I'<<1=0x92, 'J'<<1=0x94, 'K'<<1=0x96, 'L'<<1=0x98, SSID=1, ch=0, res0=1, res1=0, ext=1: 0x23
    uint8_t header_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xAE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x23 };
    DEBUG_FRAME("Raw header input", header_data, sizeof(header_data));
    header_decode_result_t result = ax25_frame_header_decode(header_data, sizeof(header_data), &err);
    TEST_ASSERT(result.header != NULL, "ax25_frame_header_decode should return non-NULL header", err);
    if (result.header) {
        // Verify all fields
        DEBUG_PRINT("Decoded destination callsign (expected ABCDEF)");
        DEBUG_VAR("Destination SSID (expected 7)", result.header->destination.ssid);
        DEBUG_BOOL("Destination ch (expected true)", result.header->destination.ch);
        DEBUG_BOOL("Destination res0 (expected true)", result.header->destination.res0);
        DEBUG_BOOL("Destination res1 (expected false)", result.header->destination.res1);
        DEBUG_BOOL("Destination extension (expected false)", result.header->destination.extension);
        TEST_ASSERT(strcmp(result.header->destination.callsign, "ABCDEF") == 0, "Destination callsign should be ABCDEF", err);
        TEST_ASSERT(result.header->destination.ssid == 7, "Destination SSID should be 7", err);
        TEST_ASSERT(result.header->destination.ch == true, "Destination ch should be true", err);
        TEST_ASSERT(result.header->destination.res0 == true, "Destination res0 should be true", err);
        TEST_ASSERT(result.header->destination.res1 == false, "Destination res1 should be false", err);
        TEST_ASSERT(result.header->destination.extension == false, "Destination extension should be false", err);

        DEBUG_PRINT("Decoded source callsign (expected GHIJKL)");
        DEBUG_VAR("Source SSID (expected 1)", result.header->source.ssid);
        DEBUG_BOOL("Source ch (expected false)", result.header->source.ch);
        DEBUG_BOOL("Source extension (expected true)", result.header->source.extension);
        TEST_ASSERT(strcmp(result.header->source.callsign, "GHIJKL") == 0, "Source callsign should be GHIJKL", err);
        TEST_ASSERT(result.header->source.ssid == 1, "Source SSID should be 1", err);
        TEST_ASSERT(result.header->source.ch == false, "Source ch should be false", err);
        TEST_ASSERT(result.header->source.res0 == true, "Source res0 should be true", err);
        TEST_ASSERT(result.header->source.res1 == false, "Source res1 should be false", err);
        TEST_ASSERT(result.header->source.extension == true, "Source extension should be true", err);

        DEBUG_BOOL("cr (expected true)", result.header->cr);
        DEBUG_BOOL("src_cr (expected false)", result.header->src_cr);
        DEBUG_VAR("num_repeaters (expected 0)", result.header->repeaters.num_repeaters);
        TEST_ASSERT(result.header->cr == true, "cr should be true (dest ch=1, src ch=0)", err);
        TEST_ASSERT(result.header->src_cr == false, "src_cr should be false", err);
        TEST_ASSERT(result.header->repeaters.num_repeaters == 0, "No repeaters expected", err);

        size_t len;
        uint8_t *encoded = ax25_frame_header_encode(result.header, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_frame_header_encode should return non-NULL", err);
        TEST_ASSERT(len == sizeof(header_data), "Encoded header length should match input", err);
        DEBUG_FRAME("Re-encoded header", encoded, len);
        COMPARE_FRAME(encoded, len, header_data, sizeof(header_data), "Header re-encoding should match");
        free(encoded);
        ax25_frame_header_free(result.header, &err);
    }
    DEBUG_PRINT("Frame header function tests complete");

    return 0;
}

int test_frame_functions() {
    printf("test_frame_functions\n");

    uint8_t err = 0;

    // Test data: UI frame with dest "ABCDEF-7", src "GHIJKL-1*", control=0x03, PID=0xF0, payload="TEST"
    // Dest SSID byte: 0xAE (ch=1, res0=1, res1=0, ssid=7, ext=0)
    // Src SSID byte: 0x23 (ch=0, res0=1, res1=0, ssid=1, ext=1)
    uint8_t frame_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xAE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x23, 0x03, 0xF0, 'T', 'E', 'S', 'T' };
    DEBUG_FRAME("Raw UI frame input", frame_data, sizeof(frame_data));
    ax25_frame_t *frame = ax25_frame_decode(frame_data, sizeof(frame_data), 0, &err);
    TEST_ASSERT(frame != NULL, "ax25_frame_decode should return non-NULL", err);
    DEBUG_VAR("Frame type (expected UI)", frame ? frame->type : 0xFF);
    if (frame) {
        TEST_ASSERT(frame->type == AX25_FRAME_UNNUMBERED_INFORMATION, "Frame type should be UI", err);
        ax25_unnumbered_information_frame_t *ui_frame = (ax25_unnumbered_information_frame_t*) frame;
        DEBUG_PRINT("Decoded destination callsign (expected ABCDEF)");
        DEBUG_VAR("Destination SSID (expected 7)", ui_frame->base.base.header.destination.ssid);
        DEBUG_VAR("Source SSID (expected 1)", ui_frame->base.base.header.source.ssid);
        DEBUG_BOOL("Poll/Final (expected false)", ui_frame->base.pf);
        DEBUG_VAR("Modifier (expected 0x03)", ui_frame->base.modifier);
        DEBUG_VAR("PID (expected 0xF0)", ui_frame->pid);
        DEBUG_VAR("Payload length (expected 4)", ui_frame->payload_len);
        DEBUG_FRAME("Decoded payload", ui_frame->payload, ui_frame->payload_len);
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
        DEBUG_FRAME("Re-encoded frame", encoded, len);
        COMPARE_FRAME(encoded, len, frame_data, sizeof(frame_data), "Frame re-encoding should match");
        free(encoded);
        ax25_frame_free(frame, &err);
    }
    DEBUG_PRINT("Frame function tests complete");

    return 0;
}

int test_raw_frame_functions() {
    printf("test_raw_frame_functions\n");

    uint8_t err = 0;

    // Test data: Raw frame with header and payload, control byte set to 0x00 (I-frame)
    uint8_t frame_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63, 0x00, 0xF0, 'T', 'E', 'S', 'T' };
    DEBUG_FRAME("Raw frame input (control=0x00, MODULO128_NONE)", frame_data, sizeof(frame_data));
    ax25_frame_t *frame = ax25_frame_decode(frame_data, sizeof(frame_data), MODULO128_NONE, &err);
    TEST_ASSERT(frame != NULL, "ax25_frame_decode should return non-NULL", err);
    DEBUG_VAR("Frame type (expected RAW)", frame ? frame->type : 0xFF);
    if (frame) {
        TEST_ASSERT(frame->type == AX25_FRAME_RAW, "Frame type should be RAW", err);
        ax25_raw_frame_t *raw_frame = (ax25_raw_frame_t*) frame;
        DEBUG_VAR("Control byte (expected 0x00)", raw_frame->control);
        DEBUG_VAR("Payload length (expected 5)", raw_frame->payload_len);
        DEBUG_FRAME("Decoded raw payload", raw_frame->payload, raw_frame->payload_len);
        TEST_ASSERT(raw_frame->control == 0x00, "Control should be 0x00", err);
        TEST_ASSERT(raw_frame->payload_len == 5, "Payload length should be 5", err);
        TEST_ASSERT(memcmp(raw_frame->payload, "\xF0TEST", 5) == 0, "Payload should be 0xF0 followed by 'TEST'", err);

        size_t len;
        uint8_t *encoded = ax25_raw_frame_encode(raw_frame, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_raw_frame_encode should return non-NULL", err);
        DEBUG_VAR("Encoded length (expected 6)", len);
        DEBUG_FRAME("Re-encoded raw frame", encoded, len);
        TEST_ASSERT(len == 6, "Encoded length should be 6 (control + payload)", err);
        TEST_ASSERT(memcmp(encoded, "\x00\xF0TEST", 6) == 0, "Encoded raw frame should match control + payload", err);
        free(encoded);
        ax25_frame_free(frame, &err);
    }
    DEBUG_PRINT("Raw frame function tests complete");

    return 0;
}

int test_unnumbered_frame_functions() {
    printf("test_unnumbered_frame_functions\n");

    uint8_t err = 0;

    // Test data: Header for "ABCDEF-7" -> "GHIJKL-1*"
    uint8_t header_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63 };
    DEBUG_FRAME("Header bytes", header_data, sizeof(header_data));
    ax25_frame_header_t *header = ax25_frame_header_decode(header_data, sizeof(header_data), &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header) {
        // Test UI frame with PID=0xF0, payload="TEST"
        uint8_t dummy_info_field[] = { 0xF0, 'T', 'E', 'S', 'T' };
        DEBUG_FRAME("Info field (PID + payload)", dummy_info_field, sizeof(dummy_info_field));
        ax25_unnumbered_frame_t *u_frame = ax25_unnumbered_frame_decode(header, 0x13, dummy_info_field, sizeof(dummy_info_field), &err);  // 0x13 = UI with P/F=1
        TEST_ASSERT(u_frame != NULL, "ax25_unnumbered_frame_decode should return non-NULL", err);
        if (u_frame) {
            DEBUG_VAR("Frame type (expected UI)", u_frame->base.type);
            DEBUG_BOOL("Poll/Final (expected true)", ((ax25_unnumbered_information_frame_t*)u_frame)->base.pf);
            DEBUG_VAR("Modifier (expected 0x03)", ((ax25_unnumbered_information_frame_t*)u_frame)->base.modifier);
            DEBUG_VAR("PID (expected 0xF0)", ((ax25_unnumbered_information_frame_t*)u_frame)->pid);
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
            DEBUG_FRAME("Encoded UI frame (control+PID+payload)", encoded, len);
            uint8_t expected[] = { 0x13, 0xF0, 'T', 'E', 'S', 'T' };
            COMPARE_FRAME(encoded, len, expected, sizeof(expected), "Encoded UI frame should match");
            free(encoded);
            ax25_frame_free((ax25_frame_t*) u_frame, &err);
        }
        ax25_frame_header_free(header, &err);
    }
    DEBUG_PRINT("Unnumbered frame function tests complete");

    return 0;
}

int test_unnumbered_information_frame_functions() {
    printf("test_unnumbered_information_frame_functions\n");

    uint8_t err = 0;

    // Test data: Header for "ABCDEF-7" -> "GHIJKL-1*"
    uint8_t header_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63 };
    DEBUG_FRAME("Header bytes", header_data, sizeof(header_data));
    ax25_frame_header_t *header = ax25_frame_header_decode(header_data, sizeof(header_data), &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header) {
        uint8_t info[] = { 0xF0, 'T', 'E', 'S', 'T' };
        DEBUG_FRAME("Info field", info, sizeof(info));
        ax25_unnumbered_information_frame_t *ui_frame = ax25_unnumbered_information_frame_decode(header, true, info, sizeof(info), &err);
        TEST_ASSERT(ui_frame != NULL, "ax25_unnumbered_information_frame_decode should return non-NULL", err);
        if (ui_frame) {
            DEBUG_BOOL("Poll/Final (expected true)", ui_frame->base.pf);
            DEBUG_VAR("Modifier (expected 0x03)", ui_frame->base.modifier);
            DEBUG_VAR("PID (expected 0xF0)", ui_frame->pid);
            DEBUG_VAR("Payload length (expected 4)", ui_frame->payload_len);
            DEBUG_FRAME("Decoded payload", ui_frame->payload, ui_frame->payload_len);
            TEST_ASSERT(ui_frame->base.pf == true, "Poll/Final should be true", err);
            TEST_ASSERT(ui_frame->base.modifier == 0x03, "Modifier should be 0x03", err);
            TEST_ASSERT(ui_frame->pid == 0xF0, "PID should be 0xF0", err);
            TEST_ASSERT(ui_frame->payload_len == 4, "Payload length should be 4", err);
            TEST_ASSERT(memcmp(ui_frame->payload, "TEST", 4) == 0, "Payload should be 'TEST'", err);

            size_t len;
            uint8_t *encoded = ax25_unnumbered_information_frame_encode(ui_frame, &len, &err);
            TEST_ASSERT(encoded != NULL, "ax25_unnumbered_information_frame_encode should return non-NULL", err);
            DEBUG_FRAME("Encoded UI frame", encoded, len);
            uint8_t expected[] = { 0x13, 0xF0, 'T', 'E', 'S', 'T' };
            COMPARE_FRAME(encoded, len, expected, sizeof(expected), "Encoded UI frame should match");
            free(encoded);
            ax25_frame_free((ax25_frame_t*) ui_frame, &err);
        }
        ax25_frame_header_free(header, &err);
    }
    DEBUG_PRINT("Unnumbered information frame function tests complete");

    return 0;
}

int test_frame_reject_frame_functions() {
    printf("test_frame_reject_frame_functions\n");

    uint8_t err = 0;

    // Test data: Header for "AAAAAA-0" -> "BBBBBB-0"
    uint8_t header_data[] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x60, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x61 };
    DEBUG_FRAME("Header bytes", header_data, sizeof(header_data));
    ax25_frame_header_t *header = ax25_frame_header_decode(header_data, sizeof(header_data), &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header) {
        // FRMR data: w=1, x=0, y=0, z=0, vr=0, frmr_cr=0, vs=2, frmr_control=0x0A
        uint8_t frmr_data[] = { 0x0A, 0x04, 0x01 };
        DEBUG_FRAME("FRMR info bytes", frmr_data, sizeof(frmr_data));
        ax25_frame_reject_frame_t *frmr_frame = ax25_frame_reject_frame_decode(header, false, frmr_data, sizeof(frmr_data), &err);
        TEST_ASSERT(frmr_frame != NULL, "ax25_frame_reject_frame_decode should return non-NULL", err);
        if (frmr_frame) {
            DEBUG_BOOL("Poll/Final (expected false)", frmr_frame->base.pf);
            DEBUG_VAR("Modifier (expected 0x87)", frmr_frame->base.modifier);
            DEBUG_BOOL("w flag (expected true)", frmr_frame->w);
            DEBUG_BOOL("x flag (expected false)", frmr_frame->x);
            DEBUG_BOOL("y flag (expected false)", frmr_frame->y);
            DEBUG_BOOL("z flag (expected false)", frmr_frame->z);
            DEBUG_VAR("vr (expected 0)", frmr_frame->vr);
            DEBUG_BOOL("frmr_cr (expected false)", frmr_frame->frmr_cr);
            DEBUG_VAR("vs (expected 2)", frmr_frame->vs);
            DEBUG_VAR("frmr_control (expected 0x0A)", frmr_frame->frmr_control);
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
            DEBUG_FRAME("Encoded FRMR frame", encoded, len);
            uint8_t expected[] = { 0x87, 0x0A, 0x04, 0x01 };
            COMPARE_FRAME(encoded, len, expected, sizeof(expected), "Encoded FRMR frame should match");
            free(encoded);
            ax25_frame_free((ax25_frame_t*) frmr_frame, &err);
        }
        ax25_frame_header_free(header, &err);
    }
    DEBUG_PRINT("Frame reject (FRMR) function tests complete");

    return 0;
}

int test_information_frame_functions() {
    printf("test_information_frame_functions\n");

    uint8_t err = 0;

    // Test data: Header for "ABCDEF-7" -> "GHIJKL-1*"
    uint8_t header_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63 };
    DEBUG_FRAME("Header bytes", header_data, sizeof(header_data));
    ax25_frame_header_t *header = ax25_frame_header_decode(header_data, sizeof(header_data), &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header) {
        // I-frame data: control=0x10 (nr=0, pf=1, ns=0), PID=0xF0, payload="TEST"
        uint8_t info[] = { 0xF0, 'T', 'E', 'S', 'T' };
        DEBUG_FRAME("I-frame info field (PID + payload)", info, sizeof(info));
        DEBUG_PRINT("Control byte: 0x10 (nr=0, pf=1, ns=0), is_modulo128=false");
        ax25_information_frame_t *i_frame = ax25_information_frame_decode(header, 0x10, info, sizeof(info), false, &err);
        TEST_ASSERT(i_frame != NULL, "ax25_information_frame_decode should return non-NULL", err);
        if (i_frame) {
            DEBUG_VAR("Frame type (expected 8-bit I)", i_frame->base.type);
            DEBUG_VAR("nr (expected 0)", i_frame->nr);
            DEBUG_BOOL("Poll/Final (expected true)", i_frame->pf);
            DEBUG_VAR("ns (expected 0)", i_frame->ns);
            DEBUG_VAR("PID (expected 0xF0)", i_frame->pid);
            DEBUG_VAR("Payload length (expected 4)", i_frame->payload_len);
            DEBUG_FRAME("Decoded payload", i_frame->payload, i_frame->payload_len);
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
            DEBUG_FRAME("Encoded I-frame (control+PID+payload)", encoded, len);
            uint8_t expected[] = { 0x10, 0xF0, 'T', 'E', 'S', 'T' };
            COMPARE_FRAME(encoded, len, expected, sizeof(expected), "Encoded I-frame should match");
            free(encoded);
            ax25_frame_free((ax25_frame_t*) i_frame, &err);
        }
        ax25_frame_header_free(header, &err);
    }
    DEBUG_PRINT("Information frame function tests complete");

    return 0;
}

int test_supervisory_frame_functions() {
    printf("test_supervisory_frame_functions\n");

    uint8_t err = 0;

    uint8_t hdr_bytes[] = { 0x82, 0xA0, 0xA4, 0xA6, 0x40, 0x40, 0xE0, 0x9C, 0x9E, 0x86, 0x82, 0x98, 0x98, 0xE1 };
    DEBUG_FRAME("Header bytes", hdr_bytes, sizeof(hdr_bytes));
    ax25_frame_header_t *header = ax25_frame_header_decode(hdr_bytes, 14, &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header == NULL)
        return 1;

    DEBUG_PRINT("Control byte: 0x21 (RR, N(R)=1, P/F=0)");
    ax25_supervisory_frame_t *s_frame = ax25_supervisory_frame_decode(header, 0x21, false, &err);  // RR with nr=1
    TEST_ASSERT(s_frame != NULL, "ax25_supervisory_frame_decode should return non-NULL", err);
    if (s_frame) {
        DEBUG_VAR("nr (expected 1)", s_frame->nr);
        DEBUG_VAR("code (expected 0x00 = RR)", s_frame->code);
        DEBUG_BOOL("Poll/Final (expected false)", s_frame->pf);
        DEBUG_VAR("Frame type (expected RR_8BIT)", s_frame->base.type);
        TEST_ASSERT(s_frame->nr == 1, "nr should be 1", err);  // Explicitly test nr
        TEST_ASSERT(s_frame->code == 0x00, "Supervisory code should be 0x00 (RR)", err);
        TEST_ASSERT(s_frame->pf == false, "Poll/Final bit should be false", err);
        TEST_ASSERT(s_frame->base.type == AX25_FRAME_SUPERVISORY_RR_8BIT, "Frame type should be RR_8BIT", err);

        size_t len;
        uint8_t *encoded = ax25_supervisory_frame_encode(s_frame, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_supervisory_frame_encode should return non-NULL", err);
        DEBUG_FRAME("Encoded supervisory frame", encoded, len);
        DEBUG_VAR("Encoded length (expected 1)", len);
        DEBUG_VAR("Encoded control byte (expected 0x21)", encoded ? encoded[0] : 0xFF);
        TEST_ASSERT(len == 1, "Encoded length should be 1 byte", err);
        TEST_ASSERT(encoded[0] == 0x21, "Encoded control byte should be 0x21", err);
        if (encoded)
            free(encoded);
        ax25_frame_free((ax25_frame_t*) s_frame, &err);
    }
    ax25_frame_header_free(header, &err);
    TEST_ASSERT(err == 0, "Freeing header", err);
    DEBUG_PRINT("Supervisory frame function tests complete");

    return err ? 1 : 0;
}

int test_xid_parameter_functions() {
    printf("test_xid_parameter_functions\n");

    uint8_t err = 0;

    uint8_t pv[] = { 0x01, 0x02 };
    DEBUG_FRAME("Raw parameter PV", pv, sizeof(pv));
    ax25_xid_parameter_t *param = ax25_xid_raw_parameter_new(1, pv, 2, &err);
    TEST_ASSERT(param != NULL, "ax25_xid_raw_parameter_new should return non-NULL", err);
    if (param) {
        size_t len;
        uint8_t *encoded = ax25_xid_raw_parameter_encode(param, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_xid_raw_parameter_encode should return non-NULL", err);
        if (encoded) {
            DEBUG_FRAME("Encoded XID raw parameter", encoded, len);
            size_t consumed;
            ax25_xid_parameter_t *decoded = ax25_xid_parameter_decode(encoded, len, &consumed, &err);
            TEST_ASSERT(decoded != NULL, "ax25_xid_parameter_decode should return non-NULL", err);
            DEBUG_VAR("Consumed bytes", consumed);
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

    DEBUG_PRINT("Testing Class of Procedures parameter (PI=2): half-duplex=1, full-duplex=0");
    param = ax25_xid_class_of_procedures_new(true, false, true, false, false, true, false, 0, &err);
    TEST_ASSERT(param != NULL, "ax25_xid_class_of_procedures_new should return non-NULL", err);
    if (param) {
        size_t len;
        uint8_t *encoded = ax25_xid_raw_parameter_encode(param, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_xid_raw_parameter_encode should return non-NULL", err);
        if (encoded) {
            DEBUG_FRAME("Encoded Class of Procedures parameter", encoded, len);
            uint8_t expected[] = { 0x01, 0x02, 0x25, 0x00 };  // PI=1, PL=2, PV=0x25,0x00
            size_t expected_len = sizeof(expected);
            COMPARE_FRAME(encoded, len, expected, expected_len, "Class of Procedures parameter encoding");
            free(encoded);
        }
        ax25_xid_raw_parameter_free(param, &err);
    }

    DEBUG_PRINT("Testing HDLC Optional Functions parameter (PI=3)");
    param = ax25_xid_hdlc_optional_functions_new(true, false, true, false, true, false, true, false, true,
    false, false, false, false, false, false, false, false,
    false, false, false, false, 0, false, &err);
    TEST_ASSERT(param != NULL, "ax25_xid_hdlc_optional_functions_new should return non-NULL", err);
    if (param)
        ax25_xid_raw_parameter_free(param, &err);

    DEBUG_PRINT("Testing big-endian parameter: PI=1, value=0x12345678, 4 bytes");
    param = ax25_xid_big_endian_new(1, 0x12345678, 4, &err);
    TEST_ASSERT(param != NULL, "ax25_xid_big_endian_new should return non-NULL", err);
    if (param)
        ax25_xid_raw_parameter_free(param, &err);

    ax25_xid_init_defaults(&err);  // No return value to check
    ax25_xid_deinit_defaults(&err);
    printf("\033[0;32m[%04d]    PASS: ax25_xid_init_defaults executed\033[0m\n", ++assert_count);
    DEBUG_PRINT("XID parameter function tests complete");

    return 0;
}

int test_exchange_identification_frame_functions() {
    printf("test_exchange_identification_frame_functions\n");

    uint8_t err = 0;

    // Test data: Header for "ABCDEF-7" -> "GHIJKL-1*"
    uint8_t header_data[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63 };
    DEBUG_FRAME("Header bytes", header_data, sizeof(header_data));
    ax25_frame_header_t *header = ax25_frame_header_decode(header_data, sizeof(header_data), &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header) {
        // XID data: FI=0x82, GI=0x80, GL=0x0004, param PI=0x01, PL=0x02, PV={0x41, 0x00}
        uint8_t data[] = { 0x82, 0x80, 0x00, 0x04, 0x01, 0x02, 0x41, 0x00 };
        DEBUG_FRAME("XID data bytes (FI=0x82 GI=0x80 GL=4 PI=1 PL=2 PV=41,00)", data, sizeof(data));
        ax25_exchange_identification_frame_t *xid_frame = ax25_exchange_identification_frame_decode(header, true, data, sizeof(data), &err);
        TEST_ASSERT(xid_frame != NULL, "ax25_exchange_identification_frame_decode should return non-NULL", err);
        if (xid_frame) {
            DEBUG_BOOL("Poll/Final (expected true)", xid_frame->base.pf);
            DEBUG_VAR("Modifier (expected 0xAF)", xid_frame->base.modifier);
            DEBUG_VAR("FI (expected 0x82)", xid_frame->fi);
            DEBUG_VAR("GI (expected 0x80)", xid_frame->gi);
            DEBUG_VAR("param_count (expected 1)", xid_frame->param_count);
            TEST_ASSERT(xid_frame->base.pf == true, "Poll/Final should be true", err);
            TEST_ASSERT(xid_frame->base.modifier == 0xAF, "Modifier should be 0xAF", err);
            TEST_ASSERT(xid_frame->fi == 0x82, "FI should be 0x82", err);
            TEST_ASSERT(xid_frame->gi == 0x80, "GI should be 0x80", err);
            TEST_ASSERT(xid_frame->param_count == 1, "Should have 1 parameter", err);
            if (xid_frame->param_count > 0) {
                DEBUG_VAR("Parameter[0] PI (expected 0x01)", xid_frame->parameters[0]->pi);
                TEST_ASSERT(xid_frame->parameters[0]->pi == 0x01, "Parameter PI should be 0x01", err);
                ax25_raw_param_data_t *param_data = (ax25_raw_param_data_t*) xid_frame->parameters[0]->data;
                DEBUG_VAR("Parameter PV length (expected 2)", param_data ? param_data->pv_len : 0xFF);
                TEST_ASSERT(param_data->pv_len == 2, "Parameter PV length should be 2", err);
                TEST_ASSERT(memcmp(param_data->pv, "\x41\x00", 2) == 0, "Parameter PV should be {0x41, 0x00}", err);
            }

            size_t len;
            uint8_t *encoded = ax25_exchange_identification_frame_encode(xid_frame, &len, &err);
            TEST_ASSERT(encoded != NULL, "ax25_exchange_identification_frame_encode should return non-NULL", err);
            DEBUG_FRAME("Encoded XID frame", encoded, len);
            uint8_t expected[] = { 0xBF, 0x82, 0x80, 0x00, 0x04, 0x01, 0x02, 0x41, 0x00 };
            COMPARE_FRAME(encoded, len, expected, sizeof(expected), "Encoded XID frame should match");
            free(encoded);
            ax25_frame_free((ax25_frame_t*) xid_frame, &err);
        }
        ax25_frame_header_free(header, &err);
    }
    DEBUG_PRINT("XID frame function tests complete");

    return 0;
}

int test_test_frame_functions() {
    printf("test_test_frame_functions\n");

    uint8_t err = 0;

    uint8_t hdr_bytes[] = { 0x82, 0xA0, 0xA4, 0xA6, 0x40, 0x40, 0xE0, 0x9C, 0x9E, 0x86, 0x82, 0x98, 0x98, 0xE1 };
    DEBUG_FRAME("Header bytes", hdr_bytes, sizeof(hdr_bytes));
    ax25_frame_header_t *header = ax25_frame_header_decode(hdr_bytes, 14, &err).header;
    TEST_ASSERT(header != NULL, "ax25_frame_header_decode should return non-NULL", err);
    if (header == NULL)
        return 1;

    uint8_t data[] = "TEST";
    DEBUG_FRAME("TEST frame payload", data, 4);
    ax25_test_frame_t *test_frame = ax25_test_frame_decode(header, true, data, 4, &err);
    TEST_ASSERT(test_frame != NULL, "ax25_test_frame_decode should return non-NULL", err);
    if (test_frame) {
        DEBUG_VAR("Payload length (expected 4)", test_frame->payload_len);
        DEBUG_FRAME("Decoded payload", test_frame->payload, test_frame->payload_len);
        TEST_ASSERT(test_frame->payload_len == 4, "Payload length should be 4", err);
        TEST_ASSERT(memcmp(test_frame->payload, data, 4) == 0, "Payload should match 'TEST'", err);
        size_t len;
        uint8_t *encoded = ax25_test_frame_encode(test_frame, &len, &err);
        TEST_ASSERT(encoded != NULL, "ax25_test_frame_encode should return non-NULL", err);
        if (encoded) {
            DEBUG_FRAME("Encoded TEST frame (control+payload)", encoded, len);
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
    DEBUG_PRINT("TEST frame function tests complete");

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
    DEBUG_FRAME("Modulo-128 RR frame bytes", ax25_rr_frame_mod128, ax25_rr_frame_mod128_len);

    frame = ax25_frame_decode(ax25_rr_frame_mod128, ax25_rr_frame_mod128_len, 1, &err);
    TEST_ASSERT(frame != NULL, "Decoding modulo-128 RR frame", err);

    if (frame) {
        DEBUG_VAR("Frame type (expected RR 16-bit)", frame->type);
        TEST_ASSERT(frame->type == AX25_FRAME_SUPERVISORY_RR_16BIT, "Frame type should be RR 16-bit", err);
        ax25_supervisory_frame_t *s_frame = (ax25_supervisory_frame_t*) frame;
        DEBUG_VAR("nr (expected 4)", s_frame->nr);
        DEBUG_BOOL("Poll/Final (expected false)", s_frame->pf);
        DEBUG_VAR("code (expected 0x00 = RR)", s_frame->code);
        TEST_ASSERT(s_frame->nr == 4, "nr should be 4", err);
        TEST_ASSERT(s_frame->pf == false, "Poll/Final should be false", err);
        TEST_ASSERT(s_frame->code == 0x00, "Code should be 0x00 (RR)", err);
        ax25_frame_free(frame, &err);
    }

    DEBUG_PRINT("Modulo-128 tests complete");

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
    DEBUG_BOOL("Source res1 set to false (modulo-128 indicator)", src->res1);

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
        DEBUG_FRAME("Encoded modulo-128 I-frame", encoded, len);
        // Expected frame:
        // Dest: NOCALL-0, ch=1, res0=1, res1=1, ext=0: 0x9C, 0x9E, 0x86, 0x82, 0x98, 0x98, 0xE0
        // Src:  REPEAT-1, ch=0, res0=1, res1=0, ext=1: 0xA6, 0x8A, 0xA0, 0x8A, 0x82, 0xA2, 0x23
        // Control: 0x0A (N(S)=5, I=0), 0x07 (N(R)=3, P/F=1)
        // PID: 0xF0, Payload: "TEST"
        uint8_t expected[] = { 0x9C, 0x9E, 0x86, 0x82, 0x98, 0x98, 0xE0, 0xA6, 0x8A, 0xA0, 0x8A, 0x82, 0xA2, 0x23, 0x0A, 0x07, 0xF0, 'T', 'E', 'S', 'T' };
        size_t expected_len = sizeof(expected);
        DEBUG_FRAME("Expected encoded bytes", expected, expected_len);
        COMPARE_FRAME(encoded, len, expected, expected_len, "Modulo-128 I-frame encoding should match expected bytes");
        DEBUG_VAR("Source SSID byte[13] (expected 0x23, bit6=0 for mod128)", encoded[13]);
        TEST_ASSERT((encoded[13] & 0x40) == 0, "Source SSID bit 6 (res1) should be 0 for modulo-128", err);
        free(encoded);
    }

    // Clean up
    free(i_frame->payload);
    free(i_frame);
    ax25_address_free(dest, &err);
    ax25_address_free(src, &err);
    DEBUG_PRINT("Modulo-128 encode test complete");

    return 0;
}

int test_ax25_connection(void) {
    printf("test_ax25_connection\n");

    uint8_t err = 0;

    // AX.25 Connection Test Packets
    // 1. CONNECT Request (Station A -> Station B: SABM)
    // Dest: VA3BBB-7 (C=1, ext=0), Src: VA3AAA-1 (C=0, ext=1), Control: 0x3F (SABM, P=1)
    // Dest SSID byte: 0xAE (ch=1, res0=1, res1=0, ssid=7, ext=0)
    // Src SSID byte: 0x23 (ch=0, res0=1, res1=0, ssid=1, ext=1)
    unsigned char ax25_sabm_packet[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xAE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x23, 0x3F };
    size_t ax25_sabm_packet_len = sizeof(ax25_sabm_packet);
    DEBUG_FRAME("SABM packet", ax25_sabm_packet, ax25_sabm_packet_len);

    // 2. CONNECT Acknowledgment (Station B -> Station A: UA)
    // Dest: VA3AAA-1 (C=0, ext=0), Src: VA3BBB-7 (C=1, ext=1), Control: 0x73 (UA, F=1)
    // Dest SSID byte: 0x22 (ch=0, res0=1, res1=0, ssid=1, ext=0)
    // Src SSID byte: 0xAF (ch=1, res0=1, res1=0, ssid=7, ext=1)
    unsigned char ax25_ua_connect_packet[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x22, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xAF, 0x73 };
    size_t ax25_ua_connect_packet_len = sizeof(ax25_ua_connect_packet);
    DEBUG_FRAME("UA connect packet", ax25_ua_connect_packet, ax25_ua_connect_packet_len);

    // 3. SEND Data (Station A -> Station B: I-Frame)
    // Dest: VA3BBB-7, Src: VA3AAA-1, Control: 0x00 (I, N(S)=0, N(R)=0), PID: 0xF0, Payload: "Hello, World!"
    // Dest SSID byte: 0xAE (ch=1, res0=1, res1=0, ssid=7, ext=0)
    // Src SSID byte: 0x23 (ch=0, res0=1, res1=0, ssid=1, ext=1)
    unsigned char ax25_i_frame_packet[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xAE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x23, 0x00, 0xF0, 0x48, 0x65, 0x6C,
            0x6C, 0x6F, 0x2C, 0x20, 0x57, 0x6F, 0x72, 0x6C, 0x64, 0x21 };
    size_t ax25_i_frame_packet_len = sizeof(ax25_i_frame_packet);
    DEBUG_FRAME("I-frame packet", ax25_i_frame_packet, ax25_i_frame_packet_len);

    // 4. RECEIVE Data Acknowledgment (Station B -> Station A: RR)
    // Dest: VA3AAA-1, Src: VA3BBB-7, Control: 0x31 (RR, N(R)=1, P/F=1)
    // Dest SSID byte: 0x22 (ch=0, res0=1, res1=0, ssid=1, ext=0)
    // Src SSID byte: 0xAF (ch=1, res0=1, res1=0, ssid=7, ext=1)
    unsigned char ax25_rr_packet[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x22, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xAF, 0x31 };
    size_t ax25_rr_packet_len = sizeof(ax25_rr_packet);
    DEBUG_FRAME("RR packet", ax25_rr_packet, ax25_rr_packet_len);

    // 5. DISCONNECT Request (Station A -> Station B: DISC)
    // Dest: VA3BBB-7, Src: VA3AAA-1, Control: 0x43 (DISC, P=0)
    // Dest SSID byte: 0xAE (ch=1, res0=1, res1=0, ssid=7, ext=0)
    // Src SSID byte: 0x23 (ch=0, res0=1, res1=0, ssid=1, ext=1)
    unsigned char ax25_disc_packet[] = {  //
            0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xAE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x23, 0x43 };
    size_t ax25_disc_packet_len = sizeof(ax25_disc_packet);
    DEBUG_FRAME("DISC packet", ax25_disc_packet, ax25_disc_packet_len);

    // 6. DISCONNECT Acknowledgment (Station B -> Station A: UA)
    // Dest: VA3AAA-1, Src: VA3BBB-7, Control: 0x73 (UA, F=1)
    // Dest SSID byte: 0x22 (ch=0, res0=1, res1=0, ssid=1, ext=0)
    // Src SSID byte: 0xAF (ch=1, res0=1, res1=0, ssid=7, ext=1)
    unsigned char ax25_ua_disconnect_packet[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x22, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xAF, 0x73 };
    size_t ax25_ua_disconnect_packet_len = sizeof(ax25_ua_disconnect_packet);
    DEBUG_FRAME("UA disconnect packet", ax25_ua_disconnect_packet, ax25_ua_disconnect_packet_len);

    // Invalid packet for error testing
    unsigned char invalid_packet[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xAE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x23, 0xFF };
    size_t invalid_packet_len = sizeof(invalid_packet);
    DEBUG_FRAME("Invalid control packet (0xFF, should fail)", invalid_packet, invalid_packet_len);

    unsigned char short_packet[] = { 0xAC, 0x82, 0x66 };
    size_t short_packet_len = sizeof(short_packet);
    DEBUG_FRAME("Short packet (should fail)", short_packet, short_packet_len);

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
    DEBUG_PRINT("--- Step 1: SABM frame ---");
    decoded_frame = ax25_frame_decode(ax25_sabm_packet, ax25_sabm_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding SABM frame", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected SABM)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_SABM, "Frame type should be SABM", err);
        ax25_unnumbered_frame_t *u_frame = (ax25_unnumbered_frame_t*) decoded_frame;
        DEBUG_BOOL("Poll/Final (expected true)", u_frame->pf);
        DEBUG_VAR("Modifier (expected 0x2F)", u_frame->modifier);
        TEST_ASSERT(u_frame->pf == true, "Poll/Final should be true", err);
        TEST_ASSERT(u_frame->modifier == 0x2F, "Modifier should be 0x2F", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding SABM frame", err);
        DEBUG_FRAME("Re-encoded SABM", encode_result, encoded_len);
        COMPARE_FRAME(encode_result, encoded_len, ax25_sabm_packet, ax25_sabm_packet_len, "SABM frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 2. Test UA connect frame
    DEBUG_PRINT("--- Step 2: UA connect frame ---");
    decoded_frame = ax25_frame_decode(ax25_ua_connect_packet, ax25_ua_connect_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding UA connect frame", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected UA)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_UA, "Frame type should be UA", err);
        ax25_unnumbered_frame_t *u_frame = (ax25_unnumbered_frame_t*) decoded_frame;
        DEBUG_BOOL("Poll/Final (expected true)", u_frame->pf);
        DEBUG_VAR("Modifier (expected 0x63)", u_frame->modifier);
        TEST_ASSERT(u_frame->pf == true, "Poll/Final should be true", err);
        TEST_ASSERT(u_frame->modifier == 0x63, "Modifier should be 0x63", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding UA connect frame", err);
        DEBUG_FRAME("Re-encoded UA connect", encode_result, encoded_len);
        COMPARE_FRAME(encode_result, encoded_len, ax25_ua_connect_packet, ax25_ua_connect_packet_len, "UA connect frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 3. Test I-Frame
    DEBUG_PRINT("--- Step 3: I-frame (Hello, World!) ---");
    decoded_frame = ax25_frame_decode(ax25_i_frame_packet, ax25_i_frame_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding I-Frame", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected I-frame 8-bit)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_INFORMATION_8BIT, "Frame type should be I-frame 8-bit", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) decoded_frame;
        DEBUG_VAR("nr (expected 0)", i_frame->nr);
        DEBUG_VAR("ns (expected 0)", i_frame->ns);
        DEBUG_BOOL("Poll/Final (expected false)", i_frame->pf);
        DEBUG_VAR("PID (expected 0xF0)", i_frame->pid);
        DEBUG_VAR("Payload length (expected 13)", i_frame->payload_len);
        DEBUG_FRAME("Decoded payload", i_frame->payload, i_frame->payload_len);
        TEST_ASSERT(i_frame->nr == 0, "nr should be 0", err);
        TEST_ASSERT(i_frame->ns == 0, "ns should be 0", err);
        TEST_ASSERT(i_frame->pf == false, "Poll/Final should be false", err);
        TEST_ASSERT(i_frame->pid == 0xF0, "PID should be 0xF0", err);
        TEST_ASSERT(i_frame->payload_len == 13, "Payload length should be 13", err);
        TEST_ASSERT(memcmp(i_frame->payload, "Hello, World!", 13) == 0, "Payload should be 'Hello, World!'", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding I-Frame", err);
        DEBUG_FRAME("Re-encoded I-frame", encode_result, encoded_len);
        COMPARE_FRAME(encode_result, encoded_len, ax25_i_frame_packet, ax25_i_frame_packet_len, "I-Frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 4. Test RR frame
    DEBUG_PRINT("--- Step 4: RR frame (ACK) ---");
    decoded_frame = ax25_frame_decode(ax25_rr_packet, ax25_rr_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding RR frame", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected RR 8-bit)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_SUPERVISORY_RR_8BIT, "Frame type should be RR 8-bit", err);
        ax25_supervisory_frame_t *s_frame = (ax25_supervisory_frame_t*) decoded_frame;
        DEBUG_VAR("nr (expected 1)", s_frame->nr);
        DEBUG_BOOL("Poll/Final (expected true)", s_frame->pf);
        DEBUG_VAR("code (expected 0x00 = RR)", s_frame->code);
        TEST_ASSERT(s_frame->nr == 1, "nr should be 1", err);
        TEST_ASSERT(s_frame->pf == true, "Poll/Final should be true", err);
        TEST_ASSERT(s_frame->code == 0x00, "Code should be 0x00 (RR)", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding RR frame", err);
        DEBUG_FRAME("Re-encoded RR frame", encode_result, encoded_len);
        COMPARE_FRAME(encode_result, encoded_len, ax25_rr_packet, ax25_rr_packet_len, "RR frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 5. Test DISC frame
    DEBUG_PRINT("--- Step 5: DISC frame ---");
    decoded_frame = ax25_frame_decode(ax25_disc_packet, ax25_disc_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding DISC frame", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected DISC)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_DISC, "Frame type should be DISC", err);
        ax25_unnumbered_frame_t *u_frame = (ax25_unnumbered_frame_t*) decoded_frame;
        DEBUG_BOOL("Poll/Final (expected false)", u_frame->pf);
        DEBUG_VAR("Modifier (expected 0x43)", u_frame->modifier);
        TEST_ASSERT(u_frame->pf == false, "Poll/Final should be false", err);
        TEST_ASSERT(u_frame->modifier == 0x43, "Modifier should be 0x43", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding DISC frame", err);
        DEBUG_FRAME("Re-encoded DISC", encode_result, encoded_len);
        COMPARE_FRAME(encode_result, encoded_len, ax25_disc_packet, ax25_disc_packet_len, "DISC frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 6. Test UA disconnect frame
    DEBUG_PRINT("--- Step 6: UA disconnect frame ---");
    decoded_frame = ax25_frame_decode(ax25_ua_disconnect_packet, ax25_ua_disconnect_packet_len, 0, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding UA disconnect frame", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected UA)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_UA, "Frame type should be UA", err);
        ax25_unnumbered_frame_t *u_frame = (ax25_unnumbered_frame_t*) decoded_frame;
        DEBUG_BOOL("Poll/Final (expected true)", u_frame->pf);
        DEBUG_VAR("Modifier (expected 0x63)", u_frame->modifier);
        TEST_ASSERT(u_frame->pf == true, "Poll/Final should be true", err);
        TEST_ASSERT(u_frame->modifier == 0x63, "Modifier should be 0x63", err);
        encode_result = ax25_frame_encode(decoded_frame, &encoded_len, &err);
        TEST_ASSERT(encode_result != NULL && err == 0, "Encoding UA disconnect frame", err);
        DEBUG_FRAME("Re-encoded UA disconnect", encode_result, encoded_len);
        COMPARE_FRAME(encode_result, encoded_len, ax25_ua_disconnect_packet, ax25_ua_disconnect_packet_len, "UA disconnect frame content");
        free(encode_result);
        ax25_frame_free(decoded_frame, &err);
    }

    // 7. Error Case: Invalid control byte
    DEBUG_PRINT("--- Step 7: Invalid control byte (0xFF, should fail) ---");
    decoded_frame = ax25_frame_decode(invalid_packet, invalid_packet_len, 0, &err);
    DEBUG_VAR("Error code from invalid control decode", err);
    TEST_ASSERT(decoded_frame == NULL && err != 0, "Decoding invalid control frame should fail", err);

    // 8. Error Case: Short frame
    DEBUG_PRINT("--- Step 8: Short frame (should fail) ---");
    decoded_frame = ax25_frame_decode(short_packet, short_packet_len, 0, &err);
    DEBUG_VAR("Error code from short frame decode", err);
    TEST_ASSERT(decoded_frame == NULL && err != 0, "Decoding short frame should fail", err);

    // 9. Error Case: Null input
    DEBUG_PRINT("--- Step 9: NULL input (should fail) ---");
    decoded_frame = ax25_frame_decode(NULL, 0, 0, &err);
    DEBUG_VAR("Error code from NULL decode", err);
    TEST_ASSERT(decoded_frame == NULL && err != 0, "Decoding null input should fail", err);

    // Clean up addresses
    ax25_address_free(station_a, &addr_err);
    ax25_address_free(station_b, &addr_err);
    DEBUG_PRINT("Connection sequence tests complete");

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
    if (!frame_mod8) {
        TEST_ASSERT(false, "FRMR frame buffer allocation should succeed", err);
        return 1;
    }
    memcpy(frame_mod8, header_mod8, header_len);
    frame_mod8[header_len] = control_byte;
    memcpy(frame_mod8 + header_len + 1, frmr_data_mod8, frmr_data_len);
    DEBUG_FRAME("Constructed FRMR frame", frame_mod8, total_len);

    // Decode the frame
    ax25_frame_t *frame = ax25_frame_decode(frame_mod8, total_len, 0, &err);
    DEBUG_VAR("Error code from FRMR decode", err);
    TEST_ASSERT(frame != NULL, "FRMR frame decode should succeed", err);
    if (!frame) {
        free(frame_mod8);
        return 1;
    }

    // Verify the decoded FRMR frame
    ax25_frame_reject_frame_t *frmr = (ax25_frame_reject_frame_t*) frame;
    DEBUG_VAR("Frame type (expected FRMR)", frmr->base.base.type);
    DEBUG_BOOL("is_modulo128 (expected false)", frmr->is_modulo128);
    DEBUG_VAR("frmr_control (expected 0x10)", frmr->frmr_control);
    DEBUG_VAR("vr (expected 1)", frmr->vr);
    DEBUG_VAR("vs (expected 2)", frmr->vs);
    DEBUG_BOOL("frmr_cr (expected false)", frmr->frmr_cr);
    DEBUG_BOOL("w (expected true)", frmr->w);
    DEBUG_BOOL("x (expected false)", frmr->x);
    DEBUG_BOOL("y (expected false)", frmr->y);
    DEBUG_BOOL("z (expected false)", frmr->z);

    // Assertions using TEST_ASSERT for consistent output
    TEST_ASSERT(frmr->base.base.type == AX25_FRAME_UNNUMBERED_FRMR, "Frame type should be FRMR", err);
    TEST_ASSERT(!frmr->is_modulo128, "FRMR should be modulo-8 (is_modulo128=false)", err);
    TEST_ASSERT(frmr->frmr_control == 0x10, "FRMR frmr_control should be 0x10", err);
    TEST_ASSERT(frmr->vr == 1, "FRMR vr should be 1", err);
    TEST_ASSERT(frmr->vs == 2, "FRMR vs should be 2", err);
    TEST_ASSERT(!frmr->frmr_cr, "FRMR frmr_cr should be false", err);
    TEST_ASSERT(frmr->w, "FRMR w flag should be true (invalid control)", err);
    TEST_ASSERT(!frmr->x, "FRMR x flag should be false", err);
    TEST_ASSERT(!frmr->y, "FRMR y flag should be false", err);
    TEST_ASSERT(!frmr->z, "FRMR z flag should be false", err);

    // Clean up
    ax25_frame_free(frame, &err);
    free(frame_mod8);
    DEBUG_PRINT("FRMR frame function tests complete");

    return 0;
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
    DEBUG_FRAME("Modulo-8 I-frame (src SSID byte=0x63, res1=1)", frame_mod8, sizeof(frame_mod8));
    ax25_frame_t *frame = ax25_frame_decode(frame_mod8, sizeof(frame_mod8), MODULO128_AUTO, &err);
    TEST_ASSERT(frame != NULL, "Decoding modulo-8 I-frame with auto detection", err);
    if (frame) {
        DEBUG_VAR("Detected frame type (expected 8-bit I)", frame->type);
        TEST_ASSERT(frame->type == AX25_FRAME_INFORMATION_8BIT, "Should decode as 8-bit I-frame", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) frame;
        DEBUG_VAR("nr (expected 0)", i_frame->nr);
        DEBUG_VAR("ns (expected 0)", i_frame->ns);
        DEBUG_BOOL("pf (expected false)", i_frame->pf);
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
    DEBUG_FRAME("Modulo-128 I-frame (src SSID byte=0x23, res1=0)", frame_mod128, sizeof(frame_mod128));
    frame = ax25_frame_decode(frame_mod128, sizeof(frame_mod128), MODULO128_AUTO, &err);
    TEST_ASSERT(frame != NULL, "Decoding modulo-128 I-frame with auto detection", err);
    if (frame) {
        DEBUG_VAR("Detected frame type (expected 16-bit I)", frame->type);
        TEST_ASSERT(frame->type == AX25_FRAME_INFORMATION_16BIT, "Should decode as 16-bit I-frame", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) frame;
        DEBUG_VAR("nr (expected 0)", i_frame->nr);
        DEBUG_VAR("ns (expected 0)", i_frame->ns);
        DEBUG_BOOL("pf (expected false)", i_frame->pf);
        TEST_ASSERT(i_frame->nr == 0, "nr should be 0", err);
        TEST_ASSERT(i_frame->ns == 0, "ns should be 0", err);
        TEST_ASSERT(i_frame->pf == false, "pf should be false", err);
        ax25_frame_free(frame, &err);
    }

    DEBUG_PRINT("Auto modulo detection tests complete");

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
        TEST_ASSERT(false, "Payload allocation should succeed", err);
        return 1;
    }
    DEBUG_VAR64("Payload size (bytes)", payload_len);
    for (size_t i = 0; i < payload_len; i++) {
        payload[i] = (uint8_t) (i % 256);  // Fill with 0x00, 0x01, ..., 0xFF, 0x00, ...
    }
    DEBUG_PRINT("Payload filled with pattern 0x00..0xFF repeating");

    // Segment the payload with N1=256
    size_t n1 = 256;
    DEBUG_VAR64("N1 (max I-field size)", n1);
    size_t num_segments;
    ax25_segmented_info_t *segments = ax25_segment_info_fields(payload, payload_len, n1, &err, &num_segments);
    if (!segments || err != 0) {
        TEST_ASSERT(false, "Segmentation should succeed", err);
        result = 1;
        goto cleanup_payload;
    }
    DEBUG_VAR64("Number of segments produced", num_segments);

    // Check segment count
    TEST_ASSERT(num_segments == 40, "Segment count should be 40", err);
    if (num_segments != 40) {
        result = 1;
        goto cleanup_segments;
    }

    // Verify first segment
    DEBUG_PRINT("Verifying first segment header and data");
    DEBUG_VAR("First segment info_field_len (expected 256)", segments[0].info_field_len);
    DEBUG_VAR("First segment byte[0] (expected 0x08 = PID_SEGMENT_FRAGMENT)", segments[0].info_field[0]);
    DEBUG_VAR("First segment byte[1] (expected 0x80 = first+seq0)", segments[0].info_field[1]);
    DEBUG_VAR("First segment byte[2] (expected 0x27 = orig PID)", segments[0].info_field[2]);
    TEST_ASSERT(segments[0].info_field_len == 256, "First segment length should be 256", err);
    TEST_ASSERT(segments[0].info_field[0] == 0x08, "First segment PID byte should be 0x08", err);
    TEST_ASSERT(segments[0].info_field[1] == 0x80, "First segment header should be 0x80 (first+seq0)", err);
    TEST_ASSERT(segments[0].info_field[2] == 0x27, "First segment original PID should be 0x27", err);
    TEST_ASSERT(memcmp(segments[0].info_field + 4, payload, 252) == 0, "First segment data should match payload start", err);
    if (segments[0].info_field_len != 256 || segments[0].info_field[0] != 0x08 || segments[0].info_field[1] != 0x80 || segments[0].info_field[2] != 0x27
            || segments[0].info_field[3] != 0x10 || memcmp(segments[0].info_field + 4, payload, 252) != 0) {
        result = 1;
        goto cleanup_segments;
    }

    // Verify second segment
    DEBUG_PRINT("Verifying second segment header and data");
    DEBUG_VAR("Second segment info_field_len (expected 256)", segments[1].info_field_len);
    DEBUG_VAR("Second segment byte[1] (expected 0x01 = seq1)", segments[1].info_field[1]);
    TEST_ASSERT(segments[1].info_field_len == 256, "Second segment length should be 256", err);
    TEST_ASSERT(segments[1].info_field[1] == 0x01, "Second segment header byte should be 0x01 (seq1)", err);
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
    DEBUG_PRINT("Verifying last segment");
    DEBUG_VAR64("Last segment index", last_seg);
    DEBUG_VAR64("Last segment data len (expected 96)", last_data_len);
    DEBUG_VAR64("Last segment info len (expected 98)", last_info_len);
    DEBUG_VAR("Last segment byte[1] (expected 0x67 = last+seq39)", segments[last_seg].info_field[1]);
    TEST_ASSERT(segments[last_seg].info_field_len == last_info_len, "Last segment info field length should match", err);
    TEST_ASSERT(segments[last_seg].info_field[1] == 0x67, "Last segment header byte should be 0x67 (last+seq39)", err);
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
    DEBUG_PRINT("Overhead calculation:");
    DEBUG_VAR64("Total segment bytes", total_segment_bytes);
    DEBUG_VAR64("Original payload bytes", payload_len);
    // Use integer comparison to avoid floating point issues: overhead should be < 1%
    // total_segment_bytes - payload_len < payload_len/100
    TEST_ASSERT((total_segment_bytes - payload_len) * 100 < payload_len, "Segmentation overhead should be < 1%", err);
    if (overhead >= 1.0) {
        result = 1;
        goto cleanup_segments;
    }

    // Reassemble segments
    DEBUG_PRINT("Reassembling segments");
    size_t reassembled_len;
    uint8_t *reassembled = ax25_reassemble_info_fields(segments, num_segments, &reassembled_len, &err);
    DEBUG_VAR64("Reassembled length (expected 10000)", reassembled_len);
    TEST_ASSERT(reassembled != NULL, "Reassembly should produce non-NULL buffer", err);
    TEST_ASSERT(err == 0, "Reassembly should succeed with no error", err);
    TEST_ASSERT(reassembled_len == payload_len, "Reassembled length should match original payload", err);
    if (!reassembled || err != 0 || reassembled_len != payload_len || memcmp(reassembled, payload, payload_len) != 0) {
        result = 1;
        free(reassembled);
        goto cleanup_segments;
    }
    TEST_ASSERT(memcmp(reassembled, payload, payload_len) == 0, "Reassembled data should match original payload", err);

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
    DEBUG_PRINT("Segmentation/reassembly tests complete");

    return result;
}

void test_ax25_frame_print() {
    printf("test_ax25_frame_print\n");

    // UI frame
    unsigned char ui_frame[] = { 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0xEE, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x63, 0x03, 0xF0, 'T', 'E', 'S', 'T' };
    DEBUG_FRAME("UI frame raw bytes", ui_frame, sizeof(ui_frame));
    printf("UI Frame:\n");
    ax25_frame_print(ui_frame, sizeof(ui_frame));

    // I-frame
    unsigned char i_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x00, 0xF0, 'H', 'e', 'l', 'l', 'o', ',',
            ' ', 'W', 'o', 'r', 'l', 'd', '!' };
    DEBUG_FRAME("I-frame raw bytes", i_frame, sizeof(i_frame));
    printf("\nI-Frame:\n");
    ax25_frame_print(i_frame, sizeof(i_frame));

    // SABM frame
    unsigned char sabm_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x3F };
    DEBUG_FRAME("SABM frame raw bytes", sabm_frame, sizeof(sabm_frame));
    printf("\nSABM Frame:\n");
    ax25_frame_print(sabm_frame, sizeof(sabm_frame));

    // UA frame
    unsigned char ua_frame[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x62, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEF, 0x73 };
    DEBUG_FRAME("UA frame raw bytes", ua_frame, sizeof(ua_frame));
    printf("\nUA Frame:\n");
    ax25_frame_print(ua_frame, sizeof(ua_frame));

    // RR frame
    unsigned char rr_frame[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x62, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEF, 0x31 };
    DEBUG_FRAME("RR frame raw bytes", rr_frame, sizeof(rr_frame));
    printf("\nRR Frame:\n");
    ax25_frame_print(rr_frame, sizeof(rr_frame));

    // DISC frame
    unsigned char disc_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x43 };
    DEBUG_FRAME("DISC frame raw bytes", disc_frame, sizeof(disc_frame));
    printf("\nDISC Frame:\n");
    ax25_frame_print(disc_frame, sizeof(disc_frame));
}

int test_extended_i_frame() {
    printf("test_extended_i_frame\n");

    uint8_t err = 0;
    // Extended I-frame: Dest: VA3BBB-7, Src: VA3AAA-1 (res1=0), Control: 0x0000 (N(S)=0, N(R)=0, P/F=0), PID: 0xF0, Payload: "Extended"
    unsigned char extended_i_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x23, 0x00, 0x00, 0xF0, 'E', 'x', 't',
            'e', 'n', 'd', 'e', 'd' };
    size_t extended_i_frame_len = sizeof(extended_i_frame);
    DEBUG_FRAME("Extended I-frame bytes (src[13]=0x23, res1=0)", extended_i_frame, extended_i_frame_len);
    ax25_frame_t *decoded_frame = ax25_frame_decode(extended_i_frame, extended_i_frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding extended I-frame", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected 16-bit I)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_INFORMATION_16BIT, "Frame type should be I-frame 16-bit", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) decoded_frame;
        DEBUG_VAR("nr (expected 0)", i_frame->nr);
        DEBUG_VAR("ns (expected 0)", i_frame->ns);
        DEBUG_BOOL("Poll/Final (expected false)", i_frame->pf);
        DEBUG_VAR("PID (expected 0xF0)", i_frame->pid);
        DEBUG_VAR("Payload length (expected 8)", i_frame->payload_len);
        DEBUG_FRAME("Decoded payload", i_frame->payload, i_frame->payload_len);
        TEST_ASSERT(i_frame->nr == 0, "nr should be 0", err);
        TEST_ASSERT(i_frame->ns == 0, "ns should be 0", err);
        TEST_ASSERT(i_frame->pf == false, "Poll/Final should be false", err);
        TEST_ASSERT(i_frame->pid == 0xF0, "PID should be 0xF0", err);
        TEST_ASSERT(i_frame->payload_len == 8, "Payload length should be 8", err);
        TEST_ASSERT(memcmp(i_frame->payload, "Extended", 8) == 0, "Payload should be 'Extended'", err);
        ax25_frame_free(decoded_frame, &err);
    }
    DEBUG_PRINT("Extended I-frame test complete");

    return 0;
}

int test_sabme_frame() {
    printf("test_sabme_frame\n");

    uint8_t err = 0;

    // SABME frame: Dest: VA3BBB-7, Src: VA3AAA-1, Control: 0x7F (SABME, P=1)
    unsigned char sabme_frame[] = { 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xEE, 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x63, 0x7F };
    size_t sabme_frame_len = sizeof(sabme_frame);
    DEBUG_FRAME("SABME frame bytes", sabme_frame, sabme_frame_len);
    ax25_frame_t *decoded_frame = ax25_frame_decode(sabme_frame, sabme_frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding SABME frame", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected SABME)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_SABME, "Frame type should be SABME", err);
        ax25_unnumbered_frame_t *u_frame = (ax25_unnumbered_frame_t*) decoded_frame;
        DEBUG_BOOL("Poll/Final (expected true)", u_frame->pf);
        DEBUG_VAR("Modifier (expected 0x6F)", u_frame->modifier);
        TEST_ASSERT(u_frame->pf == true, "Poll/Final should be true", err);
        TEST_ASSERT(u_frame->modifier == 0x6F, "Modifier should be 0x6F", err);
        ax25_frame_free(decoded_frame, &err);
    }
    DEBUG_PRINT("SABME frame test complete");

    return 0;
}

int test_extended_s_frame() {
    printf("test_extended_s_frame\n");

    uint8_t err = 0;

    // Extended RR frame: Dest: VA3AAA-1, Src: VA3BBB-1 (res1=0), Control: 0x0100 (RR, N(R)=0, P/F=0)
    unsigned char extended_rr_frame[] = { 0xAC, 0x82, 0x66, 0x82, 0x82, 0x82, 0x62, 0xAC, 0x82, 0x66, 0x84, 0x84, 0x84, 0xA3, 0x01, 0x00 };
    size_t extended_rr_frame_len = sizeof(extended_rr_frame);
    DEBUG_FRAME("Extended RR frame bytes (src[13]=0xA3, res1=0)", extended_rr_frame, extended_rr_frame_len);
    ax25_frame_t *decoded_frame = ax25_frame_decode(extended_rr_frame, extended_rr_frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding extended RR frame", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected RR 16-bit)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_SUPERVISORY_RR_16BIT, "Frame type should be RR 16-bit", err);
        ax25_supervisory_frame_t *s_frame = (ax25_supervisory_frame_t*) decoded_frame;
        DEBUG_VAR("nr (expected 0)", s_frame->nr);
        DEBUG_BOOL("Poll/Final (expected false)", s_frame->pf);
        DEBUG_VAR("code (expected 0x00 = RR)", s_frame->code);
        TEST_ASSERT(s_frame->nr == 0, "nr should be 0", err);
        TEST_ASSERT(s_frame->pf == false, "Poll/Final should be false", err);
        TEST_ASSERT(s_frame->code == 0x00, "Code should be 0x00 (RR)", err);
        ax25_frame_free(decoded_frame, &err);
    }
    DEBUG_PRINT("Extended S-frame test complete");

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
    DEBUG_FRAME("Max repeaters frame", max_repeaters_frame, frame_len);
    ax25_frame_t *decoded_frame = ax25_frame_decode(max_repeaters_frame, frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding frame with maximum repeaters", err);
    if (decoded_frame) {
        DEBUG_VAR("num_repeaters (expected 8)", decoded_frame->header.repeaters.num_repeaters);
        TEST_ASSERT(decoded_frame->header.repeaters.num_repeaters == 8, "Should have 8 repeaters", err);
        ax25_frame_free(decoded_frame, &err);
    }
    DEBUG_PRINT("Max repeaters test complete");

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
    DEBUG_FRAME("Frame header section", large_payload_frame, 16);
    DEBUG_VAR64("Total frame length (expected 272)", frame_len);

    ax25_frame_t *decoded_frame = ax25_frame_decode(large_payload_frame, frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding frame with large payload", err);
    if (decoded_frame == NULL) {
        return 1;
    }

    TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_INFORMATION, "Frame type should be UI", err);
    if (decoded_frame->type != AX25_FRAME_UNNUMBERED_INFORMATION) {
        ax25_frame_free(decoded_frame, &err);
        return 1;
    }

    ax25_unnumbered_information_frame_t *ui_frame = (ax25_unnumbered_information_frame_t*) decoded_frame;
    DEBUG_VAR("Decoded payload length (expected 256)", ui_frame->payload_len);
    TEST_ASSERT(ui_frame->payload_len == 256, "Payload length should be 256", err);

    // Check the entire payload at once
    int cmp_result = memcmp(ui_frame->payload, large_payload_frame + 16, 256);
    if (cmp_result == 0) {
        printf("\033[0;32m[%04d]    PASS: Payload data matches\033[0m\n", ++assert_count);
    } else {
        // Find and report first mismatch byte with DEBUG info
        for (int i = 0; i < 256; i++) {
            if (ui_frame->payload[i] != large_payload_frame[16 + i]) {
                DEBUG_VAR("First mismatch at byte index", i);
                DEBUG_VAR("Expected byte", large_payload_frame[16 + i]);
                DEBUG_VAR("Got byte", ui_frame->payload[i]);
                break;
            }
        }
        TEST_ASSERT(false, "Payload data should match original 256-byte pattern", cmp_result);
    }
    ax25_frame_free(decoded_frame, &err);
    DEBUG_PRINT("Large payload test complete");

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
    DEBUG_FRAME("UI frame with no payload", frame, frame_len);
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding UI frame with no payload", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected UI)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_UNNUMBERED_INFORMATION, "Frame type should be UI", err);
        ax25_unnumbered_information_frame_t *ui_frame = (ax25_unnumbered_information_frame_t*) decoded_frame;
        DEBUG_VAR("Payload length (expected 0)", ui_frame->payload_len);
        TEST_ASSERT(ui_frame->payload_len == 0, "Payload length should be 0", err);
        ax25_frame_free(decoded_frame, &err);
    }
    DEBUG_PRINT("UI frame no-payload test complete");

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
    DEBUG_FRAME("Modulo-8 I-frame no payload (src[13]=0x61, res1=1)", frame, frame_len);
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_FALSE, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding I frame with no payload", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected I-frame 8-bit)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_INFORMATION_8BIT, "Frame type should be I frame (modulo 8)", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) decoded_frame;
        DEBUG_VAR("Payload length (expected 0)", i_frame->payload_len);
        DEBUG_VAR("ns (expected 0)", i_frame->ns);
        DEBUG_VAR("nr (expected 0)", i_frame->nr);
        DEBUG_BOOL("pf (expected false)", i_frame->pf);
        TEST_ASSERT(i_frame->payload_len == 0, "Payload length should be 0", err);
        TEST_ASSERT(i_frame->ns == 0, "N(S) should be 0", err);
        TEST_ASSERT(i_frame->nr == 0, "N(R) should be 0", err);
        TEST_ASSERT(i_frame->pf == false, "P/F should be 0", err);
        ax25_frame_free(decoded_frame, &err);
    }
    DEBUG_PRINT("I-frame no-payload test complete");

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
    DEBUG_FRAME("Modulo-128 I-frame no payload (src[13]=0x21, res1=0)", frame, frame_len);
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding I frame with no payload (modulo 128)", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type (expected I-frame 16-bit)", decoded_frame->type);
        TEST_ASSERT(decoded_frame->type == AX25_FRAME_INFORMATION_16BIT, "Frame type should be I frame (modulo 128)", err);
        ax25_information_frame_t *i_frame = (ax25_information_frame_t*) decoded_frame;
        DEBUG_VAR("Payload length (expected 0)", i_frame->payload_len);
        DEBUG_VAR("ns (expected 0)", i_frame->ns);
        DEBUG_VAR("nr (expected 0)", i_frame->nr);
        DEBUG_BOOL("pf (expected false)", i_frame->pf);
        TEST_ASSERT(i_frame->payload_len == 0, "Payload length should be 0", err);
        TEST_ASSERT(i_frame->ns == 0, "N(S) should be 0", err);
        TEST_ASSERT(i_frame->nr == 0, "N(R) should be 0", err);
        TEST_ASSERT(i_frame->pf == false, "P/F should be 0", err);
        ax25_frame_free(decoded_frame, &err);
    }
    DEBUG_PRINT("Modulo-128 I-frame no-payload test complete");

    return 0;
}

int test_invalid_address_field() {
    printf("test_invalid_address_field\n");

    uint8_t err = 0;

    // Create a frame with destination (E=0) and source (E=0, invalid termination)
    unsigned char frame[15] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x60,  // Destination: AAAAAA-0, E=0
            0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x60,  // Source: BBBBBB-0, E=0 (invalid - no address field terminator)
            0x03  // Control byte for UI
            };
    size_t frame_len = 15;
    DEBUG_FRAME("Invalid address frame (src ext=0)", frame, frame_len);
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_AUTO, &err);
    DEBUG_VAR("Error code (expected 5)", err);
    TEST_ASSERT(decoded_frame == NULL && err == 5, "Decoding frame with invalid address field", err);
    if (decoded_frame) {
        ax25_frame_free(decoded_frame, &err);
    }
    DEBUG_PRINT("Invalid address field test complete");

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
    DEBUG_FRAME("Valid address frame (Dest E=0 C=1, Src E=1 C=0)", frame, frame_len);
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_AUTO, &err);
    DEBUG_VAR("Error code (expected 0)", err);
    TEST_ASSERT(decoded_frame != NULL && err == 0, "Decoding frame with valid address field", err);
    if (decoded_frame) {
        DEBUG_VAR("Frame type", decoded_frame->type);
        ax25_frame_free(decoded_frame, &err);
    }
    DEBUG_PRINT("Valid address field test complete");

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
    DEBUG_FRAME("Frame with invalid control 0xFF", frame, frame_len);
    ax25_frame_t *decoded_frame = ax25_frame_decode(frame, frame_len, MODULO128_AUTO, &err);
    DEBUG_VAR("Error code (expected 6)", err);
    TEST_ASSERT(decoded_frame == NULL && err == 6, "Decoding U frame with invalid control field", err);
    DEBUG_PRINT("Invalid control field test complete");

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
    DEBUG_PRINT("Test 1: SABME + UA -> should indicate modulo-128");
    ax25_unnumbered_frame_t *ua_response = malloc(sizeof(ax25_unnumbered_frame_t));
    ua_response->base.type = AX25_FRAME_UNNUMBERED_UA;
    ua_response->base.header = sabme_frame->base.header;  // Copy header
    ua_response->base.header.destination.ch = false;
    ua_response->base.header.source.ch = true;
    ua_response->base.header.cr = false;
    ua_response->base.header.src_cr = true;
    ua_response->pf = false;
    ua_response->modifier = 0x63;
    bool mod128_ua = is_modulo128_used((ax25_frame_t*) sabme_frame, (ax25_frame_t*) ua_response);
    DEBUG_BOOL("is_modulo128_used with UA (expected true)", mod128_ua);
    TEST_ASSERT(mod128_ua == true, "UA response should indicate modulo-128", err);

    // Test 2: DM Response (fallback to modulo-8)
    DEBUG_PRINT("Test 2: SABME + DM -> should indicate modulo-8 (fallback)");
    ax25_unnumbered_frame_t *dm_response = malloc(sizeof(ax25_unnumbered_frame_t));
    dm_response->base.type = AX25_FRAME_UNNUMBERED_DM;
    dm_response->base.header = sabme_frame->base.header;
    dm_response->base.header.destination.ch = false;
    dm_response->base.header.source.ch = true;
    dm_response->base.header.cr = false;
    dm_response->base.header.src_cr = true;
    dm_response->pf = false;
    dm_response->modifier = 0x0F;
    bool mod128_dm = is_modulo128_used((ax25_frame_t*) sabme_frame, (ax25_frame_t*) dm_response);
    DEBUG_BOOL("is_modulo128_used with DM (expected false)", mod128_dm);
    TEST_ASSERT(mod128_dm == false, "DM response should indicate modulo-8", err);

    // Test 3: FRMR Response (fallback to modulo-8)
    DEBUG_PRINT("Test 3: SABME + FRMR -> should indicate modulo-8 (fallback)");
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
    bool mod128_frmr = is_modulo128_used((ax25_frame_t*) sabme_frame, (ax25_frame_t*) frmr_response);
    DEBUG_BOOL("is_modulo128_used with FRMR (expected false)", mod128_frmr);
    TEST_ASSERT(mod128_frmr == false, "FRMR response should indicate modulo-8", err);

    // Cleanup
    ax25_frame_free((ax25_frame_t*) sabme_frame, &err);
    ax25_frame_free((ax25_frame_t*) ua_response, &err);
    ax25_frame_free((ax25_frame_t*) dm_response, &err);
    ax25_frame_free((ax25_frame_t*) frmr_response, &err);

    DEBUG_PRINT("SABME/UA negotiation tests complete");

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
    if (encoded_127) {
        DEBUG_FRAME("Encoded frame N(S)=127", encoded_127, len_127);
    }
    if (encoded_0) {
        DEBUG_FRAME("Encoded frame N(S)=0 (wrap)", encoded_0, len_0);
    }

    // Decode and verify
    ax25_frame_t *decoded_127 = ax25_frame_decode(encoded_127, len_127, MODULO128_TRUE, &err);
    ax25_frame_t *decoded_0 = ax25_frame_decode(encoded_0, len_0, MODULO128_TRUE, &err);
    TEST_ASSERT(decoded_127 != NULL && decoded_0 != NULL, "Decoding frames should succeed", err);

    ax25_information_frame_t *i_frame_127 = (ax25_information_frame_t*) decoded_127;
    ax25_information_frame_t *i_frame_0 = (ax25_information_frame_t*) decoded_0;
    DEBUG_VAR("Decoded ns from frame_127 (expected 127)", i_frame_127 ? i_frame_127->ns : 0xFF);
    DEBUG_VAR("Decoded ns from frame_0 (expected 0)", i_frame_0 ? i_frame_0->ns : 0xFF);
    TEST_ASSERT(i_frame_127->ns == 127 && i_frame_0->ns == 0, "Sequence numbers should wrap from 127 to 0", err);

    // Cleanup
    free(encoded_127);
    free(encoded_0);
    ax25_frame_free((ax25_frame_t*) frame_127, &err);
    ax25_frame_free((ax25_frame_t*) frame_0, &err);
    ax25_frame_free(decoded_127, &err);
    ax25_frame_free(decoded_0, &err);

    DEBUG_PRINT("Sequence number wrap-around test complete");

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
    DEBUG_VAR64("Payload size (bytes)", payload_size);
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
    DEBUG_BOOL("UI frame Poll/Final (expected false)", ui_frame->base.pf);
    DEBUG_VAR("UI frame PID (expected 0xF0)", ui_frame->pid);
    DEBUG_VAR("UI frame payload_len", ui_frame->payload_len);

    // Encode the frame
    size_t encoded_len;
    uint8_t *encoded = ax25_frame_encode((ax25_frame_t*) ui_frame, &encoded_len, &err);
    TEST_ASSERT(encoded != NULL, "Encoding UI frame with large payload should succeed", err);
    DEBUG_VAR64("Encoded frame total length", encoded_len);
    if (encoded) {
        DEBUG_FRAME("Encoded frame header (first 16 bytes)", encoded, encoded_len > 16 ? 16 : encoded_len);
    }

    // Decode the frame
    ax25_frame_t *decoded_frame = ax25_frame_decode(encoded, encoded_len, MODULO128_AUTO, &err);
    TEST_ASSERT(decoded_frame != NULL, "Decoding UI frame with large payload should succeed", err);

    ax25_unnumbered_information_frame_t *decoded_ui = (ax25_unnumbered_information_frame_t*) decoded_frame;
    DEBUG_VAR("Decoded payload_len (expected 512)", decoded_ui ? decoded_ui->payload_len : 0xFFFF);
    TEST_ASSERT(decoded_ui->payload_len == payload_size, "Decoded payload size should match original", err);
    TEST_ASSERT(memcmp(decoded_ui->payload, payload, payload_size) == 0, "Decoded payload data should match original", err);

    // Cleanup
    free(payload);
    free(encoded);
    ax25_frame_free((ax25_frame_t*) ui_frame, &err);
    ax25_frame_free(decoded_frame, &err);

    DEBUG_PRINT("Large payloads test complete");

    return 0;
}

int test_srej_functionality() {
    printf("test_srej_functionality\n");

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

    DEBUG_PRINT("I-frames created: N(S)=0 ('A'), N(S)=1 ('B'), N(S)=2 ('C')");
    DEBUG_PRINT("Simulating packet loss of N(S)=1 -> generating SREJ for N(R)=1");

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
    DEBUG_VAR("SREJ N(R) (requesting retransmit of N(S)=1)", srej_frame->nr);
    DEBUG_VAR("SREJ frame type", srej_frame->base.type);

    // Encode SREJ
    size_t srej_len;
    uint8_t *srej_encoded = ax25_supervisory_frame_encode(srej_frame, &srej_len, &err);
    TEST_ASSERT(srej_encoded != NULL, "Encoding SREJ frame should succeed", err);
    if (srej_encoded) {
        DEBUG_FRAME("Encoded SREJ frame", srej_encoded, srej_len);
    }

    // Decode SREJ
    ax25_frame_t *decoded_srej = ax25_frame_decode(srej_encoded, srej_len, MODULO128_FALSE, &err);
    TEST_ASSERT(decoded_srej != NULL, "Decoding SREJ frame should succeed", err);
    DEBUG_VAR("Decoded SREJ frame type (expected SREJ_8BIT)", decoded_srej ? decoded_srej->type : 0xFF);
    TEST_ASSERT(decoded_srej->type == AX25_FRAME_SUPERVISORY_SREJ_8BIT, "Decoded frame should be SREJ", err);
    ax25_supervisory_frame_t *decoded_srej_frame = (ax25_supervisory_frame_t*) decoded_srej;
    DEBUG_VAR("Decoded SREJ N(R) (expected 1)", decoded_srej_frame->nr);
    TEST_ASSERT(decoded_srej_frame->nr == 1, "SREJ should request ns=1", err);

    // Simulate retransmission of frame_1
    DEBUG_PRINT("Simulating retransmission of N(S)=1 after SREJ");
    size_t retransmitted_len;
    uint8_t *retransmitted = ax25_frame_encode((ax25_frame_t*) frame_1, &retransmitted_len, &err);
    TEST_ASSERT(retransmitted != NULL, "Encoding retransmitted frame should succeed", err);
    if (retransmitted) {
        DEBUG_FRAME("Encoded retransmitted I-frame (N(S)=1)", retransmitted, retransmitted_len);
    }

    ax25_frame_t *decoded_retransmitted = ax25_frame_decode(retransmitted, retransmitted_len, MODULO128_FALSE, &err);
    TEST_ASSERT(decoded_retransmitted != NULL, "Decoding retransmitted frame should succeed", err);
    DEBUG_VAR("Decoded retransmitted frame type (expected I-frame 8-bit)", decoded_retransmitted ? decoded_retransmitted->type : 0xFF);
    TEST_ASSERT(decoded_retransmitted->type == AX25_FRAME_INFORMATION_8BIT, "Retransmitted frame should be I-frame", err);
    ax25_information_frame_t *retransmitted_frame = (ax25_information_frame_t*) decoded_retransmitted;
    DEBUG_VAR("Retransmitted N(S) (expected 1)", retransmitted_frame->ns);
    DEBUG_FRAME("Retransmitted payload (expected 'B')", retransmitted_frame->payload, retransmitted_frame->payload_len);
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

    DEBUG_PRINT("SREJ functionality test complete");

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

    printf("\n----------------------------------------------------------------------------------\n\n");
    test_ax25_frame_print();
    printf("\n----------------------------------------------------------------------------------\n");
    printf("Tests AX.25 Completed. %s\n", result == 0 ? "All tests passed" : "Some tests failed");
    printf("----------------------------------------------------------------------------------\n\n");

    return result;
}
