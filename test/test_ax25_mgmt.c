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

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "test_common.h"
#include "ax25_mgmt.h"
#include "ax25.h"

static uint32_t assert_count = 0;

static uint8_t captured_buffer[2048];
static size_t captured_len = 0;

static void capture_transmit(uint8_t *data, size_t len) {
    if (len <= sizeof(captured_buffer)) {
        memcpy(captured_buffer, data, len);
        captured_len = len;
    } else {
        captured_len = 0;
    }
}

static int test_parameter_negotiation(void) {
    assert_count = 0;
    printf("\n--- test_parameter_negotiation ---\n");

    ax25_mgmt_context_t initiator;
    ax25_mgmt_context_t responder;

    uint8_t err = 0;

    ax25_mgmt_init(&initiator);
    ax25_mgmt_init(&responder);

    // Set differing local parameters to force intersection logic
    initiator.local_params.full_duplex = false;
    initiator.local_params.selective_reject = true;
    initiator.local_params.implicit_reject = true;
    initiator.local_params.modulo128 = true;
    initiator.local_params.ifield_length = 2048;
    initiator.local_params.window_size = 32;
    initiator.local_params.ack_timer = 5000;
    initiator.local_params.retries = 20;

    responder.local_params.full_duplex = true;
    responder.local_params.selective_reject = false;
    responder.local_params.implicit_reject = true;
    responder.local_params.modulo128 = false;
    responder.local_params.ifield_length = 256;
    responder.local_params.window_size = 4;
    responder.local_params.ack_timer = 2000;
    responder.local_params.retries = 10;

    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    TEST_ASSERT(dest != NULL && err == 0, "Parsed destination address", err);

    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    TEST_ASSERT(src != NULL && err == 0, "Parsed source address", err);

    captured_len = 0;
    uint8_t res = ax25_mgmt_start_negotiation(&initiator, dest, src, capture_transmit);
    TEST_ASSERT(res == 0, "ax25_mgmt_start_negotiation succeeded", res);
    TEST_ASSERT(captured_len > 14 + 1, "Captured XID command frame has reasonable length", 0);

    // Manual parsing for command frame (no repeaters → 14 bytes addresses + 1 byte control)
    ax25_frame_header_t cmd_header = { 0 };
    cmd_header.destination = *dest;
    cmd_header.source = *src;
    cmd_header.repeaters.num_repeaters = 0;

    bool cmd_pf = (captured_buffer[14] & 0x10) != 0;
    const uint8_t *cmd_payload = captured_buffer + 15;
    size_t cmd_payload_len = captured_len - 15;

    ax25_exchange_identification_frame_t *cmd_xid = ax25_exchange_identification_frame_decode(&cmd_header, cmd_pf, cmd_payload, cmd_payload_len, &err);
    TEST_ASSERT(cmd_xid != NULL && err == 0, "Parsed XID command structure", err);

    captured_len = 0;
    res = ax25_mgmt_process_xid(&responder, cmd_xid, capture_transmit);
    TEST_ASSERT(res == 0, "Responder processed XID command", res);
    TEST_ASSERT(captured_len > 14 + 1, "Captured XID response frame has reasonable length", 0);

    // Manual parsing for response frame (addresses reversed)
    ax25_frame_header_t resp_header = { 0 };
    resp_header.destination = *src;   // reversed
    resp_header.source = *dest;       // reversed
    resp_header.repeaters.num_repeaters = 0;

    bool resp_pf = (captured_buffer[14] & 0x10) != 0;
    const uint8_t *resp_payload = captured_buffer + 15;
    size_t resp_payload_len = captured_len - 15;

    ax25_exchange_identification_frame_t *resp_xid = ax25_exchange_identification_frame_decode(&resp_header, resp_pf, resp_payload, resp_payload_len, &err);
    TEST_ASSERT(resp_xid != NULL && err == 0, "Parsed XID response structure", err);

    res = ax25_mgmt_process_xid(&initiator, resp_xid, NULL);
    TEST_ASSERT(res == 0, "Initiator processed XID response", res);

    TEST_ASSERT(initiator.state == AX25_MGMT_NEGOTIATED, "Initiator reached NEGOTIATED state", 0);
    TEST_ASSERT(responder.state == AX25_MGMT_NEGOTIATED, "Responder reached NEGOTIATED state", 0);

    TEST_ASSERT(memcmp(&initiator.agreed_params, &responder.agreed_params, sizeof(ax25_negotiated_params_t)) == 0,
            "Both sides have identical agreed parameters", 0);

    ax25_negotiated_params_t agreed = initiator.agreed_params;

    TEST_ASSERT(agreed.full_duplex == false, "Full duplex = false (logical AND)", 0);
    TEST_ASSERT(agreed.selective_reject == false, "Selective reject = false (logical AND)", 0);
    TEST_ASSERT(agreed.implicit_reject == true, "Implicit reject = true (logical AND)", 0);
    TEST_ASSERT(agreed.modulo128 == false, "Modulo-128 = false (logical AND)", 0);
    TEST_ASSERT(agreed.ifield_length == 256, "I-field length = MIN(2048,256)", 0);
    TEST_ASSERT(agreed.window_size == 4, "Window size = MIN(32,4)", 0);
    TEST_ASSERT(agreed.ack_timer == 2000, "ACK timer = MIN(5000,2000)", 0);
    TEST_ASSERT(agreed.retries == 10, "Retries = MIN(20,10)", 0);

    return 0;
}

static int test_negotiation_timeout(void) {
    assert_count = 0;
    printf("\n--- test_negotiation_timeout ---\n");

    ax25_mgmt_context_t ctx;
    ax25_mgmt_init(&ctx);

    uint8_t err = 0;
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    TEST_ASSERT(dest != NULL && err == 0, "Parsed destination address", err);

    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    TEST_ASSERT(src != NULL && err == 0, "Parsed source address", err);

    captured_len = 0;
    uint8_t res = ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    TEST_ASSERT(res == 0, "ax25_mgmt_start_negotiation succeeded", res);
    TEST_ASSERT(ctx.state == AX25_MGMT_AWAITING_RESPONSE, "Initial state AWAITING_RESPONSE", 0);

    uint32_t current_tick = 0;
    ctx.timeout_tick = 0;

    // Trigger three timeouts → retry_count reaches 3 → IDLE
    current_tick += 4000;
    ax25_mgmt_tick(&ctx, current_tick);
    current_tick += 4000;
    ax25_mgmt_tick(&ctx, current_tick);
    current_tick += 4000;
    ax25_mgmt_tick(&ctx, current_tick);

    TEST_ASSERT(ctx.state == AX25_MGMT_IDLE, "State returned to IDLE after timeout/retries", 0);

    return 0;
}

int test_ax25_mgmt_main(void) {
    int result = 0;

    printf("\n----------------------------------------------------------------------------------\n");
    printf("Starting AX.25 Management (XID Negotiation) Tests\n");
    printf("----------------------------------------------------------------------------------\n\n");

    result |= test_parameter_negotiation();
    result |= test_negotiation_timeout();

    printf("\n----------------------------------------------------------------------------------\n");
    printf("AX.25 Management Tests Completed. %s\n", result == 0 ? "All tests passed" : "Some tests failed");
    printf("----------------------------------------------------------------------------------\n\n");

    return result;
}
