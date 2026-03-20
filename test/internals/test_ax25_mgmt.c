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
#include <stdlib.h>

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

// Tracking variables for MDL-ERROR indication delivery verification.
static bool mdl_error_fired;
static ax25_mdl_error_t mdl_error_code_received;

// MDL-ERROR indication callback: records code for test assertions.
static void test_on_mdl_error(ax25_mdl_error_t error, void *user_data) {
    mdl_error_fired = true;
    mdl_error_code_received = error;
    (void) user_data;
}

static int test_default_parameters(void) {
    assert_count = 0;
    printf("\n--- test_default_parameters ---\n");

    ax25_mgmt_context_t ctx;
    ax25_mgmt_init(&ctx);

    TEST_ASSERT(ctx.state == AX25_MGMT_IDLE, "Initial state is IDLE", 0);

    TEST_ASSERT(ctx.local_params.full_duplex == false, "Default full_duplex = false", 0);
    TEST_ASSERT(ctx.local_params.selective_reject == true, "Default selective_reject = true", 0);
    TEST_ASSERT(ctx.local_params.implicit_reject == true, "Default implicit_reject = true", 0);
    TEST_ASSERT(ctx.local_params.modulo128 == true, "Default modulo128 = true", 0);
    TEST_ASSERT(ctx.local_params.ifield_length == 256, "Default ifield_length = 256", 0);
    TEST_ASSERT(ctx.local_params.window_size == 32, "Default window_size = 32 (modulo-128 default per AX.25 v2.2 Table C1)", 0);
    // start modified part: ack_timer default is 10000 ms per Linux AX25_DEF_T1
    TEST_ASSERT(ctx.local_params.ack_timer == 10000, "Default ack_timer = 10000", 0);
    // end modified part
    TEST_ASSERT(ctx.local_params.retries == 10, "Default retries = 10", 0);

    return 0;
}

static int test_start_negotiation(void) {
    assert_count = 0;
    printf("\n--- test_start_negotiation ---\n");

    ax25_mgmt_context_t ctx;
    ax25_mgmt_init(&ctx);

    uint8_t err = 0;
    // start modified part
    // Allocate both before any TEST_ASSERT so both are freed on any early-return path.
    // ax25_mgmt_start_negotiation copies addresses by value; free immediately after.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || err != 0 || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "Parsed dest and src addresses", err);
    }
    // end modified part

    // Modify some parameters to ensure XID frame is non-empty
    ctx.local_params.ifield_length = 512;
    ctx.local_params.window_size = 15;
    ctx.local_params.ack_timer = 5000;

    captured_len = 0;
    uint8_t res = ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately — addresses copied into ctx by value above.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part
    TEST_ASSERT(res == 0, "ax25_mgmt_start_negotiation returns success", res);
    TEST_ASSERT(ctx.state == AX25_MGMT_AWAITING_RESPONSE, "State changed to AWAITING_RESPONSE", 0);
    TEST_ASSERT(captured_len > 20, "XID command frame transmitted (reasonable length)", 0);
    TEST_ASSERT(ctx.retry_count == 0, "Initial retry_count = 0", 0);

    return 0;
}

static int test_negotiation_timeout(void) {
    assert_count = 0;
    printf("\n--- test_negotiation_timeout ---\n");

    ax25_mgmt_context_t ctx;
    ax25_mgmt_init(&ctx);

    // Register MDL-ERROR callback so Layer 3 delivery is verified by assertions.
    mdl_error_fired = false;
    mdl_error_code_received = AX25_MDL_ERROR_B;
    ctx.on_mdl_error = test_on_mdl_error;

    uint8_t err = 0;
    // start modified part
    // Allocate both addresses before any TEST_ASSERT so a single cleanup path covers both.
    // ax25_mgmt_start_negotiation copies addresses by value (ctx->peer = *dest),
    // so dest and src can be freed immediately after the call without affecting ctx.
    ax25_address_t *dest = ax25_address_from_string("DEST-0", &err);
    ax25_address_t *src = ax25_address_from_string("SRC-0", &err);
    if (!dest || err != 0 || !src) {
        free(dest);
        free(src);
        TEST_ASSERT(false, "Parsed dest and src addresses", err);
    }
    // end modified part

    captured_len = 0;
    uint8_t res = ax25_mgmt_start_negotiation(&ctx, dest, src, capture_transmit);
    // start modified part
    // Free immediately after start_negotiation — addresses are copied into ctx by value.
    // All subsequent TEST_ASSERT paths are now leak-free.
    free(dest);
    free(src);
    dest = NULL;
    src = NULL;
    // end modified part
    TEST_ASSERT(res == 0, "ax25_mgmt_start_negotiation succeeded", res);
    TEST_ASSERT(ctx.state == AX25_MGMT_AWAITING_RESPONSE, "State is AWAITING_RESPONSE", 0);

    uint32_t current_tick = 0;
    ctx.timeout_tick = 0;

    // start modified part: tick increments raised from 4000 to 11000 ms
    // tm201 = max(ack_timer=10000, 3000) = 10000; each step needs elapsed >= 10000
    current_tick += 11000;
    ax25_mgmt_tick(&ctx, current_tick);
    TEST_ASSERT(ctx.retry_count == 1, "Retry count incremented", 0);

    current_tick += 11000;
    ax25_mgmt_tick(&ctx, current_tick);
    TEST_ASSERT(ctx.retry_count == 2, "Retry count incremented", 0);

    current_tick += 11000;
    ax25_mgmt_tick(&ctx, current_tick);
    TEST_ASSERT(ctx.retry_count == 3, "Retry count incremented", 0);

    // MDL-ERROR K must be delivered to Layer 3 when NM201 retries are exhausted.
    TEST_ASSERT(mdl_error_fired == true, "MDL-ERROR indication delivered to Layer 3", 0);
    TEST_ASSERT(mdl_error_code_received == AX25_MDL_ERROR_K, "MDL-ERROR code is K (retransmission limit)", 0);

    current_tick += 11000;
    ax25_mgmt_tick(&ctx, current_tick);
    TEST_ASSERT(ctx.state == AX25_MGMT_IDLE, "State returned to IDLE after max retries", 0);
    // end modified part

    return 0;
}

int test_ax25_mgmt_main(void) {
    int result = 0;

    printf("\n----------------------------------------------------------------------------------\n");
    printf("Starting AX.25 Management (XID Negotiation) Tests\n");
    printf("----------------------------------------------------------------------------------\n\n");

    result |= test_default_parameters();
    result |= test_start_negotiation();
    result |= test_negotiation_timeout();

    printf("\n----------------------------------------------------------------------------------\n");
    printf("AX.25 Management Tests Completed. %s\n", result == 0 ? "All tests passed" : "Some tests failed");
    printf("----------------------------------------------------------------------------------\n\n");

    return result;
}
