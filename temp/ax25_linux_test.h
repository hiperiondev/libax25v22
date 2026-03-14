/**
 * @file ax25_linux_test.h
 * @brief Test-harness declarations for libax25v22 Linux bridge tests.
 *
 * FIXES vs previous version
 * ──────────────────────────
 * FIX-GAP-BOOL    bool / true / false replaced with uint8_t / 1U / 0U so the
 *                 header compiles under strict C89 / embedded toolchains.
 * FIX-GAP-LB_SIZE lb_ab_len / lb_ba_len changed from size_t to uint16_t;
 *                 LB_FRAME_MAX capped to uint16_t range.
 * FIX-MCU-STATICBUF  MCU_TEST_SMALL guard: reduces LB_MAX to 4 and
 *                 LB_FRAME_MAX to 128 when RAM is tight (Cortex-M0/M3).
 * FIX-MCU-SIZECAST   All encode-size values are now held in uint16_t; a
 *                 bounds-check macro is provided.
 * FIX-TEST-COMMON    All assertion macros replaced with test_common.h macros;
 *                 test functions return int (0 = pass, 1 = fail).
 */

#ifndef AX25_LINUX_TEST_H
#define AX25_LINUX_TEST_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "ax25.h"
#include "ax25_state_machine.h"
#include "ax25_mux.h"
#include "kiss.h"
#include "hal.h"
#include "test_common.h"

/* =========================================================================
 * FIX-MCU-STATICBUF: compile-time RAM budget selector
 * ========================================================================= */
#if defined(MCU_TEST_SMALL)
#   define LB_MAX       4U
#   define LB_FRAME_MAX 128U
#else
#   define LB_MAX       16U
#   define LB_FRAME_MAX 340U
#endif

/* =========================================================================
 * FIX-MCU-SIZECAST: safe size-to-uint16_t conversion with bounds check
 * SAFE_U16 now uses test_common.h TEST_ASSERT (3-arg form, err=0).
 * ========================================================================= */
#define SAFE_U16(sz)  \
    ( (TEST_ASSERT((sz) <= (size_t)UINT16_MAX, "encode size overflows uint16_t", 0)), \
      (uint16_t)(sz) )

/* =========================================================================
 * KISS TX CAPTURE
 * ========================================================================= */
#define KISS_CAP_MAX 512U

extern uint8_t g_kiss_rx_data[KISS_CAP_MAX];
extern uint16_t g_kiss_rx_len;

static inline void kiss_cap_flush(void) {
    memset(g_kiss_rx_data, 0, sizeof(g_kiss_rx_data));
    g_kiss_rx_len = 0U;
}

/* =========================================================================
 * LOOPBACK HARNESS
 * ========================================================================= */
extern uint8_t lb_ab_data[LB_MAX][LB_FRAME_MAX];
extern uint16_t lb_ab_len[LB_MAX];
extern uint8_t lb_ab_head;
extern uint8_t lb_ab_tail;

extern uint8_t lb_ba_data[LB_MAX][LB_FRAME_MAX];
extern uint16_t lb_ba_len[LB_MAX];
extern uint8_t lb_ba_head;
extern uint8_t lb_ba_tail;

static inline void lb_flush(void) {
    memset(lb_ab_data, 0, sizeof(lb_ab_data));
    memset(lb_ba_data, 0, sizeof(lb_ba_data));
    memset(lb_ab_len, 0, sizeof(lb_ab_len));
    memset(lb_ba_len, 0, sizeof(lb_ba_len));
    lb_ab_head = lb_ab_tail = 0U;
    lb_ba_head = lb_ba_tail = 0U;
}

static inline uint8_t lb_count(uint8_t head, uint8_t tail) {
    return (uint8_t) (((uint8_t) (LB_MAX) + tail - head) % (uint8_t) (LB_MAX));
}

void lb_deliver_and_drain_ab(ax25_connection_t *conn_b, ax25_mux_t *mux_b, uint32_t tick_10ms);
void lb_deliver_and_drain_ba(ax25_connection_t *conn_a, ax25_mux_t *mux_a, uint32_t tick_10ms);

/* =========================================================================
 * TEST HELPER PROTOTYPES
 * ========================================================================= */
uint16_t build_ax25_frame(const char *dest, const char *src, uint8_t ctrl, const uint8_t *payload, uint16_t plen, uint8_t pid, uint8_t *buf, uint16_t buf_max);

uint16_t build_kiss_frame(const uint8_t *ax25_frame, uint16_t ax25_len, uint8_t port, uint8_t *out, uint16_t out_max);

/* =========================================================================
 * TEST ENTRY POINTS  — all return int: 0 = pass, 1 = fail
 * ========================================================================= */

/* Group A: codec / encoding */
int test_t01_address_encode_decode(void);
int test_t02_frame_encode_ui(void);
int test_t03_frame_decode_ui(void);
int test_t04_kiss_encode(void);
int test_t05_kiss_decode(void);
int test_t06_kiss_fesc_in_payload(void);
int test_t07_kiss_fend_in_payload(void);

/* Group B: address parsing */
int test_t08_address_boundary_ssids(void);
int test_t09_address_invalid_string(void);
int test_t10_address_from_null(void);

/* Group C: CRC / HAL */
int test_t11_crc16_known_vector(void);
int test_t12_crc16_incremental(void);
int test_t13_crc16_zero_length(void);

/* Group D: SM loopback — basic */
int test_t14_sm_ui_loopback(void);
int test_t15_sm_connect_disconnect(void);
int test_t16_sm_i_frame_data(void);
int test_t17_sm_rnr_flow_control(void);
int test_t18_sm_rnr_poll_supervisory(void);
int test_t19_sm_multiple_i_frames(void);
int test_t20_sm_reject_retransmit(void);

/* Group E: frame types */
int test_t21_frmr_codec(void);
int test_t22_digipeat(void);
int test_t23_sabme_mod128(void);

/* Group F: mux */
int test_t24_mux_init_deinit(void);
int test_t25_mux_routing_two_conns(void);
int test_t26_xid_encode_decode(void);
int test_t27_xid_values_verified(void);

/* Group G: SM rejection / retransmit / protocol */
int test_t28_rej_retransmit_frame_verify(void);
int test_t29_frmr_on_protocol_violation(void);
int test_t30_dm_on_data_disconnected(void);
int test_t31_window_exhaustion(void);

/* Group H: bridge inject path */
int test_t32_bridge_init_deinit(void);
int test_t33_bridge_inject_ui(void);
int test_t34_bridge_connect_ua(void);
int test_t35_bridge_fesc_inject(void);
int test_t36_bridge_port_range_check(void);
int test_t37_bridge_null_userdata(void);
int test_t38_bridge_connect_ua_bits(void);

/* Group I: interoperability */
int test_t39_sabme_through_bridge(void);
int test_t40_xid_through_bridge(void);
int test_t41_multiport_routing(void);
int test_t42_mod128_seq_wrap(void);

/* Group J: edge / stress */
int test_t43_tick_wrap_uint32(void);
int test_t44_address_all_ssids(void);
int test_t45_sm_loopback_mod8_wrap(void);
int test_t46_sm_loopback_mod128_wrap(void);

/* =========================================================================
 * TEST RUNNER
 * ========================================================================= */
int linux_test_main(void);

#endif /* AX25_LINUX_TEST_H */
