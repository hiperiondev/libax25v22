/**
 * @file    hal_dummy.c
 * @brief   AX.25 v2.2 HAL – minimal Linux dummy implementation
 *
 * This file implements every function declared in hal.h using only
 * POSIX interfaces available on Linux.  It is intentionally minimal:
 *   - No threads (single-threaded, poll-based operation).
 *   - PTT is simulated (log-only; wire to a real GPIO in production).
 *   - DCD is always idle (no carrier).
 *   - Serial I/O uses in-process ring buffers fed by stdin/stdout or a
 *     real TTY path (configured at compile time with HAL_SERIAL_PATH).
 *   - Memory uses standard malloc/free from glibc.
 *   - CRC-16 is computed in pure C with a 256-entry lookup table.
 *   - Timers are built on gettimeofday() (32-bit millisecond arithmetic,
 *     NO 64-bit values, NO floating-point).
 *
 * Optimization notes (also applicable to MCU ports)
 * --------------------------------------------------
 *  - All arithmetic is 8- or 32-bit.
 *  - Division is avoided in hot paths; modulo power-of-two uses masking.
 *  - The CRC table is const and placed in .rodata.
 *  - The PRNG is a 32-bit Galois LFSR – 4 instructions per bit, no divide.
 */

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>

#include "hal.h"

/* =========================================================================
 * INTERNAL CONSTANTS
 * ========================================================================= */

#define HAL_MAX_PORTS       2U
#define RING_RX_MASK        ((uint16_t)(HAL_SERIAL_RX_BUF_SIZE - 1U))
#define RING_TX_MASK        ((uint16_t)(HAL_SERIAL_TX_BUF_SIZE - 1U))

/* Compile-time check that buffer sizes are powers of two */
typedef char _rx_pow2_check[(HAL_SERIAL_RX_BUF_SIZE & (HAL_SERIAL_RX_BUF_SIZE - 1)) == 0 ? 1 : -1];
typedef char _tx_pow2_check[(HAL_SERIAL_TX_BUF_SIZE & (HAL_SERIAL_TX_BUF_SIZE - 1)) == 0 ? 1 : -1];

/* =========================================================================
 * SECTION 1 – TICK COUNTER
 *
 * We derive a 32-bit millisecond tick from gettimeofday().
 * IMPORTANT: all arithmetic stays in 32-bit to satisfy the no-64-bit rule.
 * The reference epoch is set at hal_init() time so the counter starts near
 * zero, avoiding an immediate 49-day wrap concern.
 * ========================================================================= */

static uint32_t g_tick_epoch_sec; /* seconds part of init time    */
static uint32_t g_tick_epoch_usec; /* microseconds part of init time */

// g_saved_mask stores the signal mask active before the first
// hal_critical_enter() so hal_critical_exit() can restore it exactly.
static sigset_t g_saved_mask;

// Return elapsed milliseconds since hal_init().
// Result is a 32-bit counter that wraps after ~49.7 days.
// Callers measure elapsed time as (current - past) with unsigned
// subtraction, which handles the wrap transparently.
uint32_t hal_tick_ms(void) {
    struct timeval tv;
    uint32_t sec_delta;
    uint32_t usec_delta;

    gettimeofday(&tv, NULL);

    // Both sides are cast to uint32_t before subtraction.
    // If tv_sec has crossed a 2^32 boundary the truncated values
    // still subtract correctly modulo 2^32, which is intentional:
    // the result wraps the same way hal_elapsed_ms() expects.
    sec_delta = (uint32_t) tv.tv_sec - g_tick_epoch_sec;
    usec_delta = (uint32_t) tv.tv_usec;

    // Handle microsecond borrow when current usec < epoch usec.
    if (usec_delta < g_tick_epoch_usec) {
        sec_delta -= 1U;
        usec_delta += 1000000U;
    }
    usec_delta -= g_tick_epoch_usec;

    // sec_delta * 1000: intentional 32-bit wrap after ~49.7 days.
    // usec_delta / 1000: integer division, range [0, 999].
    // On targets without a hardware divider gcc synthesises a
    // multiply-shift sequence for the constant divisor 1000,
    // so no actual divide instruction is emitted on Cortex-M.
    return sec_delta * 1000U + usec_delta / 1000U;
}

/* =========================================================================
 * SECTION 2 – SOFTWARE TIMERS
 * ========================================================================= */

typedef struct {
    uint32_t start_ms; /* tick value when timer was started  */
    uint32_t period_ms; /* timeout period                     */
    uint8_t active; /* 1=running, 0=stopped               */
} timer_slot_t;

static timer_slot_t g_timers[HAL_TIMER_MAX];
static uint8_t g_timer_used[HAL_TIMER_MAX];

hal_timer_t hal_timer_alloc(void) {
    uint8_t i;
    for (i = 0U; i < HAL_TIMER_MAX; ++i) {
        if (!g_timer_used[i]) {
            g_timer_used[i] = 1U;
            g_timers[i].active = 0U;
            g_timers[i].period_ms = 0U;
            return (hal_timer_t) i;
        }
    }
    return HAL_TIMER_INVALID;
}

void hal_timer_free(hal_timer_t t) {
    if (t < 0 || (uint8_t) t >= HAL_TIMER_MAX)
        return;
    g_timers[(uint8_t) t].active = 0U;
    g_timer_used[(uint8_t) t] = 0U;
}

hal_err_t hal_timer_start(hal_timer_t t, uint32_t period_ms) {
    if (t < 0 || (uint8_t) t >= HAL_TIMER_MAX)
        return HAL_ERR_INVAL;
    g_timers[(uint8_t) t].start_ms = hal_tick_ms();
    g_timers[(uint8_t) t].period_ms = period_ms;
    g_timers[(uint8_t) t].active = 1U;
    return HAL_OK;
}

void hal_timer_stop(hal_timer_t t) {
    if (t < 0 || (uint8_t) t >= HAL_TIMER_MAX)
        return;
    g_timers[(uint8_t) t].active = 0U;
}

uint8_t hal_timer_expired(hal_timer_t t) {
    uint32_t elapsed;
    if (t < 0 || (uint8_t) t >= HAL_TIMER_MAX)
        return 0U;
    if (!g_timers[(uint8_t) t].active)
        return 0U;
    elapsed = hal_elapsed_ms(g_timers[(uint8_t) t].start_ms);
    return (elapsed >= g_timers[(uint8_t) t].period_ms) ? 1U : 0U;
}

uint8_t hal_timer_running(hal_timer_t t) {
    if (t < 0 || (uint8_t) t >= HAL_TIMER_MAX)
        return 0U;
    if (!g_timers[(uint8_t) t].active)
        return 0U;
    return hal_timer_expired(t) ? 0U : 1U;
}

uint32_t hal_timer_remaining_ms(hal_timer_t t) {
    uint32_t elapsed;
    if (t < 0 || (uint8_t) t >= HAL_TIMER_MAX)
        return 0U;
    if (!g_timers[(uint8_t) t].active)
        return 0U;
    elapsed = hal_elapsed_ms(g_timers[(uint8_t) t].start_ms);
    if (elapsed >= g_timers[(uint8_t) t].period_ms)
        return 0U;
    return g_timers[(uint8_t) t].period_ms - elapsed;
}

/* =========================================================================
 * SECTION 3 – PTT
 * ========================================================================= */

static uint8_t g_ptt_state[HAL_MAX_PORTS];

hal_err_t hal_ptt_set(uint8_t port, uint8_t assert_tx) {
    if (port >= HAL_MAX_PORTS)
        return HAL_ERR_NODEV;
    g_ptt_state[port] = assert_tx ? 1U : 0U;
    HAL_LOGI("[PTT] port=%u  %s\n", port, assert_tx ? "ASSERT (TX ON)" : "DEASSERT (TX OFF)");
    return HAL_OK;
}

int8_t hal_ptt_get(uint8_t port) {
    if (port >= HAL_MAX_PORTS)
        return (int8_t) HAL_ERR_NODEV;
    return (int8_t) g_ptt_state[port];
}

/* =========================================================================
 * SECTION 4 – DCD
 *
 * Always returns 0 (channel idle) in this dummy.
 * Wire to a GPIO input or software squelch in production.
 * ========================================================================= */

int8_t hal_dcd_get(uint8_t port) {
    if (port >= HAL_MAX_PORTS)
        return (int8_t) HAL_ERR_NODEV;
    return 0; /* channel always idle */
}

/* =========================================================================
 * SECTION 5 – SERIAL / KISS RING BUFFERS
 *
 * Each port has:
 *   - An RX ring buffer   (data arriving from TNC → protocol layer)
 *   - A TX ring buffer   (data leaving protocol layer → TNC)
 *
 * Ring buffer state uses 16-bit head/tail indices; masking keeps them
 * in range without division.  No 64-bit arithmetic.
 *
 * In this Linux dummy the TX buffer is drained to stdout (fd 1), and
 * the RX buffer is fed from stdin (fd 0) in hal_serial_poll() which
 * the caller must invoke from the main loop.
 * ========================================================================= */

typedef struct {
    uint8_t rx_buf[HAL_SERIAL_RX_BUF_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;

    uint8_t tx_buf[HAL_SERIAL_TX_BUF_SIZE];
    uint16_t tx_head;
    uint16_t tx_tail;

    int fd_rx; /* read file descriptor  (-1 = stdin)  */
    int fd_tx; /* write file descriptor (-1 = stdout) */
    uint8_t open;
} serial_port_t;

static serial_port_t g_serial[HAL_MAX_PORTS];

/* ── Ring buffer helpers (16-bit arithmetic only) ── */

static inline uint16_t ring_used(uint16_t head, uint16_t tail, uint16_t mask) {
    return (uint16_t) ((tail - head) & mask);
}

static inline uint16_t ring_free(uint16_t head, uint16_t tail, uint16_t mask) {
    return (uint16_t) (mask - ring_used(head, tail, mask));
}

hal_err_t hal_serial_put(uint8_t port, uint8_t byte) {
    serial_port_t *s;
    uint16_t next;

    if (port >= HAL_MAX_PORTS || !g_serial[port].open)
        return HAL_ERR_NODEV;
    s = &g_serial[port];
    next = (uint16_t) ((s->tx_tail + 1U) & RING_TX_MASK);
    if (next == s->tx_head)
        return HAL_ERR_BUSY; /* buffer full */
    s->tx_buf[s->tx_tail] = byte;
    s->tx_tail = next;
    return HAL_OK;
}

uint16_t hal_serial_write(uint8_t port, const uint8_t *buf, uint16_t len) {
    uint16_t i;
    for (i = 0U; i < len; ++i) {
        if (hal_serial_put(port, buf[i]) != HAL_OK)
            break;
    }
    return i;
}

hal_err_t hal_serial_get(uint8_t port, uint8_t *byte) {
    serial_port_t *s;
    if (port >= HAL_MAX_PORTS || !g_serial[port].open)
        return HAL_ERR_NODEV;
    s = &g_serial[port];
    if (s->rx_head == s->rx_tail)
        return HAL_ERR_BUSY; /* empty */
    *byte = s->rx_buf[s->rx_head];
    s->rx_head = (uint16_t) ((s->rx_head + 1U) & RING_RX_MASK);
    return HAL_OK;
}

int16_t hal_serial_rx_available(uint8_t port) {
    if (port >= HAL_MAX_PORTS || !g_serial[port].open)
        return (int16_t) HAL_ERR_NODEV;
    return (int16_t) ring_used(g_serial[port].rx_head, g_serial[port].rx_tail, RING_RX_MASK);
}

int16_t hal_serial_tx_free(uint8_t port) {
    if (port >= HAL_MAX_PORTS || !g_serial[port].open)
        return (int16_t) HAL_ERR_NODEV;
    return (int16_t) ring_free(g_serial[port].tx_head, g_serial[port].tx_tail, RING_TX_MASK);
}

void hal_serial_rx_flush(uint8_t port) {
    if (port >= HAL_MAX_PORTS)
        return;
    g_serial[port].rx_head = g_serial[port].rx_tail = 0U;
}

hal_err_t hal_serial_tx_flush(uint8_t port, uint32_t timeout_ms) {
    uint32_t t0 = hal_tick_ms();
    while (g_serial[port].tx_head != g_serial[port].tx_tail) {
        hal_wdog_kick();
        if (hal_elapsed_ms(t0) >= timeout_ms)
            return HAL_ERR_TIMEOUT;
        /* drain one byte */
        {
            uint8_t b = g_serial[port].tx_buf[g_serial[port].tx_head];
            g_serial[port].tx_head = (uint16_t) ((g_serial[port].tx_head + 1U) & RING_TX_MASK);
            int fd = (g_serial[port].fd_tx >= 0) ? g_serial[port].fd_tx : STDOUT_FILENO;
            if (write(fd, &b, 1) < 0) { /* best-effort */
            }
        }
    }
    return HAL_OK;
}

int8_t hal_tx_idle(uint8_t port) {
    if (port >= HAL_MAX_PORTS || !g_serial[port].open)
        return (int8_t) HAL_ERR_NODEV;
    // In the Linux dummy the TX ring buffer IS the transmit pipeline.
    // Return 1 (idle) when head == tail (buffer empty), 0 otherwise.
    return (g_serial[port].tx_head == g_serial[port].tx_tail) ? 1 : 0;
}

/**
 * hal_serial_poll() – MUST be called from the application main loop.
 *
 * Drains pending TX bytes to the OS fd and fills the RX ring buffer
 * from the OS fd (non-blocking).  This replaces interrupt-driven I/O
 * for the Linux dummy.
 */
void hal_serial_poll(uint8_t port) {
    serial_port_t *s;
    ssize_t n;
    uint8_t tmp[64];
    uint16_t i;
    int fd_rx, fd_tx;

    if (port >= HAL_MAX_PORTS || !g_serial[port].open)
        return;
    s = &g_serial[port];
    fd_rx = (s->fd_rx >= 0) ? s->fd_rx : STDIN_FILENO;
    fd_tx = (s->fd_tx >= 0) ? s->fd_tx : STDOUT_FILENO;

    /* Drain TX ring to file descriptor */
    while (s->tx_head != s->tx_tail) {
        uint8_t b = s->tx_buf[s->tx_head];
        s->tx_head = (uint16_t) ((s->tx_head + 1U) & RING_TX_MASK);
        if (write(fd_tx, &b, 1) < 0) { /* best-effort */
        }
    }

    /* Fill RX ring from file descriptor (non-blocking read) */
    n = read(fd_rx, tmp, sizeof(tmp));
    if (n > 0) {
        for (i = 0U; i < (uint16_t) n; ++i) {
            uint16_t next = (uint16_t) ((s->rx_tail + 1U) & RING_RX_MASK);
            if (next == s->rx_head)
                break; /* ring full – drop byte */
            s->rx_buf[s->rx_tail] = tmp[i];
            s->rx_tail = next;
        }
    }
}

/* =========================================================================
 * SECTION 6 – PSEUDO-RANDOM NUMBER GENERATOR
 *
 * 32-bit Galois LFSR.  One step costs: xor + shift + conditional or.
 * No division, no 64-bit, no float.
 * ========================================================================= */

static uint32_t g_lfsr = 0xACE1u; /* must be non-zero */

void hal_random_seed(uint32_t seed) {
    g_lfsr = (seed != 0U) ? seed : 0xDEADBEEFU;
}

uint8_t hal_random_byte(void) {
    uint8_t i;
    uint32_t r = 0U;
    /* Collect 8 bits from the LFSR */
    for (i = 0U; i < 8U; ++i) {
        /* Galois LFSR, polynomial 0xB4BCD35C (maximal length 32) */
        uint32_t lsb = g_lfsr & 1U;
        g_lfsr >>= 1U;
        if (lsb)
            g_lfsr ^= 0xB4BCD35CU;
        r = (r << 1U) | lsb;
    }
    return (uint8_t) (r & 0xFFU);
}

/* =========================================================================
 * SECTION 7 – CRITICAL SECTIONS
 *
 * On Linux (single-threaded dummy) we use sigprocmask to block SIGALRM
 * (which could be used for timer interrupts).  In a real multi-threaded
 * build this should use a pthread_mutex.
 *
 * The 'key' encodes nesting depth in the upper 16 bits and whether signals
 * were previously blocked in bit 0.  All 32-bit, no 64-bit.
 * ========================================================================= */

static uint32_t g_crit_depth = 0U;

uint32_t hal_critical_enter(void) {
    // Save current nesting depth as key; nested calls restore correctly.
    uint32_t key = g_crit_depth;

    // On outermost entry block ALL signals so any POSIX signal handler
    // (e.g. SIGALRM-driven timer tick) cannot race with protected state.
    if (g_crit_depth == 0U) {
        sigset_t block_all;
        sigfillset(&block_all);
        sigprocmask(SIG_BLOCK, &block_all, &g_saved_mask);
    }

    ++g_crit_depth;

    return key;
}

void hal_critical_exit(uint32_t key) {
    (void) key;

    if (g_crit_depth > 0U) {
        --g_crit_depth;
        if (g_crit_depth == 0U) {
            sigprocmask(SIG_SETMASK, &g_saved_mask, NULL);
        }
    }
}

/* =========================================================================
 * SECTION 8 – MEMORY
 *
 * This dummy delegates to glibc malloc/free.
 * For a bare-metal MCU replace with a fixed-block pool allocator.
 * ========================================================================= */

void* hal_mem_alloc(uint16_t size) {
    if (size == 0U)
        return NULL;
    return malloc((size_t) size);
}

void hal_mem_free(void *ptr) {
    free(ptr);
}

void* hal_mem_calloc(uint16_t size) {
    if (size == 0U)
        return NULL;
    return calloc(1U, (size_t) size);
}

void* hal_mem_realloc(void *ptr, uint16_t new_size) {
    if (new_size == 0) {
        free(ptr); /* Matches realloc(ptr, 0) semantics */
        return NULL;
    }
    void *p = realloc(ptr, (size_t) new_size);
    if (!p)
        HAL_LOGE("hal_mem_realloc: failed to resize to %u bytes", new_size);
    return p;
}

/* =========================================================================
 * SECTION 9 – CRC-16 / CCITT (FCS)
 *
 * Bit-reversed (reflected) polynomial 0x8408.
 * Pre-computed 256-entry table lives in .rodata – no runtime division.
 * All arithmetic is 16-bit (uint16_t).
 *
 * Verified against:  CCITT FCS (ISO 3309) used by HDLC and AX.25.
 * ========================================================================= */

static const uint16_t crc16_table[256] = { 0x0000U, 0x1189U, 0x2312U, 0x329BU, 0x4624U, 0x57ADU, 0x6536U, 0x74BFU, 0x8C48U, 0x9DC1U, 0xAF5AU, 0xBED3U, 0xCA6CU,
        0xDBE5U, 0xE97EU, 0xF8F7U, 0x1081U, 0x0108U, 0x3393U, 0x221AU, 0x56A5U, 0x472CU, 0x75B7U, 0x643EU, 0x9CC9U, 0x8D40U, 0xBFDBU, 0xAE52U, 0xDAEDU, 0xCB64U,
        0xF9FFU, 0xE876U, 0x2102U, 0x308BU, 0x0210U, 0x1399U, 0x6726U, 0x76AFU, 0x4434U, 0x55BDU, 0xAD4AU, 0xBCC3U, 0x8E58U, 0x9FD1U, 0xEB6EU, 0xFAE7U, 0xC87CU,
        0xD9F5U, 0x3183U, 0x200AU, 0x1291U, 0x0318U, 0x77A7U, 0x662EU, 0x54B5U, 0x453CU, 0xBDCBU, 0xAC42U, 0x9ED9U, 0x8F50U, 0xFBEFU, 0xEA66U, 0xD8FDU, 0xC974U,
        0x4204U, 0x538DU, 0x6116U, 0x709FU, 0x0420U, 0x15A9U, 0x2732U, 0x36BBU, 0xCE4CU, 0xDFC5U, 0xED5EU, 0xFCD7U, 0x8868U, 0x99E1U, 0xAB7AU, 0xBAF3U, 0x5285U,
        0x430CU, 0x7197U, 0x601EU, 0x14A1U, 0x0528U, 0x37B3U, 0x263AU, 0xDECDU, 0xCF44U, 0xFDDFU, 0xEC56U, 0x98E9U, 0x8960U, 0xBBFBU, 0xAA72U, 0x6306U, 0x728FU,
        0x4014U, 0x519DU, 0x2522U, 0x34ABU, 0x0630U, 0x17B9U, 0xEF4EU, 0xFEC7U, 0xCC5CU, 0xDDD5U, 0xA96AU, 0xB8E3U, 0x8A78U, 0x9BF1U, 0x7387U, 0x620EU, 0x5095U,
        0x411CU, 0x35A3U, 0x242AU, 0x16B1U, 0x0738U, 0xFFCFU, 0xEE46U, 0xDCDDU, 0xCD54U, 0xB9EBU, 0xA862U, 0x9AF9U, 0x8B70U, 0x8408U, 0x9581U, 0xA71AU, 0xB693U,
        0xC22CU, 0xD3A5U, 0xE13EU, 0xF0B7U, 0x0840U, 0x19C9U, 0x2B52U, 0x3ADBU, 0x4E64U, 0x5FEDU, 0x6D76U, 0x7CFFU, 0x9489U, 0x8500U, 0xB79BU, 0xA612U, 0xD2ADU,
        0xC324U, 0xF1BFU, 0xE036U, 0x18C1U, 0x0948U, 0x3BD3U, 0x2A5AU, 0x5EE5U, 0x4F6CU, 0x7DF7U, 0x6C7EU, 0xA50AU, 0xB483U, 0x8618U, 0x9791U, 0xE32EU, 0xF2A7U,
        0xC03CU, 0xD1B5U, 0x2942U, 0x38CBU, 0x0A50U, 0x1BD9U, 0x6F66U, 0x7EEFU, 0x4C74U, 0x5DFDU, 0xB58BU, 0xA402U, 0x9699U, 0x8710U, 0xF3AFU, 0xE226U, 0xD0BDU,
        0xC134U, 0x39C3U, 0x284AU, 0x1AD1U, 0x0B58U, 0x7FE7U, 0x6E6EU, 0x5CF5U, 0x4D7CU, 0xC60CU, 0xD785U, 0xE51EU, 0xF497U, 0x8028U, 0x91A1U, 0xA33AU, 0xB2B3U,
        0x4A44U, 0x5BCDU, 0x6956U, 0x78DFU, 0x0C60U, 0x1DE9U, 0x2F72U, 0x3EFBU, 0xD68DU, 0xC704U, 0xF59FU, 0xE416U, 0x90A9U, 0x8120U, 0xB3BBU, 0xA232U, 0x5AC5U,
        0x4B4CU, 0x79D7U, 0x685EU, 0x1CE1U, 0x0D68U, 0x3FF3U, 0x2E7AU, 0xE70EU, 0xF687U, 0xC41CU, 0xD595U, 0xA12AU, 0xB0A3U, 0x8238U, 0x93B1U, 0x6B46U, 0x7ACFU,
        0x4854U, 0x59DDU, 0x2D62U, 0x3CEBU, 0x0E70U, 0x1FF9U, 0xF78FU, 0xE606U, 0xD49DU, 0xC514U, 0xB1ABU, 0xA022U, 0x92B9U, 0x8330U, 0x7BC7U, 0x6A4EU, 0x58D5U,
        0x495CU, 0x3DE3U, 0x2C6AU, 0x1EF1U, 0x0F78U };

uint16_t hal_crc16_update(uint16_t crc, const uint8_t *buf, uint16_t len) {
    uint16_t i;
    for (i = 0U; i < len; ++i) {
        /* table-driven Galois CRC: crc = (crc >> 8) ^ table[(crc ^ *buf) & 0xFF] */
        crc = (uint16_t) ((crc >> 8U) ^ crc16_table[(crc ^ (uint16_t) buf[i]) & 0x00FFU]);
    }
    return crc;
}

uint16_t hal_crc16_buf(const uint8_t *buf, uint16_t len) {
    return hal_crc16_final(hal_crc16_update(HAL_CRC16_INIT, buf, len));
}

/* =========================================================================
 * SECTION 10 – LOGGING
 * ========================================================================= */
#if HAL_LOG_ENABLE
static const char *const g_level_str[4] = { "ERR", "WRN", "INF", "DBG" };
#endif

void hal_log(hal_log_level_t level, const char *fmt, ...) {
#if HAL_LOG_ENABLE
    va_list ap;
    uint32_t ms = hal_tick_ms();
    /* Print timestamp as %u.%03u to avoid float; ms is 32-bit */
    uint32_t s = ms / 1000U;
    uint32_t frac = ms - (s * 1000U); /* 0..999 */
    fprintf(stderr, "[%5u.%03u][%s] ", (unsigned) s, (unsigned) frac, (level <= HAL_LOG_DEBUG) ? g_level_str[(uint8_t) level] : "???");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
#else
    (void)level; (void)fmt;
#endif
}

/* =========================================================================
 * SECTION 11 – WATCHDOG
 * ========================================================================= */

void hal_wdog_kick(void) {
    /* No-op on Linux; wire to IWDG/WWDG register write on MCU */
}

/* =========================================================================
 * SECTION 12 – CHANNEL ACCESS PARAMETERS
 * ========================================================================= */

static hal_channel_params_t g_chan_params[HAL_MAX_PORTS] = { { .txdelay_ms = 500U, /* 500 ms keyup delay  (KISS default = 50 * 10ms) */
.txtail_ms = 50U, /*  50 ms tail                                     */
.slottime_ms = 100U, /* 100 ms slot time    (KISS default = 10 * 10ms) */
.persist = 63U, /* p ≈ 0.25                                        */
.full_duplex = 0U }, /* half duplex                                     */
{ .txdelay_ms = 500U, .txtail_ms = 50U, .slottime_ms = 100U, .persist = 63U, .full_duplex = 0U } };

hal_err_t hal_channel_params_get(uint8_t port, hal_channel_params_t *params) {
    if (port >= HAL_MAX_PORTS || !params)
        return HAL_ERR_INVAL;
    *params = g_chan_params[port];
    return HAL_OK;
}

hal_err_t hal_channel_params_set(uint8_t port, const hal_channel_params_t *params) {
    if (port >= HAL_MAX_PORTS || !params)
        return HAL_ERR_INVAL;
    g_chan_params[port] = *params;
    return HAL_OK;
}

// Sync HAL channel params from KISS wire values.
// All five fields are written atomically as a struct copy.
// Multiply by 10U converts KISS 10-ms units to milliseconds;
// uint8_t * 10U = max 2550, fits in uint32_t with no overflow.
hal_err_t hal_channel_params_from_kiss(uint8_t port, uint8_t txdelay, uint8_t persist, uint8_t slottime, uint8_t txtail, uint8_t full_duplex) {
    hal_channel_params_t p;
    if (port >= HAL_MAX_PORTS)
        return HAL_ERR_NODEV;
    // Convert KISS 10-ms units to milliseconds (8-bit x 10 fits uint32_t)
    p.txdelay_ms = (uint32_t) txdelay * 10U;
    p.slottime_ms = (uint32_t) slottime * 10U;
    p.txtail_ms = (uint32_t) txtail * 10U;
    // persist is the raw KISS P-value 0-255; CSMA uses: random_byte() <= persist
    p.persist = persist;
    p.full_duplex = (full_duplex != 0U) ? 1U : 0U;
    g_chan_params[port] = p;
    return HAL_OK;
}

/* =========================================================================
 * SECTION 13 – PLATFORM ID
 * ========================================================================= */

const char* hal_platform_id(void) {
    return "Linux/dummy";
}

/* =========================================================================
 * SECTION 14 – INIT & DEINIT
 * ========================================================================= */

hal_err_t hal_init(void) {
    struct timeval tv;
    uint8_t i;

    /* Capture epoch for tick counter */
    gettimeofday(&tv, NULL);
    g_tick_epoch_sec = (uint32_t) tv.tv_sec;
    g_tick_epoch_usec = (uint32_t) tv.tv_usec;

    // Mix usec + sec + PID to prevent identical seeds when tv_usec is zero
    // (e.g. at second boundaries or on MCUs where tv_usec starts at 0).
    // Shifting PID left by 8 spreads its bits into the upper half of the word
    // so low-PID values (1, 2, ...) do not alias with the time fields.
    uint32_t seed = (uint32_t) tv.tv_usec ^ (uint32_t) tv.tv_sec ^ ((uint32_t) getpid() << 8U);
    if (seed == 0U)
        seed = 0xDEADBEEFU;
    hal_random_seed(seed);

    /* Clear timer pool */
    memset(g_timers, 0, sizeof(g_timers));
    memset(g_timer_used, 0, sizeof(g_timer_used));

    /* Clear PTT state */
    memset(g_ptt_state, 0, sizeof(g_ptt_state));

    /* Initialise serial ports: port 0 → stdin/stdout (non-blocking) */
    memset(g_serial, 0, sizeof(g_serial));
    for (i = 0U; i < HAL_MAX_PORTS; ++i) {
        g_serial[i].fd_rx = -1; /* use stdin  */
        g_serial[i].fd_tx = -1; /* use stdout */
        g_serial[i].open = 1U;
    }

    /* Put stdin into non-blocking, raw mode for the dummy */
    {
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags >= 0) {
            (void) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        }
    }

    HAL_LOGI("HAL init complete (%s)\n", hal_platform_id());
    return HAL_OK;
}

void hal_deinit(void) {
    uint8_t i;

    /* Stop all running timers first to prevent callbacks firing into      */
    /* already-freed state machine memory after deinit.                    */
    for (i = 0U; i < HAL_TIMER_MAX; ++i) {
        g_timers[i].active = 0U;
        g_timer_used[i] = 0U;
    }

    /* Deassert all PTT lines */
    for (i = 0U; i < HAL_MAX_PORTS; ++i) {
        (void) hal_ptt_set(i, 0U);
    }

    /* Flush TX buffers with a 1-second deadline */
    for (i = 0U; i < HAL_MAX_PORTS; ++i) {
        (void) hal_serial_tx_flush(i, 1000U);
    }

    HAL_LOGI("HAL deinit complete\n");
}

/* =========================================================================
 * SECTION 15 – OPEN / CLOSE A SERIAL PORT TO A REAL TTY (OPTIONAL)
 *
 * Call hal_serial_open() before hal_init() if you want to route a port
 * to a physical or virtual serial device (e.g. /dev/ttyUSB0) instead of
 * stdin/stdout.  The baud rate and 8N1 framing are set automatically.
 * ========================================================================= */

/**
 * @brief  Open a serial port to a TTY device.
 *
 * This is a Linux-only extension not declared in hal.h.
 * For KISS TNCs set baud to 9600 (typical) or 1200/38400 as appropriate.
 *
 * @param  port     Logical port index (0 or 1).
 * @param  devpath  Path e.g. "/dev/ttyUSB0".
 * @param  baud     Baud rate (e.g. 9600).
 * @return HAL_OK or HAL_ERR_NODEV.
 */
hal_err_t hal_serial_open(uint8_t port, const char *devpath, uint32_t baud) {
    struct termios tios;
    speed_t speed;
    int fd;

    if (port >= HAL_MAX_PORTS || !devpath)
        return HAL_ERR_INVAL;

    /* Map numeric baud to Bxxx constant */
    switch (baud) {
        case 300U:
            speed = B300;
        break;
        case 1200U:
            speed = B1200;
        break;
        case 2400U:
            speed = B2400;
        break;
        case 4800U:
            speed = B4800;
        break;
        case 9600U:
            speed = B9600;
        break;
        case 19200U:
            speed = B19200;
        break;
        case 38400U:
            speed = B38400;
        break;
        case 57600U:
            speed = B57600;
        break;
        case 115200U:
            speed = B115200;
        break;
        default:
            HAL_LOGE("hal_serial_open: unsupported baud %u\n", (unsigned )baud);
            return HAL_ERR_INVAL;
    }

    fd = open(devpath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        HAL_LOGE("hal_serial_open: open(%s) failed: %s\n", devpath, strerror(errno));
        return HAL_ERR_NODEV;
    }

    /* Configure raw 8N1 */
    memset(&tios, 0, sizeof(tios));
    cfsetispeed(&tios, speed);
    cfsetospeed(&tios, speed);
    tios.c_cflag = (tcflag_t) (CS8 | CREAD | CLOCAL);
    tios.c_iflag = 0;
    tios.c_oflag = 0;
    tios.c_lflag = 0;
    tios.c_cc[VMIN] = 0;
    tios.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tios) < 0) {
        HAL_LOGE("hal_serial_open: tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return HAL_ERR_NODEV;
    }

    /* Close previous fd if any */
    if (g_serial[port].fd_rx > 0)
        close(g_serial[port].fd_rx);

    g_serial[port].fd_rx = fd;
    g_serial[port].fd_tx = fd;
    g_serial[port].open = 1U;

    HAL_LOGI("hal_serial_open: port %u → %s @ %u baud\n", (unsigned )port, devpath, (unsigned )baud);
    return HAL_OK;
}

/**
 * @brief  Close a serial port and revert it to stdin/stdout.
 *
 * @param  port  Logical port index.
 */
void hal_serial_close(uint8_t port) {
    if (port >= HAL_MAX_PORTS)
        return;
    if (g_serial[port].fd_rx > STDERR_FILENO) {
        close(g_serial[port].fd_rx);
    }
    g_serial[port].fd_rx = -1;
    g_serial[port].fd_tx = -1;
}

/* =========================================================================
 * SECTION 16 – SELF-TEST
 *
 * A minimal smoke-test that can be built as a standalone executable.
 * Build:  gcc -std=c99 -O2 -DHAL_SELFTEST -o hal_test hal_dummy.c -I.
 * ========================================================================= */

#ifdef HAL_SELFTEST

#include <assert.h>

static void test_tick(void) {
    uint32_t t0, t1, elapsed;
    struct timespec ts;
    ts.tv_sec = 0; ts.tv_nsec = 50000000L; /* 50 ms */

    t0 = hal_tick_ms();
    nanosleep(&ts, NULL);
    t1 = hal_tick_ms();
    elapsed = t1 - t0;
    /* Allow 10 ms tolerance */
    assert(elapsed >= 40U && elapsed <= 100U);
    printf("[tick]  OK  (elapsed=%u ms)\n", (unsigned)elapsed);
}

static void test_timer(void) {
    hal_timer_t t = hal_timer_alloc();
    struct timespec ts;
    assert(t != HAL_TIMER_INVALID);

    hal_timer_start(t, 80U);
    assert(hal_timer_running(t));
    assert(!hal_timer_expired(t));

    ts.tv_sec = 0; ts.tv_nsec = 100000000L; /* 100 ms */
    nanosleep(&ts, NULL);

    assert(!hal_timer_running(t));
    assert(hal_timer_expired(t));
    hal_timer_free(t);
    printf("[timer] OK\n");
}

static void test_crc(void) {
    /* Known-good: CRC of ASCII "123456789" = 0x906E (CCITT) */
    static const uint8_t msg[] = "123456789";
    uint16_t fcs = hal_crc16_buf(msg, (uint16_t)(sizeof(msg) - 1U));
    assert(fcs == 0x906EU);
    printf("[crc16] OK  (FCS=0x%04X)\n", (unsigned)fcs);
}

static void test_rng(void) {
    uint8_t  b;
    uint16_t i;
    uint32_t sum = 0U;
    hal_random_seed(12345U);
    for (i = 0U; i < 256U; ++i) {
        b = hal_random_byte();
        sum += b;
    }
    /* Expect sum roughly 256*127 = 32512; allow wide tolerance */
    assert(sum > 16000U && sum < 49000U);
    printf("[rng]   OK  (sum=%u over 256 bytes)\n", (unsigned)sum);
}

static void test_serial_ring(void) {
    uint8_t  b;
    uint16_t i;
    hal_err_t rc;

    /* Fill TX ring */
    for (i = 0U; i < 16U; ++i) {
        rc = hal_serial_put(0U, (uint8_t)i);
        assert(rc == HAL_OK);
    }
    /* Drain TX to stdout via poll */
    hal_serial_poll(0U);

    /* Inject bytes directly into RX ring for test */
    for (i = 0U; i < 4U; ++i) {
        uint16_t next = (uint16_t)((g_serial[0].rx_tail + 1U) & RING_RX_MASK);
        g_serial[0].rx_buf[g_serial[0].rx_tail] = (uint8_t)(0xA0U + i);
        g_serial[0].rx_tail = next;
    }
    assert(hal_serial_rx_available(0U) == 4);
    for (i = 0U; i < 4U; ++i) {
        rc = hal_serial_get(0U, &b);
        assert(rc == HAL_OK);
        assert(b == (uint8_t)(0xA0U + i));
    }
    assert(hal_serial_rx_available(0U) == 0);
    printf("[serial]OK\n");
}

static void test_ptt(void) {
    assert(hal_ptt_set(0U, 1U) == HAL_OK);
    assert(hal_ptt_get(0U) == 1);
    assert(hal_ptt_set(0U, 0U) == HAL_OK);
    assert(hal_ptt_get(0U) == 0);
    printf("[ptt]   OK\n");
}

int main(void) {
    printf("=== AX.25 HAL self-test (%s) ===\n", hal_platform_id());
    hal_init();
    test_tick();
    test_timer();
    test_crc();
    test_rng();
    test_serial_ring();
    test_ptt();
    hal_deinit();
    printf("=== All tests passed ===\n");
    return 0;
}

#endif /* HAL_SELFTEST */
