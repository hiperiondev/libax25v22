/**
 * @file    hal.h
 * @brief   AX.25 v2.2 Hardware Abstraction Layer (HAL) API
 *
 * This header defines ALL hardware-dependent interfaces required by an
 * AX.25 v2.2 implementation.  Every function must be provided by a
 * platform-specific translation unit (e.g. hal_linux.c, hal_stm32.c …).
 *
 * Design constraints
 * ------------------
 *  - No 64-bit integer arithmetic anywhere in this API.
 *  - No floating-point types.
 *  - Tick / time values are 32-bit unsigned millisecond counters
 *    (wrap-around after ~49 days; the protocol must handle wrap).
 *  - Suitable for bare-metal microcontrollers without FPU.
 *
 * AX.25 v2.2 hardware dependencies (per spec July 1998)
 * ------------------------------------------------------
 *  1. Monotonic millisecond tick counter     (T1/T2/T3/TXDELAY timing)
 *  2. Software timers (start/stop/query)     (T1, T2, T3, IDLE)
 *  3. PTT (Push-To-Talk) keying             (transmitter on/off)
 *  4. DCD (Data Carrier Detect)             (channel busy sensing)
 *  5. KISS / serial byte I/O                (TNC ↔ host frame exchange)
 *  6. Pseudo-random byte                    (CSMA p-persistence)
 *  7. Critical section (interrupt masking)  (shared-state protection)
 *  8. Memory allocation (static or dynamic) (frame buffers)
 *  9. CRC-16/CCITT (FCS)                    (optionally hardware-assisted)
 * 10. Logging / debug output               (optional, compile-time flag)
 * 11. Watchdog / idle kick                 (optional, platform safety)
 *
 * All time values are in milliseconds unless stated otherwise.
 *
 */

#ifndef AX25_HAL_H
#define AX25_HAL_H

#include <stdint.h>
#include <stddef.h>

#include "ax25_mux.h"

/* =========================================================================
 * 0.  COMPILE-TIME PLATFORM CONFIGURATION
 * ========================================================================= */

/**
 * Maximum number of concurrent software timers the HAL must support.
 * AX.25 v2.2 needs at minimum 4 per logical link (T1, T2, T3, IDLE).
 * Increase for multi-link implementations.
 */
#ifndef HAL_TIMER_MAX
#define HAL_TIMER_MAX   ((AX25_MUX_MAX_LINKS * 4U) + 4U)
#endif

/**
 * Size of the receive ring buffer (bytes).
 * NOTE: Actual usable capacity is (HAL_SERIAL_RX_BUF_SIZE - 1) bytes
 * due to the sentinel-based full/empty distinction used by the ring buffer
 * (head == tail means empty; (tail+1)&mask == head means full).
 * Callers must not assume all HAL_SERIAL_RX_BUF_SIZE bytes are storable.
 * Must be a power of two.
 */
#ifndef HAL_SERIAL_RX_BUF_SIZE
#define HAL_SERIAL_RX_BUF_SIZE   512U
#endif

/* Actual maximum bytes that can be buffered before overflow occurs. */
#define HAL_SERIAL_RX_BUF_CAPACITY  (HAL_SERIAL_RX_BUF_SIZE - 1U)

/**
 * Size of the transmit ring buffer (bytes).
 */
#ifndef HAL_SERIAL_TX_BUF_SIZE
#define HAL_SERIAL_TX_BUF_SIZE   512
#endif

/**
 * Maximum number of simultaneous memory allocations the HAL pool must
 * support.  Used only when HAL_MEM_USE_POOL is defined.
 */
#ifndef HAL_MEM_POOL_ENTRIES
#define HAL_MEM_POOL_ENTRIES  32
#endif

/** Enable debug/log output (define to 0 to strip all log calls). */
#ifndef HAL_LOG_ENABLE
#define HAL_LOG_ENABLE  1
#endif

/* =========================================================================
 * 1.  RETURN CODES
 * ========================================================================= */

typedef int8_t hal_err_t;

#define HAL_OK          ( 0)  /**< Success                                  */
#define HAL_ERR_GENERIC (-1)  /**< Unspecified error                        */
#define HAL_ERR_BUSY    (-2)  /**< Resource is busy / buffer full           */
#define HAL_ERR_TIMEOUT (-3)  /**< Operation timed out                      */
#define HAL_ERR_INVAL   (-4)  /**< Invalid argument                         */
#define HAL_ERR_NOMEM   (-5)  /**< Out of memory                            */
#define HAL_ERR_NODEV   (-6)  /**< Device not present / not initialised     */

/* =========================================================================
 * 2.  INITIALISATION & SHUTDOWN
 * ========================================================================= */

/**
 * @brief  Initialise all HAL subsystems.
 *
 * Must be called once before any other HAL function.  Implementations
 * should initialise timers, serial hardware, PTT GPIO, CRC unit, etc.
 *
 * @return HAL_OK on success, negative error code otherwise.
 */
hal_err_t hal_init(void);

/**
 * @brief  Shut down all HAL subsystems gracefully.
 *
 * Implementations should deassert PTT, disable interrupts as needed,
 * flush buffers, and release any OS resources.
 */
void hal_deinit(void);

/* =========================================================================
 * 3.  MONOTONIC TICK COUNTER
 *
 * The tick counter is the foundation of all protocol timing.  It must be
 * a free-running 32-bit counter incremented every millisecond.  Wrap-
 * around is allowed; callers use modular arithmetic (a-b) to measure
 * elapsed time.
 * ========================================================================= */

/**
 * @brief  Return the current monotonic tick value in milliseconds.
 *
 * Must be safe to call from both normal and interrupt context.
 * Must NOT use 64-bit arithmetic internally.
 *
 * @return 32-bit millisecond tick counter (wraps after ~49.7 days).
 */
uint32_t hal_tick_ms(void);

/**
 * @brief  Return elapsed milliseconds since a reference tick.
 *
 * Handles 32-bit wrap transparently.
 *
 * @param  since_ms   Reference tick obtained from hal_tick_ms().
 * @return Elapsed time in milliseconds (capped at UINT32_MAX).
 */
static inline uint32_t hal_elapsed_ms(uint32_t since_ms) {
    return hal_tick_ms() - since_ms; /* unsigned subtraction wraps OK */
}

/* =========================================================================
 * 4.  SOFTWARE TIMERS
 *
 * AX.25 v2.2 requires four timers per logical link:
 *   T1  – Retransmission timer    (default 3 × round-trip propagation)
 *   T2  – Response delay timer    (default 3 s,  min 100 ms)
 *   T3  – Inactive link timer     (default 300 s)
 *   IDLE– Connected-mode idle     (configurable)
 *
 * The HAL provides a simple start/stop/expired interface backed by the
 * monotonic tick counter.  No OS threads are required.
 * ========================================================================= */

/** Opaque timer handle.  Negative value = invalid / stopped. */
typedef int8_t hal_timer_t;

#define HAL_TIMER_INVALID  ((hal_timer_t)-1)

/**
 * @brief  Allocate a software timer slot.
 *
 * The returned handle must be passed to all subsequent timer calls.
 * The timer is initially stopped.
 *
 * @return Valid handle (>= 0), or HAL_TIMER_INVALID if no slots remain.
 */
hal_timer_t hal_timer_alloc(void);

/**
 * @brief  Release a software timer slot back to the pool.
 *
 * The timer is stopped (if running) before being freed.
 *
 * @param  t   Timer handle returned by hal_timer_alloc().
 */
void hal_timer_free(hal_timer_t t);

/**
 * @brief  Start (or restart) a software timer.
 *
 * @param  t          Timer handle.
 * @param  period_ms  Timeout interval in milliseconds (32-bit, no floats).
 * @return HAL_OK or HAL_ERR_INVAL if t is invalid.
 */
hal_err_t hal_timer_start(hal_timer_t t, uint32_t period_ms);

/**
 * @brief  Stop a running software timer without freeing the slot.
 *
 * @param  t   Timer handle.
 */
void hal_timer_stop(hal_timer_t t);

/**
 * @brief  Query whether a timer has expired.
 *
 * A timer that has never been started, or has been stopped, returns 0.
 *
 * @param  t   Timer handle.
 * @return 1 if expired, 0 if still running or stopped.
 */
uint8_t hal_timer_expired(hal_timer_t t);

/**
 * @brief  Query whether a timer is currently running (started, not expired).
 *
 * @param  t   Timer handle.
 * @return 1 if running, 0 otherwise.
 */
uint8_t hal_timer_running(hal_timer_t t);

/**
 * @brief  Return the remaining time (ms) on a running timer.
 *
 * @param  t   Timer handle.
 * @return Remaining milliseconds, or 0 if stopped/expired.
 */
uint32_t hal_timer_remaining_ms(hal_timer_t t);

/* =========================================================================
 * 5.  PTT – PUSH TO TALK
 *
 * The PTT line keys (asserts) the radio transmitter.  The AX.25 state
 * machine must assert PTT before any frame is sent and deassert it after
 * the TX tail time has elapsed following the last transmitted bit.
 *
 * On a multi-port implementation an 8-bit port index selects which radio.
 * ========================================================================= */

/**
 * @brief  Assert or deassert the PTT line for a given port.
 *
 * @param  port    Logical port index (0 = first radio).
 * @param  assert  1 = key transmitter, 0 = unkey transmitter.
 * @return HAL_OK or HAL_ERR_NODEV if port is invalid.
 */
hal_err_t hal_ptt_set(uint8_t port, uint8_t assert_tx);

/**
 * @brief  Return current PTT state.
 *
 * @param  port  Logical port index.
 * @return 1 if PTT is asserted, 0 if not, negative on error.
 */
int8_t hal_ptt_get(uint8_t port);

/* =========================================================================
 * 6.  DCD – DATA CARRIER DETECT
 *
 * AX.25 CSMA access uses DCD to detect a busy channel before transmitting.
 * DCD may be provided by a hardware modem input pin or by software squelch.
 * ========================================================================= */

/**
 * @brief  Return whether the channel is currently busy (carrier detected).
 *
 * @param  port  Logical port index.
 * @return 1 if channel is busy (carrier present), 0 if idle,
 *         negative on error.
 */
int8_t hal_dcd_get(uint8_t port);

/* =========================================================================
 * 7.  SERIAL / KISS BYTE I/O
 *
 * AX.25 frames are exchanged with the physical TNC via the KISS protocol
 * over an asynchronous serial link (UART / USB-CDC).
 * The HAL provides non-blocking byte-at-a-time primitives; frame assembly
 * and KISS encoding/decoding are handled by the protocol layer above.
 *
 * KISS special bytes (for reference):
 *   FEND  = 0xC0  Frame End (also used as start delimiter)
 *   FESC  = 0xDB  Frame Escape
 *   TFEND = 0xDC  Transposed FEND
 *   TFESC = 0xDD  Transposed FESC
 * ========================================================================= */

/**
 * @brief  Write a single byte to the serial transmit buffer.
 *
 * Non-blocking.  Returns HAL_ERR_BUSY if the TX buffer is full.
 *
 * @param  port  Logical port index.
 * @param  byte  Byte to send.
 * @return HAL_OK or HAL_ERR_BUSY or HAL_ERR_NODEV.
 */
hal_err_t hal_serial_put(uint8_t port, uint8_t byte);

/**
 * @brief  Write a buffer of bytes to the serial transmit buffer.
 *
 * May be implemented as repeated calls to hal_serial_put().
 *
 * @param  port   Logical port index.
 * @param  buf    Pointer to data.
 * @param  len    Number of bytes to write.
 * @return Number of bytes actually written (may be less if buffer fills).
 */
uint16_t hal_serial_write(uint8_t port, const uint8_t *buf, uint16_t len);

/**
 * @brief  Read a single byte from the serial receive buffer.
 *
 * Non-blocking.  Returns HAL_ERR_BUSY if no byte is available.
 *
 * @param  port  Logical port index.
 * @param  byte  Destination for the received byte.
 * @return HAL_OK if a byte was read, HAL_ERR_BUSY if buffer is empty,
 *         negative error code on hardware fault.
 */
hal_err_t hal_serial_get(uint8_t port, uint8_t *byte);

/**
 * @brief  Return number of bytes available in the receive buffer.
 *
 * @param  port  Logical port index.
 * @return Byte count (0 = empty), or negative on error.
 */
int16_t hal_serial_rx_available(uint8_t port);

/**
 * @brief  Return number of free bytes remaining in the transmit buffer.
 *
 * @param  port  Logical port index.
 * @return Free byte count, or negative on error.
 */
int16_t hal_serial_tx_free(uint8_t port);

/**
 * @brief  Flush (discard) all pending bytes in the receive buffer.
 *
 * @param  port  Logical port index.
 */
void hal_serial_rx_flush(uint8_t port);

/**
 * @brief  Wait (spin or yield) until the transmit buffer is empty.
 *
 * Used to ensure all bytes have been sent before deasserting PTT.
 * Implementations on RTOS platforms may yield instead of busy-waiting.
 *
 * @param  port       Logical port index.
 * @param  timeout_ms Maximum wait in milliseconds.
 * @return HAL_OK when buffer is empty, HAL_ERR_TIMEOUT otherwise.
 */
hal_err_t hal_serial_tx_flush(uint8_t port, uint32_t timeout_ms);

// Non-blocking transmit-idle query.
// Returns 1 when the TX ring buffer is empty and no bytes remain in flight.
// Used by the AX.25 physical layer to detect when it is safe to deassert PTT
// without blocking the main loop.
// Returns negative (HAL_ERR_NODEV) for an invalid or closed port.
int8_t hal_tx_idle(uint8_t port);

/* =========================================================================
 * 8.  PSEUDO-RANDOM NUMBER GENERATOR
 *
 * Required for the CSMA p-persistence algorithm:
 *   "With probability p, transmit; with probability (1-p), wait one
 *    slot time and repeat."  (AX.25 v2.2 §6.3)
 *
 * The RNG must NOT use floating-point.  A 32-bit LCG or LFSR is sufficient.
 * ========================================================================= */

/**
 * @brief  Return a pseudo-random 8-bit value in [0, 255].
 *
 * @return Random byte.
 */
uint8_t hal_random_byte(void);

/**
 * @brief  Seed the pseudo-random generator.
 *
 * Called once at startup, ideally from an analogue noise source or timer.
 *
 * @param  seed  32-bit seed value (no 64-bit needed).
 */
void hal_random_seed(uint32_t seed);

/* =========================================================================
 * 9.  CRITICAL SECTIONS
 *
 * Shared data structures (timer state, ring buffers, link state machines)
 * must be protected against concurrent access from interrupt handlers.
 *
 * On bare-metal MCUs these disable/restore the global interrupt flag.
 * On RTOS platforms they may acquire a mutex or spinlock.
 * ========================================================================= */

/**
 * @brief  Enter a critical section (disable / mask interrupts).
 *
 * Must be nestable (implementation maintains a nesting counter).
 * Returns a "key" that encodes the previous interrupt state; pass it to
 * hal_critical_exit() to restore exactly that state.
 *
 * @return Opaque interrupt-state key.
 */
uint32_t hal_critical_enter(void);

/**
 * @brief  Exit a critical section, restoring the interrupt state.
 *
 * @param  key  Value returned by the matching hal_critical_enter().
 */
void hal_critical_exit(uint32_t key);

/* =========================================================================
 * 10. MEMORY ALLOCATION
 *
 * Dynamic heap allocation is avoided in many embedded targets.  The HAL
 * may implement this as a fixed block-pool allocator (preferred), or
 * delegate to malloc/free on platforms that have a safe heap.
 *
 * All allocations are assumed to be byte-aligned to at least 4 bytes.
 * Size is 16-bit to keep arithmetic 16/32-bit on small MCUs.
 * ========================================================================= */

/**
 * @brief  Allocate a contiguous block of memory.
 *
 * @param  size  Number of bytes to allocate (max 65535).
 * @return Pointer to allocated block, or NULL on failure.
 */
void* hal_mem_alloc(uint16_t size);

/**
 * @brief  Free a previously allocated block.
 *
 * @param  ptr  Pointer returned by hal_mem_alloc().  NULL is safe.
 */
void hal_mem_free(void *ptr);

/**
 * @brief  Zero-initialised allocation (equivalent to calloc(1, size)).
 *
 * Default implementation may call hal_mem_alloc() + memset.
 *
 * @param  size  Number of bytes.
 * @return Zeroed memory block, or NULL on failure.
 */
void* hal_mem_calloc(uint16_t size);

/**
 * @brief  Resize a previously allocated memory block.
 *
 * Equivalent to realloc(ptr, new_size).  If the block cannot be grown
 * in place, a new block is allocated, the old contents are copied, and
 * the old block is freed automatically.
 *
 * - If ptr is NULL, behaves like hal_mem_alloc(new_size).
 * - If new_size is 0, frees the block and returns NULL.
 * - Returns NULL on allocation failure; the original block is NOT freed
 *   in that case (matches standard realloc semantics).
 *
 * @param  ptr       Pointer returned by a previous hal_mem_alloc /
 *                   hal_mem_calloc / hal_mem_realloc call, or NULL.
 * @param  new_size  Desired new size in bytes (max 65535).
 * @return Pointer to resized block, or NULL on failure.
 */
void* hal_mem_realloc(void *ptr, uint16_t new_size);

/* =========================================================================
 * 11. CRC-16 / CCITT (FCS)
 *
 * AX.25 uses a 16-bit CRC (CCITT polynomial 0x8408 / 0x1021 reflected)
 * as the Frame Check Sequence.  The HAL exposes this so that a hardware
 * CRC unit may be used on platforms that provide one (e.g. STM32, nRF52).
 *
 * Usage:
 *   uint16_t fcs = HAL_CRC16_INIT;
 *   fcs = hal_crc16_update(fcs, buf, len);
 *   fcs = hal_crc16_final(fcs);
 *   transmit fcs LSB then MSB.
 * ========================================================================= */

/** Initial CRC-16/CCITT value. */
#define HAL_CRC16_INIT   ((uint16_t)0xFFFFU)

/**
 * @brief  Update a running CRC-16 with a block of bytes.
 *
 * Bit-reversed (reflected) CRC matching the HDLC / AX.25 FCS.
 * Polynomial: 0x8408 (reflected 0x1021).
 *
 * @param  crc  Running CRC value (start with HAL_CRC16_INIT).
 * @param  buf  Pointer to data bytes.
 * @param  len  Number of bytes.
 * @return Updated CRC-16 value.
 */
uint16_t hal_crc16_update(uint16_t crc, const uint8_t *buf, uint16_t len);

/**
 * @brief  Finalise the CRC-16 computation.
 *
 * For AX.25 this XORs the CRC with 0xFFFF (final XOR).
 *
 * @param  crc  Value from the last hal_crc16_update() call.
 * @return Final 16-bit FCS value.
 */
static inline uint16_t hal_crc16_final(uint16_t crc) {
    return crc ^ (uint16_t) 0xFFFFU;
}

/**
 * @brief  Compute FCS over a complete buffer in one call.
 *
 * @param  buf  Data bytes (excluding flags and FCS field itself).
 * @param  len  Byte count.
 * @return 16-bit FCS ready to append to the frame (LSB first).
 */
uint16_t hal_crc16_buf(const uint8_t *buf, uint16_t len);

/* =========================================================================
 * 12. LOGGING / DEBUG OUTPUT
 *
 * Optional diagnostic output.  When HAL_LOG_ENABLE == 0 the macros expand
 * to nothing, generating zero code on resource-constrained targets.
 * ========================================================================= */

/** Log severity levels */
typedef enum {
    HAL_LOG_ERROR = 0, /**< Critical errors                        */
    HAL_LOG_WARN = 1, /**< Warnings (recoverable)                 */
    HAL_LOG_INFO = 2, /**< Informational state changes            */
    HAL_LOG_DEBUG = 3 /**< Verbose debug (frame-level)            */
} hal_log_level_t;

/**
 * @brief  Platform-specific log output function.
 *
 * Implementations may write to a UART, syslog, file, or circular buffer.
 * The format string follows printf conventions; no float conversions
 * should be used (use %%d, %%u, %%x, %%s only).
 *
 * @param  level   Severity level.
 * @param  fmt     printf-style format string (string literal).
 * @param  ...     Arguments matching the format.
 */
void hal_log(hal_log_level_t level, const char *fmt, ...);

#if HAL_LOG_ENABLE
#  define HAL_LOGE(...)  hal_log(HAL_LOG_ERROR, __VA_ARGS__)
#  define HAL_LOGW(...)  hal_log(HAL_LOG_WARN,  __VA_ARGS__)
#  define HAL_LOGI(...)  hal_log(HAL_LOG_INFO,  __VA_ARGS__)
#  define HAL_LOGD(...)  hal_log(HAL_LOG_DEBUG, __VA_ARGS__)
#else
#  define HAL_LOGE(...)  ((void)0)
#  define HAL_LOGW(...)  ((void)0)
#  define HAL_LOGI(...)  ((void)0)
#  define HAL_LOGD(...)  ((void)0)
#endif

/* =========================================================================
 * 13. WATCHDOG (OPTIONAL)
 *
 * On embedded targets a hardware watchdog resets the MCU if the firmware
 * stops responding.  The protocol main loop must call hal_wdog_kick()
 * periodically to keep the watchdog satisfied.
 * ========================================================================= */

/**
 * @brief  Reset (kick) the hardware watchdog timer.
 *
 * Call from the AX.25 main processing loop.  On platforms without a
 * watchdog this function may be left as an empty stub.
 */
void hal_wdog_kick(void);

/* =========================================================================
 * 14. CHANNEL ACCESS PARAMETERS
 *
 * These KISS-configurable parameters govern the CSMA channel access
 * algorithm.  They are stored in the HAL so that the KISS "SetHardware"
 * command (0x06) and parameter commands can update them directly.
 * ========================================================================= */

/**
 * @brief  Channel-access parameter block.
 *
 * All timing values in milliseconds (32-bit), no floats.
 */
typedef struct {
    uint32_t txdelay_ms; /**< PTT keyup delay before first flag (def 500)  */
    uint32_t txtail_ms; /**< PTT hold after last bit         (def  50)    */
    uint32_t slottime_ms; /**< CSMA slot time                  (def 100)    */
    uint8_t persist; /**< p-persistence 0-255 (p=(v+1)/256)(def  63)   */
    uint8_t full_duplex; /**< 0=half, 1=full duplex                        */
} hal_channel_params_t;

/**
 * @brief  Get current channel-access parameters for a port.
 *
 * @param  port    Logical port index.
 * @param  params  Output struct (must not be NULL).
 * @return HAL_OK or HAL_ERR_NODEV.
 */
hal_err_t hal_channel_params_get(uint8_t port, hal_channel_params_t *params);

/**
 * @brief  Set channel-access parameters for a port.
 *
 * @param  port    Logical port index.
 * @param  params  New parameter values (must not be NULL).
 * @return HAL_OK or HAL_ERR_INVAL / HAL_ERR_NODEV.
 */
hal_err_t hal_channel_params_set(uint8_t port, const hal_channel_params_t *params);

// Synchronize HAL channel parameters from a received KISS parameter set.
// Converts KISS 10-ms-unit fields to milliseconds and writes them into
// the HAL channel parameter block. No float; all arithmetic is 8/16-bit.
//
// @param port       Logical port index.
// @param txdelay    KISS TxDelay in 10 ms units.
// @param persist    KISS Persistence P value 0-255.
// @param slottime   KISS SlotTime in 10 ms units.
// @param txtail     KISS TxTail in 10 ms units.
// @param full_duplex  0 = half-duplex, 1 = full-duplex.
// @return HAL_OK or HAL_ERR_NODEV.
hal_err_t hal_channel_params_from_kiss(uint8_t port, uint8_t txdelay, uint8_t persist, uint8_t slottime, uint8_t txtail, uint8_t full_duplex);

/* =========================================================================
 * 15. PLATFORM INFO (OPTIONAL INTROSPECTION)
 * ========================================================================= */

/**
 * @brief  Return a human-readable platform identifier string.
 *
 * E.g. "Linux/dummy", "STM32F4", "AVR/Arduino".
 *
 * @return Pointer to a constant string (not heap allocated).
 */
const char* hal_platform_id(void);

/* =========================================================================
 * END OF AX25 HAL API
 * ========================================================================= */

#endif /* AX25_HAL_H */
