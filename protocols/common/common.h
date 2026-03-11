/**
 * @file common.h
 * @brief AX.25 v2.2 Protocol Library - Common Utilities and Definitions
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @section Overview
 * This header provides common utility functions, type definitions, and constants
 * used throughout the libax25v22 library. It includes CRC calculation functions
 * compliant with ISO 3309 HDLC standards, string manipulation utilities for
 * embedded systems, and timing conversion macros.
 *
 * @section Standards_References
 * - AX.25 Link Access Protocol for Amateur Packet Radio, Version 2.2 (July 1998)
 *   Section 2.2.7: Frame-Check Sequence (FCS)
 * - ISO 3309:1979 HDLC frame structure
 * - ITU-T Recommendation X.25: CRC-CCITT implementation
 *
 * @section CRC_Implementation
 * The CRC-CCITT implementation follows AX.25 v2.2 Section 2.2.7 which specifies:
 * - Generator polynomial: G(x) = x^16 + x^12 + x^5 + 1 (0x1021)
 * - Initial value: 0xFFFF (all ones)
 * - Final XOR: 0xFFFF (ones' complement of result)
 * - Bit order: LSB-first processing (reflected input)
 *
 * Two implementations are provided:
 * 1. Bit-by-bit (default): Zero memory overhead, suitable for microcontrollers
 * 2. Table-driven (USE_CRC_TABLE defined): 512 bytes flash, 8x faster
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*============================================================================*/
/* Frame Size Constants                                                       */
/*============================================================================*/

/**
 * @defgroup FrameSizeConstants Frame Size Limits
 * @brief AX.25 frame size constraints per protocol specification
 *
 * These constants define the minimum and maximum frame sizes as specified
 * in AX.25 v2.2. Frame size validation is critical for protocol compliance
 * and buffer allocation safety.
 *
 * @section Minimum_Frame_Size
 * Per AX.25 v2.2 Section 2.2.9: Any frame consisting of less than 136 bits
 * (17 bytes) including opening and closing flags shall be considered invalid.
 *
 * Minimum composition:
 * - Opening flag: 1 byte (0x7E)
 * - Destination address: 7 bytes
 * - Source address: 7 bytes
 * - Control field: 1 byte (minimum)
 * - FCS: 2 bytes
 * - Closing flag: 1 byte (may be shared with next frame)
 *
 * @section Maximum_Frame_Size
 * The maximum frame size is implementation-dependent but typically limited
 * by the maximum I-field length (N1) of 256 octets plus overhead.
 * Total maximum: 7 + 7 + (2*7) + 2 + 1 + 256 + 2 = 289 bytes (AX.25 v2.2 max, 2 digipeaters)
 */

/**
 * @brief Maximum expected AX.25 frame size in bytes
 *
 * Defines the upper bound for CRC computation and frame buffer allocation.
 * AX.25 v2.2 Section 3.12.5 limits repeater subfields to a maximum of TWO.
 * Maximum frame composition:
 *
 *   Destination address  :  7 bytes
 *   Source address       :  7 bytes
 *   2 digipeater addrs   : 14 bytes  (AX.25 v2.2 Section 3.12.5: max 2 hops)
 *   Control field        :  2 bytes  (modulo-128, 16-bit)
 *   PID field            :  1 byte
 *   I-field (default N1) : 256 bytes (AX.25 v2.2 default maximum)
 *   FCS field            :  2 bytes
 *   -------------------------
 *   Exact maximum total  : 289 bytes
 *   Safety margin        :  51 bytes  (accommodates non-standard extensions)
 *   -------------------------
 *   MAX_FRAME_SIZE       : 340 bytes
 *
 * @warning Frames exceeding this size are rejected by CRC() with the error
 *          sentinel 0xFFFF, which callers treat as a CRC failure. Any value
 *          smaller than 289 will cause silent, false CRC errors on fully-loaded
 *          frames that are otherwise perfectly valid per AX.25 v2.2.
 *          The 340-byte value is retained for compatibility with legacy
 *          implementations that may pass more than 2 digipeater subfields
 *          (e.g., AX.25 v2.0 paths with up to 8 repeaters). No valid
 *          AX.25 v2.2 frame can exceed 289 bytes.
 *
 * @note On memory-constrained targets where digipeater forwarding is unused,
 *       the minimum safe value is 280 bytes:
 *       (14 addr + 2 ctrl + 1 PID + 256 data + 2 FCS + 5 margin).
 *       Define AX25_NO_DIGIPEATER_PATH and rebuild to use that reduced limit.
 */
#ifdef AX25_NO_DIGIPEATER_PATH
// Reduced limit for targets that never forward multi-hop digipeater frames:
// 14 (2 addr) + 2 (ctrl) + 1 (PID) + 256 (I-field) + 2 (FCS) + 5 (margin)
#define MAX_FRAME_SIZE 280u
#else
// Full limit (v2.2 max 289 bytes true; 340 retained for legacy v2.0 8-repeater compatibility)
#define MAX_FRAME_SIZE 340u
#endif

/**
 * @brief Minimum valid AX.25 frame size in bytes
 *
 * Per AX.25 v2.2 Section 2.2.9, the minimum frame size is 136 bits (17 bytes)
 * including flags. This minimum ensures the frame contains at minimum:
 * - Valid address field (destination + source = 14 bytes minimum)
 * - Control field (1 byte)
 * - FCS (2 bytes)
 *
 * Frames smaller than this are considered invalid and should be discarded
 * by the link layer without further processing.
 *
 * @note This is the minimum WITH FCS. Without FCS, minimum is 15 bytes.
 * @see ax25_validate_frame_size()
 */
// Minimum AX.25 frame size including 2-byte FCS (HDLC-layer input).
// Dest(7) + Src(7) + Ctrl(1) + FCS(2) = 17 bytes.
#define AX25_MIN_FRAME_SIZE_WITH_FCS   17u

// Minimum AX.25 frame size after FCS has been stripped (decode-layer input).
// Dest(7) + Src(7) + Ctrl(1) = 15 bytes.
#define AX25_MIN_FRAME_SIZE_NO_FCS     15u

// Backward-compatible alias (with FCS) — existing callers of AX25_MIN_FRAME_SIZE unchanged.
#define AX25_MIN_FRAME_SIZE            AX25_MIN_FRAME_SIZE_WITH_FCS

/*============================================================================*/
/* Timing Conversion Macros                                                   */
/*============================================================================*/

/**
 * @defgroup TimingMacros Timing Conversion Utilities
 * @brief Macros for converting between milliseconds and system ticks
 *
 * These macros provide portable conversion between real-time milliseconds
 * and system timer ticks. The conversion assumes a 10ms tick interval
 * which is common in embedded AX.25 implementations.
 *
 * @section AX25_Timers
 * AX.25 v2.2 Section 2.4.7.1 defines several timers that must be implemented:
 * - T1 (Acknowledgement Timer): Default 3000ms, negotiable via XID
 * - T2 (Response Delay Timer): Implementation-dependent delay for batching
 * - T3 (Inactive Link Timer): Periodic link integrity verification
 *
 * @section Usage_Example
 * @code
 * uint32_t t1_timeout_ticks = MS_TO_TICKS(3000);  // 300 ticks @ 10ms
 * uint32_t t1_timeout_ms = TICKS_TO_MS(300);      // 3000ms
 * @endcode
 */

/**
 * @brief Convert milliseconds to system timer ticks
 *
 * Converts a time value in milliseconds to the equivalent number of
 * system timer ticks, assuming a 10ms tick interval.
 *
 * @param[in] ms Time value in milliseconds
 * @return Number of timer ticks (ms / 10)
 *
 * @note This macro performs integer division. Values not divisible by 10
 *       will be truncated (e.g., 15ms -> 1 tick).
 * @warning Overflow may occur if ms > 42949672950 (approximately 497 days)
 */
#define MS_TO_TICKS(ms)  ((uint32_t)(ms) / 10u)

/**
 * @brief Convert system timer ticks to milliseconds
 *
 * Converts a time value in system timer ticks to the equivalent number
 * of milliseconds, assuming a 10ms tick interval.
 *
 * @param[in] t Time value in timer ticks
 * @return Time in milliseconds (t * 10)
 *
 * @note This is the inverse operation of MS_TO_TICKS()
 * @warning Overflow may occur if t > 429496729 (approximately 4.9 days of ticks)
 */
#define TICKS_TO_MS(t)   ((uint32_t)(t) * 10u)

/*============================================================================*/
/* CRC-CCITT Function Declarations                                            */
/*============================================================================*/

/**
 * @defgroup CRCFunctions CRC-CCITT Calculation Functions
 * @brief Frame Check Sequence (FCS) calculation per ISO 3309 / AX.25 v2.2
 *
 * These functions implement the 16-bit Cyclic Redundancy Check (CRC) used
 * for error detection in AX.25 frames. The implementation is compliant with:
 * - AX.25 v2.2 Section 2.2.7: Frame-Check Sequence
 * - ISO 3309:1979 HDLC frame structure
 * - ITU-T Recommendation X.25 (CRC-CCITT)
 *
 * @section CRC_Algorithm
 * The CRC-CCITT uses the following parameters:
 * - Name: CRC-CCITT, CRC-16-X25, CRC-16-CCITT
 * - Polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
 * - Reversed polynomial (LSB-first): 0x8408
 * - Initial value: 0xFFFF (all ones)
 * - Final XOR: 0xFFFF (ones' complement)
 * - Check value: 0xF0B8 (CRC over "123456789")
 *
 * @section Implementation_Notes
 * Two implementations are available:
 * 1. Bit-by-bit (default): Processes each bit individually, zero RAM overhead
 * 2. Table-driven (USE_CRC_TABLE defined): 512-byte lookup table, ~8x faster
 *
 * The bit-by-bit method is suitable for memory-constrained microcontrollers.
 * The table-driven method is recommended when flash space is available.
 */

/**
 * @brief Calculate CRC-CCITT over a data buffer
 *
 * Computes the 16-bit Frame Check Sequence (FCS) for the provided data
 * buffer using the CRC-CCITT algorithm. This function can be used to:
 * - Generate FCS for outgoing frames (append result to frame)
 * - Verify incoming frames (result should be 0xF0B8 for valid frames)
 *
 * @param[in] frame Pointer to data buffer for CRC calculation
 * @param[in] len   Length of data in bytes
 *
 * @return Calculated CRC value (ones' complement of final shift register)
 *         Returns 0xFFFF if frame is NULL or len is 0
 *         Returns 0xFFFF if len exceeds MAX_FRAME_SIZE
 *
 * @section Usage_FCS_Generation
 * @code
 * uint8_t frame[MAX_FRAME_SIZE];
 * size_t frame_len = ...;  // Length without FCS
 * uint16_t fcs = CRC(frame, frame_len);
 * frame[frame_len++] = fcs & 0xFF;      // Low byte first (LSB)
 * frame[frame_len++] = (fcs >> 8) & 0xFF; // High byte
 * @endcode
 *
 * @section Usage_FCS_Verification
 * @code
 * uint16_t result = CRC(frame_with_fcs, total_len);
 * if (result == 0xF0B8) {
 *     // Frame is valid
 * }
 * @endcode
 *
 * @note The CRC is calculated LSB-first (bit 0 of each byte first).
 * @note For verification, calculate CRC over entire frame including FCS.
 *       A valid frame will yield the magic constant 0xF0B8.
 * @see CRC_verify() for simplified verification
 * @see MAX_FRAME_SIZE for maximum allowed length
 */
uint16_t CRC(unsigned char *frame, size_t len);

/**
 * @brief Verify frame integrity using embedded FCS
 *
 * Validates a received frame by calculating the CRC over the entire
 * frame including the Frame Check Sequence field. For a valid AX.25
 * frame, this calculation yields the magic constant 0xF0B8.
 *
 * This method is preferred over extracting and comparing the FCS because:
 * - No byte order reversal required
 * - Single CRC calculation
 * - Matches HDLC/AX.25 standard verification method
 *
 * @param[in] frame Pointer to frame buffer including FCS
 * @param[in] len   Total length of frame including FCS (must be >= 2)
 *
 * @return true if frame CRC is valid (result == 0xF0B8)
 *         false if frame is NULL, len < 2, or CRC mismatch
 *
 * @section Verification_Algorithm
 * Per AX.25 v2.2 and ISO 3309, a valid frame satisfies:
 * @code
 * CRC(frame_data || fcs_low || fcs_high) == 0xF0B8
 * @endcode
 *
 * @note The magic constant 0xF0B8 is the residual value of a correct CRC.
 * @see CRC() for the underlying calculation function
 */
bool CRC_verify(unsigned char *frame, size_t len);

/*============================================================================*/
/* String Utility Function Declarations                                       */
/*============================================================================*/

/**
 * @defgroup StringUtilities String Manipulation Utilities
 * @brief Portable string functions for embedded systems
 *
 * These functions provide common string operations that may not be
 * available in all C standard libraries, particularly on embedded
 * systems with limited libc implementations. They are designed to
 * be safe, portable, and resource-conscious.
 *
 * @section Safety_Notes
 * - All functions perform NULL pointer checks where applicable
 * - Length parameters prevent buffer overruns
 * - Memory allocation failures are handled gracefully (return NULL)
 * - String length limits prevent excessive memory usage
 */

/**
 * @brief Remove trailing space characters from a string
 *
 * Modifies the input string in-place to remove all trailing ASCII
 * space characters (0x20). This is particularly useful for AX.25
 * callsign processing, where 6-character callsigns are space-padded
 * to fit the 7-byte address field format.
 *
 * @param[in,out] str Pointer to null-terminated string to modify
 *
 * @section AX25_Usage
 * In AX.25, callsigns are encoded as 6 characters space-padded:
 * @code
 * "N0CALL  " -> "N0CALL"  (after trim)
 * "W1AW    " -> "W1AW"    (after trim)
 * @endcode
 *
 * @note This function modifies the string in-place. No memory allocation.
 * @note If str is NULL or all spaces, results in empty string ("").
 * @see ax25_address_decode() for callsign extraction
 * @see CALLSIGN_MAX in ax25.h
 */
void trim_trailing_spaces(char *str);

/**
 * @brief Portable strnlen implementation
 *
 * Returns the length of a string, but never examines more than maxlen
 * characters. This is a safer alternative to strlen() for potentially
 * unterminated strings or untrusted input.
 *
 * @param[in] s       Pointer to string (may be NULL)
 * @param[in] maxlen  Maximum number of characters to examine
 *
 * @return Length of string (excluding null terminator), capped at maxlen
 *         Returns 0 if s is NULL
 *
 * @section Security_Notes
 * This function prevents buffer overruns when examining potentially
 * unterminated strings received over radio links or from untrusted
 * sources. Essential for robust AX.25 frame parsing.
 *
 * @note Unlike strnlen_s (C11), this returns 0 for NULL input.
 * @note If no null terminator found within maxlen, returns maxlen.
 * @see strnlen_s (C11 standard)
 */
size_t my_strnlen(const char *s, size_t maxlen);

/**
 * @brief Portable strdup implementation for C99
 *
 * Duplicates a string by allocating memory and copying the contents.
 * This implementation is provided for systems that do not have strdup
 * in their standard library (C99 compliance) or for embedded systems
 * with limited libc support.
 *
 * @param[in] s String to duplicate (may be NULL)
 *
 * @return Pointer to newly allocated copy of string
 *         NULL if s is NULL, memory allocation fails, or string too long
 *
 * @section Memory_Management
 * The returned pointer must be freed by the caller using free() when
 * no longer needed. Failure to do so will result in memory leaks.
 *
 * @section Safety_Limits
 * To prevent memory exhaustion attacks, strings longer than 1024
 * characters are rejected and NULL is returned.
 *
 * @note Uses memcpy for efficient copying (includes null terminator).
 * @note Thread safety depends on malloc implementation.
 * @see free() to release allocated memory
 */
char* my_strdup(const char *s);

#endif /* COMMON_H_ */

