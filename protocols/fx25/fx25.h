/**
 * @file fx25.h
 * @brief FX.25 Forward Error Correction Extension for AX.25 - Protocol Interface
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @section Overview
 * This header defines the complete FX.25 (Forward Error Correction for AX.25)
 * protocol interface. FX.25 is an extension to the AX.25 Link Access Protocol
 * that adds Reed-Solomon forward error correction while maintaining full
 * backward compatibility with legacy AX.25 equipment.
 *
 * FX.25 was developed by the Stensat Group (http://www.stensat.org) and first
 * published in 2006. The protocol enables reliable packet transmission over
 * degraded radio links by correcting byte-level errors without requiring
 * retransmission, making it particularly valuable for APRS and unidirectional
 * applications.
 *
 * @section Protocol_Structure
 * An FX.25 frame consists of three components:
 * - 8-byte Correlation Tag: Identifies the FEC mode and provides frame sync
 * - AX.25 Frame: Complete standard AX.25 frame (including flags, FCS, etc.)
 * - Reed-Solomon Parity: 16, 32, or 64 bytes for error correction
 *
 * The protocol uses shortened Reed-Solomon codes over GF(2^8) with primitive
 * polynomial x^8 + x^4 + x^3 + x^2 + 1 (0x11D). The correlation tags are
 * specifically chosen 64-bit patterns with high Hamming distance to ensure
 * reliable mode detection even with bit errors.
 *
 * @section Backward_Compatibility
 * FX.25 maintains 100% backward compatibility with standard AX.25:
 * - Non-FX.25 decoders see the correlation tag as noise preceding the flag
 * - The AX.25 frame is transmitted unmodified with standard HDLC framing
 * - Legacy equipment can ignore FEC parity as post-frame noise
 * - No changes required to existing AX.25 protocol stacks
 *
 * @section Error_Correction_Capability
 * Depending on the selected mode, FX.25 can correct:
 * - Up to 8 byte errors (16 parity bytes)
 * - Up to 16 byte errors (32 parity bytes)
 * - Up to 32 byte errors (64 parity bytes)
 *
 * The correlation tag itself can tolerate up to 6 bit errors while still
 * being correctly identified per FX.25 v01.06 Section 2.2.
 *
 * @section Standards_References
 * - FX.25 v01.06 Specification: "AX.25 + FEC = FX.25" (Stensat Group, 2006)
 * - Reed-Solomon Coding: ISO/IEC 646, CCSDS 101.0-B-6
 * - AX.25 Base Protocol: AX.25 Link Access Protocol v2.2 (July 1998)
 * - HDLC Framing: ISO 3309, ITU-T X.25
 *
 * @section Implementation_Notes
 * This implementation uses:
 * - GF(2^8) arithmetic with precomputed log/antilog tables
 * - Shortened Reed-Solomon codes (RS(255, 255-2T) shortened to RS(n, k))
 * - Hamming distance calculation for correlation tag matching
 * - Automatic mode selection based on frame size and channel quality
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef FX25_H_
#define FX25_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>

/*============================================================================*/
/* Protocol Version and Constants                                             */
/*============================================================================*/

/**
 * @defgroup FX25Constants FX.25 Protocol Constants
 * @brief Fundamental constants defining FX.25 protocol limits and capabilities
 *
 * FX.25 operates on a maximum codeword length of 255 bytes (RS symbol size).
 * The correlation tag is always 8 bytes (64 bits) regardless of mode.
 * Parity bytes must be even and are typically 16, 32, or 64.
 */

/**
 * @brief FX.25 protocol version identifier
 * @details Matches specification document version 01.06
 */
#define FX25_VERSION        0x0106

/**
 * @brief Length of correlation tag in bytes
 * @details Fixed 8-byte (64-bit) synchronization pattern per Section 2.1
 */
#define FX25_TAG_LENGTH     8

/**
 * @brief Maximum Reed-Solomon codeword length in bytes
 * @details RS(255, k) over GF(2^8) - one byte per symbol
 */
#define FX25_MAX_CODEWORD   255

/**
 * @brief Maximum tolerable bit errors in correlation tag
 * @details Per FX.25 v01.06 Section 2.2: up to 6 bit errors allowed
 */
#define FX25_TAG_ERROR_TOLERANCE  6

/**
 * @brief Reed-Solomon primitive polynomial for GF(2^8)
 * @details x^8 + x^4 + x^3 + x^2 + 1 = 0x11D
 * @note This is the CCSDS/NASA standard primitive polynomial
 */
#define FX25_RS_PRIMITIVE_POLY  0x11D

/*============================================================================*/
/* FX.25 Mode Definitions                                                     */
/*============================================================================*/

/**
 * @defgroup FX25Modes FX.25 Operating Modes
 * @brief Predefined correlation tags and coding parameters
 *
 * FX.25 defines 11 standard modes with varying data capacity and error
 * correction strength. Each mode is identified by a unique 8-byte
 * correlation tag and specifies:
 * - Data bytes (D): Maximum AX.25 frame size supported
 * - Parity bytes (P): Reed-Solomon parity symbols appended
 * - Correctable bytes (T): Maximum correctable byte errors (T = P/2)
 *
 * Mode selection balances overhead against error correction capability.
 * Larger parity provides stronger correction but increases transmission
 * time and reduces effective throughput.
 *
 * @section Mode_Selection_Strategy
 * - Short frames (≤32 bytes): Use 32-byte data modes for efficiency
 * - Medium frames (33-128 bytes): Use 64 or 128-byte data modes
 * - Long frames (129-239 bytes): Use 191, 223, or 239-byte data modes
 * - Poor channels: Increase parity bytes for stronger correction
 * - Good channels: Minimize parity to reduce overhead
 */

/**
 * @brief Mode 0x01: 239 data bytes, 16 parity bytes, corrects 8 errors
 * @details Best for long AX.25 frames up to 239 bytes
 *          Lowest overhead (6.7% parity) for large frames
 *          Correlation tag: 0xB74DB7DF8A532F3E
 */
#define FX25_MODE_239_16    0x01

/**
 * @brief Mode 0x02: 128 data bytes, 16 parity bytes, corrects 8 errors
 * @details Optimized for medium AX.25 frames up to 128 bytes
 *          Moderate overhead (12.5% parity)
 *          Correlation tag: 0x26FF60A600CC8FDE
 */
#define FX25_MODE_128_16    0x02

/**
 * @brief Mode 0x03: 64 data bytes, 16 parity bytes, corrects 8 errors
 * @details Suitable for short AX.25 frames up to 64 bytes
 *          Higher overhead (25% parity) but good protection
 *          Correlation tag: 0xC7DC0508F3D9B09E
 */
#define FX25_MODE_64_16     0x03

/**
 * @brief Mode 0x04: 32 data bytes, 16 parity bytes, corrects 8 errors
 * @details For very short AX.25 frames up to 32 bytes
 *          High overhead (50% parity) but maximum protection ratio
 *          Correlation tag: 0x8F056EB4369660EE
 */
#define FX25_MODE_32_16     0x04

/**
 * @brief Mode 0x05: 223 data bytes, 32 parity bytes, corrects 16 errors
 * @details Extended protection for long frames up to 223 bytes
 *          Moderate overhead (14.3% parity), corrects up to 16 errors
 *          Correlation tag: 0x6E260B1AC5835FAE
 */
#define FX25_MODE_223_32    0x05

/**
 * @brief Mode 0x06: 128 data bytes, 32 parity bytes, corrects 16 errors
 * @details Strong protection for medium frames up to 128 bytes
 *          Higher overhead (25% parity) but corrects 16 errors
 *          Correlation tag: 0xFF94DC634F1CFF4E
 */
#define FX25_MODE_128_32    0x06

/**
 * @brief Mode 0x07: 64 data bytes, 32 parity bytes, corrects 16 errors
 * @details Robust protection for short frames up to 64 bytes
 *          Significant overhead (50% parity) for poor channel conditions
 *          Correlation tag: 0x1EB7B9CDBC09C00E
 */
#define FX25_MODE_64_32     0x07

/**
 * @brief Mode 0x08: 32 data bytes, 32 parity bytes, corrects 16 errors
 * @details Maximum protection for very short frames up to 32 bytes
 *          100% overhead (equal data and parity) for critical data
 *          Correlation tag: 0xDBF869BD2DBB1776
 */
#define FX25_MODE_32_32     0x08

/**
 * @brief Mode 0x09: 191 data bytes, 64 parity bytes, corrects 32 errors
 * @details Maximum protection for medium-long frames up to 191 bytes
 *          High overhead (33.5% parity) but corrects up to 32 errors
 *          Correlation tag: 0x3ADB0C13DEAE2836
 */
#define FX25_MODE_191_64    0x09

/**
 * @brief Mode 0x0A: 128 data bytes, 64 parity bytes, corrects 32 errors
 * @details Extreme protection for medium frames up to 128 bytes
 *          Very high overhead (50% parity) for severely degraded channels
 *          Correlation tag: 0xAB69DB6A543188D6
 */
#define FX25_MODE_128_64    0x0A

/**
 * @brief Mode 0x0B: 64 data bytes, 64 parity bytes, corrects 32 errors
 * @details Maximum error correction capability for short frames
 *          100% overhead for maximum reliability in worst conditions
 *          Correlation tag: 0x4A4ABEC4A724B796
 */
#define FX25_MODE_64_64     0x0B

/*============================================================================*/
/* FX.25 Mode Information Structure                                           */
/*============================================================================*/

/**
 * @brief FX.25 mode descriptor structure
 * @details Contains all parameters defining an FX.25 operating mode.
 *          The correlation tag is stored as a byte array in big-endian
 *          order to avoid 64-bit alignment issues on embedded platforms.
 *
 * @section Structure_Layout
 * The correlation tags are specifically chosen to have:
 * - High Hamming distance between different modes (prevents misidentification)
 * - Low autocorrelation sidelobes (prevents false sync on data)
 * - No resemblance to AX.25 flag pattern 0x7E (prevents ambiguity)
 *
 * @note The tag_id field corresponds to the mode identifier (0x01-0x0B)
 * @note correctable_bytes is always half of parity_bytes (T = P/2)
 */
typedef struct {
    uint8_t tag_id;              /**< Mode identifier (1-11, corresponding to 0x01-0x0B) */
    uint8_t correlation_tag[8];  /**< 8-byte (64-bit) correlation/synchronization tag */
    uint8_t data_bytes;          /**< D: Maximum AX.25 frame payload size (32-239 bytes) */
    uint8_t parity_bytes;        /**< P: Reed-Solomon parity symbols (16, 32, or 64) */
    uint8_t correctable_bytes;   /**< T: Maximum correctable byte errors (P/2, i.e., 8, 16, or 32) */
} fx25_mode_t;

/*============================================================================*/
/* FX.25 Frame Structure                                                      */
/*============================================================================*/

/**
 * @brief FX.25 encoded frame structure
 * @details Represents a complete FX.25 frame ready for transmission or
 *          received from the channel. The structure contains the correlation
 *          tag, the Reed-Solomon codeword (data + parity), and metadata.
 *
 * @section Memory_Management
 * The rs_codeword field is dynamically allocated to accommodate variable
 * frame sizes. Callers must:
 * - Allocate the structure or declare as automatic variable
 * - Use fx25_frame_free() to release dynamically allocated memory
 * - Not assume fixed buffer sizes (use codeword_len for bounds)
 *
 * @section Frame_Composition
 * On-air format: [Correlation Tag: 8 bytes][Data: D bytes][Parity: P bytes]
 * Total length: 8 + D + P bytes (where D + P ≤ 255)
 */
typedef struct {
    uint8_t correlation_tag[8];  /**< 8-byte correlation tag copied from mode table */
    uint8_t *rs_codeword;       /**< Reed-Solomon codeword buffer (data + parity) */
    size_t codeword_len;        /**< Total codeword length in bytes (D + P) */
    uint8_t mode_id;            /**< Selected mode identifier (FX25_MODE_*) */
} fx25_frame_t;

/*============================================================================*/
/* Mode Information and Selection                                             */
/*============================================================================*/

/**
 * @brief Retrieve FX.25 mode descriptor by identifier
 * @details Looks up the mode table and returns a pointer to the mode
 *          structure containing correlation tag, data size, and parity
 *          parameters.
 *
 * @param[in] mode_id Mode identifier (FX25_MODE_* constants, 0x01-0x0B)
 * @return Pointer to const fx25_mode_t structure, or NULL if mode_id invalid
 *
 * @section Usage
 * Used internally by encode/decode functions but exposed for applications
 * that need to inspect mode parameters or validate mode selection.
 *
 * @note Returns NULL for mode_id 0x00 (terminator) or undefined modes
 * @note Returned pointer points to static const data (not allocated)
 */
const fx25_mode_t* fx25_get_mode(uint8_t mode_id);

/**
 * @brief Automatically select optimal FX.25 mode for frame size
 * @details Selects the smallest mode (minimum overhead) that can
 *          accommodate the specified AX.25 frame length.
 *
 * @section Selection_Algorithm
 * - If ax25_len ≤ 32:  returns FX25_MODE_32_16 (smallest overhead)
 * - If ax25_len ≤ 64:  returns FX25_MODE_64_16
 * - If ax25_len ≤ 128: returns FX25_MODE_128_16
 * - If ax25_len ≤ 191: returns FX25_MODE_191_64 (best protection medium)
 * - If ax25_len ≤ 223: returns FX25_MODE_223_32
 * - If ax25_len ≤ 239: returns FX25_MODE_239_16 (largest standard frame)
 * - Otherwise: returns 0 (frame too large for FX.25 single block)
 *
 * @param[in] ax25_len Length of AX.25 frame to be encoded (bytes)
 * @return Mode identifier (FX25_MODE_*) or 0 if frame too large
 *
 * @note This function prioritizes minimal overhead over error protection
 * @note For better protection, use fx25_select_mode_for_conditions()
 */
uint8_t fx25_select_mode(size_t ax25_len);

/**
 * @brief Select optimal FX.25 mode based on channel conditions
 * @details Intelligent mode selection considering both frame size and
 *          estimated channel quality to balance overhead against reliability.
 *
 * @section Channel_Quality_Interpretation
 * Channel quality is specified as 0-100:
 * - 0-29% (Poor): Use maximum parity (64 bytes) for robust correction
 * - 30-69% (Medium): Use moderate parity (32 bytes) for balanced protection
 * - 70-100% (Good): Use minimum parity (16 bytes) for efficiency
 *
 * @section Mode_Selection_Logic
 * For each quality tier, selects the smallest data block that fits the
 * frame while providing the specified parity level.
 *
 * @param[in] ax25_len Length of AX.25 frame to be encoded (bytes)
 * @param[in] channel_quality Estimated channel quality (0-100, 100=perfect)
 * @return Mode identifier (FX25_MODE_*) or 0 if frame too large
 *
 * @note Channel quality > 100 is clamped to 100
 * @note For APRS/beacon use, recommend quality < 50 due to unidirectional nature
 * @note Connected mode may use higher quality thresholds due to ARQ fallback
 */
uint8_t fx25_select_mode_for_conditions(size_t ax25_len, uint8_t channel_quality);

/*============================================================================*/
/* Encoding and Decoding Functions                                            */
/*============================================================================*/

/**
 * @brief Encode an AX.25 frame with FX.25 forward error correction
 * @details Wraps a standard AX.25 frame with FX.25 error correction.
 *          The AX.25 frame (including flags, addresses, control, info, FCS)
 *          is treated as the data payload. Reed-Solomon parity is computed
 *          and appended to form the complete FX.25 transmission unit.
 *
 * @section Encoding_Process
 * 1. Validate input parameters and mode selection
 * 2. Retrieve mode parameters (correlation tag, data size, parity size)
 * 3. Verify AX.25 frame fits within selected mode's data capacity
 * 4. Allocate Reed-Solomon codeword buffer (D + P bytes)
 * 5. Copy AX.25 frame data into codeword buffer
 * 6. Compute shortened RS parity over GF(2^8)
 * 7. Store correlation tag in frame structure
 *
 * @section Shortened_RS_Codes
 * FX.25 uses shortened Reed-Solomon codes where:
 * - Full code: RS(255, 255-2T) over GF(2^8)
 * - Shortened: RS(D+P, D) where D+P < 255
 * - Implementation pads message with leading zeros to full length
 * - Only D data bytes and P parity bytes are transmitted
 *
 * @param[in]  ax25_frame Pointer to AX.25 frame data (complete HDLC frame)
 * @param[in]  ax25_len   Length of AX.25 frame in bytes (must be ≤ mode data_bytes)
 * @param[in]  mode_id    FX.25 mode identifier (FX25_MODE_*)
 * @param[out] fx25_frame Pointer to FX.25 frame structure to populate
 * @return Error code: 0=success, 1=invalid parameters, 2=invalid mode,
 *         3=frame too large for mode, 4=memory allocation failed
 *
 * @note fx25_frame->rs_codeword is dynamically allocated (caller must free via fx25_frame_free)
 * @note The AX.25 frame is copied into the codeword; original buffer not modified
 * @note Correlation tag is copied from mode table to fx25_frame structure
 */
uint8_t fx25_encode(const uint8_t *ax25_frame, size_t ax25_len, uint8_t mode_id, fx25_frame_t *fx25_frame);

/**
 * @brief Decode and error-correct an FX.25 frame
 * @details Processes received FX.25 data, identifies the mode via correlation
 *          tag matching, performs Reed-Solomon decoding with error correction,
 *          and extracts the original AX.25 frame.
 *
 * @section Decoding_Process
 * 1. Validate input buffer meets minimum size requirements
 * 2. Extract correlation tag from first 8 bytes of received data
 * 3. Match tag against known modes using Hamming distance (≤6 errors tolerated)
 * 4. Reconstruct shortened RS codeword with leading zero padding
 * 5. Perform RS decoding (Berlekamp-Massey algorithm for error location)
 * 6. If correctable: return corrected AX.25 frame
 * 7. If uncorrectable: return error status with corrected_errors = 0xFF
 *
 * @section Correlation_Tag_Matching
 * The received tag is compared against all defined modes. A match is
 * declared if Hamming distance ≤ FX25_TAG_ERROR_TOLERANCE (6 bits).
 * The closest match is selected if multiple modes are within tolerance.
 *
 * @param[in]  rx_data          Received data buffer (tag + codeword)
 * @param[in]  rx_len           Length of received data in bytes
 * @param[out] fx25_frame       Decoded FX.25 frame structure
 * @param[out] corrected_errors Number of byte errors corrected (0=none, 0xFF=uncorrectable)
 * @return Error code: 0=success, 1=invalid parameters, 2=buffer too short,
 *         3=no matching correlation tag, 4=buffer shorter than expected,
 *         5=memory allocation failed, 6=uncorrectable errors detected
 *
 * @note Successful decode populates fx25_frame with allocated rs_codeword
 * @note Corrected AX.25 frame is in rs_codeword[0 .. mode.data_bytes-1]
 * @note corrected_errors set to 0xFF indicates uncorrectable error pattern
 * @note Even with uncorrectable errors, partial correction may have occurred
 */
uint8_t fx25_decode(const uint8_t *rx_data, size_t rx_len, fx25_frame_t *fx25_frame, uint8_t *corrected_errors);

/**
 * @brief Release FX.25 frame resources
 * @details Frees dynamically allocated memory associated with an FX.25
 *          frame structure. Safe to call on partially initialized structures.
 *
 * @param[in,out] frame Pointer to FX.25 frame structure
 *
 * @section Safety
 * - Sets rs_codeword pointer to NULL after freeing
 * - Safe to call with NULL frame pointer (no operation)
 * - Safe to call multiple times (idempotent after first call)
 * - Does not free the fx25_frame_t structure itself (caller allocated)
 *
 * @note Caller remains responsible for freeing the fx25_frame_t structure
 * @note Must be called to prevent memory leaks after fx25_encode() or fx25_decode()
 */
void fx25_frame_free(fx25_frame_t *frame);

#endif /* FX25_H_ */
