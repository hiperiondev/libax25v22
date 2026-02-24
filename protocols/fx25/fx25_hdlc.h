/**
 * @file fx25_hdlc.h
 * @brief FX.25 Forward Error Correction Wrapper for AX.25/HDLC Frames
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @section Overview
 * This header defines the interface for encapsulating standard AX.25 frames
 * within FX.25 forward error correction (FEC) structures. FX.25 is an
 * extension to AX.25 that adds Reed-Solomon error correction while maintaining
 * full backward compatibility with legacy AX.25 equipment.
 *
 * @section Protocol_Description
 * FX.25 (Frame Extension 25) was developed by the Stensat Group around 2009
 * and documented in the FX.25 specification v01.06. It addresses the
 * vulnerability of standard AX.25 frames where a single bit error causes
 * complete frame loss due to FCS failure.
 *
 * The FX.25 frame structure consists of:
 * - Preamble: 32-bit alternating 0x55 pattern for clock recovery
 * - Correlation Tag: 8-byte unique identifier determining RS parameters
 * - RS Codeword: Data portion (AX.25 HDLC frame) + Parity bytes
 * - Postamble: 32-bit alternating pattern for receiver synchronization
 *
 * @section Correlation_Tags
 * Eleven predefined 64-bit correlation tags identify the Reed-Solomon
 * configuration. Each tag specifies:
 * - Data bytes (D): 32 to 239 bytes capacity for AX.25 frame
 * - Parity bytes (P): 16, 32, or 64 bytes (RS(255,239), RS(255,223), etc.)
 * - Error correction capability: Up to P/2 byte errors correctable
 *
 * Example tag values (full list in fx25.h):
 * - Tag 0x01: 0xB74DB7DF8A532F3E, 239 data bytes, 16 parity bytes (8 errors)
 * - Tag 0x05: 0x6E260B1AC5835FAE, 223 data bytes, 32 parity bytes (16 errors)
 * - Tag 0x09: 0x3ADB0C13DEAE2836, 191 data bytes, 64 parity bytes (32 errors)
 *
 * @section Encoding_Process
 * 1. AX.25 frame is first wrapped in HDLC structure (flags, bit stuffing, FCS)
 * 2. HDLC frame is treated as raw data bytes for Reed-Solomon encoding
 * 3. Correlation tag is prepended to identify the RS parameters
 * 4. RS parity bytes are appended to create systematic codeword
 * 5. Preamble and postamble are added for physical layer synchronization
 *
 * @section Decoding_Process
 * 1. Receiver scans incoming bitstream for valid 64-bit correlation tags
 * 2. Upon tag detection, D+P bytes are extracted as RS codeword
 * 3. Reed-Solomon decoder computes syndrome and corrects up to t errors
 * 4. Corrected data portion is searched for HDLC start flag (0x7E)
 * 5. HDLC decoder extracts original AX.25 frame from RS data field
 * 6. AX.25 FCS is verified to ensure error-free delivery
 *
 * @section Backward_Compatibility
 * Legacy AX.25 receivers without FX.25 capability will see the correlation
 * tag as noise/preamble and synchronize on the embedded HDLC flags. The
 * RS parity bytes appear as trailing noise after the closing flag. This
 * allows mixed networks where only FX.25-capable stations benefit from FEC.
 *
 * @section Implementation_Notes
 * - Uses shortened Reed-Solomon codes over GF(2^8)
 * - Primitive polynomial: x^8 + x^4 + x^3 + x^2 + 1  (= 0x11D)
 *   Note: the form x^8+x^7+x^2+x+1 would be 0x187, which is incorrect here.
 * - Maximum frame size: 255 bytes total (data + parity)
 * - Supports 11 distinct operating modes via correlation tags
 * - Auto-mode selection based on frame size and channel quality
 *
 * @section Standards_References
 * - FX.25 Specification v01.06: Forward Error Correction Extension to AX.25
 *   https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 * - AX.25 Link Access Protocol for Amateur Packet Radio, Version 2.2
 *   https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * - Reed-Solomon Error Correction (IEEE 802.16, DVB-T standards)
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef FX25_HDLC_H_
#define FX25_HDLC_H_

#include <stdint.h>
#include <stddef.h>

/*============================================================================*/
/* FX.25 Physical Layer Constants                                             */
/*============================================================================*/

/**
 * @defgroup FX25PhysicalConstants FX.25 Physical Layer Timing
 * @brief Default timing parameters for FX.25 frame synchronization
 *
 * The preamble and postamble provide bit synchronization for the receiver's
 * clock recovery circuit. The alternating 0x55 pattern (01010101) produces
 * a maximum transition density clock signal.
 *
 * @section Preamble_Purpose
 * - Allows receiver to lock onto bit timing before critical data arrives
 * - Provides distinct pattern that doesn't resemble correlation tags
 * - Minimum duration ensures reliable clock recovery on noisy channels
 *
 * @section Postamble_Purpose
 * - Maintains bit timing through end of transmission
 * - Prevents premature carrier drop affecting last bytes
 * - Allows receiver to complete processing of final parity bytes
 */
#define FX25_DEFAULT_PREAMBLE_BITS  32  /**< Default preamble length in bits (4 bytes of 0x55) */
#define FX25_DEFAULT_POSTAMBLE_BITS 32  /**< Default postamble length in bits (4 bytes of 0x55) */

/*============================================================================*/
/* FX.25 Mode Selection Constants                                               */
/*============================================================================*/

/**
 * @defgroup FX25ModeSelection FX.25 Mode Selection Values
 * @brief Special values for mode_id parameter in encoding functions
 *
 * These constants control how the FX.25 mode (correlation tag) is selected
 * during the encoding process.
 */
#define FX25_MODE_AUTO 0  /**< Automatic mode selection based on frame size and quality */

/*============================================================================*/
/* Error Return Codes                                                           */
/*============================================================================*/

// Unified error code enum covering both encode and decode directions.
// Previously, both fx25_hdlc_encode and fx25_hdlc_decode returned uint8_t
// with overlapping numeric values (e.g., both used 2, 4, 5, 6) making
// debug logs ambiguous without external context about which function was called.
typedef enum {
    FX25_HDLC_OK = 0,  // Success
    // Shared errors
    FX25_HDLC_ERR_INVALID_PARAM = 1,  // NULL pointer or zero-length input
    // Encode-specific errors
    FX25_HDLC_ERR_HDLC_ENCODE = 2,  // HDLC layer encoding failure
    FX25_HDLC_ERR_AUTO_SELECT = 3,  // Auto mode selection failed (frame too large)
    FX25_HDLC_ERR_BAD_MODE = 4,  // Invalid or unknown FX.25 mode identifier
    FX25_HDLC_ERR_FRAME_LARGE = 5,  // Frame exceeds selected mode capacity
    FX25_HDLC_ERR_RS_ENCODE = 6,  // Reed-Solomon encoding failure
    // Decode-specific errors
    FX25_HDLC_ERR_RS_DECODE = 7,  // FX.25 RS decoding failed (uncorrectable errors)
    FX25_HDLC_ERR_BAD_MODE_ID = 8,  // Invalid mode ID in decoded frame
    FX25_HDLC_ERR_NO_FLAG = 9,  // No HDLC 0x7E start flag found in decoded data
    FX25_HDLC_ERR_HDLC_DECODE = 10,  // HDLC layer decoding failure
    FX25_HDLC_ERR_EMPTY = 11  // Empty frame after decode
} fx25_hdlc_err_t;

// Function signatures updated from uint8_t to fx25_hdlc_err_t return type
// so callers can distinguish encode vs decode errors by name, not just number.
fx25_hdlc_err_t fx25_hdlc_encode(const uint8_t *ax25_frame, size_t ax25_len, uint8_t mode_id, uint8_t channel_quality, uint8_t *output, size_t *output_len);

fx25_hdlc_err_t fx25_hdlc_decode(const uint8_t *rx_data, size_t rx_len, uint8_t *ax25_frame, size_t *ax25_len, uint8_t *corrected_errors);

/*============================================================================*/
/* Channel Quality Indicators                                                   */
/*============================================================================*/

/**
 * @defgroup FX25ChannelQuality FX.25 Channel Quality Indicators
 * @brief Recommended values for channel_quality parameter
 *
 * Channel quality influences automatic mode selection when FX25_MODE_AUTO
 * is specified. Higher quality channels favor larger data payloads with
 * less parity overhead, while poor channels trigger more robust modes.
 *
 * @section Quality_Guidelines
 * - 90-100: Excellent channel (line-of-sight VHF/UHF, minimal noise)
 * - 70-89:  Good channel (typical VHF packet, moderate signal strength)
 * - 50-69:  Fair channel (marginal VHF, noisy environment)
 * - 30-49:  Poor channel (HF conditions, fading, interference)
 * - 0-29:   Very poor channel (severe QSB, high BER, satellite links)
 */
#define FX25_QUALITY_EXCELLENT 100  /**< Line-of-sight, strong signal, minimal noise */
#define FX25_QUALITY_GOOD       80  /**< Typical VHF/UFM packet conditions */
#define FX25_QUALITY_FAIR        60  /**< Marginal conditions, some noise */
#define FX25_QUALITY_POOR        40  /**< HF conditions, fading present */
#define FX25_QUALITY_VERY_POOR   20  /**< Severe conditions, satellite links */

/*============================================================================*/
/* Encoding/Decoding Functions                                                  */
/*============================================================================*/

/**
 * @brief Encode AX.25 frame with FX.25 FEC wrapper
 *
 * Encapsulates a raw AX.25 frame within an FX.25 forward error correction
 * structure. The process involves HDLC encoding, Reed-Solomon parity
 * generation, and framing with correlation tag and synchronization patterns.
 *
 * @section Process_Details
 * 1. **HDLC Encoding**: The input AX.25 frame is wrapped with HDLC flags
 *    (0x7E), bit stuffing is applied, and FCS-16 is calculated and appended.
 *    This creates a standard HDLC frame suitable for NRZI transmission.
 *
 * 2. **Mode Selection**: If mode_id is FX25_MODE_AUTO (0), the function
 *    analyzes the HDLC frame length and channel_quality to select the
 *    optimal correlation tag that balances overhead against error protection.
 *    Otherwise, the specified mode is validated against the frame size.
 *
 * 3. **Reed-Solomon Encoding**: The HDLC frame is treated as the message
 *    polynomial. Systematic RS encoding generates parity bytes that are
 *    appended to create the complete codeword of length n = k + 2t.
 *
 * 4. **Frame Assembly**: The final output consists of:
 *    - Preamble bytes (0x55 pattern)
 *    - 8-byte correlation tag (identifies RS parameters)
 *    - RS codeword (HDLC data + parity bytes)
 *    - Postamble bytes (0x55 pattern)
 *
 * @section Mode_Selection_Logic
 * For automatic mode selection (mode_id = 0):
 * - Frames ≤ 32 bytes: Tag 0x04 or 0x08 (32 data bytes, high protection)
 * - Frames 33-64 bytes: Tag 0x03 or 0x07 (64 data bytes)
 * - Frames 65-128 bytes: Tag 0x02 or 0x06 (128 data bytes)
 * - Frames 129-239 bytes: Tag 0x01 or 0x05 (239/223 data bytes)
 *
 * Channel quality affects parity selection:
 * - Quality < 50: Prefer 32 or 64 parity byte modes (stronger FEC)
 * - Quality ≥ 50: Prefer 16 parity byte modes (lower overhead)
 *
 * @param[in]  ax25_frame      Raw AX.25 frame buffer (address, control, PID, info, FCS)
 * @param[in]  ax25_len        Length of AX.25 frame in bytes (must be > 0)
 * @param[in]  mode_id         FX.25 mode identifier (1-11) or FX25_MODE_AUTO (0)
 * @param[in]  channel_quality Channel condition indicator (0-100), used for auto mode
 * @param[out] output          Output buffer for complete FX.25 frame
 * @param[out] output_len      Pointer to store total output length in bytes
 *
 * @return Error code indicating success or failure type
 * @retval 0 Success - FX.25 frame encoded successfully
 * @retval 1 Invalid parameter - NULL pointer or zero length input
 * @retval 2 HDLC encoding failed - internal HDLC layer error
 * @retval 3 Auto mode selection failed - frame too large for any mode
 * @retval 4 Invalid mode ID - mode not recognized or auto-select returned invalid
 * @retval 5 Frame too large for selected mode - exceeds data_bytes capacity
 * @retval 6 FX.25 encoding failed - Reed-Solomon encoder error
 *
 * @section Buffer_Requirements
 * The output buffer must accommodate:
 * - Preamble: FX25_DEFAULT_PREAMBLE_BITS/8 bytes (typically 4)
 * - Correlation tag: 8 bytes
 * - RS codeword: Up to 255 bytes (depending on mode)
 * - Postamble: FX25_DEFAULT_POSTAMBLE_BITS/8 bytes (typically 4)
 * Maximum total: 4 + 8 + 255 + 4 = 271 bytes
 *
 * @section Example_Usage
 * @code
 * uint8_t ax25_frame[] = {0xA6, 0x92, 0x98, 0x40, 0x40, 0x60, ...}; // AX.25 content
 * size_t ax25_len = sizeof(ax25_frame);
 * uint8_t fx25_output[512];
 * size_t fx25_len;
 *
 * uint8_t err = fx25_hdlc_encode(ax25_frame, ax25_len,
 *                                FX25_MODE_AUTO, FX25_QUALITY_GOOD,
 *                                fx25_output, &fx25_len);
 * if (err == 0) {
 *     // Transmit fx25_output[0..fx25_len-1] over radio interface
 * }
 * @endcode
 *
 * @see fx25_hdlc_decode()
 * @see fx25_select_mode_for_conditions()
 * @see hdlc_frame_encode()
 */
fx25_hdlc_err_t fx25_hdlc_encode(const uint8_t *ax25_frame, size_t ax25_len, uint8_t mode_id, uint8_t channel_quality, uint8_t *output, size_t *output_len);

/**
 * @brief Decode FX.25 frame and extract AX.25 data
 *
 * Processes a received FX.25 frame to recover the original AX.25 payload.
 * Performs Reed-Solomon error correction, HDLC decoding, and validation.
 *
 * @section Process_Details
 * 1. **Preamble/Postamble Removal**: The raw received data is stripped of
 *    synchronization patterns to isolate the FX.25 core structure.
 *
 * 2. **Correlation Tag Detection**: The decoder searches for one of the 11
 *    valid 64-bit correlation tags. Tag detection must be perfect (zero bit
 *    errors) as the tag determines how many bytes to read for the codeword.
 *
 * 3. **Reed-Solomon Decoding**: The D+P bytes following the tag form the
 *    RS codeword. The decoder computes syndromes to detect errors. If errors
 *    are found, the Berlekamp-Massey algorithm locates and corrects up to t
 *    byte errors (where t = P/2).
 *
 * 4. **HDLC Extraction**: The corrected data portion is scanned for the
 *    HDLC start flag (0x7E). This handles cases where the RS data field
 *    contains padding before the actual HDLC frame.
 *
 * 5. **HDLC Decoding**: Bit destuffing reverses the transmission bit stuffing.
 *    The FCS-16 is verified to ensure the AX.25 frame is error-free.
 *
 * @section Error_Correction_Reporting
 * The corrected_errors parameter reports the number of byte errors fixed
 * by the Reed-Solomon decoder:
 * - 0: No errors detected (clean reception)
 * - 1 to t: Successful correction of indicated error count
 * - > t: Decoder failure (too many errors, frame discarded)
 *
 * Note: This reports RS byte corrections, not bit errors. Each corrected
 * byte may represent 1-8 bit errors depending on error distribution.
 *
 * @param[in]  rx_data          Received FX.25 frame buffer (raw demodulated bytes)
 * @param[in]  rx_len           Length of received data in bytes
 * @param[out] ax25_frame       Output buffer for decoded AX.25 frame
 * @param[out] ax25_len         Pointer to store length of decoded AX.25 frame
 * @param[out] corrected_errors Pointer to store number of RS-corrected byte errors
 *
 * @return Error code indicating success or failure type
 * @retval 0 Success - AX.25 frame decoded and FCS verified
 * @retval 1 Invalid parameter - NULL pointer or insufficient data length
 * @retval 2 FX.25 decoding failed - tag not found or uncorrectable RS errors
 * @retval 3 Invalid mode ID - decoded tag corresponds to unknown mode
 * @retval 4 No HDLC flag - start delimiter 0x7E not found in RS data
 * @retval 5 HDLC decoding failed - bit destuffing error or FCS mismatch
 * @retval 6 Empty frame - decoded HDLC frame has zero length payload
 *
 * @section Buffer_Requirements
 * The rx_data buffer should contain at minimum:
 * - Preamble + Correlation tag (8 bytes) + Minimum codeword + Postamble
 * For smallest mode (Tag 0x04): 4 + 8 + 48 + 4 = 64 bytes minimum
 *
 * The ax25_frame output buffer should accommodate maximum AX.25 frame size:
 * - Maximum 330 bytes (256 byte payload + 14 byte address + control + PID + FCS)
 *
 * @section Example_Usage
 * @code
 * uint8_t rx_buffer[512]; // Filled by radio demodulator
 * size_t rx_len = read_from_radio(rx_buffer);
 * uint8_t ax25_output[512];
 * size_t ax25_len;
 * uint8_t errors;
 *
 * uint8_t err = fx25_hdlc_decode(rx_buffer, rx_len,
 *                                ax25_output, &ax25_len, &errors);
 * if (err == 0) {
 *     if (errors > 0) {
 *         printf("Frame corrected with %d byte errors\n", errors);
 *     }
 *     // Process ax25_output[0..ax25_len-1] as standard AX.25 frame
 * }
 * @endcode
 *
 * @section Compatibility_Notes
 * This function handles both FX.25 frames and will fail gracefully if
 * presented with pure AX.25 frames (no correlation tag found). For
 * dual-mode receivers, call this first; if it returns error 2, fall back
 * to standard AX.25/HDLC decoding.
 *
 * @see fx25_hdlc_encode()
 * @see fx25_decode()
 * @see hdlc_frame_decode()
 */
fx25_hdlc_err_t fx25_hdlc_decode(const uint8_t *rx_data, size_t rx_len, uint8_t *ax25_frame, size_t *ax25_len, uint8_t *corrected_errors);

#endif /* FX25_HDLC_H_ */
