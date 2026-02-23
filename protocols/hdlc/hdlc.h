/**
 * @file hdlc.h
 * @brief HDLC (High-Level Data Link Control) Framing for AX.25 v2.2
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @section Overview
 * This header defines the HDLC framing layer for AX.25 v2.2 packet radio
 * communications. HDLC provides the synchronous data link layer framing
 * mechanism that encapsulates AX.25 packets for transmission over radio
 * channels, implementing bit stuffing, frame delimitation, and error
 * detection via Frame Check Sequence (FCS).
 *
 * @section Standards_References
 * - ISO 3309: HDLC frame structure
 * - ISO 4335: HDLC elements of procedures
 * - AX.25 Link Access Protocol for Amateur Packet Radio, Version 2.2 (July 1998)
 *   Section 2.2: Frame Structure, Section 3.6: Bit Stuffing
 * - ITU-T Recommendation X.25: Interface between DTE and DCE for terminals
 *   operating in the packet mode
 *
 * @section HDLC_Frame_Structure
 * HDLC frames used in AX.25 consist of the following elements:
 * - Opening Flag (0x7E): Marks the beginning of the frame
 * - Address Field (14-70 bytes): Destination, source, and optional digipeaters
 * - Control Field (1-2 bytes): Frame type and sequence numbers
 * - PID Field (0-1 byte): Protocol identifier (I and UI frames only)
 * - Information Field (0-N bytes): User data payload
 * - Frame Check Sequence (2 bytes): CRC-CCITT (X.25) for error detection
 * - Closing Flag (0x7E): Marks the end of the frame
 *
 * @section Bit_Stuffing
 * To prevent the flag sequence (0x7E = 01111110) from appearing in the
 * data field, HDLC employs bit stuffing (zero insertion):
 * - Transmitter: After transmitting five consecutive 1-bits, insert a 0-bit
 * - Receiver: After receiving five consecutive 1-bits, discard the following 0-bit
 *
 * This ensures data transparency and frame synchronization integrity.
 * Per AX.25 v2.2 Section 3.6, bit stuffing is applied to all fields
 * between the opening and closing flags.
 *
 * @section Bit_Transmission_Order
 * Per AX.25 v2.2 Section 3.8:
 * - All fields except FCS: Least Significant Bit (LSB) first
 * - FCS field: Most Significant Bit (MSB) first
 *
 * This differs from standard HDLC and must be handled correctly during
 * encoding and decoding operations.
 *
 * @section Frame_Abort
 * Per AX.25 v2.2 Section 3.10, if a frame must be prematurely aborted,
 * at least fifteen contiguous 1-bits shall be transmitted with no bit
 * stuffing applied. This signals the receiver to discard the current frame.
 *
 * @section Invalid_Frames
 * Per AX.25 v2.2 Section 3.9, a frame is considered invalid if:
 * - It consists of less than 136 bits (17 bytes) including flags
 * - It is not bounded by opening and closing flags
 * - It is not octet-aligned (not an integral number of octets)
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef HDLC_H_
#define HDLC_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* HDLC Constants and Flag Definitions                                        */
/*============================================================================*/

/**
 * @defgroup HDLC_Flags HDLC Frame Delimiters and Constants
 * @brief Standard HDLC flag patterns and control constants per ISO 3309
 *
 * The HDLC flag byte (0x7E) serves as both the opening and closing delimiter
 * for frames. Between consecutive frames, a single flag may serve as both
 * the closing flag of one frame and the opening flag of the next.
 *
 * The flag pattern 01111110 was chosen because it:
 * - Contains six consecutive 1-bits (maximum allowed in stuffed data is five)
 * - Has 0-bit boundaries to prevent false synchronization
 * - Is unique enough to serve as a frame delimiter
 */
#define HDLC_FLAG_BYTE      0x7E    /**< HDLC flag: 01111110 - Frame delimiter */
#define HDLC_FLAG_PATTERN   0x7E    /**< Alias for HDLC_FLAG_BYTE */
#define HDLC_ABORT_BYTE     0xFF    /**< Abort sequence byte: 11111111 */
#define HDLC_ESCAPE_BYTE    0x7D    /**< Escape byte for KISS protocol (TNC) */

/**
 * @defgroup Bit_Stuffing_Parameters Bit Stuffing Configuration
 * @brief Parameters controlling the bit stuffing algorithm
 *
 * Bit stuffing ensures the flag pattern does not appear in the data.
 * After HDLC_STUFF_THRESHOLD (5) consecutive 1-bits, a 0-bit is inserted.
 */
#define HDLC_STUFF_THRESHOLD    5   /**< Number of consecutive 1s before stuffing */
#define HDLC_STUFF_INSERT_BIT   0   /**< Bit value inserted after threshold */
#define HDLC_ABORT_MIN_ONES     15  /**< Minimum consecutive 1s for abort sequence */

/*============================================================================*/
/* HDLC Error Codes                                                           */
/*============================================================================*/

/**
 * @brief HDLC processing error codes
 *
 * Enumeration of possible error conditions that can occur during HDLC
 * frame encoding, decoding, or validation operations.
 *
 * @section Error_Handling_Strategy
 * - HDLC_OK: Operation completed successfully
 * - HDLC_ERR_NO_START_FLAG: Frame does not begin with 0x7E
 * - HDLC_ERR_NO_END_FLAG: Frame does not end with 0x7E
 * - HDLC_ERR_TOO_SHORT: Frame shorter than minimum valid length
 * - HDLC_ERR_CRC_FAIL: Frame Check Sequence verification failed
 * - HDLC_ERR_STUFFING: Bit stuffing violation detected (6+ consecutive 1s)
 * - HDLC_ERR_ABORT: Abort sequence detected during reception
 */
typedef enum HDLC_ERROR_E {
    HDLC_OK = 0, /**< Operation completed successfully */
    HDLC_ERR_NO_START_FLAG, /**< Missing opening flag (0x7E) */
    HDLC_ERR_NO_END_FLAG, /**< Missing closing flag (0x7E) */
    HDLC_ERR_TOO_SHORT, /**< Frame shorter than 136 bits (17 bytes) */
    HDLC_ERR_CRC_FAIL, /**< Frame Check Sequence mismatch */
    HDLC_ERR_STUFFING, /**< Bit stuffing violation detected */
    HDLC_ERR_ABORT /**< Abort sequence (15+ consecutive 1s) */
} hdlc_error_t;

/*============================================================================*/
/* Bit Manipulation Macros                                                    */
/*============================================================================*/

/**
 * @defgroup Bit_Manipulation Bit-Level Operations
 * @brief Macros for bit extraction and manipulation during HDLC processing
 *
 * These macros support the bit-oriented nature of HDLC processing where
 * data must be handled at the bit level for proper stuffing and destuffing.
 */

/**
 * @brief Extract a specific bit from a byte
 *
 * @param[in] byte_val The source byte
 * @param[in] bit_pos Bit position to extract (0=LSB, 7=MSB)
 * @return The bit value (0 or 1) at the specified position
 */
#define GET_BIT(byte_val, bit_pos) (((byte_val) >> (bit_pos)) & 0x01)

/**
 * @brief Set a specific bit in a byte
 *
 * @param[in,out] byte_val The target byte
 * @param[in] bit_pos Bit position to set (0=LSB, 7=MSB)
 */
#define SET_BIT(byte_val, bit_pos) ((byte_val) |= (1 << (bit_pos)))

/**
 * @brief Clear a specific bit in a byte
 *
 * @param[in,out] byte_val The target byte
 * @param[in] bit_pos Bit position to clear (0=LSB, 7=MSB)
 */
#define CLEAR_BIT(byte_val, bit_pos) ((byte_val) &= ~(1 << (bit_pos)))

/**
 * @brief Helper macro to output a single bit with proper HDLC bit stuffing
 *
 * HDLC requires that after 5 consecutive 1-bits, a 0-bit is inserted
 * to prevent the flag sequence (0x7E = 01111110) from appearing in data.
 *
 * This macro:
 * 1. Outputs the current data bit
 * 2. Tracks consecutive 1-bits
 * 3. Inserts a stuffing 0-bit after 5 consecutive 1-bits
 * 4. Handles byte boundary transitions correctly
 * uses maxEncodedLen and overflow local variables
 * that must be declared in the enclosing function (hdlc_frame_encode).
 * Sets overflow=1 and aborts the macro body if the output buffer is full.
 * Callers must declare: int maxEncodedLen = hdlc_encoded_size_max(frameLen);
 *                       int overflow = 0;
 * and check 'overflow' after each loop that calls OUTPUT_BIT.
 */
#define OUTPUT_BIT(bit_val) do { \
    if (overflow) break; \
    if ((bit_val)) { \
        byte |= (1u << bitIndex); \
        cnt++; \
        if (cnt == 5) { \
            bitIndex++; \
            if (bitIndex > 7) { \
                if (encodedIndex >= maxEncodedLen) { overflow = 1; break; } \
                encodedFrame[encodedIndex++] = byte; \
                byte = 0; \
                bitIndex = 0; \
            } \
            cnt = 0; \
        } \
    } else { \
        cnt = 0; \
    } \
    bitIndex++; \
    if (bitIndex > 7) { \
        if (encodedIndex >= maxEncodedLen) { overflow = 1; break; } \
        encodedFrame[encodedIndex++] = byte; \
        byte = 0; \
        bitIndex = 0; \
    } \
} while(0)

// Returns the minimum output buffer size required by hdlc_frame_encode
// for a raw input frame of frameLen bytes (before CRC or bit stuffing).
// Formula: worst-case bit-stuffed (frameLen+2 CRC bytes)*9 bits / 8, rounded
// up to bytes, plus 2 flag bytes.
static inline int hdlc_encoded_size_max(int frameLen) {
    return (((frameLen + 2) * 9 + 7) / 8) + 2;
}

/*============================================================================*/
/* HDLC Encoding/Decoding Functions                                           */
/*============================================================================*/

/**
 * @brief Encode an AX.25 frame into HDLC format for transmission
 *
 * Transforms a raw AX.25 frame into an HDLC-encoded frame suitable for
 * radio transmission. The encoding process includes:
 *
 * @section Encoding_Steps
 * 1. Calculate 16-bit CRC-CCITT (X.25) Frame Check Sequence (FCS)
 * 2. Append FCS to frame (low byte first, consistent with AX.25 bit order)
 * 3. Perform bit stuffing: insert 0-bit after every 5 consecutive 1-bits
 * 4. Add opening flag (0x7E) at beginning
 * 5. Add closing flag (0x7E) at end
 *
 * @section Bit_Order_Considerations
 * - Address, Control, PID, and Information fields: LSB-first per byte
 * - FCS field: Transmitted LSB-first for low byte, then LSB-first for high byte
 *   (resulting in MSB-first overall for the 16-bit FCS value)
 *
 * @param[in]  frame          Pointer to raw AX.25 frame data (without FCS or flags)
 * @param[in]  frameLen       Length of input frame in bytes (excluding FCS)
 * @param[out] encodedFrame   Output buffer for HDLC-encoded frame
 * @param[out] encodedLen     Pointer to store length of encoded frame in bytes
 *
 * @pre frame must not be NULL
 * @pre encodedFrame must have sufficient capacity (recommend 1.25x frameLen + 4)
 * @pre frameLen must be >= 15 (minimum AX.25 frame size without FCS)
 *
 * @post encodedLen contains total bytes including flags and stuffed bits
 * @post encodedFrame contains valid HDLC frame ready for transmission
 *
 * @note The output buffer must be large enough to accommodate worst-case
 *       bit stuffing (every 5 bits could expand by 20%) plus 2 flag bytes.
 *       Recommended size: (frameLen + 2) * 1.25 + 2
 *
 * @warning The input frame buffer is not modified by this function.
 *          The FCS is calculated on the original data and appended
 *          during encoding.
 *
 * @see hdlc_frame_decode()
 * @see CRC() (defined in common.h)
 */
void hdlc_frame_encode(unsigned char *frame, int frameLen, unsigned char *encodedFrame, int *encodedLen);

/**
 * @brief Decode an HDLC-encoded frame back to AX.25 format
 *
 * Reverses the HDLC encoding process to extract the original AX.25 frame
 * from a received HDLC frame. The decoding process includes:
 *
 * @section Decoding_Steps
 * 1. Verify opening flag (0x7E) is present
 * 2. Process bit stream LSB-first, removing stuffed bits (0 after 5 consecutive 1s)
 * 3. Detect closing flag (0x7E) to determine frame end
 * 4. Extract FCS (last 2 bytes of destuffed data)
 * 5. Verify FCS against calculated CRC of frame content
 * 6. Return decoded frame without FCS
 *
 * @section Error_Detection
 * The function detects and reports:
 * - Missing start/end flags
 * - Bit stuffing violations (6 consecutive 1s in data)
 * - Abort sequences (7+ consecutive 1s)
 * - CRC mismatches indicating corrupted data
 * - Frames shorter than minimum valid length
 *
 * @param[in]  encodedFrame   Pointer to HDLC-encoded frame data with flags
 * @param[in]  encodedLen     Length of encoded frame in bytes
 * @param[out] decodedFrame   Output buffer for decoded AX.25 frame (no FCS)
 * @param[out] decodedLen     Pointer to store length of decoded frame (no FCS)
 *
 * @return hdlc_error_t indicating success or specific error condition
 * @retval HDLC_OK Frame successfully decoded and CRC verified
 * @retval HDLC_ERR_NO_START_FLAG Frame does not begin with 0x7E
 * @retval HDLC_ERR_NO_END_FLAG Frame does not end with 0x7E
 * @retval HDLC_ERR_TOO_SHORT Frame shorter than minimum 136 bits
 * @retval HDLC_ERR_CRC_FAIL FCS verification failed (corrupted frame)
 * @retval HDLC_ERR_STUFFING Bit stuffing violation detected
 * @retval HDLC_ERR_ABORT Abort sequence detected in frame
 *
 * @pre encodedFrame must not be NULL
 * @pre decodedFrame must have capacity for at least encodedLen bytes
 * @pre encodedLen must be >= 2 (minimum for two flag bytes)
 *
 * @post decodedLen contains length of frame without FCS (if HDLC_OK)
 * @post decodedFrame contains raw AX.25 frame ready for protocol parsing
 *
 * @note The decoded frame length will always be less than or equal to
 *       encodedLen minus 4 (accounting for flags and FCS).
 *
 * @warning If return value is not HDLC_OK, contents of decodedFrame
 *          and decodedLen are undefined.
 *
 * @see hdlc_frame_encode()
 * @see CRC() (defined in common.h)
 */
hdlc_error_t hdlc_frame_decode(unsigned char *encodedFrame, int encodedLen, unsigned char *decodedFrame, int *decodedLen);

/**
 * @brief Generate an HDLC abort sequence
 *
 * Creates a frame abort sequence per AX.25 v2.2 Section 3.10 and
 * HDLC specification. When transmitted, this sequence causes the
 * receiver to immediately discard the current frame being received.
 *
 * @section Abort_Specification
 * Per AX.25 v2.2 Section 3.10:
 * "If a frame must be prematurely aborted, at least fifteen contiguous
 * ones shall be sent with no bit stuffing added."
 *
 * This implementation generates 16 consecutive 1-bits (2 bytes of 0xFF)
 * to ensure the minimum requirement is exceeded.
 *
 * @section Usage_Scenarios
 * - Transmission error detected mid-frame
 * - Higher priority frame needs immediate channel access
 * - Receiver buffer overflow condition
 * - Link layer reset required
 *
 * @param[out] abortSeq   Pointer to output buffer (minimum 2 bytes)
 * @param[out] abortLen   Pointer to store length of abort sequence (will be 2)
 *
 * @pre abortSeq must have capacity for at least 2 bytes
 * @pre abortLen must not be NULL
 *
 * @post abortSeq[0] = 0xFF, abortSeq[1] = 0xFF
 * @post *abortLen = 2
 *
 * @note The abort sequence must be transmitted without bit stuffing.
 *       It will be recognized by the receiver as an abnormal condition.
 *
 * @warning If abortSeq is NULL or abortLen is NULL, the function
 *          safely returns without action (if abortLen is valid, set to 0).
 */
void hdlc_frame_abort(unsigned char *abortSeq, int *abortLen);

/**
 * @brief Reverse the bit order of a byte
 *
 * Helper function that reverses the bit order of a single byte.
 * Required for FCS processing because FCS bytes are transmitted
 * MSB-first but assembled LSB-first during bit destuffing.
 *
 * @section Bit_Reversal_Mapping
 * - Bit 0 (LSB) becomes Bit 7 (MSB)
 * - Bit 1 becomes Bit 6
 * - ...
 * - Bit 7 (MSB) becomes Bit 0 (LSB)
 *
 * Example: 0b10110001 (0xB1) -> 0b10001101 (0x8D)
 *
 * @param[in] byte The input byte to reverse
 * @return The byte with reversed bit order
 *
 * @note This is a pure function with no side effects.
 * @note Commonly used to correct FCS byte order after destuffing.
 *
 * @see hdlc_frame_decode() for FCS handling context
 */
unsigned char reverse_bits(unsigned char byte);

/*============================================================================*/
/* Advanced HDLC Operations                                                   */
/*============================================================================*/

/**
 * @brief Validate HDLC frame structure
 *
 * Performs basic validation of an HDLC frame without full decoding.
 * Checks for proper flag delimiters and minimum length requirements.
 *
 * @param[in]  frame    Pointer to potential HDLC frame
 * @param[in]  len      Length of data in bytes
 * @param[out] dataLen  Pointer to store length of data between flags (may be NULL)
 *
 * @return true if frame appears valid, false otherwise
 *
 * @retval true  Frame has valid start flag, end flag, and meets minimum length
 * @retval false Frame is invalid or too short
 *
 * @pre frame must not be NULL
 * @pre len must be >= 2
 *
 * @note This is a lightweight check that does not verify CRC or bit stuffing.
 *       Use hdlc_frame_decode() for complete validation.
 */
bool hdlc_validate_frame(const unsigned char *frame, int len, int *dataLen);

/**
 * @brief Check if buffer contains a complete HDLC frame
 *
 * Scans a receive buffer to determine if a complete frame (from opening
 * flag to closing flag) is present. Useful for stream-based reception
 * where frames may arrive in fragments.
 *
 * @param[in]  buffer   Pointer to receive buffer
 * @param[in]  len      Current length of data in buffer
 * @param[out] frameLen Pointer to store detected frame length if found
 *
 * @return true if complete frame detected, false if incomplete or no frame
 *
 * @retval true  Complete frame found, *frameLen contains total length
 * @retval false No complete frame in buffer
 *
 * @pre buffer must not be NULL
 * @pre frameLen must not be NULL if return value is true
 *
 * @note This function does not validate frame content, only presence
 *       of start and end flags with sufficient data between them.
 */
bool hdlc_frame_complete(const unsigned char *buffer, int len, int *frameLen);

#endif /* HDLC_H_ */
