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

#ifndef HDLC_H_
#define HDLC_H_

#include <stdint.h>
#include <stdbool.h>

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
 */
#define OUTPUT_BIT(bit_val) do { \
    if ((bit_val)) { \
        byte |= (1 << bitIndex); \
        cnt++; \
        if (cnt == 5) { \
            /* After 5 consecutive 1s, insert a stuffing 0 bit */ \
            bitIndex++; \
            if (bitIndex > 7) { \
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
        encodedFrame[encodedIndex++] = byte; \
        byte = 0; \
        bitIndex = 0; \
    } \
} while(0)

typedef enum HDLC_ERROR_E {
    HDLC_OK,                 ///<
    HDLC_ERR_NO_START_FLAG,  ///<
    HDLC_ERR_NO_END_FLAG,    ///<
    HDLC_ERR_TOO_SHORT,      ///<
    HDLC_ERR_CRC_FAIL,       ///<
    HDLC_ERR_STUFFING,       ///<
    HDLC_ERR_ABORT,          ///<
} hdlc_error_t;

/**
 * @brief Encodes an AX.25 frame into an HDLC frame.
 *
 * This function transforms an AX.25 frame into an HDLC frame suitable for transmission.
 * The encoding process includes several steps:
 * - Reverses the bits of each byte in the input frame using ReverseBits() to match the LSB-first
 *   transmission order of AX.25.
 * - Calculates a 16-bit Frame Check Sequence (FCS) using the CRC function from common.h and appends
 *   it to the frame for error detection.
 * - Performs bit stuffing: after five consecutive 1 bits, a 0 bit is inserted to prevent the flag
 *   sequence (0x7E) from appearing within the data.
 * - Adds the HDLC flag byte (0x7E) at the beginning and end of the frame to delimit the frame boundaries.
 *
 * The encoded frame is written to the provided encodedFrame buffer, and its length is stored in encodedLen.
 * The input frame buffer is modified in-place to include the FCS before encoding.
 *
 * INPUT: Raw AX.25 frame WITHOUT FCS and WITHOUT flags
 *   - Address field (14-70 bytes)
 *   - Control field (1-2 bytes)
 *   - PID field (0-1 byte, only in I and UI frames)
 *   - Information field (0-N bytes)
 *
 * OUTPUT: HDLC-encoded frame WITH bit stuffing and flags
 *   - Start flag (0x7E)
 *   - Address field (bit-stuffed, LSB-first per byte, MSB-first for FCS)
 *   - Control field (bit-stuffed)
 *   - PID field (bit-stuffed, if present)
 *   - Information field (bit-stuffed, if present)
 *   - FCS - 16-bit CRC (2 bytes, MSB-first per AX.25 spec)
 *   - End flag (0x7E)
 *
 * @param frame Pointer to the input AX.25 frame data. This buffer must have enough space to append
 *              two additional bytes for the FCS (frameLen + 2 bytes minimum).
 * @param frameLen Length of the input frame in bytes, excluding the FCS.
 * @param encodedFrame Pointer to the output buffer where the encoded HDLC frame will be stored.
 *                     Must be large enough to hold the encoded data, including flags and potential
 *                     bit stuffing (typically up to 1.25x frameLen + 2 bytes).
 * @param encodedLen Pointer to an integer where the length of the encoded frame will be stored,
 *                   in bytes, including the start and end flags.
 */
void hdlc_frame_encode(unsigned char *frame, int frameLen, unsigned char *encodedFrame, int *encodedLen);

/**
 * @brief Decodes an HDLC frame back into an AX.25 frame.
 *
 * This function reverses the HDLC encoding process to extract an AX.25 frame from an HDLC-encoded frame.
 * The decoding process involves:
 * - Detecting the start and end flags (0x7E) to identify the frame boundaries.
 * - Removing bit stuffing: when five consecutive 1 bits are followed by a 0 bit, the 0 bit is skipped.
 * - Extracting the frame data and the appended FCS (16-bit CRC).
 * - Verifying the FCS using the CRC function from common.h to ensure the frame is not corrupted.
 * - Reversing the bits of each byte in the decoded frame using ReverseBits() to restore the original
 *   byte order expected by AX.25.
 *
 * The decoded frame is written to the decodedFrame buffer, and its length (excluding FCS) is stored
 * in decodedLen. The function returns 0 on success or -1 if decoding fails due to invalid flags,
 * FCS mismatch, or insufficient data.
 *
 * @param encodedFrame Pointer to the input HDLC frame data, including start and end flags (0x7E).
 * @param encodedLen Length of the input HDLC frame in bytes.
 * @param decodedFrame Pointer to the output buffer where the decoded AX.25 frame will be stored.
 *                     Must be large enough to hold the decoded data (typically encodedLen or less).
 * @param decodedLen Pointer to an integer where the length of the decoded frame will be stored,
 *                   in bytes, excluding the FCS.
 * @return error.
 */
hdlc_error_t hdlc_frame_decode(unsigned char *encodedFrame, int encodedLen, unsigned char *decodedFrame, int *decodedLen);

/**
 * @brief Generates a frame abort sequence per AX.25 specification section 3.6
 *
 * According to AX.25 v2.2 specification section 3.6:
 * "If a frame must be prematurely aborted, at least fifteen contiguous ones
 * shall be sent with no bit stuffing added."
 *
 * This function creates an abort sequence consisting of at least 15 consecutive
 * 1-bits with no bit stuffing. The implementation uses 16 consecutive 1-bits
 * (2 bytes of 0xFF) to ensure the minimum requirement is met.
 *
 * When transmitted, this sequence will be recognized by the receiver as an
 * abnormal condition and the current frame will be discarded.
 *
 * @param abortSeq Pointer to output buffer for abort sequence (minimum 2 bytes)
 * @param abortLen Pointer to integer where the length will be stored (will be set to 2)
 */
void hdlc_frame_abort(unsigned char *abortSeq, int *abortLen);

/**
 * @brief Reverses the bit order of a byte
 *
 * Helper function needed for FCS processing since FCS bytes are transmitted
 * MSB-first but assembled LSB-first during destuffing.
 *
 * Example: 0b10110001 -> 0b10001101
 *
 * @param byte Input byte
 * @return Byte with reversed bit order
 */
unsigned char reverse_bits(unsigned char byte);

#endif /* HDLC_H_ */
