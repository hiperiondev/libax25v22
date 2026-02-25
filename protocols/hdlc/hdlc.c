/**
 * @file hdlc.c
 * @brief HDLC (High-Level Data Link Control) Framing for AX.25 v2.2
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "hdlc.h"
#include "common.h"

void hdlc_frame_encode(unsigned char *frame, int frameLen, unsigned char *encodedFrame, int *encodedLen) {
    // Calculate CRC on original frame data (BEFORE bit reversal)
    // The FCS is calculated on all bits except the flags and the FCS itself
    uint16_t crc = CRC(frame, frameLen);

    // AX.25 FCS transmission order: low byte first, then high byte
    // Each byte is transmitted LSB-first
    unsigned char crcLow = crc & 0xFF;
    unsigned char crcHigh = (crc >> 8) & 0xFF;

    // Bit stuffing state
    int cnt = 0;              // Counter for consecutive 1-bits
    int bitIndex = 0;         // Current bit position in byte (0=LSB, 7=MSB)
    unsigned char byte = 0;   // Current output byte being constructed
    int encodedIndex = 0;     // Output buffer index
    // maxEncodedLen: worst-case output size for frameLen input bytes.
    // Used by OUTPUT_BIT to prevent writing past the end of encodedFrame.
    // overflow: set to 1 by OUTPUT_BIT if a write would exceed maxEncodedLen.
    // On overflow *encodedLen is set to 0 to signal failure to the caller.
    int maxEncodedLen = hdlc_encoded_size_max(frameLen);
    int overflow = 0;

    // Output HDLC start flag (0x7E = 01111110)
    encodedFrame[encodedIndex++] = 0x7E;

    // Process frame bytes - output bit-by-bit with stuffing
    // AX.25 transmits each byte LSB-first (bit 0 first)
    for (int i = 0; i < frameLen; i++) {
        unsigned char currentByte = frame[i];
        // Extract bits LSB-first (bit 0, 1, 2, ..., 7)
        for (int bit_pos = 0; bit_pos < 8; bit_pos++) {
            unsigned char bit = (currentByte >> bit_pos) & 0x01;
            OUTPUT_BIT(bit);
            if (overflow)
                break;
        }
        if (overflow)
            break;
    }

    // Process CRC low byte (transmitted first, LSB-first)
    if (!overflow) {
        for (int bit_pos = 0; bit_pos < 8; bit_pos++) {
            unsigned char bit = (crcLow >> bit_pos) & 0x01;
            OUTPUT_BIT(bit);
            if (overflow)
                break;
        }
    }

    // Process CRC high byte (transmitted second, LSB-first)
    if (!overflow) {
        for (int bit_pos = 0; bit_pos < 8; bit_pos++) {
            unsigned char bit = (crcHigh >> bit_pos) & 0x01;
            OUTPUT_BIT(bit);
            if (overflow)
                break;
        }
    }

    // Output end flag (0x7E)
    // If there's a partial byte, output it first
    if (!overflow) {
        if (bitIndex > 0) {
            if (encodedIndex < maxEncodedLen) {
                encodedFrame[encodedIndex++] = byte;
            } else {
                overflow = 1;
            }
        }
        if (!overflow) {
            if (encodedIndex < maxEncodedLen) {
                encodedFrame[encodedIndex++] = 0x7E;
            } else {
                overflow = 1;
            }
        }
    }

    if (overflow) {
        // Signal buffer overflow to caller: 0 is an unambiguously invalid HDLC length.
        *encodedLen = 0;
        return;
    }

    *encodedLen = encodedIndex;
}

hdlc_error_t hdlc_frame_decode(unsigned char *encodedFrame, int encodedLen, unsigned char *decodedFrame, int *decodedLen) {
    int endFlagFound = 0;
    int cnt = 0;  // Count of consecutive 1 bits
    int bitIndex = 0;  // Bit position in current output byte (0=LSB, 7=MSB)
    unsigned char byte = 0;  // Current byte being assembled
    int decodedIndex = 0;
    int pending_stuffing = 0;
    int byteIndex = 0;

    if (encodedLen < 2 || encodedFrame[0] != 0x7E) {
        return HDLC_ERR_NO_START_FLAG;  // Must start with flag
    }
    byteIndex = 1;  // Skip start flag

    // Process bits LSB-first to match encoder
    for (int i = byteIndex; i < encodedLen; i++) {
        unsigned char currentByte = encodedFrame[i];

        // Check if this entire byte is the end flag
        if (currentByte == 0x7E) {
            endFlagFound = 1;
            break;
        }

        // Process each bit LSB-first
        for (int k = 0; k < 8; k++) {
            unsigned char bit = (currentByte >> k) & 0x01;

            // Improved bit destuffing with proper handling of stuffing errors (exactly 6 ones)
            // and abort sequences (>=7 ones) per AX.25 v2.2 section 3.6 and HDLC rules
            if (cnt == 5 && bit == 0) {
                // Stuffed 0 inserted by transmitter - remove it
                cnt = 0;
                continue;
            }

            if (bit == 1) {
                cnt++;
            } else {
                cnt = 0;
            }

            // cnt==6 must not return immediately; we defer to allow cnt to reach
            // 7+ for ABORT. A pending_stuffing flag records the 6-ones state.
            // Next iteration: if bit==1 -> cnt==7 -> ABORT; if bit==0 -> STUFFING.
            if (pending_stuffing) {
                if (bit == 1) {
                    // 7th consecutive one -> abort sequence
                    return HDLC_ERR_ABORT;
                } else {
                    // 6 ones then a zero: stuffing violation (no zero-bit stuffing at cnt=6)
                    pending_stuffing = 0;
                    return HDLC_ERR_STUFFING;
                }
            }

            if (cnt == 6) {
                /// 6 contiguous ones: defer decision to next bit
                pending_stuffing = 1;
                continue;
            }

            // Assemble byte LSB-first
            if (bit) {
                byte |= (1 << bitIndex);
            }
            bitIndex++;

            if (bitIndex == 8) {
                decodedFrame[decodedIndex++] = byte;
                byte = 0;
                bitIndex = 0;
            }
        }
    }

    if (!endFlagFound)
        return HDLC_ERR_NO_END_FLAG;
    if (decodedIndex < 2)
        return HDLC_ERR_TOO_SHORT;

    // Extract and verify CRC
    uint16_t frameCRC = (decodedFrame[decodedIndex - 2]) | (decodedFrame[decodedIndex - 1] << 8);
    decodedIndex -= 2;

    uint16_t crc = CRC(decodedFrame, decodedIndex);
    if (crc != frameCRC)
        return HDLC_ERR_CRC_FAIL;

    *decodedLen = decodedIndex;
    return HDLC_OK;
}

void hdlc_frame_abort(unsigned char *abortSeq, int *abortLen) {
    if (!abortSeq || !abortLen) {
        if (abortLen)
            *abortLen = 0;
        return;
    }

    // AX.25 v2.2 specification section 3.6:
    // "If a frame must be prematurely aborted, at least fifteen contiguous ones
    // shall be sent with no bit stuffing added."
    //
    // We use 16 consecutive 1-bits (2 bytes of 0xFF) to meet the requirement
    abortSeq[0] = 0xFF;
    abortSeq[1] = 0xFF;
    *abortLen = 2;
}

unsigned char reverse_bits(unsigned char byte) {
    unsigned char reversed = 0;
    for (int i = 0; i < 8; i++) {
        reversed |= ((byte >> i) & 0x01) << (7 - i);
    }
    return reversed;
}

