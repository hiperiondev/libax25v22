/**
 * @file hdlc.c
 * @brief HDLC (High-Level Data Link Control) Framing for AX.25 v2.2
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
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

// hdlc_rx_bit: process one NRZI-decoded bit from the raw bit stream.
// Detects 0x7E flag via 8-bit shift register, performs bit-destuffing,
// suppresses empty frames (consecutive flags produce no output), and
// assembles bytes LSB-first per AX.25 v2.2 section 3.8.
// Returns 1 when a complete frame is in h->buf[0..h->len-1].
// No 64-bit types, no float. Safe for all MCU targets.
int hdlc_rx_bit(hdlc_rx_t *h, uint8_t bit) {
    if (!h)
        return 0;

    // Feed bit into shift register: new bit enters at MSB, old bits shift right.
    // After 8 bits the register holds the last 8 received bits for flag matching.
    h->shift = (uint8_t) ((h->shift >> 1) | (bit ? 0x80u : 0u));

    // Flag detection: 01111110 = 0x7E
    if (h->shift == HDLC_FLAG_BYTE) {
        // Inside a frame with enough data: signal frame complete.
        // Minimum 4 bytes prevents pure flag sequences from appearing as frames
        // (empty-frame suppression per AX.25 v2.2 section 2.2.1 requirement).
        if (h->in_frame && h->len >= 4) {
            h->in_frame = 0;
            // h->len intentionally NOT reset; caller reads buf[0..len-1]
            return 1;
        }
        // Consecutive flag or opening flag: reset accumulator.
        // Empty-frame suppression: consecutive 0x7E simply restart the state.
        h->in_frame = 1;
        h->len = 0;
        h->bit_pos = 0;
        h->cur_byte = 0;
        h->ones = 0;
        return 0;
    }

    // Not inside a frame: discard bit
    if (!h->in_frame)
        return 0;

    // Bit-destuffing per AX.25 v2.2 section 3.6:
    // After 5 consecutive one-bits the transmitter inserts a zero; discard it.
    // 6 or more consecutive one-bits is an abort sequence; discard the frame.
    if (bit) {
        h->ones++;
        if (h->ones >= 6) {
            // Abort sequence detected - discard frame in progress
            h->in_frame = 0;
            h->len = 0;
            h->ones = 0;
            return 0;
        }
    } else {
        if (h->ones == 5) {
            // Stuffed zero - discard this bit and reset ones counter
            h->ones = 0;
            return 0;
        }
        h->ones = 0;
    }

    // Assemble byte LSB-first (AX.25 transmits LSb first per section 3.8)
    if (bit)
        h->cur_byte = (uint8_t) (h->cur_byte | (uint8_t) (1u << h->bit_pos));
    h->bit_pos++;

    if (h->bit_pos == 8) {
        if (h->len < HDLC_MAX_FRAME_RX) {
            h->buf[h->len++] = h->cur_byte;
        } else {
            // Buffer overflow - silently discard frame in progress
            h->in_frame = 0;
            h->len = 0;
        }
        h->bit_pos = 0;
        h->cur_byte = 0;
    }

    return 0;
}

// hdlc_tx_interframe_fill: fill buf with fill_count 0x7E flag bytes.
// Inter-frame time fill for continuous HDLC streams per AX.25 v2.2 section 2.2.1.
// Returns number of bytes actually written (capped at buf_len).
int hdlc_tx_interframe_fill(unsigned char *buf, int buf_len, int fill_count) {
    int i;
    if (!buf || buf_len <= 0 || fill_count <= 0)
        return 0;
    if (fill_count > buf_len)
        fill_count = buf_len;
    for (i = 0; i < fill_count; i++)
        buf[i] = (unsigned char) HDLC_FLAG_BYTE;
    return fill_count;
}

// NRZI encoder/decoder implementation per AX.25 v2.2 §3.8.
// See hdlc.h for full usage documentation and pipeline diagram.

// hdlc_nrzi_init: reset encoder and decoder state.
// HDLC channel idle is continuous mark (1-bits), so both levels start at 1.
void hdlc_nrzi_init(nrzi_t *n) {
    if (!n)
        return;
    n->last_level = 1u;  // HDLC idle = mark = 1
    n->prev_level = 1u;  // receiver starts at same idle level
}

// hdlc_nrzi_encode_bit: NRZ -> NRZI.
// A 0-bit causes a transition; a 1-bit leaves the level unchanged.
// Returns the new channel level (0 or 1) to transmit.
uint8_t hdlc_nrzi_encode_bit(nrzi_t *n, uint8_t nrz_bit) {
    if (!n)
        return 0u;
    // 0 = transition: flip the current output level
    if (nrz_bit == 0u)
        n->last_level ^= 1u;
    // 1 = no transition: last_level is unchanged
    return n->last_level;
}

// hdlc_nrzi_decode_bit: NRZI -> NRZ.
// A transition relative to the previous level means NRZ 0.
// No transition means NRZ 1.
// Returns the recovered NRZ bit (0 or 1) for hdlc_rx_bit().
uint8_t hdlc_nrzi_decode_bit(nrzi_t *n, uint8_t nrzi_bit) {
    uint8_t nrz;
    if (!n)
        return 0u;
    // transition detected = NRZ 0; same level = NRZ 1
    nrz = (nrzi_bit != n->prev_level) ? 0u : 1u;
    n->prev_level = nrzi_bit;  // update state for next bit
    return nrz;
}
