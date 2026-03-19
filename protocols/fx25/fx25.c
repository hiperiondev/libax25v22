/**
 * @file fx25.c
 * @brief FX.25 Forward Error Correction Extension for AX.25 - Protocol Interface
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#include <string.h>

#include "hal.h"
#include "fx25_rs.h"
#include "fx25.h"

// Correlation tags per FX.25 v01.06 specification
// Stored as byte arrays in big-endian order to avoid 64-bit arithmetic
static const fx25_mode_t fx25_modes[] = {  //
        { 0x01, { 0xB7, 0x4D, 0xB7, 0xDF, 0x8A, 0x53, 0x2F, 0x3E }, 239, 16, 8 },  //
                { 0x02, { 0x26, 0xFF, 0x60, 0xA6, 0x00, 0xCC, 0x8F, 0xDE }, 128, 16, 8 },  //
                { 0x03, { 0xC7, 0xDC, 0x05, 0x08, 0xF3, 0xD9, 0xB0, 0x9E }, 64, 16, 8 },   //
                { 0x04, { 0x8F, 0x05, 0x6E, 0xB4, 0x36, 0x96, 0x60, 0xEE }, 32, 16, 8 },   //
                { 0x05, { 0x6E, 0x26, 0x0B, 0x1A, 0xC5, 0x83, 0x5F, 0xAE }, 223, 32, 16 },  //
                { 0x06, { 0xFF, 0x94, 0xDC, 0x63, 0x4F, 0x1C, 0xFF, 0x4E }, 128, 32, 16 },  //
                { 0x07, { 0x1E, 0xB7, 0xB9, 0xCD, 0xBC, 0x09, 0xC0, 0x0E }, 64, 32, 16 },  //
                { 0x08, { 0xDB, 0xF8, 0x69, 0xBD, 0x2D, 0xBB, 0x17, 0x76 }, 32, 32, 16 },  //
                { 0x09, { 0x3A, 0xDB, 0x0C, 0x13, 0xDE, 0xAE, 0x28, 0x36 }, 191, 64, 32 },  //
                { 0x0A, { 0xAB, 0x69, 0xDB, 0x6A, 0x54, 0x31, 0x88, 0xD6 }, 128, 64, 32 },  //
                { 0x0B, { 0x4A, 0x4A, 0xBE, 0xC4, 0xA7, 0x24, 0xB7, 0x96 }, 64, 64, 32 },  //
                { 0, { 0, 0, 0, 0, 0, 0, 0, 0 }, 0, 0, 0 }                                 //
        };

static int hamming_distance_tags(const uint8_t *tag1, const uint8_t *tag2) {
    int distance = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t xor_byte = tag1[i] ^ tag2[i];
        // Count set bits (Brian Kernighan's algorithm - microcontroller friendly)
        while (xor_byte) {
            distance++;
            xor_byte &= (xor_byte - 1);  // Clear lowest set bit
        }
    }
    return distance;
}

const fx25_mode_t* fx25_get_mode(uint8_t mode_id) {
    for (int i = 0; fx25_modes[i].tag_id != 0; i++) {
        if (fx25_modes[i].tag_id == mode_id) {
            return &fx25_modes[i];
        }
    }
    return NULL;
}

uint8_t fx25_select_mode(size_t ax25_len) {
    // Select smallest mode that fits the frame
    if (ax25_len <= 32)
        return FX25_MODE_32_16;  // Smallest overhead
    if (ax25_len <= 64)
        return FX25_MODE_64_16;
    if (ax25_len <= 128)
        return FX25_MODE_128_16;
    if (ax25_len <= 191)
        return FX25_MODE_191_64;  // Best protection for medium
    if (ax25_len <= 223)
        return FX25_MODE_223_32;
    if (ax25_len <= 239)
        return FX25_MODE_239_16;
    return 0;  // Too large for FX.25 single block
}

// fx25_frame_free: no-op; rs_codeword is now an inline array, no heap to free.
// Kept for API compatibility so callers do not need changes.
void fx25_frame_free(fx25_frame_t *frame) {
    (void) frame;  // inline array: nothing to free
}

// Select optimal FX.25 mode based on channel quality and frame size
// channel_quality: 0-100 (0=worst, 100=perfect)
// Returns: mode_id for fx25_encode()
uint8_t fx25_select_mode_for_conditions(size_t ax25_len, uint8_t channel_quality) {
    // Clamp channel quality to valid range
    if (channel_quality > 100) {
        channel_quality = 100;
    }

    // For poor channels (<30%), use high redundancy (64 parity bytes)
    if (channel_quality < 30) {
        if (ax25_len <= 32) {
            return FX25_MODE_32_32;  // 32+32 provides better ratio for tiny frames
        }
        if (ax25_len <= 64) {
            return FX25_MODE_64_64;
        }
        if (ax25_len <= 128) {
            return FX25_MODE_128_64;
        }
        return FX25_MODE_191_64;
    }
    // For medium channels (30-70%), use medium redundancy (32 parity bytes)
    else if (channel_quality < 70) {
        if (ax25_len <= 32) {
            return FX25_MODE_32_32;
        }
        if (ax25_len <= 64) {
            return FX25_MODE_64_32;
        }
        if (ax25_len <= 128) {
            return FX25_MODE_128_32;
        }
        return FX25_MODE_223_32;
    }
    // For good channels (>=70%), use low redundancy (16 parity bytes)
    else {
        if (ax25_len <= 32) {
            return FX25_MODE_32_16;
        }
        if (ax25_len <= 64) {
            return FX25_MODE_64_16;
        }
        if (ax25_len <= 128) {
            return FX25_MODE_128_16;
        }
        return FX25_MODE_239_16;
    }
}

uint8_t fx25_encode(const uint8_t *ax25_frame, size_t ax25_len, uint8_t mode_id, fx25_frame_t *fx25_frame) {
    if (!ax25_frame || !fx25_frame || ax25_len == 0)
        return 1;

    const fx25_mode_t *mode = fx25_get_mode(mode_id);
    if (!mode)
        return 2;
    if (ax25_len > mode->data_bytes)
        return 3;  // Frame too large

    fx25_frame->mode_id = mode_id;

    for (int i = 0; i < 8; i++) {
        fx25_frame->correlation_tag[i] = mode->correlation_tag[i];
    }

    fx25_frame->codeword_len = (uint16_t) (mode->data_bytes + mode->parity_bytes);

    // Stack buffer for the full-length message with leading zero padding.
    // full_k = 255 - parity_bytes; minimum parity is 16, so full_k <= 239 < 255.
    uint8_t full_message[255];
    int full_k = 255 - mode->parity_bytes;
    int shorten = full_k - mode->data_bytes;

    memset(full_message, 0, (size_t) shorten);
    memcpy(full_message + shorten, ax25_frame, ax25_len);

    // Pad unused data bytes with 0x7E per FX.25 spec §4.3 (HDLC idle / AX.25 flag fill).
    // 0x7E is the value recommended by the specification for unused codeblock bytes.
    // The RS codec only requires encoder and decoder to agree on the same pad value.
    if (ax25_len < mode->data_bytes) {
        memset(full_message + shorten + ax25_len, 0x7E, mode->data_bytes - ax25_len);
    }

    // Copy only the transmitted message portion (no leading zeros)
    memcpy(fx25_frame->rs_codeword, full_message + shorten, mode->data_bytes);

    uint8_t *parity = fx25_frame->rs_codeword + mode->data_bytes;

    rs_params_t params;
    rs_init_params(&params, mode->parity_bytes);
    rs_encode(&params, full_message, parity);

    return 0;
}

uint8_t fx25_decode(const uint8_t *rx_data, size_t rx_len, fx25_frame_t *fx25_frame, uint8_t *corrected_errors) {
    if (!rx_data || !fx25_frame || !corrected_errors)
        return 1;

    // Minimum size: correlation tag (8) + at least some data + parity
    if (rx_len < 8 + 32 + 16)
        return 2;

    // Extract correlation tag (first 8 bytes)
    uint8_t rx_tag[8];
    for (int i = 0; i < 8; i++) {
        rx_tag[i] = rx_data[i];
    }

    // Find matching mode with Hamming distance tolerance (up to 6 bit errors per FX.25 spec Section 2.2)
    const fx25_mode_t *mode = NULL;
    int best_distance = 65;  // Impossible value (max is 64 bits)
    for (int i = 0; fx25_modes[i].tag_id != 0; i++) {
        int distance = hamming_distance_tags(rx_tag, fx25_modes[i].correlation_tag);
        if (distance <= 6 && distance < best_distance) {
            best_distance = distance;
            mode = &fx25_modes[i];
        }
    }

    if (!mode)
        return 3;  // No matching tag found within tolerance

    size_t expected_len = 8 + mode->data_bytes + mode->parity_bytes;
    if (rx_len < expected_len)
        return 4;  // Too short

    fx25_frame->mode_id = mode->tag_id;

    for (int i = 0; i < 8; i++) {
        fx25_frame->correlation_tag[i] = rx_tag[i];
    }
    fx25_frame->codeword_len = (uint16_t) (mode->data_bytes + mode->parity_bytes);

    // Correct shortened RS decoding: insert leading zeros to form full 255-symbol codeword.
    // Stack buffer — always exactly 255 bytes.
    int full_k = 255 - mode->parity_bytes;
    int shorten = full_k - mode->data_bytes;

    uint8_t full_codeword[255];
    memset(full_codeword, 0, (size_t) shorten);
    memcpy(full_codeword + shorten, rx_data + 8, mode->data_bytes + mode->parity_bytes);

    rs_params_t params;
    rs_init_params(&params, mode->parity_bytes);

    int result = rs_decode(&params, full_codeword);

    if (result < 0) {
        *corrected_errors = 0xFF;  // Uncorrectable
        return 6;
    }

    *corrected_errors = (uint8_t) result;

    // Copy corrected short codeword into inline array (data + parity, no leading zeros)
    memcpy(fx25_frame->rs_codeword, full_codeword + shorten, fx25_frame->codeword_len);

    return 0;
}
