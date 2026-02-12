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
 */

#include <string.h>

#include "fx25.h"
#include "hdlc.h"

// FX.25 preamble and postamble (configurable, default 32 bits each)
#define FX25_PREAMBLE_BITS 32
#define FX25_POSTAMBLE_BITS 32

uint8_t fx25_hdlc_encode(const uint8_t *ax25_frame, size_t ax25_len, uint8_t mode_id, uint8_t channel_quality, uint8_t *output, size_t *output_len) {
    if (!ax25_frame || !output || !output_len || ax25_len == 0)
        return 1;

    // Step 1: Encode AX.25 to HDLC
    uint8_t hdlc_frame[512];
    int hdlc_len = 0;

    hdlc_frame_encode((unsigned char*) ax25_frame, ax25_len, hdlc_frame, &hdlc_len);

    if (hdlc_len <= 0)
        return 2;  // HDLC encoding failed

    // Step 2: Select or validate FX.25 mode
    const fx25_mode_t *selected_mode = NULL;

    if (mode_id == 0) {
        // Auto-select mode based on HDLC frame length and channel quality
        mode_id = fx25_select_mode_for_conditions(hdlc_len, channel_quality);
        if (mode_id == 0)
            return 3;  // Frame too large for any FX.25 mode
        selected_mode = fx25_get_mode(mode_id);
        if (!selected_mode)
            return 4;  // Invalid mode ID returned by auto-select
    } else {
        // User specified mode - validate it exists and fits the HDLC frame
        selected_mode = fx25_get_mode(mode_id);
        if (!selected_mode)
            return 4;  // Invalid mode ID

        // Validate HDLC frame fits in selected mode's data capacity
        if ((size_t) hdlc_len > selected_mode->data_bytes)
            return 5;  // Frame too large for selected mode
    }

    // Step 3: Apply FX.25 RS encoding
    fx25_frame_t fx25;
    uint8_t err = fx25_encode(hdlc_frame, hdlc_len, mode_id, &fx25);
    if (err != 0) {
        return 6;  // FX.25 encoding failed
    }

    // Step 4: Build complete FX.25 transmission frame
    size_t idx = 0;

    // Preamble (alternating 0x55 pattern for clock recovery)
    for (int i = 0; i < FX25_PREAMBLE_BITS / 8; i++) {
        output[idx++] = 0x55;  // 01010101 pattern
    }

    // Correlation tag (8 bytes)
    memcpy(output + idx, fx25.correlation_tag, 8);
    idx += 8;

    // RS codeword (data + parity)
    memcpy(output + idx, fx25.rs_codeword, fx25.codeword_len);
    idx += fx25.codeword_len;

    // Postamble (more 0x55 for receiver sync)
    for (int i = 0; i < FX25_POSTAMBLE_BITS / 8; i++) {
        output[idx++] = 0x55;
    }

    *output_len = idx;

    // Clean up FX.25 frame resources
    fx25_frame_free(&fx25);

    return 0;
}

uint8_t fx25_hdlc_decode(const uint8_t *rx_data, size_t rx_len, uint8_t *ax25_frame, size_t *ax25_len, uint8_t *corrected_errors) {
    if (!rx_data || !ax25_frame || !ax25_len || !corrected_errors)
        return 1;

    size_t preamble_bytes = FX25_PREAMBLE_BITS / 8;
    size_t postamble_bytes = FX25_POSTAMBLE_BITS / 8;

    if (rx_len < preamble_bytes + 8 + postamble_bytes) {
        return 1;
    }

    rx_data += preamble_bytes;
    rx_len -= preamble_bytes + postamble_bytes;

    // Step 1: FX.25 decode (find correlation tag, RS decode)
    fx25_frame_t fx25;
    uint8_t err = fx25_decode(rx_data, rx_len, &fx25, corrected_errors);
    if (err != 0) {
        return 2;  // FX.25 decoding failed
    }

    const fx25_mode_t *mode = fx25_get_mode(fx25.mode_id);
    if (!mode) {
        fx25_frame_free(&fx25);
        return 3;  // Invalid mode ID in decoded frame
    }

    // Step 2: Find actual HDLC frame within RS data portion
    // FX.25 pads data with zeros if AX.25 frame is smaller than data_bytes
    // HDLC frames start with 0x7E flag - search for it in decoded data
    int hdlc_start = -1;
    for (size_t i = 0; i < mode->data_bytes; i++) {
        if (fx25.rs_codeword[i] == 0x7E) {
            hdlc_start = (int) i;
            break;
        }
    }

    if (hdlc_start < 0) {
        fx25_frame_free(&fx25);
        return 4;  // No HDLC start flag found in decoded data
    }

    // Step 3: Decode HDLC frame starting from found flag
    uint8_t hdlc_frame[512];
    int hdlc_len = 0;

    // Calculate remaining data length from start flag to end of data area
    size_t remaining_data = mode->data_bytes - hdlc_start;

    hdlc_error_t hdlc_err = hdlc_frame_decode(fx25.rs_codeword + hdlc_start, (int) remaining_data, hdlc_frame, &hdlc_len);

    fx25_frame_free(&fx25);

    if (hdlc_err != HDLC_OK) {
        return 5;  // HDLC decoding failed
    }

    if (hdlc_len > 0) {
        memcpy(ax25_frame, hdlc_frame, hdlc_len);
        *ax25_len = (size_t) hdlc_len;
        return 0;  // Success
    }

    return 6;  // Empty frame decoded
}
