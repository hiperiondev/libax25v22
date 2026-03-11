/**
 * @file fx25_hdlc.c
 * @brief FX.25 Forward Error Correction Wrapper for AX.25/HDLC Frames
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

#include "fx25.h"
#include "hdlc.h"
#include "fx25_hdlc.h"

// FX.25 preamble and postamble (configurable, default 32 bits each)
#define FX25_PREAMBLE_BITS  32
#define FX25_POSTAMBLE_BITS 32

// Return type changed from uint8_t to fx25_hdlc_err_t.
// All return statements updated to use named enum constants instead of
// raw integers, eliminating the overlap with decode error codes.
fx25_hdlc_err_t fx25_hdlc_encode(const uint8_t *ax25_frame, size_t ax25_len, uint8_t mode_id, uint8_t channel_quality, uint8_t *output, size_t *output_len) {
    if (!ax25_frame || !output || !output_len || ax25_len == 0)
        return FX25_HDLC_ERR_INVALID_PARAM;

    // Step 1: Encode AX.25 to HDLC
    uint8_t hdlc_frame[512];
    int hdlc_len = 0;

    hdlc_frame_encode((unsigned char*) ax25_frame, ax25_len, hdlc_frame, &hdlc_len);

    if (hdlc_len <= 0)
        return FX25_HDLC_ERR_HDLC_ENCODE;

    // Step 2: Select or validate FX.25 mode
    const fx25_mode_t *selected_mode = NULL;

    if (mode_id == 0) {
        // Auto-select mode based on HDLC frame length and channel quality
        mode_id = fx25_select_mode_for_conditions(hdlc_len, channel_quality);
        if (mode_id == 0)
            return FX25_HDLC_ERR_AUTO_SELECT;

        selected_mode = fx25_get_mode(mode_id);
        if (!selected_mode)
            return FX25_HDLC_ERR_BAD_MODE;
    } else {
        // User specified mode - validate it exists and fits the HDLC frame
        selected_mode = fx25_get_mode(mode_id);
        if (!selected_mode)
            return FX25_HDLC_ERR_BAD_MODE;

        // Validate HDLC frame fits in selected mode's data capacity
        if ((size_t) hdlc_len > selected_mode->data_bytes)
            return FX25_HDLC_ERR_FRAME_LARGE;
    }

    // Step 3: Apply FX.25 RS encoding
    fx25_frame_t fx25;
    uint8_t err = fx25_encode(hdlc_frame, hdlc_len, mode_id, &fx25);
    if (err != 0)
        return FX25_HDLC_ERR_RS_ENCODE;

    // Step 4: Build complete FX.25 transmission frame
    size_t idx = 0;

    // Preamble: 0x55 clock-sync pattern per this implementation's convention
    for (int i = 0; i < FX25_PREAMBLE_BITS / 8; i++) {
        output[idx++] = 0x55;  // 01010101 clock-sync pattern
    }

    // Correlation tag (8 bytes)
    memcpy(output + idx, fx25.correlation_tag, 8);
    idx += 8;

    // RS codeword (data + parity)
    memcpy(output + idx, fx25.rs_codeword, fx25.codeword_len);
    idx += fx25.codeword_len;

    // Postamble: 0x55 clock-sync pattern
    for (int i = 0; i < FX25_POSTAMBLE_BITS / 8; i++) {
        output[idx++] = 0x55;  // 01010101 clock-sync pattern
    }

    *output_len = idx;

    // Clean up FX.25 frame resources
    fx25_frame_free(&fx25);

    return FX25_HDLC_OK;
}

fx25_hdlc_err_t fx25_hdlc_decode(const uint8_t *rx_data, size_t rx_len, uint8_t *ax25_frame, size_t *ax25_len, uint8_t *corrected_errors) {
    if (!rx_data || !ax25_frame || !ax25_len || !corrected_errors)
        return FX25_HDLC_ERR_INVALID_PARAM;

    size_t preamble_bytes = FX25_PREAMBLE_BITS / 8;
    size_t postamble_bytes = FX25_POSTAMBLE_BITS / 8;

    if (rx_len < preamble_bytes + 8 + postamble_bytes)
        return FX25_HDLC_ERR_INVALID_PARAM;

    rx_data += preamble_bytes;
    rx_len -= preamble_bytes + postamble_bytes;

    // Step 1: FX.25 decode (find correlation tag, RS decode)
    fx25_frame_t fx25;
    uint8_t err = fx25_decode(rx_data, rx_len, &fx25, corrected_errors);
    if (err != 0) {
        return FX25_HDLC_ERR_RS_DECODE;
    }

    const fx25_mode_t *mode = fx25_get_mode(fx25.mode_id);
    if (!mode) {
        fx25_frame_free(&fx25);
        return FX25_HDLC_ERR_BAD_MODE_ID;
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
        return FX25_HDLC_ERR_NO_FLAG;
    }

    // Step 3: Decode HDLC frame starting from found flag
    uint8_t hdlc_frame[512];
    int hdlc_len = 0;

    // Calculate remaining data length from start flag to end of data area
    size_t remaining_data = mode->data_bytes - hdlc_start;

    hdlc_error_t hdlc_err = hdlc_frame_decode(fx25.rs_codeword + hdlc_start, (int) remaining_data, hdlc_frame, &hdlc_len);

    fx25_frame_free(&fx25);

    if (hdlc_err != HDLC_OK)
        return FX25_HDLC_ERR_HDLC_DECODE;

    if (hdlc_len > 0) {
        memcpy(ax25_frame, hdlc_frame, hdlc_len);
        *ax25_len = (size_t) hdlc_len;
        return FX25_HDLC_OK;
    }

    return FX25_HDLC_ERR_EMPTY;
}
