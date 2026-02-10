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

#ifndef FX25_HDLC_H_
#define FX25_HDLC_H_

#include <stdint.h>
#include <stddef.h>

/**
 * Encode AX.25 frame with FX.25 FEC wrapper
 *
 * Process:
 * 1. Encode AX.25 frame to HDLC (with flags, stuffing, FCS)
 * 2. Apply FX.25 RS encoding to HDLC frame
 * 3. Prepend correlation tag
 * 4. Add preamble/postamble
 *
 * @param ax25_frame Raw AX.25 frame (no flags, no FCS)
 * @param ax25_len Length of AX.25 frame
 * @param mode_id FX.25 mode (determines RS parameters)
 * @param channel_quality 0-100 (for auto mode selection)
 * @param output Output buffer for FX.25 frame
 * @param output_len Length of output frame
 * @return 0 on success, error code otherwise
 */
uint8_t fx25_hdlc_encode(const uint8_t *ax25_frame, size_t ax25_len, uint8_t mode_id, uint8_t channel_quality, uint8_t *output, size_t *output_len);

/**
 * Decode FX.25 frame and extract AX.25 data
 *
 * @param rx_data Received FX.25 frame (raw bytes from demodulator)
 * @param rx_len Length of received data
 * @param ax25_frame Output buffer for decoded AX.25 frame
 * @param ax25_len Length of decoded AX.25 frame
 * @param corrected_errors Number of errors corrected by RS
 * @return 0 on success, error code otherwise
 */
uint8_t fx25_hdlc_decode(const uint8_t *rx_data, size_t rx_len, uint8_t *ax25_frame, size_t *ax25_len, uint8_t *corrected_errors);

#endif /* FX25_HDLC_H_ */
