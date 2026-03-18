/* -----------------------------------------------------------------------
 * il2p.h  —  Top-level IL2P encode/decode
 * ----------------------------------------------------------------------- */
#ifndef IL2P_H
#define IL2P_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ax25.h"
#include "il2p_header.h"
#include "il2p_lfsr.h"
#include "il2p_rs.h"

/* Maximum IL2P frame size:
 *   3 sync + 15 header-with-RS + 1081 payload-with-RS + 4 trailing CRC = 1103 bytes
 *   (1081 = ceil(1023/239)*255 = 5*255 - no, let's compute properly)
 *   Max payload data: 1023 bytes
 *   Max payload blocks: ceil(1023/239) = 5 blocks
 *   Max payload RS bytes: 5 * (239 + 16) = 1275 bytes
 *   Total: 3 + 15 + 1275 + 4 = 1297 bytes (conservative upper bound)
 */
#define IL2P_MAX_FRAME_BYTES  1300u

/* Optional trailing CRC enable flag */
#define IL2P_TRAILING_CRC_ENABLED  1u

/**
 * @brief Encode an AX.25 frame into an IL2P bitstream (byte-oriented output).
 *
 * Automatically selects Type 0 or Type 1 encapsulation.
 * Does NOT include preamble; caller prepends preamble if needed.
 * Output begins with the sync word (0xF1, 0x5E, 0x48).
 *
 * @param frame       Parsed AX.25 frame.
 * @param raw_ax25    Raw AX.25 frame bytes (used for Type 0 payload or
 *                    as the I-field source for Type 1).
 * @param raw_len     Length of raw_ax25 in bytes.
 * @param out         Output buffer.
 * @param out_max     Output buffer capacity.
 * @param out_len     Output: actual bytes written.
 * @return            true on success.
 */
bool il2p_encode(const ax25_frame_t *frame, const uint8_t *raw_ax25, size_t raw_len, uint8_t *out, size_t out_max, size_t *out_len);

/**
 * @brief Attempt to decode one IL2P frame from a byte buffer.
 *
 * Searches for sync word, decodes header, then decodes payload.
 *
 * @param in           Input byte buffer (may start at or before sync word).
 * @param in_len       Length of input buffer.
 * @param ax25_out     Output: reconstructed raw AX.25 frame bytes.
 * @param ax25_max     Capacity of ax25_out.
 * @param ax25_len     Output: length of decoded AX.25 frame.
 * @return             true if a valid frame was decoded.
 */
bool il2p_decode(const uint8_t *in, size_t in_len, uint8_t *ax25_out, size_t ax25_max, size_t *ax25_len);

#endif /* IL2P_H */
