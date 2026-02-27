// -----------------------------------------------------------------------
// il2p.c  --  IL2P top-level encode/decode
// Covers Type 0 (transparent) and Type 1 (translated) encapsulation.
// -----------------------------------------------------------------------
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "il2p.h"
#include "il2p_sync.h"
#include "il2p_sixbit.h"
#include "fx25_gf256.h"
#include "common.h"

// -----------------------------------------------------------------------
// Internal: encode one RS block (scramble then append parity)
// block[0..data_len-1] = data (will be scrambled in place)
// block[data_len..data_len+nroots-1] = parity (appended)
// lfsr state is used and advanced (reset before call for first block)
static bool encode_rs_block(il2p_lfsr_t *lfsr, uint8_t *block,
                              uint8_t data_len, uint8_t nroots) {
    // Scramble data bytes in place
    for (uint8_t i = 0u; i < data_len; i++) {
        block[i] = il2p_lfsr_step_byte(lfsr, block[i]);
    }
    // RS encode: appends parity after data
    return il2p_rs_encode(block, data_len, nroots);
}

// -----------------------------------------------------------------------
// Internal: decode one RS block (RS correct then descramble)
static int8_t decode_rs_block(il2p_lfsr_t *lfsr, uint8_t *block,
                               uint8_t data_len, uint8_t nroots) {
    // RS decode: corrects errors in block[0..data_len+nroots-1]
    int8_t errs = il2p_rs_decode(block, data_len, nroots);
    if (errs < 0) return -1;
    // Descramble data bytes
    for (uint8_t i = 0u; i < data_len; i++) {
        block[i] = il2p_lfsr_step_byte(lfsr, block[i]);
    }
    return errs;
}

bool il2p_encode(const ax25_frame_t *frame,
                 const uint8_t *raw_ax25, size_t raw_len,
                 uint8_t *out, size_t out_max, size_t *out_len) {
    if (!frame || !raw_ax25 || !out || !out_len) return false;
    if (raw_len > IL2P_MAX_PAYLOAD_BYTES) return false;

    size_t pos = 0u;

    // -- 1. Sync word
    if (pos + 3u > out_max) return false;
    il2p_sync_write(&out[pos]);
    pos += 3u;

    // -- 2. Build IL2P header
    il2p_header_t hdr;
    il2p_lfsr_t   lfsr;

    // Determine payload: Type 1 = I-field only, Type 0 = entire AX.25 frame
    uint16_t payload_len;
    const uint8_t *payload_data;

    {
        il2p_header_t tmp_hdr;
        // Use raw_len as payload tentatively to determine header type
        if (!il2p_header_from_ax25(frame, (uint16_t)raw_len, &tmp_hdr)) return false;

        if (tmp_hdr.hdr_type == IL2P_HDR_TYPE_1_TRANSLATED) {
            // start modified part
            // Type 1: payload = AX.25 information field
            // ax25_frame_t is a base struct; I-frames use ax25_information_frame_t
            // UI frames use ax25_unnumbered_information_frame_t
            // Both carry payload/payload_len fields for the information content
            if (frame->type == AX25_FRAME_INFORMATION_8BIT ||
                frame->type == AX25_FRAME_INFORMATION_16BIT) {
                // Cast to I-frame subtype to access payload and payload_len
                const ax25_information_frame_t *iframe =
                    (const ax25_information_frame_t *)frame;
                payload_data = iframe->payload;
                payload_len  = (uint16_t)iframe->payload_len;
            } else if (frame->type == AX25_FRAME_UNNUMBERED_INFORMATION) {
                // Cast to UI-frame subtype to access payload and payload_len
                const ax25_unnumbered_information_frame_t *uiframe =
                    (const ax25_unnumbered_information_frame_t *)frame;
                payload_data = uiframe->payload;
                payload_len  = (uint16_t)uiframe->payload_len;
            } else {
                // Non-data frame types have no information field; payload is empty
                payload_data = raw_ax25;
                payload_len  = 0u;
            }
            // end modified part
        } else {
            // Type 0: payload = entire raw AX.25 frame bytes
            payload_data = raw_ax25;
            payload_len  = (uint16_t)raw_len;
        }

        // Rebuild header with correct payload_len
        if (!il2p_header_from_ax25(frame, payload_len, &hdr)) return false;
    }

    // -- 3. Encode header block
    uint8_t hdr_block[IL2P_HEADER_RS_SIZE];  // 13 data + 2 parity
    if (il2p_header_encode(&hdr, hdr_block) != IL2P_HEADER_SIZE) return false;
    // Scramble header then RS encode
    il2p_lfsr_reset(&lfsr);
    if (!encode_rs_block(&lfsr, hdr_block, IL2P_HEADER_SIZE, IL2P_HEADER_PARITY_BYTES))
        return false;
    if (pos + IL2P_HEADER_RS_SIZE > out_max) return false;
    memcpy(&out[pos], hdr_block, IL2P_HEADER_RS_SIZE);
    pos += IL2P_HEADER_RS_SIZE;

    // -- 4. Encode payload blocks
    if (payload_len > 0u) {
        uint16_t num_blocks, large_count, small_count;
        uint8_t  large_size, small_size;
        il2p_payload_block_sizes(payload_len, &num_blocks,
                                  &large_size, &small_size,
                                  &large_count, &small_count);

        uint16_t offset = 0u;
        // Large blocks first (closest to header)
        for (uint16_t b = 0u; b < num_blocks; b++) {
            uint8_t bsize = (b < large_count) ? large_size : small_size;
            uint8_t blk[IL2P_RS_PAY_MAX_DATA + IL2P_RS_PAY_PARITY];
            memcpy(blk, &payload_data[offset], bsize);
            // Reset LFSR for each block (v0.6 spec)
            il2p_lfsr_reset(&lfsr);
            if (!encode_rs_block(&lfsr, blk, bsize, IL2P_RS_PAY_PARITY))
                return false;
            uint8_t blk_total = (uint8_t)(bsize + IL2P_RS_PAY_PARITY);
            if (pos + blk_total > out_max) return false;
            memcpy(&out[pos], blk, blk_total);
            pos += blk_total;
            offset += bsize;
        }
    }

    // -- 5. Optional trailing CRC (4 bytes Hamming-encoded)
#if IL2P_TRAILING_CRC_ENABLED
    // CRC-16-CCITT over the original raw AX.25 data (before IL2P encoding)
    uint16_t crc16 = CRC((unsigned char *)(uintptr_t)raw_ax25, raw_len);
    // Hamming encode table (from IL2P spec section "Hamming Encode Table")
    static const uint8_t hamming_enc[16] = {
        0x00, 0x71, 0x62, 0x13, 0x54, 0x25, 0x36, 0x47,
        0x38, 0x49, 0x5A, 0x2B, 0x6C, 0x1D, 0x0E, 0x7F
    };
    // Split CRC into 4 nibbles: CRC3 (high nibble first), CRC2, CRC1, CRC0
    uint8_t nibbles[4];
    nibbles[0] = (uint8_t)((crc16 >> 12u) & 0x0Fu);  // CRC3 = high nibble, first
    nibbles[1] = (uint8_t)((crc16 >>  8u) & 0x0Fu);  // CRC2
    nibbles[2] = (uint8_t)((crc16 >>  4u) & 0x0Fu);  // CRC1
    nibbles[3] = (uint8_t)( crc16         & 0x0Fu);  // CRC0
    if (pos + 4u > out_max) return false;
    for (uint8_t i = 0u; i < 4u; i++) {
        out[pos++] = hamming_enc[nibbles[i]];  // 7-bit value, MSB = 0
    }
#endif

    *out_len = pos;
    return true;
}

bool il2p_decode(const uint8_t *in, size_t in_len,
                 uint8_t *ax25_out, size_t ax25_max, size_t *ax25_len) {
    if (!in || !ax25_out || !ax25_len) return false;
    *ax25_len = 0u;

    // -- 1. Find sync word
    size_t  sync_byte;
    uint8_t sync_bit;
    if (!il2p_sync_search(in, in_len, &sync_byte, &sync_bit)) return false;
    if (sync_bit != 0u) return false;  // Must be byte-aligned for this impl

    size_t pos = sync_byte + 3u;  // Skip sync word

    // -- 2. Decode header block
    if (pos + IL2P_HEADER_RS_SIZE > in_len) return false;
    uint8_t hdr_block[IL2P_HEADER_RS_SIZE];
    memcpy(hdr_block, &in[pos], IL2P_HEADER_RS_SIZE);
    pos += IL2P_HEADER_RS_SIZE;

    il2p_lfsr_t lfsr;
    il2p_lfsr_reset(&lfsr);
    if (decode_rs_block(&lfsr, hdr_block, IL2P_HEADER_SIZE, IL2P_HEADER_PARITY_BYTES) < 0)
        return false;  // RS decoding failed: false sync match

    il2p_header_t hdr;
    if (!il2p_header_decode(hdr_block, &hdr)) return false;

    uint16_t payload_len = hdr.payload_byte_count;

    // -- 3. Decode payload blocks
    size_t payload_out_pos = 0u;
    uint8_t payload_buf[IL2P_MAX_PAYLOAD_BYTES];

    if (payload_len > 0u) {
        uint16_t num_blocks, large_count, small_count;
        uint8_t  large_size, small_size;
        il2p_payload_block_sizes(payload_len, &num_blocks,
                                  &large_size, &small_size,
                                  &large_count, &small_count);

        for (uint16_t b = 0u; b < num_blocks; b++) {
            uint8_t bsize    = (b < large_count) ? large_size : small_size;
            uint8_t blk_total = (uint8_t)(bsize + IL2P_RS_PAY_PARITY);
            if (pos + blk_total > in_len) return false;
            uint8_t blk[IL2P_RS_PAY_MAX_DATA + IL2P_RS_PAY_PARITY];
            memcpy(blk, &in[pos], blk_total);
            pos += blk_total;
            il2p_lfsr_reset(&lfsr);
            if (decode_rs_block(&lfsr, blk, bsize, IL2P_RS_PAY_PARITY) < 0)
                return false;
            if (payload_out_pos + bsize > sizeof(payload_buf)) return false;
            memcpy(&payload_buf[payload_out_pos], blk, bsize);
            payload_out_pos += bsize;
        }
    }

    // -- 4. Reconstruct AX.25 frame from IL2P header + payload
    if (hdr.hdr_type == IL2P_HDR_TYPE_0_TRANSPARENT) {
        // Type 0: payload IS the raw AX.25 frame
        if (payload_out_pos > ax25_max) return false;
        memcpy(ax25_out, payload_buf, payload_out_pos);
        *ax25_len = payload_out_pos;
    } else {
        // Type 1: reconstruct AX.25 header + payload
        // Build minimal AX.25 frame: [DEST 7B][SRC 7B][CTRL 1B][PID 1B][INFO]
        size_t p = 0u;
        // Destination address (6 ASCII chars + SSID byte)
        char dest_ascii[7], src_ascii[7];
        il2p_sixbit_decode_callsign(hdr.dest_callsign, dest_ascii);
        il2p_sixbit_decode_callsign(hdr.src_callsign,  src_ascii);

        for (uint8_t i = 0u; i < 6u; i++) {
            ax25_out[p++] = (uint8_t)((dest_ascii[i] & 0x7Fu) << 1u);
        }
        ax25_out[p++] = (uint8_t)((hdr.dest_ssid & 0x0Fu) << 1u);
        for (uint8_t i = 0u; i < 6u; i++) {
            ax25_out[p++] = (uint8_t)((src_ascii[i] & 0x7Fu) << 1u);
        }
        // Source last-address-bit (bit 0 = 1 if no repeaters)
        ax25_out[p++] = (uint8_t)(((hdr.src_ssid & 0x0Fu) << 1u) | 0x01u);

        // Reconstruct AX.25 Control byte from IL2P 7-bit control subfield
        uint8_t ax25_pid = il2p_pid_to_ax25(hdr.pid);
        bool has_pid = (ax25_pid != 0xFFu) || hdr.ui;

        // Control byte reconstruction depends on frame type (pid subfield encodes type)
        if (hdr.pid == IL2P_PID_S_FRAME) {
            // S-frame: control = 0b??RRR0C1 (mod-8)
            uint8_t nr     = (uint8_t)((hdr.control >> 3u) & 0x07u);
            uint8_t c_bit  = (uint8_t)((hdr.control >> 2u) & 0x01u);
            uint8_t opcode = (uint8_t)( hdr.control        & 0x03u);
            ax25_out[p++] = (uint8_t)(0x01u | (opcode << 2u) | (c_bit << 4u) | (nr << 5u));
        } else if (hdr.pid == IL2P_PID_U_FRAME || hdr.ui) {
            // U-frame: reconstruct from IL2P opcode
            uint8_t pf     = (uint8_t)((hdr.control >> 6u) & 1u);
            uint8_t opcode = (uint8_t)((hdr.control >> 1u) & 0x0Fu);
            uint8_t c_bit  = (uint8_t)( hdr.control        & 1u);
            ax25_out[p++] = (uint8_t)(0x03u | (opcode << 2u) | (pf << 4u));
            (void)c_bit;  // C/R is set in address SSID bits by higher layer
        } else {
            // I-frame
            uint8_t ns = (uint8_t)( hdr.control        & 0x07u);
            uint8_t nr = (uint8_t)((hdr.control >> 3u) & 0x07u);
            uint8_t pf = (uint8_t)((hdr.control >> 6u) & 0x01u);
            ax25_out[p++] = (uint8_t)((ns << 1u) | (pf << 4u) | (nr << 5u));
        }

        // PID byte (if applicable)
        if (has_pid && !hdr.ui && hdr.pid >= 2u) {
            ax25_out[p++] = ax25_pid;
        } else if (hdr.ui) {
            ax25_out[p++] = ax25_pid;  // UI frame: use mapped PID
        }

        // Payload (I-field)
        if (payload_out_pos > 0u) {
            if (p + payload_out_pos > ax25_max) return false;
            memcpy(&ax25_out[p], payload_buf, payload_out_pos);
            p += payload_out_pos;
        }
        *ax25_len = p;
    }

    return true;
}
