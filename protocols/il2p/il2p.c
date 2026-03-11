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
static bool encode_rs_block(il2p_lfsr_t *lfsr, uint8_t *block, uint8_t data_len, uint8_t nroots) {
    // Scramble data bytes in place
    for (uint8_t i = 0u; i < data_len; i++) {
        block[i] = il2p_lfsr_step_byte(lfsr, block[i]);
    }
    // RS encode: appends parity after data
    return il2p_rs_encode(block, data_len, nroots);
}

// -----------------------------------------------------------------------
// Internal: decode one RS block (RS correct then descramble)
static int8_t decode_rs_block(il2p_lfsr_t *lfsr, uint8_t *block, uint8_t data_len, uint8_t nroots) {
    // RS decode: corrects errors in block[0..data_len+nroots-1]
    int8_t errs = il2p_rs_decode(block, data_len, nroots);
    if (errs < 0)
        return -1;
    // Descramble data bytes
    for (uint8_t i = 0u; i < data_len; i++) {
        block[i] = il2p_lfsr_step_byte(lfsr, block[i]);
    }
    return errs;
}

bool il2p_encode(const ax25_frame_t *frame, const uint8_t *raw_ax25, size_t raw_len, uint8_t *out, size_t out_max, size_t *out_len) {
    if (!frame || !raw_ax25 || !out || !out_len)
        return false;
    if (raw_len > IL2P_MAX_PAYLOAD_BYTES)
        return false;

    size_t pos = 0u;

    // -- 1. Sync word
    if (pos + 3u > out_max)
        return false;
    il2p_sync_write(&out[pos]);
    pos += 3u;

    // -- 2. Build IL2P header
    il2p_header_t hdr;
    il2p_lfsr_t lfsr;

    // Determine payload: Type 1 = I-field only, Type 0 = entire AX.25 frame
    uint16_t payload_len;
    const uint8_t *payload_data;

    {
        il2p_header_t tmp_hdr;
        // Use raw_len as payload tentatively to determine header type
        if (!il2p_header_from_ax25(frame, (uint16_t) raw_len, &tmp_hdr))
            return false;

        if (tmp_hdr.hdr_type == IL2P_HDR_TYPE_1_TRANSLATED) {
            // Type 1: payload = AX.25 information field
            // ax25_frame_t is a base struct; I-frames use ax25_information_frame_t
            // UI frames use ax25_unnumbered_information_frame_t
            // Both carry payload/payload_len fields for the information content
            if (frame->type == AX25_FRAME_INFORMATION_8BIT || frame->type == AX25_FRAME_INFORMATION_16BIT) {
                // Cast to I-frame subtype to access payload and payload_len
                const ax25_information_frame_t *iframe = (const ax25_information_frame_t*) frame;
                payload_data = iframe->payload;
                payload_len = (uint16_t) iframe->payload_len;
            } else if (frame->type == AX25_FRAME_UNNUMBERED_INFORMATION) {
                // Cast to UI-frame subtype to access payload and payload_len
                const ax25_unnumbered_information_frame_t *uiframe = (const ax25_unnumbered_information_frame_t*) frame;
                payload_data = uiframe->payload;
                payload_len = (uint16_t) uiframe->payload_len;
            } else {
                // Non-data frame types have no information field; payload is empty
                payload_data = raw_ax25;
                payload_len = 0u;
            }
        } else {
            // Type 0: payload = entire raw AX.25 frame bytes
            payload_data = raw_ax25;
            payload_len = (uint16_t) raw_len;
        }

        // Rebuild header with correct payload_len
        if (!il2p_header_from_ax25(frame, payload_len, &hdr))
            return false;
    }

    // -- 3. Encode header block
    uint8_t hdr_block[IL2P_HEADER_RS_SIZE];  // 13 data + 2 parity
    if (il2p_header_encode(&hdr, hdr_block) != IL2P_HEADER_SIZE)
        return false;
    // Scramble header then RS encode
    il2p_lfsr_reset(&lfsr);
    if (!encode_rs_block(&lfsr, hdr_block, IL2P_HEADER_SIZE, IL2P_HEADER_PARITY_BYTES))
        return false;
    if (pos + IL2P_HEADER_RS_SIZE > out_max)
        return false;
    memcpy(&out[pos], hdr_block, IL2P_HEADER_RS_SIZE);
    pos += IL2P_HEADER_RS_SIZE;

    // -- 4. Encode payload blocks
    if (payload_len > 0u) {
        uint16_t num_blocks, large_count, small_count;
        uint8_t large_size, small_size;
        il2p_payload_block_sizes(payload_len, &num_blocks, &large_size, &small_size, &large_count, &small_count);

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
            uint8_t blk_total = (uint8_t) (bsize + IL2P_RS_PAY_PARITY);
            if (pos + blk_total > out_max)
                return false;
            memcpy(&out[pos], blk, blk_total);
            pos += blk_total;
            offset += bsize;
        }
    }

    // -- 5. Optional trailing CRC (4 bytes Hamming-encoded)
#if IL2P_TRAILING_CRC_ENABLED
    // For Type 1 frames, C/R bits (address bytes 6 and 13, bit 7) are not
    // transmitted in the IL2P header. The decoder applies a canonical C/R
    // convention when reconstructing the AX.25 frame. The encoder must compute
    // the trailing CRC over the same canonical form so that CRC verification
    // in the decoder succeeds even when the original raw_ax25 C/R bits differ.
    uint16_t crc16;
    if (hdr.hdr_type == IL2P_HDR_TYPE_1_TRANSLATED && raw_len >= 14u) {
        // Determine canonical C/R bit using same logic as il2p_decode Type 1 path
        uint8_t c_r_bit;
        if (hdr.pid != IL2P_PID_S_FRAME && hdr.pid != IL2P_PID_U_FRAME && !hdr.ui) {
            c_r_bit = 1u;  // I-frame: always command
        } else if (hdr.pid == IL2P_PID_S_FRAME) {
            c_r_bit = (uint8_t) ((hdr.control >> 2u) & 1u);
        } else {
            c_r_bit = (uint8_t) (hdr.control & 1u);
        }
        // Compute CRC over raw_ax25 with bytes 6 and 13 replaced by canonical C/R values.
        // Process one byte at a time to avoid a large temporary buffer on the stack.
        uint16_t running = 0xFFFFu;
        for (size_t idx = 0u; idx < raw_len; idx++) {
            uint8_t b;
            if (idx == 6u)
                b = (uint8_t) ((raw_ax25[6u] & 0x7Fu) | (uint8_t) (c_r_bit << 7u));
            else if (idx == 13u)
                b = (uint8_t) ((raw_ax25[13u] & 0x7Fu) | (uint8_t) ((1u - c_r_bit) << 7u));
            else
                b = raw_ax25[idx];
            // CRC-CCITT bit-by-bit step (LSB-first, reversed polynomial 0x8408)
            for (uint8_t bit = 0u; bit < 8u; bit++) {
                uint8_t bv = (b >> bit) & 1u;
                uint8_t lsb = (uint8_t) (running & 1u);
                running >>= 1u;
                if (bv ^ lsb)
                    running ^= 0x8408u;
            }
        }
        crc16 = (uint16_t) (~running & 0xFFFFu);
    } else {
        // Type 0: raw AX.25 passes through verbatim; CRC over unmodified bytes
        crc16 = CRC((unsigned char*) (uintptr_t) raw_ax25, raw_len);
    }

    // Hamming encode table (from IL2P spec section "Hamming Encode Table")
    static const uint8_t hamming_enc[16] = { 0x00, 0x71, 0x62, 0x13, 0x54, 0x25, 0x36, 0x47, 0x38, 0x49, 0x5A, 0x2B, 0x6C, 0x1D, 0x0E, 0x7F };
    // Split CRC into 4 nibbles: CRC3 (high nibble first), CRC2, CRC1, CRC0
    uint8_t nibbles[4];
    nibbles[0] = (uint8_t) ((crc16 >> 12u) & 0x0Fu);  // CRC3 = high nibble, first
    nibbles[1] = (uint8_t) ((crc16 >> 8u) & 0x0Fu);  // CRC2
    nibbles[2] = (uint8_t) ((crc16 >> 4u) & 0x0Fu);  // CRC1
    nibbles[3] = (uint8_t) (crc16 & 0x0Fu);  // CRC0
    if (pos + 4u > out_max)
        return false;
    for (uint8_t i = 0u; i < 4u; i++) {
        out[pos++] = hamming_enc[nibbles[i]];  // 7-bit value, MSB = 0
    }
#endif

    *out_len = pos;
    return true;
}

bool il2p_decode(const uint8_t *in, size_t in_len, uint8_t *ax25_out, size_t ax25_max, size_t *ax25_len) {
    if (!in || !ax25_out || !ax25_len)
        return false;
    *ax25_len = 0u;

    // -- 1. Find sync word
    size_t sync_byte;
    uint8_t sync_bit;
    if (!il2p_sync_search(in, in_len, &sync_byte, &sync_bit))
        return false;
    if (sync_bit != 0u)
        return false;  // Must be byte-aligned for this impl

    size_t pos = sync_byte + 3u;  // Skip sync word

    // -- 2. Decode header block
    if (pos + IL2P_HEADER_RS_SIZE > in_len)
        return false;
    uint8_t hdr_block[IL2P_HEADER_RS_SIZE];
    memcpy(hdr_block, &in[pos], IL2P_HEADER_RS_SIZE);
    pos += IL2P_HEADER_RS_SIZE;

    il2p_lfsr_t lfsr;
    il2p_lfsr_reset(&lfsr);
    if (decode_rs_block(&lfsr, hdr_block, IL2P_HEADER_SIZE, IL2P_HEADER_PARITY_BYTES) < 0)
        return false;  // RS decoding failed: false sync match

    il2p_header_t hdr;
    if (!il2p_header_decode(hdr_block, &hdr))
        return false;

    uint16_t payload_len = hdr.payload_byte_count;

    // -- 3. Decode payload blocks
    size_t payload_out_pos = 0u;
    uint8_t payload_buf[IL2P_MAX_PAYLOAD_BYTES];

    if (payload_len > 0u) {
        uint16_t num_blocks, large_count, small_count;
        uint8_t large_size, small_size;
        il2p_payload_block_sizes(payload_len, &num_blocks, &large_size, &small_size, &large_count, &small_count);

        for (uint16_t b = 0u; b < num_blocks; b++) {
            uint8_t bsize = (b < large_count) ? large_size : small_size;
            uint8_t blk_total = (uint8_t) (bsize + IL2P_RS_PAY_PARITY);
            if (pos + blk_total > in_len)
                return false;
            uint8_t blk[IL2P_RS_PAY_MAX_DATA + IL2P_RS_PAY_PARITY];
            memcpy(blk, &in[pos], blk_total);
            pos += blk_total;
            il2p_lfsr_reset(&lfsr);
            if (decode_rs_block(&lfsr, blk, bsize, IL2P_RS_PAY_PARITY) < 0)
                return false;
            if (payload_out_pos + bsize > sizeof(payload_buf))
                return false;
            memcpy(&payload_buf[payload_out_pos], blk, bsize);
            payload_out_pos += bsize;
        }
    }

    // -- 4. Reconstruct AX.25 frame from IL2P header + payload
    if (hdr.hdr_type == IL2P_HDR_TYPE_0_TRANSPARENT) {
        // Type 0: payload IS the raw AX.25 frame
        if (payload_out_pos > ax25_max)
            return false;
        memcpy(ax25_out, payload_buf, payload_out_pos);
        *ax25_len = payload_out_pos;
    } else {
        // Type 1: reconstruct AX.25 header + payload
        // Build minimal AX.25 frame: [DEST 7B][SRC 7B][CTRL 1B][PID 1B][INFO]
        size_t p = 0u;
        // Destination address (6 ASCII chars + SSID byte)
        char dest_ascii[7], src_ascii[7];
        il2p_sixbit_decode_callsign(hdr.dest_callsign, dest_ascii);
        il2p_sixbit_decode_callsign(hdr.src_callsign, src_ascii);

        for (uint8_t i = 0u; i < 6u; i++) {
            ax25_out[p++] = (uint8_t) ((dest_ascii[i] & 0x7Fu) << 1u);
        }
        ax25_out[p++] = (uint8_t) ((hdr.dest_ssid & 0x0Fu) << 1u);
        for (uint8_t i = 0u; i < 6u; i++) {
            ax25_out[p++] = (uint8_t) ((src_ascii[i] & 0x7Fu) << 1u);
        }
        ax25_out[p++] = (uint8_t) (((hdr.src_ssid & 0x0Fu) << 1u) | 0x01u);

        // Propagate C/R bits into SSID bytes per AX.25 v2.2 Section 6.1.1.
        // COMMAND: dest SSID bit 7 = 1, src SSID bit 7 = 0.
        // RESPONSE: dest SSID bit 7 = 0, src SSID bit 7 = 1.
        // After writing both address fields above, dest SSID is at absolute
        // index 6 and src SSID is at absolute index 13 in ax25_out[].
        // I-frames are always commands per AX.25 spec (c_r_bit = 1).
        // S-frames: IL2P control bit 2 carries the P/F bit (used as C here
        //   since the address-level C/R is not separately preserved in IL2P).
        // U-frames and UI: IL2P control bit 0 carries the original cr bit
        //   from frame->header.cr as encoded by il2p_header_from_ax25().
        // UI frames must be detected via hdr.ui, NOT by hdr.pid, because
        //   hdr.pid for UI holds the actual AX.25 PID (e.g. IL2P_PID_NO_L3),
        //   not IL2P_PID_U_FRAME; omitting !hdr.ui would misclassify UI as
        //   I-frame and always force c_r_bit = 1, losing the encoded cr bit.
        uint8_t c_r_bit = 0u;
        if (hdr.pid != IL2P_PID_S_FRAME && hdr.pid != IL2P_PID_U_FRAME && !hdr.ui) {
            c_r_bit = 1u;  // I-frames are always commands
        } else if (hdr.pid == IL2P_PID_S_FRAME) {
            c_r_bit = (uint8_t) ((hdr.control >> 2u) & 1u);
        } else {
            // U-frame or UI: c_bit stored at control bit 0
            c_r_bit = (uint8_t) (hdr.control & 1u);
        }
        // Dest SSID byte is at absolute index 6: set bit 7 = C
        ax25_out[6u] |= (uint8_t) (c_r_bit << 7u);
        // Src SSID byte is at absolute index 13: set bit 7 = opposite of C
        ax25_out[13u] |= (uint8_t) ((uint8_t) (1u - c_r_bit) << 7u);

        // Control byte reconstruction depends on frame type (pid subfield encodes type)
        if (hdr.pid == IL2P_PID_S_FRAME) {
            // S-frame: control = 0b??RRR0C1 (mod-8)
            uint8_t nr = (uint8_t) ((hdr.control >> 3u) & 0x07u);
            uint8_t c_bit = (uint8_t) ((hdr.control >> 2u) & 0x01u);
            uint8_t opcode = (uint8_t) (hdr.control & 0x03u);
            ax25_out[p++] = (uint8_t) (0x01u | (opcode << 2u) | (c_bit << 4u) | (nr << 5u));
        } else if (hdr.pid == IL2P_PID_U_FRAME || hdr.ui) {
            // U-frame: map IL2P 4-bit opcode back to AX.25 modifier bits via lookup table.
            // Bug fix: direct (opcode << 2) was wrong; IL2P opcode is a sequential index,
            // not the raw AX.25 modifier bit pattern.
            uint8_t pf = (uint8_t) ((hdr.control >> 6u) & 1u);
            uint8_t opcode = (uint8_t) ((hdr.control >> 1u) & 0x0Fu);
            // Base modifier bits for each IL2P opcode (bits 7:5 and 3:2 of AX.25 control byte).
            // U-frame marker bits (1:0 = 11) and P/F bit (4) are OR'd in separately below.
            // Values derived from AX.25 v2.2 Table 4-3 with P/F=0 and marker bits masked out:
            //   SABM=0x2F->base=0x2C, DISC=0x43->base=0x40, DM=0x0F->base=0x0C,
            //   UA=0x63->base=0x60,   FRMR=0x87->base=0x84, XID=0xAF->base=0xAC,
            //   TEST=0xE3->base=0xE0, UI handled separately as 0x03
            static const uint8_t uframe_opcode_to_ax25[16u] = { 0x00u,  // 0x0 - reserved
                    0x2Cu,  // 0x1 - SABM  base (modifier bits only, no P/F, no marker)
                    0x00u,  // 0x2 - reserved
                    0x40u,  // 0x3 - DISC  base
                    0x0Cu,  // 0x4 - DM    base
                    0x00u,  // 0x5 - UI    (handled via hdr.ui branch below)
                    0x60u,  // 0x6 - UA    base
                    0xE0u,  // 0x7 - TEST  base
                    0x84u,  // 0x8 - FRMR  base
                    0xACu,  // 0x9 - XID   base
                    0x00u,  // 0xA - reserved
                    0x00u,  // 0xB - reserved
                    0x00u,  // 0xC - reserved
                    0x00u,  // 0xD - reserved
                    0x00u,  // 0xE - reserved
                    0x00u,  // 0xF - reserved
                    };
            uint8_t ax25_ctrl;
            if (hdr.ui) {
                // UI frame control byte is always 0x03 with P/F at bit 4
                ax25_ctrl = (uint8_t) (0x03u | (pf << 4u));
            } else {
                // Non-UI U-frame: OR in U-frame marker bits (0x03) and P/F
                ax25_ctrl = (uint8_t) (uframe_opcode_to_ax25[opcode & 0x0Fu] | 0x03u | (pf << 4u));
            }
            ax25_out[p++] = ax25_ctrl;
        } else {
            // I-frame
            uint8_t ns = (uint8_t) (hdr.control & 0x07u);
            uint8_t nr = (uint8_t) ((hdr.control >> 3u) & 0x07u);
            uint8_t pf = (uint8_t) ((hdr.control >> 6u) & 0x01u);
            ax25_out[p++] = (uint8_t) ((ns << 1u) | (pf << 4u) | (nr << 5u));
        }

        // PID byte rules per AX.25:
        //   I-frames  (pid >= IL2P_PID_AX25_L3, ui=0): always carry a PID byte.
        //   UI frames (ui=1):                           always carry a PID byte.
        //   S-frames  (pid == IL2P_PID_S_FRAME):        never carry a PID byte.
        //   U-frames non-UI (pid == IL2P_PID_U_FRAME):  never carry a PID byte.
        // Replaces the obscure has_pid + dual-branch pattern with a single clear condition.
        {
            bool needs_pid = (hdr.pid >= IL2P_PID_AX25_L3) || hdr.ui;
            if (needs_pid) {
                ax25_out[p++] = il2p_pid_to_ax25(hdr.pid);
            }
        }

        // Payload (I-field)
        if (payload_out_pos > 0u) {
            if (p + payload_out_pos > ax25_max)
                return false;
            memcpy(&ax25_out[p], payload_buf, payload_out_pos);
            p += payload_out_pos;
        }
        *ax25_len = p;
    }

    // -- 5. Optional trailing CRC verification
    // The CRC was computed by the encoder over the original raw AX.25 frame.
    // The decoder recomputes it over the reconstructed AX.25 frame and compares.
    // Both Type 0 (payload == raw frame) and Type 1 (frame reconstructed above)
    // have *ax25_len set at this point, so ax25_out/*ax25_len are always valid here.
#if IL2P_TRAILING_CRC_ENABLED
    // Hamming decode table: index with 7-bit received codeword, yields corrected nibble.
    // Derived from IL2P spec "Hamming Decode Table".
    // Cross-verification against hamming_enc[]:
    //   hamming_enc[n] = codeword C  =>  hamming_dec[C & 0x7F] must equal n.
    //   hamming_enc: 0x00,0x71,0x62,0x13,0x54,0x25,0x36,0x47,
    //                0x38,0x49,0x5A,0x2B,0x6C,0x1D,0x0E,0x7F
    //   Spot-checks: hamming_dec[0x71]=hamming_dec[113]=0x1 (row14[1]=0x1) checked
    //                hamming_dec[0x38]=hamming_dec[56] =0x8 (row7[0] =0x8) checked
    //                hamming_dec[0x7F]=hamming_dec[127]=0xF (row15[7]=0xF) checked
    static const uint8_t hamming_dec[128u] = { 0x0u, 0x0u, 0x0u, 0x3u, 0x0u, 0x5u, 0xEu, 0x7u,  // indices   0..7
            0x0u, 0x9u, 0xEu, 0xBu, 0xEu, 0xDu, 0xEu, 0xEu,  // indices   8..15
            0x0u, 0x3u, 0x3u, 0x3u, 0x4u, 0xDu, 0x6u, 0x3u,  // indices  16..23
            0x8u, 0xDu, 0xAu, 0x3u, 0xDu, 0xDu, 0xEu, 0xDu,  // indices  24..31
            0x0u, 0x5u, 0x2u, 0xBu, 0x5u, 0x5u, 0x6u, 0x5u,  // indices  32..39
            0x8u, 0xBu, 0xBu, 0xBu, 0xCu, 0x5u, 0xEu, 0xBu,  // indices  40..47
            0x8u, 0x1u, 0x6u, 0x3u, 0x6u, 0x5u, 0x6u, 0x6u,  // indices  48..55
            0x8u, 0x8u, 0x8u, 0xBu, 0x8u, 0xDu, 0x6u, 0xFu,  // indices  56..63
            0x0u, 0x9u, 0x2u, 0x7u, 0x4u, 0x7u, 0x7u, 0x7u,  // indices  64..71
            0x9u, 0x9u, 0xAu, 0x9u, 0xCu, 0x9u, 0xEu, 0x7u,  // indices  72..79
            0x4u, 0x1u, 0xAu, 0x3u, 0x4u, 0x4u, 0x4u, 0x7u,  // indices  80..87
            0xAu, 0x9u, 0xAu, 0xAu, 0x4u, 0xDu, 0xAu, 0xFu,  // indices  88..95
            0x2u, 0x1u, 0x2u, 0x2u, 0xCu, 0x5u, 0x2u, 0x7u,  // indices  96..103
            0xCu, 0x9u, 0x2u, 0xBu, 0xCu, 0xCu, 0xCu, 0xFu,  // indices 104..111
            0x1u, 0x1u, 0x2u, 0x1u, 0x4u, 0x1u, 0x6u, 0xFu,  // indices 112..119
            0x8u, 0x1u, 0xAu, 0xFu, 0xCu, 0xFu, 0xFu, 0xFu   // indices 120..127
            };
    // Verify 4 Hamming-encoded CRC bytes are present in the input buffer
    if (pos + 4u > in_len)
        return false;
    // Decode each Hamming codeword to recover one CRC nibble (MSB nibble first)
    uint16_t rx_crc16 = 0u;
    for (uint8_t i = 0u; i < 4u; i++) {
        uint8_t codeword = in[pos + i] & 0x7Fu;  // mask to 7 bits per spec
        uint8_t nibble = hamming_dec[codeword] & 0x0Fu;
        rx_crc16 = (uint16_t) ((rx_crc16 << 4u) | nibble);
    }
    pos += 4u;
    // Recompute CRC over the fully reconstructed AX.25 frame and compare
    uint16_t calc_crc = CRC((unsigned char*) ax25_out, *ax25_len);
    if (calc_crc != rx_crc16) {
        return false;  // CRC mismatch: RS corrected but frame data is still wrong
    }
#endif

    return true;
}
