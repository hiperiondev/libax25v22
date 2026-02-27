// -----------------------------------------------------------------------
// il2p_header.c
// -----------------------------------------------------------------------
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "il2p_header.h"
#include "il2p_sixbit.h"

// PID mapping table: AX.25 PID -> IL2P PID (index 0..15 is il2p_pid, value is ax25_pid)
static const uint8_t il2p_pid_ax25_table[16] = { 0xFF,  // 0x0 S-frame  -- no PID byte
        0xFF,  // 0x1 U-frame  -- no PID byte
        0xFF,  // 0x2 AX.25 L3 -- match by pattern, not single value
        0x01,  // 0x3 ISO 8208 / X.25 PLP
        0x06,  // 0x4 Compressed TCP/IP
        0x07,  // 0x5 Uncompressed TCP/IP
        0x08,  // 0x6 Segmentation fragment
        0xFF,  // 0x7 Future
        0xFF,  // 0x8 Future
        0xFF,  // 0x9 Future
        0xFF,  // 0xA Future
        0xCC,  // 0xB ARPA Internet Protocol
        0xCD,  // 0xC ARPA Address Resolution
        0xCE,  // 0xD FlexNet
        0xCF,  // 0xE TheNET
        0xF0,  // 0xF No Layer 3
        };

uint8_t il2p_pid_from_ax25(uint8_t ax25_pid, bool is_s_frame, bool is_u_frame) {
    if (is_s_frame)
        return IL2P_PID_S_FRAME;
    if (is_u_frame)
        return IL2P_PID_U_FRAME;  // UI frame handled by caller
    // Linear search -- table is 16 entries, trivial cost on MCU
    for (uint8_t i = 2u; i < 16u; i++) {
        if (il2p_pid_ax25_table[i] == ax25_pid)
            return i;
    }

    // AX.25 Layer 3 PIDs: match yy10yyyy or yy01yyyy pattern
    // restricted to the practically occurring range 0x10..0x1F only.
    // PIDs above 0x1F that match the bit pattern are not valid L3 PIDs
    // in practice and would cause incorrect Type 1 encoding.
    // Original range 0x10..0x3F was too wide: 0x20..0x2F also matched
    // the 10-pattern but those values are not genuine AX.25 L3 PIDs
    // in real traffic. Narrowed to 0x10..0x1F per NinoTNC reference.
    if ((ax25_pid & 0x30u) == 0x20u || (ax25_pid & 0x30u) == 0x10u) {
        if (ax25_pid <= 0x1Fu) {
            return IL2P_PID_AX25_L3;
        }
    }

    return 0xFFu;  // Not mappable -- force Type 0
}

uint8_t il2p_pid_to_ax25(uint8_t il2p_pid) {
    if (il2p_pid > 15u)
        return 0xFFu;
    return il2p_pid_ax25_table[il2p_pid];
}

// Bit offsets within the 104-bit (13-byte) header
#define HBIT_DEST_CS_BASE    0u    // 36 bits: 6 chars x 6 bits
#define HBIT_DEST_SSID       36u   // 4 bits
#define HBIT_SRC_CS_BASE     40u   // 36 bits: 6 chars x 6 bits
#define HBIT_SRC_SSID        76u   // 4 bits
#define HBIT_UI              80u   // 1 bit
#define HBIT_PID             81u   // 4 bits
#define HBIT_CONTROL         85u   // 7 bits
#define HBIT_FEC_LEVEL       92u   // 1 bit (RESERVED, always 1 = max FEC per v0.6)
#define HBIT_HDR_TYPE        93u   // 1 bit
#define HBIT_PAYLOAD_COUNT   94u   // 10 bits (MSB first)
// Total = 36+4+36+4+1+4+7+1+1+10 = 104 bits = 13 bytes

// Helper: write 'count' bits of 'value' into buf[] at starting bit 'start_bit' (MSB first)
static void pack_bits(uint8_t *buf, uint16_t start_bit, uint8_t count, uint32_t value) {
    for (int8_t i = (int8_t) (count - 1u); i >= 0; i--) {
        uint16_t bit_pos = (uint16_t) (start_bit + (uint8_t) (count - 1u) - (uint8_t) i);
        uint8_t byte_idx = (uint8_t) (bit_pos / 8u);
        uint8_t bit_idx = (uint8_t) (7u - (bit_pos % 8u));  // MSB first within byte
        if ((value >> (uint8_t) i) & 1u) {
            buf[byte_idx] |= (uint8_t) (1u << bit_idx);
        } else {
            buf[byte_idx] &= (uint8_t) ~(1u << bit_idx);
        }
    }
}

// Helper: read 'count' bits from buf[] at starting bit 'start_bit' (MSB first)
static uint32_t unpack_bits(const uint8_t *buf, uint16_t start_bit, uint8_t count) {
    uint32_t result = 0u;
    for (uint8_t i = 0u; i < count; i++) {
        uint16_t bit_pos = (uint16_t) (start_bit + i);
        uint8_t byte_idx = (uint8_t) (bit_pos / 8u);
        uint8_t bit_idx = (uint8_t) (7u - (bit_pos % 8u));
        result = (result << 1u) | ((buf[byte_idx] >> bit_idx) & 1u);
    }
    return result;
}

uint8_t il2p_header_encode(const il2p_header_t *hdr, uint8_t out[IL2P_HEADER_SIZE]) {
    if (!hdr || !out)
        return 0u;
    memset(out, 0, IL2P_HEADER_SIZE);

    if (hdr->hdr_type == IL2P_HDR_TYPE_1_TRANSLATED) {
        // Pack destination callsign (36 bits)
        for (uint8_t i = 0u; i < 6u; i++) {
            pack_bits(out, (uint16_t) (HBIT_DEST_CS_BASE + i * 6u), 6u, hdr->dest_callsign[i]);
        }
        // Destination SSID
        pack_bits(out, HBIT_DEST_SSID, 4u, hdr->dest_ssid & 0x0Fu);
        // Source callsign (36 bits)
        for (uint8_t i = 0u; i < 6u; i++) {
            pack_bits(out, (uint16_t) (HBIT_SRC_CS_BASE + i * 6u), 6u, hdr->src_callsign[i]);
        }
        // Source SSID
        pack_bits(out, HBIT_SRC_SSID, 4u, hdr->src_ssid & 0x0Fu);
        // UI bit
        pack_bits(out, HBIT_UI, 1u, hdr->ui & 1u);
        // 4-bit PID
        pack_bits(out, HBIT_PID, 4u, hdr->pid & 0x0Fu);
        // 7-bit control
        pack_bits(out, HBIT_CONTROL, 7u, hdr->control & 0x7Fu);
    }
    // FEC level: always 1 (max FEC = 16 parity bytes) per v0.6 spec
    pack_bits(out, HBIT_FEC_LEVEL, 1u, 1u);
    // Header type bit
    pack_bits(out, HBIT_HDR_TYPE, 1u, hdr->hdr_type & 1u);
    // 10-bit payload byte count
    pack_bits(out, HBIT_PAYLOAD_COUNT, 10u, hdr->payload_byte_count & 0x3FFu);

    return IL2P_HEADER_SIZE;
}

bool il2p_header_decode(const uint8_t in[IL2P_HEADER_SIZE], il2p_header_t *hdr) {
    if (!in || !hdr)
        return false;
    memset(hdr, 0, sizeof(il2p_header_t));

    hdr->hdr_type = (uint8_t) unpack_bits(in, HBIT_HDR_TYPE, 1u);
    hdr->payload_byte_count = (uint16_t) unpack_bits(in, HBIT_PAYLOAD_COUNT, 10u);

    if (hdr->hdr_type == IL2P_HDR_TYPE_1_TRANSLATED) {
        for (uint8_t i = 0u; i < 6u; i++) {
            hdr->dest_callsign[i] = (uint8_t) unpack_bits(in, (uint16_t) (HBIT_DEST_CS_BASE + i * 6u), 6u);
        }
        hdr->dest_ssid = (uint8_t) unpack_bits(in, HBIT_DEST_SSID, 4u);
        for (uint8_t i = 0u; i < 6u; i++) {
            hdr->src_callsign[i] = (uint8_t) unpack_bits(in, (uint16_t) (HBIT_SRC_CS_BASE + i * 6u), 6u);
        }
        hdr->src_ssid = (uint8_t) unpack_bits(in, HBIT_SRC_SSID, 4u);
        hdr->ui = (uint8_t) unpack_bits(in, HBIT_UI, 1u);
        hdr->pid = (uint8_t) unpack_bits(in, HBIT_PID, 4u);
        hdr->control = (uint8_t) unpack_bits(in, HBIT_CONTROL, 7u);
    }
    return true;
}

uint16_t il2p_payload_block_count(uint16_t payload_byte_count) {
    if (payload_byte_count == 0u)
        return 0u;
    // Integer ceiling division: ceil(a/b) = (a + b - 1) / b -- no floating point
    return (uint16_t) ((payload_byte_count + IL2P_MAX_BLOCK_DATA_BYTES - 1u) / IL2P_MAX_BLOCK_DATA_BYTES);
}

void il2p_payload_block_sizes(uint16_t payload_byte_count, uint16_t *num_blocks, uint8_t *large_block_size, uint8_t *small_block_size,
        uint16_t *large_block_count, uint16_t *small_block_count) {
    uint16_t nb = il2p_payload_block_count(payload_byte_count);
    if (nb == 0u) {
        *num_blocks = 0u;
        *large_block_size = 0u;
        *small_block_size = 0u;
        *large_block_count = 0u;
        *small_block_count = 0u;
        return;
    }
    // Integer floor division -- no floating point
    uint8_t sbs = (uint8_t) (payload_byte_count / nb);  // small_block_size
    uint8_t lbs = (uint8_t) (sbs + 1u);                 // large_block_size
    // large_block_count = remainder after even distribution
    uint16_t lbc = (uint16_t) (payload_byte_count - (uint16_t) ((uint16_t) nb * (uint16_t) sbs));
    uint16_t sbc = (uint16_t) (nb - lbc);

    *num_blocks = nb;
    *large_block_size = lbs;
    *small_block_size = sbs;
    *large_block_count = lbc;
    *small_block_count = sbc;
}

bool il2p_header_from_ax25(const ax25_frame_t *frame, uint16_t payload_len, il2p_header_t *hdr) {
    if (!frame || !hdr)
        return false;
    memset(hdr, 0, sizeof(il2p_header_t));
    hdr->payload_byte_count = payload_len;

    // start modified part
    // ax25_frame_t is a base struct; repeaters count is in header.repeaters.num_repeaters
    // (not header.num_repeaters as in the original erroneous code)
    if (frame->header.repeaters.num_repeaters > 0) {
        hdr->hdr_type = IL2P_HDR_TYPE_0_TRANSPARENT;
        return true;
    }
    // end modified part

    // Try to SIXBIT-encode destination callsign
    uint8_t dest_sb[6], src_sb[6];
    char dest_ascii[7], src_ascii[7];
    for (uint8_t i = 0u; i < 6u; i++) {
        dest_ascii[i] = (char) (frame->header.destination.callsign[i]);
        src_ascii[i] = (char) (frame->header.source.callsign[i]);
    }
    dest_ascii[6] = src_ascii[6] = '\0';

    if (!il2p_sixbit_encode_callsign(dest_ascii, dest_sb) || !il2p_sixbit_encode_callsign(src_ascii, src_sb)) {
        // Callsign not SIXBIT-encodable: fall back to Type 0
        hdr->hdr_type = IL2P_HDR_TYPE_0_TRANSPARENT;
        return true;
    }

    // start modified part
    // ax25_frame_t is a base struct; frame type uses the exact enum values from ax25.h.
    // There is no AX25_FRAME_INFORMATION, AX25_FRAME_SUPERVISORY, or AX25_FRAME_UNNUMBERED
    // generic enum value -- only the specific variants with _8BIT/_16BIT/_RR_ etc.
    // Cast to appropriate subtype to access sequence numbers, pid, modifier, etc.

    // Classify frame type using actual ax25.h enum values
    bool is_i_frame = (frame->type == AX25_FRAME_INFORMATION_8BIT || frame->type == AX25_FRAME_INFORMATION_16BIT);

    bool is_s_frame = (frame->type == AX25_FRAME_SUPERVISORY_RR_8BIT || frame->type == AX25_FRAME_SUPERVISORY_RNR_8BIT
            || frame->type == AX25_FRAME_SUPERVISORY_REJ_8BIT || frame->type == AX25_FRAME_SUPERVISORY_SREJ_8BIT
            || frame->type == AX25_FRAME_SUPERVISORY_RR_16BIT || frame->type == AX25_FRAME_SUPERVISORY_RNR_16BIT
            || frame->type == AX25_FRAME_SUPERVISORY_REJ_16BIT || frame->type == AX25_FRAME_SUPERVISORY_SREJ_16BIT);

    bool is_ui = (frame->type == AX25_FRAME_UNNUMBERED_INFORMATION);

    // All remaining unnumbered non-UI types (SABM, DISC, DM, UA, FRMR, XID, TEST)
    bool is_u_frame = (!is_i_frame && !is_s_frame && !is_ui && frame->type != AX25_FRAME_RAW);

    // SABME (extended mode) not supported in Type 1 -- force Type 0
    if (frame->type == AX25_FRAME_UNNUMBERED_SABME) {
        hdr->hdr_type = IL2P_HDR_TYPE_0_TRANSPARENT;
        return true;
    }

    // Map PID -- obtain from appropriate subtype pointer
    uint8_t il2p_pid;
    if (is_s_frame) {
        il2p_pid = IL2P_PID_S_FRAME;
    } else if (is_u_frame && !is_ui) {
        il2p_pid = IL2P_PID_U_FRAME;
    } else {
        // I-frame or UI: read PID from subtype struct
        uint8_t ax25_pid;
        if (is_i_frame) {
            // Cast to I-frame subtype to access pid field
            const ax25_information_frame_t *iframe = (const ax25_information_frame_t*) frame;
            ax25_pid = iframe->pid;
        } else {
            // UI frame: cast to UI subtype to access pid field
            const ax25_unnumbered_information_frame_t *uiframe = (const ax25_unnumbered_information_frame_t*) frame;
            ax25_pid = uiframe->pid;
        }
        il2p_pid = il2p_pid_from_ax25(ax25_pid, false, false);
        if (il2p_pid == 0xFFu) {
            // Unknown PID: fall back to Type 0
            hdr->hdr_type = IL2P_HDR_TYPE_0_TRANSPARENT;
            return true;
        }
    }

    // Build control subfield (7 bits) by casting to appropriate subtype
    uint8_t ctrl7 = 0u;
    if (is_i_frame) {
        // I-frame: P/F | N(R)[2:0] | N(S)[2:0]
        // Cast to ax25_information_frame_t to access nr, ns, pf
        const ax25_information_frame_t *iframe = (const ax25_information_frame_t*) frame;
        uint8_t ns = (uint8_t) (iframe->ns & 0x07);
        uint8_t nr = (uint8_t) (iframe->nr & 0x07);
        uint8_t pf = (uint8_t) (iframe->pf ? 1u : 0u);
        ctrl7 = (uint8_t) ((pf << 6u) | (nr << 3u) | ns);
    } else if (is_s_frame) {
        // S-frame: N(R)[2:0] | C | OPCODE[1:0]
        // Cast to ax25_supervisory_frame_t to access nr, pf, code
        const ax25_supervisory_frame_t *sframe = (const ax25_supervisory_frame_t*) frame;
        uint8_t nr = (uint8_t) (sframe->nr & 0x07);
        uint8_t c_bit = (uint8_t) (sframe->pf ? 1u : 0u);

        // sframe->code must use the encoding RR=0, RNR=1, REJ=2, SREJ=3
        // which is identical to the IL2P opcode table (IL2P spec Table 3-4).
        // No translation is needed. If ax25.h ever changes this mapping,
        // a translation table must be inserted here.
        // AX.25 v2.2 S-frame supervisory bits S1:S0 (control byte bits 3:2):
        //   0b00 = RR, 0b01 = RNR, 0b10 = REJ, 0b11 = SREJ
        uint8_t opcode = (uint8_t) (sframe->code & 0x03u);
        ctrl7 = (uint8_t) ((nr << 3u) | (c_bit << 2u) | opcode);
    } else {
        // U-frame or UI: cast to ax25_unnumbered_frame_t for pf and modifier
        // C/R bit comes from frame->header.cr (address field command/response bit)
        const ax25_unnumbered_frame_t *uframe = (const ax25_unnumbered_frame_t*) frame;
        uint8_t pf = (uint8_t) (uframe->pf ? 1u : 0u);
        uint8_t c_bit = (uint8_t) (frame->header.cr ? 1u : 0u);
        // modifier is a 5-bit field encoding U-frame type; map to IL2P 4-bit opcode
        // AX.25 modifier values per ax25.h comments:
        //   SABM=0x2F, SABME=0x6F, DISC=0x43, DM=0x0F, UA=0x63,
        //   FRMR=0x87, XID=0xAF, TEST=0xE3, UI=0x03
        // IL2P uses a 4-bit opcode field; map the relevant types
        uint8_t opcode;
        switch (frame->type) {
            case AX25_FRAME_UNNUMBERED_SABM:
                opcode = 0x01u;
            break;  // SABM  -> 0001
            case AX25_FRAME_UNNUMBERED_DISC:
                opcode = 0x03u;
            break;  // DISC  -> 0011
            case AX25_FRAME_UNNUMBERED_DM:
                opcode = 0x04u;
            break;  // DM    -> 0100
            case AX25_FRAME_UNNUMBERED_UA:
                opcode = 0x06u;
            break;  // UA    -> 0110
            case AX25_FRAME_UNNUMBERED_FRMR:
                opcode = 0x08u;
            break;  // FRMR  -> 1000
            case AX25_FRAME_UNNUMBERED_INFORMATION:
                opcode = 0x05u;
            break;  // UI    -> 0101
            case AX25_FRAME_UNNUMBERED_XID:
                opcode = 0x09u;
            break;  // XID   -> 1001
            case AX25_FRAME_UNNUMBERED_TEST:
                opcode = 0x07u;
            break;  // TEST  -> 0111
            default:
                opcode = 0x00u;
            break;
        }
        // Suppress unused variable warning for uframe if only used for pf
        (void) uframe;
        ctrl7 = (uint8_t) ((pf << 6u) | (opcode << 1u) | c_bit);
    }
    // end modified part

    // Fill in Type 1 header
    hdr->hdr_type = IL2P_HDR_TYPE_1_TRANSLATED;
    memcpy(hdr->dest_callsign, dest_sb, 6u);
    hdr->dest_ssid = (uint8_t) (frame->header.destination.ssid & 0x0F);
    memcpy(hdr->src_callsign, src_sb, 6u);
    hdr->src_ssid = (uint8_t) (frame->header.source.ssid & 0x0F);
    hdr->ui = is_ui ? 1u : 0u;
    hdr->pid = il2p_pid;
    hdr->control = ctrl7;
    return true;
}
