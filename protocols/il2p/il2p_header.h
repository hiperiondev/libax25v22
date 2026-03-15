/* -----------------------------------------------------------------------
 * il2p_header.h
 * ----------------------------------------------------------------------- */
#ifndef IL2P_HEADER_H
#define IL2P_HEADER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ax25.h"

/* IL2P header is always exactly 13 bytes before RS encoding */
#define IL2P_HEADER_SIZE           13u
/* After RS encoding (2 parity bytes appended) = 15 bytes transmitted */
#define IL2P_HEADER_RS_SIZE        15u
/* Maximum IL2P payload size (10-bit field) */
#define IL2P_MAX_PAYLOAD_BYTES     1023u
/* RS parity bytes for the header block */
#define IL2P_HEADER_PARITY_BYTES   2u
/* RS parity bytes per payload block (fixed per v0.6 spec) */
#define IL2P_PAYLOAD_PARITY_BYTES  16u
/* Maximum bytes per data portion of one payload block */
#define IL2P_MAX_BLOCK_DATA_BYTES  239u

/* Header type constants stored in hdr_type bit of header byte 12 */
#define IL2P_HDR_TYPE_0_TRANSPARENT  0u
#define IL2P_HDR_TYPE_1_TRANSLATED   1u

/* IL2P PID mapping table (AX.25 PID → 4-bit IL2P PID) */
/* NOTE: PID 0x0 = S-frame (no PID), 0x1 = U-frame (no PID except UI) */
#define IL2P_PID_S_FRAME      0x0u
#define IL2P_PID_U_FRAME      0x1u
#define IL2P_PID_AX25_L3      0x2u  /* 0xCF/0xCE pattern */
#define IL2P_PID_X25_PLP      0x3u  /* 0x01 */
#define IL2P_PID_COMP_TCP     0x4u  /* 0x06 */
#define IL2P_PID_UNCOMP_TCP   0x5u  /* 0x07 */
#define IL2P_PID_SEGMENT      0x6u  /* 0x08 */
#define IL2P_PID_ARPA_IP      0xBu  /* 0xCC */
#define IL2P_PID_ARPA_ARP     0xCu  /* 0xCD */
#define IL2P_PID_FLEXNET      0xDu  /* 0xCE */
#define IL2P_PID_THENET       0xEu  /* 0xCF */
#define IL2P_PID_NO_L3        0xFu  /* 0xF0 */

/**
 * @brief Parsed representation of an IL2P Type 1 header (before bit-packing).
 */
typedef struct {
    uint8_t  dest_callsign[6];  /**< SIXBIT-encoded destination callsign */
    uint8_t  dest_ssid;         /**< Destination SSID (4 bits, 0..15) */
    uint8_t  src_callsign[6];   /**< SIXBIT-encoded source callsign */
    uint8_t  src_ssid;          /**< Source SSID (4 bits, 0..15) */
    uint8_t  ui;                /**< 1 if UI frame (PID field present in U-frame) */
    uint8_t  pid;               /**< IL2P 4-bit PID code */
    uint8_t  control;           /**< 7-bit translated control subfield */
    uint8_t  hdr_type;          /**< Header type: IL2P_HDR_TYPE_0 or IL2P_HDR_TYPE_1 */
    uint16_t payload_byte_count;/**< Total payload data bytes (0..1023) */
} il2p_header_t;

/**
 * @brief Map AX.25 8-bit PID to 4-bit IL2P PID.
 *
 * @param ax25_pid  AX.25 PID byte.
 * @param is_s_frame  True if this is an S-frame (no AX.25 PID byte).
 * @param is_u_frame  True if this is a U-frame (no PID unless UI).
 * @return IL2P 4-bit PID, or 0xFF if not mappable (force Type 0).
 */
uint8_t il2p_pid_from_ax25(uint8_t ax25_pid, bool is_s_frame, bool is_u_frame);

/**
 * @brief Map IL2P 4-bit PID back to AX.25 8-bit PID.
 *
 * @param il2p_pid  4-bit IL2P PID.
 * @return AX.25 PID byte, or 0xFF if the PID indicates S/U-frame (no PID).
 */
uint8_t il2p_pid_to_ax25(uint8_t il2p_pid);

/**
 * @brief Encode IL2P header to 13 bytes.
 *
 * @param hdr    Populated header structure.
 * @param out    Output buffer of at least IL2P_HEADER_SIZE bytes.
 * @return       IL2P_HEADER_SIZE (13) on success, 0 on error.
 */
uint8_t il2p_header_encode(const il2p_header_t *hdr, uint8_t out[IL2P_HEADER_SIZE]);

/**
 * @brief Decode 13 bytes into an IL2P header structure.
 *
 * @param in     Input buffer of exactly IL2P_HEADER_SIZE bytes.
 * @param hdr    Output header structure.
 * @return       true on success.
 */
bool il2p_header_decode(const uint8_t in[IL2P_HEADER_SIZE], il2p_header_t *hdr);

/**
 * @brief Build an IL2P header from an AX.25 frame.
 *
 * Inspects the AX.25 frame and selects Type 0 or Type 1 automatically.
 * For Type 1, extracts and translates all fields.
 * Sets hdr->hdr_type to indicate which type was chosen.
 *
 * @param frame          Parsed AX.25 frame.
 * @param payload_len    Length of payload data to follow header (bytes).
 * @param hdr            Output IL2P header.
 * @return               true if header was built, false on error.
 */
bool il2p_header_from_ax25(const ax25_frame_t *frame, uint16_t payload_len,
                             il2p_header_t *hdr);

/**
 * @brief Reconstruct an AX.25 frame header from an IL2P Type 1 header.
 *
 * @param hdr    Decoded IL2P Type 1 header.
 * @param frame  Output AX.25 frame (address/control fields populated).
 * @return       true on success.
 */
bool il2p_header_to_ax25(const il2p_header_t *hdr, ax25_frame_t *frame);

/**
 * @brief Compute the number of payload RS blocks for a given payload size.
 */
uint16_t il2p_payload_block_count(uint16_t payload_byte_count);

/**
 * @brief Compute block sizes for payload segmentation.
 *
 * @param payload_byte_count  Total payload bytes.
 * @param num_blocks          Output: total number of blocks.
 * @param large_block_size    Output: data bytes in a "large" block.
 * @param small_block_size    Output: data bytes in a "small" block.
 * @param large_block_count   Output: number of large blocks.
 * @param small_block_count   Output: number of small blocks.
 */
void il2p_payload_block_sizes(uint16_t payload_byte_count,
                               uint16_t *num_blocks,
                               uint8_t  *large_block_size,
                               uint8_t  *small_block_size,
                               uint16_t *large_block_count,
                               uint16_t *small_block_count);

#endif /* IL2P_HEADER_H */
