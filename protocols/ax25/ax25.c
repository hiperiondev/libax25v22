/**
 * @file ax25.c
 * @brief AX.25 v2.2 Protocol Library - Core Frame Structures and Functions
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
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "hal.h"
#include "ax25.h"
#include "common.h"

// Default XID Parameters
ax25_xid_parameter_t *AX25_20_DEFAULT_XID_COP = NULL;
ax25_xid_parameter_t *AX25_22_DEFAULT_XID_COP = NULL;
ax25_xid_parameter_t *AX25_20_DEFAULT_XID_HDLCOPTFUNC = NULL;
ax25_xid_parameter_t *AX25_22_DEFAULT_XID_HDLCOPTFUNC = NULL;
ax25_xid_parameter_t *AX25_20_DEFAULT_XID_IFIELDRX = NULL;
ax25_xid_parameter_t *AX25_22_DEFAULT_XID_IFIELDRX = NULL;
ax25_xid_parameter_t *AX25_20_DEFAULT_XID_WINDOWSZRX = NULL;
ax25_xid_parameter_t *AX25_22_DEFAULT_XID_WINDOWSZRX = NULL;
ax25_xid_parameter_t *AX25_20_DEFAULT_XID_ACKTIMER = NULL;
ax25_xid_parameter_t *AX25_22_DEFAULT_XID_ACKTIMER = NULL;
ax25_xid_parameter_t *AX25_20_DEFAULT_XID_RETRIES = NULL;
ax25_xid_parameter_t *AX25_22_DEFAULT_XID_RETRIES = NULL;

// Helper function for robust SSID parsing
// Parse SSID manually to avoid strtol() issues with negative numbers
// Returns -1 on error, otherwise returns SSID value (0-15)
static int parse_ssid(const char *str, size_t max_len, uint8_t *err) {
    *err = 0;
    int ssid = 0;
    size_t pos = 0;

    if (!str || max_len == 0) {
        *err = 1;
        return -1;
    }

    // Parse digits manually - reject non-digit characters and negative signs
    while (pos < max_len && str[pos] >= '0' && str[pos] <= '9') {
        ssid = ssid * 10 + (str[pos] - '0');
        if (ssid > 15) {  // Reject immediately if out of range
            *err = 1;
            return -1;
        }
        pos++;
    }

    // If we didn't parse any digits, it's an error
    if (pos == 0) {
        *err = 1;
        return -1;
    }

    return ssid;
}

static uint8_t* uint_encode(uint32_t value, bool big_endian, size_t length, size_t *out_len, uint8_t *err) {
    *err = 0;

    uint8_t *bytes = (uint8_t*) hal_mem_alloc((uint16_t) (length));
    if (!bytes) {
        *err = 1;
        return NULL;
    }

    for (size_t i = 0; i < length; i++) {
        bytes[big_endian ? length - 1 - i : i] = (value >> (i * 8)) & 0xFF;
    }

    *out_len = length;
    return bytes;
}

static uint32_t uint_decode(const uint8_t *data, size_t len, bool big_endian, uint8_t *err) {
    *err = 0;
    uint32_t value = 0;

    for (size_t i = 0; i < len; i++) {
        uint32_t byte = (uint32_t) data[big_endian ? (len - 1u - i) : i];
        value |= byte << (i * 8u);
    }

    return value;
}

ax25_address_t* ax25_address_decode(const uint8_t *data, uint8_t *err) {
    *err = 0;
    if (data == NULL) {
        *err = 2;
        return NULL;
    }
    ax25_address_t *addr = (ax25_address_t*) hal_mem_calloc((uint16_t) sizeof(ax25_address_t));
    if (!addr) {
        *err = 1;
        return NULL;
    }
    for (int i = 0; i < 6; i++) {
        addr->callsign[i] = (data[i] >> 1) & 0x7F;
    }
    addr->callsign[6] = '\0';
    addr->ssid = (data[6] & 0x1E) >> 1;
    addr->ch = (data[6] & 0x80) != 0;
    addr->res0 = (data[6] & 0x20) != 0;
    addr->res1 = (data[6] & 0x40) != 0;
    addr->mod8_legacy = addr->res1;
    addr->extension = (data[6] & 0x01) != 0;
    return addr;
}

ax25_address_t* ax25_address_from_string(const char *str, uint8_t *err) {
    *err = 0;
    if (str == NULL) {
        *err = 2;
        return NULL;
    }

    size_t total_len = strlen(str);

    // Check for empty string - error code 4
    if (total_len == 0) {
        *err = 4;
        return NULL;
    }

    if (total_len > 11) {  // Allows "REPEATER-1*" exactly
        *err = 4;
        return NULL;
    }

    ax25_address_t *addr = (ax25_address_t*) hal_mem_calloc((uint16_t) sizeof(ax25_address_t));
    if (!addr) {
        *err = 1;
        return NULL;
    }

    // Initialize all fields to prevent use of uninitialized memory
    memset(addr, 0, sizeof(ax25_address_t));
    addr->res0 = true;
    addr->res1 = true;
    addr->extension = false;

    char callsign[7] = { 0 };
    int ssid = 0;
    bool ch = false;

    const char *dash = strchr(str, '-');
    if (dash) {
        size_t callsign_len = dash - str;
        if (callsign_len == 0) {
            *err = 4;
            goto cleanup;
        }
        if (callsign_len > 6) {
            callsign_len = 6;
        }

        // Check for asterisk in callsign part (before dash) - error code 6
        for (size_t i = 0; i < callsign_len; i++) {
            if (str[i] == '*') {
                *err = 6;  // Asterisk not at the end
                goto cleanup;
            }
        }

        strncpy(callsign, str, callsign_len);
        callsign[callsign_len] = '\0';

        const char *ssid_str = dash + 1;
        size_t ssid_len = strlen(ssid_str);
        if (ssid_len == 0) {
            *err = 4;
            goto cleanup;
        }

        // Count digits before '*' or end of string
        const char *p = ssid_str;
        while (*p && isdigit((unsigned char )*p))
            p++;
        if (*p == '*') {
            ch = true;
            p++;
        }
        if (*p != '\0') {
            *err = 5;
            goto cleanup;
        }

        // Calculate number of digits to parse
        size_t digit_count = (ch ? (p - ssid_str - 1) : (p - ssid_str));
        ssid = parse_ssid(ssid_str, digit_count, err);
        if (*err != 0) {
            // parse_ssid returns error 1 for non-numeric, but we want error 4 for invalid SSID range
            *err = 4;  // Use error code 4 for any SSID parsing failure
            goto cleanup;
        }

        // Validate SSID value per AX.25 v2.2 Section 3.12.2
        // SSID must be in range 0-15 (4-bit field)
        if (!ax25_validate_ssid(ssid)) {
            *err = 4;  // Error code 4 for invalid SSID value (out of range)
            goto cleanup;
        }
    } else {
        // No dash found - entire string is callsign (no SSID)
        size_t callsign_len = total_len;

        // Check for asterisk in callsign-only string - error code 6
        for (size_t i = 0; i < callsign_len; i++) {
            if (str[i] == '*') {
                *err = 6;  // Asterisk not at the end (no SSID present)
                goto cleanup;
            }
        }

        if (callsign_len > 6) {
            callsign_len = 6;
        }
        memcpy(callsign, str, callsign_len);
        callsign[callsign_len] = '\0';
    }

    // Copy to address structure
    for (int i = 0; i < 6; i++) {
        addr->callsign[i] = (i < (int) strlen(callsign)) ? callsign[i] : ' ';
    }
    addr->callsign[6] = '\0';
    addr->ssid = ssid;
    addr->ch = ch;

    // Trim trailing spaces from callsign for consistency
    trim_trailing_spaces(addr->callsign);

    return addr;

    cleanup:
    if (addr != NULL) {
        hal_mem_free(addr);
        addr = NULL;
    }

    return NULL;
}

uint8_t* ax25_address_encode(const ax25_address_t *addr, size_t *len, uint8_t *err) {
    *err = 0;
    if (addr == NULL) {
        *err = 2;
        return NULL;
    }
    uint8_t *data = (uint8_t*) hal_mem_alloc(7u);
    if (!data) {
        *err = 1;
        return NULL;
    }

    // Always encode 6 bytes, padding with spaces if callsign is shorter
    size_t callsign_len = strlen(addr->callsign);
    for (int i = 0; i < 6; i++) {
        char c = (i < callsign_len) ? addr->callsign[i] : ' ';
        data[i] = (c & 0x7F) << 1;
    }

    uint8_t ssid_byte = (addr->ssid << 1) & 0x1E;
    if (addr->ch)
        ssid_byte |= 0x80;
    /* Spec §3.12.2: reserved bit 5 (res0) MUST always be 1.
     * Bit 6 (res1) is intentionally caller-controlled: ax25_frame_encode()
     * clears it on the source address to signal Modulo-128 mode per the
     * PE1CHL extension; real-world packets also carry res1=0 for modulo-8
     * frames, so we preserve the caller's intent here rather than forcing 1.
     * Bit 7 is the C/H bit, set above.
     */
    ssid_byte |= 0x20; /* res0 (bit 5) forced to 1 per §3.12.2 */
    if (addr->res1)
        ssid_byte |= 0x40; /* res1 (bit 6): caller-controlled (mod-128 signal) */
    if (addr->extension)
        ssid_byte |= 0x01;
    data[6] = ssid_byte;
    *len = 7;
    return data;
}

ax25_address_t* ax25_address_copy(const ax25_address_t *addr, uint8_t *err) {
    *err = 0;
    ax25_address_t *copy = (ax25_address_t*) hal_mem_calloc((uint16_t) sizeof(ax25_address_t));

    if (!copy) {
        *err = 1;
        return NULL;
    }
    memcpy(copy, addr, sizeof(ax25_address_t));

    return copy;
}

void ax25_address_free(ax25_address_t *addr, uint8_t *err) {
    hal_mem_free(addr);
}

ax25_path_t* ax25_path_new(ax25_address_t **repeaters, int num, uint8_t *err) {
    *err = 0;

    // Validate input parameters
    if (repeaters == NULL || num <= 0 || num > AX25_MAX_REPEATERS) {
        *err = 2;  // Invalid input
        return NULL;
    }

    // Check for NULL pointers in the repeaters array
    for (int i = 0; i < num; i++) {
        if (repeaters[i] == NULL) {
            *err = 2;  // Invalid repeater address
            return NULL;
        }
    }

    // Allocate memory for the path
    ax25_path_t *path = (ax25_path_t*) hal_mem_calloc((uint16_t) sizeof(ax25_path_t));
    if (!path) {
        *err = 1;  // Memory allocation failure
        return NULL;
    }

    // Initialize number of repeaters
    path->num_repeaters = num;

    // Copy repeater addresses
    for (int i = 0; i < num; i++) {
        ax25_address_t *copy = ax25_address_copy(repeaters[i], err);
        if (!copy) {
            // Free previously allocated copies and path on failure
            for (int j = 0; j < i; j++) {
                ax25_address_free(&path->repeaters[j], err);
            }
            hal_mem_free(path);
            return NULL;
        }
        path->repeaters[i] = *copy;
        hal_mem_free(copy);  // Free the temporary pointer after copying
    }

    return path;
}

void ax25_path_free(ax25_path_t *path, uint8_t *err) {
    hal_mem_free(path);
}

header_decode_result_t ax25_frame_header_decode(const uint8_t *data, size_t len, uint8_t *err) {
    *err = 0;
    header_decode_result_t result = { NULL, data, len };

    ax25_address_t addresses[2 + AX25_MAX_REPEATERS];
    int addr_count = 0;
    size_t pos = 0;

    // Parse addresses until extension bit is set or max repeaters reached
    while (pos + 7 <= len && addr_count < 2 + AX25_MAX_REPEATERS) {
        // Decode address directly into stack array
        uint8_t *addr_data = (uint8_t*) (data + pos);

        for (int i = 0; i < 6; i++) {
            addresses[addr_count].callsign[i] = (addr_data[i] >> 1) & 0x7F;
        }
        addresses[addr_count].callsign[6] = '\0';
        addresses[addr_count].ssid = (addr_data[6] & 0x1E) >> 1;

        // Validate SSID value per AX.25 v2.2 Section 3.12.2
        // SSID is a 4-bit field and must be in range 0-15
        if (!ax25_validate_ssid(addresses[addr_count].ssid)) {
            *err = 7;  // Invalid SSID value
            return result;
        }

        addresses[addr_count].ch = (addr_data[6] & 0x80) != 0;
        addresses[addr_count].res1 = (addr_data[6] & 0x40) != 0;
        addresses[addr_count].res0 = (addr_data[6] & 0x20) != 0;
        addresses[addr_count].extension = (addr_data[6] & 0x01) != 0;

        pos += 7;
        addr_count++;

        // Continue parsing if fewer than 2 addresses or extension bit is 0
        if (addr_count >= 2 && addresses[addr_count - 1].extension)
            break;
    }

    // Check for minimum address requirement (destination + source)
    if (addr_count < 2) {
        *err = 4;  // Too few addresses
        return result;
    }

    // Check if last address has extension bit set (required for termination)
    if (!addresses[addr_count - 1].extension) {
        *err = 5;  // Last address doesn't have extension bit set
        return result;
    }

    // Allocate header
    ax25_frame_header_t *header = (ax25_frame_header_t*) hal_mem_calloc((uint16_t) sizeof(ax25_frame_header_t));
    if (!header) {
        *err = 6;  // Memory allocation failure
        return result;
    }

    // Populate header fields from stack-allocated addresses
    header->destination = addresses[0];
    header->source = addresses[1];
    header->cr = (header->destination.ch && !header->source.ch);
    header->src_cr = header->source.ch;
    header->repeaters.num_repeaters = addr_count - 2;
    for (int i = 0; i < header->repeaters.num_repeaters; i++) {
        header->repeaters.repeaters[i] = addresses[i + 2];
    }

    result.header = header;
    result.remaining = data + pos;
    result.remaining_len = len - pos;

    return result;
}

uint8_t* ax25_frame_header_encode(const ax25_frame_header_t *header, size_t *len, uint8_t *err) {
    *err = 0;
    size_t total_len = 7 * (2 + header->repeaters.num_repeaters);
    uint8_t *bytes = (uint8_t*) hal_mem_alloc((uint16_t) (total_len));
    if (!bytes) {
        *err = 1;
        return NULL;
    }

    size_t offset = 0;
    ax25_address_t dest = header->destination;
    dest.extension = false;
    ax25_address_t src = header->source;
    src.extension = (header->repeaters.num_repeaters == 0);

    /* AX.25 v2.2 §3.12: derive the C/H bits from the frame's command/response
     * designation rather than trusting whatever ch values happen to be in the
     * address structs.  This is the most commonly mis-implemented detail in AX.25:
     *   Command frame  → dest.ch = 1, src.ch = 0
     *   Response frame → dest.ch = 0, src.ch = 1
     *
     * We apply the correction only when the CR flags form a valid command/response
     * pair (cr XOR src_cr == 1).  When both flags are 0 — which occurs with
     * non-conforming or legacy on-air frames decoded verbatim — we leave the ch
     * bits from the address structs unchanged so that decode→encode round-trips
     * remain byte-for-byte faithful even for malformed packets.
     */
    if (header->cr && !header->src_cr) {
        /* Command frame */
        dest.ch = true;
        src.ch = false;
    } else if (!header->cr && header->src_cr) {
        /* Response frame */
        dest.ch = false;
        src.ch = true;
    }
    /* else: both zero (non-conforming) — preserve address struct ch bits */
    size_t dest_len;
    uint8_t *dest_bytes = ax25_address_encode(&dest, &dest_len, err);
    memcpy(bytes + offset, dest_bytes, dest_len);
    offset += dest_len;
    hal_mem_free(dest_bytes);

    size_t src_len;
    uint8_t *src_bytes = ax25_address_encode(&src, &src_len, err);
    memcpy(bytes + offset, src_bytes, src_len);
    offset += src_len;
    hal_mem_free(src_bytes);

    for (int i = 0; i < header->repeaters.num_repeaters; i++) {
        ax25_address_t rpt = header->repeaters.repeaters[i];
        rpt.extension = (i == header->repeaters.num_repeaters - 1);
        size_t rpt_len;
        uint8_t *rpt_bytes = ax25_address_encode(&rpt, &rpt_len, err);
        memcpy(bytes + offset, rpt_bytes, rpt_len);
        offset += rpt_len;
        hal_mem_free(rpt_bytes);
    }

    *len = total_len;
    return bytes;
}

void ax25_frame_header_free(ax25_frame_header_t *header, uint8_t *err) {
    hal_mem_free(header);
}

ax25_frame_t* ax25_frame_decode(const uint8_t *data, size_t len, int modulo128, uint8_t *err) {
    *err = 0;

    // Validate minimum frame size per AX.25 v2.2 Section 3.9
    // Note: Input frames to this function do NOT include FCS
    // Minimum without FCS: Dest(7) + Source(7) + Control(1) = 15 bytes
    if (len < AX25_MIN_FRAME_SIZE_NO_FCS) {
        *err = 1;  // Frame too short - does not meet minimum size requirement
        return NULL;
    }

    // Validate address field structure before attempting to decode
    // This checks extension bits, field alignment, and address count
    size_t validated_addr_len = 0;
    if (!ax25_validate_address_field(data, len, &validated_addr_len)) {
        *err = 5;  // Invalid address field structure - error code 5 per test expectations
        return NULL;
    }

    header_decode_result_t hdr_result = ax25_frame_header_decode(data, len, err);
    if (!hdr_result.header) {
        return NULL;  // Error is already set by ax25_frame_header_decode
    }

    if (hdr_result.remaining_len == 0) {
        *err = 3;
        ax25_frame_header_free(hdr_result.header, err);
        return NULL;
    }

    uint8_t control = hdr_result.remaining[0];
    ax25_frame_t *frame = NULL;

    if ((control & CONTROL_US_MASK) == CONTROL_U_VAL) {
        frame = (ax25_frame_t*) ax25_unnumbered_frame_decode(hdr_result.header, control, hdr_result.remaining + 1, hdr_result.remaining_len - 1, err);
        // If unnumbered frame decode failed with invalid control, propagate error code 6
        if (!frame && *err == 1) {
            *err = 6;  // Invalid control field for unnumbered frame
        }
    } else {
        if (modulo128 == MODULO128_NONE) {
            if (hdr_result.remaining_len < 1) {
                // Error: no control byte
                *err = 4;
                ax25_frame_header_free(hdr_result.header, err);
                return NULL;
            }
            ax25_raw_frame_t *raw = hal_mem_alloc((uint16_t) (sizeof(ax25_raw_frame_t)));
            if (!raw) {
                *err = 4;
                ax25_frame_header_free(hdr_result.header, err);
                return NULL;
            }
            raw->base.type = AX25_FRAME_RAW;
            raw->base.header = *hdr_result.header;
            raw->control = hdr_result.remaining[0];
            raw->payload_len = hdr_result.remaining_len - 1;
            raw->payload = hal_mem_alloc((uint16_t) (raw->payload_len));
            if (!raw->payload) {
                *err = 5;
                hal_mem_free(raw);
                ax25_frame_header_free(hdr_result.header, err);
                return NULL;
            }
            memcpy(raw->payload, hdr_result.remaining + 1, raw->payload_len);
            frame = (ax25_frame_t*) raw;
        } else {
            bool is_16bit;
            // MODULO128_AUTO (2): auto-detect via res1 bit in source SSID byte
            //   (res1=false signals modulo-128 / 16-bit control field)
            // MODULO128_FALSE (0): explicitly force 8-bit (modulo-8) control field
            //   Real-world packets may have res1=0 even for modulo-8 frames so
            //   auto-detection cannot be relied upon when the caller already
            //   knows the frame uses 8-bit sequence numbering.
            // MODULO128_TRUE (1): explicitly force modulo-128 (16-bit control)
            if (modulo128 == MODULO128_AUTO) {
                // Auto-detect only when caller requests it: res1=false means 16-bit
                is_16bit = !hdr_result.header->source.res1;
            } else {
                // MODULO128_TRUE forces 16-bit
                is_16bit = (modulo128 == MODULO128_TRUE);
                // When MODULO128_FALSE (0) is used, supervisory frames can still be
                // identified as 16-bit by frame length: S-frames have no info field,
                // so exactly 2 remaining bytes after the address field unambiguously
                // means 2 control bytes (16-bit supervisory). This allows callers that
                // pass 0 (e.g. count_sframes_of_type in tests) to correctly detect
                // SREJ_16BIT and other 16-bit S-frame types without affecting I-frame
                // decoding, which always has more than 2 bytes remaining (ctrl+PID+data).
                if (!is_16bit && hdr_result.remaining_len == 2) {
                    uint8_t first_ctrl = hdr_result.remaining[0];
                    if ((first_ctrl & CONTROL_US_MASK) == CONTROL_S_VAL) {
                        // Exactly 2 bytes remaining with S-frame indicator: 16-bit S-frame
                        is_16bit = true;
                    }
                }
            }
            size_t control_size = is_16bit ? 2 : 1;
            if (hdr_result.remaining_len < control_size) {
                *err = 6;
                ax25_frame_header_free(hdr_result.header, err);
                return NULL;
            }
            uint16_t full_control = control;
            if (is_16bit)
                full_control |= (uint16_t) ((uint16_t) hdr_result.remaining[1] << 8u);

            const uint8_t *data_start = hdr_result.remaining + control_size;
            size_t data_len = hdr_result.remaining_len - control_size;

            if ((full_control & CONTROL_I_MASK) == CONTROL_I_VAL) {
                frame = (ax25_frame_t*) ax25_information_frame_decode(hdr_result.header, full_control, data_start, data_len, is_16bit, err);
            } else if ((full_control & CONTROL_US_MASK) == CONTROL_S_VAL) {
                frame = (ax25_frame_t*) ax25_supervisory_frame_decode(hdr_result.header, full_control, is_16bit, err);
            }
        }
    }

    ax25_frame_header_free(hdr_result.header, err);

    return frame;
}

uint8_t* ax25_frame_encode(const ax25_frame_t *frame, size_t *len, uint8_t *err) {
    *err = 0;

    // Determine if it's a modulo-128 frame
    bool is_modulo128 = (frame->type == AX25_FRAME_INFORMATION_16BIT || frame->type == AX25_FRAME_SUPERVISORY_RR_16BIT
            || frame->type == AX25_FRAME_SUPERVISORY_RNR_16BIT || frame->type == AX25_FRAME_SUPERVISORY_REJ_16BIT
            || frame->type == AX25_FRAME_SUPERVISORY_SREJ_16BIT || frame->type == AX25_FRAME_UNNUMBERED_SABME);

    // Create a copy of the header
    ax25_frame_header_t header_copy = frame->header;
    // For modulo-128 frames: clear res1 to signal 16-bit mode per AX.25 v2.2 Section 3.12.
    // For modulo-8 frames: preserve the original res1 value from the header so that
    // round-trip decode->encode is faithful. Real-world AX.25 packets (e.g. from TNC or
    // radio) may carry res1=0 even for modulo-8 frames; overriding res1 to 1 would corrupt
    // such frames on re-encoding. Callers that construct addresses themselves (e.g. ax25_connect)
    // already set res1=true on the peer address when appropriate.
    if (is_modulo128) {
        // Clear res1 to signal modulo-128 mode per AX.25 v2.2 Section 3.12
        header_copy.source.res1 = false;
    }

    size_t header_len;
    uint8_t *header_bytes = ax25_frame_header_encode(&header_copy, &header_len, err);
    if (!header_bytes) {
        *err = 1;
        return NULL;
    }

    uint8_t *payload_bytes = NULL;
    size_t payload_len;
    switch (frame->type) {
        case AX25_FRAME_RAW:
            payload_bytes = ax25_raw_frame_encode((ax25_raw_frame_t*) frame, &payload_len, err);
        break;
        case AX25_FRAME_UNNUMBERED_INFORMATION:
            payload_bytes = ax25_unnumbered_information_frame_encode((ax25_unnumbered_information_frame_t*) frame, &payload_len, err);
        break;
        case AX25_FRAME_UNNUMBERED_SABM:
        case AX25_FRAME_UNNUMBERED_SABME:
        case AX25_FRAME_UNNUMBERED_DISC:
        case AX25_FRAME_UNNUMBERED_DM:
        case AX25_FRAME_UNNUMBERED_UA:
            payload_bytes = ax25_unnumbered_frame_encode((ax25_unnumbered_frame_t*) frame, &payload_len, err);
        break;
        case AX25_FRAME_UNNUMBERED_FRMR:
            payload_bytes = ax25_frame_reject_frame_encode((ax25_frame_reject_frame_t*) frame, &payload_len, err);
        break;
        case AX25_FRAME_UNNUMBERED_XID:
            payload_bytes = ax25_exchange_identification_frame_encode((ax25_exchange_identification_frame_t*) frame, &payload_len, err);
        break;
        case AX25_FRAME_UNNUMBERED_TEST:
            payload_bytes = ax25_test_frame_encode((ax25_test_frame_t*) frame, &payload_len, err);
        break;
        case AX25_FRAME_INFORMATION_8BIT:
        case AX25_FRAME_INFORMATION_16BIT:
            payload_bytes = ax25_information_frame_encode((ax25_information_frame_t*) frame, &payload_len, err);
        break;
        case AX25_FRAME_SUPERVISORY_RR_8BIT:
        case AX25_FRAME_SUPERVISORY_RNR_8BIT:
        case AX25_FRAME_SUPERVISORY_REJ_8BIT:
        case AX25_FRAME_SUPERVISORY_SREJ_8BIT:
        case AX25_FRAME_SUPERVISORY_RR_16BIT:
        case AX25_FRAME_SUPERVISORY_RNR_16BIT:
        case AX25_FRAME_SUPERVISORY_REJ_16BIT:
        case AX25_FRAME_SUPERVISORY_SREJ_16BIT:
            payload_bytes = ax25_supervisory_frame_encode((ax25_supervisory_frame_t*) frame, &payload_len, err);
        break;
        default:
            *err = 2;
            hal_mem_free(header_bytes);
            return NULL;
    }

    if (!payload_bytes) {
        *err = 3;
        hal_mem_free(header_bytes);
        return NULL;
    }

    *len = header_len + payload_len;
    uint8_t *result = hal_mem_alloc((uint16_t) (*len));
    if (!result) {
        *err = 4;
        hal_mem_free(header_bytes);
        hal_mem_free(payload_bytes);
        return NULL;
    }

    memcpy(result, header_bytes, header_len);
    memcpy(result + header_len, payload_bytes, payload_len);
    hal_mem_free(header_bytes);
    hal_mem_free(payload_bytes);

    return result;
}

// AX25_BUF_FROM_DATA: recover the ax25_buf_t* owning a payload data pointer.
// All pool-backed payloads are assigned as frame->payload = buf->data, so the
// owning slot can be found by subtracting the offsetof(ax25_buf_t, data).
// This avoids the broken (ax25_buf_t*)(payload) cast used previously, which
// produced a wrong pointer and silently leaked the pool slot.
#define AX25_BUF_FROM_DATA(ptr)     ((ax25_buf_t *)((uint8_t *)(ptr) - offsetof(ax25_buf_t, data)))

void ax25_frame_free(ax25_frame_t *frame, uint8_t *err) {
    *err = 0;
    uint8_t *p = NULL;

    if (!frame) {
        *err = 1;
        return;
    }

    switch (frame->type) {
        case AX25_FRAME_RAW:
            hal_mem_free(((ax25_raw_frame_t*) frame)->payload);
        break;
        case AX25_FRAME_UNNUMBERED_INFORMATION:
            p = ((ax25_unnumbered_information_frame_t*) frame)->payload;
            if (p)
                hal_mem_free(p);
        break;
        case AX25_FRAME_UNNUMBERED_XID: {
            ax25_exchange_identification_frame_t *xid = (ax25_exchange_identification_frame_t*) frame;
            for (size_t i = 0; i < xid->param_count; i++) {
                xid->parameters[i]->free(xid->parameters[i], err);
            }
            // free the parameters pointer array allocated by realloc in the decoder
            hal_mem_free(xid->parameters);
            break;
        }
        case AX25_FRAME_UNNUMBERED_TEST:
            // payload = pool buf->data; calling hal_mem_free() on it is undefined behaviour
            // because it points into the static ax25_pool array, not the heap.
            // Recover the owning ax25_buf_t* and release via ax25_buf_free instead.
            p = ((ax25_test_frame_t*) frame)->payload;
            if (p)
                ax25_buf_free(AX25_BUF_FROM_DATA(p));
        break;
        case AX25_FRAME_INFORMATION_8BIT:
        case AX25_FRAME_INFORMATION_16BIT:
            // same as UI/TEST: payload is buf->data; recover owning slot via AX25_BUF_FROM_DATA
            p = ((ax25_information_frame_t*) frame)->payload;
            if (p)
                ax25_buf_free(AX25_BUF_FROM_DATA(p));
        break;
        default:
        break;
    }

    hal_mem_free(frame);
}

ax25_frame_t* ax25_frame_create(ax25_frame_type_t type, const ax25_frame_header_t *header, uint8_t *err) {
    *err = 0;

    if (!header) {
        *err = 1;  // Invalid header
        return NULL;
    }

    ax25_frame_t *frame = NULL;

    switch (type) {
        case AX25_FRAME_INFORMATION_8BIT:
        case AX25_FRAME_INFORMATION_16BIT: {
            ax25_information_frame_t *iframe = hal_mem_alloc((uint16_t) (sizeof(ax25_information_frame_t)));
            if (!iframe) {
                *err = 2;
                return NULL;
            }
            iframe->base.type = type;
            iframe->base.header = *header;
            iframe->nr = 0;
            iframe->ns = 0;
            iframe->pf = false;
            iframe->pid = 0;
            iframe->payload = NULL;
            iframe->payload_len = 0;
            frame = (ax25_frame_t*) iframe;
            break;
        }

        case AX25_FRAME_SUPERVISORY_RR_8BIT:
        case AX25_FRAME_SUPERVISORY_RNR_8BIT:
        case AX25_FRAME_SUPERVISORY_REJ_8BIT:
        case AX25_FRAME_SUPERVISORY_SREJ_8BIT:
        case AX25_FRAME_SUPERVISORY_RR_16BIT:
        case AX25_FRAME_SUPERVISORY_RNR_16BIT:
        case AX25_FRAME_SUPERVISORY_REJ_16BIT:
        case AX25_FRAME_SUPERVISORY_SREJ_16BIT: {
            ax25_supervisory_frame_t *sframe = hal_mem_alloc((uint16_t) (sizeof(ax25_supervisory_frame_t)));
            if (!sframe) {
                *err = 2;
                return NULL;
            }
            sframe->base.type = type;
            sframe->base.header = *header;
            sframe->nr = 0;
            sframe->pf = false;
            sframe->code = 0;
            frame = (ax25_frame_t*) sframe;
            break;
        }

        case AX25_FRAME_UNNUMBERED_INFORMATION: {
            ax25_unnumbered_information_frame_t *uiframe = hal_mem_alloc((uint16_t) (sizeof(ax25_unnumbered_information_frame_t)));
            if (!uiframe) {
                *err = 2;
                return NULL;
            }
            uiframe->base.base.type = type;
            uiframe->base.base.header = *header;
            uiframe->base.pf = false;
            uiframe->base.modifier = 0x03;
            uiframe->pid = 0;
            uiframe->payload = NULL;
            uiframe->payload_len = 0;
            frame = (ax25_frame_t*) uiframe;
            break;
        }

        case AX25_FRAME_RAW: {
            ax25_raw_frame_t *rframe = hal_mem_alloc((uint16_t) (sizeof(ax25_raw_frame_t)));
            if (!rframe) {
                *err = 2;
                return NULL;
            }
            rframe->base.type = type;
            rframe->base.header = *header;
            rframe->control = 0;
            rframe->payload = NULL;
            rframe->payload_len = 0;
            frame = (ax25_frame_t*) rframe;
            break;
        }

        default:
            *err = 3;  // Unsupported frame type
            return NULL;
    }

    return frame;
}

uint8_t* ax25_raw_frame_encode(const ax25_raw_frame_t *frame, size_t *len, uint8_t *err) {
    *err = 0;
    *len = 1 + frame->payload_len;
    uint8_t *bytes = hal_mem_alloc((uint16_t) (*len));
    if (!bytes) {
        *err = 1;
        return NULL;
    }
    bytes[0] = frame->control;
    memcpy(bytes + 1, frame->payload, frame->payload_len);
    return bytes;
}

// Parses one or two control bytes into ax25_ctrl_t.  All output fields are
// zero-initialised before any branch so callers never read uninitialised memory.
// Bit layouts verified against ax25_information_frame_decode() and
// ax25_supervisory_frame_decode() in this file:
//   mod-8  I: ns=bits[3:1], pf=bit4, nr=bits[7:5]
//   mod-128 I: ns=c0 bits[7:1], pf=c1 bit0, nr=c1 bits[7:1]
//   mod-8  S: s_cmd=bits[3:2], pf=bit4, nr=bits[7:5]
//   mod-128 S: s_cmd=c0 bits[3:2], pf=c1 bit0, nr=c1 bits[7:1]
//   U (always 1 byte): pf=bit4, u_cmd = c0 & 0xEF (P/F stripped)
uint8_t ax25_parse_ctrl(ax25_ctrl_t *out, const uint8_t *ctrl, size_t avail, uint8_t mod128) {
    // Validate pointers and minimum buffer size
    if (!out || !ctrl)
        return 1u;
    if (avail < 1u)
        return 2u;

    // Zero all fields so unused ones are always deterministic
    out->type = 0u;
    out->s_cmd = 0u;
    out->u_cmd = 0u;
    out->pf = 0u;
    out->ns = 0u;
    out->nr = 0u;
    out->ctrl_len = 0u;

    uint8_t c0 = ctrl[0];

    if ((c0 & 0x01u) == 0u) {
        // I-frame: bit 0 of first byte is 0
        out->type = 'I';
        if (mod128) {
            // mod-128 requires a second control byte
            if (avail < 2u)
                return 2u;
            uint8_t c1 = ctrl[1];
            // N(S): bits 7-1 of c0 (7-bit sequence number, range 0-127)
            out->ns = (c0 >> 1) & 0x7Fu;
            // P/F: bit 0 of c1 (bit 8 of the 16-bit control word)
            // Confirmed: ax25_information_frame_decode uses (control & 0x0100)
            out->pf = (c1 >> 0) & 0x01u;
            // N(R): bits 7-1 of c1 (7-bit sequence number, range 0-127)
            // Confirmed: ax25_information_frame_decode uses (control >> 9) & 0x7F
            out->nr = (c1 >> 1) & 0x7Fu;
            out->ctrl_len = 2u;
        } else {
            // N(S): bits 3-1 of c0 (3-bit sequence number, range 0-7)
            out->ns = (c0 >> 1) & 0x07u;
            // P/F: bit 4 of c0 -- POLL_FINAL_8BIT = 0x10
            out->pf = (c0 >> 4) & 0x01u;
            // N(R): bits 7-5 of c0 (3-bit sequence number, range 0-7)
            out->nr = (c0 >> 5) & 0x07u;
            out->ctrl_len = 1u;
        }
    } else if ((c0 & 0x03u) == 0x01u) {
        // S-frame: bits 1-0 = 01
        out->type = 'S';
        // Supervisory code: bits 3-2 (0=RR, 1=RNR, 2=REJ, 3=SREJ)
        // Confirmed: ax25_supervisory_frame_decode uses (control >> 2) & 0x03
        out->s_cmd = (c0 >> 2) & 0x03u;
        if (mod128) {
            // mod-128 requires a second control byte
            if (avail < 2u)
                return 2u;
            uint8_t c1 = ctrl[1];
            // P/F: bit 0 of c1 -- POLL_FINAL_16BIT = 0x0100 (bit 8 of 16-bit word)
            out->pf = (c1 >> 0) & 0x01u;
            // N(R): bits 7-1 of c1
            out->nr = (c1 >> 1) & 0x7Fu;
            out->ctrl_len = 2u;
        } else {
            // P/F: bit 4 of c0 -- POLL_FINAL_8BIT = 0x10
            out->pf = (c0 >> 4) & 0x01u;
            // N(R): bits 7-5 of c0
            out->nr = (c0 >> 5) & 0x07u;
            out->ctrl_len = 1u;
        }
    } else {
        // U-frame: bits 1-0 = 11; always 1 byte regardless of mod128
        out->type = 'U';
        // P/F: bit 4 of c0 -- U-frames always use 8-bit control
        out->pf = (c0 >> 4) & 0x01u;
        // Strip P/F bit to get canonical opcode -- matches existing: modifier = control & 0xEF
        // Explicit cast to uint8_t prevents ~0x10u (uint32_t) from widening the expression
        out->u_cmd = (uint8_t) (c0 & (uint8_t) ~0x10u);
        out->ctrl_len = 1u;
    }

    return 0u;
}

ax25_unnumbered_frame_t* ax25_unnumbered_frame_decode(ax25_frame_header_t *header, uint8_t control, const uint8_t *data, size_t len, uint8_t *err) {
    *err = 0;
    uint8_t modifier = control & 0xEF;
    bool pf = (control & POLL_FINAL_8BIT) != 0;
    ax25_unnumbered_frame_t *result = NULL;

    switch (modifier) {
        case 0x03:  // UI
            result = (ax25_unnumbered_frame_t*) ax25_unnumbered_information_frame_decode(header, pf, data, len, err);
        break;
        case 0x87:  // FRMR
            result = (ax25_unnumbered_frame_t*) ax25_frame_reject_frame_decode(header, pf, data, len, err);
        break;
        case 0xAF:  // XID
            result = (ax25_unnumbered_frame_t*) ax25_exchange_identification_frame_decode(header, pf, data, len, err);
        break;
        case 0xE3:  // TEST
            result = (ax25_unnumbered_frame_t*) ax25_test_frame_decode(header, pf, data, len, err);
        break;
        case 0x2F:  // SABM
        case 0x6F:  // SABME
        case 0x43:  // DISC
        case 0x0F:  // DM
        case 0x63:  // UA
            result = hal_mem_alloc((uint16_t) (sizeof(ax25_unnumbered_frame_t)));
            if (!result) {
                *err = 6;
                return NULL;
            }
            result->base.type = (modifier == 0x2F) ? AX25_FRAME_UNNUMBERED_SABM : (modifier == 0x6F) ? AX25_FRAME_UNNUMBERED_SABME :
                                (modifier == 0x43) ? AX25_FRAME_UNNUMBERED_DISC : (modifier == 0x0F) ? AX25_FRAME_UNNUMBERED_DM : AX25_FRAME_UNNUMBERED_UA;
            result->base.header = *header;
            result->pf = pf;
            result->modifier = modifier;
        break;
        default:
            *err = 6;  // Invalid U-frame modifier
            return NULL;
    }

    return result;
}

uint8_t* ax25_unnumbered_frame_encode(const ax25_unnumbered_frame_t *frame, size_t *len, uint8_t *err) {
    *err = 0;
    uint8_t control = frame->modifier | (frame->pf ? POLL_FINAL_8BIT : 0);
    *len = 1;
    uint8_t *bytes = hal_mem_alloc((uint16_t) (1));

    if (!bytes) {
        *err = 1;
        return NULL;
    }

    bytes[0] = control;

    return bytes;
}

ax25_unnumbered_information_frame_t* ax25_unnumbered_information_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len,
        uint8_t *err) {
    if (len < 1) {  // Need at least PID byte
        *err = 1;
        return NULL;
    }

    ax25_unnumbered_information_frame_t *ui_frame = hal_mem_alloc((uint16_t) (sizeof(ax25_unnumbered_information_frame_t)));

    if (!ui_frame) {
        *err = 1;
        return NULL;
    }

    ui_frame->base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
    ui_frame->base.base.header = *header;
    ui_frame->base.pf = pf;
    ui_frame->base.modifier = 0x03;  // UI frame modifier
    ui_frame->pid = data[0];

    ui_frame->payload_len = len - 1;
    // UI frames are connectionless and may carry large datagrams (e.g. APRS, test payloads);
    // the pool is fixed at AX25_MAX_INFO bytes and cannot hold oversized payloads.
    // Free path in ax25_frame_free (AX25_FRAME_UNNUMBERED_INFORMATION case) uses hal_mem_free() directly.
    if (ui_frame->payload_len > 0) {
        ui_frame->payload = hal_mem_alloc((uint16_t) (ui_frame->payload_len + 1u));
        if (!ui_frame->payload) {
            *err = 1;
            hal_mem_free(ui_frame);
            return NULL;
        }
        memcpy(ui_frame->payload, data + 1, ui_frame->payload_len);
        ui_frame->payload[ui_frame->payload_len] = '\0';  // null terminate for text safety
    } else {
        ui_frame->payload = NULL;
    }

    ax25_buf_t *ui_buf = ax25_buf_alloc();
    if (!ui_buf) {
        // Pool exhausted - treat as allocation failure
        *err = 1;
        hal_mem_free(ui_frame);
        return NULL;
    }

    *err = 0;
    return ui_frame;
}

uint8_t* ax25_unnumbered_information_frame_encode(const ax25_unnumbered_information_frame_t *frame, size_t *len, uint8_t *err) {
    *err = 0;
    *len = 1 + 1 + frame->payload_len;
    uint8_t *bytes = hal_mem_alloc((uint16_t) (*len));

    if (!bytes) {
        *err = 1;
        return NULL;
    }

    bytes[0] = frame->base.modifier | (frame->base.pf ? POLL_FINAL_8BIT : 0);
    bytes[1] = frame->pid;
    memcpy(bytes + 2, frame->payload, frame->payload_len);

    return bytes;
}

ax25_frame_reject_frame_t* ax25_frame_reject_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len, uint8_t *err) {
    *err = 0;

    // Improved modulo type determination with explicit validation
    // According to AX.25 v2.2 Section 4.3.3.4:
    // Modulo-8 FRMR: 3 bytes of data (control + V(S)/V(R)/CR + flags)
    // Modulo-128 FRMR: 5 bytes of data (control-low + control-high + N(S)/CR + N(R) + flags)
    bool is_modulo128;

    if (len == 5) {
        is_modulo128 = true;
    } else if (len == 3) {
        is_modulo128 = false;
    } else {
        // Invalid FRMR data length
        *err = 1;
        return NULL;
    }

    // Allocate frame structure
    ax25_frame_reject_frame_t *frame = hal_mem_alloc((uint16_t) (sizeof(ax25_frame_reject_frame_t)));
    if (!frame) {
        *err = 2;  // Memory allocation failed
        return NULL;
    }

    // Initialize base fields
    frame->base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
    frame->base.base.header = *header;
    frame->base.pf = pf;
    frame->base.modifier = 0x87;  // FRMR control byte
    frame->is_modulo128 = is_modulo128;

    if (is_modulo128) {
        // Parse 5-byte modulo-128 FRMR data field with explicit comments
        // Byte 0: Control field low byte (bits 0-7 of 16-bit control)
        // Byte 1: Control field high byte (bits 8-15 of 16-bit control)
        // This is the control field of the frame that was rejected
        frame->frmr_control = (uint16_t) ((uint16_t) data[0] | ((uint16_t) data[1] << 8u));

        // Byte 2: N(S) in bits 1-7 (7 bits), CR bit in bit 0
        // N(S) is the send sequence number of the rejected frame
        frame->vs = (data[2] >> 1) & 0x7F;
        frame->frmr_cr = (data[2] & 0x01) != 0;

        // Byte 3: N(R) in bits 1-7 (7 bits)
        // N(R) is the receive sequence number of the rejected frame
        frame->vr = (data[3] >> 1) & 0x7F;

        // Byte 4: Rejection reason flags
        uint8_t flags = data[4];
        frame->w = (flags & 0x01) != 0;  // W: Invalid control field
        frame->x = (flags & 0x02) != 0;  // X: Invalid information field
        frame->y = (flags & 0x04) != 0;  // Y: Unable to recover
        frame->z = (flags & 0x08) != 0;  // Z: Reserved
    } else {
        // Parse 3-byte modulo-8 FRMR data field with explicit comments
        // Byte 0: Control field (8-bit)
        // This is the control field of the frame that was rejected
        frame->frmr_control = data[0];

        // Byte 1: V(R) in bits 5-7 (3 bits), CR in bit 4, V(S) in bits 1-3 (3 bits)
        // V(S) is the send sequence number of the rejected frame
        // V(R) is the receive sequence number of the rejected frame
        uint8_t vr_cr_vs = data[1];
        frame->vr = (vr_cr_vs >> 5) & 0x07;
        frame->frmr_cr = (vr_cr_vs & 0x10) != 0;
        frame->vs = (vr_cr_vs >> 1) & 0x07;

        // Byte 2: Rejection reason flags
        uint8_t flags = data[2];
        frame->w = (flags & 0x01) != 0;  // W: Invalid control field
        frame->x = (flags & 0x02) != 0;  // X: Invalid information field
        frame->y = (flags & 0x04) != 0;  // Y: Unable to recover
        frame->z = (flags & 0x08) != 0;  // Z: Reserved
    }

    return frame;
}

uint8_t* ax25_frame_reject_frame_encode(const ax25_frame_reject_frame_t *frame, size_t *len, uint8_t *err) {
    *err = 0;
    bool is_modulo128 = frame->is_modulo128;

    // Total length: 1 control byte + data field (3 or 5 bytes)
    *len = is_modulo128 ? 6 : 4;
    uint8_t *bytes = hal_mem_alloc((uint16_t) (*len));
    if (!bytes) {
        *err = 1;  // Memory allocation failed
        return NULL;
    }

    // Encode control byte
    bytes[0] = frame->base.modifier | (frame->base.pf ? POLL_FINAL_8BIT : 0);

    if (is_modulo128) {
        // Encode 5-byte data field
        bytes[1] = frame->frmr_control & 0xFF;                // Control low byte
        bytes[2] = (frame->frmr_control >> 8) & 0xFF;         // Control high byte
        bytes[3] = ((frame->vs & 0x7F) << 1) | (frame->frmr_cr ? 0x01 : 0);  // N(s) and CR
        bytes[4] = (frame->vr & 0x7F) << 1;                   // N(r)
        bytes[5] = (frame->w ? 0x01 : 0) | (frame->x ? 0x02 : 0) | (frame->y ? 0x04 : 0) | (frame->z ? 0x08 : 0);     // Flags
    } else {
        // Encode 3-byte data field
        bytes[1] = frame->frmr_control & 0xFF;                // Control byte
        bytes[2] = ((frame->vr & 0x07) << 5) | (frame->frmr_cr ? 0x10 : 0) | ((frame->vs & 0x07) << 1);                 // V(r), CR, V(s)
        bytes[3] = (frame->w ? 0x01 : 0) | (frame->x ? 0x02 : 0) | (frame->y ? 0x04 : 0) | (frame->z ? 0x08 : 0);     // Flags
    }

    return bytes;
}

ax25_information_frame_t* ax25_information_frame_decode(ax25_frame_header_t *header, uint16_t control, const uint8_t *data, size_t len, bool is_16bit,
        uint8_t *err) {
    *err = 0;
    ax25_information_frame_t *frame = hal_mem_alloc((uint16_t) (sizeof(ax25_information_frame_t)));
    if (!frame) {
        *err = 1;
        return NULL;
    }
    frame->base.type = is_16bit ? AX25_FRAME_INFORMATION_16BIT : AX25_FRAME_INFORMATION_8BIT;
    frame->base.header = *header;

    // According to AX.25 v2.2 Section 4.3.1:
    // Modulo-8 (8-bit control): N(S) in bits 1-3, N(R) in bits 5-7
    // Modulo-128 (16-bit control): N(S) in bits 1-7, N(R) in bits 9-15
    if (is_16bit) {
        // Modulo-128: N(R) in bits 9-15 (7 bits)
        frame->nr = (control >> 9) & 0x7F;
        // Poll/Final in bit 8
        frame->pf = (control & 0x0100) != 0;
        // N(S) in bits 1-7 (7 bits)
        frame->ns = (control >> 1) & 0x7F;
    } else {
        // Modulo-8: N(R) in bits 5-7 (3 bits)
        frame->nr = (control >> 5) & 0x07;
        // Poll/Final in bit 4
        frame->pf = (control & 0x10) != 0;
        // N(S) in bits 1-3 (3 bits)
        frame->ns = (control >> 1) & 0x07;
    }

    if (len == 0) {
        frame->pid = 0;
        frame->payload_len = 0;
        frame->payload = NULL;
    } else {
        if (len < 1) {
            *err = 2;
            hal_mem_free(frame);
            return NULL;
        }
        frame->pid = data[0];
        frame->payload_len = len - 1;
        if (frame->payload_len > AX25_MAX_INFO) {
            // Payload exceeds N1 maximum - reject per AX.25 v2.2 section 6.7.2.1
            *err = 3;
            hal_mem_free(frame);
            return NULL;
        }
        if (frame->payload_len > 0) {
            ax25_buf_t *i_buf = ax25_buf_alloc();
            if (!i_buf) {
                *err = 3;
                hal_mem_free(frame);
                return NULL;
            }
            i_buf->len = (uint16_t) frame->payload_len;
            memcpy(i_buf->data, data + 1, frame->payload_len);
            i_buf->data[frame->payload_len] = 0;  // null terminate for safety
            frame->payload = i_buf->data;
        } else {
            frame->payload = NULL;
        }
    }
    return frame;
}

uint8_t* ax25_information_frame_encode(const ax25_information_frame_t *frame, size_t *len, uint8_t *err) {
    *err = 0;
    bool is_16bit = (frame->base.type == AX25_FRAME_INFORMATION_16BIT);
    *len = (is_16bit ? 2 : 1) + 1 + frame->payload_len;
    uint8_t *bytes = hal_mem_alloc((uint16_t) (*len));

    if (!bytes) {
        *err = 1;
        return NULL;
    }

    if (is_16bit) {
        uint16_t control = ((frame->nr << 9) & 0xFE00) | (frame->pf ? POLL_FINAL_16BIT : 0) | ((frame->ns << 1) & 0x01FE) | CONTROL_I_VAL;
        bytes[0] = control & 0xFF;
        bytes[1] = (control >> 8) & 0xFF;
        bytes[2] = frame->pid;
        memcpy(bytes + 3, frame->payload, frame->payload_len);
    } else {
        bytes[0] = ((frame->nr << 5) & 0xE0) | (frame->pf ? POLL_FINAL_8BIT : 0) | ((frame->ns << 1) & 0x0E) | CONTROL_I_VAL;
        bytes[1] = frame->pid;
        memcpy(bytes + 2, frame->payload, frame->payload_len);
    }

    return bytes;
}

uint8_t* ax25_supervisory_frame_encode(const ax25_supervisory_frame_t *frame, size_t *len, uint8_t *err) {
    *err = 0;
    bool is_16bit = (frame->base.type >= AX25_FRAME_SUPERVISORY_RR_16BIT);
    *len = is_16bit ? 2 : 1;
    uint8_t *bytes = hal_mem_alloc((uint16_t) (*len));

    if (!bytes) {
        *err = 1;
        return NULL;
    }

    // Extract supervisory code correctly and ensure proper bit positioning
    // Code is stored in frame->code as normalized value (0x00, 0x01, 0x02, 0x03)
    // and must be shifted to bits 2-3 for the control field
    uint8_t code_bits = (frame->code << 2) & 0x0C;

    if (is_16bit) {
        // Modulo-128: N(R) in bits 9-15, code in bits 2-3, P/F in bit 8
        uint16_t control = ((frame->nr << 9) & 0xFE00) | (frame->pf ? POLL_FINAL_16BIT : 0) | code_bits | CONTROL_S_VAL;
        bytes[0] = control & 0xFF;
        bytes[1] = (control >> 8) & 0xFF;
    } else {
        // Modulo-8: N(R) in bits 5-7, code in bits 2-3, P/F in bit 4
        bytes[0] = ((frame->nr << 5) & 0xE0) | (frame->pf ? POLL_FINAL_8BIT : 0) | code_bits | CONTROL_S_VAL;
    }

    return bytes;
}

ax25_supervisory_frame_t* ax25_supervisory_frame_decode(ax25_frame_header_t *header, uint16_t control, bool is_16bit, uint8_t *err) {
    *err = 0;

    // Extract code from bits 2-3 and normalize to 0-3
    uint8_t code = (control >> 2) & 0x03;
    ax25_frame_type_t type;

    if (is_16bit) {
        switch (code) {
            case 0x00:
                type = AX25_FRAME_SUPERVISORY_RR_16BIT;
            break;
            case 0x01:
                type = AX25_FRAME_SUPERVISORY_RNR_16BIT;
            break;
            case 0x02:
                type = AX25_FRAME_SUPERVISORY_REJ_16BIT;
            break;
            case 0x03:
                type = AX25_FRAME_SUPERVISORY_SREJ_16BIT;
            break;
            default:
                *err = 1;
                return NULL;
        }
    } else {
        switch (code) {
            case 0x00:
                type = AX25_FRAME_SUPERVISORY_RR_8BIT;
            break;
            case 0x01:
                type = AX25_FRAME_SUPERVISORY_RNR_8BIT;
            break;
            case 0x02:
                type = AX25_FRAME_SUPERVISORY_REJ_8BIT;
            break;
            case 0x03:
                type = AX25_FRAME_SUPERVISORY_SREJ_8BIT;
            break;
            default:
                *err = 1;
                return NULL;
        }
    }

    ax25_supervisory_frame_t *frame = hal_mem_alloc((uint16_t) (sizeof(ax25_supervisory_frame_t)));
    if (!frame) {
        *err = 2;
        return NULL;
    }
    frame->base.type = type;
    frame->base.header = *header;

    // Extract N(R) with proper bit masking for both modulo-8 and modulo-128
    if (is_16bit) {
        // Modulo-128: N(R) in bits 9-15 (7 bits)
        frame->nr = (control >> 9) & 0x7F;
    } else {
        // Modulo-8: N(R) in bits 5-7 (3 bits)
        frame->nr = (control >> 5) & 0x07;
    }

    // Extract P/F bit with correct position for both modulo-8 and modulo-128
    frame->pf = (control & (is_16bit ? POLL_FINAL_16BIT : POLL_FINAL_8BIT)) != 0;

    // Store normalized code value (0-3) not shifted
    frame->code = code;

    // Note: Supervisory frames may have optional data payload after control field
    // This is handled by the caller (ax25_frame_decode) if needed

    return frame;
}

ax25_xid_parameter_t* ax25_xid_raw_parameter_new(int pi, const uint8_t *pv, size_t pv_len, uint8_t *err) {
    *err = 0;
    if (pv_len > 255) {
        *err = 1;
        return NULL;
    }
    ax25_xid_parameter_t *param = hal_mem_alloc((uint16_t) (sizeof(ax25_xid_parameter_t)));
    if (!param) {
        *err = 2;
        return NULL;
    }
    ax25_raw_param_data_t *data = NULL;
    if (pv) {
        data = hal_mem_alloc((uint16_t) (sizeof(ax25_raw_param_data_t) + pv_len));
        if (!data) {
            *err = 3;
            hal_mem_free(param);
            return NULL;
        }
        data->pv_len = pv_len;
        memcpy(data->pv, pv, pv_len);
    }
    param->pi = pi;
    param->encode = ax25_xid_raw_parameter_encode;
    param->copy = ax25_xid_raw_parameter_copy;
    param->free = ax25_xid_raw_parameter_free;
    param->data = data;
    return param;
}

uint8_t* ax25_xid_raw_parameter_encode(const ax25_xid_parameter_t *param, size_t *len, uint8_t *err) {
    *err = 0;
    ax25_raw_param_data_t *data = (ax25_raw_param_data_t*) param->data;
    size_t pv_len = data ? data->pv_len : 0;
    uint8_t *pv = data ? data->pv : NULL;
    *len = 2 + pv_len;
    uint8_t *bytes = hal_mem_alloc((uint16_t) (*len));
    if (!bytes) {
        *err = 1;
        return NULL;
    }
    bytes[0] = param->pi;
    bytes[1] = (uint8_t) pv_len;
    if (pv_len)
        memcpy(bytes + 2, pv, pv_len);
    return bytes;
}

ax25_xid_parameter_t* ax25_xid_raw_parameter_copy(const ax25_xid_parameter_t *param, uint8_t *err) {
    *err = 0;
    ax25_raw_param_data_t *data = (ax25_raw_param_data_t*) param->data;
    size_t pv_len = data ? data->pv_len : 0;
    uint8_t *pv = data ? data->pv : NULL;
    return ax25_xid_raw_parameter_new(param->pi, pv, pv_len, err);
}

void ax25_xid_raw_parameter_free(ax25_xid_parameter_t *param, uint8_t *err) {
    *err = 0;
    if (!param) {
        *err = 1;
        return;
    }
    hal_mem_free(param->data);
    hal_mem_free(param);
}

ax25_xid_parameter_t* ax25_xid_parameter_decode(const uint8_t *data, size_t len, size_t *consumed, uint8_t *err) {
    *err = 0;

    if (len < 2) {
        *err = 1;
        return NULL;
    }

    int pi = data[0];
    size_t pv_len = data[1];
    if (len < 2 + pv_len) {
        *err = 2;
        return NULL;
    }

    ax25_xid_parameter_t *param = ax25_xid_raw_parameter_new(pi, data + 2, pv_len, err);
    if (!param) {
        *err = 3;
        return NULL;
    }

    *consumed = 2 + pv_len;

    return param;
}

ax25_exchange_identification_frame_t* ax25_exchange_identification_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len,
        uint8_t *err) {
    *err = 0;

    if (len < 4) {
        *err = 1;
        return NULL;
    }

    uint8_t fi = data[0];
    uint8_t gi = data[1];
    uint16_t gl = uint_decode(data + 2, 2, true, err);

    if (len - 4 != gl) {
        *err = 2;
        return NULL;
    }

    ax25_xid_parameter_t **params = NULL;
    size_t param_count = 0;
    const uint8_t *param_data = data + 4;
    size_t remaining = gl;

    while (remaining > 0) {
        size_t consumed;
        ax25_xid_parameter_t *param = ax25_xid_parameter_decode(param_data, remaining, &consumed, err);
        if (!param) {
            *err = 3;
            for (size_t i = 0; i < param_count; i++)
                params[i]->free(params[i], err);
            hal_mem_free(params);
            return NULL;
        }

        ax25_xid_parameter_t **new_params = (ax25_xid_parameter_t **)hal_mem_realloc(params, (uint16_t)((param_count + 1) * sizeof(ax25_xid_parameter_t*)));
        if (!new_params) {
            *err = 4;
            param->free(param, err);
            for (size_t i = 0; i < param_count; i++)
                params[i]->free(params[i], err);
            hal_mem_free(params);
            return NULL;
        }

        params = new_params;
        params[param_count++] = param;
        param_data += consumed;
        remaining -= consumed;
    }

    ax25_exchange_identification_frame_t *frame = hal_mem_alloc((uint16_t) (sizeof(ax25_exchange_identification_frame_t)));
    if (!frame) {
        *err = 5;
        for (size_t i = 0; i < param_count; i++)
            params[i]->free(params[i], err);
        hal_mem_free(params);
        return NULL;
    }

    frame->base.base.type = AX25_FRAME_UNNUMBERED_XID;
    frame->base.base.header = *header;
    frame->base.pf = pf;
    frame->base.modifier = 0xAF;
    frame->fi = fi;
    frame->gi = gi;
    frame->parameters = params;
    frame->param_count = param_count;

    return frame;
}

uint8_t* ax25_exchange_identification_frame_encode(const ax25_exchange_identification_frame_t *frame, size_t *len, uint8_t *err) {
    *err = 0;

    // Pass 1: Calculate total parameter size without allocation
    size_t params_len = 0;
    for (size_t i = 0; i < frame->param_count; i++) {
        ax25_xid_parameter_t *param = frame->parameters[i];
        ax25_raw_param_data_t *data = (ax25_raw_param_data_t*) param->data;
        size_t pv_len = data ? data->pv_len : 0;
        // Each parameter: PI (1 byte) + PL (1 byte) + PV (pv_len bytes)
        params_len += 2 + pv_len;
    }

    // Single allocation for complete frame
    *len = 1 + 4 + params_len;  // control + fi + gi + gl (2 bytes) + parameters
    uint8_t *bytes = hal_mem_alloc((uint16_t)(*len));
    if (!bytes) {
        *err = 1;
        return NULL;
    }

    // Build frame header directly in output buffer
    bytes[0] = frame->base.modifier | (frame->base.pf ? POLL_FINAL_8BIT : 0);
    bytes[1] = frame->fi;
    bytes[2] = frame->gi;

    // Encode GL (group length) directly in big-endian format
    // No need for temporary uint_encode allocation
    bytes[3] = (params_len >> 8) & 0xFF;
    bytes[4] = params_len & 0xFF;

    // Pass 2: Encode parameters directly into output buffer
    size_t offset = 5;
    for (size_t i = 0; i < frame->param_count; i++) {
        ax25_xid_parameter_t *param = frame->parameters[i];
        ax25_raw_param_data_t *data = (ax25_raw_param_data_t*) param->data;
        size_t pv_len = data ? data->pv_len : 0;
        uint8_t *pv = data ? data->pv : NULL;

        // Write PI (Parameter Identifier)
        bytes[offset++] = param->pi;
        // Write PL (Parameter Length)
        bytes[offset++] = (uint8_t) pv_len;
        // Write PV (Parameter Value)
        if (pv_len > 0) {
            memcpy(bytes + offset, pv, pv_len);
            offset += pv_len;
        }
    }

    return bytes;
}

ax25_test_frame_t* ax25_test_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len, uint8_t *err) {
    *err = 0;

    ax25_test_frame_t *frame = hal_mem_alloc((uint16_t)(sizeof(ax25_test_frame_t)));
    if (!frame) {
        *err = 1;
        return NULL;
    }

    frame->base.base.type = AX25_FRAME_UNNUMBERED_TEST;
    frame->base.base.header = *header;
    frame->base.pf = pf;
    frame->base.modifier = 0xE3;
    frame->payload_len = len;
    if (len > AX25_MAX_INFO) {
        // TEST frame payload exceeds N1 - reject per AX.25 v2.2 section 6.7.2.1
        *err = 2;
        hal_mem_free(frame);
        return NULL;
    }
    ax25_buf_t *test_buf = ax25_buf_alloc();
    if (!test_buf) {
        *err = 2;
        hal_mem_free(frame);
        return NULL;
    }
    test_buf->len = (uint16_t) len;
    memcpy(test_buf->data, data, len);
    test_buf->data[len] = 0;
    frame->payload = test_buf->data;

    return frame;
}

uint8_t* ax25_test_frame_encode(const ax25_test_frame_t *frame, size_t *len, uint8_t *err) {
    *err = 0;
    *len = 1 + frame->payload_len;
    uint8_t *bytes = hal_mem_alloc((uint16_t)(*len));

    if (!bytes) {
        *err = 1;
        return NULL;
    }

    bytes[0] = frame->base.modifier | (frame->base.pf ? POLL_FINAL_8BIT : 0);
    memcpy(bytes + 1, frame->payload, frame->payload_len);

    return bytes;
}

ax25_xid_parameter_t* ax25_xid_class_of_procedures_new(
bool a_flag, bool b_flag, bool c_flag, bool d_flag,
bool e_flag, bool f_flag, bool g_flag, uint8_t reserved, uint8_t *err) {
    *err = 0;
    uint8_t pv[2];
    pv[0] = (a_flag ? 0x01 : 0) | (b_flag ? 0x02 : 0) | (c_flag ? 0x04 : 0) | (d_flag ? 0x08 : 0) | (e_flag ? 0x10 : 0) | (f_flag ? 0x20 : 0)
            | (g_flag ? 0x40 : 0);
    pv[1] = reserved;

    return ax25_xid_raw_parameter_new(1, pv, 2, err);
}

ax25_xid_parameter_t* ax25_xid_hdlc_optional_functions_new(
bool rnr, bool rej, bool srej, bool sabm, bool sabme, bool dm, bool disc,
bool ua, bool frmr, bool ui, bool xid, bool test, bool modulo8, bool modulo128,
bool res1, bool res2, bool res3, bool res4, bool res5, bool res6, bool res7, uint8_t reserved, bool ext, uint8_t *err) {
    *err = 0;
    uint8_t pv[4];
    pv[0] = (rnr ? 0x01 : 0) | (rej ? 0x02 : 0) | (srej ? 0x04 : 0) | (sabm ? 0x08 : 0) | (sabme ? 0x10 : 0) | (dm ? 0x20 : 0) | (disc ? 0x40 : 0)
            | (ua ? 0x80 : 0);
    pv[1] = (frmr ? 0x01 : 0) | (ui ? 0x02 : 0) | (xid ? 0x04 : 0) | (test ? 0x08 : 0) | (modulo8 ? 0x10 : 0) | (modulo128 ? 0x20 : 0) | (res1 ? 0x40 : 0)
            | (res2 ? 0x80 : 0);
    pv[2] = (res3 ? 0x01 : 0) | (res4 ? 0x02 : 0) | (res5 ? 0x04 : 0) | (res6 ? 0x06 : 0) | (res7 ? 0x08 : 0);
    pv[3] = reserved | (ext ? 0x80 : 0);
    return ax25_xid_raw_parameter_new(2, pv, 4, err);
}

ax25_xid_parameter_t* ax25_xid_big_endian_new(int pi, uint32_t value, size_t length, uint8_t *err) {
    *err = 0;
    size_t len;
    uint8_t *pv = uint_encode(value, true, length, &len, err);
    if (!pv) {
        *err = 1;
        return NULL;
    }

    ax25_xid_parameter_t *param = ax25_xid_raw_parameter_new(pi, pv, len, err);
    hal_mem_free(pv);

    return param;
}

void ax25_xid_init_defaults(uint8_t *err) {
    AX25_20_DEFAULT_XID_COP = ax25_xid_class_of_procedures_new(true, false, false, false, false, false, true, 0, err);
    AX25_22_DEFAULT_XID_COP = ax25_xid_class_of_procedures_new(true, false, false, false, false, false, true, 0, err);
    AX25_20_DEFAULT_XID_HDLCOPTFUNC = ax25_xid_hdlc_optional_functions_new(
    false, true, false, true, false, false, false, false, true, false, true, false, true, false,
    false, false, true, false, false, false, false, 0, false, err);
    AX25_22_DEFAULT_XID_HDLCOPTFUNC = ax25_xid_hdlc_optional_functions_new(
    false, true, true, false, false, false, false, false, true, false, true, false, true, false,
    false, false, true, false, false, false, false, 0, false, err);
    AX25_20_DEFAULT_XID_IFIELDRX = ax25_xid_big_endian_new(6, 2048, 2, err);
    AX25_22_DEFAULT_XID_IFIELDRX = ax25_xid_big_endian_new(6, 2048, 2, err);
    AX25_20_DEFAULT_XID_WINDOWSZRX = ax25_xid_big_endian_new(8, 7, 1, err);
    AX25_22_DEFAULT_XID_WINDOWSZRX = ax25_xid_big_endian_new(8, 7, 1, err);
    AX25_20_DEFAULT_XID_ACKTIMER = ax25_xid_big_endian_new(9, 3000, 2, err);
    AX25_22_DEFAULT_XID_ACKTIMER = ax25_xid_big_endian_new(9, 3000, 2, err);
    AX25_20_DEFAULT_XID_RETRIES = ax25_xid_big_endian_new(10, 10, 2, err);
    AX25_22_DEFAULT_XID_RETRIES = ax25_xid_big_endian_new(10, 10, 2, err);
}

void ax25_xid_deinit_defaults(uint8_t *err) {
    uint8_t first_error = 0;

#define FREE_XID_PARAM(param) \
    do { \
        if (param) { \
            if (param->free) { \
                uint8_t local_err = 0; \
                param->free(param, &local_err); \
                if (local_err != 0 && first_error == 0) { \
                    first_error = local_err; \
                } \
            } else { \
            	 hal_mem_free(param->data); \
            	 hal_mem_free(param); \
            } \
            param = NULL; \
        } \
    } while (0)

    FREE_XID_PARAM(AX25_20_DEFAULT_XID_COP);
    FREE_XID_PARAM(AX25_22_DEFAULT_XID_COP);
    FREE_XID_PARAM(AX25_20_DEFAULT_XID_HDLCOPTFUNC);
    FREE_XID_PARAM(AX25_22_DEFAULT_XID_HDLCOPTFUNC);
    FREE_XID_PARAM(AX25_20_DEFAULT_XID_IFIELDRX);
    FREE_XID_PARAM(AX25_22_DEFAULT_XID_IFIELDRX);
    FREE_XID_PARAM(AX25_20_DEFAULT_XID_WINDOWSZRX);
    FREE_XID_PARAM(AX25_22_DEFAULT_XID_WINDOWSZRX);
    FREE_XID_PARAM(AX25_20_DEFAULT_XID_ACKTIMER);
    FREE_XID_PARAM(AX25_22_DEFAULT_XID_ACKTIMER);
    FREE_XID_PARAM(AX25_20_DEFAULT_XID_RETRIES);
    FREE_XID_PARAM(AX25_22_DEFAULT_XID_RETRIES);

#undef FREE_XID_PARAM

    *err = first_error;
}

bool is_modulo128_used(ax25_frame_t *sabme, ax25_frame_t *response) {
    if (sabme->type != AX25_FRAME_UNNUMBERED_SABME) {
        return false;
    }
    if (response->type == AX25_FRAME_UNNUMBERED_UA) {
        return true;  // UA response to SABME indicates modulo-128
    } else if (response->type == AX25_FRAME_UNNUMBERED_DM || response->type == AX25_FRAME_UNNUMBERED_FRMR) {
        return false;  // DM or FRMR response indicates fallback to modulo-8
    }
    return false;  // Default case, assume modulo-8 if unknown response
}

ax25_segmented_info_t* ax25_segment_info_fields(const uint8_t *payload, size_t payload_len, size_t n1, uint8_t *err, size_t *num_segments) {
    *err = 0;
    *num_segments = 0;

    if (payload == NULL || err == NULL || num_segments == NULL) {
        if (err)
            *err = 1;
        return NULL;
    }

    // Check maximum payload size (16-bit length field)
    // Segmentation stores payload length in 2 bytes (big-endian), max value 65535
    if (payload_len > 0xFFFF) {
        *err = 1;
        return NULL;
    }

    if (n1 < 4) {  // Minimum: PID (1) + control (1) + length (2)
        *err = 2;
        return NULL;
    }

    size_t max_first_data = n1 - 4;  // PID + control + total_length
    size_t max_other_data = n1 - 2;  // PID + control
    if (max_first_data == 0 || max_other_data == 0) {
        *err = 3;
        return NULL;
    }

    // Calculate estimated segments to prevent huge allocations
    size_t estimated_segments = 1;  // First segment
    if (payload_len > max_first_data) {
        size_t remaining = payload_len - max_first_data;
        estimated_segments += (remaining + max_other_data - 1) / max_other_data;
    }

    if (estimated_segments > 63) {
        *err = 7;  // Too many segments
        return NULL;
    }

    ax25_segmented_info_t *segments = NULL;
    size_t offset = 0;
    size_t segment_number = 0;

    while (offset < payload_len) {
        // Calculate remaining data with explicit overflow prevention
        size_t remaining_data = payload_len - offset;
        size_t max_data = (segment_number == 0) ? max_first_data : max_other_data;
        size_t data_len = (remaining_data > max_data) ? max_data : remaining_data;

        if (data_len > remaining_data || data_len > max_data) {
            *err = 4;
            goto cleanup_segments;
        }

        size_t info_field_len = 2 + data_len;  // PID(1) + Control(1) + data
        if (segment_number == 0) {
            info_field_len += 2;  // Add total_length field
        }

        // Sanity check
        if (info_field_len > n1) {
            *err = 4;
            goto cleanup_segments;
        }

        uint8_t *info_field = hal_mem_alloc((uint16_t)(info_field_len));
        if (!info_field) {
            *err = 5;
            goto cleanup_segments;
        }

        info_field[0] = 0x08;  // Segmentation PID

        // bounds checking for segment_number
        if (segment_number > 63) {
            *err = 7;
            hal_mem_free(info_field);
            goto cleanup_segments;
        }

        uint8_t control = (uint8_t) (segment_number & 0x3F);
        if (segment_number == 0) {
            control |= 0x80;  // Begin flag
        }
        // Check for end flag without overflow risk
        if (offset + data_len >= payload_len) {
            control |= 0x40;  // End flag
        }
        info_field[1] = control;

        size_t pos = 2;
        if (segment_number == 0) {
            // Clearer comment about 16-bit safety
            // payload_len is guaranteed <= 0xFFFF from check above, safe for 16-bit encoding
            info_field[pos++] = (uint8_t) ((payload_len >> 8) & 0xFF);
            info_field[pos++] = (uint8_t) (payload_len & 0xFF);
        }
        memcpy(info_field + pos, payload + offset, data_len);

        ax25_segmented_info_t *new_segments = (ax25_segmented_info_t *)hal_mem_realloc(segments, (uint16_t)((segment_number + 1) * sizeof(ax25_segmented_info_t)));
        if (!new_segments) {
            *err = 6;
            hal_mem_free(info_field);
            goto cleanup_segments;
        }
        segments = new_segments;
        segments[segment_number].info_field = info_field;
        segments[segment_number].info_field_len = info_field_len;

        offset += data_len;
        segment_number++;

        // Prevent excessive segmentation (max 64 segments per spec)
        if (segment_number > 63) {
            *err = 7;
            goto cleanup_segments;
        }
    }
    *num_segments = segment_number;
    return segments;

    cleanup_segments:
    if (segments) {
        for (size_t i = 0; i < segment_number; i++) {
            hal_mem_free(segments[i].info_field);
        }
        hal_mem_free(segments);
    }
    return NULL;
}

uint8_t* ax25_reassemble_info_fields(ax25_segmented_info_t *info_fields, size_t num_info_fields, size_t *reassembled_len, uint8_t *err) {
    *err = 0;
    if (num_info_fields == 0) {
        *reassembled_len = 0;
        return NULL;
    }

    ax25_reassembly_segment_t *segments = hal_mem_alloc((uint16_t)(num_info_fields * sizeof(ax25_reassembly_segment_t)));
    if (!segments) {
        *err = 1;
        return NULL;
    }

    size_t total_length = 0;
    bool has_first = false;
    for (size_t i = 0; i < num_info_fields; i++) {
        uint8_t *info = info_fields[i].info_field;
        size_t len = info_fields[i].info_field_len;
        if (len < 2 || info[0] != 0x08) {
            *err = 2;
            hal_mem_free(segments);
            return NULL;
        }
        uint8_t control = info[1];
        bool begin = (control & 0x80) != 0;
        int segment_number = (control & 0x3F);
        size_t offset = 2;
        if (begin) {
            if (len < 4) {
                *err = 3;
                hal_mem_free(segments);
                return NULL;
            }
            total_length = (uint16_t) (((uint16_t) info[2] << 8u) | (uint16_t) info[3]);
            offset = 4;
        }
        size_t data_len = len - offset;
        segments[i].control = control;
        segments[i].total_length = total_length;
        segments[i].data = info + offset;
        segments[i].data_len = data_len;
        segments[i].segment_number = segment_number;
        if (begin)
            has_first = true;
    }

    if (!has_first) {
        *err = 4;
        hal_mem_free(segments);
        return NULL;
    }

    // Insertion sort by segment_number.
    // O(n) best case (already ordered), O(n^2) worst case.
    // n <= 64 per AX.25 §C6.3.1 (6-bit segment sequence field) — at most 4096 comparisons, negligible cost.
    for (size_t si = 1; si < num_info_fields; si++) {
        ax25_reassembly_segment_t key = segments[si];
        ptrdiff_t sj = (ptrdiff_t) si - 1;
        while (sj >= 0 && segments[sj].segment_number > key.segment_number) {
            segments[sj + 1] = segments[sj];
            sj--;
        }
        segments[sj + 1] = key;
    }

    // Check for duplicates or missing segments
    int expected_segments = -1;
    for (size_t i = 0; i < num_info_fields; i++) {
        if (segments[i].segment_number != (int) i) {
            *err = 5;
            hal_mem_free(segments);
            return NULL;
        }
        if ((segments[i].control & 0x40) != 0) {  // End flag
            expected_segments = i + 1;
        }
    }
    if (expected_segments == -1 || expected_segments != (int) num_info_fields) {
        *err = 6;
        hal_mem_free(segments);
        return NULL;
    }

    // Reassemble
    uint8_t *reassembled = hal_mem_alloc((uint16_t)(total_length));
    if (!reassembled) {
        *err = 7;
        hal_mem_free(segments);
        return NULL;
    }
    size_t offset = 0;
    for (size_t i = 0; i < num_info_fields; i++) {
        memcpy(reassembled + offset, segments[i].data, segments[i].data_len);
        offset += segments[i].data_len;
    }
    if (offset != total_length) {
        *err = 8;
        hal_mem_free(reassembled);
        hal_mem_free(segments);
        return NULL;
    }

    *reassembled_len = total_length;
    hal_mem_free(segments);
    return reassembled;
}

void ax25_free_segmented_info(ax25_segmented_info_t *segments, size_t num_segments) {
    for (size_t i = 0; i < num_segments; i++) {
        hal_mem_free(segments[i].info_field);
    }
    hal_mem_free(segments);
}

bool ax25_validate_frame_size(size_t frame_len) {
    // Minimum frame size is 17 bytes:
    // - Destination address: 7 bytes
    // - Source address: 7 bytes
    // - Control field: 1 byte (minimum)
    // - FCS: 2 bytes
    return (frame_len >= AX25_MIN_FRAME_SIZE);
}

bool ax25_validate_address_field(const uint8_t *addr_field, size_t len, size_t *addr_field_len) {
    if (addr_field == NULL || len < 14) {
        return false;  // Minimum: dest(7) + source(7) = 14 bytes
    }

    size_t offset = 0;
    bool found_extension = false;

    // Maximum 10 addresses: dest + source + 8 repeaters
    // Per AX.25 v2.2 Section 3.12.3: up to 8 digipeaters allowed
    for (int i = 0; i < 10; i++) {
        if (offset + 7 > len) {
            return false;  // Incomplete address - not enough data
        }

        // Check if this is the last address (extension bit set)
        // Extension bit is bit 0 of the 7th byte (SSID byte)
        if (addr_field[offset + 6] & 0x01) {
            found_extension = true;
            *addr_field_len = offset + 7;
            break;
        }

        offset += 7;
    }

    if (!found_extension) {
        return false;  // No terminating address found - invalid frame
    }

    // Must have at least 2 addresses (dest + source)
    // Per AX.25 v2.2 Section 3.12: destination and source are mandatory
    return (*addr_field_len >= 14);
}

bool ax25_validate_ssid(int ssid) {
    // SSID must be in range 0-15 (4 bits)
    return (ssid >= 0 && ssid <= 15);
}

void ax25_reverse_repeater_path(ax25_frame_header_t *header) {
    // Input validation
    if (!header || header->repeaters.num_repeaters == 0) {
        return;
    }

    // AX.25 v2.2 Section 3.12.4: Reverse the digipeater path for response frames
    // Maximum 8 digipeaters per Section 3.12.3
    if (header->repeaters.num_repeaters > AX25_MAX_REPEATERS) {
        return;  // Invalid path, don't modify
    }

    // Create temporary copy for reversal
    ax25_address_t temp[AX25_MAX_REPEATERS];
    uint8_t count = (uint8_t) header->repeaters.num_repeaters;

    // Copy in reverse order
    for (uint8_t i = 0; i < count; i++) {
        temp[i] = header->repeaters.repeaters[count - 1 - i];
    }

    // Copy back and reset H-bits
    for (uint8_t i = 0; i < count; i++) {
        header->repeaters.repeaters[i] = temp[i];
        // Per Section 3.12.4: Reset H-bits for reverse path
        // The response will traverse the path anew
        header->repeaters.repeaters[i].ch = false;
    }
}

bool ax25_frame_digipeated_by(const ax25_frame_header_t *header, const char *our_call, uint8_t our_ssid) {
    // Input validation
    if (!header || !our_call) {
        return false;
    }

    // Validate SSID range per AX.25 v2.2 Section 3.12.2
    if (!ax25_validate_ssid(our_ssid)) {
        return false;
    }

    // AX.25 v2.2 Section 3.12.3: Check each digipeater in path
    // The H-bit (ch field) indicates has-been-repeated status
    for (int i = 0; i < header->repeaters.num_repeaters; i++) {
        const ax25_address_t *digi = &header->repeaters.repeaters[i];

        // Compare callsign (case-sensitive per spec) and SSID
        if (strcmp(digi->callsign, our_call) == 0 && digi->ssid == our_ssid && digi->ch) {  // H-bit must be set
            return true;
        }
    }

    return false;
}

int8_t ax25_find_next_digi(const ax25_frame_header_t *header) {
    // Input validation
    if (!header || header->repeaters.num_repeaters == 0) {
        return -1;  // No digipeaters in path
    }

    // AX.25 v2.2 Section 3.12.3: Find first digipeater with H-bit not set
    // Digipeaters are processed left to right (index 0 to n)
    for (int i = 0; i < header->repeaters.num_repeaters; i++) {
        if (!header->repeaters.repeaters[i].ch) {  // H-bit not set
            return (int8_t) i;
        }
    }

    // All digipeaters have been used
    return -1;
}

void ax25_digipeat_frame(uint8_t *frame_data, size_t len, const char *my_call, uint8_t my_ssid, void (*retransmit)(uint8_t*, size_t)) {
    // Input validation
    if (!frame_data || !my_call || !retransmit) {
        return;
    }

    // Validate SSID range per AX.25 v2.2 Section 3.12.2
    if (!ax25_validate_ssid(my_ssid)) {
        return;
    }

    // Decode the frame - use MODULO128_AUTO to let decoder determine modulo
    uint8_t err;
    ax25_frame_t *frame = ax25_frame_decode(frame_data, len, MODULO128_AUTO, &err);
    if (!frame) {
        return;  // Invalid frame, cannot digipeat
    }

    // AX.25 v2.2 Section 3.12.3: Find next digipeater slot
    int8_t next_idx = ax25_find_next_digi(&frame->header);
    if (next_idx < 0) {
        // No digipeater slots available or all already used
        ax25_frame_free(frame, &err);
        return;
    }

    // Check if this digipeater slot is for us
    ax25_address_t *digi = &frame->header.repeaters.repeaters[next_idx];

    // Compare callsign and SSID
    // Per AX.25 v2.2: Callsign comparison is case-sensitive
    if (strcmp(digi->callsign, my_call) == 0 && digi->ssid == my_ssid) {
        // This frame is addressed to us for digipeating
        // Set H-bit to mark as used per Section 3.12.3
        digi->ch = true;

        // Re-encode the frame with modified path
        size_t new_len;
        uint8_t *new_frame = ax25_frame_encode(frame, &new_len, &err);
        if (new_frame) {
            // Retransmit the modified frame
            retransmit(new_frame, new_len);
            hal_mem_free(new_frame);
        }
    }

    // Free the decoded frame structure
    ax25_frame_free(frame, &err);
}

// The H-bit (has-been-repeated) is bit 7 (mask 0x80) of the 7th byte of each
// 7-byte address field.  Address offsets: dest=0, src=7, digi_n=14+(7*n).
// Verified against ax25_address_decode() and ax25_address_encode() in this file,
// both of which use (data[6] & 0x80) / (ssid_byte |= 0x80) for the C/H bit.

uint8_t ax25_get_h_bit(const uint8_t *frame_buf, size_t frame_len, uint8_t digi_idx) {
    // Validate pointer and index range (0..AX25_MAX_REPEATERS-1)
    if (!frame_buf || digi_idx >= AX25_MAX_REPEATERS)
        return 0xFFu;
    // Calculate byte offset: dest(7) + src(7) + digi_idx * 7, then +6 for SSID byte
    uint16_t offset = (uint16_t) (14u + (uint16_t) digi_idx * 7u);
    // Validate that the SSID byte is within the supplied buffer
    if ((size_t) (offset + 6u) >= frame_len)
        return 0xFFu;
    // H-bit is bit 7 of the SSID byte (confirmed: ax25_address_decode uses 0x80)
    return (frame_buf[offset + 6u] >> 7) & 0x01u;
}

void ax25_set_h_bit(uint8_t *frame_buf, size_t frame_len, uint8_t digi_idx) {
    // Validate pointer and index range (0..AX25_MAX_REPEATERS-1)
    if (!frame_buf || digi_idx >= AX25_MAX_REPEATERS)
        return;
    // Calculate byte offset: dest(7) + src(7) + digi_idx * 7, then +6 for SSID byte
    uint16_t offset = (uint16_t) (14u + (uint16_t) digi_idx * 7u);
    // Validate that the SSID byte is within the supplied buffer
    if ((size_t) (offset + 6u) >= frame_len)
        return;
    // Set H-bit (bit 7, mask 0x80) — confirmed: ax25_address_encode uses 0x80
    frame_buf[offset + 6u] |= 0x80u;
}

// Static dispatch table: fixed size, no dynamic allocation, MCU-safe.
// pid_count tracks the number of active entries (0..AX25_MAX_PID_HANDLERS).
static ax25_pid_entry_t pid_table[AX25_MAX_PID_HANDLERS];
static uint8_t pid_count = 0;

// ax25_register_pid: add a handler for pid to the dispatch table.
// If pid is already registered the call is ignored (first-wins policy).
// Returns 0 on success, 1 if table is full, 2 if fn is NULL.
uint8_t ax25_register_pid(uint8_t pid, ax25_pid_handler_fn fn, void *ctx) {
    uint8_t i;

    if (!fn)
        return 2;

    // Duplicate check: ignore if already registered
    for (i = 0; i < pid_count; i++) {
        if (pid_table[i].pid == pid)
            return 0;  // already registered - no error, first-wins
    }

    if (pid_count >= AX25_MAX_PID_HANDLERS)
        return 1;  // table full

    pid_table[pid_count].pid = pid;
    pid_table[pid_count].fn = fn;
    pid_table[pid_count].ctx = ctx;
    pid_count++;
    return 0;
}

// ax25_unregister_pid: remove the handler for pid.
// Compacts the table by shifting remaining entries down.
// Returns 0 on success, 1 if PID not found.
uint8_t ax25_unregister_pid(uint8_t pid) {
    uint8_t i;
    for (i = 0; i < pid_count; i++) {
        if (pid_table[i].pid == pid) {
            // Shift remaining entries left to fill the gap
            uint8_t j;
            for (j = i; j < (uint8_t) (pid_count - 1u); j++)
                pid_table[j] = pid_table[j + 1u];
            pid_count--;
            return 0;
        }
    }
    return 1;  // not found
}

// ax25_dispatch_pid: route received frame payload to registered handler.
// pid:  PID byte from the received I-frame or UI-frame.
// info: information field data pointer (does NOT include the PID byte).
// len:  length of info in bytes.
//
// Special cases handled per AX.25 v2.2 spec:
//   PID_SEGMENTATION (0x08): routed to registered SAR handler if any.
//   PID_ESCAPE (0xFF):       extended PID - second byte consumed and used
//                            as the actual PID for table lookup; info and
//                            len are adjusted past the extended PID byte.
//   PID_NO_L3 (0xF0):        plain text / no-layer-3; routed normally.
//
// Returns 0 if a handler was called, 1 if no handler registered for PID.
uint8_t ax25_dispatch_pid(uint8_t pid, const uint8_t *info, uint16_t len) {
    uint8_t i;
    uint8_t lookup_pid = pid;

    if (!info)
        return 1;

    // PID_ESCAPE (0xFF): consume the extended PID byte per AX.25 v2.2 section 6.5
    if (pid == PID_ESCAPE) {
        if (len < 1u)
            return 1;  // malformed extended PID frame
        lookup_pid = info[0];
        info++;      // advance past extended PID byte
        len--;       // adjust remaining length
    }

    // Linear scan of the fixed dispatch table
    for (i = 0; i < pid_count; i++) {
        if (pid_table[i].pid == lookup_pid) {
            pid_table[i].fn(info, len, pid_table[i].ctx);
            return 0;  // handler called
        }
    }

    // No handler registered for this PID - silently drop per spec
    return 1;
}

// ax25_pid_handler_count: return the number of currently registered handlers.
uint8_t ax25_pid_handler_count(void) {
    return pid_count;
}

// Static pool: AX25_POOL_SIZE slots, each holding up to AX25_MAX_INFO+1 bytes.
// No heap allocation; safe for MCUs with 32 KB SRAM or less.
static ax25_buf_t ax25_pool[AX25_POOL_SIZE];

// ax25_buf_alloc: find and return the first free slot.
// Returns NULL when the pool is exhausted (caller must handle gracefully).
ax25_buf_t* ax25_buf_alloc(void) {
    uint8_t i;
    for (i = 0; i < AX25_POOL_SIZE; i++) {
        if (!ax25_pool[i].in_use) {
            ax25_pool[i].in_use = 1;
            ax25_pool[i].len = 0;
            return &ax25_pool[i];
        }
    }
    return NULL;  // pool exhausted
}

// ax25_buf_free: mark a pool slot as free.
// NULL-safe: calling with NULL is a harmless no-op.
void ax25_buf_free(ax25_buf_t *b) {
    if (b)
        b->in_use = 0;
}

// ax25_buf_pool_free_count: count available (un-allocated) pool slots.
uint8_t ax25_buf_pool_free_count(void) {
    uint8_t i, count = 0;
    for (i = 0; i < AX25_POOL_SIZE; i++) {
        if (!ax25_pool[i].in_use)
            count++;
    }
    return count;
}

// ax25_encode_address_to_buf: encode one AX.25 address into dst[0..6].
// extension=true sets bit 0 of byte 6 (last-address marker).
// Mirrors ax25_address_encode() byte-for-byte but writes to a caller buffer;
// no malloc, no free.
static void ax25_encode_address_to_buf(uint8_t *dst, const ax25_address_t *addr,
bool extension) {
    size_t callsign_len = strlen(addr->callsign);
    int i;
    for (i = 0; i < 6; i++) {
        char c = (i < (int) callsign_len) ? addr->callsign[i] : ' ';
        dst[i] = (uint8_t) ((c & 0x7F) << 1);
    }
    uint8_t ssid_byte = (uint8_t) ((addr->ssid << 1) & 0x1E);
    if (addr->ch)
        ssid_byte |= 0x80u;
    ssid_byte |= 0x20u;  // res0 forced to 1 per AX.25 §3.12.2
    if (addr->res1)
        ssid_byte |= 0x40u;  // res1: caller-controlled (mod-128 signal)
    if (extension)
        ssid_byte |= 0x01u;
    dst[6] = ssid_byte;
}

uint8_t ax25_encode_frame_to_buf(const ax25_frame_t *frame, uint8_t *buf, size_t buf_size, size_t *out_len) {
    if (!frame || !buf || !out_len)
        return 1;

    // Determine modulo-128 flag (mirrors logic in ax25_frame_encode)
    bool is_modulo128 = (frame->type == AX25_FRAME_INFORMATION_16BIT || frame->type == AX25_FRAME_SUPERVISORY_RR_16BIT
            || frame->type == AX25_FRAME_SUPERVISORY_RNR_16BIT || frame->type == AX25_FRAME_SUPERVISORY_REJ_16BIT
            || frame->type == AX25_FRAME_SUPERVISORY_SREJ_16BIT || frame->type == AX25_FRAME_UNNUMBERED_SABME);

    // Build a local header copy so we can adjust ch/res1 without touching caller data
    ax25_frame_header_t hdr = frame->header;
    if (is_modulo128)
        hdr.source.res1 = false;  // signal mod-128 per §3.12

    // Apply C/H bits from CR flags (mirrors ax25_frame_header_encode)
    if (hdr.cr && !hdr.src_cr) {
        hdr.destination.ch = true;
        hdr.source.ch = false;
    } else if (!hdr.cr && hdr.src_cr) {
        hdr.destination.ch = false;
        hdr.source.ch = true;
    }

    int num_rep = hdr.repeaters.num_repeaters;
    size_t header_len = (size_t) (2 + num_rep) * 7u;

    // Encode header directly into buf — no malloc
    if (header_len > buf_size)
        return 2;

    size_t offset = 0;

    // Destination: extension bit always 0
    hdr.destination.extension = false;
    ax25_encode_address_to_buf(buf + offset, &hdr.destination, false);
    offset += 7u;

    // Source: extension bit set when no repeaters follow
    hdr.source.extension = (num_rep == 0);
    ax25_encode_address_to_buf(buf + offset, &hdr.source, (num_rep == 0));
    offset += 7u;

    // Repeaters
    int i;
    for (i = 0; i < num_rep; i++) {
        ax25_address_t rpt = hdr.repeaters.repeaters[i];
        bool last = (i == num_rep - 1);
        ax25_encode_address_to_buf(buf + offset, &rpt, last);
        offset += 7u;
    }

    // Encode payload (control + optional data) directly after header
    uint8_t *pbuf = buf + offset;
    size_t pspace = buf_size - offset;
    size_t plen = 0;

    switch (frame->type) {
        case AX25_FRAME_UNNUMBERED_SABM:
        case AX25_FRAME_UNNUMBERED_SABME:
        case AX25_FRAME_UNNUMBERED_DISC:
        case AX25_FRAME_UNNUMBERED_DM:
        case AX25_FRAME_UNNUMBERED_UA: {
            // 1-byte control field only
            if (pspace < 1u)
                return 2;
            const ax25_unnumbered_frame_t *uf = (const ax25_unnumbered_frame_t*) frame;
            pbuf[0] = (uint8_t) (uf->modifier | (uf->pf ? POLL_FINAL_8BIT : 0));
            plen = 1u;
            break;
        }
        case AX25_FRAME_UNNUMBERED_FRMR: {
            // 1 control + 3 or 5 data bytes
            const ax25_frame_reject_frame_t *f = (const ax25_frame_reject_frame_t*) frame;
            size_t need = f->is_modulo128 ? 6u : 4u;
            if (pspace < need)
                return 2;
            pbuf[0] = (uint8_t) (f->base.modifier | (f->base.pf ? POLL_FINAL_8BIT : 0));
            if (f->is_modulo128) {
                pbuf[1] = (uint8_t) (f->frmr_control & 0xFFu);
                pbuf[2] = (uint8_t) ((f->frmr_control >> 8) & 0xFFu);
                pbuf[3] = (uint8_t) (((f->vs & 0x7Fu) << 1) | (f->frmr_cr ? 0x01u : 0u));
                pbuf[4] = (uint8_t) ((f->vr & 0x7Fu) << 1);
                pbuf[5] = (uint8_t) ((f->w ? 0x01u : 0u) | (f->x ? 0x02u : 0u) | (f->y ? 0x04u : 0u) | (f->z ? 0x08u : 0u));
            } else {
                pbuf[1] = (uint8_t) (f->frmr_control & 0xFFu);
                pbuf[2] = (uint8_t) (((f->vr & 0x07u) << 5) | (f->frmr_cr ? 0x10u : 0u) | ((f->vs & 0x07u) << 1));
                pbuf[3] = (uint8_t) ((f->w ? 0x01u : 0u) | (f->x ? 0x02u : 0u) | (f->y ? 0x04u : 0u) | (f->z ? 0x08u : 0u));
            }
            plen = need;
            break;
        }
        case AX25_FRAME_SUPERVISORY_RR_8BIT:
        case AX25_FRAME_SUPERVISORY_RNR_8BIT:
        case AX25_FRAME_SUPERVISORY_REJ_8BIT:
        case AX25_FRAME_SUPERVISORY_SREJ_8BIT: {
            if (pspace < 1u)
                return 2;
            const ax25_supervisory_frame_t *sf = (const ax25_supervisory_frame_t*) frame;
            uint8_t code_bits = (uint8_t) ((sf->code << 2) & 0x0Cu);
            pbuf[0] = (uint8_t) (((sf->nr << 5) & 0xE0u) | (sf->pf ? POLL_FINAL_8BIT : 0u) | code_bits | CONTROL_S_VAL);
            plen = 1u;
            break;
        }
        case AX25_FRAME_SUPERVISORY_RR_16BIT:
        case AX25_FRAME_SUPERVISORY_RNR_16BIT:
        case AX25_FRAME_SUPERVISORY_REJ_16BIT:
        case AX25_FRAME_SUPERVISORY_SREJ_16BIT: {
            if (pspace < 2u)
                return 2;
            const ax25_supervisory_frame_t *sf = (const ax25_supervisory_frame_t*) frame;
            uint8_t code_bits = (uint8_t) ((sf->code << 2) & 0x0Cu);
            uint16_t ctrl = (uint16_t) (((sf->nr << 9) & 0xFE00u) | (sf->pf ? POLL_FINAL_16BIT : 0u) | code_bits | CONTROL_S_VAL);
            pbuf[0] = (uint8_t) (ctrl & 0xFFu);
            pbuf[1] = (uint8_t) ((ctrl >> 8) & 0xFFu);
            plen = 2u;
            break;
        }
        case AX25_FRAME_UNNUMBERED_TEST: {
            // 1 control + payload
            const ax25_test_frame_t *tf = (const ax25_test_frame_t*) frame;
            size_t need = 1u + tf->payload_len;
            if (pspace < need)
                return 2;
            pbuf[0] = (uint8_t) (tf->base.modifier | (tf->base.pf ? POLL_FINAL_8BIT : 0));
            if (tf->payload_len > 0 && tf->payload)
                memcpy(pbuf + 1, tf->payload, tf->payload_len);
            plen = need;
            break;
        }
        default:
            // I-frames and XID/UI are not supported here; use ax25_frame_encode()
            return 3;
    }

    *out_len = offset + plen;
    return 0;
}
