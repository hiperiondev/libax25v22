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
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "common.h"
#include "ax25.h"
#include "hdlc.h"
#include "hal.h"

// Helper function to check if data is printable ASCII
bool is_printable_ascii(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (data[i] < 32 || data[i] > 126) {
            return false;
        }
    }
    return true;
}

// Helper function to get frame type string
const char* frame_type_to_str(ax25_frame_type_t type) {
    switch (type) {
        case AX25_FRAME_RAW:
            return "Raw";
        case AX25_FRAME_UNNUMBERED_INFORMATION:
            return "Unnumbered Information (UI)";
        case AX25_FRAME_UNNUMBERED_SABM:
            return "Set Asynchronous Balanced Mode (SABM)";
        case AX25_FRAME_UNNUMBERED_SABME:
            return "Set Asynchronous Balanced Mode Extended (SABME)";
        case AX25_FRAME_UNNUMBERED_DISC:
            return "Disconnect (DISC)";
        case AX25_FRAME_UNNUMBERED_DM:
            return "Disconnected Mode (DM)";
        case AX25_FRAME_UNNUMBERED_UA:
            return "Unnumbered Acknowledge (UA)";
        case AX25_FRAME_UNNUMBERED_FRMR:
            return "Frame Reject (FRMR)";
        case AX25_FRAME_UNNUMBERED_XID:
            return "Exchange Identification (XID)";
        case AX25_FRAME_UNNUMBERED_TEST:
            return "Test";
        case AX25_FRAME_INFORMATION_8BIT:
            return "Information (I) modulo-8";
        case AX25_FRAME_INFORMATION_16BIT:
            return "Information (I) modulo-128";
        case AX25_FRAME_SUPERVISORY_RR_8BIT:
            return "Receive Ready (RR) modulo-8";
        case AX25_FRAME_SUPERVISORY_RNR_8BIT:
            return "Receive Not Ready (RNR) modulo-8";
        case AX25_FRAME_SUPERVISORY_REJ_8BIT:
            return "Reject (REJ) modulo-8";
        case AX25_FRAME_SUPERVISORY_SREJ_8BIT:
            return "Selective Reject (SREJ) modulo-8";
        case AX25_FRAME_SUPERVISORY_RR_16BIT:
            return "Receive Ready (RR) modulo-128";
        case AX25_FRAME_SUPERVISORY_RNR_16BIT:
            return "Receive Not Ready (RNR) modulo-128";
        case AX25_FRAME_SUPERVISORY_REJ_16BIT:
            return "Reject (REJ) modulo-128";
        case AX25_FRAME_SUPERVISORY_SREJ_16BIT:
            return "Selective Reject (SREJ) modulo-128";
        default:
            return "Unknown";
    }
}

// Helper function to print hex data
void print_hex(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

// Main function to print AX.25 frame
void ax25_frame_print(unsigned char *ax25_frame, int ax25_frame_len) {
    uint8_t err = 0;
    ax25_frame_t *frame = ax25_frame_decode((const uint8_t*) ax25_frame, (size_t) ax25_frame_len, MODULO128_AUTO, &err);
    if (frame == NULL) {
        printf("Failed to decode frame: error %d\n", err);
        return;
    }

    // Print frame type
    const char *type_str = frame_type_to_str(frame->type);
    printf("Frame Type: %s\n", type_str);

    // Print addresses
    ax25_frame_header_t *header = &frame->header;
    // Determine command or response
    if (header->cr && !header->src_cr) {
        printf("Frame is a Command\n");
    } else if (!header->cr && header->src_cr) {
        printf("Frame is a Response\n");
    } else {
        printf("Invalid C bits combination\n");
    }
    printf("Destination: %s-%d\n", header->destination.callsign, header->destination.ssid);
    printf("Source: %s-%d\n", header->source.callsign, header->source.ssid);
    if (header->repeaters.num_repeaters > 0) {
        printf("Repeaters:\n");
        for (int i = 0; i < header->repeaters.num_repeaters; i++) {
            ax25_address_t *rep = &header->repeaters.repeaters[i];
            printf("  %s-%d (ch=%d)\n", rep->callsign, rep->ssid, rep->ch);
        }
    }

    // Print specific fields based on frame type
    switch (frame->type) {
        case AX25_FRAME_RAW: {
            ax25_raw_frame_t *raw_frame = (ax25_raw_frame_t*) frame;
            printf("Control: 0x%02X\n", raw_frame->control);
            printf("Payload (%zu bytes): ", raw_frame->payload_len);
            print_hex(raw_frame->payload, raw_frame->payload_len);
            break;
        }
        case AX25_FRAME_UNNUMBERED_INFORMATION: {
            ax25_unnumbered_information_frame_t *ui_frame = (ax25_unnumbered_information_frame_t*) frame;
            printf("Control: 0x%02X (UI, P/F=%d)\n", ui_frame->base.modifier, ui_frame->base.pf);
            printf("PID: 0x%02X\n", ui_frame->pid);
            if (is_printable_ascii(ui_frame->payload, ui_frame->payload_len)) {
                printf("Payload: %.*s\n", (int) ui_frame->payload_len, ui_frame->payload);
            } else {
                printf("Payload (%zu bytes): ", ui_frame->payload_len);
                print_hex(ui_frame->payload, ui_frame->payload_len);
            }
            break;
        }
        case AX25_FRAME_UNNUMBERED_SABM:
        case AX25_FRAME_UNNUMBERED_SABME:
        case AX25_FRAME_UNNUMBERED_DISC:
        case AX25_FRAME_UNNUMBERED_DM:
        case AX25_FRAME_UNNUMBERED_UA:
        case AX25_FRAME_UNNUMBERED_TEST: {
            ax25_unnumbered_frame_t *u_frame = (ax25_unnumbered_frame_t*) frame;
            const char *u_type = frame_type_to_str(frame->type);
            printf("Control: %s, P/F=%d\n", u_type, u_frame->pf);
            if (frame->type == AX25_FRAME_UNNUMBERED_TEST) {
                ax25_test_frame_t *test_frame = (ax25_test_frame_t*) frame;
                if (is_printable_ascii(test_frame->payload, test_frame->payload_len)) {
                    printf("Payload: %.*s\n", (int) test_frame->payload_len, test_frame->payload);
                } else {
                    printf("Payload (%zu bytes): ", test_frame->payload_len);
                    print_hex(test_frame->payload, test_frame->payload_len);
                }
            }
            break;
        }
        case AX25_FRAME_UNNUMBERED_FRMR: {
            ax25_frame_reject_frame_t *frmr_frame = (ax25_frame_reject_frame_t*) frame;
            printf("Control: FRMR, P/F=%d\n", frmr_frame->base.pf);
            printf("FRMR Control: 0x%04X\n", frmr_frame->frmr_control);
            printf("VS: %d, VR: %d, C/R: %d\n", frmr_frame->vs, frmr_frame->vr, frmr_frame->frmr_cr);
            printf("Flags: W=%d, X=%d, Y=%d, Z=%d\n", frmr_frame->w, frmr_frame->x, frmr_frame->y, frmr_frame->z);
            break;
        }
        case AX25_FRAME_UNNUMBERED_XID: {
            ax25_exchange_identification_frame_t *xid_frame = (ax25_exchange_identification_frame_t*) frame;
            printf("Control: XID, P/F=%d\n", xid_frame->base.pf);
            printf("FI: 0x%02X, GI: 0x%02X\n", xid_frame->fi, xid_frame->gi);
            printf("Parameters:\n");
            for (size_t i = 0; i < xid_frame->param_count; i++) {
                ax25_xid_parameter_t *param = xid_frame->parameters[i];
                printf("  PI: %d\n", param->pi);
            }
            break;
        }
        case AX25_FRAME_INFORMATION_8BIT:
        case AX25_FRAME_INFORMATION_16BIT: {
            ax25_information_frame_t *i_frame = (ax25_information_frame_t*) frame;
            int modulo = (frame->type == AX25_FRAME_INFORMATION_8BIT) ? 8 : 128;
            printf("Control: N(S)=%d, N(R)=%d, P/F=%d (modulo %d)\n", i_frame->ns, i_frame->nr, i_frame->pf, modulo);
            printf("PID: 0x%02X\n", i_frame->pid);
            if (is_printable_ascii(i_frame->payload, i_frame->payload_len)) {
                printf("Payload: %.*s\n", (int) i_frame->payload_len, i_frame->payload);
            } else {
                printf("Payload (%zu bytes): ", i_frame->payload_len);
                print_hex(i_frame->payload, i_frame->payload_len);
            }
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
            ax25_supervisory_frame_t *s_frame = (ax25_supervisory_frame_t*) frame;
            const char *s_type = "";
            int modulo = 8;
            if (frame->type >= AX25_FRAME_SUPERVISORY_RR_16BIT) {
                modulo = 128;
            }
            switch (frame->type) {
                case AX25_FRAME_SUPERVISORY_RR_8BIT:
                case AX25_FRAME_SUPERVISORY_RR_16BIT:
                    s_type = "RR";
                break;
                case AX25_FRAME_SUPERVISORY_RNR_8BIT:
                case AX25_FRAME_SUPERVISORY_RNR_16BIT:
                    s_type = "RNR";
                break;
                case AX25_FRAME_SUPERVISORY_REJ_8BIT:
                case AX25_FRAME_SUPERVISORY_REJ_16BIT:
                    s_type = "REJ";
                break;
                case AX25_FRAME_SUPERVISORY_SREJ_8BIT:
                case AX25_FRAME_SUPERVISORY_SREJ_16BIT:
                    s_type = "SREJ";
                break;
                default:
                break;
            }
            printf("Control: %s, N(R)=%d, P/F=%d (modulo %d)\n", s_type, s_frame->nr, s_frame->pf, modulo);
            break;
        }
        default:
            printf("Unsupported frame type\n");
        break;
    }

    printf("Note: FCS is not included in the input frame.\n");

    // Free the frame
    ax25_frame_free(frame, &err);
}

void hdlc_frame_print(unsigned char *hdlc_frame, int hdlc_len) {
    if (hdlc_len < 19) {  // Minimum: 2 flags + 14 address + 1 control + 2 FCS
        printf("Invalid HDLC frame: too short\n");
        return;
    }
    if (hdlc_frame[0] != 0x7E || hdlc_frame[hdlc_len - 1] != 0x7E) {
        printf("Invalid HDLC frame: missing flags\n");
        return;
    }

    // Decode HDLC frame to get AX.25 frame using proper HDLC decoder
    unsigned char *ax25_frame_original = malloc(hdlc_len);  // Allocate enough space
    if (!ax25_frame_original) {
        printf("Memory allocation failed\n");
        return;
    }

    int ax25_len = 0;
    if (hdlc_frame_decode(hdlc_frame, hdlc_len, ax25_frame_original, &ax25_len) != 0) {
        printf("Failed to decode HDLC frame\n");
        free(ax25_frame_original);
        return;
    }

    // Extract FCS from decoded frame (last 2 bytes of decoded data before CRC check removed them)
    // Actually hdlc_frame_decode returns frame without FCS, so we need to get FCS differently
    // The FCS is the last 2 bytes before the end flag in the HDLC frame
    int content_len = hdlc_len - 2;  // Without flags
    unsigned char *content = malloc(content_len);
    if (!content) {
        printf("Memory allocation failed\n");
        free(ax25_frame_original);
        return;
    }
    memcpy(content, &hdlc_frame[1], content_len);

    // Calculate CRC on the decoded content (which is already bit-unstuffed and in correct order)
    // We need to reconstruct the frame with FCS for CRC calculation
    unsigned char *frame_with_fcs = malloc(ax25_len + 2);
    if (!frame_with_fcs) {
        printf("Memory allocation failed\n");
        free(ax25_frame_original);
        free(content);
        return;
    }
    memcpy(frame_with_fcs, ax25_frame_original, ax25_len);
    // FCS in HDLC content is at the end (last 2 bytes before flag)
    frame_with_fcs[ax25_len] = content[content_len - 2];
    frame_with_fcs[ax25_len + 1] = content[content_len - 1];

    // Use hal_crc16_buf() directly; CRC() wrapper has been removed.
    // hal_crc16_buf computes CRC-CCITT (init 0xFFFF, final XOR 0xFFFF) over data only.
    uint16_t calculated_crc = hal_crc16_buf(ax25_frame_original, (uint16_t) ax25_len);

    // FCS in HDLC is stored LSB-first: low byte at content_len-2, high byte at content_len-1
    uint16_t fcs = (uint16_t) (content[content_len - 2]) | ((uint16_t) (content[content_len - 1]) << 8);

    // Print start flag
    printf("Start Flag: 0x7E\n");

    // Print AX.25 frame
    printf("AX.25 Frame:\n");
    ax25_frame_print(ax25_frame_original, ax25_len);

    // Print FCS
    printf("FCS: 0x%04X\n", fcs);

    // Check if CRC matches
    if (calculated_crc == fcs) {
        printf("FCS check: OK\n");
    } else {
        printf("FCS check: Failed (calculated 0x%04X)\n", calculated_crc);
    }

    // Print end flag
    printf("End Flag: 0x7E\n");

    free(ax25_frame_original);
    free(content);
    free(frame_with_fcs);
}
