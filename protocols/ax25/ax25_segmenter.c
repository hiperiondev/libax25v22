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
#include <stdlib.h>

#include "ax25_segmenter.h"

// Reassembly timeout: 30 seconds at 10ms tick resolution
#define SEG_TIMEOUT_TICKS 3000

// Encode segment header into 1 byte
// AX.25 v2.2 format: [F][L][S5][S4][S3][S2][S1][S0]
// F = First segment flag (bit 7)
// L = Last segment flag (bit 6)
// S5-S0 = Sequence number (bits 5-0, range 0-63)
static uint8_t encode_segment_header(ax25_segment_header_t *hdr) {
    uint8_t byte = 0;
    if (hdr->first_segment) {
        byte |= 0x80;  // Set bit 7
    }
    if (hdr->last_segment) {
        byte |= 0x40;  // Set bit 6
    }
    byte |= (hdr->sequence & 0x3F);  // Bits 5-0
    return byte;
}

// Decode segment header from 1 byte
static void decode_segment_header(uint8_t byte, ax25_segment_header_t *hdr) {
    hdr->first_segment = (byte & 0x80) != 0;
    hdr->last_segment = (byte & 0x40) != 0;
    hdr->sequence = byte & 0x3F;
}

// Initialize segmenter context
uint8_t ax25_segmenter_init(ax25_segmenter_t *seg, uint16_t max_iframe_size) {
    if (!seg) {
        return 1;
    }

    memset(seg, 0, sizeof(ax25_segmenter_t));
    seg->state = SEG_STATE_IDLE;

    // Calculate segment size: N1 - 2 bytes overhead (1 for seg header + 1 for original PID)
    if (max_iframe_size > 2) {
        seg->segment_size = max_iframe_size - 2;
    } else {
        seg->segment_size = 254;  // Fallback minimum
    }

    return 0;
}

// Segment and send large data
uint8_t ax25_segmenter_send(ax25_segmenter_t *seg, uint8_t *data, uint16_t len, uint8_t original_pid) {
    if (!seg || !data || len == 0) {
        return 1;
    }

    if (seg->state != SEG_STATE_IDLE) {
        return 2;  // Already segmenting or reassembling
    }

    // Calculate total segments needed
    // Avoid division on simple MCUs - use repeated subtraction
    uint16_t remaining = len;
    uint8_t segments = 0;

    while (remaining > 0) {
        if (remaining > seg->segment_size) {
            remaining -= seg->segment_size;
        } else {
            remaining = 0;
        }
        segments++;

        if (segments > 63) {
            return 3;  // Too many segments (max 63 per spec)
        }
    }

    // Initialize segmentation state
    seg->total_segments = segments;
    seg->current_segment = 0;
    seg->bytes_sent = 0;
    seg->pending_data = data;
    seg->pending_length = len;
    seg->pending_pid = original_pid;
    seg->state = SEG_STATE_SEGMENTING;

    // Transmit all segments sequentially
    for (uint8_t i = 0; i < segments; i++) {
        ax25_segment_header_t hdr;
        hdr.first_segment = (i == 0);
        hdr.last_segment = (i == segments - 1);
        hdr.sequence = i;

        // Calculate offset and size for this segment
        // Use 16-bit arithmetic only (no 32-bit multiplication)
        uint16_t offset = 0;
        for (uint8_t j = 0; j < i; j++) {
            offset += seg->segment_size;
        }

        uint16_t chunk_size = seg->segment_size;
        if (offset + chunk_size > len) {
            chunk_size = len - offset;
        }

        // Build segment frame in static buffer
        // Format: [seg_header][original_pid][data_chunk]
        uint8_t segment_frame[260];  // Max segment size + 2 bytes overhead

        segment_frame[0] = encode_segment_header(&hdr);
        segment_frame[1] = original_pid;
        memcpy(&segment_frame[2], data + offset, chunk_size);

        // Transmit segment as I-frame with PID = 0x08
        if (seg->transmit_iframe) {
            seg->transmit_iframe(segment_frame, chunk_size + 2, AX25_PID_SEGMENT_FRAGMENT, seg->user_data);
        }

        seg->bytes_sent += chunk_size;
        seg->current_segment = i + 1;
    }

    // Segmentation complete
    seg->state = SEG_STATE_IDLE;
    seg->pending_data = NULL;
    seg->pending_length = 0;

    return 0;
}

// Process received segment
void ax25_segmenter_receive(ax25_segmenter_t *seg, uint8_t *data, uint16_t len, uint8_t pid) {
    if (!seg || !data || len < 3) {
        return;  // Invalid parameters or data too short
    }

    if (pid != AX25_PID_SEGMENT_FRAGMENT) {
        return;  // Not a segment frame
    }

    // Decode segment header
    ax25_segment_header_t hdr;
    decode_segment_header(data[0], &hdr);

    uint8_t original_pid = data[1];
    uint8_t *payload = data + 2;
    uint16_t payload_len = len - 2;

    // First segment received - initialize reassembly
    if (hdr.first_segment) {
        seg->rx_buffer_used = 0;
        seg->rx_expected_segment = 0;
        seg->rx_segment_bitmap = 0;
        seg->rx_first_received = true;
        seg->rx_original_pid = original_pid;
        seg->state = SEG_STATE_REASSEMBLING;
        seg->rx_timeout_tick = 0;  // Will be set by tick handler
    }

    // Verify we're in reassembly state
    if (seg->state != SEG_STATE_REASSEMBLING) {
        return;  // Not reassembling or wrong state
    }

    // Check sequence number
    if (hdr.sequence != seg->rx_expected_segment) {
        // Out of order segment received
        // For simplicity, abort reassembly (could implement buffering)
        seg->state = SEG_STATE_IDLE;
        seg->rx_buffer_used = 0;
        return;
    }

    // Check buffer overflow
    if (seg->rx_buffer_used + payload_len > sizeof(seg->rx_buffer)) {
        // Buffer overflow - abort reassembly
        seg->state = SEG_STATE_IDLE;
        seg->rx_buffer_used = 0;
        return;
    }

    // Append payload to reassembly buffer
    memcpy(seg->rx_buffer + seg->rx_buffer_used, payload, payload_len);
    seg->rx_buffer_used += payload_len;
    seg->rx_expected_segment++;

    // Mark segment as received in bitmap (if using bitmap tracking)
    if (hdr.sequence < 8) {
        seg->rx_segment_bitmap |= (1U << hdr.sequence);
    }

    // Check if this is the last segment
    if (hdr.last_segment) {
        // Reassembly complete - deliver to upper layer
        if (seg->on_reassembly_complete) {
            seg->on_reassembly_complete(seg->rx_buffer, seg->rx_buffer_used, seg->rx_original_pid, seg->user_data);
        }

        // Reset state
        seg->state = SEG_STATE_IDLE;
        seg->rx_buffer_used = 0;
        seg->rx_expected_segment = 0;
        seg->rx_segment_bitmap = 0;
        seg->rx_first_received = false;
    }
}

// Timer tick handler for timeout management
void ax25_segmenter_tick(ax25_segmenter_t *seg, uint32_t current_tick) {
    if (!seg) {
        return;
    }

    // Check reassembly timeout
    if (seg->state == SEG_STATE_REASSEMBLING) {
        // Initialize timeout on first tick in this state
        if (seg->rx_timeout_tick == 0) {
            seg->rx_timeout_tick = current_tick;
        }

        // Check if timeout exceeded
        if ((current_tick - seg->rx_timeout_tick) > SEG_TIMEOUT_TICKS) {
            // Timeout - abort reassembly
            seg->state = SEG_STATE_IDLE;
            seg->rx_buffer_used = 0;
            seg->rx_expected_segment = 0;
            seg->rx_segment_bitmap = 0;
            seg->rx_first_received = false;
            seg->rx_timeout_tick = 0;
        }
    } else {
        // Not reassembling - reset timeout
        seg->rx_timeout_tick = 0;
    }
}
