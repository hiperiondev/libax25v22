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

// Helper function to set a bit in the bitmap
static void set_segment_received(ax25_segmenter_t *seg, uint8_t sequence) {
    if (sequence >= 64) {
        return;  // Invalid sequence
    }
    uint8_t byte_index = sequence / 8;
    uint8_t bit_index = sequence % 8;
    seg->rx_segment_bitmap[byte_index] |= (1U << bit_index);
}

static bool buffer_out_of_order_segment(ax25_segmenter_t *seg, uint8_t *payload, uint16_t payload_len, uint8_t sequence,
bool is_last) {
    // Validate sequence range - AX.25 v2.2 uses 6-bit sequence numbers (0-63)
    if (sequence >= 64) {
        return false;
    }

    // Check if already buffered (duplicate)
    for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
        if (seg->ooo_buffer[i].valid && seg->ooo_buffer[i].sequence == sequence) {
            return true;  // Silently accept duplicate
        }
    }

    // Validate payload size before attempting to store
    if (payload_len > sizeof(seg->ooo_buffer[0].data)) {
        return false;  // Segment too large
    }

    // Find free slot
    for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
        if (!seg->ooo_buffer[i].valid) {
            memcpy(seg->ooo_buffer[i].data, payload, payload_len);
            seg->ooo_buffer[i].length = payload_len;
            seg->ooo_buffer[i].sequence = sequence;
            seg->ooo_buffer[i].is_last = is_last;
            seg->ooo_buffer[i].valid = true;
            seg->ooo_count++;
            return true;
        }
    }

    // Buffer full - try intelligent eviction
    // Strategy: evict the segment farthest from rx_expected_segment
    // This maximizes chance of near-term delivery

    uint8_t max_distance = 0;
    uint8_t evict_idx = 0xFF;

    for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
        if (seg->ooo_buffer[i].valid) {
            // Calculate distance from expected (with wraparound handling)
            uint8_t distance;
            if (seg->ooo_buffer[i].sequence >= seg->rx_expected_segment) {
                distance = seg->ooo_buffer[i].sequence - seg->rx_expected_segment;
            } else {
                distance = 64 + seg->ooo_buffer[i].sequence - seg->rx_expected_segment;
            }

            if (distance > max_distance) {
                max_distance = distance;
                evict_idx = i;
            }
        }
    }

    // Calculate distance for new segment
    uint8_t new_distance;
    if (sequence >= seg->rx_expected_segment) {
        new_distance = sequence - seg->rx_expected_segment;
    } else {
        new_distance = 64 + sequence - seg->rx_expected_segment;
    }

    // Only evict if new segment is closer than farthest buffered segment
    if (evict_idx != 0xFF && new_distance < max_distance) {
        // Evict and insert new segment
        memcpy(seg->ooo_buffer[evict_idx].data, payload, payload_len);
        seg->ooo_buffer[evict_idx].length = payload_len;
        seg->ooo_buffer[evict_idx].sequence = sequence;
        seg->ooo_buffer[evict_idx].is_last = is_last;
        // valid flag already set
        return true;
    }

    // Can't buffer - but this is not a fatal error
    // Segment will be retransmitted by sender
    return false;
}

static bool process_buffered_segments(ax25_segmenter_t *seg) {
    bool progress = true;

    while (progress) {
        progress = false;

        // Look for next expected segment in buffer
        for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
            if (seg->ooo_buffer[i].valid && seg->ooo_buffer[i].sequence == seg->rx_expected_segment) {

                // Found next segment - append it
                uint16_t payload_len = seg->ooo_buffer[i].length;

                // Check buffer space
                if (seg->rx_buffer_used + payload_len > sizeof(seg->rx_buffer)) {
                    return false;  // Overflow
                }

                // Append to reassembly buffer
                memcpy(seg->rx_buffer + seg->rx_buffer_used, seg->ooo_buffer[i].data, payload_len);
                seg->rx_buffer_used += payload_len;
                seg->rx_expected_segment++;

                // Set last received if this was the last segment
                if (seg->ooo_buffer[i].is_last) {
                    seg->rx_last_received = true;
                }

                set_segment_received(seg, seg->ooo_buffer[i].sequence);

                // Free slot
                seg->ooo_buffer[i].valid = false;
                seg->ooo_count--;

                progress = true;  // Keep checking for more
                break;
            }
        }
    }

    return true;
}

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

    // Initialize gap detection threshold - report error if 8 segments received past a gap
    // Per AX.25 v2.2 Appendix C6, this prevents indefinite waiting for missing segments
    seg->rx_gap_threshold = 8;  // Configurable threshold

    return 0;
}

void ax25_segmenter_tick(ax25_segmenter_t *seg, uint32_t current_tick) {
    if (!seg) {
        return;
    }

    // Only process timeout during reassembly state
    if (seg->state != SEG_STATE_REASSEMBLING) {
        return;
    }

    // Skip timeout check if timer not initialized
    if (seg->rx_timeout_tick == 0) {
        return;
    }

    // Use signed arithmetic to handle wraparound correctly
    // This prevents issues when current_tick wraps around UINT32_MAX
    int32_t time_until_timeout = (int32_t) (seg->rx_timeout_tick - current_tick);

    if (time_until_timeout <= 0) {
        // TR210 timer expired - per AX.25 v2.2 Appendix C6:
        // "If TR210 expires, all buffered segments shall be discarded and
        // an error reported to the layer 3 entity"

        // Clean up reassembly state
        seg->state = SEG_STATE_IDLE;
        seg->rx_buffer_used = 0;
        seg->rx_expected_segment = 0;
        // Clear full 8-byte bitmap array
        memset(seg->rx_segment_bitmap, 0, sizeof(seg->rx_segment_bitmap));
        seg->rx_first_received = false;
        seg->rx_timeout_tick = 0;
        seg->rx_last_received = false;

        // Clear out-of-order segment buffer
        seg->ooo_count = 0;
        for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
            seg->ooo_buffer[i].valid = false;
        }

        // Report timeout error to layer 3 via callback
        if (seg->on_reassembly_error) {
            seg->on_reassembly_error(AX25_SEG_ERROR_TIMEOUT, seg->user_data);
        }
    }
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

void ax25_segmenter_receive(ax25_segmenter_t *seg, uint8_t *data, uint16_t len, uint8_t pid, uint32_t current_tick) {
    if (!seg || !data || len < 3) {
        return;
    }
    if (pid != AX25_PID_SEGMENT_FRAGMENT) {
        return;
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
        // Clear full 8-byte bitmap array
        memset(seg->rx_segment_bitmap, 0, sizeof(seg->rx_segment_bitmap));
        seg->rx_first_received = true;
        seg->rx_original_pid = original_pid;
        seg->state = SEG_STATE_REASSEMBLING;
        seg->rx_timeout_tick = current_tick + SEG_TIMEOUT_TICKS;

        // Initialize out-of-order tracking
        seg->ooo_count = 0;
        seg->rx_highest_received = 0;
        seg->rx_last_received = false;  // Corrected: Initialize last received flag
        seg->rx_gap_count = 0;  // Initialize gap counter
        for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
            seg->ooo_buffer[i].valid = false;
        }
    }
    // Verify we're in reassembly state
    if (seg->state != SEG_STATE_REASSEMBLING) {
        return;
    }
    // Track highest received sequence
    if (hdr.sequence > seg->rx_highest_received) {
        seg->rx_highest_received = hdr.sequence;
    }
    // Handle out-of-order segments
    if (hdr.sequence == seg->rx_expected_segment) {
        // In-order segment - process immediately

        // Check buffer overflow
        if (seg->rx_buffer_used + payload_len > sizeof(seg->rx_buffer)) {
            seg->state = SEG_STATE_IDLE;
            seg->rx_buffer_used = 0;
            seg->rx_timeout_tick = 0;

            // Clear out-of-order buffer
            seg->ooo_count = 0;
            for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
                seg->ooo_buffer[i].valid = false;
            }

            if (seg->on_reassembly_error) {
                seg->on_reassembly_error(AX25_SEG_ERROR_OVERFLOW, seg->user_data);
            }
            return;
        }
        // Append payload
        memcpy(seg->rx_buffer + seg->rx_buffer_used, payload, payload_len);
        seg->rx_buffer_used += payload_len;
        seg->rx_expected_segment++;
        // Set last received if this is last
        if (hdr.last_segment) {
            seg->rx_last_received = true;
        }
        // Use helper function for bitmap tracking
        set_segment_received(seg, hdr.sequence);

        // Reset gap counter - we're making progress with in-order reception
        seg->rx_gap_count = 0;

        // Process any buffered segments that can now be delivered
        if (!process_buffered_segments(seg)) {
            // Overflow during buffered segment processing
            seg->state = SEG_STATE_IDLE;
            seg->rx_buffer_used = 0;
            seg->rx_timeout_tick = 0;
            seg->ooo_count = 0;

            if (seg->on_reassembly_error) {
                seg->on_reassembly_error(AX25_SEG_ERROR_OVERFLOW, seg->user_data);
            }
            return;
        }
        // Restart TR210 per spec using direct deadline calculation
        seg->rx_timeout_tick = current_tick + SEG_TIMEOUT_TICKS;

    } else if (hdr.sequence > seg->rx_expected_segment) {
        // Out-of-order segment (future) - indicates gap

        // Calculate gap size
        uint8_t gap_size = hdr.sequence - seg->rx_expected_segment;

        // Request retransmission of missing segments if SREJ callback available
        // Per AX.25 v2.2 Section 6.4.4, SREJ can request specific missing frames
        // Only request if gap is small (<=4 segments) to avoid overwhelming sender
        if (gap_size <= 4 && seg->on_request_retransmit) {
            // Request retransmission for each missing segment
            for (uint8_t missing = seg->rx_expected_segment; missing < hdr.sequence; missing++) {
                // Check if we already have this segment buffered
                bool already_buffered = false;
                for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
                    if (seg->ooo_buffer[i].valid && seg->ooo_buffer[i].sequence == missing) {
                        already_buffered = true;
                        break;
                    }
                }

                // Only request if not already buffered
                if (!already_buffered) {
                    seg->on_request_retransmit(missing, seg->user_data);
                }
            }
        }

        if (!buffer_out_of_order_segment(seg, payload, payload_len, hdr.sequence, hdr.last_segment)) {  // Pass is_last
            // Buffer full or error - continue but log
            // Don't abort - wait for retransmission or timeout
            if (seg->on_reassembly_error) {
                seg->on_reassembly_error(AX25_SEG_ERROR_SEQUENCE, seg->user_data);
            }
        }

        // Increment gap counter - we're receiving segments beyond missing one(s)
        seg->rx_gap_count++;

        // Check if we've received too many segments beyond the gap
        // This indicates persistent loss that won't be recovered
        // Per AX.25 v2.2 Appendix C6, report error rather than waiting indefinitely
        if (seg->rx_gap_count > seg->rx_gap_threshold) {
            // Abort reassembly - missing segment likely unrecoverable
            seg->state = SEG_STATE_IDLE;
            seg->rx_buffer_used = 0;
            seg->rx_expected_segment = 0;
            memset(seg->rx_segment_bitmap, 0, sizeof(seg->rx_segment_bitmap));
            seg->rx_first_received = false;
            seg->rx_timeout_tick = 0;
            seg->rx_last_received = false;
            seg->ooo_count = 0;
            seg->rx_gap_count = 0;

            for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
                seg->ooo_buffer[i].valid = false;
            }

            if (seg->on_reassembly_error) {
                seg->on_reassembly_error(AX25_SEG_ERROR_SEQUENCE, seg->user_data);
            }
            return;
        }

        // Restart TR210 even for out-of-order segments per AX.25 v2.2 Appendix C6
        // Spec requires timer restart on ANY segment receipt to indicate link activity
        seg->rx_timeout_tick = current_tick + SEG_TIMEOUT_TICKS;

    } else {
        // Duplicate segment (sequence < expected) - ignore silently
        // This is normal in radio environments with retransmissions
    }
    // Check for completion after processing (handles last segment from buffer)
    if (seg->rx_last_received && seg->ooo_count == 0) {
        if (seg->on_reassembly_complete) {
            seg->on_reassembly_complete(seg->rx_buffer, seg->rx_buffer_used, seg->rx_original_pid, seg->user_data);
        }
        // Reset state
        seg->state = SEG_STATE_IDLE;
        seg->rx_buffer_used = 0;
        seg->rx_expected_segment = 0;
        // Clear full 8-byte bitmap array
        memset(seg->rx_segment_bitmap, 0, sizeof(seg->rx_segment_bitmap));
        seg->rx_first_received = false;
        seg->rx_timeout_tick = 0;
        seg->ooo_count = 0;
        seg->rx_last_received = false;
    }
}
