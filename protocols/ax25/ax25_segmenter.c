/**
 * @file ax25_segmenter.c
 * @brief AX.25 v2.2 Protocol Library - Segmentation and Reassembly Module
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
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

uint8_t ax25_segmenter_send(ax25_segmenter_t *seg, uint8_t *data, uint16_t len, uint8_t original_pid) {
    if (!seg || !data || len == 0) {
        return 1;
    }
    if (len > 4096) {
        return 2;  // Data too large
    }
    if (!seg->transmit_iframe) {
        return 3;  // No transmit callback
    }
    if (seg->state != SEG_STATE_IDLE) {
        return 4;  // Busy
    }
    seg->pending_data = data;
    seg->pending_length = len;
    seg->pending_pid = original_pid;
    seg->state = SEG_STATE_SEGMENTING;
    seg->bytes_sent = 0;
    seg->current_segment = 0;
    uint16_t segment_data_size = seg->segment_size;
    seg->total_segments = (len + segment_data_size - 1) / segment_data_size;
    if (seg->total_segments > 64) {
        seg->state = SEG_STATE_IDLE;
        return 2;  // Too many segments
    }

    while (seg->bytes_sent < len) {
        uint8_t header = seg->current_segment & 0x3F;
        if (seg->current_segment == 0) {
            header |= 0x80;  // First segment
        }
        if (seg->current_segment == seg->total_segments - 1) {
            header |= 0x40;  // Last segment
        }
        uint16_t this_len = segment_data_size;
        if (seg->bytes_sent + this_len > len) {
            this_len = len - seg->bytes_sent;
        }
        uint8_t segment[260];
        uint16_t offset = 0;
        segment[offset++] = header;
        if (seg->current_segment == 0) {
            segment[offset++] = original_pid;
        }
        memcpy(&segment[offset], &data[seg->bytes_sent], this_len);
        offset += this_len;
        if (seg->transmit_iframe) {
            seg->transmit_iframe(segment, offset, AX25_PID_SEGMENT_FRAGMENT, seg->user_data);
        }
        seg->bytes_sent += this_len;
        seg->current_segment++;
    }
    seg->state = SEG_STATE_IDLE;
    return 0;
}

void ax25_segmenter_receive(ax25_segmenter_t *seg, uint8_t *data, uint16_t len, uint8_t pid, uint32_t current_tick) {
    if (!seg || !data || len < 1) {
        return;
    }
    if (pid != AX25_PID_SEGMENT_FRAGMENT) {
        if (seg->on_segment_ready) {
            seg->on_segment_ready(data, len, pid, seg->user_data);
        }
        return;
    }
    uint8_t header = data[0];
    ax25_segment_header_t hdr;
    hdr.first_segment = (header & 0x80) >> 7;
    hdr.last_segment = (header & 0x40) >> 6;
    hdr.sequence = header & 0x3F;
    uint8_t *payload = &data[1];
    uint16_t payload_len = len - 1;
    if (hdr.first_segment) {
        if (seg->state != SEG_STATE_IDLE && seg->state != SEG_STATE_REASSEMBLING) {
            if (seg->on_reassembly_error) {
                seg->on_reassembly_error(AX25_SEG_ERROR_INVALID, seg->user_data);
            }
            return;
        }
        if (seg->state == SEG_STATE_IDLE) {
            seg->state = SEG_STATE_REASSEMBLING;
            seg->rx_buffer_used = 0;
            seg->rx_expected_segment = 0;
            memset(seg->rx_segment_bitmap, 0, sizeof(seg->rx_segment_bitmap));
            seg->rx_first_received = true;
            seg->rx_timeout_tick = current_tick + SEG_TIMEOUT_TICKS;
            seg->ooo_count = 0;
            seg->rx_gap_count = 0;
            seg->rx_highest_received = 0;
            seg->rx_last_received = false;
            for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
                seg->ooo_buffer[i].valid = false;
            }
            seg->rx_original_pid = 0;
        }
        if (hdr.sequence != 0) {
            seg->state = SEG_STATE_IDLE;
            if (seg->on_reassembly_error) {
                seg->on_reassembly_error(AX25_SEG_ERROR_SEQUENCE, seg->user_data);
            }
            return;
        }
    } else {
        if (seg->state != SEG_STATE_REASSEMBLING) {
            return;
        }
    }
    if (hdr.sequence >= 64) {
        if (seg->on_reassembly_error) {
            seg->on_reassembly_error(AX25_SEG_ERROR_INVALID, seg->user_data);
        }
        return;
    }
    uint8_t byte_index = hdr.sequence / 8;
    uint8_t bit_index = hdr.sequence % 8;
    if (seg->rx_segment_bitmap[byte_index] & (1U << bit_index)) {
        return;
    }
    if (hdr.sequence > seg->rx_highest_received) {
        seg->rx_highest_received = hdr.sequence;
    }
    if (hdr.sequence == seg->rx_expected_segment) {
        seg->rx_gap_count = 0;  // Reset gap count on valid progress
        if (hdr.first_segment) {
            seg->rx_original_pid = payload[0];
            payload++;
            payload_len--;
        }
        if (seg->rx_buffer_used + payload_len > sizeof(seg->rx_buffer)) {
            seg->state = SEG_STATE_IDLE;
            if (seg->on_reassembly_error) {
                seg->on_reassembly_error(AX25_SEG_ERROR_OVERFLOW, seg->user_data);
            }
            return;
        }
        memcpy(&seg->rx_buffer[seg->rx_buffer_used], payload, payload_len);
        seg->rx_buffer_used += payload_len;
        set_segment_received(seg, hdr.sequence);
        seg->rx_expected_segment = (seg->rx_expected_segment + 1) & 0x3F;
        if (hdr.last_segment) {
            seg->rx_last_received = true;
        }
        bool progress = true;
        while (progress) {
            progress = false;
            for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
                if (seg->ooo_buffer[i].valid && seg->ooo_buffer[i].sequence == seg->rx_expected_segment) {
                    if (seg->rx_buffer_used + seg->ooo_buffer[i].length > sizeof(seg->rx_buffer)) {
                        seg->state = SEG_STATE_IDLE;
                        if (seg->on_reassembly_error) {
                            seg->on_reassembly_error(AX25_SEG_ERROR_OVERFLOW, seg->user_data);
                        }
                        return;
                    }
                    memcpy(&seg->rx_buffer[seg->rx_buffer_used], seg->ooo_buffer[i].data, seg->ooo_buffer[i].length);
                    seg->rx_buffer_used += seg->ooo_buffer[i].length;
                    set_segment_received(seg, seg->ooo_buffer[i].sequence);
                    seg->rx_expected_segment = (seg->rx_expected_segment + 1) & 0x3F;
                    if (seg->ooo_buffer[i].is_last) {
                        seg->rx_last_received = true;
                    }
                    seg->ooo_buffer[i].valid = false;
                    seg->ooo_count--;
                    progress = true;
                    break;
                }
            }
        }
        seg->rx_timeout_tick = current_tick + SEG_TIMEOUT_TICKS;
    } else if (hdr.sequence > seg->rx_expected_segment) {
        uint8_t gap_size = hdr.sequence - seg->rx_expected_segment;
        if (gap_size <= 4 && seg->on_request_retransmit) {
            for (uint8_t missing = seg->rx_expected_segment; missing < hdr.sequence; missing++) {
                bool already_buffered = false;
                for (uint8_t i = 0; i < AX25_MAX_OUT_OF_ORDER_SEGMENTS; i++) {
                    if (seg->ooo_buffer[i].valid && seg->ooo_buffer[i].sequence == missing) {
                        already_buffered = true;
                        break;
                    }
                }
                if (!already_buffered) {
                    seg->on_request_retransmit(missing, seg->user_data);
                }
            }
        }
        if (!buffer_out_of_order_segment(seg, payload, payload_len, hdr.sequence, hdr.last_segment)) {
            if (seg->on_reassembly_error) {
                seg->on_reassembly_error(AX25_SEG_ERROR_SEQUENCE, seg->user_data);
            }
        }
        seg->rx_gap_count++;
        if (seg->rx_gap_count > seg->rx_gap_threshold) {
            seg->state = SEG_STATE_IDLE;
            seg->rx_buffer_used = 0;
            seg->rx_expected_segment = 0;
            memset(seg->rx_segment_bitmap, 0, sizeof(seg->rx_segment_bitmap));
            seg->rx_first_received = false;
            seg->rx_timeout_tick = 0;
            seg->rx_last_received = false;
            seg->ooo_count = 0;
            seg->rx_gap_count = 0;
            if (seg->on_reassembly_error) {
                seg->on_reassembly_error(AX25_SEG_ERROR_SEQUENCE, seg->user_data);
            }
            return;
        }
        seg->rx_timeout_tick = current_tick + SEG_TIMEOUT_TICKS;
    }

    if (seg->rx_last_received && seg->ooo_count == 0 && seg->rx_expected_segment == ((seg->rx_highest_received + 1) & 0x3F)) {
        if (seg->on_reassembly_complete) {
            seg->on_reassembly_complete(seg->rx_buffer, seg->rx_buffer_used, seg->rx_original_pid, seg->user_data);
        }
        seg->state = SEG_STATE_IDLE;
        seg->rx_buffer_used = 0;
        seg->rx_expected_segment = 0;
        memset(seg->rx_segment_bitmap, 0, sizeof(seg->rx_segment_bitmap));
        seg->rx_first_received = false;
        seg->rx_timeout_tick = 0;
        seg->ooo_count = 0;
        seg->rx_last_received = false;
    }
}
