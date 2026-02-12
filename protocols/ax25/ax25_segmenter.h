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

#ifndef AX25_SEGMENTER_H_
#define AX25_SEGMENTER_H_

#include <stdint.h>
#include <stdbool.h>

// AX.25 v2.2 Section 2.3.2, 6.5.3 and Appendix C6 - Segmentation support

// AX.25 v2.2 Section 6.5.3 - Segmentation PID values
#define AX25_PID_SEGMENT_FRAGMENT  0x08   // Segment fragment indicator
#define AX25_PID_ESCAPE_CHARACTER  0xFF   // Escape for extended PID

// Maximum segments that can be buffered (configurable for memory constraints)
#define AX25_MAX_SEGMENTS 8

#ifndef AX25_TR210_TIMEOUT_MS
#define AX25_TR210_TIMEOUT_MS 30000 // Default 30 seconds (configurable)
#endif
// Convert to 10ms ticks (avoid division in tick handler)
#define SEG_TIMEOUT_TICKS (AX25_TR210_TIMEOUT_MS / 10)

// NOTE; Out-of-Order Segment Handling - Limited Buffer Depth and No Selective Retransmit
// Out-of-order uses ax25_segment_slot_t ooo_buffer[4] (configurable via #define AX25_MAX_OUT_OF_ORDER_SEGMENTS 4).
// buffer_out_of_order_segment() scans for free slot, stores data/sequence/is_last, increments ooo_count.
// process_buffered_segments() loops while progress: finds next expected in buffer, appends to rx_buffer, increments rx_expected_segment,
// sets rx_last_received if is_last, marks bitmap, invalidates slot, decrements ooo_count. Called after in-order receipt in receive().
// Bitmap (uint8_t rx_segment_bitmap) tracks up to 8 segments (bit 0-7). If buffer full, silently drops (returns false, calls on_reassembly_error(AX25_SEG_ERROR_SEQUENCE)).
// Per Appendix C6, out-of-order must be buffered "until the missing segments arrive," but spec limits sequences to 0-7, implying small buffers.
// No SREJ/REJ integration for requesting missing segments—relies on sender retries.
#define AX25_MAX_OUT_OF_ORDER_SEGMENTS 8 // Configurable for MCU memory

typedef enum {
    AX25_SEG_ERROR_TIMEOUT = 1,   // TR210 expiration
    AX25_SEG_ERROR_OVERFLOW = 2,  // Buffer overflow
    AX25_SEG_ERROR_SEQUENCE = 3,  // Sequence error (out-of-order)
    AX25_SEG_ERROR_INVALID = 4    // Invalid segment format
} ax25_seg_error_t;

// Segment state machine states
typedef enum {
    SEG_STATE_IDLE = 0,           // No segmentation/reassembly in progress
    SEG_STATE_SEGMENTING,         // Actively segmenting data for transmission
    SEG_STATE_AWAITING_ACK,       // Waiting for acknowledgment of segments
    SEG_STATE_REASSEMBLING        // Receiving and reassembling segments
} ax25_seg_state_t;

typedef struct {
    uint8_t data[260];  // Segment data (max segment size)
    uint16_t length;    // Data length
    uint8_t sequence;   // Sequence number
    bool valid;         // Slot in use
    bool is_last;       // Added to handle last segment flag in buffered segments
} ax25_segment_slot_t;

// Segment header structure (1 byte format)
// Bit 7: First segment flag
// Bit 6: Last segment flag
// Bits 5-0: Segment sequence number (0-63)
typedef struct {
    bool first_segment;     // True if this is the first segment
    bool last_segment;      // True if this is the last segment
    uint8_t sequence;       // Segment sequence number (0-63)
} ax25_segment_header_t;

// Segmentation/Reassembly context
typedef struct {
    ax25_seg_state_t state;      // Current state

    // Transmit segmentation state
    uint8_t *pending_data;           // Pointer to data being segmented (caller owns)
    uint16_t pending_length;         // Total length of data to segment
    uint16_t segment_size;           // Maximum data per segment (N1 - overhead)
    uint8_t current_segment;         // Current segment being transmitted
    uint8_t total_segments;          // Total number of segments
    uint16_t bytes_sent;             // Bytes transmitted so far
    uint8_t pending_pid;             // Original PID for segmented data

    // Receive reassembly state
    uint8_t rx_buffer[4096];         // Static buffer for reassembly
    uint16_t rx_buffer_used;         // Bytes accumulated in rx_buffer
    uint8_t rx_expected_segment;     // Next expected segment sequence number
    uint8_t rx_segment_bitmap[8];    // Bitmap of received segments (up to 64)
    bool rx_first_received;          // True if first segment received
    uint32_t rx_timeout_tick;        // Timeout timestamp for reassembly
    uint8_t rx_original_pid;         // Original PID from segmented data

    // Callbacks
    void (*on_segment_ready)(uint8_t *data, uint16_t len, uint8_t pid, void *user_data);        //
    void (*on_reassembly_complete)(uint8_t *data, uint16_t len, uint8_t pid, void *user_data);  //
    void (*transmit_iframe)(uint8_t *data, uint16_t len, uint8_t pid, void *user_data);         //
    void *user_data;                                                                            // User context pointer
    void (*on_reassembly_error)(ax25_seg_error_t error, void *user_data);                       //
    void (*on_request_retransmit)(uint8_t sequence, void *user_data);                           // Request retransmission of missing segment (SREJ integration)

    // Out-of-order segment buffering
    ax25_segment_slot_t ooo_buffer[AX25_MAX_OUT_OF_ORDER_SEGMENTS];  //
    uint8_t ooo_count;                                               // Number of buffered segments
    uint8_t rx_highest_received;                                     // Highest sequence number seen
    bool rx_last_received;                                           // Track if last segment has been received/processed
    uint8_t rx_gap_count;                                            // Number of segments received beyond a gap
    uint8_t rx_gap_threshold;                                        // Threshold to trigger gap error (configurable)
} ax25_segmenter_t;

// Initialize segmenter context
// seg: Pointer to segmenter context
// max_iframe_size: Maximum I-field size negotiated (N1)
// Returns: 0 on success, 1 on error
uint8_t ax25_segmenter_init(ax25_segmenter_t *seg, uint16_t max_iframe_size);

// This function implements TR210 timer per AX.25 v2.2 Appendix C6
// TR210 must start when first segment is received and restart on each in-sequence segment
// If TR210 expires, all buffered segments must be discarded and error reported
void ax25_segmenter_tick(ax25_segmenter_t *seg, uint32_t current_tick);

// Segment and send large data
// seg: Pointer to segmenter context
// data: Data to segment and transmit
// len: Length of data
// original_pid: PID value for the original data
// Returns: 0 on success, 1-3 on error
uint8_t ax25_segmenter_send(ax25_segmenter_t *seg, uint8_t *data, uint16_t len, uint8_t original_pid);

// Process received segment
// seg: Pointer to segmenter context
// data: Received segment data (including segment header)
// len: Length of segment data
// pid: PID of received frame (should be 0x08 for segments)
// current_tick:
void ax25_segmenter_receive(ax25_segmenter_t *seg, uint8_t *data, uint16_t len, uint8_t pid, uint32_t current_tick);

// Timer tick handler for timeout management
// seg: Pointer to segmenter context
// current_tick: Current system tick count (10ms units)
void ax25_segmenter_tick(ax25_segmenter_t *seg, uint32_t current_tick);

#endif // AX25_SEGMENTER_H_
