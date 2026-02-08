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

// start modified part - New file for AX.25 v2.2 Segmentation/Reassembly Layer
// AX.25 v2.2 Section 2.3.2, 6.5.3 and Appendix C6 - Segmentation support

// AX.25 v2.2 Section 6.5.3 - Segmentation PID values
#define AX25_PID_SEGMENT_FRAGMENT  0x08   // Segment fragment indicator
#define AX25_PID_ESCAPE_CHARACTER  0xFF   // Escape for extended PID

// Maximum segments that can be buffered (configurable for memory constraints)
#define AX25_MAX_SEGMENTS 8

// Segment state machine states
typedef enum {
    SEG_STATE_IDLE = 0,           // No segmentation/reassembly in progress
    SEG_STATE_SEGMENTING,         // Actively segmenting data for transmission
    SEG_STATE_AWAITING_ACK,       // Waiting for acknowledgment of segments
    SEG_STATE_REASSEMBLING        // Receiving and reassembling segments
} ax25_seg_state_t;

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
    uint8_t rx_buffer[2048];         // Static buffer for reassembly (no malloc)
    uint16_t rx_buffer_used;         // Bytes accumulated in rx_buffer
    uint8_t rx_expected_segment;     // Next expected segment sequence number
    uint8_t rx_segment_bitmap;       // Bitmap of received segments (up to 8)
    bool rx_first_received;          // True if first segment received
    uint32_t rx_timeout_tick;        // Timeout timestamp for reassembly
    uint8_t rx_original_pid;         // Original PID from segmented data

    // Callbacks
    void (*on_segment_ready)(uint8_t *data, uint16_t len, uint8_t pid, void *user_data);
    void (*on_reassembly_complete)(uint8_t *data, uint16_t len, uint8_t pid, void *user_data);
    void (*transmit_iframe)(uint8_t *data, uint16_t len, uint8_t pid, void *user_data);
    void *user_data;                 // User context pointer
} ax25_segmenter_t;

// Initialize segmenter context
// seg: Pointer to segmenter context
// max_iframe_size: Maximum I-field size negotiated (N1)
// Returns: 0 on success, 1 on error
uint8_t ax25_segmenter_init(ax25_segmenter_t *seg, uint16_t max_iframe_size);

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
void ax25_segmenter_receive(ax25_segmenter_t *seg, uint8_t *data, uint16_t len, uint8_t pid);

// Timer tick handler for timeout management
// seg: Pointer to segmenter context
// current_tick: Current system tick count (10ms units)
void ax25_segmenter_tick(ax25_segmenter_t *seg, uint32_t current_tick);

#endif // AX25_SEGMENTER_H_
