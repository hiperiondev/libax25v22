/**
 * @file ax25_segmenter.h
 * @brief AX.25 v2.2 Protocol Library - Segmentation and Reassembly Module
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @section Overview
 * This header defines the AX.25 v2.2 Segmenter/Reassembler functionality per
 * Appendix C6 of the AX.25 Link Access Protocol specification. The segmenter
 * enables transmission of data units larger than the negotiated maximum I-field
 * length (N1) by splitting them into smaller segments that are transmitted
 * as separate I-frames or UI frames and reassembled at the receiving station.
 *
 * @section Protocol_Features
 * - Segmentation of payloads exceeding N1 octets into multiple fragments
 * - 6-bit sequence numbering supporting up to 64 segments (0-63)
 * - First/Last segment indicators in segment header
 * - TR210 timer for reassembly timeout protection
 * - Out-of-order segment buffering with configurable depth
 * - Integration with Selective Reject (SREJ) for missing segment recovery
 * - Support for both connection-oriented (I-frame) and connectionless (UI) modes
 *
 * @section Standards_References
 * - AX.25 Link Access Protocol for Amateur Packet Radio, Version 2.2 (July 1998)
 *   Appendix C6: Segmenter/Reassembler
 * - Section 6.6: Disassembler/Reassembler
 * - Section 6.7.1.13: Next Segment Timer TR210
 *
 * @section Segment_Header_Format
 * Per AX.25 v2.2 Section 6.6 and Appendix C6, each segment has a 1-2 octet header:
 * - Octet 1 (mandatory): Control byte
 *   - Bit 7 (0x80): First segment indicator (BEG)
 *   - Bit 6 (0x40): Last segment indicator (END)
 *   - Bits 5-0: 6-bit segment sequence number (0-63)
 * - Octet 2 (first segment only): Original PID
 *
 * @section TR210_Timer
 * The TR210 timer ("R" for reassembler, "2" for level 2, "10" to avoid confusion)
 * supervises the reassembly process. Per Appendix C6.3:
 * - Starts when first segment is received
 * - Restarts on each in-sequence segment receipt
 * - If expired, all buffered segments are discarded and error reported to Layer 3
 * - Default: 30 seconds (configurable via AX25_TR210_TIMEOUT_MS)
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef AX25_SEGMENTER_H_
#define AX25_SEGMENTER_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* Protocol Identifier Constants                                              */
/*============================================================================*/

/**
 * @defgroup PIDConstants Protocol Identifier Values
 * @brief PID codes defined in AX.25 v2.2 Section 6.5.3 and Appendix C6
 *
 * The Protocol Identifier field identifies the layer 3 protocol or special
 * handling required for the frame payload.
 */
#define AX25_PID_SEGMENT_FRAGMENT  0x08u  /**< Segmentation Fragment — AX.25 v2.2 Section 3.4, Figure 3.2 */
#define AX25_PID_ESCAPE_CHARACTER  0xFF   /**< Escape for extended PID (next byte contains extended PID) */

/*============================================================================*/
/* Configuration Constants                                                      */
/*============================================================================*/

/**
 * @defgroup ConfigConstants Compile-Time Configuration
 * @brief Configurable parameters for memory and timing constraints
 *
 * These constants may be overridden at compile time to adapt the segmenter
 * to specific hardware constraints (e.g., embedded systems with limited RAM).
 */

/**
 * @def AX25_TR210_TIMEOUT_MS
 * @brief TR210 reassembly timeout in milliseconds
 *
 * Per AX.25 v2.2 Appendix C6.3, this timer ensures the reassembler doesn't
 * wait indefinitely for the next segment. If the timer expires, all buffered
 * segments are discarded and an error is reported to Layer 3.
 *
 * Default: 30000 ms (30 seconds)
 * Range: 1000-60000 ms recommended
 */
#ifndef AX25_TR210_TIMEOUT_MS
#define AX25_TR210_TIMEOUT_MS 30000
#endif

/**
 * @brief Convert TR210 timeout to 10ms ticks
 *
 * Internal representation uses 10ms ticks to avoid division operations
 * in the time-critical tick handler.
 */
#define SEG_TIMEOUT_TICKS (AX25_TR210_TIMEOUT_MS / 10)

/**
 * @def AX25_MAX_OUT_OF_ORDER_SEGMENTS
 * @brief Maximum number of out-of-order segments to buffer
 *
 * Per AX.25 v2.2 Appendix C6, out-of-order segments must be buffered
 * "until the missing segments arrive." The specification implies small
 * buffers given the 6-bit sequence number range (0-63).
 *
 * This implementation uses an 8-slot buffer with intelligent eviction
 * based on distance from expected sequence number.
 *
 * Default: 8 segments
 * Minimum: 2 (to handle basic out-of-order scenarios)
 * Maximum: 64 (full sequence space, memory permitting)
 */
#ifndef AX25_MAX_OUT_OF_ORDER_SEGMENTS
#define AX25_MAX_OUT_OF_ORDER_SEGMENTS 8
#endif

// AX25_MAX_SEGMENT_BUF replaces the hardcoded 260-byte literal used in
// ax25_segment_slot_t and in the transmit buffer inside ax25_segmenter_send().
// Sized for maximum AX.25 N1 (256) + segment control byte (1) + original PID
// byte (1) + 2-byte safety margin = 260. Override at compile time if needed.
#ifndef AX25_MAX_SEGMENT_BUF
#define AX25_MAX_SEGMENT_BUF 260u
#endif

/*============================================================================*/
/* Error Codes                                                                  */
/*============================================================================*/

/**
 * @brief Segmentation error codes reported via on_reassembly_error callback
 *
 * Per AX.25 v2.2 Appendix C6.3, the reassembler detects but does not
 * correct segmentation errors. Errors are reported to Layer 3 for recovery.
 */
typedef enum {
    AX25_SEG_ERROR_TIMEOUT = 1, /**< TR210 timer expired - incomplete reassembly */
    AX25_SEG_ERROR_OVERFLOW = 2, /**< Reassembly buffer overflow (payload > 4096 bytes) */
    AX25_SEG_ERROR_SEQUENCE = 3, /**< Sequence error - missing segment detected */
    AX25_SEG_ERROR_INVALID = 4 /**< Invalid segment format or parameters */
} ax25_seg_error_t;

/*============================================================================*/
/* State Machine States                                                         */
/*============================================================================*/

/**
 * @brief Segmenter/Reassembler state machine states
 *
 * Per AX.25 v2.2 Appendix C6.3, the segmenter has one state (Ready) while
 * the reassembler has three states (Null, Reassembling Data, Reassembling Unit Data).
 * This implementation combines the reassembler states into REASSEMBLING.
 */
typedef enum {
    SEG_STATE_IDLE = 0, /**< No segmentation/reassembly in progress (Null state) */
    SEG_STATE_SEGMENTING, /**< Actively segmenting data for transmission */
    SEG_STATE_AWAITING_ACK, /**< Waiting for acknowledgment of segments (future use) */
    SEG_STATE_REASSEMBLING /**< Receiving and reassembling segments */
} ax25_seg_state_t;

/*============================================================================*/
/* Data Structures                                                              */
/*============================================================================*/

/**
 * @brief Out-of-order segment buffer slot
 *
 * Stores a single out-of-order segment during reassembly.
 * Per AX.25 v2.2 Appendix C6, segments arriving out of sequence must be
 * buffered until the missing segments arrive.
 *
 * @section Buffer_Management
 * - Maximum data per segment: 260 bytes ( accommodates N1 up to 256 + header)
 * - Valid flag indicates slot occupancy
 * - is_last preserves the END flag for proper reassembly completion detection
 */
typedef struct {
    uint8_t data[AX25_MAX_SEGMENT_BUF]; /**<  Segment payload (max N1 + overhead) */
    uint16_t length; /**<  Actual data length in bytes */
    uint8_t sequence; /**<  6-bit segment sequence number (0-63) */
    bool valid; /**<  Slot in use flag */
    bool is_last; /**<  Original END flag state for this segment */
} ax25_segment_slot_t;

/**
 * @brief Segment header structure
 *
 * Per AX.25 v2.2 Section 6.6 and Figure 6.2, the segment header is the
 * first octet of the information field in segmented frames (PID=0x08).
 *
 * @section Header_Format
 * Bit 7: First segment flag (BEG) - Set to 1 in first segment only
 * Bit 6: Last segment flag (END) - Set to 1 in final segment only
 * Bits 5-0: Sequence number - 6-bit value 0-63 identifying segment order
 *
 * The first segment additionally includes the original PID in octet 2.
 */
typedef struct {
    bool first_segment; /**< True if this is the first segment (BEG flag) */
    bool last_segment; /**< True if this is the last segment (END flag) */
    uint8_t sequence; /**< 6-bit segment sequence number (0-63) */
} ax25_segment_header_t;

/**
 * @brief Segmentation/Reassembly context structure
 *
 * Main state container for the AX.25 v2.2 segmenter/reassembler.
 * One instance exists per data link (DLSAP) per AX.25 v2.2 Section 2.4.
 *
 * @section State_Machine
 * The segmenter/reassembler operates as a state machine per Appendix C6:
 * - Segmenter: Single Ready state, segments data on DL-DATA/DL-UNIT-DATA Request
 * - Reassembler: Null -> Reassembling -> Null on completion or error
 *
 * @section Memory_Management
 * - Uses static 4096-byte receive buffer (suitable for IP datagrams)
 * - Out-of-order buffer configurable via AX25_MAX_OUT_OF_ORDER_SEGMENTS
 * - No dynamic memory allocation during operation
 */
typedef struct {
    /*------------------------------------------------------------------------*/
    /* State Machine                                                          */
    /*------------------------------------------------------------------------*/
    ax25_seg_state_t state; /**< Current segmenter/reassembler state */

    /*------------------------------------------------------------------------*/
    /* Transmit Segmentation State                                            */
    /*------------------------------------------------------------------------*/
    uint8_t *pending_data; /**< Pointer to data being segmented (caller owned) */
    uint16_t pending_length; /**< Total length of data to segment */
    uint16_t segment_size; /**< Maximum data per segment (N1 - 2 octets overhead) */
    uint8_t current_segment; /**< Current segment being transmitted (0-63) */
    uint8_t total_segments; /**< Total number of segments calculated */
    uint16_t bytes_sent; /**< Bytes transmitted so far */
    uint8_t pending_pid; /**< Original PID for segmented data */
    uint8_t tx_segment_buf[AX25_MAX_SEGMENT_BUF]; /**< Reusable transmit segment buffer - lives in struct (BSS/static) instead of the stack, preventing stack overflow on Cortex-M0/M0+ with 4KB RAM. */

    /*------------------------------------------------------------------------*/
    /* Receive Reassembly State                                               */
    /*------------------------------------------------------------------------*/
    uint8_t rx_buffer[4096]; /**< Static reassembly buffer (4KB max payload) */
    uint16_t rx_buffer_used; /**< Bytes accumulated in rx_buffer */
    uint8_t rx_expected_segment; /**< Next expected sequence number (0-63) */
    uint8_t rx_segment_bitmap[8]; /**< Bitmap of received segments (64 bits) */
    bool rx_first_received; /**< True if first segment (seq 0) received */
    uint32_t rx_timeout_tick; /**< TR210 timeout timestamp (10ms ticks) */
    bool rx_timer_armed; /**< true when rx_timeout_tick is valid (armed) */
    uint8_t rx_original_pid; /**< Original PID extracted from first segment */

    /*------------------------------------------------------------------------*/
    /* Out-of-Order Segment Handling                                          */
    /*------------------------------------------------------------------------*/
    ax25_segment_slot_t ooo_buffer[AX25_MAX_OUT_OF_ORDER_SEGMENTS]; /**< Out-of-order buffer */
    uint8_t ooo_count; /**< Number of buffered segments */
    uint8_t rx_highest_received; /**< Highest sequence number seen */
    bool rx_last_received; /**< END flag seen for expected sequence */
    uint8_t rx_gap_count; /**< Segments received beyond gap */
    uint8_t rx_gap_threshold; /**< Threshold to trigger gap error */

    /*------------------------------------------------------------------------*/
    /* Callback Functions                                                     */
    /*------------------------------------------------------------------------*/
    /**
     * @brief Callback for non-segmented data or completed reassembly
     *
     * Invoked when:
     * - Non-segmented frame received (PID != 0x08)
     * - Segmented payload fully reassembled
     *
     * @param data      Pointer to data (callback must copy if needed)
     * @param len       Length of data in bytes
     * @param pid       Protocol Identifier (original PID for reassembled data)
     * @param user_data User context pointer
     */
    void (*on_segment_ready)(uint8_t *data, uint16_t len, uint8_t pid, void *user_data);

    /**
     * @brief Callback for reassembly completion
     *
     * Invoked when all segments have been received and reassembled.
     * The reassembled data is in rx_buffer, passed here for convenience.
     *
     * @param data      Pointer to reassembled payload
     * @param len       Total payload length
     * @param pid       Original PID from first segment
     * @param user_data User context pointer
     */
    void (*on_reassembly_complete)(uint8_t *data, uint16_t len, uint8_t pid, void *user_data);

    /**
     * @brief Callback for I-frame transmission
     *
     * Called by the segmenter to transmit each segment as an I-frame
     * or UI frame. The callback must encapsulate the data appropriately.
     *
     * @param data      Segment data (header + payload)
     * @param len       Total segment length
     * @param pid       Always AX25_PID_SEGMENT_FRAGMENT (0x08)
     * @param user_data User context pointer
     */
    void (*transmit_iframe)(uint8_t *data, uint16_t len, uint8_t pid, void *user_data);

    /**
     * @brief User context pointer passed to all callbacks
     */
    void *user_data;

    /**
     * @brief Callback for reassembly errors
     *
     * Per AX.25 v2.2 Appendix C6.3, errors detected during reassembly
     * are reported to Layer 3. No automatic recovery is attempted.
     *
     * @param error     Error code indicating failure reason
     * @param user_data User context pointer
     */
    void (*on_reassembly_error)(ax25_seg_error_t error, void *user_data);

    /**
     * @brief Callback for selective retransmit request
     *
     * Optional callback for integration with SREJ (Selective Reject).
     * Invoked when a gap in sequence numbers is detected, allowing
     * the Layer 2 entity to request retransmission of missing segments.
     *
     * @param sequence  Missing segment sequence number (0-63)
     * @param user_data User context pointer
     */
    void (*on_request_retransmit)(uint8_t sequence, void *user_data);
} ax25_segmenter_t;

/*============================================================================*/
/* Initialization Function                                                      */
/*============================================================================*/

/**
 * @brief Initialize segmenter context
 *
 * Prepares the segmenter/reassembler for operation. Must be called before
 * any other segmenter functions.
 *
 * @section Initialization
 * - Zeroes all state variables
 * - Sets state to IDLE
 * - Calculates segment_size from max_iframe_size (N1 - 2)
 * - Configures default gap threshold (8 segments)
 *
 * @param[in]  seg             Pointer to segmenter context to initialize
 * @param[in]  max_iframe_size Maximum I-field size negotiated (N1 parameter)
 *                             Per AX.25 v2.2 default is 256 octets
 * @return     0 on success, 1 on error (NULL pointer)
 *
 * @note The segmenter does not allocate dynamic memory. All buffers are
 *       statically sized within the ax25_segmenter_t structure.
 */
uint8_t ax25_segmenter_init(ax25_segmenter_t *seg, uint16_t max_iframe_size);

/*============================================================================*/
/* Timer Management                                                             */
/*============================================================================*/

/**
 * @brief TR210 timer tick handler
 *
 * Implements the TR210 Next Segment Timer per AX.25 v2.2 Appendix C6.3
 * and Section 6.7.1.13.
 *
 * @section Timer_Operation
 * - Must be called periodically with 10ms resolution
 * - Checks for timeout only in REASSEMBLING state
 * - Uses signed arithmetic to handle 32-bit tick counter wraparound
 * - On timeout: discards all buffers, resets state, invokes on_reassembly_error
 *
 * @param[in,out] seg          Pointer to segmenter context
 * @param[in]     current_tick Current system tick count (10ms units)
 *
 * @note The tick counter must increment every 10ms. The implementation
 *       handles 32-bit unsigned wraparound correctly using signed delta.
 */
void ax25_segmenter_tick(ax25_segmenter_t *seg, uint32_t current_tick);

/*============================================================================*/
/* Transmission Functions                                                       */
/*============================================================================*/

/**
 * @brief Segment and transmit large data unit
 *
 * Segments data exceeding N1 octets into multiple I-frames or UI frames
 * per AX.25 v2.2 Appendix C6.1.
 *
 * @section Segmentation_Process
 * 1. Validates parameters and state (must be IDLE)
 * 2. Calculates total segments: ceil(len / (N1 - 2))
 * 3. Rejects if more than 64 segments (sequence number limit)
 * 4. Constructs each segment with proper header:
 *    - First segment: BEG=1, sequence=0, includes original PID
 *    - Middle segments: BEG=0, END=0, sequence increments
 *    - Last segment: END=1, final sequence number
 * 5. Calls transmit_iframe callback for each segment
 *
 * @param[in,out] seg          Pointer to segmenter context
 * @param[in]     data         Data to segment and transmit
 * @param[in]     len          Length of data (max 4096 bytes)
 * @param[in]     original_pid Original PID for the payload
 * @return      0 on success
 *              1 on invalid parameters
 *              2 on data too large (>4096 bytes or >64 segments)
 *              3 on missing transmit callback
 *              4 on busy (not in IDLE state)
 *
 * @note This function blocks until all segments are queued via the
 *       transmit_iframe callback. It does not wait for acknowledgments.
 */
uint8_t ax25_segmenter_send(ax25_segmenter_t *seg, uint8_t *data, uint16_t len, uint8_t original_pid);

/*============================================================================*/
/* Reception Functions                                                          */
/*============================================================================*/

/**
 * @brief Process received segment
 *
 * Reassembles segmented frames or passes through non-segmented frames.
 * Implements the reassembler state machine per AX.25 v2.2 Appendix C6.3.
 *
 * @section Reassembly_Process
 * 1. If PID != 0x08: Pass directly to on_segment_ready callback
 * 2. Parse segment header (BEG, END, sequence)
 * 3. If first segment (BEG=1, sequence=0):
 *    - Initialize reassembly state
 *    - Start TR210 timer
 *    - Clear buffers and bitmap
 * 4. If in-sequence: Append to rx_buffer, update bitmap, check completion
 * 5. If out-of-order: Buffer if space available, request retransmit if gap small
 * 6. On completion (END received, all sequences present): Invoke on_reassembly_complete
 *
 * @section Error_Handling
 * - Invalid first segment (sequence != 0): Error AX25_SEG_ERROR_SEQUENCE
 * - Duplicate segments: Silently accepted
 * - Buffer overflow (>4096 bytes): Error AX25_SEG_ERROR_OVERFLOW
 * - Gap threshold exceeded: Error AX25_SEG_ERROR_SEQUENCE
 * - Timeout: Error AX25_SEG_ERROR_TIMEOUT (handled in tick function)
 *
 * @param[in,out] seg          Pointer to segmenter context
 * @param[in]     data         Received segment data (including segment header)
 * @param[in]     len          Length of received data
 * @param[in]     pid          PID of received frame (0x08 for segments)
 * @param[in]     current_tick Current system tick for TR210 management
 *
 * @note The data pointer is only valid for the duration of the call.
 *       The implementation copies data into internal buffers as needed.
 */
void ax25_segmenter_receive(ax25_segmenter_t *seg, uint8_t *data, uint16_t len, uint8_t pid, uint32_t current_tick);

#endif // AX25_SEGMENTER_H_
