/**
 * @file ax25_state_machine.h
 * @brief AX.25 v2.2 Data Link State Machine Implementation
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @section Overview
 * This header defines the complete Data Link State Machine for AX.25 v2.2
 * Link Access Protocol. The state machine implements the peer-to-peer
 * balanced operation mode specified in AX.25 v2.2 Section 2.5 and Appendix C4.
 *
 * @section Architecture
 * The AX.25 protocol consists of multiple interacting finite state machines:
 * - Data Link State Machine (this implementation): Core connection management
 * - Link Multiplexer State Machine: Channel sharing between multiple links
 * - Physical State Machine: Radio transmitter/receiver control
 * - Management Data Link State Machine: Parameter negotiation via XID
 * - Segmenter/Reassembler: Large payload fragmentation
 *
 * @section State_Machine_Operation
 * The Data Link State Machine provides:
 * - Connection establishment via SABM/SABME commands and UA responses
 * - Connection-oriented reliable data transfer using I-frames
 * - Connectionless datagram service via UI frames
 * - Link termination via DISC commands
 * - Error recovery through REJ, SREJ, and timeout mechanisms
 * - Flow control via RNR/RR frames and window management
 *
 * @section Standards_References
 * - AX.25 Link Access Protocol for Amateur Packet Radio, Version 2.2 (July 1998)
 *   Section 2.5: Data Link State Machine
 *   Section 4.2: Control Field Parameters and State Variables
 *   Section 6.4: Procedures for Information Transfer
 *   Appendix C4: Data-Link State Machine SDL Diagrams
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef AX25_STATE_MACHINE_H_
#define AX25_STATE_MACHINE_H_

#include <stdint.h>
#include <stdbool.h>

#include "ax25.h"
#include "ax25_mgmt.h"
#include "ax25_physical.h"
#include "ax25_segmenter.h"

/*============================================================================*/
/* Timer Constants and Sentinel Values                                        */
/*============================================================================*/

/**
 * @defgroup TimerConstants Timer Management Constants
 * @brief Special values for timer state management
 */

// All defaults follow AX.25 v2.2 §6.7.2 and PE1CHL §5.
// Override at build time with -DAX25_DEFAULT_T1_MS=<value> etc.
// T1: Acknowledgment timer default — 3000 ms (§6.7.1.1)
// Stored as 10ms ticks in ax25_timers_t.t1: 3000/10 = 300 ticks.
#ifndef AX25_DEFAULT_T1_MS
#define AX25_DEFAULT_T1_MS  3000u
#endif
// T2: Response delay timer default — 3000 ms (§6.7.1.2).
// Spec says T2 < T1; default kept at 1500 ms (half of T1) for piggybacking.
// Stored as 10ms ticks: 1500/10 = 150 ticks.
#ifndef AX25_DEFAULT_T2_MS
#define AX25_DEFAULT_T2_MS  1500u
#endif
// T3: Inactive link timer default — 30000 ms (§6.7.1.3).
// Stored as 10ms ticks: 30000/10 = 3000 ticks.
#ifndef AX25_DEFAULT_T3_MS
#define AX25_DEFAULT_T3_MS  30000u
#endif

// N1: Maximum I-field length in octets (§6.7.2).
#ifndef AX25_DEFAULT_N1
#define AX25_DEFAULT_N1  256u
#endif

// SREJ buffer slot size: tied to N1 so the buffer is never
// larger than the maximum negotiated I-field. Override at
// compile time with -DMAX_SREJ_BUFFER_SIZE=128 on MCUs.
#ifndef AX25_SREJ_BUFFER_SIZE
#define AX25_SREJ_BUFFER_SIZE  AX25_DEFAULT_N1
#endif

// N2: Maximum number of retransmission attempts (§6.7.2).
#ifndef AX25_DEFAULT_N2
#define AX25_DEFAULT_N2  10u
#endif

#ifndef AX25_DEFAULT_K_MOD8
#define AX25_DEFAULT_K_MOD8  7u
#endif

// k: Window size default for mod-128 connections.
// Spec default: 32 per §6.7.2 / PE1CHL §5.
#ifndef AX25_DEFAULT_K_MOD128
#define AX25_DEFAULT_K_MOD128  32u
#endif

// Non-blocking tick-driven timer (32-bit milliseconds).
// Handles 32-bit wrap-around; 2^32 ms ~= 49.7 days — sufficient for AX.25.
// All fields are in milliseconds; caller converts 10ms ticks via * 10u before calling.
typedef struct {
    uint32_t start_ms;    // tick value (ms) when timer was started
    uint32_t duration_ms;  // how long the timer runs (ms)
    uint8_t running;     // 1 = active, 0 = stopped
} ax25_timer_t;

// ax25_timer_start: arm the timer with a duration and current time in ms.
static inline void ax25_timer_start(ax25_timer_t *t, uint32_t duration_ms, uint32_t now_ms) {
    t->start_ms = now_ms;
    t->duration_ms = duration_ms;
    t->running = 1u;
}

// ax25_timer_stop: disarm the timer (no-op on NULL guarded by caller).
static inline void ax25_timer_stop(ax25_timer_t *t) {
    t->running = 0u;
}

// ax25_timer_expired: returns 1 if timer is running and has expired.
// Subtraction wraps correctly even when now_ms has wrapped past start_ms.
static inline uint8_t ax25_timer_expired(const ax25_timer_t *t, uint32_t now_ms) {
    if (!t->running)
        return 0u;
    return ((now_ms - t->start_ms) >= t->duration_ms) ? 1u : 0u;
}

// T101: Priority Window (PRIACK) default duration — 2000 ms per §6.7.1 timers table.
// Bounds the piggyback-ACK opportunity window in the connection layer.
// When T101 fires before an outgoing I-frame has been queued, a standalone
// RR (or RNR if locally busy) is sent so the peer is not left waiting.
// Override at compile time with -DAX25_T101_PRIACK_MS=<ms>.
#ifndef AX25_T101_PRIACK_MS
#define AX25_T101_PRIACK_MS 2000u
#endif

/*============================================================================*/
/* FRMR Reason Code Definitions                                               */
/*============================================================================*/

/**
 * @defgroup FRMRCodes Frame Reject Reason Codes
 * @brief Bit definitions for FRMR information field flags
 *
 * Per AX.25 v2.2 Section 4.3.3.6, the FRMR frame contains a control field
 * copy and error flags indicating the reason for frame rejection. These
 * flags are transmitted in the final byte of the FRMR information field.
 *
 * @section FRMR_Triggers
 * FRMR is transmitted when:
 * - Invalid control field received (W-bit)
 * - Information field present in frame type that prohibits it (X-bit)
 * - Information field exceeds maximum length N1 (Y-bit)
 * - Invalid N(R) received acknowledging unsent frames (Z-bit)
 */
#define FRMR_W  0x01 /**< Invalid control field or not implemented */
#define FRMR_X  0x02 /**< Information field not permitted for this frame type */
#define FRMR_Y  0x04 /**< Information field exceeded maximum length N1 */
#define FRMR_Z  0x08 /**< Invalid N(R) received - acknowledgment error */

// DL-ERROR indication codes per AX.25 v2.2 §17.2 / Appendix C4 SDL.
// Complete set A-V (22 codes); original ended at N=13.
typedef enum {
    AX25_DL_ERROR_A = 0,   // F=1 received unexpectedly in connected state
    AX25_DL_ERROR_B = 1,   // FRMR received - link reset required
    AX25_DL_ERROR_C = 2,   // UA received without F=1 in AWAITING_CONNECTION
    AX25_DL_ERROR_D = 3,   // DM received in connected state
    AX25_DL_ERROR_E = 4,   // Frame received in incorrect state
    AX25_DL_ERROR_F = 5,   // FRMR sent by us - protocol violation detected
    AX25_DL_ERROR_G = 6,   // I field exceeded maximum length N1
    AX25_DL_ERROR_H = 7,   // FRMR received after FRMR sent - link failure
    AX25_DL_ERROR_I = 8,   // Information field not permitted in this frame type
    AX25_DL_ERROR_J = 9,   // N(R) sequence error - invalid N(R) received (Z condition)
    AX25_DL_ERROR_K = 10,  // DM received with F=1 in AWAITING_CONNECTION (connect refused)
    AX25_DL_ERROR_L = 11,  // Control field invalid or not implemented
    AX25_DL_ERROR_M = 12,  // I frame received while not in information transfer state
    AX25_DL_ERROR_N = 13,  // N2 retries exceeded - T1 timer exhaustion
    AX25_DL_ERROR_O = 14,  // DM received in AWAITING_RELEASE (peer confirmed disconnect)
    AX25_DL_ERROR_P = 15,  // N2 retransmissions exceeded in AWAITING_RELEASE
    AX25_DL_ERROR_Q = 16,  // DM received in AWAITING_CONNECTION (connection refused)
    AX25_DL_ERROR_R = 17,  // FRMR received in AWAITING_CONNECTION or AWAITING_RELEASE
    AX25_DL_ERROR_S = 18,  // UA received in wrong state (not awaiting connect or release)
    AX25_DL_ERROR_T = 19,  // SABM or SABME received while in AWAITING_RELEASE state
    AX25_DL_ERROR_U = 20,  // XID received but XID negotiation not implemented
    AX25_DL_ERROR_V = 21   // Unrecognised control field in FRAME_REJECT state
} ax25_dl_error_t;

/*============================================================================*/
/* Protocol Handler Configuration                                             */
/*============================================================================*/

/**
 * @defgroup ProtocolConfig Protocol Multiplexing Configuration
 * @brief Layer 3 protocol demultiplexing support
 *
 * AX.25 v2.2 Section 6.5 defines Protocol Identifier (PID) based multiplexing
 * to support multiple Layer 3 protocols over a single data link connection.
 * This implementation supports up to 8 concurrent protocol handlers plus
 * a default handler for unregistered PIDs.
 */
#define AX25_MAX_PROTOCOL_HANDLERS 8 /**< Maximum registered protocol handlers */

/*============================================================================*/
/* Type Definitions                                                           */
/*============================================================================*/

/**
 * @brief Protocol handler callback function type
 *
 * Callback signature for Layer 3 protocol data reception. Registered handlers
 * receive payload data from I-frames matching their registered PID.
 *
 * @param user_data Context pointer provided during registration
 * @param data Pointer to received payload data (I-frame information field)
 * @param len Length of payload data in bytes
 * @param pid Protocol Identifier from received frame
 *
 * @section Handler_Registration
 * Handlers are registered via ax25_register_protocol() and remain active
 * until explicitly unregistered. Multiple handlers may be registered for
 * different PIDs on the same connection.
 *
 * @see ax25_register_protocol()
 * @see ax25_unregister_protocol()
 */
typedef void (*ax25_protocol_handler_t)(void *user_data, uint8_t *data, size_t len, uint8_t pid);

/*============================================================================*/
/* Statistics Structure                                                       */
/*============================================================================*/

/**
 * @brief AX.25 link statistics and diagnostics structure
 *
 * Comprehensive statistics collection for link quality monitoring,
 * performance analysis, and debugging. Counters are designed to handle
 * high-volume links without frequent overflow.
 *
 * @section Counter_Design
 * - Frame counters: 32-bit to handle high-traffic links
 * - Error counters: 16-bit sufficient for error tracking
 * - State variables: Current operational parameters
 *
 * @section Statistics_Usage
 * Statistics are updated automatically during frame processing.
 * Access via ax25_get_statistics() for read-only access.
 * Reset via ax25_reset_statistics() to clear all counters.
 */
typedef struct {
    /* Frame counters - 32-bit for high volume handling */
    uint32_t iframe_sent; /**< Total I-frames transmitted */
    uint32_t iframe_received; /**< Total I-frames received */
    uint32_t iframe_retransmitted; /**< I-frames retransmitted due to error recovery */
    uint32_t sframe_sent; /**< Total S-frames (RR/RNR/REJ/SREJ) transmitted */
    uint32_t sframe_received; /**< Total S-frames received */
    uint32_t uframe_sent; /**< Total U-frames transmitted */
    uint32_t uframe_received; /**< Total U-frames received */

    /* Error counters - 16-bit sufficient for error tracking */
    uint16_t fcs_errors; /**< Frames discarded due to FCS mismatch */
    uint16_t aborts; /**< Frame abort sequences received */
    uint16_t overruns; /**< Receiver buffer overruns */
    uint16_t crc_errors; /**< CRC calculation errors */
    uint16_t frmr_sent; /**< FRMR frames transmitted */
    uint16_t frmr_received; /**< FRMR frames received */

    /* Performance metrics */
    uint16_t t1_expirations; /**< T1 acknowledgment timer expirations */
    uint16_t retries; /**< Total retransmission attempts */
    uint32_t bytes_sent; /**< Total payload bytes transmitted */
    uint32_t bytes_received; /**< Total payload bytes received */

    /* Current state snapshot */
    uint8_t current_vs; /**< Current send state variable V(S) */
    uint8_t current_vr; /**< Current receive state variable V(R) */
    uint8_t current_va; /**< Current acknowledge state variable V(A) */
    uint8_t tx_queue_depth; /**< Current transmit queue occupancy */
} ax25_statistics_t;

/*============================================================================*/
/* Protocol Handler Entry                                                     */
/*============================================================================*/

/**
 * @brief Protocol handler registration entry
 *
 * Internal structure tracking registered Layer 3 protocol handlers.
 * Maintains handler function pointer, context data, PID value, and
 * active status.
 *
 * @section Entry_Management
 * Entries are managed internally by the registration API. Active entries
 * are scanned during I-frame reception to dispatch payloads to the
 * appropriate handler based on PID matching.
 */
typedef struct {
    uint8_t pid; /**< Protocol Identifier for this handler */
    ax25_protocol_handler_t handler; /**< Handler callback function */
    void *user_data; /**< Context pointer passed to handler */
    bool active; /**< Entry valid flag */
} ax25_protocol_entry_t;

/*============================================================================*/
/* Link State Enumeration                                                     */
/*============================================================================*/

/**
 * @brief Data Link State Machine states
 *
 * Per AX.25 v2.2 Section 2.5 and Appendix C4, the Data Link State Machine
 * operates in the following states. State transitions are triggered by
 * primitive requests from Layer 3, received frames from the peer station,
 * or timer expirations.
 *
 * @section State_Descriptions
 * - DISCONNECTED: No link established, initial state
 * - AWAITING_CONNECTION: SABM/SABME sent, waiting for UA response
 * - AWAITING_RELEASE: DISC sent, waiting for UA/DM response
 * - CONNECTED: Information transfer phase active
 * - TIMER_RECOVERY: T1 expired, retransmitting unacknowledged frames
 * - AWAITING_SABM: Received SABM, preparing UA response
 * - AWAITING_DISC: Received DISC, preparing UA response
 * - FRAME_REJECT: FRMR sent due to protocol error, awaiting reset
 *
 * @section State_Transitions
 * State transitions follow the SDL diagrams in Appendix C4. Key transitions:
 * - DISCONNECTED -> AWAITING_CONNECTION: DL-CONNECT request
 * - AWAITING_CONNECTION -> CONNECTED: UA received
 * - CONNECTED -> TIMER_RECOVERY: T1 expiration with unacknowledged frames
 * - TIMER_RECOVERY -> CONNECTED: Valid acknowledgment received
 * - Any -> DISCONNECTED: DL-DISCONNECT request or excessive retries
 */
typedef enum {
    AX25_STATE_DISCONNECTED = 0, /**< No connection established */
    AX25_STATE_AWAITING_CONNECTION, /**< Sent SABM (mod-8), awaiting UA */
    AX25_STATE_AWAITING_RELEASE, /**< Sent DISC, awaiting UA/DM */
    AX25_STATE_CONNECTED, /**< Information transfer phase */
    AX25_STATE_TIMER_RECOVERY, /**< Retransmitting after T1 expiry */
    AX25_STATE_AWAITING_CONN_2_2, /**< Sent SABME (mod-128), awaiting UA */
    AX25_STATE_AWAITING_SABM, /**< Received SABM, sending UA */
    AX25_STATE_AWAITING_DISC, /**< Received DISC, sending UA */
    AX25_STATE_FRAME_REJECT, /**< Protocol error, FRMR sent */
    AX25_STATE_COUNT /**< Sentinel: total number of states */
} ax25_link_state_t;

/*============================================================================*/
/* State Variables Structure                                                  */
/*============================================================================*/

/**
 * @brief AX.25 state variables per Section 4.2.2
 *
 * Sequence number management variables maintaining the sliding window
 * protocol state. All sequence numbers use modulo arithmetic (8 or 128).
 *
 * @section Variable_Descriptions
 * - V(S): Send state variable - next N(S) to assign to outgoing I-frame
 * - V(R): Receive state variable - next expected N(S) from peer
 * - V(A): Acknowledge state variable - oldest unacknowledged N(S)
 * - mod: Modulus (8 or 128) determining sequence number range
 *
 * @section Window_Management
 * The transmit window is defined by [V(A), V(S)) modulo mod.
 * Outstanding frames = (V(S) - V(A)) mod mod.
 * Window is closed when outstanding >= k (window size).
 */
typedef struct {
    uint8_t vs; /**< Send state variable V(S) - next transmit sequence number */
    uint8_t vr; /**< Receive state variable V(R) - expected receive sequence number */
    uint8_t va; /**< Acknowledge state variable V(A) - oldest unacknowledged frame */
    uint8_t mod; /**< Modulus: 8 (standard) or 128 (extended) */
} ax25_state_vars_t;

/*============================================================================*/
/* Timer Configuration Structure                                              */
/*============================================================================*/

/**
 * @brief Timer and parameter configuration per Section 6.7
 *
 * System-defined parameters governing link operation timing and capacity.
 * Timers are specified in 10ms units for efficient embedded processing.
 *
 * @section Timer_Descriptions
 * - T1: Acknowledgment timer - wait for ACK before retransmission
 * - T2: Response delay timer - delay before sending standalone ACK
 * - T3: Inactive link timer - poll link during idle periods
 * - N2: Maximum retry count before connection failure
 * - k: Window size - maximum outstanding I-frames
 * - N1: Maximum I-field length in bytes
 *
 * @section Default_Values
 * - T1: 300 (3 seconds) - must be > 2x maximum round-trip time
 * - T2: 150 (1.5 seconds) - allows piggyback ACK opportunity
 * - T3: 3000 (30 seconds) - idle link polling interval
 * - N2: 10 retries
 * - k: 7 (modulo-8) or 32 (modulo-128)
 * - N1: 256 bytes
 */
typedef struct {
    uint16_t t1; /**< T1: Acknowledgment timer (10ms units) */
    uint16_t t2; /**< T2: Response delay timer (10ms units) */
    uint16_t t3; /**< T3: Inactive link timer (10ms units) */
    uint8_t n2; /**< N2: Maximum number of retries */
    uint8_t k; /**< k: Window size (max outstanding I-frames) */
    uint16_t n1; /**< N1: Maximum I-field length (octets) */
} ax25_timers_t;

/*============================================================================*/
/* Retransmission Queue                                                       */
/*============================================================================*/

/**
 * @brief I-frame retransmission queue structure
 *
 * Circular buffer maintaining unacknowledged I-frames for potential
 * retransmission. Stores encoded frame data, frame lengths, and sequence
 * numbers for window management.
 *
 * @section Queue_Operation
 * - Frames added at tail when transmitted
 * - Frames removed from head when acknowledged (N(R) advancement)
 * - Retransmission scans from head to tail
 * - Full when count >= k (window size)
 *
 * @section Memory_Management
 * Frame data is dynamically allocated and stored until acknowledged.
 * On queue removal, frame memory is freed. Queue capacity is limited
 * by AX25_MAX_QUEUE_SIZE to prevent unbounded memory growth.
 */
#define AX25_MAX_QUEUE_SIZE 16 /**< Maximum queued frames for retransmission */

typedef struct {
    uint8_t *frames[AX25_MAX_QUEUE_SIZE]; /**< Encoded frame data pointers */
    size_t lengths[AX25_MAX_QUEUE_SIZE]; /**< Frame length for each entry */
    uint8_t ns[AX25_MAX_QUEUE_SIZE]; /**< N(S) sequence number tracking */
    uint8_t head; /**< Queue head index (oldest frame) */
    uint8_t tail; /**< Queue tail index (newest frame) */
    uint8_t count; /**< Current queue occupancy */
} ax25_frame_queue_t;

/*============================================================================*/
/* Callback Interface                                                         */
/*============================================================================*/

/**
 * @brief Upper layer callback interface
 *
 * Function pointers for Data-Link Service Access Point (DLSAP) primitives
 * per AX.25 v2.2 Section 5.3 and Appendix D.
 *
 * @section Primitive_Types
 * - on_connect: DL-CONNECT confirm - link establishment notification
 * - on_disconnect: DL-DISCONNECT indicate - link termination notification
 * - on_data: DL-DATA indicate - received I-frame data delivery
 * - on_busy: Remote busy state change notification
 * - transmit: PH-DATA request - physical layer transmission
 * - abort_tx: Optional transmit abort for full-duplex operation
 *
 * @section Callback_Requirements
 * All callbacks except abort_tx are mandatory for proper operation.
 * The transmit callback must handle frame delivery to the physical layer.
 * Callbacks are invoked from ax25_process_frame() and ax25_tick() contexts.
 */
typedef struct {
    /**
     * @brief Connection established notification
     * @param user_data         Context pointer from connection initialization
     * @param initiated_locally true  = DL-CONNECT confirm  (active open, UA received)
     *                          false = DL-CONNECT indication (passive open, SABM received)
     *
     * Invoked on transition to CONNECTED state.  Layer 3 uses initiated_locally
     * to distinguish confirm from indication per AX.25 v2.2 Section 5.3 / Appendix D.3.
     * conn->peer_addr is valid and stable at callback time.
     */
    void (*on_connect)(void *user_data, bool initiated_locally);

    /**
     * @brief Connection terminated notification
     * @param user_data Context pointer from connection initialization
     * @param reason Termination reason code:
     *               0 = Normal disconnect (DISC/UA exchange)
     *               1 = Remote disconnect (DISC received)
     *               2 = FRMR received (protocol error)
     *               3 = Retry limit exceeded (N2 timeout)
     *
     * Invoked on transition to DISCONNECTED state from any active state.
     */
    void (*on_disconnect)(void *user_data, uint8_t reason);

    /**
     * @brief Received data delivery
     * @param user_data Context pointer from connection initialization
     * @param data Pointer to received I-frame payload
     * @param len Payload length in bytes
     *
     * Delivers validated, in-sequence I-frame payload to Layer 3.
     * Data pointer is valid only during callback execution.
     *  AX.25 v2.2 Appendix D.4 DL-DATA indication:
     * Layer 3 must receive PID to demultiplex IP (0xCC), NET/ROM (0xCF), etc.
     */
    void (*on_data)(void *user_data, uint8_t *data, size_t len, uint8_t pid);

    /**
     * @brief Remote busy state change
     * @param user_data Context pointer from connection initialization
     * @param busy true = peer entered busy (RNR received)
     *             false = peer cleared busy (RR/REJ/SABM received)
     *
     * Notifies Layer 3 of peer receiver status changes.
     */
    void (*on_busy)(void *user_data, bool busy);

    /**
     * @brief Physical layer transmission request
     * @param user_data Context pointer from connection initialization
     * @param frame Pointer to encoded frame data
     * @param len Frame length in bytes
     *
     * Delivers fully encoded AX.25 frame to physical layer for
     * transmission. Frame data must be transmitted as-is without
     * modification.
     */
    void (*transmit)(void *user_data, uint8_t *frame, size_t len);

    /**
     * @brief Optional transmit abort (full-duplex only)
     * @param user_data Context pointer from connection initialization
     *
     * Aborts in-progress transmission to allow immediate retransmission.
     * Required for full-duplex REJ response per Section 6.4.7.
     * May be NULL if physical layer does not support abort.
     */
    void (*abort_tx)(void *user_data);

    // DL-ERROR indication per AX.25 v2.2 Section 17.2
    void (*on_dl_error)(void *user_data, ax25_dl_error_t error);  // NULL = errors ignored

    // DL-UNIT-DATA indication per AX.25 v2.2 Appendix D.4.
    // on_data does not supply the sender address; connectionless protocols
    // such as APRS and NET/ROM require the source callsign from every UI frame.
    // Invoked for all received UI frames before dispatch_to_protocol.
    // Set to NULL if the upper layer does not need UI source-address notification.
    void (*on_ui_data)(void *user_data, const ax25_address_t *src, uint8_t *data, size_t len, uint8_t pid);

} ax25_callbacks_t;

/*============================================================================*/
/* Reject Mode Configuration                                                  */
/*============================================================================*/

/**
 * @brief Error recovery mode enumeration
 *
 * Per AX.25 v2.2 Section 6.4.4, defines the error recovery mechanism
 * negotiated for the link. Modes are ordered by capability:
 * SREJ/REJ > SREJ > REJ > NONE.
 *
 * @section Mode_Descriptions
 * - NONE: No error recovery negotiated (invalid operational state)
 * - REJ: Implicit reject only - go-back-N recovery
 * - SREJ: Selective reject only - single frame recovery
 * - SREJ_REJ: Both modes - selective reject with REJ fallback
 *
 * @section Negotiation
 * Mode is negotiated via XID HDLC Optional Functions parameter (PI=3).
 * Per Section 6.3.2, default is SREJ/REJ for AX.25 v2.2.
 * Fallback to lower capability if either station cannot support higher.
 */
typedef enum {
    AX25_REJ_MODE_NONE = 0, /**< No reject mode (uninitialized) */
    AX25_REJ_MODE_REJ, /**< Implicit reject (go-back-N) only */
    AX25_REJ_MODE_SREJ, /**< Selective reject only */
    AX25_REJ_MODE_SREJ_REJ /**< Selective reject with REJ fallback (default) */
} ax25_rej_mode_t;

/*============================================================================*/
/* TEST Frame Statistics                                                      */
/*============================================================================*/

/**
 * @brief TEST frame round-trip time statistics
 *
 * Per AX.25 v2.2 Section 6.4.13, TEST frames provide link quality
 * verification and round-trip time measurement. Statistics are used
 * for adaptive T1 timer adjustment per Section 6.7.1.1.
 *
 * @section RTT_Calculation
 * RTT measured from TEST command transmission to TEST response reception.
 * EMA RTT (alpha=1/8). ema_rtt * 10ms gives milliseconds.
 * Used to set T1 = 2 * RTT + margin.
 */
typedef struct {
    uint16_t test_sent; /**< TEST commands transmitted */
    uint16_t test_received; /**< TEST responses received */
    uint16_t test_lost; /**< TEST timeouts (no response) */
    uint32_t last_test_tick; /**< Timestamp of last TEST command (10ms) */
    uint32_t ema_rtt; /**< EMA of RTT in 10ms ticks; 0 = no data yet */
    uint8_t ema_seeded; /**< 1 once first sample loaded */
    uint8_t test_sequence; /**< TEST sequence number for tracking */
} ax25_test_stats_t;

/*============================================================================*/
/* Event-Driven FSM Interface (AX.25 v2.2 Appendix C4 SDL)                   */
/*============================================================================*/

// ax25_event_t: all events that drive the AX.25 v2.2 Data-Link State Machine.
// Corresponds to the SDL event inputs in Appendix C4 of the specification.
typedef enum {
    AX25_EV_DL_CONNECT = 0,  // Upper layer: request connection (DL-CONNECT request)
    AX25_EV_DL_DISCONNECT,   // Upper layer: request disconnect (DL-DISCONNECT request)
    AX25_EV_DL_DATA,         // Upper layer: send data (DL-DATA request)
    AX25_EV_DL_FLOW_ON,      // Upper layer: clear local busy (DL-FLOW-ON request)
    AX25_EV_DL_FLOW_OFF,     // Upper layer: set local busy (DL-FLOW-OFF request)
    AX25_EV_RX_SABM,         // Received SABM command from peer
    AX25_EV_RX_SABME,        // Received SABME command from peer (mod-128)
    AX25_EV_RX_DISC,         // Received DISC command from peer
    AX25_EV_RX_UA,           // Received UA response (F=0) from peer
    AX25_EV_RX_UA_F1,        // Received UA response with F=1 from peer
    AX25_EV_RX_DM,           // Received DM response from peer
    AX25_EV_RX_I,            // Received I-frame from peer
    AX25_EV_RX_RR,           // Received RR supervisory frame
    AX25_EV_RX_RNR,          // Received RNR supervisory frame
    AX25_EV_RX_REJ,          // Received REJ supervisory frame
    AX25_EV_RX_SREJ,         // Received SREJ supervisory frame
    AX25_EV_RX_FRMR,         // Received FRMR frame
    AX25_EV_RX_UI,           // Received UI unnumbered information frame
    AX25_EV_RX_TEST,         // Received TEST frame (command or response)
    AX25_EV_T1_EXPIRED,      // T1 acknowledgment timer expired
    AX25_EV_T2_EXPIRED,      // T2 response delay timer expired
    AX25_EV_T3_EXPIRED,      // T3 inactive link timer expired
    AX25_EV_COUNT            // Sentinel: total number of events
} ax25_event_t;

// ax25_event_data_t: event-specific parameter block.
// The active union member depends on the ax25_event_t value:
//   AX25_EV_DL_CONNECT  -> connect.{dest, src}
//   AX25_EV_DL_DATA     -> data.{data, len, pid}
//   AX25_EV_RX_*        -> frame (decoded ax25_frame_t*) + tick
//   AX25_EV_T*_EXPIRED  -> tick (current 10ms tick counter)
typedef union {
    struct {
        ax25_address_t *dest;
        ax25_address_t *src;
    } connect;
    struct {
        uint8_t *data;
        size_t len;
        uint8_t pid;
    } data;
    ax25_frame_t *frame;
    uint32_t tick;
} ax25_event_data_t;

/*============================================================================*/
/* Main Connection Context                                                    */
/*============================================================================*/

/**
 * @brief AX.25 connection context structure
 *
 * Complete state representation for a single AX.25 data link connection.
 * Contains all state variables, timers, queues, and configuration needed
 * to implement the Data Link State Machine per AX.25 v2.2 Appendix C4.
 *
 * @section Context_Lifecycle
 * 1. Initialize with ax25_connection_init()
 * 2. Establish connection with ax25_connect() or process incoming SABM
 * 3. Exchange data via ax25_send_data() and ax25_process_frame()
 * 4. Maintain with periodic ax25_tick() calls (every 10ms)
 * 5. Terminate with ax25_disconnect() or process incoming DISC
 *
 * @section Thread_Safety
 * This structure is not thread-safe. External synchronization required
 * if accessed from multiple contexts (ISR, main loop, callbacks).
 *
 * @section Memory_Management
 * Dynamically allocates frame data for retransmission queue.
 * Freed on acknowledgment or connection termination.
 */
typedef struct {
    /* Core state machine */
    ax25_link_state_t state; /**< Current link state */
    ax25_state_vars_t vars; /**< Sequence number state variables */
    ax25_timers_t timers; /**< Timer configuration and parameters */
    ax25_frame_header_t peer_addr; /**< Connected station address */
    ax25_frame_queue_t tx_queue; /**< Unacknowledged I-frame queue */
    ax25_callbacks_t callbacks; /**< Upper layer callback interface */
    void *user_data; /**< Upper layer context pointer */

    // Raw tick fields t1_start_tick / t2_start_tick / t3_start_tick removed.
    // ax25_timer_t carries start_ms, duration_ms and a running flag, eliminating
    // both the UINT32_MAX sentinel and the separate t2_running flag.
    // last_tick_10ms caches the most recent tick for helpers that have no tick param.
    ax25_timer_t t1;             // T1 acknowledgment timer
    ax25_timer_t t2;             // T2 response delay timer (running replaces t2_running)
    ax25_timer_t t3;             // T3 inactive link timer
    uint32_t last_tick_10ms;     // last tick seen; updated by ax25_tick/ax25_process_frame
    uint8_t retry_count; /**< Current retry counter for T1 expiry */

    /* Flow control state */
    bool peer_busy; /**< Remote station in busy condition (RNR) */
    bool local_busy; /**< Local station in busy condition */
    uint32_t rnr_start_tick; /**< Time when peer busy was detected */

    /* Address and negotiation */
    ax25_negotiated_params_t params; /**< XID negotiated parameters */
    uint16_t t3_timeout; /**< T3 timeout value (10ms units) */

    // SABME/SABM modulo negotiation per PE1CHL §6
    // want_mod128: set to 1 before calling ax25_connect() to request a
    // mod-128 connection via SABME. If the peer responds with DM or FRMR
    // the state machine falls back automatically to mod-8 SABM.
    // Cleared to 0 after fallback or after a successful connection.
    uint8_t want_mod128;

    /* Duplex mode */
    bool full_duplex; /**< Full-duplex operation enabled */

    /* Selective Reject state per Section 6.4.4 */
    ax25_rej_mode_t rej_mode; /**< Negotiated error recovery mode */
    bool srej_exception; /**< SREJ recovery in progress */
    uint8_t srej_first_missing; /**< First missing frame sequence number */
    uint8_t srej_count; /**< Number of pending SREJ conditions */
    uint8_t srej_max; /**< Maximum simultaneous SREJ (typically 1) */
    uint8_t srej_bitmap[16]; /**< Bitmap of frames with pending SREJ */

    /* Implicit Reject state */
    bool rej_exception; /**< REJ recovery in progress */

    /* SREJ frame buffering per Section 6.4.4.2 */
    uint8_t srej_buffer[AX25_MAX_QUEUE_SIZE][AX25_SREJ_BUFFER_SIZE];  // Buffered out-of-seq frames (was magic literal 256)
    // srej_buffer_len widened from uint8_t to uint16_t so a full-size N1=256 payload
    // (AX25_DEFAULT_N1) is stored without silent truncation to 255 bytes.
    // uint16_t covers [0, 65535], comfortably exceeding AX25_SREJ_BUFFER_SIZE (max 256).
    uint16_t srej_buffer_len[AX25_MAX_QUEUE_SIZE]; /**< Buffered frame lengths */
    uint8_t srej_buffer_ns[AX25_MAX_QUEUE_SIZE]; /**< Buffered frame N(S) values */
    uint8_t srej_buffer_pid[AX25_MAX_QUEUE_SIZE]; /**< Buffered frame PID values - required for correct DL-DATA indication on reorder */
    uint8_t srej_buffer_count; /**< Number of buffered frames */

    /* FRMR state per Section 4.4.5 */
    bool frmr_pending; /**< FRMR retransmission required */
    uint8_t frmr_info[5]; /**< Stored FRMR info field (3 or 5 bytes) */
    uint8_t frmr_info_len; /**< Length of stored FRMR info */
    uint8_t frmr_retry_count; /**< FRMR retransmission counter */

    bool t2_ack_pending; /**< Delayed ACK pending T2 expiry */
    uint8_t t2_pending_nr; /**< N(R) value for delayed ACK */

    // T101 PRIACK timer per §6.7.1 timers table
    // T101 bounds the piggyback-ACK opportunity window. Started whenever an
    // I-frame is received and a deferred RR ACK is pending (same trigger as T2).
    // When T101 expires before T2, a standalone RR/RNR is sent immediately so
    // the peer is not left unacknowledged for longer than 2 s (AX.25 §6.7.1).
    // Uses ax25_timer_t (ms resolution) driven by ax25_tick (10ms ticks * 10u).
    ax25_timer_t t101;  // T101 Priority Window (PRIACK) 2 s timer

    /* Link quality monitoring */
    ax25_test_stats_t test_stats; /**< TEST frame RTT statistics */

    /* Protocol multiplexing per Section 6.5 */
    ax25_protocol_entry_t protocols[AX25_MAX_PROTOCOL_HANDLERS]; /**< PID handlers */
    ax25_protocol_handler_t default_handler; /**< Fallback handler */
    void *default_user_data; /**< Fallback context */

    /* Diagnostics */
    ax25_statistics_t stats; /**< Link statistics counters */

    // optional MDL context pointer for FRMR-to-MDL error bridging
    ax25_mgmt_context_t *mgmt_ctx;  // NULL = MDL bridging disabled

    // MDL transmit trampoline for XID frame delivery.
    // ax25_mgmt_start_negotiation and ax25_mgmt_process_xid expect a
    // void (*)(uint8_t*, size_t) transmit function with no user_data argument,
    // while conn->callbacks.transmit carries void* user_data as first argument.
    // The caller sets this pointer to a thin wrapper that captures the connection
    // and forwards to conn->callbacks.transmit. NULL disables automatic XID negotiation
    // even when mgmt_ctx is set; both must be non-NULL to enable MDL auto-negotiation.
    void (*mdl_transmit_trampoline)(uint8_t *frame, size_t len);

    // Segmenter/reassembler per AX.25 v2.2 Section 2.4 / Appendix C6
    // Automatically applied: ax25_send_data uses it when len > N1,
    // and received PID=0x08 frames are routed through it for reassembly.
    ax25_segmenter_t segmenter;
} ax25_connection_t;

// ax25_action_fn: function pointer for one (state, event) FSM cell.
// NULL entries in the dispatch table mean "ignore this event in this state"
// per the SDL "else" transition convention.
typedef void (*ax25_action_fn)(ax25_connection_t *conn, const ax25_event_data_t *ev_data);

/*============================================================================*/
/* Initialization and Connection Management                                   */
/*============================================================================*/

/**
 * @brief Initialize connection context
 *
 * Initializes all state variables, timers, and queues to default values.
 * Must be called before any other connection operations.
 *
 * @param[out] conn Connection context to initialize
 * @param[in]  cb   Callback interface structure (copied)
 * @param[in]  user_data Context pointer passed to all callbacks
 * @return 0 on success, 1 on invalid parameters
 *
 * @section Postconditions
 * - State = DISCONNECTED
 * - Modulus = 8 (standard)
 * - Timers set to specification defaults
 * - All counters and flags cleared
 * - Queue empty
 */
uint8_t ax25_connection_init(ax25_connection_t *conn, ax25_callbacks_t *cb, void *user_data);

/**
 * @brief Initiate connection establishment
 *
 * Transitions from DISCONNECTED to AWAITING_CONNECTION state.
 * Sends SABM command to peer station and starts T1 timer.
 *
 * @param[in,out] conn Connection context
 * @param[in]     dest Destination station address
 * @param[in]     src  Local station address
 * @return 0 on success, 1 on invalid parameters, 2 if not disconnected
 *
 * @section Preconditions
 * Connection must be in DISCONNECTED state.
 *
 * @section Postconditions
 * - State = AWAITING_CONNECTION
 * - SABM transmitted with P=1
 * - T1 started for UA response timeout
 */
uint8_t ax25_connect(ax25_connection_t *conn, ax25_address_t *dest, ax25_address_t *src);

/**
 * @brief Initiate connection termination
 *
 * Transitions to AWAITING_RELEASE state. Sends DISC command,
 * flushes transmit queue, and awaits UA response.
 *
 * @param[in,out] conn Connection context
 * @return 0 on success, 1 on invalid parameters, 2 if not connected
 *
 * @section Preconditions
 * Connection must be in CONNECTED or TIMER_RECOVERY state.
 *
 * @section Postconditions
 * - State = AWAITING_RELEASE
 * - DISC transmitted with P=1
 * - Transmit queue cleared (frames discarded)
 * - T1 started for UA/DM response timeout
 */
uint8_t ax25_disconnect(ax25_connection_t *conn);

/*============================================================================*/
/* Data Transfer                                                              */
/*============================================================================*/

/**
 * @brief Send connected-mode data (I-frame)
 *
 * Transmits data as an Information frame with sequence numbering.
 * Frame is queued for retransmission until acknowledged.
 *
 * @param[in,out] conn Connection context
 * @param[in]     data Payload data pointer
 * @param[in]     len  Payload length (clamped to N1)
 * @param[in]     pid  Protocol Identifier for Layer 3 multiplexing
 * @return 0 on success,
 *         1 = invalid parameters,
 *         2 = not connected,
 *         3 = window closed or queue full,
 *         4 = encoding failed,
 *         5 = peer busy (RNR received)
 *         6 = recovery in progress (TIMER_RECOVERY state, retry later)
 *
 * @section Preconditions
 * - Connection in CONNECTED state
 * - Window open: (V(S)-V(A)) mod mod < k
 * - Peer not in busy condition
 *
 * @section Postconditions
 * - I-frame transmitted
 * - Frame queued for retransmission
 * - V(S) incremented
 * - T1 started if not running
 */
uint8_t ax25_send_data(ax25_connection_t *conn, uint8_t *data, size_t len, uint8_t pid);

/*============================================================================*/
/* Frame Processing                                                           */
/*============================================================================*/

/**
 * @brief Process received AX.25 frame
 *
 * Main entry point for received frames. Dispatches to appropriate
 * handler based on frame type and current state.
 *
 * @param[in,out] conn Connection context
 * @param[in]     frame Decoded frame structure
 * @param[in]     current_tick_10ms Current system tick (10ms resolution)
 *
 * @section Processing
 * - Validates frame based on current state
 * - Updates state variables (V(R), V(A)) per Section 4.2.2
 * - Handles acknowledgments (N(R) processing)
 * - Dispatches to type-specific handlers (RR, REJ, I-frame, etc.)
 * - Manages timer restarts on activity
 */
void ax25_process_frame(ax25_connection_t *conn, ax25_frame_t *frame, uint32_t current_tick_10ms);

/**
 * @brief Periodic timer processing
 *
 * Must be called every 10ms to maintain protocol timers.
 * Handles T1 (acknowledgment), T2 (response delay), and T3
 * (inactive link) timer expirations.
 *
 * @param[in,out] conn Connection context
 * @param[in]     current_tick_10ms Current system tick (10ms resolution)
 *
 * @section Timer_Handling
 * - T1 expiry: Retransmit unacknowledged frames or send poll
 * - T2 expiry: Send delayed RR acknowledgment
 * - T3 expiry: Send poll to verify link integrity
 *
 * @section Implementation_Notes
 * Timer arithmetic uses unsigned 32-bit subtraction to handle
 * wraparound correctly. Ticks must increment monotonically.
 */
void ax25_tick(ax25_connection_t *conn, uint32_t current_tick_10ms);

/*============================================================================*/
/* Flow Control                                                               */
/*============================================================================*/

/**
 * @brief Send Receive Not Ready (RNR) indication
 *
 * Signals local busy condition to peer station. Peer must stop
 * sending I-frames until RR or REJ received.
 *
 * @param[in,out] conn Connection context
 * @return 0 on success, 1 on invalid parameters or not connected
 *
 * @section Preconditions
 * Connection in CONNECTED state.
 *
 * @section Postconditions
 * - RNR transmitted with current N(R)
 * - local_busy flag set
 */
uint8_t ax25_send_rnr(ax25_connection_t *conn);

/**
 * @brief Clear local busy condition
 *
 * Signals ready to receive by sending RR frame. Clears local
 * busy condition established by ax25_send_rnr().
 *
 * @param[in,out] conn Connection context
 * @return 0 on success, 1 on invalid parameters or not busy
 *
 * @section Postconditions
 * - RR transmitted with current N(R)
 * - local_busy flag cleared
 */
uint8_t ax25_clear_local_busy(ax25_connection_t *conn);

/*============================================================================*/
/* Connectionless Operations                                                  */
/*============================================================================*/

/**
 * @brief Send Unnumbered Information (UI) frame
 *
 * Connectionless datagram transmission per Section 6.4.12.
 * No acknowledgment, sequence numbers, or connection required.
 * Accepted in any state.
 *
 * @param[in] dest Destination station address
 * @param[in] src  Source station address
 * @param[in] data Payload data
 * @param[in] len  Payload length
 * @param[in] pid  Protocol Identifier
 * @param[in] transmit Physical layer transmit callback
 * @return 0 on success, 1 = invalid parameters, 2 = no data
 *
 * @section Usage
 * Used for APRS, beacons, and connectionless Layer 3 protocols.
 * Frame is sent immediately without queueing or retransmission.
 */
uint8_t ax25_send_ui(ax25_address_t *dest, ax25_address_t *src, uint8_t *data, size_t len, uint8_t pid, void (*transmit)(uint8_t*, size_t));

/**
 * @brief Send UI frame using connection context (DL-UNIT-DATA request)
 *
 * Connection-context variant of ax25_send_ui(). Uses the connection's
 * own transmit callback and user_data, and updates uframe_sent statistics.
 * Per AX.25 v2.2 Appendix D.4, UI frames are valid in all connection states.
 *
 * @param[in,out] conn Connection context (provides addresses and transmit callback)
 * @param[in]     data Payload data
 * @param[in]     len  Payload length (must be > 0)
 * @param[in]     pid  Protocol Identifier
 * @return 0 on success, 1 = invalid parameters or no transmit callback,
 *         3 = encoding failed
 */
uint8_t ax25_send_ui_conn(ax25_connection_t *conn, uint8_t *data, size_t len, uint8_t pid);

/*============================================================================*/
/* Link Quality and Testing                                                   */
/*============================================================================*/

/**
 * @brief Send TEST command frame
 *
 * Transmits TEST command per Section 6.4.13 for link quality
 * verification. Peer must echo payload in TEST response.
 *
 * @param[in,out] conn Connection context
 * @param[in]     payload Test payload data (echoed by peer)
 * @param[in]     payload_len Payload length (max 256 bytes)
 * @return 0 on success, 1 = invalid parameters, 2 = payload too large,
 *         3 = encoding failed
 *
 * @section RTT_Measurement
 * Caller should record current_tick before calling, then calculate
 * RTT when TEST response received. Update conn->test_stats.last_test_tick
 * to enable automatic RTT tracking.
 */
uint8_t ax25_send_test_command(ax25_connection_t *conn, uint8_t *payload, size_t payload_len);

/**
 * @brief Get average round-trip time
 *
 * Calculates average RTT from accumulated TEST frame statistics.
 *
 * @param[in] conn Connection context
 * @return Average RTT in milliseconds, 0 if no measurements
 *
 * @section Calculation
 * Returns ema_rtt * 10ms. Returns 0 if no measurement taken yet.
 * Used for adaptive T1 adjustment per Section 6.7.1.1.
 */
uint32_t ax25_get_average_rtt_ms(ax25_connection_t *conn);

/*============================================================================*/
/* Parameter Negotiation                                                      */
/*============================================================================*/

/**
 * @brief Apply XID negotiated parameters
 *
 * Updates connection parameters based on XID negotiation results
 * per Section 6.7.2. Called after successful XID exchange.
 *
 * @param[in]     mgmt_ctx Management context with negotiated parameters
 * @param[in,out] conn     Connection to update
 * @param[in,out] phys     Physical layer for duplex mode propagation (may be NULL)
 * @return 0 on success, 1 on invalid parameters or negotiation incomplete
 *
 * @section Applied_Parameters
 * - Full/half duplex mode
 * - Modulo 8/128 selection
 * - REJ/SREJ/SREJ-REJ mode
 * - Timer values (T1, T2, T3)
 * - Retry count N2
 * - Window size k
 * - Maximum I-field length N1
 */
uint8_t ax25_apply_negotiated_params(ax25_mgmt_context_t *mgmt_ctx, ax25_connection_t *conn, ax25_physical_t *phys);

/**
 * @brief Query duplex mode status
 *
 * @param[in] conn Connection context
 * @return true if full-duplex operation enabled, false if half-duplex
 */
bool ax25_is_full_duplex(ax25_connection_t *conn);

/*============================================================================*/
/* Protocol Multiplexing                                                      */
/*============================================================================*/

/**
 * @brief Register Layer 3 protocol handler
 *
 * Registers callback for specific PID value per Section 6.5.
 * Incoming I-frames with matching PID are dispatched to handler.
 *
 * @param[in,out] conn      Connection context
 * @param[in]     pid       Protocol Identifier to register
 * @param[in]     handler   Callback function for this PID
 * @param[in]     user_data Context pointer passed to handler
 * @return 0 on success, 1 = invalid parameters, 2 = no free slots
 *
 * @section Registration_Notes
 * - Maximum AX25_MAX_PROTOCOL_HANDLERS concurrent registrations
 * - Duplicate PID registration updates existing handler
 * - Handler remains active until unregistered
 */
uint8_t ax25_register_protocol(ax25_connection_t *conn, uint8_t pid, ax25_protocol_handler_t handler, void *user_data);

/**
 * @brief Unregister protocol handler
 *
 * Removes registration for specified PID. Subsequent frames with
 * this PID will be dispatched to default handler or discarded.
 *
 * @param[in,out] conn Connection context
 * @param[in]     pid  Protocol Identifier to unregister
 */
void ax25_unregister_protocol(ax25_connection_t *conn, uint8_t pid);

/**
 * @brief Set default protocol handler
 *
 * Sets fallback handler for PIDs without specific registration.
 * Called when no specific handler found for received PID.
 *
 * @param[in,out] conn      Connection context
 * @param[in]     handler   Default callback function (NULL to disable)
 * @param[in]     user_data Context pointer passed to handler
 */
void ax25_set_default_protocol_handler(ax25_connection_t *conn, ax25_protocol_handler_t handler, void *user_data);

/*============================================================================*/
/* Adaptive Timing                                                            */
/*============================================================================*/

/**
 * @brief Adjust T1 based on measured RTT
 *
 * Adaptive timer adjustment per Section 6.7.1.1.
 * Sets T1 = 2 * average_RTT + margin.
 *
 * @param[in,out] conn Connection context
 *
 * @section Algorithm
 * - Calculates average RTT from test_stats
 * - Applies 2x multiplier for safety margin
 * - Adds 100ms margin (full-duplex) or 300ms (half-duplex)
 * - Clamps result to [100ms, 30s] range
 *
 * @section Usage
 * Call after receiving TEST response to progressively optimize
 * T1 for actual link conditions.
 */
void ax25_adjust_t1_adaptive(ax25_connection_t *conn);

/*============================================================================*/
/* Statistics and Diagnostics                                                 */
/*============================================================================*/

/**
 * @brief Get link statistics
 *
 * Returns pointer to statistics structure with current counters
 * and state snapshot. Updates state variables (V(S), V(R), etc.)
 * before returning.
 *
 * @param[in] conn Connection context
 * @return Pointer to statistics structure, NULL on error
 *
 * @section Thread_Safety
 * Structure is internal to connection context. Copy data if
 * needed beyond immediate use. Do not modify returned structure.
 */
const ax25_statistics_t* ax25_get_statistics(ax25_connection_t *conn);

/**
 * @brief Reset all statistics counters
 *
 * Clears all counters in statistics structure to zero.
 * Does not affect current state variables.
 *
 * @param[in,out] conn Connection context
 */
void ax25_reset_statistics(ax25_connection_t *conn);

/*============================================================================*/
/* Forward Error Correction (FX.25)                                           */
/*============================================================================*/

/**
 * @brief Send data with optional FX.25 FEC
 *
 * Wraps AX.25 frame with FX.25 Reed-Solomon forward error correction.
 * Maintains backward compatibility - non-FX.25 receivers see standard
 * AX.25 frame.
 *
 * @param[in,out] conn            Connection context
 * @param[in]     data            Payload data
 * @param[in]     len             Payload length
 * @param[in]     pid             Protocol Identifier
 * @param[in]     use_fx25        Enable FX.25 wrapping if true
 * @param[in]     channel_quality Channel quality indicator (0-100) for mode selection
 * @return 0 on success,
 *         1 = invalid parameters,
 *         2 = frame too large,
 *         3 = window full or FX.25 encoding failed,
 *         4 = memory allocation failed,
 *         5 = peer busy
 *         6 = recovery in progress (TIMER_RECOVERY state, retry later)
 *
 * @section FX25_Integration
 * FX.25 adds 8-byte correlation tag and Reed-Solomon parity bytes
 * to standard AX.25 frame. Improves reliability on noisy channels.
 * See FX.25 specification v0.01.06 for details.
 */
uint8_t ax25_send_data_with_fec(ax25_connection_t *conn, uint8_t *data, size_t len, uint8_t pid, bool use_fx25, uint8_t channel_quality);

/**
 * @brief Free all resources held by a connection without sending any frames
 *
 * Call this before the ax25_connection_t goes out of scope when a full
 * ax25_disconnect() handshake is not needed (e.g. at end of unit tests).
 * Frees every frame currently queued in conn->tx_queue.
 *
 * @param conn Connection to clean up (may be NULL, then no-op)
 */
void ax25_connection_cleanup(ax25_connection_t *conn);

// ax25_process_event: dispatch an event through the FSM sparse action table.
// This provides direct access to the AX.25 v2.2 Appendix C4 SDL.
// For received frames prefer ax25_process_frame(); for timers prefer ax25_tick().
// This function is primarily for testing and explicit SDL-level driving.
//
// @param conn     Connection context
// @param ev       Event identifier (ax25_event_t)
// @param ev_data  Event parameters (may be NULL for events with no payload)
void ax25_process_event(ax25_connection_t *conn, ax25_event_t ev, const ax25_event_data_t *ev_data);

#endif /* AX25_STATE_MACHINE_H_ */
