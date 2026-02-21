/**
 * @file ax25_mgmt.h
 * @brief AX.25 v2.2 Protocol Management Layer - XID Parameter Negotiation
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3.0
 * @date 2026
 *
 * @section Overview
 * This header defines the Management Data-Link State Machine (MDL) for AX.25 v2.2
 * protocol parameter negotiation. The MDL handles the negotiation/notification of
 * operational parameters using XID (Exchange Identification) frames as specified
 * in AX.25 v2.2 Section 6.3.2 and Appendix C5.
 *
 * @section Standards_Reference
 * - AX.25 Link Access Protocol for Amateur Packet Radio, Version 2.2, July 1998
 *   (TAPR Publication)
 * - ISO/IEC 13239:2002 HDLC Procedures
 * - ISO 8885 XID Information Field Exchange
 *
 * @section XID_Negotiation
 * Parameter negotiation occurs using a single command/response exchange:
 * 1. Initiating station sends XID command after UA frame reception
 * 2. Remote station responds with XID response (v2.2+) or FRMR (<v2.2)
 * 3. Both stations apply negotiated parameters from XID response
 *
 * Negotiable parameters include:
 * - Half/Full duplex operation (Class of Procedures)
 * - REJ/SREJ recovery mechanisms (HDLC Optional Functions)
 * - Modulo 8/128 sequence numbers
 * - I-field length (N1), Window size (k)
 * - Timers T1, T2 and retry count N2
 *
 * @section Implementation_Notes
 * - XID frames comply with ISO 8885 general-purpose XID format
 * - FI (Format Identifier) = 0x82 for parameter negotiation
 * - GI (Group Identifier) = 0x80 for general group
 * - Unknown PI values are silently ignored per specification
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef AX25_MGMT_H_
#define AX25_MGMT_H_

#include <stdint.h>
#include <stdbool.h>

#include "ax25.h"

/*============================================================================*/
/* XID Parameter Identifiers (PI) - AX.25 v2.2 Section 6.7.2                  */
/*============================================================================*/

/**
 * @defgroup XID_Parameter_Identifiers XID Parameter Identifier Constants
 * @brief Parameter Identifier (PI) values for XID negotiation frames
 *
 * These constants define the standardized parameter identifiers used in the
 * XID information field. Each PI is followed by a Parameter Length (PL) and
 * Parameter Value (PV) structure.
 *
 * @see AX.25 v2.2 Section 6.7.2, Table 6.1
 * @{
 */

/**
 * @def XID_PI_CLASS_OF_PROCEDURES
 * @brief Parameter Identifier 2: Class of Procedures
 * @details Negotiates half-duplex vs full-duplex operation. Type E (bit field).
 *          PI=2, PL=2. Default: Half-duplex.
 * @see XID_COP_HALF_DUPLEX, XID_COP_FULL_DUPLEX
 */
#define XID_PI_CLASS_OF_PROCEDURES     2

/**
 * @def XID_PI_HDLC_OPTIONAL_FUNCTIONS
 * @brief Parameter Identifier 3: HDLC Optional Functions
 * @details Negotiates REJ/SREJ recovery and modulo 8/128 operation. Type E.
 *          PI=3, PL=3 (3 octets of bit flags).
 *          Default: Selective reject, modulo 8.
 * @see XID_HDLC_REJ, XID_HDLC_SREJ, XID_HDLC_MOD128
 */
#define XID_PI_HDLC_OPTIONAL_FUNCTIONS 3

/**
 * @def XID_PI_IFIELD_LENGTH_RX
 * @brief Parameter Identifier 6: I-Field Length Receive (N1)
 * @details Notification parameter specifying maximum receive I-field size.
 *          Type B (big-endian numeric), PI=6, PL=2.
 *          Default: 256 octets (2048 bits).
 * @note This is a notification parameter, not negotiated. The smaller
 *       of the two values is used.
 */
#define XID_PI_IFIELD_LENGTH_RX        6

/**
 * @def XID_PI_WINDOW_SIZE_RX
 * @brief Parameter Identifier 8: Window Size Receive (k)
 * @details Notification parameter for maximum outstanding I-frames.
 *          Type B, PI=8, PL=1.
 *          Default: 4 (modulo 8), 32 (modulo 128).
 * @warning For selective reject, receiver must buffer k frames.
 */
#define XID_PI_WINDOW_SIZE_RX          8

/**
 * @def XID_PI_ACK_TIMER
 * @brief Parameter Identifier 9: Acknowledge Timer (T1)
 * @details Negotiates the "wait for acknowledgement" timer in milliseconds.
 *          Type B, PI=9, PL=2.
 *          Default: 3000 msec.
 * @note Negotiation uses the greater of the two offered values.
 */
#define XID_PI_ACK_TIMER               9

/**
 * @def XID_PI_RETRIES
 * @brief Parameter Identifier 10: Retries (N2)
 * @details Negotiates maximum retry count for unacknowledged frames.
 *          Type B, PI=10, PL=1.
 *          Default: 10 retries.
 * @note Negotiation uses the greater of the two offered values.
 */
#define XID_PI_RETRIES                 10

/**
 * @def XID_PI_RESP_DELAY_TIMER
 * @brief Parameter Identifier 11: Response Delay Timer (T2)
 * @details Negotiates delay before sending acknowledgements in milliseconds.
 *          Type B, PI=11, PL=2.
 *          Default: 500 msec (implementation specific).
 * @note Used to delay ACKs for piggybacking opportunities.
 */
#define XID_PI_RESP_DELAY_TIMER        11

/** @} *//* end of XID_Parameter_Identifiers */

/*============================================================================*/
/* Class of Procedures Bit Definitions - PI=2                                 */
/*============================================================================*/

/**
 * @defgroup Class_of_Procedures_Bits Class of Procedures Bit Masks
 * @brief Bit definitions for XID_PI_CLASS_OF_PROCEDURES parameter (PI=2)
 *
 * The Class of Procedures parameter is a 2-octet bit field (Type E) that
 * negotiates the duplex mode of operation. Per AX.25 v2.2:
 * - Bit 0 is always set to 1
 * - Bits 1-4 and 7-15 are always zero
 * - Either bit 5 (half-duplex) OR bit 6 (full-duplex) must be set, not both
 *
 * @see AX.25 v2.2 Section 6.7.2.1
 * @{
 */

/**
 * @def XID_COP_HALF_DUPLEX
 * @brief Half-duplex operation bit (bit 5)
 * @details Set to select half-duplex operation. Mutually exclusive with
 *          XID_COP_FULL_DUPLEX. Default mode for AX.25.
 * @note In half-duplex, stations alternate transmission and reception.
 */
#define XID_COP_HALF_DUPLEX  0x01

/**
 * @def XID_COP_FULL_DUPLEX
 * @brief Full-duplex operation bit (bit 6, value 0x02 in shifted position)
 * @details Set to select full-duplex operation. Both stations may
 *          transmit simultaneously.
 * @warning Requires hardware capable of full-duplex operation.
 */
#define XID_COP_FULL_DUPLEX  0x02

/**
 * @def XID_COP_NORMAL_RESP
 * @brief Normal response mode bit (bit 6, 0x40)
 * @details Defined in ISO 8885 for unbalanced operation. Not used in AX.25
 *          balanced mode but reserved for compatibility.
 */
#define XID_COP_NORMAL_RESP  0x40

/**
 * @def XID_COP_ASYNC_RESP
 * @brief Asynchronous response mode bit (bit 7, 0x80)
 * @details Defined in ISO 8885. Reserved for compatibility.
 */
#define XID_COP_ASYNC_RESP   0x80

/** @} *//* end of Class_of_Procedures_Bits */

/*============================================================================*/
/* HDLC Optional Functions Bit Definitions - PI=3                             */
/*============================================================================*/

/**
 * @defgroup HDLC_Optional_Functions_Byte0 HDLC Optional Functions Byte 0
 * @brief Bit masks for first octet of HDLC Optional Functions parameter
 *
 * Byte 0 contains flags for frame types and recovery mechanisms.
 * Bits 0, 3-6 are always zero in AX.25 v2.2.
 * Bits 7 is always one.
 * @{
 */

/**
 * @def XID_HDLC_RNR
 * @brief Receive Not Ready support bit (bit 0)
 * @details Always set to 1 in AX.25 implementations. Indicates support
 *          for RNR supervisory frames for flow control.
 */
#define XID_HDLC_RNR    0x01

/**
 * @def XID_HDLC_REJ
 * @brief Implicit Reject (REJ) support bit (bit 1)
 * @details Set to indicate support for REJ recovery (go-back-n).
 *          Mutually exclusive with SREJ in pure form, but both may be
 *          set for SREJ/REJ mode.
 * @see XID_HDLC_SREJ for selective reject
 */
#define XID_HDLC_REJ    0x02

/**
 * @def XID_HDLC_SREJ
 * @brief Selective Reject (SREJ) support bit (bit 2)
 * @details Set to indicate support for SREJ recovery (selective retrans).
 *          Preferred over REJ in AX.25 v2.2.
 * @note If both REJ and SREJ bits set, SREJ/REJ mode is selected.
 */
#define XID_HDLC_SREJ   0x04

/**
 * @def XID_HDLC_SABM
 * @brief Set Asynchronous Balanced Mode support bit (bit 3)
 * @details Always set to 1. Indicates support for SABM command (modulo 8).
 */
#define XID_HDLC_SABM   0x08

/**
 * @def XID_HDLC_SABME
 * @brief Set Asynchronous Balanced Mode Extended bit (bit 4)
 * @details Set to indicate support for SABME command (modulo 128).
 *          Requires XID_HDLC_MOD128 also set.
 */
#define XID_HDLC_SABME  0x10

/**
 * @def XID_HDLC_DM
 * @brief Disconnected Mode support bit (bit 5)
 * @details Always set to 1. Indicates support for DM response frames.
 */
#define XID_HDLC_DM     0x20

/**
 * @def XID_HDLC_DISC
 * @brief Disconnect support bit (bit 6)
 * @details Always set to 1. Indicates support for DISC command frames.
 */
#define XID_HDLC_DISC   0x40

/**
 * @def XID_HDLC_UA
 * @brief Unnumbered Acknowledge support bit (bit 7)
 * @details Always set to 1. Indicates support for UA response frames.
 */
#define XID_HDLC_UA     0x80

/** @} *//* end of HDLC_Optional_Functions_Byte0 */

/**
 * @defgroup HDLC_Optional_Functions_Byte1 HDLC Optional Functions Byte 1
 * @brief Bit masks for second octet of HDLC Optional Functions parameter
 *
 * Byte 1 contains flags for additional frame types and sequence numbering.
 * Bits 0, 1, 2, 4, 6, 8-15 are always zero in AX.25 v2.2.
 * Bits 3, 5, 7 are always one.
 * @{
 */

/**
 * @def XID_HDLC_FRMR
 * @brief Frame Reject support bit (byte 1, bit 0, value 0x01)
 * @details Always set to 1. Indicates support for FRMR error reporting.
 */
#define XID_HDLC_FRMR     0x01

/**
 * @def XID_HDLC_UI
 * @brief Unnumbered Information support bit (byte 1, bit 1, value 0x02)
 * @details Always set to 1. Indicates support for UI frames (connectionless).
 */
#define XID_HDLC_UI       0x02

/**
 * @def XID_HDLC_XID
 * @brief Exchange Identification support bit (byte 1, bit 2, value 0x04)
 * @details Always set to 1 in v2.2+. Indicates support for XID negotiation.
 */
#define XID_HDLC_XID      0x04

/**
 * @def XID_HDLC_TEST
 * @brief Test frame support bit (byte 1, bit 3, value 0x08)
 * @details Always set to 1. Indicates support for TEST diagnostic frames.
 */
#define XID_HDLC_TEST     0x08

/**
 * @def XID_HDLC_MOD8
 * @brief Modulo 8 sequence numbers bit (byte 1, bit 4, value 0x10)
 * @details Always set to 1. Indicates support for modulo-8 sequencing (3-bit).
 * @note Default and mandatory for all AX.25 implementations.
 */
#define XID_HDLC_MOD8     0x10

/**
 * @def XID_HDLC_MOD128
 * @brief Modulo 128 sequence numbers bit (byte 1, bit 5, value 0x20)
 * @details Set to indicate support for extended sequencing (7-bit, modulo 128).
 *          Allows window sizes up to 127 and improved throughput on
 *          high-latency links (satellite, etc.).
 * @warning Requires significantly more memory for buffering.
 */
#define XID_HDLC_MOD128   0x20

/** @} *//* end of HDLC_Optional_Functions_Byte1 */

/*============================================================================*/
/* Data Structures                                                            */
/*============================================================================*/

/**
 * @struct ax25_negotiated_params_t
 * @brief Negotiated operational parameters for AX.25 v2.2 connection
 *
 * This structure holds the complete set of parameters negotiated between
 * two AX.25 stations using XID frames. These parameters govern the
 * behavior of the Data-Link State Machine (DLSM) during the information
 * transfer phase.
 *
 * @section Parameter_Negotiation_Rules
 * - full_duplex: Logical AND of local and remote capabilities
 * - selective_reject: Logical AND (both must support)
 * - implicit_reject: Logical AND
 * - modulo128: Logical AND (both must support extended)
 * - ifield_length: Minimum of local and remote values
 * - window_size: Minimum of local and remote values
 * - ack_timer: Maximum of local and remote values (more conservative)
 * - retries: Minimum of local and remote values
 * - response_delay_timer: Maximum of local and remote values
 *
 * @see AX.25 v2.2 Section 6.3.2, Appendix C5
 */
typedef struct {
    /**
     * @brief Full-duplex operation enabled
     * @details true if both stations support full-duplex and negotiation
     *          resulted in full-duplex mode. false for half-duplex.
     */
    bool full_duplex;

    /**
     * @brief Selective Reject (SREJ) supported
     * @details true if both stations support SREJ recovery mechanism.
     *          Allows selective retransmission of single frames rather
     *          than go-back-n. More efficient on noisy channels.
     * @see AX.25 v2.2 Section 4.4.4
     */
    bool selective_reject;

    /**
     * @brief Implicit Reject (REJ) supported
     * @details true if both stations support REJ recovery mechanism.
     *          Standard go-back-n retransmission.
     * @note If both REJ and SREJ are true, SREJ/REJ mode is used.
     */
    bool implicit_reject;

    /**
     * @brief Modulo 128 sequence numbers enabled
     * @details true if extended sequencing (7-bit) is negotiated.
     *          Allows window sizes up to 127 vs 7 for modulo 8.
     *          Essential for high-speed or satellite links.
     * @warning Increases memory requirements for frame buffering.
     */
    bool modulo128;

    /**
     * @brief Maximum I-field length (N1) in octets
     * @details Maximum number of octets in the information field that
     *          the station can receive. Negotiated as minimum of both
     *          stations' capabilities.
     * @note Default is 256 octets (2048 bits). Maximum practical is
     *       often limited by hardware/software constraints.
     */
    uint16_t ifield_length;

    /**
     * @brief Window size (k) - maximum outstanding I-frames
     * @details Maximum number of unacknowledged I-frames that may be
     *          transmitted. Range: 1-7 (modulo 8) or 1-127 (modulo 128).
     *          Default: 4 (mod 8), 32 (mod 128).
     * @warning For SREJ operation, receiver must buffer k frames.
     */
    uint8_t window_size;

    /**
     * @brief Acknowledge timer T1 in milliseconds
     * @details Timer for retransmission of unacknowledged I-frames.
     *          Started when I-frame is transmitted. If expires before
     *          acknowledgment, frame is retransmitted.
     *          Default: 3000 ms.
     * @note Should be set based on round-trip delay of link.
     */
    uint16_t ack_timer;

    /**
     * @brief Maximum retry count (N2)
     * @details Maximum number of retransmission attempts for a frame
     *          before declaring link failure. Default: 10.
     * @note After N2 retries, link is reset or disconnected.
     */
    uint8_t retries;

    /**
     * @brief Response delay timer T2 in milliseconds
     * @details Delay before sending acknowledgment to allow piggybacking
     *          on outgoing I-frames. Started when frame received that
     *          requires acknowledgment.
     *          Default: 500 ms (implementation specific).
     */
    uint16_t response_delay_timer;
} ax25_negotiated_params_t;

/**
 * @enum ax25_mgmt_state_t
 * @brief Management Data-Link State Machine (MDL) states
 *
 * The MDL state machine manages XID parameter negotiation as specified
 * in AX.25 v2.2 Appendix C5. It operates independently of the main
 * Data-Link State Machine.
 *
 * @dot
 * digraph MDL_States {
 *   IDLE -> AWAITING_RESPONSE [label="XID cmd sent"];
 *   AWAITING_RESPONSE -> NEGOTIATED [label="XID resp rcvd"];
 *   AWAITING_RESPONSE -> IDLE [label="Timeout/Error"];
 *   NEGOTIATED -> IDLE [label="Link reset"];
 * }
 * @enddot
 */
typedef enum {
    /**
     * @brief Ready state - no negotiation in progress
     * @details Initial state. MDL is idle and ready to initiate or
     *          respond to negotiation. XID commands may be received
     *          and processed, triggering transition to NEGOTIATED.
     */
    AX25_MGMT_IDLE = 0,

    /**
     * @brief Negotiating state - awaiting XID response
     * @details Entered when local station sends XID command. Waiting
     *          for XID response from remote station. Timer TM201 active.
     * @see AX.25 v2.2 Appendix C5.2
     */
    AX25_MGMT_AWAITING_RESPONSE,

    /**
     * @brief Negotiated state - parameters agreed
     * @details XID exchange completed successfully. Agreed parameters
     *          are active and used by Data-Link State Machine. Link
     *          may proceed to information transfer phase.
     */
    AX25_MGMT_NEGOTIATED
} ax25_mgmt_state_t;

/**
 * @struct ax25_mgmt_context_t
 * @brief Management Data-Link State Machine context structure
 *
 * Complete context for the MDL state machine including current state,
 * all parameter sets (local, remote, agreed), retry management, and
 * peer identification.
 *
 * @section Usage
 * 1. Initialize with ax25_mgmt_init()
 * 2. Start negotiation with ax25_mgmt_start_negotiation()
 * 3. Process incoming XID with ax25_mgmt_process_xid()
 * 4. Call ax25_mgmt_tick() periodically for timeout handling
 */
typedef struct {
    /**
     * @brief Current MDL state machine state
     * @see ax25_mgmt_state_t
     */
    ax25_mgmt_state_t state;

    /**
     * @brief Local station's preferred parameters
     * @details Parameters offered to remote station in XID command.
     *          Set during initialization and typically constant.
     */
    ax25_negotiated_params_t local_params;

    /**
     * @brief Remote station's offered parameters
     * @details Parameters received from remote in XID command/response.
     *          Parsed and stored for negotiation logic.
     */
    ax25_negotiated_params_t remote_params;

    /**
     * @brief Agreed/negotiated parameters
     * @details Result of negotiation between local and remote params.
     *          These are the active parameters used for the connection.
     */
    ax25_negotiated_params_t agreed_params;

    /**
     * @brief Timeout tick counter for retry management
     * @details Timestamp (milliseconds) when current operation started.
     *          Used with ax25_mgmt_tick() to detect timeouts.
     */
    uint32_t timeout_tick;

    /**
     * @brief Current retry count
     * @details Number of XID command retransmissions attempted.
     *          Compared against NM201 (max retries, typically 3).
     */
    uint8_t retry_count;

    /**
     * @brief Peer station address
     * @details AX.25 address of the remote station being negotiated with.
     *          Set when negotiation initiated.
     */
    ax25_address_t peer;
} ax25_mgmt_context_t;

/*============================================================================*/
/* Function Prototypes                                                        */
/*============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize AX.25 management context
 *
 * Initializes the MDL context to default state (IDLE) and sets local
 * parameters to AX.25 v2.2 recommended defaults:
 * - Half-duplex operation
 * - Selective reject enabled (SREJ)
 * - Implicit reject enabled (REJ)
 * - Modulo 128 supported
 * - I-field length: 256 octets
 * - Window size: 7 (modulo-8 default)
 * - T1 timer: 3000 ms
 * - Retries: 10
 * - T2 timer: 500 ms
 *
 * @param[out] ctx Pointer to management context to initialize
 *
 * @return uint8_t Error code
 * @retval 0 Success - context initialized
 * @retval 1 Error - ctx is NULL
 *
 * @note Must be called before any other MDL functions
 * @warning Context is zeroed; any previous state is lost
 *
 * @see AX.25 v2.2 Appendix C5.1 for default values
 */
uint8_t ax25_mgmt_init(ax25_mgmt_context_t *ctx);

/**
 * @brief Start parameter negotiation by sending XID command
 *
 * Initiates XID parameter negotiation with remote station. Builds and
 * transmits XID command frame containing local parameters. Transitions
 * state to AX25_MGMT_AWAITING_RESPONSE.
 *
 * @param[in,out] ctx Management context
 * @param[in] dest Destination station address (remote)
 * @param[in] src Source station address (local)
 * @param[in] transmit Callback function for frame transmission
 *
 * @return uint8_t Error code
 * @retval 0 Success - XID command transmitted
 * @retval 1 Error - Invalid parameter (NULL pointer)
 * @retval 2 Error - State not IDLE (negotiation already in progress)
 *
 * @section Frame_Construction
 * The XID command frame is constructed per AX.25 v2.2 Section 4.3.3.7:
 * - Control field: 0xAF (XID command, P=1)
 * - Information field: FI=0x82, GI=0x80, followed by PI/PL/PV parameters
 *
 * Parameters included:
 * - PI=2: Class of Procedures (duplex mode)
 * - PI=3: HDLC Optional Functions (REJ/SREJ, modulo)
 * - PI=6: I-Field Length Rx (N1)
 * - PI=8: Window Size Rx (k)
 * - PI=9: Ack Timer (T1)
 * - PI=10: Retries (N2)
 * - PI=11: Response Delay Timer (T2)
 *
 * @note P/F bit is set (P=1) to request immediate response
 * @note TM201 timer should be started after transmission
 *
 * @see ax25_mgmt_process_xid() for response handling
 */
uint8_t ax25_mgmt_start_negotiation(ax25_mgmt_context_t *ctx, ax25_address_t *dest, ax25_address_t *src, void (*transmit)(uint8_t*, size_t));

/**
 * @brief Process received XID frame (command or response)
 *
 * Parses incoming XID frame, extracts parameters, performs negotiation
 * logic, and sends XID response if command frame received.
 *
 * @param[in,out] ctx Management context
 * @param[in] xid Parsed XID frame structure
 * @param[in] transmit Callback function for frame transmission (response)
 *
 * @return uint8_t Error code
 * @retval 0 Success - XID processed, negotiation complete
 * @retval 1 Error - Invalid parameter (NULL pointer)
 *
 * @section Processing_Steps
 * 1. Parse all PI/PL/PV parameters from XID information field
 * 2. Populate remote_params with received values
 * 3. Apply negotiation rules to determine agreed_params:
 *    - Boolean params: Logical AND of local and remote
 *    - Lengths/Windows: Minimum value
 *    - Timers: Maximum value (conservative)
 * 4. If XID is command (C=1), build and send XID response
 * 5. Set state to AX25_MGMT_NEGOTIATED
 *
 * @section Parameter_Parsing
 * Unknown PI values are silently ignored per AX.25 v2.2 specification.
 * Malformed parameters (wrong length) are skipped.
 * Missing parameters use default values.
 *
 * @note If remote station is v2.0 or earlier, it will respond with FRMR
 *       instead of XID. This must be handled by the caller.
 *
 * @see AX.25 v2.2 Section 6.3.2, Appendix C5.3
 */
uint8_t ax25_mgmt_process_xid(ax25_mgmt_context_t *ctx, ax25_exchange_identification_frame_t *xid, void (*transmit)(uint8_t*, size_t));

/**
 * @brief Timer tick handler for MDL timeout management
 *
 * Must be called periodically (e.g., every 100ms) to manage XID command
 * retry timeouts. Handles TM201 timer expiration.
 *
 * @param[in,out] ctx Management context
 * @param[in] current_tick Current system tick count (milliseconds)
 *
 * @return void
 *
 * @section Timeout_Handling
 * If state is AX25_MGMT_AWAITING_RESPONSE and timeout exceeds 3000ms:
 * - Increment retry count
 * - If retry_count >= 3: Set state to AX25_MGMT_IDLE (failure)
 * - Else: Retransmit XID command, reset timeout_tick
 *
 * @note Actual retransmission logic may be implemented by checking
 *       state transition or via callback
 * @note Timer value (3000ms) should match T1 or be configurable
 *
 * @warning Must be called regularly to ensure timely timeout detection
 *
 * @see AX.25 v2.2 Appendix C5.2 (TM201 timer)
 */
void ax25_mgmt_tick(ax25_mgmt_context_t *ctx, uint32_t current_tick);

#endif /* AX25_MGMT_H_ */
