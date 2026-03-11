/**
 * @file kiss.h
 * @brief AX.25 v2.2 Protocol Library - KISS TNC Interface Protocol
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @section Overview
 * Implements the KISS (Keep It Simple, Stupid) TNC host-to-TNC serial
 * interface protocol as originally defined by Mike Chepponis K3MC and
 * Phil Karn KA9Q (ARRL 6th Computer Networking Conference, 1987).
 *
 * KISS moves AX.25 framing, protocol handling and higher-layer functions
 * out of the TNC and into the host computer, leaving the TNC responsible
 * only for physical layer operation: PTT control, p-persistence CSMA,
 * HDLC encoding/decoding and baud-rate modem operations.
 *
 * This header additionally covers the following KISS variants and
 * extensions discovered in widespread amateur radio deployment:
 *
 * - Standard KISS (Chepponis/Karn 1987)
 * - SMACK - Stuttgart Modified Amateurradio-CRC-KISS (DL5UE/DK5SG, 1991)
 *   Adds a 16-bit CRC-CCITT checksum to data frames for serial link integrity.
 *   Uses the MSB of the type byte as a CRC-present flag; backward-compatible
 *   because standard KISS TNCs discard frames with unknown high-nibble ports.
 * - G8BPQ KISS - Single-byte XOR checksum variant used in G8BPQ KISS ROMs.
 * - FlexNet/BayCom KISS - 16-bit CRC variant used by FlexNet nodes and
 *   BayCom Mailbox software.
 * - MKISS - Multi-port KISS splitter (Linux ax25-tools mkiss utility).
 *   Multiplexes up to 16 logical ports over one physical serial device.
 *   Supports optional polling mode (G8BPQ polled KISS) and hardware
 *   handshaking.
 * - TCP-KISS (AGWPE/Dire Wolf style) - KISS framing over TCP/IP stream
 *   sockets (port 8001 is the de facto standard for AGWPE).
 *
 * @section Framing
 * Each KISS frame is delimited by FEND (0xC0) characters:
 *
 *   FEND [type_byte] [escaped_data...] FEND
 *
 * Special character escaping:
 * - FEND (0xC0) in data  ->  FESC (0xDB) TFEND (0xDC)
 * - FESC (0xDB) in data  ->  FESC (0xDB) TFESC (0xDD)
 *
 * Two consecutive FESC characters are a protocol violation and indicate an
 * aborted transmission; any data before the following FEND is discarded.
 *
 * @section Type_Byte
 * The first byte of each frame is the type indicator:
 *   bits [7:4] = port number  (0-15; port 0xF reserved)
 *   bits [3:0] = command code (see KISS_CMD_* constants)
 *
 * For SMACK frames, bit 7 of the type byte is set (0x80) instead of the
 * port number, making the effective port 0 and signalling CRC presence.
 *
 * @section Commands
 *   0x00  DATA        - Raw AX.25 frame data follows
 *   0x01  TXDELAY     - Next byte: TX key-up delay, 10 ms units (default 50 = 500 ms)
 *   0x02  PERSISTENCE - Next byte: p-persistence P value 0-255 (default 63, p≈0.25)
 *   0x03  SLOTTIME    - Next byte: slot interval, 10 ms units (default 10 = 100 ms)
 *   0x04  TXTAIL      - Next byte: TX tail after FCS, 10 ms units (obsolete, default 0)
 *   0x05  FULLDUPLEX  - Next byte: 0 = half-duplex, non-zero = full-duplex (default 0)
 *   0x06  SETHARDWARE - Remaining bytes: TNC hardware-specific parameter
 *   0xFF  RETURN      - Exit KISS mode, return to higher-level TNC control
 *
 * @section Multi_Port
 * The high nibble of the type byte selects one of up to 16 logical ports.
 * Port 0xF is reserved. Single-port TNCs always use port 0.
 *
 * @section CSMA_Parameters
 * p-persistence CSMA (Carrier Sense Multiple Access):
 * - When the channel is idle, the TNC generates a uniform random number R in [0,255].
 * - If R <= P it transmits immediately; otherwise it waits one slot time and repeats.
 * - Low P causes heavy back-off (polite but high latency); high P risks collisions.
 * - Default P=63 gives p=63/256≈0.25, suitable for moderately loaded channels.
 *
 * @section Standards_References
 * - KISS TNC specification: http://www.ka9q.net/papers/kiss.html
 * - KISS at ax25.net:       https://www.ax25.net/kiss.aspx
 * - AX.25 v2.2:             https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * - SMACK protocol:         http://www.symek.com/g/smack.html
 * - MKISS man page:         https://manpages.debian.org/testing/ax25-tools/mkiss.8.en.html
 * - 6PACK (Linux kernel):   https://docs.kernel.org/networking/6pack.html
 * - M17 KISS extensions:    https://m17-protocol-specification.readthedocs.io/en/latest/kiss_protocol.html
 * - KISS (Wikipedia):       https://en.wikipedia.org/wiki/KISS_(TNC)
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef AX25_KISS_H_
#define AX25_KISS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*============================================================================*/
/* KISS Special Character Codes (SLIP-derived framing)                        */
/*============================================================================*/

/**
 * @defgroup KISSSpecialChars KISS Frame Delimiter and Escape Characters
 * @brief Special byte values for KISS frame framing and escaping (SLIP-based)
 *
 * These four special byte values form the backbone of KISS framing.
 * They are derived from the Serial Line Internet Protocol (SLIP, RFC 1055).
 *
 * @section Escaping_Rules
 * If FEND or FESC appear in the user data payload they MUST be replaced with
 * a two-byte escape sequence before transmission:
 * - 0xC0 (FEND) -> 0xDB 0xDC (FESC TFEND)
 * - 0xDB (FESC) -> 0xDB 0xDD (FESC TFESC)
 * The receiver reverses the substitution after stripping the KISS framing.
 *
 * A double FESC (0xDB 0xDB) in the input stream is a protocol violation
 * indicating an aborted frame; the receiver must discard all data up to
 * and including the following FEND.
 */
#define KISS_FEND   0xC0u  /**< Frame End delimiter (also used as Frame Start) */
#define KISS_FESC   0xDBu  /**< Frame Escape prefix for data-byte substitution  */
#define KISS_TFEND  0xDCu  /**< Transposed FEND: follows FESC to represent 0xC0 */
#define KISS_TFESC  0xDDu  /**< Transposed FESC: follows FESC to represent 0xDB */

/*============================================================================*/
/* KISS Command Codes (low nibble of type indicator byte)                     */
/*============================================================================*/

/**
 * @defgroup KISSCommandCodes Standard KISS Command Codes
 * @brief Command codes occupying the low nibble of the KISS type indicator byte
 *
 * Command codes 0x07-0x0E are reserved for future use and must be silently
 * ignored by any conforming KISS implementation.  Only the "Data frame" code
 * (0x00) should be sent from TNC to host; all other commands flow from
 * host to TNC.
 *
 * @section Persistence_Parameters
 * The TxDelay, Persistence, SlotTime and TxTail parameters control the TNC's
 * transmitter timing and CSMA algorithm.  They are sent as single-byte values
 * in 10 ms units (TxDelay, SlotTime, TxTail) or as a raw 0-255 probability
 * value (Persistence).  Factory defaults match the original specification.
 */
#define KISS_CMD_DATA         0x00u  /**< Data frame: raw AX.25 frame payload follows */
#define KISS_CMD_TXDELAY      0x01u  /**< TxDelay: transmitter key-up delay (10 ms units, default 50) */
#define KISS_CMD_PERSISTENCE  0x02u  /**< Persistence: p-persistence P value 0-255 (default 63) */
#define KISS_CMD_SLOTTIME     0x03u  /**< SlotTime: CSMA slot interval (10 ms units, default 10) */
#define KISS_CMD_TXTAIL       0x04u  /**< TxTail: TX hold after FCS (10 ms units, obsolete, default 0) */
#define KISS_CMD_FULLDUPLEX   0x05u  /**< FullDuplex: 0=half-duplex, non-zero=full-duplex (default 0) */
#define KISS_CMD_SETHARDWARE  0x06u  /**< SetHardware: TNC-specific parameter data */
#define KISS_CMD_RETURN       0x0Fu  /**< Return: exit KISS mode (sent as full-byte 0xFF) */

/*============================================================================*/
/* KISS Type Indicator Byte Construction / Deconstruction                     */
/*============================================================================*/

/**
 * @defgroup KISSTypeByte Type Indicator Byte Macros
 * @brief Macros for building and parsing the KISS type indicator byte
 *
 * The type indicator byte has the following structure:
 * @verbatim
 *   bits [7:4]  port number  (0-15, 0xF reserved)
 *   bits [3:0]  command code (see KISS_CMD_*)
 * @endverbatim
 *
 * Exception: the Return command uses the fixed full-byte value 0xFF.
 * Exception: SMACK data frames use type byte 0x80 (MSB set, port 0, DATA).
 */
/** Build type indicator byte from port (0-15) and command (0-15) */
#define KISS_TYPE_BYTE(port, cmd)      ((uint8_t)(((uint8_t)(port) << 4u) | ((uint8_t)(cmd) & 0x0Fu)))
/** Extract port number from type byte */
#define KISS_PORT(type_byte)           ((uint8_t)(((uint8_t)(type_byte) >> 4u) & 0x0Fu))
/** Extract command code from type byte */
#define KISS_CMD(type_byte)            ((uint8_t)((uint8_t)(type_byte) & 0x0Fu))
/** Full-byte value used exclusively for the Return command */
#define KISS_RETURN_TYPE_BYTE          0xFFu
/** SMACK CRC-flag mask: bit 7 of type byte signals CRC presence */
#define KISS_SMACK_CRC_FLAG            0x80u
/** True if type byte carries a SMACK CRC checksum */
#define KISS_IS_SMACK_FRAME(type_byte) (((uint8_t)(type_byte) & KISS_SMACK_CRC_FLAG) != 0u)

/*============================================================================*/
/* Capacity and Size Limits                                                   */
/*============================================================================*/

/**
 * @defgroup KISSLimits Capacity and Buffer Size Constants
 * @brief Maximum frame sizes, port counts and derived buffer dimensions
 *
 * KISS_MAX_FRAME_SIZE controls the raw (pre-escape) maximum AX.25 frame
 * payload.  The original KISS specification states that implementations
 * "should allow" at least 1024-byte packets; the value here is conservative
 * for embedded targets.  Increase it for host implementations.
 *
 * KISS_TX_BUF_SIZE is the worst-case encoded output size:
 * - Opening FEND (1) + type byte possibly escaped (2) +
 *   each payload byte worst-case escaped (2 per byte) + closing FEND (1)
 *   = 2 + 2 + KISS_MAX_FRAME_SIZE * 2 bytes
 *
 * KISS_SMACK_CRC_SIZE is the two extra trailer bytes added by SMACK.
 * KISS_G8BPQ_CRC_SIZE is the one extra trailer byte added by G8BPQ KISS.
 */
#define KISS_MAX_PORTS         16u    /**< Maximum logical ports per multi-port TNC (port 0xF reserved) */
#define KISS_MAX_FRAME_SIZE    128u   /**< Maximum raw AX.25 payload in bytes (increase for host use) */

// SMACK appends 2 CRC bytes each potentially SLIP-escaped to 2 bytes = 4 extra
#define KISS_TX_BUF_SIZE       (KISS_MAX_FRAME_SIZE * 2u + 8u) /**< Worst-case escaped+framed TX buffer including SMACK CRC */
#define KISS_SMACK_CRC_SIZE    2u     /**< Bytes appended by SMACK: 16-bit CRC (LSB first) */
#define KISS_G8BPQ_CRC_SIZE    1u     /**< Bytes appended by G8BPQ KISS: 8-bit XOR checksum */
#define KISS_HARDWARE_BUF_SIZE 64u    /**< Maximum SetHardware parameter byte storage */

/*============================================================================*/
/* Default KISS TNC Parameter Values                                          */
/*============================================================================*/

/**
 * @defgroup KISSDefaults Default TNC Parameter Values
 * @brief Factory-default values for per-port TNC timing and CSMA parameters
 *
 * All timing values are in 10 ms units as defined by the KISS specification.
 * These defaults may be overridden via ax25_kiss_set_port_params().
 */
#define KISS_DEFAULT_TXDELAY       50u    /**< TX key-up delay: 50 * 10ms = 500 ms */
#define KISS_DEFAULT_PERSISTENCE   63u    /**< p-persistence P: 63/256 ≈ 0.25 */
#define KISS_DEFAULT_SLOTTIME      10u    /**< CSMA slot time: 10 * 10ms = 100 ms */
#define KISS_DEFAULT_TXTAIL        0u     /**< TX tail (obsolete): 0 ms */
#define KISS_DEFAULT_FULLDUPLEX    false  /**< Half-duplex operation */
#define KISS_DEFAULT_POLL_INTERVAL 0u     /**< Polling disabled by default (units: 100 ms) */

/*============================================================================*/
/* Return Codes                                                               */
/*============================================================================*/

/**
 * @defgroup KISSReturnCodes KISS API Return Codes
 * @brief Status codes returned by all public API functions
 *
 * Functions return KISS_OK (0) on success. Non-zero values indicate error
 * conditions. Callers should always check return values from transmit
 * functions; receive callbacks run synchronously during ax25_kiss_receive_byte()
 * and errors within callbacks are not propagated back to the caller.
 */
#define KISS_OK                0u   /**< Operation succeeded */
#define KISS_ERR_NULL          1u   /**< NULL pointer passed as required argument */
#define KISS_ERR_PORT          2u   /**< Port number out of range (0-14) or reserved (15) */
#define KISS_ERR_FRAME_SIZE    3u   /**< Frame length exceeds KISS_MAX_FRAME_SIZE */
#define KISS_ERR_NO_SERIAL     4u   /**< Context has no serial_write callback set */
#define KISS_ERR_NOT_KISSMOD   5u   /**< Context is not currently in KISS mode */
#define KISS_ERR_BAD_CHECKSUM  6u   /**< SMACK/G8BPQ/FlexNet CRC verification failed */
#define KISS_ERR_BAD_ESCAPE    7u   /**< Invalid escape sequence in received frame */
#define KISS_ERR_ABORTED_FRAME 8u   /**< Double-FESC abort sequence detected */
#define KISS_ERR_CMD_INVALID   9u   /**< Command code not valid for this operation */
#define KISS_ERR_MODE_CONFLICT 10u  /**< Requested operation conflicts with current mode */
#define KISS_ERR_OVERFLOW      11u  /**< Receive buffer overflowed; frame dropped */

/*============================================================================*/
/* KISS Variant / Checksum Mode Enumeration                                   */
/*============================================================================*/

/**
 * @brief KISS protocol variant / checksum mode selection
 *
 * Selects which checksum scheme, if any, is applied to transmitted data
 * frames and expected on received data frames.  The variant affects only
 * data frames (CMD=0); parameter command frames are never checksummed.
 *
 * @section Variant_Compatibility
 * - KISS_VARIANT_STANDARD:   No checksum. Interoperates with all KISS TNCs.
 * - KISS_VARIANT_SMACK:      16-bit CRC-CCITT trailer. Sets bit 7 of type byte
 *                             (0x80 for port 0). KISS TNCs discard unknown command
 *                             codes, making SMACK backward-compatible.
 * - KISS_VARIANT_G8BPQ:      1-byte XOR checksum trailer. Used by G8BPQ KISS ROMs.
 *                             Not interoperable with standard KISS without negotiation.
 * - KISS_VARIANT_FLEXNET:    16-bit CRC-CCITT, same algorithm as SMACK but different
 *                             framing conventions used by FlexNet Node and BayCom
 *                             Mailbox software.
 * - KISS_VARIANT_AUTO:       Auto-detect mode. Starts as standard KISS, switches to
 *                             SMACK once a frame with the CRC flag is received (per the
 *                             SMACK negotiation procedure). Cannot be reversed without
 *                             a context reset.
 */
typedef enum {
    KISS_VARIANT_STANDARD = 0, /**< Standard KISS, no checksum (default) */
    KISS_VARIANT_SMACK, /**< SMACK: 16-bit CRC-CCITT, MSB type-byte flag */
    KISS_VARIANT_G8BPQ, /**< G8BPQ: 8-bit XOR checksum trailer */
    KISS_VARIANT_FLEXNET, /**< FlexNet/BayCom: 16-bit CRC-CCITT trailer */
    KISS_VARIANT_AUTO, /**< Auto-detect: starts KISS, auto-upgrades to SMACK */
} ax25_kiss_variant_t;

/*============================================================================*/
/* KISS Transport Type Enumeration                                            */
/*============================================================================*/

/**
 * @brief Underlying transport layer carrying the KISS byte stream
 *
 * KISS was designed for asynchronous RS-232 serial links, but its simple
 * framing is equally useful over TCP streams (AGWPE, Dire Wolf, kissnetd)
 * and pipe/pty pairs (mkiss virtual TNCs, inter-process communication).
 *
 * @section TCP_KISS
 * The de facto TCP-KISS standard (AGWPE compatible) uses port 8001.
 * Dire Wolf also supports TCP KISS on a configurable port.  The framing
 * is identical to serial KISS; only the transport mechanism differs.
 * The serial_write callback is repurposed to write to the TCP socket in
 * this mode; the caller is responsible for socket lifecycle management.
 */
typedef enum {
    KISS_TRANSPORT_SERIAL = 0, /**< RS-232 / UART asynchronous serial link (default) */
    KISS_TRANSPORT_TCP, /**< TCP stream socket (AGWPE, Dire Wolf, kissnetd) */
    KISS_TRANSPORT_PIPE, /**< UNIX pipe or pseudo-tty (mkiss, inter-process) */
} ax25_kiss_transport_t;

/*============================================================================*/
/* Receive State Machine States                                               */
/*============================================================================*/

/**
 * @brief Receive byte-stream state machine states
 *
 * The state machine processes one byte at a time as delivered by the serial
 * port or TCP stream.  It assembles complete frames, handles escape sequences,
 * detects SMACK CRC presence, and invokes the appropriate callbacks.
 *
 * @section State_Transitions
 * @verbatim
 *  KISS_RX_IDLE
 *      | FEND received
 *      v
 *  KISS_RX_IN_FRAME  <--+
 *      | FESC received   |
 *      v                 |
 *  KISS_RX_ESCAPED  -----+  (after consuming one byte)
 *      | FEND while !rx_got_type  -> if port-12 DATA (0xC0) set type, stay
 *      | FEND while rx_got_type   -> dispatch frame, reset, stay IN_FRAME
 *      | FEND FEND (double)       -> KISS_RX_ABORT (empty inter-frame gap)
 *      v
 *  KISS_RX_ABORT
 *      | FEND received
 *      v
 *  KISS_RX_IDLE / KISS_RX_IN_FRAME
 * @endverbatim
 */
typedef enum {
    KISS_RX_IDLE, /**< Waiting for opening FEND character */
    KISS_RX_IN_FRAME, /**< Receiving frame bytes (type byte and data payload) */
    KISS_RX_ESCAPED, /**< Received FESC; next byte is the escaped data byte */
    KISS_RX_ABORT, /**< Double-FESC abort detected; discarding until next FEND */
} ax25_kiss_rx_state_t;

/*============================================================================*/
/* Per-Port TNC Parameter Set                                                 */
/*============================================================================*/

/**
 * @brief Per-port TNC operating parameters
 *
 * Each logical port (0-14) on a multi-port TNC has an independent set of
 * timing and CSMA parameters.  This structure mirrors the values last sent
 * to the TNC via SetXxx commands or received from the TNC when it reflects
 * parameter commands back.
 *
 * @section TX_Timing_Overview
 * The TNC transmitter timing sequence is:
 * 1. Channel becomes idle; CSMA p-persistence test starts.
 * 2. p-persistence random access: wait up to TxDelay ms before keying PTT.
 * 3. PTT asserted; TxDelay 10ms-unit count transmits preamble flags.
 * 4. HDLC frame body transmitted.
 * 5. FCS transmitted.
 * 6. TxTail (obsolete, should be 0) additional flags transmitted.
 * 7. PTT released.
 *
 * @section Hardware_Parameter
 * The hardware[] array stores raw bytes for the SetHardware command, which
 * is TNC-specific.  Common uses include setting modem type, DCD squelch,
 * or FX.25 Forward Error Correction mode on supporting firmware.
 */
typedef struct {
    uint8_t txdelay; /**< TX key-up delay in 10 ms units (default 50 = 500 ms) */
    uint8_t persistence; /**< p-persistence P value 0-255 (default 63 ≈ p=0.25)   */
    uint8_t slottime; /**< CSMA slot interval in 10 ms units (default 10 = 100 ms) */
    uint8_t txtail; /**< TX tail after FCS in 10 ms units (obsolete, default 0) */
    bool full_duplex; /**< false = half-duplex (default), true = full-duplex */
    uint8_t hardware[KISS_HARDWARE_BUF_SIZE]; /**< SetHardware raw parameter bytes (TNC-specific) */
    uint8_t hardware_len; /**< Number of valid bytes in hardware[] */
} ax25_kiss_port_params_t;

/*============================================================================*/
/* Statistics Counters                                                        */
/*============================================================================*/

/**
 * @brief Per-context KISS frame statistics counters
 *
 * Maintained by the receive state machine and transmit functions.
 * Useful for diagnostics, monitoring channel load, and detecting
 * serial link integrity problems (bad_checksum increments).
 *
 * @section Counter_Semantics
 * - rx_frames:       Complete, successfully dispatched DATA frames received.
 * - tx_frames:       Complete DATA frames sent via ax25_kiss_send_frame().
 * - rx_cmd_frames:   Non-data command frames received from TNC.
 * - tx_cmd_frames:   Non-data command frames sent to TNC.
 * - rx_dropped:      Frames dropped (buffer overflow, bad checksum, abort).
 * - rx_bad_checksum: SMACK/G8BPQ/FlexNet CRC mismatches detected.
 * - rx_aborted:      Frames aborted by double-FESC protocol violation.
 * - rx_overflows:    Frames where the receive buffer was exhausted.
 * - rx_bytes:        Total raw bytes processed by receive state machine.
 * - tx_bytes:        Total encoded bytes written via serial_write callback.
 */
typedef struct {
    uint32_t rx_frames; /**< DATA frames successfully received and dispatched */
    uint32_t tx_frames; /**< DATA frames transmitted */
    uint32_t rx_cmd_frames; /**< Non-DATA command frames received */
    uint32_t tx_cmd_frames; /**< Non-DATA command frames transmitted */
    uint32_t rx_dropped; /**< Frames discarded for any reason */
    uint32_t rx_bad_checksum; /**< Frames with SMACK/G8BPQ/FlexNet CRC mismatch */
    uint32_t rx_aborted; /**< Frames aborted by double-FESC violation */
    uint32_t rx_overflows; /**< Frames dropped due to rx_buf overflow */
    uint32_t rx_bytes; /**< Total bytes consumed by receive state machine */
    uint32_t tx_bytes; /**< Total bytes emitted via serial_write callback */
} ax25_kiss_stats_t;

/*============================================================================*/
/* Forward Declaration for Callback Types                                     */
/*============================================================================*/

/** Forward declaration of main context structure for callback signatures */
typedef struct ax25_kiss_ctx ax25_kiss_ctx_t;

/*============================================================================*/
/* Callback Function Type Definitions                                         */
/*============================================================================*/

/**
 * @defgroup KISSCallbacks Callback Function Types
 * @brief Callback signatures for KISS frame events and I/O integration
 *
 * All callbacks receive the context pointer and the caller-supplied
 * user_data opaque pointer, enabling clean integration into larger systems
 * without requiring global state.
 *
 * Callbacks are invoked synchronously from within ax25_kiss_receive_byte()
 * or ax25_kiss_receive_bytes().  Implementations must not call back into
 * ax25_kiss_send_frame() from within on_frame unless the underlying serial
 * write is non-blocking (e.g., buffered I/O).
 */

/**
 * @brief Data frame receive callback (CMD=0x00)
 *
 * Invoked when a complete DATA frame has been reassembled from the byte
 * stream.  The frame buffer is owned by the context and is valid only for
 * the duration of the callback; callers must copy if they need to retain it.
 *
 * @param ctx       KISS context that received the frame
 * @param port      Logical port number (0-14) from the type indicator byte
 * @param frame     Pointer to raw AX.25 frame bytes (no KISS framing, no FCS)
 * @param len       Number of bytes in frame
 * @param user_data Opaque pointer from ctx->user_data
 */
typedef void (*ax25_kiss_frame_cb_t)(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t *frame, size_t len, void *user_data);

/**
 * @brief SetHardware command callback (CMD=0x06)
 *
 * Invoked when a SetHardware command frame is received.  The data buffer is
 * owned by the context port parameter store and remains valid after the call.
 *
 * @param ctx       KISS context
 * @param port      Logical port (0-14)
 * @param data      Hardware parameter bytes following the type byte
 * @param len       Number of hardware parameter bytes
 * @param user_data Opaque pointer from ctx->user_data
 */
typedef void (*ax25_kiss_hw_cb_t)(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t *data, size_t len, void *user_data);

/**
 * @brief Return command callback (type byte 0xFF)
 *
 * Invoked when the Return/Exit-KISS command is received.  The implementation
 * should switch the TNC hardware back to normal (non-KISS) host-mode operation.
 * ctx->kiss_mode is already set to false when this callback is invoked.
 *
 * @param ctx       KISS context
 * @param user_data Opaque pointer from ctx->user_data
 */
typedef void (*ax25_kiss_return_cb_t)(ax25_kiss_ctx_t *ctx, void *user_data);

/**
 * @brief Serial/stream output callback
 *
 * Called by all transmit functions to push encoded KISS frame bytes to the
 * TNC.  The implementation MUST transmit all len bytes atomically from the
 * TNC's perspective; partial writes may result in malformed KISS frames.
 * For TCP transport, this callback writes to the connected socket.
 *
 * @param data      Pointer to bytes to transmit
 * @param len       Number of bytes to transmit
 * @param user_data Opaque pointer from ctx->user_data
 */
typedef void (*ax25_kiss_serial_write_cb_t)(uint8_t *data, size_t len, void *user_data);

/**
 * @brief SMACK / G8BPQ / FlexNet CRC error notification callback
 *
 * Called when a received data frame fails its checksum verification.
 * The raw frame (including the bad checksum bytes) is provided for logging
 * or diagnostic purposes.  The frame is NOT dispatched to on_frame.
 *
 * @param ctx       KISS context
 * @param port      Logical port from type byte
 * @param frame     Raw frame payload including bad checksum trailer bytes
 * @param len       Length of frame including checksum trailer
 * @param user_data Opaque pointer from ctx->user_data
 */
typedef void (*ax25_kiss_crc_error_cb_t)(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t *frame, size_t len, void *user_data);

/**
 * @brief Diagnostic / logging callback
 *
 * Optional callback for diagnostic messages generated internally by the
 * KISS layer (e.g., buffer overflow warnings, abort detections, mode
 * transitions).  The message string is null-terminated and owned by the
 * caller; it must be copied if it needs to outlive the callback invocation.
 *
 * @param ctx       KISS context
 * @param level     Severity: 0=debug, 1=info, 2=warning, 3=error
 * @param msg       Null-terminated diagnostic message string
 * @param user_data Opaque pointer from ctx->user_data
 */
typedef void (*ax25_kiss_log_cb_t)(ax25_kiss_ctx_t *ctx, uint8_t level, const char *msg, void *user_data);

/*============================================================================*/
/* Main KISS Context Structure                                                */
/*============================================================================*/

/**
 * @brief Main KISS protocol context
 *
 * Encapsulates the complete state for one KISS link (serial port, TCP
 * connection, or pipe).  For multi-port TNCs all 16 logical port parameter
 * sets are stored here; MKISS splits one physical context into multiple
 * virtual contexts via the mkiss multiplexer (not part of this structure).
 *
 * @section Thread_Safety
 * The context is NOT thread-safe.  If the receive state machine runs in
 * one thread while transmit functions are called from another, the caller
 * must provide external mutual exclusion.
 *
 * @section Initialization
 * Always initialize via ax25_kiss_init() before any other operation.
 * After init, set serial_write (mandatory for TX) and any desired callback
 * pointers, then call ax25_kiss_enter() to begin KISS operation.
 *
 * @section Variant_Auto_Detection
 * When variant == KISS_VARIANT_AUTO the context starts in standard KISS
 * mode.  On receipt of the first frame with bit 7 of the type byte set
 * (SMACK frame) it automatically switches to KISS_VARIANT_SMACK and begins
 * appending CRC16 to all subsequent transmitted data frames.
 */
struct ax25_kiss_ctx {
    /* --- Per-port parameter mirrors (index = port 0-15) --- */
    ax25_kiss_port_params_t ports[KISS_MAX_PORTS]; /**< Per-port TNC parameter sets */

    /* --- Receive state machine --- */
    ax25_kiss_rx_state_t rx_state; /**< Current byte-stream parser state */
    uint8_t rx_buf[KISS_MAX_FRAME_SIZE]; /**< Frame accumulation buffer */
    size_t rx_len; /**< Bytes written to rx_buf so far */
    bool rx_got_type; /**< true once the type indicator byte has been received */
    uint8_t rx_type; /**< Saved type indicator byte for current frame */
    bool rx_at_frame_start; /**< true when the very next byte after FEND is expected
     *   to be the type byte of a new frame; used to
     *   disambiguate port-12 DATA frames (type=0xC0) from
     *   inter-frame gap FENDs */
    bool rx_double_fesc; /**< true if last two bytes were FESC FESC (abort sequence) */

    /* --- Protocol variant and checksum state --- */
    ax25_kiss_variant_t variant; /**< Active KISS variant / checksum mode */
    bool smack_active; /**< true when SMACK mode has been engaged (auto or explicit) */

    /* --- Operational mode flags --- */
    bool kiss_mode; /**< true = context is in KISS mode; false = normal TNC mode */
    ax25_kiss_transport_t transport; /**< Underlying transport type (serial / TCP / pipe) */

    /* --- Polling mode (G8BPQ polled KISS / MKISS) --- */
    bool poll_mode; /**< true = polled mode active (G8BPQ KISS ROMs) */
    uint8_t poll_interval; /**< Poll interval in 100 ms units (0 = polling disabled) */

    /* --- Hardware flow control flag --- */
    bool hw_flowctrl; /**< true = hardware RTS/CTS flow control in use
     *   (non-standard; most KISS TNCs do NOT support this) */

    /* --- Callbacks --- */
    ax25_kiss_frame_cb_t on_frame; /**< Called on successful DATA frame receipt */
    ax25_kiss_hw_cb_t on_hardware; /**< Called on SetHardware command receipt */
    ax25_kiss_return_cb_t on_return; /**< Called on Return command receipt */
    ax25_kiss_serial_write_cb_t serial_write; /**< Output bytes to serial port / TCP / pipe */
    ax25_kiss_crc_error_cb_t on_crc_error; /**< Called when checksum verification fails */
    ax25_kiss_log_cb_t on_log; /**< Optional diagnostic/log message callback */
    void *user_data; /**< Opaque pointer passed to all callbacks */

    /* --- Statistics --- */
    ax25_kiss_stats_t stats; /**< Frame and byte counters for diagnostics */
};

/*============================================================================*/
/* CRC Utility Functions (SMACK CRC-16/ARC and G8BPQ XOR)                     */
/*============================================================================*/

/**
 * @defgroup KISSCRCUtils CRC Computation Utilities
 * @brief CRC-CCITT (polynomial 0x1021, initial value 0xFFFF) utilities
 */

/**
 * @brief Compute 16-bit CRC-CCITT over a byte buffer
 *
 * SMACK CRC-16/ARC: poly 0x8005 reflected LSB-first, init 0x0000, no final XOR
 * Covers type byte + all data bytes computed BEFORE SLIP encoding, appended LSB first
 *
 * @param[in] data   Pointer to input bytes
 * @param[in] len    Number of bytes to process
 * @return 16-bit CRC-CCITT value; transmitted LSB first per SMACK spec
 */
uint16_t ax25_kiss_smack_crc16(const uint8_t *data, size_t len);

/**
 * @brief Compute 8-bit XOR checksum (G8BPQ KISS ROM style)
 *
 * G8BPQ 8-bit XOR checksum of all bytes including type byte, before SLIP encoding
 *
 * @param[in] data  Pointer to input bytes (including type byte)
 * @param[in] len   Number of bytes to process
 * @return 8-bit XOR checksum value
 */
uint8_t ax25_kiss_crc8_xor(const uint8_t *data, size_t len);

/*============================================================================*/
/* Public API - Context Lifecycle                                             */
/*============================================================================*/

/**
 * @brief Initialize a KISS context to factory defaults
 *
 * Zeroes the entire context and applies specification-default values to all
 * 16 per-port parameter sets.  The receive state machine is reset to IDLE.
 * All callback pointers are set to NULL.
 *
 * @note The caller MUST set serial_write before calling any transmit function.
 *       Optional callbacks (on_frame, on_hardware, on_return, on_crc_error,
 *       on_log) should be set before the first call to ax25_kiss_receive_byte().
 *
 * @param[out] ctx   Pointer to context to initialize
 * @return KISS_OK on success, KISS_ERR_NULL if ctx is NULL
 */
uint8_t ax25_kiss_init(ax25_kiss_ctx_t *ctx);

/**
 * @brief Reset the receive state machine without clearing parameters
 *
 * Resets only the RX parser state, counters, and in-progress frame buffer.
 * Per-port parameters, callbacks, mode flags, and statistics are preserved.
 * Useful after link reconnection or a detected framing error.
 *
 * @param[in,out] ctx  KISS context
 * @return KISS_OK on success, KISS_ERR_NULL if ctx is NULL
 */
uint8_t ax25_kiss_reset_rx(ax25_kiss_ctx_t *ctx);

/**
 * @brief Reset statistics counters to zero
 *
 * @param[in,out] ctx  KISS context
 * @return KISS_OK on success, KISS_ERR_NULL if ctx is NULL
 */
uint8_t ax25_kiss_reset_stats(ax25_kiss_ctx_t *ctx);

/*============================================================================*/
/* Public API - Mode Control                                                  */
/*============================================================================*/

/**
 * @brief Enter KISS mode
 *
 * Transmits an opening FEND to the TNC to signal the start of KISS operation
 * and sets kiss_mode = true.  Must be called before any frame transmission.
 *
 * @note For TCP-KISS transports, call this after the TCP connection is
 *       established to synchronize the KISS frame boundary on the stream.
 *
 * @param[in,out] ctx  KISS context
 * @return KISS_OK, KISS_ERR_NULL, or KISS_ERR_NO_SERIAL
 */
uint8_t ax25_kiss_enter(ax25_kiss_ctx_t *ctx);

/**
 * @brief Set the KISS protocol variant / checksum mode
 *
 * Changes the checksum mode applied to transmitted data frames and expected
 * on received data frames.  This may be called at any time, including after
 * KISS mode has been entered.
 *
 * When switching to KISS_VARIANT_AUTO, the current mode is set to standard
 * KISS; the context upgrades automatically to SMACK when it receives the
 * first SMACK-flagged frame.
 *
 * @param[in,out] ctx      KISS context
 * @param[in]     variant  Desired protocol variant
 * @return KISS_OK on success, KISS_ERR_NULL if ctx is NULL
 */
uint8_t ax25_kiss_set_variant(ax25_kiss_ctx_t *ctx, ax25_kiss_variant_t variant);

/**
 * @brief Query the current active KISS variant
 *
 * In KISS_VARIANT_AUTO mode this returns the variant actually in use
 * (KISS_VARIANT_STANDARD before the first SMACK frame, KISS_VARIANT_SMACK
 * afterward).
 *
 * @param[in]  ctx      KISS context (const)
 * @param[out] variant  Set to the current active variant
 * @return KISS_OK, KISS_ERR_NULL
 */
uint8_t ax25_kiss_get_variant(const ax25_kiss_ctx_t *ctx, ax25_kiss_variant_t *variant);

/*============================================================================*/
/* Public API - Frame Reception                                               */
/*============================================================================*/

/**
 * @brief Feed a single byte from the serial port into the receive state machine
 *
 * The primary receive entry point.  Call for every byte received from the
 * TNC serial interface, TCP stream, or pipe.  When a complete frame is
 * assembled the appropriate callback is invoked synchronously before this
 * function returns.
 *
 * On buffer overflow (rx_len == KISS_MAX_FRAME_SIZE), the byte is silently
 * discarded, stats.rx_overflows is incremented, and frame assembly continues.
 * The resulting truncated frame will be dispatched when FEND arrives; upper
 * layers should treat it as corrupt.
 *
 * @param[in,out] ctx   KISS context
 * @param[in]     byte  Received byte from TNC/stream
 */
void ax25_kiss_receive_byte(ax25_kiss_ctx_t *ctx, uint8_t byte);

/**
 * @brief Feed a byte buffer from the serial port into the state machine
 *
 * Convenience wrapper around ax25_kiss_receive_byte() for processing bulk
 * read buffers from read()/recv() system calls.
 *
 * @param[in,out] ctx   KISS context
 * @param[in]     data  Input byte buffer
 * @param[in]     len   Number of bytes to process
 */
void ax25_kiss_receive_bytes(ax25_kiss_ctx_t *ctx, const uint8_t *data, size_t len);

/*============================================================================*/
/* Public API - Frame Transmission                                            */
/*============================================================================*/

/**
 * @brief Send a raw AX.25 data frame via KISS (CMD=0x00)
 *
 * Wraps the frame in KISS framing:
 *   FEND [type_byte] [escaped_data...] [optional_checksum] FEND
 *
 * The type byte is built as (port << 4) | KISS_CMD_DATA.
 * For SMACK mode the type byte MSB is set (0x80 for port 0) and a 16-bit
 * CRC-CCITT is appended to the escaped payload before the closing FEND.
 * For G8BPQ mode an 8-bit XOR checksum byte is appended.
 *
 * @note Port 15 (0xF) is reserved and rejected with KISS_ERR_PORT.
 *
 * @param[in,out] ctx    KISS context (must have serial_write set, must be in KISS mode)
 * @param[in]     port   Logical port number (0-14)
 * @param[in]     frame  Raw AX.25 frame bytes (no HDLC flags, no FCS)
 * @param[in]     len    Frame length in bytes (0 < len <= KISS_MAX_FRAME_SIZE)
 * @return KISS_OK on success, or KISS_ERR_* error code
 */
uint8_t ax25_kiss_send_frame(ax25_kiss_ctx_t *ctx, uint8_t port, const uint8_t *frame, size_t len);

/**
 * @brief Send a KISS parameter command (CMD=0x01..0x06) to the TNC
 *
 * Constructs and transmits: FEND [type_byte] [data[0..len-1]] FEND
 * Does not apply any checksum regardless of active variant (parameter
 * commands are never checksummed in any KISS variant).
 *
 * KISS_CMD_DATA and KISS_CMD_RETURN are rejected; use ax25_kiss_send_frame()
 * and ax25_kiss_send_return() respectively.
 *
 * @param[in,out] ctx    KISS context
 * @param[in]     port   Logical port (0-14)
 * @param[in]     cmd    Command code (KISS_CMD_TXDELAY .. KISS_CMD_SETHARDWARE)
 * @param[in]     data   Parameter byte(s) to send after the type byte
 * @param[in]     len    Number of parameter bytes (1 for most commands)
 * @return KISS_OK on success, or KISS_ERR_* error code
 */
uint8_t ax25_kiss_send_command(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t cmd, const uint8_t *data, size_t len);

/**
 * @brief Send the Return / Exit-KISS command (type byte 0xFF)
 *
 * Transmits: FEND 0xFF FEND  (no payload; 0xFF never contains FEND/FESC so
 * no escaping is required).
 * Sets kiss_mode = false after the bytes are handed to serial_write.
 *
 * @param[in,out] ctx  KISS context
 * @return KISS_OK on success, or KISS_ERR_* error code
 */
uint8_t ax25_kiss_send_return(ax25_kiss_ctx_t *ctx);

/*============================================================================*/
/* Public API - Port Parameter Management                                     */
/*============================================================================*/

/**
 * @brief Apply a full set of port parameters and transmit them to the TNC
 *
 * Sends TxDelay, Persistence, SlotTime, TxTail, FullDuplex and (if
 * hardware_len > 0) SetHardware commands in sequence for the specified port,
 * then updates the local parameter mirror.
 *
 * Returns at the first failed command; the local mirror is updated only
 * after all commands succeed.
 *
 * @param[in,out] ctx     KISS context
 * @param[in]     port    Logical port (0-14)
 * @param[in]     params  Parameter set to apply (must not be NULL)
 * @return KISS_OK on success, or KISS_ERR_* from the first failing command
 */
uint8_t ax25_kiss_set_port_params(ax25_kiss_ctx_t *ctx, uint8_t port, const ax25_kiss_port_params_t *params);

/**
 * @brief Read the locally mirrored port parameters (does NOT query TNC)
 *
 * Returns the parameter values as last set by ax25_kiss_set_port_params() or
 * updated when a parameter command frame was received from the TNC.
 *
 * @param[in]  ctx     KISS context (const)
 * @param[in]  port    Logical port (0-14)
 * @param[out] params  Destination for current parameter values (must not be NULL)
 * @return KISS_OK on success, or KISS_ERR_* error code
 */
uint8_t ax25_kiss_get_port_params(const ax25_kiss_ctx_t *ctx, uint8_t port, ax25_kiss_port_params_t *params);

/**
 * @brief Reset port parameters for a single port to specification defaults
 *
 * Applies KISS_DEFAULT_* values to the local mirror for the given port
 * and transmits all five standard parameter commands to the TNC.
 *
 * @param[in,out] ctx   KISS context
 * @param[in]     port  Logical port (0-14)
 * @return KISS_OK on success, or KISS_ERR_* error code
 */
uint8_t ax25_kiss_reset_port_params(ax25_kiss_ctx_t *ctx, uint8_t port);

/**
 * @brief Reset all 15 user ports (0-14) to specification defaults
 *
 * Calls ax25_kiss_reset_port_params() for every valid port in sequence.
 * Returns at the first error.
 *
 * @param[in,out] ctx  KISS context
 * @return KISS_OK if all ports reset successfully, or KISS_ERR_* on first failure
 */
uint8_t ax25_kiss_reset_all_ports(ax25_kiss_ctx_t *ctx);

/*============================================================================*/
/* Public API - MKISS Multi-Port Support                                      */
/*============================================================================*/

/**
 * @defgroup MKISSSupport MKISS Multi-Port Context Functions
 * @brief Utilities for the Linux MKISS multi-port TNC demultiplexer model
 *
 * MKISS (ax25-tools mkiss) splits a single physical KISS serial device into
 * up to 16 virtual single-port KISS devices using the port nibble in the
 * type indicator byte.  This section provides helpers for managing the
 * port mapping, optional G8BPQ one-byte checksum mode, optional SMACK 16-bit
 * CRC mode, polled-access mode, and per-port statistics.
 *
 * These functions operate on the same ax25_kiss_ctx_t used for standard KISS.
 * They do not require a separate context type.
 */

/**
 * @brief Enable or disable G8BPQ polling mode
 *
 * Polled mode is used by G8BPQ KISS ROMs to prevent bus contention when
 * multiple TNCs share one serial line.  When enabled the host must poll
 * the TNC by sending a FEND with no payload at the interval specified.
 *
 * @param[in,out] ctx          KISS context
 * @param[in]     enable       true to enable polling, false to disable
 * @param[in]     interval_100ms Poll interval in 100 ms units (1-255, 0 = disable)
 * @return KISS_OK on success, KISS_ERR_NULL if ctx is NULL
 */
uint8_t ax25_kiss_set_poll_mode(ax25_kiss_ctx_t *ctx, bool enable, uint8_t interval_100ms);

/**
 * @brief Enable or disable hardware flow control on the serial link
 *
 * The KISS specification explicitly states that no hardware flow control
 * shall be used.  However some implementations (e.g., certain G8BPQ builds)
 * do use RTS/CTS.  Setting this flag causes it to be recorded in the
 * context for the application layer's reference; the KISS library itself
 * does not control the UART hardware.
 *
 * @param[in,out] ctx     KISS context
 * @param[in]     enable  true = hardware flow control in use
 * @return KISS_OK on success, KISS_ERR_NULL if ctx is NULL
 */
uint8_t ax25_kiss_set_hw_flowctrl(ax25_kiss_ctx_t *ctx, bool enable);

/*============================================================================*/
/* Public API - Statistics and Diagnostics                                    */
/*============================================================================*/

/**
 * @brief Retrieve a snapshot of the current statistics counters
 *
 * Copies the stats structure from the context into the caller-supplied
 * buffer.  The copy is not atomic; if the receive state machine runs
 * concurrently, the caller should provide external locking.
 *
 * @param[in]  ctx    KISS context (const)
 * @param[out] stats  Destination for statistics snapshot
 * @return KISS_OK on success, KISS_ERR_NULL if either pointer is NULL
 */
uint8_t ax25_kiss_get_stats(const ax25_kiss_ctx_t *ctx, ax25_kiss_stats_t *stats);

/**
 * @brief Query whether a SMACK CRC upgrade has been auto-negotiated
 *
 * In KISS_VARIANT_AUTO mode this returns true once the first SMACK frame
 * has been received and CRC mode has been activated.
 *
 * @param[in]  ctx    KISS context (const)
 * @param[out] active Set to true if SMACK is currently active
 * @return KISS_OK on success, KISS_ERR_NULL if either pointer is NULL
 */
uint8_t ax25_kiss_smack_is_active(const ax25_kiss_ctx_t *ctx, bool *active);

#endif /* AX25_KISS_H_ */
