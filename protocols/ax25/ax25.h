/**
 * @file ax25.h
 * @brief AX.25 v2.2 Protocol Library - Core Frame Structures and Functions
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @section Overview
 * This header defines the complete AX.25 v2.2 Link Access Protocol for Amateur
 * Packet Radio. AX.25 is derived from the X.25 protocol and HDLC standard,
 * designed specifically for amateur radio environments where both ends of
 * the link operate as peer stations rather than master/slave configurations.
 *
 * @section Protocol_Features
 * - HDLC-based frame structure with bit stuffing and flag delimiters
 * - 7-byte address fields with callsign, SSID (0-15), and control bits
 * - Modulo-8 (3-bit) and Modulo-128 (7-bit) sequence numbering
 * - Three frame types: Information (I), Supervisory (S), and Unnumbered (U)
 * - Level 2 digipeater support with up to 8 repeaters in path
 * - XID parameter negotiation for operational characteristics
 * - Selective Reject (SREJ) and Implicit Reject (REJ) error recovery
 * - Full-duplex and half-duplex operation modes
 * - Protocol Identifier (PID) field for layer 3 multiplexing
 *
 * @section Standards_References
 * - AX.25 Link Access Protocol for Amateur Packet Radio, Version 2.2 (July 1998)
 * - ISO 3309 HDLC frame structure
 * - ITU-T X.25 packet layer protocol
 * - FX.25 Forward Error Correction extension
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef AX25_H_
#define AX25_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/*============================================================================*/
/* Control Field Masks and Values                                             */
/*============================================================================*/

/**
 * @defgroup ControlFieldMasks Control Field Identification Masks
 * @brief Bit masks for identifying frame types from the control field
 *
 * The AX.25 control field uses specific bit patterns to identify frame types:
 * - I-frames: Bit 0 = 0 (information transfer with sequence numbers)
 * - S-frames: Bits 0-1 = 01 (supervisory functions: RR, RNR, REJ, SREJ)
 * - U-frames: Bits 0-1 = 11 (unnumbered control and information)
 *
 * @section Control_Field_Structure
 * For modulo-8 (8-bit control):
 * - I-frame: [N(R) 3bits][P/F 1bit][N(S) 3bits][0]
 * - S-frame: [N(R) 3bits][P/F 1bit][S 2bits][01]
 * - U-frame: [M 3bits][P/F 1bit][M 2bits][11]
 *
 * For modulo-128 (16-bit control):
 * - I-frame: [N(R) 7bits][P/F 1bit][N(S) 7bits][0]
 * - S-frame: [N(R) 7bits][P/F 1bit][S 2bits][01]
 */
#define CONTROL_I_MASK  0x01 /**< Mask for I-frame identification (bit 0 = 0) */
#define CONTROL_I_VAL   0x00 /**< Value indicating I-frame type */
#define CONTROL_US_MASK 0x03 /**< Mask for S/U frame identification (bits 0-1) */
#define CONTROL_S_VAL   0x01 /**< Value indicating S-frame type (binary 01) */
#define CONTROL_U_VAL   0x03 /**< Value indicating U-frame type (binary 11) */

// ax25_frame_class_t enum and canonical classification helpers
// Canonical frame class mirroring the three categories used by Linux ax25_decode()
// in net/ax25/ax25_in.c.  Distinct from ax25_frame_type_t which carries full
// per-subtype resolution needed by the state machine.
typedef enum {
    AX25_FRAME_CLASS_I = 0,   // Information frame:  bit  0 of control == 0
    AX25_FRAME_CLASS_S,       // Supervisory frame:  bits 1:0 == 0b01
    AX25_FRAME_CLASS_U,       // Unnumbered frame:   bits 1:0 == 0b11
    AX25_FRAME_CLASS_UNKNOWN  // Defensive catch-all, unreachable in practice
} ax25_frame_class_t;

// ax25_frame_class: classify the first control byte exactly as Linux does.
// Priority: I first (bit 0 == 0), then S (bits[1:0]==01), then U (bits[1:0]==11).
// This prevents UI (0x03, bits[1:0]==11) from ever being misidentified as an
// I-frame because 0x03 & 0x01 == 1 (not 0).
static inline ax25_frame_class_t ax25_frame_class(uint8_t ctrl) {
    if ((ctrl & 0x01u) == 0x00u)
        return AX25_FRAME_CLASS_I;
    if ((ctrl & 0x03u) == 0x01u)
        return AX25_FRAME_CLASS_S;
    if ((ctrl & 0x03u) == 0x03u)
        return AX25_FRAME_CLASS_U;
    return AX25_FRAME_CLASS_UNKNOWN;  // unreachable: covers all 2-bit patterns
}

// ax25_get_pf_mod8: extract P/F bit from a mod-8 (1-byte) control field.
// P/F occupies bit 4 (0x10) for I, S, and U frames in 8-bit mode.
static inline uint8_t ax25_get_pf_mod8(uint8_t ctrl) {
    return (ctrl >> 4u) & 0x01u;  // bit 4 per AX.25 v2.2 tables 4, 5, 6
}

// ax25_get_pf_mod128: extract P/F bit from the SECOND byte of a mod-128
// (2-byte) control field (I or S frames only).
// In the 16-bit control word the P/F bit is bit 8, i.e. bit 0 of byte 1.
static inline uint8_t ax25_get_pf_mod128(uint8_t ctrl_byte1) {
    return ctrl_byte1 & 0x01u;  // bit 0 of second byte == bit 8 of 16-bit word
}

// ax25_u_subtype: return canonical U-frame opcode with P/F stripped.
// Masking bit 4 removes the Poll/Final bit, giving the modifier that can be
// compared directly against AX25_U_* constants (e.g. AX25_U_UI == 0x03).
static inline uint8_t ax25_u_subtype(uint8_t ctrl) {
    return (uint8_t) (ctrl & 0xEFu);  // clear bit 4 (P/F), keep modifier bits
}

// ax25_s_subtype: return supervisory sub-command code (0-3) from control byte.
// Bits 3:2 carry the code (0=RR, 1=RNR, 2=REJ, 3=SREJ).
// Bits 1:0 are always 0b01 for S-frames; P/F is bit 4.
static inline uint8_t ax25_s_subtype(uint8_t ctrl) {
    return (uint8_t) ((ctrl >> 2u) & 0x03u);  // bits 3:2, normalised to 0-3
}

/*============================================================================*/
/* Poll/Final Bit Positions                                                     */
/*============================================================================*/

/**
 * @defgroup PollFinalBits Poll/Final Bit Position Constants
 * @brief Bit positions for the Poll/Final bit in control fields
 *
 * The P/F bit serves different purposes depending on frame direction:
 * - Command frames (C=1): P bit requests immediate response
 * - Response frames (C=0): F bit indicates final response to poll
 *
 * The P/F bit position varies between modulo-8 and modulo-128 modes.
 */
#define POLL_FINAL_8BIT  0x10   /**< P/F bit position in 8-bit control field (bit 4) */
#define POLL_FINAL_16BIT 0x0100 /**< P/F bit position in 16-bit control field (bit 8) */

// U-frame opcodes after stripping the P/F bit (bit 4) from the control byte.
// These values match the modifier literals used in ax25_unnumbered_frame_decode()
// and are defined here so callers of ax25_parse_ctrl() can switch on out->u_cmd
// without embedding magic numbers.
#define AX25_U_UI     0x03u  // Unnumbered Information
#define AX25_U_DM     0x0Fu  // Disconnected Mode
#define AX25_U_SABM   0x2Fu  // Set ABM (mod-8)
#define AX25_U_DISC   0x43u  // Disconnect
#define AX25_U_UA     0x63u  // Unnumbered Acknowledge
#define AX25_U_SABME  0x6Fu  // Set ABM Extended (mod-128)
#define AX25_U_FRMR   0x87u  // Frame Reject
#define AX25_U_XID    0xAFu  // Exchange Identification
#define AX25_U_TEST   0xE3u  // Test

/*============================================================================*/
/* Modulo Sequence Numbering Constants                                        */
/*============================================================================*/

/**
 * @defgroup ModuloConstants Modulo Sequence Numbering Modes
 * @brief Constants controlling sequence number modulus selection
 *
 * AX.25 supports two sequence numbering modes:
 * - Modulo-8: 3-bit sequence numbers (0-7), 8-bit control field
 * - Modulo-128: 7-bit sequence numbers (0-127), 16-bit control field
 *
 * Modulo-128 is negotiated via XID or indicated by the SABME frame.
 * The res1 bit in the source address SSID byte indicates modulo-128
 * when cleared (0 = modulo-128, 1 = modulo-8).
 */
#define MODULO128_NONE  -1 /**< No modulo 128 - return raw frame without parsing */
#define MODULO128_FALSE 0  /**< Explicitly force modulo-8 (8-bit control) */
#define MODULO128_TRUE  1  /**< Explicitly force modulo-128 (16-bit control) */
#define MODULO128_AUTO  2  /**< Auto-detect based on res1 bit in source address */

/*============================================================================*/
/* Address Field Constants                                                      */
/*============================================================================*/

/**
 * @defgroup AddressConstants Address Field Limits
 * @brief Maximum values for address field components
 *
 * The AX.25 address field supports:
 * - Up to 8 level-2 repeaters (digipeaters) in a single path
 * - 6-character callsigns padded with spaces
 * - 4-bit SSID values (0-15) for multiple stations per callsign
 */
#define AX25_MAX_REPEATERS 8    /**< Maximum repeaters in digipeater path per AX.25 v2.2 Section 3.12.3 */
#define CALLSIGN_MAX 7          /**< Maximum callsign length (6 chars + null terminator) */

/*============================================================================*/
/* Protocol Identifier (PID) Codes                                              */
/*============================================================================*/

/**
 * @defgroup PIDCodes Layer 3 Protocol Identifiers
 * @brief Protocol Identifier values for layer 3 protocol multiplexing
 *
 * The PID field identifies the layer 3 protocol carried in I-frames and
 * UI frames. Values are defined in AX.25 v2.2 Section 6.5.
 *
 * @section Common_PIDs
 * - 0xF0: No layer 3 protocol (most common for raw data)
 * - 0xCC: ARPA Internet Protocol (IP over AX.25)
 * - 0x08: Segmentation fragment (for large payload splitting)
 */
#define PID_ISO8208_CCITT   0x01 /**< ISO 8208/CCITT X.25 PLP */
#define PID_VJ_IP4_COMPRESS 0x06 /**< Compressed TCP/IP (Van Jacobson, RFC 1144) */
#define PID_VJ_IP4          0x07 /**< Uncompressed TCP/IP (Van Jacobson, RFC 1144) */
#define PID_SEGMENTATION    0x08 /**< Segmentation fragment per AX.25 v2.2 Appendix C6 */
#define PID_TEXNET          0xC3 /**< TEXNET datagram protocol */
#define PID_LINKQUALITY     0xC4 /**< Link Quality Protocol */
#define PID_APPLETALK       0xCA /**< Appletalk */
#define PID_APPLETALK_ARP   0xCB /**< Appletalk ARP */
#define PID_ARPA_IP4        0xCC /**< ARPA Internet Protocol Version 4 */
#define PID_APRA_ARP        0xCD /**< ARPA Address Resolution Protocol */
#define PID_FLEXNET         0xCE /**< FlexNet protocol */
#define PID_NETROM          0xCF /**< NET/ROM protocol */
#define PID_NO_L3           0xF0 /**< No layer 3 protocol implemented */
#define PID_ESCAPE          0xFF /**< Escape for extended PID (next byte contains extended PID) */

// Maximum byte length of any fully-encoded AX.25 control frame (no I-field).
// Worst-case: dest(7) + src(7) + 8 repeaters(56) + 2-byte ctrl + 5-byte FRMR = 77, +3 margin = 80.
// I-frames use ax25_frame_encode() + heap (stored in tx_queue for retransmission).
#define AX25_ENCODE_SCRATCH_LEN  80u

/*============================================================================*/
/* Frame Type Enumeration                                                       */
/*============================================================================*/

// ax25_ctrl_t: result of ax25_parse_ctrl().
// All fields not relevant to the decoded frame type are zero-initialised.
typedef struct {
    uint8_t type;     // Frame type: 'I' = information, 'S' = supervisory, 'U' = unnumbered
    uint8_t s_cmd;    // S-frame sub-type: 0=RR, 1=RNR, 2=REJ, 3=SREJ (valid when type=='S')
    uint8_t u_cmd;    // U-frame opcode with P/F stripped (valid when type=='U'); compare to AX25_U_* defines
    uint8_t pf;       // Poll/Final bit: 1=set, 0=clear
    uint8_t ns;       // N(S): mod-8 range 0-7, mod-128 range 0-127 (valid when type=='I')
    uint8_t nr;       // N(R): mod-8 range 0-7, mod-128 range 0-127 (valid when type=='I' or 'S')
    uint8_t ctrl_len;  // Bytes consumed from ctrl[]: 1 for mod-8 I/S and all U, 2 for mod-128 I/S
} ax25_ctrl_t;

/**
 * @brief Enumeration of all AX.25 frame types
 *
 * Defines the complete set of frame types in AX.25 v2.2, including:
 * - Raw frames: Unparsed control field
 * - Unnumbered frames: UI, SABM, SABME, DISC, DM, UA, FRMR, XID, TEST
 * - Information frames: I-frames with 8-bit or 16-bit control
 * - Supervisory frames: RR, RNR, REJ, SREJ with 8-bit or 16-bit control
 *
 * The distinction between 8-bit and 16-bit variants corresponds to
 * modulo-8 versus modulo-128 sequence numbering.
 */
typedef enum {
    AX25_FRAME_RAW, /**< Raw frame with unparsed control field */
    AX25_FRAME_UNNUMBERED_INFORMATION, /**< UI frame - connectionless data transfer */
    AX25_FRAME_UNNUMBERED_SABM, /**< Set Asynchronous Balanced Mode (modulo-8) */
    AX25_FRAME_UNNUMBERED_SABME, /**< Set Asynchronous Balanced Mode Extended (modulo-128) */
    AX25_FRAME_UNNUMBERED_DISC, /**< Disconnect command */
    AX25_FRAME_UNNUMBERED_DM, /**< Disconnected Mode response */
    AX25_FRAME_UNNUMBERED_UA, /**< Unnumbered Acknowledge response */
    AX25_FRAME_UNNUMBERED_FRMR, /**< Frame Reject - protocol error indication */
    AX25_FRAME_UNNUMBERED_XID, /**< Exchange Identification - parameter negotiation */
    AX25_FRAME_UNNUMBERED_TEST, /**< Test frame - link quality verification */
    AX25_FRAME_INFORMATION_8BIT, /**< I-frame with 8-bit control (modulo-8) */
    AX25_FRAME_INFORMATION_16BIT, /**< I-frame with 16-bit control (modulo-128) */
    AX25_FRAME_SUPERVISORY_RR_8BIT, /**< Receive Ready with 8-bit control */
    AX25_FRAME_SUPERVISORY_RNR_8BIT, /**< Receive Not Ready with 8-bit control */
    AX25_FRAME_SUPERVISORY_REJ_8BIT, /**< Reject with 8-bit control */
    AX25_FRAME_SUPERVISORY_SREJ_8BIT, /**< Selective Reject with 8-bit control */
    AX25_FRAME_SUPERVISORY_RR_16BIT, /**< Receive Ready with 16-bit control */
    AX25_FRAME_SUPERVISORY_RNR_16BIT, /**< Receive Not Ready with 16-bit control */
    AX25_FRAME_SUPERVISORY_REJ_16BIT, /**< Reject with 16-bit control */
    AX25_FRAME_SUPERVISORY_SREJ_16BIT /**< Selective Reject with 16-bit control */
} ax25_frame_type_t;

/*============================================================================*/
/* Segmentation Structures                                                      */
/*============================================================================*/

/**
 * @brief Segmented information field structure
 *
 * Represents a single segment of a larger payload that has been split
 * across multiple I-frames per AX.25 v2.2 Appendix C6 (Segmentation).
 *
 * @section Segmentation_Overview
 * When the payload exceeds the negotiated maximum I-field length (N1),
 * the segmenter splits it into multiple fragments. Each fragment is
 * carried in a separate I-frame with PID=0x08 (segmentation).
 *
 * @section Segment_Header_Format
 * The first byte of each segmented info field contains:
 * - Bit 7 (0x80): First segment indicator (BEG)
 * - Bit 6 (0x40): Last segment indicator (END)
 * - Bits 5-0: 6-bit segment sequence number (0-63)
 *
 * The first segment additionally contains a 2-byte total length field
 * in big-endian format.
 */
typedef struct {
    uint8_t *info_field; /**< Pointer to segmented data buffer */
    size_t info_field_len; /**< Length of this segment in bytes */
} ax25_segmented_info_t;

/**
 * @brief Internal reassembly segment tracking structure
 *
 * Used during reassembly of segmented frames to track individual
 * segments before they are ordered and combined into the complete payload.
 */
typedef struct {
    uint8_t control; /**< Segment header byte (BEG/END/sequence) */
    uint16_t total_length; /**< Total original payload length (from first segment) */
    uint8_t *data; /**< Pointer to segment payload data */
    size_t data_len; /**< Length of segment payload */
    int segment_number; /**< 6-bit sequence number for ordering */
} ax25_reassembly_segment_t;

/*============================================================================*/
/* Address Structures                                                           */
/*============================================================================*/

/**
 * @brief AX.25 address structure
 *
 * Represents a single AX.25 station address including callsign, SSID,
 * and control bits as defined in AX.25 v2.2 Section 6.2.
 *
 * @section Address_Encoding
 * On-air format (7 bytes per address):
 * - Bytes 0-5: Callsign characters, each shifted left 1 bit (ASCII * 2)
 * - Byte 6: SSID byte with control bits
 *
 * @section SSID_Byte_Structure
 * Bit 7: C/H bit - Command/Response (dest/source) or Has-been-repeated (repeater)
 * Bit 6: res1 - Reserved (modulo-128 indicator: 0=mod128, 1=mod8)
 * Bit 5: res0 - Reserved (should be 1)
 * Bits 4-1: SSID value (0-15)
 * Bit 0: Extension bit (0=more addresses follow, 1=last address)
 */
typedef struct {
    char callsign[CALLSIGN_MAX]; /**< Callsign, 1-6 chars, space-padded, null-terminated */
    int ssid; /**< Secondary Station Identifier (0-15) */
    bool ch; /**< C bit (command/response) or H bit (has-been-repeated) */
    bool res0; /**< Reserved bit 0 - should be set to 1 */
    bool res1; /**< Reserved bit 1 - modulo-128 indicator: 0=mod-128, 1=mod-8 (on-air) */
    bool mod8_legacy; /**< true = peer signals modulo-8 (res1=1 on-air); false = peer is modulo-128 capable (res1=0 on-air). Per AX.25 sec 3.12.2 and PE1CHL sec 4. */
    bool extension; /**< HDLC extension bit - 1 indicates last address */
} ax25_address_t;

/**
 * @brief Repeater path structure
 *
 * Contains the ordered list of digipeater addresses for frames that
 * traverse multiple level-2 repeaters. Supports up to 8 repeaters per
 * AX.25 v2.2 Section 3.12.3.
 *
 * @section Digipeater_Operation
 * Each repeater in the path processes frames sequentially:
 * 1. Examines next unused address (H-bit = 0)
 * 2. If matches own callsign, sets H-bit to 1 and retransmits
 * 3. Subsequent repeaters process until destination reached
 */
typedef struct {
    ax25_address_t repeaters[AX25_MAX_REPEATERS]; /**< Array of repeater addresses */
    int num_repeaters; /**< Number of repeaters in path (0-8) */
} ax25_path_t;

/*============================================================================*/
/* Frame Header Structure                                                       */
/*============================================================================*/

/**
 * @brief AX.25 frame header structure
 *
 * Contains the complete address field information for any AX.25 frame,
 * including destination, source, and optional repeater path.
 *
 * @section Header_Structure
 * The address field always contains at minimum:
 * - Destination address (7 bytes)
 * - Source address (7 bytes)
 *
 * Optionally followed by repeater addresses (7 bytes each).
 *
 * @section CR_Bit_Derivation
 * The command/response (CR) flag is derived from the C-bits:
 * - Command frame: Destination C-bit = 1, Source C-bit = 0
 * - Response frame: Destination C-bit = 0, Source C-bit = 1
 */
typedef struct {
    ax25_address_t destination; /**< Destination station address */
    ax25_address_t source; /**< Source station address */
    ax25_path_t repeaters; /**< Repeater path (empty if direct) */
    bool cr; /**< Command/Response flag for frame */
    bool src_cr; /**< Source C-bit value for processing */
} ax25_frame_header_t;

/*============================================================================*/
/* Base Frame Structure                                                         */
/*============================================================================*/

/**
 * @brief Base frame structure for all AX.25 frames
 *
 * This structure serves as the common header for all specific frame types.
 * It contains the frame type discriminator and the address header.
 *
 * @section Type_Polymorphism
 * All frame types can be cast to ax25_frame_t* for type-agnostic operations.
 * The type field determines the actual structure type for safe casting:
 * - AX25_FRAME_INFORMATION_* -> ax25_information_frame_t*
 * - AX25_FRAME_SUPERVISORY_* -> ax25_supervisory_frame_t*
 * - AX25_FRAME_UNNUMBERED_* -> ax25_unnumbered_frame_t* or specific subtype
 */
typedef struct {
    ax25_frame_type_t type; /**< Frame type discriminator */
    ax25_frame_header_t header; /**< Frame header with address information */
} ax25_frame_t;

/*============================================================================*/
/* Raw Frame Structure                                                          */
/*============================================================================*/

/**
 * @brief Raw AX.25 frame structure
 *
 * Used when the control field cannot be parsed or when MODULO128_NONE
 * is specified. Preserves the raw control byte and payload without
 * protocol interpretation.
 */
typedef struct {
    ax25_frame_t base; /**< Base frame structure */
    uint8_t control; /**< Raw control field byte(s) */
    uint8_t *payload; /**< Raw payload data following control */
    size_t payload_len; /**< Payload length in bytes */
} ax25_raw_frame_t;

/*============================================================================*/
/* Unnumbered Frame Structures                                                  */
/*============================================================================*/

/**
 * @brief Base unnumbered frame structure
 *
 * Common structure for all U-frames containing the modifier bits
 * and Poll/Final bit. U-frames have no sequence numbers.
 *
 * @section U_Frame_Modifiers
 * The modifier field (5 bits: M3,M4,M2,M1,M0) determines U-frame type:
 * - 0x2F (01111): SABM
 * - 0x6F (11011): SABME
 * - 0x43 (10001): DISC
 * - 0x0F (00111): DM
 * - 0x63 (11000): UA
 * - 0x87 (11100): FRMR
 * - 0xAF (10101): XID
 * - 0xE3 (11100): TEST
 * - 0x03 (00011): UI
 */
typedef struct {
    ax25_frame_t base; /**< Base frame structure */
    bool pf; /**< Poll/Final bit */
    uint8_t modifier; /**< 5-bit modifier determining U-frame type */
} ax25_unnumbered_frame_t;

/**
 * @brief Unnumbered Information (UI) frame structure
 *
 * Used for connectionless datagram transmission. UI frames carry
 * payload without establishing a connection and are not acknowledged.
 *
 * @section UI_Frame_Usage
 * Common applications include:
 * - APRS (Automatic Packet Reporting System)
 * - Beacon transmissions
 * - Connectionless layer 3 protocols
 *
 * @section UI_Control_Field
 * Control byte: 0x03 (modifier 00011) with optional P/F bit
 */
typedef struct {
    ax25_unnumbered_frame_t base; /**< Base unnumbered frame */
    uint8_t pid; /**< Protocol Identifier */
    uint8_t *payload; /**< UI payload data */
    size_t payload_len; /**< UI payload length */
} ax25_unnumbered_information_frame_t;

/**
 * @brief Frame Reject (FRMR) frame structure
 *
 * Sent to indicate a protocol error requiring link reset.
 * FRMR carries information about the rejected frame for diagnostics.
 *
 * @section FRMR_Triggers
 * Per AX.25 v2.2 Section 4.3.3.6, FRMR is sent when:
 * - W: Invalid control field received
 * - X: Frame with info field not permitted (U/S frame with wrong length)
 * - Y: Info field exceeded maximum length (N1)
 * - Z: Invalid N(R) received (acknowledgment of unsent frame)
 *
 * @section FRMR_Info_Field
 * For modulo-8: 3 bytes (control, V(S)/V(R)/CR, flags)
 * For modulo-128: 5 bytes (control-low, control-high, N(S)/CR, N(R), flags)
 */
typedef struct {
    ax25_unnumbered_frame_t base; /**< Base unnumbered frame */
    bool is_modulo128; /**< True if modulo-128 sequence numbers */
    uint16_t frmr_control; /**< Control field of rejected frame */
    int vs; /**< Send sequence number V(S) at error */
    int vr; /**< Receive sequence number V(R) at error */
    bool frmr_cr; /**< CR bit of rejected frame */
    bool w; /**< Invalid control field flag */
    bool x; /**< Info field not permitted flag */
    bool y; /**< Info field too long flag */
    bool z; /**< Invalid N(R) received flag */
} ax25_frame_reject_frame_t;

/*============================================================================*/
/* Information Frame Structure                                                  */
/*============================================================================*/

/**
 * @brief Information (I) frame structure
 *
 * Used for reliable data transfer with sequence numbering and
 * acknowledgment. I-frames carry the actual user data payload.
 *
 * @section I_Frame_Control
 * For modulo-8:  [N(R) 3bits][P/F 1bit][N(S) 3bits][0]
 * For modulo-128: [N(R) 7bits][P/F 1bit][N(S) 7bits][0]
 *
 * @section Sequence_Numbers
 * - N(S): Send sequence number (0-7 or 0-127)
 * - N(R): Receive sequence number - acknowledges receipt up to N(R)-1
 */
typedef struct {
    ax25_frame_t base; /**< Base frame structure */
    int nr; /**< Receive sequence number N(R) */
    bool pf; /**< Poll/Final bit */
    int ns; /**< Send sequence number N(S) */
    uint8_t pid; /**< Protocol Identifier */
    uint8_t *payload; /**< Information field payload */
    size_t payload_len; /**< Payload length (max N1 bytes) */
} ax25_information_frame_t;

/*============================================================================*/
/* Supervisory Frame Structure                                                  */
/*============================================================================*/

/**
 * @brief Supervisory (S) frame structure
 *
 * Used for flow control and error recovery without carrying data.
 * S-frames acknowledge received I-frames and indicate receiver status.
 *
 * @section S_Frame_Types
 * Code field (bits 2-3) determines type:
 * - 00 (0): RR (Receive Ready) - ready to receive, acknowledge N(R)-1
 * - 01 (1): RNR (Receive Not Ready) - busy condition, cannot receive
 * - 10 (2): REJ (Reject) - go-back-N retransmission request from N(R)
 * - 11 (3): SREJ (Selective Reject) - request retransmission of N(R) only
 *
 * @section Flow_Control
 * RR and RNR control the flow of I-frames:
 * - RR indicates receiver is ready for more data
 * - RNR indicates temporary busy condition (e.g., buffer full)
 */
typedef struct {
    ax25_frame_t base; /**< Base frame structure */
    int nr; /**< Receive sequence number N(R) */
    bool pf; /**< Poll/Final bit */
    uint8_t code; /**< Supervisory code (0=RR, 1=RNR, 2=REJ, 3=SREJ) */
} ax25_supervisory_frame_t;

/*============================================================================*/
/* XID Parameter Structures                                                     */
/*============================================================================*/

/**
 * @brief XID parameter structure
 *
 * Represents a single parameter in an XID (Exchange Identification) frame.
 * XID parameters are used to negotiate operational characteristics.
 *
 * @section XID_Parameter_Format
 * Each parameter encoded as: [PI 1byte][PL 1byte][PV PL bytes]
 * - PI: Parameter Identifier
 * - PL: Parameter Length (0-255)
 * - PV: Parameter Value
 *
 * @section Common_Parameters
 * - PI=2: Class of Procedures (half/full duplex)
 * - PI=3: HDLC Optional Functions (REJ/SREJ, modulo-8/128)
 * - PI=6: I-Field Length Receive (N1)
 * - PI=8: Window Size Receive (k)
 * - PI=9: Acknowledge Timer (T1)
 * - PI=10: Retries (N2)
 */
typedef struct AX25XIDParameter {
    int pi; /**< Parameter Identifier */
    uint8_t* (*encode)(const struct AX25XIDParameter*, size_t*, uint8_t *err); /**< Encode to binary */
    struct AX25XIDParameter* (*copy)(const struct AX25XIDParameter*, uint8_t *err); /**< Deep copy */
    void (*free)(struct AX25XIDParameter*, uint8_t *err); /**< Free resources */
    void *data; /**< Parameter-specific data */
} ax25_xid_parameter_t;

/**
 * @brief XID frame structure
 *
 * Exchange Identification frames negotiate link parameters between
 * stations. Sent after SABM/SABME connection establishment.
 *
 * @section XID_Structure
 * - FI (Function Identifier): 0x82 for parameter negotiation
 * - GI (Group Identifier): 0x80 for general group
 * - GL (Group Length): Total length of all parameters
 * - Parameters: Array of PI/PL/PV triplets
 */
typedef struct {
    ax25_unnumbered_frame_t base; /**< Base unnumbered frame */
    uint8_t fi; /**< Function Identifier */
    uint8_t gi; /**< Group Identifier */
    ax25_xid_parameter_t **parameters; /**< Array of parameter pointers */
    size_t param_count; /**< Number of parameters */
} ax25_exchange_identification_frame_t;

/**
 * @brief Raw parameter value structure (pointer-based)
 *
 * Alternative representation for XID parameter values using separate
 * pointer and length. More portable than flexible array members.
 */
typedef struct {
    uint8_t *pv; /**< Parameter value data pointer */
    size_t pv_len; /**< Length of parameter value */
} ax25_raw_parameter_t;

/*============================================================================*/
/* Test Frame Structure                                                         */
/*============================================================================*/

/**
 * @brief Test frame structure
 *
 * Used for link quality verification and round-trip time measurement.
 * The receiving station echoes the test payload back to the sender.
 *
 * @section TEST_Operation
 * 1. Station A sends TEST command with payload and P=1
 * 2. Station B receives TEST and sends TEST response with same payload and F=1
 * 3. Station A measures RTT from command to response
 *
 * @section Payload_Limits
 * Maximum test payload is implementation-dependent but typically
 * limited by maximum frame size (256 bytes excluding headers).
 */
typedef struct {
    ax25_unnumbered_frame_t base; /**< Base unnumbered frame */
    uint8_t *payload; /**< Test payload (echoed in response) */
    size_t payload_len; /**< Payload length */
} ax25_test_frame_t;

/*============================================================================*/
/* Header Decode Result                                                         */
/*============================================================================*/

/**
 * @brief Header decode result structure
 *
 * Returned by ax25_frame_header_decode() containing the decoded header
 * and information about remaining data in the buffer.
 */
typedef struct {
    ax25_frame_header_t *header; /**< Decoded header or NULL on failure */
    const uint8_t *remaining; /**< Pointer to data after header */
    size_t remaining_len; /**< Length of remaining data */
} header_decode_result_t;

/**
 * @brief Raw parameter data with flexible array member
 *
 * Internal structure for storing XID parameter values with embedded data.
 * Requires dynamic allocation with size calculation.
 *
 * @warning This structure uses a flexible array member and must be
 * allocated as: malloc(sizeof(ax25_raw_param_data_t) + pv_len)
 */
typedef struct {
    size_t pv_len; /**< Length of parameter value */
    uint8_t pv[]; /**< Flexible array member for parameter data */
} ax25_raw_param_data_t;

/*============================================================================*/
/* Global XID Defaults                                                          */
/*============================================================================*/

/**
 * @defgroup XID_Defaults Global XID Default Parameters
 * @brief Pre-initialized default XID parameters for AX.25 v2.0 and v2.2
 *
 * These global variables are initialized by ax25_xid_init_defaults() and
 * contain factory default parameters for XID negotiation.
 */
extern ax25_xid_parameter_t *AX25_20_DEFAULT_XID_COP; /**< AX.25 v2.0 Class of Procedures default */
extern ax25_xid_parameter_t *AX25_22_DEFAULT_XID_COP; /**< AX.25 v2.2 Class of Procedures default */
extern ax25_xid_parameter_t *AX25_20_DEFAULT_XID_HDLCOPTFUNC; /**< AX.25 v2.0 HDLC Optional Functions default */
extern ax25_xid_parameter_t *AX25_22_DEFAULT_XID_HDLCOPTFUNC; /**< AX.25 v2.2 HDLC Optional Functions default */
extern ax25_xid_parameter_t *AX25_20_DEFAULT_XID_IFIELDRX; /**< AX.25 v2.0 I-Field Length default */
extern ax25_xid_parameter_t *AX25_22_DEFAULT_XID_IFIELDRX; /**< AX.25 v2.2 I-Field Length default */
extern ax25_xid_parameter_t *AX25_20_DEFAULT_XID_WINDOWSZRX; /**< AX.25 v2.0 Window Size default */
extern ax25_xid_parameter_t *AX25_22_DEFAULT_XID_WINDOWSZRX; /**< AX.25 v2.2 Window Size default */
extern ax25_xid_parameter_t *AX25_20_DEFAULT_XID_ACKTIMER; /**< AX.25 v2.0 Ack Timer default */
extern ax25_xid_parameter_t *AX25_22_DEFAULT_XID_ACKTIMER; /**< AX.25 v2.2 Ack Timer default */
extern ax25_xid_parameter_t *AX25_20_DEFAULT_XID_RETRIES; /**< AX.25 v2.0 Retries default */
extern ax25_xid_parameter_t *AX25_22_DEFAULT_XID_RETRIES; /**< AX.25 v2.2 Retries default */

// Public inline utilities for modular sequence arithmetic used by the data link
// state machine and any upper-layer code that needs window validation.
// These cover the three per-connection state variables V(S), V(R), V(A) and
// the window arithmetic defined in AX.25 v2.2 §4.2.2 and §6.4.

// Bit masks for sequence number modulo modes
#define MOD8_MASK   0x07u  // Modulo-8: 3-bit sequence numbers (range 0-7)
#define MOD128_MASK 0x7Fu  // Modulo-128: 7-bit sequence numbers (range 0-127)

// seqn_mask: derive the correct mask from the modulus value stored in
// ax25_state_vars_t.mod (8 or 128).  Avoids scattering ternaries at call sites.
static inline uint8_t seqn_mask(uint8_t mod) {
    return (mod == 128u) ? MOD128_MASK : MOD8_MASK;
}

// seqn_inc: increment a sequence number by 1 with modular wrap.
// Equivalent to (v + 1) mod (mask+1).
// Usage: conn->vars.vs = seqn_inc(conn->vars.vs, seqn_mask(conn->vars.mod));
static inline uint8_t seqn_inc(uint8_t v, uint8_t mask) {
    return (uint8_t) ((v + 1u) & mask);
}

// seqn_sub: modular subtraction a - b, result always in [0, mask].
// C unsigned arithmetic wraps naturally; masking gives the modular distance.
// Example (mod-8): a=1, b=6 -> (1-6) in uint8 = 0xFB, & 0x07 = 3 (correct: 6->7->0->1).
// Simpler and equivalent to the (a - b + mask + 1) & mask form; no risk of
// intermediate overflow since operands are promoted to int before subtraction.
static inline uint8_t seqn_sub(uint8_t a, uint8_t b, uint8_t mask) {
    return (uint8_t) ((a - b) & mask);
}

// seqn_in_window: test whether sequence number n lies in the half-open window
// [low, high) modulo (mask+1).  Used to validate received N(R) and N(S).
// Works correctly across the modular wrap-around point.
// Returns 1 if n is inside the window, 0 otherwise.
static inline uint8_t seqn_in_window(uint8_t n, uint8_t low, uint8_t high, uint8_t mask) {
    // Translate both n and high to offsets from low; n is in-window iff its
    // offset is strictly less than the window width.
    uint8_t offset_n = seqn_sub(n, low, mask);
    uint8_t offset_high = seqn_sub(high, low, mask);
    return (offset_n < offset_high) ? 1u : 0u;
}

// seqn_window_full: returns 1 if the transmit window is exhausted and no more
// I-frames may be sent.
// Condition: (V(S) - V(A)) mod modulo >= k   (AX.25 v2.2 §6.4.1)
// k is the negotiated window size stored in ax25_timers_t.k.
static inline uint8_t seqn_window_full(uint8_t vs, uint8_t va, uint8_t k, uint8_t mask) {
    return (seqn_sub(vs, va, mask) >= k) ? 1u : 0u;
}

/*============================================================================*/
/* XID Initialization Functions                                               */
/*============================================================================*/

/**
 * @brief Initialize default XID parameters
 *
 * Allocates and initializes global default XID parameters for both
 * AX.25 v2.0 and v2.2 compatibility. Must be called once during
 * program initialization before using XID functions.
 *
 * @section Default_Values
 * AX.25 v2.0 defaults:
 * - Half-duplex operation
 * - Implicit reject (REJ) only
 * - Modulo-8
 * - Window size: 4
 *
 * AX.25 v2.2 defaults:
 * - Half-duplex operation
 * - Selective reject-reject (SREJ/REJ)
 * - Modulo-8 (negotiable to 128)
 * - Window size: 7
 *
 * @param[out] err Error code: 0=success, non-zero=failure
 */
void ax25_xid_init_defaults(uint8_t *err);

/**
 * @brief Deinitialize default XID parameters
 *
 * Frees all resources allocated by ax25_xid_init_defaults().
 * Must be called at program termination to prevent memory leaks.
 *
 * @param[out] err Error code: 0=success, non-zero=failure (first error if multiple)
 */
void ax25_xid_deinit_defaults(uint8_t *err);

/*============================================================================*/
/* Frame Encode/Decode Functions                                                */
/*============================================================================*/

/**
 * @brief Encode an AX.25 frame to binary format
 *
 * Serializes a complete AX.25 frame including address field, control field,
 * PID (if applicable), and payload into a binary buffer suitable for
 * HDLC framing.
 *
 * @section Encoding_Process
 * 1. Encodes address field (destination, source, repeaters)
 * 2. Encodes appropriate control field based on frame type
 * 3. Adds PID for I-frames and UI frames
 * 4. Copies payload data
 *
 * @param[in]  frame Pointer to frame structure to encode
 * @param[out] len   Pointer to store encoded length
 * @param[out] err   Error code: 0=success, 1=malloc fail, 2=invalid type, 3=payload fail, 4=result malloc fail
 * @return Pointer to encoded buffer (caller must free), or NULL on error
 */
uint8_t* ax25_frame_encode(const ax25_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Decode binary data to AX.25 frame structure
 *
 * Parses binary AX.25 frame data and creates appropriate frame structure
 * based on control field analysis and modulo setting.
 *
 * @section Decoding_Logic
 * - MODULO128_NONE: Returns raw frame without parsing control
 * - MODULO128_FALSE: Forces 8-bit control interpretation
 * - MODULO128_TRUE: Forces 16-bit control interpretation
 * - MODULO128_AUTO: Auto-detects from res1 bit or frame length
 *
 * @param[in]  data       Binary frame data (no FCS)
 * @param[in]  len        Length of input data
 * @param[in]  modulo128  Modulo mode selection constant
 * @param[out] err        Error code: 0=success, 1=too short, 2=invalid addr, 3=no control, 4=malloc fail, 5=invalid addr field, 6=invalid control
 * @return Pointer to decoded frame (caller must free with ax25_frame_free), or NULL on error
 */
ax25_frame_t* ax25_frame_decode(const uint8_t *data, size_t len, int modulo128, uint8_t *err);

// ax25_encode_frame_to_buf: zero-malloc control-frame encoder.
// Encodes frame directly into buf[0..buf_size-1].
// On success writes byte count to *out_len and returns 0.
// Returns: 1=NULL arg, 2=buf too small, 3=unsupported type (use ax25_frame_encode for I/XID/UI).
// NOT suitable for frames stored for later retransmission; use ax25_frame_encode() for those.
uint8_t ax25_encode_frame_to_buf(const ax25_frame_t *frame, uint8_t *buf, size_t buf_size, size_t *out_len);

/**
 * @brief Free AX.25 frame structure and associated resources
 *
 * Deallocates frame structure and all internally allocated memory
 * including payloads and nested structures.
 *
 * @param[in,out] frame Frame to free (pointer becomes invalid)
 * @param[out]    err   Error code: 0=success, 1=NULL frame
 */
void ax25_frame_free(ax25_frame_t *frame, uint8_t *err);

/**
 * @brief Create new AX.25 frame with specified type
 *
 * Allocates and initializes frame structure with given type and header.
 * Caller must populate type-specific fields after creation.
 *
 * @param[in]  type   Frame type to create
 * @param[in]  header Header information (copied to frame)
 * @param[out] err    Error code: 0=success, 1=invalid header, 2=malloc fail, 3=unsupported type
 * @return Pointer to new frame, or NULL on error
 */
ax25_frame_t* ax25_frame_create(ax25_frame_type_t type, const ax25_frame_header_t *header, uint8_t *err);

/*============================================================================*/
/* Address Functions                                                            */
/*============================================================================*/

/**
 * @brief Encode AX.25 address to 7-byte binary format
 *
 * Converts address structure to on-air format per AX.25 v2.2 Section 6.2:
 * - Callsign characters shifted left 1 bit
 * - SSID byte with control bits
 *
 * @param[in]  addr Address to encode
 * @param[out] len  Pointer to store encoded length (always 7)
 * @param[out] err  Error code: 0=success, 1=malloc fail, 2=NULL addr
 * @return Pointer to 7-byte encoded buffer (caller must free), or NULL on error
 */
uint8_t* ax25_address_encode(const ax25_address_t *addr, size_t *len, uint8_t *err);

/**
 * @brief Decode 7-byte binary data to AX.25 address
 *
 * Parses on-air address format extracting callsign, SSID, and control bits.
 *
 * @param[in]  data 7-byte binary address data
 * @param[out] err  Error code: 0=success, 1=malloc fail, 2=NULL data
 * @return Pointer to decoded address (caller must free), or NULL on error
 */
ax25_address_t* ax25_address_decode(const uint8_t *data, uint8_t *err);

/**
 * @brief Create AX.25 address from string representation
 *
 * Parses callsign string in format "CALLSIGN-SSID*" where:
 * - CALLSIGN: 1-6 alphanumeric characters
 * - SSID: 0-15 (optional, defaults to 0)
 * - *: Indicates C/H bit set (optional)
 *
 * @param[in]  str Callsign string (e.g., "N0CALL-7" or "REPEATER-1*")
 * @param[out] err Error code: 0=success, 1=malloc fail, 2=NULL str, 4=invalid format, 5=invalid chars, 6=asterisk position error
 * @return Pointer to address structure (caller must free), or NULL on error
 */
ax25_address_t* ax25_address_from_string(const char *str, uint8_t *err);

/**
 * @brief Create deep copy of AX.25 address
 *
 * Duplicates address structure including all fields.
 *
 * @param[in]  addr Address to copy
 * @param[out] err  Error code: 0=success, 1=malloc fail
 * @return Pointer to copied address (caller must free), or NULL on error
 */
ax25_address_t* ax25_address_copy(const ax25_address_t *addr, uint8_t *err);

/**
 * @brief Free AX.25 address structure
 *
 * @param[in,out] addr Address to free
 * @param[out]    err  Error code: 0=success
 */
void ax25_address_free(ax25_address_t *addr, uint8_t *err);

/*============================================================================*/
/* Path Functions                                                               */
/*============================================================================*/

/**
 * @brief Create new repeater path from address array
 *
 * Allocates path structure containing copies of repeater addresses.
 *
 * @param[in]  repeaters Array of pointers to repeater addresses
 * @param[in]  num       Number of repeaters (1-8)
 * @param[out] err       Error code: 0=success, 1=malloc fail, 2=invalid input
 * @return Pointer to path structure (caller must free), or NULL on error
 */
ax25_path_t* ax25_path_new(ax25_address_t **repeaters, int num, uint8_t *err);

/**
 * @brief Free repeater path structure
 *
 * @param[in,out] path Path to free
 * @param[out]    err  Error code: 0=success
 */
void ax25_path_free(ax25_path_t *path, uint8_t *err);

/*============================================================================*/
/* Frame Header Functions                                                       */
/*============================================================================*/

/**
 * @brief Decode AX.25 frame header from binary data
 *
 * Parses complete address field including destination, source, and
 * optional repeater path. Validates extension bit termination.
 *
 * @param[in]  data Binary frame data starting at address field
 * @param[in]  len  Length of available data
 * @param[out] err  Error code: 0=success, 4=too few addresses, 5=no extension bit, 6=malloc fail, 7=invalid SSID
 * @return Header decode result structure (header pointer NULL on failure)
 */
header_decode_result_t ax25_frame_header_decode(const uint8_t *data, size_t len, uint8_t *err);

/**
 * @brief Encode AX.25 frame header to binary format
 *
 * Serializes address field with proper extension bit settings.
 *
 * @param[in]  header Header to encode
 * @param[out] len    Pointer to store encoded length
 * @param[out] err    Error code: 0=success, 1=malloc fail
 * @return Pointer to encoded buffer (caller must free), or NULL on error
 */
uint8_t* ax25_frame_header_encode(const ax25_frame_header_t *header, size_t *len, uint8_t *err);

/**
 * @brief Free frame header structure
 *
 * @param[in,out] header Header to free
 * @param[out]    err    Error code: 0=success
 */
void ax25_frame_header_free(ax25_frame_header_t *header, uint8_t *err);

/*============================================================================*/
/* Raw Frame Functions                                                          */
/*============================================================================*/

/**
 * @brief Encode raw frame payload
 *
 * Encodes control byte and raw payload without protocol interpretation.
 *
 * @param[in]  frame Raw frame to encode
 * @param[out] len   Pointer to store encoded length
 * @param[out] err    Error code: 0=success, 1=malloc fail
 * @return Pointer to encoded buffer (control + payload), or NULL on error
 */
uint8_t* ax25_raw_frame_encode(const ax25_raw_frame_t *frame, size_t *len, uint8_t *err);

/*============================================================================*/
/* Unnumbered Frame Functions                                                   */
/*============================================================================*/

/**
 * @brief Decode unnumbered frame from binary data
 *
 * Interprets control byte modifier and dispatches to specific U-frame
 * decoder (UI, FRMR, XID, TEST, or simple U-frame).
 *
 * @param[in,out] header  Frame header (modified with CR bits)
 * @param[in]     control Control byte from frame
 * @param[in]     data    Data following control byte
 * @param[in]     len     Length of remaining data
 * @param[out]    err     Error code: 0=success, 1=malloc fail, 6=invalid modifier
 * @return Pointer to unnumbered frame structure, or NULL on error
 */
ax25_unnumbered_frame_t* ax25_unnumbered_frame_decode(ax25_frame_header_t *header, uint8_t control, const uint8_t *data, size_t len, uint8_t *err);

/**
 * @brief Encode unnumbered frame control byte
 *
 * @param[in]  frame U-frame to encode
 * @param[out] len   Pointer to store encoded length (always 1)
 * @param[out] err    Error code: 0=success, 1=malloc fail
 * @return Pointer to 1-byte control buffer, or NULL on error
 */
uint8_t* ax25_unnumbered_frame_encode(const ax25_unnumbered_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Decode UI frame from binary data
 *
 * Extracts PID and payload from UI frame data.
 *
 * @param[in,out] header Frame header
 * @param[in]     pf     Poll/Final bit from control byte
 * @param[in]     data   Data following control byte (starts with PID)
 * @param[in]     len    Length of data
 * @param[out]    err    Error code: 0=success, 1=malloc fail or too short
 * @return Pointer to UI frame structure, or NULL on error
 */
ax25_unnumbered_information_frame_t* ax25_unnumbered_information_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len,
        uint8_t *err);

/**
 * @brief Encode UI frame to binary format
 *
 * @param[in]  frame UI frame to encode
 * @param[out] len   Pointer to store encoded length
 * @param[out] err   Error code: 0=success, 1=malloc fail
 * @return Pointer to encoded buffer (control + PID + payload), or NULL on error
 */
uint8_t* ax25_unnumbered_information_frame_encode(const ax25_unnumbered_information_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Decode FRMR frame from binary data
 *
 * Parses FRMR info field extracting rejection reason and state variables.
 * Handles both modulo-8 (3-byte) and modulo-128 (5-byte) formats.
 *
 * @param[in,out] header Frame header
 * @param[in]     pf     Poll/Final bit
 * @param[in]     data   FRMR info field data
 * @param[in]     len    Length of info field (3 or 5 bytes)
 * @param[out]    err    Error code: 0=success, 1=invalid length, 2=malloc fail
 * @return Pointer to FRMR frame structure, or NULL on error
 */
ax25_frame_reject_frame_t* ax25_frame_reject_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len, uint8_t *err);

/**
 * @brief Encode FRMR frame to binary format
 *
 * @param[in]  frame FRMR frame to encode
 * @param[out] len   Pointer to store encoded length (4 or 6 bytes)
 * @param[out] err    Error code: 0=success, 1=malloc fail
 * @return Pointer to encoded buffer, or NULL on error
 */
uint8_t* ax25_frame_reject_frame_encode(const ax25_frame_reject_frame_t *frame, size_t *len, uint8_t *err);

/*============================================================================*/
/* Information Frame Functions                                                  */
/*============================================================================*/

/**
 * @brief Decode I-frame from binary data
 *
 * Extracts sequence numbers, PID, and payload from I-frame data.
 *
 * @param[in,out] header   Frame header
 * @param[in]     control  Control field value (8 or 16 bits)
 * @param[in]     data     Data following control field (starts with PID)
 * @param[in]     len      Length of remaining data
 * @param[in]     is_16bit True if 16-bit control (modulo-128)
 * @param[out]    err      Error code: 0=success, 1=malloc fail, 2=too short, 3=payload malloc fail
 * @return Pointer to I-frame structure, or NULL on error
 */
ax25_information_frame_t* ax25_information_frame_decode(ax25_frame_header_t *header, uint16_t control, const uint8_t *data, size_t len, bool is_16bit,
        uint8_t *err);

/**
 * @brief Encode I-frame to binary format
 *
 * @param[in]  frame I-frame to encode
 * @param[out] len   Pointer to store encoded length
 * @param[out] err   Error code: 0=success, 1=malloc fail
 * @return Pointer to encoded buffer (control + PID + payload), or NULL on error
 */
uint8_t* ax25_information_frame_encode(const ax25_information_frame_t *frame, size_t *len, uint8_t *err);

/*============================================================================*/
/* Supervisory Frame Functions                                                  */
/*============================================================================*/

/**
 * @brief Decode S-frame from binary data
 *
 * Extracts supervisory code, N(R), and P/F bit from control field.
 *
 * @param[in,out] header   Frame header
 * @param[in]     control  Control field value (8 or 16 bits)
 * @param[in]     is_16bit True if 16-bit control (modulo-128)
 * @param[out]    err      Error code: 0=success, 1=invalid code, 2=malloc fail
 * @return Pointer to S-frame structure, or NULL on error
 */
ax25_supervisory_frame_t* ax25_supervisory_frame_decode(ax25_frame_header_t *header, uint16_t control, bool is_16bit, uint8_t *err);

/**
 * @brief Encode S-frame to binary format
 *
 * @param[in]  frame S-frame to encode
 * @param[out] len   Pointer to store encoded length (1 or 2 bytes)
 * @param[out] err   Error code: 0=success, 1=malloc fail
 * @return Pointer to encoded buffer, or NULL on error
 */
uint8_t* ax25_supervisory_frame_encode(const ax25_supervisory_frame_t *frame, size_t *len, uint8_t *err);

/*============================================================================*/
/* XID Parameter Functions                                                      */
/*============================================================================*/

/**
 * @brief Create raw XID parameter
 *
 * Allocates XID parameter with specified PI and raw value data.
 *
 * @param[in]  pi     Parameter Identifier
 * @param[in]  pv     Parameter value data (can be NULL for zero-length)
 * @param[in]  pv_len Length of parameter value (0-255)
 * @param[out] err    Error code: 0=success, 1=pv too long, 2=malloc fail, 3=data malloc fail
 * @return Pointer to XID parameter, or NULL on error
 */
ax25_xid_parameter_t* ax25_xid_raw_parameter_new(int pi, const uint8_t *pv, size_t pv_len, uint8_t *err);

/**
 * @brief Encode XID parameter to binary format
 *
 * Serializes parameter as [PI][PL][PV] triplet.
 *
 * @param[in]  param Parameter to encode
 * @param[out] len   Pointer to store encoded length (2 + pv_len)
 * @param[out] err   Error code: 0=success, 1=malloc fail
 * @return Pointer to encoded buffer, or NULL on error
 */
uint8_t* ax25_xid_raw_parameter_encode(const ax25_xid_parameter_t *param, size_t *len, uint8_t *err);

/**
 * @brief Create deep copy of XID parameter
 *
 * @param[in]  param Parameter to copy
 * @param[out] err   Error code: 0=success
 * @return Pointer to copied parameter, or NULL on error
 */
ax25_xid_parameter_t* ax25_xid_raw_parameter_copy(const ax25_xid_parameter_t *param, uint8_t *err);

/**
 * @brief Free XID parameter and associated data
 *
 * @param[in,out] param Parameter to free
 * @param[out]    err   Error code: 0=success, 1=NULL param
 */
void ax25_xid_raw_parameter_free(ax25_xid_parameter_t *param, uint8_t *err);

/**
 * @brief Decode XID parameter from binary data
 *
 * Parses PI, PL, and PV from data buffer.
 *
 * @param[in]  data     Binary parameter data
 * @param[in]  len      Length of available data
 * @param[out] consumed Pointer to store bytes consumed (2 + PL)
 * @param[out] err      Error code: 0=success, 1=too short, 2=PL exceeds data, 3=malloc fail
 * @return Pointer to decoded parameter, or NULL on error
 */
ax25_xid_parameter_t* ax25_xid_parameter_decode(const uint8_t *data, size_t len, size_t *consumed, uint8_t *err);

/**
 * @brief Decode complete XID frame from binary data
 *
 * Parses FI, GI, GL, and all parameter fields.
 *
 * @param[in,out] header Frame header
 * @param[in]     pf     Poll/Final bit
 * @param[in]     data   Data following control byte (XID info field)
 * @param[in]     len    Length of XID info field
 * @param[out]    err    Error code: 0=success, 1=too short, 2=length mismatch, 3=param decode fail, 4=realloc fail, 5=frame malloc fail
 * @return Pointer to XID frame structure, or NULL on error
 */
ax25_exchange_identification_frame_t* ax25_exchange_identification_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len,
        uint8_t *err);

/**
 * @brief Encode XID frame to binary format
 *
 * @param[in]  frame XID frame to encode
 * @param[out] len   Pointer to store encoded length
 * @param[out] err   Error code: 0=success, 1=malloc fail
 * @return Pointer to encoded buffer (control + FI + GI + GL + parameters), or NULL on error
 */
uint8_t* ax25_exchange_identification_frame_encode(const ax25_exchange_identification_frame_t *frame, size_t *len, uint8_t *err);

/*============================================================================*/
/* Test Frame Functions                                                         */
/*============================================================================*/

/**
 * @brief Decode TEST frame from binary data
 *
 * @param[in,out] header Frame header
 * @param[in]     pf     Poll/Final bit
 * @param[in]     data   Test payload data
 * @param[in]     len    Length of payload
 * @param[out]    err    Error code: 0=success, 1=malloc fail, 2=payload malloc fail
 * @return Pointer to TEST frame structure, or NULL on error
 */
ax25_test_frame_t* ax25_test_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len, uint8_t *err);

/**
 * @brief Encode TEST frame to binary format
 *
 * @param[in]  frame TEST frame to encode
 * @param[out] len   Pointer to store encoded length (1 + payload_len)
 * @param[out] err   Error code: 0=success, 1=malloc fail
 * @return Pointer to encoded buffer, or NULL on error
 */
uint8_t* ax25_test_frame_encode(const ax25_test_frame_t *frame, size_t *len, uint8_t *err);

/*============================================================================*/
/* XID Convenience Constructors                                                 */
/*============================================================================*/

/**
 * @brief Create Class of Procedures XID parameter
 *
 * Constructs PI=2 parameter negotiating operational mode.
 *
 * @section COP_Bits
 * Byte 0 bits:
 * - 0x01 (a): Half-duplex
 * - 0x02 (b): Full-duplex
 * - 0x04 (c): Reserved
 * - 0x08 (d): Reserved
 * - 0x10 (e): Reserved
 * - 0x20 (f): Reserved
 * - 0x40 (g): Reserved
 * - 0x80: Reserved
 *
 * @param[in]  a_flag    Half-duplex flag
 * @param[in]  b_flag    Full-duplex flag
 * @param[in]  c_flag    Reserved
 * @param[in]  d_flag    Reserved
 * @param[in]  e_flag    Reserved
 * @param[in]  f_flag    Reserved
 * @param[in]  g_flag    Reserved
 * @param[in]  reserved  Reserved byte value
 * @param[out] err       Error code
 * @return Pointer to XID parameter, or NULL on error
 */
ax25_xid_parameter_t* ax25_xid_class_of_procedures_new(bool a_flag, bool b_flag, bool c_flag, bool d_flag, bool e_flag, bool f_flag, bool g_flag,
        uint8_t reserved, uint8_t *err);

/**
 * @brief Create HDLC Optional Functions XID parameter
 *
 * Constructs PI=3 parameter negotiating protocol features.
 *
 * @section HDLC_Opt_Bits
 * Byte 0: RNR(0x01), REJ(0x02), SREJ(0x04), SABM(0x08), SABME(0x10), DM(0x20), DISC(0x40), UA(0x80)
 * Byte 1: FRMR(0x01), UI(0x02), XID(0x04), TEST(0x08), MOD8(0x10), MOD128(0x20)
 *
 * @param[in]  rnr       Receiver Not Ready supported
 * @param[in]  rej       Implicit Reject supported
 * @param[in]  srej      Selective Reject supported
 * @param[in]  sabm      SABM supported
 * @param[in]  sabme     SABME supported
 * @param[in]  dm        DM supported
 * @param[in]  disc      DISC supported
 * @param[in]  ua        UA supported
 * @param[in]  frmr      FRMR supported
 * @param[in]  ui        UI supported
 * @param[in]  xid       XID supported
 * @param[in]  test      TEST supported
 * @param[in]  modulo8   Modulo-8 supported
 * @param[in]  modulo128 Modulo-128 supported
 * @param[in]  res1-res7 Reserved flags
 * @param[in]  reserved  Reserved byte
 * @param[in]  ext       Extension bit
 * @param[out] err       Error code
 * @return Pointer to XID parameter, or NULL on error
 */
ax25_xid_parameter_t* ax25_xid_hdlc_optional_functions_new(bool rnr, bool rej, bool srej, bool sabm, bool sabme, bool dm, bool disc, bool ua, bool frmr,
bool ui, bool xid, bool test, bool modulo8, bool modulo128, bool res1, bool res2, bool res3, bool res4, bool res5, bool res6, bool res7, uint8_t reserved,
bool ext, uint8_t *err);

/**
 * @brief Create big-endian integer XID parameter
 *
 * Constructs parameter with integer value encoded in big-endian format.
 * Used for PI=6 (N1), PI=8 (k), PI=9 (T1), PI=10 (N2), PI=11 (T2).
 *
 * @param[in]  pi     Parameter Identifier
 * @param[in]  value  Integer value to encode
 * @param[in]  length Number of bytes (1, 2, or 4)
 * @param[out] err    Error code: 0=success, 1=encode fail
 * @return Pointer to XID parameter, or NULL on error
 */
ax25_xid_parameter_t* ax25_xid_big_endian_new(int pi, uint32_t value, size_t length, uint8_t *err);

/*============================================================================*/
/* Segmentation Functions                                                       */
/*============================================================================*/

/**
 * @brief Segment payload into multiple info fields
 *
 * Splits large payload into segments suitable for individual I-frames.
 * Per AX.25 v2.2 Appendix C6.
 *
 * @section Segment_Structure
 * First segment: [PID=0x08][Control=0x80|seq][TotalLength 2bytes][data...]
 * Other segments: [PID=0x08][Control=0x00/0x40|seq][data...]
 *
 * @param[in]  payload      Data to segment
 * @param[in]  payload_len  Length of payload (max 65535)
 * @param[in]  n1           Maximum info field size per segment
 * @param[out] err          Error code: 0=success, 1=invalid input, 2=n1 too small, 3=calculation error, 4=overflow, 5=malloc fail, 6=realloc fail, 7=too many segments
 * @param[out] num_segments Pointer to store number of segments created
 * @return Pointer to array of segments (caller must free with ax25_free_segmented_info), or NULL on error
 */
ax25_segmented_info_t* ax25_segment_info_fields(const uint8_t *payload, size_t payload_len, size_t n1, uint8_t *err, size_t *num_segments);

/**
 * @brief Reassemble payload from segmented info fields
 *
 * Combines multiple segments into original payload. Validates sequence
 * numbers and completeness.
 *
 * @param[in]  info_fields      Array of segmented info fields
 * @param[in]  num_info_fields  Number of segments
 * @param[out] reassembled_len  Pointer to store reassembled length
 * @param[out] err              Error code: 0=success, 1=malloc fail, 2=invalid format, 3=first segment error, 4=no first segment, 5=missing segment, 6=count mismatch, 7=malloc fail, 8=length mismatch
 * @return Pointer to reassembled payload (caller must free), or NULL on error
 */
uint8_t* ax25_reassemble_info_fields(ax25_segmented_info_t *info_fields, size_t num_info_fields, size_t *reassembled_len, uint8_t *err);

/**
 * @brief Free segmented info array
 *
 * @param[in,out] segments     Array to free
 * @param[in]     num_segments Number of segments in array
 */
void ax25_free_segmented_info(ax25_segmented_info_t *segments, size_t num_segments);

/*============================================================================*/
/* Utility Functions                                                            */
/*============================================================================*/

/**
 * @brief Determine if modulo-128 is used based on SABME/response exchange
 *
 * @param[in] sabme  SABME frame sent
 * @param[in] response Response frame received (UA, DM, or FRMR)
 * @return true if modulo-128 negotiated, false for modulo-8
 */
bool is_modulo128_used(ax25_frame_t *sabme, ax25_frame_t *response);

/**
 * @brief Validate frame meets minimum size requirement
 *
 * Per AX.25 v2.2 Section 3.9: minimum 136 bits (17 bytes)
 * Minimum without FCS: Dest(7) + Source(7) + Control(1) = 15 bytes
 * Minimum with FCS: 15 + FCS(2) = 17 bytes
 *
 * @param[in] frame_len Frame length in bytes
 * @return true if frame meets minimum size, false otherwise
 */
bool ax25_validate_frame_size(size_t frame_len);

/**
 * @brief Validate address field structure
 *
 * Checks per AX.25 v2.2 Section 3.12:
 * - At least 2 addresses (destination + source)
 * - Maximum 10 addresses (dest + source + 8 repeaters)
 * - Proper extension bit termination
 * - Complete addresses (no truncation)
 *
 * @param[in]  addr_field      Pointer to address field data
 * @param[in]  len             Length of available data
 * @param[out] addr_field_len  Output: validated address field length
 * @return true if valid structure, false otherwise
 */
bool ax25_validate_address_field(const uint8_t *addr_field, size_t len, size_t *addr_field_len);

/**
 * @brief Validate SSID value range
 *
 * Per AX.25 v2.2 Section 3.12.2: SSID is 4-bit field (0-15)
 *
 * @param[in] ssid SSID value to validate
 * @return true if valid (0-15), false otherwise
 */
bool ax25_validate_ssid(int ssid);

/**
 * @brief Reverse repeater path for response frames
 *
 * Per AX.25 v2.2 Section 3.12.4: When responding through digipeaters,
 * the path must be reversed and H-bits cleared.
 *
 * Example: Received path DIGI1*,DIGI2*,DIGI3 becomes DIGI3,DIGI2,DIGI1
 *
 * @param[in,out] header Frame header containing path to reverse
 */
void ax25_reverse_repeater_path(ax25_frame_header_t *header);

/**
 * @brief Check if frame has been digipeated by specified station
 *
 * Examines repeater path for matching callsign with H-bit set.
 *
 * @param[in] header    Frame header to check
 * @param[in] our_call  Callsign to search for
 * @param[in] our_ssid  SSID to search for (0-15)
 * @return true if frame was digipeated by this station, false otherwise
 */
bool ax25_frame_digipeated_by(const ax25_frame_header_t *header, const char *our_call, uint8_t our_ssid);

/**
 * @brief Find next unused digipeater in path
 *
 * Per AX.25 v2.2 Section 3.12.3: Digipeaters process sequentially.
 * Returns index of first repeater with H-bit = 0.
 *
 * @param[in] header Frame header to search
 * @return Index of next unused digipeater (0-7), or -1 if all used or no path
 */
int8_t ax25_find_next_digi(const ax25_frame_header_t *header);

/**
 * @brief Simple digipeater frame forwarding function
 *
 * Implements basic digipeater operation per AX.25 v2.2 Section 3.12.3:
 * 1. Decodes frame
 * 2. Finds next unused digipeater slot
 * 3. If matches my_call/my_ssid, sets H-bit and retransmits
 *
 * @param[in,out] frame_data  Frame buffer (modified if digipeated)
 * @param[in]     len         Frame length
 * @param[in]     my_call     This digipeater's callsign
 * @param[in]     my_ssid     This digipeater's SSID
 * @param[in]     retransmit  Callback to transmit modified frame
 */
void ax25_digipeat_frame(uint8_t *frame_data, size_t len, const char *my_call, uint8_t my_ssid, void (*retransmit)(uint8_t*, size_t));

// ax25_get_h_bit: read the H (has-been-repeated) bit directly from a raw
// AX.25 frame byte buffer for a given digipeater slot index (0-based).
// Address layout: dest=offset 0, src=offset 7, digi_n=offset 14+(7*n).
// The H-bit is bit 7 (0x80) of the 7th (SSID) byte of the address field,
// matching ax25_address_decode() which does: addr->ch = (data[6] & 0x80) != 0.
// Returns 1 if H-bit is set, 0 if clear.
// Returns 0xFF on invalid arguments (NULL, index out of range, or buffer
// too short to contain the addressed digipeater slot).
uint8_t ax25_get_h_bit(const uint8_t *frame_buf, size_t frame_len, uint8_t digi_idx);

// ax25_set_h_bit: set the H-bit in a raw AX.25 frame byte buffer for a
// given digipeater slot index (0-based).  Modifies the buffer in-place.
// The H-bit is bit 7 (0x80) of the SSID byte, matching ax25_address_encode()
// which does: if (addr->ch) ssid_byte |= 0x80.
// No-op if arguments are invalid (NULL, index out of range, buffer too short).
void ax25_set_h_bit(uint8_t *frame_buf, size_t frame_len, uint8_t digi_idx);

/*============================================================================*/
/* PID Dispatch Table                                                           */
/*============================================================================*/

/*============================================================================*/
/* Information Field Buffer Pool                                               */
/*============================================================================*/

// static buffer pool for info-field payloads
// AX25_MAX_INFO: maximum information field size in bytes per AX.25 v2.2 section 6.7.2.1.
// Default N1 = 256 bytes. Override at compile time with -DAX25_MAX_INFO=<n>.
#ifndef AX25_MAX_INFO
#define AX25_MAX_INFO 256u
#endif

// AX25_POOL_SIZE: number of simultaneously active info-field buffers.
// Each slot holds one frame payload up to AX25_MAX_INFO bytes.
// Size the pool to the maximum number of frames that can be in flight
// concurrently (RX pipeline + TX window). Default 16 covers mod-128 with
// window size k=7 plus a generous RX margin.
#ifndef AX25_POOL_SIZE
#define AX25_POOL_SIZE 16u
#endif

// ax25_buf_t: one slot in the static info-field buffer pool.
// The data array is sized to AX25_MAX_INFO + 1 to allow a null terminator
// for string-safe access of text payloads without a separate allocation.
typedef struct {
    uint8_t data[AX25_MAX_INFO + 1u];  // payload bytes (+1 for null terminator)
    uint16_t len;                       // valid bytes in data[]
    uint8_t in_use;                    // 1 = slot allocated, 0 = free
} ax25_buf_t;

// ax25_buf_alloc: claim a free pool slot.
// Returns pointer to slot on success, NULL if pool is exhausted.
// Caller must call ax25_buf_free() when the payload is no longer needed.
ax25_buf_t* ax25_buf_alloc(void);

// ax25_buf_free: release a pool slot back to the free list.
// Safe to call with NULL (no-op).
void ax25_buf_free(ax25_buf_t *b);

// ax25_buf_pool_free_count: return the number of currently free pool slots.
// Useful for diagnostics and flow control decisions.
uint8_t ax25_buf_pool_free_count(void);

// Maximum number of simultaneously registered PID handlers.
// Fixed-size table: no dynamic allocation, safe for all MCU targets.
#ifndef AX25_MAX_PID_HANDLERS
#define AX25_MAX_PID_HANDLERS 8
#endif

// Callback type invoked when a frame with a matching PID is received.
// info: pointer to the information field payload (after PID byte).
// len:  length of the information field payload in bytes.
// ctx:  user-supplied context pointer registered with ax25_register_pid().
typedef void (*ax25_pid_handler_fn)(const uint8_t *info, uint16_t len, void *ctx);

// Single entry in the PID dispatch table.
typedef struct {
    uint8_t pid;  // PID value this entry handles
    ax25_pid_handler_fn fn;  // handler callback
    void *ctx;  // user context passed to fn
} ax25_pid_entry_t;

// PID dispatch function declarations

// ax25_register_pid: register a handler for a specific PID value.
// Duplicate PID registrations are silently ignored (first registration wins).
// Returns 0 on success, 1 if the table is full, 2 if fn is NULL.
uint8_t ax25_register_pid(uint8_t pid, ax25_pid_handler_fn fn, void *ctx);

// ax25_unregister_pid: remove a previously registered PID handler.
// Returns 0 on success, 1 if PID not found.
uint8_t ax25_unregister_pid(uint8_t pid);

// ax25_dispatch_pid: dispatch an I-frame or UI-frame payload to the
// registered handler for its PID value.
// pid:  PID byte extracted from the received frame.
// info: pointer to information field data (NOT including the PID byte).
// len:  length of information field data in bytes.
// Frames with PID=0x08 (PID_SEGMENTATION) are routed to the SAR handler
// if one has been registered; otherwise silently dropped.
// Frames with PID=0xFF (PID_ESCAPE) require a second PID byte; if the
// extended PID has a registered handler it is called with info advanced
// past the second PID byte.
// Returns 0 if a handler was called, 1 if no handler registered for PID.
uint8_t ax25_dispatch_pid(uint8_t pid, const uint8_t *info, uint16_t len);

// ax25_pid_handler_count: return the number of currently registered handlers.
uint8_t ax25_pid_handler_count(void);

// ax25_parse_ctrl: unified control-field decoder for both modulo modes.
// ctrl:   pointer to the first control field byte in the frame buffer.
// avail:  number of bytes available starting at ctrl (must be >= 1; >= 2 for mod-128 I/S).
// mod128: 1 = use extended 2-byte control field (SABME connection), 0 = 1-byte control.
// out:    filled on success; all fields zero-initialised before parsing so unused
//         fields (e.g. ns for an S-frame) are always 0 rather than garbage.
// Returns 0 on success, 1 if ctrl is NULL or out is NULL, 2 if avail is too small
// for the required control field width.
uint8_t ax25_parse_ctrl(ax25_ctrl_t *out, const uint8_t *ctrl, size_t avail, uint8_t mod128);

#endif /* AX25_H_ */
