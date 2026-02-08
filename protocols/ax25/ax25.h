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
 *
 */

#ifndef AX25_H_
#define AX25_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/**
 * @defgroup ControlFieldMasks Control Field Masks and Values
 * @{
 * Masks and values used to identify frame types in the control field of an AX.25 frame.
 */
#define CONTROL_I_MASK  0x01 ///< Mask for I-frame (bit 0 = 0)
#define CONTROL_I_VAL   0x00 ///< Value for I-frame
#define CONTROL_US_MASK 0x03 ///< Mask for S or U frames (bits 0-1)
#define CONTROL_S_VAL   0x01 ///< Value for S-frame (01)
#define CONTROL_U_VAL   0x03 ///< Value for U-frame (11)
/** @} */

/**
 * @defgroup PollFinalBits Poll/Final Bit Positions
 * @{
 * Positions of the Poll/Final bit in 8-bit and 16-bit control fields.
 */
#define POLL_FINAL_8BIT  0x10   ///< Poll/Final bit position in 8-bit control field
#define POLL_FINAL_16BIT 0x0100 ///< Poll/Final bit position in 16-bit control field
/** @} */

/**
 * @defgroup ModuloConstants Modulo Sequence Numbering Constants
 * @{
 * Constants to control sequence numbering mode (modulo 8 or modulo 128).
 */
#define MODULO128_NONE  -1 ///< No modulo 128 (use modulo 8)
#define MODULO128_FALSE 0  ///< Explicitly use modulo 8
#define MODULO128_TRUE  1  ///< Use modulo 128
#define MODULO128_AUTO  2  ///< Automatically determine modulo based on frame type
/** @} */

/**
 * @defgroup AddressConstants Address Field Constants
 * @{
 * Constants defining the maximum number of repeaters and callsign length.
 */
#define MAX_REPEATERS 8    ///< Maximum number of repeaters in path
#define CALLSIGN_MAX 7     ///< Maximum length of callsign (6 chars + null)
/** @} */

/**
 * @defgroup PIDCodes Protocol Identifier (PID) Codes
 * @{
 * Protocol Identifiers for layer 3 protocols as per AX.25 specification (Section 6.5).
 */
#define PID_ISO8208_CCITT   0x01 ///< ISO 8208/CCITT X.25 PLP
#define PID_VJ_IP4_COMPRESS 0x06 ///< Compressed TCP/IP (Van Jacobson, RFC 1144)
#define PID_VJ_IP4          0x07 ///< Uncompressed TCP/IP (Van Jacobson, RFC 1144)
#define PID_SEGMENTATION    0x08 ///< Segmentation fragment
#define PID_TEXNET          0xC3 ///< TEXNET datagram protocol
#define PID_LINKQUALITY     0xC4 ///< Link Quality Protocol
#define PID_APPLETALK       0xCA ///< Appletalk
#define PID_APPLETALK_ARP   0xCB ///< Appletalk ARP
#define PID_ARPA_IP4        0xCC ///< ARPA Internet Protocol
#define PID_APRA_ARP        0xCD ///< ARPA Address resolution
#define PID_FLEXNET         0xCE ///< FlexNet
#define PID_NETROM          0xCF ///< NET/ROM
#define PID_NO_L3           0xF0 ///< No layer 3 protocol
#define PID_ESCAPE          0xFF ///< Escape character for extended PID
/** @} */

/**
 * @brief Enumeration of AX.25 frame types.
 *
 * Defines all possible frame types in the AX.25 protocol, including raw frames,
 * unnumbered frames (e.g., UI, SABM), information frames (I-frames), and supervisory
 * frames (e.g., RR, RNR). Distinctions are made for 8-bit and 16-bit control fields,
 * corresponding to modulo 8 or modulo 128 sequence numbering (Section 6.3).
 */
typedef enum {
    AX25_FRAME_RAW,                    ///< Raw frame, unparsed control field
    AX25_FRAME_UNNUMBERED_INFORMATION,  ///< Unnumbered Information (UI) frame
    AX25_FRAME_UNNUMBERED_SABM,        ///< Set Asynchronous Balanced Mode frame
    AX25_FRAME_UNNUMBERED_SABME,       ///< Set Asynchronous Balanced Mode Extended frame
    AX25_FRAME_UNNUMBERED_DISC,        ///< Disconnect frame
    AX25_FRAME_UNNUMBERED_DM,          ///< Disconnected Mode frame
    AX25_FRAME_UNNUMBERED_UA,          ///< Unnumbered Acknowledge frame
    AX25_FRAME_UNNUMBERED_FRMR,        ///< Frame Reject frame
    AX25_FRAME_UNNUMBERED_XID,         ///< Exchange Identification frame
    AX25_FRAME_UNNUMBERED_TEST,        ///< Test frame
    AX25_FRAME_INFORMATION_8BIT,       ///< Information frame with 8-bit control field (modulo 8)
    AX25_FRAME_INFORMATION_16BIT,      ///< Information frame with 16-bit control field (modulo 128)
    AX25_FRAME_SUPERVISORY_RR_8BIT,    ///< Receive Ready supervisory frame (8-bit, modulo 8)
    AX25_FRAME_SUPERVISORY_RNR_8BIT,   ///< Receive Not Ready supervisory frame (8-bit, modulo 8)
    AX25_FRAME_SUPERVISORY_REJ_8BIT,   ///< Reject supervisory frame (8-bit, modulo 8)
    AX25_FRAME_SUPERVISORY_SREJ_8BIT,  ///< Selective Reject supervisory frame (8-bit, modulo 8)
    AX25_FRAME_SUPERVISORY_RR_16BIT,   ///< Receive Ready supervisory frame (16-bit, modulo 128)
    AX25_FRAME_SUPERVISORY_RNR_16BIT,  ///< Receive Not Ready supervisory frame (16-bit, modulo 128)
    AX25_FRAME_SUPERVISORY_REJ_16BIT,  ///< Reject supervisory frame (16-bit, modulo 128)
    AX25_FRAME_SUPERVISORY_SREJ_16BIT  ///< Selective Reject supervisory frame (16-bit, modulo 128)
} ax25_frame_type_t;

/**
 * @brief Structure to hold segmented information fields for AX.25 frames.
 *
 * Stores the information field of a segmented frame, part of a larger payload
 * split across multiple frames per AX.25 segmentation rules (Section 6.9).
 *
 * @var uint8_t *info_field
 * Pointer to the byte array containing the segmented information field data.
 *
 * @var size_t info_field_len
 * Length of the info_field in bytes.
 */
typedef struct {
    uint8_t *info_field;    ///< Segmented information field data
    size_t info_field_len;  ///< Length of the info_field in bytes
} ax25_segmented_info_t;

/**
 * @brief Internal structure for reassembly of segmented AX.25 frames.
 *
 * Holds information needed to reassemble a complete payload from multiple
 * segmented frames, used internally during frame processing (Section 6.9).
 *
 * @var uint8_t control
 * Control byte of the frame, indicating frame type and sequence information.
 *
 * @var uint16_t total_length
 * Total length of the complete payload before segmentation.
 *
 * @var uint8_t *data
 * Pointer to the byte array containing the segment data.
 *
 * @var size_t data_len
 * Length of the data in bytes for this segment.
 *
 * @var int segment_number
 * Sequence number of this segment, used to order segments during reassembly.
 */
typedef struct {
    uint8_t control;        ///< Control byte of the segment
    uint16_t total_length;  ///< Total length of the original payload
    uint8_t *data;          ///< Segment data
    size_t data_len;        ///< Length of the segment data in bytes
    int segment_number;     ///< Segment sequence number
} ax25_reassembly_segment_t;

/**
 * @brief Structure representing an AX.25 address.
 *
 * Holds the components of an AX.25 address, including the callsign, SSID, and
 * control bits as per the AX.25 protocol specification (Section 6.2). Used for
 * destination, source, and repeater addresses.
 *
 * @var char callsign[CALLSIGN_MAX]
 * Callsign of the station, up to 6 characters, padded with spaces, null-terminated.
 *
 * @var int ssid
 * Secondary Station Identifier (SSID), 4-bit value (0-15) to distinguish stations.
 *
 * @var bool ch
 * Command/Response (C) bit for destination/source addresses; Has-Been-Repeated (H) bit for repeaters.
 *
 * @var bool res0
 * Reserved bit 0, typically set to 0 (Section 6.2).
 *
 * @var bool res1
 * Reserved bit 1, typically set to 0 (Section 6.2).
 *
 * @var bool extension
 * HDLC extension bit; set to 1 for the last address in the address field, 0 otherwise.
 */
typedef struct {
    char callsign[CALLSIGN_MAX];  ///< Callsign, up to 6 chars, null-terminated
    int ssid;                     ///< SSID, 4-bit value (0-15)
    bool ch;                      ///< C bit (dest/source) or H bit (repeater)
    bool res0;                    ///< Reserved bit 0, typically 0
    bool res1;                    ///< Reserved bit 1, typically 0
    bool mod128;                  ///< bit 6: false = modulo-128 frame, true = modulo-8 frame
    bool extension;               ///< HDLC extension bit (1 = last address)
} ax25_address_t;

/**
 * @brief Structure representing the path of repeaters in an AX.25 frame.
 *
 * Holds an array of repeater addresses and the count of repeaters, used for
 * digipeating in the AX.25 protocol (Section 6.2).
 *
 * @var ax25_address_t repeaters[MAX_REPEATERS]
 * Array of repeater addresses, up to MAX_REPEATERS (8).
 *
 * @var int num_repeaters
 * Number of repeaters in the path, capped at MAX_REPEATERS.
 */
typedef struct {
    ax25_address_t repeaters[MAX_REPEATERS];  ///< Array of repeater addresses
    int num_repeaters;                      ///< Number of repeaters (0 to 8)
} ax25_path_t;

/**
 * @brief Structure representing the header of an AX.25 frame.
 *
 * Contains destination and source addresses, repeater path, and command/response
 * flags, as defined in the AX.25 address field (Section 6.2).
 *
 * @var ax25_address_t destination
 * Destination address of the frame.
 *
 * @var ax25_address_t source
 * Source address of the frame.
 *
 * @var ax25_path_t repeaters
 * Path of repeaters for digipeating, if any.
 *
 * @var bool cr
 * Command/Response flag for the frame, derived from address fields.
 *
 * @var bool src_cr
 * Source Command/Response flag, used for specific frame processing.
 */
typedef struct {
    ax25_address_t destination;  ///< Destination address
    ax25_address_t source;      ///< Source address
    ax25_path_t repeaters;      ///< Repeater path
    bool cr;                    ///< Command/Response flag
    bool src_cr;                ///< Source Command/Response flag
} ax25_frame_header_t;

/**
 * @brief Base structure for all AX.25 frames.
 *
 * Serves as the base for all specific frame types, containing the frame type
 * and header (Section 6.3).
 *
 * @var ax25_frame_type_t type
 * Type of the frame (e.g., I-frame, S-frame, U-frame).
 *
 * @var ax25_frame_header_t header
 * Frame header with address information.
 */
typedef struct {
    ax25_frame_type_t type;     ///< Frame type
    ax25_frame_header_t header;  ///< Frame header
} ax25_frame_t;

/**
 * @brief Structure representing a raw AX.25 frame.
 *
 * Holds the base frame information, control byte, and raw payload, used when
 * the frame type is not further interpreted (Section 6.3).
 *
 * @var ax25_frame_t base
 * Base frame structure.
 *
 * @var uint8_t control
 * Control byte defining frame type and control bits.
 *
 * @var uint8_t *payload
 * Pointer to the raw payload data.
 *
 * @var size_t payload_len
 * Length of the payload in bytes.
 */
typedef struct {
    ax25_frame_t base;      ///< Base frame
    uint8_t control;        ///< Control byte
    uint8_t *payload;       ///< Raw payload data
    size_t payload_len;     ///< Payload length in bytes
} ax25_raw_frame_t;

/**
 * @brief Structure representing an unnumbered AX.25 frame.
 *
 * Holds the base frame information, poll/final bit, and modifier for unnumbered
 * frames (e.g., SABM, UI) (Section 6.3.3).
 *
 * @var ax25_frame_t base
 * Base frame structure.
 *
 * @var bool pf
 * Poll/Final bit for command/response signaling.
 *
 * @var uint8_t modifier
 * Modifier bits defining the specific unnumbered frame type (e.g., SABM, UI).
 */
typedef struct {
    ax25_frame_t base;    ///< Base frame
    bool pf;              ///< Poll/Final bit
    uint8_t modifier;     ///< Modifier bits for U-frame type
} ax25_unnumbered_frame_t;

/**
 * @brief Structure representing an Unnumbered Information (UI) frame.
 *
 * Extends the unnumbered frame with a Protocol Identifier (PID) and payload,
 * used for connectionless data transfer (Section 6.3.7).
 *
 * @var ax25_unnumbered_frame_t base
 * Base unnumbered frame structure.
 *
 * @var uint8_t pid
 * Protocol Identifier (e.g., 0xF0 for no layer 3).
 *
 * @var uint8_t *payload
 * Pointer to the payload data.
 *
 * @var size_t payload_len
 * Length of the payload in bytes.
 */
typedef struct {
    ax25_unnumbered_frame_t base;  ///< Base unnumbered frame
    uint8_t pid;                  ///< Protocol Identifier
    uint8_t *payload;             ///< Payload data
    size_t payload_len;           ///< Payload length in bytes
} ax25_unnumbered_information_frame_t;

/**
 * @brief Structure representing a Frame Reject (FRMR) frame.
 *
 * Holds information about a rejected frame, including control fields and sequence
 * numbers, used to indicate errors in received frames (Section 6.4.4).
 *
 * @var ax25_unnumbered_frame_t base
 * Base unnumbered frame structure.
 *
 * @var bool is_modulo128
 * True if modulo 128 sequence numbering is used, false for modulo 8.
 *
 * @var uint16_t frmr_control
 * Control field of the rejected frame (8 or 16 bits).
 *
 * @var int vs
 * Send sequence number of the rejected frame (3 bits for modulo 8, 7 bits for modulo 128).
 *
 * @var int vr
 * Receive sequence number of the rejected frame (3 bits for modulo 8, 7 bits for modulo 128).
 *
 * @var bool frmr_cr
 * Command/Response flag of the rejected frame.
 *
 * @var bool w, x, y, z
 * Flags indicating the reason for rejection (e.g., invalid control field).
 */
typedef struct {
    ax25_unnumbered_frame_t base;  ///< Base unnumbered frame
    bool is_modulo128;           ///< Modulo 128 sequence numbering flag
    uint16_t frmr_control;       ///< Control field of rejected frame
    int vs;                      ///< Send sequence number
    int vr;                      ///< Receive sequence number
    bool frmr_cr;                ///< Command/Response flag of rejected frame
    bool w, x, y, z;             ///< Rejection reason flags
} ax25_frame_reject_frame_t;

/**
 * @brief Structure representing an Information (I) frame.
 *
 * Holds the base frame information, sequence numbers, poll/final bit, PID, and
 * payload for I-frames, used for reliable data transfer (Section 6.3.1).
 *
 * @var ax25_frame_t base
 * Base frame structure.
 *
 * @var int nr
 * Receive sequence number (3 bits for modulo 8, 7 bits for modulo 128).
 *
 * @var bool pf
 * Poll/Final bit for flow control.
 *
 * @var int ns
 * Send sequence number (3 bits for modulo 8, 7 bits for modulo 128).
 *
 * @var uint8_t pid
 * Protocol Identifier (e.g., 0xCC for ARPA IP).
 *
 * @var uint8_t *payload
 * Pointer to the payload data.
 *
 * @var size_t payload_len
 * Length of the payload in bytes, up to 256 octets by default.
 */
typedef struct {
    ax25_frame_t base;    ///< Base frame
    int nr;               ///< Receive sequence number
    bool pf;              ///< Poll/Final bit
    int ns;               ///< Send sequence number
    uint8_t pid;          ///< Protocol Identifier
    uint8_t *payload;     ///< Payload data
    size_t payload_len;   ///< Payload length in bytes
} ax25_information_frame_t;

/**
 * @brief Structure representing a Supervisory (S) frame.
 *
 * Holds the base frame information, receive sequence number, poll/final bit, and
 * supervisory code (RR, RNR, REJ, SREJ) for flow control (Section 6.3.2).
 *
 * @var ax25_frame_t base
 * Base frame structure.
 *
 * @var int nr
 * Receive sequence number (3 bits for modulo 8, 7 bits for modulo 128).
 *
 * @var bool pf
 * Poll/Final bit for flow control.
 *
 * @var uint8_t code
 * Supervisory code (00=RR, 01=RNR, 10=REJ, 11=SREJ).
 */
typedef struct {
    ax25_frame_t base;  ///< Base frame
    int nr;            ///< Receive sequence number
    bool pf;           ///< Poll/Final bit
    uint8_t code;      ///< Supervisory code
} ax25_supervisory_frame_t;

/**
 * @brief Structure representing an XID parameter.
 *
 * Holds parameters for Exchange Identification (XID) frames, including the
 * parameter identifier and function pointers for encoding, copying, and freeing
 * (Section 4.3.3.7).
 *
 * @var int pi
 * Parameter Identifier (e.g., 2 for Class of Procedures, 3 for HDLC Optional Functions).
 *
 * @var uint8_t* (*encode)(const ax25_xid_parameter_t*, size_t*, uint8_t *err)
 * Function pointer to encode the parameter into a binary format.
 *
 * @var ax25_xid_parameter_t* (*copy)(const ax25_xid_parameter_t*, uint8_t *err)
 * Function pointer to create a deep copy of the parameter.
 *
 * @var void (*free)(ax25_xid_parameter_t*, uint8_t *err)
 * Function pointer to free the parameter and its data.
 *
 * @var void *data
 * Pointer to parameter-specific data (e.g., raw bytes or structured data).
 */
typedef struct AX25XIDParameter {
    int pi;                                    ///< Parameter Identifier
    uint8_t* (*encode)(const struct AX25XIDParameter*, size_t*, uint8_t *err);  ///< Encode function
    struct AX25XIDParameter* (*copy)(const struct AX25XIDParameter*, uint8_t *err);  ///< Copy function
    void (*free)(struct AX25XIDParameter*, uint8_t *err);  ///< Free function
    void *data;                                ///< Parameter-specific data
} ax25_xid_parameter_t;

/**
 * @brief Structure representing an Exchange Identification (XID) frame.
 *
 * Holds the base unnumbered frame, function identifier, group identifier, and
 * a list of parameters for XID frames (Section 4.3.3.7).
 *
 * @var ax25_unnumbered_frame_t base
 * Base unnumbered frame structure.
 *
 * @var uint8_t fi
 * Function Identifier, defining the XID frame's purpose.
 *
 * @var uint8_t gi
 * Group Identifier, grouping related parameters.
 *
 * @var ax25_xid_parameter_t **parameters
 * Array of pointers to XID parameters.
 *
 * @var size_t param_count
 * Number of parameters in the array.
 */
typedef struct {
    ax25_unnumbered_frame_t base;  ///< Base unnumbered frame
    uint8_t fi;                   ///< Function Identifier
    uint8_t gi;                   ///< Group Identifier
    ax25_xid_parameter_t **parameters;  ///< Array of XID parameters
    size_t param_count;           ///< Number of parameters
} ax25_exchange_identification_frame_t;

/**
 * @brief Structure representing a Test (TEST) frame.
 *
 * Holds the base unnumbered frame and test payload, used for testing link
 * connectivity (Section 6.3.3.8).
 *
 * @var ax25_unnumbered_frame_t base
 * Base unnumbered frame structure.
 *
 * @var uint8_t *payload
 * Pointer to the test payload data.
 *
 * @var size_t payload_len
 * Length of the payload in bytes.
 */
typedef struct {
    ax25_unnumbered_frame_t base;  ///< Base unnumbered frame
    uint8_t *payload;             ///< Test payload data
    size_t payload_len;           ///< Payload length in bytes
} ax25_test_frame_t;

/**
 * @brief Structure to hold raw parameter data for XID parameters.
 *
 * Stores the parameter value (PV) as a flexible byte array for XID parameters,
 * used to encode specific parameter data in XID frames (Section 4.3.3.7).
 *
 * @var uint8_t *pv
 * Pointer to the parameter value data.
 *
 * @var size_t pv_len
 * Length of the parameter value in bytes.
 */
typedef struct {
    uint8_t *pv;      ///< Parameter value data
    size_t pv_len;    ///< Length of parameter value in bytes
} ax25_raw_parameter_t;

/**
 * @brief Structure to hold the result of decoding an AX.25 frame header.
 *
 * Encapsulates the decoded header and the remaining data after decoding,
 * facilitating further processing of the frame (Section 6.2).
 *
 * @var ax25_frame_header_t *header
 * Pointer to the decoded header, or NULL on failure.
 *
 * @var const uint8_t *remaining
 * Pointer to the data following the header.
 *
 * @var size_t remaining_len
 * Length of the remaining data in bytes.
 */
typedef struct {
    ax25_frame_header_t *header;  ///< Pointer to the decoded header, or NULL on failure
    const uint8_t *remaining;     ///< Pointer to the data following the header
    size_t remaining_len;         ///< Length of the remaining data in bytes
} header_decode_result_t;

/**
 * @brief Structure to hold raw parameter data for XID parameters.
 *
 * This structure stores the parameter value (PV) for Exchange Identification (XID)
 * parameters in AX.25 frames, used to convey configuration data such as Class of
 * Procedures or HDLC Optional Functions (Section 4.3.3.7 of AX.25 2.2). The structure
 * uses a flexible array member to hold a variable-length byte array, requiring careful
 * memory allocation and management.
 *
 * @note The use of a flexible array member (`pv[]`) requires that this structure be
 * dynamically allocated with sufficient space for `pv_len` bytes following the `pv_len`
 * field. It must be allocated using `malloc` or similar, with the total size calculated
 * as `sizeof(ax25_raw_param_data_t) + pv_len`. This structure is non-portable in some
 * contexts due to the flexible array member; consider using `ax25_raw_parameter_t` with
 * a pointer and length for better portability.
 *
 * @var size_t pv_len
 * Length of the parameter value data in bytes. Specifies the size of the `pv` array.
 *
 * @var uint8_t pv[]
 * Flexible array member holding the raw parameter value data. The actual size is
 * determined by `pv_len`. This array contains the raw bytes of the parameter value
 * (PV) as defined in the XID parameter format [PI, PL, PV], where PV is the payload
 * specific to the parameter identifier (PI).
 */
typedef struct {
    size_t pv_len;
    uint8_t pv[];
} ax25_raw_param_data_t;

/**
 * @brief Initializes default XID parameters for AX.25 versions 2.0 and 2.2.
 *
 * Sets up global default XID parameters (e.g., Class of Procedures, HDLC Optional Functions)
 * for use in XID frames. Must be called once during program initialization (Section 4.3.3.7).
 *
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 */
void ax25_xid_init_defaults(uint8_t *err);

/**
 * @brief Deinitializes default XID parameters.
 *
 * Frees resources allocated for default XID parameters. Must be called at
 * program termination to prevent memory leaks (Section 4.3.3.7).
 *
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 */
void ax25_xid_deinit_defaults(uint8_t *err);

/**
 * @brief Encodes an AX.25 frame into a binary buffer.
 *
 * Converts an AX.25 frame structure into its binary representation, including
 * address, control, PID (if applicable), and payload fields (Section 6.1).
 *
 * @param frame Pointer to the AX.25 frame to encode.
 * @param len Pointer to a size_t where the length of the encoded buffer will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the encoded buffer (must be freed by caller).
 */
uint8_t* ax25_frame_encode(const ax25_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Decodes an AX.25 frame from binary data based on the specified modulo setting.
 *
 * Interprets the binary data as an AX.25 frame, determining its type (raw,
 * unnumbered, information, or supervisory) based on the control field and the modulo128
 * parameter (Section 6.1).
 *
 * @param data Pointer to the binary data containing the frame.
 * @param len Length of the input data in bytes.
 * @param modulo128 Integer flag controlling frame decoding:
 *                  - MODULO128_NONE (-1): Returns a raw frame with unparsed payload.
 *                  - MODULO128_FALSE (0): Decodes using 8-bit control field.
 *                  - MODULO128_TRUE (1): Decodes using 16-bit control field.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the decoded AX.25 frame (must be freed with ax25_frame_free).
 */
ax25_frame_t* ax25_frame_decode(const uint8_t *data, size_t len, int modulo128, uint8_t *err);

/**
 * @brief Frees an AX.25 frame and its associated resources.
 *
 * Deallocates memory for the frame and its payload, ensuring no memory leaks.
 * Must be called for frames returned by ax25_frame_decode or other creation
 * functions (Section 6.1).
 *
 * @param frame Pointer to the AX.25 frame to free.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 */
void ax25_frame_free(ax25_frame_t *frame, uint8_t *err);

/**
 * @brief Creates a new AX.25 frame with the specified type and header.
 *
 * Allocates and initializes a new AX.25 frame with the given type and header
 * information. Caller must set additional fields (e.g., payload, PID) as needed
 * (Section 6.1).
 *
 * @param type The type of frame to create (e.g., AX25_FRAME_INFORMATION_8BIT).
 * @param header The frame header containing address information.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the new AX.25 frame (must be freed with ax25_frame_free).
 */
ax25_frame_t* ax25_frame_create(ax25_frame_type_t type, const ax25_frame_header_t *header, uint8_t *err);

/**
 * @brief Encodes an AX.25 address into a 7-byte binary array.
 *
 * Serializes an AX.25 address into the 7-byte format, with callsign characters
 * shifted left by 1 and SSID/control bits in the seventh byte (Section 6.2).
 *
 * @param addr Pointer to the AX.25 address to encode.
 * @param len Pointer to a size_t where the length of the encoded data (always 7) will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the 7-byte encoded buffer (must be freed by caller).
 */
uint8_t* ax25_address_encode(const ax25_address_t *addr, size_t *len, uint8_t *err);

/**
 * @brief Decodes a 7-byte binary data segment into an AX.25 address.
 *
 * Extracts an AX.25 address from a 7-byte buffer, parsing the callsign (shifted right by 1)
 * and SSID/control bits (Section 6.2).
 *
 * @param data Pointer to the 7-byte binary data containing the encoded address.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the decoded AX.25 address (must be freed by caller).
 */
ax25_address_t* ax25_address_decode(const uint8_t *data, uint8_t *err);

/**
 * @brief Creates an AX.25 address from a string representation.
 *
 * Parses a string in the format "CALLSIGN-SSID*" (e.g., "N0CALL-7*") to create
 * an AX.25 address. The asterisk (*) indicates the 'ch' bit (Section 6.2).
 *
 * @param str Pointer to a null-terminated string containing the address.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the new AX.25 address (must be freed with ax25_address_free).
 */
ax25_address_t* ax25_address_from_string(const char *str, uint8_t *err);

/**
 * @brief Creates a deep copy of an AX.25 address.
 *
 * Duplicates an AX.25 address, allocating new memory and copying all fields,
 * ensuring the copy is independent of the original (Section 6.2).
 *
 * @param addr Pointer to the AX.25 address to copy.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the new AX.25 address (must be freed with ax25_address_free).
 */
ax25_address_t* ax25_address_copy(const ax25_address_t *addr, uint8_t *err);

/**
 * @brief Frees the memory allocated for an AX.25 address.
 *
 * Deallocates an AX.25 address structure to prevent memory leaks (Section 6.2).
 *
 * @param addr Pointer to the AX.25 address to free. If NULL, the function does nothing.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 */
void ax25_address_free(ax25_address_t *addr, uint8_t *err);

/**
 * @brief Creates a new AX.25 path structure containing repeater addresses.
 *
 * Allocates and initializes an AX.25 path with up to MAX_REPEATERS repeater
 * addresses, copying the provided addresses (Section 6.2).
 *
 * @param repeaters Array of pointers to AX.25 address structures for repeaters.
 * @param num Number of repeaters, capped at MAX_REPEATERS.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the new AX.25 path (must be freed with ax25_path_free).
 */
ax25_path_t* ax25_path_new(ax25_address_t **repeaters, int num, uint8_t *err);

/**
 * @brief Frees the memory allocated for an AX.25 path.
 *
 * Deallocates an AX.25 path structure. Does not free the individual addresses,
 * as they are assumed to be managed elsewhere (Section 6.2).
 *
 * @param path Pointer to the AX.25 path to free. If NULL, the function does nothing.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 */
void ax25_path_free(ax25_path_t *path, uint8_t *err);

/**
 * @brief Decodes an AX.25 frame header from binary data.
 *
 * Parses the address field of an AX.25 frame, extracting destination, source, and
 * repeater addresses, handling the extension bit to determine the field’s end (Section 6.2).
 *
 * @param data Pointer to the binary data containing the frame header.
 * @param len Length of the input data in bytes.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Structure containing the decoded header, remaining data, and its length.
 */
header_decode_result_t ax25_frame_header_decode(const uint8_t *data, size_t len, uint8_t *err);

/**
 * @brief Encodes an AX.25 frame header into a binary array.
 *
 * Serializes an AX.25 frame header, including destination, source, and repeater
 * addresses, setting the extension bit appropriately (Section 6.2).
 *
 * @param header Pointer to the AX.25 frame header to encode.
 * @param len Pointer to a size_t where the length of the encoded data will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the encoded buffer (must be freed by caller).
 */
uint8_t* ax25_frame_header_encode(const ax25_frame_header_t *header, size_t *len, uint8_t *err);

/**
 * @brief Frees the memory allocated for an AX.25 frame header.
 *
 * Deallocates an AX.25 frame header structure, including its nested addresses
 * (Section 6.2).
 *
 * @param header Pointer to the AX.25 frame header to free. If NULL, the function does nothing.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 */
void ax25_frame_header_free(ax25_frame_header_t *header, uint8_t *err);

/**
 * @brief Encodes a raw AX.25 frame’s payload into a binary array.
 *
 * Copies the raw frame’s control field and payload as-is, without further
 * interpretation (Section 6.3).
 *
 * @param frame Pointer to the raw AX.25 frame to encode.
 * @param len Pointer to a size_t where the length of the encoded payload will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the encoded buffer (must be freed by caller).
 */
uint8_t* ax25_raw_frame_encode(const ax25_raw_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Decodes an unnumbered AX.25 frame from binary data.
 *
 * Interprets an unnumbered frame based on its control byte modifier, creating
 * the appropriate subtype (e.g., UI, SABM, FRMR) (Section 6.3.3).
 *
 * @param header Pointer to the AX.25 frame header.
 * @param control The control byte from the frame data.
 * @param data Pointer to the remaining data after the control byte.
 * @param len Length of the remaining data in bytes.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the decoded unnumbered frame (must be freed with ax25_frame_free).
 */
ax25_unnumbered_frame_t* ax25_unnumbered_frame_decode(ax25_frame_header_t *header, uint8_t control, const uint8_t *data, size_t len, uint8_t *err);

/**
 * @brief Encodes an unnumbered AX.25 frame into a binary array.
 *
 * Serializes the control byte of an unnumbered frame, combining the modifier
 * with the poll/final bit (Section 6.3.3).
 *
 * @param frame Pointer to the unnumbered AX.25 frame to encode.
 * @param len Pointer to a size_t where the length of the encoded data (always 1) will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the encoded buffer (must be freed by caller).
 */
uint8_t* ax25_unnumbered_frame_encode(const ax25_unnumbered_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Decodes an Unnumbered Information (UI) frame from binary data.
 *
 * Creates a UI frame, extracting the PID and payload following the control byte
 * (Section 6.3.7).
 *
 * @param header Pointer to the AX.25 frame header.
 * @param pf Boolean indicating the poll/final bit from the control byte.
 * @param data Pointer to the data containing PID and payload.
 * @param len Length of the data in bytes (must be at least 1 for PID).
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the decoded UI frame (must be freed with ax25_frame_free).
 */
ax25_unnumbered_information_frame_t* ax25_unnumbered_information_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len,
        uint8_t *err);

/**
 * @brief Encodes an Unnumbered Information (UI) frame into a binary array.
 *
 * Serializes a UI frame, including the control byte, PID, and payload (Section 6.3.7).
 *
 * @param frame Pointer to the UI frame to encode.
 * @param len Pointer to a size_t where the length of the encoded data will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the encoded buffer (must be freed by caller).
 */
uint8_t* ax25_unnumbered_information_frame_encode(const ax25_unnumbered_information_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Decodes a Frame Reject (FRMR) frame from binary data.
 *
 * Interprets a 3-byte FRMR payload, extracting rejection flags, sequence numbers,
 * and control fields (Section 6.4.4).
 *
 * @param header Pointer to the AX.25 frame header.
 * @param pf Boolean indicating the poll/final bit.
 * @param data Pointer to the 3-byte data containing FRMR fields.
 * @param len Length of the data (must be exactly 3 bytes).
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the decoded FRMR frame (must be freed with ax25_frame_free).
 */
ax25_frame_reject_frame_t* ax25_frame_reject_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len, uint8_t *err);

/**
 * @brief Encodes a Frame Reject (FRMR) frame into a binary array.
 *
 * Serializes an FRMR frame, including the control byte and 3-byte rejection payload
 * (Section 6.4.4).
 *
 * @param frame Pointer to the FRMR frame to encode.
 * @param len Pointer to a size_t where the length of the encoded data (always 4) will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the encoded buffer (must be freed by caller).
 */
uint8_t* ax25_frame_reject_frame_encode(const ax25_frame_reject_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Decodes an Information (I) frame from binary data.
 *
 * Interprets an I-frame, supporting 8-bit or 16-bit control fields, extracting
 * sequence numbers, PID, and payload (Section 6.3.1).
 *
 * @param header Pointer to the AX.25 frame header.
 * @param control The control field (up to 16 bits).
 * @param data Pointer to the data containing PID and payload.
 * @param len Length of the data in bytes (must be at least 1 for PID).
 * @param is_16bit Boolean flag indicating 16-bit (true) or 8-bit (false) control field.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the decoded I-frame (must be freed with ax25_frame_free).
 */
ax25_information_frame_t* ax25_information_frame_decode(ax25_frame_header_t *header, uint16_t control, const uint8_t *data, size_t len, bool is_16bit,
        uint8_t *err);

/**
 * @brief Encodes an Information (I) frame into a binary array.
 *
 * Serializes an I-frame, including the control field, PID, and payload (Section 6.3.1).
 *
 * @param frame Pointer to the I-frame to encode.
 * @param len Pointer to a size_t where the length of the encoded data will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the encoded buffer (must be freed by caller).
 */
uint8_t* ax25_information_frame_encode(const ax25_information_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Decodes a Supervisory (S) frame from binary data.
 *
 * Interprets an S-frame (RR, RNR, REJ, SREJ), extracting the receive sequence number
 * and frame type from 8-bit or 16-bit control fields (Section 6.3.2).
 *
 * @param header Pointer to the AX.25 frame header.
 * @param control The control field (up to 16 bits).
 * @param is_16bit Boolean flag indicating 16-bit (true) or 8-bit (false) control field.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the decoded S-frame (must be freed with ax25_frame_free).
 */
ax25_supervisory_frame_t* ax25_supervisory_frame_decode(ax25_frame_header_t *header, uint16_t control, bool is_16bit, uint8_t *err);

/**
 * @brief Encodes a Supervisory (S) frame into a binary array.
 *
 * Serializes an S-frame, including the control field (8-bit or 16-bit) (Section 6.3.2).
 *
 * @param frame Pointer to the S-frame to encode.
 * @param len Pointer to a size_t where the length of the encoded data (1 or 2) will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the encoded buffer (must be freed by caller).
 */
uint8_t* ax25_supervisory_frame_encode(const ax25_supervisory_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Creates a new raw XID parameter with specified identifier and value.
 *
 * Allocates an XID parameter with a parameter identifier (PI) and raw byte array
 * as the parameter value (PV) for XID frames (Section 4.3.3.7).
 *
 * @param pi The parameter identifier.
 * @param pv Pointer to the parameter value bytes, or NULL if no value.
 * @param pv_len Length of the parameter value in bytes.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the new XID parameter (must be freed with its free function).
 */
ax25_xid_parameter_t* ax25_xid_raw_parameter_new(int pi, const uint8_t *pv, size_t pv_len, uint8_t *err);

/**
 * @brief Encodes a raw XID parameter into a binary array.
 *
 * Serializes an XID parameter into the format [PI, PL, PV], where PI is the
 * parameter identifier, PL is the parameter length, and PV is the value (Section 4.3.3.7).
 *
 * @param param Pointer to the XID parameter to encode.
 * @param len Pointer to a size_t where the length of the encoded data will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the encoded buffer (must be freed by caller).
 */
uint8_t* ax25_xid_raw_parameter_encode(const ax25_xid_parameter_t *param, size_t *len, uint8_t *err);

/**
 * @brief Creates a deep copy of a raw XID parameter.
 *
 * Duplicates an XID parameter, including its parameter value, ensuring the copy
 * is independent (Section 4.3.3.7).
 *
 * @param param Pointer to the XID parameter to copy.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the new XID parameter (must be freed with its free function).
 */
ax25_xid_parameter_t* ax25_xid_raw_parameter_copy(const ax25_xid_parameter_t *param, uint8_t *err);

/**
 * @brief Frees a raw XID parameter and its data.
 *
 * Deallocates an XID parameter and its parameter value to prevent memory leaks
 * (Section 4.3.3.7).
 *
 * @param param Pointer to the XID parameter to free. If NULL, the function does nothing.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 */
void ax25_xid_raw_parameter_free(ax25_xid_parameter_t *param, uint8_t *err);

/**
 * @brief Decodes an XID parameter from binary data.
 *
 * Interprets a segment of XID frame data as a parameter, extracting PI, PL, and PV
 * fields (Section 4.3.3.7).
 *
 * @param data Pointer to the binary data containing the parameter.
 * @param len Length of the available data in bytes.
 * @param consumed Pointer to a size_t where the number of bytes consumed will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the decoded XID parameter (must be freed with its free function).
 */
ax25_xid_parameter_t* ax25_xid_parameter_decode(const uint8_t *data, size_t len, size_t *consumed, uint8_t *err);

/**
 * @brief Decodes an Exchange Identification (XID) frame from binary data.
 *
 * Interprets an XID frame, extracting the function identifier (FI), group identifier (GI),
 * group length (GL), and parameters (Section 4.3.3.7).
 *
 * @param header Pointer to the AX.25 frame header.
 * @param pf Boolean indicating the poll/final bit.
 * @param data Pointer to the data containing XID fields and parameters.
 * @param len Length of the data in bytes (must be at least 4).
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the decoded XID frame (must be freed with ax25_frame_free).
 */
ax25_exchange_identification_frame_t* ax25_exchange_identification_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len,
        uint8_t *err);

/**
 * @brief Encodes an Exchange Identification (XID) frame into a binary array.
 *
 * Serializes an XID frame, including the control byte, FI, GI, GL, and parameters
 * (Section 4.3.3.7).
 *
 * @param frame Pointer to the XID frame to encode.
 * @param len Pointer to a size_t where the length of the encoded data will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the encoded buffer (must be freed by caller).
 */
uint8_t* ax25_exchange_identification_frame_encode(const ax25_exchange_identification_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Decodes a Test (TEST) frame from binary data.
 *
 * Interprets a TEST frame, extracting the payload for link connectivity testing
 * (Section 6.3.3.8).
 *
 * @param header Pointer to the AX.25 frame header.
 * @param pf Boolean indicating the poll/final bit.
 * @param data Pointer to the test payload data.
 * @param len Length of the payload data in bytes.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the decoded TEST frame (must be freed with ax25_frame_free).
 */
ax25_test_frame_t* ax25_test_frame_decode(ax25_frame_header_t *header, bool pf, const uint8_t *data, size_t len, uint8_t *err);

/**
 * @brief Encodes a Test (TEST) frame into a binary array.
 *
 * Serializes a TEST frame, including the control byte and test payload (Section 6.3.3.8).
 *
 * @param frame Pointer to the TEST frame to encode.
 * @param len Pointer to a size_t where the length of the encoded data will be stored.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the encoded buffer (must be freed by caller).
 */
uint8_t* ax25_test_frame_encode(const ax25_test_frame_t *frame, size_t *len, uint8_t *err);

/**
 * @brief Creates an XID parameter for Class of Procedures (COP).
 *
 * Constructs an XID parameter for Class of Procedures with specified flags and
 * reserved field (Section 4.3.3.7).
 *
 * @param a_flag Boolean flag for procedure A.
 * @param b_flag Boolean flag for procedure B.
 * @param c_flag Boolean flag for procedure C.
 * @param d_flag Boolean flag for procedure D.
 * @param e_flag Boolean flag for procedure E.
 * @param f_flag Boolean flag for procedure F.
 * @param g_flag Boolean flag for procedure G.
 * @param reserved Reserved field value (8 bits).
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the new XID parameter (must be freed with its free function).
 */
ax25_xid_parameter_t* ax25_xid_class_of_procedures_new(bool a_flag, bool b_flag, bool c_flag, bool d_flag, bool e_flag, bool f_flag, bool g_flag,
        uint8_t reserved, uint8_t *err);

/**
 * @brief Creates an XID parameter for HDLC Optional Functions.
 *
 * Constructs an XID parameter for HDLC optional functions with specified flags
 * and reserved fields (Section 4.3.3.7).
 *
 * @param rnr Boolean flag for Receiver Not Ready.
 * @param rej Boolean flag for Reject.
 * @param srej Boolean flag for Selective Reject.
 * @param sabm Boolean flag for Set Asynchronous Balanced Mode.
 * @param sabme Boolean flag for SABM Extended.
 * @param dm Boolean flag for Disconnect Mode.
 * @param disc Boolean flag for Disconnect.
 * @param ua Boolean flag for Unnumbered Acknowledge.
 * @param frmr Boolean flag for Frame Reject.
 * @param ui Boolean flag for Unnumbered Information.
 * @param xid Boolean flag for Exchange Identification.
 * @param test Boolean flag for Test.
 * @param modulo8 Boolean flag for modulo 8 operation.
 * @param modulo128 Boolean flag for modulo 128 operation.
 * @param res1 Boolean reserved flag 1.
 * @param res2 Boolean reserved flag 2.
 * @param res3 Boolean reserved flag 3.
 * @param res4 Boolean reserved flag 4.
 * @param res5 Boolean reserved flag 5.
 * @param res6 Boolean reserved flag 6.
 * @param res7 Boolean reserved flag 7.
 * @param reserved Reserved field value (8 bits).
 * @param ext Boolean flag for extension bit.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the new XID parameter (must be freed with its free function).
 */
ax25_xid_parameter_t* ax25_xid_hdlc_optional_functions_new(bool rnr, bool rej, bool srej, bool sabm, bool sabme, bool dm, bool disc, bool ua, bool frmr,
bool ui, bool xid, bool test, bool modulo8, bool modulo128, bool res1, bool res2, bool res3, bool res4, bool res5, bool res6, bool res7, uint8_t reserved,
bool ext, uint8_t *err);

/**
 * @brief Creates an XID parameter with a big-endian integer value.
 *
 * Constructs an XID parameter with a specified PI and a big-endian encoded integer
 * value for parameters like I-field length or window size (Section 4.3.3.7).
 *
 * @param pi The parameter identifier.
 * @param value The integer value to encode.
 * @param length The number of bytes for the value (1, 2, or 4).
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the new XID parameter (must be freed with its free function).
 */
ax25_xid_parameter_t* ax25_xid_big_endian_new(int pi, uint32_t value, size_t length, uint8_t *err);

/**
 * @brief Segments a payload into info fields for AX.25 frames.
 *
 * Splits a payload into multiple segments suitable for AX.25 frames, respecting
 * the maximum segment size (n1) (Section 6.9).
 *
 * @param payload Pointer to the payload data to segment.
 * @param payload_len Length of the payload in bytes.
 * @param n1 Maximum size of each segment in bytes.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @param num_segments Pointer to store the number of segments created.
 * @return Pointer to an array of segmented info fields (must be freed with ax25_free_segmented_info).
 */
ax25_segmented_info_t* ax25_segment_info_fields(const uint8_t *payload, size_t payload_len, size_t n1, uint8_t *err, size_t *num_segments);

/**
 * @brief Reassembles the original payload from segmented info fields.
 *
 * Combines segmented info fields into the original payload (Section 6.9).
 *
 * @param info_fields Array of segmented info fields.
 * @param num_info_fields Number of info fields in the array.
 * @param reassembled_len Pointer to store the length of the reassembled payload.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the reassembled payload (must be freed by caller).
 */
uint8_t* ax25_reassemble_info_fields(ax25_segmented_info_t *info_fields, size_t num_info_fields, size_t *reassembled_len, uint8_t *err);

/**
 * @brief Frees the memory allocated for segmented info fields.
 *
 * Deallocates an array of segmented info fields to prevent memory leaks (Section 6.9).
 *
 * @param segments Array of segmented info fields to free.
 * @param num_segments Number of segments in the array.
 */
void ax25_free_segmented_info(ax25_segmented_info_t *segments, size_t num_segments);

/**
 * @brief Determines if modulo 128 sequence numbering is used.
 *
 * Checks if a SABME frame and its response indicate the use of modulo 128
 * sequence numbering (Section 4.3).
 *
 * @param sabme Pointer to the SABME frame.
 * @param response Pointer to the response frame.
 * @return True if modulo 128 is used, false otherwise.
 */
bool is_modulo128_used(ax25_frame_t *sabme, ax25_frame_t *response);

/**
 * @brief Creates an XID parameter with raw data.
 *
 * Allocates and initializes an XID parameter with the given parameter identifier
 * and raw parameter value (Section 4.3.3.7).
 *
 * @param pi Parameter Identifier (e.g., 2 for Class of Procedures, 3 for HDLC Optional Functions).
 * @param pv Pointer to the parameter value data.
 * @param pv_len Length of the parameter value in bytes.
 * @param err Pointer to store error code (0 on success, non-zero on failure).
 * @return Pointer to the new XID parameter (must be freed with its free function).
 */
ax25_xid_parameter_t* ax25_xid_parameter_create(int pi, const uint8_t *pv, size_t pv_len, uint8_t *err);

/**
 * @brief Validate AX.25 frame meets minimum size requirement
 *
 * Per AX.25 v2.2 Section 3.9: minimum 136 bits (17 bytes)
 * Minimum: Dest(7) + Source(7) + Control(1) + FCS(2) = 17 bytes
 *
 * @param frame_len Length of the frame in bytes
 * @return true if frame meets minimum size, false otherwise
 */
bool ax25_validate_frame_size(size_t frame_len);

/**
 * @brief Validate address field structure
 *
 * Checks extension bits and field alignment according to AX.25 v2.2 Section 3.12
 * Validates that:
 * - At least 2 addresses present (destination + source)
 * - Maximum 10 addresses (dest + source + 8 repeaters)
 * - Proper extension bit termination
 * - No incomplete addresses
 *
 * @param addr_field Pointer to the address field data
 * @param len Length of available data
 * @param addr_field_len Output pointer to store validated address field length
 * @return true if address field is valid, false otherwise
 */
bool ax25_validate_address_field(const uint8_t *addr_field, size_t len, size_t *addr_field_len);

/**
 * @brief Validate SSID value
 *
 * Per AX.25 v2.2 Section 3.12.2: SSID must be in range 0-15 (4 bits)
 *
 * @param ssid SSID value to validate
 * @return true if SSID is valid (0-15), false otherwise
 */
bool ax25_validate_ssid(int ssid);

#endif /* AX25_H_ */
