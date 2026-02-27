/* -----------------------------------------------------------------------
 * il2p_sixbit.h  —  DEC SIXBIT callsign encoding for IL2P
 * ----------------------------------------------------------------------- */
#ifndef IL2P_SIXBIT_H
#define IL2P_SIXBIT_H

#include <stdint.h>
#include <stdbool.h>

#define IL2P_SIXBIT_VALID_MIN  0x20u   /* ASCII space  */
#define IL2P_SIXBIT_VALID_MAX  0x5Fu   /* ASCII '_'    */
#define IL2P_CALLSIGN_LEN      6u      /* Always 6 characters */

/**
 * @brief Encode one ASCII character to DEC SIXBIT.
 *
 * @param ascii  Input character (printable ASCII).
 * @param out    Output 6-bit value (bits 5..0 used).
 * @return       true on success, false if character is not encodable.
 */
static inline bool il2p_sixbit_encode_char(uint8_t ascii, uint8_t *out) {
    if (ascii < IL2P_SIXBIT_VALID_MIN || ascii > IL2P_SIXBIT_VALID_MAX)
        return false;
    *out = (uint8_t)(ascii - IL2P_SIXBIT_VALID_MIN) & 0x3Fu;
    return true;
}

/**
 * @brief Decode one DEC SIXBIT value to ASCII.
 *
 * @param sixbit  6-bit value (bits 5..0).
 * @return        ASCII character.
 */
static inline uint8_t il2p_sixbit_decode_char(uint8_t sixbit) {
    return (uint8_t)((sixbit & 0x3Fu) + IL2P_SIXBIT_VALID_MIN);
}

/**
 * @brief Encode a 6-character space-padded AX.25 callsign to 36 bits of SIXBIT.
 *
 * The 36 bits are packed into 5 bytes (bits 7..2 of each byte are used,
 * lower 2 bits of each destination byte are left unchanged / zeroed).
 * Packing into the IL2P header is done by il2p_header.c.
 *
 * @param ascii_callsign   6-char space-padded callsign (may not be NUL-terminated).
 * @param sixbit_out       Output array of 6 six-bit values (one per character).
 * @return                 true if all characters are encodable, false otherwise.
 */
bool il2p_sixbit_encode_callsign(const char *ascii_callsign, uint8_t sixbit_out[6]);

/**
 * @brief Decode 6 SIXBIT values back to an ASCII callsign.
 *
 * @param sixbit_in        6 six-bit input values.
 * @param ascii_out        Output 7-byte buffer (6 chars + NUL terminator).
 */
void il2p_sixbit_decode_callsign(const uint8_t sixbit_in[6], char ascii_out[7]);

#endif /* IL2P_SIXBIT_H */
