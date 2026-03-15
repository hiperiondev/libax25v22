/* -----------------------------------------------------------------------
 * il2p_rs.h
 * ----------------------------------------------------------------------- */
#ifndef IL2P_RS_H
#define IL2P_RS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Header block parameters */
#define IL2P_RS_HDR_DATA    13u   /* IL2P header data bytes */
#define IL2P_RS_HDR_PARITY   2u   /* Parity bytes for header */

/* Payload block parameters */
#define IL2P_RS_PAY_PARITY  16u   /* Parity bytes per payload block (fixed per v0.6) */
#define IL2P_RS_PAY_MAX_DATA 239u  /* Maximum data bytes per payload block */

/**
 * @brief Encode a block using RS systematic encoding.
 *
 * Appends 'nroots' parity bytes after the 'len' data bytes in 'block'.
 * The output block is: [data[0]..data[len-1] | parity[0]..parity[nroots-1]].
 *
 * This is a shortened RS code: the underlying RS(255, 255-nroots) code is
 * applied to a zero-padded message, and only 'len' data bytes are transmitted.
 *
 * @param block   Data buffer. Must have capacity len + nroots bytes.
 *                On return, bytes [len..len+nroots-1] are the parity symbols.
 * @param len     Number of data bytes (for header: 13, for payload: up to 239).
 * @param nroots  Number of parity bytes (for header: 2, for payload: 16).
 * @return        true on success.
 */
bool il2p_rs_encode(uint8_t *block, uint8_t len, uint8_t nroots);

/**
 * @brief Decode and correct a received RS block.
 *
 * @param block   Received block of (len + nroots) bytes (data + parity).
 *                Corrected data will be in bytes [0..len-1] on return.
 * @param len     Number of data bytes expected.
 * @param nroots  Number of parity bytes.
 * @return        Number of errors corrected (0..nroots/2), or -1 on failure.
 */
int8_t il2p_rs_decode(uint8_t *block, uint8_t len, uint8_t nroots);

#endif /* IL2P_RS_H */
