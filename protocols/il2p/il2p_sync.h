/* -----------------------------------------------------------------------
 * il2p_sync.h  —  IL2P sync word generation and detection
 * ----------------------------------------------------------------------- */
#ifndef IL2P_SYNC_H
#define IL2P_SYNC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* IL2P sync word per specification (24-bit, MSB-first on-air) */
#define IL2P_SYNC_WORD        0xF15E48UL
#define IL2P_SYNC_WORD_BYTES  3u
/* Maximum number of bits that may differ and still be accepted */
#define IL2P_SYNC_MAX_ERRORS  1u

/**
 * @brief Write the 3-byte sync word into output buffer (MSB first).
 *
 * @param buf  Output buffer, must have room for at least 3 bytes.
 */
static inline void il2p_sync_write(uint8_t *buf) {
    buf[0] = (uint8_t)((IL2P_SYNC_WORD >> 16) & 0xFFu);  /* 0xF1 */
    buf[1] = (uint8_t)((IL2P_SYNC_WORD >>  8) & 0xFFu);  /* 0x5E */
    buf[2] = (uint8_t)( IL2P_SYNC_WORD        & 0xFFu);  /* 0x48 */
}

/**
 * @brief Count differing bits between a 24-bit value and the sync word.
 *
 * Uses 32-bit integer XOR and a popcount loop — no 64-bit arithmetic,
 * no hardware popcount instruction assumed.
 *
 * @param candidate  24-bit value extracted from received bit-stream.
 * @return           Number of bits that differ from IL2P_SYNC_WORD (0..24).
 */
static inline uint8_t il2p_sync_bit_errors(uint32_t candidate) {
    uint32_t diff = (candidate ^ (uint32_t)IL2P_SYNC_WORD) & 0x00FFFFFFuL;
    uint8_t  cnt  = 0u;
    /* Brian Kernighan popcount — safe on any 32-bit MCU, no 64-bit ops */
    while (diff) {
        diff &= diff - 1u;
        cnt++;
    }
    return cnt;
}

/**
 * @brief Test whether a 24-bit candidate matches the IL2P sync word.
 *
 * A match is declared when the number of differing bits is within the
 * allowed tolerance (1 bit per spec).
 *
 * @param candidate  24-bit value from receive shift register.
 * @return           true if this is a valid sync word match.
 */
static inline bool il2p_sync_match(uint32_t candidate) {
    return il2p_sync_bit_errors(candidate) <= IL2P_SYNC_MAX_ERRORS;
}

/**
 * @brief Sliding-window sync word search over a received byte buffer.
 *
 * Shifts a 24-bit window across the buffer one bit at a time.
 * On match, returns the byte offset and bit offset of the sync word's
 * MSB within the buffer.
 *
 * @param buf      Received bytes, MSB first.
 * @param buf_len  Number of bytes in buf.
 * @param byte_off Output: byte index where sync word was found.
 * @param bit_off  Output: bit offset within that byte (0 = MSB).
 * @return         true if sync word found, false otherwise.
 */
bool il2p_sync_search(const uint8_t *buf, size_t buf_len,
                      size_t *byte_off, uint8_t *bit_off);

#endif /* IL2P_SYNC_H */
