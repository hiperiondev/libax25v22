/* -----------------------------------------------------------------------
 * il2p_lfsr.h  —  IL2P Galois LFSR scrambler/descrambler
 * ----------------------------------------------------------------------- */
#ifndef IL2P_LFSR_H
#define IL2P_LFSR_H

#include <stdint.h>
#include <stddef.h>

/*
 * IL2P LFSR: polynomial x^9 + x^4 + 1
 *
 * Galois configuration, 9-bit register.
 * Initial state: all ones = 0x1FF
 * Processing: MSB-first (IL2P sends all bytes MSB first)
 *
 * Feedback mask: when the MSB (bit 8) shifts out as '1', XOR the
 * lower register bits corresponding to the polynomial taps:
 *   x^4 → bit 4 of the 9-bit register → mask bit = (1 << 4) = 0x010
 *   x^0 (constant +1) is the output tap itself, handled by the shift
 *
 * The 5-bit Galois pipeline delay means that after all N data bytes
 * are clocked through the LFSR, 5 additional "flush" bits (zeros fed
 * as input) must be processed to extract all information from the
 * register. The scrambled output of these 5 flush bits is appended
 * to the data block before RS encoding.
 */

#define IL2P_LFSR_POLY_MASK   0x010u   /* Tap mask for Galois feedback (x^4 term only) */
#define IL2P_LFSR_INIT        0x1FFu   /* Initial state: all 9 bits set */
#define IL2P_LFSR_FLUSH_BITS  5u       /* Pipeline flush bits */

/** LFSR state container */
typedef struct {
    uint16_t state;  /**< 9-bit LFSR state (bits 8..0 used, bits 15..9 always 0) */
} il2p_lfsr_t;

/**
 * @brief Reset LFSR to initial conditions.
 *
 * Must be called at the start of each RS code block (header or payload).
 *
 * @param lfsr  LFSR state.
 */
static inline void il2p_lfsr_reset(il2p_lfsr_t *lfsr) {
    lfsr->state = IL2P_LFSR_INIT;
}

/**
 * @brief Process one bit through the Galois LFSR (MSB-first convention).
 *
 * Both scramble and descramble use the same operation when the LFSR
 * runs independently (additive / packet-synchronous mode). Feed 0
 * for flush bits.
 *
 * @param lfsr      LFSR state (updated in place).
 * @param data_bit  Input data bit (0 or 1).
 * @return          Scrambled/descrambled output bit.
 */
static inline uint8_t il2p_lfsr_step_bit(il2p_lfsr_t *lfsr, uint8_t data_bit) {
    /* Extract MSB (this is the LFSR output bit in Galois config, MSB-first) */
    uint8_t out_bit = (uint8_t)((lfsr->state >> 8u) & 1u);
    /* Shift left by 1 */
    lfsr->state = (uint16_t)((lfsr->state << 1u) & 0x1FFu);
    /* Galois feedback: if out_bit=1, XOR tap mask into register */
    if (out_bit) {
        lfsr->state ^= (uint16_t)IL2P_LFSR_POLY_MASK;
    }
    /* XOR data bit with LFSR output to produce scrambled bit */
    return (uint8_t)(data_bit ^ out_bit);
}

/**
 * @brief Scramble or descramble one byte (MSB first, 8 bits).
 *
 * @param lfsr  LFSR state.
 * @param byte  Input byte.
 * @return      Scrambled/descrambled byte.
 */
static inline uint8_t il2p_lfsr_step_byte(il2p_lfsr_t *lfsr, uint8_t byte) {
    uint8_t result = 0u;
    for (int8_t b = 7; b >= 0; b--) {
        uint8_t in_bit  = (uint8_t)((byte >> (uint8_t)b) & 1u);
        uint8_t out_bit = il2p_lfsr_step_bit(lfsr, in_bit);
        result |= (uint8_t)(out_bit << (uint8_t)b);
    }
    return result;
}

/**
 * @brief Scramble a buffer of bytes in place.
 *
 * Resets the LFSR to initial conditions before processing.
 * After processing all data bytes, appends flush_out[0..4] (5 bytes,
 * only the 5 MSBs of flush_out[0] are valid; the rest of flush_out
 * bytes hold the remaining scrambled flush bits packed MSB-first).
 *
 * NOTE: This function does NOT append the flush bits to buf.
 * The caller must handle flush bits separately for RS block sizing.
 *
 * @param lfsr      LFSR state (will be reset then modified).
 * @param buf       Data buffer (modified in place).
 * @param len       Number of bytes.
 */
void il2p_lfsr_scramble(il2p_lfsr_t *lfsr, uint8_t *buf, size_t len);

/**
 * @brief Descramble a buffer of bytes in place.
 *
 * Identical operation to scramble (XOR is its own inverse when LFSR
 * state is synchronised, which is guaranteed by packet-synchronous reset).
 *
 * @param lfsr  LFSR state (will be reset then modified).
 * @param buf   Data buffer (modified in place).
 * @param len   Number of bytes.
 */
void il2p_lfsr_descramble(il2p_lfsr_t *lfsr, uint8_t *buf, size_t len);

#endif /* IL2P_LFSR_H */
