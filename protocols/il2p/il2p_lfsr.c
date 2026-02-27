/* -----------------------------------------------------------------------
 * il2p_lfsr.c
 * ----------------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>

#include "il2p_lfsr.h"

void il2p_lfsr_scramble(il2p_lfsr_t *lfsr, uint8_t *buf, size_t len) {
    il2p_lfsr_reset(lfsr);
    for (size_t i = 0u; i < len; i++) {
        buf[i] = il2p_lfsr_step_byte(lfsr, buf[i]);
    }
    /* Caller must flush 5 bits if producing RS blocks;
     * see il2p_rs_encode_block() for flush bit handling. */
}

void il2p_lfsr_descramble(il2p_lfsr_t *lfsr, uint8_t *buf, size_t len) {
    /* Descramble is identical to scramble when LFSR is synchronised */
    il2p_lfsr_scramble(lfsr, buf, len);
}
