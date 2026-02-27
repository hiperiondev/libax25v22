/* -----------------------------------------------------------------------
 * il2p_lfsr.c
 * ----------------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>

#include "il2p_lfsr.h"

// NOTE: These helper functions are self-contained (they reset the LFSR internally).
// The primary encode/decode paths use il2p_lfsr_step_byte() directly for
// finer-grained per-block LFSR control. These helpers are provided only for
// convenience when processing a standalone contiguous buffer outside the
// normal RS block encode/decode flow.
void il2p_lfsr_scramble(il2p_lfsr_t *lfsr, uint8_t *buf, size_t len) {
    il2p_lfsr_reset(lfsr);
    for (size_t i = 0u; i < len; i++) {
        buf[i] = il2p_lfsr_step_byte(lfsr, buf[i]);
    }
}

void il2p_lfsr_descramble(il2p_lfsr_t *lfsr, uint8_t *buf, size_t len) {
    // XOR scrambling is self-inverse when the LFSR state matches TX;
    // packet-synchronous reset guarantees that condition here.
    il2p_lfsr_scramble(lfsr, buf, len);
}
