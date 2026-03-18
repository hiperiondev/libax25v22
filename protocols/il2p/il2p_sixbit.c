/* -----------------------------------------------------------------------
 * il2p_sixbit.c
 * ----------------------------------------------------------------------- */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "il2p_sixbit.h"

bool il2p_sixbit_encode_callsign(const char *ascii_callsign, uint8_t sixbit_out[6]) {
    if (!ascii_callsign || !sixbit_out)
        return false;

    for (uint8_t i = 0u; i < IL2P_CALLSIGN_LEN; i++) {
        uint8_t ch = (uint8_t) ascii_callsign[i];
        /* Treat NUL-terminated short callsigns: pad remainder with spaces */
        if (ch == '\0')
            ch = ' ';
        if (!il2p_sixbit_encode_char(ch, &sixbit_out[i]))
            return false; /* Caller must fall back to Type 0 */
    }
    return true;
}

void il2p_sixbit_decode_callsign(const uint8_t sixbit_in[6], char ascii_out[7]) {
    if (!sixbit_in || !ascii_out)
        return;

    for (uint8_t i = 0u; i < IL2P_CALLSIGN_LEN; i++) {
        ascii_out[i] = (char) il2p_sixbit_decode_char(sixbit_in[i]);
    }
    ascii_out[IL2P_CALLSIGN_LEN] = '\0';
}
