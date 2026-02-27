/* -----------------------------------------------------------------------
 * il2p_sync.c
 * ----------------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>

#include "il2p_sync.h"

bool il2p_sync_search(const uint8_t *buf, size_t buf_len, size_t *byte_off, uint8_t *bit_off) {
    if (!buf || buf_len < 3u || !byte_off || !bit_off)
        return false;

    // Process every bit from the start, no pre-loading shortcut.
    // Pre-loading skipped the case where the sync word begins at byte 0.
    uint32_t window = 0u;
    uint32_t bits_loaded = 0u;

    for (size_t i = 0u; i < buf_len; i++) {
        uint8_t byte_in = buf[i];
        // Process 8 new bits, MSB first
        for (int8_t b = 7; b >= 0; b--) {
            uint8_t new_bit = (byte_in >> (uint8_t) b) & 1u;
            window = ((window << 1u) | new_bit) & 0x00FFFFFFuL;
            bits_loaded++;
            if (bits_loaded >= 24u && il2p_sync_match(window)) {
                // Use signed arithmetic to avoid uint32_t underflow when
                // the sync word starts at or near offset 0 in the buffer.
                int32_t last_bit = (int32_t) (i * 8u) + (int32_t) (7 - b);
                int32_t start_bit = last_bit - 23;
                if (start_bit < 0) {
                    // Sync word start is before buffer begin -- invalid
                    continue;
                }
                *byte_off = (size_t) ((uint32_t) start_bit / 8u);
                *bit_off = (uint8_t) ((uint32_t) start_bit % 8u);
                return true;
            }
        }
    }
    return false;
}
