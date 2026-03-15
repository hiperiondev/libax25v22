/**
 * @file kiss.c
 * @brief AX.25 v2.2 Protocol Library - KISS TNC Interface Protocol
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 * @date 2026
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#include <string.h>
#include <stddef.h>

#include "kiss.h"
#include "hal.h"

// ---------------------------------------------------------------------------
// Internal helper: encode one raw data byte into the tx buffer applying
// KISS byte-stuffing rules.  Returns number of bytes written (1 or 2).
// buf must have at least 2 bytes of space.
// ---------------------------------------------------------------------------

static size_t kiss_stuff_byte(uint8_t *buf, uint8_t byte) {
    if (byte == KISS_FEND) {
        // Escape FEND -> FESC TFEND
        buf[0] = KISS_FESC;
        buf[1] = KISS_TFEND;
        return 2u;
    }
    if (byte == KISS_FESC) {
        // Escape FESC -> FESC TFESC
        buf[0] = KISS_FESC;
        buf[1] = KISS_TFESC;
        return 2u;
    }
    // Normal byte passes through unchanged
    buf[0] = byte;
    return 1u;
}

// SMACK CRC-16/ARC implementation
// CRC-16/ARC: poly 0x8005 reflected = 0xA001, init 0x0000, no final XOR
// All arithmetic is 8-bit or 16-bit; safe for MCUs without FPU, no 64-bit

#ifndef USE_CRC_TABLE

// Bit-by-bit version: zero lookup table overhead
uint16_t ax25_kiss_smack_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000u;
    size_t i;
    uint8_t bit;
    uint8_t byte_val;
    uint8_t lsb;
    if (data == NULL) {
        return 0x0000u;
    }
    for (i = 0u; i < len; i++) {
        byte_val = data[i];
        for (bit = 0u; bit < 8u; bit++) {
            lsb = (uint8_t) ((crc ^ (uint16_t) byte_val) & 0x0001u);
            crc >>= 1u;
            if (lsb != 0u) {
                crc ^= 0xA001u;
            }
            byte_val >>= 1u;
        }
    }
    return crc;
}

#else // USE_CRC_TABLE

// Table for CRC-16/ARC (reflected poly 0xA001, init 0x0000)
// 256 entries x 2 bytes = 512 bytes const/flash
// Different table from common.c AX.25 CRC which uses poly 0x8408
static const uint16_t smack_crc16_table[256] = {
    0x0000u, 0xC0C1u, 0xC181u, 0x0140u, 0xC301u, 0x03C0u, 0x0280u, 0xC241u,
    0xC601u, 0x06C0u, 0x0780u, 0xC741u, 0x0500u, 0xC5C1u, 0xC481u, 0x0440u,
    0xCC01u, 0x0CC0u, 0x0D80u, 0xCD41u, 0x0F00u, 0xCFC1u, 0xCE81u, 0x0E40u,
    0x0A00u, 0xCAC1u, 0xCB81u, 0x0B40u, 0xC901u, 0x09C0u, 0x0880u, 0xC841u,
    0xD801u, 0x18C0u, 0x1980u, 0xD941u, 0x1B00u, 0xDBC1u, 0xDA81u, 0x1A40u,
    0x1E00u, 0xDEC1u, 0xDF81u, 0x1F40u, 0xDD01u, 0x1DC0u, 0x1C80u, 0xDC41u,
    0x1400u, 0xD4C1u, 0xD581u, 0x1540u, 0xD701u, 0x17C0u, 0x1680u, 0xD641u,
    0xD201u, 0x12C0u, 0x1380u, 0xD341u, 0x1100u, 0xD1C1u, 0xD081u, 0x1040u,
    0xF001u, 0x30C0u, 0x3180u, 0xF141u, 0x3300u, 0xF3C1u, 0xF281u, 0x3240u,
    0x3600u, 0xF6C1u, 0xF781u, 0x3740u, 0xF501u, 0x35C0u, 0x3480u, 0xF441u,
    0x3C00u, 0xFCC1u, 0xFD81u, 0x3D40u, 0xFF01u, 0x3FC0u, 0x3E80u, 0xFE41u,
    0xFA01u, 0x3AC0u, 0x3B80u, 0xFB41u, 0x3900u, 0xF9C1u, 0xF881u, 0x3840u,
    0x2800u, 0xE8C1u, 0xE981u, 0x2940u, 0xEB01u, 0x2BC0u, 0x2A80u, 0xEA41u,
    0xEE01u, 0x2EC0u, 0x2F80u, 0xEF41u, 0x2D00u, 0xEDC1u, 0xEC81u, 0x2C40u,
    0xE401u, 0x24C0u, 0x2580u, 0xE541u, 0x2700u, 0xE7C1u, 0xE681u, 0x2640u,
    0x2200u, 0xE2C1u, 0xE381u, 0x2340u, 0xE101u, 0x21C0u, 0x2080u, 0xE041u,
    0xA001u, 0x60C0u, 0x6180u, 0xA141u, 0x6300u, 0xA3C1u, 0xA281u, 0x6240u,
    0x6600u, 0xA6C1u, 0xA781u, 0x6740u, 0xA501u, 0x65C0u, 0x6480u, 0xA441u,
    0x6C00u, 0xACC1u, 0xAD81u, 0x6D40u, 0xAF01u, 0x6FC0u, 0x6E80u, 0xAE41u,
    0xAA01u, 0x6AC0u, 0x6B80u, 0xAB41u, 0x6900u, 0xA9C1u, 0xA881u, 0x6840u,
    0x7800u, 0xB8C1u, 0xB981u, 0x7940u, 0xBB01u, 0x7BC0u, 0x7A80u, 0xBA41u,
    0xBE01u, 0x7EC0u, 0x7F80u, 0xBF41u, 0x7D00u, 0xBDC1u, 0xBC81u, 0x7C40u,
    0xB401u, 0x74C0u, 0x7580u, 0xB541u, 0x7700u, 0xB7C1u, 0xB681u, 0x7640u,
    0x7200u, 0xB2C1u, 0xB381u, 0x7340u, 0xB101u, 0x71C0u, 0x7080u, 0xB041u,
    0x5000u, 0x90C1u, 0x9181u, 0x5140u, 0x9301u, 0x53C0u, 0x5280u, 0x9241u,
    0x9601u, 0x56C0u, 0x5780u, 0x9741u, 0x9500u, 0x55C1u, 0x5481u, 0x5440u,
    0x9C01u, 0x5CC0u, 0x5D80u, 0x9D41u, 0x5F00u, 0x9FC1u, 0x9E81u, 0x5E40u,
    0x5A00u, 0x9AC1u, 0x9B81u, 0x5B40u, 0x9901u, 0x59C0u, 0x5880u, 0x9841u,
    0x8801u, 0x48C0u, 0x4980u, 0x8941u, 0x4B00u, 0x8BC1u, 0x8A81u, 0x4A40u,
    0x4E00u, 0x8EC1u, 0x8F81u, 0x4F40u, 0x8D01u, 0x4DC0u, 0x4C80u, 0x8C41u,
    0x4400u, 0x84C1u, 0x8581u, 0x4540u, 0x8701u, 0x47C0u, 0x4680u, 0x8641u,
    0x8201u, 0x42C0u, 0x4380u, 0x8341u, 0x4100u, 0x81C1u, 0x8081u, 0x4040u
};

uint16_t ax25_kiss_smack_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000u;
    uint8_t  idx;
    size_t   i;
    if (data == NULL) {
        return 0x0000u;
    }
    for (i = 0u; i < len; i++) {
        idx = (uint8_t)((crc ^ (uint16_t)data[i]) & 0x00FFu);
        crc = (uint16_t)((crc >> 8u) ^ smack_crc16_table[idx]);
    }
    return crc;
}

#endif // USE_CRC_TABLE

// G8BPQ 8-bit XOR checksum over all bytes including type byte, before SLIP encoding
uint8_t ax25_kiss_crc8_xor(const uint8_t *data, size_t len) {
    uint8_t chk = 0x00u;
    size_t i;
    if (data == NULL) {
        return 0x00u;
    }
    for (i = 0u; i < len; i++) {
        chk ^= data[i];
    }
    return chk;
}

// smack_crc_frame: internal incremental CRC helper
// Computes SMACK CRC-16 over type_byte then payload without a contiguous
// pre-SLIP buffer allocation.  Used by kiss_build_and_send (TX) and
// kiss_dispatch_frame (RX).  Only 8-bit and 16-bit arithmetic; MCU-safe.
static uint16_t smack_crc_frame(uint8_t type_byte, const uint8_t *payload, size_t payload_len) {
    uint16_t crc = 0x0000u;
    size_t i;
#ifdef USE_CRC_TABLE
    uint8_t  idx;
    idx = (uint8_t)((crc ^ (uint16_t)type_byte) & 0x00FFu);
    crc = (uint16_t)((crc >> 8u) ^ smack_crc16_table[idx]);
    if (payload != NULL) {
        for (i = 0u; i < payload_len; i++) {
            idx = (uint8_t)((crc ^ (uint16_t)payload[i]) & 0x00FFu);
            crc = (uint16_t)((crc >> 8u) ^ smack_crc16_table[idx]);
        }
    }
#else
    uint8_t byte_val;
    uint8_t bit;
    uint8_t lsb;
    byte_val = type_byte;
    for (bit = 0u; bit < 8u; bit++) {
        lsb = (uint8_t) ((crc ^ (uint16_t) byte_val) & 0x0001u);
        crc >>= 1u;
        if (lsb != 0u) {
            crc ^= 0xA001u;
        }
        byte_val >>= 1u;
    }
    if (payload != NULL) {
        for (i = 0u; i < payload_len; i++) {
            byte_val = payload[i];
            for (bit = 0u; bit < 8u; bit++) {
                lsb = (uint8_t) ((crc ^ (uint16_t) byte_val) & 0x0001u);
                crc >>= 1u;
                if (lsb != 0u) {
                    crc ^= 0xA001u;
                }
                byte_val >>= 1u;
            }
        }
    }
#endif
    return crc;
}

// Internal helper: build a complete KISS frame in the caller-supplied buffer
// and call serial_write once with the complete frame.
// frame_payload: bytes after the type indicator (may be NULL if len=0)
// Returns KISS_OK or KISS_ERR_FRAME_SIZE.
// kiss_build_and_send: added SMACK CRC TX path
// Original ignored ctx->variant entirely.  Now sets bit 7 of type byte for SMACK,
// computes CRC-16 over [smack_type, payload] pre-SLIP, SLIP-encodes and appends
// crc_lo then crc_hi before the closing FEND.
static uint8_t kiss_build_and_send(ax25_kiss_ctx_t *ctx, uint8_t type_byte, const uint8_t *frame_payload, size_t payload_len) {
    bool use_smack;
    uint8_t smack_type;
    uint16_t smack_crc;
    uint8_t crc_lo;
    uint8_t crc_hi;
    uint8_t tx[KISS_TX_BUF_SIZE];
    size_t pos = 0u;
    size_t i;
    uint8_t tmp[2];
    size_t n;

    // Worst case: type(2) + payload(N*2) + CRC trailer(2*2) + 2 FENDs = N*2+8
    size_t max_needed = 2u + payload_len * 2u + 4u + 2u;

    if (max_needed > KISS_TX_BUF_SIZE) {
        return KISS_ERR_FRAME_SIZE;
    }

    // Only DATA frames are checksummed; parameter commands are never checksummed
    use_smack = false;
    if (KISS_CMD(type_byte) == KISS_CMD_DATA) {
        if (ctx->variant == KISS_VARIANT_SMACK) {
            use_smack = true;
        } else if ((ctx->variant == KISS_VARIANT_AUTO) && ctx->smack_active) {
            use_smack = true;
        }
    }

    // For SMACK set bit 7 (CRC-present flag); port stays in bits [6:4]
    smack_type = type_byte;
    if (use_smack) {
        smack_type = (uint8_t) (type_byte | KISS_SMACK_CRC_FLAG);
    }

    // Pre-compute SMACK CRC over [smack_type, payload] BEFORE SLIP encoding
    crc_lo = 0x00u;
    crc_hi = 0x00u;
    if (use_smack) {
        smack_crc = smack_crc_frame(smack_type, frame_payload, payload_len);
        crc_lo = (uint8_t) (smack_crc & 0x00FFu);
        crc_hi = (uint8_t) ((smack_crc >> 8u) & 0x00FFu);
    }

    tx[pos++] = KISS_FEND;

    // SLIP-encode type byte
    tmp[0] = 0u;
    tmp[1] = 0u;
    n = kiss_stuff_byte(tmp, smack_type);
    tx[pos] = tmp[0];
    tx[pos + 1u] = tmp[1u];
    pos += n;

    // SLIP-encode payload
    if (frame_payload != NULL && payload_len > 0u) {
        for (i = 0u; i < payload_len; i++) {
            tmp[0] = 0u;
            tmp[1] = 0u;
            n = kiss_stuff_byte(tmp, frame_payload[i]);
            tx[pos] = tmp[0];
            tx[pos + 1u] = tmp[1u];
            pos += n;
        }
    }

    // SLIP-encode SMACK CRC trailer: LSB first then MSB
    if (use_smack) {
        tmp[0] = 0u;
        tmp[1] = 0u;
        n = kiss_stuff_byte(tmp, crc_lo);
        tx[pos] = tmp[0];
        tx[pos + 1u] = tmp[1u];
        pos += n;
        tmp[0] = 0u;
        tmp[1] = 0u;
        n = kiss_stuff_byte(tmp, crc_hi);
        tx[pos] = tmp[0];
        tx[pos + 1u] = tmp[1u];
        pos += n;
    }

    // Closing FEND
    tx[pos++] = KISS_FEND;

    ctx->serial_write(tx, pos, ctx->user_data);
    // Update tx_bytes with actual encoded bytes written to serial
    ctx->stats.tx_bytes += (uint32_t) pos;

    return KISS_OK;
}

// Internal: dispatch a fully assembled frame stored in ctx->rx_buf.
// ctx->rx_type holds the type indicator byte.
// ctx->rx_len holds the number of payload bytes (NOT including type byte).
// SMACK CRC RX verify/strip path
// checks bit 7 of rx_type; if set verifies SMACK CRC-16 over
// [rx_type, payload], strips the 2 CRC trailer bytes on success, or drops
// the frame and calls on_crc_error on failure.  Also auto-latches smack_active.
static void kiss_dispatch_frame(ax25_kiss_ctx_t *ctx) {
    bool is_smack;
    uint8_t eff_port;
    uint8_t port;
    uint8_t cmd;
    uint16_t rx_crc;
    uint16_t calc_crc;
    uint8_t crc_lo;
    uint8_t crc_hi;
    size_t data_len;

    // Minimum valid frame: at least a type byte was received
    if (!ctx->rx_got_type) {
        return;
    }

    // SMACK detection: check bit-7 of type byte for SMACK and AUTO variants.
    // In AUTO mode we must check unconditionally (regardless of smack_active)
    // so the very first SMACK frame can trigger the latch.
    // In STANDARD mode bit-7 belongs to the port nibble and must NOT be
    // treated as a CRC flag (ports 8-14 have bit-7 set naturally).
    is_smack = false;

    if (ctx->variant == KISS_VARIANT_SMACK || ctx->variant == KISS_VARIANT_AUTO) {
        is_smack = ((ctx->rx_type & KISS_SMACK_CRC_FLAG) != 0u);
    }

    // Auto-negotiate: latch SMACK TX on first received SMACK frame in AUTO mode
    if (is_smack && (ctx->variant == KISS_VARIANT_AUTO) && !ctx->smack_active) {
        ctx->smack_active = true;
    }

    // For SMACK, port is in bits [6:4] (bit 7 is CRC flag, not port)
    // For standard KISS, port is in bits [7:4] via KISS_PORT macro
    if (is_smack) {
        eff_port = (uint8_t) ((ctx->rx_type >> 4u) & 0x07u);
    } else {
        eff_port = KISS_PORT(ctx->rx_type);
    }
    port = eff_port;
    cmd = KISS_CMD(ctx->rx_type);

    // Special case: RETURN uses full type byte 0xFF
    // The port nibble of 0xFF is 0xF which we treat as the RETURN command.
    // Per spec cmd=0xF is the return indicator.
    if (ctx->rx_type == KISS_RETURN_TYPE_BYTE || cmd == KISS_CMD_RETURN) {
        ctx->kiss_mode = false;
        if (ctx->on_return) {
            ctx->on_return(ctx, ctx->user_data);
        }
        return;
    }

    // Port 0xF is reserved for non-RETURN uses; ignore silently
    if (port == 0x0Fu) {
        return;
    }

    switch (cmd) {
        case KISS_CMD_DATA:
            if (is_smack) {
                // Need at least 2 CRC trailer bytes
                if (ctx->rx_len < KISS_SMACK_CRC_SIZE) {
                    if (ctx->on_crc_error) {
                        ctx->on_crc_error(ctx, eff_port, ctx->rx_buf, ctx->rx_len, ctx->user_data);
                    }
                    ctx->stats.rx_bad_checksum++;
                    ctx->stats.rx_dropped++;
                    break;
                }
                data_len = ctx->rx_len - KISS_SMACK_CRC_SIZE;
                // Extract received CRC LSB first (already SLIP-decoded by RX state machine)
                crc_lo = ctx->rx_buf[data_len];
                crc_hi = ctx->rx_buf[data_len + 1u];
                rx_crc = (uint16_t) ((uint16_t) crc_lo | ((uint16_t) crc_hi << 8u));
                // Recompute CRC over [type_byte, payload] on post-SLIP-decoded data
                calc_crc = smack_crc_frame(ctx->rx_type, ctx->rx_buf, data_len);
                if (calc_crc != rx_crc) {
                    // CRC mismatch: drop frame and fire error callback
                    if (ctx->on_crc_error) {
                        ctx->on_crc_error(ctx, eff_port, ctx->rx_buf, ctx->rx_len, ctx->user_data);
                    }
                    ctx->stats.rx_bad_checksum++;
                    ctx->stats.rx_dropped++;
                    break;
                }
                // CRC valid: deliver payload without CRC trailer bytes
                if (ctx->on_frame && data_len > 0u) {
                    ctx->on_frame(ctx, eff_port, ctx->rx_buf, data_len, ctx->user_data);
                }
                ctx->stats.rx_frames++;
            } else {
                // Standard KISS DATA: original behaviour, no checksum
                if (ctx->on_frame && ctx->rx_len > 0u) {
                    ctx->on_frame(ctx, port, ctx->rx_buf, ctx->rx_len, ctx->user_data);
                }
                ctx->stats.rx_frames++;
            }

        break;

        case KISS_CMD_TXDELAY:
            // One parameter byte: TxDelay in 10ms units
            if (ctx->rx_len >= 1u) {
                ctx->ports[port].txdelay = ctx->rx_buf[0];
            }

            // Sync all KISS params to HAL after any parameter command.
            // Fields not updated in this command carry their current values.
            hal_channel_params_from_kiss(port, ctx->ports[port].txdelay, ctx->ports[port].persistence, ctx->ports[port].slottime, ctx->ports[port].txtail,
                    ctx->ports[port].full_duplex ? 1U : 0U);

            // Count received command frames
            ctx->stats.rx_cmd_frames++;

        break;

        case KISS_CMD_PERSISTENCE:
            // One parameter byte: p-persistence P value 0-255
            if (ctx->rx_len >= 1u) {
                ctx->ports[port].persistence = ctx->rx_buf[0];
            }

            // Sync all KISS params to HAL after any parameter command.
            // Fields not updated in this command carry their current values.
            hal_channel_params_from_kiss(port, ctx->ports[port].txdelay, ctx->ports[port].persistence, ctx->ports[port].slottime, ctx->ports[port].txtail,
                    ctx->ports[port].full_duplex ? 1U : 0U);

            // Count received command frames
            ctx->stats.rx_cmd_frames++;
        break;

        case KISS_CMD_SLOTTIME:
            // One parameter byte: slot time in 10ms units
            if (ctx->rx_len >= 1u) {
                ctx->ports[port].slottime = ctx->rx_buf[0];
            }

            // Sync all KISS params to HAL after any parameter command.
            // Fields not updated in this command carry their current values.
            hal_channel_params_from_kiss(port, ctx->ports[port].txdelay, ctx->ports[port].persistence, ctx->ports[port].slottime, ctx->ports[port].txtail,
                    ctx->ports[port].full_duplex ? 1U : 0U);

            // Count received command frames
            ctx->stats.rx_cmd_frames++;
        break;

        case KISS_CMD_TXTAIL:
            // One parameter byte: TX tail in 10ms units (obsolete)
            if (ctx->rx_len >= 1u) {
                ctx->ports[port].txtail = ctx->rx_buf[0];
            }

            // Sync all KISS params to HAL after any parameter command.
            // Fields not updated in this command carry their current values.
            hal_channel_params_from_kiss(port, ctx->ports[port].txdelay, ctx->ports[port].persistence, ctx->ports[port].slottime, ctx->ports[port].txtail,
                    ctx->ports[port].full_duplex ? 1U : 0U);

            // Count received command frames
            ctx->stats.rx_cmd_frames++;
        break;

        case KISS_CMD_FULLDUPLEX:
            // One parameter byte: 0=half-duplex, non-zero=full-duplex
            if (ctx->rx_len >= 1u) {
                ctx->ports[port].full_duplex = (ctx->rx_buf[0] != 0u);
            }

            // Sync all KISS params to HAL after any parameter command.
            // Fields not updated in this command carry their current values.
            hal_channel_params_from_kiss(port, ctx->ports[port].txdelay, ctx->ports[port].persistence, ctx->ports[port].slottime, ctx->ports[port].txtail,
                    ctx->ports[port].full_duplex ? 1U : 0U);

            // Count received command frames
            ctx->stats.rx_cmd_frames++;
        break;

        case KISS_CMD_SETHARDWARE:
            // Hardware-specific: copy raw bytes into port hardware buffer
            size_t copy_len = ctx->rx_len;
            if (copy_len > sizeof(ctx->ports[port].hardware)) {
                copy_len = sizeof(ctx->ports[port].hardware);
            }
            memcpy(ctx->ports[port].hardware, ctx->rx_buf, copy_len);
            ctx->ports[port].hardware_len = (uint8_t) copy_len;
            if (ctx->on_hardware) {
                ctx->on_hardware(ctx, port, ctx->ports[port].hardware, ctx->ports[port].hardware_len, ctx->user_data);
            }

            // Count received command frames
            ctx->stats.rx_cmd_frames++;
        break;

        default:
            // Unknown command: KISS implementations MUST ignore per specification
        break;
    }
}

// ---------------------------------------------------------------------------
// Public API implementations
// ---------------------------------------------------------------------------

uint8_t ax25_kiss_init(ax25_kiss_ctx_t *ctx) {
    if (!ctx) {
        return KISS_ERR_NULL;
    }

    memset(ctx, 0, sizeof(ax25_kiss_ctx_t));

    // Apply specification defaults to all ports
    for (uint8_t i = 0u; i < KISS_MAX_PORTS; i++) {
        ctx->ports[i].txdelay = KISS_DEFAULT_TXDELAY;
        ctx->ports[i].persistence = KISS_DEFAULT_PERSISTENCE;
        ctx->ports[i].slottime = KISS_DEFAULT_SLOTTIME;
        ctx->ports[i].txtail = KISS_DEFAULT_TXTAIL;
        ctx->ports[i].full_duplex = KISS_DEFAULT_FULLDUPLEX;
        ctx->ports[i].hardware_len = 0u;
    }

    ctx->rx_state = KISS_RX_IDLE;
    ctx->rx_len = 0u;
    ctx->rx_got_type = false;
    ctx->rx_type = 0u;
    ctx->kiss_mode = false;
    ctx->rx_at_frame_start = false;

    // Callbacks left NULL; caller populates them after init
    ctx->on_frame = NULL;
    ctx->on_hardware = NULL;
    ctx->on_return = NULL;
    ctx->serial_write = NULL;
    ctx->user_data = NULL;

    return KISS_OK;
}

uint8_t ax25_kiss_enter(ax25_kiss_ctx_t *ctx) {
    if (!ctx) {
        return KISS_ERR_NULL;
    }
    if (!ctx->serial_write) {
        return KISS_ERR_NO_SERIAL;
    }

    // Send a leading FEND to flush any garbage on the line
    uint8_t fend = KISS_FEND;
    ctx->serial_write(&fend, 1u, ctx->user_data);
    ctx->kiss_mode = true;
    return KISS_OK;
}

void ax25_kiss_receive_byte(ax25_kiss_ctx_t *ctx, uint8_t byte) {
    if (!ctx) {
        return;
    }

    // Count every byte fed into the state machine
    ctx->stats.rx_bytes++;

    switch (ctx->rx_state) {
        case KISS_RX_IDLE:
            // Only FEND advances us into frame reception
            if (byte == KISS_FEND) {
                // Reset accumulator for new frame
                ctx->rx_len = 0u;
                ctx->rx_got_type = false;
                ctx->rx_type = 0u;
                ctx->rx_state = KISS_RX_IN_FRAME;
                // Automatically enter KISS mode when we see a FEND from TNC
                ctx->kiss_mode = true;
                // Next byte is the type byte; a FEND here is a port-12 type byte,
                // not an inter-frame gap (we just came from IDLE)
                ctx->rx_at_frame_start = false;
            }
            // All other bytes while IDLE are silently discarded
        break;

        case KISS_RX_IN_FRAME:
            if (byte == KISS_FEND) {
                // KISS_TYPE_BYTE(12, KISS_CMD_DATA) == 0xC0 == KISS_FEND, so receiving
                // a port-12 DATA frame produces two consecutive FENDs in the byte stream.
                // If no type byte has arrived yet, treat this FEND as type byte 0xC0
                // instead of discarding it as an empty inter-frame padding FEND.
                if (!ctx->rx_got_type) {
                    // rx_at_frame_start == true: inter-frame boundary, skip
                    // rx_at_frame_start == false: port-12 DATA type byte (0xC0)
                    if (ctx->rx_at_frame_start) {
                        ctx->rx_len = 0u;
                        ctx->rx_at_frame_start = false;
                    } else {
                        ctx->rx_type = KISS_FEND;
                        ctx->rx_got_type = true;
                    }
                } else {
                    // Normal end-of-frame: dispatch and reset for next frame
                    kiss_dispatch_frame(ctx);
                    ctx->rx_len = 0u;
                    ctx->rx_got_type = false;
                    ctx->rx_type = 0u;
                    // Signal that the next FEND is an inter-frame gap, not a port-12 type byte
                    ctx->rx_at_frame_start = true;
                    // Remain in KISS_RX_IN_FRAME: FEND both ends and starts frames
                }
            } else if (byte == KISS_FESC) {
                // Begin escape sequence
                ctx->rx_state = KISS_RX_ESCAPED;
            } else {
                // Normal data byte
                if (!ctx->rx_got_type) {
                    // First byte is the type indicator (unescaped)
                    ctx->rx_type = byte;
                    ctx->rx_got_type = true;
                    // A real type byte has arrived; clear the inter-frame flag
                    ctx->rx_at_frame_start = false;
                } else {
                    // Append payload byte if buffer not full
                    if (ctx->rx_len < KISS_MAX_FRAME_SIZE) {
                        ctx->rx_buf[ctx->rx_len++] = byte;
                    }
                    // If buffer full: silently drop byte; frame will be malformed
                    // but KISS has no error signaling mechanism
                }
            }
        break;

        case KISS_RX_ESCAPED:
            // Resolve escape sequence back to original byte
        {
            uint8_t unescaped;
            if (byte == KISS_TFEND) {
                unescaped = KISS_FEND;
            } else if (byte == KISS_TFESC) {
                unescaped = KISS_FESC;
            } else {
                // Per spec: any other byte after FESC is an error;
                // no action taken, frame assembly continues unchanged.
                ctx->rx_state = KISS_RX_IN_FRAME;
                return;
            }
            ctx->rx_state = KISS_RX_IN_FRAME;

            // Store unescaped byte as type or payload
            if (!ctx->rx_got_type) {
                ctx->rx_type = unescaped;
                ctx->rx_got_type = true;
                // Escaped type byte consumed; clear inter-frame flag
                ctx->rx_at_frame_start = false;
            } else {
                if (ctx->rx_len < KISS_MAX_FRAME_SIZE) {
                    ctx->rx_buf[ctx->rx_len++] = unescaped;
                } else {
                    // Buffer full: count overflow, byte is dropped
                    ctx->stats.rx_overflows++;
                }
            }
        }
        break;

        default:
            // Should never reach here; reset to safe state
            ctx->rx_state = KISS_RX_IDLE;
            ctx->rx_len = 0u;
            ctx->rx_got_type = false;
        break;
    }
}

void ax25_kiss_receive_bytes(ax25_kiss_ctx_t *ctx, const uint8_t *data, size_t len) {
    if (!ctx || !data) {
        return;
    }
    for (size_t i = 0u; i < len; i++) {
        ax25_kiss_receive_byte(ctx, data[i]);
    }
}

uint8_t ax25_kiss_send_frame(ax25_kiss_ctx_t *ctx, uint8_t port, const uint8_t *frame, size_t len) {
    uint8_t result;
    uint8_t type_byte;

    if (!ctx) {
        return KISS_ERR_NULL;
    }
    if (!ctx->serial_write) {
        return KISS_ERR_NO_SERIAL;
    }
    // Port 0xF is reserved per KISS specification
    if (port >= 0x0Fu) {
        return KISS_ERR_PORT;
    }
    if (len > KISS_MAX_FRAME_SIZE) {
        return KISS_ERR_FRAME_SIZE;
    }

    type_byte = KISS_TYPE_BYTE(port, KISS_CMD_DATA);
    result = kiss_build_and_send(ctx, type_byte, frame, len);
    if (result == KISS_OK) {
        // Increment data frame transmit counter on success
        ctx->stats.tx_frames++;
    }

    return result;
}

uint8_t ax25_kiss_send_command(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t cmd, const uint8_t *data, size_t len) {
    uint8_t type_byte;
    uint8_t result;

    if (!ctx) {
        return KISS_ERR_NULL;
    }
    if (!ctx->serial_write) {
        return KISS_ERR_NO_SERIAL;
    }
    // Port 0xF is reserved
    if (port >= 0x0Fu) {
        return KISS_ERR_PORT;
    }
    // Reject DATA command via this path (use ax25_kiss_send_frame instead)
    // and reject RETURN (use ax25_kiss_send_return instead)
    if (cmd == KISS_CMD_DATA || cmd == KISS_CMD_RETURN) {
        return KISS_ERR_PORT;
    }
    if (len > KISS_MAX_FRAME_SIZE) {
        return KISS_ERR_FRAME_SIZE;
    }

    type_byte = KISS_TYPE_BYTE(port, cmd);
    result = kiss_build_and_send(ctx, type_byte, data, len);
    if (result == KISS_OK) {
        // Increment command frame transmit counter on success
        ctx->stats.tx_cmd_frames++;
    }

    return result;
}

uint8_t ax25_kiss_send_return(ax25_kiss_ctx_t *ctx) {
    if (!ctx) {
        return KISS_ERR_NULL;
    }
    if (!ctx->serial_write) {
        return KISS_ERR_NO_SERIAL;
    }

    // Return command: FEND 0xFF FEND (no payload, no escaping needed for 0xFF)
    uint8_t frame[3];
    frame[0] = KISS_FEND;
    frame[1] = KISS_RETURN_TYPE_BYTE;  // 0xFF - never contains FEND or FESC
    frame[2] = KISS_FEND;
    ctx->serial_write(frame, 3u, ctx->user_data);
    ctx->kiss_mode = false;
    return KISS_OK;
}

uint8_t ax25_kiss_set_port_params(ax25_kiss_ctx_t *ctx, uint8_t port, const ax25_kiss_port_params_t *params) {
    if (!ctx || !params) {
        return KISS_ERR_NULL;
    }
    if (!ctx->serial_write) {
        return KISS_ERR_NO_SERIAL;
    }
    if (port >= 0x0Fu) {
        return KISS_ERR_PORT;
    }

    uint8_t result;
    uint8_t param_byte;

    // Send TxDelay
    param_byte = params->txdelay;
    result = ax25_kiss_send_command(ctx, port, KISS_CMD_TXDELAY, &param_byte, 1u);
    if (result != KISS_OK) {
        return result;
    }

    // Send Persistence
    param_byte = params->persistence;
    result = ax25_kiss_send_command(ctx, port, KISS_CMD_PERSISTENCE, &param_byte, 1u);
    if (result != KISS_OK) {
        return result;
    }

    // Send SlotTime
    param_byte = params->slottime;
    result = ax25_kiss_send_command(ctx, port, KISS_CMD_SLOTTIME, &param_byte, 1u);
    if (result != KISS_OK) {
        return result;
    }

    // Send TxTail (obsolete but included for compatibility)
    param_byte = params->txtail;
    result = ax25_kiss_send_command(ctx, port, KISS_CMD_TXTAIL, &param_byte, 1u);
    if (result != KISS_OK) {
        return result;
    }

    // Send FullDuplex
    param_byte = params->full_duplex ? 1u : 0u;
    result = ax25_kiss_send_command(ctx, port, KISS_CMD_FULLDUPLEX, &param_byte, 1u);
    if (result != KISS_OK) {
        return result;
    }

    // Send SetHardware only if there are hardware bytes to send
    if (params->hardware_len > 0u) {
        result = ax25_kiss_send_command(ctx, port, KISS_CMD_SETHARDWARE, params->hardware, params->hardware_len);
        if (result != KISS_OK) {
            return result;
        }
    }

    // Mirror parameters locally after successful transmission
    ctx->ports[port] = *params;
    return KISS_OK;
}

uint8_t ax25_kiss_get_port_params(const ax25_kiss_ctx_t *ctx, uint8_t port, ax25_kiss_port_params_t *params) {
    if (!ctx || !params) {
        return KISS_ERR_NULL;
    }
    if (port >= KISS_MAX_PORTS) {
        return KISS_ERR_PORT;
    }

    *params = ctx->ports[port];
    return KISS_OK;
}

// sets KISS variant and initializes smack_active state
uint8_t ax25_kiss_set_variant(ax25_kiss_ctx_t *ctx, ax25_kiss_variant_t variant) {
    if (ctx == NULL) {
        return KISS_ERR_NULL;
    }
    ctx->variant = variant;
    // Explicit SMACK: latch smack_active immediately
    ctx->smack_active = (variant == KISS_VARIANT_SMACK);
    // AUTO mode: start inactive; auto-upgrade on first received SMACK frame
    if (variant == KISS_VARIANT_AUTO) {
        ctx->smack_active = false;
    }
    return KISS_OK;
}

// ax25_kiss_get_variant: returns the variant actually in use
uint8_t ax25_kiss_get_variant(const ax25_kiss_ctx_t *ctx, ax25_kiss_variant_t *variant) {
    if (ctx == NULL || variant == NULL) {
        return KISS_ERR_NULL;
    }
    if (ctx->variant == KISS_VARIANT_AUTO) {
        *variant = ctx->smack_active ? KISS_VARIANT_SMACK : KISS_VARIANT_STANDARD;
    } else {
        *variant = ctx->variant;
    }
    return KISS_OK;
}

// ax25_kiss_smack_is_active: reports whether SMACK CRC mode is engaged
uint8_t ax25_kiss_smack_is_active(const ax25_kiss_ctx_t *ctx, bool *active) {
    if (ctx == NULL || active == NULL) {
        return KISS_ERR_NULL;
    }
    *active = ctx->smack_active;
    return KISS_OK;
}

// ax25_kiss_reset_rx: resets only the RX parser state without clearing params
uint8_t ax25_kiss_reset_rx(ax25_kiss_ctx_t *ctx) {
    if (ctx == NULL) {
        return KISS_ERR_NULL;
    }
    ctx->rx_state = KISS_RX_IDLE;
    ctx->rx_len = 0u;
    ctx->rx_got_type = false;
    ctx->rx_type = 0u;
    ctx->rx_at_frame_start = false;
    ctx->rx_double_fesc = false;
    return KISS_OK;
}

// ax25_kiss_reset_stats: zeros all statistics counters
uint8_t ax25_kiss_reset_stats(ax25_kiss_ctx_t *ctx) {
    if (ctx == NULL) {
        return KISS_ERR_NULL;
    }
    memset(&ctx->stats, 0, sizeof(ax25_kiss_stats_t));
    return KISS_OK;
}

// ax25_kiss_get_stats: copies statistics snapshot to caller buffer
uint8_t ax25_kiss_get_stats(const ax25_kiss_ctx_t *ctx, ax25_kiss_stats_t *stats) {
    if (ctx == NULL || stats == NULL) {
        return KISS_ERR_NULL;
    }
    *stats = ctx->stats;
    return KISS_OK;
}

// ax25_kiss_set_poll_mode: enables or disables G8BPQ polled mode
uint8_t ax25_kiss_set_poll_mode(ax25_kiss_ctx_t *ctx, bool enable, uint8_t interval_100ms) {
    if (ctx == NULL) {
        return KISS_ERR_NULL;
    }
    ctx->poll_mode = enable;
    ctx->poll_interval = enable ? interval_100ms : 0u;
    return KISS_OK;
}

// ax25_kiss_set_hw_flowctrl: records hardware flow control state
uint8_t ax25_kiss_set_hw_flowctrl(ax25_kiss_ctx_t *ctx, bool enable) {
    if (ctx == NULL) {
        return KISS_ERR_NULL;
    }
    ctx->hw_flowctrl = enable;
    return KISS_OK;
}

// ax25_kiss_reset_port_params: restores one port to specification defaults
uint8_t ax25_kiss_reset_port_params(ax25_kiss_ctx_t *ctx, uint8_t port) {
    ax25_kiss_port_params_t defaults;
    if (ctx == NULL) {
        return KISS_ERR_NULL;
    }
    if (port >= 0x0Fu) {
        return KISS_ERR_PORT;
    }
    defaults.txdelay = KISS_DEFAULT_TXDELAY;
    defaults.persistence = KISS_DEFAULT_PERSISTENCE;
    defaults.slottime = KISS_DEFAULT_SLOTTIME;
    defaults.txtail = KISS_DEFAULT_TXTAIL;
    defaults.full_duplex = KISS_DEFAULT_FULLDUPLEX;
    defaults.hardware_len = 0u;
    memset(defaults.hardware, 0, sizeof(defaults.hardware));
    return ax25_kiss_set_port_params(ctx, port, &defaults);
}

// ax25_kiss_reset_all_ports: restores all 15 user ports to specification defaults
uint8_t ax25_kiss_reset_all_ports(ax25_kiss_ctx_t *ctx) {
    uint8_t port;
    uint8_t result;
    if (ctx == NULL) {
        return KISS_ERR_NULL;
    }
    for (port = 0u; port < 0x0Fu; port++) {
        result = ax25_kiss_reset_port_params(ctx, port);
        if (result != KISS_OK) {
            return result;
        }
    }
    return KISS_OK;
}
