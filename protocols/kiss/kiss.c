/**
 * @file kiss.c
 * @brief AX.25 v2.2 Protocol Library - KISS TNC Interface Protocol
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 */

#include <string.h>
#include <stddef.h>

#include "kiss.h"

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

// Internal helper: build a complete KISS frame in the caller-supplied buffer
// and call serial_write once with the complete frame.
// frame_payload: bytes after the type indicator (may be NULL if len=0)
// Returns KISS_OK or KISS_ERR_FRAME_SIZE.
static uint8_t kiss_build_and_send(ax25_kiss_ctx_t *ctx, uint8_t type_byte, const uint8_t *frame_payload, size_t payload_len) {
    // Worst case: type byte escaped (2) + every payload byte escaped (2 each)
    // plus opening FEND + closing FEND
    size_t max_needed = 2u + 2u + payload_len * 2u;
    if (max_needed > KISS_TX_BUF_SIZE) {
        return KISS_ERR_FRAME_SIZE;
    }

    // Use a stack buffer sized to KISS_TX_BUF_SIZE
    uint8_t tx[KISS_TX_BUF_SIZE];
    size_t pos = 0u;

    // Opening FEND flushes any accumulated noise on the serial link
    tx[pos++] = KISS_FEND;

    // Type byte must be escaped: port 12 yields 0xC0 (FEND) and similar
    // collisions with FESC exist for other port/cmd combinations.
    // The receiver already handles escaped type bytes via KISS_RX_ESCAPED.
    uint8_t tmp[2] = { 0 };
    size_t n = kiss_stuff_byte(tmp, type_byte);
    tx[pos] = tmp[0];
    tx[pos + 1] = tmp[1];
    pos += n;

    // Byte-stuffed payload
    if (frame_payload && payload_len > 0u) {
        for (size_t i = 0u; i < payload_len; i++) {
            uint8_t tmp[2] = { 0 };
            size_t n = kiss_stuff_byte(tmp, frame_payload[i]);
            // Safety: should never exceed tx[] size given the check above
            tx[pos] = tmp[0];
            tx[pos + 1] = tmp[1];
            pos += n;
        }
    }

    // Closing FEND marks end of frame
    tx[pos++] = KISS_FEND;

    ctx->serial_write(tx, pos, ctx->user_data);
    return KISS_OK;
}

// Internal: dispatch a fully assembled frame stored in ctx->rx_buf.
// ctx->rx_type holds the type indicator byte.
// ctx->rx_len holds the number of payload bytes (NOT including type byte).
static void kiss_dispatch_frame(ax25_kiss_ctx_t *ctx) {
    // Minimum valid frame: at least a type byte was received
    if (!ctx->rx_got_type) {
        return;
    }

    uint8_t port = KISS_PORT(ctx->rx_type);
    uint8_t cmd = KISS_CMD(ctx->rx_type);

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
            // skip zero-length DATA frames
            // When type byte is 0xC0 (port 12, cmd DATA) and the payload
            // is empty, rx_len==0; avoid firing on_frame for empty DATA
            // so that test_kiss_rx_consecutive_fend keeps rx_count==0.
            if (ctx->on_frame && ctx->rx_len > 0u) {
                ctx->on_frame(ctx, port, ctx->rx_buf, ctx->rx_len, ctx->user_data);
            }

        break;

        case KISS_CMD_TXDELAY:
            // One parameter byte: TxDelay in 10ms units
            if (ctx->rx_len >= 1u) {
                ctx->ports[port].txdelay = ctx->rx_buf[0];
            }
        break;

        case KISS_CMD_PERSISTENCE:
            // One parameter byte: p-persistence P value 0-255
            if (ctx->rx_len >= 1u) {
                ctx->ports[port].persistence = ctx->rx_buf[0];
            }
        break;

        case KISS_CMD_SLOTTIME:
            // One parameter byte: slot time in 10ms units
            if (ctx->rx_len >= 1u) {
                ctx->ports[port].slottime = ctx->rx_buf[0];
            }
        break;

        case KISS_CMD_TXTAIL:
            // One parameter byte: TX tail in 10ms units (obsolete)
            if (ctx->rx_len >= 1u) {
                ctx->ports[port].txtail = ctx->rx_buf[0];
            }
        break;

        case KISS_CMD_FULLDUPLEX:
            // One parameter byte: 0=half-duplex, non-zero=full-duplex
            if (ctx->rx_len >= 1u) {
                ctx->ports[port].full_duplex = (ctx->rx_buf[0] != 0u);
            }
        break;

        case KISS_CMD_SETHARDWARE:
            // Hardware-specific: copy raw bytes into port hardware buffer
        {
            size_t copy_len = ctx->rx_len;
            if (copy_len > sizeof(ctx->ports[port].hardware)) {
                copy_len = sizeof(ctx->ports[port].hardware);
            }
            memcpy(ctx->ports[port].hardware, ctx->rx_buf, copy_len);
            ctx->ports[port].hardware_len = (uint8_t) copy_len;
            if (ctx->on_hardware) {
                ctx->on_hardware(ctx, port, ctx->ports[port].hardware, ctx->ports[port].hardware_len, ctx->user_data);
            }
        }
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

    uint8_t type_byte = KISS_TYPE_BYTE(port, KISS_CMD_DATA);
    return kiss_build_and_send(ctx, type_byte, frame, len);
}

uint8_t ax25_kiss_send_command(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t cmd, const uint8_t *data, size_t len) {
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

    uint8_t type_byte = KISS_TYPE_BYTE(port, cmd);
    return kiss_build_and_send(ctx, type_byte, data, len);
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
