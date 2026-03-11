/**
 * @file ax25_linux_bridge.c
 * @brief Bridge between libax25v22 and the Linux kernel AF_AX25 socket API
 *
 * Problem: libax25v22 is hardware-agnostic.  The Linux kernel provides AX.25
 * connectivity via AF_AX25 sockets (SOCK_SEQPACKET for connected,
 * SOCK_DGRAM for UI).  We need to:
 *   1. Accept frames from the kernel via SOCK_RAW / SOCK_DGRAM and feed them
 *      into ax25_frame_decode() / ax25_process_frame()
 *   2. Take frames produced by the libax25v22 state machine and inject them
 *      into the kernel via the KISS interface or AF_PACKET
 *   3. Register callbacks so upper-layer code gets connect/data/disconnect events
 *
 * Solution architecture:
 *   ax25_linux_ctx_t holds:
 *     - A KISS context (ax25_kiss_ctx_t) for TNC-facing byte I/O
 *     - An ax25_mux_t for multiplexing multiple logical connections
 *     - An array of ax25_connection_t state machines
 *     - AF_AX25 socket for kernel monitoring / comparison tests
 *
 * No 64-bit arithmetic.  No float.  All timer values in uint32_t ms.
 */

#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

/* Linux AX.25 socket headers */
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>

#include "ax25.h"
#include "ax25_state_machine.h"
#include "ax25_mux.h"
#include "kiss.h"
#include "hal.h"

/* =========================================================================
 * TYPES
 * ========================================================================= */

#define AX25_BRIDGE_MAX_CONNS   AX25_MUX_MAX_LINKS
#define AX25_BRIDGE_MAX_PORTS   2U

typedef struct {
    char ax25_call[7];
} ax25_address;

struct sockaddr_ax25 {
    sa_family_t sax25_family;
    ax25_address sax25_call;
    int sax25_ndigis;
};

/**
 * @brief Per-connection slot
 *
 * Problem: We need to track application-layer state alongside the
 * libax25v22 state machine.
 * Solution: Keep both together in one slot; the void *user_data in
 * ax25_connection_t points back here for callbacks.
 */
typedef struct {
    ax25_connection_t sm; /**< libax25v22 state machine              */
    uint8_t active; /**< 1 = slot in use                       */
    uint8_t link_id; /**< mux link id (from ax25_mux_register)  */
    uint8_t port; /**< HAL port index                        */
    /* application callbacks */
    void (*on_connect)(uint8_t conn_id, int initiated_locally, void *ctx);
    void (*on_disconnect)(uint8_t conn_id, uint8_t reason, void *ctx);
    void (*on_data)(uint8_t conn_id, const uint8_t *data, uint16_t len, uint8_t pid, void *ctx);
    void *app_ctx;
    uint8_t conn_id; /**< index in g_ctx.conns[]                */
} ax25_conn_slot_t;

/**
 * @brief Main bridge context
 */
typedef struct {
    ax25_kiss_ctx_t kiss;
    ax25_mux_t mux;
    ax25_conn_slot_t conns[AX25_BRIDGE_MAX_CONNS];
    uint8_t num_conns;
    uint8_t hal_port;
    /* AF_AX25 kernel socket (optional: for monitoring / comparison) */
    int kernel_sock;
    uint8_t kernel_sock_open;
    /* callsign for this station */
    char mycall[10]; /* e.g. "N0CALL-7\0" */
    uint8_t my_ssid;
} ax25_linux_ctx_t;

/* Singleton bridge context (one instance per application) */
static ax25_linux_ctx_t g_ctx;

/* =========================================================================
 * HELPERS: callsign conversion between libax25v22 and kernel formats
 *
 * Problem: The kernel uses 7-byte AX.25-encoded addresses in ax25_address
 * (6 chars left-shifted 1 bit, SSID byte). libax25v22 uses plain ASCII
 * callsign strings.
 *
 * Solution: Provide bidirectional converters using only 8-bit arithmetic.
 * ========================================================================= */

/**
 * @brief Convert ASCII callsign "CALL-SSID" to kernel ax25_address format.
 *
 * Each callsign char is shifted left 1 bit (bit 0 = extension/reserved).
 * The SSID byte encodes SSID in bits 4-1; bit 0 = extension (1 = last address).
 * No 64-bit, no float.
 */
static void callsign_to_kernel(const char *str, ax25_address *addr) {
    const char *p = str;
    char buf[7];
    uint8_t ssid = 0U;
    uint8_t i;
    const char *dash;

    memset(buf, ' ', 6);
    buf[6] = '\0';

    dash = strchr(str, '-');
    if (dash) {
        /* parse SSID numerically without atoi to avoid locale issues */
        const char *q = dash + 1;
        ssid = 0U;
        while (*q >= '0' && *q <= '9') {
            ssid = (uint8_t) (ssid * 10U + (uint8_t) (*q - '0'));
            q++;
        }
        /* Copy up to 6 chars of callsign */
        for (i = 0; i < 6U && p != dash; i++, p++)
            buf[i] = *p;
    } else {
        for (i = 0; i < 6U && *p; i++, p++)
            buf[i] = *p;
    }

    /* Shift left 1 bit per AX.25 encoding */
    for (i = 0; i < 6U; i++) {
        addr->ax25_call[i] = (uint8_t) ((uint8_t) buf[i] << 1U);
    }
    /* SSID byte: bit 0 = extension (set by caller chain), bits 4-1 = SSID */
    addr->ax25_call[6] = (uint8_t) (0x60U | (uint8_t) (ssid << 1U));
}

/**
 * @brief Convert kernel ax25_address to ASCII "CALL-SSID" string.
 * @param addr   Kernel AX.25 address
 * @param buf    Output buffer (at least 10 bytes)
 */
static void kernel_to_callsign(const ax25_address *addr, char *buf) {
    uint8_t i, ssid;
    char *p = buf;

    for (i = 0; i < 6U; i++) {
        char c = (char) ((uint8_t) (addr->ax25_call[i]) >> 1U);
        if (c != ' ')
            *p++ = c;
    }
    ssid = (uint8_t) ((addr->ax25_call[6] >> 1U) & 0x0FU);
    if (ssid > 0U) {
        *p++ = '-';
        if (ssid >= 10U) {
            *p++ = '1';
            *p++ = (char) ('0' + ssid - 10U);
        } else {
            *p++ = (char) ('0' + ssid);
        }
    }
    *p = '\0';
}

/* =========================================================================
 * KISS CALLBACKS: feed KISS-decoded frames into the libax25v22 stack
 * ========================================================================= */

/**
 * @brief Called by KISS layer when a complete AX.25 frame is received.
 *
 * Problem: KISS delivers raw AX.25 bytes (no HDLC flags, no FCS).
 * libax25v22 expects decoded ax25_frame_t* with no FCS.
 *
 * Solution: call ax25_frame_decode() then route through the mux.
 * FCS is already stripped by the hardware TNC; we pass MODULO128_AUTO
 * so the library detects mod-8 vs mod-128 automatically.
 */
static void on_kiss_frame(ax25_kiss_ctx_t *ctx, uint8_t port, uint8_t *frame, size_t len, void *user_data) {
    ax25_frame_t *f;
    uint8_t err = 0U;
    uint32_t tick;
    (void) ctx;
    (void) port;
    (void) user_data;

    if (len < AX25_MIN_FRAME_SIZE_NO_FCS) {
        hal_log(HAL_LOG_WARN, "KISS RX: frame too short (%u bytes)", (unsigned) len);
        return;
    }

    f = ax25_frame_decode(frame, len, MODULO128_AUTO, &err);
    if (!f || err) {
        hal_log(HAL_LOG_WARN, "frame_decode error=%u", err);
        if (f) {
            uint8_t fe = 0U;
            ax25_frame_free(f, &fe);
        }
        return;
    }

    tick = hal_tick_ms() / 10U; /* convert ms -> 10ms ticks */
    ax25_mux_receive_frame(&g_ctx.mux, f, (uint32_t) tick);

    {
        uint8_t fe = 0U;
        ax25_frame_free(f, &fe);
    }
}

/**
 * @brief KISS serial write callback — pushes bytes into the HAL TX ring.
 *
 * This is how the KISS layer sends bytes toward the TNC.
 */
static void kiss_serial_write(uint8_t *data, size_t len, void *user_data) {
    uint8_t port = (uint8_t) (uintptr_t) user_data;
    hal_serial_write(port, data, (uint16_t) len);
}

/* =========================================================================
 * STATE MACHINE CALLBACKS: upper layer receives events from libax25v22
 * ========================================================================= */

/** @brief Called when connection is established (DL-CONNECT indication) */
static void sm_on_connect(void *user_data, bool initiated_locally) {
    ax25_conn_slot_t *slot = (ax25_conn_slot_t*) user_data;
    hal_log(HAL_LOG_INFO, "CONN[%u] connected (local=%d)", slot->conn_id, (int) initiated_locally);
    if (slot->on_connect)
        slot->on_connect(slot->conn_id, (int) initiated_locally, slot->app_ctx);
}

/** @brief Called when link is terminated (DL-DISCONNECT indication) */
static void sm_on_disconnect(void *user_data, uint8_t reason) {
    ax25_conn_slot_t *slot = (ax25_conn_slot_t*) user_data;
    hal_log(HAL_LOG_INFO, "CONN[%u] disconnected reason=%u", slot->conn_id, reason);
    if (slot->on_disconnect)
        slot->on_disconnect(slot->conn_id, reason, slot->app_ctx);
}

/** @brief Called when I-frame data arrives (DL-DATA indication) */
static void sm_on_data(void *user_data, uint8_t *data, size_t len, uint8_t pid) {
    ax25_conn_slot_t *slot = (ax25_conn_slot_t*) user_data;
    hal_log(HAL_LOG_DEBUG, "CONN[%u] data: %u bytes pid=0x%02X", slot->conn_id, (unsigned) len, pid);
    if (slot->on_data)
        slot->on_data(slot->conn_id, data, (uint16_t) len, pid, slot->app_ctx);
}

/** @brief Called for received UI frames */
static void sm_on_ui(void *user_data, const ax25_address_t *src, uint8_t *data, size_t len, uint8_t pid) {
    (void) user_data;
    hal_log(HAL_LOG_DEBUG, "UI from %s len=%u pid=0x%02X", src ? src->callsign : "?", (unsigned) len, pid);
}

/**
 * @brief Transmit callback — sends encoded frame through KISS.
 *
 * Problem: The state machine calls this with a raw AX.25 frame (no HDLC
 * flags, no FCS).  We must wrap it in KISS and deliver via the TX ring.
 *
 * Solution: ax25_kiss_send_frame() does the KISS framing.
 */
static void sm_transmit(void *user_data, uint8_t *frame, size_t len) {
    ax25_conn_slot_t *slot = (ax25_conn_slot_t*) user_data;
    uint8_t rc;
    hal_log(HAL_LOG_DEBUG, "CONN[%u] TX %u bytes", slot->conn_id, (unsigned) len);
    rc = ax25_kiss_send_frame(&g_ctx.kiss, slot->port, frame, len);
    if (rc != KISS_OK) {
        hal_log(HAL_LOG_WARN, "CONN[%u] kiss_send_frame err=%u", slot->conn_id, rc);
    }
}

/* Standalone UI transmit trampoline (no connection context) */
static void sm_transmit_ui(uint8_t *frame, size_t len) {
    ax25_kiss_send_frame(&g_ctx.kiss, g_ctx.hal_port, frame, len);
}

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialize the bridge.
 *
 * @param mycall  Station callsign (e.g. "N0CALL-7")
 * @param port    HAL port to use for KISS I/O (0–3)
 */
int ax25_bridge_init(const char *mycall, uint8_t port) {
    uint8_t err;
    const char *dash;
    uint8_t i;

    if (!mycall)
        return -1;

    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.hal_port = port;
    g_ctx.kernel_sock = -1;

    /* Parse mycall -> callsign + ssid */
    strncpy(g_ctx.mycall, mycall, sizeof(g_ctx.mycall) - 1U);
    dash = strchr(mycall, '-');
    if (dash) {
        g_ctx.my_ssid = (uint8_t) (dash[1] - '0');
        if (dash[2] >= '0' && dash[2] <= '9')
            g_ctx.my_ssid = (uint8_t) (g_ctx.my_ssid * 10U + (uint8_t) (dash[2] - '0'));
    }

    /* Init XID defaults */
    err = 0U;
    ax25_xid_init_defaults(&err);
    if (err) {
        hal_log(HAL_LOG_ERROR, "xid_init_defaults failed");
        return -1;
    }

    /* Init mux */
    if (ax25_mux_init(&g_ctx.mux) != 0U) {
        hal_log(HAL_LOG_ERROR, "mux_init failed");
        return -1;
    }

    /* Init KISS context */
    if (ax25_kiss_init(&g_ctx.kiss) != KISS_OK) {
        hal_log(HAL_LOG_ERROR, "kiss_init failed");
        return -1;
    }
    g_ctx.kiss.on_frame = on_kiss_frame;
    g_ctx.kiss.serial_write = kiss_serial_write;
    g_ctx.kiss.user_data = (void*) (uintptr_t) port;
    ax25_kiss_enter(&g_ctx.kiss);

    /* Mark all slots free */
    for (i = 0; i < AX25_BRIDGE_MAX_CONNS; i++) {
        g_ctx.conns[i].active = 0U;
        g_ctx.conns[i].conn_id = i;
    }

    hal_log(HAL_LOG_INFO, "Bridge init: call=%s port=%u", mycall, port);
    return 0;
}

/**
 * @brief Connect to a remote station.
 *
 * @param dest_call   Remote callsign (e.g. "W1AW-3")
 * @param on_connect  Called when link is up
 * @param on_disc     Called when link is down
 * @param on_data     Called with received data
 * @param app_ctx     Opaque pointer passed to all callbacks
 * @param mod128      1 = request modulo-128 (SABME), 0 = modulo-8 (SABM)
 * @return conn_id (0–7) or negative on error
 */
int ax25_bridge_connect(const char *dest_call, void (*on_connect)(uint8_t, int, void*), void (*on_disc)(uint8_t, uint8_t, void*),
        void (*on_data)(uint8_t, const uint8_t*, uint16_t, uint8_t, void*), void *app_ctx, uint8_t mod128) {
    ax25_conn_slot_t *slot = NULL;
    ax25_callbacks_t cbs;
    ax25_address_t local_addr, peer_addr;
    uint8_t i, err, link_id;

    /* Find free slot */
    for (i = 0; i < AX25_BRIDGE_MAX_CONNS; i++) {
        if (!g_ctx.conns[i].active) {
            slot = &g_ctx.conns[i];
            break;
        }
    }
    if (!slot) {
        hal_log(HAL_LOG_ERROR, "No free connection slots");
        return -1;
    }

    /* Fill slot */
    memset(slot, 0, sizeof(*slot));
    slot->active = 1U;
    slot->conn_id = i;
    slot->port = g_ctx.hal_port;
    slot->on_connect = on_connect;
    slot->on_disconnect = on_disc;
    slot->on_data = on_data;
    slot->app_ctx = app_ctx;

    /* Build callbacks for state machine */
    memset(&cbs, 0, sizeof(cbs));
    cbs.on_connect = sm_on_connect;
    cbs.on_disconnect = sm_on_disconnect;
    cbs.on_data = sm_on_data;
    cbs.on_ui_data = sm_on_ui;
    cbs.transmit = sm_transmit;

    err = 0U;
    if (ax25_connection_init(&slot->sm, &cbs, slot) != 0U) {
        hal_log(HAL_LOG_ERROR, "connection_init failed");
        slot->active = 0U;
        return -1;
    }

    slot->sm.want_mod128 = mod128;

    /* Build addresses */
    err = 0U;
    {
        ax25_address_t *pa = ax25_address_from_string(dest_call, &err);
        if (!pa || err) {
            hal_log(HAL_LOG_ERROR, "bad dest callsign: %s", dest_call);
            if (pa) {
                uint8_t fe = 0U;
                ax25_address_free(pa, &fe);
            }
            slot->active = 0U;
            return -1;
        }
        peer_addr = *pa;
        {
            uint8_t fe = 0U;
            ax25_address_free(pa, &fe);
        }
    }
    {
        ax25_address_t *la = ax25_address_from_string(g_ctx.mycall, &err);
        if (!la || err) {
            hal_log(HAL_LOG_ERROR, "bad local callsign: %s", g_ctx.mycall);
            if (la) {
                uint8_t fe = 0U;
                ax25_address_free(la, &fe);
            }
            slot->active = 0U;
            return -1;
        }
        local_addr = *la;
        {
            uint8_t fe = 0U;
            ax25_address_free(la, &fe);
        }
    }

    /* Register with mux */
    if (ax25_mux_register_link(&g_ctx.mux, &slot->sm, &local_addr, &peer_addr, &link_id) != 0U) {
        hal_log(HAL_LOG_ERROR, "mux_register failed");
        slot->active = 0U;
        return -1;
    }
    slot->link_id = link_id;

    /* Set up transmit adapter via mux */
    ax25_mux_set_lm_seize_confirm(&g_ctx.mux, link_id, ax25_mux_transmit_adapter, slot);

    /* Issue DL-CONNECT */
    if (ax25_connect(&slot->sm, &peer_addr, &local_addr) != 0U) {
        hal_log(HAL_LOG_ERROR, "ax25_connect failed");
        ax25_mux_unregister_link(&g_ctx.mux, link_id);
        slot->active = 0U;
        return -1;
    }

    hal_log(HAL_LOG_INFO, "Connecting: %s -> %s (conn_id=%u mod128=%u)", g_ctx.mycall, dest_call, i, mod128);
    return (int) i;
}

/**
 * @brief Send data on an established connection.
 *
 * Problem: The upper layer has arbitrary-sized data to send.
 * Solution: Feed it to ax25_send_data() which queues I-frames.
 * Segmentation (for payloads > N1) is handled transparently by the
 * segmenter module in libax25v22.
 *
 * @param conn_id   Connection identifier from ax25_bridge_connect()
 * @param data      Payload bytes
 * @param len       Payload length
 * @param pid       Protocol identifier (PID_NO_L3 = 0xF0 for plain text)
 * @return 0 on success, negative on error
 */
int ax25_bridge_send(uint8_t conn_id, const uint8_t *data, uint16_t len, uint8_t pid) {
    uint8_t rc;
    if (conn_id >= AX25_BRIDGE_MAX_CONNS || !g_ctx.conns[conn_id].active)
        return -1;
    rc = ax25_send_data(&g_ctx.conns[conn_id].sm, (uint8_t*) data, (size_t) len, pid);
    return (rc == 0U) ? 0 : -1;
}

/**
 * @brief Send a UI (unconnected information) frame.
 *
 * Connectionless broadcast, suitable for APRS, beacons, etc.
 *
 * @param dest_call   Destination callsign (e.g. "APRS" or "CQ")
 * @param data        Payload
 * @param len         Payload length
 * @param pid         Protocol ID (PID_NO_L3 = 0xF0 for APRS)
 * @return 0 on success, negative on error
 */
int ax25_bridge_send_ui(const char *dest_call, const uint8_t *data, uint16_t len, uint8_t pid) {
    ax25_address_t *dest = NULL, *src = NULL;
    uint8_t err = 0U, rc;

    dest = ax25_address_from_string(dest_call, &err);
    if (!dest || err)
        goto fail;
    src = ax25_address_from_string(g_ctx.mycall, &err);
    if (!src || err)
        goto fail;

    rc = ax25_send_ui(dest, src, (uint8_t*) data, (size_t) len, pid, sm_transmit_ui);
    if (dest) {
        uint8_t fe = 0U;
        ax25_address_free(dest, &fe);
    }
    if (src) {
        uint8_t fe = 0U;
        ax25_address_free(src, &fe);
    }
    return (rc == 0U) ? 0 : -1;
    fail:
    if (dest) {
        uint8_t fe = 0U;
        ax25_address_free(dest, &fe);
    }
    if (src) {
        uint8_t fe = 0U;
        ax25_address_free(src, &fe);
    }
    return -1;
}

/**
 * @brief Disconnect from a remote station.
 * @param conn_id  Connection identifier
 */
void ax25_bridge_disconnect(uint8_t conn_id) {
    if (conn_id >= AX25_BRIDGE_MAX_CONNS || !g_ctx.conns[conn_id].active)
        return;
    ax25_disconnect(&g_ctx.conns[conn_id].sm);
    hal_log(HAL_LOG_INFO, "Disconnect requested: conn_id=%u", conn_id);
}

/**
 * @brief Main service loop — call as often as possible (at least every 10 ms).
 *
 * Problem: The AX.25 state machine needs periodic ticks for timer expiry.
 * KISS needs byte-by-byte feeding from the serial ring.
 * The mux needs to service pending seize requests.
 *
 * Solution: ax25_bridge_tick() does everything in one call:
 *   1. hal_port_poll() — service serial I/O
 *   2. Drain RX ring into KISS state machine
 *   3. Call ax25_mux_tick() for all links
 *   4. Call ax25_tick() per connection for timer processing
 */
void ax25_bridge_tick(void) {
    uint8_t byte, i;
    uint32_t tick_10ms;

    /* 2. Feed received bytes into KISS state machine */
    while (hal_serial_get(g_ctx.hal_port, &byte) == HAL_OK) {
        ax25_kiss_receive_byte(&g_ctx.kiss, byte);
    }

    /* 3. Compute current 10ms tick */
    tick_10ms = hal_tick_ms() / 10U;

    /* 4. Tick the mux */
    ax25_mux_tick(&g_ctx.mux, tick_10ms);

    /* 5. Tick each active connection */
    for (i = 0; i < AX25_BRIDGE_MAX_CONNS; i++) {
        if (g_ctx.conns[i].active) {
            ax25_tick(&g_ctx.conns[i].sm, tick_10ms);
        }
    }
}

/**
 * @brief Open AF_AX25 kernel socket for monitoring.
 *
 * Problem: When running alongside the kernel AX.25 stack we want to
 * compare what the kernel receives vs what libax25v22 sees.
 *
 * Solution: Open a raw AF_AX25 SOCK_PACKET-equivalent socket bound to
 * the configured interface.  Frames received here are logged but not
 * processed by libax25v22 (they're already handled via KISS).
 *
 * @param ifname  Network interface name (e.g. "ax0")
 * @return 0 on success, -1 on failure (not fatal)
 */
int ax25_bridge_open_kernel_monitor(const char *ifname) {
    struct sockaddr_ax25 sa;
    int fd;

    /* Use SOCK_DGRAM for UI monitor (connectionless raw AX.25 frames) */
    fd = socket(AF_AX25, SOCK_DGRAM, 0);
    if (fd < 0) {
        hal_log(HAL_LOG_WARN, "AF_AX25 socket: %s (kernel AX.25 not available?)", strerror(errno));
        return -1;
    }

    memset(&sa, 0, sizeof(sa));
    sa.sax25_family = AF_AX25;
    callsign_to_kernel(g_ctx.mycall, &sa.sax25_call);

    if (bind(fd, (struct sockaddr*) &sa, sizeof(sa)) < 0) {
        hal_log(HAL_LOG_WARN, "AF_AX25 bind(%s): %s", ifname, strerror(errno));
        close(fd);
        return -1;
    }

    /* Bind to specific interface if requested */
    if (ifname && *ifname) {
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
        if (setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, ifr.ifr_name, (socklen_t) strlen(ifr.ifr_name)) < 0) {
            hal_log(HAL_LOG_WARN, "SO_BINDTODEVICE %s: %s", ifname, strerror(errno));
        }
    }

    /* Non-blocking */
    fcntl(fd, F_SETFL, O_NONBLOCK);
    g_ctx.kernel_sock = fd;
    g_ctx.kernel_sock_open = 1U;
    hal_log(HAL_LOG_INFO, "Kernel AX.25 monitor open: iface=%s", ifname ? ifname : "any");
    return 0;
}

/**
 * @brief Service the kernel monitor socket — call from ax25_bridge_tick() if needed.
 *
 * Drains any UI frames the kernel received and logs them.
 * Non-blocking; returns immediately if no data available.
 */
void ax25_bridge_poll_kernel(void) {
    uint8_t buf[340];
    ssize_t n;
    struct sockaddr_ax25 from;
    socklen_t flen = (socklen_t) sizeof(from);
    char src_call[10];

    if (!g_ctx.kernel_sock_open || g_ctx.kernel_sock < 0)
        return;

    n = recvfrom(g_ctx.kernel_sock, buf, sizeof(buf), MSG_DONTWAIT, (struct sockaddr*) &from, &flen);
    if (n <= 0)
        return;

    kernel_to_callsign(&from.sax25_call, src_call);
    hal_log(HAL_LOG_INFO, "Kernel RX: %u bytes from %s", (unsigned) n, src_call);
}

/**
 * @brief Shut down the bridge cleanly.
 */
void ax25_bridge_deinit(void) {
    uint8_t i, err;
    for (i = 0; i < AX25_BRIDGE_MAX_CONNS; i++) {
        if (g_ctx.conns[i].active) {
            ax25_disconnect(&g_ctx.conns[i].sm);
            ax25_mux_unregister_link(&g_ctx.mux, g_ctx.conns[i].link_id);
            g_ctx.conns[i].active = 0U;
        }
    }
    if (g_ctx.kernel_sock >= 0) {
        close(g_ctx.kernel_sock);
        g_ctx.kernel_sock = -1;
    }
    err = 0U;
    ax25_xid_deinit_defaults(&err);
    hal_log(HAL_LOG_INFO, "Bridge deinit complete");
}
