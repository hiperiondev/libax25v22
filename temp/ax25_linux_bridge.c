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
 *
 * FIXES applied vs original:
 *   FIX-B1: kernel_sock_is_packet is now set from which socket type was
 *            actually opened, not from whether ifname was non-NULL.
 *   FIX-B2: sm_on_ui now forwards received UI frames to the per-connection
 *            on_ui callback registered via ax25_bridge_set_ui_handler().
 *   NEW:     ax25_bridge_send_ui(), ax25_bridge_set_ui_handler(),
 *            ax25_bridge_set_serial_write_cb(), ax25_bridge_inject_rx_bytes(),
 *            ax25_bridge_encode_callsign(), ax25_bridge_decode_callsign(),
 *            ax25_bridge_max_connections(), ax25_bridge_tick_manual()
 *            are now publicly declared and available for testing.
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

#if defined(__has_include)
#  if __has_include(<netax25/ax25.h>)
#    include <netax25/ax25.h>
#    define AX25_BRIDGE_HAVE_KERNEL_HEADERS 1
#  endif
#endif
#include <netpacket/packet.h>
#include <linux/if_ether.h>
#include <errno.h>

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

#ifndef AX25_BRIDGE_HAVE_KERNEL_HEADERS
typedef struct {
    char ax25_call[7];
} ax25_address;

struct sockaddr_ax25 {
    sa_family_t  sax25_family;
    ax25_address sax25_call;
    int          sax25_ndigis;
};
#endif

/**
 * @brief Per-connection slot
 */
typedef struct {
    ax25_connection_t sm;        /**< libax25v22 state machine              */
    uint8_t           active;    /**< 1 = slot in use                       */
    uint8_t           link_id;   /**< mux link id (from ax25_mux_register)  */
    uint8_t           port;      /**< HAL port index                        */
    void (*on_connect)(uint8_t conn_id, int initiated_locally, void *ctx);
    void (*on_disconnect)(uint8_t conn_id, uint8_t reason, void *ctx);
    void (*on_data)(uint8_t conn_id, const uint8_t *data, uint16_t len,
                    uint8_t pid, void *ctx);
    /* FIX-B2: UI receive callback (was missing, causing UI frames to be
     * silently dropped at the bridge layer) */
    void (*on_ui)(uint8_t conn_id, const uint8_t *data, uint16_t len,
                  uint8_t pid, void *ctx);
    void    *app_ctx;
    void    *ui_ctx;             /**< context pointer passed to on_ui       */
    uint8_t  conn_id;            /**< index in g_ctx.conns[]                */
} ax25_conn_slot_t;

/**
 * @brief Main bridge context
 */
typedef struct {
    ax25_kiss_ctx_t    kiss;
    ax25_mux_t         mux;
    ax25_conn_slot_t   conns[AX25_BRIDGE_MAX_CONNS];
    uint8_t            num_conns;
    uint8_t            hal_port;
    int                kernel_sock;
    uint8_t            kernel_sock_open;
    char               mycall[10];
    uint8_t            kernel_sock_is_packet; /* 1=AF_PACKET, 0=AF_AX25 */
    uint32_t           kiss_current_tick;
} ax25_linux_ctx_t;

static ax25_linux_ctx_t g_ctx;

/**
 * @brief Extract numeric SSID from "CALLSIGN-SS" string.
 * @param str  NUL-terminated callsign string (e.g. "W1AW-15")
 * @return SSID value 0–15; 0 if no dash present or SSID out of range.
 */
static uint8_t parse_ssid_from_string(const char *str) {
    const char *dash;
    uint8_t     ssid = 0U;

    if (!str) return 0U;
    dash = strchr(str, '-');
    if (dash) {
        const char *q = dash + 1;
        while (*q >= '0' && *q <= '9') {
            ssid = (uint8_t)(ssid * 10U + (uint8_t)(*q - '0'));
            q++;
        }
        if (ssid > 15U) ssid = 0U;   /* clamp on invalid SSID */
    }
    return ssid;
}

/* =========================================================================
 * HELPERS: callsign conversion
 *
 * Problem: The kernel uses 7-byte AX.25-encoded addresses. libax25v22 uses
 * plain ASCII callsign strings.
 *
 * Solution: Bidirectional converters using only 8-bit arithmetic.
 * ========================================================================= */

/**
 * @brief Convert ASCII "CALL-SSID" to kernel ax25_address format.
 *
 * Each character is shifted left 1 bit. SSID in bits 4-1 of the 7th byte.
 * No 64-bit, no float.
 */
static void callsign_to_kernel(const char *str, ax25_address *addr) {
    const char *p   = str;
    const char *dash;
    char        buf[7];
    uint8_t     ssid;
    uint8_t     i;

    memset(buf, ' ', 6);
    buf[6] = '\0';

    ssid = parse_ssid_from_string(str);

    dash = strchr(str, '-');
    if (dash) {
        for (i = 0U; i < 6U && p != dash; i++, p++)
            buf[i] = *p;
    } else {
        for (i = 0U; i < 6U && *p; i++, p++)
            buf[i] = *p;
    }

    for (i = 0U; i < 6U; i++) {
        addr->ax25_call[i] = (uint8_t)((uint8_t)buf[i] << 1U);
    }
    addr->ax25_call[6] = (uint8_t)(0x60U | (uint8_t)(ssid << 1U));
}

/**
 * @brief Convert kernel ax25_address to ASCII "CALL-SSID" string.
 * @param addr  Kernel AX.25 address
 * @param buf   Output buffer (at least 10 bytes)
 */
static void kernel_to_callsign(const ax25_address *addr, char *buf) {
    uint8_t i, ssid;
    char   *p = buf;

    for (i = 0U; i < 6U; i++) {
        char c = (char)((uint8_t)(addr->ax25_call[i]) >> 1U);
        if (c != ' ')
            *p++ = c;
    }
    ssid = (uint8_t)((addr->ax25_call[6] >> 1U) & 0x0FU);
    if (ssid > 0U) {
        *p++ = '-';
        if (ssid >= 10U) {
            *p++ = '1';
            *p++ = (char)('0' + ssid - 10U);
        } else {
            *p++ = (char)('0' + ssid);
        }
    }
    *p = '\0';
}

/* =========================================================================
 * KISS CALLBACKS
 * ========================================================================= */

/**
 * @brief Called by KISS layer when a complete AX.25 frame is received.
 */
static void on_kiss_frame(ax25_kiss_ctx_t *ctx, uint8_t port,
                           uint8_t *frame, size_t len, void *user_data) {
    ax25_frame_t *f;
    uint8_t       err = 0U;
    (void)ctx;
    (void)port;
    (void)user_data;

    if (len < AX25_MIN_FRAME_SIZE_NO_FCS) {
        hal_log(HAL_LOG_WARN, "KISS RX: frame too short (%u bytes)", (unsigned)len);
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

    ax25_mux_receive_frame(&g_ctx.mux, f, g_ctx.kiss_current_tick);

    {
        uint8_t fe = 0U;
        ax25_frame_free(f, &fe);
    }
}

/**
 * @brief KISS serial write callback.
 */
static void kiss_serial_write(uint8_t *data, size_t len, void *user_data) {
    uint8_t port = (uint8_t)(uintptr_t)user_data;
    hal_serial_write(port, data, (uint16_t)len);
}

/* =========================================================================
 * STATE MACHINE CALLBACKS
 * ========================================================================= */

static void sm_on_connect(void *user_data, bool initiated_locally) {
    ax25_conn_slot_t *slot = (ax25_conn_slot_t*)user_data;
    hal_log(HAL_LOG_INFO, "CONN[%u] connected (local=%d)",
            slot->conn_id, (int)initiated_locally);
    if (slot->on_connect)
        slot->on_connect(slot->conn_id, (int)initiated_locally, slot->app_ctx);
}

static void sm_on_disconnect(void *user_data, uint8_t reason) {
    ax25_conn_slot_t *slot = (ax25_conn_slot_t*)user_data;
    hal_log(HAL_LOG_INFO, "CONN[%u] disconnected reason=%u",
            slot->conn_id, reason);
    if (slot->on_disconnect)
        slot->on_disconnect(slot->conn_id, reason, slot->app_ctx);
}

static void sm_on_data(void *user_data, uint8_t *data, size_t len, uint8_t pid) {
    ax25_conn_slot_t *slot = (ax25_conn_slot_t*)user_data;
    hal_log(HAL_LOG_DEBUG, "CONN[%u] data: %u bytes pid=0x%02X",
            slot->conn_id, (unsigned)len, pid);
    if (slot->on_data)
        slot->on_data(slot->conn_id, data, (uint16_t)len, pid, slot->app_ctx);
}

/**
 * FIX-B2: sm_on_ui previously only called hal_log and discarded the frame,
 * making it impossible for applications to receive UI traffic.  It now
 * forwards the frame to the per-connection on_ui callback when set.
 */
static void sm_on_ui(void *user_data, const ax25_address_t *src,
                     uint8_t *data, size_t len, uint8_t pid) {
    ax25_conn_slot_t *slot = (ax25_conn_slot_t*)user_data;
    hal_log(HAL_LOG_DEBUG, "UI from %s len=%u pid=0x%02X",
            src ? src->callsign : "?", (unsigned)len, pid);
    if (slot && slot->on_ui)
        slot->on_ui(slot->conn_id, data, (uint16_t)len, pid, slot->ui_ctx);
}

static void sm_transmit(void *user_data, uint8_t *frame, size_t len) {
    ax25_conn_slot_t *slot = (ax25_conn_slot_t*)user_data;
    uint8_t rc;
    hal_log(HAL_LOG_DEBUG, "CONN[%u] TX %u bytes", slot->conn_id, (unsigned)len);
    rc = ax25_kiss_send_frame(&g_ctx.kiss, slot->port, frame, len);
    if (rc != KISS_OK) {
        hal_log(HAL_LOG_WARN, "CONN[%u] kiss_send_frame err=%u", slot->conn_id, rc);
    }
}

static void sm_transmit_ui(uint8_t *frame, size_t len) {
    ax25_kiss_send_frame(&g_ctx.kiss, g_ctx.hal_port, frame, len);
}

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialize the bridge.
 */
int ax25_bridge_init(const char *mycall, uint8_t port) {
    uint8_t err;
    uint8_t i;

    if (!mycall)
        return -1;

    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.hal_port    = port;
    g_ctx.kernel_sock = -1;
    g_ctx.kiss_current_tick = 0U;

    strncpy(g_ctx.mycall, mycall, sizeof(g_ctx.mycall) - 1U);

    (void)parse_ssid_from_string;  /* suppress unused-function warning */

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
    g_ctx.kiss.on_frame     = on_kiss_frame;
    g_ctx.kiss.serial_write = kiss_serial_write;
    g_ctx.kiss.user_data    = (void*)(uintptr_t)port;
    ax25_kiss_enter(&g_ctx.kiss);

    for (i = 0U; i < AX25_BRIDGE_MAX_CONNS; i++) {
        g_ctx.conns[i].active  = 0U;
        g_ctx.conns[i].conn_id = i;
    }

    hal_log(HAL_LOG_INFO, "Bridge init: call=%s port=%u", mycall, port);

    return 0;
}

/**
 * @brief Connect to a remote station.
 */
int ax25_bridge_connect(
        const char *dest_call,
        void (*on_connect)(uint8_t, int, void*),
        void (*on_disc)(uint8_t, uint8_t, void*),
        void (*on_data)(uint8_t, const uint8_t*, uint16_t, uint8_t, void*),
        void *app_ctx,
        uint8_t mod128) {

    ax25_conn_slot_t *slot = NULL;
    ax25_callbacks_t  cbs;
    ax25_address_t    local_addr, peer_addr;
    uint8_t           i, err, link_id;

    for (i = 0U; i < AX25_BRIDGE_MAX_CONNS; i++) {
        if (!g_ctx.conns[i].active) {
            slot = &g_ctx.conns[i];
            break;
        }
    }
    if (!slot) {
        hal_log(HAL_LOG_ERROR, "No free connection slots");
        return -1;
    }

    memset(slot, 0, sizeof(*slot));
    slot->active        = 1U;
    slot->conn_id       = i;
    g_ctx.num_conns++;
    slot->port          = g_ctx.hal_port;
    slot->on_connect    = on_connect;
    slot->on_disconnect = on_disc;
    slot->on_data       = on_data;
    slot->app_ctx       = app_ctx;
    /* on_ui and ui_ctx default to NULL — set via ax25_bridge_set_ui_handler() */

    memset(&cbs, 0, sizeof(cbs));
    cbs.on_connect    = sm_on_connect;
    cbs.on_disconnect = sm_on_disconnect;
    cbs.on_data       = sm_on_data;
    cbs.on_ui_data    = sm_on_ui;
    cbs.transmit      = sm_transmit;

    err = 0U;
    if (ax25_connection_init(&slot->sm, &cbs, slot) != 0U) {
        hal_log(HAL_LOG_ERROR, "connection_init failed");
        slot->active = 0U;
        g_ctx.num_conns--;
        return -1;
    }

    slot->sm.want_mod128 = mod128;

    /* Build peer address */
    err = 0U;
    {
        ax25_address_t *pa = ax25_address_from_string(dest_call, &err);
        if (!pa || err) {
            hal_log(HAL_LOG_ERROR, "bad dest callsign: %s", dest_call);
            if (pa) { uint8_t fe=0U; ax25_address_free(pa, &fe); }
            slot->active = 0U;
            g_ctx.num_conns--;
            return -1;
        }
        peer_addr = *pa;
        { uint8_t fe=0U; ax25_address_free(pa, &fe); }
    }

    /* Build local address */
    {
        ax25_address_t *la = ax25_address_from_string(g_ctx.mycall, &err);
        if (!la || err) {
            hal_log(HAL_LOG_ERROR, "bad local callsign: %s", g_ctx.mycall);
            if (la) { uint8_t fe=0U; ax25_address_free(la, &fe); }
            slot->active = 0U;
            g_ctx.num_conns--;
            return -1;
        }
        local_addr = *la;
        { uint8_t fe=0U; ax25_address_free(la, &fe); }
    }

    if (ax25_mux_register_link(&g_ctx.mux, &slot->sm,
                                &local_addr, &peer_addr, &link_id) != 0U) {
        hal_log(HAL_LOG_ERROR, "mux_register failed");
        slot->active = 0U;
        g_ctx.num_conns--;
        return -1;
    }
    slot->link_id = link_id;

    ax25_mux_set_lm_seize_confirm(&g_ctx.mux, link_id,
                                   ax25_mux_transmit_adapter, slot);

    if (ax25_connect(&slot->sm, &peer_addr, &local_addr) != 0U) {
        hal_log(HAL_LOG_ERROR, "ax25_connect failed");
        ax25_mux_unregister_link(&g_ctx.mux, link_id);
        slot->active = 0U;
        g_ctx.num_conns--;
        return -1;
    }

    hal_log(HAL_LOG_INFO, "Connecting: %s -> %s (conn_id=%u mod128=%u)",
            g_ctx.mycall, dest_call, i, mod128);
    return (int)i;
}

/**
 * @brief Send data on an established connection.
 */
int ax25_bridge_send(uint8_t conn_id, const uint8_t *data, uint16_t len, uint8_t pid) {
    uint8_t rc;
    if (conn_id >= AX25_BRIDGE_MAX_CONNS || !g_ctx.conns[conn_id].active)
        return -1;
    rc = ax25_send_data(&g_ctx.conns[conn_id].sm, (uint8_t*)data, (size_t)len, pid);
    return (rc == 0U) ? 0 : -1;
}

/**
 * @brief Send a UI (unconnected information) frame.
 */
int ax25_bridge_send_ui(const char *dest_call, const uint8_t *data,
                         uint16_t len, uint8_t pid) {
    ax25_address_t *dest = NULL, *src = NULL;
    uint8_t err = 0U, rc;

    dest = ax25_address_from_string(dest_call, &err);
    if (!dest || err) goto fail;
    src = ax25_address_from_string(g_ctx.mycall, &err);
    if (!src || err) goto fail;

    rc = ax25_send_ui(dest, src, (uint8_t*)data, (size_t)len, pid, sm_transmit_ui);

    if (dest) { uint8_t fe=0U; ax25_address_free(dest, &fe); }
    if (src)  { uint8_t fe=0U; ax25_address_free(src,  &fe); }
    return (rc == 0U) ? 0 : -1;

fail:
    if (dest) { uint8_t fe=0U; ax25_address_free(dest, &fe); }
    if (src)  { uint8_t fe=0U; ax25_address_free(src,  &fe); }
    return -1;
}

/**
 * @brief Disconnect from a remote station.
 */
void ax25_bridge_disconnect(uint8_t conn_id) {
    if (conn_id >= AX25_BRIDGE_MAX_CONNS || !g_ctx.conns[conn_id].active)
        return;
    ax25_disconnect(&g_ctx.conns[conn_id].sm);
    hal_log(HAL_LOG_INFO, "Disconnect requested: conn_id=%u", conn_id);
}

/**
 * @brief Main service loop — call as often as possible (at least every 10 ms).
 */
void ax25_bridge_tick(void) {
    uint8_t  byte, i;
    uint32_t tick_10ms;

#if defined(HAL_HAS_PORT_POLL)
    hal_port_poll(g_ctx.hal_port);
#endif

    tick_10ms                = hal_tick_ms() / 10U;
    g_ctx.kiss_current_tick  = tick_10ms;

    /* Drain RX ring into KISS state machine */
    while (hal_serial_get(g_ctx.hal_port, &byte) == HAL_OK) {
        ax25_kiss_receive_byte(&g_ctx.kiss, byte);
    }

    /* Tick the mux */
    ax25_mux_tick(&g_ctx.mux, tick_10ms);

    /* Tick each active connection */
    for (i = 0U; i < AX25_BRIDGE_MAX_CONNS; i++) {
        if (g_ctx.conns[i].active) {
            ax25_tick(&g_ctx.conns[i].sm, tick_10ms);
        }
    }
}

/**
 * @brief Open kernel socket for monitoring all AX.25 layer-2 traffic.
 *
 * FIX-B1: kernel_sock_is_packet is now set based on which socket type was
 * actually opened, not on whether ifname was provided.  The original code
 * set the flag whenever ifname was non-NULL, even if AF_PACKET creation
 * failed and the code fell through to the AF_AX25 fallback — causing
 * ax25_bridge_poll_kernel() to call recvfrom() with a sockaddr_ll on an
 * AF_AX25 socket (wrong type, wrong struct layout, undefined behaviour).
 */
int ax25_bridge_open_kernel_monitor(const char *ifname) {
    int     fd             = -1;
    uint8_t got_packet_socket = 0U;   /* tracks whether AF_PACKET succeeded */

    if (ifname && *ifname) {
        fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_AX25));
        if (fd >= 0) {
            struct sockaddr_ll sll;
            struct ifreq ifr;
            memset(&sll, 0, sizeof(sll));
            memset(&ifr, 0, sizeof(ifr));
            strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1U);
            if (ioctl(fd, SIOCGIFINDEX, &ifr) == 0) {
                sll.sll_family   = AF_PACKET;
                sll.sll_protocol = htons(ETH_P_AX25);
                sll.sll_ifindex  = ifr.ifr_ifindex;
                if (bind(fd, (struct sockaddr*)&sll, sizeof(sll)) == 0) {
                    got_packet_socket = 1U; /* AF_PACKET actually opened */
                } else {
                    hal_log(HAL_LOG_WARN, "AF_PACKET bind(%s): %s",
                            ifname, strerror(errno));
                    close(fd);
                    fd = -1;
                }
            } else {
                hal_log(HAL_LOG_WARN, "SIOCGIFINDEX %s: %s",
                        ifname, strerror(errno));
                close(fd);
                fd = -1;
            }
        } else {
            hal_log(HAL_LOG_WARN,
                    "AF_PACKET unavailable (need CAP_NET_RAW): %s — "
                    "falling back to AF_AX25/SOCK_DGRAM (UI frames only)",
                    strerror(errno));
        }
    }

    /* Fallback: AF_AX25/SOCK_DGRAM — receives only UI frames to mycall. */
    if (fd < 0) {
        struct sockaddr_ax25 sa;
        fd = socket(AF_AX25, SOCK_DGRAM, 0);
        if (fd < 0) {
            hal_log(HAL_LOG_WARN, "AF_AX25 socket: %s", strerror(errno));
            return -1;
        }
        memset(&sa, 0, sizeof(sa));
        sa.sax25_family = AF_AX25;
        callsign_to_kernel(g_ctx.mycall, &sa.sax25_call);
        if (bind(fd, (struct sockaddr*)&sa, sizeof(sa)) < 0) {
            hal_log(HAL_LOG_WARN, "AF_AX25 bind(%s): %s",
                    ifname ? ifname : "", strerror(errno));
            close(fd);
            return -1;
        }
        if (ifname && *ifname) {
            struct ifreq ifr;
            memset(&ifr, 0, sizeof(ifr));
            strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1U);
            setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE,
                       ifr.ifr_name, (socklen_t)strlen(ifr.ifr_name));
        }
        hal_log(HAL_LOG_INFO, "Kernel monitor: AF_AX25/SOCK_DGRAM "
                "(UI frames only, iface=%s)", ifname ? ifname : "any");
    } else {
        hal_log(HAL_LOG_INFO, "Kernel monitor: AF_PACKET/SOCK_RAW "
                "(all frames, iface=%s)", ifname ? ifname : "any");
    }

    /* FIX-B1: flag reflects the actual socket type, not the ifname argument */
    g_ctx.kernel_sock_is_packet = got_packet_socket;
    fcntl(fd, F_SETFL, O_NONBLOCK);
    g_ctx.kernel_sock       = fd;
    g_ctx.kernel_sock_open  = 1U;
    return 0;
}

/**
 * @brief Service the kernel monitor socket.
 */
void ax25_bridge_poll_kernel(void) {
    uint8_t  buf[340];
    int32_t  n;

    if (!g_ctx.kernel_sock_open || g_ctx.kernel_sock < 0)
        return;

    if (g_ctx.kernel_sock_is_packet) {
        /* AF_PACKET: use sockaddr_ll; frame bytes start at offset 0 (raw L2) */
        struct sockaddr_ll sll;
        socklen_t flen = (socklen_t)sizeof(sll);
        n = (int32_t)recvfrom(g_ctx.kernel_sock, buf, sizeof(buf),
                               MSG_DONTWAIT, (struct sockaddr*)&sll, &flen);
        if (n <= 0)
            return;
        hal_log(HAL_LOG_INFO, "Kernel RX (raw): %u bytes on ifindex=%d",
                (uint16_t)n, sll.sll_ifindex);
    } else {
        /* AF_AX25/SOCK_DGRAM: payload is L3 data only (kernel strips header) */
        struct sockaddr_ax25 from;
        socklen_t flen = (socklen_t)sizeof(from);
        char src_call[10];
        n = (int32_t)recvfrom(g_ctx.kernel_sock, buf, sizeof(buf),
                               MSG_DONTWAIT, (struct sockaddr*)&from, &flen);
        if (n <= 0)
            return;
        kernel_to_callsign(&from.sax25_call, src_call);
        hal_log(HAL_LOG_INFO, "Kernel RX (UI): %u bytes from %s",
                (uint16_t)n, src_call);
    }

    {
        uint8_t       err = 0U;
        ax25_frame_t *f   = ax25_frame_decode(buf, (size_t)n,
                                               MODULO128_AUTO, &err);
        if (f) {
            ax25_mux_receive_frame(&g_ctx.mux, f, g_ctx.kiss_current_tick);
            uint8_t fe = 0U;
            ax25_frame_free(f, &fe);
        } else {
            hal_log(HAL_LOG_WARN, "Kernel RX: ax25_frame_decode failed err=%u", err);
        }
    }
}

/**
 * @brief Shut down the bridge cleanly.
 */
void ax25_bridge_deinit(void) {
    uint8_t i, err;
    for (i = 0U; i < AX25_BRIDGE_MAX_CONNS; i++) {
        if (g_ctx.conns[i].active) {
            ax25_disconnect(&g_ctx.conns[i].sm);
            ax25_mux_unregister_link(&g_ctx.mux, g_ctx.conns[i].link_id);
            g_ctx.conns[i].active = 0U;
            if (g_ctx.num_conns > 0U) g_ctx.num_conns--;
        }
    }
    if (g_ctx.kernel_sock >= 0) {
        close(g_ctx.kernel_sock);
        g_ctx.kernel_sock = -1;
    }
    err = 0U;
    ax25_xid_deinit_defaults(&err);
    g_ctx.num_conns = 0U;

    hal_log(HAL_LOG_INFO, "Bridge deinit complete");
}

/* =========================================================================
 * TEST / INTEGRATION HELPER FUNCTIONS
 *
 * These functions expose internal bridge mechanisms needed by the test suite.
 * They are safe to call from production code but their primary purpose is
 * enabling white-box testing without HAL-level mocking.
 * ========================================================================= */

/**
 * @brief Register a UI-frame receive callback on an existing connection.
 *
 * FIX-B2 companion: because ax25_bridge_connect() already has a fixed
 * signature that predates the UI callback, the handler is registered
 * separately.  Must be called after ax25_bridge_connect().
 *
 * @param conn_id  Connection index returned by ax25_bridge_connect()
 * @param fn       Callback invoked for every received UI frame on this link
 * @param ctx      Opaque context pointer passed back to fn
 */
void ax25_bridge_set_ui_handler(uint8_t conn_id,
    void (*fn)(uint8_t, const uint8_t*, uint16_t, uint8_t, void*),
    void *ctx)
{
    if (conn_id >= AX25_BRIDGE_MAX_CONNS) return;
    g_ctx.conns[conn_id].on_ui  = fn;
    g_ctx.conns[conn_id].ui_ctx = ctx;
}

/**
 * @brief Override the KISS serial-write callback for testing.
 *
 * Replaces the HAL-backed kiss_serial_write with a custom function, letting
 * tests capture every byte the bridge would write to a TNC.  Must be called
 * AFTER ax25_bridge_init() (which installs the default HAL callback).
 *
 * @param cb  New write callback.  Pass NULL to restore the HAL default.
 */
void ax25_bridge_set_serial_write_cb(void (*cb)(uint8_t*, size_t, void*)) {
    if (cb)
        g_ctx.kiss.serial_write = cb;
    else
        g_ctx.kiss.serial_write = kiss_serial_write;
}

/**
 * @brief Synchronously inject raw KISS bytes into the bridge RX path.
 *
 * Equivalent to the bridge receiving these bytes from the TNC via the HAL,
 * but bypasses hal_serial_get() so tests do not need a HAL-level mock.
 * The KISS decoder, frame decoder, mux, and all state-machine callbacks fire
 * exactly as they would during normal operation.
 *
 * @param data  KISS-framed bytes to inject (including FEND delimiters)
 * @param len   Number of bytes
 */
void ax25_bridge_inject_rx_bytes(const uint8_t *data, uint16_t len) {
    if (!data || len == 0U) return;
    ax25_kiss_receive_bytes(&g_ctx.kiss, (uint8_t*)data, (size_t)len);
}

/**
 * @brief Encode an ASCII callsign into 7-byte AX.25 kernel format.
 *
 * Thin public wrapper around the internal callsign_to_kernel() helper,
 * allowing tests to verify the encoding independently of the bridge.
 *
 * @param str   NUL-terminated "CALL-SSID" string (e.g. "W1AW-3")
 * @param out   7-byte output buffer — filled with left-shifted bytes
 */
void ax25_bridge_encode_callsign(const char *str, uint8_t out[7]) {
    ax25_address tmp;
    if (!str || !out) return;
    callsign_to_kernel(str, &tmp);
    memcpy(out, tmp.ax25_call, 7U);
}

/**
 * @brief Decode a 7-byte AX.25 kernel address back to ASCII "CALL-SSID".
 *
 * Thin public wrapper around the internal kernel_to_callsign() helper.
 *
 * @param in       7-byte AX.25-encoded address (as produced by encode)
 * @param out      Output buffer for the ASCII result
 * @param out_len  Size of @p out (must be >= 10 to hold any valid callsign)
 */
void ax25_bridge_decode_callsign(const uint8_t in[7], char *out, uint8_t out_len) {
    ax25_address tmp;
    if (!in || !out || out_len == 0U) return;
    memcpy(tmp.ax25_call, in, 7U);
    kernel_to_callsign(&tmp, out);
    out[out_len - 1U] = '\0';   /* guarantee NUL termination */
}

/**
 * @brief Return the maximum number of simultaneous connections the bridge
 *        supports (AX25_BRIDGE_MAX_CONNS).
 */
uint8_t ax25_bridge_max_connections(void) {
    return (uint8_t)AX25_BRIDGE_MAX_CONNS;
}

/**
 * @brief Advance all active state machines by a caller-supplied timestamp.
 *
 * Equivalent to ax25_bridge_tick() but uses an explicit millisecond value
 * instead of hal_tick_ms(), allowing deterministic timer-driven testing
 * without real-time delays or HAL-level mocking.
 *
 * @param tick_ms  Simulated wall-clock time in milliseconds
 */
void ax25_bridge_tick_manual(uint32_t tick_ms) {
    uint32_t tick_10ms = tick_ms / 10U;
    uint8_t  i;

    g_ctx.kiss_current_tick = tick_10ms;
    ax25_mux_tick(&g_ctx.mux, tick_10ms);
    for (i = 0U; i < AX25_BRIDGE_MAX_CONNS; i++) {
        if (g_ctx.conns[i].active)
            ax25_tick(&g_ctx.conns[i].sm, tick_10ms);
    }
}
