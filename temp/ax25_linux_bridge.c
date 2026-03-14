/**
 * @file ax25_linux_bridge.c
 * @brief Bridge between libax25v22 and the Linux kernel AF_AX25 socket API.
 *
 * FIXES vs previous version
 * ──────────────────────────
 * FIX-B1        kernel_sock_is_packet set from which socket type actually opened.
 * FIX-B2        sm_on_ui forwards received UI frames to the per-connection on_ui
 *               callback instead of silently dropping them.
 * FIX-B-SM1     NULL-guard added to every SM callback before dereferencing
 *               user_data; conn_id is also range-checked.
 * FIX-B-SM2     sm_transmit_ui captures and logs the KISS return code.
 * FIX-B-UI      Dead "if (enc) free(enc)" after "enc = NULL" removed from
 *               ax25_bridge_send_ui; single canonical goto-fail used instead.
 * FIX-B-PORT    port argument range-checked against AX25_BRIDGE_MAX_PORTS
 *               before being stored.
 * FIX-B-DIV     Division-by-10 in tick helpers replaced by a branch-free
 *               32-bit reciprocal multiply; no hardware divider required.
 *               (Correct for all uint32_t values ≤ 4 294 967 280.)
 * FIX-B-DEINIT  ax25_mux_deinit() called on teardown (guarded by feature
 *               macro so it compiles even when the symbol is absent).
 * FIX-B-FREE    Frame-ownership contract documented; the existing free-after-
 *               mux-receive is correct because the mux deep-copies the frame.
 * FIX-B-ETH     AF_PACKET socket opened with ETH_P_ALL; frames are filtered
 *               by sll_protocol == ETH_P_AX25 in poll_kernel() to avoid the
 *               htons(0x0008)==0x0800==ETH_P_IP bug on little-endian hosts.
 * FIX-B-NUMCONNS g_ctx.num_conns++ moved to the single success path in
 *               ax25_bridge_connect() so it is never incremented on failures.
 *
 * Portability constraints (ARM Cortex-M, no FPU):
 *   - No 64-bit arithmetic.
 *   - No float or double.
 *   - All integer types: uint8_t / uint16_t / uint32_t.
 */

#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <malloc.h>   /* malloc_usable_size — distinguishes heap from static */

/* ── Linux socket headers (host-only; not compiled on bare-metal) ── */
#if defined(__linux__) || defined(HOST_TEST)
#  include <sys/socket.h>
#  include <sys/ioctl.h>
#  include <net/if.h>
#  include <netinet/in.h>
#  include <netpacket/packet.h>
#  include <linux/if_ether.h>
#  if defined(__has_include)
#    if __has_include(<netax25/ax25.h>)
#      include <netax25/ax25.h>
#      define AX25_BRIDGE_HAVE_KERNEL_HEADERS 1
#    endif
#  endif
#endif /* __linux__ || HOST_TEST */

#include "ax25.h"
#include "ax25_state_machine.h"
#include "ax25_mux.h"
#include "kiss.h"
#include "hal.h"

/* =========================================================================
 * CONSTANTS
 * ========================================================================= */

#define AX25_BRIDGE_MAX_CONNS  AX25_MUX_MAX_LINKS
#define AX25_BRIDGE_MAX_PORTS  2U

/* =========================================================================
 * KERNEL TYPE STUBS (bare-metal / non-Linux builds)
 * ========================================================================= */

#ifndef AX25_BRIDGE_HAVE_KERNEL_HEADERS
typedef struct {
    char ax25_call[7];
} ax25_address;

struct sockaddr_ax25 {
    unsigned short sax25_family;
    ax25_address   sax25_call;
    int            sax25_ndigis;
};
#endif /* AX25_BRIDGE_HAVE_KERNEL_HEADERS */

/* =========================================================================
 * TYPES
 * ========================================================================= */

/** Per-connection state. */
typedef struct {
    ax25_connection_t sm;
    uint8_t  active;
    uint8_t  link_id;
    uint8_t  port;
    uint8_t  conn_id;
    /* Connected-mode callbacks */
    void (*on_connect   )(uint8_t conn_id, int   initiated_locally, void *ctx);
    void (*on_disconnect)(uint8_t conn_id, uint8_t reason,          void *ctx);
    void (*on_data      )(uint8_t conn_id, const uint8_t *data,
                          uint16_t len, uint8_t pid, void *ctx);
    void *app_ctx;
    /* FIX-B2: UI-frame receive callback (was missing) */
    void (*on_ui        )(uint8_t conn_id, const uint8_t *data,
                          uint16_t len, uint8_t pid, void *ctx);
    void *ui_ctx;
} ax25_conn_slot_t;

/** Bridge singleton context. */
typedef struct {
    ax25_kiss_ctx_t  kiss;
    ax25_mux_t       mux;
    ax25_conn_slot_t conns[AX25_BRIDGE_MAX_CONNS];
    uint8_t          num_conns;
    uint8_t          hal_port;
    char             mycall[10];
    int              kernel_sock;
    uint8_t          kernel_sock_open;
    uint8_t          kernel_sock_is_packet; /* FIX-B1: 1=AF_PACKET, 0=AF_AX25 */
    uint32_t         kiss_current_tick;     /* in 10 ms units */
} ax25_linux_ctx_t;

static ax25_linux_ctx_t g_ctx;

/* =========================================================================
 * FIX-B-DIV — 32-bit divide-by-10 without hardware divider
 *
 * Uses the shift-and-add reciprocal from Hacker's Delight §10-8.
 * Exact for all n in [0, 4 294 967 280].  The one edge case above that
 * limit produces q+1 instead of q, which is harmless for tick values that
 * would long have wrapped through zero before reaching it.
 * ========================================================================= */
static uint32_t div10_u32(uint32_t n)
{
    uint32_t q = (n >> 1U) + (n >> 2U);
    uint32_t r;
    q = q + (q >> 4U);
    q = q + (q >> 8U);
    q = q + (q >> 16U);
    q = q >> 3U;
    r = n - ((q << 3U) + (q << 1U));   /* n - q*10; wrap is intentional */
    return q + ((r + 6U) >> 4U);
}

/* =========================================================================
 * HELPERS — callsign encoding / decoding
 * ========================================================================= */

static uint8_t callsign_parse_ssid(const char *str)
{
    const char *d;
    uint8_t     ssid = 0U;
    if (!str) return 0U;
    d = strchr(str, '-');
    if (d) {
        const char *p = d + 1U;
        while (*p >= '0' && *p <= '9') {
            ssid = (uint8_t)(ssid * 10U + (uint8_t)(*p - '0'));
            p++;
        }
        if (ssid > 15U) ssid = 0U;
    }
    return ssid;
}

/** Convert ASCII "CALL-SSID" → 7-byte AX.25 kernel format. */
static void callsign_to_kernel(const char *str, ax25_address *addr)
{
    const char *dash;
    char        buf[6];
    uint8_t     ssid, i;

    memset(buf, ' ', 6U);
    ssid = callsign_parse_ssid(str);
    dash = strchr(str, '-');

    for (i = 0U; i < 6U; i++) {
        char c = (dash && str == dash) ? ' ' : (str && *str ? *str++ : ' ');
        buf[i] = c;
        if (dash && str == dash) break;   /* reached dash; rest is spaces */
    }
    /* second pass: copy only up-to-dash or up-to-end */
    {
        const char *p = str - i;  /* rewind; rebuild cleanly */
        memset(buf, ' ', 6U);
        for (i = 0U; i < 6U && *p && p != dash; i++, p++)
            buf[i] = *p;
    }

    for (i = 0U; i < 6U; i++)
        addr->ax25_call[i] = (uint8_t)((uint8_t)buf[i] << 1U);
    addr->ax25_call[6] = (uint8_t)(0x60U | (uint8_t)(ssid << 1U));
}

/** Convert 7-byte AX.25 kernel format → ASCII "CALL-SSID". */
static void kernel_to_callsign(const ax25_address *addr, char *buf, uint8_t blen)
{
    char   *p = buf;
    uint8_t i, ssid;

    for (i = 0U; i < 6U && (uint8_t)(p - buf) < blen - 1U; i++) {
        char c = (char)((uint8_t)(addr->ax25_call[i]) >> 1U);
        if (c != ' ') *p++ = c;
    }
    ssid = (uint8_t)((addr->ax25_call[6] >> 1U) & 0x0FU);
    if (ssid > 0U && (uint8_t)(p - buf) < blen - 4U) {
        *p++ = '-';
        if (ssid >= 10U) { *p++ = '1'; *p++ = (char)('0' + ssid - 10U); }
        else              { *p++ = (char)('0' + ssid); }
    }
    *p = '\0';
    buf[blen - 1U] = '\0';
}

/* =========================================================================
 * KISS CALLBACKS
 * ========================================================================= */

/**
 * Called by the KISS decoder each time a complete AX.25 frame arrives.
 *
 * FIX-B-FREE: ax25_mux_receive_frame() deep-copies the frame internally,
 * so we remain the owner of `f` and free it after the call.  If the mux
 * implementation is ever changed to take ownership (no copy), remove the
 * ax25_frame_free() below and update this comment.
 */
static void on_kiss_frame(ax25_kiss_ctx_t *ctx, uint8_t port,
                          uint8_t *frame, size_t len, void *user_data)
{
    ax25_frame_t *f;
    uint8_t       err = 0U;
    (void)ctx; (void)port; (void)user_data;

    if (len < AX25_MIN_FRAME_SIZE_NO_FCS) {
        hal_log(HAL_LOG_WARN, "KISS RX: frame too short (%u bytes)",
                (unsigned)len);
        return;
    }

    f = ax25_frame_decode(frame, len, MODULO128_AUTO, &err);
    if (!f || err) {
        hal_log(HAL_LOG_WARN, "frame_decode err=%u", err);
        if (f) { uint8_t fe = 0U; ax25_frame_free(f, &fe); }
        return;
    }

    ax25_mux_receive_frame(&g_ctx.mux, f, g_ctx.kiss_current_tick);
    { uint8_t fe = 0U; ax25_frame_free(f, &fe); }   /* FIX-B-FREE */
}

static void kiss_serial_write(uint8_t *data, size_t len, void *user_data)
{
    uint8_t port = (uint8_t)(uintptr_t)user_data;
    hal_serial_write(port, data, (uint16_t)len);
}

/* =========================================================================
 * STATE MACHINE CALLBACKS  (FIX-B-SM1: NULL-guard on every callback)
 * ========================================================================= */

static void sm_on_connect(void *user_data, _Bool initiated_locally)
{
    ax25_conn_slot_t *s;
    if (!user_data) return;
    s = (ax25_conn_slot_t *)user_data;
    if (s->conn_id >= AX25_BRIDGE_MAX_CONNS) return;
    hal_log(HAL_LOG_INFO, "CONN[%u] connected (local=%d)",
            s->conn_id, (int)initiated_locally);
    if (s->on_connect)
        s->on_connect(s->conn_id, (int)initiated_locally, s->app_ctx);
}

static void sm_on_disconnect(void *user_data, uint8_t reason)
{
    ax25_conn_slot_t *s;
    if (!user_data) return;
    s = (ax25_conn_slot_t *)user_data;
    if (s->conn_id >= AX25_BRIDGE_MAX_CONNS) return;
    hal_log(HAL_LOG_INFO, "CONN[%u] disconnected reason=%u",
            s->conn_id, reason);
    if (s->on_disconnect)
        s->on_disconnect(s->conn_id, reason, s->app_ctx);
}

static void sm_on_data(void *user_data, uint8_t *data, size_t len, uint8_t pid)
{
    ax25_conn_slot_t *s;
    if (!user_data || !data) return;
    s = (ax25_conn_slot_t *)user_data;
    if (s->conn_id >= AX25_BRIDGE_MAX_CONNS) return;
    hal_log(HAL_LOG_DEBUG, "CONN[%u] data %u B pid=0x%02X",
            s->conn_id, (unsigned)len, pid);
    if (s->on_data)
        s->on_data(s->conn_id, data, (uint16_t)len, pid, s->app_ctx);
}

/**
 * FIX-B2: Forward UI frame to per-connection on_ui callback.
 * FIX-B-SM1: guard user_data and conn_id before dereference.
 */
static void sm_on_ui(void *user_data, const ax25_address_t *src,
                     uint8_t *data, size_t len, uint8_t pid)
{
    ax25_conn_slot_t *s;
    if (!user_data) return;
    s = (ax25_conn_slot_t *)user_data;
    if (s->conn_id >= AX25_BRIDGE_MAX_CONNS) return;
    hal_log(HAL_LOG_DEBUG, "UI from %s len=%u pid=0x%02X",
            src ? src->callsign : "?",
            (unsigned)len, pid);
    /* FIX-B2: deliver to registered handler */
    if (s->on_ui)
        s->on_ui(s->conn_id, data, (uint16_t)len, pid, s->ui_ctx);
}

/* FIX-B-SM1: guard user_data before dereference.
 * Same dual-path ownership contract as lb_transmit_a/b in the test harness:
 * ax25_transmit_frame frees before calling us (static path → do not free);
 * ax25_send_data_raw passes a heap buffer directly (heap path → must free).
 * malloc_usable_size() distinguishes the two cases portably on Linux/glibc. */
static void sm_transmit(void *user_data, uint8_t *frame, size_t len)
{
    ax25_conn_slot_t *s;
    uint8_t           rc;
    uint8_t           is_heap = (malloc_usable_size(frame) > 0U) ? 1U : 0U;
    if (!user_data || !frame || len == 0U) {
        if (is_heap) free(frame);
        return;
    }
    s = (ax25_conn_slot_t *)user_data;
    if (s->conn_id >= AX25_BRIDGE_MAX_CONNS) {
        if (is_heap) free(frame);
        return;
    }
    rc = ax25_kiss_send_frame(&g_ctx.kiss, s->port, frame, len);
    if (rc != KISS_OK)
        hal_log(HAL_LOG_WARN, "CONN[%u] kiss_send err=%u", s->conn_id, rc);
    if (is_heap) free(frame);
}

/** FIX-B-SM2: return code captured and logged.
 * NOTE: caller (ax25_bridge_send_ui) owns enc; do NOT free here. */
static void sm_transmit_ui(uint8_t *frame, size_t len)
{
    uint8_t rc;
    if (!frame || len == 0U) {
        return;
    }
    rc = ax25_kiss_send_frame(&g_ctx.kiss, g_ctx.hal_port, frame, len);
    if (rc != KISS_OK)
        hal_log(HAL_LOG_WARN, "sm_transmit_ui: kiss err=%u", rc);
}

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialise the bridge.
 * @param mycall  Local station callsign (e.g. "N0CALL-1").
 * @param port    KISS port index; must be < AX25_BRIDGE_MAX_PORTS.
 * @return 0 on success, -1 on error.
 *
 * FIX-B-PORT: port is range-checked before storage.
 */
int ax25_bridge_init(const char *mycall, uint8_t port)
{
    uint8_t err, i;

    if (!mycall) return -1;

    /* FIX-B-PORT */
    if (port >= AX25_BRIDGE_MAX_PORTS) {
        hal_log(HAL_LOG_ERROR,
                "bridge_init: port %u >= AX25_BRIDGE_MAX_PORTS (%u)",
                port, AX25_BRIDGE_MAX_PORTS);
        return -1;
    }

    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.hal_port    = port;
    g_ctx.kernel_sock = -1;

    strncpy(g_ctx.mycall, mycall, sizeof(g_ctx.mycall) - 1U);
    g_ctx.mycall[sizeof(g_ctx.mycall) - 1U] = '\0';

    err = 0U;
    ax25_xid_init_defaults(&err);
    if (err) {
        hal_log(HAL_LOG_ERROR, "xid_init_defaults failed");
        return -1;
    }

    if (ax25_mux_init(&g_ctx.mux) != 0U) {
        hal_log(HAL_LOG_ERROR, "mux_init failed");
        return -1;
    }

    if (ax25_kiss_init(&g_ctx.kiss) != KISS_OK) {
        hal_log(HAL_LOG_ERROR, "kiss_init failed");
        return -1;
    }
    g_ctx.kiss.on_frame     = on_kiss_frame;
    g_ctx.kiss.serial_write = kiss_serial_write;
    g_ctx.kiss.user_data    = (void *)(uintptr_t)port;
    ax25_kiss_enter(&g_ctx.kiss);

    for (i = 0U; i < AX25_BRIDGE_MAX_CONNS; i++) {
        g_ctx.conns[i].active  = 0U;
        g_ctx.conns[i].conn_id = i;
    }

    hal_log(HAL_LOG_INFO, "Bridge init: call=%s port=%u", mycall, port);
    return 0;
}

/**
 * @brief Initiate an outgoing AX.25 connected-mode session.
 * @return conn_id (≥ 0) on success, -1 on error.
 *
 * FIX-B-NUMCONNS: g_ctx.num_conns++ only at the single success return.
 */
int ax25_bridge_connect(const char   *dest_call,
                        void (*on_connect   )(uint8_t, int,     void *),
                        void (*on_disc      )(uint8_t, uint8_t, void *),
                        void (*on_data      )(uint8_t, const uint8_t *,
                                              uint16_t, uint8_t, void *),
                        void         *app_ctx,
                        uint8_t       mod128)
{
    ax25_conn_slot_t *slot        = NULL;
    ax25_callbacks_t  cbs;
    ax25_address_t    local_addr, peer_addr;
    uint8_t           i, err, link_id = 0U;
    uint8_t           mux_registered  = 0U;

    /* Find free slot */
    for (i = 0U; i < AX25_BRIDGE_MAX_CONNS; i++) {
        if (!g_ctx.conns[i].active) { slot = &g_ctx.conns[i]; break; }
    }
    if (!slot) {
        hal_log(HAL_LOG_ERROR, "No free connection slots");
        return -1;
    }

    memset(slot, 0, sizeof(*slot));
    slot->active        = 1U;
    slot->conn_id       = i;
    slot->port          = g_ctx.hal_port;
    slot->on_connect    = on_connect;
    slot->on_disconnect = on_disc;
    slot->on_data       = on_data;
    slot->app_ctx       = app_ctx;

    memset(&cbs, 0, sizeof(cbs));
    cbs.on_connect    = sm_on_connect;
    cbs.on_disconnect = sm_on_disconnect;
    cbs.on_data       = sm_on_data;
    cbs.on_ui_data    = sm_on_ui;
    cbs.transmit      = sm_transmit;

    if (ax25_connection_init(&slot->sm, &cbs, slot) != 0U)
        goto fail;
    slot->sm.want_mod128 = mod128;

    /* Decode peer address */
    err = 0U;
    {
        ax25_address_t *pa = ax25_address_from_string(dest_call, &err);
        if (!pa || err) {
            if (pa) { uint8_t fe = 0U; ax25_address_free(pa, &fe); }
            goto fail;
        }
        peer_addr = *pa;
        { uint8_t fe = 0U; ax25_address_free(pa, &fe); }
    }

    /* Decode local address */
    err = 0U;
    {
        ax25_address_t *la = ax25_address_from_string(g_ctx.mycall, &err);
        if (!la || err) {
            if (la) { uint8_t fe = 0U; ax25_address_free(la, &fe); }
            goto fail;
        }
        local_addr = *la;
        { uint8_t fe = 0U; ax25_address_free(la, &fe); }
    }

    /* Register with mux */
    if (ax25_mux_register_link(&g_ctx.mux, &slot->sm,
                                &local_addr, &peer_addr, &link_id) != 0U)
        goto fail;
    mux_registered = 1U;
    slot->link_id  = link_id;

    ax25_mux_set_lm_seize_confirm(&g_ctx.mux, link_id,
                                   ax25_mux_transmit_adapter, slot);

    if (ax25_connect(&slot->sm, &peer_addr, &local_addr) != 0U)
        goto fail;

    /* FIX-B-NUMCONNS: increment only on full success */
    g_ctx.num_conns++;
    hal_log(HAL_LOG_INFO, "Connecting %s -> %s (conn_id=%u mod128=%u)",
            g_ctx.mycall, dest_call, i, mod128);
    return (int)i;

fail:
    if (mux_registered) ax25_mux_unregister_link(&g_ctx.mux, link_id);
    slot->active = 0U;
    return -1;
}

/**
 * @brief Transmit data on an established connection.
 */
int ax25_bridge_send(uint8_t conn_id, const uint8_t *data,
                     uint16_t len, uint8_t pid)
{
    uint8_t rc;
    if (conn_id >= AX25_BRIDGE_MAX_CONNS || !g_ctx.conns[conn_id].active)
        return -1;
    if (!data || len == 0U)
        return -1;
    rc = ax25_send_data(&g_ctx.conns[conn_id].sm,
                        (uint8_t *)data, (size_t)len, pid);
    return (rc == 0U) ? 0 : -1;
}

/**
 * @brief Send a connectionless UI frame.
 *
 * FIX-B-UI: the previous code had:
 *   enc = NULL;
 *   if (enc) free(enc);    ← dead free: enc is always NULL here
 * and the `fail:` label was placed *after* this block, meaning failures
 * earlier in the function jumped past all cleanup.
 *
 * Fixed by: removing the dead block; using a single `goto fail` with
 * one cleanup sequence that runs on every exit path (success or failure).
 */
int ax25_bridge_send_ui(const char *dest_call, const uint8_t *data,
                        uint16_t len, uint8_t pid)
{
    ax25_address_t                        *dest   = NULL;
    ax25_address_t                        *src    = NULL;
    ax25_unnumbered_information_frame_t    uif;
    ax25_frame_header_t                    hdr;
    uint8_t                               *enc    = NULL;
    size_t                                 enc_sz = 0U;
    uint8_t                                err    = 0U;
    int                                    result = -1;

    if (!dest_call || !data) goto fail;

    dest = ax25_address_from_string(dest_call, &err);
    if (!dest || err) goto fail;

    err = 0U;
    src  = ax25_address_from_string(g_ctx.mycall, &err);
    if (!src || err) goto fail;

    memset(&hdr, 0, sizeof(hdr));
    hdr.destination = *dest;
    hdr.source      = *src;
    hdr.cr          = 1U;

    memset(&uif, 0, sizeof(uif));
    uif.base.base.type   = AX25_FRAME_UNNUMBERED_INFORMATION;
    uif.base.base.header = hdr;
    uif.base.pf          = 0U;
    uif.base.modifier    = AX25_U_UI;
    uif.pid              = pid;
    uif.payload          = (uint8_t *)data;
    uif.payload_len      = len;

    err = 0U;
    enc = ax25_frame_encode((ax25_frame_t *)&uif, &enc_sz, &err);
    if (!enc || err || enc_sz == 0U) goto fail;

    sm_transmit_ui(enc, enc_sz);
    result = 0;

fail:
    /* FIX-B-UI: single cleanup — always reached, enc may legitimately be NULL */
    if (enc)  { free(enc); }
    if (dest) { uint8_t fe = 0U; ax25_address_free(dest, &fe); }
    if (src)  { uint8_t fe = 0U; ax25_address_free(src,  &fe); }
    return result;
}

/**
 * @brief Request disconnection on a connected-mode link.
 */
void ax25_bridge_disconnect(uint8_t conn_id)
{
    if (conn_id >= AX25_BRIDGE_MAX_CONNS || !g_ctx.conns[conn_id].active)
        return;
    ax25_disconnect(&g_ctx.conns[conn_id].sm);
    hal_log(HAL_LOG_INFO, "Disconnect requested: conn_id=%u", conn_id);
}

/**
 * @brief Periodic service routine — call from a tight poll loop.
 *
 * FIX-B-DIV: tick converted to 10 ms units via div10_u32 (no hardware divider).
 */
void ax25_bridge_tick(void)
{
    uint8_t  byte, i;
    uint32_t tick_10ms;

#if defined(HAL_HAS_PORT_POLL)
    hal_port_poll(g_ctx.hal_port);
#endif

    /* FIX-B-DIV */
    tick_10ms               = div10_u32(hal_tick_ms());
    g_ctx.kiss_current_tick = tick_10ms;

    while (hal_serial_get(g_ctx.hal_port, &byte) == HAL_OK)
        ax25_kiss_receive_byte(&g_ctx.kiss, byte);

    ax25_mux_tick(&g_ctx.mux, tick_10ms);

    for (i = 0U; i < AX25_BRIDGE_MAX_CONNS; i++) {
        if (g_ctx.conns[i].active)
            ax25_tick(&g_ctx.conns[i].sm, tick_10ms);
    }
}

/**
 * @brief Test/simulation helper: advance all timers with an explicit timestamp.
 *
 * FIX-B-DIV: uses div10_u32 — no hardware divider.
 * @param tick_ms  Absolute time in milliseconds.
 */
void ax25_bridge_tick_manual(uint32_t tick_ms)
{
    /* FIX-B-DIV */
    uint32_t tick_10ms = div10_u32(tick_ms);
    uint8_t  i;

    g_ctx.kiss_current_tick = tick_10ms;
    ax25_mux_tick(&g_ctx.mux, tick_10ms);

    for (i = 0U; i < AX25_BRIDGE_MAX_CONNS; i++) {
        if (g_ctx.conns[i].active)
            ax25_tick(&g_ctx.conns[i].sm, tick_10ms);
    }
}

/**
 * @brief Open a raw socket to observe AF_AX25 traffic on the kernel stack.
 *
 * FIX-B1:   kernel_sock_is_packet reflects the actual socket that was opened,
 *            not a hard-coded assumption.
 * FIX-B-ETH: AF_PACKET socket uses ETH_P_ALL; poll_kernel() filters by
 *            sll_protocol == ETH_P_AX25 to avoid the htons(0x0008)==0x0800
 *            IPv4 collision on little-endian hosts.
 */
int ax25_bridge_open_kernel_monitor(const char *ifname)
{
#if defined(__linux__) || defined(HOST_TEST)
    int     fd                = -1;
    uint8_t got_packet_socket = 0U;

    if (ifname && *ifname) {
        /* FIX-B-ETH: ETH_P_ALL to avoid htons(ETH_P_AX25) == ETH_P_IP */
        fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
        if (fd >= 0) {
            struct sockaddr_ll sll;
            struct ifreq       ifr;
            memset(&sll, 0, sizeof(sll));
            memset(&ifr, 0, sizeof(ifr));
            strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1U);
            if (ioctl(fd, SIOCGIFINDEX, &ifr) == 0) {
                sll.sll_family   = AF_PACKET;
                sll.sll_protocol = htons(ETH_P_ALL);
                sll.sll_ifindex  = ifr.ifr_ifindex;
                if (bind(fd, (struct sockaddr *)&sll, sizeof(sll)) == 0) {
                    got_packet_socket = 1U;
                    hal_log(HAL_LOG_INFO,
                            "Kernel monitor: AF_PACKET/ETH_P_ALL iface=%s",
                            ifname);
                } else {
                    hal_log(HAL_LOG_WARN, "AF_PACKET bind(%s): %s",
                            ifname, strerror(errno));
                    close(fd); fd = -1;
                }
            } else {
                hal_log(HAL_LOG_WARN, "SIOCGIFINDEX %s: %s",
                        ifname, strerror(errno));
                close(fd); fd = -1;
            }
        } else {
            hal_log(HAL_LOG_WARN,
                    "AF_PACKET unavailable (need CAP_NET_RAW): %s — "
                    "falling back to AF_AX25/SOCK_DGRAM", strerror(errno));
        }
    }

    /* Fallback: AF_AX25 SOCK_DGRAM — receives UI frames addressed to mycall */
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
        if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
            hal_log(HAL_LOG_WARN, "AF_AX25 bind: %s", strerror(errno));
            close(fd); return -1;
        }
        if (ifname && *ifname) {
            struct ifreq ifr;
            memset(&ifr, 0, sizeof(ifr));
            strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1U);
            setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE,
                       ifr.ifr_name, (socklen_t)strlen(ifr.ifr_name));
        }
        hal_log(HAL_LOG_INFO,
                "Kernel monitor: AF_AX25/SOCK_DGRAM (UI only, iface=%s)",
                ifname ? ifname : "any");
    }

    /* FIX-B1: record which socket type we actually got */
    g_ctx.kernel_sock_is_packet = got_packet_socket;
    fcntl(fd, F_SETFL, O_NONBLOCK);
    g_ctx.kernel_sock      = fd;
    g_ctx.kernel_sock_open = 1U;
    return 0;

#else   /* bare-metal build */
    (void)ifname;
    hal_log(HAL_LOG_WARN, "kernel_monitor not available on bare-metal");
    return -1;
#endif
}

/**
 * @brief Non-blocking poll of the kernel monitor socket.
 *
 * FIX-B-ETH: when using AF_PACKET, frames with sll_protocol ≠ ETH_P_AX25
 *            are discarded (they were IPv4 in the original buggy code).
 */
void ax25_bridge_poll_kernel(void)
{
#if defined(__linux__) || defined(HOST_TEST)
    uint8_t buf[340];
    int32_t n;

    if (!g_ctx.kernel_sock_open || g_ctx.kernel_sock < 0) return;

    if (g_ctx.kernel_sock_is_packet) {
        struct sockaddr_ll sll;
        socklen_t          flen = (socklen_t)sizeof(sll);

        n = (int32_t)recvfrom(g_ctx.kernel_sock, buf, sizeof(buf),
                              MSG_DONTWAIT,
                              (struct sockaddr *)&sll, &flen);
        if (n <= 0) return;

        /*
         * FIX-B-ETH: ETH_P_AX25 == 0x0008 (host order).
         * The kernel returns sll_protocol in network byte order, so we
         * must compare against htons(0x0008U).
         * Without this filter we would process IPv4 frames (ETH_P_IP=0x0800)
         * as AX.25 because htons(ETH_P_AX25)==0x0800 on little-endian hosts.
         */
        if (sll.sll_protocol != htons((uint16_t)0x0008U)) return;

        hal_log(HAL_LOG_INFO, "Kernel RX raw: %u B on ifindex=%d",
                (uint16_t)n, sll.sll_ifindex);
    } else {
        struct sockaddr_ax25 from;
        socklen_t            flen = (socklen_t)sizeof(from);
        char                 src_call[10];

        n = (int32_t)recvfrom(g_ctx.kernel_sock, buf, sizeof(buf),
                              MSG_DONTWAIT,
                              (struct sockaddr *)&from, &flen);
        if (n <= 0) return;
        kernel_to_callsign(&from.sax25_call, src_call, (uint8_t)sizeof(src_call));
        hal_log(HAL_LOG_INFO, "Kernel RX UI: %u B from %s",
                (uint16_t)n, src_call);
    }

    {
        uint8_t       err = 0U;
        ax25_frame_t *f   = ax25_frame_decode(buf, (size_t)n,
                                              MODULO128_AUTO, &err);
        if (f) {
            ax25_mux_receive_frame(&g_ctx.mux, f, g_ctx.kiss_current_tick);
            { uint8_t fe = 0U; ax25_frame_free(f, &fe); }  /* FIX-B-FREE */
        } else {
            hal_log(HAL_LOG_WARN, "Kernel RX: decode failed err=%u", err);
        }
    }
#endif
}

/**
 * @brief Release all bridge resources.
 *
 * FIX-B-DEINIT: ax25_mux_deinit() called when available.
 * Removed in-loop num_conns decrement (was wrong); counter zeroed at end.
 */
void ax25_bridge_deinit(void)
{
    uint8_t i, err;

    for (i = 0U; i < AX25_BRIDGE_MAX_CONNS; i++) {
        if (g_ctx.conns[i].active) {
            ax25_disconnect(&g_ctx.conns[i].sm);
            ax25_mux_unregister_link(&g_ctx.mux, g_ctx.conns[i].link_id);
            g_ctx.conns[i].active = 0U;
        }
    }

    /* FIX-B-DEINIT */
#if defined(AX25_MUX_HAS_DEINIT)
    ax25_mux_deinit(&g_ctx.mux);
#endif

    if (g_ctx.kernel_sock >= 0) {
        close(g_ctx.kernel_sock);
        g_ctx.kernel_sock = -1;
    }

    err = 0U;
    ax25_xid_deinit_defaults(&err);

    g_ctx.num_conns        = 0U;
    g_ctx.kernel_sock_open = 0U;

    hal_log(HAL_LOG_INFO, "Bridge deinit complete");
}

/* =========================================================================
 * TEST / INTEGRATION HELPERS
 * ========================================================================= */

/**
 * @brief Register a UI-frame receive handler on an existing connection.
 * FIX-B2 companion — call after ax25_bridge_connect().
 */
void ax25_bridge_set_ui_handler(
    uint8_t conn_id,
    void (*fn)(uint8_t, const uint8_t *, uint16_t, uint8_t, void *),
    void *ctx)
{
    if (conn_id >= AX25_BRIDGE_MAX_CONNS) return;
    g_ctx.conns[conn_id].on_ui  = fn;
    g_ctx.conns[conn_id].ui_ctx = ctx;
}

/**
 * @brief Override the KISS serial-write callback (useful for test stubs).
 */
void ax25_bridge_set_serial_write_cb(
    void (*cb)(uint8_t *, size_t, void *))
{
    g_ctx.kiss.serial_write = cb ? cb : kiss_serial_write;
}

/**
 * @brief Inject raw KISS-framed bytes directly into the bridge RX path.
 * Allows tests to feed frames without a physical serial port.
 */
void ax25_bridge_inject_rx_bytes(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0U) return;
    ax25_kiss_receive_bytes(&g_ctx.kiss, (uint8_t *)data, (size_t)len);
}

/**
 * @brief Encode ASCII callsign to 7-byte AX.25 kernel address format.
 */
void ax25_bridge_encode_callsign(const char *str, uint8_t out[7])
{
    ax25_address tmp;
    if (!str || !out) return;
    callsign_to_kernel(str, &tmp);
    memcpy(out, tmp.ax25_call, 7U);
}

/**
 * @brief Decode a 7-byte AX.25 kernel address to ASCII "CALL-SSID".
 */
void ax25_bridge_decode_callsign(const uint8_t in[7], char *out, uint8_t blen)
{
    ax25_address tmp;
    if (!in || !out || blen == 0U) return;
    memcpy(tmp.ax25_call, in, 7U);
    kernel_to_callsign(&tmp, out, blen);
}

/** @return Maximum number of simultaneous connections supported. */
uint8_t ax25_bridge_max_connections(void)
{
    return (uint8_t)AX25_BRIDGE_MAX_CONNS;
}
