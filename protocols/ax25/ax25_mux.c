/**
 * @file ax25_mux.c
 * @brief AX.25 v2.2 Protocol Library - Link Multiplexer
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 */

#include <string.h>
#include "ax25_mux.h"

// static connection table per AX.25 v2.2 SDL Appendix C3
// Provides O(n) lookup of connections by (dest, src) address-pair key.
// Static allocation: no heap required, safe for all embedded targets.
static ax25_conn_t conn_table[AX25_MAX_CONNECTIONS];

static bool address_equals(const ax25_address_t *a, const ax25_address_t *b) {
    if (strncmp(a->callsign, b->callsign, 6) != 0)
        return false;

    return a->ssid == b->ssid;
}

static bool is_broadcast_address(const ax25_address_t *addr) {
    const char *c = addr->callsign;

    // Use direct character comparisons to avoid strncmp dependency on null
    // termination and to prevent c[6] out-of-bounds access for "BEACON"
    // (callsign field is exactly 6 chars, may not be null-terminated).

    // "CQ    " — 2 chars + 4 spaces
    if (c[0] == 'C' && c[1] == 'Q' && (c[2] == ' ' || c[2] == '\0'))
        return true;

    // "APRS  " — 4 chars + 2 spaces
    if (c[0] == 'A' && c[1] == 'P' && c[2] == 'R' && c[3] == 'S' && (c[4] == ' ' || c[4] == '\0'))
        return true;

    // "BEACON" — exactly 6 chars, no indexing beyond the field boundary
    if (c[0] == 'B' && c[1] == 'E' && c[2] == 'A' && c[3] == 'C' && c[4] == 'O' && c[5] == 'N')
        return true;

    return false;
}

static int8_t ax25_mux_select_next(const ax25_mux_t *mux) {
    uint8_t max_pri = 0;
    bool any = false;
    uint8_t i;
    /* Pass 1: find highest priority among all pending requests */
    for (i = 0; i < AX25_MUX_MAX_LINKS; i++) {
        if (mux->links[i].active && mux->links[i].seize_pending) {
            if (!any || mux->links[i].seize_priority > max_pri) {
                max_pri = mux->links[i].seize_priority;
                any = true;
            }
        }
    }
    if (!any)
        return -1;
    /* Pass 2: round-robin among links with max_pri, start after last_served */
    for (i = 0; i < AX25_MUX_MAX_LINKS; i++) {
        uint8_t idx = (uint8_t) ((mux->last_served + 1u + i) % AX25_MUX_MAX_LINKS);
        if (mux->links[idx].active && mux->links[idx].seize_pending && mux->links[idx].seize_priority == max_pri) {
            return (int8_t) idx;
        }
    }
    return -1;
}

uint8_t ax25_mux_lm_seize_request(ax25_mux_t *mux, uint8_t link_id, const uint8_t *frame, size_t len, uint8_t priority) {
    if (!mux || link_id >= AX25_MUX_MAX_LINKS || !frame || len == 0 || len > AX25_MUX_FRAME_BUF)
        return 1;
    ax25_mux_link_t *l = &mux->links[link_id];
    if (!l->active || l->seize_pending)
        return 2;
    memcpy(l->pending_frame, frame, len);
    l->pending_len = len;
    l->seize_pending = true;
    l->seize_priority = priority;
    return 0;
}

void ax25_mux_lm_release(ax25_mux_t *mux, uint8_t link_id) {
    if (!mux || link_id >= AX25_MUX_MAX_LINKS)
        return;
    ax25_mux_link_t *l = &mux->links[link_id];
    l->seize_pending = false;
    l->pending_len = 0;
    if (mux->seized_link == link_id) {
        mux->seized_link = AX25_MUX_NO_SEIZED;
        /* immediately serve next pending request (burst support) */
        int8_t next = ax25_mux_select_next(mux);
        if (next >= 0) {
            mux->seized_link = (uint8_t) next;
            ax25_mux_link_t *nl = &mux->links[next];
            if (nl->lm_seize_confirm && nl->pending_len > 0) {
                nl->lm_seize_confirm(nl->confirm_user_data, nl->pending_frame, nl->pending_len);
            }
        }
    }
}

void ax25_mux_tick(ax25_mux_t *mux, uint32_t current_tick_10ms) {
    if (!mux)
        return;
    (void) current_tick_10ms; /* reserved for future timed seize */
    if (mux->seized_link != AX25_MUX_NO_SEIZED)
        return; /* already seized, wait for release */
    int8_t next = ax25_mux_select_next(mux);
    if (next >= 0) {
        mux->seized_link = (uint8_t) next;
        ax25_mux_link_t *l = &mux->links[next];
        if (l->lm_seize_confirm && l->pending_len > 0) {
            l->lm_seize_confirm(l->confirm_user_data, l->pending_frame, l->pending_len);
        }
    }
}

void ax25_mux_set_lm_seize_confirm(ax25_mux_t *mux, uint8_t link_id, ax25_lm_seize_confirm_t confirm, void *user_data) {
    if (!mux || link_id >= AX25_MUX_MAX_LINKS)
        return;
    ax25_mux_link_t *l = &mux->links[link_id];
    l->lm_seize_confirm = confirm;
    l->confirm_user_data = user_data;
}

uint8_t ax25_mux_classify_priority(const uint8_t *frame, size_t len) {
    if (!frame || len < 15u)
        return AX25_MUX_PRI_UI;

    size_t pos = 0;
    while (pos + 7 <= len) {
        if (frame[pos + 6] & 0x01u) {
            pos += 7;
            break;
        }
        pos += 7;
    }
    if (pos >= len)
        return AX25_MUX_PRI_UI;

    uint8_t ctrl = frame[pos];

    // Check specific known U-frames FIRST to prevent misclassification:
    // 0xE1 (TEST rsp F=0) has bits 1-0 = 01 which would falsely match the
    // S-frame pattern below if not intercepted here first.
    if (ctrl == 0x03u || ctrl == 0x13u) {  // UI command / response
        return AX25_MUX_PRI_UI;
    }
    if (ctrl == 0xAFu || ctrl == 0xBFu) {  // XID command / response
        return AX25_MUX_PRI_UI;
    }
    if (ctrl == 0xE3u || ctrl == 0xE1u) {  // TEST command / response
        return AX25_MUX_PRI_UI;
    }

    if ((ctrl & 0x01u) == 0u) {  // I-frame: bit 0 cleared
        return AX25_MUX_PRI_DATA;
    }
    if ((ctrl & 0x03u) == 0x01u) {  // S-frame: bits 1-0 = 01
        return AX25_MUX_PRI_ACK;
    }

    // All other U-frames (SABM/SABME/DISC/DM/UA/FRMR) are urgent
    return AX25_MUX_PRI_URGENT;
}

void ax25_mux_transmit_adapter(void *user_data, uint8_t *frame, size_t len) {
    ax25_mux_adapter_ctx_t *ctx = (ax25_mux_adapter_ctx_t*) user_data;
    if (!ctx || !ctx->mux)
        return;
    uint8_t pri = ax25_mux_classify_priority(frame, len);
    ax25_mux_lm_seize_request(ctx->mux, ctx->link_id, frame, len, pri);
}

int8_t ax25_mux_get_next_to_serve(ax25_mux_t *mux) {
    if (!mux)
        return -1;
    int8_t idx = ax25_mux_select_next(mux);
    if (idx >= 0) {
        mux->links[idx].seize_pending = false;
        mux->last_served = (uint8_t) idx;
    }

    return idx;
}

uint8_t ax25_mux_init(ax25_mux_t *mux) {
    if (!mux)
        return 1;

    memset(mux, 0, sizeof(ax25_mux_t));
    mux->last_served = 0;
    mux->seized_link = AX25_MUX_NO_SEIZED;

    return 0;
}

uint8_t ax25_mux_register_link(ax25_mux_t *mux, ax25_connection_t *conn, const ax25_address_t *local_addr, const ax25_address_t *peer_addr,
        uint8_t *link_id_out) {
    if (!mux || !conn || !local_addr || !peer_addr || !link_id_out)
        return 1;
    for (int i = 0; i < AX25_MUX_MAX_LINKS; i++) {
        if (!mux->links[i].active) {
            mux->links[i].conn = conn;
            mux->links[i].local_addr = *local_addr;
            mux->links[i].peer_addr = *peer_addr;
            mux->links[i].active = true;
            mux->links[i].seize_pending = false;
            mux->links[i].seize_priority = 128;  // default medium priority
            mux->links[i].pending_len = 0;
            mux->links[i].lm_seize_confirm = NULL;
            mux->links[i].confirm_user_data = NULL;
            *link_id_out = (uint8_t) i;
            mux->num_active++;
            return 0;
        }
    }
    return 2;  // no free slots
}

uint8_t ax25_mux_unregister_link(ax25_mux_t *mux, uint8_t link_id) {
    if (!mux || link_id >= AX25_MUX_MAX_LINKS)
        return 1;
    if (mux->links[link_id].active) {
        mux->links[link_id].active = false;
        mux->links[link_id].conn = NULL;
        if (mux->num_active > 0)
            mux->num_active--;
    }
    return 0;
}

void ax25_mux_receive_frame(ax25_mux_t *mux, ax25_frame_t *frame, uint32_t current_tick_10ms) {
    if (!mux || !frame)
        return;
    bool is_ui_broadcast = (frame->type == AX25_FRAME_UNNUMBERED_INFORMATION) && is_broadcast_address(&frame->header.destination);
    for (int i = 0; i < AX25_MUX_MAX_LINKS; i++) {
        if (!mux->links[i].active)
            continue;
        ax25_mux_link_t *l = &mux->links[i];
        if (is_ui_broadcast) {
            ax25_process_frame(l->conn, frame, current_tick_10ms);
            continue;
        }
        if (address_equals(&frame->header.destination, &l->local_addr) && address_equals(&frame->header.source, &l->peer_addr)) {
            ax25_process_frame(l->conn, frame, current_tick_10ms);
        }
    }
}

// connection table implementation per AX.25 v2.2 SDL Appendix C3

ax25_conn_t* ax25_find_conn(const char *dest, const char *src) {
    uint8_t i;
    // validate arguments before scanning the table
    if (!dest || !src)
        return NULL;
    for (i = 0; i < AX25_MAX_CONNECTIONS; i++) {
        if (conn_table[i].active && strncmp(conn_table[i].dest, dest, 7) == 0 && strncmp(conn_table[i].src, src, 7) == 0) {
            return &conn_table[i];
        }
    }
    return NULL;
}

ax25_conn_t* ax25_alloc_conn(const char *dest, const char *src, ax25_connection_t *conn) {
    uint8_t i;
    // validate arguments
    if (!dest || !src || !conn)
        return NULL;
    // reject duplicate (dest, src) pair — first registration wins
    if (ax25_find_conn(dest, src) != NULL)
        return NULL;
    // claim the first free slot
    for (i = 0; i < AX25_MAX_CONNECTIONS; i++) {
        if (!conn_table[i].active) {
            strncpy(conn_table[i].dest, dest, 7);
            strncpy(conn_table[i].src, src, 7);
            conn_table[i].conn = conn;
            conn_table[i].active = 1;
            return &conn_table[i];
        }
    }
    return NULL;  // table full
}

void ax25_free_conn(ax25_conn_t *entry) {
    if (!entry)
        return;
    entry->active = 0;
    entry->conn = NULL;
    // clear keys so stale data does not cause false positives on future lookups
    memset(entry->dest, 0, sizeof(entry->dest));
    memset(entry->src, 0, sizeof(entry->src));
}
