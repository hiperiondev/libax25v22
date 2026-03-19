/**
 * @file ax25_mux.c
 * @brief AX.25 v2.2 Protocol Library - Link Multiplexer
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#include <string.h>
#include "ax25_mux.h"

#include "hal.h"

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
    uint32_t key = hal_critical_enter();
    ax25_mux_link_t *l = &mux->links[link_id];
    if (!l->active || l->seize_pending) {
        hal_critical_exit(key);
        return 2;
    }
    memcpy(l->pending_frame, frame, len);
    l->pending_len = len;
    l->seize_pending = true;
    l->seize_priority = priority;
    hal_critical_exit(key);
    HAL_LOGD("ax25_mux: link %u seize request pri=%u len=%u", (unsigned)link_id, (unsigned)priority, (unsigned)len);

    return 0;
}

void ax25_mux_lm_release(ax25_mux_t *mux, uint8_t link_id) {
    if (!mux || link_id >= AX25_MUX_MAX_LINKS)
        return;

    uint32_t key = hal_critical_enter();
    ax25_mux_link_t *l = &mux->links[link_id];
    l->seize_pending = false;
    l->pending_len = 0;
    if (mux->seized_link == link_id) {
        mux->seized_link = AX25_MUX_NO_SEIZED;
        int8_t next = ax25_mux_select_next(mux);
        if (next >= 0) {
            mux->seized_link = (uint8_t) next;
            mux->links[next].seize_pending = false;
            mux->last_served = (uint8_t) next;
            ax25_mux_link_t *nl = &mux->links[next];
            hal_critical_exit(key);
            HAL_LOGD("ax25_mux: link %u released, serving link %d", (unsigned)link_id, (int)next);
            if (nl->lm_seize_confirm && nl->pending_len > 0) {
                nl->lm_seize_confirm(nl->confirm_user_data, nl->pending_frame, nl->pending_len);
            }
            return;
        }
    }

    hal_critical_exit(key);
    HAL_LOGD("ax25_mux: link %u released, channel idle", (unsigned)link_id);
}

void ax25_mux_tick(ax25_mux_t *mux, uint32_t current_tick_10ms) {
    if (!mux)
        return;
    (void) current_tick_10ms;
    uint32_t key = hal_critical_enter();
    if (mux->seized_link != AX25_MUX_NO_SEIZED) {
        hal_critical_exit(key);
        return;
    }
    int8_t next = ax25_mux_select_next(mux);
    if (next >= 0) {
        mux->seized_link = (uint8_t) next;
        mux->links[next].seize_pending = false;
        mux->last_served = (uint8_t) next;
        ax25_mux_link_t *l = &mux->links[next];
        hal_critical_exit(key);
        HAL_LOGD("ax25_mux: tick grants channel to link %d", (int)next);
        if (l->lm_seize_confirm && l->pending_len > 0) {
            l->lm_seize_confirm(l->confirm_user_data, l->pending_frame, l->pending_len);
        }
        return;
    }
    hal_critical_exit(key);
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

    // start modified part: use canonical helpers for frame classification
    // Previous code used a hardcoded literal table with two bugs:
    //   1. 0xE1 was listed as "TEST rsp F=0" but is not a valid AX.25 frame;
    //      TEST P/F=0 = 0xE3, TEST P/F=1 = 0xE3|0x10 = 0xF3. The value 0xE1
    //      has bits[1:0]=01 so it falls through to the S-frame branch, causing
    //      any TEST frame with P=1 (0xF3) to be misclassified as ACK priority.
    //   2. The table only handled specific P/F variants (0x03 and 0x13 for UI,
    //      0xAF/0xBF for XID) leaving other valid P/F combinations uncovered.
    // Fix: ax25_frame_class() checks I first (bit0==0), then S (bits[1:0]==01),
    // then U (bits[1:0]==11), matching Linux ax25_decode() priority exactly.
    // ax25_u_subtype() strips bit 4 (P/F) before comparing against AX25_U_*
    // constants, covering all P/F combinations with a single comparison.
    {
        ax25_frame_class_t fc = ax25_frame_class(ctrl);
        if (fc == AX25_FRAME_CLASS_I) {
            // I-frame: reliable data transfer, highest data priority
            return AX25_MUX_PRI_DATA;
        }
        if (fc == AX25_FRAME_CLASS_S) {
            // S-frame: acknowledgment / flow control
            return AX25_MUX_PRI_ACK;
        }
        // U-frame: strip P/F bit and compare subtype against named constants
        uint8_t sub = ax25_u_subtype(ctrl);
        if (sub == AX25_U_UI || sub == AX25_U_XID || sub == AX25_U_TEST) {
            // UI, XID, TEST: connectionless / management frames, low priority
            return AX25_MUX_PRI_UI;
        }
        // SABM, SABME, DISC, DM, UA, FRMR: urgent connection-control frames
        return AX25_MUX_PRI_URGENT;
    }
    // end modified part: use canonical helpers for frame classification
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
    HAL_LOGI("ax25_mux: initialized (max links=%u)", (unsigned)AX25_MUX_MAX_LINKS);

    return 0;
}

uint8_t ax25_mux_register_link(ax25_mux_t *mux, ax25_connection_t *conn, const ax25_address_t *local_addr, const ax25_address_t *peer_addr,
        uint8_t *link_id_out) {
    if (!mux || !conn || !local_addr || !peer_addr || !link_id_out)
        return 1;

    uint32_t key = hal_critical_enter();
    for (int i = 0; i < AX25_MUX_MAX_LINKS; i++) {
        if (!mux->links[i].active) {
            mux->links[i].conn = conn;
            mux->links[i].local_addr = *local_addr;
            mux->links[i].peer_addr = *peer_addr;
            mux->links[i].active = true;
            mux->links[i].seize_pending = false;
            mux->links[i].seize_priority = 128;
            mux->links[i].pending_len = 0;
            mux->links[i].lm_seize_confirm = NULL;
            mux->links[i].confirm_user_data = NULL;
            *link_id_out = (uint8_t) i;
            mux->num_active++;
            hal_critical_exit(key);
            HAL_LOGI("ax25_mux: link %u registered (active=%u)", (unsigned)i, (unsigned)mux->num_active);
            return 0;
        }
    }
    hal_critical_exit(key);
    HAL_LOGW("ax25_mux: no free link slots");

    return 2;
}

uint8_t ax25_mux_unregister_link(ax25_mux_t *mux, uint8_t link_id) {
    if (!mux || link_id >= AX25_MUX_MAX_LINKS)
        return 1;

    uint32_t key = hal_critical_enter();
    if (mux->links[link_id].active) {
        mux->links[link_id].active = false;
        mux->links[link_id].conn = NULL;
        if (mux->num_active > 0)
            mux->num_active--;
        hal_critical_exit(key);
        HAL_LOGI("ax25_mux: link %u unregistered (active=%u)", (unsigned)link_id, (unsigned)mux->num_active);
        return 0;
    }
    hal_critical_exit(key);

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
    if (!dest || !src || !conn)
        return NULL;

    uint32_t key = hal_critical_enter();
    if (ax25_find_conn(dest, src) != NULL) {
        hal_critical_exit(key);
        HAL_LOGW("ax25_mux: ax25_alloc_conn duplicate entry rejected");
        return NULL;
    }
    for (i = 0; i < AX25_MAX_CONNECTIONS; i++) {
        if (!conn_table[i].active) {
            strncpy(conn_table[i].dest, dest, 7);
            strncpy(conn_table[i].src, src, 7);
            conn_table[i].conn = conn;
            conn_table[i].active = 1;
            hal_critical_exit(key);
            HAL_LOGD("ax25_mux: connection allocated slot %u", (unsigned)i);
            return &conn_table[i];
        }
    }
    hal_critical_exit(key);
    HAL_LOGE("ax25_mux: connection table full");

    return NULL;
}

void ax25_free_conn(ax25_conn_t *entry) {
    if (!entry)
        return;

    uint32_t key = hal_critical_enter();
    entry->active = 0;
    entry->conn = NULL;
    // clear keys so stale data does not cause false positives on future lookups
    memset(entry->dest, 0, sizeof(entry->dest));
    memset(entry->src, 0, sizeof(entry->src));
    hal_critical_exit(key);
    HAL_LOGD("ax25_mux: connection slot freed");

}
