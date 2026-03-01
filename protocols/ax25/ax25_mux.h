/**
 * @file ax25_mux.h
 * @brief AX.25 v2.2 Protocol Library - Link Multiplexer
 * @author Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
 * @copyright GNU General Public License v3
 *
 * @section Overview
 * Manages up to 8 concurrent AX.25 links over a shared physical channel.
 * Routes received frames to the correct connection based on destination/source
 * addresses or dispatches UI broadcast frames (CQ/APRS/BEACON) to all active
 * links per AX.25 v2.2 Section 2.7 (Link Multiplexer) and Section 3.12.5.
 *
 * @section Features
 * - Address equality with callsign/SSID comparison (space-padded callsigns).
 * - Broadcast UI detection for connectionless traffic (APRS, beacons).
 * - Registration/unregistration with link ID output for application use.
 * - Tick-based processing for timers in ax25_process_frame().
 * - Connection table indexed by address pair for O(n) lookup.
 *
 * @section Standards_References
 * - AX.25 v2.2 Sections 2.7, 3.12 (addresses), 6.3 (UI handling)
 * - Compatible with DireWolf/Linux kernel multi-link operation.
 *
 * @see https://github.com/hiperiondev/libax25v22
 * @see https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
 * @see https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf
 * @see https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf
 */

#ifndef AX25_MUX_H_
#define AX25_MUX_H_

#include "ax25.h"
#include "ax25_state_machine.h"
#include "common.h"

/** Maximum simultaneous links per multiplexer (configurable). */
#define AX25_MUX_MAX_LINKS 8
#ifndef AX25_MUX_FRAME_BUF
#define AX25_MUX_FRAME_BUF  MAX_FRAME_SIZE  /**< 340 bytes, matches common.h MAX_FRAME_SIZE */
#endif
#define AX25_MUX_PRI_UI     0u   /**< UI, XID, TEST, management frames - lowest priority */
#define AX25_MUX_PRI_DATA   100u /**< I-frames - normal data priority */
#define AX25_MUX_PRI_ACK    200u /**< S-frames (RR/RNR/REJ/SREJ) - acknowledgment priority */
#define AX25_MUX_PRI_URGENT 255u /**< U-frames (SABM/SABME/DISC/DM/UA/FRMR) - urgent link control */
#define AX25_MUX_NO_SEIZED  0xFF
// connection table definition per AX.25 v2.2 SDL Appendix C3
// AX25_MAX_CONNECTIONS: maximum simultaneous AX.25 connections indexed by address pair.
// Matches AX25_MUX_MAX_LINKS so each link slot can have one connection entry.
#ifndef AX25_MAX_CONNECTIONS
#define AX25_MAX_CONNECTIONS 8
#endif

// ax25_conn_t: one entry in the static connection table.
// Indexed by (dest callsign+SSID, src callsign+SSID) pair for O(n) lookup.
// dest and src store the AX.25 encoded callsign as 6 printable ASCII chars
// plus a 1-byte SSID (packed: chars[0..5] = callsign space-padded, chars[6] = SSID 0-15).
typedef struct {
    uint8_t active;   // 1 = slot in use, 0 = free
    char dest[7];  // remote callsign (6 chars space-padded) + SSID byte
    char src[7];   // local  callsign (6 chars space-padded) + SSID byte
    ax25_connection_t *conn;    // pointer to the associated connection state machine
} ax25_conn_t;

typedef void (*ax25_lm_seize_confirm_t)(void *user_data, uint8_t *frame, size_t len);

typedef struct {
    ax25_connection_t *conn; /**< Owning connection state machine */
    ax25_address_t local_addr; /**< Local station address */
    ax25_address_t peer_addr; /**< Remote station address */
    bool active; /**< Link slot in use */
    bool seize_pending; /**< Link has a pending channel seize request */
    uint8_t seize_priority; /**< Priority of the pending seize request (0 = lowest, 255 = highest) */
    uint8_t pending_frame[AX25_MUX_FRAME_BUF];
    size_t pending_len;
    ax25_lm_seize_confirm_t lm_seize_confirm;
    void *confirm_user_data;
} ax25_mux_link_t;

typedef struct {
    ax25_mux_link_t links[AX25_MUX_MAX_LINKS]; /**< Link slots */
    uint8_t num_active; /**< Count of active links */
    uint8_t last_served; /**< Last link served by round-robin scheduler (0..7) */
    uint8_t seized_link; /**< Currently seized link or AX25_MUX_NO_SEIZED */
} ax25_mux_t;

typedef struct {
    ax25_mux_t *mux; /**< Pointer to the multiplexer */
    uint8_t link_id; /**< This link's slot ID in the mux */
} ax25_mux_adapter_ctx_t;

/**
 * @brief Initialize multiplexer to empty state
 *
 * @param[in,out] mux Pointer to mux structure
 * @return 0 on success, 1 on NULL
 */
uint8_t ax25_mux_init(ax25_mux_t *mux);

/**
 * @brief Register a new link
 *
 * @param[in,out] mux          Multiplexer
 * @param[in]     conn         Connection handle
 * @param[in]     local_addr   Local address
 * @param[in]     peer_addr    Peer address
 * @param[out]    link_id_out  Assigned slot ID (0..7)
 * @return 0 success, 1 invalid param, 2 no free slots
 */
uint8_t ax25_mux_register_link(ax25_mux_t *mux, ax25_connection_t *conn, const ax25_address_t *local_addr, const ax25_address_t *peer_addr,
        uint8_t *link_id_out);

/**
 * @brief Unregister a link by ID
 *
 * @param[in,out] mux     Multiplexer
 * @param[in]     link_id Slot to free
 * @return 0 success, 1 invalid
 */
uint8_t ax25_mux_unregister_link(ax25_mux_t *mux, uint8_t link_id);

/**
 * @brief Dispatch received frame to matching link(s)
 *
 * UI broadcast frames (type AX25_FRAME_UNNUMBERED_INFORMATION with
 * destination CQ/APRS/BEACON) are delivered to ALL active links.
 * Point-to-point frames routed by exact destination+source match.
 *
 * @param[in,out] mux               Multiplexer
 * @param[in]     frame             Decoded frame
 * @param[in]     current_tick_10ms System tick for timers
 */
void ax25_mux_receive_frame(ax25_mux_t *mux, ax25_frame_t *frame, uint32_t current_tick_10ms);

/**
 * @brief Get the next link that should be allowed to seize the channel (priority + round-robin)
 *
 * Call this before a link is permitted to queue a new frame to the physical layer.
 * The returned link's seize_pending flag is automatically cleared and last_served is updated.
 *
 * @param[in,out] mux Multiplexer
 * @return Link ID (0..7) to serve next, or -1 if no pending seize requests
 */
int8_t ax25_mux_get_next_to_serve(ax25_mux_t *mux);

/**
 * @brief Request channel seize for this link (LM-SEIZE-REQUEST)
 */
uint8_t ax25_mux_lm_seize_request(ax25_mux_t *mux, uint8_t link_id, const uint8_t *frame, size_t len, uint8_t priority);

/**
 * @brief Release the channel (LM-RELEASE-REQUEST) - must be called by the confirm callback after queuing to physical
 */
void ax25_mux_lm_release(ax25_mux_t *mux, uint8_t link_id);

/**
 * @brief Periodic mux scheduler (call every 10 ms together with physical tick)
 */
void ax25_mux_tick(ax25_mux_t *mux, uint32_t current_tick_10ms);

/**
 * @brief Set the seize-confirm callback for a link
 */
void ax25_mux_set_lm_seize_confirm(ax25_mux_t *mux, uint8_t link_id, ax25_lm_seize_confirm_t confirm, void *user_data);

/**
 * @brief Priority classification from raw frame (used by adapter)
 */
uint8_t ax25_mux_classify_priority(const uint8_t *frame, size_t len);

/**
 * @brief Transparent transmit adapter for ax25_connection_t->callbacks.transmit
 *        (zero changes to ax25_state_machine.c required)
 */
void ax25_mux_transmit_adapter(void *user_data, uint8_t *frame, size_t len);

//  connection table API per AX.25 v2.2 SDL Appendix C3

// ax25_find_conn: look up an existing connection by (dest, src) address pair.
// dest: remote callsign key, 6 chars space-padded + SSID as 7th byte.
// src:  local  callsign key, 6 chars space-padded + SSID as 7th byte.
// Returns pointer to the matching ax25_conn_t slot, or NULL if not found.
ax25_conn_t* ax25_find_conn(const char *dest, const char *src);

// ax25_alloc_conn: claim a free slot in the static connection table.
// dest: remote callsign + SSID key (7 bytes).
// src:  local  callsign + SSID key (7 bytes).
// conn: associated connection state machine instance.
// Returns pointer to the allocated ax25_conn_t on success, NULL if table full
// or if an entry with the same (dest, src) pair already exists.
ax25_conn_t* ax25_alloc_conn(const char *dest, const char *src, ax25_connection_t *conn);

// ax25_free_conn: release a connection table slot back to the free list.
// Safe to call with NULL (no-op).
void ax25_free_conn(ax25_conn_t *entry);

#endif /* AX25_MUX_H_ */
