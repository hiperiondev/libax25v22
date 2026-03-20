#!/usr/bin/env python3
"""
Apply 4 AX.25 State Machine fixes to ax25_state_machine.c
Uses exact-text replacement -- line-number independent.
Usage: python3 apply_fixes.py protocols/ax25/ax25_state_machine.c
"""
import sys, re

def apply(src, old, new, tag):
    if old in src:
        print(f"  [OK] {tag}")
        return src.replace(old, new, 1)
    print(f"  [FAIL] {tag} -- block not found")
    return src

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 apply_fixes.py <path/to/ax25_state_machine.c>")
        sys.exit(1)
    path = sys.argv[1]
    with open(path, 'r') as f:
        src = f.read()

    # -------------------------------------------------------------------------
    # Fix 1: handle_received_rr  State 4 complete handling
    # -------------------------------------------------------------------------
    OLD1 = (
        "// Stop T1 here; ax25_tick will restart it if needed\n"
        "    conn->retry_count = 0;\n"
        "    T1_STOP(conn);  // Stop T1 - will be restarted by ax25_tick if frames pending\n"
        "\n"
        "// per AX.25 v2.2 §6.4.7 SDL, when in Timer Recovery\n"
        "// state and we receive RR F=1 (a response to our poll) with V(A)==V(S) (all\n"
        "// outstanding frames have now been acknowledged), we must return to Connected\n"
        "// state.  Without this transition ax25_send_data_raw() returns 6 forever and\n"
        "// the link is permanently blocked after the first T1 expiry.\n"
        "    if (conn->state == AX25_STATE_TIMER_RECOVERY && pf && conn->vars.va == conn->vars.vs) {\n"
        "        conn->state = AX25_STATE_CONNECTED;\n"
        "    }\n"
        "\n"
        "    if (pf) {"
    )
    NEW1 = (
        "    // start modified part\n"
        "    // State 4 (TIMER_RECOVERY) RR handling per Linux ax25_std_frame_in.c:\n"
        "    //   F=1: peer responded to our poll: decide whether to return to CONNECTED or retransmit.\n"
        "    //   F=0: ack already processed by ax25_process_nr; do NOT send response, stay in recovery.\n"
        "    // Sending RR/RNR in TIMER_RECOVERY would create a spurious poll loop with the Linux peer.\n"
        "    if (conn->state == AX25_STATE_TIMER_RECOVERY) {\n"
        "        if (pf) {\n"
        "            // RR F=1: peer answered our RR P=1 enquiry\n"
        "            conn->retry_count = 0;\n"
        "            T1_STOP(conn);\n"
        "            if (conn->vars.va == conn->vars.vs) {\n"
        "                // All outstanding frames acknowledged: return to CONNECTED, start T3\n"
        "                conn->state = AX25_STATE_CONNECTED;\n"
        "                T3_START(conn);\n"
        "            } else {\n"
        "                // Frames still outstanding: retransmit from V(A), restart T1 to await next F=1\n"
        "                uint8_t idx = conn->tx_queue.head;\n"
        "                for (uint8_t i = 0; i < conn->tx_queue.count; i++) {\n"
        "                    if (conn->callbacks.transmit)\n"
        "                        conn->callbacks.transmit(conn->user_data, conn->tx_queue.frames[idx],\n"
        "                                                 conn->tx_queue.lengths[idx]);\n"
        "                    conn->stats.iframe_retransmitted++;\n"
        "                    if (conn->stats.iframe_retransmitted == 0)\n"
        "                        conn->stats.iframe_retransmitted = 1;\n"
        "                    idx = (idx + 1) % AX25_MAX_QUEUE_SIZE;\n"
        "                }\n"
        "                T1_START(conn);\n"
        "            }\n"
        "        }\n"
        "        // F=0 in TIMER_RECOVERY: ax25_process_nr already advanced V(A);\n"
        "        // T1 was stopped there if queue drained, else keeps running (waiting for F=1).\n"
        "        // Never respond with RR/RNR in TIMER_RECOVERY state.\n"
        "        return;\n"
        "    }\n"
        "    // end modified part\n"
        "\n"
        "    // CONNECTED and other states\n"
        "    conn->retry_count = 0;\n"
        "    T1_STOP(conn);\n"
        "\n"
        "    if (pf) {"
    )
    src = apply(src, OLD1, NEW1, "Fix 1: handle_received_rr State 4")

    # -------------------------------------------------------------------------
    # Fix 2: ax25_tick T1 expiry -- TIMER_RECOVERY and FRAME_REJECT branches
    # -------------------------------------------------------------------------
    OLD2 = (
        "            } else if (conn->state == AX25_STATE_AWAITING_RELEASE) {\n"
        "                // Retransmit DISC per AX.25 v2.2 Section 6.3.4 and C4 SDL.\n"
        "                // Do NOT transition to TIMER_RECOVERY: stay in AWAITING_RELEASE.\n"
        "                send_disc(conn);\n"
        "                T1_START(conn);\n"
        "            } else {\n"
        "                // Enter timer recovery and retransmit per AX.25 v2.2 Section 6.7.1.1"
    )
    NEW2 = (
        "            } else if (conn->state == AX25_STATE_AWAITING_RELEASE) {\n"
        "                // Retransmit DISC per AX.25 v2.2 Section 6.3.4 and C4 SDL.\n"
        "                // Do NOT transition to TIMER_RECOVERY: stay in AWAITING_RELEASE.\n"
        "                send_disc(conn);\n"
        "                T1_START(conn);\n"
        "            // start modified part\n"
        "            } else if (conn->state == AX25_STATE_TIMER_RECOVERY) {\n"
        "                // State 4 T1 expiry: send RR P=1 enquiry, do NOT retransmit I-frames.\n"
        "                // Per Linux ax25_std_t1timer_expiry() State 4: only poll, let peer's\n"
        "                // F=1 response drive retransmission via handle_received_rr.\n"
        "                // This prevents the Linux peer seeing non-poll RR frames it counts\n"
        "                // against its own N2, causing premature disconnect.\n"
        "                if (conn->local_busy)\n"
        "                    send_rnr(conn, true);\n"
        "                else\n"
        "                    send_rr(conn, true);\n"
        "                T1_START(conn);\n"
        "            } else if (conn->state == AX25_STATE_FRAME_REJECT) {\n"
        "                // FRAME_REJECT T1 expiry: resend stored FRMR and restart T1\n"
        "                // per AX.25 v2.2 Section 4.4.5.  Do NOT enter TIMER_RECOVERY.\n"
        "                resend_stored_frmr(conn);\n"
        "                T1_START(conn);\n"
        "            // end modified part\n"
        "            } else {\n"
        "                // CONNECTED: enter timer recovery, retransmit per AX.25 v2.2 Section 6.7.1.1"
    )
    src = apply(src, OLD2, NEW2, "Fix 2: ax25_tick T1 TIMER_RECOVERY+FRAME_REJECT branches")

    # -------------------------------------------------------------------------
    # Fix 3: ax25_tick T3 expiry -- transition to TIMER_RECOVERY
    # -------------------------------------------------------------------------
    OLD3 = (
        "        if (conn->local_busy) {\n"
        "            send_rnr(conn, true);  // RNR P=1: locally busy, poll peer for status\n"
        "        } else {\n"
        "            send_rr(conn, true);   // RR P=1: not busy, idle link poll\n"
        "        }\n"
        "\n"
        "        T1_START(conn);  // Start T1 for poll response\n"
        "        T3_START(conn);  // Restart T3\n"
        "    }\n"
        "}\n"
        "\n"
        "// Main entry point for received frames"
    )
    NEW3 = (
        "        // start modified part\n"
        "        // T3 expiry in State 3 (CONNECTED): send RR/RNR P=1 and move to State 4\n"
        "        // (TIMER_RECOVERY) per Linux ax25_std_t3timer_expiry().  The Linux peer\n"
        "        // drives its own N2 counter only via T1 retries in State 4; staying in\n"
        "        // State 3 would produce unsolicited non-poll RR frames that Linux counts\n"
        "        // against its own N2, causing premature disconnect from the Linux side.\n"
        "        T3_STOP(conn);\n"
        "        if (conn->local_busy) {\n"
        "            send_rnr(conn, true);  // RNR P=1: locally busy, poll peer for status\n"
        "        } else {\n"
        "            send_rr(conn, true);   // RR P=1: not busy, idle link poll\n"
        "        }\n"
        "        if (conn->state == AX25_STATE_CONNECTED) {\n"
        "            // Transition to State 4: T1 monitors F=1 response; T3 stopped above\n"
        "            conn->retry_count = 0;\n"
        "            conn->state = AX25_STATE_TIMER_RECOVERY;\n"
        "        }\n"
        "        // end modified part\n"
        "        T1_START(conn);  // Start T1 for poll response\n"
        "    }\n"
        "}\n"
        "\n"
        "// Main entry point for received frames"
    )
    src = apply(src, OLD3, NEW3, "Fix 3: ax25_tick T3 transition to TIMER_RECOVERY")

    # -------------------------------------------------------------------------
    # Fix 4: ax25_process_frame UA in TIMER_RECOVERY
    # -------------------------------------------------------------------------
    OLD4 = (
        "            } else if (conn->state == AX25_STATE_FRAME_REJECT) {\n"
        "                // UA received in FRMR state - return to DISCONNECTED per Section 4.4.5\n"
        "                conn->state = AX25_STATE_DISCONNECTED;\n"
        "\n"
        "                // Clear all state\n"
        "                clear_srej_state(conn);\n"
        "                conn->rej_exception = false;\n"
        "                conn->peer_busy = false;\n"
        "                conn->local_busy = false;\n"
        "                conn->rnr_start_tick = 0;\n"
        "                conn->frmr_pending = false;\n"
        "                conn->frmr_retry_count = 0;\n"
        "\n"
        "                // Notify upper layer\n"
        "                // DL-CONNECT confirm: local station initiated (sent SABM), peer replied UA.\n"
        "                // initiated_locally = true per AX.25 v2.2 Section 5.3 / Appendix D.3.\n"
        "                if (conn->callbacks.on_connect) {\n"
        "                    conn->callbacks.on_connect(conn->user_data, true);\n"
        "                }\n"
        "            }\n"
        "            // UA frames in other states are ignored per AX.25 v2.2\n"
        "            break;"
    )
    NEW4 = (
        "            } else if (conn->state == AX25_STATE_FRAME_REJECT) {\n"
        "                // UA received in FRMR state - return to DISCONNECTED per Section 4.4.5\n"
        "                conn->state = AX25_STATE_DISCONNECTED;\n"
        "\n"
        "                // Clear all state\n"
        "                clear_srej_state(conn);\n"
        "                conn->rej_exception = false;\n"
        "                conn->peer_busy = false;\n"
        "                conn->local_busy = false;\n"
        "                conn->rnr_start_tick = 0;\n"
        "                conn->frmr_pending = false;\n"
        "                conn->frmr_retry_count = 0;\n"
        "\n"
        "                // Notify upper layer\n"
        "                // DL-CONNECT confirm: local station initiated (sent SABM), peer replied UA.\n"
        "                // initiated_locally = true per AX.25 v2.2 Section 5.3 / Appendix D.3.\n"
        "                if (conn->callbacks.on_connect) {\n"
        "                    conn->callbacks.on_connect(conn->user_data, true);\n"
        "                }\n"
        "            // start modified part\n"
        "            } else if (conn->state == AX25_STATE_TIMER_RECOVERY) {\n"
        "                // UA unexpected in State 4 (TIMER_RECOVERY): we never sent SABM/SABME\n"
        "                // so this UA is unsolicited.  Linux ax25_std_frame_in State 4 treats it\n"
        "                // like DM (remote reset) -- issue DL-ERROR A and disconnect.\n"
        "                FIRE_DL_ERROR(conn, AX25_DL_ERROR_A);\n"
        "                conn->state = AX25_STATE_DISCONNECTED;\n"
        "                T1_STOP(conn);\n"
        "                T3_STOP(conn);\n"
        "                conn->retry_count = 0;\n"
        "                conn->peer_busy = false;\n"
        "                conn->local_busy = false;\n"
        "                while (conn->tx_queue.count > 0) {\n"
        "                    hal_mem_free(conn->tx_queue.frames[conn->tx_queue.head]);\n"
        "                    conn->tx_queue.frames[conn->tx_queue.head] = NULL;\n"
        "                    conn->tx_queue.head = (conn->tx_queue.head + 1) % AX25_MAX_QUEUE_SIZE;\n"
        "                    conn->tx_queue.count--;\n"
        "                }\n"
        "                if (conn->callbacks.on_disconnect)\n"
        "                    conn->callbacks.on_disconnect(conn->user_data, 1);\n"
        "            // end modified part\n"
        "            }\n"
        "            // UA frames in other states are ignored per AX.25 v2.2\n"
        "            break;"
    )
    src = apply(src, OLD4, NEW4, "Fix 4: ax25_process_frame UA in TIMER_RECOVERY")

    with open(path, 'w') as f:
        f.write(src)
    print(f"\nDone. Written to {path}")

if __name__ == '__main__':
    main()
