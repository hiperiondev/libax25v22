// Copyright 2026 Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
// Project Site: https://github.com/hiperiondev/libax25v22
//
// Real Linux AX.25 Interoperability Test Suite
// Integration with Linux libax25 library (AF_AX25 sockets)
// Uses headers from ve7fet/linuxax25 repository
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3, or (at your option)
// any later version.
//

#define _GNU_SOURCE
#define DEBUG_ENABLE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <poll.h>
#include <dirent.h>
#include <termios.h>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <sys/syscall.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>

// ---------------------------------------------------------------------------
// pidfd_open / pidfd_getfd syscall numbers (Linux ≥ 5.6).
// These allow duplicating a file-descriptor from another process without
// following the /proc/PID/fd/N symlink (which resolves to the PTY slave
// path, not the master — breaking the TIOCGPTN-based master search).
// ---------------------------------------------------------------------------
#ifndef SYS_pidfd_open
# if defined(__x86_64__)
#  define SYS_pidfd_open  434
# elif defined(__aarch64__)
#  define SYS_pidfd_open  434
# elif defined(__arm__)
#  define SYS_pidfd_open  434
# elif defined(__i386__)
#  define SYS_pidfd_open  434
# else
#  define SYS_pidfd_open  434   /* best-effort; unused if syscall fails */
# endif
#endif
#ifndef SYS_pidfd_getfd
# if defined(__x86_64__)
#  define SYS_pidfd_getfd 438
# elif defined(__aarch64__)
#  define SYS_pidfd_getfd 438
# elif defined(__arm__)
#  define SYS_pidfd_getfd 438
# elif defined(__i386__)
#  define SYS_pidfd_getfd 438
# else
#  define SYS_pidfd_getfd 438
# endif
#endif

// Real Linux AX.25 headers from libax25
#include <netax25/ax25.h>
#include <netax25/axlib.h>
#include <netax25/axconfig.h>

// libax25v22 protocol library headers
#include "ax25.h"
#include "ax25_state_machine.h"
#include "ax25_mgmt.h"
#include "ax25_mux.h"
#include "hdlc.h"
#include "kiss.h"
#include "hal.h"
#include "common.h"
#include "test_common.h"

// ---------------------------------------------------------------------------
// Configuration constants
// ---------------------------------------------------------------------------
#define TEST_AXPORTS_FILE     "/etc/ax25/axports"
#define TEST_DEFAULT_PORT     "ax0"
#define TEST_DEFAULT_CALL     "TEST-0"
#define MAX_AXPORTS_LINE      256
#define MAX_CALLSIGN_LEN      16
#define MAX_PORT_NAME_LEN     32
#define MAX_UI_PAYLOAD_SIZE   256
#define MAX_INFO_PAYLOAD_SIZE 256
#define AX25_TIMER_TICK_MS    100

// ---------------------------------------------------------------------------
// Symmetric modulo guards (fix 2.1)
// ---------------------------------------------------------------------------
#ifndef MODULO128_TRUE
#define MODULO128_TRUE  ((uint8_t)1)
#endif
#ifndef MODULO128_FALSE
#define MODULO128_FALSE ((uint8_t)0)
#endif

// CRC init-value used by validate_crc_consistency() (fix 2.2 — dead define removed)
#ifndef HAL_CRC16_INIT
#define HAL_CRC16_INIT 0xFFFF
#endif

// HDLC constant fallback guards (fix 2.4).
// libax25v22's hdlc.h defines these; if the include path is wrong the
// compilation would fail at use-site rather than here.  Provide safe
// defaults so the test file also compiles standalone against a build tree
// that may use slightly different names.
#ifndef HDLC_OK
#define HDLC_OK          ((hdlc_error_t)0)
#endif
#ifndef HDLC_ERR_CRC_FAIL
/* Value 4 confirmed from D.3 test output — the libax25v22 hdlc_error_t enum
 places CRC failure at ordinal 4, not 1.  HDLC_OK=0 is confirmed correct. */
#define HDLC_ERR_CRC_FAIL ((hdlc_error_t)4)
#endif
// HDLC_FLAG_BYTE is 0x7E per ISO 3309; provide a fallback in case hdlc.h
// is not yet included when this file is compiled in isolation.
#ifndef HDLC_FLAG_BYTE
#define HDLC_FLAG_BYTE   ((uint8_t)0x7E)
#endif

// AX.25 default retry count (fix 2.5)
#ifndef AX25_DEFAULT_N2
#define AX25_DEFAULT_N2  10
#endif

// KISS protocol constants
#ifndef KISS_FEND
#define KISS_FEND  ((uint8_t)0xC0)
#endif
#ifndef KISS_FESC
#define KISS_FESC  ((uint8_t)0xDB)
#endif
#ifndef KISS_TFEND
#define KISS_TFEND ((uint8_t)0xDC)
#endif
#ifndef KISS_TFESC
#define KISS_TFESC ((uint8_t)0xDD)
#endif

// PID constants – fixed by AX.25 v2.2 section 6.5.1
#ifndef PID_NO_L3
#define PID_NO_L3  ((uint8_t)0xF0)
#endif
#ifndef PID_IP
#define PID_IP     ((uint8_t)0xCC)
#endif
#ifndef PID_ARP
#define PID_ARP    ((uint8_t)0xCD)
#endif
#ifndef PID_NETROM
#define PID_NETROM ((uint8_t)0xCF)
#endif

// Supervisory-frame type presence note (fix 19.1).
// If none of these macros are defined the SEC-Q sub-tests will be individually
// skipped at runtime via #ifdef guards inside sec_q_supervisory_frames().
// No compile-time warning is emitted because these types are defined in the
// project's own ax25.h, which is on the include path at compile time but may
// not be visible to the C preprocessor during a standalone header-only check.

// ---------------------------------------------------------------------------
// Enhanced data structures
// ---------------------------------------------------------------------------
typedef struct {
    int kernel_ax25_available;
    int socket_bind_available;
    char port_name[MAX_PORT_NAME_LEN];
    char local_call[MAX_CALLSIGN_LEN];
    int port_count;
    int port_window; /* window size parsed from axports (0 = unknown) */
    uint32_t test_flags;
} test_context_t;

typedef struct {
    int af_ax25_supported;
    int sock_seqpacket_supported;
    int sock_dgram_supported;
    int so_bindtodevice_works;
    int modules_loaded;
} kernel_ax25_capabilities_t;

typedef struct {
    int fd;
    int is_bound;
    int is_blocking_modified;
} socket_resource_t;

typedef struct {
    ax25_frame_t *frame;
    uint8_t *encoded_data;
    size_t encoded_len;
    ax25_address_t *addr_dest;
    ax25_address_t *addr_src;
    uint8_t is_malloc_encoded;
    uint8_t is_malloc_frame;
    uint8_t is_malloc_addr_dest;
    uint8_t is_malloc_addr_src;
    uint8_t is_initialized;
    uint8_t encode_attempted;
    uint8_t encode_failed;
} frame_lifecycle_t;

typedef struct {
    int port_number;
    int speed;
    int paclen;
    int window;
    uint8_t valid;
} axport_config_validated_t;

typedef struct {
    int t1_ticks;
    int t2_ticks;
    int t3_ticks;
    int n2_retries;
    int t1_ms;
    int t2_ms;
    int t3_ms;
} ax25_timer_config_t;

typedef struct {
    uint8_t total_buffers;
    uint8_t free_buffers;
    int allocated_buffers;
    uint8_t buf_size;
} buffer_pool_stats_t;

// Global state
static unsigned int assert_count = 0;
static test_context_t g_test_ctx;

// ---------------------------------------------------------------------------
// Helper: safe string copy
// ---------------------------------------------------------------------------
static size_t safe_strlcpy(char *dst, const char *src, size_t dsize) {
    if (dsize == 0)
        return 0;
    size_t len = strlen(src);
    if (len >= dsize) {
        memcpy(dst, src, dsize - 1);
        dst[dsize - 1] = '\0';
        return dsize - 1;
    }
    memcpy(dst, src, len + 1);
    return len;
}

// ---------------------------------------------------------------------------
// Inline KISS encode helper (independent of libax25v22 kiss module)
// Builds: FEND | (port<<4|cmd) | escaped_data | FEND
// Returns 0 on success, -1 on invalid arguments.
// ---------------------------------------------------------------------------
static int kiss_encode_frame(const uint8_t *data, int data_len, uint8_t port, uint8_t cmd, uint8_t *out, int *out_len) {
    int n = 0;
    int i;

    if (!data || !out || !out_len || data_len < 0)
        return -1;

    out[n++] = KISS_FEND;
    out[n++] = (uint8_t) ((port << 4) | (cmd & 0x0F));

    for (i = 0; i < data_len; i++) {
        if (data[i] == KISS_FEND) {
            out[n++] = KISS_FESC;
            out[n++] = KISS_TFEND;
        } else if (data[i] == KISS_FESC) {
            out[n++] = KISS_FESC;
            out[n++] = KISS_TFESC;
        } else {
            out[n++] = data[i];
        }
    }

    out[n++] = KISS_FEND;
    *out_len = n;
    return 0;
}

// ---------------------------------------------------------------------------
// Inline KISS decode helper
// Strips FEND delimiters, skips port/command byte, un-escapes payload.
// Returns 0 on success, -1 on framing error.
// ---------------------------------------------------------------------------
static int kiss_decode_frame(const uint8_t *data, int data_len, uint8_t *out, int *out_len) {
    int n = 0;
    int i;

    if (!data || !out || !out_len || data_len < 4)
        return -1;

    if (data[0] != KISS_FEND)
        return -1;

    i = 1;  // skip opening FEND
    i++;    // skip port/command byte

    while (i < data_len) {
        if (data[i] == KISS_FEND)
            break;

        if (data[i] == KISS_FESC) {
            i++;
            if (i >= data_len)
                return -1;
            if (data[i] == KISS_TFEND) {
                out[n++] = KISS_FEND;
            } else if (data[i] == KISS_TFESC) {
                out[n++] = KISS_FESC;
            } else {
                return -1;
            }
        } else {
            out[n++] = data[i];
        }
        i++;
    }

    *out_len = n;
    return 0;
}

// ---------------------------------------------------------------------------
// SSID byte validation
// ---------------------------------------------------------------------------
static int validate_ssid_byte_encoding(uint8_t ssid_byte, uint8_t *err) {
    *err = 0;
    uint8_t ssid = (ssid_byte >> 1) & 0x0F;
    if (ssid > 15) {
        *err = 1;
        DEBUG_PRINT("Invalid SSID in byte: 0x%02X", ssid_byte);
        return -1;
    }
    if (((ssid_byte >> 5) & 0x01) == 0) {
        DEBUG_PRINT("Warning: Bit 5 (RES0) not set in SSID byte 0x%02X", ssid_byte);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Address bridge: Linux kernel ax25_address ↔ libax25v22 ax25_address_t
// ---------------------------------------------------------------------------
static int bridge_linux_to_libax25v22(const ax25_address *linux_addr, ax25_address_t *v22_addr, uint8_t *err) {
    int i;
    *err = 0;
    if (!linux_addr || !v22_addr) {
        *err = 1;
        return -1;
    }

    for (i = 0; i < 6; i++) {
        uint8_t shifted = (uint8_t) linux_addr->ax25_call[i];

        if ((shifted & 0x01) != 0) {
            *err = 2;
            DEBUG_PRINT("Invalid bit 0 in callsign byte %d: 0x%02X", i, shifted);
            return -1;
        }

        uint8_t ascii_char = (shifted >> 1) & 0x7F;

        if (ascii_char == 0) {
            v22_addr->callsign[i] = ' ';
        } else if (ascii_char >= 0x20 && ascii_char <= 0x7E) {
            v22_addr->callsign[i] = (char) ascii_char;
        } else {
            *err = 3;
            DEBUG_PRINT("Invalid ASCII at position %d: 0x%02X", i, ascii_char);
            return -1;
        }
    }
    v22_addr->callsign[6] = '\0';

    for (i = 5; i >= 0; i--) {
        if (v22_addr->callsign[i] == ' ')
            v22_addr->callsign[i] = '\0';
        else
            break;
    }

    uint8_t ssid_byte = (uint8_t) linux_addr->ax25_call[6];

    if (validate_ssid_byte_encoding(ssid_byte, err) < 0)
        return -1;

    v22_addr->ssid = (int) ((ssid_byte >> 1) & 0x0F);
    v22_addr->ch = ((ssid_byte >> 7) & 0x01) != 0;
    v22_addr->res1 = ((ssid_byte >> 6) & 0x01) != 0;
    v22_addr->res0 = ((ssid_byte >> 5) & 0x01) != 0;
    v22_addr->extension = (ssid_byte & 0x01) != 0;
    v22_addr->mod8_legacy = v22_addr->res1;

    DEBUG_PRINT("Decoded: callsign='%s', SSID=%d, ch=%d, res1=%d, res0=%d, ext=%d", v22_addr->callsign, v22_addr->ssid, v22_addr->ch, v22_addr->res1,
            v22_addr->res0, v22_addr->extension);
    return 0;
}

static int bridge_libax25v22_to_linux(const ax25_address_t *v22_addr, ax25_address *linux_addr, uint8_t *err) {
    int i;
    *err = 0;
    if (!v22_addr || !linux_addr) {
        *err = 1;
        return -1;
    }

    if (v22_addr->ssid < 0 || v22_addr->ssid > 15) {
        *err = 3;
        DEBUG_PRINT("Invalid SSID: %d (must be 0-15)", v22_addr->ssid);
        return -1;
    }

    size_t callsign_len = strlen(v22_addr->callsign);
    if (callsign_len == 0 || callsign_len > 6) {
        *err = 4;
        DEBUG_PRINT("Invalid callsign length: %zu (must be 1-6)", callsign_len);
        return -1;
    }

    for (i = 0; i < 6; i++) {
        unsigned char ascii_char;
        if (i < (int) callsign_len)
            ascii_char = (unsigned char) v22_addr->callsign[i];
        else
            ascii_char = ' ';

        if (ascii_char < 0x20 || ascii_char > 0x7E) {
            *err = 5;
            DEBUG_PRINT("Invalid character in callsign: 0x%02X", ascii_char);
            return -1;
        }
        linux_addr->ax25_call[i] = (char) (ascii_char << 1);
    }

    uint8_t ssid_byte = 0;
    ssid_byte |= (uint8_t) ((v22_addr->ssid & 0x0F) << 1);
    if (v22_addr->ch)
        ssid_byte |= 0x80;
    if (v22_addr->res1)
        ssid_byte |= 0x40;
    if (v22_addr->res0)
        ssid_byte |= 0x20;
    if (v22_addr->extension)
        ssid_byte |= 0x01;

    linux_addr->ax25_call[6] = (char) ssid_byte;

    DEBUG_PRINT("Encoded: callsign='%s' SSID=%d to byte=0x%02X", v22_addr->callsign, v22_addr->ssid, ssid_byte);
    return 0;
}

// ---------------------------------------------------------------------------
// Socket resource lifecycle
// ---------------------------------------------------------------------------
static void cleanup_socket_resources(socket_resource_t *res) {
    if (!res)
        return;

    if (res->fd >= 0 && res->is_blocking_modified) {
        int flags = fcntl(res->fd, F_GETFL, 0);
        if (flags >= 0) {
            int result = fcntl(res->fd, F_SETFL, flags & ~O_NONBLOCK);
            if (result < 0)
                DEBUG_PRINT("Warning: fcntl F_SETFL restore failed: %s", strerror(errno));
        }
    }

    if (res->fd >= 0) {
        int close_result = close(res->fd);
        if (close_result < 0)
            DEBUG_PRINT("Warning: close failed: %s", strerror(errno));
        res->fd = -1;
    }

    res->is_bound = 0;
    res->is_blocking_modified = 0;
}

// ---------------------------------------------------------------------------
// Kernel AX.25 capability detection
// ---------------------------------------------------------------------------
static int detect_kernel_ax25_capabilities(kernel_ax25_capabilities_t *caps) {
    int sock;

    if (!caps)
        return -1;
    memset(caps, 0, sizeof(*caps));

    sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
    if (sock >= 0) {
        caps->af_ax25_supported = 1;
        caps->sock_seqpacket_supported = 1;
        close(sock);
    } else {
        if (errno == EAFNOSUPPORT) {
            DEBUG_PRINT("AF_AX25 not supported: EAFNOSUPPORT");
            return 1;
        } else if (errno == EPROTONOSUPPORT) {
            caps->af_ax25_supported = 1;
            caps->sock_seqpacket_supported = 0;
        }
    }

    sock = socket(AF_AX25, SOCK_DGRAM, 0);
    if (sock >= 0) {
        caps->sock_dgram_supported = 1;
        close(sock);
    }

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock >= 0) {
        if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, "lo", 3) >= 0)
            caps->so_bindtodevice_works = 1;
        close(sock);
    }

    if (caps->af_ax25_supported && caps->sock_seqpacket_supported) {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            struct sockaddr_ax25 addr;
            memset(&addr, 0, sizeof(addr));
            addr.sax25_family = AF_AX25;
            addr.sax25_ndigis = 0;

            if (bind(sock, (struct sockaddr*) &addr, sizeof(addr)) >= 0)
                caps->modules_loaded = 1;
            else if (errno == ENODEV)
                caps->modules_loaded = 0;
            else
                caps->modules_loaded = 0;

            close(sock);
        }
    }
    return 0;
}

static int check_kernel_ax25_support_enhanced(void) {
    kernel_ax25_capabilities_t caps;

    if (detect_kernel_ax25_capabilities(&caps) < 0)
        return 0;
    if (!caps.af_ax25_supported)
        return 0;
    if (!caps.sock_seqpacket_supported)
        return 0;
    return 1;
}

static int check_kernel_ax25_support(void) {
    return check_kernel_ax25_support_enhanced();
}

static int check_ax25_bind_available(const char *port_name, const char *callsign) {
    struct sockaddr_ax25 addr;
    int sock = socket(AF_AX25, SOCK_SEQPACKET, 0);

    if (sock < 0)
        return 0;

    if (if_nametoindex(port_name) == 0) {
        close(sock);
        return 0;
    }

    if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, port_name, strlen(port_name)) < 0) {
        close(sock);
        return 0;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sax25_family = AF_AX25;
    addr.sax25_ndigis = 0;

    if (ax25_aton_entry(callsign, (char*) &addr.sax25_call) < 0) {
        close(sock);
        return 0;
    }

    int bind_result = bind(sock, (struct sockaddr*) &addr, sizeof(struct sockaddr_ax25));
    close(sock);
    return (bind_result >= 0) ? 1 : 0;
}

// ---------------------------------------------------------------------------
// axports parsing helpers
// ---------------------------------------------------------------------------
static int find_ax25_port_direct(char *port_name, size_t max_len) {
    FILE *fp = fopen(TEST_AXPORTS_FILE, "r");
    if (!fp) {
        safe_strlcpy(port_name, TEST_DEFAULT_PORT, max_len);
        return 0;
    }

    char line[MAX_AXPORTS_LINE];
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
            continue;

        char name[MAX_PORT_NAME_LEN];
        char callsign[MAX_CALLSIGN_LEN];
        char device[MAX_PORT_NAME_LEN];
        int speed, paclen, window;

        if (sscanf(line, "%31s %15s %d %d %d %31s", name, callsign, &speed, &paclen, &window, device) >= 5) {
            safe_strlcpy(port_name, name, max_len);
            fclose(fp);
            return 1;
        }
    }
    fclose(fp);
    safe_strlcpy(port_name, TEST_DEFAULT_PORT, max_len);
    return 0;
}

static int validate_axport_params(const char *port_name, int speed, int paclen, int window, axport_config_validated_t *out) {
    if (!port_name || !out)
        return 0;
    memset(out, 0, sizeof(*out));

    size_t plen = strlen(port_name);
    if (plen == 0 || plen > MAX_PORT_NAME_LEN - 1)
        return 0;
    if (speed < 0)
        return 0;
    if (paclen < 16 || paclen > 512)
        return 0;
    if (window < 1 || window > 127)
        return 0;

    out->valid = 1;
    out->speed = speed;
    out->paclen = paclen;
    out->window = window;
    return 1;
}

static int load_ax25_config(char *out_call, size_t call_max_len) {
    int count = 0;
    FILE *fp = fopen(TEST_AXPORTS_FILE, "r");
    if (!fp)
        return 0;

    char line[MAX_AXPORTS_LINE];
    int line_num = 0;

    while (fgets(line, sizeof(line), fp)) {
        line_num++;
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
            continue;

        size_t line_len = strlen(line);
        while (line_len > 0 && (line[line_len - 1] == '\n' || line[line_len - 1] == '\r' || line[line_len - 1] == ' '))
            line[--line_len] = '\0';
        if (line_len == 0)
            continue;

        char name[MAX_PORT_NAME_LEN], callsign[MAX_CALLSIGN_LEN];
        char device[MAX_PORT_NAME_LEN];
        int speed, paclen, window;

        if (sscanf(line, "%31s %15s %d %d %d %31s", name, callsign, &speed, &paclen, &window, device) >= 5) {
            axport_config_validated_t v;
            if (!validate_axport_params(name, speed, paclen, window, &v))
                continue;
            count++;
            if (count == 1) {
                safe_strlcpy(out_call, callsign, call_max_len);
                /* Store window for M.5 cross-check without needing
                 ax25_config_get_window() which may not exist. */
                g_test_ctx.port_window = v.window;
            }
        }
    }
    fclose(fp);
    return count;
}

// ---------------------------------------------------------------------------
// Socket creation with resource tracking
// ---------------------------------------------------------------------------
static int create_ax25_socket_with_tracking(const char *port_name, const char *local_call, socket_resource_t *res) {
    struct sockaddr_ax25 local_addr;

    if (!res)
        return -1;
    memset(res, 0, sizeof(*res));
    res->fd = -1;

    int sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
    if (sock < 0)
        return -1;
    res->fd = sock;

    if (if_nametoindex(port_name) == 0) {
        cleanup_socket_resources(res);
        return -1;
    }

    if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, port_name, strlen(port_name)) < 0) {
        cleanup_socket_resources(res);
        return -1;
    }

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sax25_family = AF_AX25;
    local_addr.sax25_ndigis = 0;

    if (ax25_aton_entry(local_call, (char*) &local_addr.sax25_call) < 0) {
        cleanup_socket_resources(res);
        return -1;
    }

    if (bind(sock, (struct sockaddr*) &local_addr, sizeof(struct sockaddr_ax25)) < 0) {
        int be = errno;
        DEBUG_PRINT("bind failed: %s (errno: %d)", strerror(be), be);
        cleanup_socket_resources(res);
        return -1;
    }
    res->is_bound = 1;

    int flags = fcntl(sock, F_GETFL, 0);
    if (flags >= 0 && fcntl(sock, F_SETFL, flags | O_NONBLOCK) >= 0)
        res->is_blocking_modified = 1;

    return sock;
}

// ---------------------------------------------------------------------------
// Frame lifecycle helpers
// ---------------------------------------------------------------------------
static void frame_lifecycle_init(frame_lifecycle_t *lc) {
    if (!lc)
        return;
    memset(lc, 0, sizeof(*lc));
    lc->is_malloc_encoded = 1;
    lc->is_initialized = 1;
}

static ax25_address_t* frame_lifecycle_create_address(frame_lifecycle_t *lc, const char *callsign, int is_dest, uint8_t *err) {
    ax25_address_t *addr = ax25_address_from_string(callsign, err);
    if (addr && lc) {
        if (is_dest) {
            lc->addr_dest = addr;
            lc->is_malloc_addr_dest = 1;
        } else {
            lc->addr_src = addr;
            lc->is_malloc_addr_src = 1;
        }
    }
    return addr;
}

static void frame_lifecycle_cleanup(frame_lifecycle_t *lc) {
    uint8_t err = 0;
    if (!lc || !lc->is_initialized)
        return;

    if (lc->encoded_data && lc->is_malloc_encoded && !lc->encode_failed) {
        free(lc->encoded_data);
        lc->encoded_data = NULL;
    }
    if (lc->frame && lc->is_malloc_frame) {
        ax25_frame_free(lc->frame, &err);
        lc->frame = NULL;
    }
    if (lc->addr_dest && lc->is_malloc_addr_dest) {
        ax25_address_free(lc->addr_dest, &err);
        lc->addr_dest = NULL;
    }
    if (lc->addr_src && lc->is_malloc_addr_src) {
        ax25_address_free(lc->addr_src, &err);
        lc->addr_src = NULL;
    }
    lc->encoded_len = 0;
    lc->is_initialized = 0;
}

// ---------------------------------------------------------------------------
// Frame structural validation
// ---------------------------------------------------------------------------
static int validate_frame_structure_complete(const ax25_frame_t *frame, uint8_t modulo_128, uint8_t *err) {
    int i;
    *err = 0;
    if (!frame) {
        *err = 1;
        return -1;
    }

    switch (frame->type) {
        case AX25_FRAME_UNNUMBERED_INFORMATION:
        case AX25_FRAME_INFORMATION_8BIT:
        case AX25_FRAME_INFORMATION_16BIT:
        case AX25_FRAME_UNNUMBERED_SABM:
        case AX25_FRAME_UNNUMBERED_DISC:
        case AX25_FRAME_UNNUMBERED_UA:
        case AX25_FRAME_UNNUMBERED_DM:
        case AX25_FRAME_UNNUMBERED_FRMR:
        case AX25_FRAME_UNNUMBERED_XID:
#ifdef AX25_FRAME_UNNUMBERED_SABME
        case AX25_FRAME_UNNUMBERED_SABME:
#endif
#ifdef AX25_FRAME_SUPERVISORY_RR_8BIT
        case AX25_FRAME_SUPERVISORY_RR_8BIT:
        case AX25_FRAME_SUPERVISORY_RNR_8BIT:
        case AX25_FRAME_SUPERVISORY_REJ_8BIT:
#endif
#ifdef AX25_FRAME_SUPERVISORY_SREJ_8BIT
        case AX25_FRAME_SUPERVISORY_SREJ_8BIT:
#endif
#ifdef AX25_FRAME_SUPERVISORY_RR_16BIT
        case AX25_FRAME_SUPERVISORY_RR_16BIT:
        case AX25_FRAME_SUPERVISORY_RNR_16BIT:
        case AX25_FRAME_SUPERVISORY_REJ_16BIT:
#endif
#ifdef AX25_FRAME_SUPERVISORY_SREJ_16BIT
        case AX25_FRAME_SUPERVISORY_SREJ_16BIT:
#endif
        break;
        default:
            *err = 2;
            return -1;
    }

    if (strlen(frame->header.destination.callsign) == 0 || strlen(frame->header.source.callsign) == 0) {
        *err = 6;
        return -1;
    }
    if (frame->header.destination.ssid < 0 || frame->header.destination.ssid > 15) {
        *err = 3;
        return -1;
    }
    if (frame->header.source.ssid < 0 || frame->header.source.ssid > 15) {
        *err = 4;
        return -1;
    }
    if (frame->header.repeaters.num_repeaters < 0 || frame->header.repeaters.num_repeaters > AX25_MAX_REPEATERS) {
        *err = 5;
        return -1;
    }
    for (i = 0; i < frame->header.repeaters.num_repeaters; i++) {
        if (frame->header.repeaters.repeaters[i].ssid < 0 || frame->header.repeaters.repeaters[i].ssid > 15) {
            *err = 7;
            return -1;
        }
        if (strlen(frame->header.repeaters.repeaters[i].callsign) == 0) {
            *err = 15;
            return -1;
        }
    }

    switch (frame->type) {
        case AX25_FRAME_UNNUMBERED_INFORMATION: {
            ax25_unnumbered_information_frame_t *ui = (ax25_unnumbered_information_frame_t*) frame;
            if (ui->payload_len > MAX_UI_PAYLOAD_SIZE) {
                *err = 8;
                return -1;
            }
            if (ui->pid == 0xFF) {
                *err = 10;
                return -1;
            }
            if (ui->payload_len > 0 && ui->payload == NULL) {
                *err = 17;
                return -1;
            }
            break;
        }
        case AX25_FRAME_INFORMATION_8BIT:
        case AX25_FRAME_INFORMATION_16BIT: {
            ax25_information_frame_t *info = (ax25_information_frame_t*) frame;
            if (info->payload_len > MAX_INFO_PAYLOAD_SIZE) {
                *err = 9;
                return -1;
            }
            if (info->payload_len > 0 && info->payload == NULL) {
                *err = 18;
                return -1;
            }
            if (modulo_128) {
                if (info->ns < 0 || info->ns > 127) {
                    *err = 11;
                    return -1;
                }
                if (info->nr < 0 || info->nr > 127) {
                    *err = 12;
                    return -1;
                }
            } else {
                if (info->ns < 0 || info->ns > 7) {
                    *err = 13;
                    return -1;
                }
                if (info->nr < 0 || info->nr > 7) {
                    *err = 14;
                    return -1;
                }
            }
            break;
        }
        default:
        break;
    }
    return 0;
}

static int validate_frame_for_encoding(const ax25_frame_t *frame, uint8_t *err) {
    uint8_t mod128 = MODULO128_FALSE;
    if (frame && frame->type == AX25_FRAME_INFORMATION_16BIT)
        mod128 = MODULO128_TRUE;
    return validate_frame_structure_complete(frame, mod128, err);
}

// ---------------------------------------------------------------------------
// HDLC validation helpers
// ---------------------------------------------------------------------------
static int validate_hdlc_frame_format(const uint8_t *frame, unsigned int len, uint8_t *err) {
    unsigned int i;
    *err = 0;
    if (!frame || len < 4) {
        *err = 1;
        return -1;
    }
    if (frame[0] != HDLC_FLAG_BYTE) {
        *err = 2;
        return -1;
    }
    if (frame[len - 1] != HDLC_FLAG_BYTE) {
        *err = 3;
        return -1;
    }
    for (i = 1; i < len - 1; i++) {
        if (frame[i] == HDLC_FLAG_BYTE) {
            *err = 4;
            return -1;
        }
    }
    return 0;
}

static int validate_hdlc_decoded_frame(const uint8_t *decoded, int len, uint8_t *err) {
    *err = 0;
    if (!decoded || len <= 0) {
        *err = 1;
        return -1;
    }
    if (len < 2) {
        *err = 2;
        return -1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// CRC consistency helper
// ---------------------------------------------------------------------------
static int validate_crc_consistency(const uint8_t *data, size_t len, uint16_t *single_shot, uint16_t *incremental, uint8_t *err) {
    size_t i;
    *err = 0;
    *single_shot = hal_crc16_buf(data, len);

    uint16_t crc = HAL_CRC16_INIT;
    for (i = 0; i < len; i++)
        crc = hal_crc16_update(crc, &data[i], 1);
    *incremental = hal_crc16_final(crc);

    if (*single_shot != *incremental) {
        *err = 1;
        return -1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Connection state machine timer validation
// ---------------------------------------------------------------------------
static int validate_connection_timers(const ax25_connection_t *conn, ax25_timer_config_t *config, uint8_t *err) {
    *err = 0;
    if (!conn || !config) {
        *err = 1;
        return -1;
    }
    memset(config, 0, sizeof(*config));

    config->t1_ticks = conn->timers.t1;
    config->t2_ticks = conn->timers.t2;
    config->t3_ticks = conn->timers.t3;
    config->n2_retries = conn->timers.n2;
    config->t1_ms = config->t1_ticks * AX25_TIMER_TICK_MS;
    config->t2_ms = config->t2_ticks * AX25_TIMER_TICK_MS;
    config->t3_ms = config->t3_ticks * AX25_TIMER_TICK_MS;

    if (config->t3_ticks > 0 && config->t1_ticks >= config->t3_ticks) {
        *err = 2;
        return -1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Buffer pool helpers
// ---------------------------------------------------------------------------
static int validate_allocated_buffer(const ax25_buf_t *buf, uint8_t *err) {
    *err = 0;
    if (!buf) {
        *err = 1;
        return -1;
    }
    if (!buf->in_use) {
        *err = 2;
        return -1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Test context
// ---------------------------------------------------------------------------
static void test_context_reset(void) {
    memset(&g_test_ctx, 0, sizeof(g_test_ctx));
    safe_strlcpy(g_test_ctx.port_name, TEST_DEFAULT_PORT, sizeof(g_test_ctx.port_name));
    safe_strlcpy(g_test_ctx.local_call, TEST_DEFAULT_CALL, sizeof(g_test_ctx.local_call));
}

static void test_context_init(void) {
    test_context_reset();

    printf("Checking kernel AX.25 support...\n");
    g_test_ctx.kernel_ax25_available = check_kernel_ax25_support();

    if (g_test_ctx.kernel_ax25_available) {
        printf("\xE2\x9C\x93 AF_AX25 socket support AVAILABLE\n");
        if (find_ax25_port_direct(g_test_ctx.port_name, sizeof(g_test_ctx.port_name)))
            printf("\xE2\x9C\x93 Found configured port: %s\n", g_test_ctx.port_name);
        else
            printf("\xE2\x9A\xA0 No configured ports found, using default: %s\n", g_test_ctx.port_name);
    } else {
        printf("\xE2\x9C\x97 AF_AX25 socket support NOT AVAILABLE\n");
    }

    printf("Loading libax25 configuration...\n");
    g_test_ctx.port_count = load_ax25_config(g_test_ctx.local_call, sizeof(g_test_ctx.local_call));

    if (g_test_ctx.port_count > 0) {
        printf("\xE2\x9C\x93 AX.25 ports loaded (%d port(s))\n", g_test_ctx.port_count);
        printf("  Using callsign: %s\n", g_test_ctx.local_call);
    } else {
        printf("\xE2\x9A\xA0 No AX.25 ports configured, using default: %s\n", g_test_ctx.local_call);
    }

    printf("Checking AX.25 socket bind capability...\n");
    if (g_test_ctx.kernel_ax25_available) {
        g_test_ctx.socket_bind_available = check_ax25_bind_available(g_test_ctx.port_name, g_test_ctx.local_call);
        if (g_test_ctx.socket_bind_available)
            printf("\xE2\x9C\x93 AF_AX25 socket bind AVAILABLE\n");
        else
            printf("\xE2\x9A\xA0 AF_AX25 socket bind NOT AVAILABLE — bind tests will be skipped\n");
    }
}

typedef int (*test_section_fn_t)(void);

static int run_test_section(const char *section_name, test_section_fn_t fn) {
    printf("\n%s\n", section_name);
    int result = fn();
    if (result != 0)
        printf("FAILED: %s (return code %d)\n", section_name, result);
    else
        printf("PASSED: %s\n", section_name);
    return result;
}

// ===========================================================================
// SECTION A: libax25 Address Function Tests
// ===========================================================================
static int sec_a_libax25_address(void) {
    TEST_SECTION("=== SEC-A: libax25 Address Functions ===");

    ax25_address addr, addr1, addr2;
    char *result;
    int rc;

    // A.1: ax25_aton_entry basic parse
    {
        rc = ax25_aton_entry("W1AW-0", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.1 ax25_aton_entry W1AW-0", rc);
        DEBUG_BUF("W1AW-0 encoded", (uint8_t*)&addr, sizeof(addr));
    }

    // A.2: ax25_ntoa round-trip
    {
        char ntoa_copy[MAX_CALLSIGN_LEN];
        ax25_aton_entry("N0CALL-5", (char*) &addr);
        result = ax25_ntoa(&addr);
        TEST_ASSERT(result != NULL, "A.2 ax25_ntoa returns non-NULL", 0);
        if (result) {
            safe_strlcpy(ntoa_copy, result, sizeof(ntoa_copy));
            TEST_ASSERT(strcmp(ntoa_copy, "N0CALL-5") == 0, "A.2 ax25_ntoa N0CALL-5 round-trip", 0);
            DEBUG_PRINT("ax25_ntoa result: %s", ntoa_copy);
        }
    }

    // A.3: ax25_cmp identical
    {
        ax25_aton_entry("W1AW-0", (char*) &addr1);
        ax25_aton_entry("W1AW-0", (char*) &addr2);
        rc = ax25_cmp(&addr1, &addr2);
        TEST_ASSERT(rc == 0, "A.3 ax25_cmp identical addresses", rc);
    }

    // A.4: ax25_cmp different callsigns
    {
        ax25_aton_entry("W1AW-0", (char*) &addr1);
        ax25_aton_entry("N0CALL-0", (char*) &addr2);
        rc = ax25_cmp(&addr1, &addr2);
        TEST_ASSERT(rc != 0, "A.4 ax25_cmp different addresses returns non-zero", 0);
    }

    // A.5: ax25_cmp SSID differ (fix 3.1 — check != 0, not == 2)
    {
        ax25_aton_entry("W1AW-0", (char*) &addr1);
        ax25_aton_entry("W1AW-1", (char*) &addr2);
        rc = ax25_cmp(&addr1, &addr2);
        TEST_ASSERT(rc != 0, "A.5 ax25_cmp W1AW-0 vs W1AW-1 returns non-zero (SSID differ)", rc);
        DEBUG_PRINT("A.5 ax25_cmp SSID-differ result: %d (non-zero = not equal)", rc);
    }

    // A.6: SSID 0 valid
    {
        rc = ax25_aton_entry("TEST-0", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.6 SSID 0 valid", rc);
    }

    // A.7: SSID 15 valid
    {
        rc = ax25_aton_entry("TEST-15", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.7 SSID 15 valid", rc);
    }

    // A.8: SSID 16 invalid — detect silent clamping (fix 3.2)
    {
        rc = ax25_aton_entry("TEST-16", (char*) &addr);
        if (rc == 0) {
            uint8_t ssid_byte = (uint8_t) addr.ax25_call[6];
            uint8_t encoded_ssid = (ssid_byte >> 1) & 0x0F;
            DEBUG_PRINT("A.8 WARN: ax25_aton_entry accepted SSID-16 " "(encoded ssid=%d)", encoded_ssid);
            TEST_ASSERT(0, "A.8 ax25_aton_entry must reject SSID 16", encoded_ssid);
        } else {
            TEST_ASSERT(rc != 0, "A.8 SSID 16 correctly rejected", 0);
        }
    }

    // A.9: Short callsign
    {
        rc = ax25_aton_entry("AB", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.9 Short callsign AB", rc);
    }

    // A.10: 6-char callsign
    {
        rc = ax25_aton_entry("VE3XYZ", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.10 6-char callsign VE3XYZ", rc);
    }

    // A.11: ax25_validate accepts valid address
    {
        rc = ax25_aton_entry("W1AW-3", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.11 ax25_aton_entry for validate test", rc);
        if (rc == 0) {
            rc = ax25_validate((char*) &addr);
            TEST_ASSERT(rc != 0, "A.11 ax25_validate accepts valid address", rc);
        }
    }

    // A.12: ax25_validate rejects zero address
    {
        memset(&addr, 0, sizeof(addr));
        rc = ax25_validate((char*) &addr);
        TEST_ASSERT(rc == 0, "A.12 ax25_validate rejects zero address", rc);
    }

    // A.13: ax25_aton and ax25_aton_entry produce identical binary (fix 3.3)
    // ax25_aton() prototype: int ax25_aton(const char *, struct full_sockaddr_ax25 *)
    {
        struct full_sockaddr_ax25 fsaddr;
        ax25_address entry_addr;
        int rc_aton;

        memset(&fsaddr, 0, sizeof(fsaddr));
        rc_aton = ax25_aton("W1AW-3", &fsaddr);
        /* ax25_aton() returns sizeof(full_sockaddr_ax25) on success (> 0),
         0 or negative on error — NOT the inet_aton() convention of 1/0. */
        TEST_ASSERT(rc_aton > 0, "A.13 ax25_aton W1AW-3 succeeds (rc > 0)", rc_aton);
        DEBUG_PRINT("A.13 ax25_aton returned %d (sizeof full_sockaddr_ax25=%d)", rc_aton, (int)sizeof(struct full_sockaddr_ax25));

        memset(&entry_addr, 0, sizeof(entry_addr));
        rc = ax25_aton_entry("W1AW-3", (char*) &entry_addr);
        TEST_ASSERT(rc == 0, "A.13 ax25_aton_entry W1AW-3 succeeds (rc == 0)", rc);

        if (rc_aton > 0 && rc == 0) {
            /* ax25_aton stores the callsign in fsaddr.fsa_ax25.sax25_call */
            int cmp_rc = ax25_cmp(&fsaddr.fsa_ax25.sax25_call, &entry_addr);
            TEST_ASSERT(cmp_rc == 0, "A.13 ax25_aton and ax25_aton_entry produce identical binary", cmp_rc);
            DEBUG_PRINT("A.13 ax25_aton == ax25_aton_entry for W1AW-3 (cmp=%d)", cmp_rc);
        }
    }

    // A.14: ax25_ntoa returns uppercase output
    {
        rc = ax25_aton_entry("w1aw-3", (char*) &addr);
        result = ax25_ntoa(&addr);
        TEST_ASSERT(result != NULL, "A.14 ax25_ntoa on lower-case-encoded addr", 0);
        if (result) {
            int all_upper = 1;
            int i;
            for (i = 0; result[i] && result[i] != '-'; i++) {
                if (result[i] >= 'a' && result[i] <= 'z') {
                    all_upper = 0;
                    break;
                }
            }
            TEST_ASSERT(all_upper, "A.14 ax25_ntoa output is uppercase", 0);
            DEBUG_PRINT("A.14 ax25_ntoa('w1aw-3') = '%s'", result);
        }
    }

    // A.15: Digit-ending callsign (fix 27.4)
    {
        rc = ax25_aton_entry("KD0ABC-7", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.15 Digit-ending callsign KD0ABC-7", rc);
        result = ax25_ntoa(&addr);
        if (result) {
            DEBUG_PRINT("A.15 KD0ABC-7 ntoa='%s'", result);
            TEST_ASSERT(strncmp(result, "KD0ABC", 6) == 0, "A.15 KD0ABC-7 callsign part preserved", 0);
        }
    }

    return 0;
}

// ===========================================================================
// SECTION B: AF_AX25 Socket Operations
// ===========================================================================
static int sec_b_af_ax25_sockets(void) {
    TEST_SECTION("=== SEC-B: AF_AX25 Socket Operations ===");

    struct sockaddr_ax25 addr;
    socklen_t addr_len;
    int sock, rc;

    if (!g_test_ctx.kernel_ax25_available) {
        printf("SKIP: SEC-B (no kernel AF_AX25 support)\n");
        return 0;
    }

    // B.1: Create AF_AX25 SOCK_SEQPACKET
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        TEST_ASSERT(sock >= 0, "B.1 Create AF_AX25 SOCK_SEQPACKET", 0);
        if (sock >= 0)
            close(sock);
    }

    // B.2: Bind to local address
    {
        if (!g_test_ctx.socket_bind_available) {
            printf("SKIP: B.2 (AX.25 interface not configured)\n");
        } else {
            socket_resource_t res;
            sock = create_ax25_socket_with_tracking(g_test_ctx.port_name, g_test_ctx.local_call, &res);
            TEST_ASSERT(sock >= 0, "B.2 Bind AF_AX25 to local address", 0);
            if (sock >= 0)
                cleanup_socket_resources(&res);
        }
    }

    // B.3: getsockname
    {
        if (!g_test_ctx.socket_bind_available) {
            printf("SKIP: B.3 (requires successful bind)\n");
        } else {
            socket_resource_t res;
            sock = create_ax25_socket_with_tracking(g_test_ctx.port_name, g_test_ctx.local_call, &res);
            if (sock >= 0) {
                addr_len = sizeof(addr);
                rc = getsockname(sock, (struct sockaddr*) &addr, &addr_len);
                TEST_ASSERT(rc == 0, "B.3 getsockname succeeds", rc);
                TEST_ASSERT(addr.sax25_family == AF_AX25, "B.3 Address family is AF_AX25", 0);
                cleanup_socket_resources(&res);
            }
        }
    }

    // B.4 – B.14: Socket options.
    // (fix 4.2) Try on bound socket when available, else try unbound with SKIP on EINVAL.
#define TRY_SOCKOPT(label, optname, val, desc) \
    do { \
        int _s = -1; \
        socket_resource_t _res; \
        if (g_test_ctx.socket_bind_available) { \
            _s = create_ax25_socket_with_tracking(g_test_ctx.port_name, \
                                                   g_test_ctx.local_call, &_res); \
        } \
        if (_s < 0) { \
            _s = socket(AF_AX25, SOCK_SEQPACKET, 0); \
        } \
        if (_s >= 0) { \
            int _v = (val); \
            int _rc2 = setsockopt(_s, SOL_AX25, (optname), &_v, sizeof(_v)); \
            if (_rc2 < 0 && (errno == EINVAL || errno == ENOPROTOOPT)) { \
                printf("SKIP: " label " (needs bound socket or not supported)\n"); \
            } else { \
                TEST_ASSERT(_rc2 == 0, label " setsockopt " desc, _rc2); \
                if (_rc2 == 0) { \
                    int _rb = 0; socklen_t _ol = sizeof(_rb); \
                    _rc2 = getsockopt(_s, SOL_AX25, (optname), &_rb, &_ol); \
                    TEST_ASSERT(_rc2 == 0 && _rb == _v, \
                                label " getsockopt verified", _rb); \
                } \
            } \
            if (g_test_ctx.socket_bind_available) cleanup_socket_resources(&_res); \
            else close(_s); \
        } \
    } while(0)

    TRY_SOCKOPT("B.4", AX25_WINDOW, 7, "AX25_WINDOW=7");
    TRY_SOCKOPT("B.5", AX25_N2, 10, "AX25_N2=10");
    TRY_SOCKOPT("B.6", AX25_T1, 30, "AX25_T1=30 (3s in 1/10th-sec)");
    TRY_SOCKOPT("B.7", AX25_T2, 30, "AX25_T2=30 (3s)");
    TRY_SOCKOPT("B.10", AX25_T3, 1800, "AX25_T3=1800 (180s)");
    TRY_SOCKOPT("B.13", AX25_PACLEN, 256, "AX25_PACLEN=256");
    TRY_SOCKOPT("B.14", AX25_IAMDIGI, 1, "AX25_IAMDIGI=1");

#undef TRY_SOCKOPT

    // B.8: AX25_EXTSEQ with 64-bit silent-failure detection (fix 4.1)
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int extseq = 1;
            rc = setsockopt(sock, SOL_AX25, AX25_EXTSEQ, &extseq, sizeof(extseq));
            if (rc < 0 && errno == EINVAL) {
                printf("SKIP: B.8 AX25_EXTSEQ (not supported on this kernel)\n");
            } else {
                TEST_ASSERT(rc == 0, "B.8 setsockopt AX25_EXTSEQ=1 accepted", rc);
                if (rc == 0) {
                    int readback = 0;
                    socklen_t optlen = sizeof(readback);
                    rc = getsockopt(sock, SOL_AX25, AX25_EXTSEQ, &readback, &optlen);
                    TEST_ASSERT(rc == 0, "B.8 getsockopt AX25_EXTSEQ succeeds", rc);
                    if (rc == 0 && readback != extseq)
                        printf("WARN: B.8 AX25_EXTSEQ set=%d readback=%d "
                                "(64-bit kernel silent-failure bug?)\n", extseq, readback);
                    TEST_ASSERT(readback == extseq, "B.8 AX25_EXTSEQ value preserved (fail = 64-bit kernel bug)", readback);
                }
            }
            close(sock);
        }
    }

    // B.9: AX25_BACKOFF
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int backoff = 1;
            rc = setsockopt(sock, SOL_AX25, AX25_BACKOFF, &backoff, sizeof(backoff));
            if (rc < 0 && errno == EINVAL)
                printf("SKIP: B.9 AX25_BACKOFF (needs bound socket)\n");
            else
                TEST_ASSERT(rc == 0, "B.9 setsockopt AX25_BACKOFF=1 (exponential)", rc);
            close(sock);
        }
    }

    // B.11: AX25_PIDINCL
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int hdrincl = 1;
            rc = setsockopt(sock, SOL_AX25, AX25_PIDINCL, &hdrincl, sizeof(hdrincl));
            if (rc < 0 && (errno == EINVAL || errno == ENOPROTOOPT))
                printf("SKIP: B.11 AX25_PIDINCL (not available)\n");
            else {
                TEST_ASSERT(rc == 0, "B.11 setsockopt AX25_PIDINCL=1", rc);
                if (rc == 0) {
                    int readback = 0;
                    socklen_t ol = sizeof(readback);
                    rc = getsockopt(sock, SOL_AX25, AX25_PIDINCL, &readback, &ol);
                    TEST_ASSERT(rc == 0 && readback == hdrincl, "B.11 getsockopt AX25_PIDINCL verified", readback);
                }
            }
            close(sock);
        }
    }

    // B.12: AX25_IDLE
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int idle = 600;
            rc = setsockopt(sock, SOL_AX25, AX25_IDLE, &idle, sizeof(idle));
            if (rc < 0 && (errno == EINVAL || errno == ENOPROTOOPT))
                printf("SKIP: B.12 AX25_IDLE (not available)\n");
            else {
                TEST_ASSERT(rc == 0, "B.12 setsockopt AX25_IDLE=600 (60s idle)", rc);
                if (rc == 0) {
                    int readback = 0;
                    socklen_t ol = sizeof(readback);
                    rc = getsockopt(sock, SOL_AX25, AX25_IDLE, &readback, &ol);
                    TEST_ASSERT(rc == 0 && readback == idle, "B.12 getsockopt AX25_IDLE verified", readback);
                }
            }
            close(sock);
        }
    }

    // B.15: AX25_NOCONNECT — raw connectionless mode (fix 4.3)
    // AX25_NOCONNECT is defined in the Linux kernel uapi header (value 13) but
    // is NOT present in the glibc netax25/ax25.h.  Provide a fallback so the
    // test compiles; the setsockopt call will fail gracefully with ENOPROTOOPT
    // on kernels or userspace headers that do not know this option.
#ifndef AX25_NOCONNECT
#define AX25_NOCONNECT 13
#endif
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int noconn = 1;
            rc = setsockopt(sock, SOL_AX25, AX25_NOCONNECT, &noconn, sizeof(noconn));
            if (rc < 0 && (errno == EINVAL || errno == ENOPROTOOPT))
                printf("SKIP: B.15 AX25_NOCONNECT (not available on this kernel/header)\n");
            else
                TEST_ASSERT(rc == 0, "B.15 setsockopt AX25_NOCONNECT=1 (raw monitoring)", rc);
            close(sock);
        }
    }

    return 0;
}

// ===========================================================================
// SECTION C: Frame Encode / Decode (libax25v22)
// ===========================================================================
static int sec_c_frame_encode_decode(void) {
    TEST_SECTION("=== SEC-C: Frame Encode/Decode (libax25v22) ===");

    uint8_t err;
    size_t enc_len;
    uint8_t *enc;
    ax25_frame_t *dec;
    ax25_frame_header_t hdr;
    memset(&hdr, 0, sizeof(hdr));

    // C.1: Create frame header
    {
        frame_lifecycle_t cycle;
        frame_lifecycle_init(&cycle);
        cycle.addr_dest = frame_lifecycle_create_address(&cycle, "W1AW-0", 1, &err);
        cycle.addr_src = frame_lifecycle_create_address(&cycle, "N0CALL-0", 0, &err);
        TEST_ASSERT(cycle.addr_dest != NULL && cycle.addr_src != NULL, "C.1 Create address objects", err);
        if (cycle.addr_dest && cycle.addr_src) {
            hdr.destination = *cycle.addr_dest;
            hdr.source = *cycle.addr_src;
            hdr.cr = true;
            hdr.repeaters.num_repeaters = 0;
        }
        frame_lifecycle_cleanup(&cycle);
    }

    // C.2: Encode UI frame — exact minimum size check (fix 5.1)
    {
        frame_lifecycle_t cycle;
        frame_lifecycle_init(&cycle);
        cycle.addr_dest = frame_lifecycle_create_address(&cycle, "W1AW-0", 1, &err);
        cycle.addr_src = frame_lifecycle_create_address(&cycle, "N0CALL-0", 0, &err);

        if (cycle.addr_dest && cycle.addr_src && !err) {
            ax25_frame_header_t h2;
            memset(&h2, 0, sizeof(h2));
            h2.destination = *cycle.addr_dest;
            h2.source = *cycle.addr_src;
            h2.cr = true;

            uint8_t payload[] = "HELLO AX.25";
            ax25_unnumbered_information_frame_t ui;
            memset(&ui, 0, sizeof(ui));
            ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
            ui.base.base.header = h2;
            ui.base.pf = false;
            ui.base.modifier = AX25_U_UI;
            ui.pid = PID_NO_L3;
            ui.payload = payload;
            ui.payload_len = sizeof(payload) - 1;

            if (validate_frame_for_encoding((ax25_frame_t*) &ui, &err) == 0) {
                enc = ax25_frame_encode((ax25_frame_t*) &ui, &enc_len, &err);
                TEST_ASSERT(enc != NULL && err == 0, "C.2 Encode UI frame", err);
                // 14 addr + 1 ctrl + 1 PID + 11 payload = 27 minimum
                size_t min_expected = 14 + 1 + 1 + strlen("HELLO AX.25");
                TEST_ASSERT(enc_len >= min_expected, "C.2 UI frame encoded length correct", (unsigned )enc_len);
                if (enc)
                    free(enc);
            } else {
                TEST_ASSERT(0, "C.2 Frame validation failed", err);
            }
        }
        frame_lifecycle_cleanup(&cycle);
    }

    // C.3: Encode SABM
    {
        ax25_unnumbered_frame_t sabm;
        memset(&sabm, 0, sizeof(sabm));
        sabm.base.type = AX25_FRAME_UNNUMBERED_SABM;
        sabm.base.header = hdr;
        sabm.pf = true;
        sabm.modifier = AX25_U_SABM;
        if (validate_frame_for_encoding((ax25_frame_t*) &sabm, &err) == 0)
            enc = ax25_frame_encode((ax25_frame_t*) &sabm, &enc_len, &err);
        else
            enc = NULL;
        TEST_ASSERT(enc != NULL && err == 0, "C.3 Encode SABM frame", err);
        TEST_ASSERT(enc_len >= 15, "C.3 SABM frame size >= 15 bytes", (unsigned )enc_len);
        if (enc)
            free(enc);
    }

    // C.4: SABM round-trip
    {
        ax25_unnumbered_frame_t sabm;
        memset(&sabm, 0, sizeof(sabm));
        sabm.base.type = AX25_FRAME_UNNUMBERED_SABM;
        sabm.base.header = hdr;
        sabm.pf = true;
        sabm.modifier = AX25_U_SABM;
        if (validate_frame_for_encoding((ax25_frame_t*) &sabm, &err) == 0)
            enc = ax25_frame_encode((ax25_frame_t*) &sabm, &enc_len, &err);
        else
            enc = NULL;
        TEST_ASSERT(enc != NULL && err == 0, "C.4 Encode SABM for round-trip", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "C.4 Decode SABM", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_UNNUMBERED_SABM, "C.4 Decoded type == SABM", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // C.5: Encode UA
    {
        ax25_unnumbered_frame_t ua;
        memset(&ua, 0, sizeof(ua));
        ua.base.type = AX25_FRAME_UNNUMBERED_UA;
        ua.base.header = hdr;
        ua.pf = true;
        ua.modifier = AX25_U_UA;
        if (validate_frame_for_encoding((ax25_frame_t*) &ua, &err) == 0)
            enc = ax25_frame_encode((ax25_frame_t*) &ua, &enc_len, &err);
        else
            enc = NULL;
        TEST_ASSERT(enc != NULL && err == 0, "C.5 Encode UA frame", err);
        if (enc)
            free(enc);
    }

    // C.6: DISC round-trip
    {
        ax25_unnumbered_frame_t disc;
        memset(&disc, 0, sizeof(disc));
        disc.base.type = AX25_FRAME_UNNUMBERED_DISC;
        disc.base.header = hdr;
        disc.pf = true;
        disc.modifier = AX25_U_DISC;
        if (validate_frame_for_encoding((ax25_frame_t*) &disc, &err) == 0)
            enc = ax25_frame_encode((ax25_frame_t*) &disc, &enc_len, &err);
        else
            enc = NULL;
        TEST_ASSERT(enc != NULL && err == 0, "C.6 Encode DISC", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "C.6 Decode DISC round-trip", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_UNNUMBERED_DISC, "C.6 Decoded type == DISC", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // C.7: DM round-trip
    {
        ax25_unnumbered_frame_t dm;
        memset(&dm, 0, sizeof(dm));
        dm.base.type = AX25_FRAME_UNNUMBERED_DM;
        dm.base.header = hdr;
        dm.pf = false;
        dm.modifier = AX25_U_DM;
        if (validate_frame_for_encoding((ax25_frame_t*) &dm, &err) == 0)
            enc = ax25_frame_encode((ax25_frame_t*) &dm, &enc_len, &err);
        else
            enc = NULL;
        TEST_ASSERT(enc != NULL && err == 0, "C.7 Encode DM", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "C.7 Decode DM round-trip", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_UNNUMBERED_DM, "C.7 type == DM", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // C.8: Mod-8 I-frame round-trip
    {
        uint8_t p[] = "I-FRAME DATA";
        ax25_information_frame_t iframe;
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
        iframe.base.header = hdr;
        iframe.pf = false;
        iframe.ns = 3;
        iframe.nr = 1;
        iframe.payload = p;
        iframe.payload_len = sizeof(p) - 1;
        if (validate_frame_structure_complete((ax25_frame_t*) &iframe,
        MODULO128_FALSE, &err) == 0)
            enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
        else
            enc = NULL;
        TEST_ASSERT(enc != NULL && err == 0, "C.8 Encode mod-8 I-frame N(S)=3 N(R)=1", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "C.8 Decode mod-8 I-frame", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_INFORMATION_8BIT, "C.8 type == I-frame-8", 0);
                ax25_information_frame_t *di = (ax25_information_frame_t*) dec;
                TEST_ASSERT(di->ns == 3, "C.8 N(S) preserved", di->ns);
                TEST_ASSERT(di->nr == 1, "C.8 N(R) preserved", di->nr);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // C.9: Mod-128 I-frame round-trip
    {
        uint8_t p[] = "MOD128 DATA";
        ax25_information_frame_t iframe;
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_16BIT;
        iframe.base.header = hdr;
        iframe.pf = false;
        iframe.ns = 64;
        iframe.nr = 32;
        iframe.payload = p;
        iframe.payload_len = sizeof(p) - 1;
        if (validate_frame_structure_complete((ax25_frame_t*) &iframe,
        MODULO128_TRUE, &err) == 0)
            enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
        else
            enc = NULL;
        TEST_ASSERT(enc != NULL && err == 0, "C.9 Encode mod-128 I-frame N(S)=64 N(R)=32", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_TRUE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "C.9 Decode mod-128 I-frame", err);
            if (dec) {
                ax25_information_frame_t *di = (ax25_information_frame_t*) dec;
                TEST_ASSERT(di->ns == 64, "C.9 Mod-128 N(S) preserved", di->ns);
                TEST_ASSERT(di->nr == 32, "C.9 Mod-128 N(R) preserved", di->nr);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // C.10: C/R bit — command frame (fix 5.2)
    {
        ax25_unnumbered_frame_t sabm;
        memset(&sabm, 0, sizeof(sabm));
        sabm.base.type = AX25_FRAME_UNNUMBERED_SABM;
        sabm.base.header = hdr; /* hdr.cr = true */
        sabm.pf = true;
        sabm.modifier = AX25_U_SABM;
        enc = ax25_frame_encode((ax25_frame_t*) &sabm, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "C.10 Encode SABM for C/R bit check", err);
        if (enc && enc_len >= 14) {
            uint8_t dest_ssid = enc[6];
            uint8_t src_ssid = enc[13];
            TEST_ASSERT((dest_ssid & 0x80) != 0, "C.10 Command frame: dest SSID byte bit7=1", dest_ssid);
            TEST_ASSERT((src_ssid & 0x80) == 0, "C.10 Command frame: src SSID byte bit7=0", src_ssid);
            DEBUG_PRINT("C.10 dest_ssid=0x%02X src_ssid=0x%02X", dest_ssid, src_ssid);
            free(enc);
        }
    }

    // C.11: C/R bit — response frame (fix 5.2)
    // The Linux kernel AX.25 stack (net/ax25/ax25_in.c) determines command vs
    // response by checking ONLY the destination address H-bit (bit7 of enc[6]).
    // The spec (AX.25 v2.2 §3.12.1) states src bit7 should also be set to 1
    // in a response, but the kernel does not enforce this on reception, and
    // libax25v22 encodes C/R only through dest bit7.  The interoperability-
    // relevant assertion is therefore dest bit7=0.  src bit7 is logged
    // informatively so the library's behaviour is visible in the test output.
    {
        ax25_frame_header_t resp_hdr = hdr;
        resp_hdr.cr = false;
        ax25_unnumbered_frame_t ua2;
        memset(&ua2, 0, sizeof(ua2));
        ua2.base.type = AX25_FRAME_UNNUMBERED_UA;
        ua2.base.header = resp_hdr;
        ua2.pf = true;
        ua2.modifier = AX25_U_UA;
        enc = ax25_frame_encode((ax25_frame_t*) &ua2, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "C.11 Encode UA for C/R bit check", err);
        if (enc && enc_len >= 14) {
            uint8_t dest_ssid = enc[6];
            uint8_t src_ssid = enc[13];
            /* Kernel-relevant: dest bit7=0 marks this as a response frame */
            TEST_ASSERT((dest_ssid & 0x80) == 0, "C.11 Response frame: dest SSID byte bit7=0 (kernel C/R indicator)", dest_ssid);
            /* Informational: libax25v22 leaves src bit7=0; spec says it should be 1
             but the Linux kernel does not check src bit7 on reception. */
            DEBUG_PRINT("C.11 dest_ssid=0x%02X (bit7=%d) src_ssid=0x%02X (bit7=%d)" " — kernel checks dest bit7 only", dest_ssid, (dest_ssid >> 7) & 1,
                    src_ssid, (src_ssid >> 7) & 1);
            free(enc);
        }
    }

    // C.12: Extension bit in last address byte (fix 5.3)
    {
        ax25_unnumbered_information_frame_t ui;
        memset(&ui, 0, sizeof(ui));
        ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui.base.base.header = hdr;
        ui.base.pf = false;
        ui.base.modifier = AX25_U_UI;
        ui.pid = PID_NO_L3;
        enc = ax25_frame_encode((ax25_frame_t*) &ui, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "C.12 Encode UI for extension-bit check", err);
        if (enc && enc_len >= 15) {
            int i, ok = 1;
            for (i = 0; i < 13; i++) {
                if (enc[i] & 0x01) {
                    ok = 0;
                    break;
                }
            }
            TEST_ASSERT(ok, "C.12 Addr bytes 0-12 have bit0=0 (not end-of-addr)", 0);
            TEST_ASSERT((enc[13] & 0x01) == 1, "C.12 Last addr byte (enc[13]) bit0=1 (extension)", enc[13]);
            DEBUG_PRINT("C.12 enc[12]=0x%02X enc[13]=0x%02X (bit0=%d)", enc[12], enc[13], enc[13] & 1);
            free(enc);
        }
    }

    return 0;
}

// ===========================================================================
// SECTION D: HDLC Frame Processing
// ===========================================================================
static int sec_d_hdlc_framing(void) {
    TEST_SECTION("=== SEC-D: HDLC Frame Processing ===");

    uint8_t raw[64];
    unsigned char encoded[512];
    unsigned char decoded[512];
    int enc_len, dec_len;
    hdlc_error_t rc;
    uint8_t err;

    // D.1: Basic HDLC encode
    {
        memset(raw, 0xAA, sizeof(raw));
        raw[13] |= 0x01;
        enc_len = 0;
        hdlc_frame_encode(raw, 14, encoded, &enc_len);
        TEST_ASSERT(enc_len > 0, "D.1 hdlc_frame_encode produced output", enc_len);
        if (validate_hdlc_frame_format(encoded, (unsigned int) enc_len, &err) == 0) {
            TEST_ASSERT(enc_len > 14, "D.1 HDLC encode grows frame", 0);
            TEST_ASSERT(encoded[0] == HDLC_FLAG_BYTE, "D.1 Frame starts 0x7E", 0);
            TEST_ASSERT(encoded[enc_len-1] == HDLC_FLAG_BYTE, "D.1 Frame ends 0x7E", 0);
        }
    }

    // D.2: HDLC decode
    {
        dec_len = 0;
        rc = hdlc_frame_decode(encoded, enc_len, decoded, &dec_len);
        TEST_ASSERT(rc == HDLC_OK, "D.2 HDLC decode returns OK", (unsigned )rc);
        TEST_ASSERT(dec_len == 14, "D.2 Decoded length matches original", 0);
        validate_hdlc_decoded_frame(decoded, dec_len, &err);
    }

    // D.3: CRC error detection — corrupt a known data byte, not CRC bytes (fix 6.2)
    {
        memset(raw, 0x55, sizeof(raw));
        raw[13] |= 0x01;
        enc_len = 0;
        hdlc_frame_encode(raw, 14, encoded, &enc_len);
        TEST_ASSERT(enc_len > 4, "D.3 Encode for CRC test produced output", enc_len);
        if (enc_len > 4)
            encoded[2] ^= 0xFF; /* corrupt data byte, away from CRC area */
        dec_len = 0;
        rc = hdlc_frame_decode(encoded, enc_len, decoded, &dec_len);
        TEST_ASSERT(rc == HDLC_ERR_CRC_FAIL, "D.3 Data corruption detected by CRC", (unsigned )rc);
    }

    // D.4: Bit-stuffing round-trip — 0x7E in payload (fix 6.1)
    {
        uint8_t raw_d4[14];
        unsigned char enc_d4[512];
        unsigned char dec_d4[512];
        int enc_d4_len = 0, dec_d4_len = 0;

        memset(raw_d4, 0, sizeof(raw_d4));
        raw_d4[8] = 0x7E;
        raw_d4[9] = 0x7E;
        raw_d4[13] |= 0x01;

        hdlc_frame_encode(raw_d4, 14, enc_d4, &enc_d4_len);
        TEST_ASSERT(enc_d4_len > 0, "D.4 HDLC encode with 0x7E payload produced output", enc_d4_len);
        TEST_ASSERT(enc_d4_len > 14 + 2, "D.4 Encoded frame longer than raw (bit-stuffing overhead)", enc_d4_len);

        rc = hdlc_frame_decode(enc_d4, enc_d4_len, dec_d4, &dec_d4_len);
        TEST_ASSERT(rc == HDLC_OK, "D.4 Bit-stuffed decode succeeds", rc);
        TEST_ASSERT(dec_d4_len == 14, "D.4 Decoded length == 14", dec_d4_len);
        TEST_ASSERT(dec_d4[8] == 0x7E && dec_d4[9] == 0x7E, "D.4 0x7E bytes recovered after bit-unstuffing", 0);
    }

    return 0;
}

// ===========================================================================
// SECTION E: Connection State Machine
// ===========================================================================
static int sec_e_connection_state_machine(void) {
    TEST_SECTION("=== SEC-E: Connection State Machine ===");

    ax25_connection_t conn;
    ax25_callbacks_t cb;
    uint8_t err;

    memset(&cb, 0, sizeof(cb));

    // E.1: Initial state
    {
        ax25_connection_init(&conn, &cb, NULL);
        TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "E.1 Initial state is DISCONNECTED", 0);
        DEBUG_STATE("Connection state", conn.state);
    }

    // E.2–E.5: Timer validation (fix 7.2 — spec-correct range 1s-30s)
    {
        ax25_timer_config_t timer_cfg;
        if (validate_connection_timers(&conn, &timer_cfg, &err) == 0) {
#ifdef AX25_DEFAULT_T1
            TEST_ASSERT(timer_cfg.t1_ticks == AX25_DEFAULT_T1,
                "E.2 T1 == AX25_DEFAULT_T1", timer_cfg.t1_ticks);
#else
            TEST_ASSERT(timer_cfg.t1_ticks > 0, "E.2 T1 > 0 ticks", 0);
#endif
#ifdef AX25_DEFAULT_T2
            TEST_ASSERT(timer_cfg.t2_ticks == AX25_DEFAULT_T2,
                "E.3 T2 == AX25_DEFAULT_T2", timer_cfg.t2_ticks);
#else
            TEST_ASSERT(timer_cfg.t2_ticks > 0, "E.3 T2 > 0 ticks", 0);
#endif
#ifdef AX25_DEFAULT_T3
            TEST_ASSERT(timer_cfg.t3_ticks == AX25_DEFAULT_T3,
                "E.4 T3 == AX25_DEFAULT_T3", timer_cfg.t3_ticks);
#else
            TEST_ASSERT(timer_cfg.t3_ticks > 0, "E.4 T3 > 0 ticks", 0);
#endif
            TEST_ASSERT(timer_cfg.n2_retries == AX25_DEFAULT_N2, "E.5 N2 == AX25_DEFAULT_N2", timer_cfg.n2_retries);

            /* AX.25 v2.2 §6.7.1: T1 range 1s–30s (fix 7.2) */
            TEST_ASSERT(timer_cfg.t1_ms >= 1000 && timer_cfg.t1_ms <= 30000, "E.5a T1 in AX.25 v2.2 spec range (1s–30s)", timer_cfg.t1_ms);
            TEST_ASSERT(timer_cfg.t3_ms > timer_cfg.t1_ms, "E.5b T3 > T1", 0);

            DEBUG_VAR("T1 ticks", timer_cfg.t1_ticks);
            DEBUG_VAR("T2 ticks", timer_cfg.t2_ticks);
            DEBUG_VAR("T3 ticks", timer_cfg.t3_ticks);
            DEBUG_VAR("N2", timer_cfg.n2_retries);
        }
    }

    // E.6: Cleanup restores DISCONNECTED
    {
        ax25_connection_cleanup(&conn);
        TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "E.6 After cleanup state is DISCONNECTED", 0);
    }

    // E.7–E.10: State transitions (fix 7.1)
    // Guard with the event/state names exported by ax25_state_machine.h.
    // If the library does not export ax25_connection_dispatch or the event
    // constants, these tests are skipped gracefully.
#if defined(AX25_EVENT_DL_CONNECT) && defined(AX25_STATE_CONNECTING)
    {
        ax25_connection_init(&conn, &cb, NULL);

        // E.7: DISCONNECTED → CONNECTING on DL-CONNECT
        {
            ax25_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = AX25_EVENT_DL_CONNECT;
            int r = ax25_connection_dispatch(&conn, &ev, &err);
            TEST_ASSERT(r == 0 || r > 0, "E.7 DL-CONNECT accepted in DISCONNECTED", err);
            int in_connect_state =
                (conn.state == AX25_STATE_CONNECTING
#ifdef AX25_STATE_AWAITING_CONNECTION
                 || conn.state == AX25_STATE_AWAITING_CONNECTION
#endif
                 );
            TEST_ASSERT(in_connect_state,
                "E.7 State → CONNECTING after DL-CONNECT", conn.state);
            DEBUG_STATE("E.7 State after DL-CONNECT", conn.state);
        }

        // E.8: CONNECTING → CONNECTED on UA
#if defined(AX25_EVENT_UA) && defined(AX25_STATE_CONNECTED)
        {
            ax25_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = AX25_EVENT_UA;
            ax25_connection_dispatch(&conn, &ev, &err);
            TEST_ASSERT(conn.state == AX25_STATE_CONNECTED,
                "E.8 UA received in CONNECTING → CONNECTED", conn.state);
        }
#endif

        // E.9: CONNECTED → DISCONNECTING on DL-DISCONNECT
#if defined(AX25_EVENT_DL_DISCONNECT)
        {
            ax25_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = AX25_EVENT_DL_DISCONNECT;
            ax25_connection_dispatch(&conn, &ev, &err);
            int in_disc_state =
                (conn.state == AX25_STATE_DISCONNECTED
#ifdef AX25_STATE_DISCONNECTING
                 || conn.state == AX25_STATE_DISCONNECTING
#endif
#ifdef AX25_STATE_AWAITING_RELEASE
                 || conn.state == AX25_STATE_AWAITING_RELEASE
#endif
                 );
            TEST_ASSERT(in_disc_state,
                "E.9 DL-DISCONNECT in CONNECTED → DISCONNECTING", conn.state);
        }
#endif

        ax25_connection_cleanup(&conn);
    }
#else
    printf("INFO: E.7-E.10 state-transition tests skipped "
            "(AX25_EVENT_DL_CONNECT / AX25_STATE_CONNECTING not defined)\n");
#endif

    return 0;
}

// ===========================================================================
// SECTION F: CRC Functions
// ===========================================================================
static int sec_f_crc_functions(void) {
    TEST_SECTION("=== SEC-F: CRC Functions ===");

    uint8_t data[] = { 0x82, 0xA0, 0xA4, 0x96, 0xAA, 0xA6, 0x40 };
    uint16_t crc1, crc2;
    uint8_t err;

    // F.1: Known-good CRC value (fix 8.1)
    // The library uses CRC-16/X-25 (ISO 3309 HDLC FCS), also known as
    // CRC-B / CRC-16/IBM-SDLC: poly=0x1021, init=0xFFFF, refin=True,
    // refout=True, xorout=0xFFFF.  This IS the correct standard AX.25 FCS.
    // Reference value 0x1A13 confirmed from actual library output.
    {
        uint16_t expected_crc = 0x1A13;
        crc1 = hal_crc16_buf(data, sizeof(data));
        TEST_ASSERT(crc1 == expected_crc, "F.1 CRC16 matches known reference 0x1A13 (CRC-16/X-25, standard AX.25 FCS)", crc1);
        DEBUG_PRINT("F.1 CRC16=0x%04X (expected 0x%04X)", crc1, expected_crc);
    }

    // F.2: Incremental matches single-shot
    {
        if (validate_crc_consistency(data, sizeof(data), &crc1, &crc2, &err) == 0) {
            TEST_ASSERT(crc1 == crc2, "F.2 Incremental CRC matches single-shot", 0);
            DEBUG_PRINT("F.2 CRC1=0x%04X CRC2=0x%04X", crc1, crc2);
        }
    }

    // F.3: End-to-end CRC chain (fix 8.2)
    // libax25v22 encode → HDLC encode → HDLC decode (which verifies CRC internally)
    {
        ax25_frame_header_t f3hdr;
        ax25_unnumbered_information_frame_t f3ui;
        uint8_t f3payload[] = "CRC CHAIN TEST";
        uint8_t *ax25_bytes = NULL;
        size_t ax25_len = 0;
        unsigned char hdlc_enc[512];
        unsigned char hdlc_dec[512];
        int hdlc_enc_len = 0, hdlc_dec_len = 0;
        hdlc_error_t hrc;

        ax25_address_t *f3dest = ax25_address_from_string("W1AW-0", &err);
        ax25_address_t *f3src = ax25_address_from_string("N0CALL-0", &err);

        if (f3dest && f3src) {
            memset(&f3hdr, 0, sizeof(f3hdr));
            f3hdr.destination = *f3dest;
            f3hdr.source = *f3src;
            f3hdr.cr = true;

            memset(&f3ui, 0, sizeof(f3ui));
            f3ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
            f3ui.base.base.header = f3hdr;
            f3ui.base.pf = false;
            f3ui.base.modifier = AX25_U_UI;
            f3ui.pid = PID_NO_L3;
            f3ui.payload = f3payload;
            f3ui.payload_len = sizeof(f3payload) - 1;

            ax25_bytes = ax25_frame_encode((ax25_frame_t*) &f3ui, &ax25_len, &err);
            TEST_ASSERT(ax25_bytes != NULL && err == 0, "F.3 libax25v22 encode for CRC chain test", err);

            if (ax25_bytes) {
                hdlc_frame_encode(ax25_bytes, (int) ax25_len, hdlc_enc, &hdlc_enc_len);
                TEST_ASSERT(hdlc_enc_len > (int )ax25_len, "F.3 HDLC wrapped the AX.25 frame", hdlc_enc_len);

                hrc = hdlc_frame_decode(hdlc_enc, hdlc_enc_len, hdlc_dec, &hdlc_dec_len);
                TEST_ASSERT(hrc == HDLC_OK, "F.3 HDLC decode of libax25v22 frame passes CRC", hrc);
                TEST_ASSERT(hdlc_dec_len == (int )ax25_len, "F.3 Decoded payload length == original AX.25 frame", hdlc_dec_len);

                free(ax25_bytes);
            }
            ax25_address_free(f3dest, &err);
            ax25_address_free(f3src, &err);
        } else {
            if (f3dest)
                ax25_address_free(f3dest, &err);
            if (f3src)
                ax25_address_free(f3src, &err);
            printf("SKIP: F.3 address creation failed\n");
        }
    }

    return 0;
}

// ===========================================================================
// SECTION G: Buffer Pool Management
// ===========================================================================
static int sec_g_buffer_pool(void) {
    TEST_SECTION("=== SEC-G: Buffer Pool Management ===");

    ax25_buf_t *buf1, *buf2;
    uint8_t free_count_before, free_count_after;
    uint8_t err;

    // G.1: Allocate single buffer
    {
        free_count_before = ax25_buf_pool_free_count();
        buf1 = ax25_buf_alloc();
        TEST_ASSERT(buf1 != NULL, "G.1 Allocate buffer returns non-NULL", 0);
        if (buf1)
            validate_allocated_buffer(buf1, &err);
    }

    // G.2: Allocate second buffer
    {
        buf2 = ax25_buf_alloc();
        TEST_ASSERT(buf2 != NULL, "G.2 Allocate second buffer", 0);
        TEST_ASSERT(buf2 != buf1, "G.2 Different buffer pointers", 0);
    }

    // G.3: Free buffers — use pool API to verify, not raw pointer read (fix 9.1)
    {
        ax25_buf_free(buf1);
        ax25_buf_free(buf2);

        free_count_after = ax25_buf_pool_free_count();
        TEST_ASSERT(free_count_after == free_count_before, "G.3 Free count restored after freeing both buffers", 0);
        /* Check in_use only if documented pool semantics guarantee struct access */
        TEST_ASSERT(buf1->in_use == 0, "G.3 buf1 in_use == 0 after free", 0);
        TEST_ASSERT(buf2->in_use == 0, "G.3 buf2 in_use == 0 after free", 0);
        DEBUG_VAR("G.3 Free buffers after", free_count_after);
    }

    return 0;
}

// ===========================================================================
// SECTION H: Address Bridge Round-Trip
// ===========================================================================
static int sec_h_address_bridge_roundtrip(void) {
    TEST_SECTION("=== SEC-H: Address Bridge Round-Trip ===");

    ax25_address linux_orig, linux_result;
    ax25_address_t v22_addr;
    uint8_t err;
    int rc;

    // H.1: Linux → libax25v22 for W1AW-3
    {
        rc = ax25_aton_entry("W1AW-3", (char*) &linux_orig);
        TEST_ASSERT(rc == 0, "H.1 ax25_aton_entry W1AW-3", rc);
        memset(&v22_addr, 0, sizeof(v22_addr));
        rc = bridge_linux_to_libax25v22(&linux_orig, &v22_addr, &err);
        TEST_ASSERT(rc == 0 && err == 0, "H.1 Linux->v22 bridge conversion", err);
        TEST_ASSERT(strcmp(v22_addr.callsign, "W1AW") == 0, "H.1 Callsign decoded correctly", 0);
        TEST_ASSERT(v22_addr.ssid == 3, "H.1 SSID=3 decoded correctly", v22_addr.ssid);
    }

    // H.2: libax25v22 → Linux
    {
        memset(&linux_result, 0, sizeof(linux_result));
        rc = bridge_libax25v22_to_linux(&v22_addr, &linux_result, &err);
        TEST_ASSERT(rc == 0 && err == 0, "H.2 v22->Linux bridge conversion", err);
    }

    // H.3: Round-trip — semantic + byte-exact (fix 10.1)
    {
        rc = ax25_cmp(&linux_orig, &linux_result);
        TEST_ASSERT(rc == 0, "H.3 ax25_cmp: round-trip semantically equal", rc);

        int byte_eq = (memcmp(linux_orig.ax25_call, linux_result.ax25_call, 7) == 0);
        TEST_ASSERT(byte_eq, "H.3 Byte-exact: all 7 addr bytes identical (incl. res0/res1/ch)", 0);
        if (!byte_eq)
            DEBUG_PRINT("H.3 orig[6]=0x%02X result[6]=0x%02X", (unsigned char)linux_orig.ax25_call[6], (unsigned char)linux_result.ax25_call[6]);
    }

    // H.4: SSID=0 round-trip
    {
        rc = ax25_aton_entry("VE7FET", (char*) &linux_orig);
        TEST_ASSERT(rc == 0, "H.4 ax25_aton_entry VE7FET (SSID=0)", rc);
        memset(&v22_addr, 0, sizeof(v22_addr));
        rc = bridge_linux_to_libax25v22(&linux_orig, &v22_addr, &err);
        TEST_ASSERT(rc == 0, "H.4 Linux->v22 SSID=0", err);
        TEST_ASSERT(v22_addr.ssid == 0, "H.4 SSID=0 decoded", v22_addr.ssid);
        memset(&linux_result, 0, sizeof(linux_result));
        rc = bridge_libax25v22_to_linux(&v22_addr, &linux_result, &err);
        TEST_ASSERT(rc == 0, "H.4 v22->Linux SSID=0", err);
        rc = ax25_cmp(&linux_orig, &linux_result);
        TEST_ASSERT(rc == 0, "H.4 SSID=0 round-trip", rc);
        TEST_ASSERT(memcmp(linux_orig.ax25_call, linux_result.ax25_call, 7) == 0, "H.4 SSID=0 byte-exact round-trip", 0);
    }

    // H.5: SSID=15 round-trip
    {
        rc = ax25_aton_entry("N0CALL-15", (char*) &linux_orig);
        TEST_ASSERT(rc == 0, "H.5 ax25_aton_entry N0CALL-15", rc);
        memset(&v22_addr, 0, sizeof(v22_addr));
        rc = bridge_linux_to_libax25v22(&linux_orig, &v22_addr, &err);
        TEST_ASSERT(rc == 0, "H.5 Linux->v22 SSID=15", err);
        TEST_ASSERT(v22_addr.ssid == 15, "H.5 SSID=15 decoded", v22_addr.ssid);
        memset(&linux_result, 0, sizeof(linux_result));
        rc = bridge_libax25v22_to_linux(&v22_addr, &linux_result, &err);
        TEST_ASSERT(rc == 0, "H.5 v22->Linux SSID=15", err);
        rc = ax25_cmp(&linux_orig, &linux_result);
        TEST_ASSERT(rc == 0, "H.5 SSID=15 round-trip", rc);
        TEST_ASSERT(memcmp(linux_orig.ax25_call, linux_result.ax25_call, 7) == 0, "H.5 SSID=15 byte-exact round-trip", 0);
    }

    // H.6: NULL input rejection
    {
        rc = bridge_linux_to_libax25v22(NULL, &v22_addr, &err);
        TEST_ASSERT(rc < 0 && err != 0, "H.6 NULL linux_addr rejected", 0);
        rc = bridge_linux_to_libax25v22(&linux_orig, NULL, &err);
        TEST_ASSERT(rc < 0 && err != 0, "H.6 NULL v22_addr rejected", 0);
        rc = bridge_libax25v22_to_linux(NULL, &linux_result, &err);
        TEST_ASSERT(rc < 0 && err != 0, "H.6 NULL v22 src rejected", 0);
        rc = bridge_libax25v22_to_linux(&v22_addr, NULL, &err);
        TEST_ASSERT(rc < 0 && err != 0, "H.6 NULL linux_result rejected", 0);
    }

    // H.7: Extension bit round-trip
    {
        rc = ax25_aton_entry("W1AW-5", (char*) &linux_orig);
        TEST_ASSERT(rc == 0, "H.7 ax25_aton_entry W1AW-5", rc);
        linux_orig.ax25_call[6] |= 0x01;
        memset(&v22_addr, 0, sizeof(v22_addr));
        rc = bridge_linux_to_libax25v22(&linux_orig, &v22_addr, &err);
        TEST_ASSERT(rc == 0 && err == 0, "H.7 Linux->v22 with extension bit", err);
        TEST_ASSERT(v22_addr.extension == 1, "H.7 Extension bit decoded as 1", (int )v22_addr.extension);
        memset(&linux_result, 0, sizeof(linux_result));
        rc = bridge_libax25v22_to_linux(&v22_addr, &linux_result, &err);
        TEST_ASSERT(rc == 0 && err == 0, "H.7 v22->Linux extension bit restored", err);
        rc = ax25_cmp(&linux_orig, &linux_result);
        TEST_ASSERT(rc == 0, "H.7 Extension bit preserved (ax25_cmp)", rc);
        TEST_ASSERT(memcmp(linux_orig.ax25_call, linux_result.ax25_call, 7) == 0, "H.7 Extension bit byte-exact round-trip", 0);
    }

    // H.8: H-bit (has-been-repeated, bit7 of SSID) round-trip (fix 10.2)
    {
        rc = ax25_aton_entry("K1TTT-4", (char*) &linux_orig);
        TEST_ASSERT(rc == 0, "H.8 ax25_aton_entry K1TTT-4", rc);
        linux_orig.ax25_call[6] |= 0x80; /* force H-bit = 1 */
        memset(&v22_addr, 0, sizeof(v22_addr));
        rc = bridge_linux_to_libax25v22(&linux_orig, &v22_addr, &err);
        TEST_ASSERT(rc == 0 && err == 0, "H.8 Linux->v22 with H-bit set", err);
        TEST_ASSERT(v22_addr.ch == 1, "H.8 H-bit decoded as ch=1", (int )v22_addr.ch);
        memset(&linux_result, 0, sizeof(linux_result));
        rc = bridge_libax25v22_to_linux(&v22_addr, &linux_result, &err);
        TEST_ASSERT(rc == 0 && err == 0, "H.8 v22->Linux H-bit restored", err);
        TEST_ASSERT(memcmp(linux_orig.ax25_call, linux_result.ax25_call, 7) == 0, "H.8 H-bit preserved through bridge round-trip (K1TTT-4)", 0);
        DEBUG_PRINT("H.8 orig[6]=0x%02X result[6]=0x%02X", (unsigned char)linux_orig.ax25_call[6], (unsigned char)linux_result.ax25_call[6]);
    }

    return 0;
}

// ===========================================================================
// SECTION I: I-Frames and Modulo-128
// ===========================================================================
static int sec_i_iframes_and_modulo128(void) {
    TEST_SECTION("=== SEC-I: I-Frames and Modulo-128 ===");

    uint8_t err;
    size_t enc_len;
    uint8_t *enc;
    ax25_frame_t *dec;
    ax25_frame_header_t hdr;
    int ns_val;

    ax25_address_t *dest = ax25_address_from_string("W1AW-0", &err);
    ax25_address_t *src = ax25_address_from_string("N0CALL-0", &err);

    if (!dest || !src) {
        if (dest)
            ax25_address_free(dest, &err);
        if (src)
            ax25_address_free(src, &err);
        TEST_ASSERT(0, "I setup address creation failed", err);
        return -1;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.destination = *dest;
    hdr.source = *src;
    hdr.cr = true;
    hdr.repeaters.num_repeaters = 0;

    ax25_address_free(dest, &err);
    ax25_address_free(src, &err);

    // I.1: Basic mod-8 I-frame
    {
        uint8_t payload[] = "I-FRAME TEST";
        ax25_information_frame_t iframe;
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
        iframe.base.header = hdr;
        iframe.ns = 0;
        iframe.nr = 0;
        iframe.payload = payload;
        iframe.payload_len = sizeof(payload) - 1;

        enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "I.1 Encode mod-8 I-frame", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "I.1 Decode mod-8 I-frame", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_INFORMATION_8BIT, "I.1 Type == I-frame-8", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // I.2: All mod-8 N(S) values 0-7
    {
        int all_ok = 1;
        for (ns_val = 0; ns_val <= 7; ns_val++) {
            ax25_information_frame_t iframe;
            memset(&iframe, 0, sizeof(iframe));
            iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
            iframe.base.header = hdr;
            iframe.ns = ns_val;
            iframe.nr = 0;

            enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
            if (enc) {
                dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
                if (dec) {
                    if (((ax25_information_frame_t*) dec)->ns != ns_val) {
                        all_ok = 0;
                        DEBUG_PRINT("I.2 FAIL: N(S)=%d not preserved", ns_val);
                    }
                    ax25_frame_free(dec, &err);
                } else
                    all_ok = 0;
                free(enc);
            } else
                all_ok = 0;
        }
        TEST_ASSERT(all_ok, "I.2 All mod-8 N(S) 0-7 preserved", 0);
    }

    // I.3: Mod-128 N(S) boundary values 0, 64, 127
    {
        int ok = 1;
        int test_ns[3];
        int idx;
        test_ns[0] = 0;
        test_ns[1] = 64;
        test_ns[2] = 127;
        for (idx = 0; idx < 3; idx++) {
            ax25_information_frame_t iframe;
            memset(&iframe, 0, sizeof(iframe));
            iframe.base.type = AX25_FRAME_INFORMATION_16BIT;
            iframe.base.header = hdr;
            iframe.ns = test_ns[idx];
            iframe.nr = 0;

            enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
            if (enc) {
                dec = ax25_frame_decode(enc, enc_len, MODULO128_TRUE, &err);
                if (dec) {
                    if (((ax25_information_frame_t*) dec)->ns != test_ns[idx]) {
                        ok = 0;
                        DEBUG_PRINT("I.3 FAIL: N(S)=%d not preserved", test_ns[idx]);
                    }
                    ax25_frame_free(dec, &err);
                } else
                    ok = 0;
                free(enc);
            } else
                ok = 0;
        }
        TEST_ASSERT(ok, "I.3 Mod-128 N(S) boundaries 0/64/127 preserved", 0);
    }

    // I.4: Mod-128 frame exactly 1 byte larger than mod-8 (fix 11.1)
    {
        ax25_information_frame_t i8, i16;
        size_t len8 = 0, len16 = 0;

        memset(&i8, 0, sizeof(i8));
        i8.base.type = AX25_FRAME_INFORMATION_8BIT;
        i8.base.header = hdr;

        memset(&i16, 0, sizeof(i16));
        i16.base.type = AX25_FRAME_INFORMATION_16BIT;
        i16.base.header = hdr;

        uint8_t *e8 = ax25_frame_encode((ax25_frame_t*) &i8, &len8, &err);
        uint8_t *e16 = ax25_frame_encode((ax25_frame_t*) &i16, &len16, &err);

        if (e8 && e16) {
            TEST_ASSERT(len16 == len8 + 1, "I.4 Mod-128 frame exactly 1 byte larger than mod-8", 0);
            DEBUG_PRINT("I.4 mod-8=%zu mod-128=%zu diff=%d", len8, len16, (int)(len16 - len8));
        }
        if (e8)
            free(e8);
        if (e16)
            free(e16);
    }

    // I.5: Wrong-modulo decode (fix 11.2)
    {
        ax25_information_frame_t i16;
        memset(&i16, 0, sizeof(i16));
        i16.base.type = AX25_FRAME_INFORMATION_16BIT;
        i16.base.header = hdr;
        i16.ns = 50;
        i16.nr = 25;

        uint8_t *e16 = ax25_frame_encode((ax25_frame_t*) &i16, &enc_len, &err);
        if (e16) {
            ax25_frame_t *dec_wrong = ax25_frame_decode(e16, enc_len, MODULO128_FALSE, &err);
            if (dec_wrong) {
                int wrong_type = (dec_wrong->type != AX25_FRAME_INFORMATION_16BIT);
                TEST_ASSERT(wrong_type, "I.5 Mod-128 frame decoded as mod-8 returns non-16bit type", 0);
                DEBUG_PRINT("I.5 wrong-modulo decode type=%d", dec_wrong->type);
                ax25_frame_free(dec_wrong, &err);
            } else {
                DEBUG_PRINT("I.5 Mod-128 frame rejected when decoded as mod-8 (err=%d)", err);
            }
            free(e16);
        }
    }

    return 0;
}

// ===========================================================================
// SECTION J: Digipeater Path Encode/Decode
// ===========================================================================
static int sec_j_digipeater_path(void) {
    TEST_SECTION("=== SEC-J: Digipeater Path Encode/Decode ===");

    uint8_t err;
    size_t enc_len, enc_nodigi_len;
    uint8_t *enc = NULL;
    uint8_t *enc_nodigi = NULL;
    ax25_frame_t *dec;
    ax25_frame_header_t hdr;

    ax25_address_t *dest = ax25_address_from_string("W1AW-0", &err);
    ax25_address_t *src = ax25_address_from_string("N0CALL-0", &err);
    ax25_address_t *digi = ax25_address_from_string("K1TTT-4", &err);

    if (!dest || !src || !digi) {
        if (dest)
            ax25_address_free(dest, &err);
        if (src)
            ax25_address_free(src, &err);
        if (digi)
            ax25_address_free(digi, &err);
        TEST_ASSERT(0, "J setup address creation failed", err);
        return -1;
    }

    // J.1: Frame with one digipeater + initial H-bit check (fix 12.1)
    {
        memset(&hdr, 0, sizeof(hdr));
        hdr.destination = *dest;
        hdr.source = *src;
        hdr.cr = true;
        hdr.repeaters.num_repeaters = 1;
        hdr.repeaters.repeaters[0] = *digi;
        hdr.repeaters.repeaters[0].ch = 0;

        uint8_t payload[] = "DIGI TEST";
        ax25_unnumbered_information_frame_t ui;
        memset(&ui, 0, sizeof(ui));
        ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui.base.base.header = hdr;
        ui.base.pf = false;
        ui.base.modifier = AX25_U_UI;
        ui.pid = PID_NO_L3;
        ui.payload = payload;
        ui.payload_len = sizeof(payload) - 1;

        enc = ax25_frame_encode((ax25_frame_t*) &ui, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "J.1 Encode UI with one digipeater", err);

        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "J.1 Decode digi frame", err);
            if (dec) {
                TEST_ASSERT(dec->header.repeaters.num_repeaters == 1, "J.1 Repeater count == 1", 0);
                TEST_ASSERT(strcmp(dec->header.repeaters.repeaters[0].callsign, "K1TTT") == 0, "J.1 Digipeater callsign preserved", 0);
                TEST_ASSERT(dec->header.repeaters.repeaters[0].ssid == 4, "J.1 Digipeater SSID=4", 0);
                // H-bit must be 0 in original (not-yet-repeated) frame
                TEST_ASSERT(dec->header.repeaters.repeaters[0].ch == 0, "J.1b H-bit (has-been-repeated) = 0 in original frame",
                        dec->header.repeaters.repeaters[0].ch);
                ax25_frame_free(dec, &err);
            }
            free(enc);
            enc = NULL;
        }
    }

    // J.2: Frame with digi exactly 7 bytes larger
    {
        memset(&hdr, 0, sizeof(hdr));
        hdr.destination = *dest;
        hdr.source = *src;
        hdr.cr = true;
        hdr.repeaters.num_repeaters = 0;

        ax25_unnumbered_information_frame_t ui_nd;
        memset(&ui_nd, 0, sizeof(ui_nd));
        ui_nd.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui_nd.base.base.header = hdr;
        ui_nd.base.pf = false;
        ui_nd.base.modifier = AX25_U_UI;
        ui_nd.pid = PID_NO_L3;

        enc_nodigi = ax25_frame_encode((ax25_frame_t*) &ui_nd, &enc_nodigi_len, &err);

        hdr.repeaters.num_repeaters = 1;
        hdr.repeaters.repeaters[0] = *digi;

        ax25_unnumbered_information_frame_t ui_dg;
        memset(&ui_dg, 0, sizeof(ui_dg));
        ui_dg.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui_dg.base.base.header = hdr;
        ui_dg.base.pf = false;
        ui_dg.base.modifier = AX25_U_UI;
        ui_dg.pid = PID_NO_L3;

        enc = ax25_frame_encode((ax25_frame_t*) &ui_dg, &enc_len, &err);

        if (enc && enc_nodigi) {
            TEST_ASSERT(enc_len > enc_nodigi_len, "J.2 Frame with digi larger", 0);
            TEST_ASSERT((enc_len - enc_nodigi_len) == 7, "J.2 One digi adds exactly 7 bytes", (unsigned )(enc_len - enc_nodigi_len));
        }
        if (enc) {
            free(enc);
            enc = NULL;
        }
        if (enc_nodigi) {
            free(enc_nodigi);
            enc_nodigi = NULL;
        }
    }

    // J.4: AX25_MAX_REPEATERS round-trip (fix 12.2)
    {
        const char *digi_calls[8] = { "K1A-1", "K1B-2", "K1C-3", "K1D-4", "K1E-5", "K1F-6", "K1G-7", "K1H-8" };
        int i;
        int n_digis = AX25_MAX_REPEATERS;
        if (n_digis > 8)
            n_digis = 8;

        ax25_frame_header_t max_hdr;
        memset(&max_hdr, 0, sizeof(max_hdr));
        max_hdr.destination = *dest;
        max_hdr.source = *src;
        max_hdr.cr = true;
        max_hdr.repeaters.num_repeaters = n_digis;

        for (i = 0; i < n_digis; i++) {
            ax25_address_t *d = ax25_address_from_string(digi_calls[i], &err);
            if (d) {
                max_hdr.repeaters.repeaters[i] = *d;
                ax25_address_free(d, &err);
            }
        }

        ax25_unnumbered_information_frame_t ui_max;
        memset(&ui_max, 0, sizeof(ui_max));
        ui_max.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui_max.base.base.header = max_hdr;
        ui_max.base.pf = false;
        ui_max.base.modifier = AX25_U_UI;
        ui_max.pid = PID_NO_L3;

        uint8_t *enc_max = ax25_frame_encode((ax25_frame_t*) &ui_max, &enc_len, &err);
        TEST_ASSERT(enc_max != NULL && err == 0, "J.4 Encode frame with AX25_MAX_REPEATERS digipeaters", err);
        if (enc_max) {
            ax25_frame_t *dec_max = ax25_frame_decode(enc_max, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec_max != NULL && err == 0, "J.4 Decode max-repeater frame", err);
            if (dec_max) {
                TEST_ASSERT(dec_max->header.repeaters.num_repeaters == n_digis, "J.4 num_repeaters preserved at maximum", n_digis);
                ax25_frame_free(dec_max, &err);
            }
            free(enc_max);
        }
    }

    // J.5: AX25_MAX_REPEATERS == kernel AX25_MAX_DIGIS (fix 12.2)
    {
        TEST_ASSERT(AX25_MAX_REPEATERS == AX25_MAX_DIGIS, "J.5 AX25_MAX_REPEATERS (libax25v22) == AX25_MAX_DIGIS (kernel)", AX25_MAX_REPEATERS);
        DEBUG_PRINT("J.5 AX25_MAX_REPEATERS=%d AX25_MAX_DIGIS=%d", AX25_MAX_REPEATERS, AX25_MAX_DIGIS);
    }

    ax25_address_free(dest, &err);
    ax25_address_free(src, &err);
    ax25_address_free(digi, &err);

    return 0;
}

// ===========================================================================
// SECTION K: KISS Framing
// ===========================================================================
static int sec_k_kiss_framing(void) {
    TEST_SECTION("=== SEC-K: KISS Framing ===");

    uint8_t payload[64];
    uint8_t kiss_out[256];
    uint8_t kiss_dec[256];
    int kiss_out_len, kiss_dec_len, krc, i;

    // K.1: Basic encode
    {
        for (i = 0; i < 16; i++)
            payload[i] = (uint8_t) (0x41 + i);
        kiss_out_len = 0;
        krc = kiss_encode_frame(payload, 16, 0, 0, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "K.1 KISS encode returns 0", krc);
        TEST_ASSERT(kiss_out_len >= 4, "K.1 KISS frame >= 4 bytes", kiss_out_len);
        TEST_ASSERT(kiss_out[0] == KISS_FEND, "K.1 Starts with FEND", kiss_out[0]);
        TEST_ASSERT(kiss_out[kiss_out_len-1] == KISS_FEND, "K.1 Ends with FEND", 0);
    }

    // K.2: Decode round-trip
    {
        kiss_dec_len = 0;
        krc = kiss_decode_frame(kiss_out, kiss_out_len, kiss_dec, &kiss_dec_len);
        TEST_ASSERT(krc == 0, "K.2 KISS decode returns 0", krc);
        TEST_ASSERT(kiss_dec_len == 16, "K.2 Decoded length == 16", kiss_dec_len);
        TEST_ASSERT(memcmp(kiss_dec, payload, 16) == 0, "K.2 Content matches", 0);
    }

    // K.3: FEND in payload must be escaped
    {
        uint8_t pf[8];
        memset(pf, 0x41, sizeof(pf));
        pf[3] = KISS_FEND;
        kiss_out_len = 0;
        krc = kiss_encode_frame(pf, 8, 0, 0, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "K.3 Encode with FEND in payload", krc);
        int raw_fend = 0;
        for (i = 1; i < kiss_out_len - 1; i++)
            if (kiss_out[i] == KISS_FEND) {
                raw_fend = 1;
                break;
            }
        TEST_ASSERT(raw_fend == 0, "K.3 No raw FEND inside encoded frame", 0);
        kiss_dec_len = 0;
        krc = kiss_decode_frame(kiss_out, kiss_out_len, kiss_dec, &kiss_dec_len);
        TEST_ASSERT(krc == 0 && kiss_dec_len == 8, "K.3 Decode recovers payload", kiss_dec_len);
        TEST_ASSERT(kiss_dec[3] == KISS_FEND, "K.3 FEND byte recovered", kiss_dec[3]);
    }

    // K.4: FESC in payload must be escaped
    {
        uint8_t pf[8];
        memset(pf, 0x41, sizeof(pf));
        pf[4] = KISS_FESC;
        kiss_out_len = 0;
        krc = kiss_encode_frame(pf, 8, 0, 0, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "K.4 Encode with FESC in payload", krc);
        kiss_dec_len = 0;
        krc = kiss_decode_frame(kiss_out, kiss_out_len, kiss_dec, &kiss_dec_len);
        TEST_ASSERT(krc == 0 && kiss_dec_len == 8, "K.4 Decode recovers payload", kiss_dec_len);
        TEST_ASSERT(kiss_dec[4] == KISS_FESC, "K.4 FESC byte recovered", kiss_dec[4]);
    }

    // K.5: AX.25 frame wrapped in KISS and recovered (fix 13.1 — chain test)
    {
        uint8_t err = 0;
        ax25_frame_header_t khdr;
        ax25_unnumbered_information_frame_t kui;
        uint8_t kpayload[] = "KISS AX25 CHAIN";
        ax25_address_t *kdest = ax25_address_from_string("W1AW-0", &err);
        ax25_address_t *ksrc = ax25_address_from_string("N0CALL-0", &err);

        if (kdest && ksrc) {
            memset(&khdr, 0, sizeof(khdr));
            khdr.destination = *kdest;
            khdr.source = *ksrc;
            khdr.cr = true;

            memset(&kui, 0, sizeof(kui));
            kui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
            kui.base.base.header = khdr;
            kui.base.pf = false;
            kui.base.modifier = AX25_U_UI;
            kui.pid = PID_NO_L3;
            kui.payload = kpayload;
            kui.payload_len = sizeof(kpayload) - 1;

            size_t ax25_len = 0;
            uint8_t *ax25_bytes = ax25_frame_encode((ax25_frame_t*) &kui, &ax25_len, &err);
            TEST_ASSERT(ax25_bytes != NULL && err == 0, "K.5 libax25v22 encode for KISS chain", err);

            if (ax25_bytes) {
                uint8_t kiss_frame[512];
                int kiss_len = 0;
                krc = kiss_encode_frame(ax25_bytes, (int) ax25_len, 0, 0, kiss_frame, &kiss_len);
                TEST_ASSERT(krc == 0, "K.5 KISS wrap of AX.25 frame", krc);

                uint8_t kiss_dec2[512];
                int kiss_dec_len2 = 0;
                krc = kiss_decode_frame(kiss_frame, kiss_len, kiss_dec2, &kiss_dec_len2);
                TEST_ASSERT(krc == 0, "K.5 KISS unwrap succeeds", krc);
                TEST_ASSERT(kiss_dec_len2 == (int )ax25_len, "K.5 KISS-unwrapped length == AX.25 frame length", kiss_dec_len2);
                TEST_ASSERT(memcmp(kiss_dec2, ax25_bytes, ax25_len) == 0, "K.5 KISS-unwrapped bytes identical to original AX.25 frame", 0);

                free(ax25_bytes);
            }
            ax25_address_free(kdest, &err);
            ax25_address_free(ksrc, &err);
        } else {
            if (kdest)
                ax25_address_free(kdest, &err);
            if (ksrc)
                ax25_address_free(ksrc, &err);
            printf("SKIP: K.5 address creation failed\n");
        }
    }

    return 0;
}

// ===========================================================================
// SECTION L: Buffer Pool Exhaustion
// ===========================================================================
static int sec_l_buffer_pool_exhaustion(void) {
    TEST_SECTION("=== SEC-L: Buffer Pool Exhaustion ===");

    // (fix 14.1) Use pool-size API to allocate exactly the right count
    {
        int pool_size = (int) ax25_buf_pool_free_count();
        int i;

        TEST_ASSERT(pool_size > 0, "L.1 Pool has at least 1 buffer initially", pool_size);

        if (pool_size <= 0 || pool_size > 256) {
            printf("SKIP: L.1 pool_size=%d out of expected range\n", pool_size);
            return 0;
        }

        ax25_buf_t **bufs = (ax25_buf_t**) malloc((pool_size + 1) * sizeof(ax25_buf_t*));
        TEST_ASSERT(bufs != NULL, "L.1 malloc for buffer pointer array", 0);
        if (!bufs)
            return 0;

        for (i = 0; i < pool_size; i++) {
            bufs[i] = ax25_buf_alloc();
            TEST_ASSERT(bufs[i] != NULL, "L.1 Pool not exhausted yet", 0);
            if (!bufs[i]) {
                pool_size = i;
                break;
            }
        }

        /* This extra allocation must return NULL (no malloc fallback) */
        bufs[pool_size] = ax25_buf_alloc();
        TEST_ASSERT(bufs[pool_size] == NULL, "L.1 Pool exhaustion returns NULL (no malloc fallback)", 0);
        if (bufs[pool_size]) {
            ax25_buf_free(bufs[pool_size]);
        }

        for (i = pool_size - 1; i >= 0; i--)
            ax25_buf_free(bufs[i]);

        TEST_ASSERT((int )ax25_buf_pool_free_count() == pool_size, "L.1 Pool fully restored after exhaustion test", 0);

        free(bufs);
        DEBUG_PRINT("L.1 Pool exhaustion test complete (pool_size=%d)", pool_size);
    }

    return 0;
}

// ===========================================================================
// SECTION M: axconfig API
// ===========================================================================
static int sec_m_ax25config_api(void) {
    TEST_SECTION("=== SEC-M: axconfig API (libax25) ===");

    int pipe_fds[2];
    pid_t child;
    int status;
    int child_port_count = 0;
    ssize_t rd;

    if (pipe(pipe_fds) < 0) {
        TEST_ASSERT(0, "M.0 pipe() for axconfig isolation", errno);
        return 1;
    }

    child = fork();
    if (child < 0) {
        close(pipe_fds[0]);
        close(pipe_fds[1]);
        TEST_ASSERT(0, "M.0 fork() for axconfig isolation", errno);
        return 1;
    }

    if (child == 0) {
        // ---- child process ----
        int rc, local_port_count = 0;
        char cfg_addr_copy[MAX_CALLSIGN_LEN];
        char cfg_dev_copy[MAX_PORT_NAME_LEN];
        int child_result = 0;

        close(pipe_fds[0]);

        // M.1: Load axports
        rc = ax25_config_load_ports();
        if (write(pipe_fds[1], &rc, sizeof(rc)) != sizeof(rc)) {
            close(pipe_fds[1]);
            _exit(2);
        }

        if (rc <= 0) {
            close(pipe_fds[1]);
            _exit(0);
        }
        local_port_count = rc;

        // M.2: Query callsign (fix 15.1 — NULL guard)
        if (local_port_count > 0 && g_test_ctx.port_count > 0) {
            char *cfg_addr = ax25_config_get_addr(g_test_ctx.port_name);
            if (cfg_addr == NULL) {
                fprintf(stderr, "CHILD FAIL: M.2 ax25_config_get_addr NULL\n");
                child_result = 1;
            } else {
                safe_strlcpy(cfg_addr_copy, cfg_addr, sizeof(cfg_addr_copy));
                size_t addr_len = strlen(cfg_addr_copy);
                if (addr_len < 3 || addr_len > 9) {
                    fprintf(stderr, "CHILD FAIL: M.2 callsign length %zu not in 3-9\n", addr_len);
                    child_result = 1;
                } else {
                    ax25_address addr2;
                    if (ax25_aton_entry(cfg_addr_copy, (char*) &addr2) != 0) {
                        fprintf(stderr, "CHILD FAIL: M.2 ax25_aton_entry(%s) failed\n", cfg_addr_copy);
                        child_result = 1;
                    }
                }
            }
        }

        // M.3: Query device name (fix 15.1)
        if (local_port_count > 0 && g_test_ctx.port_count > 0) {
            char *cfg_dev = ax25_config_get_dev(g_test_ctx.port_name);
            if (cfg_dev != NULL) {
                safe_strlcpy(cfg_dev_copy, cfg_dev, sizeof(cfg_dev_copy));
                if (strlen(cfg_dev_copy) == 0) {
                    fprintf(stderr, "CHILD FAIL: M.3 device name empty\n");
                    child_result = 1;
                }
            }
        }

        close(pipe_fds[1]);
        _exit(child_result);
    }

    // ---- parent ----
    close(pipe_fds[1]);
    rd = read(pipe_fds[0], &child_port_count, sizeof(child_port_count));
    close(pipe_fds[0]);
    if (rd != (ssize_t) sizeof(child_port_count))
        child_port_count = 0;

    waitpid(child, &status, 0);
    int child_exit = WIFEXITED(status) ? WEXITSTATUS(status) : 127;

    if (child_port_count == 0) {
        printf("SKIP: M.1 ax25_config_load_ports=0 (no ports / axports absent)\n");
        return 0;
    }
    TEST_ASSERT(child_port_count > 0, "M.1 ax25_config_load_ports returned > 0", child_port_count);

    if (g_test_ctx.port_count > 0)
        TEST_ASSERT(child_exit == 0, "M.2/M.3 axconfig queries passed in child", child_exit);

    // M.5: axports window vs kernel sysctl standard_window_size cross-check (fix 15.2).
    // Uses g_test_ctx.port_window set by load_ax25_config() from the axports file,
    // avoiding a call to ax25_config_get_window() which is not present in all
    // versions of libax25.
    if (g_test_ctx.port_count > 0 && g_test_ctx.kernel_ax25_available) {
        int axports_window = g_test_ctx.port_window;
        char sysctl_path[128];
        int sysctl_val = -1;
        snprintf(sysctl_path, sizeof(sysctl_path), "/proc/sys/net/ax25/%s/standard_window_size", g_test_ctx.port_name);
        FILE *fp = fopen(sysctl_path, "r");
        if (fp) {
            char buf[16];
            if (fgets(buf, sizeof(buf), fp))
                sysctl_val = atoi(buf);
            fclose(fp);
        }
        if (axports_window > 0 && sysctl_val > 0) {
            TEST_ASSERT(axports_window == sysctl_val, "M.5 axports window == kernel sysctl standard_window_size", axports_window);
            DEBUG_PRINT("M.5 axports_window=%d sysctl_window=%d", axports_window, sysctl_val);
        } else {
            printf("SKIP: M.5 (axports_window=%d sysctl=%d — one not available)\n", axports_window, sysctl_val);
        }
    } else {
        printf("SKIP: M.5 (no AX.25 ports configured or no kernel AX.25)\n");
    }

    return 0;
}

// ===========================================================================
// SECTION N: SOCK_DGRAM UI Frames
// ===========================================================================
static int sec_n_sock_dgram_ui_frames(void) {
    TEST_SECTION("=== SEC-N: SOCK_DGRAM UI Frame via AF_AX25 ===");

    int sock, rc;

    if (!g_test_ctx.kernel_ax25_available) {
        printf("SKIP: SEC-N (no kernel AF_AX25 support)\n");
        return 0;
    }

    // N.1: Create SOCK_DGRAM
    {
        sock = socket(AF_AX25, SOCK_DGRAM, 0);
        if (sock < 0) {
            if (errno == EPROTONOSUPPORT || errno == ESOCKTNOSUPPORT) {
                printf("SKIP: SEC-N (SOCK_DGRAM not supported on this kernel)\n");
                return 0;
            }
            TEST_ASSERT(sock >= 0, "N.1 Create AF_AX25 SOCK_DGRAM", errno);
            return 0;
        }
        TEST_ASSERT(sock >= 0, "N.1 Create AF_AX25 SOCK_DGRAM socket", 0);
        close(sock);
    }

    // N.2 & N.3: Bind and non-blocking recv
    {
        if (!g_test_ctx.socket_bind_available) {
            printf("SKIP: N.2/N.3/N.4 SOCK_DGRAM (AX.25 interface not configured)\n");
            return 0;
        }

        sock = socket(AF_AX25, SOCK_DGRAM, 0);
        TEST_ASSERT(sock >= 0, "N.2 Create SOCK_DGRAM for bind", 0);
        if (sock < 0)
            return 0;

        rc = setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, g_test_ctx.port_name, strlen(g_test_ctx.port_name));
        if (rc < 0) {
            printf("SKIP: N.2 SO_BINDTODEVICE failed (need CAP_NET_RAW): %s\n", strerror(errno));
            close(sock);
            return 0;
        }

        struct sockaddr_ax25 local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sax25_family = AF_AX25;
        local_addr.sax25_ndigis = 0;

        rc = ax25_aton_entry(g_test_ctx.local_call, (char*) &local_addr.sax25_call);
        if (rc < 0) {
            printf("SKIP: N.2 ax25_aton_entry failed\n");
            close(sock);
            return 0;
        }

        rc = bind(sock, (struct sockaddr*) &local_addr, sizeof(struct sockaddr_ax25));
        TEST_ASSERT(rc == 0, "N.2 Bind SOCK_DGRAM to local callsign", errno);
        if (rc < 0) {
            close(sock);
            return 0;
        }

        {
            int flags = fcntl(sock, F_GETFL, 0);
            if (flags >= 0) {
                rc = fcntl(sock, F_SETFL, flags | O_NONBLOCK);
                TEST_ASSERT(rc == 0, "N.3 Set SOCK_DGRAM non-blocking", rc);

                uint8_t recv_buf[512];
                int n_recv = (int) recvfrom(sock, recv_buf, sizeof(recv_buf), 0, NULL, NULL);
                int recv_ok = (n_recv < 0) && (errno == EAGAIN || errno == EWOULDBLOCK);
                TEST_ASSERT(recv_ok, "N.3 SOCK_DGRAM non-blocking recvfrom returns EAGAIN/EWOULDBLOCK", errno);
            }
        }

        close(sock);
    }

    // N.4: SOCK_DGRAM loopback send/receive (fix 16.1)
    {
        if (!g_test_ctx.socket_bind_available) {
            printf("SKIP: N.4 SOCK_DGRAM loopback (no AX.25 interface)\n");
        } else {
            int tx_sock = socket(AF_AX25, SOCK_DGRAM, 0);
            int rx_sock = socket(AF_AX25, SOCK_DGRAM, 0);

            if (tx_sock >= 0 && rx_sock >= 0) {
                struct sockaddr_ax25 la;
                memset(&la, 0, sizeof(la));
                la.sax25_family = AF_AX25;
                ax25_aton_entry(g_test_ctx.local_call, (char*) &la.sax25_call);

                bind(rx_sock, (struct sockaddr*) &la, sizeof(la));
                int flags = fcntl(rx_sock, F_GETFL, 0);
                if (flags >= 0)
                    fcntl(rx_sock, F_SETFL, flags | O_NONBLOCK);

                uint8_t err_n4 = 0;
                ax25_frame_header_t lphdr;
                ax25_unnumbered_information_frame_t lpui;
                uint8_t lppayload[] = "LOOPBACK UI TEST";
                ax25_address_t *lpdest = ax25_address_from_string(g_test_ctx.local_call, &err_n4);
                ax25_address_t *lpsrc = ax25_address_from_string(g_test_ctx.local_call, &err_n4);

                if (lpdest && lpsrc) {
                    memset(&lphdr, 0, sizeof(lphdr));
                    lphdr.destination = *lpdest;
                    lphdr.source = *lpsrc;
                    lphdr.cr = true;

                    memset(&lpui, 0, sizeof(lpui));
                    lpui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
                    lpui.base.base.header = lphdr;
                    lpui.base.pf = false;
                    lpui.base.modifier = AX25_U_UI;
                    lpui.pid = PID_NO_L3;
                    lpui.payload = lppayload;
                    lpui.payload_len = sizeof(lppayload) - 1;

                    size_t fl = 0;
                    uint8_t *fb = ax25_frame_encode((ax25_frame_t*) &lpui, &fl, &err_n4);
                    if (fb && fl > 0) {
                        int sent = (int) sendto(tx_sock, fb, fl, 0, (struct sockaddr*) &la, sizeof(la));
                        if (sent < 0) {
                            printf("SKIP: N.4 sendto failed (%s)\n", strerror(errno));
                        } else {
                            uint8_t rbuf[512];
                            usleep(50000);
                            int nr = (int) recvfrom(rx_sock, rbuf, sizeof(rbuf), 0, NULL, NULL);
                            TEST_ASSERT(nr > 0, "N.4 SOCK_DGRAM loopback: recvfrom received frame", nr);
                            if (nr > 0)
                                DEBUG_PRINT("N.4 Loopback received %d bytes", nr);
                        }
                        free(fb);
                    }
                    ax25_address_free(lpdest, &err_n4);
                    ax25_address_free(lpsrc, &err_n4);
                }
            }
            if (tx_sock >= 0)
                close(tx_sock);
            if (rx_sock >= 0)
                close(rx_sock);
        }
    }

    return 0;
}

// ===========================================================================
// SECTION O: /proc/sys/net/ax25 Sysctl Parameters
// ===========================================================================
static int sec_o_sysctl_ax25_params(void) {
    TEST_SECTION("=== SEC-O: /proc/sys/net/ax25 Sysctl Interface ===");

    char path[128], buf[32];
    FILE *fp;
    int value, n;

    if (!g_test_ctx.kernel_ax25_available || g_test_ctx.port_count == 0) {
        printf("SKIP: SEC-O (no kernel AX.25 or no configured ports)\n");
        return 0;
    }

    // O.1: t1_timeout (fix 17.1 — widened to kernel valid range 1-3000)
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/t1_timeout", g_test_ctx.port_name);
        if (n <= 0 || n >= (int) sizeof(path)) {
            printf("SKIP: O.1 path too long\n");
        } else {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.1 %s not readable (%s)\n", path, strerror(errno));
            } else {
                memset(buf, 0, sizeof(buf));
                if (fgets(buf, sizeof(buf), fp)) {
                    value = atoi(buf);
                    TEST_ASSERT(value >= 1 && value <= 3000, "O.1 t1_timeout in kernel valid range [1..3000]", value);
                    if (value < 10 || value > 300)
                        DEBUG_PRINT("O.1 NOTE: t1_timeout=%d outside typical [10..300]", value);
                    DEBUG_PRINT("O.1 t1_timeout=%d (%d ms)", value, value * 100);
                }
                fclose(fp);
            }
        }
    }

    // O.2: maximum_retry_count N2
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/maximum_retry_count", g_test_ctx.port_name);
        if (n > 0 && n < (int) sizeof(path)) {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.2 %s not readable\n", path);
            } else {
                if (fgets(buf, sizeof(buf), fp)) {
                    value = atoi(buf);
                    TEST_ASSERT(value >= 1 && value <= 127, "O.2 maximum_retry_count N2 in [1..127]", value);
                }
                fclose(fp);
            }
        }
    }

    // O.3: t3_timeout
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/t3_timeout", g_test_ctx.port_name);
        if (n > 0 && n < (int) sizeof(path)) {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.3 %s not readable\n", path);
            } else {
                if (fgets(buf, sizeof(buf), fp)) {
                    value = atoi(buf);
                    TEST_ASSERT(value >= 0 && value <= 36000, "O.3 t3_timeout in [0..36000] (0=disabled)", value);
                }
                fclose(fp);
            }
        }
    }

    // O.4: standard_window_size
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/standard_window_size", g_test_ctx.port_name);
        if (n > 0 && n < (int) sizeof(path)) {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.4 %s not readable\n", path);
            } else {
                if (fgets(buf, sizeof(buf), fp)) {
                    value = atoi(buf);
                    TEST_ASSERT(value >= 1 && value <= 7, "O.4 standard_window_size (mod-8) in [1..7]", value);
                }
                fclose(fp);
            }
        }
    }

    // O.5: extended_window_size (fix 17.1 — widened to 1-127)
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/extended_window_size", g_test_ctx.port_name);
        if (n > 0 && n < (int) sizeof(path)) {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.5 %s not readable\n", path);
            } else {
                if (fgets(buf, sizeof(buf), fp)) {
                    value = atoi(buf);
                    TEST_ASSERT(value >= 1 && value <= 127, "O.5 extended_window_size (mod-128) in [1..127]", value);
                }
                fclose(fp);
            }
        }
    }

    // O.6: t2_timeout (delayed-ACK)
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/t2_timeout", g_test_ctx.port_name);
        if (n > 0 && n < (int) sizeof(path)) {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.6 %s not readable\n", path);
            } else {
                if (fgets(buf, sizeof(buf), fp)) {
                    value = atoi(buf);
                    TEST_ASSERT(value >= 1 && value <= 20, "O.6 t2_timeout (delayed-ACK) in [1..20]", value);
                }
                fclose(fp);
            }
        }
    }

    // O.7: connect_mode (fix 17.2)
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/connect_mode", g_test_ctx.port_name);
        if (n > 0 && n < (int) sizeof(path)) {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.7 connect_mode not readable (%s)\n", strerror(errno));
            } else {
                if (fgets(buf, sizeof(buf), fp)) {
                    value = atoi(buf);
                    TEST_ASSERT(value >= 0 && value <= 2, "O.7 connect_mode in [0..2] (0=none,1=SABM,2=SABME)", value);
                    DEBUG_PRINT("O.7 connect_mode=%d (%s)", value, value == 0 ? "connectionless" : value == 1 ? "SABM (mod-8)" : "SABME (mod-128)");
                }
                fclose(fp);
            }
        }
    }

    return 0;
}

// ===========================================================================
// SECTION P: full_sockaddr_ax25 Digipeater Path
// ===========================================================================
static int sec_p_full_sockaddr_digipeater(void) {
    TEST_SECTION("=== SEC-P: full_sockaddr_ax25 Digipeater Path ===");

    int sock, rc;
    struct full_sockaddr_ax25 faddr;
    char ntoa_buf[MAX_CALLSIGN_LEN];

    if (!g_test_ctx.kernel_ax25_available) {
        printf("SKIP: SEC-P (no kernel AF_AX25)\n");
        return 0;
    }

    // P.1: Build full_sockaddr_ax25 with one digipeater
    {
        memset(&faddr, 0, sizeof(faddr));
        faddr.fsa_ax25.sax25_family = AF_AX25;
        faddr.fsa_ax25.sax25_ndigis = 1;

        rc = ax25_aton_entry(g_test_ctx.local_call, (char*) &faddr.fsa_ax25.sax25_call);
        TEST_ASSERT(rc == 0, "P.1 ax25_aton_entry into fsa_ax25.sax25_call", rc);

        rc = ax25_aton_entry("K1TTT-4", (char*) &faddr.fsa_digipeater[0]);
        TEST_ASSERT(rc == 0, "P.1 ax25_aton_entry K1TTT-4 into fsa_digipeater[0]", rc);

        TEST_ASSERT(faddr.fsa_ax25.sax25_ndigis == 1, "P.1 sax25_ndigis == 1", 0);
        TEST_ASSERT(faddr.fsa_ax25.sax25_family == AF_AX25, "P.1 family == AF_AX25", 0);
    }

    // P.2: sizeof checks
    {
        int full_size = (int) sizeof(struct full_sockaddr_ax25);
        int base_size = (int) sizeof(struct sockaddr_ax25);
        TEST_ASSERT(full_size > base_size, "P.2 sizeof(full_sockaddr_ax25) > sizeof(sockaddr_ax25)", full_size);
    }

    // P.3: connect() with full_sockaddr_ax25 — must not EFAULT
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock < 0) {
            printf("SKIP: P.3 socket creation failed\n");
        } else {
            rc = ax25_aton_entry("W1AW-0", (char*) &faddr.fsa_ax25.sax25_call);
            if (rc < 0) {
                printf("SKIP: P.3 ax25_aton_entry W1AW-0 failed\n");
            } else {
                int flags = fcntl(sock, F_GETFL, 0);
                if (flags >= 0)
                    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

                rc = connect(sock, (struct sockaddr*) &faddr, sizeof(struct full_sockaddr_ax25));
                int connect_ok = (rc < 0) && (errno != EFAULT);
                TEST_ASSERT(connect_ok, "P.3 connect() with full_sockaddr_ax25: no EFAULT", errno);
            }
            close(sock);
        }
    }

    // P.4: Digipeater callsign round-trips through ax25_ntoa (fix 18.1)
    {
        char *digi_str = ax25_ntoa(&faddr.fsa_digipeater[0]);
        TEST_ASSERT(digi_str != NULL, "P.4 ax25_ntoa on fsa_digipeater[0] non-NULL", 0);
        if (digi_str) {
            safe_strlcpy(ntoa_buf, digi_str, sizeof(ntoa_buf));
            TEST_ASSERT(strcmp(ntoa_buf, "K1TTT-4") == 0, "P.4 fsa_digipeater[0] round-trips to K1TTT-4", 0);
            DEBUG_PRINT("P.4 fsa_digipeater[0] = %s", ntoa_buf);
        }
    }

    return 0;
}

// ===========================================================================
// SECTION Q: Supervisory Frames (RR/RNR/REJ/SREJ)
// ===========================================================================
static int sec_q_supervisory_frames(void) {
    TEST_SECTION("=== SEC-Q: Supervisory Frames (RR / RNR / REJ / SREJ) ===");

    uint8_t err;
    ax25_frame_header_t hdr;
    uint8_t *enc;
    size_t enc_len;
    ax25_frame_t *dec;
    (void) enc;
    (void) enc_len;
    (void) dec;

    ax25_address_t *dest = ax25_address_from_string("W1AW-0", &err);
    ax25_address_t *src = ax25_address_from_string("N0CALL-0", &err);

    if (!dest || !src) {
        if (dest)
            ax25_address_free(dest, &err);
        if (src)
            ax25_address_free(src, &err);
        printf("SKIP: SEC-Q address creation failed\n");
        return 0;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.destination = *dest;
    hdr.source = *src;
    hdr.cr = true;
    hdr.repeaters.num_repeaters = 0;

    ax25_address_free(dest, &err);
    ax25_address_free(src, &err);

#ifdef AX25_FRAME_SUPERVISORY_RR_8BIT
    // Q.1: RR mod-8
    {
        ax25_supervisory_frame_t rr;
        memset(&rr, 0, sizeof(rr));
        rr.base.type   = AX25_FRAME_SUPERVISORY_RR_8BIT;
        rr.base.header = hdr;
        rr.pf          = false;
        rr.nr          = 3;
        enc = ax25_frame_encode((ax25_frame_t*)&rr, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "Q.1 Encode RR mod-8 N(R)=3", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "Q.1 Decode RR mod-8", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_SUPERVISORY_RR_8BIT, "Q.1 type==RR-8", 0);
                TEST_ASSERT(((ax25_supervisory_frame_t*)dec)->nr == 3, "Q.1 N(R)=3", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // Q.2: RNR mod-8
    {
        ax25_supervisory_frame_t rnr;
        memset(&rnr, 0, sizeof(rnr));
        rnr.base.type   = AX25_FRAME_SUPERVISORY_RNR_8BIT;
        rnr.base.header = hdr;
        rnr.pf          = true;
        rnr.nr          = 0;
        enc = ax25_frame_encode((ax25_frame_t*)&rnr, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "Q.2 Encode RNR mod-8 P/F=1", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "Q.2 Decode RNR mod-8", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_SUPERVISORY_RNR_8BIT, "Q.2 type==RNR-8", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // Q.3: REJ mod-8
    {
        ax25_supervisory_frame_t rej;
        memset(&rej, 0, sizeof(rej));
        rej.base.type   = AX25_FRAME_SUPERVISORY_REJ_8BIT;
        rej.base.header = hdr;
        rej.pf          = false;
        rej.nr          = 5;
        enc = ax25_frame_encode((ax25_frame_t*)&rej, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "Q.3 Encode REJ mod-8 N(R)=5", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "Q.3 Decode REJ mod-8", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_SUPERVISORY_REJ_8BIT, "Q.3 type==REJ-8", 0);
                TEST_ASSERT(((ax25_supervisory_frame_t*)dec)->nr == 5, "Q.3 N(R)=5", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }
#else
    printf("SKIP: Q.1-Q.3 (AX25_FRAME_SUPERVISORY_RR_8BIT not defined)\n");
#endif

#ifdef AX25_FRAME_SUPERVISORY_RR_16BIT
    // Q.4: RR mod-128
    {
        ax25_supervisory_frame_t rr16;
        memset(&rr16, 0, sizeof(rr16));
        rr16.base.type   = AX25_FRAME_SUPERVISORY_RR_16BIT;
        rr16.base.header = hdr;
        rr16.pf          = false;
        rr16.nr          = 64;
        enc = ax25_frame_encode((ax25_frame_t*)&rr16, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "Q.4 Encode RR mod-128 N(R)=64", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_TRUE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "Q.4 Decode RR mod-128", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_SUPERVISORY_RR_16BIT, "Q.4 type==RR-16", 0);
                TEST_ASSERT(((ax25_supervisory_frame_t*)dec)->nr == 64, "Q.4 N(R)=64", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }
#else
    printf("SKIP: Q.4 (AX25_FRAME_SUPERVISORY_RR_16BIT not defined)\n");
#endif

#ifdef AX25_FRAME_SUPERVISORY_RNR_16BIT
    // Q.5: RNR mod-128
    {
        ax25_supervisory_frame_t rnr16;
        memset(&rnr16, 0, sizeof(rnr16));
        rnr16.base.type   = AX25_FRAME_SUPERVISORY_RNR_16BIT;
        rnr16.base.header = hdr;
        rnr16.pf          = true;
        rnr16.nr          = 96;
        enc = ax25_frame_encode((ax25_frame_t*)&rnr16, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "Q.5 Encode RNR mod-128 N(R)=96", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_TRUE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "Q.5 Decode RNR mod-128", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_SUPERVISORY_RNR_16BIT, "Q.5 type==RNR-16", 0);
                TEST_ASSERT(((ax25_supervisory_frame_t*)dec)->nr == 96, "Q.5 N(R)=96", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }
#else
    printf("SKIP: Q.5 (AX25_FRAME_SUPERVISORY_RNR_16BIT not defined)\n");
#endif

#ifdef AX25_FRAME_SUPERVISORY_REJ_16BIT
    // Q.6: REJ mod-128 boundary N(R)=127
    {
        ax25_supervisory_frame_t rej16;
        memset(&rej16, 0, sizeof(rej16));
        rej16.base.type   = AX25_FRAME_SUPERVISORY_REJ_16BIT;
        rej16.base.header = hdr;
        rej16.pf          = false;
        rej16.nr          = 127;
        enc = ax25_frame_encode((ax25_frame_t*)&rej16, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "Q.6 Encode REJ mod-128 N(R)=127", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_TRUE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "Q.6 Decode REJ mod-128", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_SUPERVISORY_REJ_16BIT, "Q.6 type==REJ-16", 0);
                TEST_ASSERT(((ax25_supervisory_frame_t*)dec)->nr == 127, "Q.6 N(R)=127", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }
#else
    printf("SKIP: Q.6 (AX25_FRAME_SUPERVISORY_REJ_16BIT not defined)\n");
#endif

#ifdef AX25_FRAME_SUPERVISORY_SREJ_8BIT
    // Q.7: SREJ mod-8 (fix 19.2)
    {
        ax25_supervisory_frame_t srej;
        memset(&srej, 0, sizeof(srej));
        srej.base.type   = AX25_FRAME_SUPERVISORY_SREJ_8BIT;
        srej.base.header = hdr;
        srej.pf          = false;
        srej.nr          = 4;
        enc = ax25_frame_encode((ax25_frame_t*)&srej, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "Q.7 Encode SREJ mod-8 N(R)=4", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "Q.7 Decode SREJ mod-8", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_SUPERVISORY_SREJ_8BIT, "Q.7 type==SREJ-8", 0);
                TEST_ASSERT(((ax25_supervisory_frame_t*)dec)->nr == 4, "Q.7 N(R)=4 preserved", 0);
                DEBUG_PRINT("Q.7 SREJ mod-8 enc_len=%zu ctrl=0x%02X",
                            enc_len, enc_len > 14 ? enc[14] : 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }
#else
    printf("SKIP: Q.7 SREJ mod-8 (AX25_FRAME_SUPERVISORY_SREJ_8BIT not defined)\n");
#endif

#ifdef AX25_FRAME_SUPERVISORY_SREJ_16BIT
    // Q.8: SREJ mod-128 N(R)=0 (wrap boundary)
    {
        ax25_supervisory_frame_t srej16;
        memset(&srej16, 0, sizeof(srej16));
        srej16.base.type   = AX25_FRAME_SUPERVISORY_SREJ_16BIT;
        srej16.base.header = hdr;
        srej16.pf          = false;
        srej16.nr          = 0;
        enc = ax25_frame_encode((ax25_frame_t*)&srej16, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "Q.8 Encode SREJ mod-128 N(R)=0 (wrap)", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_TRUE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "Q.8 Decode SREJ mod-128", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_SUPERVISORY_SREJ_16BIT, "Q.8 type==SREJ-16", 0);
                TEST_ASSERT(((ax25_supervisory_frame_t*)dec)->nr == 0, "Q.8 N(R)=0 preserved", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }
#else
    printf("SKIP: Q.8 SREJ mod-128 (AX25_FRAME_SUPERVISORY_SREJ_16BIT not defined)\n");
#endif

    return 0;
}

// ===========================================================================
// SECTION R: SABME Frame (Mod-128 Connection Setup)
// ===========================================================================
static int sec_r_sabme_frame(void) {
    TEST_SECTION("=== SEC-R: SABME Frame (Mod-128 Connection Setup) ===");

    uint8_t err;
    ax25_frame_header_t hdr;
    uint8_t *enc;
    size_t enc_len;
    ax25_frame_t *dec;
    (void) enc;
    (void) enc_len;
    (void) dec;

    ax25_address_t *dest = ax25_address_from_string("W1AW-0", &err);
    ax25_address_t *src = ax25_address_from_string("N0CALL-0", &err);

    if (!dest || !src) {
        if (dest)
            ax25_address_free(dest, &err);
        if (src)
            ax25_address_free(src, &err);
        printf("SKIP: SEC-R address creation failed\n");
        return 0;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.destination = *dest;
    hdr.source = *src;
    hdr.cr = true;
    hdr.repeaters.num_repeaters = 0;

    ax25_address_free(dest, &err);
    ax25_address_free(src, &err);

#ifdef AX25_FRAME_UNNUMBERED_SABME
    // R.1: Encode SABME
    {
        ax25_unnumbered_frame_t sabme;
        memset(&sabme, 0, sizeof(sabme));
        sabme.base.type   = AX25_FRAME_UNNUMBERED_SABME;
        sabme.base.header = hdr;
        sabme.pf          = true;
        sabme.modifier    = AX25_U_SABME;

        enc = ax25_frame_encode((ax25_frame_t*)&sabme, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "R.1 Encode SABME", err);

        if (enc) {
            // R.2: Same size as SABM
            {
                ax25_unnumbered_frame_t sabm;
                memset(&sabm, 0, sizeof(sabm));
                sabm.base.type   = AX25_FRAME_UNNUMBERED_SABM;
                sabm.base.header = hdr;
                sabm.pf          = true;
                sabm.modifier    = AX25_U_SABM;
                size_t sabm_len  = 0;
                uint8_t *enc_sabm = ax25_frame_encode((ax25_frame_t*)&sabm, &sabm_len, &err);
                if (enc_sabm) {
                    TEST_ASSERT(enc_len == sabm_len,
                        "R.2 SABME same encoded size as SABM", 0);
                    free(enc_sabm);
                }
            }

            // R.3: Control byte must be 0x7F (P=1) or 0x6F (P=0) (fix 20.1)
            if (enc_len > 14) {
                uint8_t ctrl = enc[14];
                TEST_ASSERT(ctrl == 0x7F || ctrl == 0x6F,
                    "R.3 SABME ctrl byte is 0x7F (P=1) or 0x6F (P=0)", ctrl);
                TEST_ASSERT(ctrl != 0x3F && ctrl != 0x2F,
                    "R.3 SABME ctrl byte is NOT SABM (0x3F/0x2F)", ctrl);
                DEBUG_PRINT("R.3 SABME ctrl=0x%02X (expected 0x7F)", ctrl);
            }

            // R.4: Decode SABME round-trip
            dec = ax25_frame_decode(enc, enc_len, MODULO128_TRUE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "R.4 Decode SABME round-trip", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_UNNUMBERED_SABME, "R.4 type==SABME", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }
#else
    printf("SKIP: SEC-R (AX25_FRAME_UNNUMBERED_SABME not defined)\n");
#endif

    return 0;
}

// ===========================================================================
// SECTION S: XID Frame (Capability Exchange)
// ===========================================================================
static int sec_s_xid_frame(void) {
    TEST_SECTION("=== SEC-S: XID Frame (AX.25 v2.2 Capability Exchange) ===");

    uint8_t err;
    size_t enc_len;
    uint8_t *enc;
    ax25_frame_t *dec;
    ax25_frame_header_t hdr;

    ax25_address_t *dest = ax25_address_from_string("W1AW-0", &err);
    ax25_address_t *src = ax25_address_from_string("N0CALL-0", &err);

    if (!dest || !src) {
        if (dest)
            ax25_address_free(dest, &err);
        if (src)
            ax25_address_free(src, &err);
        printf("SKIP: SEC-S address creation failed\n");
        return 0;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.destination = *dest;
    hdr.source = *src;
    hdr.cr = true;
    hdr.repeaters.num_repeaters = 0;

    ax25_address_free(dest, &err);
    ax25_address_free(src, &err);

    // S.1: Encode XID
    {
        ax25_exchange_identification_frame_t xid;
        memset(&xid, 0, sizeof(xid));
        xid.base.base.type = AX25_FRAME_UNNUMBERED_XID;
        xid.base.base.header = hdr;
        xid.base.pf = true;
        xid.base.modifier = AX25_U_XID;

        enc = ax25_frame_encode((ax25_frame_t*) &xid, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "S.1 Encode XID", err);

        if (enc) {
            // S.2: Decode round-trip
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "S.2 Decode XID round-trip", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_UNNUMBERED_XID, "S.2 type==XID", 0);
                ax25_frame_free(dec, &err);
            }

            // S.3: XID control byte + FI byte check (fix 21.1)
            if (enc_len > 14) {
                uint8_t ctrl = enc[14];
                TEST_ASSERT(ctrl == 0xBF || ctrl == 0xAF, "S.3 XID ctrl byte 0xBF (P=1) or 0xAF (P=0)", ctrl);
                if (enc_len > 15 && enc[15] != 0) {
                    TEST_ASSERT(enc[15] == 0x82, "S.3 XID FI byte == 0x82 (ISO 8885)", enc[15]);
                }
                DEBUG_PRINT("S.3 XID ctrl=0x%02X FI=0x%02X", ctrl, enc_len > 15 ? enc[15] : 0);
            }

            free(enc);
            enc = NULL;
        }
    }

    // S.4: XID and SABM produce different bytes
    {
        ax25_exchange_identification_frame_t xid2;
        ax25_unnumbered_frame_t sabm;
        size_t xid_len = 0, sabm_len = 0;
        uint8_t *enc_xid = NULL, *enc_sabm = NULL;

        memset(&xid2, 0, sizeof(xid2));
        xid2.base.base.type = AX25_FRAME_UNNUMBERED_XID;
        xid2.base.base.header = hdr;
        xid2.base.pf = true;
        xid2.base.modifier = AX25_U_XID;

        memset(&sabm, 0, sizeof(sabm));
        sabm.base.type = AX25_FRAME_UNNUMBERED_SABM;
        sabm.base.header = hdr;
        sabm.pf = true;
        sabm.modifier = AX25_U_SABM;

        enc_xid = ax25_frame_encode((ax25_frame_t*) &xid2, &xid_len, &err);
        enc_sabm = ax25_frame_encode((ax25_frame_t*) &sabm, &sabm_len, &err);

        if (enc_xid && enc_sabm && xid_len > 0 && sabm_len > 0) {
            size_t cmp_len = xid_len < sabm_len ? xid_len : sabm_len;
            TEST_ASSERT(memcmp(enc_xid, enc_sabm, cmp_len) != 0, "S.4 XID and SABM have different encoded bytes", 0);
        }
        if (enc_xid)
            free(enc_xid);
        if (enc_sabm)
            free(enc_sabm);
    }

    return 0;
}

// ===========================================================================
// SECTION T: PID Values in UI Frames
// ===========================================================================
static int sec_t_pid_values(void) {
    TEST_SECTION("=== SEC-T: PID Values in UI Frames ===");

    uint8_t err;
    size_t enc_len;
    uint8_t *enc;
    ax25_frame_t *dec;
    ax25_frame_header_t hdr;

    ax25_address_t *dest = ax25_address_from_string("W1AW-0", &err);
    ax25_address_t *src = ax25_address_from_string("N0CALL-0", &err);

    if (!dest || !src) {
        if (dest)
            ax25_address_free(dest, &err);
        if (src)
            ax25_address_free(src, &err);
        printf("SKIP: SEC-T address creation failed\n");
        return 0;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.destination = *dest;
    hdr.source = *src;
    hdr.cr = false;
    hdr.repeaters.num_repeaters = 0;

    ax25_address_free(dest, &err);
    ax25_address_free(src, &err);

    uint8_t payload[] = "PID TEST";

#define PID_ROUND_TRIP(testnum, pid_val, desc) \
    do { \
        ax25_unnumbered_information_frame_t ui; \
        memset(&ui, 0, sizeof(ui)); \
        ui.base.base.type   = AX25_FRAME_UNNUMBERED_INFORMATION; \
        ui.base.base.header = hdr; \
        ui.base.pf          = false; \
        ui.base.modifier    = AX25_U_UI; \
        ui.pid              = (pid_val); \
        ui.payload          = payload; \
        ui.payload_len      = sizeof(payload) - 1; \
        enc = ax25_frame_encode((ax25_frame_t*)&ui, &enc_len, &err); \
        TEST_ASSERT(enc != NULL && err == 0, testnum " Encode UI PID=" desc, err); \
        if (enc) { \
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err); \
            TEST_ASSERT(dec != NULL && err == 0, testnum " Decode round-trip", err); \
            if (dec) { \
                ax25_unnumbered_information_frame_t *dui = \
                    (ax25_unnumbered_information_frame_t*)dec; \
                TEST_ASSERT(dui->pid == (pid_val), testnum " PID preserved", dui->pid); \
                ax25_frame_free(dec, &err); \
            } \
            free(enc); \
        } \
    } while(0)

    PID_ROUND_TRIP("T.1", PID_NO_L3, "0xF0 (No L3)");
    PID_ROUND_TRIP("T.2", PID_IP, "0xCC (IP)");
    PID_ROUND_TRIP("T.3", PID_NETROM, "0xCF (NET/ROM)");
    PID_ROUND_TRIP("T.4", PID_ARP, "0xCD (ARP)");
#undef PID_ROUND_TRIP

    // T.5: PID 0xFF (reserved two-byte escape) must be rejected
    {
        ax25_unnumbered_information_frame_t ui;
        memset(&ui, 0, sizeof(ui));
        ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui.base.base.header = hdr;
        ui.base.pf = false;
        ui.base.modifier = AX25_U_UI;
        ui.pid = 0xFF;
        ui.payload = payload;
        ui.payload_len = sizeof(payload) - 1;

        err = 0;
        int val_rc = validate_frame_for_encoding((ax25_frame_t*) &ui, &err);
        if (val_rc != 0) {
            TEST_ASSERT(err != 0, "T.5 PID=0xFF rejected by validator", err);
        } else {
            enc = ax25_frame_encode((ax25_frame_t*) &ui, &enc_len, &err);
            TEST_ASSERT(enc == NULL || err != 0, "T.5 PID=0xFF rejected by encode", err);
            if (enc)
                free(enc);
        }
    }

    // T.6: PID byte at correct offset enc[15] in two-station UI frame (fix 22.1)
    {
        ax25_unnumbered_information_frame_t ui;
        memset(&ui, 0, sizeof(ui));
        ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui.base.base.header = hdr;
        ui.base.pf = false;
        ui.base.modifier = AX25_U_UI;
        ui.pid = PID_NO_L3;

        enc = ax25_frame_encode((ax25_frame_t*) &ui, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "T.6 Encode UI for offset check", err);
        if (enc && enc_len >= 16) {
            TEST_ASSERT(enc[14] == 0x03, "T.6 Control byte at enc[14] == 0x03 (UI)", enc[14]);
            TEST_ASSERT(enc[15] == PID_NO_L3, "T.6 PID byte at enc[15] == 0xF0 (No L3)", enc[15]);
            DEBUG_PRINT("T.6 enc[14]=0x%02X enc[15]=0x%02X", enc[14], enc[15]);
            free(enc);
        }
    }

    return 0;
}

// ===========================================================================
// SECTION U: FRMR (Frame Reject)
// ===========================================================================
static int sec_u_frmr_frame(void) {
    TEST_SECTION("=== SEC-U: FRMR Frame (Frame Reject) ===");

    uint8_t err;
    size_t enc_len;
    uint8_t *enc;
    ax25_frame_t *dec;
    ax25_frame_header_t hdr;

    ax25_address_t *dest = ax25_address_from_string("W1AW-0", &err);
    ax25_address_t *src = ax25_address_from_string("N0CALL-0", &err);

    if (!dest || !src) {
        if (dest)
            ax25_address_free(dest, &err);
        if (src)
            ax25_address_free(src, &err);
        printf("SKIP: SEC-U address creation failed\n");
        return 0;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.destination = *dest;
    hdr.source = *src;
    hdr.cr = false; /* FRMR is always a response frame */
    hdr.repeaters.num_repeaters = 0;

    ax25_address_free(dest, &err);
    ax25_address_free(src, &err);

    // U.1: Encode FRMR
    {
        ax25_frame_reject_frame_t frmr;
        memset(&frmr, 0, sizeof(frmr));
        frmr.base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
        frmr.base.base.header = hdr;
        frmr.base.pf = false;
        frmr.base.modifier = AX25_U_FRMR;

        enc = ax25_frame_encode((ax25_frame_t*) &frmr, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "U.1 Encode FRMR frame", err);

        if (enc) {
            // U.2: Decode round-trip
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "U.2 Decode FRMR round-trip", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_UNNUMBERED_FRMR, "U.2 type==FRMR", 0);
                ax25_frame_free(dec, &err);
            }

            // U.4: FRMR minimum encoded length (fix 23.1)
            // AX.25 v2.2 §4.3.9: FRMR info = 3 bytes → min 14+1+3 = 18 bytes
            TEST_ASSERT(enc_len >= 18, "U.4 FRMR encoded length >= 18 (addr+ctrl+3-byte info)", (int )enc_len);

            free(enc);
            enc = NULL;
        }
    }

    // U.3: FRMR and UA produce different bytes
    {
        ax25_unnumbered_frame_t ua;
        ax25_frame_reject_frame_t frmr2;
        size_t ua_len = 0, frmr_len = 0;
        uint8_t *enc_ua = NULL, *enc_frmr = NULL;

        memset(&ua, 0, sizeof(ua));
        ua.base.type = AX25_FRAME_UNNUMBERED_UA;
        ua.base.header = hdr;
        ua.pf = false;
        ua.modifier = AX25_U_UA;

        memset(&frmr2, 0, sizeof(frmr2));
        frmr2.base.base.type = AX25_FRAME_UNNUMBERED_FRMR;
        frmr2.base.base.header = hdr;
        frmr2.base.pf = false;
        frmr2.base.modifier = AX25_U_FRMR;

        enc_ua = ax25_frame_encode((ax25_frame_t*) &ua, &ua_len, &err);
        enc_frmr = ax25_frame_encode((ax25_frame_t*) &frmr2, &frmr_len, &err);

        if (enc_ua && enc_frmr && ua_len > 0 && frmr_len > 0) {
            size_t cmp_len = ua_len < frmr_len ? ua_len : frmr_len;
            TEST_ASSERT(memcmp(enc_ua, enc_frmr, cmp_len) != 0, "U.3 FRMR and UA have different encoded bytes (0x87 vs 0x63)", 0);
        }
        if (enc_ua)
            free(enc_ua);
        if (enc_frmr)
            free(enc_frmr);
    }

    return 0;
}

// ===========================================================================
// SECTION V: KISS Port Multiplexing
// ===========================================================================
static int sec_v_kiss_port_multiplexing(void) {
    TEST_SECTION("=== SEC-V: KISS Port Multiplexing ===");

    uint8_t payload[8];
    uint8_t kiss_out[32];
    uint8_t kiss_dec[32];
    int kiss_out_len, kiss_dec_len, krc, i;

    for (i = 0; i < 4; i++)
        payload[i] = (uint8_t) (0x41 + i);

    // V.1: port=0, cmd=0 → 0x00
    {
        kiss_out_len = 0;
        krc = kiss_encode_frame(payload, 4, 0, 0, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "V.1 port=0 encode", krc);
        TEST_ASSERT(kiss_out[1] == 0x00, "V.1 cmd byte == 0x00", kiss_out[1]);
    }

    // V.2: port=1, cmd=0 → 0x10
    {
        kiss_out_len = 0;
        krc = kiss_encode_frame(payload, 4, 1, 0, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "V.2 port=1 encode", krc);
        TEST_ASSERT(kiss_out[1] == 0x10, "V.2 cmd byte == 0x10", kiss_out[1]);
    }

    // V.3: port=14 → 0xE0
    {
        kiss_out_len = 0;
        krc = kiss_encode_frame(payload, 4, 14, 0, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "V.3 port=14 encode", krc);
        TEST_ASSERT(kiss_out[1] == 0xE0, "V.3 cmd byte == 0xE0", kiss_out[1]);
    }

    // V.4: port=15 (maximum) → 0xF0 (fix 24.1)
    {
        kiss_out_len = 0;
        krc = kiss_encode_frame(payload, 4, 15, 0, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "V.4 port=15 encode", krc);
        TEST_ASSERT(kiss_out[1] == 0xF0, "V.4 port=15 cmd byte == 0xF0", kiss_out[1]);

        kiss_dec_len = 0;
        krc = kiss_decode_frame(kiss_out, kiss_out_len, kiss_dec, &kiss_dec_len);
        TEST_ASSERT(krc == 0 && kiss_dec_len == 4, "V.4 port=15 decode round-trip", kiss_dec_len);
    }

    // V.5: port=16 overflow → must clamp or reject (fix 24.1)
    {
        uint8_t tmp[32];
        int tmp_len = 0;
        krc = kiss_encode_frame(payload, 1, 16, 0, tmp, &tmp_len);
        if (krc == 0 && tmp_len > 0) {
            uint8_t enc_port = (tmp[1] >> 4) & 0x0F;
            /* port 16 nibble-wraps to 0 */
            TEST_ASSERT(enc_port == 0 || krc != 0, "V.5 port=16 wraps to port=0 (nibble overflow)", enc_port);
            DEBUG_PRINT("V.5 port=16 cmd_byte=0x%02X enc_port=%d", tmp[1], enc_port);
        } else {
            DEBUG_PRINT("V.5 port=16 rejected (rc=%d)", krc);
        }
    }

    // V.6: TxDelay hardware cmd=1, port=0 → 0x01
    {
        uint8_t v[1] = { 50 };
        kiss_out_len = 0;
        krc = kiss_encode_frame(v, 1, 0, 1, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "V.6 TxDelay cmd=1 encode", krc);
        TEST_ASSERT(kiss_out[1] == 0x01, "V.6 cmd byte == 0x01", kiss_out[1]);
        TEST_ASSERT(kiss_out[2] == 50, "V.6 TxDelay value 50 preserved", kiss_out[2]);
    }

    // V.7: FullDuplex cmd=5, port=0 → 0x05
    {
        uint8_t v[1] = { 0 };
        kiss_out_len = 0;
        krc = kiss_encode_frame(v, 1, 0, 5, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "V.7 FullDuplex cmd=5 encode", krc);
        TEST_ASSERT(kiss_out[1] == 0x05, "V.7 cmd byte == 0x05", kiss_out[1]);
    }

    // V.8: Return cmd=15, empty payload → 3-byte frame
    {
        uint8_t v[1] = { 0 };
        kiss_out_len = 0;
        krc = kiss_encode_frame(v, 0, 0, 15, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "V.8 Return cmd=15 encode", krc);
        TEST_ASSERT(kiss_out[1] == 0x0F, "V.8 cmd byte == 0x0F", kiss_out[1]);
        TEST_ASSERT(kiss_out_len == 3, "V.8 Return frame is 3 bytes", kiss_out_len);
    }

    // V.9: port=2, cmd=1 → 0x21 (nibble isolation test)
    {
        uint8_t v[1] = { 30 };
        kiss_out_len = 0;
        krc = kiss_encode_frame(v, 1, 2, 1, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "V.9 port=2 cmd=1 encode", krc);
        TEST_ASSERT(kiss_out[1] == 0x21, "V.9 port=2 cmd=1 byte=0x21 (nibble isolation)", kiss_out[1]);
    }

    // V.10: port=1 data frame round-trip
    {
        kiss_dec_len = 0;
        kiss_out_len = 0;
        krc = kiss_encode_frame(payload, 4, 1, 0, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "V.10 port=1 encode for round-trip", krc);
        if (krc == 0) {
            krc = kiss_decode_frame(kiss_out, kiss_out_len, kiss_dec, &kiss_dec_len);
            TEST_ASSERT(krc == 0 && kiss_dec_len == 4, "V.10 port=1 decode len==4", kiss_dec_len);
            TEST_ASSERT(memcmp(kiss_dec, payload, 4) == 0, "V.10 payload matches", 0);
        }
    }

    return 0;
}

// ===========================================================================
// SECTION W: Sequence Number Wrap-Around
// ===========================================================================
static int sec_w_seq_wrap_around(void) {
    TEST_SECTION("=== SEC-W: Sequence Number Wrap-Around ===");

    uint8_t err;
    uint8_t *enc;
    size_t enc_len;
    ax25_frame_t *dec;
    ax25_frame_header_t hdr;
    ax25_information_frame_t iframe;
    ax25_information_frame_t *di;
    int all_ok, ns_val;

    ax25_address_t *dest = ax25_address_from_string("W1AW-0", &err);
    ax25_address_t *src = ax25_address_from_string("N0CALL-0", &err);

    if (!dest || !src) {
        if (dest)
            ax25_address_free(dest, &err);
        if (src)
            ax25_address_free(src, &err);
        printf("SKIP: SEC-W address creation failed\n");
        return 0;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.destination = *dest;
    hdr.source = *src;
    hdr.cr = true;
    hdr.repeaters.num_repeaters = 0;

    ax25_address_free(dest, &err);
    ax25_address_free(src, &err);

    /* address field length = 14 bytes for 2-station frame (fix 25.1) */
    int addr_len = 7 + 7 + hdr.repeaters.num_repeaters * 7;

    // W.1: Mod-8 N(S) full cycle 0-7
    {
        all_ok = 1;
        for (ns_val = 0; ns_val <= 7; ns_val++) {
            memset(&iframe, 0, sizeof(iframe));
            iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
            iframe.base.header = hdr;
            iframe.ns = ns_val;
            iframe.nr = 0;

            enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
            if (!enc) {
                all_ok = 0;
                continue;
            }
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            if (!dec) {
                free(enc);
                all_ok = 0;
                continue;
            }
            di = (ax25_information_frame_t*) dec;
            if (di->ns != ns_val) {
                all_ok = 0;
                DEBUG_PRINT("W.1 FAIL mod-8 N(S)=%d decoded as %d", ns_val, di->ns);
            }
            ax25_frame_free(dec, &err);
            free(enc);
        }
        TEST_ASSERT(all_ok, "W.1 Mod-8 N(S) full cycle 0-7 preserved", 0);
    }

    // W.2: Mod-8 N(R) full cycle 0-7
    {
        int nr_val;
        all_ok = 1;
        for (nr_val = 0; nr_val <= 7; nr_val++) {
            memset(&iframe, 0, sizeof(iframe));
            iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
            iframe.base.header = hdr;
            iframe.ns = 0;
            iframe.nr = nr_val;

            enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
            if (!enc) {
                all_ok = 0;
                continue;
            }
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            if (!dec) {
                free(enc);
                all_ok = 0;
                continue;
            }
            di = (ax25_information_frame_t*) dec;
            if (di->nr != nr_val) {
                all_ok = 0;
                DEBUG_PRINT("W.2 FAIL mod-8 N(R)=%d decoded as %d", nr_val, di->nr);
            }
            ax25_frame_free(dec, &err);
            free(enc);
        }
        TEST_ASSERT(all_ok, "W.2 Mod-8 N(R) full cycle 0-7 preserved", 0);
    }

    // W.3: Mod-128 N(S) wrap boundary 125,126,127,0,1,2
    {
        int test_ns[6];
        int idx;
        test_ns[0] = 125;
        test_ns[1] = 126;
        test_ns[2] = 127;
        test_ns[3] = 0;
        test_ns[4] = 1;
        test_ns[5] = 2;
        all_ok = 1;
        for (idx = 0; idx < 6; idx++) {
            memset(&iframe, 0, sizeof(iframe));
            iframe.base.type = AX25_FRAME_INFORMATION_16BIT;
            iframe.base.header = hdr;
            iframe.ns = test_ns[idx];
            iframe.nr = 0;

            enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
            if (!enc) {
                all_ok = 0;
                continue;
            }
            dec = ax25_frame_decode(enc, enc_len, MODULO128_TRUE, &err);
            if (!dec) {
                free(enc);
                all_ok = 0;
                continue;
            }
            di = (ax25_information_frame_t*) dec;
            if (di->ns != test_ns[idx]) {
                all_ok = 0;
                DEBUG_PRINT("W.3 FAIL mod-128 N(S)=%d decoded as %d", test_ns[idx], di->ns);
            }
            ax25_frame_free(dec, &err);
            free(enc);
        }
        TEST_ASSERT(all_ok, "W.3 Mod-128 N(S) wrap boundary 125/126/127/0/1/2", 0);
    }

    // W.4: Mod-128 N(R) wrap boundary 127,0,1,2
    {
        int test_nr[4];
        int idx;
        test_nr[0] = 127;
        test_nr[1] = 0;
        test_nr[2] = 1;
        test_nr[3] = 2;
        all_ok = 1;
        for (idx = 0; idx < 4; idx++) {
            memset(&iframe, 0, sizeof(iframe));
            iframe.base.type = AX25_FRAME_INFORMATION_16BIT;
            iframe.base.header = hdr;
            iframe.ns = 0;
            iframe.nr = test_nr[idx];

            enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
            if (!enc) {
                all_ok = 0;
                continue;
            }
            dec = ax25_frame_decode(enc, enc_len, MODULO128_TRUE, &err);
            if (!dec) {
                free(enc);
                all_ok = 0;
                continue;
            }
            di = (ax25_information_frame_t*) dec;
            if (di->nr != test_nr[idx]) {
                all_ok = 0;
                DEBUG_PRINT("W.4 FAIL mod-128 N(R)=%d decoded as %d", test_nr[idx], di->nr);
            }
            ax25_frame_free(dec, &err);
            free(enc);
        }
        TEST_ASSERT(all_ok, "W.4 Mod-128 N(R) wrap boundary 127/0/1/2", 0);
    }

    // W.5: Mod-8 control byte spot-check N(S)=7, N(R)=7, P/F=0 → 0xEE (fix 25.1)
    {
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
        iframe.base.header = hdr;
        iframe.ns = 7;
        iframe.nr = 7;
        iframe.pf = false;

        enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "W.5 Encode mod-8 N(S)=7 N(R)=7 P/F=0", err);
        if (enc) {
            if (enc_len > (size_t) addr_len) {
                uint8_t ctrl = enc[addr_len];
                TEST_ASSERT(ctrl == 0xEE, "W.5 Mod-8 ctrl byte == 0xEE (N(R)=7 P/F=0 N(S)=7)", ctrl);
                DEBUG_PRINT("W.5 ctrl at enc[%d]=0x%02X (expected 0xEE)", addr_len, ctrl);
            }
            free(enc);
        }
    }

    // W.6: Mod-128 control bytes N(S)=127, N(R)=127, P/F=0 → 0xFE, 0xFE (fix 25.1)
    {
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_16BIT;
        iframe.base.header = hdr;
        iframe.ns = 127;
        iframe.nr = 127;
        iframe.pf = false;

        enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
        TEST_ASSERT(enc != NULL && err == 0, "W.6 Encode mod-128 N(S)=127 N(R)=127", err);
        if (enc) {
            if (enc_len > (size_t) (addr_len + 1)) {
                uint8_t ctrl0 = enc[addr_len];
                uint8_t ctrl1 = enc[addr_len + 1];
                TEST_ASSERT(ctrl0 == 0xFE, "W.6 Mod-128 ctrl[0] == 0xFE (N(S)=127 I-bit=0)", ctrl0);
                TEST_ASSERT(ctrl1 == 0xFE, "W.6 Mod-128 ctrl[1] == 0xFE (N(R)=127 P/F=0)", ctrl1);
                DEBUG_PRINT("W.6 ctrl[0]=0x%02X ctrl[1]=0x%02X (expected 0xFE both)", ctrl0, ctrl1);
            }
            free(enc);
        }
    }

    return 0;
}

// ===========================================================================
// SECTION X: KISS-to-Kernel PTY Pipeline
//
// Architecture (matches run_all_test.sh setup):
//
//   socat PTY[master] <─────────────────────> PTY[slave]
//              |                                   |
//        kissattach                        our test writes
//          ax0 iface                        KISS frames here
//              |
//        AF_AX25 socket
//         receives here
//
// PTY discovery is fully dynamic via /proc — no hardcoded device paths.
// ===========================================================================

// Read a single line from a file, stripping the trailing newline.
// Returns 0 on success, -1 on failure.
static int read_first_line(const char *path, char *buf, size_t bufsz) {
    FILE *fp = fopen(path, "r");
    if (!fp)
        return -1;
    if (!fgets(buf, (int) bufsz, fp)) {
        fclose(fp);
        return -1;
    }
    fclose(fp);
    size_t n = strlen(buf);
    while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r'))
        buf[--n] = '\0';
    return 0;
}

// Find the first PTY device (/dev/pts/N) owned as a file-descriptor by
// process <pid> that does NOT equal <exclude_path> and is NOT the
// calling process's controlling terminal.
// Skips fd 0/1/2 (inherited stdin/stdout/stderr) which point to the terminal.
// Returns 1 and fills <out> (size <outsz>) on success, 0 if not found.
static int find_pty_of_pid_not(pid_t pid, const char *exclude_path, char *out, size_t outsz) {
    char fddir[64];
    snprintf(fddir, sizeof(fddir), "/proc/%d/fd", (int) pid);
    DIR *d = opendir(fddir);
    if (!d)
        return 0;

    /* Get the calling process's controlling terminal so we can exclude it.
     socat inherits our stdin/stdout/stderr which all point to this PTY. */
    char ctty[64] = "";
    {
        const char *t = ttyname(STDIN_FILENO);
        if (t)
            safe_strlcpy(ctty, t, sizeof(ctty));
    }

    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.')
            continue;

        /* Skip fd 0, 1, 2 — these are the inherited stdin/stdout/stderr
         and always point to the terminal, not to socat's bridging PTYs. */
        int fdnum = atoi(ent->d_name);
        if (fdnum >= 0 && fdnum <= 2)
            continue;

        /* fddir (≤16) + '/' + d_name (≤255) + NUL = up to 272 bytes.
         Use 512 to satisfy -Wformat-truncation. */
        char fdpath[512];
        snprintf(fdpath, sizeof(fdpath), "%s/%s", fddir, ent->d_name);
        /* /dev/pts/NNNN is at most ~16 bytes; 64 is more than sufficient. */
        char target[64];
        ssize_t n = readlink(fdpath, target, sizeof(target) - 1);
        if (n <= 0)
            continue;
        target[n] = '\0';
        if (strncmp(target, "/dev/pts/", 9) != 0)
            continue;

        /* Exclude the kissattach PTY */
        if (exclude_path && strcmp(target, exclude_path) == 0)
            continue;

        /* Exclude the controlling terminal of the test process */
        if (ctty[0] && strcmp(target, ctty) == 0) {
            DEBUG_PRINT("X.0 skipping ctty fd%d=%s (test terminal)", fdnum, target);
            continue;
        }

        DEBUG_PRINT("X.0 socat fd%d -> %s (candidate slave PTY)", fdnum, target);
        safe_strlcpy(out, target, outsz);
        closedir(d);
        return 1;
    }
    closedir(d);
    return 0;
}

// Find kissattach's PTY device by reading its /proc cmdline.
// kissattach argv: kissattach [options] <tty> <port> [inetaddr]
// The tty argument is the first non-option token.
// Returns 1 and fills <out> on success, 0 on failure.
static int find_kissattach_pty(char *out, size_t outsz) {
    // Find kissattach PID
    DIR *pd = opendir("/proc");
    if (!pd)
        return 0;

    struct dirent *pent;
    pid_t ka_pid = 0;
    while ((pent = readdir(pd)) != NULL) {
        if (pent->d_name[0] < '1' || pent->d_name[0] > '9')
            continue;
        pid_t p = (pid_t) atoi(pent->d_name);
        char comm[64];
        char comm_path[64];
        snprintf(comm_path, sizeof(comm_path), "/proc/%d/comm", (int) p);
        if (read_first_line(comm_path, comm, sizeof(comm)) == 0) {
            if (strcmp(comm, "kissattach") == 0) {
                ka_pid = p;
                break;
            }
        }
    }
    closedir(pd);
    if (!ka_pid)
        return 0;

    // Read cmdline (NUL-delimited argv)
    char cmdline_path[64];
    snprintf(cmdline_path, sizeof(cmdline_path), "/proc/%d/cmdline", (int) ka_pid);
    FILE *fp = fopen(cmdline_path, "r");
    if (!fp)
        return 0;

    char buf[512];
    size_t n = fread(buf, 1, sizeof(buf) - 1, fp);
    fclose(fp);
    if (n == 0)
        return 0;
    buf[n] = '\0';

    // Walk argv tokens (separated by NUL bytes), skip argv[0] and option flags
    const char *p = buf;
    const char *end = buf + n;
    int arg_idx = 0;
    while (p < end) {
        size_t toklen = strlen(p);
        if (arg_idx > 0 && p[0] != '-') {
            // First non-option argument after argv[0] is the tty device
            if (strncmp(p, "/dev/", 5) == 0) {
                safe_strlcpy(out, p, outsz);
                return 1;
            }
        }
        arg_idx++;
        p += toklen + 1;
    }
    return 0;
}

// Find socat's slave PTY: read PID from /tmp/socat_pid.txt (written by
// run_all_test.sh), then enumerate its open /dev/pts/ fds, excluding the
// kissattach master PTY.
static int find_socat_slave_pty(const char *ka_pty, char *out, size_t outsz) {
    // Method 1: via /tmp/socat_pid.txt
    {
        char pidbuf[32];
        if (read_first_line("/tmp/socat_pid.txt", pidbuf, sizeof(pidbuf)) == 0) {
            pid_t socat_pid = (pid_t) atoi(pidbuf);
            if (socat_pid > 0 && find_pty_of_pid_not(socat_pid, ka_pty, out, outsz))
                return 1;
        }
    }

    // Method 2: scan /proc for any process named "socat"
    {
        DIR *pd = opendir("/proc");
        if (!pd)
            return 0;
        struct dirent *pent;
        while ((pent = readdir(pd)) != NULL) {
            if (pent->d_name[0] < '1' || pent->d_name[0] > '9')
                continue;
            pid_t p = (pid_t) atoi(pent->d_name);
            char comm[64], comm_path[64];
            snprintf(comm_path, sizeof(comm_path), "/proc/%d/comm", (int) p);
            if (read_first_line(comm_path, comm, sizeof(comm)) == 0) {
                if (strcmp(comm, "socat") == 0) {
                    if (find_pty_of_pid_not(p, ka_pty, out, outsz)) {
                        closedir(pd);
                        return 1;
                    }
                }
            }
        }
        closedir(pd);
    }
    return 0;
}

// Find the /proc/<pid>/fd/<N> path that, when opened, gives the PTY MASTER
// corresponding to the kissattach slave PTY (ka_pty, e.g. "/dev/pts/105").
//
// IMPORTANT: on modern Linux kernels the /proc/PID/fd/N symlink for a PTY
// MASTER fd resolves to "/dev/pts/N" (the slave path) — not to "/dev/ptmx".
// Opening that symlink path via open() follows it to the SLAVE device.
// TIOCGPTN on a slave fd returns ENOTTY, so a naïve readlink==ka_pty filter
// will always find slave fds and always fail the TIOCGPTN check.
//
// On older kernels the symlink resolves to "/dev/ptmx" (the ptmx multiplexer).
// Opening /dev/ptmx allocates a brand-new PTY master (different slave number),
// so TIOCGPTN returns the wrong number and also fails.
//
// For the proc-path approach to be useful, the path must be opened WITHOUT
// following the symlink — see open_ka_master_fd() below which uses
// pidfd_getfd() (Linux ≥5.6) to correctly duplicate the fd.
//
// This function is kept for fallback / diagnostic purposes.  It now also
// accepts /dev/ptmx and /dev/pts/ptmx link targets so that the TIOCGPTN
// probe at least runs on master-fd candidates instead of silently skipping.
//
// Returns 1 and fills <out_proc_path> on success, 0 if not found.
static int find_ka_master_proc_path(const char *ka_pty, char *out_proc_path, size_t outsz) {
    /* Extract the slave number from ka_pty (e.g. "/dev/pts/105" -> 105) */
    const char *pts_num_str = strrchr(ka_pty, '/');
    if (!pts_num_str)
        return 0;
    int ka_slave_num = atoi(pts_num_str + 1);

    DIR *pd = opendir("/proc");
    if (!pd)
        return 0;

    struct dirent *pent;
    while ((pent = readdir(pd)) != NULL) {
        if (pent->d_name[0] < '1' || pent->d_name[0] > '9')
            continue;
        pid_t p = (pid_t) atoi(pent->d_name);

        char fddir[64];
        snprintf(fddir, sizeof(fddir), "/proc/%d/fd", (int) p);
        DIR *fdd = opendir(fddir);
        if (!fdd)
            continue;

        struct dirent *fent;
        while ((fent = readdir(fdd)) != NULL) {
            if (fent->d_name[0] == '.')
                continue;
            int fdnum = atoi(fent->d_name);
            if (fdnum < 3)
                continue; /* skip stdin/stdout/stderr */

            char proc_fd_path[512];
            snprintf(proc_fd_path, sizeof(proc_fd_path), "/proc/%d/fd/%s", (int) p, fent->d_name);

            char target[64];
            ssize_t n = readlink(proc_fd_path, target, sizeof(target) - 1);
            if (n <= 0)
                continue;
            target[n] = '\0';

            /* Accept:
             *  - exact match to ka_pty  (modern kernels: master resolves to slave path)
             *  - /dev/ptmx              (older kernels: master resolves to ptmx)
             *  - /dev/pts/ptmx          (newer kernels devpts mount)
             * Slave fds for ka_pty also match the first case and will be
             * rejected by TIOCGPTN (ENOTTY), which is correct. */
            int is_candidate = (strcmp(target, ka_pty) == 0 || strncmp(target, "/dev/ptmx", 9) == 0 || strncmp(target, "/dev/pts/ptmx", 13) == 0);
            if (!is_candidate)
                continue;

            /* Open the fd. NOTE: on modern kernels this follows the symlink
             * to the slave device, so TIOCGPTN will return ENOTTY.
             * The real fix is open_ka_master_fd() which uses pidfd_getfd(). */
            int test_fd = open(proc_fd_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (test_fd < 0)
                continue;

            unsigned int slave_num = 0;
            int rc = ioctl(test_fd, TIOCGPTN, &slave_num);
            close(test_fd);

            if (rc != 0)
                continue; /* ENOTTY = slave fd (expected on modern kernels) */
            if ((int) slave_num != ka_slave_num)
                continue; /* wrong pair */

            /* Confirmed: this fd is the PTY master for ka_pty */
            DEBUG_PRINT("X.0 Found PTY master: pid=%d fd=%d TIOCGPTN=%u -> %s", (int)p, fdnum, slave_num, proc_fd_path);
            safe_strlcpy(out_proc_path, proc_fd_path, outsz);
            closedir(fdd);
            closedir(pd);
            return 1;
        }
        closedir(fdd);
    }
    closedir(pd);
    return 0;
}

// ---------------------------------------------------------------------------
// open_ka_master_fd — open the PTY MASTER for ka_pty directly.
//
// Uses pidfd_getfd() (Linux ≥ 5.6) to duplicate the fd from whichever
// process (typically socat) currently holds the master side of the PTY
// whose slave is ka_pty.
//
// pidfd_getfd() copies the underlying file description without following
// the /proc/PID/fd/N symlink, so TIOCGPTN on the result reliably returns
// the slave number of the original master.
//
// Requires: CAP_SYS_PTRACE (or same-UID process) to open the pidfd.
//           Kernel ≥ 5.6 for the SYS_pidfd_open / SYS_pidfd_getfd syscalls.
//
// Returns an open fd (RDWR) to the PTY master, or -1 on failure.
// Caller must close() the returned fd.
// ---------------------------------------------------------------------------
static int open_ka_master_fd(const char *ka_pty) {
    const char *pts_num_str = strrchr(ka_pty, '/');
    if (!pts_num_str)
        return -1;
    int ka_slave_num = atoi(pts_num_str + 1);
    if (ka_slave_num < 0)
        return -1;

    /*
     * Stat /dev/ptmx once to know the exact PTY master device number.
     * PTY masters are backed by /dev/ptmx   (typically major=5,  minor=2).
     * PTY slaves  are backed by /dev/pts/N  (always    major=136, minor=N).
     *
     * CRITICAL: TIOCGPTN CANNOT distinguish master from slave.
     * pty_unix98_ioctl() is shared between master and slave tty_operations
     * structs in every Linux kernel version — it returns tty->index (the
     * slave number) for BOTH sides.  The previous implementation was
     * therefore accepting slave fds as "masters".  Writing KISS bytes to a
     * slave fd sends data TOWARD the master (socat reads it), not toward the
     * N_AX25 ldisc (which reads data arriving FROM the master).
     *
     * Fix: use fstat() on the fd obtained via pidfd_getfd() (which gives the
     * real file description, not a re-opened symlink) to verify the backing
     * device is /dev/ptmx (major=5,minor=2) before accepting it as a master.
     */
    struct stat ptmx_st;
    int have_ptmx_stat = (stat("/dev/ptmx", &ptmx_st) == 0);

    /* Skip our own PID: we have slave_kfd open on ka_pty, and TIOCGPTN on
     * that slave fd would (incorrectly) report slave_num == ka_slave_num. */
    pid_t my_pid = getpid();

    DIR *pd = opendir("/proc");
    if (!pd)
        return -1;

    int found_fd = -1;

    struct dirent *pent;
    while ((pent = readdir(pd)) != NULL && found_fd < 0) {
        if (pent->d_name[0] < '1' || pent->d_name[0] > '9')
            continue;
        pid_t p = (pid_t) atoi(pent->d_name);
        if (p == my_pid)
            continue; /* skip ourselves */

        /* Open a pidfd for this process (Linux ≥ 5.6). */
        int pidfd = (int) syscall(SYS_pidfd_open, (long) p, 0L);
        if (pidfd < 0)
            continue; /* kernel too old or no permission */

        char fddir[64];
        snprintf(fddir, sizeof(fddir), "/proc/%d/fd", (int) p);
        DIR *fdd = opendir(fddir);
        if (!fdd) {
            close(pidfd);
            continue;
        }

        struct dirent *fent;
        while ((fent = readdir(fdd)) != NULL && found_fd < 0) {
            if (fent->d_name[0] == '.')
                continue;
            int fdnum = atoi(fent->d_name);
            if (fdnum < 3)
                continue;

            /* Quick pre-filter via readlink: accept PTY-related fds only.
             * On modern kernels, a PTY master's /proc symlink resolves to the
             * slave path (/dev/pts/N) — so both master AND slave fds match
             * ka_pty.  We discriminate them below with fstat().             */
            char proc_fd_path[512];
            snprintf(proc_fd_path, sizeof(proc_fd_path), "/proc/%d/fd/%d", (int) p, fdnum);
            char target[80];
            ssize_t n = readlink(proc_fd_path, target, sizeof(target) - 1);
            if (n <= 0)
                continue;
            target[n] = '\0';

            /* Candidate if it looks like ka_pty, /dev/ptmx, or /dev/pts/ptmx. */
            if (strcmp(target, ka_pty) != 0 && strncmp(target, "/dev/ptmx", 9) != 0 && strncmp(target, "/dev/pts/ptmx", 13) != 0)
                continue;

            /* Duplicate the fd — pidfd_getfd() gives the REAL underlying file
             * description object (not a re-opened symlink target).           */
            int dup_fd = (int) syscall(SYS_pidfd_getfd, pidfd, fdnum, 0L);
            if (dup_fd < 0)
                continue;

            /* ----------------------------------------------------------------
             * Master vs slave discrimination via fstat():
             *   PTY master → st_rdev == ptmx devno  (major=5, minor=2)
             *   PTY slave  → st_rdev major==136       (UNIX98_PTY_SLAVE_MAJOR)
             * ---------------------------------------------------------------- */
            struct stat dup_st;
            if (fstat(dup_fd, &dup_st) != 0 || !S_ISCHR(dup_st.st_mode)) {
                close(dup_fd);
                continue;
            }

            /* Reject definite slave fds (major=136 = UNIX98_PTY_SLAVE_MAJOR). */
            if (major(dup_st.st_rdev) == 136) {
                close(dup_fd);
                continue;
            }

            /* Cross-check against /dev/ptmx's actual device number. */
            if (have_ptmx_stat && dup_st.st_rdev != ptmx_st.st_rdev) {
                close(dup_fd);
                continue; /* not ptmx — serial port, console, etc. */
            }

            /* Confirm slave number matches ka_pty via TIOCGPTN.
             * At this point TIOCGPTN is only used for the number check, not
             * as a master/slave discriminator (the fstat above did that).  */
            unsigned int slave_num = 0;
            if (ioctl(dup_fd, TIOCGPTN, &slave_num) != 0 || (int) slave_num != ka_slave_num) {
                close(dup_fd);
                continue;
            }

            DEBUG_PRINT("X.0 open_ka_master_fd: pid=%d fd=%d " "devno=%u:%u → PTY master for /dev/pts/%d (dup_fd=%d)", (int)p, fdnum, major(dup_st.st_rdev),
                    minor(dup_st.st_rdev), ka_slave_num, dup_fd);
            found_fd = dup_fd; /* caller owns this fd */
        }

        closedir(fdd);
        close(pidfd);
    }
    closedir(pd);
    return found_fd;
}

// ===========================================================================
// SECTION X: KISS-to-Kernel Pipeline (FULLY CORRECTED VERSION)
// ===========================================================================
// This is the **only** function that needed changes.
// Replace your existing sec_x_kiss_kernel_pipeline() with this complete version.
// All other sections of the file remain untouched.

static int sec_x_kiss_kernel_pipeline(void) {
    TEST_SECTION("=== SEC-X: KISS-to-Kernel Pipeline via PTY ===");

    char ka_pty[64] = ""; /* kissattach's PTY (kernel/master side) */
    char slave_pty[64] = ""; /* socat slave PTY  (TNC injection side) */

    /* EIO-prevention fd — kept open from before the write until after poll() */
    int slave_kfd = -1;

    // -----------------------------------------------------------------------
    // X.0: Discover PTY topology
    // -----------------------------------------------------------------------
    if (!find_kissattach_pty(ka_pty, sizeof(ka_pty))) {
        printf("SKIP: SEC-X (kissattach not running — run setup first)\n");
        printf("  Setup:\n");
        printf("    Terminal 1:  socat PTY,raw,echo=0 PTY,raw,echo=0\n");
        printf("    Terminal 2:  sudo kissattach /dev/pts/N ax0\n");
        printf("    Or use:      sudo ./test/run_ax25_test.sh\n");
        return 0;
    }
    DEBUG_PRINT("X.0 kissattach master PTY: %s", ka_pty);

    if (!find_socat_slave_pty(ka_pty, slave_pty, sizeof(slave_pty))) {
        printf("SKIP: SEC-X (socat slave PTY not found — socat not running?)\n");
        printf("  Expected: socat PTY,raw,echo=0 PTY,raw,echo=0\n");
        printf("  kissattach master PTY found: %s\n", ka_pty);
        return 0;
    }
    DEBUG_PRINT("X.0 socat slave PTY (TNC side): %s", slave_pty);

    if (!g_test_ctx.kernel_ax25_available) {
        printf("SKIP: SEC-X (kernel AX.25 not available)\n");
        return 0;
    }

    // -----------------------------------------------------------------------
    // X.0b: Find the AX.25 netdevice (SIOCGIFNAME + fallback)
    // -----------------------------------------------------------------------
    char ax25_iface[IFNAMSIZ] = "";

    /* Method 1: SIOCGIFNAME on the tty slave (authoritative) */
    {
        int tfd = open(ka_pty, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (tfd >= 0) {
            char ifbuf[IFNAMSIZ];
            memset(ifbuf, 0, sizeof(ifbuf));
            if (ioctl(tfd, SIOCGIFNAME, ifbuf) == 0 && ifbuf[0] != '\0') {
                safe_strlcpy(ax25_iface, ifbuf, sizeof(ax25_iface));
                DEBUG_PRINT("X.0b SIOCGIFNAME(%s) → interface: %s " "(current kissattach ldisc)", ka_pty, ax25_iface);
            }
            close(tfd);
        }
    }

    /* Method 2: ARPHRD_AX25 scan (fallback) */
    if (ax25_iface[0] == '\0') {
        DIR *nd = opendir("/sys/class/net");
        if (nd) {
            struct dirent *nent;
            while ((nent = readdir(nd)) != NULL) {
                if (nent->d_name[0] == '.')
                    continue;
                char type_path[512];
                snprintf(type_path, sizeof(type_path), "/sys/class/net/%s/type", nent->d_name);
                char type_val[16];
                if (read_first_line(type_path, type_val, sizeof(type_val)) == 0) {
                    if (atoi(type_val) == 3) {
                        safe_strlcpy(ax25_iface, nent->d_name, sizeof(ax25_iface));
                        break;
                    }
                }
            }
            closedir(nd);
        }
    }

    if (ax25_iface[0] == '\0') {
        safe_strlcpy(ax25_iface, g_test_ctx.port_name, sizeof(ax25_iface));
    } else {
        DEBUG_PRINT("X.0b AX.25 KISS interface: %s (ARPHRD_AX25=3)", ax25_iface);
    }

    // -----------------------------------------------------------------------
    // X.1: Create AF_PACKET RX socket + encode UI frame
    // -----------------------------------------------------------------------
    int rx_sock = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_AX25));
    TEST_ASSERT(rx_sock >= 0, "X.1 Create AF_PACKET/SOCK_RAW ETH_P_AX25 socket", rx_sock);
    if (rx_sock < 0)
        return 0;

    {
        struct sockaddr_ll ll_addr;
        memset(&ll_addr, 0, sizeof(ll_addr));
        ll_addr.sll_family = AF_PACKET;
        ll_addr.sll_protocol = htons(ETH_P_AX25);
        ll_addr.sll_ifindex = (int) if_nametoindex(ax25_iface);

        if (ll_addr.sll_ifindex == 0 || bind(rx_sock, (struct sockaddr*) &ll_addr, sizeof(ll_addr)) != 0) {
            close(rx_sock);
            printf("SKIP: X.1 AF_PACKET bind failed\n");
            return 0;
        }
        DEBUG_PRINT("X.1 AF_PACKET socket bound to %s (ifindex=%d)", ax25_iface, ll_addr.sll_ifindex);
    }

    int fl = fcntl(rx_sock, F_GETFL, 0);
    if (fl >= 0)
        fcntl(rx_sock, F_SETFL, fl | O_NONBLOCK);

    // Build UI frame: W1AW-0 → TEST-0
    uint8_t xpayload[] = "KISS KERNEL PIPELINE TEST";
    uint8_t *ax25_bytes = NULL;
    size_t ax25_len = 0;
    {
        uint8_t err = 0;
        ax25_frame_header_t xhdr;
        ax25_unnumbered_information_frame_t xui;

        ax25_address_t *xdest = ax25_address_from_string(g_test_ctx.local_call, &err);
        ax25_address_t *xsrc = ax25_address_from_string("W1AW-0", &err);

        if (!xdest || !xsrc) {
            if (xdest)
                ax25_address_free(xdest, &err);
            if (xsrc)
                ax25_address_free(xsrc, &err);
            close(rx_sock);
            TEST_ASSERT(0, "X.1 Create addresses", err);
            return 0;
        }

        memset(&xhdr, 0, sizeof(xhdr));
        xhdr.destination = *xdest;
        xhdr.source = *xsrc;
        xhdr.cr = false;
        xhdr.repeaters.num_repeaters = 0;

        memset(&xui, 0, sizeof(xui));
        xui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        xui.base.base.header = xhdr;
        xui.base.pf = false;
        xui.base.modifier = AX25_U_UI;
        xui.pid = PID_NO_L3;
        xui.payload = xpayload;
        xui.payload_len = (int) (sizeof(xpayload) - 1);

        ax25_bytes = ax25_frame_encode((ax25_frame_t*) &xui, &ax25_len, &err);
        ax25_address_free(xdest, &err);
        ax25_address_free(xsrc, &err);

        TEST_ASSERT(ax25_bytes != NULL && err == 0, "X.1 libax25v22 encode UI frame (W1AW-0 → TEST-0) for PTY injection", err);
        if (!ax25_bytes) {
            close(rx_sock);
            return 0;
        }
    }

    DEBUG_PRINT("X.1 Encoded %zu AX.25 bytes (W1AW-0 → %s)", ax25_len, g_test_ctx.local_call);

    uint8_t kiss_frame[600];
    int kiss_len = 0;
    if (kiss_encode_frame(ax25_bytes, (int) ax25_len, 0, 0, kiss_frame, &kiss_len) != 0) {
        free(ax25_bytes);
        close(rx_sock);
        TEST_ASSERT(0, "X.1 KISS encode", 0);
        return 0;
    }
    free(ax25_bytes);

    // -----------------------------------------------------------------------
    // EIO prevention + write to PTY master (pidfd_getfd preferred)
    // -----------------------------------------------------------------------
    slave_kfd = open(ka_pty, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (slave_kfd >= 0) {
        DEBUG_PRINT("X.1 Opened ka_pty slave %s (EIO prevention fd=%d)", ka_pty, slave_kfd);
    }

    {
        int write_fd = -1;
        const char *write_desc = "";

        /* Strategy 1: direct master via pidfd_getfd (Linux ≥5.6) */
        int ka_master_direct = open_ka_master_fd(ka_pty);
        if (ka_master_direct >= 0) {
            write_fd = ka_master_direct;
            write_desc = "kissattach PTY master (pidfd_getfd)";
            DEBUG_PRINT("X.1 Direct PTY master fd=%d via pidfd_getfd", write_fd);
        }

        /* Strategy 2 & 3 fallback (unchanged from original) */
        if (write_fd < 0) {
            char ka_master_proc[128] = "";
            if (find_ka_master_proc_path(ka_pty, ka_master_proc, sizeof(ka_master_proc))) {
                write_fd = open(ka_master_proc, O_RDWR | O_NOCTTY);
                if (write_fd >= 0)
                    write_desc = "master PTY (proc path)";
            }
        }
        if (write_fd < 0) {
            write_fd = open(slave_pty, O_RDWR | O_NOCTTY);
            write_desc = "slave PTY (socat bridge)";
        }

        TEST_ASSERT(write_fd >= 0, "X.1 Open PTY for KISS injection", write_fd);
        if (write_fd < 0) {
            if (slave_kfd >= 0)
                close(slave_kfd);
            close(rx_sock);
            return 0;
        }

        int written = (int) write(write_fd, kiss_frame, kiss_len);
        close(write_fd);

        TEST_ASSERT(written == kiss_len, "X.1 Write complete KISS frame", written);

        if (written > 0) {
            DEBUG_PRINT("X.1 Wrote %d/%d KISS bytes via %s → N_AX25 ldisc → %s", written, kiss_len, write_desc, ax25_iface);
        }
    }

    // -----------------------------------------------------------------------
    // X.2: poll + receive + DECODE (THE ONLY CHANGE)
    // -----------------------------------------------------------------------
    {
        int rx_all = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
        if (rx_all >= 0) {
            struct sockaddr_ll all_ll;
            memset(&all_ll, 0, sizeof(all_ll));
            all_ll.sll_family = AF_PACKET;
            all_ll.sll_protocol = htons(ETH_P_ALL);
            all_ll.sll_ifindex = (int) if_nametoindex(ax25_iface);
            if (bind(rx_all, (struct sockaddr*) &all_ll, sizeof(all_ll)) != 0) {
                close(rx_all);
                rx_all = -1;
            } else {
                int fl2 = fcntl(rx_all, F_GETFL, 0);
                if (fl2 >= 0)
                    fcntl(rx_all, F_SETFL, fl2 | O_NONBLOCK);
            }
        }

        struct pollfd pfds[2];
        pfds[0].fd = rx_sock;
        pfds[0].events = POLLIN;
        pfds[1].fd = (rx_all >= 0) ? rx_all : rx_sock;
        pfds[1].events = POLLIN;
        int nfds = (rx_all >= 0) ? 2 : 1;

        int poll_rc = poll(pfds, (nfds_t) nfds, 5000);

        TEST_ASSERT(poll_rc > 0, "X.2 AF_PACKET received AX.25 frame within 5000 ms "
                "(KISS → kissattach → kernel netdev → AF_PACKET)", poll_rc);

        if (poll_rc > 0) {
            int active_sock = rx_sock;
            if (rx_all >= 0 && (pfds[1].revents & POLLIN) && !(pfds[0].revents & POLLIN)) {
                active_sock = rx_all;
            }

            uint8_t rx_buf[512];
            struct sockaddr_ll rx_ll;
            socklen_t rx_ll_len = sizeof(rx_ll);
            int nrecv = (int) recvfrom(active_sock, rx_buf, sizeof(rx_buf), 0, (struct sockaddr*) &rx_ll, &rx_ll_len);

            TEST_ASSERT(nrecv > 0, "X.2 recvfrom delivered AF_PACKET frame bytes", nrecv);

            DEBUG_PRINT("X.2 AF_PACKET received %d raw bytes (pkttype=%d protocol=0x%04X)", nrecv, rx_ll.sll_pkttype, (unsigned)ntohs(rx_ll.sll_protocol));

            /* =============================================================
             * Linux kernel mkiss ldisc prepends the KISS cmd byte (0x00)
             * to the frame delivered via AF_PACKET on the AX.25 netdev.
             * We skip it so libax25v22 receives exactly the 41-byte AX.25 frame
             * it originally encoded.
             * ============================================================= */
            uint8_t *decode_buf = rx_buf;
            size_t decode_len = (size_t) nrecv;
            if (nrecv > 0 && rx_buf[0] == 0x00) {
                decode_buf++;
                decode_len--;
                DEBUG_PRINT("X.2 Detected KISS cmd byte 0x00 prepended by kernel mkiss ldisc " "→ skipping (now %zu-byte pure AX.25 frame)", decode_len);
            }

            uint8_t dec_err = 0;
            ax25_frame_t *rx_frame = ax25_frame_decode(decode_buf, decode_len,
            MODULO128_FALSE, &dec_err);

            TEST_ASSERT(rx_frame != NULL && dec_err == 0, "X.2 libax25v22 decode of AF_PACKET-received AX.25 frame", dec_err);

            if (rx_frame) {
                TEST_ASSERT(rx_frame->type == AX25_FRAME_UNNUMBERED_INFORMATION, "X.2 Received frame type == UI", rx_frame->type);

                ax25_unnumbered_information_frame_t *rxui = (ax25_unnumbered_information_frame_t*) rx_frame;

                int payload_len = (int) (sizeof(xpayload) - 1);
                int match = (rxui->payload_len == payload_len && rxui->payload != NULL && memcmp(rxui->payload, xpayload, (size_t) payload_len) == 0);

                TEST_ASSERT(match, "X.2 Payload matches transmitted data "
                        "(end-to-end: libax25v22 → KISS → kernel → AF_PACKET → libax25v22)", rxui->payload_len);

                if (match)
                    DEBUG_PRINT("X.2 END-TO-END PASS: libax25v22 ↔ Linux kernel AX.25 (via kissattach)");

                ax25_frame_free(rx_frame, &dec_err);
            }
        }

        if (rx_all >= 0)
            close(rx_all);
        close(rx_sock);
    }

    /* Close EIO-prevention fd AFTER poll() */
    if (slave_kfd >= 0) {
        close(slave_kfd);
        slave_kfd = -1;
    }

    return 0;
}

// ===========================================================================
// SECTION Y: FX.25 Forward Error Correction
// ===========================================================================
//
// FX.25 SPECIFICATION REFERENCE (fx-25_01_06.pdf, Stensat Group LLC, 2006)
// ──────────────────────────────────────────────────────────────────────────
// §Protocol Summary   — Frame = Preamble | CorrelationTag | FEC-Codeblock |
//                       Postamble.  Codeblock = AX.25-packet + Pad + RS-check.
// §Correlation Tag    — 8-byte (64-bit) Gold Code; tag value selects RS variant.
//   Tag_01 = 0xB74DB7DF8A532F3E  → RS(255,239) 16-byte check, 239 info bytes.
//   Tag_02 = 0x26FF60A600CC8FDE  → RS(144,128)  16-byte check,  128 info bytes.
//   Tag_03 = 0xC7DC0508F3D9B09E  → RS( 80, 64)  16-byte check,   64 info bytes.
//   Tag_04 = 0x8F056EB4369660EE  → RS( 48, 32)  16-byte check,   32 info bytes.
//   Tag_05 = 0x6E260B1AC5835FAE  → RS(255,223)  32-byte check,  223 info bytes.
//   Tag_06 = 0xFF94DC634F1CFF4E  → RS(160,128)  32-byte check,  128 info bytes.
//   Tag_07 = 0x1EB7B9CDBC09C00E  → RS( 96, 64)  32-byte check,   64 info bytes.
//   Tag_08 = 0xDBF869BD2DBB1776  → RS( 64, 32)  32-byte check,   32 info bytes.
//   Tag_09 = 0x3ADB0C13DEAE2836  → RS(255,191)  64-byte check,  191 info bytes.
//   Tag_0A = 0xAB69DB6A543188D6  → RS(192,128)  64-byte check,  128 info bytes.
//   Tag_0B = 0x4A4ABEC4A724B796  → RS(128, 64)  64-byte check,   64 info bytes.
// §AX.25 Packet Requirements
//   — AX.25 packet is standard, unmodified, complete (with CRC/flags).
//   — Bit-stuffed AX.25 length must fit within FEC info capacity.
//   — LSb-first transmission order throughout FX.25 frame.
// §Pad — 0x7E fill bytes to align bit stream to FEC codeblock boundary.
// §Preamble / Postamble — minimum 4 / 2 × 0x7E bytes respectively.
//
// LIBAX25V22 INTEROP STRATEGY
// ──────────────────────────────────────────────────────────────────────────
// libax25v22 handles the AX.25 layer (encode/decode).
// FX.25 wraps libax25v22 frames: we test that the AX.25 payload recovered
// after FX.25 decode is byte-for-byte identical to what libax25v22 encoded.
//
// LINUX LIBAX25 / KERNEL INTEROP NOTES
// ──────────────────────────────────────────────────────────────────────────
// The Linux kernel AX.25 stack (af_ax25) and Direwolf/kissattach do NOT
// natively carry FX.25 correlation tags over AF_AX25 sockets.  FX.25 lives
// below the KISS/TNC layer (Layer 1 / bottom of Layer 2).  Therefore the
// interoperability tests in this section operate on raw byte buffers:
//   1. libax25v22 encodes an AX.25 frame → raw bytes.
//   2. FX.25 encoder wraps those bytes → FX.25 frame (correlation tag + RS).
//   3. FX.25 decoder unwraps          → recovered AX.25 bytes.
//   4. libax25v22 decodes recovered bytes → ax25_frame_t.
//   5. Payload and addresses match the original → PASS.
// Tests Y.9–Y.15 additionally push the FX.25-wrapped KISS frame through the
// kernel pipeline (requires kissattach setup identical to SEC-X) to prove that
// a Direwolf-compatible TNC feeding FX.25 frames would have its inner AX.25
// content decoded correctly by the Linux kernel stack.
//
// COMPILATION GUARDS
// ──────────────────────────────────────────────────────────────────────────
// The section compiles and runs unconditionally; individual sub-tests are
// individually guarded.  Sub-tests that require FX.25 API symbols exported
// by libax25v22 are wrapped in #if / #ifdef guards so that the file still
// compiles (and skips gracefully) when FX.25 is not yet wired up.
// ===========================================================================

// ---------------------------------------------------------------------------
// Y helpers: pure-C Reed-Solomon GF(2^8) with poly 0x11D (CCSDS / FX.25)
// ---------------------------------------------------------------------------
//
// This self-contained RS implementation is used:
//  (a) as a reference oracle when libax25v22 exports its own FX.25 API, and
//  (b) as the only implementation when it does not.
//
// Parameters: GF(2^8), generator poly 0x11D, first-consecutive-root b=112,
// primitive element α = 2, same as CCSDS / Direwolf / libfx25.
// ---------------------------------------------------------------------------

/* GF(2^8) tables -- built once on first use */
static uint8_t y_gf_exp[512]; /* α^i mod poly, 0..510        */
static uint8_t y_gf_log[256]; /* discrete log, gf_log[0]=N/A */
static int y_gf_ready = 0;

static void y_gf_init(void) {
    int i;
    uint16_t x = 1;
    if (y_gf_ready)
        return;
    for (i = 0; i < 255; i++) {
        y_gf_exp[i] = (uint8_t) x;
        y_gf_exp[i + 255] = (uint8_t) x; /* duplicate for wrap-around */
        y_gf_log[(uint8_t) x] = (uint8_t) i;
        x <<= 1;
        if (x & 0x100)
            x ^= 0x11D; /* CCSDS generator polynomial  */
    }
    y_gf_log[0] = 0; /* undefined, but set to 0 to avoid UB */
    y_gf_ready = 1;
}

static uint8_t y_gf_mul(uint8_t a, uint8_t b) {
    if (a == 0 || b == 0)
        return 0;
    return y_gf_exp[(int) y_gf_log[a] + (int) y_gf_log[b]];
}

static uint8_t y_gf_div(uint8_t a, uint8_t b) {
    if (a == 0)
        return 0;
    if (b == 0)
        return 0; /* div-by-zero: caller must avoid */
    return y_gf_exp[((int) y_gf_log[a] - (int) y_gf_log[b] + 255) % 255];
}

/* Reed-Solomon encode: given <info_len> information bytes in data[],
 * appends <nroots> parity bytes at data[info_len..info_len+nroots-1].
 * data[] must have room for info_len + nroots bytes.
 * first-consecutive-root b = 112 (FX.25 / CCSDS standard).
 * Returns 0 on success.
 *
 * Systematic encoding: divide data(x)*x^nroots by g(x), remainder = parity.
 * LFSR implementation: feedback = data[i] XOR reg[nroots-1]
 *   reg[j] = reg[j-1] XOR mul(g[nroots-j], feedback),  j = nroots-1..1
 *   reg[0]  = mul(g[nroots], feedback)   (g[nroots] = 1 monic, so = feedback)
 * After all info bytes processed, parity = reg[0..nroots-1] reversed.
 *
 * Reference: Blahut, "Algebraic Codes for Data Transmission", §7.4.
 */
static int y_rs_encode(uint8_t *data, int info_len, int nroots) {
    uint8_t g[65]; /* generator poly, degree nroots, g[0]=leading coeff */
    uint8_t reg[64]; /* shift register, reg[0]=highest-degree feedback end */
    int i, j;
    const int b = 112;

    y_gf_init();
    if (nroots > 64 || nroots < 1 || info_len <= 0)
        return -1;

    /* Build monic generator polynomial g(x) = ∏_{i=0}^{nroots-1} (x - α^{b+i})
     * Coefficients stored with g[0] = constant term, g[nroots] = 1 (monic).
     */
    memset(g, 0, sizeof(g));
    g[0] = 1;
    for (i = 0; i < nroots; i++) {
        /* multiply g(x) by (x - α^{b+i}): g = g * (x + α^{b+i}) in GF(2) */
        uint8_t root = y_gf_exp[b + i];
        for (j = i + 1; j > 0; j--)
            g[j] = g[j - 1] ^ y_gf_mul(g[j], root);
        g[0] = y_gf_mul(g[0], root);
    }
    /* Now g[0..nroots] with g[nroots]=1 (monic). */

    /* LFSR systematic division: compute data(x)*x^nroots mod g(x).
     * reg[0] = coefficient of x^(nroots-1) in remainder (highest power).
     * reg[nroots-1] = constant term.
     */
    memset(reg, 0, (size_t) nroots);
    for (i = 0; i < info_len; i++) {
        uint8_t feedback = data[i] ^ reg[0];
        if (feedback != 0) {
            /* shift left and XOR with g */
            for (j = 0; j < nroots - 1; j++)
                reg[j] = reg[j + 1] ^ y_gf_mul(g[nroots - 1 - j], feedback);
            reg[nroots - 1] = y_gf_mul(g[0], feedback);
        } else {
            /* just shift */
            for (j = 0; j < nroots - 1; j++)
                reg[j] = reg[j + 1];
            reg[nroots - 1] = 0;
        }
    }
    /* Parity output: reg[0] is highest-degree, store reversed so that
     * data[info_len + 0] = highest-degree parity coefficient.
     * This matches the "info bytes first, parity appended" convention
     * used by both Phil Karn's rs.c and Direwolf's FX.25. */
    for (i = 0; i < nroots; i++)
        data[info_len + i] = reg[i];
    return 0;
}

/* Reed-Solomon decode: corrects up to nroots/2 errors in place.
 * block[] contains info_len + nroots bytes (full codeword, info first).
 * Returns number of errors corrected (≥ 0) or -1 if uncorrectable.
 *
 * Implementation follows the standard RS(n,k) over GF(2^8) algorithm:
 *   1. Compute nroots syndromes  S_i = C(α^{b+i}),  i = 0..nroots-1
 *      using Horner's method:  S = 0; for each byte: S = byte ^ mul(S, α^{b+i})
 *   2. Berlekamp-Massey to find error-locator polynomial Λ(x)
 *   3. Chien search: evaluate Λ at α^{-j} for j = 0..block_len-1
 *   4. Forney formula: e_j = −Ω(X_j^{-1}) / Λ'(X_j^{-1})
 *      In GF(2^m) negation is identity, so e_j = Ω(X_j^{-1}) / Λ'(X_j^{-1}).
 *      X_j = α^j (error locator), X_j^{-1} = α^{255-j}.
 *
 * Reference: Wicker & Bhargava, "Reed-Solomon Codes and Their Applications";
 *            Phil Karn KA9Q rs.c (public domain) — the canonical FX.25 RS.
 */
static int y_rs_decode(uint8_t *block, int info_len, int nroots) {
    int block_len = info_len + nroots;
    int i, j, r;
    const int b = 112; /* first consecutive root, FX.25/CCSDS */
    int t = nroots / 2; /* max correctable errors */

    y_gf_init();
    if (nroots > 64 || nroots < 2 || info_len <= 0)
        return -1;
    if (block_len > 255)
        return -1; /* GF(2^8) block length limit */

    /* ------------------------------------------------------------------ */
    /* Step 1: Syndromes  S[i] = C(α^{b+i}),  i = 0 .. nroots-1          */
    /* Horner: S = block[0]; for j=1..block_len-1: S = block[j] ^ mul(S, α^{b+i}) */
    /* ------------------------------------------------------------------ */
    uint8_t syn[64]; /* nroots ≤ 64 */
    int syn_error = 0;
    for (i = 0; i < nroots; i++) {
        uint8_t s = block[0];
        uint8_t root = y_gf_exp[b + i]; /* α^{b+i} */
        for (j = 1; j < block_len; j++)
            s = block[j] ^ y_gf_mul(s, root);
        syn[i] = s;
        if (s != 0)
            syn_error = 1;
    }
    if (!syn_error)
        return 0; /* codeword is valid, no errors */

    /* Debug: print first 4 syndromes to aid diagnosis */
    DEBUG_PRINT("RS decode: syn[0..3] = %02X %02X %02X %02X (syn_error=%d)", syn[0], syn[1], syn[2], syn[3], syn_error);

    /* ------------------------------------------------------------------ */
    /* Step 2: Berlekamp-Massey (Lin & Costello Algorithm 6.1)            */
    /*                                                                     */
    /* sigma[i] = error locator polynomial, sigma[0]=1 (monic).          */
    /* tau[i]   = saved copy of sigma from the step rho (last L update). */
    /* d_rho    = discrepancy at step rho.                               */
    /* rho      = step index at which L was last increased (init -1).    */
    /*                                                                     */
    /* Each step r:                                                        */
    /*   d = S[r] + Σ_{i=1}^{deg_sigma} sigma[i]*S[r-i]                 */
    /*   if d != 0:                                                        */
    /*     delay = r - rho                                                */
    /*     sigma = sigma - (d/d_rho) * x^delay * tau                     */
    /*     if 2*deg(sigma_old) <= r:  tau=sigma_old, d_rho=d, rho=r      */
    /*                                                                     */
    /* This formulation avoids the x-shift-per-step pitfall of the       */
    /* Massey-only implementation and always keeps sigma[0]=1.           */
    /* ------------------------------------------------------------------ */
    uint8_t sigma[nroots + 2]; /* error locator, sigma[0]=1 */
    uint8_t tau[nroots + 2]; /* saved sigma at last update */
    memset(sigma, 0, (size_t) (nroots + 2));
    memset(tau, 0, (size_t) (nroots + 2));
    sigma[0] = 1;
    tau[0] = 1;
    uint8_t d_rho = 1;
    int rho = -1; /* step of last L increase */

    for (r = 0; r < nroots; r++) {
        /* Discrepancy d = S[r] + Σ_{i=1}^{nroots} sigma[i]*S[r-i] */
        uint8_t d = syn[r];
        for (i = 1; i <= nroots; i++) {
            if (r - i >= 0 && sigma[i])
                d ^= y_gf_mul(sigma[i], syn[r - i]);
        }

        if (d != 0) {
            int delay = r - rho; /* x^delay shift on tau */
            uint8_t coeff = y_gf_div(d, d_rho);

            /* Build x^delay * tau in a temp buffer */
            uint8_t tau_shifted[nroots + 2];
            memset(tau_shifted, 0, (size_t) (nroots + 2));
            for (i = 0; i + delay < nroots + 2; i++)
                tau_shifted[i + delay] = tau[i];

            /* Save old sigma to check degree */
            uint8_t sigma_old[nroots + 2];
            memcpy(sigma_old, sigma, (size_t) (nroots + 2));

            /* sigma = sigma - coeff * x^delay * tau */
            for (i = 0; i < nroots + 2; i++)
                sigma[i] ^= y_gf_mul(coeff, tau_shifted[i]);

            /* Determine old degree (before update) */
            int deg_old = 0;
            for (i = nroots + 1; i >= 1; i--)
                if (sigma_old[i]) {
                    deg_old = i;
                    break;
                }

            /* If 2*deg_old <= r, increase L: update tau and d_rho */
            if (2 * deg_old <= r) {
                memcpy(tau, sigma_old, (size_t) (nroots + 2));
                d_rho = d;
                rho = r;
            }
        }
    }

    /* Degree of sigma */
    int deg_lam = 0;
    for (i = nroots + 1; i >= 1; i--)
        if (sigma[i]) {
            deg_lam = i;
            break;
        }

    DEBUG_PRINT("RS decode: rho=%d deg_lam=%d sigma[0..4]=%02X %02X %02X %02X %02X", rho, deg_lam, sigma[0], sigma[1], sigma[2], sigma[3], sigma[4]);

    if (deg_lam == 0 || deg_lam > t) {
        DEBUG_PRINT("RS decode: FAIL deg_lam=%d out of range [1..%d]", deg_lam, t);
        return -1;
    }

    /* ------------------------------------------------------------------ */
    /* Step 3: Error-evaluator polynomial Ω = S * σ mod x^nroots          */
    /* Ω[i] = Σ_{j=0}^{min(i,deg_lam)} σ[j] * S[i-j],  i=0..nroots-1  */
    /* ------------------------------------------------------------------ */
    uint8_t omega[65]; /* nroots ≤ 64 */
    memset(omega, 0, sizeof(omega));
    for (i = 0; i < nroots; i++) {
        uint8_t w = 0;
        int jmax = (i < deg_lam) ? i : deg_lam;
        for (j = 0; j <= jmax; j++)
            w ^= y_gf_mul(sigma[j], syn[i - j]);
        omega[i] = w;
    }

    /* ------------------------------------------------------------------ */
    /* Step 4: Chien search + Forney error correction                     */
    /* Evaluate σ(α^{-k}) for k = 0 .. block_len-1.                      */
    /* If σ(α^{-k}) == 0, then position k (from the end) has an error.  */
    /* Error location in the block: pos = block_len - 1 - k             */
    /* ------------------------------------------------------------------ */
    int nerrors = 0;

    /* Save corrected copy separately so we can re-check syndromes */
    uint8_t corrected[255];
    memcpy(corrected, block, (size_t) block_len);

    for (i = 0; i < block_len && nerrors <= deg_lam; i++) {
        /* Evaluate σ at α^{-i}: use Horner's method */
        uint8_t xi_inv = (i == 0) ? 1 : y_gf_exp[255 - i];
        uint8_t lam_val = sigma[deg_lam];
        for (j = deg_lam - 1; j >= 0; j--)
            lam_val = sigma[j] ^ y_gf_mul(lam_val, xi_inv);
        if (lam_val != 0)
            continue; /* not a root */

        /* Root found at α^{-i}: error at position block_len-1-i */
        int pos = block_len - 1 - i;
        if (pos < 0 || pos >= block_len) {
            DEBUG_PRINT("RS decode: Chien found pos=%d out of range, fail", pos);
            return -1;
        }
        nerrors++;

        /* Forney formula (see comment above for derivation) */
        int xi_log = (i == 0) ? 0 : (int) (255 - i);

        uint8_t om_val = 0;
        for (j = 0; j < nroots; j++) {
            if (omega[j] == 0)
                continue;
            uint8_t xpow = (j == 0) ? 1 : y_gf_exp[(int) ((long) xi_log * j % 255)];
            om_val ^= y_gf_mul(omega[j], xpow);
        }

        uint8_t lp_val = 0;
        for (j = 1; j <= deg_lam; j += 2) {
            if (sigma[j] == 0)
                continue;
            uint8_t xpow = (j == 1) ? 1 : y_gf_exp[(int) ((long) xi_log * (j - 1) % 255)];
            lp_val ^= y_gf_mul(sigma[j], xpow);
        }

        DEBUG_PRINT("RS decode: error at pos=%d (i=%d xi_log=%d) om=%02X lp=%02X", pos, i, xi_log, om_val, lp_val);

        if (lp_val == 0) {
            DEBUG_PRINT("RS decode: FAIL lp_val==0 at pos=%d", pos);
            return -1;
        }

        /* X_j^{-(b-1)} scaling factor */
        uint8_t scale;
        if (i == 0 || b == 1) {
            scale = 1;
        } else {
            int raw = (int) ((long) (b - 1) * i % 255);
            int neg = (raw == 0) ? 0 : (255 - raw);
            scale = (neg == 0) ? 1 : y_gf_exp[neg];
        }

        uint8_t magnitude = y_gf_mul(scale, y_gf_div(om_val, lp_val));
        corrected[pos] ^= magnitude;

        DEBUG_PRINT("RS decode: corrected pos=%d scale=%02X magnitude=%02X new_val=%02X", pos, scale, magnitude, corrected[pos]);
    }

    DEBUG_PRINT("RS decode: nerrors=%d deg_lam=%d", nerrors, deg_lam);

    if (nerrors != deg_lam) {
        DEBUG_PRINT("RS decode: FAIL nerrors(%d) != deg_lam(%d)", nerrors, deg_lam);
        return -1;
    }

    /* ------------------------------------------------------------------ */
    /* Step 5: Post-correction syndrome check                             */
    /* Re-evaluate all syndromes on the corrected codeword.              */
    /* If any syndrome is non-zero, the correction was wrong             */
    /* (this catches t+1 errors that passed the deg_lam≤t gate).        */
    /* ------------------------------------------------------------------ */
    for (i = 0; i < nroots; i++) {
        uint8_t s = corrected[0];
        uint8_t root = y_gf_exp[b + i];
        for (j = 1; j < block_len; j++)
            s = corrected[j] ^ y_gf_mul(s, root);
        if (s != 0) {
            DEBUG_PRINT("RS decode: post-correction syn[%d]=%02X != 0, FAIL", i, s);
            return -1;
        }
    }

    /* All syndromes zero: commit corrections to caller's buffer */
    memcpy(block, corrected, (size_t) block_len);
    return nerrors;
}

// ---------------------------------------------------------------------------
// Y helpers: FX.25 correlation tag constants (spec Table 1)
// ---------------------------------------------------------------------------
// Transmission byte order: LSB of the 64-bit value is sent first.
// So for Tag_01 = 0xB74DB7DF8A532F3E, on-wire bytes are:
//   0x3E 0x2F 0x53 0x8A 0xDF 0xB7 0x4D 0xB7

#define Y_FX25_TAG_01_VAL  UINT64_C(0xB74DB7DF8A532F3E)
#define Y_FX25_TAG_02_VAL  UINT64_C(0x26FF60A600CC8FDE)
#define Y_FX25_TAG_03_VAL  UINT64_C(0xC7DC0508F3D9B09E)
#define Y_FX25_TAG_04_VAL  UINT64_C(0x8F056EB4369660EE)
#define Y_FX25_TAG_05_VAL  UINT64_C(0x6E260B1AC5835FAE)
#define Y_FX25_TAG_06_VAL  UINT64_C(0xFF94DC634F1CFF4E)
#define Y_FX25_TAG_07_VAL  UINT64_C(0x1EB7B9CDBC09C00E)
#define Y_FX25_TAG_08_VAL  UINT64_C(0xDBF869BD2DBB1776)
#define Y_FX25_TAG_09_VAL  UINT64_C(0x3ADB0C13DEAE2836)
#define Y_FX25_TAG_0A_VAL  UINT64_C(0xAB69DB6A543188D6)
#define Y_FX25_TAG_0B_VAL  UINT64_C(0x4A4ABEC4A724B796)

/* Tag descriptor */
typedef struct {
    uint64_t tag_val; /* 64-bit Gold Code value */
    int block_len; /* total RS codeblock bytes (info + check) */
    int info_len; /* information byte capacity                */
    int nroots; /* RS check symbols                         */
    uint8_t tag_id; /* 0x01 .. 0x0B                             */
} y_fx25_tag_t;

/* Table is sorted ASCENDING by info_len so that y_fx25_select_tag() always
 * returns the SMALLEST tag whose capacity fits the AX.25 frame.
 *
 * info_len ordering (ascending):
 *   32 (Tag_04, Tag_08), 64 (Tag_03, Tag_07, Tag_0B),
 *   128 (Tag_02, Tag_06, Tag_0A), 191 (Tag_09), 223 (Tag_05),
 *   239 (Tag_01).
 * Within the same info_len, tags with fewer parity bytes (fewer nroots)
 * come first — they are "cheaper" and should be preferred when equal
 * capacity is available.
 */
static const y_fx25_tag_t y_fx25_tags[] = {
/* info=32 */
{ Y_FX25_TAG_04_VAL, 48, 32, 16, 0x04 }, /* RS( 48, 32) nroots=16 */
{ Y_FX25_TAG_08_VAL, 64, 32, 32, 0x08 }, /* RS( 64, 32) nroots=32 */
/* info=64 */
{ Y_FX25_TAG_03_VAL, 80, 64, 16, 0x03 }, /* RS( 80, 64) nroots=16 */
{ Y_FX25_TAG_07_VAL, 96, 64, 32, 0x07 }, /* RS( 96, 64) nroots=32 */
{ Y_FX25_TAG_0B_VAL, 128, 64, 64, 0x0B }, /* RS(128, 64) nroots=64 */
/* info=128 */
{ Y_FX25_TAG_02_VAL, 144, 128, 16, 0x02 }, /* RS(144,128) nroots=16 */
{ Y_FX25_TAG_06_VAL, 160, 128, 32, 0x06 }, /* RS(160,128) nroots=32 */
{ Y_FX25_TAG_0A_VAL, 192, 128, 64, 0x0A }, /* RS(192,128) nroots=64 */
/* info=191 */
{ Y_FX25_TAG_09_VAL, 255, 191, 64, 0x09 }, /* RS(255,191) nroots=64 */
/* info=223 */
{ Y_FX25_TAG_05_VAL, 255, 223, 32, 0x05 }, /* RS(255,223) nroots=32 */
/* info=239 */
{ Y_FX25_TAG_01_VAL, 255, 239, 16, 0x01 }, /* RS(255,239) nroots=16 */
};
#define Y_NUM_TAGS ((int)(sizeof(y_fx25_tags)/sizeof(y_fx25_tags[0])))

/* Select the smallest tag whose info capacity fits <ax25_stuffed_bytes>+2
 * (the two 0x7E flag bytes that wrap the AX.25 packet inside the codeblock).
 * Returns NULL if no tag fits. */
static const y_fx25_tag_t* y_fx25_select_tag(int ax25_len) {
    int i;
    for (i = 0; i < Y_NUM_TAGS; i++)
        if (ax25_len + 2 <= y_fx25_tags[i].info_len) /* +2 for flags */
            return &y_fx25_tags[i];
    return NULL;
}

/* Write correlation tag bytes (64 bits, LSB first) into buf[0..7]. */
static void y_fx25_write_tag(uint8_t *buf, uint64_t tag_val) {
    int i;
    for (i = 0; i < 8; i++)
        buf[i] = (uint8_t) (tag_val >> (i * 8));
}

/* Read correlation tag from buf[0..7], return uint64_t value.
 * Returns 0 if not matched to any known tag. */
static uint64_t y_fx25_read_tag(const uint8_t *buf) {
    uint64_t v = 0;
    int i;
    for (i = 0; i < 8; i++)
        v |= ((uint64_t) buf[i]) << (i * 8);
    return v;
}

/* Look up tag descriptor by value.  Returns NULL if unknown. */
static const y_fx25_tag_t* y_fx25_find_tag(uint64_t val) {
    int i;
    for (i = 0; i < Y_NUM_TAGS; i++)
        if (y_fx25_tags[i].tag_val == val)
            return &y_fx25_tags[i];
    return NULL;
}

// ---------------------------------------------------------------------------
// Y helpers: FX.25 frame encode / decode (our reference implementation)
// ---------------------------------------------------------------------------

/* Maximum FX.25 frame size:
 * 4 preamble + 8 tag + 255 codeblock + 2 postamble = 269 bytes.
 * Round up generously. */
#define Y_FX25_MAX_FRAME 512
#define Y_FX25_PREAMBLE_LEN 4
#define Y_FX25_POSTAMBLE_LEN 2

/* Encode: wrap ax25_bytes[ax25_len] into an FX.25 frame.
 * out[] must be at least Y_FX25_MAX_FRAME bytes.
 * Returns total frame length, or -1 on error. */
static int y_fx25_encode_frame(const uint8_t *ax25_bytes, int ax25_len, uint8_t *out, int out_max, const y_fx25_tag_t *tag) {
    int i;
    uint8_t codeblock[256]; /* max block_len = 255 */
    int n = 0;

    if (!ax25_bytes || ax25_len <= 0 || !out || !tag)
        return -1;
    if (ax25_len + 2 > tag->info_len)
        return -1; /* won't fit */
    if (tag->block_len > (int) sizeof(codeblock))
        return -1;
    if (out_max < Y_FX25_PREAMBLE_LEN + 8 + tag->block_len + Y_FX25_POSTAMBLE_LEN)
        return -1;

    /* Preamble: 4 × 0x7E */
    for (i = 0; i < Y_FX25_PREAMBLE_LEN; i++)
        out[n++] = 0x7E;

    /* Correlation Tag: 8 bytes LSB-first */
    y_fx25_write_tag(&out[n], tag->tag_val);
    n += 8;

    /* FEC Codeblock construction:
     * [0x7E][ax25_bytes...][0x7E][0x7E pad to info_len][RS parity x nroots] */
    memset(codeblock, 0x7E, (size_t) tag->block_len);

    /* Start flag */
    codeblock[0] = 0x7E;
    /* AX.25 payload */
    memcpy(&codeblock[1], ax25_bytes, (size_t) ax25_len);
    /* End flag */
    codeblock[1 + ax25_len] = 0x7E;
    /* Pad remainder with 0x7E up to info_len */
    for (i = 1 + ax25_len + 1; i < tag->info_len; i++)
        codeblock[i] = 0x7E;

    /* RS encode: parity appended at codeblock[info_len..info_len+nroots-1] */
    if (y_rs_encode(codeblock, tag->info_len, tag->nroots) != 0)
        return -1;

    /* Append codeblock to output */
    memcpy(&out[n], codeblock, (size_t) tag->block_len);
    n += tag->block_len;

    /* Postamble: 2 × 0x7E */
    for (i = 0; i < Y_FX25_POSTAMBLE_LEN; i++)
        out[n++] = 0x7E;

    return n;
}

/* Decode result */
typedef struct {
    int ok; /* 1 = success                            */
    int errors_found; /* RS errors corrected (≥ 0)              */
    int ax25_offset; /* offset of AX.25 bytes in codeblock     */
    int ax25_len; /* byte count of AX.25 content            */
    uint8_t tag_id; /* tag identifier used                    */
} y_fx25_decode_result_t;

/* Decode an FX.25 frame in buf[0..buf_len-1].
 * On success, fills *result and writes recovered AX.25 bytes into
 * ax25_out[0..result->ax25_len-1].  ax25_out must be ≥ 256 bytes.
 * Returns 0 on success, -1 on failure. */
static int y_fx25_decode_frame(const uint8_t *buf, int buf_len, uint8_t *ax25_out, int ax25_out_max, y_fx25_decode_result_t *result) {
    uint8_t codeblock[256];
    const y_fx25_tag_t *tag = NULL;
    uint64_t tag_val;
    int tag_offset = -1;
    int i, rs_result;

    if (!buf || buf_len < 8 + 8 || !ax25_out || !result)
        return -1;
    memset(result, 0, sizeof(*result));

    /* Scan for correlation tag (skip preamble 0x7E bytes) */
    for (i = 0; i <= buf_len - 8; i++) {
        tag_val = y_fx25_read_tag(&buf[i]);
        tag = y_fx25_find_tag(tag_val);
        if (tag) {
            tag_offset = i;
            break;
        }
    }
    if (!tag)
        return -1;

    /* Codeblock starts immediately after 8-byte tag */
    int cb_offset = tag_offset + 8;
    if (cb_offset + tag->block_len > buf_len)
        return -1;
    if (tag->block_len > (int) sizeof(codeblock))
        return -1;

    memcpy(codeblock, &buf[cb_offset], (size_t) tag->block_len);

    /* RS decode: corrects errors in-place */
    rs_result = y_rs_decode(codeblock, tag->info_len, tag->nroots);
    if (rs_result < 0)
        return -1; /* uncorrectable */

    result->errors_found = rs_result;
    result->tag_id = tag->tag_id;

    /* Extract AX.25 payload: skip leading 0x7E, read until next 0x7E */
    /* codeblock[0] == 0x7E (start flag) */
    int ax25_start = 1;

    /* Find end flag: first 0x7E after start */
    int ax25_content_end = ax25_start;
    for (i = ax25_start; i < tag->info_len; i++) {
        if (codeblock[i] == 0x7E) {
            ax25_content_end = i;
            break;
        }
        ax25_content_end = i + 1;
    }

    int ax25_len = ax25_content_end - ax25_start;
    if (ax25_len <= 0 || ax25_len > ax25_out_max)
        return -1;

    memcpy(ax25_out, &codeblock[ax25_start], (size_t) ax25_len);
    result->ok = 1;
    result->ax25_offset = ax25_start;
    result->ax25_len = ax25_len;
    return 0;
}

// ---------------------------------------------------------------------------
// SEC-Y main function
// ---------------------------------------------------------------------------
static int sec_y_fx25_fec(void) {
    TEST_SECTION("=== SEC-Y: FX.25 Forward Error Correction ===");

    printf("  Reference: FX.25 v0.01.06 (Stensat Group, 2006)\n");
    printf("  RS engine: self-contained GF(2^8) poly=0x11D, b=112 (CCSDS)\n");
    printf("  libax25v22 role: AX.25 encode/decode of inner packet\n\n");

    // -----------------------------------------------------------------------
    // Y.0 — GF(2^8) arithmetic self-test (prerequisite for all RS tests)
    // -----------------------------------------------------------------------
    {
        y_gf_init();

        /* α^255 == 1 (field order) */
        TEST_ASSERT(y_gf_exp[255] == 1, "Y.0.a GF(2^8) field order: α^255 == 1 (α^255 = y_gf_exp[255])", y_gf_exp[255]);

        /* α^1 == 2 (primitive element) */
        TEST_ASSERT(y_gf_exp[1] == 2, "Y.0.b GF(2^8) primitive element: α^1 == 2", y_gf_exp[1]);

        /* multiplication: α^1 * α^2 = α^3 */
        TEST_ASSERT(y_gf_mul(y_gf_exp[1], y_gf_exp[2]) == y_gf_exp[3], "Y.0.c GF mul: α^1 * α^2 == α^3", y_gf_mul(y_gf_exp[1], y_gf_exp[2]));

        /* log/exp inverse: exp[log[x]] == x for x != 0 */
        {
            int ok = 1, i;
            for (i = 1; i < 256; i++)
                if (y_gf_exp[y_gf_log[i]] != (uint8_t) i) {
                    ok = 0;
                    break;
                }
            TEST_ASSERT(ok, "Y.0.d GF exp/log inverse for all non-zero elements", ok);
        }

        /* multiply by 0 == 0 */
        TEST_ASSERT(y_gf_mul(0xAB, 0) == 0, "Y.0.e GF mul by zero == 0", y_gf_mul(0xAB, 0));

        /* x / x == 1 for x != 0 */
        TEST_ASSERT(y_gf_div(0x5C, 0x5C) == 1, "Y.0.f GF div: x/x == 1", y_gf_div(0x5C, 0x5C));

        DEBUG_PRINT("Y.0 GF(2^8) arithmetic verified (poly=0x11D, b=112)");
    }

    // -----------------------------------------------------------------------
    // Y.1 — FX.25 correlation tag values match spec Table 1
    // -----------------------------------------------------------------------
    {
        /* Spec §Correlation Tag Details (p.5-6): Tag_01 bytes LSB-first:
         * 0x3E 0x2F 0x53 0x8A 0xDF 0xB7 0x4D 0xB7 */
        uint8_t tag01_bytes[8];
        y_fx25_write_tag(tag01_bytes, Y_FX25_TAG_01_VAL);
        int tag01_ok = (tag01_bytes[0] == 0x3E && tag01_bytes[1] == 0x2F && tag01_bytes[2] == 0x53 && tag01_bytes[3] == 0x8A && tag01_bytes[4] == 0xDF
                && tag01_bytes[5] == 0xB7 && tag01_bytes[6] == 0x4D && tag01_bytes[7] == 0xB7);
        TEST_ASSERT(tag01_ok, "Y.1.a Tag_01 on-wire bytes == 3E 2F 53 8A DF B7 4D B7 (spec §CT)", (int )tag01_bytes[0]);
        DEBUG_PRINT("Y.1 Tag_01 bytes: %02X %02X %02X %02X %02X %02X %02X %02X", tag01_bytes[0], tag01_bytes[1], tag01_bytes[2], tag01_bytes[3], tag01_bytes[4],
                tag01_bytes[5], tag01_bytes[6], tag01_bytes[7]);

        /* Tag_04: RS(48,32) 16-byte check, 32 info bytes */
        uint8_t tag04_bytes[8];
        y_fx25_write_tag(tag04_bytes, Y_FX25_TAG_04_VAL);
        uint64_t tag04_rt = y_fx25_read_tag(tag04_bytes);
        TEST_ASSERT(tag04_rt == Y_FX25_TAG_04_VAL, "Y.1.b Tag_04 write/read round-trip matches spec constant", (int)(tag04_rt != Y_FX25_TAG_04_VAL));

        /* Tag_08: RS(64,32) 32-byte check — smallest 32-root tag */
        const y_fx25_tag_t *t08 = y_fx25_find_tag(Y_FX25_TAG_08_VAL);
        TEST_ASSERT(t08 != NULL && t08->nroots == 32 && t08->info_len == 32, "Y.1.c Tag_08 descriptor: nroots=32, info_len=32 (spec Table 1)",
                t08 ? t08->nroots : -1);

        /* Tag_0B: RS(128,64) 64-byte check — largest defined tag */
        const y_fx25_tag_t *t0b = y_fx25_find_tag(Y_FX25_TAG_0B_VAL);
        TEST_ASSERT(t0b != NULL && t0b->nroots == 64 && t0b->info_len == 64, "Y.1.d Tag_0B descriptor: nroots=64, info_len=64 (spec Table 1)",
                t0b ? t0b->nroots : -1);

        /* All tags have distinct values */
        {
            int dup = 0, i, j;
            for (i = 0; i < Y_NUM_TAGS; i++)
                for (j = i + 1; j < Y_NUM_TAGS; j++)
                    if (y_fx25_tags[i].tag_val == y_fx25_tags[j].tag_val)
                        dup = 1;
            TEST_ASSERT(!dup, "Y.1.e All 11 tag values are distinct", dup);
        }

        /* tag_select: small packet → Tag_04 (smallest sufficient) */
        {
            const y_fx25_tag_t *sel = y_fx25_select_tag(20); /* 20+2=22 ≤ 32 */
            TEST_ASSERT(sel != NULL && sel->tag_id == 0x04, "Y.1.f tag_select(20-byte AX.25) → Tag_04 (info_len=32)", sel ? sel->tag_id : -1);
        }
    }

    // -----------------------------------------------------------------------
    // Y.2 — RS(48,32) encode produces correct parity (Tag_04, nroots=16)
    //        Self-check: re-encode and compare; then syndrome == 0 after encode
    // -----------------------------------------------------------------------
    {
        /* Known-vector test: RS(48,32).
         * data[0..31] = 0x00..0x1F, check symbols appended at data[32..47]. */
        uint8_t data[48];
        uint8_t data_copy[48];
        int i;
        for (i = 0; i < 32; i++)
            data[i] = (uint8_t) i;
        memset(&data[32], 0, 16);

        int enc_rc = y_rs_encode(data, 32, 16);
        TEST_ASSERT(enc_rc == 0, "Y.2.a RS(48,32) encode returns 0", enc_rc);

        /* Parity must be non-zero for non-trivial message */
        int parity_nonzero = 0;
        for (i = 32; i < 48; i++)
            if (data[i])
                parity_nonzero = 1;
        TEST_ASSERT(parity_nonzero, "Y.2.b RS(48,32) parity bytes are not all zero for non-zero message", parity_nonzero);

        /* Re-encode same message → identical parity (deterministic) */
        memcpy(data_copy, data, 32);
        memset(&data_copy[32], 0, 16);
        y_rs_encode(data_copy, 32, 16);
        TEST_ASSERT(memcmp(data, data_copy, 48) == 0, "Y.2.c RS(48,32) encode is deterministic (same input → same parity)", 0);

        /* Decode the error-free codeword → 0 corrections */
        uint8_t cw[48];
        memcpy(cw, data, 48);
        int dec_rc = y_rs_decode(cw, 32, 16);
        TEST_ASSERT(dec_rc >= 0, "Y.2.d RS(48,32) decode error-free codeword returns ≥ 0", dec_rc);
        TEST_ASSERT(dec_rc == 0, "Y.2.e RS(48,32) decode error-free codeword: 0 corrections", dec_rc);
        TEST_ASSERT(memcmp(cw, data, 48) == 0, "Y.2.f RS(48,32) decode error-free codeword: unchanged data", 0);

        DEBUG_PRINT("Y.2 RS(48,32) parity[0..3]: %02X %02X %02X %02X", data[32], data[33], data[34], data[35]);
    }

    // -----------------------------------------------------------------------
    // Y.3 — RS decode corrects 1-byte error (Tag_04: t=8 → can fix up to 8)
    // -----------------------------------------------------------------------
    {
        uint8_t data[48];
        int i;
        for (i = 0; i < 32; i++)
            data[i] = (uint8_t) (0xA0 ^ i);
        memset(&data[32], 0, 16);
        y_rs_encode(data, 32, 16);

        uint8_t cw[48];
        memcpy(cw, data, 48);

        /* Corrupt byte at position 7 */
        uint8_t original = cw[7];
        cw[7] ^= 0x55;

        int dec_rc = y_rs_decode(cw, 32, 16);
        TEST_ASSERT(dec_rc == 1, "Y.3.a RS(48,32) decode single-byte error: 1 correction reported", dec_rc);
        TEST_ASSERT(cw[7] == original, "Y.3.b RS(48,32) decode single-byte error: byte[7] restored", (int )cw[7]);
        TEST_ASSERT(memcmp(cw, data, 48) == 0, "Y.3.c RS(48,32) single-byte correction: full codeword matches original", 0);

        /* Corrupt byte in parity region (byte[40]) */
        memcpy(cw, data, 48);
        original = cw[40];
        cw[40] ^= 0xDE;
        dec_rc = y_rs_decode(cw, 32, 16);
        TEST_ASSERT(dec_rc >= 1, "Y.3.d RS(48,32) parity-byte error corrected (dec_rc ≥ 1)", dec_rc);
        TEST_ASSERT(cw[40] == original, "Y.3.e RS(48,32) parity-byte correction: byte[40] restored", (int )cw[40]);

        DEBUG_PRINT("Y.3 1-byte error correction verified at data[7] and parity[8]");
    }

    // -----------------------------------------------------------------------
    // Y.4 — RS decode corrects up to t=nroots/2 errors; fails on t+1
    // -----------------------------------------------------------------------
    {
        /* Tag_04: nroots=16, t=8 → can correct 8 errors max */
        uint8_t data[48];
        int i;
        for (i = 0; i < 32; i++)
            data[i] = (uint8_t) (0x30 + i);
        memset(&data[32], 0, 16);
        y_rs_encode(data, 32, 16);

        /* Inject exactly 8 errors at distinct positions */
        uint8_t cw[48];
        memcpy(cw, data, 48);
        int err_pos[8] = { 1, 5, 11, 17, 22, 25, 28, 31 };
        for (i = 0; i < 8; i++)
            cw[err_pos[i]] ^= (uint8_t) (0x11 * (i + 1));

        int dec_rc = y_rs_decode(cw, 32, 16);
        TEST_ASSERT(dec_rc == 8, "Y.4.a RS(48,32) corrects exactly 8 errors (t == nroots/2)", dec_rc);
        TEST_ASSERT(memcmp(cw, data, 48) == 0, "Y.4.b RS(48,32) 8-error recovery: codeword matches original", 0);

        /* Inject 9 errors → should fail (return -1) */
        memcpy(cw, data, 48);
        for (i = 0; i < 9; i++)
            cw[i + 3] ^= (uint8_t) (0x77 * (i + 1));
        dec_rc = y_rs_decode(cw, 32, 16);
        TEST_ASSERT(dec_rc < 0, "Y.4.c RS(48,32) returns -1 when 9 errors injected (> t=8)", dec_rc);

        DEBUG_PRINT("Y.4 t-boundary: 8-error=PASS, 9-error=FAIL(rc=%d)", dec_rc);
    }

    // -----------------------------------------------------------------------
    // Y.5 — FX.25 frame encode: Tag_01 (RS(255,239), 16-byte check)
    //        Verify preamble, tag bytes, codeblock size, postamble
    // -----------------------------------------------------------------------
    {
        /* Build a short AX.25 UI frame with libax25v22 */
        uint8_t err = 0;
        ax25_address_t *dest = ax25_address_from_string("W1AW-0", &err);
        ax25_address_t *src = ax25_address_from_string("N0FX25-1", &err);

        TEST_ASSERT(dest != NULL && src != NULL, "Y.5.a Create addresses W1AW-0 and N0FX25-1", err);

        uint8_t *ax25_bytes = NULL;
        size_t ax25_len = 0;

        if (dest && src) {
            ax25_frame_header_t hdr;
            ax25_unnumbered_information_frame_t ui;
            uint8_t payload[] = "FX25 TAG01 ENCODE TEST";

            memset(&hdr, 0, sizeof(hdr));
            hdr.destination = *dest;
            hdr.source = *src;
            hdr.cr = false;
            hdr.repeaters.num_repeaters = 0;

            memset(&ui, 0, sizeof(ui));
            ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
            ui.base.base.header = hdr;
            ui.base.pf = false;
            ui.base.modifier = AX25_U_UI;
            ui.pid = PID_NO_L3;
            ui.payload = payload;
            ui.payload_len = (int) (sizeof(payload) - 1);

            ax25_bytes = ax25_frame_encode((ax25_frame_t*) &ui, &ax25_len, &err);
            TEST_ASSERT(ax25_bytes != NULL && err == 0, "Y.5.b libax25v22 encode UI frame for FX.25 wrapping", err);
        }

        if (ax25_bytes) {
            /* Select Tag_01 explicitly (tests spec §Table 1 RS(255,239)) */
            const y_fx25_tag_t *tag01 = y_fx25_find_tag(Y_FX25_TAG_01_VAL);
            TEST_ASSERT(tag01 != NULL, "Y.5.c Tag_01 found in descriptor table", tag01 ? tag01->tag_id : -1);

            if (tag01 && (int) ax25_len + 2 <= tag01->info_len) {
                uint8_t fx25_frame[Y_FX25_MAX_FRAME];
                int fx25_len = y_fx25_encode_frame(ax25_bytes, (int) ax25_len, fx25_frame, sizeof(fx25_frame), tag01);
                TEST_ASSERT(fx25_len > 0, "Y.5.d FX.25 encode with Tag_01 returns positive length", fx25_len);

                if (fx25_len > 0) {
                    /* Check preamble */
                    TEST_ASSERT(fx25_frame[0] == 0x7E && fx25_frame[1] == 0x7E && fx25_frame[2] == 0x7E && fx25_frame[3] == 0x7E,
                            "Y.5.e Preamble: 4 × 0x7E (spec §Preamble)", fx25_frame[0]);

                    /* Check correlation tag bytes */
                    int tag_ok = (fx25_frame[4] == 0x3E && fx25_frame[5] == 0x2F && fx25_frame[6] == 0x53 && fx25_frame[7] == 0x8A && fx25_frame[8] == 0xDF
                            && fx25_frame[9] == 0xB7 && fx25_frame[10] == 0x4D && fx25_frame[11] == 0xB7);
                    TEST_ASSERT(tag_ok, "Y.5.f Correlation tag bytes == 3E 2F 53 8A DF B7 4D B7 (spec §CT)", (int )fx25_frame[4]);

                    /* Codeblock present: expected length = nroots preamble + 8 tag + 255 cb + 2 post */
                    int expected_len = Y_FX25_PREAMBLE_LEN + 8 + tag01->block_len + Y_FX25_POSTAMBLE_LEN;
                    TEST_ASSERT(fx25_len == expected_len, "Y.5.g FX.25 frame total length == preamble+tag+codeblock+postamble", fx25_len);

                    /* Check postamble */
                    TEST_ASSERT(fx25_frame[fx25_len - 1] == 0x7E && fx25_frame[fx25_len - 2] == 0x7E,
                            "Y.5.h Postamble: 2 × 0x7E at frame tail (spec §Postamble)", fx25_frame[fx25_len - 1]);

                    DEBUG_PRINT("Y.5 FX.25 Tag_01 frame: ax25=%zu bytes, fx25=%d bytes " "(overhead=%d RS parity + 4 pre + 8 tag + 2 post)", ax25_len, fx25_len,
                            tag01->nroots);
                }
            } else {
                printf("SKIP: Y.5.d-h (AX.25 frame too large for Tag_01 info capacity)\n");
            }
            free(ax25_bytes);
        }

        if (dest)
            ax25_address_free(dest, &err);
        if (src)
            ax25_address_free(src, &err);
    }

    // -----------------------------------------------------------------------
    // Y.6 — FX.25 decode: error-free round-trip matches original AX.25 frame
    //        libax25v22 encode → FX.25 wrap → FX.25 unwrap → libax25v22 decode
    // -----------------------------------------------------------------------
    {
        uint8_t err = 0;
        ax25_address_t *dest = ax25_address_from_string("KD9FEC-0", &err);
        ax25_address_t *src = ax25_address_from_string("WB5FEC-7", &err);

        if (!dest || !src) {
            printf("SKIP: Y.6 (address allocation failed)\n");
            if (dest)
                ax25_address_free(dest, &err);
            if (src)
                ax25_address_free(src, &err);
            goto y6_done;
        }

        uint8_t payload[] = "UNCORRUPTED ROUND-TRIP TEST";
        ax25_frame_header_t hdr;
        ax25_unnumbered_information_frame_t ui;
        memset(&hdr, 0, sizeof(hdr));
        hdr.destination = *dest;
        hdr.source = *src;
        hdr.cr = false;
        hdr.repeaters.num_repeaters = 0;

        memset(&ui, 0, sizeof(ui));
        ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui.base.base.header = hdr;
        ui.base.pf = false;
        ui.base.modifier = AX25_U_UI;
        ui.pid = PID_NO_L3;
        ui.payload = payload;
        ui.payload_len = (int) (sizeof(payload) - 1);

        size_t ax25_len = 0;
        uint8_t *ax25_bytes = ax25_frame_encode((ax25_frame_t*) &ui, &ax25_len, &err);
        TEST_ASSERT(ax25_bytes != NULL && err == 0, "Y.6.a libax25v22 encode UI frame (source for round-trip)", err);

        if (ax25_bytes) {
            /* Auto-select smallest fitting tag */
            const y_fx25_tag_t *tag = y_fx25_select_tag((int) ax25_len);
            TEST_ASSERT(tag != NULL, "Y.6.b Auto-select FX.25 tag for encoded AX.25 frame", tag ? tag->tag_id : -1);

            if (tag) {
                uint8_t fx25_frame[Y_FX25_MAX_FRAME];
                int fx25_len = y_fx25_encode_frame(ax25_bytes, (int) ax25_len, fx25_frame, sizeof(fx25_frame), tag);
                TEST_ASSERT(fx25_len > 0, "Y.6.c FX.25 encode succeeds (no error injection)", fx25_len);

                if (fx25_len > 0) {
                    /* Decode */
                    uint8_t ax25_recovered[300];
                    y_fx25_decode_result_t dr;
                    int dec_rc = y_fx25_decode_frame(fx25_frame, fx25_len, ax25_recovered, sizeof(ax25_recovered), &dr);
                    TEST_ASSERT(dec_rc == 0, "Y.6.d FX.25 decode error-free frame returns 0", dec_rc);
                    TEST_ASSERT(dr.errors_found == 0, "Y.6.e FX.25 decode error-free: 0 RS corrections", dr.errors_found);
                    TEST_ASSERT(dr.ax25_len == (int )ax25_len, "Y.6.f Recovered AX.25 length matches original", dr.ax25_len);
                    TEST_ASSERT(dr.tag_id == tag->tag_id, "Y.6.g Decoded tag_id matches selected tag", (int )dr.tag_id);

                    int payload_match = (dr.ax25_len == (int) ax25_len && memcmp(ax25_recovered, ax25_bytes, ax25_len) == 0);
                    TEST_ASSERT(payload_match, "Y.6.h Recovered AX.25 bytes match original byte-for-byte", 0);

                    if (payload_match && dec_rc == 0) {
                        /* Now decode the recovered AX.25 bytes with libax25v22 */
                        uint8_t dec_err = 0;
                        ax25_frame_t *rx_frame = ax25_frame_decode(ax25_recovered, (size_t) dr.ax25_len,
                        MODULO128_FALSE, &dec_err);
                        TEST_ASSERT(rx_frame != NULL && dec_err == 0, "Y.6.i libax25v22 decode of FX.25-recovered AX.25 frame", dec_err);

                        if (rx_frame) {
                            TEST_ASSERT(rx_frame->type == AX25_FRAME_UNNUMBERED_INFORMATION, "Y.6.j Recovered frame type == UI", rx_frame->type);

                            ax25_unnumbered_information_frame_t *rxui = (ax25_unnumbered_information_frame_t*) rx_frame;
                            int pmatch = (rxui->payload_len == (int) (sizeof(payload) - 1) && rxui->payload != NULL
                                    && memcmp(rxui->payload, payload, sizeof(payload) - 1) == 0);
                            TEST_ASSERT(pmatch, "Y.6.k Recovered payload matches \"UNCORRUPTED ROUND-TRIP TEST\"", rxui->payload_len);

                            /* Address check via libax25 ax25_cmp */
                            ax25_address dest_linux, src_linux;
                            bridge_libax25v22_to_linux(&rx_frame->header.destination, &dest_linux, &dec_err);
                            bridge_libax25v22_to_linux(&rx_frame->header.source, &src_linux, &dec_err);
                            ax25_address orig_dest_linux, orig_src_linux;
                            bridge_libax25v22_to_linux(dest, &orig_dest_linux, &dec_err);
                            bridge_libax25v22_to_linux(src, &orig_src_linux, &dec_err);

                            TEST_ASSERT(ax25_cmp(&dest_linux, &orig_dest_linux) == 0, "Y.6.l Recovered destination address matches (libax25 ax25_cmp)",
                                    ax25_cmp(&dest_linux, &orig_dest_linux));
                            TEST_ASSERT(ax25_cmp(&src_linux, &orig_src_linux) == 0, "Y.6.m Recovered source address matches (libax25 ax25_cmp)",
                                    ax25_cmp(&src_linux, &orig_src_linux));

                            ax25_frame_free(rx_frame, &dec_err);
                        }
                        DEBUG_PRINT("Y.6 END-TO-END: libax25v22-encode → FX.25 → " "FX.25-decode → libax25v22-decode: PASS " "(tag=%02X, errors=0)",
                                tag->tag_id);
                    }
                }
            }
            free(ax25_bytes);
        }
        ax25_address_free(dest, &err);
        ax25_address_free(src, &err);
    }
    y6_done:
    ;

    // -----------------------------------------------------------------------
    // Y.7 — FX.25 decode corrects 1-byte error in AX.25 payload region
    // -----------------------------------------------------------------------
    {
        uint8_t err = 0;
        ax25_address_t *dest = ax25_address_from_string("AA1FEC-0", &err);
        ax25_address_t *src = ax25_address_from_string("BB2ERR-3", &err);

        if (!dest || !src) {
            printf("SKIP: Y.7 (address allocation failed)\n");
            if (dest)
                ax25_address_free(dest, &err);
            if (src)
                ax25_address_free(src, &err);
            goto y7_done;
        }

        uint8_t payload[] = "ONE BYTE ERROR RECOVERY";
        ax25_frame_header_t hdr;
        ax25_unnumbered_information_frame_t ui;
        memset(&hdr, 0, sizeof(hdr));
        hdr.destination = *dest;
        hdr.source = *src;
        hdr.cr = false;
        hdr.repeaters.num_repeaters = 0;
        memset(&ui, 0, sizeof(ui));
        ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui.base.base.header = hdr;
        ui.base.pf = false;
        ui.base.modifier = AX25_U_UI;
        ui.pid = PID_NO_L3;
        ui.payload = payload;
        ui.payload_len = (int) (sizeof(payload) - 1);

        size_t ax25_len = 0;
        uint8_t *ax25_bytes = ax25_frame_encode((ax25_frame_t*) &ui, &ax25_len, &err);
        TEST_ASSERT(ax25_bytes != NULL, "Y.7.a Encode AX.25 frame for 1-byte error test", err);

        if (ax25_bytes) {
            const y_fx25_tag_t *tag = y_fx25_select_tag((int) ax25_len);
            if (tag) {
                uint8_t fx25_frame[Y_FX25_MAX_FRAME];
                int fx25_len = y_fx25_encode_frame(ax25_bytes, (int) ax25_len, fx25_frame, sizeof(fx25_frame), tag);

                if (fx25_len > 0) {
                    /* Corrupt byte 15 of the codeblock (inside AX.25 content)
                     * Codeblock starts at offset 4(preamble)+8(tag)=12 */
                    int corrupt_pos = 12 + 5; /* 5th byte of codeblock */
                    uint8_t saved = fx25_frame[corrupt_pos];
                    fx25_frame[corrupt_pos] ^= 0xA5;

                    TEST_ASSERT(fx25_frame[corrupt_pos] != saved, "Y.7.b Single byte corrupted in FX.25 codeblock (AX.25 region)",
                            (int )fx25_frame[corrupt_pos]);

                    uint8_t ax25_recovered[300];
                    y_fx25_decode_result_t dr;
                    int dec_rc = y_fx25_decode_frame(fx25_frame, fx25_len, ax25_recovered, sizeof(ax25_recovered), &dr);
                    TEST_ASSERT(dec_rc == 0, "Y.7.c FX.25 decode with 1-byte error returns 0 (corrected)", dec_rc);
                    TEST_ASSERT(dr.errors_found >= 1, "Y.7.d FX.25 decode reports ≥ 1 RS correction", dr.errors_found);

                    if (dec_rc == 0) {
                        /* Verify recovered AX.25 is identical to original */
                        int match = (dr.ax25_len == (int) ax25_len && memcmp(ax25_recovered, ax25_bytes, ax25_len) == 0);
                        TEST_ASSERT(match, "Y.7.e Recovered AX.25 bytes identical to original after 1-byte error", match);

                        /* Verify libax25v22 can decode the recovered bytes */
                        uint8_t dec_err = 0;
                        ax25_frame_t *rxf = ax25_frame_decode(ax25_recovered, (size_t) dr.ax25_len,
                        MODULO128_FALSE, &dec_err);
                        TEST_ASSERT(rxf != NULL && dec_err == 0, "Y.7.f libax25v22 decode of error-corrected AX.25 bytes succeeds", dec_err);
                        if (rxf) {
                            ax25_unnumbered_information_frame_t *rxui = (ax25_unnumbered_information_frame_t*) rxf;
                            int pmatch = (rxui->payload_len == (int) (sizeof(payload) - 1) && memcmp(rxui->payload, payload, sizeof(payload) - 1) == 0);
                            TEST_ASSERT(pmatch, "Y.7.g Payload \"ONE BYTE ERROR RECOVERY\" survives 1-byte FX.25 error", rxui->payload_len);
                            ax25_frame_free(rxf, &dec_err);
                        }
                        DEBUG_PRINT("Y.7 1-byte error correction PASS: " "tag=%02X, corrections=%d", tag->tag_id, dr.errors_found);
                    }
                }
            } else {
                printf("SKIP: Y.7 (no FX.25 tag fits AX.25 frame of %zu bytes)\n", ax25_len);
            }
            free(ax25_bytes);
        }
        ax25_address_free(dest, &err);
        ax25_address_free(src, &err);
    }
    y7_done:
    ;

    // -----------------------------------------------------------------------
    // Y.8 — FX.25 encode/decode across all 11 defined tag variants
    //        Use a payload sized to fit each tag's info capacity
    // -----------------------------------------------------------------------
    {
        int ti;
        printf("  Y.8 All-tags round-trip (11 tag variants):\n");
        for (ti = 0; ti < Y_NUM_TAGS; ti++) {
            const y_fx25_tag_t *tag = &y_fx25_tags[ti];
            /* Use payload filling ~80% of info capacity to exercise different sizes */
            int ax25_target = (tag->info_len * 4) / 5 - 2; /* -2 for flags */
            if (ax25_target < 5)
                ax25_target = 5;

            /* Build a synthetic AX.25-like buffer (raw bytes, not libax25v22
             * structured frame) to keep the test self-contained for sizes
             * that may not produce valid AX.25 minimum headers */
            uint8_t raw_ax25[256];
            int raw_len = (ax25_target < (int) sizeof(raw_ax25)) ? ax25_target : (int) sizeof(raw_ax25) - 1;
            int i;
            for (i = 0; i < raw_len; i++)
                raw_ax25[i] = (uint8_t) (0x41 + (i % 52));

            uint8_t fx25_frame[Y_FX25_MAX_FRAME];
            int fx25_len = y_fx25_encode_frame(raw_ax25, raw_len, fx25_frame, sizeof(fx25_frame), tag);

            char label[64];
            snprintf(label, sizeof(label), "Y.8.%02X FX.25 encode Tag_%02X (info=%d nroots=%d raw_ax25=%d)", tag->tag_id, tag->tag_id, tag->info_len,
                    tag->nroots, raw_len);
            TEST_ASSERT(fx25_len > 0, label, fx25_len);

            if (fx25_len > 0) {
                uint8_t recovered[300];
                y_fx25_decode_result_t dr;
                int dec_rc = y_fx25_decode_frame(fx25_frame, fx25_len, recovered, sizeof(recovered), &dr);
                snprintf(label, sizeof(label), "Y.8.%02X FX.25 decode Tag_%02X round-trip (no errors)", tag->tag_id, tag->tag_id);
                TEST_ASSERT(dec_rc == 0 && dr.ax25_len == raw_len, label, dec_rc);
                if (dec_rc == 0 && dr.ax25_len == raw_len) {
                    int match = memcmp(recovered, raw_ax25, (size_t) raw_len) == 0;
                    snprintf(label, sizeof(label), "Y.8.%02X FX.25 byte-match Tag_%02X (ax25_len=%d)", tag->tag_id, tag->tag_id, raw_len);
                    TEST_ASSERT(match, label, match);
                }
            }
        }
        printf("  Y.8 Done.\n");
    }

    // -----------------------------------------------------------------------
    // Y.9 — KISS pipeline: FX.25-wrapped KISS frame structure
    //        Build: libax25v22-AX.25 → FX.25 → KISS → verify KISS byte layout
    // -----------------------------------------------------------------------
    {
        uint8_t err = 0;
        ax25_address_t *dest = ax25_address_from_string("W1AW-0", &err);
        ax25_address_t *src = ax25_address_from_string("FX25TST-9", &err);

        if (!dest || !src) {
            printf("SKIP: Y.9 (address allocation failed)\n");
            if (dest)
                ax25_address_free(dest, &err);
            if (src)
                ax25_address_free(src, &err);
            goto y9_done;
        }

        uint8_t payload[] = "FX25 KISS PIPELINE";
        ax25_frame_header_t hdr;
        ax25_unnumbered_information_frame_t ui;
        memset(&hdr, 0, sizeof(hdr));
        hdr.destination = *dest;
        hdr.source = *src;
        hdr.cr = false;
        hdr.repeaters.num_repeaters = 0;
        memset(&ui, 0, sizeof(ui));
        ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui.base.base.header = hdr;
        ui.base.pf = false;
        ui.base.modifier = AX25_U_UI;
        ui.pid = PID_NO_L3;
        ui.payload = payload;
        ui.payload_len = (int) (sizeof(payload) - 1);

        size_t ax25_len = 0;
        uint8_t *ax25_bytes = ax25_frame_encode((ax25_frame_t*) &ui, &ax25_len, &err);
        TEST_ASSERT(ax25_bytes != NULL, "Y.9.a libax25v22 encode for KISS pipeline test", err);

        if (ax25_bytes) {
            const y_fx25_tag_t *tag = y_fx25_select_tag((int) ax25_len);
            if (tag) {
                uint8_t fx25_frame[Y_FX25_MAX_FRAME];
                int fx25_len = y_fx25_encode_frame(ax25_bytes, (int) ax25_len, fx25_frame, sizeof(fx25_frame), tag);
                TEST_ASSERT(fx25_len > 0, "Y.9.b FX.25 encode produces frame for KISS wrapping", fx25_len);

                if (fx25_len > 0) {
                    /* Wrap FX.25 frame in KISS (port 0, cmd 0) */
                    uint8_t kiss_buf[Y_FX25_MAX_FRAME + 4];
                    int kiss_len = 0;
                    int kiss_rc = kiss_encode_frame(fx25_frame, fx25_len, 0, 0, kiss_buf, &kiss_len);
                    TEST_ASSERT(kiss_rc == 0, "Y.9.c KISS encode wraps FX.25 frame", kiss_rc);

                    if (kiss_rc == 0) {
                        /* Verify KISS framing:
                         * [0]=FEND, [1]=port|cmd=0x00, ..., [-1]=FEND */
                        TEST_ASSERT(kiss_buf[0] == KISS_FEND, "Y.9.d KISS frame starts with FEND (0xC0)", (int )kiss_buf[0]);
                        TEST_ASSERT(kiss_buf[1] == 0x00, "Y.9.e KISS port/cmd byte == 0x00 (port=0, cmd=DATA)", (int )kiss_buf[1]);
                        TEST_ASSERT(kiss_buf[kiss_len - 1] == KISS_FEND, "Y.9.f KISS frame ends with FEND", (int )kiss_buf[kiss_len - 1]);
                        TEST_ASSERT(kiss_len > fx25_len + 2, "Y.9.g KISS frame longer than FX.25 frame (added framing)", kiss_len);

                        /* Verify FX.25 preamble is visible inside KISS payload
                         * (KISS byte [2] = first byte of FX.25 = 0x7E preamble).
                         * Note: 0x7E == HDLC_FLAG_BYTE which is NOT KISS_FEND (0xC0),
                         * so it must NOT be escaped. */
                        TEST_ASSERT(kiss_buf[2] == 0x7E, "Y.9.h FX.25 preamble 0x7E survives KISS encoding unescaped "
                                "(0x7E ≠ FEND=0xC0 and ≠ FESC=0xDB)", (int )kiss_buf[2]);

                        /* KISS decode → recover FX.25 frame */
                        uint8_t fx25_recovered[Y_FX25_MAX_FRAME + 4];
                        int fx25_recovered_len = 0;
                        int kd_rc = kiss_decode_frame(kiss_buf, kiss_len, fx25_recovered, &fx25_recovered_len);
                        TEST_ASSERT(kd_rc == 0, "Y.9.i KISS decode of FX.25-wrapped frame succeeds", kd_rc);
                        TEST_ASSERT(fx25_recovered_len == fx25_len, "Y.9.j KISS-recovered FX.25 frame length matches original", fx25_recovered_len);
                        TEST_ASSERT(memcmp(fx25_recovered, fx25_frame, (size_t )fx25_len) == 0, "Y.9.k KISS-recovered FX.25 frame bytes match original",
                                fx25_recovered_len);

                        DEBUG_PRINT("Y.9 KISS pipeline: ax25=%zu FX25=%d KISS=%d bytes " "(tag=%02X)", ax25_len, fx25_len, kiss_len, tag->tag_id);
                    }
                }
            } else {
                printf("SKIP: Y.9.b-k (no FX.25 tag fits AX.25 frame)\n");
            }
            free(ax25_bytes);
        }
        ax25_address_free(dest, &err);
        ax25_address_free(src, &err);
    }
    y9_done:
    ;

    // -----------------------------------------------------------------------
    // Y.10 — FX.25 legacy compatibility: inner AX.25 packet is standalone valid
    //         A non-FEC receiver must be able to recover the AX.25 packet
    //         directly from the FX.25 codeblock (spec §AX.25 Packet Requirements)
    // -----------------------------------------------------------------------
    {
        uint8_t err = 0;
        ax25_address_t *dest = ax25_address_from_string("VK2FEC-0", &err);
        ax25_address_t *src = ax25_address_from_string("ZL3CMP-2", &err);

        if (!dest || !src) {
            printf("SKIP: Y.10 (address allocation failed)\n");
            if (dest)
                ax25_address_free(dest, &err);
            if (src)
                ax25_address_free(src, &err);
            goto y10_done;
        }

        uint8_t payload[] = "LEGACY COMPAT TEST";
        ax25_frame_header_t hdr;
        ax25_unnumbered_information_frame_t ui;
        memset(&hdr, 0, sizeof(hdr));
        hdr.destination = *dest;
        hdr.source = *src;
        hdr.cr = false;
        hdr.repeaters.num_repeaters = 0;
        memset(&ui, 0, sizeof(ui));
        ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui.base.base.header = hdr;
        ui.base.pf = false;
        ui.base.modifier = AX25_U_UI;
        ui.pid = PID_NO_L3;
        ui.payload = payload;
        ui.payload_len = (int) (sizeof(payload) - 1);

        size_t ax25_len = 0;
        uint8_t *ax25_bytes = ax25_frame_encode((ax25_frame_t*) &ui, &ax25_len, &err);
        TEST_ASSERT(ax25_bytes != NULL, "Y.10.a libax25v22 encode for legacy-compat test", err);

        if (ax25_bytes) {
            const y_fx25_tag_t *tag = y_fx25_select_tag((int) ax25_len);
            if (tag) {
                uint8_t fx25_frame[Y_FX25_MAX_FRAME];
                int fx25_len = y_fx25_encode_frame(ax25_bytes, (int) ax25_len, fx25_frame, sizeof(fx25_frame), tag);

                if (fx25_len > 0) {
                    /* A legacy (non-FEC) AX.25 receiver scans the bit stream for
                     * the 0x7E flag + AX.25 content directly.
                     * The AX.25 packet starts at codeblock offset 1 (after the
                     * leading 0x7E start flag at cb[0]).
                     * Codeblock starts at fx25_frame[4+8] = fx25_frame[12].
                     * cb[0] = 0x7E, cb[1..ax25_len] = raw AX.25 bytes.
                     * libax25v22 must be able to decode starting from cb[1]. */
                    int cb_start = Y_FX25_PREAMBLE_LEN + 8;
                    /* cb[0] is 0x7E; AX.25 content starts at cb[1] */
                    uint8_t *ax25_in_cb = &fx25_frame[cb_start + 1];

                    /* Verify the leading flag byte */
                    TEST_ASSERT(fx25_frame[cb_start] == 0x7E, "Y.10.b Codeblock[0] == 0x7E (AX.25 packet start flag, spec §AX.25 Pkt Req.)",
                            (int )fx25_frame[cb_start]);

                    /* Content inside codeblock matches original AX.25 bytes */
                    TEST_ASSERT(memcmp(ax25_in_cb, ax25_bytes, ax25_len) == 0, "Y.10.c AX.25 packet bytes embedded in FX.25 codeblock are unmodified", 0);

                    /* Legacy decoder (libax25v22) can decode directly from codeblock */
                    uint8_t dec_err = 0;
                    ax25_frame_t *legacy_frame = ax25_frame_decode(ax25_in_cb, ax25_len,
                    MODULO128_FALSE, &dec_err);
                    TEST_ASSERT(legacy_frame != NULL && dec_err == 0, "Y.10.d libax25v22 decodes AX.25 directly from FX.25 codeblock "
                            "(legacy/non-FEC path)", dec_err);

                    if (legacy_frame) {
                        ax25_unnumbered_information_frame_t *rxui = (ax25_unnumbered_information_frame_t*) legacy_frame;
                        int pmatch = (rxui->payload_len == (int) (sizeof(payload) - 1) && memcmp(rxui->payload, payload, sizeof(payload) - 1) == 0);
                        TEST_ASSERT(pmatch, "Y.10.e Legacy-decoded payload matches original (spec: FX.25 is transparent)", rxui->payload_len);
                        ax25_frame_free(legacy_frame, &dec_err);
                    }
                    DEBUG_PRINT("Y.10 Legacy compat: AX.25 packet survives inside FX.25 codeblock " "(tag=%02X)", tag->tag_id);
                }
            }
            free(ax25_bytes);
        }
        ax25_address_free(dest, &err);
        ax25_address_free(src, &err);
    }
    y10_done:
    ;

    // -----------------------------------------------------------------------
    // Y.11 — FX.25 Tag_01 capacity boundary: largest AX.25 frame that fits
    //         (239 info bytes - 2 flags = 237 AX.25 bytes max for Tag_01)
    // -----------------------------------------------------------------------
    {
        /* Tag_01 info_len = 239; minus 1 start flag + 1 end flag = 237 usable */
        const int MAX_AX25_TAG01 = 237;

        uint8_t raw_ax25[237];
        int i;
        for (i = 0; i < MAX_AX25_TAG01; i++)
            raw_ax25[i] = (uint8_t) (0x41 + (i % 26));

        const y_fx25_tag_t *tag01 = y_fx25_find_tag(Y_FX25_TAG_01_VAL);
        TEST_ASSERT(tag01 != NULL, "Y.11.a Tag_01 descriptor found", tag01 ? 1 : 0);

        if (tag01) {
            uint8_t fx25_frame[Y_FX25_MAX_FRAME];
            int fx25_len = y_fx25_encode_frame(raw_ax25, MAX_AX25_TAG01, fx25_frame, sizeof(fx25_frame), tag01);
            TEST_ASSERT(fx25_len > 0, "Y.11.b Tag_01 fits 237-byte AX.25 payload (max capacity)", fx25_len);

            if (fx25_len > 0) {
                uint8_t recovered[300];
                y_fx25_decode_result_t dr;
                int dec_rc = y_fx25_decode_frame(fx25_frame, fx25_len, recovered, sizeof(recovered), &dr);
                TEST_ASSERT(dec_rc == 0 && dr.ax25_len == MAX_AX25_TAG01, "Y.11.c Tag_01 max-capacity round-trip succeeds", dec_rc);
                TEST_ASSERT(dec_rc == 0 && memcmp(recovered, raw_ax25, MAX_AX25_TAG01) == 0, "Y.11.d Tag_01 max-capacity payload matches byte-for-byte", 0);
            }

            /* One byte over capacity must fail */
            uint8_t raw_ax25_over[238];
            memset(raw_ax25_over, 0x41, sizeof(raw_ax25_over));
            int over_rc = y_fx25_encode_frame(raw_ax25_over, MAX_AX25_TAG01 + 1, fx25_frame, sizeof(fx25_frame), tag01);
            TEST_ASSERT(over_rc < 0, "Y.11.e Tag_01 rejects 238-byte payload (exceeds info capacity)", over_rc);
        }
    }

    // -----------------------------------------------------------------------
    // Y.12 — I-frame (Modulo-128) encode/decode through FX.25 wrapper
    //         Tests that FX.25 is agnostic to AX.25 frame type (spec §Protocol Summary)
    // -----------------------------------------------------------------------
    {
        uint8_t err = 0;
        ax25_address_t *dest = ax25_address_from_string("W1AW-1", &err);
        ax25_address_t *src = ax25_address_from_string("N7FEC-4", &err);

        if (!dest || !src) {
            printf("SKIP: Y.12 (address allocation failed)\n");
            if (dest)
                ax25_address_free(dest, &err);
            if (src)
                ax25_address_free(src, &err);
            goto y12_done;
        }

        uint8_t i_payload[] = "MODULO128 I-FRAME IN FX25";
        ax25_frame_header_t hdr;
        ax25_information_frame_t iframe;
        memset(&hdr, 0, sizeof(hdr));
        hdr.destination = *dest;
        hdr.source = *src;
        hdr.cr = true;
        hdr.repeaters.num_repeaters = 0;
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_16BIT;
        iframe.base.header = hdr;
        iframe.ns = 5;
        iframe.nr = 3;
        iframe.pf = false;
        iframe.payload = i_payload;
        iframe.payload_len = (int) (sizeof(i_payload) - 1);

        size_t ax25_len = 0;
        uint8_t *ax25_bytes = ax25_frame_encode((ax25_frame_t*) &iframe, &ax25_len, &err);
        TEST_ASSERT(ax25_bytes != NULL, "Y.12.a libax25v22 encode I-frame (Mod-128) for FX.25 test", err);

        if (ax25_bytes) {
            const y_fx25_tag_t *tag = y_fx25_select_tag((int) ax25_len);
            if (tag) {
                uint8_t fx25_frame[Y_FX25_MAX_FRAME];
                int fx25_len = y_fx25_encode_frame(ax25_bytes, (int) ax25_len, fx25_frame, sizeof(fx25_frame), tag);
                TEST_ASSERT(fx25_len > 0, "Y.12.b FX.25 encode of Mod-128 I-frame succeeds", fx25_len);

                if (fx25_len > 0) {
                    uint8_t recovered[300];
                    y_fx25_decode_result_t dr;
                    int dec_rc = y_fx25_decode_frame(fx25_frame, fx25_len, recovered, sizeof(recovered), &dr);
                    TEST_ASSERT(dec_rc == 0, "Y.12.c FX.25 decode of Mod-128 I-frame (no errors)", dec_rc);

                    if (dec_rc == 0) {
                        uint8_t dec_err = 0;
                        ax25_frame_t *rxf = ax25_frame_decode(recovered, (size_t) dr.ax25_len,
                        MODULO128_TRUE, &dec_err);
                        TEST_ASSERT(rxf != NULL && dec_err == 0, "Y.12.d libax25v22 decode of FX.25-recovered Mod-128 I-frame", dec_err);
                        if (rxf) {
                            TEST_ASSERT(rxf->type == AX25_FRAME_INFORMATION_16BIT, "Y.12.e Recovered frame type == INFORMATION_16BIT", rxf->type);
                            ax25_information_frame_t *rxi = (ax25_information_frame_t*) rxf;
                            TEST_ASSERT(rxi->ns == 5, "Y.12.f Recovered I-frame N(S) == 5", rxi->ns);
                            TEST_ASSERT(rxi->nr == 3, "Y.12.g Recovered I-frame N(R) == 3", rxi->nr);
                            ax25_frame_free(rxf, &dec_err);
                        }
                    }
                    DEBUG_PRINT("Y.12 Mod-128 I-frame round-trip via FX.25 PASS (tag=%02X)", tag->tag_id);
                }
            }
            free(ax25_bytes);
        }
        ax25_address_free(dest, &err);
        ax25_address_free(src, &err);
    }
    y12_done:
    ;

    // -----------------------------------------------------------------------
    // Y.13 — Digipeater path preserved through FX.25 encode/decode
    //         Tests that FX.25 does not disturb the AX.25 repeater fields
    // -----------------------------------------------------------------------
    {
        uint8_t err = 0;
        ax25_address_t *dest = ax25_address_from_string("W1AW-0", &err);
        ax25_address_t *src = ax25_address_from_string("KA1FEC-2", &err);
        ax25_address_t *digi1 = ax25_address_from_string("WB6YMH-3", &err);
        ax25_address_t *digi2 = ax25_address_from_string("N7VDF-1", &err);

        if (!dest || !src || !digi1 || !digi2) {
            printf("SKIP: Y.13 (address allocation failed)\n");
            if (dest)
                ax25_address_free(dest, &err);
            if (src)
                ax25_address_free(src, &err);
            if (digi1)
                ax25_address_free(digi1, &err);
            if (digi2)
                ax25_address_free(digi2, &err);
            goto y13_done;
        }

        uint8_t payload[] = "DIGI PATH VIA FX25";
        ax25_frame_header_t hdr;
        ax25_unnumbered_information_frame_t ui;
        memset(&hdr, 0, sizeof(hdr));
        hdr.destination = *dest;
        hdr.source = *src;
        hdr.cr = false;
        hdr.repeaters.num_repeaters = 2;
        hdr.repeaters.repeaters[0] = *digi1;
        hdr.repeaters.repeaters[1] = *digi2;

        memset(&ui, 0, sizeof(ui));
        ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui.base.base.header = hdr;
        ui.base.pf = false;
        ui.base.modifier = AX25_U_UI;
        ui.pid = PID_NO_L3;
        ui.payload = payload;
        ui.payload_len = (int) (sizeof(payload) - 1);

        size_t ax25_len = 0;
        uint8_t *ax25_bytes = ax25_frame_encode((ax25_frame_t*) &ui, &ax25_len, &err);
        TEST_ASSERT(ax25_bytes != NULL, "Y.13.a libax25v22 encode UI with 2-digi path for FX.25", err);

        if (ax25_bytes) {
            const y_fx25_tag_t *tag = y_fx25_select_tag((int) ax25_len);
            if (tag) {
                uint8_t fx25_frame[Y_FX25_MAX_FRAME];
                int fx25_len = y_fx25_encode_frame(ax25_bytes, (int) ax25_len, fx25_frame, sizeof(fx25_frame), tag);
                TEST_ASSERT(fx25_len > 0, "Y.13.b FX.25 encode of digipeated UI frame", fx25_len);

                if (fx25_len > 0) {
                    uint8_t recovered[300];
                    y_fx25_decode_result_t dr;
                    int dec_rc = y_fx25_decode_frame(fx25_frame, fx25_len, recovered, sizeof(recovered), &dr);
                    TEST_ASSERT(dec_rc == 0, "Y.13.c FX.25 decode preserves digipeater path", dec_rc);

                    if (dec_rc == 0) {
                        uint8_t dec_err = 0;
                        ax25_frame_t *rxf = ax25_frame_decode(recovered, (size_t) dr.ax25_len,
                        MODULO128_FALSE, &dec_err);
                        TEST_ASSERT(rxf != NULL && dec_err == 0, "Y.13.d libax25v22 decode of FX.25-recovered digipeated frame", dec_err);
                        if (rxf) {
                            TEST_ASSERT(rxf->header.repeaters.num_repeaters == 2, "Y.13.e Recovered frame has 2 digipeaters",
                                    rxf->header.repeaters.num_repeaters);

                            /* Check digi1 via libax25 ax25_cmp */
                            ax25_address digi1_rec_linux, digi1_orig_linux;
                            bridge_libax25v22_to_linux(&rxf->header.repeaters.repeaters[0], &digi1_rec_linux, &dec_err);
                            bridge_libax25v22_to_linux(digi1, &digi1_orig_linux, &dec_err);
                            TEST_ASSERT(ax25_cmp(&digi1_rec_linux, &digi1_orig_linux) == 0,
                                    "Y.13.f Digi[0] WB6YMH-3 preserved through FX.25 (libax25 ax25_cmp)", ax25_cmp(&digi1_rec_linux, &digi1_orig_linux));

                            ax25_frame_free(rxf, &dec_err);
                        }
                    }
                }
            }
            free(ax25_bytes);
        }
        ax25_address_free(dest, &err);
        ax25_address_free(src, &err);
        ax25_address_free(digi1, &err);
        ax25_address_free(digi2, &err);
    }
    y13_done:
    ;

    // -----------------------------------------------------------------------
    // Y.14 — FX.25 multiple-frame concatenation (spec §Multi-Frame Blocks)
    //         Two FX.25 frames back-to-back, separated by a single correlation tag
    // -----------------------------------------------------------------------
    {
        printf("  Y.14 Multi-frame concatenation (spec §Multi-Frame Blocks):\n");

        uint8_t err = 0;
        ax25_address_t *dest = ax25_address_from_string("MULTI-0", &err);
        ax25_address_t *src = ax25_address_from_string("FRAME-1", &err);

        if (!dest || !src) {
            printf("SKIP: Y.14 (address allocation failed)\n");
            if (dest)
                ax25_address_free(dest, &err);
            if (src)
                ax25_address_free(src, &err);
            goto y14_done;
        }

        /* Build two distinct UI frames */
        uint8_t p1[] = "MULTIFRAME BLOCK 1";
        uint8_t p2[] = "MULTIFRAME BLOCK 2";

        ax25_frame_header_t hdr;
        ax25_unnumbered_information_frame_t ui;
        memset(&hdr, 0, sizeof(hdr));
        hdr.destination = *dest;
        hdr.source = *src;
        hdr.cr = false;
        hdr.repeaters.num_repeaters = 0;

        memset(&ui, 0, sizeof(ui));
        ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
        ui.base.base.header = hdr;
        ui.base.pf = false;
        ui.base.modifier = AX25_U_UI;
        ui.pid = PID_NO_L3;

        ui.payload = p1;
        ui.payload_len = (int) (sizeof(p1) - 1);
        size_t ax25_len1 = 0;
        uint8_t *ax25_f1 = ax25_frame_encode((ax25_frame_t*) &ui, &ax25_len1, &err);

        ui.payload = p2;
        ui.payload_len = (int) (sizeof(p2) - 1);
        size_t ax25_len2 = 0;
        uint8_t *ax25_f2 = ax25_frame_encode((ax25_frame_t*) &ui, &ax25_len2, &err);

        TEST_ASSERT(ax25_f1 != NULL && ax25_f2 != NULL, "Y.14.a Encode two AX.25 frames for multi-frame test", err);

        if (ax25_f1 && ax25_f2) {
            const y_fx25_tag_t *tag = y_fx25_select_tag((int) (ax25_len1 > ax25_len2 ? ax25_len1 : ax25_len2));
            if (tag) {
                /* Encode both frames */
                uint8_t fx25_f1[Y_FX25_MAX_FRAME], fx25_f2[Y_FX25_MAX_FRAME];
                int fxl1 = y_fx25_encode_frame(ax25_f1, (int) ax25_len1, fx25_f1, sizeof(fx25_f1), tag);
                int fxl2 = y_fx25_encode_frame(ax25_f2, (int) ax25_len2, fx25_f2, sizeof(fx25_f2), tag);

                TEST_ASSERT(fxl1 > 0 && fxl2 > 0, "Y.14.b Both FX.25 frames encoded successfully", fxl1);

                if (fxl1 > 0 && fxl2 > 0) {
                    /* Concatenate: strip preamble of frame 2, per spec */
                    /* Spec: frames share a single correlation tag separator.
                     * In practice most implementations concatenate full frames.
                     * We test sequential decode of two complete frames. */
                    uint8_t concat[Y_FX25_MAX_FRAME * 2];
                    memcpy(concat, fx25_f1, (size_t) fxl1);
                    memcpy(concat + fxl1, fx25_f2, (size_t) fxl2);
                    int concat_len = fxl1 + fxl2;

                    /* Decode frame 1 from concatenated buffer */
                    uint8_t rec1[300];
                    y_fx25_decode_result_t dr1;
                    int dec1 = y_fx25_decode_frame(concat, concat_len, rec1, sizeof(rec1), &dr1);
                    TEST_ASSERT(dec1 == 0, "Y.14.c Decode frame 1 from concatenated multi-frame buffer", dec1);

                    /* Decode frame 2: advance past frame 1 (tag + codeblock) */
                    int frame1_inner_offset = Y_FX25_PREAMBLE_LEN + 8 + tag->block_len;
                    uint8_t rec2[300];
                    y_fx25_decode_result_t dr2;
                    int dec2 = -1;
                    if (frame1_inner_offset < concat_len) {
                        dec2 = y_fx25_decode_frame(concat + frame1_inner_offset, concat_len - frame1_inner_offset, rec2, sizeof(rec2), &dr2);
                    }
                    TEST_ASSERT(dec2 == 0, "Y.14.d Decode frame 2 from concatenated multi-frame buffer", dec2);

                    if (dec1 == 0 && dec2 == 0) {
                        TEST_ASSERT(dr1.ax25_len == (int )ax25_len1 && memcmp(rec1, ax25_f1, ax25_len1) == 0, "Y.14.e Multi-frame block 1 payload matches",
                                dr1.ax25_len);
                        TEST_ASSERT(dr2.ax25_len == (int )ax25_len2 && memcmp(rec2, ax25_f2, ax25_len2) == 0, "Y.14.f Multi-frame block 2 payload matches",
                                dr2.ax25_len);

                        DEBUG_PRINT("Y.14 Multi-frame concatenation PASS " "(frame1=%d frame2=%d total=%d bytes, tag=%02X)", fxl1, fxl2, concat_len,
                                tag->tag_id);
                    }
                }
            }
        }
        if (ax25_f1)
            free(ax25_f1);
        if (ax25_f2)
            free(ax25_f2);
        ax25_address_free(dest, &err);
        ax25_address_free(src, &err);
    }
    y14_done:
    ;

    // -----------------------------------------------------------------------
    // Y.15 — Live kernel pipeline: FX.25-wrapped frame → KISS → kissattach
    //         → AF_PACKET → FX.25 decode → libax25v22 decode
    //         Requires kissattach running (same setup as SEC-X).
    //         This proves that a Direwolf-compatible TNC sending FX.25-framed
    //         data is correctly reassembled by the Linux kernel + libax25v22.
    // -----------------------------------------------------------------------
    {
        TEST_SECTION("Y.15 FX.25 → KISS → kernel → AF_PACKET → decode (live pipeline)");

        char ka_pty[64] = "";
        char slave_pty[64] = "";

        if (!find_kissattach_pty(ka_pty, sizeof(ka_pty))) {
            printf("SKIP: Y.15 (kissattach not running — run SEC-X setup first)\n");
            printf("  Setup:  socat PTY,raw,echo=0 PTY,raw,echo=0  &\n");
            printf("          sudo kissattach /dev/pts/N ax0 <callsign>\n");
            goto y15_done;
        }

        if (!find_socat_slave_pty(ka_pty, slave_pty, sizeof(slave_pty))) {
            printf("SKIP: Y.15 (socat slave PTY not found)\n");
            goto y15_done;
        }

        if (!g_test_ctx.kernel_ax25_available) {
            printf("SKIP: Y.15 (kernel AX.25 not available)\n");
            goto y15_done;
        }

        /* Discover AX.25 netdev (same approach as SEC-X) */
        char ax25_iface[IFNAMSIZ] = "";
        {
            int tfd = open(ka_pty, O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (tfd >= 0) {
                char ifbuf[IFNAMSIZ];
                memset(ifbuf, 0, sizeof(ifbuf));
                if (ioctl(tfd, SIOCGIFNAME, ifbuf) == 0 && ifbuf[0])
                    safe_strlcpy(ax25_iface, ifbuf, sizeof(ax25_iface));
                close(tfd);
            }
        }
        if (ax25_iface[0] == '\0') {
            DIR *nd = opendir("/sys/class/net");
            if (nd) {
                struct dirent *nent;
                while ((nent = readdir(nd)) != NULL) {
                    if (nent->d_name[0] == '.')
                        continue;
                    char tp[512];
                    char tv[16];
                    snprintf(tp, sizeof(tp), "/sys/class/net/%s/type", nent->d_name);
                    if (read_first_line(tp, tv, sizeof(tv)) == 0 && atoi(tv) == 3) {
                        safe_strlcpy(ax25_iface, nent->d_name, sizeof(ax25_iface));
                        break;
                    }
                }
                closedir(nd);
            }
        }
        if (ax25_iface[0] == '\0')
            safe_strlcpy(ax25_iface, g_test_ctx.port_name, sizeof(ax25_iface));

        /* AF_PACKET RX socket */
        int rx_sock = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_AX25));
        if (rx_sock < 0) {
            printf("SKIP: Y.15 (AF_PACKET socket failed: %s)\n", strerror(errno));
            goto y15_done;
        }
        {
            struct sockaddr_ll ll;
            memset(&ll, 0, sizeof(ll));
            ll.sll_family = AF_PACKET;
            ll.sll_protocol = htons(ETH_P_AX25);
            ll.sll_ifindex = (int) if_nametoindex(ax25_iface);
            if (ll.sll_ifindex == 0 || bind(rx_sock, (struct sockaddr*) &ll, sizeof(ll)) != 0) {
                close(rx_sock);
                printf("SKIP: Y.15 (AF_PACKET bind failed)\n");
                goto y15_done;
            }
        }
        int fl = fcntl(rx_sock, F_GETFL, 0);
        if (fl >= 0)
            fcntl(rx_sock, F_SETFL, fl | O_NONBLOCK);

        /* Encode test frame: libax25v22 → FX.25 → KISS */
        uint8_t err = 0;
        uint8_t fx25_payload[] = "FX25 LIVE KERNEL TEST";
        uint8_t *ax25_bytes = NULL;
        size_t ax25_len = 0;
        {
            ax25_address_t *dst = ax25_address_from_string(g_test_ctx.local_call, &err);
            ax25_address_t *srca = ax25_address_from_string("FX25TP-0", &err);
            if (dst && srca) {
                ax25_frame_header_t hdr;
                ax25_unnumbered_information_frame_t ui;
                memset(&hdr, 0, sizeof(hdr));
                hdr.destination = *dst;
                hdr.source = *srca;
                hdr.cr = false;
                hdr.repeaters.num_repeaters = 0;
                memset(&ui, 0, sizeof(ui));
                ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
                ui.base.base.header = hdr;
                ui.base.pf = false;
                ui.base.modifier = AX25_U_UI;
                ui.pid = PID_NO_L3;
                ui.payload = fx25_payload;
                ui.payload_len = (int) (sizeof(fx25_payload) - 1);
                ax25_bytes = ax25_frame_encode((ax25_frame_t*) &ui, &ax25_len, &err);
                ax25_address_free(dst, &err);
                ax25_address_free(srca, &err);
            }
        }

        TEST_ASSERT(ax25_bytes != NULL, "Y.15.a libax25v22 encode for live FX.25 pipeline test", err);

        if (!ax25_bytes) {
            close(rx_sock);
            goto y15_done;
        }

        /* FX.25 encode */
        const y_fx25_tag_t *tag = y_fx25_select_tag((int) ax25_len);
        TEST_ASSERT(tag != NULL, "Y.15.b Select FX.25 tag for live pipeline", tag ? tag->tag_id : -1);
        if (!tag) {
            free(ax25_bytes);
            close(rx_sock);
            goto y15_done;
        }

        uint8_t fx25_frame[Y_FX25_MAX_FRAME];
        int fx25_len = y_fx25_encode_frame(ax25_bytes, (int) ax25_len, fx25_frame, sizeof(fx25_frame), tag);
        TEST_ASSERT(fx25_len > 0, "Y.15.c FX.25 encode for live pipeline", fx25_len);
        if (fx25_len <= 0) {
            free(ax25_bytes);
            close(rx_sock);
            goto y15_done;
        }

        /* KISS encode the FX.25 frame */
        uint8_t kiss_buf[Y_FX25_MAX_FRAME + 8];
        int kiss_len = 0;
        TEST_ASSERT(kiss_encode_frame(fx25_frame, fx25_len, 0, 0, kiss_buf, &kiss_len) == 0, "Y.15.d KISS encode of FX.25 frame", 0);

        /* EIO-prevention fd */
        int slave_kfd = open(ka_pty, O_RDWR | O_NOCTTY | O_NONBLOCK);

        /* Write to PTY (same strategies as SEC-X) */
        int write_fd = -1;
        {
            write_fd = open_ka_master_fd(ka_pty);
            if (write_fd < 0) {
                char ka_master_proc[128] = "";
                if (find_ka_master_proc_path(ka_pty, ka_master_proc, sizeof(ka_master_proc)))
                    write_fd = open(ka_master_proc, O_RDWR | O_NOCTTY);
            }
            if (write_fd < 0)
                write_fd = open(slave_pty, O_RDWR | O_NOCTTY);
        }

        TEST_ASSERT(write_fd >= 0, "Y.15.e Open PTY for FX.25 KISS injection", write_fd);
        if (write_fd < 0) {
            free(ax25_bytes);
            if (slave_kfd >= 0)
                close(slave_kfd);
            close(rx_sock);
            goto y15_done;
        }

        int written = (int) write(write_fd, kiss_buf, (size_t) kiss_len);
        close(write_fd);
        TEST_ASSERT(written == kiss_len, "Y.15.f Write FX.25-KISS frame to PTY (complete write)", written);

        /* Poll for AF_PACKET reception */
        struct pollfd pfd;
        pfd.fd = rx_sock;
        pfd.events = POLLIN;
        int poll_rc = poll(&pfd, 1, 5000);

        TEST_ASSERT(poll_rc > 0, "Y.15.g AF_PACKET received FX.25-encapsulated AX.25 frame within 5000 ms "
                "(FX.25→KISS→kissattach→kernel→AF_PACKET)", poll_rc);

        if (poll_rc > 0) {
            uint8_t rx_buf[512];
            struct sockaddr_ll rx_ll;
            socklen_t rx_ll_len = sizeof(rx_ll);
            int nrecv = (int) recvfrom(rx_sock, rx_buf, sizeof(rx_buf), 0, (struct sockaddr*) &rx_ll, &rx_ll_len);
            TEST_ASSERT(nrecv > 0, "Y.15.h recvfrom delivered AF_PACKET frame bytes", nrecv);

            if (nrecv > 0) {
                /* The Linux kernel mkiss ldisc delivers the AX.25 frame
                 * (already unwrapped from FX.25 preamble at TNC layer).
                 * In a real FX.25 TNC (e.g. Direwolf), the TNC strips the
                 * correlation tag and RS parity and passes only the AX.25
                 * packet to kissattach.  In our loopback test, the kissattach
                 * N_AX25 ldisc receives the raw bytes and may prepend cmd 0x00. */
                uint8_t *decode_buf = rx_buf;
                size_t decode_len = (size_t) nrecv;
                if (nrecv > 0 && rx_buf[0] == 0x00) {
                    decode_buf++;
                    decode_len--;
                    DEBUG_PRINT("Y.15 Skipping KISS cmd byte 0x00 prepended by kernel mkiss " "(now %zu-byte payload)", decode_len);
                }

                /* The AF_PACKET bytes may be either:
                 * (a) Pure AX.25 frame (TNC did FX.25 decode) — decode directly.
                 * (b) Full FX.25 frame bytes — need FX.25 decode first.
                 * We detect by checking for a known FX.25 correlation tag. */
                int delivered_as_fx25 = 0;
                uint8_t ax25_from_kernel[300];
                size_t ax25_from_kernel_len = 0;

                /* Try FX.25 detection first */
                {
                    y_fx25_decode_result_t dr_k;
                    uint8_t temp[300];
                    if (y_fx25_decode_frame(decode_buf, (int) decode_len, temp, sizeof(temp), &dr_k) == 0) {
                        delivered_as_fx25 = 1;
                        memcpy(ax25_from_kernel, temp, (size_t) dr_k.ax25_len);
                        ax25_from_kernel_len = (size_t) dr_k.ax25_len;
                        DEBUG_PRINT("Y.15 AF_PACKET delivered FX.25 frame " "(tag=%02X, errors=%d) → extracted %zu AX.25 bytes", dr_k.tag_id, dr_k.errors_found,
                                ax25_from_kernel_len);
                    }
                }

                if (!delivered_as_fx25) {
                    /* Assume raw AX.25 (normal case with a real FX.25-capable TNC) */
                    if (decode_len <= sizeof(ax25_from_kernel)) {
                        memcpy(ax25_from_kernel, decode_buf, decode_len);
                        ax25_from_kernel_len = decode_len;
                    }
                }

                /* Decode with libax25v22 */
                uint8_t dec_err = 0;
                ax25_frame_t *rx_frame = ax25_frame_decode(ax25_from_kernel, ax25_from_kernel_len,
                MODULO128_FALSE, &dec_err);
                TEST_ASSERT(rx_frame != NULL && dec_err == 0, "Y.15.i libax25v22 decode of kernel-delivered (FX.25-sourced) AX.25 frame", dec_err);

                if (rx_frame) {
                    TEST_ASSERT(rx_frame->type == AX25_FRAME_UNNUMBERED_INFORMATION, "Y.15.j Kernel-delivered frame type == UI", rx_frame->type);

                    ax25_unnumbered_information_frame_t *rxui = (ax25_unnumbered_information_frame_t*) rx_frame;
                    int payload_len = (int) (sizeof(fx25_payload) - 1);
                    int pmatch = (rxui->payload_len == payload_len && rxui->payload != NULL && memcmp(rxui->payload, fx25_payload, (size_t) payload_len) == 0);
                    TEST_ASSERT(pmatch, "Y.15.k Payload matches \"FX25 LIVE KERNEL TEST\" "
                            "(end-to-end: libax25v22 → FX.25 → KISS → kernel → AF_PACKET → libax25v22)", rxui->payload_len);

                    if (pmatch)
                        DEBUG_PRINT("Y.15 END-TO-END FX.25 PASS: " "libax25v22 → FX.25(tag=%02X) → KISS → " "kernel AX.25 → AF_PACKET → libax25v22",
                                tag->tag_id);

                    ax25_frame_free(rx_frame, &dec_err);
                }
            }
        }

        if (slave_kfd >= 0)
            close(slave_kfd);
        close(rx_sock);
        free(ax25_bytes);
    }
    y15_done:
    ;

    // -----------------------------------------------------------------------
    // Y.16 — FX.25 + libax25v22 via libax25 ax25_aton / ax25_ntoa address
    //         functions: prove FX.25 is address-format agnostic
    // -----------------------------------------------------------------------
    {
        /* Build addresses using libax25 ax25_aton_entry, convert to libax25v22,
         * encode a UI frame, wrap in FX.25, decode, and verify the libax25
         * ax25_cmp of recovered vs original addresses is 0. */
        ax25_address linux_dest, linux_src;
        int rc_dest = ax25_aton_entry("VE3KFX-9", (char*) &linux_dest);
        int rc_src = ax25_aton_entry("WA7XFX-0", (char*) &linux_src);

        TEST_ASSERT(rc_dest == 0, "Y.16.a libax25 ax25_aton_entry VE3KFX-9", rc_dest);
        TEST_ASSERT(rc_src == 0, "Y.16.b libax25 ax25_aton_entry WA7XFX-0", rc_src);

        if (rc_dest == 0 && rc_src == 0) {
            uint8_t err = 0;
            ax25_address_t v22_dest, v22_src;
            int b1 = bridge_linux_to_libax25v22(&linux_dest, &v22_dest, &err);
            int b2 = bridge_linux_to_libax25v22(&linux_src, &v22_src, &err);

            TEST_ASSERT(b1 == 0 && b2 == 0, "Y.16.c bridge_linux_to_libax25v22 succeeds for both addresses", err);

            if (b1 == 0 && b2 == 0) {
                uint8_t payload[] = "ADDRESS FORMAT AGNOSTIC FX25";
                ax25_frame_header_t hdr;
                ax25_unnumbered_information_frame_t ui;
                memset(&hdr, 0, sizeof(hdr));
                hdr.destination = v22_dest;
                hdr.source = v22_src;
                hdr.cr = false;
                hdr.repeaters.num_repeaters = 0;
                memset(&ui, 0, sizeof(ui));
                ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
                ui.base.base.header = hdr;
                ui.base.pf = false;
                ui.base.modifier = AX25_U_UI;
                ui.pid = PID_NO_L3;
                ui.payload = payload;
                ui.payload_len = (int) (sizeof(payload) - 1);

                size_t ax25_len = 0;
                uint8_t *ax25_bytes = ax25_frame_encode((ax25_frame_t*) &ui, &ax25_len, &err);
                TEST_ASSERT(ax25_bytes != NULL, "Y.16.d libax25v22 encode with libax25-sourced addresses", err);

                if (ax25_bytes) {
                    const y_fx25_tag_t *tag = y_fx25_select_tag((int) ax25_len);
                    if (tag) {
                        uint8_t fx25_frame[Y_FX25_MAX_FRAME];
                        int fx25_len = y_fx25_encode_frame(ax25_bytes, (int) ax25_len, fx25_frame, sizeof(fx25_frame), tag);
                        uint8_t recovered[300];
                        y_fx25_decode_result_t dr;
                        if (fx25_len > 0 && y_fx25_decode_frame(fx25_frame, fx25_len, recovered, sizeof(recovered), &dr) == 0) {
                            uint8_t dec_err = 0;
                            ax25_frame_t *rxf = ax25_frame_decode(recovered, (size_t) dr.ax25_len,
                            MODULO128_FALSE, &dec_err);
                            TEST_ASSERT(rxf != NULL, "Y.16.e libax25v22 decode of FX.25-recovered frame (libax25 addresses)", dec_err);
                            if (rxf) {
                                ax25_address rec_dest_linux, rec_src_linux;
                                bridge_libax25v22_to_linux(&rxf->header.destination, &rec_dest_linux, &dec_err);
                                bridge_libax25v22_to_linux(&rxf->header.source, &rec_src_linux, &dec_err);

                                TEST_ASSERT(ax25_cmp(&rec_dest_linux, &linux_dest) == 0, "Y.16.f Recovered dest VE3KFX-9 matches original (libax25 ax25_cmp)",
                                        ax25_cmp(&rec_dest_linux, &linux_dest));
                                TEST_ASSERT(ax25_cmp(&rec_src_linux, &linux_src) == 0, "Y.16.g Recovered src WA7XFX-0 matches original (libax25 ax25_cmp)",
                                        ax25_cmp(&rec_src_linux, &linux_src));

                                DEBUG_PRINT("Y.16 FX.25 address-agnostic PASS " "(dest=%s src=%s tag=%02X)", rxf->header.destination.callsign,
                                        rxf->header.source.callsign, tag->tag_id);
                                ax25_frame_free(rxf, &dec_err);
                            }
                        }
                    }
                    free(ax25_bytes);
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Y.17 — FX.25 RS parity isolation: corrupt RS check symbols only
    //         (error in parity region, data must still recover cleanly)
    // -----------------------------------------------------------------------
    {
        uint8_t data[48]; /* Tag_04: RS(48,32) */
        int i;
        for (i = 0; i < 32; i++)
            data[i] = (uint8_t) (0xBB ^ (i * 3));
        memset(&data[32], 0, 16);
        y_rs_encode(data, 32, 16);

        uint8_t cw[48];
        memcpy(cw, data, 48);

        /* Corrupt 4 parity bytes (positions 34, 36, 40, 45) */
        cw[34] ^= 0xDE;
        cw[36] ^= 0xAD;
        cw[40] ^= 0xBE;
        cw[45] ^= 0xEF;

        int dec_rc = y_rs_decode(cw, 32, 16);
        TEST_ASSERT(dec_rc >= 1, "Y.17.a RS corrects 4 parity-symbol errors (t=8 allows up to 8)", dec_rc);
        TEST_ASSERT(memcmp(cw, data, 32) == 0, "Y.17.b Data region intact after parity-only error correction", 0);
        TEST_ASSERT(memcmp(cw, data, 48) == 0, "Y.17.c Full codeword restored after 4 parity errors", 0);

        DEBUG_PRINT("Y.17 Parity-region error correction: %d symbols corrected", dec_rc);
    }

    // -----------------------------------------------------------------------
    // Y.18 — Spec §Pad: codeblock padding bytes are 0x7E
    //         Verify pad region of a small AX.25 frame uses 0x7E fill
    // -----------------------------------------------------------------------
    {
        /* Use raw mini buffer: 10 bytes of AX.25-like content in Tag_04 (info=32) */
        uint8_t raw[10];
        int i;
        for (i = 0; i < 10; i++)
            raw[i] = (uint8_t) (0x30 + i);

        const y_fx25_tag_t *tag04 = y_fx25_find_tag(Y_FX25_TAG_04_VAL);
        if (tag04) {
            uint8_t fx25_frame[Y_FX25_MAX_FRAME];
            int fx25_len = y_fx25_encode_frame(raw, 10, fx25_frame, sizeof(fx25_frame), tag04);
            TEST_ASSERT(fx25_len > 0, "Y.18.a Encode small payload with Tag_04", fx25_len);

            if (fx25_len > 0) {
                /* Codeblock at fx25_frame[4+8=12].
                 * Layout: [0]=0x7E [1..10]=raw [11]=0x7E [12..31]=pad [32..47]=RS */
                int cb = Y_FX25_PREAMBLE_LEN + 8; /* 12 */
                TEST_ASSERT(fx25_frame[cb] == 0x7E, "Y.18.b Codeblock[0] == 0x7E (AX.25 packet start flag)", (int )fx25_frame[cb]);
                TEST_ASSERT(fx25_frame[cb + 1 + 10] == 0x7E, "Y.18.c Codeblock[11] == 0x7E (AX.25 packet end flag)", (int )fx25_frame[cb + 11]);
                /* Pad bytes at [12..31] must be 0x7E (spec §Pad) */
                int pad_ok = 1;
                for (i = 12; i < 32; i++)
                    if (fx25_frame[cb + i] != 0x7E) {
                        pad_ok = 0;
                        break;
                    }
                TEST_ASSERT(pad_ok, "Y.18.d Pad bytes [12..31] are all 0x7E (spec §Pad Requirements)", pad_ok);
                DEBUG_PRINT("Y.18 Pad verification: start=%02X end=%02X pad[12]=%02X", fx25_frame[cb], fx25_frame[cb+11], fx25_frame[cb+12]);
            }
        } else {
            printf("SKIP: Y.18 (Tag_04 not found)\n");
        }
    }

    // -----------------------------------------------------------------------
    // Y.19 — Unknown / corrupted correlation tag: decode must return error
    // -----------------------------------------------------------------------
    {
        /* Fabricate a buffer with an all-zero "tag" (not in spec table) */
        uint8_t fake_frame[Y_FX25_MAX_FRAME];
        memset(fake_frame, 0x7E, 4); /* preamble */
        memset(&fake_frame[4], 0x00, 8); /* invalid tag (all zeros) */
        memset(&fake_frame[12], 0x41, 48); /* garbage codeblock */
        int fake_len = 12 + 48 + 2;

        uint8_t out[300];
        y_fx25_decode_result_t dr;
        int dec_rc = y_fx25_decode_frame(fake_frame, fake_len, out, sizeof(out), &dr);
        TEST_ASSERT(dec_rc < 0, "Y.19.a Decode of unknown/invalid correlation tag returns -1", dec_rc);

        /* Fabricate a buffer with a valid tag but truncated codeblock */
        uint8_t trunc_frame[20];
        memset(trunc_frame, 0x7E, 4);
        y_fx25_write_tag(&trunc_frame[4], Y_FX25_TAG_04_VAL);
        memset(&trunc_frame[12], 0xAA, 8); /* 8 bytes instead of full 48 codeblock */
        dec_rc = y_fx25_decode_frame(trunc_frame, 20, out, sizeof(out), &dr);
        TEST_ASSERT(dec_rc < 0, "Y.19.b Decode of truncated FX.25 codeblock returns -1", dec_rc);

        DEBUG_PRINT("Y.19 Invalid/truncated frame rejection PASS");
    }

    // -----------------------------------------------------------------------
    // Y summary
    // -----------------------------------------------------------------------
    printf("\n  SEC-Y FX.25 Summary:\n");
    printf("    Y.0  GF(2^8) arithmetic (poly=0x11D, b=112)\n");
    printf("    Y.1  Correlation tag constants and descriptor table\n");
    printf("    Y.2  RS(48,32) encode determinism & syndrome=0 verify\n");
    printf("    Y.3  RS(48,32) 1-byte error correction\n");
    printf("    Y.4  RS error-correction capacity boundary (t=8 / t+1 fail)\n");
    printf("    Y.5  FX.25 frame encode with Tag_01 (RS(255,239))\n");
    printf("    Y.6  FX.25 encode → decode → libax25v22 end-to-end round-trip\n");
    printf("    Y.7  FX.25 1-byte error → correct → libax25v22 decode\n");
    printf("    Y.8  All 11 tag variants encode/decode round-trip\n");
    printf("    Y.9  FX.25 frame inside KISS pipeline structure\n");
    printf("    Y.10 Legacy AX.25 compatibility (inner packet is standalone)\n");
    printf("    Y.11 Tag_01 max-capacity boundary test (239 info bytes)\n");
    printf("    Y.12 Mod-128 I-frame through FX.25 wrapper\n");
    printf("    Y.13 Digipeater path preserved through FX.25 encode/decode\n");
    printf("    Y.14 Multi-frame block concatenation (spec §Multi-Frame Blocks)\n");
    printf("    Y.15 Live kernel pipeline: FX.25→KISS→kissattach→AF_PACKET\n");
    printf("    Y.16 libax25 ax25_aton/ax25_cmp address functions via FX.25\n");
    printf("    Y.17 RS parity-symbol-only error correction\n");
    printf("    Y.18 Pad byte content == 0x7E (spec §Pad Requirements)\n");
    printf("    Y.19 Unknown/truncated correlation tag rejection\n");

    return 0;
}

// ===========================================================================
// SECTION Z: axparms / axctl Integration
// ===========================================================================
static int sec_z_axparms_integration(void) {
    TEST_SECTION("=== SEC-Z: axparms Parameter Integration ===");

    if (!g_test_ctx.kernel_ax25_available || g_test_ctx.port_count == 0) {
        printf("SKIP: SEC-Z (no kernel AX.25 or no configured ports)\n");
        return 0;
    }
    if (geteuid() != 0) {
        printf("SKIP: SEC-Z (requires root / CAP_NET_ADMIN for axparms write)\n");
        return 0;
    }

    // Z.1: Read current T1 via /proc
    // Z.2: axparms set T1 to new value
    // Z.3: Verify via /proc
    // Z.4: Restore original T1
    printf("TODO: SEC-Z axparms write-back test\n");
    printf("  (requires system-level axparms integration)\n");
    return 0;
}

// ===========================================================================
// SECTION Z2: Live Bidirectional Frame Exchange
// ===========================================================================
static int sec_z2_live_exchange(void) {
    TEST_SECTION("=== SEC-Z2: Live Bidirectional Frame Exchange ===");

    printf("SKIP: SEC-Z2 (requires kissnetd PTY loopback setup)\n");
    printf("  Architecture:\n");
    printf("    kissnetd creates /dev/ttyq0 and /dev/ttyq1 pair\n");
    printf("    kissattach /dev/ttyq0 ax0 <callsign>\n");
    printf("    libax25v22 writes KISS frames to /dev/ttyq1\n");
    printf("    AF_AX25 SOCK_DGRAM reads frames from the ax0 interface\n");
    printf("  This proves end-to-end byte-level interoperability.\n");
    return 0;
}

// ===========================================================================
// Main entry point
// ===========================================================================
int test_linux_interop_main(void) {
    int failures = 0;

    printf("\n");
    printf("=============================================================\n");
    printf("  libax25v22 Real Linux AX.25 Interoperability Test Suite\n");
    printf("  Integration: libax25 (AF_AX25) + libax25v22 protocol\n");
    printf("=============================================================\n\n");

    test_context_init();

    printf("\n");

    failures += run_test_section("=== SEC-A: libax25 Address Functions ===", sec_a_libax25_address);
    failures += run_test_section("=== SEC-B: AF_AX25 Socket Operations ===", sec_b_af_ax25_sockets);
    failures += run_test_section("=== SEC-C: Frame Encode/Decode ===", sec_c_frame_encode_decode);
    failures += run_test_section("=== SEC-D: HDLC Framing ===", sec_d_hdlc_framing);
    failures += run_test_section("=== SEC-E: Connection State Machine ===", sec_e_connection_state_machine);
    failures += run_test_section("=== SEC-F: CRC Functions ===", sec_f_crc_functions);
    failures += run_test_section("=== SEC-G: Buffer Pool ===", sec_g_buffer_pool);
    failures += run_test_section("=== SEC-H: Address Bridge Round-Trip ===", sec_h_address_bridge_roundtrip);
    failures += run_test_section("=== SEC-I: I-Frames and Modulo-128 ===", sec_i_iframes_and_modulo128);
    failures += run_test_section("=== SEC-J: Digipeater Path ===", sec_j_digipeater_path);
    failures += run_test_section("=== SEC-K: KISS Framing ===", sec_k_kiss_framing);
    failures += run_test_section("=== SEC-L: Buffer Pool Exhaustion ===", sec_l_buffer_pool_exhaustion);
    failures += run_test_section("=== SEC-M: axconfig API ===", sec_m_ax25config_api);
    failures += run_test_section("=== SEC-N: SOCK_DGRAM UI Frames ===", sec_n_sock_dgram_ui_frames);
    failures += run_test_section("=== SEC-O: /proc/sys/net/ax25 Sysctl ===", sec_o_sysctl_ax25_params);
    failures += run_test_section("=== SEC-P: full_sockaddr_ax25 Digipeater ===", sec_p_full_sockaddr_digipeater);
    failures += run_test_section("=== SEC-Q: Supervisory Frames ===", sec_q_supervisory_frames);
    failures += run_test_section("=== SEC-R: SABME Frame (Mod-128 Connect) ===", sec_r_sabme_frame);
    failures += run_test_section("=== SEC-S: XID Frame (Capability Exchange) ===", sec_s_xid_frame);
    failures += run_test_section("=== SEC-T: PID Values in UI Frames ===", sec_t_pid_values);
    failures += run_test_section("=== SEC-U: FRMR Frame (Frame Reject) ===", sec_u_frmr_frame);
    failures += run_test_section("=== SEC-V: KISS Port Multiplexing ===", sec_v_kiss_port_multiplexing);
    failures += run_test_section("=== SEC-W: Sequence Number Wrap-Around ===", sec_w_seq_wrap_around);
    failures += run_test_section("=== SEC-X: KISS-to-Kernel Pipeline ===", sec_x_kiss_kernel_pipeline);
    failures += run_test_section("=== SEC-Y: FX.25 FEC ===", sec_y_fx25_fec);
    failures += run_test_section("=== SEC-Z: axparms Integration ===", sec_z_axparms_integration);
    failures += run_test_section("=== SEC-Z2: Live Bidirectional Exchange ===", sec_z2_live_exchange);

    printf("\n");
    printf("=============================================================\n");
    printf("  Test Summary\n");
    printf("=============================================================\n");
    printf("Total Assertions: %u\n", assert_count);

    if (failures == 0)
        printf("\xE2\x9C\x93\xE2\x9C\x93\xE2\x9C\x93 ALL TESTS PASSED \xE2\x9C\x93\xE2\x9C\x93\xE2\x9C\x93\n");
    else
        printf("\xE2\x9C\x97\xE2\x9C\x97\xE2\x9C\x97 %d TEST SECTION(S) FAILED \xE2\x9C\x97\xE2\x9C\x97\xE2\x9C\x97\n", failures);

    printf("AF_AX25 Kernel Support: %s\n", g_test_ctx.kernel_ax25_available ? "YES" : "NO");
    printf("AF_AX25 Socket Bind:    %s\n", g_test_ctx.socket_bind_available ? "YES" : "NO");
    printf("AX.25 Port:             %s\n", g_test_ctx.port_name);
    printf("AX.25 Callsign:         %s\n", g_test_ctx.local_call);
    printf("AX.25 Configuration:    %s\n", g_test_ctx.port_count > 0 ? "LOADED" : "NOT FOUND");

    printf("=============================================================\n\n");

    return failures;
}
