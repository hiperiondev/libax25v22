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

// Test configuration and macros
#define TEST_AXPORTS_FILE "/etc/ax25/axports"
#define TEST_DEFAULT_PORT "ax0"
#define TEST_DEFAULT_CALL "TEST-0"
#define MAX_AXPORTS_LINE 256
#define MAX_CALLSIGN_LEN 16
#define MAX_PORT_NAME_LEN 32
#define MAX_UI_PAYLOAD_SIZE 256
#define MAX_INFO_PAYLOAD_SIZE 256
#define HAL_CRC16_CCITT_INIT 0xFFFF
#define AX25_TIMER_TICK_MS 100

// Provide MODULO128_TRUE fallback if not defined in ax25.h
#ifndef MODULO128_TRUE
#define MODULO128_TRUE ((uint8_t)1)
#endif

// KISS protocol constants - fallback defines in case kiss.h does not export them.
// These are fixed by the KISS specification (TNC-2 standard, 1987).
#ifndef KISS_FEND
#define KISS_FEND  ((uint8_t)0xC0)  // Frame End delimiter
#endif
#ifndef KISS_FESC
#define KISS_FESC  ((uint8_t)0xDB)  // Frame Escape
#endif
#ifndef KISS_TFEND
#define KISS_TFEND ((uint8_t)0xDC)  // Transposed Frame End (follows FESC)
#endif
#ifndef KISS_TFESC
#define KISS_TFESC ((uint8_t)0xDD)  // Transposed Frame Escape (follows FESC)
#endif

// Enhanced data structures for comprehensive testing
typedef struct {
    int kernel_ax25_available;
    int socket_bind_available;
    char port_name[MAX_PORT_NAME_LEN];
    char local_call[MAX_CALLSIGN_LEN];
    int port_count;
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

// Helper function for safe string handling
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

// Inline KISS encode helper.
// Builds a complete KISS frame: FEND | (port<<4|cmd) | escaped_data | FEND.
// Does not call any libax25v22 kiss module function so the test is independent
// of the actual kiss.h API. Returns 0 on success, -1 on overflow.
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

// Inline KISS decode helper.
// Strips the FEND delimiters, skips the port/command byte, and un-escapes the
// payload. Returns 0 on success, -1 on framing error.
static int kiss_decode_frame(const uint8_t *data, int data_len, uint8_t *out, int *out_len) {
    int n = 0;
    int i;

    if (!data || !out || !out_len || data_len < 4)
        return -1;

    // Expect opening FEND
    if (data[0] != KISS_FEND)
        return -1;

    i = 1;  // skip opening FEND
    i++;  // skip port/command byte

    // Copy payload up to closing FEND, un-escaping
    while (i < data_len) {
        if (data[i] == KISS_FEND)
            break;  // closing delimiter

        if (data[i] == KISS_FESC) {
            i++;
            if (i >= data_len)
                return -1;
            if (data[i] == KISS_TFEND) {
                out[n++] = KISS_FEND;
            } else if (data[i] == KISS_TFESC) {
                out[n++] = KISS_FESC;
            } else {
                return -1;  // invalid escape sequence
            }
        } else {
            out[n++] = data[i];
        }
        i++;
    }

    *out_len = n;
    return 0;
}

// Fixed SSID byte validation
static int validate_ssid_byte_encoding(uint8_t ssid_byte, uint8_t *err) {
    *err = 0;

    // Extract SSID (bits 1-4)
    uint8_t ssid = (ssid_byte >> 1) & 0x0F;
    if (ssid > 15) {
        *err = 1;
        DEBUG_PRINT("Invalid SSID in byte: 0x%02X", ssid_byte);
        return -1;
    }

    // Bit 5 should typically be 1 for data frames
    if (((ssid_byte >> 5) & 0x01) == 0) {
        DEBUG_PRINT("Warning: Bit 5 (RES0) not set in SSID byte 0x%02X", ssid_byte);
    }

    return 0;
}

__attribute__((unused))
static int bridge_linux_to_libax25v22(const ax25_address *linux_addr, ax25_address_t *v22_addr, uint8_t *err) {
    *err = 0;
    if (!linux_addr || !v22_addr) {
        *err = 1;
        return -1;
    }

    // Decode callsign: 6 characters, each left-shifted by 1 bit in Linux format
    for (int i = 0; i < 6; i++) {
        uint8_t shifted = (uint8_t) linux_addr->ax25_call[i];

        // In AX.25 encoding, bit 0 should always be 0 for callsign bytes
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
            DEBUG_PRINT("Invalid ASCII character at position %d: 0x%02X", i, ascii_char);
            return -1;
        }
    }
    v22_addr->callsign[6] = '\0';

    // Remove trailing spaces from callsign
    for (int i = 5; i >= 0; i--) {
        if (v22_addr->callsign[i] == ' ') {
            v22_addr->callsign[i] = '\0';
        } else {
            break;
        }
    }

    uint8_t ssid_byte = (uint8_t) linux_addr->ax25_call[6];

    if (validate_ssid_byte_encoding(ssid_byte, err) < 0) {
        return -1;
    }

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

__attribute__((unused))
static int bridge_libax25v22_to_linux(const ax25_address_t *v22_addr, ax25_address *linux_addr, uint8_t *err) {
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

    for (int i = 0; i < 6; i++) {
        unsigned char ascii_char;
        if (i < (int) callsign_len) {
            ascii_char = (unsigned char) v22_addr->callsign[i];
        } else {
            ascii_char = ' ';
        }

        if (ascii_char < 0x20 || ascii_char > 0x7E) {
            *err = 5;
            DEBUG_PRINT("Invalid character in callsign: 0x%02X", ascii_char);
            return -1;
        }

        linux_addr->ax25_call[i] = (char) (ascii_char << 1);
    }

    uint8_t ssid_byte = 0;
    ssid_byte |= ((uint8_t) (v22_addr->ssid & 0x0F) << 1);
    if (v22_addr->ch)
        ssid_byte |= 0x80;
    if (v22_addr->res1)
        ssid_byte |= 0x40;
    if (v22_addr->res0)
        ssid_byte |= 0x20;

    linux_addr->ax25_call[6] = (char) ssid_byte;

    DEBUG_PRINT("Encoded: callsign='%s' SSID=%d to byte=0x%02X", v22_addr->callsign, v22_addr->ssid, ssid_byte);

    return 0;
}

// Enhanced socket resource lifecycle management
static void cleanup_socket_resources(socket_resource_t *res) {
    if (!res)
        return;

    if (res->fd >= 0 && res->is_blocking_modified) {
        int flags = fcntl(res->fd, F_GETFL, 0);
        if (flags >= 0) {
            int result = fcntl(res->fd, F_SETFL, flags & ~O_NONBLOCK);
            if (result < 0) {
                DEBUG_PRINT("Warning: fcntl F_SETFL restore failed: %s", strerror(errno));
            }
        }
    }

    if (res->fd >= 0) {
        int close_result = close(res->fd);
        if (close_result < 0) {
            DEBUG_PRINT("Warning: close failed: %s", strerror(errno));
        }
        res->fd = -1;
    }

    res->is_bound = 0;
    res->is_blocking_modified = 0;
}

// Enhanced kernel AX.25 capability detection
static int detect_kernel_ax25_capabilities(kernel_ax25_capabilities_t *caps) {
    if (!caps) {
        return -1;
    }

    memset(caps, 0, sizeof(*caps));

    // Test 1: AF_AX25 SOCK_SEQPACKET support
    int sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
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
            DEBUG_PRINT("AF_AX25 exists but SOCK_SEQPACKET not supported");
        }
    }

    // Test 2: AF_AX25 SOCK_DGRAM support
    sock = socket(AF_AX25, SOCK_DGRAM, 0);
    if (sock >= 0) {
        caps->sock_dgram_supported = 1;
        close(sock);
    } else {
        DEBUG_PRINT("AF_AX25 SOCK_DGRAM not supported: %s", strerror(errno));
    }

    // Test 3: SO_BINDTODEVICE support (requires capability)
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock >= 0) {
        if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, "lo", 2) >= 0) {
            caps->so_bindtodevice_works = 1;
        } else if (errno == EPERM) {
            caps->so_bindtodevice_works = 1;
            DEBUG_PRINT("SO_BINDTODEVICE available but needs elevated privileges");
        }
        close(sock);
    }

    // Test 4: Check if ax25 modules are loaded
    if (caps->af_ax25_supported && caps->sock_seqpacket_supported) {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            struct sockaddr_ax25 addr;
            memset(&addr, 0, sizeof(addr));
            addr.sax25_family = AF_AX25;
            addr.sax25_ndigis = 0;

            if (bind(sock, (struct sockaddr*) &addr, sizeof(addr)) >= 0) {
                caps->modules_loaded = 1;
            } else if (errno == ENODEV) {
                DEBUG_PRINT("ax25 module may not be loaded or interface not found");
                caps->modules_loaded = 0;
            } else {
                caps->modules_loaded = 0;
                DEBUG_PRINT("Module check bind failed: %s", strerror(errno));
            }
            // Always close the probe socket regardless of bind result
            close(sock);
        }
    }

    return 0;
}

static int check_kernel_ax25_support_enhanced(void) {
    kernel_ax25_capabilities_t caps;

    if (detect_kernel_ax25_capabilities(&caps) < 0) {
        DEBUG_PRINT("Failed to detect kernel AX.25 capabilities");
        return 0;
    }

    if (!caps.af_ax25_supported) {
        DEBUG_PRINT("ERROR: AF_AX25 socket family not supported by kernel");
        DEBUG_PRINT("Required: Linux kernel with AX.25 protocol stack");
        DEBUG_PRINT("Install: apt install linux-image-generic (or your distro)");
        return 0;
    }

    if (!caps.sock_seqpacket_supported) {
        DEBUG_PRINT("ERROR: SOCK_SEQPACKET not available for AF_AX25");
        DEBUG_PRINT("This indicates incomplete AX.25 kernel configuration");
        return 0;
    }

    if (!caps.modules_loaded) {
        DEBUG_PRINT("WARNING: AX.25 modules may not be loaded");
        DEBUG_PRINT("Try: sudo modprobe ax25 mkiss");
    }

    return 1;
}

static int check_kernel_ax25_support(void) {
    return check_kernel_ax25_support_enhanced();
}

// Check if AF_AX25 bind is available with proper interface verification
static int check_ax25_bind_available(const char *port_name, const char *callsign) {
    struct sockaddr_ax25 addr;
    int sock = socket(AF_AX25, SOCK_SEQPACKET, 0);

    if (sock < 0) {
        DEBUG_PRINT("Socket creation failed for bind check");
        return 0;
    }

    int ifindex = if_nametoindex(port_name);
    if (ifindex == 0) {
        close(sock);
        DEBUG_PRINT("Interface %s not found", port_name);
        return 0;
    }

    if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, port_name, strlen(port_name)) < 0) {
        close(sock);
        DEBUG_PRINT("SO_BINDTODEVICE failed for %s: %s", port_name, strerror(errno));
        return 0;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sax25_family = AF_AX25;
    addr.sax25_ndigis = 0;

    int ret = ax25_aton_entry(callsign, (char*) &addr.sax25_call);
    if (ret < 0) {
        close(sock);
        DEBUG_PRINT("ax25_aton_entry failed for callsign: %s", callsign);
        return 0;
    }

    int bind_result = bind(sock, (struct sockaddr*) &addr, sizeof(struct sockaddr_ax25));

    if (bind_result < 0) {
        DEBUG_PRINT("Bind failed: %s (errno: %d)", strerror(errno), errno);
        if (errno == ENODEV) {
            DEBUG_PRINT("Hint: AX.25 interface %s needs kissattach setup", port_name);
        }
        close(sock);
        return 0;
    }

    close(sock);
    return 1;
}

// Parse axports configuration file with comprehensive error handling
static int find_ax25_port_direct(char *port_name, size_t max_len) {
    FILE *fp = fopen(TEST_AXPORTS_FILE, "r");
    if (!fp) {
        DEBUG_PRINT("Cannot read %s: %s", TEST_AXPORTS_FILE, strerror(errno));
        safe_strlcpy(port_name, TEST_DEFAULT_PORT, max_len);
        return 0;
    }

    char line[MAX_AXPORTS_LINE];
    int found = 0;

    while (fgets(line, sizeof(line), fp)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
            continue;

        char name[MAX_PORT_NAME_LEN];
        char callsign[MAX_CALLSIGN_LEN];
        char device[MAX_PORT_NAME_LEN];
        int speed, paclen, window;

        int parsed = sscanf(line, "%31s %31s %d %d %d %31s", name, callsign, &speed, &paclen, &window, device);

        if (parsed >= 5) {
            safe_strlcpy(port_name, name, max_len);
            fclose(fp);
            DEBUG_PRINT("Found AX.25 port: %s (device: %s, speed: %d)", name, device, speed);
            found = 1;
            return 1;
        }
    }

    fclose(fp);

    if (!found) {
        DEBUG_PRINT("No valid AX.25 ports found in %s", TEST_AXPORTS_FILE);
    }

    safe_strlcpy(port_name, TEST_DEFAULT_PORT, max_len);
    return 0;
}

// Validated axports configuration parsing
static int validate_axport_params(const char *port_name, int speed, int paclen, int window, axport_config_validated_t *out) {
    if (!port_name || !out)
        return 0;

    memset(out, 0, sizeof(*out));

    size_t port_name_len = strlen(port_name);
    if (port_name_len == 0 || port_name_len > MAX_PORT_NAME_LEN - 1) {
        DEBUG_PRINT("Invalid port name: '%s' (length %zu)", port_name, port_name_len);
        return 0;
    }

    // Speed validation: accept 0 (auto-detect / no serial speed, used by sound modems
    // and virtual interfaces such as direwolf) and all non-negative values.
    if (speed < 0) {
        DEBUG_PRINT("Invalid speed for %s: %d (must be >= 0)", port_name, speed);
        return 0;
    }

    if (paclen < 16 || paclen > 512) {
        DEBUG_PRINT("Invalid paclen for %s: %d (valid range 16-512)", port_name, paclen);
        return 0;
    }

    if (window < 1 || window > 127) {
        DEBUG_PRINT("Invalid window for %s: %d (valid range 1-127)", port_name, window);
        return 0;
    }

    out->valid = 1;
    out->speed = speed;
    out->paclen = paclen;
    out->window = window;
    return 1;
}

// Load and verify axports configuration with safe string handling
static int load_ax25_config(char *out_call, size_t call_max_len) {
    int count = 0;

    DEBUG_PRINT("Attempting to load AX.25 configuration via libax25...");

    FILE *fp = fopen(TEST_AXPORTS_FILE, "r");
    if (!fp) {
        DEBUG_PRINT("axports file not accessible: %s", strerror(errno));
        return 0;
    }

    char line[MAX_AXPORTS_LINE];
    int line_num = 0;

    while (fgets(line, sizeof(line), fp)) {
        line_num++;

        if (line[0] != '#' && line[0] != '\n' && line[0] != '\r') {
            size_t line_len = strlen(line);
            while (line_len > 0 && (line[line_len - 1] == '\n' || line[line_len - 1] == '\r' || line[line_len - 1] == ' ')) {
                line[--line_len] = '\0';
            }

            if (line_len == 0)
                continue;

            char name[MAX_PORT_NAME_LEN];
            char callsign[MAX_CALLSIGN_LEN];
            char device[MAX_PORT_NAME_LEN];
            int speed, paclen, window;

            int parsed = sscanf(line, "%31s %31s %d %d %d %31s", name, callsign, &speed, &paclen, &window, device);

            if (parsed >= 5) {
                axport_config_validated_t validated;
                if (!validate_axport_params(name, speed, paclen, window, &validated)) {
                    DEBUG_PRINT("Invalid parameters in axports line %d: %s", line_num, line);
                    continue;
                }

                count++;
                DEBUG_PRINT("Found AX.25 port entry: %s callsign: %s", name, callsign);

                if (count == 1) {
                    safe_strlcpy(out_call, callsign, call_max_len);
                }
            }
        }
    }

    fclose(fp);

    if (count > 0) {
        DEBUG_PRINT("Loaded %d AX.25 port(s) from configuration", count);
        DEBUG_PRINT("Using callsign from axports: %s", out_call);
    } else {
        DEBUG_PRINT("No valid AX.25 ports found in configuration");
    }

    return count;
}

// Socket creation with resource tracking and error handling
static int create_ax25_socket_with_tracking(const char *port_name, const char *local_call, socket_resource_t *res) {
    struct sockaddr_ax25 local_addr;

    if (!res) {
        DEBUG_PRINT("Error: resource tracker is NULL");
        return -1;
    }

    memset(res, 0, sizeof(*res));
    res->fd = -1;

    int sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
    if (sock < 0) {
        DEBUG_PRINT("socket(AF_AX25) failed: %s (errno: %d)", strerror(errno), errno);
        return -1;
    }
    res->fd = sock;

    int ifindex = if_nametoindex(port_name);
    if (ifindex == 0) {
        DEBUG_PRINT("Interface %s not found: %s", port_name, strerror(errno));
        cleanup_socket_resources(res);
        return -1;
    }

    if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, port_name, strlen(port_name)) < 0) {
        DEBUG_PRINT("setsockopt(SO_BINDTODEVICE, %s) failed: %s (errno: %d)", port_name, strerror(errno), errno);
        cleanup_socket_resources(res);
        return -1;
    }

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sax25_family = AF_AX25;
    local_addr.sax25_ndigis = 0;

    if (ax25_aton_entry(local_call, (char*) &local_addr.sax25_call) < 0) {
        DEBUG_PRINT("ax25_aton_entry failed for: %s", local_call);
        cleanup_socket_resources(res);
        return -1;
    }

    if (bind(sock, (struct sockaddr*) &local_addr, sizeof(struct sockaddr_ax25)) < 0) {
        int bind_errno = errno;
        DEBUG_PRINT("bind failed: %s (errno: %d)", strerror(bind_errno), bind_errno);

        if (bind_errno == ENODEV) {
            DEBUG_PRINT("  -> Hint: Interface %s not found or not configured", port_name);
            DEBUG_PRINT("  -> Hint: AX.25 interface needs kissattach setup");
            DEBUG_PRINT("  -> Check: ip link show %s", port_name);
        } else if (bind_errno == EACCES) {
            DEBUG_PRINT("  -> Hint: Permission denied - may need root or ax25 group");
        } else if (bind_errno == EINVAL) {
            DEBUG_PRINT("  -> Hint: Invalid argument - check callsign %s or interface", local_call);
        } else if (bind_errno == EADDRINUSE) {
            DEBUG_PRINT("  -> Hint: Address already in use - callsign %s may be bound elsewhere", local_call);
        }

        cleanup_socket_resources(res);
        return -1;
    }
    res->is_bound = 1;

    int flags = fcntl(sock, F_GETFL, 0);
    if (flags >= 0) {
        if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) >= 0) {
            res->is_blocking_modified = 1;
        }
    }

    DEBUG_PRINT("Created AF_AX25 socket: %s on interface: %s", local_call, port_name);
    return sock;
}

__attribute__((unused))
static int create_ax25_socket(const char *port_name, const char *local_call) {
    socket_resource_t res;
    int result = create_ax25_socket_with_tracking(port_name, local_call, &res);
    return result;
}

// Frame lifecycle management with allocation tracking
__attribute__((unused))
static void frame_lifecycle_init_with_frame(frame_lifecycle_t *lifecycle, ax25_frame_t *frame, int is_malloc_frame) {
    if (!lifecycle)
        return;

    memset(lifecycle, 0, sizeof(*lifecycle));
    lifecycle->frame = frame;
    lifecycle->is_malloc_frame = is_malloc_frame ? 1 : 0;
    lifecycle->is_malloc_encoded = 1;
    lifecycle->is_initialized = 1;
}

static void frame_lifecycle_init(frame_lifecycle_t *lifecycle) {
    if (!lifecycle)
        return;

    memset(lifecycle, 0, sizeof(*lifecycle));
    lifecycle->is_malloc_encoded = 1;
    lifecycle->is_initialized = 1;
}

__attribute__((unused))
static void frame_lifecycle_set_encoded_data(frame_lifecycle_t *lifecycle, uint8_t *data, size_t len, int is_malloc) {
    if (lifecycle) {
        lifecycle->encoded_data = data;
        lifecycle->encoded_len = len;
        lifecycle->is_malloc_encoded = is_malloc ? 1 : 0;
    }
}

__attribute__((unused))
static void frame_lifecycle_mark_encode_failed(frame_lifecycle_t *lifecycle) {
    if (lifecycle) {
        lifecycle->encode_failed = 1;
        lifecycle->encode_attempted = 1;
    }
}

__attribute__((unused))
static void frame_lifecycle_mark_encode_success(frame_lifecycle_t *lifecycle, uint8_t *encoded, size_t len) {
    if (lifecycle) {
        lifecycle->encoded_data = encoded;
        lifecycle->encoded_len = len;
        lifecycle->encode_attempted = 1;
        lifecycle->encode_failed = 0;
    }
}

static ax25_address_t* frame_lifecycle_create_address(frame_lifecycle_t *lifecycle, const char *callsign, int is_dest, uint8_t *err) {
    ax25_address_t *addr = ax25_address_from_string(callsign, err);

    if (addr && lifecycle) {
        if (is_dest) {
            lifecycle->addr_dest = addr;
            lifecycle->is_malloc_addr_dest = 1;
        } else {
            lifecycle->addr_src = addr;
            lifecycle->is_malloc_addr_src = 1;
        }
    }

    return addr;
}

static void frame_lifecycle_cleanup(frame_lifecycle_t *lifecycle) {
    uint8_t err = 0;

    if (!lifecycle || !lifecycle->is_initialized)
        return;

    if (lifecycle->encoded_data && lifecycle->is_malloc_encoded && !lifecycle->encode_failed) {
        free(lifecycle->encoded_data);
        lifecycle->encoded_data = NULL;
    }

    if (lifecycle->frame && lifecycle->is_malloc_frame) {
        ax25_frame_free(lifecycle->frame, &err);
        lifecycle->frame = NULL;
    }

    if (lifecycle->addr_dest && lifecycle->is_malloc_addr_dest) {
        ax25_address_free(lifecycle->addr_dest, &err);
        lifecycle->addr_dest = NULL;
    }

    if (lifecycle->addr_src && lifecycle->is_malloc_addr_src) {
        ax25_address_free(lifecycle->addr_src, &err);
        lifecycle->addr_src = NULL;
    }

    lifecycle->encoded_len = 0;
    lifecycle->is_initialized = 0;
}

// Enhanced frame validation for AX.25 v2.2 compliance
static int validate_frame_structure_complete(const ax25_frame_t *frame, uint8_t modulo_128, uint8_t *err) {
    *err = 0;

    if (!frame) {
        *err = 1;
        DEBUG_PRINT("Frame pointer is NULL");
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
        break;
        default:
            *err = 2;
            DEBUG_PRINT("Invalid frame type enum: %d", frame->type);
            return -1;
    }

    if (strlen(frame->header.destination.callsign) == 0 || strlen(frame->header.source.callsign) == 0) {
        *err = 6;
        DEBUG_PRINT("Source or destination callsign is empty");
        return -1;
    }

    if (frame->header.destination.ssid < 0 || frame->header.destination.ssid > 15) {
        *err = 3;
        DEBUG_PRINT("Destination SSID out of range: %d", frame->header.destination.ssid);
        return -1;
    }

    if (frame->header.source.ssid < 0 || frame->header.source.ssid > 15) {
        *err = 4;
        DEBUG_PRINT("Source SSID out of range: %d", frame->header.source.ssid);
        return -1;
    }

    if (frame->header.repeaters.num_repeaters < 0 || frame->header.repeaters.num_repeaters > AX25_MAX_REPEATERS) {
        *err = 5;
        DEBUG_PRINT("Too many repeaters: %d (max %d)", frame->header.repeaters.num_repeaters, AX25_MAX_REPEATERS);
        return -1;
    }

    for (int i = 0; i < frame->header.repeaters.num_repeaters; i++) {
        if (frame->header.repeaters.repeaters[i].ssid < 0 || frame->header.repeaters.repeaters[i].ssid > 15) {
            *err = 7;
            DEBUG_PRINT("Repeater %d SSID out of range: %d", i, frame->header.repeaters.repeaters[i].ssid);
            return -1;
        }

        if (strlen(frame->header.repeaters.repeaters[i].callsign) == 0) {
            *err = 15;
            DEBUG_PRINT("Repeater %d has empty callsign", i);
            return -1;
        }
    }

    switch (frame->type) {
        case AX25_FRAME_UNNUMBERED_INFORMATION: {
            ax25_unnumbered_information_frame_t *ui = (ax25_unnumbered_information_frame_t*) frame;

            if (ui->payload_len > MAX_UI_PAYLOAD_SIZE) {
                *err = 8;
                DEBUG_PRINT("UI frame payload too large: %zu (max %d)", ui->payload_len, MAX_UI_PAYLOAD_SIZE);
                return -1;
            }

            if (ui->pid == 0xFF) {
                *err = 10;
                DEBUG_PRINT("UI frame has reserved PID 0xFF");
                return -1;
            }

            if (ui->payload_len > 0 && ui->payload == NULL) {
                *err = 17;
                DEBUG_PRINT("UI frame payload_len > 0 but payload is NULL");
                return -1;
            }
            break;
        }

        case AX25_FRAME_INFORMATION_8BIT:
        case AX25_FRAME_INFORMATION_16BIT: {
            ax25_information_frame_t *info = (ax25_information_frame_t*) frame;

            if (info->payload_len > MAX_INFO_PAYLOAD_SIZE) {
                *err = 9;
                DEBUG_PRINT("I-frame payload too large: %zu (max %d)", info->payload_len, MAX_INFO_PAYLOAD_SIZE);
                return -1;
            }

            if (info->payload_len > 0 && info->payload == NULL) {
                *err = 18;
                DEBUG_PRINT("I-frame payload_len > 0 but payload is NULL");
                return -1;
            }

            if (modulo_128) {
                if (info->ns < 0 || info->ns > 127) {
                    *err = 11;
                    DEBUG_PRINT("I-frame Mod-128 N(S) out of range: %d", info->ns);
                    return -1;
                }
                if (info->nr < 0 || info->nr > 127) {
                    *err = 12;
                    DEBUG_PRINT("I-frame Mod-128 N(R) out of range: %d", info->nr);
                    return -1;
                }
            } else {
                if (info->ns < 0 || info->ns > 7) {
                    *err = 13;
                    DEBUG_PRINT("I-frame Mod-8 N(S) out of range: %d", info->ns);
                    return -1;
                }
                if (info->nr < 0 || info->nr > 7) {
                    *err = 14;
                    DEBUG_PRINT("I-frame Mod-8 N(R) out of range: %d", info->nr);
                    return -1;
                }
            }
            break;
        }

        case AX25_FRAME_UNNUMBERED_SABM:
        case AX25_FRAME_UNNUMBERED_DISC:
        case AX25_FRAME_UNNUMBERED_UA:
        case AX25_FRAME_UNNUMBERED_DM:
        case AX25_FRAME_UNNUMBERED_FRMR:
        case AX25_FRAME_UNNUMBERED_XID:
        break;

        default:
        break;
    }

    return 0;
}

static int validate_frame_for_encoding(const ax25_frame_t *frame, uint8_t *err) {
    return validate_frame_structure_complete(frame, MODULO128_FALSE, err);
}

// Enhanced HDLC validation for bit-stuffing correctness
static int validate_hdlc_frame_format(const uint8_t *frame, unsigned int len, uint8_t *err) {
    *err = 0;

    if (!frame || len < 4) {
        *err = 1;
        DEBUG_PRINT("HDLC frame too short: %u bytes (minimum 4)", len);
        return -1;
    }

    if (frame[0] != HDLC_FLAG_BYTE) {
        *err = 2;
        DEBUG_PRINT("Frame does not start with HDLC flag: 0x%02X", frame[0]);
        return -1;
    }

    if (frame[len - 1] != HDLC_FLAG_BYTE) {
        *err = 3;
        DEBUG_PRINT("Frame does not end with HDLC flag: 0x%02X", frame[len - 1]);
        return -1;
    }

    for (unsigned int i = 1; i < len - 1; i++) {
        if (frame[i] == HDLC_FLAG_BYTE) {
            *err = 4;
            DEBUG_PRINT("Unescaped HDLC flag found at offset %u", i);
            return -1;
        }
    }

    return 0;
}

static int validate_hdlc_decoded_frame(const uint8_t *decoded, unsigned int len, uint8_t *err) {
    *err = 0;

    if (!decoded || len == 0) {
        *err = 1;
        DEBUG_PRINT("Decoded frame is NULL or zero-length");
        return -1;
    }

    if (len < 2) {
        *err = 2;
        DEBUG_PRINT("Decoded frame too short: %u bytes (minimum 2)", len);
        return -1;
    }

    return 0;
}

// Enhanced CRC validation for AX.25 CCITT-16
__attribute__((unused))
static int validate_crc_state(void) {
    uint16_t init_val = HAL_CRC16_INIT;

    if (init_val != HAL_CRC16_CCITT_INIT) {
        DEBUG_PRINT("WARNING: CRC16 init mismatch. Got 0x%04X, expected 0x%04X", init_val, HAL_CRC16_CCITT_INIT);
        return -1;
    }

    uint16_t crc = init_val;
    crc = hal_crc16_final(crc);

    if (crc == 0) {
        DEBUG_PRINT("WARNING: Empty CRC result is 0, verify algorithm");
    }

    return 0;
}

static int validate_crc_consistency(const uint8_t *data, size_t len, uint16_t *single_shot, uint16_t *incremental, uint8_t *err) {
    *err = 0;
    *single_shot = hal_crc16_buf(data, len);

    uint16_t crc = HAL_CRC16_INIT;
    for (size_t i = 0; i < len; i++) {
        crc = hal_crc16_update(crc, &data[i], 1);
    }
    *incremental = hal_crc16_final(crc);

    if (*single_shot != *incremental) {
        *err = 1;
        DEBUG_PRINT("CRC mismatch: single-shot=0x%04X, incremental=0x%04X", *single_shot, *incremental);
        return -1;
    }

    return 0;
}

// Enhanced connection state machine timer validation
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

    if (config->t1_ms < 1000 || config->t1_ms > 30000) {
        DEBUG_PRINT("WARNING: T1=%dms may be out of typical range (1-30s)", config->t1_ms);
    }

    if (config->t2_ms < 500 || config->t2_ms > 5000) {
        DEBUG_PRINT("WARNING: T2=%dms may be out of typical range (0.5-5s)", config->t2_ms);
    }

    if (config->t3_ms < 5000 || config->t3_ms > 600000) {
        DEBUG_PRINT("WARNING: T3=%dms may be out of typical range (5-600s)", config->t3_ms);
    }

    if (config->n2_retries < 2 || config->n2_retries > 30) {
        DEBUG_PRINT("WARNING: N2=%d may be out of typical range (2-30 retries)", config->n2_retries);
    }

    if (config->t1_ticks >= config->t3_ticks) {
        *err = 2;
        DEBUG_PRINT("ERROR: T1 (%d) must be less than T3 (%d)", config->t1_ticks, config->t3_ticks);
        return -1;
    }

    return 0;
}

__attribute__((unused))
static int validate_connection_initial_state(const ax25_connection_t *conn, uint8_t *err) {
    *err = 0;

    if (!conn) {
        *err = 1;
        return -1;
    }

    if (conn->state != AX25_STATE_DISCONNECTED) {
        *err = 2;
        DEBUG_PRINT("ERROR: Initial state should be DISCONNECTED, got %d", conn->state);
        return -1;
    }

    if (conn->timers.t1 <= 0 || conn->timers.t2 <= 0 || conn->timers.t3 <= 0 || conn->timers.n2 <= 0) {
        *err = 3;
        DEBUG_PRINT("ERROR: Timer values must be positive");
        return -1;
    }

    return 0;
}

// Enhanced buffer pool validation
__attribute__((unused))
static int get_buffer_pool_stats(buffer_pool_stats_t *stats, uint8_t *err) {
    *err = 0;

    if (!stats) {
        *err = 1;
        return -1;
    }

    memset(stats, 0, sizeof(*stats));
    stats->free_buffers = ax25_buf_pool_free_count();
    stats->total_buffers = 32;
    stats->allocated_buffers = stats->total_buffers - stats->free_buffers;

    if (stats->allocated_buffers < 0 || stats->allocated_buffers > stats->total_buffers) {
        *err = 2;
        DEBUG_PRINT("ERROR: Buffer count inconsistency - allocated=%d, total=%d", stats->allocated_buffers, stats->total_buffers);
        return -1;
    }

    return 0;
}

static int validate_allocated_buffer(const ax25_buf_t *buf, uint8_t *err) {
    *err = 0;

    if (!buf) {
        *err = 1;
        return -1;
    }

    if (!buf->in_use) {
        *err = 2;
        DEBUG_PRINT("ERROR: Allocated buffer not marked in_use");
        return -1;
    }

    if (buf->len == 0) {
        DEBUG_PRINT("WARNING: Buffer allocated with zero length");
    }

    return 0;
}

__attribute__((unused))
static int test_buffer_pool_exhaustion(uint8_t *err) {
    *err = 0;

    ax25_buf_t *buffers[32];
    memset(buffers, 0, sizeof(buffers));

    int allocated = 0;
    for (int i = 0; i < 32; i++) {
        buffers[i] = ax25_buf_alloc();
        if (buffers[i] != NULL) {
            allocated++;
        } else {
            DEBUG_PRINT("Buffer pool exhausted after %d allocations", i);
            break;
        }
    }

    if (allocated == 0) {
        *err = 1;
        DEBUG_PRINT("ERROR: Cannot allocate any buffers");
        return -1;
    }

    for (int i = allocated - 1; i >= 0; i--) {
        if (buffers[i]) {
            ax25_buf_free(buffers[i]);
        }
    }

    return 0;
}

// Test context isolation management
static void test_context_reset(void) {
    memset(&g_test_ctx, 0, sizeof(g_test_ctx));
    safe_strlcpy(g_test_ctx.port_name, TEST_DEFAULT_PORT, sizeof(g_test_ctx.port_name));
    safe_strlcpy(g_test_ctx.local_call, TEST_DEFAULT_CALL, sizeof(g_test_ctx.local_call));
    g_test_ctx.kernel_ax25_available = 0;
    g_test_ctx.socket_bind_available = 0;
    g_test_ctx.port_count = 0;
    g_test_ctx.test_flags = 0;
}

static void test_context_init(void) {
    test_context_reset();

    printf("Checking kernel AX.25 support...\n");
    g_test_ctx.kernel_ax25_available = check_kernel_ax25_support();

    if (g_test_ctx.kernel_ax25_available) {
        printf("✓ AF_AX25 socket support AVAILABLE\n");

        if (find_ax25_port_direct(g_test_ctx.port_name, sizeof(g_test_ctx.port_name))) {
            printf("✓ Found configured port: %s\n", g_test_ctx.port_name);
        } else {
            printf("⚠ No configured ports found, using default: %s\n", g_test_ctx.port_name);
        }
    } else {
        printf("✗ AF_AX25 socket support NOT AVAILABLE\n");
        printf("  Kernel tests will be skipped\n");
        printf("  Required: ax25, mkiss kernel modules\n");
    }

    printf("Loading libax25 configuration...\n");
    g_test_ctx.port_count = load_ax25_config(g_test_ctx.local_call, sizeof(g_test_ctx.local_call));

    if (g_test_ctx.port_count > 0) {
        printf("✓ AX.25 ports loaded (%d port(s))\n", g_test_ctx.port_count);
        printf("  Using callsign: %s\n", g_test_ctx.local_call);
    } else {
        printf("⚠ No AX.25 ports configured\n");
        printf("  Using default callsign: %s\n", g_test_ctx.local_call);
    }

    printf("Checking AX.25 socket bind capability...\n");
    if (g_test_ctx.kernel_ax25_available) {
        g_test_ctx.socket_bind_available = check_ax25_bind_available(g_test_ctx.port_name, g_test_ctx.local_call);
        if (g_test_ctx.socket_bind_available) {
            printf("✓ AF_AX25 socket bind AVAILABLE\n");
        } else {
            printf("⚠ AF_AX25 socket bind NOT AVAILABLE\n");
            printf("  Bind tests will be skipped\n");
            printf("  Setup required:\n");
            printf("    1. Load modules: sudo modprobe ax25 mkiss\n");
            printf("    2. Configure /etc/ax25/axports\n");
            printf("    3. Run: sudo kissattach /dev/ttyUSB0 %s\n", g_test_ctx.port_name);
            printf("    4. Run: sudo ifconfig %s up\n", g_test_ctx.port_name);
        }
    }
}

typedef int (*test_section_fn_t)(void);

static int run_test_section(const char *section_name, test_section_fn_t section_fn) {
    printf("\n%s\n", section_name);
    int result = section_fn();
    if (result != 0) {
        printf("FAILED: %s (return code %d)\n", section_name, result);
    } else {
        printf("PASSED: %s\n", section_name);
    }
    return result;
}

// SECTION A: libax25 Address Function Tests
static int sec_a_libax25_address(void) {
    TEST_SECTION("=== SEC-A: libax25 Address Functions ===");

    ax25_address addr, addr1, addr2;
    char *result;
    int rc;

    // A.1: ax25_aton_entry - Parse callsign to binary
    {
        rc = ax25_aton_entry("W1AW-0", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.1 ax25_aton_entry W1AW-0", rc);
        DEBUG_BUF("W1AW-0 encoded", (uint8_t *)&addr, sizeof(addr));
    }

    // A.2: ax25_ntoa - Convert binary to string
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

    // A.3: ax25_cmp - Compare identical addresses
    {
        ax25_aton_entry("W1AW-0", (char*) &addr1);
        ax25_aton_entry("W1AW-0", (char*) &addr2);
        rc = ax25_cmp(&addr1, &addr2);
        TEST_ASSERT(rc == 0, "A.3 ax25_cmp identical addresses", rc);
    }

    // A.4: ax25_cmp - Compare different addresses
    {
        ax25_aton_entry("W1AW-0", (char*) &addr1);
        ax25_aton_entry("N0CALL-0", (char*) &addr2);
        rc = ax25_cmp(&addr1, &addr2);
        TEST_ASSERT(rc != 0, "A.4 ax25_cmp different addresses", 0);
    }

    // A.5: ax25_cmp - Compare SSID differences
    {
        ax25_aton_entry("W1AW-0", (char*) &addr1);
        ax25_aton_entry("W1AW-1", (char*) &addr2);
        rc = ax25_cmp(&addr1, &addr2);
        TEST_ASSERT(rc == 2, "A.5 ax25_cmp SSID differ", rc);
        DEBUG_PRINT("SSID differ comparison result: %d", rc);
    }

    // A.6: SSID boundary - SSID 0 valid
    {
        rc = ax25_aton_entry("TEST-0", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.6 SSID 0 valid", rc);
    }

    // A.7: SSID boundary - SSID 15 valid
    {
        rc = ax25_aton_entry("TEST-15", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.7 SSID 15 valid", rc);
    }

    // A.8: SSID boundary - SSID 16 invalid (ax25_aton_entry rejects it)
    {
        rc = ax25_aton_entry("TEST-16", (char*) &addr);
        if (rc != 0) {
            DEBUG_PRINT("A.8: ax25_aton_entry correctly rejected TEST-16 (rc=%d)", rc);
            TEST_ASSERT(rc != 0, "A.8 SSID 16 invalid - rejected by ax25_aton_entry", 0);
        } else {
            TEST_ASSERT(0, "A.8 SSID 16 should be rejected", 0);
        }
    }

    // A.9: Short callsign
    {
        rc = ax25_aton_entry("AB", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.9 Short callsign AB", rc);
    }

    // A.10: Maximum callsign length
    {
        rc = ax25_aton_entry("VE3XYZ", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.10 6-char callsign VE3XYZ", rc);
    }

    // start modified part
    // A.11: ax25_validate - verify a correctly-encoded address is accepted
    {
        rc = ax25_aton_entry("W1AW-3", (char*) &addr);
        TEST_ASSERT(rc == 0, "A.11 ax25_aton_entry for validate test", rc);
        if (rc == 0) {
            rc = ax25_validate((char*) &addr);
            TEST_ASSERT(rc != 0, "A.11 ax25_validate accepts valid address", rc);
            DEBUG_PRINT("ax25_validate returned %d for W1AW-3", rc);
        }
    }

    // A.12: ax25_validate - verify a zeroed address is rejected
    {
        memset(&addr, 0, sizeof(addr));
        rc = ax25_validate((char*) &addr);
        TEST_ASSERT(rc == 0, "A.12 ax25_validate rejects zero address", rc);
        DEBUG_PRINT("ax25_validate returned %d for zero address", rc);
    }

    return 0;
}

// SECTION B: AF_AX25 Socket Operations
static int sec_b_af_ax25_sockets(void) {
    TEST_SECTION("=== SEC-B: AF_AX25 Socket Operations ===");

    struct sockaddr_ax25 addr;
    socklen_t addr_len;
    int sock, rc;

    if (!g_test_ctx.kernel_ax25_available) {
        printf("SKIP: SEC-B (no kernel AF_AX25 support)\n");
        return 0;
    }

    // B.1: Create AF_AX25 socket
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        TEST_ASSERT(sock >= 0, "B.1 Create AF_AX25 SOCK_SEQPACKET", 0);
        if (sock >= 0) {
            DEBUG_PRINT("Socket FD: %d", sock);
            close(sock);
        }
    }

    // B.2: Bind to local address
    {
        if (!g_test_ctx.socket_bind_available) {
            printf("SKIP: B.2 (AX.25 interface not configured for binding)\n");
        } else {
            socket_resource_t res;
            sock = create_ax25_socket_with_tracking(g_test_ctx.port_name, g_test_ctx.local_call, &res);
            TEST_ASSERT(sock >= 0, "B.2 Bind AF_AX25 to local address", 0);
            if (sock >= 0) {
                cleanup_socket_resources(&res);
            }
        }
    }

    // B.3: Get socket name (getsockname)
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
                DEBUG_PRINT("Socket address family: %d", addr.sax25_family);
                cleanup_socket_resources(&res);
            }
        }
    }

    // B.4: Set AX25_WINDOW socket option
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int window = 7;
            rc = setsockopt(sock, SOL_AX25, AX25_WINDOW, &window, sizeof(window));

            if (rc < 0 && errno == EINVAL) {
                printf("SKIP: B.4 AX25_WINDOW (needs bound socket on this kernel)\n");
            } else {
                TEST_ASSERT(rc == 0, "B.4 setsockopt AX25_WINDOW=7", rc);
                if (rc == 0) {
                    int readback = 0;
                    socklen_t optlen = sizeof(readback);
                    rc = getsockopt(sock, SOL_AX25, AX25_WINDOW, &readback, &optlen);
                    TEST_ASSERT(rc == 0 && readback == window, "B.4 getsockopt AX25_WINDOW verified", readback);
                    DEBUG_PRINT("AX25_WINDOW set and verified: %d", readback);
                }
            }

            close(sock);
        }
    }

    // B.5: Set AX25_N2 socket option
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int n2 = 10;
            rc = setsockopt(sock, SOL_AX25, AX25_N2, &n2, sizeof(n2));

            if (rc < 0 && errno == EINVAL) {
                printf("SKIP: B.5 AX25_N2 (needs bound socket on this kernel)\n");
            } else {
                TEST_ASSERT(rc == 0, "B.5 setsockopt AX25_N2=10", rc);
                if (rc == 0) {
                    int readback = 0;
                    socklen_t optlen = sizeof(readback);
                    rc = getsockopt(sock, SOL_AX25, AX25_N2, &readback, &optlen);
                    TEST_ASSERT(rc == 0 && readback == n2, "B.5 getsockopt AX25_N2 verified", readback);
                    DEBUG_PRINT("AX25_N2 set and verified: %d", readback);
                }
            }

            close(sock);
        }
    }

    // B.6: Set AX25_T1 socket option
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int t1 = 30;
            rc = setsockopt(sock, SOL_AX25, AX25_T1, &t1, sizeof(t1));

            if (rc < 0 && errno == EINVAL) {
                printf("SKIP: B.6 AX25_T1 (needs bound socket on this kernel)\n");
            } else {
                TEST_ASSERT(rc == 0, "B.6 setsockopt AX25_T1=30 (3s in 1/10th-sec units)", rc);
                if (rc == 0) {
                    int readback = 0;
                    socklen_t optlen = sizeof(readback);
                    rc = getsockopt(sock, SOL_AX25, AX25_T1, &readback, &optlen);
                    TEST_ASSERT(rc == 0 && readback == t1, "B.6 getsockopt AX25_T1 verified", readback);
                    DEBUG_PRINT("AX25_T1 set and verified: %d (1/10ths sec)", readback);
                }
            }

            close(sock);
        }
    }

    // B.7: Set AX25_T2 socket option (response delay, 1/10ths of second)
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int t2 = 30;  // 3 seconds in 1/10th-second units
            rc = setsockopt(sock, SOL_AX25, AX25_T2, &t2, sizeof(t2));
            if (rc < 0 && errno == EINVAL) {
                printf("SKIP: B.7 AX25_T2 (needs bound socket on this kernel)\n");
            } else {
                TEST_ASSERT(rc == 0, "B.7 setsockopt AX25_T2=30 (3s)", rc);
                if (rc == 0) {
                    int readback = 0;
                    socklen_t optlen = sizeof(readback);
                    rc = getsockopt(sock, SOL_AX25, AX25_T2, &readback, &optlen);
                    TEST_ASSERT(rc == 0 && readback == t2, "B.7 getsockopt AX25_T2 verified", readback);
                    DEBUG_PRINT("AX25_T2 set and verified: %d (1/10ths sec)", readback);
                }
            }
            close(sock);
        }
    }

    // B.8: Set AX25_EXTSEQ socket option (modulo-128 extended sequence numbers)
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int extseq = 1;
            rc = setsockopt(sock, SOL_AX25, AX25_EXTSEQ, &extseq, sizeof(extseq));
            if (rc < 0 && errno == EINVAL) {
                printf("SKIP: B.8 AX25_EXTSEQ (not supported on this kernel build)\n");
            } else {
                TEST_ASSERT(rc == 0, "B.8 setsockopt AX25_EXTSEQ=1 (mod-128)", rc);
                if (rc == 0) {
                    int readback = 0;
                    socklen_t optlen = sizeof(readback);
                    rc = getsockopt(sock, SOL_AX25, AX25_EXTSEQ, &readback, &optlen);
                    TEST_ASSERT(rc == 0 && readback == extseq, "B.8 getsockopt AX25_EXTSEQ verified", readback);
                    DEBUG_PRINT("AX25_EXTSEQ (mod-128) set and verified: %d", readback);
                }
            }
            close(sock);
        }
    }

    // B.9: Set AX25_BACKOFF socket option (0=linear, 1=exponential)
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int backoff = 1;
            rc = setsockopt(sock, SOL_AX25, AX25_BACKOFF, &backoff, sizeof(backoff));
            if (rc < 0 && errno == EINVAL) {
                printf("SKIP: B.9 AX25_BACKOFF (needs bound socket on this kernel)\n");
            } else {
                TEST_ASSERT(rc == 0, "B.9 setsockopt AX25_BACKOFF=1 (exponential)", rc);
                DEBUG_PRINT("AX25_BACKOFF set: %d", backoff);
            }
            close(sock);
        }
    }

    // B.10: Set AX25_T3 socket option (keep-alive/link-check timer, 1/10ths of second)
    // T3 is TAPR CHECK parameter: after T3 expires without data the kernel sends
    // an RR supervisory frame to verify the link is still alive.
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int t3 = 1800;  // 180 seconds (3 minutes) in 1/10th-second units
            rc = setsockopt(sock, SOL_AX25, AX25_T3, &t3, sizeof(t3));
            if (rc < 0 && errno == EINVAL) {
                printf("SKIP: B.10 AX25_T3 (needs bound socket on this kernel)\n");
            } else {
                TEST_ASSERT(rc == 0, "B.10 setsockopt AX25_T3=1800 (180s in 1/10th-sec units)", rc);
                if (rc == 0) {
                    int readback = 0;
                    socklen_t optlen = sizeof(readback);
                    rc = getsockopt(sock, SOL_AX25, AX25_T3, &readback, &optlen);
                    TEST_ASSERT(rc == 0 && readback == t3, "B.10 getsockopt AX25_T3 verified", readback);
                    DEBUG_PRINT("AX25_T3 set and verified: %d (1/10ths sec)", readback);
                }
            }
            close(sock);
        }
    }

    // B.11: Set AX25_PIDINCL socket option (return full raw AX.25 header to application)
    // When set, recvfrom() prepends the complete AX.25 address field to every packet.
    // Required by monitors, axlisten, and applications that need PID or digi H-bits.
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock >= 0) {
            int hdrincl = 1;
            rc = setsockopt(sock, SOL_AX25, AX25_PIDINCL, &hdrincl, sizeof(hdrincl));
            if (rc < 0 && (errno == EINVAL || errno == ENOPROTOOPT)) {
                printf("SKIP: B.11 AX25_PIDINCL (not available on this kernel build)\n");
            } else {
                TEST_ASSERT(rc == 0, "B.11 setsockopt AX25_PIDINCL=1", rc);
                if (rc == 0) {
                    int readback = 0;
                    socklen_t optlen = sizeof(readback);
                    rc = getsockopt(sock, SOL_AX25, AX25_PIDINCL, &readback, &optlen);
                    TEST_ASSERT(rc == 0 && readback == hdrincl, "B.11 getsockopt AX25_PIDINCL verified", readback);
                    DEBUG_PRINT("AX25_PIDINCL set and verified: %d", readback);
                }
            }
            close(sock);
        }
    }

    return 0;
}

// SECTION C: Frame Encode/Decode with libax25v22
static int sec_c_frame_encode_decode(void) {
    TEST_SECTION("=== SEC-C: Frame Encode/Decode (libax25v22) ===");

    uint8_t err;
    size_t enc_len;
    uint8_t *enc;
    ax25_frame_t *dec;
    ax25_frame_header_t hdr;
    memset(&hdr, 0, sizeof(hdr));

    // C.1: Create frame header from libax25v22
    {
        frame_lifecycle_t cycle;
        frame_lifecycle_init(&cycle);

        cycle.addr_dest = frame_lifecycle_create_address(&cycle, "W1AW-0", 1, &err);
        cycle.addr_src = frame_lifecycle_create_address(&cycle, "N0CALL-0", 0, &err);

        TEST_ASSERT(cycle.addr_dest != NULL && cycle.addr_src != NULL, "C.1 Create address objects", err);

        if (cycle.addr_dest && cycle.addr_src) {
            memset(&hdr, 0, sizeof(hdr));
            hdr.destination = *cycle.addr_dest;
            hdr.source = *cycle.addr_src;
            hdr.cr = true;
            hdr.repeaters.num_repeaters = 0;

            DEBUG_BUF("Destination callsign", (uint8_t*)cycle.addr_dest->callsign, 6);
            DEBUG_BUF("Source callsign", (uint8_t*)cycle.addr_src->callsign, 6);
        }

        frame_lifecycle_cleanup(&cycle);
    }

    // C.2: Encode UI frame (connectionless)
    {
        frame_lifecycle_t cycle;
        frame_lifecycle_init(&cycle);

        cycle.addr_dest = frame_lifecycle_create_address(&cycle, "W1AW-0", 1, &err);
        cycle.addr_src = frame_lifecycle_create_address(&cycle, "N0CALL-0", 0, &err);

        if (cycle.addr_dest && cycle.addr_src && !err) {
            memset(&hdr, 0, sizeof(hdr));
            hdr.destination = *cycle.addr_dest;
            hdr.source = *cycle.addr_src;
            hdr.cr = true;
            hdr.repeaters.num_repeaters = 0;

            uint8_t payload[] = "HELLO AX.25";
            ax25_unnumbered_information_frame_t ui;
            memset(&ui, 0, sizeof(ui));
            ui.base.base.type = AX25_FRAME_UNNUMBERED_INFORMATION;
            ui.base.base.header = hdr;
            ui.base.pf = false;
            ui.base.modifier = AX25_U_UI;
            ui.pid = PID_NO_L3;
            ui.payload = payload;
            ui.payload_len = sizeof(payload) - 1;

            if (validate_frame_for_encoding((ax25_frame_t*) &ui, &err) == 0) {
                enc = ax25_frame_encode((ax25_frame_t*) &ui, &enc_len, &err);
                TEST_ASSERT(enc != NULL && err == 0, "C.2 Encode UI frame", err);
                TEST_ASSERT(enc_len >= 20, "C.2 UI frame size >= 20 bytes", (unsigned ) enc_len);
                DEBUG_BUF("Encoded UI frame", enc, enc_len);
                if (enc) {
                    free(enc);
                }
            } else {
                TEST_ASSERT(0, "C.2 Frame validation failed", err);
            }
        }

        frame_lifecycle_cleanup(&cycle);
    }

    // C.3: Encode SABM frame (connection setup)
    {
        ax25_unnumbered_frame_t sabm;
        memset(&sabm, 0, sizeof(sabm));
        sabm.base.type = AX25_FRAME_UNNUMBERED_SABM;
        sabm.base.header = hdr;
        sabm.pf = true;
        sabm.modifier = AX25_U_SABM;

        if (validate_frame_for_encoding((ax25_frame_t*) &sabm, &err) == 0) {
            enc = ax25_frame_encode((ax25_frame_t*) &sabm, &enc_len, &err);
        } else {
            enc = NULL;
        }

        TEST_ASSERT(enc != NULL && err == 0, "C.3 Encode SABM frame", err);
        TEST_ASSERT(enc_len >= 15, "C.3 SABM frame size >= 15 bytes", (unsigned ) enc_len);
        DEBUG_BUF("Encoded SABM frame", enc, enc_len);
        if (enc) {
            free(enc);
        }
    }

    // C.4: Encode and decode round-trip (SABM)
    {
        ax25_unnumbered_frame_t sabm;
        memset(&sabm, 0, sizeof(sabm));
        sabm.base.type = AX25_FRAME_UNNUMBERED_SABM;
        sabm.base.header = hdr;
        sabm.pf = true;
        sabm.modifier = AX25_U_SABM;

        if (validate_frame_for_encoding((ax25_frame_t*) &sabm, &err) == 0) {
            enc = ax25_frame_encode((ax25_frame_t*) &sabm, &enc_len, &err);
        } else {
            enc = NULL;
        }

        TEST_ASSERT(enc != NULL && err == 0, "C.4 Encode SABM for round-trip", err);

        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "C.4 Decode SABM", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_UNNUMBERED_SABM, "C.4 Decoded frame type matches", 0);
                DEBUG_HEX("Decoded frame type", dec->type);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // C.5: Encode UA frame (connection response)
    {
        ax25_unnumbered_frame_t ua;
        memset(&ua, 0, sizeof(ua));
        ua.base.type = AX25_FRAME_UNNUMBERED_UA;
        ua.base.header = hdr;
        ua.pf = true;
        ua.modifier = AX25_U_UA;

        if (validate_frame_for_encoding((ax25_frame_t*) &ua, &err) == 0) {
            enc = ax25_frame_encode((ax25_frame_t*) &ua, &enc_len, &err);
        } else {
            enc = NULL;
        }

        TEST_ASSERT(enc != NULL && err == 0, "C.5 Encode UA frame", err);
        if (enc) {
            free(enc);
        }
    }

    // C.6: Encode DISC frame (normal disconnection) with round-trip
    {
        ax25_unnumbered_frame_t disc;
        memset(&disc, 0, sizeof(disc));
        disc.base.type = AX25_FRAME_UNNUMBERED_DISC;
        disc.base.header = hdr;
        disc.pf = true;
        disc.modifier = AX25_U_DISC;

        if (validate_frame_for_encoding((ax25_frame_t*) &disc, &err) == 0) {
            enc = ax25_frame_encode((ax25_frame_t*) &disc, &enc_len, &err);
        } else {
            enc = NULL;
        }

        TEST_ASSERT(enc != NULL && err == 0, "C.6 Encode DISC frame", err);
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

    // C.7: Encode DM frame (disconnect mode - not ready to connect) with round-trip
    {
        ax25_unnumbered_frame_t dm;
        memset(&dm, 0, sizeof(dm));
        dm.base.type = AX25_FRAME_UNNUMBERED_DM;
        dm.base.header = hdr;
        dm.pf = false;
        dm.modifier = AX25_U_DM;

        if (validate_frame_for_encoding((ax25_frame_t*) &dm, &err) == 0) {
            enc = ax25_frame_encode((ax25_frame_t*) &dm, &enc_len, &err);
        } else {
            enc = NULL;
        }

        TEST_ASSERT(enc != NULL && err == 0, "C.7 Encode DM frame", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "C.7 Decode DM round-trip", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_UNNUMBERED_DM, "C.7 Decoded type == DM", 0);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // C.8: Encode and decode modulo-8 I-frame (primary data frame)
    {
        uint8_t iframe_payload[] = "I-FRAME DATA";
        ax25_information_frame_t iframe;
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_8BIT;
        iframe.base.header = hdr;
        iframe.pf = false;
        iframe.ns = 3;
        iframe.nr = 1;
        iframe.payload = iframe_payload;
        iframe.payload_len = sizeof(iframe_payload) - 1;

        if (validate_frame_structure_complete((ax25_frame_t*) &iframe, MODULO128_FALSE, &err) == 0) {
            enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
        } else {
            enc = NULL;
        }

        TEST_ASSERT(enc != NULL && err == 0, "C.8 Encode mod-8 I-frame N(S)=3 N(R)=1", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "C.8 Decode mod-8 I-frame", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_INFORMATION_8BIT, "C.8 Decoded type == I-frame-8", 0);
                ax25_information_frame_t *di = (ax25_information_frame_t*) dec;
                TEST_ASSERT(di->ns == 3, "C.8 N(S) preserved", di->ns);
                TEST_ASSERT(di->nr == 1, "C.8 N(R) preserved", di->nr);
                DEBUG_PRINT("C.8 I-frame round-trip: N(S)=%d N(R)=%d len=%zu", di->ns, di->nr, di->payload_len);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // C.9: Encode and decode modulo-128 I-frame (AX.25 v2.2 extended sequence numbers)
    {
        uint8_t iframe_payload[] = "MOD128 DATA";
        ax25_information_frame_t iframe;
        memset(&iframe, 0, sizeof(iframe));
        iframe.base.type = AX25_FRAME_INFORMATION_16BIT;
        iframe.base.header = hdr;
        iframe.pf = false;
        iframe.ns = 64;
        iframe.nr = 32;
        iframe.payload = iframe_payload;
        iframe.payload_len = sizeof(iframe_payload) - 1;

        if (validate_frame_structure_complete((ax25_frame_t*) &iframe, MODULO128_TRUE, &err) == 0) {
            enc = ax25_frame_encode((ax25_frame_t*) &iframe, &enc_len, &err);
        } else {
            enc = NULL;
        }

        TEST_ASSERT(enc != NULL && err == 0, "C.9 Encode mod-128 I-frame N(S)=64 N(R)=32", err);
        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_TRUE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "C.9 Decode mod-128 I-frame", err);
            if (dec) {
                TEST_ASSERT(dec->type == AX25_FRAME_INFORMATION_16BIT, "C.9 Decoded type == I-frame-16", 0);
                ax25_information_frame_t *di = (ax25_information_frame_t*) dec;
                TEST_ASSERT(di->ns == 64, "C.9 Mod-128 N(S) preserved", di->ns);
                TEST_ASSERT(di->nr == 32, "C.9 Mod-128 N(R) preserved", di->nr);
                DEBUG_PRINT("C.9 Mod-128 I-frame round-trip: N(S)=%d N(R)=%d", di->ns, di->nr);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    return 0;
}

// SECTION D: HDLC Frame Processing
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

        if (validate_hdlc_frame_format(encoded, enc_len, &err) == 0) {
            TEST_ASSERT(enc_len > 14, "D.1 HDLC encode grows frame", 0);
            TEST_ASSERT(encoded[0] == HDLC_FLAG_BYTE, "D.1 Frame starts with HDLC flag 0x7E", 0);
            TEST_ASSERT(encoded[enc_len-1] == HDLC_FLAG_BYTE, "D.1 Frame ends with HDLC flag 0x7E", 0);
        }

        DEBUG_PRINT("HDLC encoded %d bytes to %d bytes", 14, enc_len);
    }

    // D.2: HDLC decode
    {
        dec_len = 0;
        rc = hdlc_frame_decode(encoded, enc_len, decoded, &dec_len);

        TEST_ASSERT(rc == HDLC_OK, "D.2 HDLC decode returns OK", (unsigned ) rc);
        TEST_ASSERT(dec_len == 14, "D.2 Decoded length matches original", 0);

        if (validate_hdlc_decoded_frame(decoded, dec_len, &err) == 0) {
            DEBUG_PRINT("HDLC decoded %d bytes successfully", dec_len);
        }
    }

    // D.3: CRC error detection on corrupted frame
    {
        memset(raw, 0x55, sizeof(raw));
        raw[13] |= 0x01;

        enc_len = 0;
        hdlc_frame_encode(raw, 14, encoded, &enc_len);

        {
            int corrupt_idx = enc_len / 2;
            if (corrupt_idx <= 0)
                corrupt_idx = 1;
            if (corrupt_idx >= enc_len - 1)
                corrupt_idx = enc_len - 2;
            encoded[corrupt_idx] ^= 0xFF;
        }

        dec_len = 0;
        rc = hdlc_frame_decode(encoded, enc_len, decoded, &dec_len);

        TEST_ASSERT(rc == HDLC_ERR_CRC_FAIL, "D.3 Corrupted frame CRC fails", (unsigned ) rc);
        DEBUG_PRINT("CRC error detected as expected: %d", rc);
    }

    // D.4: Bit stuffing with HDLC flag in payload
    {
        memset(raw, 0, sizeof(raw));
        raw[8] = 0x7E;
        raw[9] = 0x7E;
        raw[13] |= 0x01;

        enc_len = 0;
        hdlc_frame_encode(raw, 14, encoded, &enc_len);

        int has_flag_inside = 0;
        for (int i = 1; i < enc_len - 1; i++) {
            if (encoded[i] == 0x7E) {
                has_flag_inside = 1;
                break;
            }
        }

        TEST_ASSERT(has_flag_inside == 0, "D.4 No unescaped HDLC flags inside frame", 0);

        dec_len = 0;
        rc = hdlc_frame_decode(encoded, enc_len, decoded, &dec_len);
        TEST_ASSERT(rc == HDLC_OK && decoded[8] == 0x7E && decoded[9] == 0x7E, "D.4 0x7E payload recovered after destuffing", 0);
    }

    return 0;
}

// SECTION E: Connection State Machine
static int sec_e_connection_state_machine(void) {
    TEST_SECTION("=== SEC-E: Connection State Machine ===");

    ax25_connection_t conn;
    ax25_callbacks_t cb;
    uint8_t err;

    memset(&cb, 0, sizeof(cb));

    // E.1: Initialize connection
    {
        ax25_connection_init(&conn, &cb, NULL);
        TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "E.1 Initial state is DISCONNECTED", 0);
        DEBUG_STATE("Connection state", conn.state);
    }

    // E.2-E.5: Verify timers with validation
    {
        ax25_timer_config_t timer_cfg;
        if (validate_connection_timers(&conn, &timer_cfg, &err) == 0) {
#ifdef AX25_DEFAULT_T1
            TEST_ASSERT(timer_cfg.t1_ticks == AX25_DEFAULT_T1,
                "E.2 T1 == AX25_DEFAULT_T1", timer_cfg.t1_ticks);
#else
            TEST_ASSERT(timer_cfg.t1_ticks > 0, "E.2 T1 > 0 ticks", 0);
            DEBUG_PRINT("E.2 T1=%d ticks (%d ms)", timer_cfg.t1_ticks, timer_cfg.t1_ms);
#endif

#ifdef AX25_DEFAULT_T2
            TEST_ASSERT(timer_cfg.t2_ticks == AX25_DEFAULT_T2,
                "E.3 T2 == AX25_DEFAULT_T2", timer_cfg.t2_ticks);
#else
            TEST_ASSERT(timer_cfg.t2_ticks > 0, "E.3 T2 > 0 ticks", 0);
            DEBUG_PRINT("E.3 T2=%d ticks (%d ms)", timer_cfg.t2_ticks, timer_cfg.t2_ms);
#endif

#ifdef AX25_DEFAULT_T3
            TEST_ASSERT(timer_cfg.t3_ticks == AX25_DEFAULT_T3,
                "E.4 T3 == AX25_DEFAULT_T3", timer_cfg.t3_ticks);
#else
            TEST_ASSERT(timer_cfg.t3_ticks > 0, "E.4 T3 > 0 ticks", 0);
            DEBUG_PRINT("E.4 T3=%d ticks (%d ms)", timer_cfg.t3_ticks, timer_cfg.t3_ms);
#endif

            TEST_ASSERT(timer_cfg.n2_retries == AX25_DEFAULT_N2, "E.5 N2 == AX25_DEFAULT_N2", timer_cfg.n2_retries);

            TEST_ASSERT(timer_cfg.t1_ms >= 500 && timer_cfg.t1_ms <= 60000, "E.5a T1 in plausible range (0.5-60s)", timer_cfg.t1_ms);
            TEST_ASSERT(timer_cfg.t3_ms > timer_cfg.t1_ms, "E.5b T3 > T1", 0);
            // end modified part

            DEBUG_VAR("T1 timer (ticks)", timer_cfg.t1_ticks);
            DEBUG_VAR("T2 timer (ticks)", timer_cfg.t2_ticks);
            DEBUG_VAR("T3 timer (ticks)", timer_cfg.t3_ticks);
            DEBUG_VAR("N2 retries", timer_cfg.n2_retries);
        }
    }

    // E.6: Connection cleanup
    {
        ax25_connection_cleanup(&conn);
        TEST_ASSERT(conn.state == AX25_STATE_DISCONNECTED, "E.6 After cleanup state is DISCONNECTED", 0);
    }

    return 0;
}

// SECTION F: CRC Functions
static int sec_f_crc_functions(void) {
    TEST_SECTION("=== SEC-F: CRC Functions ===");

    uint8_t data[] = { 0x82, 0xA0, 0xA4, 0x96, 0xAA, 0xA6, 0x40 };
    uint16_t crc1, crc2;
    uint8_t err;

    // F.1: Single-shot CRC16
    {
        crc1 = hal_crc16_buf(data, sizeof(data));
        TEST_ASSERT(crc1 != 0, "F.1 CRC16 non-zero result", 0);
        DEBUG_HEX("CRC16 single-shot", crc1);
    }

    // F.2: Incremental CRC16 matches single-shot
    {
        if (validate_crc_consistency(data, sizeof(data), &crc1, &crc2, &err) == 0) {
            TEST_ASSERT(crc1 == crc2, "F.2 Incremental CRC matches single-shot", 0);
            DEBUG_PRINT("CRC1: 0x%04X, CRC2: 0x%04X", crc1, crc2);
        }
    }

    return 0;
}

// SECTION G: Buffer Pool Management
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
        TEST_ASSERT(buf1->in_use == 1, "G.1 Buffer marked in_use", 0);

        if (validate_allocated_buffer(buf1, &err) == 0) {
            DEBUG_VAR("Free buffers before", free_count_before);
        }
    }

    // G.2: Allocate second buffer
    {
        buf2 = ax25_buf_alloc();
        TEST_ASSERT(buf2 != NULL, "G.2 Allocate second buffer", 0);
        TEST_ASSERT(buf2 != buf1, "G.2 Different buffer pointers", 0);
        DEBUG_PRINT("Allocated buffers: %p, %p", (void*)buf1, (void*)buf2);
    }

    // G.3: Free buffers
    {
        ax25_buf_free(buf1);
        TEST_ASSERT(buf1->in_use == 0, "G.3 Buffer1 marked not in_use", 0);

        ax25_buf_free(buf2);
        TEST_ASSERT(buf2->in_use == 0, "G.3 Buffer2 marked not in_use", 0);

        free_count_after = ax25_buf_pool_free_count();
        TEST_ASSERT(free_count_after == free_count_before, "G.3 Free count restored", 0);
        DEBUG_VAR("Free buffers after", free_count_after);
    }

    return 0;
}

// SECTION H: Address Bridge Round-Trip Tests
static int sec_h_address_bridge_roundtrip(void) {
    TEST_SECTION("=== SEC-H: Address Bridge Round-Trip ===");

    ax25_address linux_orig, linux_result;
    ax25_address_t v22_addr;
    uint8_t err;
    int rc;

    // H.1: Forward conversion Linux -> libax25v22 for W1AW-3
    {
        rc = ax25_aton_entry("W1AW-3", (char*) &linux_orig);
        TEST_ASSERT(rc == 0, "H.1 ax25_aton_entry W1AW-3", rc);

        memset(&v22_addr, 0, sizeof(v22_addr));
        rc = bridge_linux_to_libax25v22(&linux_orig, &v22_addr, &err);
        TEST_ASSERT(rc == 0 && err == 0, "H.1 Linux->v22 bridge conversion", err);
        TEST_ASSERT(strcmp(v22_addr.callsign, "W1AW") == 0, "H.1 Callsign decoded correctly", 0);
        TEST_ASSERT(v22_addr.ssid == 3, "H.1 SSID=3 decoded correctly", v22_addr.ssid);
        DEBUG_PRINT("H.1 Decoded: callsign='%s' SSID=%d", v22_addr.callsign, v22_addr.ssid);
    }

    // H.2: Reverse conversion libax25v22 -> Linux
    {
        memset(&linux_result, 0, sizeof(linux_result));
        rc = bridge_libax25v22_to_linux(&v22_addr, &linux_result, &err);
        TEST_ASSERT(rc == 0 && err == 0, "H.2 v22->Linux bridge conversion", err);
    }

    // H.3: Round-trip fidelity
    {
        rc = ax25_cmp(&linux_orig, &linux_result);
        TEST_ASSERT(rc == 0, "H.3 Bridge round-trip W1AW-3 produces identical address", rc);
        char *str = ax25_ntoa(&linux_result);
        DEBUG_PRINT("H.3 Round-trip result: %s", str ? str : "(null)");
    }

    // H.4: Round-trip with SSID=0
    {
        rc = ax25_aton_entry("VE7FET", (char*) &linux_orig);
        TEST_ASSERT(rc == 0, "H.4 ax25_aton_entry VE7FET (SSID=0)", rc);

        memset(&v22_addr, 0, sizeof(v22_addr));
        rc = bridge_linux_to_libax25v22(&linux_orig, &v22_addr, &err);
        TEST_ASSERT(rc == 0, "H.4 Linux->v22 SSID=0 conversion", err);
        TEST_ASSERT(v22_addr.ssid == 0, "H.4 SSID=0 decoded correctly", v22_addr.ssid);
        TEST_ASSERT(strcmp(v22_addr.callsign, "VE7FET") == 0, "H.4 VE7FET callsign decoded", 0);

        memset(&linux_result, 0, sizeof(linux_result));
        rc = bridge_libax25v22_to_linux(&v22_addr, &linux_result, &err);
        TEST_ASSERT(rc == 0, "H.4 v22->Linux SSID=0 reverse conversion", err);
        rc = ax25_cmp(&linux_orig, &linux_result);
        TEST_ASSERT(rc == 0, "H.4 SSID=0 round-trip fidelity", rc);
    }

    // H.5: Round-trip with maximum SSID=15
    {
        rc = ax25_aton_entry("N0CALL-15", (char*) &linux_orig);
        TEST_ASSERT(rc == 0, "H.5 ax25_aton_entry N0CALL-15", rc);

        memset(&v22_addr, 0, sizeof(v22_addr));
        rc = bridge_linux_to_libax25v22(&linux_orig, &v22_addr, &err);
        TEST_ASSERT(rc == 0, "H.5 Linux->v22 SSID=15 conversion", err);
        TEST_ASSERT(v22_addr.ssid == 15, "H.5 SSID=15 decoded correctly", v22_addr.ssid);

        memset(&linux_result, 0, sizeof(linux_result));
        rc = bridge_libax25v22_to_linux(&v22_addr, &linux_result, &err);
        TEST_ASSERT(rc == 0, "H.5 v22->Linux SSID=15 reverse conversion", err);
        rc = ax25_cmp(&linux_orig, &linux_result);
        TEST_ASSERT(rc == 0, "H.5 SSID=15 round-trip fidelity", rc);
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

    return 0;
}

// SECTION I: I-Frames and Modulo-128 Extended Sequence Numbers
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

    // I.1: Encode and decode a basic mod-8 I-frame
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
                DEBUG_PRINT("I.1 Mod-8 I-frame encoded %zu bytes", enc_len);
                ax25_frame_free(dec, &err);
            }
            free(enc);
        }
    }

    // I.2: Verify all mod-8 N(S) values 0-7 survive round-trip
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
                    ax25_information_frame_t *di = (ax25_information_frame_t*) dec;
                    if (di->ns != ns_val) {
                        all_ok = 0;
                        DEBUG_PRINT("I.2 FAIL: N(S)=%d not preserved (got %d)", ns_val, di->ns);
                    }
                    ax25_frame_free(dec, &err);
                } else {
                    all_ok = 0;
                }
                free(enc);
            } else {
                all_ok = 0;
            }
        }
        TEST_ASSERT(all_ok, "I.2 All mod-8 N(S) values 0-7 preserved", 0);
        DEBUG_PRINT("I.2 Mod-8 N(S) sweep 0-7 complete");
    }

    // I.3: Mod-128 N(S) boundary values: 0, 64, 127
    {
        int boundary_ok = 1;
        int test_ns[] = { 0, 64, 127 };
        int n_tests = 3;
        int idx;
        for (idx = 0; idx < n_tests; idx++) {
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
                    ax25_information_frame_t *di = (ax25_information_frame_t*) dec;
                    if (di->ns != test_ns[idx]) {
                        boundary_ok = 0;
                        DEBUG_PRINT("I.3 FAIL: Mod-128 N(S)=%d not preserved (got %d)", test_ns[idx], di->ns);
                    }
                    ax25_frame_free(dec, &err);
                } else {
                    boundary_ok = 0;
                }
                free(enc);
            } else {
                boundary_ok = 0;
            }
        }
        TEST_ASSERT(boundary_ok, "I.3 Mod-128 N(S) boundary values 0/64/127 preserved", 0);
    }

    // I.4: Mod-128 frame must be larger than mod-8 (2-byte vs 1-byte control field)
    {
        ax25_information_frame_t iframe8, iframe16;
        size_t len8 = 0, len16 = 0;
        uint8_t *enc8 = NULL, *enc16 = NULL;

        memset(&iframe8, 0, sizeof(iframe8));
        iframe8.base.type = AX25_FRAME_INFORMATION_8BIT;
        iframe8.base.header = hdr;

        memset(&iframe16, 0, sizeof(iframe16));
        iframe16.base.type = AX25_FRAME_INFORMATION_16BIT;
        iframe16.base.header = hdr;

        enc8 = ax25_frame_encode((ax25_frame_t*) &iframe8, &len8, &err);
        enc16 = ax25_frame_encode((ax25_frame_t*) &iframe16, &len16, &err);

        if (enc8 && enc16) {
            TEST_ASSERT(len16 > len8, "I.4 Mod-128 frame larger than mod-8 (2-byte control field)", 0);
            DEBUG_PRINT("I.4 Mod-8: %zu bytes, Mod-128: %zu bytes", len8, len16);
        }

        if (enc8)
            free(enc8);
        if (enc16)
            free(enc16);
    }

    return 0;
}

// SECTION J: Digipeater Path Encode/Decode
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

    // J.1: Frame with one digipeater - round-trip
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
        TEST_ASSERT(enc != NULL && err == 0, "J.1 Encode UI frame with one digipeater", err);

        if (enc) {
            dec = ax25_frame_decode(enc, enc_len, MODULO128_FALSE, &err);
            TEST_ASSERT(dec != NULL && err == 0, "J.1 Decode digi frame", err);
            if (dec) {
                TEST_ASSERT(dec->header.repeaters.num_repeaters == 1, "J.1 Repeater count == 1", dec->header.repeaters.num_repeaters);
                TEST_ASSERT(strcmp(dec->header.repeaters.repeaters[0].callsign, "K1TTT") == 0, "J.1 Digipeater callsign preserved", 0);
                TEST_ASSERT(dec->header.repeaters.repeaters[0].ssid == 4, "J.1 Digipeater SSID=4 preserved", dec->header.repeaters.repeaters[0].ssid);
                DEBUG_PRINT("J.1 Digi: %s-%d H-bit=%d", dec->header.repeaters.repeaters[0].callsign, dec->header.repeaters.repeaters[0].ssid,
                        dec->header.repeaters.repeaters[0].ch);
                ax25_frame_free(dec, &err);
            }
            free(enc);
            enc = NULL;
        }
    }

    // J.2: Frame with digi must be exactly 7 bytes larger than without
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
            TEST_ASSERT(enc_len > enc_nodigi_len, "J.2 Frame with digi larger than without", 0);
            TEST_ASSERT((enc_len - enc_nodigi_len) == 7, "J.2 One digi adds exactly 7 bytes", (unsigned )(enc_len - enc_nodigi_len));
            DEBUG_PRINT("J.2 No-digi: %zu, with-digi: %zu, delta: %zu", enc_nodigi_len, enc_len, enc_len - enc_nodigi_len);
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

    ax25_address_free(dest, &err);
    ax25_address_free(src, &err);
    ax25_address_free(digi, &err);

    return 0;
}

// SECTION K: KISS Framing Interoperability
// Tests the KISS framing layer used by kissattach/direwolf to communicate with
// the Linux kernel AX.25 stack. Uses inline KISS helpers because the libax25v22
// kiss.h API does not export kiss_frame_encode/kiss_frame_decode/kiss_error_t
// under those names. The constants KISS_FEND/KISS_FESC etc. are defined at the
// top of this file with #ifndef fallbacks.
static int sec_k_kiss_framing(void) {
    TEST_SECTION("=== SEC-K: KISS Framing ===");

    uint8_t payload[64];
    uint8_t kiss_out[256];
    uint8_t kiss_dec[256];
    int kiss_out_len;
    int kiss_dec_len;
    int krc;
    int i;

    // K.1: Basic KISS encode (port 0, command 0 = data frame)
    {
        for (i = 0; i < 16; i++)
            payload[i] = (uint8_t) (0x41 + i);  // 'A' through 'P'

        kiss_out_len = 0;
        krc = kiss_encode_frame(payload, 16, 0, 0, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "K.1 KISS encode returns 0", krc);
        TEST_ASSERT(kiss_out_len >= 4, "K.1 KISS frame >= 4 bytes", kiss_out_len);
        // Frame must start and end with FEND (0xC0)
        TEST_ASSERT(kiss_out[0] == KISS_FEND, "K.1 KISS frame starts with FEND 0xC0", kiss_out[0]);
        TEST_ASSERT(kiss_out[kiss_out_len - 1] == KISS_FEND, "K.1 KISS frame ends with FEND 0xC0", kiss_out[kiss_out_len - 1]);
        DEBUG_PRINT("K.1 KISS encoded %d bytes -> %d bytes", 16, kiss_out_len);
    }

    // K.2: KISS decode round-trip
    {
        kiss_dec_len = 0;
        krc = kiss_decode_frame(kiss_out, kiss_out_len, kiss_dec, &kiss_dec_len);
        TEST_ASSERT(krc == 0, "K.2 KISS decode returns 0", krc);
        TEST_ASSERT(kiss_dec_len == 16, "K.2 KISS decoded length matches original", kiss_dec_len);
        TEST_ASSERT(memcmp(kiss_dec, payload, 16) == 0, "K.2 KISS decoded content matches original", 0);
        DEBUG_PRINT("K.2 KISS decode round-trip verified");
    }

    // K.3: KISS byte stuffing - FEND (0xC0) in payload must be escaped
    {
        uint8_t pf[8];
        memset(pf, 0x41, sizeof(pf));
        pf[3] = KISS_FEND;  // embed FEND in middle

        kiss_out_len = 0;
        krc = kiss_encode_frame(pf, 8, 0, 0, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "K.3 KISS encode with FEND in payload", krc);

        // No raw 0xC0 inside frame (positions 1 to kiss_out_len-2)
        int raw_fend = 0;
        for (i = 1; i < kiss_out_len - 1; i++) {
            if (kiss_out[i] == KISS_FEND) {
                raw_fend = 1;
                break;
            }
        }
        TEST_ASSERT(raw_fend == 0, "K.3 No raw FEND inside encoded KISS frame", 0);

        kiss_dec_len = 0;
        krc = kiss_decode_frame(kiss_out, kiss_out_len, kiss_dec, &kiss_dec_len);
        TEST_ASSERT(krc == 0 && kiss_dec_len == 8, "K.3 KISS decode with FEND recovers payload", kiss_dec_len);
        TEST_ASSERT(kiss_dec[3] == KISS_FEND, "K.3 FEND byte recovered after destuffing", kiss_dec[3]);
        DEBUG_PRINT("K.3 KISS FEND stuffing/destuffing verified");
    }

    // K.4: KISS byte stuffing - FESC (0xDB) in payload must be escaped
    {
        uint8_t pf[8];
        memset(pf, 0x41, sizeof(pf));
        pf[4] = KISS_FESC;  // embed FESC in middle

        kiss_out_len = 0;
        krc = kiss_encode_frame(pf, 8, 0, 0, kiss_out, &kiss_out_len);
        TEST_ASSERT(krc == 0, "K.4 KISS encode with FESC in payload", krc);

        kiss_dec_len = 0;
        krc = kiss_decode_frame(kiss_out, kiss_out_len, kiss_dec, &kiss_dec_len);
        TEST_ASSERT(krc == 0 && kiss_dec_len == 8, "K.4 KISS decode with FESC recovers payload", kiss_dec_len);
        TEST_ASSERT(kiss_dec[4] == KISS_FESC, "K.4 FESC byte recovered after destuffing", kiss_dec[4]);
        DEBUG_PRINT("K.4 KISS FESC stuffing/destuffing verified");
    }

    return 0;
}

// SECTION L: Buffer Pool Exhaustion Test
static int sec_l_buffer_pool_exhaustion(void) {
    TEST_SECTION("=== SEC-L: Buffer Pool Exhaustion ===");

    uint8_t err;
    uint8_t free_before, free_after;

    {
        free_before = ax25_buf_pool_free_count();
        TEST_ASSERT(free_before > 0, "L.1 Pool has at least 1 buffer initially", (unsigned ) free_before);

        int rc = test_buffer_pool_exhaustion(&err);
        TEST_ASSERT(rc == 0 && err == 0, "L.1 Pool exhaustion and recovery succeeded", err);

        free_after = ax25_buf_pool_free_count();
        TEST_ASSERT(free_after == free_before, "L.1 Pool count restored to pre-exhaustion level", (unsigned ) free_after);
        DEBUG_PRINT("L.1 Pool: before=%u after=%u", (unsigned) free_before, (unsigned) free_after);
    }

    return 0;
}

// SECTION M: axconfig API Test
static int sec_m_ax25config_api(void) {
    TEST_SECTION("=== SEC-M: axconfig API (libax25) ===");

    int pipe_fds[2];
    pid_t child;
    int status;
    int child_port_count = 0;
    ssize_t rd;

    // Create pipe before fork to relay port count from child.
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
        // All libax25 static heap is reclaimed by OS on _exit().
        int rc;
        int local_port_count = 0;
        char cfg_addr_copy[MAX_CALLSIGN_LEN];
        char cfg_dev_copy[MAX_PORT_NAME_LEN];
        int child_result = 0;

        close(pipe_fds[0]);

        // M.1: Load axports via libax25 axconfig API.
        rc = ax25_config_load_ports();
        // Relay raw port count to parent before any skip/exit.
        if (write(pipe_fds[1], &rc, sizeof(rc)) != sizeof(rc)) {
            close(pipe_fds[1]);
            _exit(2);
        }

        if (rc == 0) {
            // No ports -- nothing more to validate.
            close(pipe_fds[1]);
            _exit(0);
        }
        local_port_count = rc;

        // M.2: Query callsign for first port.
        if (local_port_count > 0 && g_test_ctx.port_count > 0) {
            char *cfg_addr = ax25_config_get_addr(g_test_ctx.port_name);
            if (cfg_addr == NULL) {
                fprintf(stderr, "CHILD FAIL: M.2 ax25_config_get_addr returned NULL\n");
                child_result = 1;
            } else {
                // Copy immediately - other calls may reuse static buffer.
                safe_strlcpy(cfg_addr_copy, cfg_addr, sizeof(cfg_addr_copy));
                ax25_address addr;
                int aton_rc = ax25_aton_entry(cfg_addr_copy, (char*) &addr);
                if (aton_rc != 0) {
                    fprintf(stderr, "CHILD FAIL: M.2 ax25_aton_entry(%s) = %d\n", cfg_addr_copy, aton_rc);
                    child_result = 1;
                }
            }
        }

        // M.3: Query device name for first port.
        if (local_port_count > 0 && g_test_ctx.port_count > 0) {
            char *cfg_dev = ax25_config_get_dev(g_test_ctx.port_name);
            if (cfg_dev != NULL) {
                safe_strlcpy(cfg_dev_copy, cfg_dev, sizeof(cfg_dev_copy));
                if (strlen(cfg_dev_copy) == 0) {
                    fprintf(stderr, "CHILD FAIL: M.3 device name is empty\n");
                    child_result = 1;
                }
            }
            // NULL cfg_dev means port not mapped to device: acceptable skip.
        }

        close(pipe_fds[1]);
        // child_result 0 = all sub-tests passed inside child.
        _exit(child_result);
        // ---- end child ----
    }

    // ---- parent process ----
    close(pipe_fds[1]);

    // Read port count written by child before any possible early _exit.
    rd = read(pipe_fds[0], &child_port_count, sizeof(child_port_count));
    close(pipe_fds[0]);
    if (rd != (ssize_t) sizeof(child_port_count))
        child_port_count = 0;

    // Reap child to avoid zombie; collect exit status for M.2/M.3.
    waitpid(child, &status, 0);
    int child_exit = (WIFEXITED(status)) ? WEXITSTATUS(status) : 127;

    // M.1 assertion: evaluated against port count relayed from child.
    if (child_port_count == 0) {
        printf("SKIP: M.1 ax25_config_load_ports returned 0 "
                "(no ports configured or axports absent)\n");
        return 0;
    }
    TEST_ASSERT(child_port_count > 0, "M.1 ax25_config_load_ports returned port count > 0", child_port_count);
    DEBUG_PRINT("M.1 ax25_config_load_ports returned %d port(s)", child_port_count);

    // M.2/M.3: child exit code 0 means all in-child sub-tests passed.
    if (g_test_ctx.port_count > 0) {
        TEST_ASSERT(child_exit == 0, "M.2/M.3 axconfig address + device queries passed in child", child_exit);
    }

    return 0;
}

// SECTION N: SOCK_DGRAM UI Frame via AF_AX25
// Tests the datagram (unconnected UI-frame) socket type. SOCK_DGRAM maps to
// UI frames at the kernel level and is used by APRS, beacons, packet monitors,
// and kissattach itself. Coverage of this code path is needed to detect any
// failure in the kernel datagram path or SOCK_DGRAM address binding.
static int sec_n_sock_dgram_ui_frames(void) {
    TEST_SECTION("=== SEC-N: SOCK_DGRAM UI Frame via AF_AX25 ===");

    int sock;
    int rc;

    if (!g_test_ctx.kernel_ax25_available) {
        printf("SKIP: SEC-N (no kernel AF_AX25 support)\n");
        return 0;
    }

    // N.1: Create AF_AX25 SOCK_DGRAM socket
    {
        sock = socket(AF_AX25, SOCK_DGRAM, 0);
        if (sock < 0) {
            if (errno == EPROTONOSUPPORT || errno == ESOCKTNOSUPPORT) {
                printf("SKIP: SEC-N (SOCK_DGRAM not supported for AF_AX25 "
                        "on this kernel)\n");
                return 0;
            }
            TEST_ASSERT(sock >= 0, "N.1 Create AF_AX25 SOCK_DGRAM", errno);
            return 0;
        }
        TEST_ASSERT(sock >= 0, "N.1 Create AF_AX25 SOCK_DGRAM socket", 0);
        DEBUG_PRINT("N.1 AF_AX25 SOCK_DGRAM socket fd=%d", sock);
        close(sock);
    }

    // N.2: Bind SOCK_DGRAM to local callsign + device (requires configured port)
    {
        if (!g_test_ctx.socket_bind_available) {
            printf("SKIP: N.2/N.3 SOCK_DGRAM bind "
                    "(AX.25 interface not configured)\n");
            return 0;
        }

        sock = socket(AF_AX25, SOCK_DGRAM, 0);
        TEST_ASSERT(sock >= 0, "N.2 Create SOCK_DGRAM for bind test", 0);
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
            printf("SKIP: N.2 ax25_aton_entry failed for %s\n", g_test_ctx.local_call);
            close(sock);
            return 0;
        }

        rc = bind(sock, (struct sockaddr*) &local_addr, sizeof(struct sockaddr_ax25));
        TEST_ASSERT(rc == 0, "N.2 Bind SOCK_DGRAM to local callsign", errno);
        if (rc < 0) {
            close(sock);
            return 0;
        }
        DEBUG_PRINT("N.2 SOCK_DGRAM bound to %s on %s", g_test_ctx.local_call, g_test_ctx.port_name);

        // N.3: Set non-blocking; recvfrom must return EAGAIN when no frame is queued.
        // Proves the kernel accepted the bind and is listening without hanging.
        {
            int flags = fcntl(sock, F_GETFL, 0);
            if (flags < 0) {
                printf("SKIP: N.3 fcntl F_GETFL failed\n");
            } else {
                rc = fcntl(sock, F_SETFL, flags | O_NONBLOCK);
                TEST_ASSERT(rc == 0, "N.3 Set SOCK_DGRAM non-blocking", rc);

                uint8_t recv_buf[512];
                int n_recv = (int) recvfrom(sock, recv_buf, sizeof(recv_buf), 0, NULL, NULL);
                // EAGAIN or EWOULDBLOCK = no frame available, which is correct behavior
                int recv_ok = (n_recv < 0) && (errno == EAGAIN || errno == EWOULDBLOCK);
                TEST_ASSERT(recv_ok, "N.3 SOCK_DGRAM non-blocking recvfrom returns EAGAIN/EWOULDBLOCK", errno);
                DEBUG_PRINT("N.3 recvfrom n=%d errno=%d (EAGAIN=%d EWOULDBLOCK=%d)", n_recv, errno, EAGAIN, EWOULDBLOCK);
            }
        }

        close(sock);
    }

    return 0;
}

// SECTION O: /proc/sys/net/ax25 Sysctl Parameter Interface
// Reads Linux kernel AX.25 per-port parameters from /proc/sys/net/ax25/<port>/.
// These are the authoritative kernel-side settings: kissattach, axparms, and the
// kernel state machine read these values at runtime.
// All timer values are in 1/10th-second units (the kernel internal representation).
static int sec_o_sysctl_ax25_params(void) {
    TEST_SECTION("=== SEC-O: /proc/sys/net/ax25 Sysctl Interface ===");

    char path[128];
    char buf[32];
    FILE *fp;
    int value;
    int n;

    if (!g_test_ctx.kernel_ax25_available || g_test_ctx.port_count == 0) {
        printf("SKIP: SEC-O (no kernel AX.25 support or no configured ports)\n");
        return 0;
    }

    // O.1: t1_timeout - retransmit timer (1s-30s range = 10-300 in 1/10th-sec units)
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/t1_timeout", g_test_ctx.port_name);
        if (n <= 0 || n >= (int) sizeof(path)) {
            printf("SKIP: O.1 path too long\n");
        } else {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.1 %s not readable (%s) - port not active in kernel\n", path, strerror(errno));
            } else {
                memset(buf, 0, sizeof(buf));
                if (fgets(buf, sizeof(buf), fp) != NULL) {
                    value = atoi(buf);
                    TEST_ASSERT(value >= 10 && value <= 300, "O.1 t1_timeout in range [10..300] (1/10th-sec units)", value);
                    DEBUG_PRINT("O.1 t1_timeout=%d (%d ms)", value, value * 100);
                }
                fclose(fp);
            }
        }
    }

    // O.2: maximum_retry_count - N2 retry counter (range 1-31 per kernel doc)
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/maximum_retry_count", g_test_ctx.port_name);
        if (n <= 0 || n >= (int) sizeof(path)) {
            printf("SKIP: O.2 path too long\n");
        } else {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.2 %s not readable (%s)\n", path, strerror(errno));
            } else {
                memset(buf, 0, sizeof(buf));
                if (fgets(buf, sizeof(buf), fp) != NULL) {
                    value = atoi(buf);
                    TEST_ASSERT(value >= 1 && value <= 31, "O.2 maximum_retry_count N2 in range [1..31]", value);
                    DEBUG_PRINT("O.2 maximum_retry_count N2=%d", value);
                }
                fclose(fp);
            }
        }
    }

    // O.3: t3_timeout - keep-alive timer (0=disabled; 0-36000 in 1/10th-sec units)
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/t3_timeout", g_test_ctx.port_name);
        if (n <= 0 || n >= (int) sizeof(path)) {
            printf("SKIP: O.3 path too long\n");
        } else {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.3 %s not readable (%s)\n", path, strerror(errno));
            } else {
                memset(buf, 0, sizeof(buf));
                if (fgets(buf, sizeof(buf), fp) != NULL) {
                    value = atoi(buf);
                    // 0 disables T3; upper limit is 1 hour = 36000 in 1/10th-sec units
                    TEST_ASSERT(value >= 0 && value <= 36000, "O.3 t3_timeout in range [0..36000] (0=disabled)", value);
                    DEBUG_PRINT("O.3 t3_timeout=%d (%d ms, 0=disabled)", value, value * 100);
                }
                fclose(fp);
            }
        }
    }

    // O.4: standard_window_size - mod-8 window size (range 1-7 per kernel doc)
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/standard_window_size", g_test_ctx.port_name);
        if (n <= 0 || n >= (int) sizeof(path)) {
            printf("SKIP: O.4 path too long\n");
        } else {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.4 %s not readable (%s)\n", path, strerror(errno));
            } else {
                memset(buf, 0, sizeof(buf));
                if (fgets(buf, sizeof(buf), fp) != NULL) {
                    value = atoi(buf);
                    TEST_ASSERT(value >= 1 && value <= 7, "O.4 standard_window_size (mod-8) in range [1..7]", value);
                    DEBUG_PRINT("O.4 standard_window_size=%d", value);
                }
                fclose(fp);
            }
        }
    }

    // O.5: extended_window_size - mod-128 window size (range 1-63 per kernel doc)
    {
        n = snprintf(path, sizeof(path), "/proc/sys/net/ax25/%s/extended_window_size", g_test_ctx.port_name);
        if (n <= 0 || n >= (int) sizeof(path)) {
            printf("SKIP: O.5 path too long\n");
        } else {
            fp = fopen(path, "r");
            if (!fp) {
                printf("SKIP: O.5 %s not readable (%s)\n", path, strerror(errno));
            } else {
                memset(buf, 0, sizeof(buf));
                if (fgets(buf, sizeof(buf), fp) != NULL) {
                    value = atoi(buf);
                    TEST_ASSERT(value >= 1 && value <= 63, "O.5 extended_window_size (mod-128) in range [1..63]", value);
                    DEBUG_PRINT("O.5 extended_window_size=%d", value);
                }
                fclose(fp);
            }
        }
    }

    return 0;
}

// SECTION P: full_sockaddr_ax25 with Digipeater Via Path
// Tests the kernel extended socket address structure (full_sockaddr_ax25) that
// carries a digipeater via path in fsa_digipeater[]. This structure is required
// by connect() calls through digipeaters and is used by ax25d and call(1).
static int sec_p_full_sockaddr_digipeater(void) {
    TEST_SECTION("=== SEC-P: full_sockaddr_ax25 Digipeater Path ===");

    int sock;
    struct full_sockaddr_ax25 faddr;
    int rc;

    if (!g_test_ctx.kernel_ax25_available) {
        printf("SKIP: SEC-P (no kernel AF_AX25 support)\n");
        return 0;
    }

    // P.1: Build full_sockaddr_ax25 with sax25_ndigis=1 and one digipeater K1TTT-4
    {
        memset(&faddr, 0, sizeof(faddr));
        faddr.fsa_ax25.sax25_family = AF_AX25;
        faddr.fsa_ax25.sax25_ndigis = 1;

        rc = ax25_aton_entry(g_test_ctx.local_call, (char*) &faddr.fsa_ax25.sax25_call);
        TEST_ASSERT(rc == 0, "P.1 ax25_aton_entry for local call into fsa_ax25.sax25_call", rc);

        rc = ax25_aton_entry("K1TTT-4", (char*) &faddr.fsa_digipeater[0]);
        TEST_ASSERT(rc == 0, "P.1 ax25_aton_entry K1TTT-4 into fsa_digipeater[0]", rc);

        TEST_ASSERT(faddr.fsa_ax25.sax25_ndigis == 1, "P.1 fsa_ax25.sax25_ndigis == 1", faddr.fsa_ax25.sax25_ndigis);
        TEST_ASSERT(faddr.fsa_ax25.sax25_family == AF_AX25, "P.1 sax25_family == AF_AX25", faddr.fsa_ax25.sax25_family);
        DEBUG_PRINT("P.1 Built full_sockaddr_ax25 with 1 digipeater K1TTT-4");
    }

    // P.2: Verify sizeof(full_sockaddr_ax25) > sizeof(sockaddr_ax25)
    // full_sockaddr_ax25 = sockaddr_ax25 (17 bytes) + AX25_MAX_DIGIS * 7 bytes
    {
        int full_size = (int) sizeof(struct full_sockaddr_ax25);
        int base_size = (int) sizeof(struct sockaddr_ax25);
        TEST_ASSERT(full_size > base_size, "P.2 sizeof(full_sockaddr_ax25) > sizeof(sockaddr_ax25)", full_size);
        DEBUG_PRINT("P.2 sizeof(full_sockaddr_ax25)=%d sizeof(sockaddr_ax25)=%d", full_size, base_size);
    }

    // P.3: Attempt non-blocking connect() using full_sockaddr_ax25 with one digipeater.
    // Target W1AW-0 via K1TTT-4. No physical interface is required: the kernel should
    // reject gracefully with a network/device error (EHOSTUNREACH, ENODEV, ENETDOWN,
    // EINPROGRESS) rather than EINVAL or EFAULT, which would indicate the kernel failed
    // to parse the extended address structure.
    {
        sock = socket(AF_AX25, SOCK_SEQPACKET, 0);
        if (sock < 0) {
            printf("SKIP: P.3 socket creation failed\n");
        } else {
            // Put destination callsign in sax25_call for the connect target
            rc = ax25_aton_entry("W1AW-0", (char*) &faddr.fsa_ax25.sax25_call);
            if (rc < 0) {
                printf("SKIP: P.3 ax25_aton_entry W1AW-0 failed\n");
            } else {
                // Set non-blocking so connect() returns immediately
                int flags = fcntl(sock, F_GETFL, 0);
                if (flags >= 0)
                    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

                rc = connect(sock, (struct sockaddr*) &faddr, sizeof(struct full_sockaddr_ax25));

                int connect_ok = (rc < 0) && (errno != EFAULT);
                TEST_ASSERT(connect_ok, "P.3 connect() with full_sockaddr_ax25 no EFAULT "
                        "(structure accepted by kernel)", errno);
                DEBUG_PRINT("P.3 connect() rc=%d errno=%d (%s) - expected network/config error", rc, errno, strerror(errno));
            }
            close(sock);
        }
    }

    return 0;
}

// Main test runner with auto-configuration and bind capability detection
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

    printf("\n");
    printf("=============================================================\n");
    printf("  Test Summary\n");
    printf("=============================================================\n");
    printf("Total Assertions: %u\n", assert_count);

    if (failures == 0) {
        printf("✓✓✓ ALL TESTS PASSED ✓✓✓\n");
    } else {
        printf("✗✗✗ %d TEST SECTION(S) FAILED ✗✗✗\n", failures);
    }

    printf("AF_AX25 Kernel Support: %s\n", g_test_ctx.kernel_ax25_available ? "YES" : "NO");
    printf("AF_AX25 Socket Bind: %s\n", g_test_ctx.socket_bind_available ? "YES" : "NO");
    printf("AX.25 Port: %s\n", g_test_ctx.port_name);
    printf("AX.25 Callsign: %s\n", g_test_ctx.local_call);
    printf("AX.25 Configuration: %s\n", g_test_ctx.port_count > 0 ? "LOADED" : "NOT FOUND");

    printf("=============================================================\n\n");

    return failures;
}
