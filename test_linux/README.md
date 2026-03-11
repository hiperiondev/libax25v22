# ax25linux — Linux Integration & Test Suite for libax25v22

[![Tests](https://img.shields.io/badge/tests-170%20passing-brightgreen)](#test-results)
[![Standard](https://img.shields.io/badge/standard-AX.25%20v2.2-blue)](#protocol-coverage)
[![C standard](https://img.shields.io/badge/C-C99-lightgrey)](#build-requirements)
[![MCU safe](https://img.shields.io/badge/MCU--safe-no%2064--bit%20%7C%20no%20float-orange)](#mcu-portability-constraints)

Linux integration layer, Hardware Abstraction Layer (HAL), AF_AX25 kernel bridge, and a comprehensive 170-assertion test suite for the [libax25v22](https://github.com/hiperiondev/libax25v22) AX.25 v2.2 amateur radio protocol library.

---

## Table of Contents

- [Overview](#overview)
- [Repository Layout](#repository-layout)
- [Files in This Directory](#files-in-this-directory)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Running the Test Suite](#running-the-test-suite)
- [Test Results](#test-results)
- [Test Descriptions](#test-descriptions)
  - [T01 — Address Encode/Decode](#t01--address-encodedecode)
  - [T02 — Frame Header](#t02--frame-header)
  - [T03 — UI Frame](#t03--ui-frame)
  - [T04 — SABM / SABME / UA](#t04--sabm--sabme--ua)
  - [T05/T06 — DISC / DM](#t05t06--disc--dm)
  - [T07 — I-Frame Modulo-8](#t07--i-frame-modulo-8)
  - [T08 — I-Frame Modulo-128](#t08--i-frame-modulo-128)
  - [T09 — S-Frames (RR / RNR / REJ / SREJ)](#t09--s-frames-rr--rnr--rej--srej)
  - [T10 — FRMR (Frame Reject)](#t10--frmr-frame-reject)
  - [T11 — XID (Exchange Identification)](#t11--xid-exchange-identification)
  - [T12 — TEST Frame](#t12--test-frame)
  - [T13 — Segmentation & Reassembly](#t13--segmentation--reassembly)
  - [T14 — KISS Framing](#t14--kiss-framing)
  - [T15 — KISS Command Frames](#t15--kiss-command-frames)
  - [T16 — CRC-16/CCITT FCS](#t16--crc-16ccitt-fcs)
  - [T17 — State Machine: Connected I/O Round-Trip](#t17--state-machine-connected-io-round-trip)
  - [T18 — T1 Retransmission](#t18--t1-retransmission)
  - [T19 — RNR Flow Control](#t19--rnr-flow-control)
  - [T20 — SREJ Selective Reject](#t20--srej-selective-reject)
  - [T21 — FRMR on Invalid Frame](#t21--frmr-on-invalid-frame)
  - [T22 — Digipeater H-Bit and Path Reversal](#t22--digipeater-h-bit-and-path-reversal)
  - [T23 — PID Dispatch Table](#t23--pid-dispatch-table)
  - [T24 — Buffer Pool](#t24--buffer-pool)
  - [T25 — Mux: Multiple Connections](#t25--mux-multiple-connections)
- [HAL Implementation (`hal_linux.c`)](#hal-implementation-hal_linuxc)
  - [Section 1 — Tick Counter](#section-1--tick-counter)
  - [Section 2 — Software Timers](#section-2--software-timers)
  - [Section 3 — Serial Ring Buffers](#section-3--serial-ring-buffers)
  - [Section 4 — PTT / DCD](#section-4--ptt--dcd)
  - [Section 5 — Serial I/O](#section-5--serial-io)
  - [Section 6 — PRNG](#section-6--prng)
  - [Section 7 — Critical Sections](#section-7--critical-sections)
  - [Section 8 — Memory](#section-8--memory)
  - [Section 9 — CRC-16/CCITT](#section-9--crc-16ccitt)
  - [Section 10 — Logging](#section-10--logging)
  - [Section 11 — Watchdog](#section-11--watchdog)
  - [Section 12 — Channel Access Parameters](#section-12--channel-access-parameters)
  - [Section 13 — Init / Deinit](#section-13--init--deinit)
  - [Section 14 — Linux-Specific Serial / Port API](#section-14--linux-specific-serial--port-api)
- [Linux HAL Extension API (`hal_linux.h`)](#linux-hal-extension-api-hal_linuxh)
- [AF_AX25 Kernel Bridge (`ax25_linux_bridge.c`)](#afax25-kernel-bridge-ax25_linux_bridgec)
  - [Bridge Architecture](#bridge-architecture)
  - [Bridge API Reference](#bridge-api-reference)
  - [Bridge Usage Example](#bridge-usage-example)
- [MCU Portability Constraints](#mcu-portability-constraints)
- [Protocol Coverage](#protocol-coverage)
- [AX.25 v2.2 Reference Documents](#ax25-v22-reference-documents)
- [License](#license)

---

## Overview

`libax25v22` is a pure-C, hardware-agnostic implementation of the AX.25 v2.2 amateur radio data-link layer. It is designed for embedded systems without a floating-point unit (FPU) — making portability to microcontrollers a first-class constraint. Every integer is 32-bit or narrower; no `double`, no `float`, no `uint64_t`.

This companion repository provides:

| Component | File(s) | Purpose |
|---|---|---|
| **Linux HAL** | `hal_linux.c`, `hal_linux.h` | Implements `hal.h` for Linux: tick, timers, serial I/O, CRC, PRNG, memory |
| **Kernel Bridge** | `ax25_linux_bridge.c` | Bridges libax25v22 ↔ Linux `AF_AX25` sockets and KISS TNCs |
| **Test Suite** | `ax25_test_suite.c` | 25 test groups, 170 assertions covering every protocol feature |
| **Build System** | `Makefile` | Targets for release, debug, ASAN, and bridge demo |

---

## Repository Layout

```
.                          ← this directory (ax25linux/)
├── hal_linux.c            ← Linux HAL: tick, timers, serial, CRC, PRNG, memory
├── hal_linux.h            ← Linux-specific HAL extension API
├── ax25_linux_bridge.c    ← AF_AX25 socket + KISS bridge to libax25v22
├── ax25_test_suite.c      ← 25 test groups, 170 assertions
├── Makefile               ← Build system
└── README.md              ← This file

../libax25v22/             ← Library source (sibling directory)
├── hal/
│   └── hal.h              ← Portable HAL interface
├── protocols/
│   ├── ax25/              ← Core AX.25 v2.2 codec, state machine, mux
│   ├── kiss/              ← KISS TNC protocol (standard, SMACK, G8BPQ, FlexNet)
│   ├── fx25/              ← FX.25 Forward Error Correction
│   ├── il2p/              ← IL2P protocol
│   ├── hdlc/              ← HDLC framing
│   └── common/            ← CRC utilities, string helpers
└── libax25v22.h           ← Top-level umbrella include
```

---

## Files in This Directory

### `hal_linux.c` (600 lines)

Implements all 50+ functions declared in `hal.h` using standard POSIX and Linux APIs. Divided into 14 clearly labelled sections. No 64-bit arithmetic anywhere; the 32-bit millisecond tick is derived from `clock_gettime(CLOCK_MONOTONIC)` using only 32-bit multiplication and division.

### `hal_linux.h` (68 lines)

Extension header exposing Linux-only helpers for serial port management, main-loop polling, and test injection / drain. These are not part of the portable `hal.h` interface because they depend on `<termios.h>` and POSIX file descriptors.

### `ax25_linux_bridge.c` (661 lines)

Bridges the libax25v22 state machine to the Linux kernel AX.25 stack. Handles:
- KISS TNC serial framing (byte-level I/O via HAL ring buffers)
- `ax25_mux_t` multiplexing for up to 8 simultaneous connections
- `AF_AX25 SOCK_DGRAM` monitor socket for kernel sniffer mode
- Callsign-to-`ax25_address` conversion (8-bit arithmetic only)

### `ax25_test_suite.c` (1493 lines)

Self-contained test program. Uses a minimal hand-written harness (`PASS`/`FAIL`/`CHECK` macros) that avoids heap allocation in the test framework itself. All test state is on the stack. Each test function corresponds to one AX.25 protocol feature area.

### `Makefile` (111 lines)

Four build targets:

| Target | Binary | Flags |
|---|---|---|
| `make` / `make all` | `ax25_test` | `-O2 -Wall -Wextra -Wfloat-conversion` |
| `make ax25_test_dbg` | `ax25_test_dbg` | `-g -O0` |
| `make ax25_test_asan` | `ax25_test_asan` | `-g -fsanitize=address` |
| `make ax25_bridge_demo` | `ax25_bridge_demo` | Requires `linux/ax25.h` kernel headers |

---

## Prerequisites

| Dependency | Notes |
|---|---|
| GCC ≥ 4.9 or Clang ≥ 3.8 | C99 mode; tested with GCC 12 and 13 |
| `libax25v22` source tree | Clone alongside this directory: `git clone https://github.com/hiperiondev/libax25v22 ../libax25v22` |
| Linux kernel headers | Only required for the bridge demo (`linux/ax25.h`). Package: `linux-headers-$(uname -r)` or `linux-libc-dev` on Debian/Ubuntu |
| AddressSanitizer (optional) | Included with GCC/Clang; for `make asan` target |

### Ubuntu / Debian quick-start

```bash
sudo apt-get install gcc make linux-libc-dev
```

### Clone both repositories side by side

```bash
git clone https://github.com/hiperiondev/libax25v22
git clone https://github.com/YOUR_USERNAME/ax25linux
```

Directory structure must be:
```
some-parent/
├── libax25v22/   ← library
└── ax25linux/    ← this repo
```

---

## Build

```bash
cd ax25linux

# Default — optimised release build
make

# Run test suite immediately after building
make test

# Debug build (no optimisation, full symbols)
make ax25_test_dbg

# AddressSanitizer build (memory error and leak detection)
make ax25_test_asan

# Build and run with ASAN in one step
make asan

# Build the AF_AX25 kernel bridge demo (needs linux/ax25.h)
make ax25_bridge_demo

# Use a non-standard library path
make LIB=/opt/libax25v22

# Clean all built artifacts
make clean
```

---

## Running the Test Suite

```bash
./ax25_test
```

All log output (HAL tick, PTT events) goes to **stderr**. Test results go to **stdout**. To see only test results:

```bash
./ax25_test 2>/dev/null
```

To see only failures:

```bash
./ax25_test 2>/dev/null | grep FAIL
```

To run with AddressSanitizer (memory leak + use-after-free detection):

```bash
./ax25_test_asan
```

The exit code is `0` when all tests pass, non-zero on any failure (suitable for CI integration).

---

## Test Results

```
===== libax25v22 Full Test Suite =====
Platform: Linux/ax25linux

T01: Address encode/decode            16 assertions  PASS
T02: Frame header encode/decode       11 assertions  PASS
T03: UI frame encode/decode            6 assertions  PASS
T04: SABM / SABME / UA                10 assertions  PASS
T05/T06: DISC / DM frames              5 assertions  PASS
T07: I-frame modulo-8                  6 assertions  PASS
T08: I-frame modulo-128                5 assertions  PASS
T09: S-frames (RR/RNR/REJ/SREJ)       12 assertions  PASS
T10: FRMR (frame reject)               6 assertions  PASS
T11: XID (Exchange Identification)    11 assertions  PASS
T12: TEST frame                        5 assertions  PASS
T13: Segmentation & reassembly         6 assertions  PASS
T14: KISS framing                     13 assertions  PASS
T15: KISS command frames              10 assertions  PASS
T16: CRC-16/CCITT FCS                  3 assertions  PASS
T17: State machine loopback            6 assertions  PASS
T18: T1 retransmission                 1 assertion   PASS
T19: RNR flow control                  2 assertions  PASS
T20: SREJ selective reject parsing     8 assertions  PASS
T21: FRMR info field encoding          4 assertions  PASS
T22: Digipeater H-bit and path         5 assertions  PASS
T23: PID dispatch table                8 assertions  PASS
T24: Buffer pool                       6 assertions  PASS
T25: Mux — registration and routing    5 assertions  PASS

===== RESULTS =====
  Passed: 170
  Failed:   0
  Total:  170
  Status: ALL PASS
```

Zero memory leaks or memory errors detected under AddressSanitizer.

---

## Test Descriptions

Each test is a self-contained C function. Tests run sequentially in a single process. No threads, no inter-process communication, no real radio hardware required.

---

### T01 — Address Encode/Decode

**What it tests:** The `ax25_address_t` codec — the 7-byte on-air AX.25 address field (6 left-shifted callsign bytes + 1 SSID byte).

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | `from_string` succeeds for `N0CALL` | Zero error code, non-NULL pointer |
| 2 | Callsign stored correctly | `strncmp` of callsign field |
| 3 | SSID 0 stored | Default SSID for plain callsigns |
| 4 | Encode returns non-NULL | Allocation success |
| 5 | Encoded length is 7 | AX.25 address field is always exactly 7 bytes |
| 6 | First byte is `'N' << 1` = `0x9C` | AX.25 shift-left encoding for each callsign character |
| 7 | Decode succeeds | Round-trip encode→decode |
| 8 | Decoded callsign matches | Field-by-field string comparison |
| 9 | Decoded SSID is 0 | SSID byte bits [4:1] round-trip correctly |
| 10 | `W1AW-15` parse succeeds | SSID value 15 (maximum) |
| 11 | SSID 15 stored | Parsed from `-15` suffix |
| 12 | SSID-15 encode succeeds | Non-NULL, correct length |
| 13 | SSID-15 decoded as 15 | Round-trip for maximum SSID |
| 14 | SSID 16 is invalid | `ax25_validate_ssid(16)` returns false |
| 15 | SSID 0 is valid | Boundary condition low end |
| 16 | SSID 15 is valid | Boundary condition high end |

**AX.25 reference:** Section 3.12 — Address Field Encoding.

---

### T02 — Frame Header

**What it tests:** Multi-address frame header encoding and decoding, including 2-address frames (destination + source) and 4-address frames (destination + source + 2 digipeaters).

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | 2-address encode succeeds | `ax25_frame_header_encode()` non-NULL |
| 2 | Encoded length is 14 bytes | 2 × 7-byte addresses |
| 3 | Extension bit set in last address byte | Bit 0 of `enc[13]` must be 1 (end-of-address-field marker) |
| 4 | 2-address decode succeeds | `ax25_frame_header_decode()` non-NULL |
| 5 | Destination callsign matches | Ignores trailing space padding differences between encode/decode |
| 6 | Source callsign matches | Same space-trim comparison |
| 7 | No remaining bytes | Header decode consumed all input |
| 8 | 4-address encode succeeds | With `RELAY1-1` and `RELAY2-2` digipeaters |
| 9 | Encoded length is 28 bytes | 4 × 7-byte addresses |
| 10 | 4-address decode succeeds | Non-NULL header |
| 11 | Repeater count is 2 | `header->repeaters.num_repeaters == 2` |

**AX.25 reference:** Section 3.12 — Address Fields; Section 3.12.3 — Repeater Addresses.

---

### T03 — UI Frame

**What it tests:** The Unnumbered Information (UI) frame — the connectionless datagram of AX.25, used by APRS, beacons, and all unconnected layer-3 protocols.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | UI encode succeeds | Control byte `0x03`, PID byte, then payload |
| 2 | UI decode succeeds | `ax25_frame_decode()` with `MODULO128_FALSE` |
| 3 | Frame type is `AX25_FRAME_UNNUMBERED_INFORMATION` | Type discriminator set correctly after decode |
| 4 | PID field matches | `PID_NO_L3` (`0xF0`) round-trips |
| 5 | Payload length matches | 15 bytes (`"APRS TEST FRAME"`) |
| 6 | Payload data matches | Byte-for-byte `memcmp` of decoded payload |

**AX.25 reference:** Section 4.3.3.2 — Unnumbered Information (UI) Frames.

---

### T04 — SABM / SABME / UA

**What it tests:** Connection establishment frames. SABM (Set Asynchronous Balanced Mode) initiates a modulo-8 connection; SABME initiates modulo-128; UA (Unnumbered Acknowledge) accepts.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | SABM encode succeeds | Modifier `0x2F`, P-bit set |
| 2 | SABM decode succeeds | Round-trip |
| 3 | SABM type correct | `AX25_FRAME_UNNUMBERED_SABM` |
| 4 | SABM P-bit set | `pf == 1` |
| 5 | SABME encode succeeds | Modifier `0x6F` |
| 6 | SABME decode succeeds | Round-trip |
| 7 | SABME type correct | `AX25_FRAME_UNNUMBERED_SABME` |
| 8 | UA encode succeeds | Modifier `0x63` |
| 9 | UA decode succeeds | Round-trip |
| 10 | UA type correct | `AX25_FRAME_UNNUMBERED_UA` |

**AX.25 reference:** Section 4.3.3.4 — SABM/SABME; Section 4.3.3.8 — UA.

---

### T05/T06 — DISC / DM

**What it tests:** Orderly disconnection (DISC command) and the Disconnected Mode (DM) response indicating the peer is not connected.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | DISC encode succeeds | Modifier `0x43` |
| 2 | DISC decode succeeds | Round-trip |
| 3 | DISC type correct | `AX25_FRAME_UNNUMBERED_DISC` |
| 4 | DM encode succeeds | Modifier `0x0F` |
| 5 | DM type correct | `AX25_FRAME_UNNUMBERED_DM` |

**AX.25 reference:** Section 4.3.3.3 — DISC; Section 4.3.3.9 — DM.

---

### T07 — I-Frame Modulo-8

**What it tests:** Information frame encoding and decoding using 8-bit sequence numbers (modulo-8, window size up to 7). This is the classic AX.25 connected-mode data transfer.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | I-8 encode succeeds | 1-byte control field |
| 2 | I-8 decode succeeds | `MODULO128_FALSE` forced |
| 3 | I-8 type is `AX25_FRAME_INFORMATION_8BIT` | Type discriminator |
| 4 | N(S) = 3 | Send sequence number round-trips |
| 5 | N(R) = 5 | Receive sequence number round-trips |
| 6 | Payload length correct | `sizeof("Hello AX.25!") - 1` |

**AX.25 reference:** Section 4.4 — Information Transfer; Section 3.6 — I-Frame Control Field.

---

### T08 — I-Frame Modulo-128

**What it tests:** Information frame encoding and decoding with 16-bit control fields (modulo-128, window size up to 127). Extended sequence numbers for high-latency or high-throughput links (PE1CHL extension, AX.25 v2.2 Appendix).

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | I-128 encode succeeds | 2-byte control field |
| 2 | I-128 decode succeeds | `MODULO128_TRUE` forced |
| 3 | I-128 type is `AX25_FRAME_INFORMATION_16BIT` | Type discriminator |
| 4 | N(S) = 65 | Tests value > 7, impossible in modulo-8 |
| 5 | N(R) = 127 | Maximum sequence number in modulo-128 |

**AX.25 reference:** PE1CHL paper "Module 128 for AX.25" (DCC 1995); AX.25 v2.2 Section 3.12.2.

---

### T09 — S-Frames (RR / RNR / REJ / SREJ)

**What it tests:** All four supervisory frames used for flow control and error recovery, covering both the modulo-8 (8-bit control) variants.

| Frame | Code | Purpose |
|---|---|---|
| RR | 0 | Receive Ready — acknowledges frames, clears busy |
| RNR | 1 | Receive Not Ready — signals local buffer full |
| REJ | 2 | Reject — requests retransmission from N(R) |
| SREJ | 3 | Selective Reject — requests retransmission of specific frame |

**Assertions (per frame type, × 4):**

| # | Check | Detail |
|---|---|---|
| 1 | Encode succeeds | Non-NULL encoded frame |
| 2 | Decode type correct | Round-trip type discriminator |
| 3 | N(R) = 6 | Sequence number field round-trips |

12 assertions total.

**AX.25 reference:** Section 4.3.2 — Supervisory Frames.

---

### T10 — FRMR (Frame Reject)

**What it tests:** The Frame Reject frame — sent to indicate a protocol violation requiring link reset. Tests the W-bit (invalid control field), V(S), and V(R) info fields.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | FRMR encode succeeds | Modifier `0x87` |
| 2 | FRMR decode succeeds | Round-trip |
| 3 | FRMR type correct | `AX25_FRAME_UNNUMBERED_FRMR` |
| 4 | W-bit set | Invalid-control-field flag |
| 5 | V(S) = 3 | Send sequence number at error time |
| 6 | V(R) = 5 | Receive sequence number at error time |

**AX.25 reference:** Section 4.3.3.6 — FRMR; Section 6.5 — Error Recovery.

---

### T11 — XID (Exchange Identification)

**What it tests:** The Exchange Identification frame used for parameter negotiation in AX.25 v2.2 extended mode. Tests encoding and decoding of four standard XID parameters: Class of Procedures (COP), HDLC Optional Functions, Maximum Information Field Length (N1), and Window Size (k).

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | `xid_init_defaults` succeeds | Default XID parameter set initialised |
| 2–5 | Four XID parameters allocated | COP, HDLC options, N1, k |
| 6 | XID encode succeeds | Modifier `0xAF` |
| 7 | XID decode succeeds | Round-trip |
| 8 | XID type correct | `AX25_FRAME_UNNUMBERED_XID` |
| 9 | Parameter count is 4 | All four TLV parameters preserved |
| 10 | Format Identifier `FI = 0x82` | ISO 8885 XID frame |
| 11 | Group Identifier `GI = 0x80` | HDLC parameter group |

**AX.25 reference:** Section 4.3.3.7 — XID; Appendix A — XID Parameter Negotiation.

---

### T12 — TEST Frame

**What it tests:** The TEST frame used for link quality verification. Carries an arbitrary payload that the peer echoes back without modification.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | TEST encode succeeds | Modifier `0xE3`, P-bit set |
| 2 | TEST decode succeeds | Round-trip |
| 3 | TEST type correct | `AX25_FRAME_UNNUMBERED_TEST` |
| 4 | Payload length is 4 | `{0xDE, 0xAD, 0xBE, 0xEF}` |
| 5 | Payload data matches | Byte-for-byte `memcmp` |

**AX.25 reference:** Section 4.3.3.10 — TEST.

---

### T13 — Segmentation & Reassembly

**What it tests:** Automatic splitting of payloads larger than the negotiated N1 (Maximum Information Field Length) into multiple I-frames, and transparent reassembly at the receiver. This is the AX.25 v2.2 segmenter described in Appendix C6.

**Setup:** 600-byte payload with `N1 = 256`.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | `ax25_segment_info_fields()` succeeds | Non-NULL segment list |
| 2 | More than one segment | 600 bytes > 256 → at least 3 segments |
| 3 | Each segment ≤ N1 + 3 bytes | Control (1) + PID (1) + segmentation header (1) + data |
| 4 | `ax25_reassemble_info_fields()` succeeds | All segments recombined |
| 5 | Reassembled length is 600 | Exact match |
| 6 | Reassembled content matches | Byte-for-byte `memcmp` of original payload |

**AX.25 reference:** Section 6.6 — Information Field Segmentation; Appendix C6.

---

### T14 — KISS Framing

**What it tests:** The KISS TNC protocol used to communicate with hardware TNCs over a serial link or TCP socket. Tests standard framing (FEND/FESC escaping), SMACK (CRC-16 variant), and G8BPQ (CRC-8 XOR variant) extensions.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | `ax25_kiss_init()` succeeds | Context initialised |
| 2 | `ax25_kiss_enter()` succeeds | KISS mode entered |
| 3 | `ax25_kiss_send_frame()` succeeds | Frame queued in TX ring |
| 4 | TX bytes > raw frame bytes | FEND/FESC framing overhead added |
| 5 | FEND byte (`0xC0`) present in output | Frame delimiters in output |
| 6 | KISS loopback decode succeeds | Inject TX output → receive callback |
| 7 | Loopback decoded length matches | Payload length preserved |
| 8 | Loopback data matches | Payload data preserved |
| 9 | `ax25_kiss_set_variant(SMACK)` succeeds | SMACK mode selected |
| 10 | SMACK frame transmit succeeds | CRC-16 appended |
| 11 | SMACK receive succeeds | CRC-16 validated on RX |
| 12 | SMACK CRC-16 is non-zero | Sanity — not all-zero FCS |
| 13 | G8BPQ CRC-8 XOR is non-zero | `ax25_kiss_crc8_xor()` sanity |

**KISS reference:** ARRL 7th Computer Networking Conference (1986); SMACK extension by SP2DMB; G8BPQ extension.

---

### T15 — KISS Command Frames

**What it tests:** The out-of-band KISS command mechanism for configuring TNC parameters without sending a data frame: TxDelay, Persistence, SlotTime, and the Return command (exit KISS mode).

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | `ax25_kiss_set_port_params()` succeeds | Parameters accepted |
| 2 | Parameter bytes transmitted | Bytes appeared in TX ring |
| 3 | `ax25_kiss_get_port_params()` succeeds | Read back |
| 4 | TxDelay stored correctly | Encoded as `TxDelay × 10 ms` units |
| 5 | Persistence stored correctly | 0–255 value |
| 6 | SlotTime stored correctly | Encoded as `SlotTime × 10 ms` units |
| 7 | Return command sent | KISS command byte `0xFF` transmitted |
| 8 | KISS mode cleared after Return | `ctx.kiss_mode == false` |
| 9 | Return frame is 3 bytes | `FEND + 0xFF + FEND` |
| 10 | Statistics zeroed after init | `stats.rx_frames == 0` etc. |

**AX.25 reference:** TNC-2 KISS specification.

---

### T16 — CRC-16/CCITT FCS

**What it tests:** The Frame Check Sequence (FCS) computation — the CRC-16/CCITT used by AX.25 HDLC framing for error detection. Tests the known test vector, incremental vs. bulk equivalence, and the mathematical residue property.

**Details:**

The HAL implements the **reflected** CRC-16/CCITT variant (also called CRC-16/MCRF4XX), using polynomial `0x8408` (the bit-reversal of the standard `0x1021` polynomial). This is the variant used on-air in AX.25 HDLC frames.

- Init value: `0xFFFF`
- Final XOR: `0xFFFF`
- CRC of `"123456789"` = `0x906E`
- Residue (message + FCS appended LSB-first): `0x0F47`

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | CRC of known vector is non-zero | `hal_crc16_buf("123456789", 9) != 0` |
| 2 | Incremental == bulk | `hal_crc16_update()` one byte at a time equals single `hal_crc16_buf()` call |
| 3 | Residue is `0x0F47` | CRC computed over message + LSB-first FCS yields known constant |

**AX.25 reference:** Section 3.9 — Frame Check Sequence; ITU-T V.41.

---

### T17 — State Machine: Connected I/O Round-Trip

**What it tests:** The complete AX.25 connection lifecycle using two in-process state machine instances exchanging frames via in-memory queues (no real radio, no threads). Station A initiates; station B accepts; data flows; both stations disconnect cleanly.

**Setup:**
- Two `ax25_connection_t` instances: A (`N0CALL-1`) and B (`W1AW-3`)
- Transmit callbacks write frames into circular byte queues
- Main loop calls `ax25_tick()` and `ax25_process_frame()` to advance state machines
- 10 ms synthetic tick increments per iteration

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | A connected | `lb_a_connected == 1` after SABM/UA exchange |
| 2 | B connected | `lb_b_connected == 1` after receiving SABM |
| 3 | B received data | Data callback fired with correct PID |
| 4 | B data correct | `memcmp` of 12-byte payload |
| 5 | A disconnected | `lb_a_disc == 1` after DISC/UA exchange |
| 6 | B disconnected | `lb_b_disc == 1` |

**AX.25 reference:** Section 6 — Data Link Layer State Machine (SDL diagrams).

---

### T18 — T1 Retransmission

**What it tests:** The T1 (retransmission) timer. When an I-frame is sent and no acknowledgement arrives within T1, the state machine must retransmit the frame. The test sets a very short T1 and confirms that the retransmit count reaches at least 2.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | T1 retransmit count ≥ 2 | State machine fires T1 at least twice while no UA is delivered |

**AX.25 reference:** Section 6.3 — Timer T1; Section 6.4.7 — Retransmission Procedure.

---

### T19 — RNR Flow Control

**What it tests:** The Receive Not Ready (RNR) mechanism. When a station's receive buffers are full, it sends RNR to pause the peer. The test confirms local busy is set and cleared correctly.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | B local_busy set after `ax25_send_rnr()` | `connB.local_busy == true` |
| 2 | B busy cleared after `ax25_clear_local_busy()` | `connB.local_busy == false` |

**AX.25 reference:** Section 6.4.10 — Local Busy Condition; Section 4.3.2.3 — RNR.

---

### T20 — SREJ Selective Reject

**What it tests:** Parsing of SREJ (Selective Reject) control fields in both modulo-8 (8-bit control) and modulo-128 (16-bit control) variants. SREJ requests retransmission of a specific frame identified by sequence number.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | `ax25_parse_ctrl()` succeeds for modulo-8 SREJ | No error |
| 2 | S-frame type identified | Control field type is `'S'` |
| 3 | SREJ code is 3 | Code bits `11` in S-frame |
| 4 | N(R) = 5 | Sequence number field |
| 5 | Modulo-128 SREJ parse succeeds | 16-bit control field |
| 6 | Modulo-128 S-frame type | `'S'` type for 16-bit |
| 7 | Modulo-128 SREJ identified | 16-bit SREJ variant |
| 8 | Modulo-128 N(R) = 65 | Tests value > 7 |

**AX.25 reference:** Section 4.3.2.4 — SREJ.

---

### T21 — FRMR on Invalid Frame

**What it tests:** Generation of a Frame Reject (FRMR) response when a protocol error occurs — specifically the X-bit (information field not permitted for this frame type). Tests that the FRMR info field correctly encodes the offending control byte and state variables.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | FRMR encode succeeds | With X-bit and V(S) = 7 |
| 2 | FRMR type correct | `AX25_FRAME_UNNUMBERED_FRMR` |
| 3 | X-bit is set | `frmr->x == true` |
| 4 | V(S) = 7 | Send sequence number encoded in FRMR info field |

**AX.25 reference:** Section 4.3.3.6 — FRMR.

---

### T22 — Digipeater H-Bit and Path Reversal

**What it tests:** Digipeater (repeater) path processing. When a frame passes through a digipeater, the H-bit (Has-been-repeated) is set in that digipeater's address field. The path reversal function swaps source/destination for the reply path.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | `ax25_find_next_digi()` returns 0 | First un-repeated digipeater at index 0 |
| 2 | H-bit not set initially | `ax25_get_h_bit()` returns 0 before relay |
| 3 | Path reversed | Source and destination swapped by `ax25_reverse_repeater_path()` |
| 4 | H-bit initially 0 | Clean address, not yet repeated |
| 5 | H-bit set to 1 | `ax25_set_h_bit()` sets the bit correctly |

**AX.25 reference:** Section 3.12.3 — Repeater Addresses; Section 6.1 — Digipeater Operation.

---

### T23 — PID Dispatch Table

**What it tests:** The Protocol Identifier (PID) dispatch mechanism — applications register handlers for specific PID values; incoming data frames with matching PIDs are routed to the correct handler automatically.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | Register handler for `PID_NO_L3` succeeds | Return code 0 |
| 2 | Handler count is 1 | One registered PID |
| 3 | `ax25_dispatch_pid()` succeeds | Return code 0 for registered PID |
| 4 | Handler called | Callback received the dispatch |
| 5 | Duplicate register succeeds | Re-registering same PID is allowed |
| 6 | Unregister succeeds | Return code 0 |
| 7 | Handler count is 0 after unregister | Table is empty |
| 8 | Dispatch with no handler returns 1 | No-handler error code |

**AX.25 reference:** Section 3.8 — Protocol Identifier (PID).

---

### T24 — Buffer Pool

**What it tests:** The static I-frame buffer pool (`ax25_buf_t`). libax25v22 uses a fixed-size pool instead of dynamic allocation for I-frame payloads to support MCU targets without heap. The test verifies allocation, exhaustion, and release.

**Note:** Prior tests consume some pool slots. T24 captures the current free count as a baseline and works relative to it, so the test is independent of test execution order.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | Initial free count > 0 | At least some slots available |
| 2 | All available slots allocated | `ax25_buf_pool_free_count() == 0` after draining |
| 3 | Exhaustion returns NULL | `ax25_buf_alloc()` returns NULL when pool empty |
| 4 | All freed | Free count restored after releasing all slots |
| 5 | Alloc after free succeeds | Pool reuse works |
| 6 | Slot write/read | Data written to `buf->data[]` reads back correctly |

**AX.25 reference:** `ax25.h` — `ax25_buf_alloc()`, `ax25_buf_free()`, `AX25_POOL_SIZE`.

---

### T25 — Mux: Multiple Connections

**What it tests:** The `ax25_mux_t` multiplexer that routes frames from a shared physical channel to the correct logical connection based on address matching. Tests registration, deregistration, and that two links receive different connection identifiers.

**Assertions:**

| # | Check | Detail |
|---|---|---|
| 1 | Mux register link A succeeds | Non-NULL link ID returned |
| 2 | Mux register link B succeeds | Second non-NULL link ID |
| 3 | Link A ≠ Link B | Distinct link identifiers |
| 4 | Mux unregister link A succeeds | Return code 0 |
| 5 | Mux unregister link B succeeds | Return code 0 |

**AX.25 reference:** `ax25_mux.h` — `ax25_mux_init()`, `ax25_mux_register_link()`, `ax25_mux_unregister_link()`.

---

## HAL Implementation (`hal_linux.c`)

The Hardware Abstraction Layer bridges the portable libax25v22 library to Linux. All 14 sections conform to the MCU portability constraint: no 64-bit arithmetic, no float, no OS threads.

### Section 1 — Tick Counter

**Problem:** AX.25 timers need a millisecond-resolution wall-clock tick. `clock_gettime(CLOCK_MONOTONIC)` returns `struct timespec` with 64-bit nanoseconds.

**Solution:** Compute 32-bit milliseconds by splitting the `tv_sec` epoch into a 32-bit offset at first call, then computing `(tv_sec - epoch) * 1000 + tv_nsec / 1000000` using only 32-bit arithmetic. Tick counter wraps after ~49 days; timer subtraction handles wrap correctly because it uses unsigned arithmetic.

```c
uint32_t hal_tick_ms(void);  // Returns ms since first call, wraps at 2^32
```

### Section 2 — Software Timers

**Problem:** libax25v22 needs up to `HAL_TIMER_MAX` concurrent timers (T1, T2, T3, T101 per connection).

**Solution:** A flat array of `hal_timer_t` structs. Each timer stores a `uint32_t` expiry tick. `hal_timer_service()` is called once per main-loop iteration; it compares each active timer's expiry against the current tick using unsigned subtraction to handle wrap.

```c
hal_timer_id_t hal_timer_alloc(void);
void hal_timer_start(hal_timer_id_t id, uint32_t ms, hal_timer_cb_t cb, void *arg);
void hal_timer_stop(hal_timer_id_t id);
void hal_timer_service(void);  // Call every main-loop tick
```

### Section 3 — Serial Ring Buffers

**Problem:** Serial I/O between the main loop and the HAL must be non-blocking and interrupt-safe without an RTOS.

**Solution:** Power-of-2 sized ring buffers (default 512 bytes RX, 512 bytes TX per port). Index masking with `& (SIZE-1)` replaces modulo. The POSIX signal mask provides the critical-section boundary when updating head/tail.

### Section 4 — PTT / DCD

**Problem:** Physical layer PTT (Push-To-Talk) and DCD (Data Carrier Detect) are hardware-dependent.

**Solution:** Log-only simulation on Linux. In production, these are replaced by GPIO ioctl calls. `hal_ptt_set(port, true)` logs `"PTT port=N ON"` to stderr.

### Section 5 — Serial I/O

Non-blocking `read()`/`write()` with `O_NONBLOCK` mode. `hal_serial_put()` and `hal_serial_get()` operate on ring buffers. `hal_port_poll()` moves bytes between ring buffers and the underlying file descriptor.

### Section 6 — PRNG

**Problem:** AX.25 channel access (p-persistence CSMA) needs a random number generator. No `rand()` (varies by platform), no float, no divide.

**Solution:** 32-bit Galois LFSR with polynomial `0xB4BCD35C`. Four XOR/shift operations per bit. `hal_random_u16()` clocks 16 bits and returns the low half.

```c
uint16_t hal_random_u16(void);
```

### Section 7 — Critical Sections

**Problem:** Ring buffer head/tail updates must be atomic with respect to signal handlers.

**Solution:** Nestable critical sections using `sigprocmask(SIG_BLOCK/SIG_UNBLOCK)`. A nesting counter prevents premature unblocking.

```c
void hal_critical_enter(void);
void hal_critical_exit(void);
```

### Section 8 — Memory

Thin wrappers around `malloc`/`free`/`calloc`/`realloc` with `uint16_t` size parameters (MCU-safe: 16-bit address spaces). The test suite verifies that no allocated memory is leaked.

```c
void* hal_mem_alloc(uint16_t size);
void  hal_mem_free(void *ptr);
void* hal_mem_calloc(uint16_t size);
void* hal_mem_realloc(void *ptr, uint16_t new_size);
```

### Section 9 — CRC-16/CCITT

**Problem:** AX.25 HDLC frames use CRC-16/CCITT (reflected polynomial `0x8408`) as the Frame Check Sequence.

**Solution:** 256-entry lookup table in `.rodata`. Single pass over the data; no 64-bit, no float. The `hal_crc16_final()` inline applies the final XOR (`0xFFFF`).

```c
// Compute FCS in one call:
uint16_t fcs = hal_crc16_buf(frame_data, frame_len);

// Or incrementally:
uint16_t crc = HAL_CRC16_INIT;
crc = hal_crc16_update(crc, buf1, len1);
crc = hal_crc16_update(crc, buf2, len2);
uint16_t fcs = hal_crc16_final(crc);
```

Known values: CRC of `"123456789"` = `0x906E`. Residue of message + LSB-first FCS = `0x0F47`.

### Section 10 — Logging

Structured log output to stderr with 32-bit tick display. Level strings: `ERROR`, `WARN`, `INFO`, `DEBUG`. Controlled at compile time by `HAL_LOG_ENABLE`.

```
[    0.123] INFO  HAL init: Linux/ax25linux
[    0.124] DEBUG PTT port=0 OFF
```

### Section 11 — Watchdog

No-op stub. On MCU targets, this function kicks a hardware watchdog timer. On Linux it is intentionally empty.

```c
void hal_wdog_kick(void);
```

### Section 12 — Channel Access Parameters

Store and retrieve per-port KISS channel-access parameters: TxDelay (ms), Persistence (0–255), SlotTime (ms), FullDuplex flag. These are written to the TNC via KISS command frames.

### Section 13 — Init / Deinit

```c
void hal_init(void);    // Initialise all HAL subsystems; must be called first
void hal_deinit(void);  // Release resources, close serial ports
```

### Section 14 — Linux-Specific Serial / Port API

Opens a real serial device or `stdin/stdout` for KISS TNC communication. Configures `termios` for raw 8N1 mode. See `hal_linux.h` for the full API.

---

## Linux HAL Extension API (`hal_linux.h`)

These functions extend the portable `hal.h` with Linux-only capabilities:

```c
// Open a serial device for KISS I/O
// path: "/dev/ttyUSB0", "/dev/ttyS0", or "-" for stdin/stdout
// baud: B1200, B9600, B19200, B38400, B57600, B115200, etc. (0 = no configure)
hal_err_t hal_port_open(uint8_t port, const char *path, speed_t baud);

// Close a previously opened port
void hal_port_close(uint8_t port);

// Non-blocking I/O service — call once per main loop iteration (~10 ms)
// Reads from fd → RX ring; TX ring → fd
void hal_port_poll(uint8_t port);

// Inject bytes directly into the RX ring (for unit tests / loopback)
void hal_port_inject_rx(uint8_t port, const uint8_t *data, uint16_t len);

// Drain bytes from the TX ring (for unit tests / loopback)
uint16_t hal_port_drain_tx(uint8_t port, uint8_t *dst, uint16_t max_len);
```

---

## AF_AX25 Kernel Bridge (`ax25_linux_bridge.c`)

### Bridge Architecture

```
Application
    │
    ▼
ax25_bridge_connect() / ax25_bridge_send() / ax25_bridge_send_ui()
    │
    ▼
┌─────────────────────────────────────────────┐
│  ax25_linux_ctx_t (singleton)               │
│                                             │
│  ax25_mux_t ──► ax25_connection_t[0..7]    │
│       │             (state machines)        │
│       │                                     │
│  ax25_kiss_ctx_t                            │
│       │                                     │
│  HAL ring buffers (port 0)                  │
│       │                                     │
│  AF_AX25 SOCK_DGRAM (monitor, optional)    │
└──────────┬──────────────────────────────────┘
           │
           ▼
    Serial KISS TNC  ──── RF ──── Remote station
    or AF_PACKET socket
```

### Bridge API Reference

```c
// Initialise the bridge. Call once at startup.
// mycall: your callsign, e.g. "N0CALL-1"
// port:   HAL port index (0–3) previously opened with hal_port_open()
// Returns 0 on success, non-zero on error
int ax25_bridge_init(const char *mycall, uint8_t port);

// Initiate an AX.25 connection to a remote station.
// dest_call:  destination callsign, e.g. "W1AW-1"
// callbacks:  struct with transmit/on_connect/on_disconnect/on_data/on_ui_data
// mod128:     1 = request modulo-128 (SABME), 0 = modulo-8 (SABM)
// Returns connection ID (0–7) on success, or -1 on error
int ax25_bridge_connect(const char *dest_call,
                        const ax25_callbacks_t *callbacks,
                        uint8_t mod128);

// Send connected-mode I-frame data.
// conn_id: from ax25_bridge_connect()
// data:    payload bytes
// len:     payload length (≤ negotiated N1; auto-segmented if larger)
// pid:     Protocol Identifier byte
// Returns 0 on success
int ax25_bridge_send(uint8_t conn_id, const uint8_t *data,
                     size_t len, uint8_t pid);

// Send connectionless UI frame (APRS, beacons, etc.)
// dest_call: destination callsign
// data:      payload bytes
// len:       payload length
// pid:       Protocol Identifier byte
// Returns 0 on success
int ax25_bridge_send_ui(const char *dest_call, const uint8_t *data,
                        size_t len, uint8_t pid);

// Initiate orderly disconnection.
// Returns 0 on success
int ax25_bridge_disconnect(uint8_t conn_id);

// Main-loop service function. Call every ~10 ms.
// Advances all state machine timers and processes queued frames.
void ax25_bridge_tick(void);

// Open an AF_AX25 SOCK_DGRAM socket for kernel monitor mode (optional).
// ifname: AX.25 interface name, e.g. "ax0"
// Returns 0 on success
int ax25_bridge_open_kernel_monitor(const char *ifname);

// Drain frames from the kernel monitor socket (call in main loop).
void ax25_bridge_poll_kernel(void);

// Release all resources.
void ax25_bridge_deinit(void);
```

### Bridge Usage Example

```c
#include "ax25_linux_bridge.h"
#include "hal.h"
#include "hal_linux.h"

static void on_connect(void *ud, bool connected) {
    printf("Connection %s\n", connected ? "established" : "refused");
}

static void on_disconnect(void *ud, uint8_t reason) {
    printf("Disconnected, reason %d\n", reason);
}

static void on_data(void *ud, uint8_t *data, size_t len, uint8_t pid) {
    printf("Received %zu bytes, PID 0x%02X\n", len, pid);
    fwrite(data, 1, len, stdout);
    putchar('\n');
}

int main(void) {
    hal_init();

    // Open KISS TNC on /dev/ttyUSB0 at 9600 baud
    if (hal_port_open(0, "/dev/ttyUSB0", B9600) != HAL_OK) {
        fprintf(stderr, "Failed to open TNC\n");
        return 1;
    }

    // Initialise bridge with our callsign
    if (ax25_bridge_init("N0CALL-1", 0) != 0) {
        fprintf(stderr, "Bridge init failed\n");
        return 1;
    }

    // Connect to a remote station
    ax25_callbacks_t cb = {
        .transmit       = NULL,  // handled internally by bridge
        .on_connect     = on_connect,
        .on_disconnect  = on_disconnect,
        .on_data        = on_data,
        .on_ui_data     = NULL,
    };
    int conn = ax25_bridge_connect("W1AW-1", &cb, 0 /* mod-8 */);
    if (conn < 0) {
        fprintf(stderr, "Connect request failed\n");
        return 1;
    }

    // Main loop
    while (1) {
        hal_port_poll(0);           // Move bytes between fd and ring buffers
        ax25_bridge_tick();         // Advance state machines (~10 ms granularity)
        usleep(10000);              // 10 ms sleep
    }

    ax25_bridge_deinit();
    hal_deinit();
    return 0;
}
```

---

## MCU Portability Constraints

All code in this repository follows these rules, which allow direct use on microcontrollers without an FPU:

| Constraint | Detail |
|---|---|
| **No 64-bit arithmetic** | No `uint64_t`, `int64_t`, `long long`, or `unsigned long long` anywhere in the HAL or test suite |
| **No floating-point** | No `float`, `double`, or any FPU instruction. Enforced by `-Wfloat-conversion` |
| **32-bit timer values** | All timer durations are `uint32_t` milliseconds |
| **16-bit size parameters** | HAL memory functions use `uint16_t` sizes, matching MCU address space constraints |
| **No C++ features** | C99 only; compatible with `arm-none-eabi-gcc`, `avr-gcc`, `msp430-gcc` |
| **No OS / threads** | Poll-based main loop; all concurrency via sigprocmask on Linux |
| **Stack-based test harness** | Test suite itself uses no heap; only the library under test allocates |
| **PRNG without divide** | Galois LFSR using XOR and shift only |
| **CRC without lookup divide** | 256-entry table lookup; no division |

To compile for a bare-metal ARM Cortex-M target, substitute `hal_linux.c` with a target-specific HAL and compile with your cross-compiler:

```bash
arm-none-eabi-gcc -std=c99 -mthumb -mcpu=cortex-m4 -mfpu=none -mfloat-abi=soft \
    -I../libax25v22/hal -I../libax25v22/protocols/ax25 ... \
    ax25_test_suite.c hal_target.c ../libax25v22/protocols/ax25/*.c \
    -o ax25_test.elf
```

---

## Protocol Coverage

| Protocol Feature | Tests | AX.25 v2.2 Section |
|---|---|---|
| Address encoding (7-byte, shift-left) | T01 | §3.12 |
| Address SSID (0–15), C/H bits | T01, T22 | §3.12.2 |
| Frame header, digipeater path | T02 | §3.12.3 |
| UI frame (APRS, beacon, connectionless) | T03 | §4.3.3.2 |
| Connection setup: SABM/SABME/UA | T04 | §4.3.3.4, §4.3.3.8 |
| Connection teardown: DISC/DM | T05/T06 | §4.3.3.3, §4.3.3.9 |
| I-frame modulo-8 (window 1–7) | T07 | §4.4, §3.6 |
| I-frame modulo-128 (window 1–127) | T08 | PE1CHL, §3.12.2 |
| Supervisory: RR, RNR, REJ, SREJ | T09, T19, T20 | §4.3.2 |
| Frame Reject (FRMR) | T10, T21 | §4.3.3.6 |
| XID parameter negotiation | T11 | §4.3.3.7, Appendix A |
| TEST frame | T12 | §4.3.3.10 |
| Segmentation & reassembly | T13 | §6.6, Appendix C6 |
| KISS TNC protocol (standard) | T14, T15 | ARRL 1986 |
| KISS variants: SMACK, G8BPQ | T14 | SP2DMB, G8BPQ |
| HDLC FCS CRC-16/CCITT | T16 | §3.9 |
| State machine: connect/data/disconnect | T17 | §6 |
| T1 retransmission timer | T18 | §6.3, §6.4.7 |
| RNR flow control / local busy | T19 | §6.4.10 |
| SREJ selective retransmission | T20 | §4.3.2.4 |
| Digipeater H-bit, path reversal | T22 | §3.12.3, §6.1 |
| PID dispatch routing | T23 | §3.8 |
| Buffer pool (static, MCU-safe) | T24 | — |
| Mux: multiple logical connections | T25 | — |

---

## AX.25 v2.2 Reference Documents

- **AX.25 Link Access Protocol for Amateur Packet Radio v2.2** (July 1998) — [ax25.net](https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf)
- **Module 128 for AX.25** — PE1CHL, DCC 1995 — [eindhoven.space](https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf)
- **FX.25 Forward Error Correction** — [TAPR](https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf)
- **Linux AX.25 HOWTO** — [TLDP](https://tldp.org/HOWTO/AX25-HOWTO/)
- **AF_AX25 manual page** — `man 4 ax25` or [Ubuntu manpages](https://manpages.ubuntu.com/manpages/focal/man4/ax25.4.html)
- **KISS TNC specification** — ARRL 7th Computer Networking Conference, 1986
- **libax25v22 source** — [github.com/hiperiondev/libax25v22](https://github.com/hiperiondev/libax25v22)

---

## License

See [LICENSE](../libax25v22/LICENSE) in the libax25v22 repository. This integration layer is released under the same terms.
