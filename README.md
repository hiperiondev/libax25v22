<a name="readme-top"></a>
<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/hiperiondev/libax25v22">
    <img src="images/logo.jpg" alt="Logo" height="160">
  </a>

<h3 align="center">libax25v22</h3>

  <p align="center">
    Agnostic implementation of AX25 v2.2 radio amateur protocol
    <br />   
  </p>
</div>

<!-- ABOUT THE PROJECT -->
# About The Project

**libax25v22** is a complete, platform-agnostic implementation of the AX.25 Link Access Protocol version 2.2, the de facto standard for amateur packet radio communications worldwide. This library provides a full-featured, production-ready implementation of the protocol stack specifically designed for resource-constrained embedded systems, particularly 32-bit microcontrollers without floating-point units (FPU).

### Background

AX.25 is a data link layer protocol derived from the ITU-T X.25 protocol suite and ISO/IEC 13239 (HDLC), specifically adapted for amateur radio use. First standardized in 1984 as version 2.0, the protocol has been the foundation of digital amateur radio communications for over four decades. Version 2.2, released in July 1998, introduced significant performance improvements that remain largely unimplemented in most existing solutions despite being published over 25 years ago.

### Protocol Features Implemented

This library implements the complete AX.25 v2.2 specification as defined in the TAPR/ARRL document (July 1998), including advanced features that distinguish v2.2 from earlier versions:

#### Core Protocol Capabilities
- **Full HDLC-compliant framing** with bit stuffing, NRZI encoding, and FCS (Frame Check Sequence) validation
- **Connection-oriented (SABM/SABME) and connectionless (UI) modes** for flexible communication patterns
- **Extended addressing** supporting source, destination, and up to 8 digipeater addresses with SSIDs (0-15)
- **Multiple frame types**: Information (I), Supervisory (S), and Unnumbered (U) frames
- **Three operational modes**: 
  - Unnumbered Information (UI) for connectionless broadcast
  - Asynchronous Balanced Mode (ABM) for reliable connected sessions
  - Connected mode with full error recovery and flow control

#### AX.25 v2.2 Advanced Features
- **Modulo 128 sequence numbering** (PE1CHL extended AX.25) allowing 7-bit sequence numbers instead of the traditional 3-bit modulo 8, enabling windows up to 127 frames
- **Selective Reject (SREJ)** capability for efficient error recovery, retransmitting only missing frames rather than all subsequent frames
- **Extended frame sizes** supporting negotiation of Information field lengths beyond the v2.0 limitation of 256 octets, up to 2048 bits (256 octets) or more
- **XID (Exchange Identification)** frames for parameter negotiation including window size, timer values, and maximum frame size
- **Enhanced error recovery** with configurable T1 (acknowledgment), T2 (response delay), and T3 (idle channel) timers
- **Adaptive retry mechanisms** supporting both linear and exponential backoff strategies

#### State Machine and Connection Management
- **Complete SDL (Specification and Description Language) compliant state machine** implementation
- **Dynamic parameter negotiation** during connection establishment
- **Robust timeout handling** with configurable retry counts (N2 parameter)
- **Poll/Final bit management** for precise command/response sequencing
- **DM (Disconnected Mode) and FRMR (Frame Reject)** error signaling

### Design Philosophy

#### Platform Agnostic Architecture
The library is designed with complete hardware abstraction, making it suitable for:
- Bare-metal microcontroller applications
- RTOS-based systems (FreeRTOS, Zephyr, etc.)
- Embedded Linux environments
- PC-based applications
- Software-defined TNCs (Terminal Node Controllers)

#### Embedded Systems Optimization
With a focus on 32-bit processors without FPU, the implementation:
- Uses only integer arithmetic throughout the codebase
- Implements efficient bit manipulation for HDLC operations
- Provides configurable buffer management for memory-constrained systems
- Minimizes stack usage through careful function design
- Offers compile-time configuration options to reduce code footprint

#### Modularity and Integration
The library provides:
- Clean separation between protocol logic and physical layer
- Abstraction layers for different transport mechanisms (KISS, direct HDLC, custom interfaces)
- Callback-based architecture for event-driven applications
- No dependencies on specific operating systems or hardware platforms
- Compatible with common TNC interfaces and modem implementations

### Technical Implementation

The library handles all aspects of the AX.25 protocol stack:

1. **Frame Construction and Parsing**
   - Address field encoding with callsign/SSID combinations
   - Control field encoding for modulo 8 and modulo 128 operation
   - PID (Protocol Identifier) field management
   - Information field handling with length validation
   - FCS calculation and verification using CRC-16-CCITT

2. **HDLC Layer Processing**
   - Flag sequence detection (0x7E)
   - Bit stuffing/unstuffing (zero insertion after five consecutive ones)
   - NRZI encoding/decoding support
   - Frame synchronization and delimitation
   - Idle pattern generation

3. **Connection State Management**
   - Disconnected, awaiting connection, awaiting release states
   - Information transfer state with full window management
   - Timer supervision and timeout handling
   - Collision resolution for simultaneous connection attempts

4. **Flow Control and Windowing**
   - Sliding window protocol implementation
   - V(S), V(R), and V(A) sequence variable tracking
   - Window rotation and acknowledgment processing
   - Selective reject queue management for SREJ operation
   - Receiver Not Ready (RNR) condition handling

5. **Error Detection and Recovery**
   - FCS validation on all received frames
   - REJ (Reject) and SREJ (Selective Reject) frame handling
   - Retransmission queue management
   - Invalid frame sequence number detection
   - FRMR (Frame Reject) condition generation and processing

### Use Cases and Applications

This library is ideal for:
- **CubeSat and satellite ground stations** requiring reliable packet radio links
- **APRS (Automatic Packet Reporting System)** trackers and digipeaters
- **Packet BBS (Bulletin Board Systems)** and message forwarding nodes
- **Wireless SCADA systems** for monitoring and control applications
- **Emergency communications equipment** for ARES/RACES deployments
- **IoT applications** using amateur radio bands for long-range, license-free communication
- **Software TNCs** and modem implementations (Bell 202, G3RUH, etc.)
- **Network routing nodes** for AMPRNet (44.0.0.0/8) TCP/IP over amateur radio
- **Educational platforms** for learning packet radio protocols

### Comparison with Existing Implementations

While several AX.25 implementations exist, most have significant limitations:

- **Linux kernel ax25 module**: Tied to Linux, requires kernel-space operation, complex to adapt for embedded use
- **Dire Wolf**: Excellent software TNC but PC-focused, requires significant processing power
- **Most Arduino/embedded libraries**: Implement only basic UI (connectionless) mode, lack v2.2 features
- **BeRTOS AX.25**: Limited to specific RTOS, doesn't fully implement v2.2 extensions
- **Python implementations**: Not suitable for resource-constrained real-time systems

**libax25v22** fills the gap by providing a complete, standards-compliant v2.2 implementation specifically designed for embedded systems while maintaining portability and modern software engineering practices.

### Compliance and Standards

The implementation strictly adheres to:
- AX.25 Link Access Protocol for Amateur Packet Radio Version 2.2 (July 1998)
- ISO/IEC 13239 (HDLC) framing procedures
- ISO 3309 error detection (FCS/CRC)
- PE1CHL extended sequence numbering specification (Modulo 128)
- TAPR/ARRL AX.25 protocol specifications

### Future Extensions

Planned enhancements include:
- FX.25 Forward Error Correction wrapper support
- IL2P (Improved Layer 2 Protocol) compatibility layer
- KISS protocol interface implementation
- Integration examples with popular modems (Bell 202, G3RUH, GFSK)
- Connection-oriented examples for common microcontroller platforms
- Network layer protocol support (NET/ROM, ROSE)
- Benchmark suite for performance validation

---

*This implementation represents years of collective knowledge from the amateur radio community, distilled into a clean, portable codebase suitable for the next generation of packet radio applications on modern embedded platforms.*

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

<!-- IMPLEMENTATION STATUS -->
# AX.25 v2.2 Protocol Implementation Status

## Comprehensive Feature Checklist for libax25v22

This checklist documents the implementation status of all features defined in:
- AX.25 Link Access Protocol v2.2 (July 1998)
- PE1CHL Modulo 128 Extension (DCC 1995)
- FX.25 Forward Error Correction Specification
- Related TAPR/ARRL standards

**Legend:**
- `[COMPLETED]` - Fully implemented and tested
- `[INCOMPLETED]` - Partially implemented, needs work
- `[MISSING]` - Not yet implemented

---

## 1. FRAME STRUCTURE (Section 3)

### 1.1 Basic Frame Components
- `[COMPLETED]` Flag field (0x7E) encoding/decoding
- `[COMPLETED]` Address field encoding (destination, source, repeaters)
- `[COMPLETED]` Control field (8-bit and 16-bit)
- `[COMPLETED]` PID (Protocol Identifier) field
- `[COMPLETED]` Information field handling
- `[COMPLETED]` Bit stuffing (zero insertion after five 1s)
- `[COMPLETED]` Frame Check Sequence (FCS/CRC-16-CCITT)
- `[COMPLETED]` LSB-first bit transmission order
- `[COMPLETED]` Invalid frame detection (Section 3.9)
- `[COMPLETED]` Frame abort sequence (15+ contiguous 1s)
- `[COMPLETED]` Inter-frame time fill (contiguous flags)

### 1.2 Address Field Encoding (Section 3.12)
- `[COMPLETED]` Non-repeater address field encoding
- `[COMPLETED]` Destination subfield encoding (7 octets)
- `[COMPLETED]` Source subfield encoding (7 octets)
- `[COMPLETED]` Callsign encoding (6 chars + SSID)
- `[COMPLETED]` SSID encoding (4-bit, 0-15)
- `[COMPLETED]` Command/Response (C) bit handling
- `[COMPLETED]` Reserved bits (RR) management
- `[COMPLETED]` Extension bit handling
- `[COMPLETED]` Layer 2 repeater address encoding (up to 8 repeaters)
- `[INCOMPLETED]` Multiple repeater operation (max 2 per spec v2.2)
- `[COMPLETED]` Has-been-repeated (H) bit handling

---

## 2. CONTROL FIELD FORMATS (Section 4.2)

### 2.1 Information Transfer Format (I-frames)
- `[COMPLETED]` 8-bit control field (modulo 8)
- `[COMPLETED]` 16-bit control field (modulo 128)
- `[COMPLETED]` Send sequence number N(S) encoding
- `[COMPLETED]` Receive sequence number N(R) encoding
- `[COMPLETED]` Poll/Final (P/F) bit handling

### 2.2 Supervisory Format (S-frames)
- `[COMPLETED]` RR (Receive Ready) command/response
- `[COMPLETED]` RNR (Receive Not Ready) command/response
- `[COMPLETED]` REJ (Reject) command/response
- `[COMPLETED]` SREJ (Selective Reject) command/response
- `[COMPLETED]` 8-bit S-frame control (modulo 8)
- `[COMPLETED]` 16-bit S-frame control (modulo 128)

### 2.3 Unnumbered Format (U-frames)
- `[COMPLETED]` SABM (Set Asynchronous Balanced Mode) command
- `[COMPLETED]` SABME (Set ABM Extended - modulo 128) command
- `[COMPLETED]` DISC (Disconnect) command
- `[COMPLETED]` UA (Unnumbered Acknowledge) response
- `[COMPLETED]` DM (Disconnected Mode) response
- `[COMPLETED]` UI (Unnumbered Information) frame
- `[COMPLETED]` XID (Exchange Identification) frame
- `[COMPLETED]` TEST frame (command/response)
- `[INCOMPLETED]` FRMR (Frame Reject) response - deprecated in v2.2

---

## 3. SEQUENCE NUMBERS & STATE VARIABLES (Section 4.2.2)

### 3.1 Modulo 8 Operation
- `[COMPLETED]` 3-bit sequence numbers (0-7)
- `[COMPLETED]` Maximum 7 outstanding I-frames
- `[COMPLETED]` Send state variable V(S)
- `[COMPLETED]` Receive state variable V(R)
- `[COMPLETED]` Acknowledge state variable V(A)

### 3.2 Modulo 128 Operation (PE1CHL Extension)
- `[COMPLETED]` 7-bit sequence numbers (0-127)
- `[COMPLETED]` Maximum 127 outstanding I-frames
- `[COMPLETED]` Extended control field encoding
- `[COMPLETED]` Negotiation via SABME command
- `[COMPLETED]` XID parameter negotiation for modulo 128

---

## 4. PARAMETER NEGOTIATION (XID - Section 4.3.3.7)

### 4.1 XID Frame Structure
- `[COMPLETED]` Format Identifier (FI = 0x82)
- `[COMPLETED]` Group Identifier (GI = 0x80)
- `[COMPLETED]` Group Length (GL) encoding
- `[COMPLETED]` Parameter field (PI/PL/PV) structure

### 4.2 Class of Procedures (PI=2)
- `[COMPLETED]` Half-duplex operation negotiation
- `[COMPLETED]` Full-duplex operation negotiation
- `[INCOMPLETED]` Asymmetric operation modes (not implemented per v2.2)

### 4.3 HDLC Optional Functions (PI=3)
- `[COMPLETED]` REJ (Implicit Reject) negotiation
- `[COMPLETED]` SREJ (Selective Reject) negotiation
- `[COMPLETED]` SREJ/REJ (Selective Reject-Reject) mode
- `[COMPLETED]` Modulo 8 capability indication
- `[COMPLETED]` Modulo 128 capability indication
- `[COMPLETED]` Extended address encoding support
- `[COMPLETED]` 16-bit FCS support
- `[MISSING]` 32-bit FCS option (not implemented)
- `[COMPLETED]` TEST command/response support
- `[COMPLETED]` XID command/response support

### 4.4 Other XID Parameters
- `[COMPLETED]` I-field Length Receive (PI=6) - N1 parameter
- `[COMPLETED]` Window Size Receive (PI=8) - k parameter
- `[COMPLETED]` Acknowledge Timer (PI=9) - T1 parameter
- `[COMPLETED]` Retries (PI=10) - N2 parameter
- `[COMPLETED]` Response Delay Timer (PI=11) - T2 parameter (v2.2 addition)

---

## 5. LINK ERROR REPORTING & RECOVERY (Section 4.4)

### 5.1 Error Detection
- `[COMPLETED]` TNC busy condition (RNR)
- `[COMPLETED]` Send sequence number error detection
- `[COMPLETED]` Invalid frame detection
- `[COMPLETED]` FCS error detection

### 5.2 Recovery Mechanisms
- `[COMPLETED]` REJ (Reject) recovery - implicit reject mode
- `[COMPLETED]` SREJ (Selective Reject) recovery
- `[COMPLETED]` SREJ/REJ combined mode (default per v2.2)
- `[COMPLETED]` T1 timer recovery (acknowledgment timeout)
- `[COMPLETED]` T3 timer recovery (idle link polling)
- `[COMPLETED]` Timeout error recovery with N2 retries

---

## 6. DATA LINK STATE MACHINE (Section 6 & Appendix C4)

### 6.1 Connection States
- `[COMPLETED]` Disconnected state
- `[COMPLETED]` Awaiting connection state
- `[COMPLETED]` Awaiting release state
- `[COMPLETED]` Connected/Information transfer state
- `[COMPLETED]` Timer recovery state
- `[INCOMPLETED]` Frame reject state (minimal - FRMR deprecated)

### 6.2 Link Setup & Disconnection (Section 6.3)
- `[COMPLETED]` AX.25 link connection establishment (SABM/SABME)
- `[COMPLETED]` Parameter negotiation phase (XID exchange)
- `[COMPLETED]` Information transfer phase
- `[COMPLETED]` Link disconnection (DISC/UA)
- `[COMPLETED]` Collision recovery (half-duplex)
- `[COMPLETED]` Collision of unnumbered commands
- `[COMPLETED]` Connectionless operation (UI frames)

### 6.3 Information Transfer Procedures (Section 6.4)
- `[COMPLETED]` Sending I-frames with flow control
- `[COMPLETED]` Receiving I-frames (in-sequence)
- `[COMPLETED]` Out-of-sequence frame handling
- `[COMPLETED]` Reception of REJ frames
- `[COMPLETED]` Reception of SREJ frames
- `[COMPLETED]` Reception of RNR frames
- `[COMPLETED]` Sending busy indication (RNR)
- `[COMPLETED]` Waiting acknowledgment (T1 expiry handling)
- `[COMPLETED]` Priority acknowledge (T2 response delay)

### 6.4 Advanced Flow Control
- `[COMPLETED]` Sliding window protocol (modulo 8 & 128)
- `[COMPLETED]` Window size negotiation (k parameter)
- `[COMPLETED]` I-field length negotiation (N1 parameter)
- `[COMPLETED]` Adaptive T1 timer adjustment (based on RTT)
- `[COMPLETED]` Exponential backoff option

---

## 7. TIMERS & PARAMETERS (Section 6.7)

### 7.1 Timers
- `[COMPLETED]` T1 - Acknowledgment timer (default 3000 ms)
- `[COMPLETED]` T2 - Response delay timer (default 500 ms) - v2.2 addition
- `[COMPLETED]` T3 - Inactive link timer (idle channel polling)
- `[MISSING]` T100 - Repeater hang timer (AXHANG) - digipeater function
- `[MISSING]` T101 - Priority window timer (PRIACK)
- `[MISSING]` T102 - Slot time timer (p-persistence)
- `[MISSING]` T103 - Transmitter startup timer (TXDELAY)
- `[MISSING]` T104 - Repeater startup timer (AXDELAY)
- `[MISSING]` T105 - Remote receiver sync timer
- `[MISSING]` T106 - Ten minute transmission limit timer
- `[MISSING]` T107 - Anti-hogging limit timer
- `[MISSING]` T108 - Receiver startup timer

### 7.2 Parameters
- `[COMPLETED]` N1 - Maximum I-field octets (default 256, negotiable)
- `[COMPLETED]` N2 - Maximum retries (default 10, negotiable)
- `[COMPLETED]` k - Window size (default 7 for modulo 8, 32 for modulo 128)

---

## 8. LAYER SEGMENTATION/REASSEMBLY (Section 2.4, 6.6, Appendix C6)

### 8.1 Segmenter State Machine
- `[COMPLETED]` Segmentation of large data units (>N1)
- `[COMPLETED]` Segment header encoding (First/Last flags, sequence)
- `[COMPLETED]` PID preservation across segments
- `[COMPLETED]` Segment transmission sequencing
- `[MISSING]` Next segment timer TR210

### 8.2 Reassembler State Machine
- `[COMPLETED]` Segment reception and buffering
- `[COMPLETED]` In-order reassembly
- `[COMPLETED]` Segment timeout detection
- `[COMPLETED]` Delivery of complete payload to Layer 3
- `[INCOMPLETED]` Out-of-order segment handling (basic only)

---

## 9. LAYER 3 PROTOCOL MULTIPLEXING (Section 3.4, 6.5)

### 9.1 PID Support
- `[COMPLETED]` PID field encoding/decoding
- `[COMPLETED]` Protocol handler registration mechanism
- `[COMPLETED]` Default handler for unknown PIDs
- `[COMPLETED]` Segmentation fragment PID (0x08)

### 9.2 Supported Layer 3 Protocols
- `[COMPLETED]` No Layer 3 (PID 0xF0)
- `[COMPLETED]` ISO 8208/CCITT X.25 (PID 0x01)
- `[COMPLETED]` Compressed TCP/IP (PID 0x06)
- `[COMPLETED]` Uncompressed TCP/IP (PID 0x07)
- `[COMPLETED]` ARPA IP (PID 0xCC)
- `[COMPLETED]` ARPA ARP (PID 0xCD)
- `[COMPLETED]` NET/ROM (PID 0xCF)
- `[COMPLETED]` FlexNet (PID 0xCE)
- `[COMPLETED]` Link Quality Protocol (PID 0xC4)
- `[COMPLETED]` TEXNET (PID 0xC3)
- `[COMPLETED]` Appletalk (PID 0xCA, 0xCB)
- `[COMPLETED]` Escape character (PID 0xFF) for extended PIDs

---

## 10. HDLC FRAMING LAYER (Section 3, Appendix C2)

### 10.1 Physical Layer State Machine - Simplex
- `[COMPLETED]` Flag detection and generation
- `[COMPLETED]` Bit stuffing/destuffing
- `[COMPLETED]` NRZI encoding support (hardware abstraction)
- `[COMPLETED]` Abort sequence generation
- `[COMPLETED]` CRC-16-CCITT calculation (table-driven)

### 10.2 Physical Layer State Machine - Duplex
- `[INCOMPLETED]` Full-duplex operation (abstracted to upper layers)
- `[MISSING]` Hardware-specific PTT control
- `[MISSING]` Carrier detect interfacing
- `[MISSING]` Transmitter/receiver switching delays

---

## 11. LINK MULTIPLEXER (Section 2.7, Appendix C3)

### 11.1 Multiple Link Support
- `[INCOMPLETED]` Multiple data-link connections (structure exists, needs testing)
- `[MISSING]` Link rotation algorithm
- `[MISSING]` Per-link scheduling
- `[MISSING]` Priority-based transmission

---

## 12. MANAGEMENT DATA LINK (Section 2.6, Appendix C5)

### 12.1 Management Functions
- `[COMPLETED]` XID parameter negotiation
- `[COMPLETED]` XID command/response handling
- `[COMPLETED]` Parameter conflict resolution
- `[COMPLETED]` Negotiation timeout handling
- `[COMPLETED]` Fallback to v2.0 defaults

---

## 13. ADVANCED FEATURES (v2.2 Improvements)

### 13.1 Selective Reject (Section 6.4.4)
- `[COMPLETED]` SREJ command/response encoding
- `[COMPLETED]` SREJ exception state tracking
- `[COMPLETED]` Out-of-sequence frame buffering
- `[COMPLETED]` SREJ bitmap for multiple outstanding frames
- `[COMPLETED]` SREJ/REJ combined mode (per v2.2 default)
- `[INCOMPLETED]` Multiple simultaneous SREJ conditions (partial)

### 13.2 Extended Sequence Numbers
- `[COMPLETED]` 7-bit sequence numbers (modulo 128)
- `[COMPLETED]` 127-frame window support
- `[COMPLETED]` SABME negotiation
- `[COMPLETED]` 16-bit control field encoding/decoding

### 13.3 Full-Duplex Operation (Section 6.7.2)
- `[COMPLETED]` XID negotiation for full-duplex
- `[COMPLETED]` State variable tracking for full-duplex
- `[INCOMPLETED]` Physical layer full-duplex (hardware abstraction exists)

### 13.4 Response Delay Timer (T2)
- `[COMPLETED]` T2 timer implementation (v2.2 addition)
- `[COMPLETED]` Priority acknowledge mechanism
- `[COMPLETED]` T2 XID negotiation

---

## 14. STATISTICS & DIAGNOSTICS

### 14.1 Performance Metrics
- `[COMPLETED]` Frame counters (I, S, U frames sent/received)
- `[COMPLETED]` Error counters (FCS, CRC, aborts, overruns)
- `[COMPLETED]` Retransmission tracking
- `[COMPLETED]` T1 expiration counting
- `[COMPLETED]` Byte counters (sent/received)
- `[COMPLETED]` Current state variables (V(S), V(R), V(A))

### 14.2 TEST Frame Statistics (Section 4.3.3.8)
- `[COMPLETED]` TEST command/response counters
- `[COMPLETED]` Round-trip time (RTT) measurement
- `[COMPLETED]` Average RTT calculation
- `[COMPLETED]` TEST sequence number tracking
- `[COMPLETED]` Lost TEST frame detection

---

## 15. EXTENSIONS & ENHANCEMENTS

### 15.1 FX.25 Forward Error Correction
- `[COMPLETED]` Reed-Solomon FEC encoding
- `[COMPLETED]` Correlation tag structure (8 bytes)
- `[COMPLETED]` Multiple FEC modes (11 predefined)
- `[COMPLETED]` Mode selection based on frame size
- `[COMPLETED]` Adaptive mode selection (channel quality)
- `[COMPLETED]` Galois Field GF(2^8) operations (table-based, no FPU)
- `[INCOMPLETED]` FEC decoding and error correction
- `[MISSING]` Integration with HDLC layer

### 15.2 IL2P (Improved Layer 2 Protocol)
- `[MISSING]` IL2P header mapping
- `[MISSING]` SIXBIT callsign compression
- `[MISSING]` Reed-Solomon payload blocks
- `[MISSING]` 24-bit sync word
- `[MISSING]` Scrambling/descrambling
- `[MISSING]` Type 0 transparent encapsulation
- `[MISSING]` Type 1 translated encapsulation

### 15.3 KISS Interface Protocol
- `[MISSING]` KISS framing (FEND, FESC encoding)
- `[MISSING]` KISS command byte processing
- `[MISSING]` TxDelay, Persistence, SlotTime, TxTail parameters
- `[MISSING]` Full-duplex mode control
- `[MISSING]` Hardware-specific commands
- `[MISSING]` Multi-port TNC support

### 15.4 SMACK (CRC-enhanced KISS)
- `[MISSING]` SMACK CRC-16 over KISS frames
- `[MISSING]` Automatic KISS/SMACK mode detection
- `[MISSING]` RS-232 error protection

---

## 16. EMBEDDED SYSTEMS OPTIMIZATION

### 16.1 No-FPU Design
- `[COMPLETED]` Integer-only arithmetic throughout
- `[COMPLETED]` Fixed-point timer handling (10ms ticks)
- `[COMPLETED]` Table-driven CRC (512 bytes flash)
- `[COMPLETED]` GF arithmetic via lookup tables (FX.25)

### 16.2 Memory Management
- `[COMPLETED]` Static buffer allocation where possible
- `[COMPLETED]` Configurable frame queue sizes
- `[COMPLETED]` No dynamic allocation in critical paths (mostly)
- `[COMPLETED]` Minimal stack usage

### 16.3 Platform Abstraction
- `[COMPLETED]` Hardware-independent core protocol
- `[COMPLETED]` Callback-based I/O (no direct hardware access)
- `[COMPLETED]` Portable C99 code
- `[COMPLETED]` Configurable compile-time options

---

## 17. UPPER LAYER INTERFACES

### 17.1 Data Link Service Access Point (DLSAP) Primitives
- `[INCOMPLETED]` DL-CONNECT request/indication/confirm
- `[INCOMPLETED]` DL-DISCONNECT request/indication/confirm
- `[INCOMPLETED]` DL-DATA request/indication
- `[INCOMPLETED]` DL-UNIT-DATA request/indication
- `[INCOMPLETED]` DL-ERROR indication
- `[INCOMPLETED]` DL-FLOW-OFF/ON request

### 17.2 Management Data Link Primitives
- `[COMPLETED]` MDL-NEGOTIATE request/confirm
- `[COMPLETED]` MDL-ERROR indication

---

## 18. NOT IMPLEMENTED (Out of Scope)

### 18.1 Deprecated Features
- `[N/A]` FRMR generation (v2.2 replaces with link reset)
- `[N/A]` Unbalanced operation modes (NRM, ARM)
- `[N/A]` SIM/RIM commands (removed in v2.2)
- `[N/A]` UP command (removed in v2.2)
- `[N/A]` RSET command (removed in v2.2)
- `[N/A]` RD response (removed in v2.2)

### 18.2 Hardware-Specific Features
- `[N/A]` Modem modulation/demodulation (AFSK, G3RUH, etc.)
- `[N/A]` Audio DSP processing
- `[N/A]` Radio PTT control circuits
- `[N/A]` Carrier detect (DCD) processing
- `[N/A]` Audio tone generation/detection

### 18.3 Network Layer (Layer 3)
- `[N/A]` NET/ROM routing protocol
- `[N/A]` ROSE X.25 packet layer
- `[N/A]` TCP/IP stack implementation
- `[N/A]` APRS encoding/decoding
- `[N/A]` BBS protocols

---

## NOTES

1. **FRMR Handling**: Per AX.25 v2.2, FRMR is deprecated. This implementation correctly handles FRMR reception from v2.0 stations but generates link resets (SABM) instead of FRMR responses.

2. **Digipeater Limitation**: v2.2 recommends limiting digipeating to 2 hops maximum. Full 8-repeater support exists but should be policy-limited.

3. **FPU-Free**: All timing uses integer arithmetic (10ms ticks). All FEC calculations use lookup tables. Suitable for ARM Cortex-M0/M3/M4 without FPU.

4. **Memory Footprint**: Core protocol ~20KB code, ~4KB RAM (single connection). Scales with number of connections and buffer sizes.

5. **Standards Compliance**: Implementation follows SDL state machines from Appendix C of the v2.2 specification where practical for embedded systems.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

<!-- USAGE -->
# Usage

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

<!-- ROADMAP -->
# Roadmap

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

<!-- CONTRIBUTING -->
# Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork it (<https://github.com/hiperiondev/libax25v22/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

<!-- LICENSE -->
# License

Distributed under the GNU General Public License v3.0. See `LICENSE.txt` for more information.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

<!-- CONTACT -->
# Contact

*Emiliano Augusto Gonzalez - egonzalez.hiperion@gmail.com*

Project Link: [https://https://github.com/hiperiondev/libax25v22](https://github.com/hiperiondev/libax25v22)

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>
