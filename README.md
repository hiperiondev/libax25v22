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
- FX.25 Forward Error Correction wrapper support

### Future Extensions

Planned enhancements include:
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
- ✅ **COMPLETED** - Fully implemented and tested
- ⚠️ **INCOMPLETED** - Partially implemented, needs work or has limitations
- ❌ **MISSING** - Not yet implemented
- ⚪ **N/A** - Out of scope (deprecated or hardware-specific)

---

## 1. FRAME STRUCTURE (Section 3)

### 1.1 Basic Frame Components
- ✅ Flag field (0x7E) encoding/decoding
- ✅ Address field encoding (destination, source, repeaters)
- ✅ Control field (8-bit and 16-bit)
- ✅ PID (Protocol Identifier) field
- ✅ Information field handling
- ✅ Bit stuffing (zero insertion after five 1s)
- ✅ Frame Check Sequence (FCS/CRC-16-CCITT)
- ✅ LSB-first bit transmission order
- ✅ Invalid frame detection (Section 3.9)
- ✅ Frame abort sequence (15+ contiguous 1s)
- ✅ Inter-frame time fill (contiguous flags)

### 1.2 Address Field Encoding (Section 3.12)
- ✅ Non-repeater address field encoding
- ✅ Destination subfield encoding (7 octets)
- ✅ Source subfield encoding (7 octets)
- ✅ Callsign encoding (6 chars + SSID)
- ✅ SSID encoding (4-bit, 0-15)
- ✅ Command/Response (C) bit handling
- ✅ Reserved bits (RR) management
- ✅ Extension bit handling
- ✅ Layer 2 repeater address encoding (up to 8 repeaters)
- ✅ Multiple repeater operation (max 2 per spec v2.2 1998 but extended to 8 in this case)
- ✅ Has-been-repeated (H) bit handling

---

## 2. CONTROL FIELD FORMATS (Section 4.2)

### 2.1 Information Transfer Format (I-frames)
- ✅ 8-bit control field (modulo 8)
- ✅ 16-bit control field (modulo 128)
- ✅ Send sequence number N(S) encoding
- ✅ Receive sequence number N(R) encoding
- ✅ Poll/Final (P/F) bit handling

### 2.2 Supervisory Format (S-frames)
- ✅ RR (Receive Ready) command/response
- ✅ RNR (Receive Not Ready) command/response
- ✅ REJ (Reject) command/response
- ✅ SREJ (Selective Reject) command/response
- ✅ 8-bit S-frame control (modulo 8)
- ✅ 16-bit S-frame control (modulo 128)

### 2.3 Unnumbered Format (U-frames)
- ✅ SABM (Set Asynchronous Balanced Mode) command
- ✅ SABME (Set ABM Extended - modulo 128) command
- ✅ DISC (Disconnect) command
- ✅ UA (Unnumbered Acknowledge) response
- ✅ DM (Disconnected Mode) response
- ✅ UI (Unnumbered Information) frame
- ✅ XID (Exchange Identification) frame
- ✅ TEST frame (command/response)
- ✅ FRMR (Frame Reject) response - Implemented for backward compatibility despite v2.2 deprecation

---

## 3. SEQUENCE NUMBERS & STATE VARIABLES (Section 4.2.2)

### 3.1 Modulo 8 Operation
- ✅ 3-bit sequence numbers (0-7)
- ✅ Maximum 7 outstanding I-frames
- ✅ Send state variable V(S)
- ✅ Receive state variable V(R)
- ✅ Acknowledge state variable V(A)

### 3.2 Modulo 128 Operation (PE1CHL Extension)
- ✅ 7-bit sequence numbers (0-127)
- ✅ Maximum 127 outstanding I-frames
- ✅ Extended control field encoding/decoding
- ✅ Negotiation via SABME command
- ✅ XID parameter negotiation for modulo 128

---

## 4. PARAMETER NEGOTIATION (XID - Section 4.3.3.7)

### 4.1 XID Frame Structure
- ✅ Format Identifier (FI = 0x82)
- ✅ Group Identifier (GI = 0x80)
- ✅ Group Length (GL) encoding
- ✅ Parameter field (PI/PL/PV) structure

### 4.2 Class of Procedures (PI=2)
- ✅ Half-duplex operation negotiation
- ✅ Full-duplex operation negotiation
- ⚠️ Asymmetric operation modes (not implemented per v2.2)

### 4.3 HDLC Optional Functions (PI=3)
- ✅ REJ (Implicit Reject) negotiation
- ✅ SREJ (Selective Reject) negotiation
- ✅ SREJ/REJ (Selective Reject-Reject) mode
- ✅ Modulo 8 capability indication
- ✅ Modulo 128 capability indication
- ✅ Extended address encoding support
- ✅ 16-bit FCS support
- ❌ 32-bit FCS option (Version 2.2 Revision(July 1998) defines only 16-bit FCS)
- ✅ TEST command/response support
- ✅ XID command/response support

### 4.4 Other XID Parameters
- ✅ I-field Length Receive (PI=6) - N1 parameter (max 256 octets, not "or more")
- ✅ Window Size Receive (PI=8) - k parameter
- ✅ Acknowledge Timer (PI=9) - T1 parameter
- ✅ Retries (PI=10) - N2 parameter
- ✅ Response Delay Timer (PI=11) - T2 parameter (v2.2 addition)

---

## 5. LINK ERROR REPORTING & RECOVERY (Section 4.4)

### 5.1 Error Detection
- ✅ TNC busy condition (RNR)
- ✅ Send sequence number error detection
- ✅ Invalid frame detection
- ✅ FCS error detection

### 5.2 Recovery Mechanisms
- ✅ REJ (Reject) recovery - implicit reject mode
- ✅ SREJ (Selective Reject) recovery
- ✅ SREJ/REJ combined mode (default per v2.2)
- ✅ T1 timer recovery (acknowledgment timeout)
- ✅ T3 timer recovery (idle link polling)
- ✅ Timeout error recovery with N2 retries

---

## 6. DATA LINK STATE MACHINE (Section 6 & Appendix C4)

### 6.1 Connection States
- ✅ Disconnected state
- ✅ Awaiting connection state
- ✅ Awaiting release state
- ✅ Connected/Information transfer state
- ✅ Timer recovery state
- ✅ Frame reject state - Full FRMR state tracking implemented

### 6.2 Link Setup & Disconnection (Section 6.3)
- ✅ AX.25 link connection establishment (SABM/SABME)
- ✅ Parameter negotiation phase (XID exchange)
- ✅ Information transfer phase
- ✅ Link disconnection (DISC/UA)
- ✅ Collision recovery (half-duplex)
- ✅ Collision of unnumbered commands
- ✅ Connectionless operation (UI frames)

### 6.3 Information Transfer Procedures (Section 6.4)
- ✅ Sending I-frames with flow control
- ✅ Receiving I-frames (in-sequence)
- ✅ Out-of-sequence frame handling
- ✅ Reception of REJ frames
- ✅ Reception of SREJ frames
- ✅ Reception of RNR frames
- ✅ Sending busy indication (RNR)
- ✅ Waiting acknowledgment (T1 expiry handling)
- ✅ Priority acknowledge (T2 response delay)

### 6.4 Advanced Flow Control
- ✅ Sliding window protocol (modulo 8 & 128) - Framework present
- ✅ Window size negotiation (k parameter)
- ✅ I-field length negotiation (N1 parameter)
- ✅ Adaptive T1 timer adjustment (based on RTT)
- ✅ Exponential backoff option

---

## 7. TIMERS & PARAMETERS (Section 6.7)

### 7.1 Timers
- ✅ T1 - Acknowledgment timer (default 3000 ms)
- ✅ T2 - Response delay timer (default 500 ms) - v2.2 addition
- ✅ T3 - Inactive link timer (idle channel polling)
- ✅ T100 - Repeater hang timer (AXHANG) - digipeater function
- ✅ T101 - Priority window timer (PRIACK)
- ✅ T102 - Slot time timer (p-persistence)
- ✅ T103 - Transmitter startup timer (TXDELAY)
- ✅ T104 - Repeater startup timer (AXDELAY)
- ✅ T105 - Remote receiver sync timer
- ✅ T106 - Ten minute transmission limit timer
- ✅ T107 - Anti-hogging limit timer
- ✅ T108 - Receiver startup timer

### 7.2 Parameters
- ✅ N1 - Maximum I-field octets (default 256, negotiable, hard limit 256 - NOT "or more")
- ✅ N2 - Maximum retries (default 10, negotiable)
- ✅ k - Window size (default 7 for modulo 8, 32 for modulo 128)

---

## 8. LAYER SEGMENTATION/REASSEMBLY (Section 2.4, 6.6, Appendix C6)

### 8.1 Segmenter State Machine
- ✅ Segmentation of large data units (>N1)
- ✅ Segment header encoding (First/Last flags, sequence)
- ✅ PID preservation across segments
- ✅ Segment transmission sequencing
- ✅ Next segment timer TR210

### 8.2 Reassembler State Machine
- ✅ Segment reception and buffering
- ✅ In-order reassembly
- ✅ Segment timeout detection
- ✅ Delivery of complete payload to Layer 3
- ✅ Out-of-order segment handling
- ⚠️ **LIMITATION:** Reassembly functions exist but integration with state machine not automatic; application layer must call reassembly functions

---

## 9. LAYER 3 PROTOCOL MULTIPLEXING (Section 3.4, 6.5)

### 9.1 PID Support
- ✅ PID field encoding/decoding
- ✅ Protocol handler registration mechanism
- ✅ Default handler for unknown PIDs
- ✅ Segmentation fragment PID (0x08)

### 9.2 Supported Layer 3 Protocols
- ✅ No Layer 3 (PID 0xF0)
- ✅ ISO 8208/CCITT X.25 (PID 0x01)
- ✅ Compressed TCP/IP (PID 0x06)
- ✅ Uncompressed TCP/IP (PID 0x07)
- ✅ ARPA IP (PID 0xCC)
- ✅ ARPA ARP (PID 0xCD)
- ✅ NET/ROM (PID 0xCF)
- ✅ FlexNet (PID 0xCE)
- ✅ Link Quality Protocol (PID 0xC4)
- ✅ TEXNET (PID 0xC3)
- ✅ Appletalk (PID 0xCA, 0xCB)
- ✅ Escape character (PID 0xFF) for extended PIDs

---

## 10. HDLC FRAMING LAYER (Section 3, Appendix C2)

### 10.1 Physical Layer State Machine - Simplex
- ✅ Flag detection and generation
- ✅ Bit stuffing/destuffing
- ✅ NRZI encoding support (hardware abstraction)
- ✅ Abort sequence generation
- ✅ CRC-16-CCITT calculation (table-driven)

### 10.2 Physical Layer State Machine - Duplex
- ✅ Full-duplex operation (abstracted to upper layers)
- ✅ Hardware-specific PTT control
- ✅ Carrier detect interfacing
- ✅ Transmitter/receiver switching delays (timer support: T103, T104, T105, T108)

---

## 11. LINK MULTIPLEXER (Section 2.7, Appendix C3)

### 11.1 Multiple Link Support
- ✅ Multiple data-link connections
- ✅ Link rotation algorithm
- ✅ Per-link scheduling
- ✅ Priority-based transmission

---

## 12. MANAGEMENT DATA LINK (Section 2.6, Appendix C5)

### 12.1 Management Functions
- ✅ XID parameter negotiation
- ✅ XID command/response handling
- ✅ Parameter conflict resolution
- ✅ Negotiation timeout handling
- ✅ Fallback to v2.0 defaults

---

## 13. ADVANCED FEATURES (v2.2 Improvements)

### 13.1 Selective Reject (Section 6.4.4)
- ✅ SREJ command/response encoding
- ✅ SREJ exception state tracking
- ✅ Out-of-sequence frame buffering
- ✅ SREJ bitmap for multiple outstanding frames
- ✅ SREJ/REJ combined mode (per v2.2 default)
- ✅ Multiple simultaneous SREJ conditions

### 13.2 Extended Sequence Numbers
- ✅ 7-bit sequence numbers (modulo 128)
- ✅ 127-frame window support
- ✅ SABME negotiation
- ✅ 16-bit control field encoding/decoding

### 13.3 Full-Duplex Operation (Section 6.7.2)
- ✅ XID negotiation for full-duplex
- ✅ State variable tracking for full-duplex
- ✅ Physical layer full-duplex

### 13.4 Response Delay Timer (T2)
- ✅ T2 timer implementation (v2.2 addition)
- ✅ Priority acknowledge mechanism
- ✅ T2 XID negotiation

---

## 14. STATISTICS & DIAGNOSTICS

### 14.1 Performance Metrics
- ✅ Frame counters (I, S, U frames sent/received)
- ✅ Error counters (FCS, CRC, aborts, overruns)
- ✅ Retransmission tracking
- ✅ T1 expiration counting
- ✅ Byte counters (sent/received)
- ✅ Current state variables (V(S), V(R), V(A))

### 14.2 TEST Frame Statistics (Section 4.3.3.8)
- ✅ TEST command/response counters
- ✅ Round-trip time (RTT) measurement
- ✅ Average RTT calculation
- ✅ TEST sequence number tracking
- ✅ Lost TEST frame detection

---

## 15. EXTENSIONS & ENHANCEMENTS

### 15.1 FX.25 Forward Error Correction
- ✅ Reed-Solomon FEC encoding
- ✅ Correlation tag structure (8 bytes)
- ✅ Multiple FEC modes (11 predefined)
- ✅ Mode selection based on frame size
- ✅ Adaptive mode selection (channel quality)
- ✅ Galois Field GF(2^8) operations (table-based, no FPU)
- ✅ FEC decoding and error correction
- ✅ Integration with HDLC layer

### 15.2 IL2P (Improved Layer 2 Protocol)
- ❌ IL2P header mapping
- ❌ SIXBIT callsign compression
- ❌ Reed-Solomon payload blocks
- ❌ 24-bit sync word
- ❌ Scrambling/descrambling
- ❌ Type 0 transparent encapsulation
- ❌ Type 1 translated encapsulation

### 15.3 KISS Interface Protocol
- ✅ KISS framing (FEND, FESC encoding)
- ✅ KISS command byte processing
- ✅ TxDelay, Persistence, SlotTime, TxTail parameters
- ✅ Full-duplex mode control
- ✅ Hardware-specific commands
- ✅ Multi-port TNC support

### 15.4 SMACK (CRC-enhanced KISS)
- ❌ SMACK CRC-16 over KISS frames
- ❌ Automatic KISS/SMACK mode detection
- ❌ RS-232 error protection

---

## 16. EMBEDDED SYSTEMS OPTIMIZATION

### 16.1 No-FPU Design
- ✅ Integer-only arithmetic throughout
- ✅ Fixed-point timer handling (10ms ticks)
- ✅ Table-driven CRC (512 bytes flash)
- ✅ GF arithmetic via lookup tables (FX.25)

### 16.2 Memory Management
- ✅ Static buffer allocation where possible
- ✅ Configurable frame queue sizes
- ⚠️ Mostly no dynamic allocation in critical paths (some allocation in management/negotiation)
- ✅ Minimal stack usage

### 16.3 Platform Abstraction
- ✅ Hardware-independent core protocol
- ✅ Callback-based I/O (no direct hardware access)
- ✅ Portable C99 code
- ✅ Configurable compile-time options

---

## 17. UPPER LAYER INTERFACES

### 17.1 Data Link Service Access Point (DLSAP) Primitives
- ✅ DL-CONNECT request/indication/confirm
- ✅ DL-DISCONNECT request/indication/confirm
- ✅ DL-DATA request/indication
- ✅ DL-UNIT-DATA request/indication
- ✅ DL-ERROR indication
- ✅ DL-FLOW-OFF/ON request

### 17.2 Management Data Link Primitives
- ✅ MDL-NEGOTIATE request/confirm
- ✅ MDL-ERROR indication

---

## 18. NOT IMPLEMENTED (Out of Scope)

### 18.1 Deprecated Features
- ⚪ FRMR generation (v2.2 replaces with link reset) - **NOTE:** FRMR IS implemented despite deprecation for backward compatibility
- ⚪ Unbalanced operation modes (NRM, ARM)
- ⚪ SIM/RIM commands (removed in v2.2)
- ⚪ UP command (removed in v2.2)
- ⚪ RSET command (removed in v2.2)
- ⚪ RD response (removed in v2.2)

### 18.2 Hardware-Specific Features (Abstracted via Callbacks)
- ⚪ Modem modulation/demodulation (AFSK, G3RUH, etc.) - Application responsibility
- ⚪ Audio DSP processing - Application responsibility
- ⚪ Radio PTT control circuits - Callback: `ptt_control(bool)`
- ⚪ Carrier detect (DCD) processing - Callback: `carrier_detect()`
- ⚪ Audio tone generation/detection - Application responsibility

### 18.3 Network Layer (Layer 3)
- ⚪ NET/ROM routing protocol
- ⚪ ROSE X.25 packet layer
- ⚪ TCP/IP stack implementation
- ⚪ APRS encoding/decoding
- ⚪ BBS protocols

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
