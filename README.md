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
## About The Project

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

<!-- GETTING STARTED -->
## Features
- [x] 

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

<!-- USAGE -->
## Usage

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

<!-- ROADMAP -->
## Roadmap

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

<!-- CONTRIBUTING -->
## Contributing

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
## License

Distributed under the GNU General Public License v3.0. See `LICENSE.txt` for more information.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

<!-- CONTACT -->
## Contact

*Emiliano Augusto Gonzalez - egonzalez.hiperion@gmail.com*

Project Link: [https://https://github.com/hiperiondev/libax25v22](https://github.com/hiperiondev/libax25v22)

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>
