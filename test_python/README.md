# AX.25 v2.2 Comprehensive Test Suite

Comprehensive functional testing suite for the `libax25v22` C library implementation using the PyHam AX.25 Python library for validation and reference.

## Overview

This test suite provides exhaustive testing of AX.25 v2.2 protocol implementation, covering:

- **Frame Types**: I-frames, S-frames (RR, RNR, REJ, SREJ), U-frames (SABM, SABME, DISC, UA, DM, UI, XID, TEST, FRMR)
- **Address Handling**: Callsign encoding, SSID handling, digipeater paths, H-bit management
- **Control Fields**: Modulo 8 and Modulo 128 sequence numbers, Poll/Final bit handling
- **FCS Calculation**: CRC-CCITT frame check sequence
- **HDLC Features**: Bit stuffing/unstuffing, flag detection
- **Parameter Negotiation**: XID frame parameter exchange
- **Flow Control**: Windowing, acknowledgments, error recovery
- **Practical Scenarios**: APRS, packet radio, file transfer

## Requirements

### Python Requirements

```bash
# Python 3.7 or later
python3 --version

# Install PyHam AX.25 library
pip install pyham-ax25

# Optional: for enhanced testing
pip install pytest pytest-cov
```

### C Library Requirements

The test suite is designed to test the `libax25v22` C library:
- https://github.com/hiperiondev/libax25v22

## Test Files

### Core Test Modules

1. **`test_libax25v22.py`** - Basic protocol tests
   - Address field encoding/decoding
   - Control field handling (all frame types)
   - Frame construction and parsing
   - FCS calculation and validation
   - PID field handling
   - Connection establishment/termination

2. **`test_advanced_ax25.py`** - Advanced feature tests
   - HDLC bit stuffing
   - Modulo 128 extended sequences
   - Parameter negotiation (XID)
   - Flow control scenarios
   - Error recovery
   - Performance tests
   - Practical scenarios (APRS, packet radio)

3. **`run_all_tests.py`** - Main test runner
   - Executes all test suites
   - Generates comprehensive reports
   - Optional C library integration

## Usage

### Running All Tests

```bash
# Run complete test suite
python3 run_all_tests.py

# With verbose output
python3 run_all_tests.py --verbose

# Generate JSON report
python3 run_all_tests.py --json test_results.json

# Specify output file
python3 run_all_tests.py --output my_report.txt
```

### Running Individual Test Suites

```bash
# Run basic tests only
python3 test_libax25v22.py

# Run advanced tests only
python3 test_advanced_ax25.py

# Run with unittest
python3 -m unittest test_libax25v22

# Run specific test class
python3 -m unittest test_libax25v22.TestAX25Addresses

# Run specific test method
python3 -m unittest test_libax25v22.TestAX25Addresses.test_address_encoding_basic
```

### Testing with C Library

If you have the compiled C library:

```bash
# Test with C library
python3 run_all_tests.py --c-lib /path/to/libax25v22.so

# Generate C test wrapper
python3 run_all_tests.py --generate-c-tests

# Compile C wrapper
gcc -o test_wrapper test_wrapper.c -lax25v22 -I/path/to/include -L/path/to/lib

# Run C tests
./test_wrapper
```

## Test Coverage

### Address Field Tests (TestAX25Addresses)

- ✓ Basic 6-character callsign encoding
- ✓ Short callsign space-padding
- ✓ SSID range validation (0-15)
- ✓ Digipeater path handling
- ✓ H-bit (has-been-repeated) management
- ✓ Extension bit handling
- ✓ C/R bit encoding

### Control Field Tests (TestAX25ControlField)

#### I-Frame Tests
- ✓ Modulo 8 sequence numbers (NS, NR)
- ✓ Poll/Final bit handling
- ✓ Control byte encoding

#### S-Frame Tests
- ✓ RR (Receive Ready)
- ✓ RNR (Receive Not Ready)
- ✓ REJ (Reject)
- ✓ SREJ (Selective Reject) - if supported

#### U-Frame Tests
- ✓ SABM (Set Asynchronous Balanced Mode)
- ✓ SABME (Extended mode for modulo 128)
- ✓ DISC (Disconnect)
- ✓ UA (Unnumbered Acknowledge)
- ✓ DM (Disconnected Mode)
- ✓ UI (Unnumbered Information)
- ✓ XID (Exchange Identification)
- ✓ TEST (Test/loopback)
- ✓ FRMR (Frame Reject) - deprecated in v2.2

### Frame Construction Tests (TestAX25FrameConstruction)

- ✓ Complete UI frame creation
- ✓ Complete I-frame creation
- ✓ Maximum digipeaters (8)
- ✓ Maximum information field (256 bytes default, negotiable to larger)
- ✓ Empty information field
- ✓ Round-trip encoding/decoding

### FCS Tests (TestAX25FCS)

- ✓ CRC-CCITT calculation
- ✓ FCS validation
- ✓ Corruption detection
- ✓ All-zero and all-one patterns

### Protocol ID Tests (TestAX25PID)

- ✓ No Layer 3 (0xF0)
- ✓ IP Protocol (0xCC)
- ✓ ARP (0xCD)
- ✓ NET/ROM (0xCF)
- ✓ All standard PIDs

### Connection Tests (TestAX25Connection)

- ✓ SABM connection establishment
- ✓ SABME extended mode establishment
- ✓ UA acknowledgment
- ✓ DISC disconnection
- ✓ DM response
- ✓ Complete connection sequence

### Advanced Tests

#### Bit Stuffing (TestBitStuffing)
- ✓ Five consecutive 1s handling
- ✓ Flag emulation prevention
- ✓ Maximum stuff bits scenario

#### Modulo 128 (TestModulo128)
- ✓ SABME indicates modulo 128
- ✓ Source SSID bit 6 indicator
- ✓ Extended sequence numbers

#### Parameter Negotiation (TestParameterNegotiation)
- ✓ XID frame structure
- ✓ Window size negotiation
- ✓ Maximum frame size negotiation
- ✓ Timer negotiation

#### Flow Control (TestFlowControl)
- ✓ Window size management
- ✓ RR acknowledgments
- ✓ RNR flow control
- ✓ REJ error recovery
- ✓ Sequence number wraparound

#### Error Conditions (TestErrorConditions)
- ✓ FRMR response
- ✓ Invalid frame handling
- ✓ Corrupted address detection
- ✓ FCS error detection

#### Practical Scenarios (TestPracticalScenarios)
- ✓ APRS position report
- ✓ Packet radio chat
- ✓ Connected file transfer
- ✓ Digipeater usage

## Test Results Interpretation

### Success Output
```
==================================================
AX.25 v2.2 COMPREHENSIVE TEST REPORT
==================================================
Start Time: 2026-02-07 10:30:00
End Time: 2026-02-07 10:30:15
Duration: 15.23 seconds

SUMMARY
--------------------------------------------------
✓ PASS Basic AX.25 Tests        Tests:  89  Failures:  0  Errors:  0
✓ PASS Advanced AX.25 Tests     Tests:  45  Failures:  0  Errors:  0

TOTALS
--------------------------------------------------
Total Tests Run:  134
Total Failures:   0
Total Errors:     0
Success Rate:     100.0%

OVERALL RESULT: ✓ ALL TESTS PASSED
==================================================
```

### Failure Output
```
✗ FAIL test_iframe_control_modulo8
AssertionError: NR mismatch: expected 5, got 3
```

## AX.25 Protocol Reference

### Frame Structure

```
+--------+----------+---------+---------+-----+--------+
| Flag   | Address  | Control | PID     | Info| FCS    |
| (0x7E) | (14-70B) | (1-2B)  | (1B)    | ... | (2B)   |
+--------+----------+---------+---------+-----+--------+
```

### Address Field Format

```
+----------+----------+--------+--------+--------+
| Dest (7) | Src (7)  | Digi1  | Digi2  | ...    |
+----------+----------+--------+--------+--------+

Each address: [Call(6B)] [SSID/Cmd/Res/Ext(1B)]
```

### Control Field Types

**I-Frame (Information):**
```
Bit:  7  6  5  4  3  2  1  0
     [N(R)  ][P][N(S)  ]  0
```

**S-Frame (Supervisory):**
```
Bit:  7  6  5  4  3  2  1  0
     [N(R)  ][P][ S  ] 0  1
```

**U-Frame (Unnumbered):**
```
Bit:  7  6  5  4  3  2  1  0
     [ M  ] [P][ M  ] 1  1
```

## Known Limitations

### PyHam AX.25 Library
- Only supports modulo 8 (not modulo 128)
- SREJ may not be fully implemented
- FRMR is deprecated in v2.2

### libax25v22 C Library
- Implementation details depend on the actual library
- Some advanced features may be under development

## Extending the Tests

To add new tests:

```python
class TestMyFeature(unittest.TestCase):
    """Test my custom feature"""
    
    def test_my_case(self):
        """Test specific case"""
        # Arrange
        dest = AX25Call(callsign="DEST", ssid=0)
        src = AX25Call(callsign="SRC", ssid=0)
        
        # Act
        frame = UIFrame(dest, src, pid=0xF0, info=b"Test")
        encoded = frame.pack()
        decoded = Frame.unpack(encoded)
        
        # Assert
        self.assertEqual(decoded.info, b"Test")
```

## References

### AX.25 Protocol Specifications
- [AX.25 v2.2 Specification (July 1998)](https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf)
- [Modulo 128 Extension](https://web.tapr.org/meetings/DCC_1995/DCC1995-Modul128-4AX.25-PE1CHL.pdf)
- [FX.25 Forward Error Correction](https://eindhoven.space/wp-content/uploads/2022/12/fx-25_01_06.pdf)

### Libraries and Tools
- [PyHam AX.25 Documentation](https://pyham.org/en/latest/)
- [libax25v22 Repository](https://github.com/hiperiondev/libax25v22)
- [Dire Wolf (Reference Implementation)](https://github.com/wb2osz/direwolf)

## License

This test suite is provided as-is for testing purposes.
Individual components may have their own licenses:
- libax25v22: GPL-3.0
- PyHam AX.25: MIT License

## Contributing

To contribute to this test suite:

1. Fork the repository
2. Create a feature branch
3. Add tests for new features
4. Ensure all tests pass
5. Submit a pull request

## Support

For issues or questions:
- Check the documentation
- Review test output carefully
- Consult AX.25 specifications
- Open an issue on GitHub

---

**Last Updated:** 2026-02-07
**Version:** 1.0.0
