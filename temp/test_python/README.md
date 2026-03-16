# AX.25 v2.2 Comprehensive Test Suite

Production-ready functional test suite for validating **`libax25v22`**, a C implementation of the AX.25 v2.2 amateur packet radio protocol.

This project provides  automated tests using the **PyHam AX.25 Python library** as a reference (“gold standard”) implementation. It validates frame encoding/decoding, control logic, CRC/FCS, HDLC behavior, flow control, and real-world scenarios.

---

## ✨ Key Features

✅ Exhaustive protocol validation
✅ Bit-accurate comparison against PyHam
✅ Supports C library integration via `ctypes`
✅ JSON + text reporting
✅ Real-world scenarios
✅ Modular and extensible design

Includes:

* I, S, and U frames
* Modulo-8 operation (+ indicators for modulo-128)
* CRC-CCITT FCS validation
* HDLC bit stuffing
* XID parameter negotiation
* Flow control and error recovery
* Digipeater path handling

---

## 📦 Contents

```
test_libax25v22.py        # Core AX.25 tests (~89)
test_advanced_ax25.py    # Advanced features (~45+)
run_all_tests.py         # Main runner + reports
example_c_integration.py# C library comparison
requirements.txt         # Python deps
README.md                # This file
QUICKSTART.md            # Quick usage reference
DELIVERABLES.md          # Detailed inventory
```

---

## 🧪 Test Coverage

### Frame Types

### I Frames

* NS / NR sequencing (modulo 8)
* Poll / Final bit
* PID handling
* Payload sizes up to 2048+

### S Frames

* RR (Receive Ready)
* RNR (Receive Not Ready)
* REJ (Reject)
* SREJ (if supported)

### U Frames

* UI
* SABM / SABME
* DISC
* UA
* DM
* XID
* TEST
* FRMR (deprecated)

---

### Protocol Features

* Address encoding (callsign, SSID, C/R, H-bit, extension)
* Up to 8 digipeaters
* CRC-CCITT FCS (poly x¹⁶+x¹²+x⁵+1, init 0xFFFF)
* HDLC bit stuffing / flag prevention
* XID negotiation (window, max frame size)
* Flow control
* Error recovery

---

### Real-World Scenarios

* APRS beacons + WIDE paths
* Connected packet radio
* Chat messages
* File transfer
* BBS connections
* Corrupted frames and invalid addresses

---

## 🚀 Installation

### Requirements

* Python ≥ 3.7
* PyHam AX.25

```bash
pip install pyham-ax25
```

Or:

```bash
pip install -r requirements.txt
```

Verify:

```bash
python3 -c "from ax25 import Frame; print('OK')"
```

---

## ▶️ Running Tests

### Run everything

```bash
python3 run_all_tests.py
```

### Verbose + reports

```bash
python3 run_all_tests.py --verbose --json results.json --output report.txt
```

### Individual suites

```bash
python3 test_libax25v22.py -v
python3 test_advanced_ax25.py -v
```

### Specific tests

```bash
python3 -m unittest test_libax25v22.TestAX25Addresses
python3 -m unittest test_advanced_ax25.TestPracticalScenarios
```

---

## 🔧 C Library Integration

Compare your compiled `libax25v22` against PyHam:

```bash
python3 run_all_tests.py --c-lib /path/to/libax25v22.so
```

Example direct usage:

```bash
python3 example_c_integration.py /path/to/libax25v22.so
```

Generate C wrapper:

```bash
python3 run_all_tests.py --generate-c-tests
gcc -o test_wrapper test_wrapper.c -lax25v22
./test_wrapper
```

---

## ⚠️ Known Limitations

### PyHam

* Modulo-8 only
* Partial SREJ
* FRMR deprecated

### Test Suite

* Modulo-128 limited by PyHam
* C bindings must be mapped manually
* Performance tests are basic

---

## 🛣 Future Work

* Full modulo-128 validation
* FX.25 FEC
* IL2P
* Timer testing (T1/T2/T3)
* Stress testing
* State machine validation
* Hardware-in-loop
* Benchmarking

---

## 📚 References

* AX.25 v2.2 Spec (1998)
* Modulo-128 Extension
* FX.25 FEC
* PyHam AX.25
* Dire Wolf
* Linux AX.25

---

## 📜 License

Provided as-is for testing.

* libax25v22: GPL-3.0
* PyHam AX.25: MIT

---

## ✅ Status

**Complete — Production Ready**

Version: 1.0.0
Created: 2026-02-07
