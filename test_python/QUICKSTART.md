# Quick Start Guide - AX.25 v2.2 Test Suite

## Installation

### 1. Install Python Dependencies

```bash
# Install PyHam AX.25
pip install pyham-ax25

# Or install all dependencies
pip install -r requirements.txt
```

### 2. Verify Installation

```bash
python3 -c "from ax25 import Frame; print('PyHam AX.25 installed successfully')"
```

## Running Tests

### Quick Test

Run all tests with default settings:

```bash
python3 run_all_tests.py
```

### Detailed Test Run

```bash
# Verbose output with JSON report
python3 run_all_tests.py --verbose --json results.json --output report.txt

# Run only basic tests
python3 test_libax25v22.py -v

# Run only advanced tests
python3 test_advanced_ax25.py -v
```

### Testing Specific Features

```bash
# Test only address encoding
python3 -m unittest test_libax25v22.TestAX25Addresses -v

# Test only I-frames
python3 -m unittest test_libax25v22.TestAX25ControlField.test_iframe_control_modulo8

# Test APRS scenarios
python3 -m unittest test_advanced_ax25.TestPracticalScenarios.test_aprs_position_report
```

## Expected Output

### Successful Run

```
==================================================
AX.25 v2.2 COMPREHENSIVE TEST REPORT
==================================================
Start Time: 2026-02-07 10:30:00
Duration: 15.23 seconds
PyHam AX.25 Version: 1.0.3

SUMMARY
--------------------------------------------------
✓ PASS Basic AX.25 Tests        Tests:  89
✓ PASS Advanced AX.25 Tests     Tests:  45

TOTALS
--------------------------------------------------
Total Tests Run:  134
Total Failures:   0
Total Errors:     0
Success Rate:     100.0%

OVERALL RESULT: ✓ ALL TESTS PASSED
==================================================
```

## Test Categories

### 1. Basic Tests (test_libax25v22.py)

**Address Tests:**
- Basic encoding
- SSID handling
- Digipeater paths

**Control Field Tests:**
- I-frames (Information)
- S-frames (Supervisory: RR, RNR, REJ)
- U-frames (Unnumbered: SABM, DISC, UA, etc.)

**Frame Tests:**
- Construction
- Parsing
- FCS validation

### 2. Advanced Tests (test_advanced_ax25.py)

**Protocol Features:**
- Bit stuffing
- Modulo 128
- Parameter negotiation

**Scenarios:**
- APRS beacons
- Connected mode
- File transfer
- Error recovery

## Common Issues

### Issue: PyHam Not Installed

```
ERROR: PyHam AX.25 library not installed
```

**Solution:**
```bash
pip install pyham-ax25
```

### Issue: Python Version

```
ERROR: Python 3.7+ required
```

**Solution:**
```bash
python3 --version  # Check version
# Upgrade Python if needed
```

### Issue: Import Errors

```
ModuleNotFoundError: No module named 'ax25'
```

**Solution:**
```bash
# Ensure in correct directory
cd /path/to/tests

# Reinstall PyHam
pip install --upgrade pyham-ax25
```

## Understanding Test Results

### Test Status Indicators

- ✓ **PASS** - Test passed successfully
- ✗ **FAIL** - Test failed (assertion error)
- **ERROR** - Test error (exception)
- **SKIP** - Test skipped

### Reading Failure Messages

```
FAIL: test_iframe_control_modulo8
----------------------------------------------------------------------
AssertionError: NR mismatch: expected 5, got 3
```

This shows:
- Which test failed
- What was expected
- What was actually received

## Next Steps

### Run Full Suite

```bash
# Complete test with reporting
python3 run_all_tests.py --verbose --json results.json
```

### Generate C Tests

```bash
# Generate C wrapper
python3 run_all_tests.py --generate-c-tests

# Compile (if you have the C library)
gcc -o test_wrapper test_wrapper.c -lax25v22
./test_wrapper
```

### Review Results

```bash
# View text report
cat test_report.txt

# View JSON report
python3 -m json.tool results.json
```

## Test Development

### Adding New Tests

1. **Choose test file:**
   - Basic features → `test_libax25v22.py`
   - Advanced features → `test_advanced_ax25.py`

2. **Create test class:**
```python
class TestMyFeature(unittest.TestCase):
    def test_my_case(self):
        # Your test code
        pass
```

3. **Run your test:**
```bash
python3 -m unittest test_libax25v22.TestMyFeature
```

### Test Template

```python
def test_my_feature(self):
    """Test description"""
    # Arrange - setup test data
    dest = AX25Call(callsign="DEST", ssid=0)
    src = AX25Call(callsign="SRC", ssid=0)
    
    # Act - perform the operation
    frame = UIFrame(dest, src, pid=0xF0, info=b"Test")
    encoded = frame.pack()
    
    # Assert - verify results
    decoded = Frame.unpack(encoded)
    self.assertEqual(decoded.info, b"Test")
```

## Troubleshooting

### Tests Hang

- Check for infinite loops in test code
- Interrupt with Ctrl+C
- Run individual tests to isolate

### Tests Fail Randomly

- May indicate race conditions
- Run multiple times to verify
- Check for state dependencies

### Memory Issues

- Large test data may cause issues
- Monitor with: `python3 -m memory_profiler test_*.py`

## Getting Help

1. **Check Documentation:**
   - README.md
   - PyHam docs: https://pyham.org
   - AX.25 spec: https://www.ax25.net

2. **Review Test Output:**
   - Read error messages carefully
   - Check stack traces
   - Verify test data

3. **Debug Tests:**
```python
# Add debug prints
print(f"Frame: {frame}")
print(f"Encoded: {encoded.hex()}")

# Use debugger
python3 -m pdb test_libax25v22.py
```

## Resources

- **AX.25 Spec:** https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf
- **PyHam Docs:** https://pyham.org/en/latest/
- **libax25v22:** https://github.com/hiperiondev/libax25v22

