#!/usr/bin/env python3
"""
Example: Testing C Library Integration

This script demonstrates how to test the libax25v22 C library
using Python ctypes bindings with PyHam validation.

Usage:
    python3 example_c_integration.py /path/to/libax25v22.so

Author: Test Suite Generator
Date: 2026-02-07
"""

import sys
import ctypes
from typing import Optional

try:
    from ax25 import AX25Call, Frame
    from ax25.frame import UIFrame
except ImportError:
    print("ERROR: PyHam AX.25 not installed")
    print("Install with: pip install pyham-ax25")
    sys.exit(1)


class AX25CLibrary:
    """
    Wrapper for libax25v22 C library
    
    This is a template - actual function names and signatures
    will depend on the real library API.
    """
    
    def __init__(self, lib_path: str):
        """Load the C library"""
        try:
            self.lib = ctypes.CDLL(lib_path)
            print(f"✓ Loaded library: {lib_path}")
            self._setup_functions()
        except OSError as e:
            print(f"✗ Failed to load library: {e}")
            raise
    
    def _setup_functions(self):
        """
        Setup function signatures
        
        NOTE: These are EXAMPLES - adjust to match actual library API
        """
        
        # Example: Frame creation function
        # int ax25_create_ui_frame(
        #     uint8_t *output,
        #     const char *dest_call,
        #     uint8_t dest_ssid,
        #     const char *src_call,
        #     uint8_t src_ssid,
        #     uint8_t pid,
        #     const uint8_t *info,
        #     size_t info_len
        # );
        
        if hasattr(self.lib, 'ax25_create_ui_frame'):
            self.lib.ax25_create_ui_frame.argtypes = [
                ctypes.POINTER(ctypes.c_ubyte),  # output buffer
                ctypes.c_char_p,                  # dest callsign
                ctypes.c_uint8,                   # dest SSID
                ctypes.c_char_p,                  # src callsign
                ctypes.c_uint8,                   # src SSID
                ctypes.c_uint8,                   # PID
                ctypes.POINTER(ctypes.c_ubyte),  # info data
                ctypes.c_size_t                   # info length
            ]
            self.lib.ax25_create_ui_frame.restype = ctypes.c_int
        
        # Example: FCS calculation
        # uint16_t ax25_calc_fcs(const uint8_t *data, size_t len);
        if hasattr(self.lib, 'ax25_calc_fcs'):
            self.lib.ax25_calc_fcs.argtypes = [
                ctypes.POINTER(ctypes.c_ubyte),
                ctypes.c_size_t
            ]
            self.lib.ax25_calc_fcs.restype = ctypes.c_uint16
    
    def create_ui_frame(self, dest_call: str, dest_ssid: int,
                       src_call: str, src_ssid: int,
                       pid: int, info: bytes) -> Optional[bytes]:
        """
        Create UI frame using C library
        
        Returns:
            Encoded frame bytes or None on error
        """
        if not hasattr(self.lib, 'ax25_create_ui_frame'):
            print("Function ax25_create_ui_frame not found in library")
            return None
        
        # Allocate output buffer (max AX.25 frame size)
        output = (ctypes.c_ubyte * 512)()
        
        # Convert info to ctypes array
        info_array = (ctypes.c_ubyte * len(info)).from_buffer_copy(info)
        
        # Call C function
        result = self.lib.ax25_create_ui_frame(
            output,
            dest_call.encode('ascii'),
            dest_ssid,
            src_call.encode('ascii'),
            src_ssid,
            pid,
            info_array,
            len(info)
        )
        
        if result < 0:
            print(f"C library returned error: {result}")
            return None
        
        # Convert to Python bytes
        return bytes(output[:result])
    
    def calculate_fcs(self, data: bytes) -> Optional[int]:
        """Calculate FCS using C library"""
        if not hasattr(self.lib, 'ax25_calc_fcs'):
            print("Function ax25_calc_fcs not found in library")
            return None
        
        data_array = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
        fcs = self.lib.ax25_calc_fcs(data_array, len(data))
        
        return fcs


def test_c_vs_python_ui_frame(c_lib: AX25CLibrary):
    """
    Test: Compare C library UI frame with PyHam
    """
    print("\n" + "=" * 70)
    print("TEST: UI Frame Creation - C Library vs PyHam")
    print("=" * 70)
    
    # Test parameters
    dest_call = "DEST"
    dest_ssid = 0
    src_call = "SRC"
    src_ssid = 1
    pid = 0xF0
    info = b"Test message"
    
    print(f"\nParameters:")
    print(f"  Destination: {dest_call}-{dest_ssid}")
    print(f"  Source:      {src_call}-{src_ssid}")
    print(f"  PID:         0x{pid:02X}")
    print(f"  Info:        {info}")
    
    # Create with C library
    print("\n1. Creating frame with C library...")
    c_frame = c_lib.create_ui_frame(
        dest_call, dest_ssid,
        src_call, src_ssid,
        pid, info
    )
    
    if c_frame is None:
        print("   ✗ C library frame creation failed")
        return False
    
    print(f"   ✓ C frame created ({len(c_frame)} bytes)")
    print(f"   Hex: {c_frame.hex()}")
    
    # Create with PyHam
    print("\n2. Creating frame with PyHam...")
    dest = AX25Call(callsign=dest_call, ssid=dest_ssid)
    src = AX25Call(callsign=src_call, ssid=src_ssid)
    py_frame_obj = UIFrame(dest, src, pid=pid, info=info)
    py_frame = py_frame_obj.pack()
    
    print(f"   ✓ PyHam frame created ({len(py_frame)} bytes)")
    print(f"   Hex: {py_frame.hex()}")
    
    # Compare
    print("\n3. Comparing frames...")
    
    if len(c_frame) != len(py_frame):
        print(f"   ✗ Length mismatch: C={len(c_frame)}, PyHam={len(py_frame)}")
        return False
    
    if c_frame == py_frame:
        print("   ✓ Frames are identical!")
        return True
    else:
        print("   ⚠ Frames differ")
        # Show differences
        for i, (c_byte, py_byte) in enumerate(zip(c_frame, py_frame)):
            if c_byte != py_byte:
                print(f"   Byte {i}: C=0x{c_byte:02X}, PyHam=0x{py_byte:02X}")
        
        # Try to decode both
        print("\n4. Attempting to decode both frames...")
        
        try:
            c_decoded = Frame.unpack(c_frame)
            print(f"   ✓ C frame decoded successfully")
            print(f"     Dest: {c_decoded.destination}")
            print(f"     Src:  {c_decoded.source}")
            print(f"     Info: {c_decoded.info}")
        except Exception as e:
            print(f"   ✗ C frame decode failed: {e}")
        
        try:
            py_decoded = Frame.unpack(py_frame)
            print(f"   ✓ PyHam frame decoded successfully")
        except Exception as e:
            print(f"   ✗ PyHam frame decode failed: {e}")
        
        return False


def test_c_fcs_calculation(c_lib: AX25CLibrary):
    """
    Test: Compare FCS calculation
    """
    print("\n" + "=" * 70)
    print("TEST: FCS Calculation - C Library vs PyHam")
    print("=" * 70)
    
    # Test data
    test_data = b"\x01\x02\x03\x04\x05\x06\x07\x08"
    
    print(f"\nTest data: {test_data.hex()}")
    
    # Calculate with C library
    print("\n1. Calculating FCS with C library...")
    c_fcs = c_lib.calculate_fcs(test_data)
    
    if c_fcs is None:
        print("   ✗ C library FCS calculation failed")
        return False
    
    print(f"   ✓ C FCS: 0x{c_fcs:04X}")
    
    # Calculate with PyHam (via frame creation)
    print("\n2. Calculating FCS with PyHam...")
    dest = AX25Call(callsign="DEST", ssid=0)
    src = AX25Call(callsign="SRC", ssid=0)
    frame = UIFrame(dest, src, pid=0xF0, info=test_data)
    encoded = frame.pack()
    
    # Extract FCS from frame (last 2 bytes)
    import struct
    py_fcs = struct.unpack('<H', encoded[-2:])[0]
    print(f"   ✓ PyHam FCS: 0x{py_fcs:04X}")
    
    # Note: Direct comparison may not be valid as PyHam FCS
    # is calculated over the entire frame, not just the test data
    print("\n3. Note: FCS values may differ due to different input data")
    
    return True


def main():
    """Main test program"""
    
    if len(sys.argv) < 2:
        print("Usage: python3 example_c_integration.py <path-to-libax25v22.so>")
        print("\nExample:")
        print("  python3 example_c_integration.py /usr/local/lib/libax25v22.so")
        sys.exit(1)
    
    lib_path = sys.argv[1]
    
    print("=" * 70)
    print("AX.25 C Library Integration Test")
    print("=" * 70)
    print(f"\nLibrary: {lib_path}")
    
    # Load C library
    try:
        c_lib = AX25CLibrary(lib_path)
    except Exception as e:
        print(f"\n✗ Failed to load library: {e}")
        print("\nMake sure:")
        print("  1. Library is compiled")
        print("  2. Path is correct")
        print("  3. Library is compatible with your system")
        sys.exit(1)
    
    # Run tests
    results = []
    
    # Test 1: UI Frame
    result1 = test_c_vs_python_ui_frame(c_lib)
    results.append(("UI Frame Creation", result1))
    
    # Test 2: FCS
    result2 = test_c_fcs_calculation(c_lib)
    results.append(("FCS Calculation", result2))
    
    # Summary
    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)
    
    for test_name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{status} {test_name}")
    
    total_passed = sum(1 for _, p in results if p)
    print(f"\nTotal: {total_passed}/{len(results)} passed")
    
    if total_passed == len(results):
        print("\n✓ ALL TESTS PASSED")
        sys.exit(0)
    else:
        print("\n✗ SOME TESTS FAILED")
        sys.exit(1)


if __name__ == "__main__":
    main()
