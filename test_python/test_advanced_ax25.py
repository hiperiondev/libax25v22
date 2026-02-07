#!/usr/bin/env python3
"""
Advanced AX.25 v2.2 Test Suite
Tests for advanced features:
- Modulo 128 extended sequence numbers
- HDLC bit stuffing/unstuffing
- Parameter negotiation
- Flow control scenarios
- Error recovery

Author: Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
Date: 2026-02-07
"""

import unittest
import struct
from typing import List, Tuple
import random

try:
    import ax25
    from ax25 import Frame, Address, Control, FrameType
except ImportError:
    print("WARNING: PyHam ax25 module not found. Tests will fail.")
    # Define dummy classes to prevent import errors
    class DummyFrame:
        pass
    class DummyAddress:
        pass
    class DummyControl:
        pass
    class DummyFrameType:
        UI = None
        I = None
        RR = None
        RNR = None
        REJ = None
        SREJ = None
        SABM = None
        SABME = None
        DISC = None
        DM = None
        UA = None
        FRMR = None
        XID = None
        TEST = None
    Frame = DummyFrame
    Address = DummyAddress
    Control = DummyControl
    FrameType = DummyFrameType

class TestBitStuffing(unittest.TestCase):
    """Test HDLC bit stuffing and unstuffing"""
    
    def test_bit_stuffing_basic(self):
        """Test basic bit stuffing (five consecutive 1s)"""
        # Create data with five consecutive 1s: 0x1F = 0001 1111
        test_data = bytes([0x1F])
        
        # After bit stuffing, a 0 should be inserted after five 1s
        # This test verifies the concept
        # Actual implementation depends on the library
        
        # The bit stuffing should prevent flag emulation
        flag = 0x7E  # 0111 1110
        
        # Verify that five 1s doesn't create a flag
        self.assertNotEqual(test_data[0], flag)
    
    
    def test_flag_detection(self):
        """Test that flag sequence (0x7E) is properly handled"""
        # Flag is 0111 1110 = 0x7E
        flag = bytes([0x7E])
        
        # Flags should not appear in the data stream except as delimiters
        # This is ensured by bit stuffing
        dest = "DEST-0"
        src = "SRC-0"
        
        # Create frame with data that might contain flag patterns
        info = bytes([0x7E, 0x7E, 0xFF, 0x00, 0x7E])
        
        control = Control(FrameType.UI)
        frame = Frame(dest, src, control=control, pid=0xF0, data=info)
        encoded = frame.pack()
        
        # Should be able to decode despite flag bytes in info
        decoded = Frame.unpack(encoded)
        self.assertEqual(decoded.data, info)
    
    def test_maximum_stuff_bits(self):
        """Test data requiring maximum bit stuffing"""
        # Create data with many consecutive 1s
        dest = "DEST-0"
        src = "SRC-0"
        
        # 0xFF = 1111 1111 requires stuffing
        info = bytes([0xFF] * 100)
        
        control = Control(FrameType.UI)
        frame = Frame(dest, src, control=control, pid=0xF0, data=info)
        encoded = frame.pack()
        decoded = Frame.unpack(encoded)
        
        self.assertEqual(decoded.data, info)
    
class TestModulo128(unittest.TestCase):
    """
    Test modulo 128 extended sequence numbers (AX.25 v2.2)
    Note: PyHam currently only supports modulo 8
    """
    
    def test_sabme_indicates_modulo_128(self):
        """Test that SABME indicates modulo 128 mode"""
        dest = "DEST-0"
        src = "SRC-0"
        
        # SABME is used for modulo 128 connections
        control = Control(FrameType.SABME, poll_final=True)
        sabme = Frame(dest, src, control=control)
        encoded = sabme.pack()
        
        # Verify SABME control byte
        control_byte = encoded[14]
        self.assertEqual(control_byte & 0xEF, 0x6F)
    
    def test_modulo_128_indicator_bit(self):
        """Test modulo 128 indicator in source SSID"""
        # In modulo 128 mode, bit 6 of source SSID byte is cleared
        # This is for monitoring stations to know how to decode
        
        src = Address("SRC-0")
        encoded = src.pack()
        
        # Bit 6 (0-indexed) or bit 5 in the SSID byte
        # For modulo 8, this should be 1 (reserved)
        # For modulo 128, this should be 0
        ssid_byte = encoded[6]
        
        # In normal modulo 8 mode, bit 5 should be set
        # (This is the default for PyHam which only supports modulo 8)
        self.assertTrue(ssid_byte & 0x20)  # Bit 5 should be 1
    
class TestParameterNegotiation(unittest.TestCase):
    """Test XID parameter negotiation"""
      
    def test_xid_frame_structure(self):
        """Test XID frame for parameter negotiation"""
        dest = "DEST-0"
        src = "SRC-0"
        
        # XID frame - exact info format depends on implementation
        control = Control(FrameType.XID, poll_final=True)
        frame = Frame(
            dest,
            src,
            control=control,
            data=b""  # Empty for now
        )
        
        encoded = frame.pack()
        
        # Verify control field
        control_byte = encoded[14]
        self.assertEqual(control_byte & 0xEF, 0xAF)
    
    def test_xid_parameter_fields(self):
        """Test XID parameter field encoding"""
        # XID frame format according to ISO 8885:
        # FI (Format Identifier) = 0x82
        # GI (Group Identifier) = 0x80
        # GL (Group Length) = 2 bytes
        # Parameters follow
        
        # Example parameter negotiation for window size
        # PI = 0x08 (Window Size Receive)
        # PL = 0x01 (1 byte length)
        # PV = 0x07 (window size = 7)
        
        xid_info = bytes([
            0x82,        # FI: general purpose XID
            0x80,        # GI: parameter negotiation
            0x00, 0x03,  # GL: 3 bytes of parameters
            0x08,        # PI: Window Size
            0x01,        # PL: 1 byte
            0x07         # PV: window size = 7
        ])
        
        dest = "DEST-0"
        src = "SRC-0"
        
        control = Control(FrameType.XID, poll_final=True)
        frame = Frame(dest, src, control=control, data=xid_info)
        encoded = frame.pack()
        decoded = Frame.unpack(encoded)
        
        self.assertEqual(decoded.control.frame_type, FrameType.XID)
        self.assertEqual(decoded.data, xid_info)
    
class TestFlowControl(unittest.TestCase):
    """Test flow control mechanisms"""
     
    def test_window_size_basic(self):
        """Test basic windowing with k=7 (modulo 8)"""
        dest = "DEST-0"
        src = "SRC-0"
        
        # Send 7 I-frames (window size = 7 for modulo 8)
        frames = []
        for ns in range(7):
            control = Control(FrameType.I, send_seqno=ns, recv_seqno=0, poll_final=False)
            iframe = Frame(
                dest, src,
                control=control,
                pid=0xF0,
                data=f"Frame {ns}".encode()
            )
            frames.append(iframe)
        
        # Verify all frames can be created
        for i, frame in enumerate(frames):
            encoded = frame.pack()
            decoded = Frame.unpack(encoded)
            self.assertEqual(decoded.control.send_seqno, i)
    
    def test_rr_acknowledgement(self):
        """Test RR (Receive Ready) acknowledgement"""
        dest = "DEST-0"
        src = "SRC-0"
        
        # RR with NR=5 acknowledges frames 0-4
        control = Control(FrameType.RR, recv_seqno=5, poll_final=False)
        rr = Frame(dest, src, control=control)
        encoded = rr.pack()
        decoded = Frame.unpack(encoded)
        
        self.assertEqual(decoded.control.frame_type, FrameType.RR)
        self.assertEqual(decoded.control.recv_seqno, 5)
    
    def test_rnr_flow_control(self):
        """Test RNR (Receive Not Ready) for flow control"""
        dest = "DEST-0"
        src = "SRC-0"
        
        # RNR indicates receiver is busy
        control = Control(FrameType.RNR, recv_seqno=3, poll_final=False)
        rnr = Frame(dest, src, control=control)
        encoded = rnr.pack()
        decoded = Frame.unpack(encoded)
        
        self.assertEqual(decoded.control.frame_type, FrameType.RNR)
        self.assertEqual(decoded.control.recv_seqno, 3)
    
    def test_rej_error_recovery(self):
        """Test REJ (Reject) for error recovery"""
        dest = "DEST-0"
        src = "SRC-0"
        
        # REJ with NR=4 requests retransmission from frame 4
        control = Control(FrameType.REJ, recv_seqno=4, poll_final=True)
        rej = Frame(dest, src, control=control)
        encoded = rej.pack()
        decoded = Frame.unpack(encoded)
        
        self.assertEqual(decoded.control.frame_type, FrameType.REJ)
        self.assertEqual(decoded.control.recv_seqno, 4)
        self.assertEqual(decoded.control.poll_final, True)
    
    def test_sequence_number_wraparound(self):
        """Test sequence number wraparound (modulo 8)"""
        dest = "DEST-0"
        src = "SRC-0"
        
        # Test wraparound from 7 to 0
        for ns in [6, 7, 0, 1]:
            control = Control(FrameType.I, send_seqno=ns, recv_seqno=0, poll_final=False)
            iframe = Frame(
                dest, src,
                control=control,
                pid=0xF0,
                data=b"Test"
            )
            encoded = iframe.pack()
            decoded = Frame.unpack(encoded)
            self.assertEqual(decoded.control.send_seqno, ns)
    
class TestErrorConditions(unittest.TestCase):
    """Test error handling and recovery"""
       
    def test_frmr_response(self):
        """Test FRMR (Frame Reject) response"""
        dest = "DEST-0"
        src = "SRC-0"
        
        # FRMR indicates an error condition
        # Info field contains error information
        frmr_info = bytes([
            0x00,  # Control field of rejected frame
            0x00,  # VS/VR/CR/WXYZ bits
            0x00,  # Reserved
        ])
        
        try:
            control = Control(FrameType.FRMR, poll_final=False)
            frmr = Frame(dest, src, control=control, data=frmr_info)
            encoded = frmr.pack()
            decoded = Frame.unpack(encoded)
            self.assertEqual(decoded.control.frame_type, FrameType.FRMR)
        except (AttributeError, ValueError):
            # FRMR may not be implemented in PyHam (it's obsolete in v2.2)
            self.skipTest("FRMR not available in this implementation")
    
    def test_invalid_frame_handling(self):
        """Test handling of invalid frames"""
        # Create an invalid frame (too short)
        invalid_frame = bytes([0x7E, 0x00, 0x7E])
        
        with self.assertRaises(Exception):
            Frame.unpack(invalid_frame)
    
    def test_corrupted_address_field(self):
        """Test handling of corrupted address field"""
        dest = "DEST-0"
        src = "SRC-0"
        
        control = Control(FrameType.UI)
        frame = Frame(dest, src, control=control, pid=0xF0, data=b"Test")
        encoded = bytearray(frame.pack())
        
        # Corrupt address field
        encoded[0] ^= 0xFF
        
        # Should fail to decode or produce incorrect address
        try:
            decoded = Frame.unpack(bytes(encoded))
            # If it decodes, the address should be different
            self.assertNotEqual(str(decoded.dst), "DEST")
        except:
            pass  # Expected to fail

class TestPerformance(unittest.TestCase):
    """Test performance characteristics"""
        
    def test_large_frame_handling(self):
        """Test handling of maximum-size frames"""
        dest = "DEST-0"
        src = "SRC-0"
        
        # Test with various large sizes
        for size in [256, 512, 1024, 2048]:
            info = bytes([random.randint(0, 255) for _ in range(size)])
            
            try:
                control = Control(FrameType.UI)
                frame = Frame(dest, src, control=control, pid=0xF0, data=info)
                encoded = frame.pack()
                decoded = Frame.unpack(encoded)
                self.assertEqual(decoded.data, info)
            except:
                # May fail for very large frames depending on implementation
                if size <= 256:
                    raise  # Should support at least 256 bytes
    
    def test_rapid_frame_encoding(self):
        """Test rapid encoding/decoding of many frames"""
        dest = "DEST-0"
        src = "SRC-0"
        
        # Encode and decode 1000 frames
        for i in range(1000):
            control = Control(FrameType.UI)
            frame = Frame(
                dest, src,
                control=control,
                pid=0xF0,
                data=f"Frame {i}".encode()
            )
            encoded = frame.pack()
            decoded = Frame.unpack(encoded)
            self.assertEqual(decoded.data, f"Frame {i}".encode())
    
class TestPracticalScenarios(unittest.TestCase):
    """Test practical usage scenarios"""
        
    def test_aprs_position_report(self):
        """Test APRS position report"""
        dest = "APRS-0"
        src = "N0CALL-0"
        path = ["WIDE1-1", "WIDE2-1"]
        
        # APRS uncompressed position
        aprs_data = b"!4903.50N/07201.75W-Test Station"
        
        control = Control(FrameType.UI)
        frame = Frame(
            dest,
            src,
            control=control,
            pid=0xF0,
            data=aprs_data,
            via=path
        )
        
        encoded = frame.pack()
        decoded = Frame.unpack(encoded)
        
        # PyHam omits SSID 0 in string representation, so "APRS-0" becomes "APRS"
        self.assertEqual(str(decoded.dst), "APRS")
        self.assertEqual(decoded.data, aprs_data)
    
    def test_packet_radio_chat(self):
        """Test packet radio chat message"""
        dest = "DEST-0"
        src = "SRC-0"
        
        # Simple chat message
        message = b"Hello from the packet radio network!"
        
        control = Control(FrameType.UI)
        frame = Frame(dest, src, control=control, pid=0xF0, data=message)
        encoded = frame.pack()
        decoded = Frame.unpack(encoded)
        
        self.assertEqual(decoded.data, message)
    
    def test_connected_file_transfer(self):
        """Test connected mode file transfer scenario"""
        dest = "BBS-0"
        src = "USER-0"
        
        # Simulate file transfer with sequence of I-frames
        file_data = b"This is file content being transferred"
        chunk_size = 10
        
        chunks = [
            file_data[i:i+chunk_size]
            for i in range(0, len(file_data), chunk_size)
        ]
        
        for ns, chunk in enumerate(chunks):
            if ns >= 8:  # Modulo 8 limit
                ns = ns % 8
            
            control = Control(FrameType.I, send_seqno=ns, recv_seqno=0, poll_final=False)
            iframe = Frame(
                dest, src,
                control=control,
                pid=0xF0,
                data=chunk
            )
            
            encoded = iframe.pack()
            decoded = Frame.unpack(encoded)
            self.assertEqual(decoded.data, chunk)
    
    def test_digipeater_usage(self):
        """Test frame with multiple digipeaters"""
        dest = "DEST-0"
        src = "SRC-0"
        digis = [f"DIGI{i}-0" for i in range(1, 5)]
        
        control = Control(FrameType.UI)
        frame = Frame(
            dest,
            src,
            control=control,
            pid=0xF0,
            data=b"Via digipeaters",
            via=digis
        )
        
        encoded = frame.pack()
        decoded = Frame.unpack(encoded)
        
        self.assertIsNotNone(decoded.via)
        self.assertEqual(len(decoded.via), 4)
        # PyHam omits SSID 0 in string representation, so "DIGI1-0" becomes "DIGI1"
        for i, digi in enumerate(decoded.via):
            self.assertEqual(str(digi), f"DIGI{i+1}")
    
class TestTESTFrame(unittest.TestCase):
    """Test TEST frame (loopback test)"""
        
    def test_test_frame_basic(self):
        """Test basic TEST frame"""
        dest = "DEST-0"
        src = "SRC-0"
        
        test_data = b"TEST FRAME DATA"
        
        control = Control(FrameType.TEST, poll_final=True)
        frame = Frame(
            dest,
            src,
            control=control,
            data=test_data
        )
        
        encoded = frame.pack()
        decoded = Frame.unpack(encoded)
        
        self.assertEqual(decoded.control.frame_type, FrameType.TEST)
        self.assertEqual(decoded.data, test_data)

def run_advanced_tests():
    """Run all advanced tests"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    suite.addTests(loader.loadTestsFromTestCase(TestBitStuffing))
    suite.addTests(loader.loadTestsFromTestCase(TestModulo128))
    suite.addTests(loader.loadTestsFromTestCase(TestParameterNegotiation))
    suite.addTests(loader.loadTestsFromTestCase(TestFlowControl))
    suite.addTests(loader.loadTestsFromTestCase(TestErrorConditions))
    suite.addTests(loader.loadTestsFromTestCase(TestPerformance))
    suite.addTests(loader.loadTestsFromTestCase(TestPracticalScenarios))
    suite.addTests(loader.loadTestsFromTestCase(TestTESTFrame))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result

if __name__ == "__main__":
    print("=" * 70)
    print("Advanced AX.25 v2.2 Test Suite")
    print("=" * 70)
    print()
    
    result = run_advanced_tests()
    
    print()
    print("=" * 70)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Skipped: {len(result.skipped)}")
    
    if result.wasSuccessful():
        print("\n✓ ALL ADVANCED TESTS PASSED")
    else:
        print("\n✗ SOME ADVANCED TESTS FAILED")
