#!/usr/bin/env python3
"""
Main Test Runner for libax25v22 C Library
Comprehensive testing with PyHam AX.25 validation

This script runs all test suites and provides detailed reporting.
It can optionally test the C library directly if available.

Usage:
    python run_all_tests.py [--c-lib /path/to/libax25v22.so] [--generate-c-tests]

Author: Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com)
Date: 2026-02-07
"""

import sys
import os
import unittest
import argparse
import json
from datetime import datetime
from typing import Dict, List

# Import test modules
try:
    import test_libax25v22
    import test_advanced_ax25
except ImportError as e:
    print(f"ERROR: Could not import test modules: {e}")
    sys.exit(1)

try:
    import ax25
    ax25_version = getattr(ax25, '__version__', 'unknown')
    print(f"✓ PyHam AX.25 library version: {ax25_version}")
except ImportError as e:
    print("✗ ERROR: PyHam AX.25 library not installed or not importable")
    print("  The validation tests require the 'ax25' module from the pyham-ax25 package")
    print()
    print("  Running Python executable:", sys.executable)
    print("  Python version:", sys.version.split('\n')[0])
    print()
    print("  Import error details:", e)
    print()
    print("  Recommended install command (uses the exact Python running the tests):")
    print(f"  {sys.executable} -m pip install pyham-ax25")
    print()
    print("  If site-packages is not writeable, it will default to user installation")
    print("  After installation, verify with:")
    print(f"  {sys.executable} -c \\\"import ax25; print('Import successful')\\\"")
    print()
    sys.exit(1)

class TestReport:
    """Generate comprehensive test report"""
    
    def __init__(self):
        self.start_time = datetime.now()
        self.results = []
        
    def add_result(self, suite_name: str, result):
        """Add test result"""
        self.results.append({
            'suite': suite_name,
            'tests_run': result.testsRun,
            'failures': len(result.failures),
            'errors': len(result.errors),
            'skipped': len(result.skipped),
            'success': result.wasSuccessful()
        })
    
    def generate_report(self) -> str:
        """Generate text report"""
        end_time = datetime.now()
        duration = (end_time - self.start_time).total_seconds()
        
        report = []
        report.append("=" * 80)
        report.append("AX.25 v2.2 COMPREHENSIVE TEST REPORT")
        report.append("=" * 80)
        report.append(f"Start Time: {self.start_time}")
        report.append(f"End Time: {end_time}")
        report.append(f"Duration: {duration:.2f} seconds")
        report.append(f"PyHam AX.25 Version: {ax25_version}")
        report.append("")
        
        total_tests = sum(r['tests_run'] for r in self.results)
        total_failures = sum(r['failures'] for r in self.results)
        total_errors = sum(r['errors'] for r in self.results)
        total_skipped = sum(r['skipped'] for r in self.results)
        
        report.append("SUMMARY")
        report.append("-" * 80)
        for result in self.results:
            status = "✓ PASS" if result['success'] else "✗ FAIL"
            report.append(f"{status} {result['suite']:<40} "
                         f"Tests: {result['tests_run']:3d} "
                         f"Failures: {result['failures']:2d} "
                         f"Errors: {result['errors']:2d} "
                         f"Skipped: {result['skipped']:2d}")
        
        report.append("")
        report.append("TOTALS")
        report.append("-" * 80)
        report.append(f"Total Tests Run:  {total_tests}")
        report.append(f"Total Failures:   {total_failures}")
        report.append(f"Total Errors:     {total_errors}")
        report.append(f"Total Skipped:    {total_skipped}")
        report.append(f"Success Rate:     {((total_tests - total_failures - total_errors) / total_tests * 100):.1f}%")
        
        overall_success = all(r['success'] for r in self.results)
        report.append("")
        if overall_success:
            report.append("OVERALL RESULT: ✓ ALL TESTS PASSED")
        else:
            report.append("OVERALL RESULT: ✗ SOME TESTS FAILED")
        
        report.append("=" * 80)
        
        return "\n".join(report)
    
    def save_json_report(self, filename: str):
        """Save report as JSON"""
        data = {
            'timestamp': self.start_time.isoformat(),
            'pyham_version': ax25_version,
            'results': self.results,
            'summary': {
                'total_tests': sum(r['tests_run'] for r in self.results),
                'total_failures': sum(r['failures'] for r in self.results),
                'total_errors': sum(r['errors'] for r in self.results),
                'total_skipped': sum(r['skipped'] for r in self.results),
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"JSON report saved to: {filename}")

def generate_c_test_wrapper():
    """Generate C test wrapper code"""
    c_code = """/*
 * AX.25 v2.2 Library Test Wrapper
 * Generated test wrapper for libax25v22
 * 
 * This file provides a C interface for testing the library
 * Compile with: gcc -o test_wrapper test_wrapper.c -lax25v22
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Include your library header
// #include "ax25v22.h"

// Test function prototypes
int test_address_encoding(void);
int test_iframe_creation(void);
int test_sframe_creation(void);
int test_uframe_creation(void);
int test_fcs_calculation(void);
int test_bit_stuffing(void);

// Main test runner
int main(int argc, char *argv[]) {
    int passed = 0;
    int failed = 0;
    
    printf("=================================================================\\n");
    printf("libax25v22 C Library Test Suite\\n");
    printf("=================================================================\\n\\n");
    
    // Run tests
    printf("Running test_address_encoding... ");
    if (test_address_encoding() == 0) {
        printf("PASS\\n");
        passed++;
    } else {
        printf("FAIL\\n");
        failed++;
    }
    
    printf("Running test_iframe_creation... ");
    if (test_iframe_creation() == 0) {
        printf("PASS\\n");
        passed++;
    } else {
        printf("FAIL\\n");
        failed++;
    }
    
    printf("Running test_sframe_creation... ");
    if (test_sframe_creation() == 0) {
        printf("PASS\\n");
        passed++;
    } else {
        printf("FAIL\\n");
        failed++;
    }
    
    printf("Running test_uframe_creation... ");
    if (test_uframe_creation() == 0) {
        printf("PASS\\n");
        passed++;
    } else {
        printf("FAIL\\n");
        failed++;
    }
    
    printf("Running test_fcs_calculation... ");
    if (test_fcs_calculation() == 0) {
        printf("PASS\\n");
        passed++;
    } else {
        printf("FAIL\\n");
        failed++;
    }
    
    printf("Running test_bit_stuffing... ");
    if (test_bit_stuffing() == 0) {
        printf("PASS\\n");
        passed++;
    } else {
        printf("FAIL\\n");
        failed++;
    }
    
    printf("\\n");
    printf("=================================================================\\n");
    printf("Total: %d passed, %d failed\\n", passed, failed);
    printf("=================================================================\\n");
    
    return (failed > 0) ? 1 : 0;
}

// Test implementations
int test_address_encoding(void) {
    // TODO: Implement using libax25v22 API
    // Example test structure:
    /*
    ax25_address_t dest, src;
    uint8_t encoded[14];
    
    ax25_address_init(&dest, "DEST", 0);
    ax25_address_init(&src, "SRC", 1);
    
    ax25_encode_address(encoded, &dest, &src);
    
    // Verify encoding
    if (encoded[0] != expected_value) {
        return -1;  // FAIL
    }
    */
    
    return 0;  // PASS (placeholder)
}

int test_iframe_creation(void) {
    // TODO: Implement I-frame creation test
    /*
    ax25_frame_t frame;
    uint8_t buffer[256];
    size_t len;
    
    ax25_create_iframe(&frame, dest, src, ns, nr, pf, info, info_len);
    len = ax25_encode_frame(&frame, buffer, sizeof(buffer));
    
    // Verify frame
    */
    
    return 0;  // PASS (placeholder)
}

int test_sframe_creation(void) {
    // TODO: Implement S-frame creation test
    return 0;
}

int test_uframe_creation(void) {
    // TODO: Implement U-frame creation test
    return 0;
}

int test_fcs_calculation(void) {
    // TODO: Implement FCS calculation test
    /*
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t fcs;
    
    fcs = ax25_calculate_fcs(data, sizeof(data));
    
    // Verify FCS matches expected value
    uint16_t expected_fcs = 0x1234;  // Calculate correct value
    if (fcs != expected_fcs) {
        return -1;
    }
    */
    
    return 0;
}

int test_bit_stuffing(void) {
    // TODO: Implement bit stuffing test
    return 0;
}
""" 
    with open("test_wrapper.c", "w") as f:
        f.write(c_code)
    
    print("C test wrapper generated: test_wrapper.c")

def main():
    """Main test runner"""
    parser = argparse.ArgumentParser(
        description="Comprehensive test suite for libax25v22"
    )
    parser.add_argument(
        "--c-lib",
        type=str,
        help="Path to libax25v22 shared library (.so file)"
    )
    parser.add_argument(
        "--generate-c-tests",
        action="store_true",
        help="Generate C test wrapper code"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="test_report.txt",
        help="Output file for test report"
    )
    parser.add_argument(
        "--json",
        type=str,
        help="Output file for JSON report"
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Verbose output"
    )
    
    args = parser.parse_args()
    
    # Generate C tests if requested
    if args.generate_c_tests:
        generate_c_test_wrapper()
        print("C test wrapper generated. Compile with:")
        print("  gcc -o test_wrapper test_wrapper.c -lax25v22")
        print()
    
    # Create test report
    report = TestReport()
    
    print("=" * 80)
    print("STARTING AX.25 v2.2 COMPREHENSIVE TEST SUITE")
    print("=" * 80)
    print()
    
    # Check PyHam installation
    try:
        import ax25
        print(f"✓ PyHam AX.25 library version: {ax25_version}")
    except ImportError:
        print("✗ ERROR: PyHam AX.25 library not installed")
        print("  Install with: pip install pyham-ax25")
        sys.exit(1)
    
    print()
    
    # Run basic test suite
    print("Running Basic AX.25 Tests...")
    print("-" * 80)
    result1 = test_libax25v22.run_tests_with_lib(args.c_lib)
    report.add_result("Basic AX.25 Tests", result1)
    print()
    
    # Run advanced test suite
    print("Running Advanced AX.25 Tests...")
    print("-" * 80)
    result2 = test_advanced_ax25.run_advanced_tests()
    report.add_result("Advanced AX.25 Tests", result2)
    print()
    
    # Generate and display report
    report_text = report.generate_report()
    print(report_text)
    
    # Save text report
    with open(args.output, 'w') as f:
        f.write(report_text)
    print(f"\nText report saved to: {args.output}")
    
    # Save JSON report if requested
    if args.json:
        report.save_json_report(args.json)
    
    # Exit with appropriate code
    overall_success = all(r['success'] for r in report.results)
    sys.exit(0 if overall_success else 1)

if __name__ == "__main__":
    main()
    