#!/bin/bash
# run_ax25_test.sh
# AX.25 Interoperability Test Runner
# Copyright 2026 Emiliano Augusto Gonzalez

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}========================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

# Setup AX.25 interface with kissattach using proper PTY pairs and verification
setup_ax25_interface() {
    local port_name=$1
    local pty_master=""
    local pty_slave=""
    local socat_pid=""
    
    # Kill any existing kissattach processes
    pkill -f "kissattach" 2>/dev/null || true
    sleep 1
    
    if ! command -v kissattach &> /dev/null; then
        print_warning "kissattach command not found"
        return 1
    fi
    
    # Verify interface exists
    if ! ip link show "$port_name" > /dev/null 2>&1; then
        print_warning "Interface $port_name does not exist"
        return 1
    fi
    
    # Get kissattach usage to check if -s option is supported
    local kissattach_help=""
    kissattach_help=$(kissattach -h 2>&1 || kissattach 2>&1 | head -3)
    local use_speed_option=""
    if echo "$kissattach_help" | grep -q "\-s"; then
        use_speed_option=1
    fi
    
    # Try method 1: Using /dev/pty pairs with socat
    if command -v socat &> /dev/null; then
        print_warning "Attempting to create PTY pair with socat..."
        
        # Create PTY pair in background, keeping process alive
        # socat will print the two PTY paths to stdout
        local socat_output=""
        socat_output=$(socat PTY,raw,echo=0 PTY,raw,echo=0 2>&1) &
        socat_pid=$!
        
        # Give socat time to create PTY devices and output them
        sleep 1
        
        # Try to get PTY devices from socat output (if available)
        # Otherwise, find them by checking /proc or /dev/pts
        local pty_list=$(ls -t /dev/pts/* 2>/dev/null | head -4)
        
        # Get the two most recent PTY devices
        local pty_array=($pty_list)
        if [ ${#pty_array[@]} -ge 2 ]; then
            pty_master="${pty_array[0]}"
            pty_slave="${pty_array[1]}"
        fi
        
        if [ -n "$pty_master" ] && [ -n "$pty_slave" ] && [ "$pty_master" != "$pty_slave" ]; then
            print_warning "Using PTY devices: $pty_master (master) <-> $pty_slave (slave)"
            
            # Attempt kissattach with proper error capture
            # Note: kissattach signature is: kissattach [-options] tty port [inetaddr]
            local kissattach_output=""
            local kissattach_status=""
            
            if [ -n "$use_speed_option" ]; then
                # Try with -s option if supported
                kissattach_output=$(kissattach -s 9600 "$pty_master" "$port_name" 2>&1)
                kissattach_status=$?
            else
                # Use without -s option (speed is not configurable)
                kissattach_output=$(kissattach "$pty_master" "$port_name" 2>&1)
                kissattach_status=$?
            fi
            
            if [ $kissattach_status -eq 0 ]; then
                sleep 2
                
                # Verify kissattach actually started and bound
                if pgrep -f "kissattach.*$pty_master.*$port_name" > /dev/null 2>&1; then
                    print_success "kissattach started successfully on $pty_master"
                    print_success "PTY pair established: $pty_master <-> $pty_slave"
                    
                    # Store socat_pid globally for cleanup
                    echo $socat_pid > /tmp/socat_pid.txt
                    return 0
                else
                    print_warning "kissattach process not found after startup"
                    if [ -n "$kissattach_output" ]; then
                        print_warning "kissattach output: $kissattach_output"
                    fi
                fi
            else
                print_warning "kissattach failed with status $kissattach_status"
                if [ -n "$kissattach_output" ]; then
                    print_warning "kissattach error: $kissattach_output"
                fi
            fi
        else
            print_warning "Could not locate valid PTY pair"
        fi
        
        # Kill socat if kissattach failed
        if [ -n "$socat_pid" ]; then
            kill $socat_pid 2>/dev/null || true
            wait $socat_pid 2>/dev/null || true
        fi
    fi
    
    # Try method 2: Using /dev/ttyS0 if available
    if [ -e /dev/ttyS0 ]; then
        print_warning "Attempting kissattach with /dev/ttyS0 (serial port)..."
        local kissattach_output=""
        
        if [ -n "$use_speed_option" ]; then
            kissattach_output=$(kissattach -s 9600 /dev/ttyS0 "$port_name" 2>&1)
        else
            kissattach_output=$(kissattach /dev/ttyS0 "$port_name" 2>&1)
        fi
        local kissattach_status=$?
        
        if [ $kissattach_status -eq 0 ]; then
            sleep 1
            if pgrep -f "kissattach.*ttyS0.*$port_name" > /dev/null 2>&1; then
                print_success "kissattach started with /dev/ttyS0"
                return 0
            fi
        else
            if [ -n "$kissattach_output" ]; then
                print_warning "kissattach /dev/ttyS0 error: $kissattach_output"
            fi
        fi
    fi
    
    # Try method 3: Using /dev/ttyUSB0 if available
    if [ -e /dev/ttyUSB0 ]; then
        print_warning "Attempting kissattach with /dev/ttyUSB0 (USB serial)..."
        local kissattach_output=""
        
        if [ -n "$use_speed_option" ]; then
            kissattach_output=$(kissattach -s 9600 /dev/ttyUSB0 "$port_name" 2>&1)
        else
            kissattach_output=$(kissattach /dev/ttyUSB0 "$port_name" 2>&1)
        fi
        local kissattach_status=$?
        
        if [ $kissattach_status -eq 0 ]; then
            sleep 1
            if pgrep -f "kissattach.*ttyUSB0.*$port_name" > /dev/null 2>&1; then
                print_success "kissattach started with /dev/ttyUSB0"
                return 0
            fi
        else
            if [ -n "$kissattach_output" ]; then
                print_warning "kissattach /dev/ttyUSB0 error: $kissattach_output"
            fi
        fi
    fi
    
    print_warning "Could not start kissattach - bind tests will be skipped"
    return 1
}

print_header "Pre-Test Verification"

# Check root
if [ "$EUID" -ne 0 ]; then
    print_error "This script must be run as root (use sudo)"
    exit 1
fi

print_success "Running as root"

# Check kernel modules and load if necessary
echo ""
echo "Checking and loading kernel modules..."
for mod in ax25 mkiss; do
    if lsmod | grep -q "^$mod"; then
        print_success "$mod module loaded"
    else
        print_warning "$mod module not loaded, attempting to load..."
        if modprobe "$mod" 2>/dev/null; then
            print_success "$mod module loaded"
        else
            print_warning "Failed to load $mod (kernel may not have AX.25 support)"
        fi
    fi
done

# Verify AX.25 configuration
echo ""
echo "Checking AX.25 configuration..."
if [ -f /etc/ax25/axports ]; then
    print_success "Found /etc/ax25/axports"
    echo "Configuration:"
    grep -v "^#" /etc/ax25/axports | grep -v "^$" | head -5
else
    print_error "Missing /etc/ax25/axports"
    print_error "Run setup_ax25_test.sh first"
    exit 1
fi

# Check for TEST-0 callsign configuration
echo ""
echo "Verifying TEST-0 callsign is configured..."
if grep -q "TEST-0" /etc/ax25/axports; then
    print_success "TEST-0 callsign is configured in axports"
else
    print_warning "TEST-0 not found in axports"
    print_warning "Update /etc/ax25/axports to include: ax0 TEST-0 9600 256 2 /dev/null"
fi

# Check libax25
echo ""
echo "Checking libax25 library..."
if pkg-config --exists libax25 2>/dev/null; then
    print_success "libax25 library found"
    VERSION=$(pkg-config --modversion libax25)
    echo "  Version: $VERSION"
else
    if [ -f /usr/lib64/libax25.so ] || [ -f /usr/lib/libax25.so ]; then
        print_success "libax25 library found (installed)"
    else
        print_warning "libax25 library not found"
    fi
fi

# Setup virtual ax0 interface if needed
echo ""
echo "Checking AX.25 interfaces..."
if ip link show ax0 > /dev/null 2>&1; then
    print_success "Virtual interface ax0 exists"
else
    print_warning "Virtual interface ax0 not found, creating..."
    if ip link add ax0 type dummy 2>/dev/null; then
        ip link set ax0 up 2>/dev/null
        print_success "Created virtual interface ax0"
    else
        print_error "Failed to create ax0 interface"
    fi
fi

# Display interface info
echo "Interface details:"
ip link show ax0 2>/dev/null | head -3

# Setup kissattach
echo ""
echo "Setting up AX.25 interface with kissattach..."
setup_ax25_interface "ax0"
KISSATTACH_RESULT=$?

# Check test executable
echo ""
echo "Locating test executable..."
TEST_BIN=""
if [ -f "Release/libax25v22" ]; then
    TEST_BIN="Release/libax25v22"
elif [ -f "Debug/libax25v22" ]; then
    TEST_BIN="Debug/libax25v22"
elif [ -f "/usr/local/bin/libax25v22" ]; then
    TEST_BIN="/usr/local/bin/libax25v22"
else
    print_error "Test executable not found"
    print_error "Build it first: cd /path/to/libax25v22 && make"
    exit 1
fi

print_success "Found test executable: $TEST_BIN"

# Display diagnostic information
echo ""
print_header "System Diagnostic Information"

echo ""
echo "Loaded AX.25 modules:"
lsmod | grep -E "^(ax25|mkiss|6pack)" || echo "No AX.25 modules loaded"

echo ""
echo "Network interfaces:"
ip link show | grep -E "^[0-9]+:" | head -10

echo ""
echo "AX.25 configured ports in kernel:"
if [ -f /proc/net/ax25 ]; then
    cat /proc/net/ax25
else
    echo "No AX.25 connections (module may not be loaded)"
fi

echo ""
echo "kissattach status:"
if pgrep -f "kissattach" > /dev/null; then
    ps aux | grep kissattach | grep -v grep
else
    echo "kissattach not running"
fi

echo ""
echo "socat status:"
if pgrep -f "socat.*PTY" > /dev/null; then
    ps aux | grep socat | grep PTY | grep -v grep
else
    echo "socat not running"
fi

echo ""
echo "Available PTY devices:"
ls -la /dev/pts/ | tail -5

# Run the test
echo ""
print_header "Running AX.25 Interoperability Tests"
echo ""

$TEST_BIN

TEST_RESULT=$?

echo ""
if [ $TEST_RESULT -eq 0 ]; then
    print_success "All tests passed!"
else
    print_warning "Some tests failed or were skipped (exit code: $TEST_RESULT)"
    echo ""
    print_header "Test Results Analysis"
    echo ""
    
    if [ $KISSATTACH_RESULT -eq 0 ]; then
        echo "✓ kissattach was successfully configured"
        echo "  → Bind tests should have PASSED"
        echo "  → If they failed, check kernel logs: dmesg | tail -20"
    else
        echo "⚠ kissattach could not be configured"
        echo "  → Bind tests will be SKIPPED (this is expected)"
        echo "  → Protocol layer tests (A, C, D, E, F, G) should still PASS"
        echo ""
        echo "To enable bind tests, manually setup kissattach:"
        echo ""
        echo "  STEP 1: Open Terminal 1 and run:"
        echo "    socat PTY,raw,echo=0 PTY,raw,echo=0"
        echo ""
        echo "  STEP 2: socat will display two PTY device paths, e.g:"
        echo "    /dev/pts/15"
        echo "    /dev/pts/16"
        echo ""
        echo "  STEP 3: Open Terminal 2 and run kissattach with the FIRST PTY:"
        echo "    sudo kissattach /dev/pts/15 ax0"
        echo ""
        echo "  STEP 4: Keep both terminals open and run the test in Terminal 3:"
        echo "    sudo ./test/run_ax25_test.sh"
    fi
    
    echo ""
    echo "General troubleshooting:"
    echo "  • Verify kernel AX.25 support: lsmod | grep ax25"
    echo "  • Check axports configuration: cat /etc/ax25/axports"
    echo "  • Verify interface exists: ip link show ax0"
    echo "  • Check kernel logs: dmesg | grep -i ax25 | tail -10"
    echo "  • Verify socat installed: which socat"
    echo "  • Verify kissattach installed: which kissattach"
    echo "  • Manual kissattach test: sudo kissattach /dev/pts/X ax0"
    echo "  • View kissattach usage: kissattach"
    echo ""
fi

# Cleanup
rm -f /tmp/socat_pty.txt
if [ -f /tmp/socat_pid.txt ]; then
    socat_pid=$(cat /tmp/socat_pid.txt 2>/dev/null)
    if [ -n "$socat_pid" ]; then
        kill $socat_pid 2>/dev/null || true
    fi
    rm -f /tmp/socat_pid.txt
fi
