#!/bin/bash
# setup_ax25_test.sh
# AX.25 Interoperability Test Setup for Fedora 43
# Copyright 2026 Emiliano Augusto Gonzalez
# This script installs and configures all required AX.25 components
# Compatible with dnf5

set +e

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Default flags
UPDATE_SYSTEM=0

# Functions
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

print_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "OPTIONS:"
    echo "  --update      Update system packages before installation"
    echo "  --help        Show this help message"
    echo ""
    echo "Examples:"
    echo "  sudo bash $0                  # Install without updating system"
    echo "  sudo bash $0 --update         # Install and update system packages"
}

check_root() {
    if [ "$EUID" -ne 0 ]; then
        print_error "This script must be run as root (use sudo)"
        exit 1
    fi
}

check_distro() {
    if ! grep -qi "fedora" /etc/os-release; then
        print_error "This script is designed for Fedora"
        exit 1
    fi
    
    VERSION=$(grep VERSION_ID /etc/os-release | cut -d= -f2 | tr -d '"')
    print_success "Detected Fedora $VERSION"
}

# dnf5 compatibility check
check_dnf_version() {
    if command -v dnf5 &> /dev/null; then
        print_success "Using dnf5"
        DNF_CMD="dnf5"
    elif command -v dnf &> /dev/null; then
        print_success "Using dnf"
        DNF_CMD="dnf"
    else
        print_error "Neither dnf nor dnf5 found"
        exit 1
    fi
}

# Parse command line arguments
parse_args() {
    while [ $# -gt 0 ]; do
        case "$1" in
            --update)
                UPDATE_SYSTEM=1
                print_warning "System update enabled"
                ;;
            --help|-h)
                print_usage
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                print_usage
                exit 1
                ;;
        esac
        shift
    done
}

# Enhanced package installation with better dnf5 compatibility
install_package() {
    local pkg=$1
    local dnf_cmd=$2
    
    print_warning "Installing $pkg..."
    
    # Try to install the package
    if $dnf_cmd install -y "$pkg" > /dev/null 2>&1; then
        # Check if package is actually installed
        if $dnf_cmd list installed "$pkg" > /dev/null 2>&1; then
            print_success "$pkg installed"
            return 0
        else
            # Package may have been installed before
            print_success "$pkg available"
            return 0
        fi
    else
        # Installation failed, but check if already installed
        if $dnf_cmd list installed "$pkg" > /dev/null 2>&1; then
            print_success "$pkg already installed"
            return 0
        else
            print_warning "Could not install $pkg"
            return 1
        fi
    fi
}

# Enhanced package group installation
install_package_group() {
    local group=$1
    local dnf_cmd=$2
    
    print_warning "Installing group: $group..."
    
    if $dnf_cmd groupinstall -y "$group" > /dev/null 2>&1; then
        print_success "Group $group installed"
        return 0
    else
        print_warning "Could not install group $group"
        return 1
    fi
}

# Check if package is installed
check_package_installed() {
    local pkg=$1
    local dnf_cmd=$2
    
    if $dnf_cmd list installed "$pkg" > /dev/null 2>&1; then
        return 0
    fi
    return 1
}

main() {
    print_header "Fedora 43 AX.25 Test Environment Setup"
    
    check_root
    check_distro
    check_dnf_version
    
    # Step 1: Update system (optional)
    if [ $UPDATE_SYSTEM -eq 1 ]; then
        print_header "Step 1: Updating system packages"
        print_warning "This may take several minutes..."
        if $DNF_CMD update -y > /dev/null 2>&1; then
            print_success "System updated"
        else
            print_warning "System update had issues, continuing anyway"
        fi
    else
        print_header "Step 1: Skipping system update (use --update flag to enable)"
        print_success "Proceeding with installation"
    fi
    
    # Step 2: Install build tools (dnf5 compatible)
    print_header "Step 2: Installing build tools"
    
    if [ "$DNF_CMD" = "dnf5" ]; then
        print_warning "dnf5 detected - installing packages individually"
        
        BUILD_PACKAGES=(
            "gcc"
            "g++"
            "make"
            "cmake"
            "pkg-config"
            "git"
            "autoconf"
            "automake"
            "libtool"
            "binutils"
        )
        
        for pkg in "${BUILD_PACKAGES[@]}"; do
            install_package "$pkg" "$DNF_CMD"
        done
    else
        print_warning "Attempting groupinstall for Development Tools..."
        if install_package_group "Development Tools" "$DNF_CMD"; then
            print_success "Development Tools group installed"
        else
            print_warning "Groupinstall failed, installing individual packages"
            BUILD_PACKAGES=(
                "gcc"
                "make"
                "cmake"
                "pkg-config"
                "git"
                "autoconf"
                "automake"
                "libtool"
            )
            for pkg in "${BUILD_PACKAGES[@]}"; do
                install_package "$pkg" "$DNF_CMD"
            done
        fi
    fi
    
    print_success "Build tools installation completed"
    
    # Step 3: Install AX.25 libraries and utilities
    print_header "Step 3: Installing AX.25 libraries and tools"
    
    AX25_PACKAGES=(
        "libax25"
        "libax25-devel"
        "ax25-tools"
        "ax25-apps"
        "socat"
    )
    
    REPO_OK=0
    for pkg in "${AX25_PACKAGES[@]}"; do
        if check_package_installed "$pkg" "$DNF_CMD"; then
            print_success "$pkg already installed"
            REPO_OK=1
        else
            if install_package "$pkg" "$DNF_CMD"; then
                REPO_OK=1
            fi
        fi
    done
    
    if [ $REPO_OK -eq 0 ]; then
        print_warning "AX.25 packages not available in repository"
        print_warning "Building libax25 from source..."
        
        mkdir -p /tmp/ax25-build
        cd /tmp/ax25-build
        
        if [ ! -d "linuxax25" ]; then
            print_warning "Cloning linuxax25 repository..."
            if ! git clone https://github.com/ve7fet/linuxax25.git 2>&1; then
                print_error "Failed to clone linuxax25 repository"
                exit 1
            fi
        fi
        cd linuxax25
        
        if [ -f "autogen.sh" ]; then
            print_warning "Running autogen.sh..."
            if ! bash autogen.sh > /dev/null 2>&1; then
                print_warning "autogen.sh had issues, continuing"
            fi
        fi
        
        if [ ! -f "configure" ]; then
            print_warning "Running configure..."
            if ! ./configure --prefix=/usr > /dev/null 2>&1; then
                print_error "Configure failed"
                exit 1
            fi
        fi
        
        print_warning "Building libax25..."
        if ! make > /dev/null 2>&1; then
            print_error "Build failed"
            exit 1
        fi
        
        print_warning "Installing libax25..."
        if ! make install > /dev/null 2>&1; then
            print_error "Install failed"
            exit 1
        fi
        
        ldconfig
        print_success "libax25 built and installed from source"
    else
        print_success "AX.25 packages installation completed"
    fi
    
    # Step 4: Enable kernel modules
    print_header "Step 4: Loading AX.25 kernel modules"
    
    MODULES=("ax25" "mkiss" "6pack")
    
    for mod in "${MODULES[@]}"; do
        print_warning "Loading module: $mod"
        if modprobe "$mod" 2>/dev/null; then
            if lsmod | grep -q "^$mod"; then
                print_success "$mod kernel module loaded"
            else
                print_warning "$mod module loaded but not visible (may load on demand)"
            fi
        else
            print_warning "Could not load $mod (may not be compiled)"
        fi
    done
    
    sleep 1
    
    if lsmod | grep -q "^ax25"; then
        print_success "AX.25 kernel module verified loaded"
    else
        print_warning "AX.25 kernel module not immediately loaded"
        print_warning "This may require kernel rebuild with CONFIG_AX25=m"
        print_warning "Continuing setup anyway..."
    fi
    
    # Step 5: Create AX.25 configuration directory
    print_header "Step 5: Setting up AX.25 configuration"
    
    mkdir -p /etc/ax25
    mkdir -p /var/ax25
    
    cat > /etc/ax25/axports << 'EOF'
# AX.25 port configuration
# Format: name callsign speed paclen window device
#
# TEST-0 is the standard callsign for automated testing
# This configuration allows socket binding for interoperability tests
#
ax0      TEST-0          9600    256     2       /dev/null
EOF
    
    chmod 644 /etc/ax25/axports
    print_success "Created /etc/ax25/axports with TEST-0 callsign"
    
    # Step 6: Create ax25d configuration
    print_header "Step 6: Configuring ax25d"
    
    cat > /etc/ax25/ax25d.conf << 'EOF'
# AX.25 daemon configuration for Fedora 43
# Defines behavior for incoming AX.25 connections

[TEST-0]
parameters PACLEN 256 WINDOW 2 N2 10 T1 20000 T2 2000 T3 600000 IDLE 0 T4 0

[W1AW-0]
parameters PACLEN 256 WINDOW 2 N2 10 T1 20000 T2 2000 T3 600000 IDLE 0 T4 0

default NOCALL   - -  -
default TEST     - -  -
default W1AW     - -  -
default *        - -  -
EOF
    
    chmod 644 /etc/ax25/ax25d.conf
    print_success "Created /etc/ax25/ax25d.conf"
    
    # Step 7: Create virtual AX.25 interface
    print_header "Step 7: Creating virtual AX.25 interface"
    
    if ip link show ax0 > /dev/null 2>&1; then
        print_warning "Interface ax0 already exists, removing..."
        ip link delete ax0 2>/dev/null || true
        sleep 1
    fi
    
    if command -v ip &> /dev/null; then
        print_warning "Creating virtual interface ax0..."
        if ip link add ax0 type dummy 2>/dev/null; then
            ip link set ax0 up 2>/dev/null
            print_success "Virtual AX.25 interface ax0 created"
            
            echo "Interface configuration:"
            ip link show ax0 2>/dev/null | head -3
        else
            print_warning "Failed to create interface (may already exist)"
        fi
    else
        print_warning "ip command not found"
    fi
    
    # Step 8: Install systemd service
    print_header "Step 8: Setting up systemd service"
    
    cat > /etc/systemd/system/ax25.service << 'EOF'
[Unit]
Description=AX.25 Packet Radio Network
After=network.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'modprobe ax25 2>/dev/null; modprobe mkiss 2>/dev/null; ip link add ax0 type dummy 2>/dev/null || true; ip link set ax0 up 2>/dev/null || true'
ExecStop=/bin/bash -c 'pkill -f kissattach 2>/dev/null || true; ip link delete ax0 2>/dev/null || true; modprobe -r mkiss 2>/dev/null; modprobe -r ax25 2>/dev/null'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
    
    systemctl daemon-reload
    if systemctl enable ax25.service 2>/dev/null; then
        print_success "AX.25 systemd service enabled"
    else
        print_warning "Could not enable ax25 service"
    fi
    print_success "AX.25 systemd service installed"
    
    # Step 9: Verify installation
    print_header "Step 9: Verifying installation"
    
    echo ""
    
    if pkg-config --exists libax25 2>/dev/null; then
        print_success "libax25 library found"
        pkg-config --modversion libax25
    else
        if [ -f /usr/lib64/libax25.so ] || [ -f /usr/lib/libax25.so ]; then
            print_success "libax25 library found (installed)"
        else
            print_warning "libax25 library not found"
        fi
    fi
    
    echo ""
    for mod in ax25 mkiss; do
        if lsmod | grep -q "^$mod"; then
            print_success "AX.25 kernel module $mod loaded"
        else
            print_warning "AX.25 kernel module $mod not loaded (will load on demand)"
        fi
    done
    
    echo ""
    if [ -f /etc/ax25/axports ]; then
        print_success "/etc/ax25/axports configured"
    else
        print_warning "/etc/ax25/axports not found"
    fi
    
    echo ""
    if ip link show ax0 &>/dev/null; then
        print_success "Virtual interface ax0 configured"
    else
        print_warning "Virtual interface ax0 not found"
    fi
    
    echo ""
    if command -v kissattach &> /dev/null; then
        print_success "kissattach command available"
    else
        print_warning "kissattach command not found"
    fi
    
    echo ""
    if command -v socat &> /dev/null; then
        print_success "socat command available"
    else
        print_warning "socat command not found (needed for bind tests)"
    fi
    
    # Final summary
    print_header "Setup Complete!"
    
    echo ""
    echo "Summary of installation:"
    echo "  • Installed build tools and libraries"
    echo "  • Loaded AX.25 kernel modules"
    echo "  • Created /etc/ax25/ configuration"
    echo "  • Configured TEST-0 callsign for testing"
    echo "  • Set up systemd service"
    echo "  • Created virtual AX.25 interface (ax0)"
    if [ $UPDATE_SYSTEM -eq 1 ]; then
        echo "  • Updated system packages"
    fi
    echo ""
    echo "To run the interoperability tests:"
    echo "  1. Build: cd /path/to/libax25v22 && make"
    echo "  2. Test:  sudo ./test/run_ax25_test.sh"
    echo ""
    echo "For full bind test support with kissattach:"
    echo ""
    echo "  METHOD A: Automatic (via run_ax25_test.sh)"
    echo "  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  The test script will automatically try to setup PTY pair"
    echo "  using socat and kissattach."
    echo ""
    echo "  METHOD B: Manual Setup (if automatic fails)"
    echo "  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  Terminal 1:"
    echo "    socat PTY,raw,echo=0 PTY,raw,echo=0"
    echo "    (Note the two PTY paths printed, e.g. /dev/pts/15 and /dev/pts/16)"
    echo ""
    echo "  Terminal 2:"
    echo "    sudo kissattach -s 9600 /dev/pts/15 ax0"
    echo "    (Use the FIRST PTY from Terminal 1)"
    echo ""
    echo "  Terminal 3:"
    echo "    sudo ./test/run_ax25_test.sh"
    echo ""
    echo "To verify AX.25 is working:"
    echo "  • Check modules: lsmod | grep ax25"
    echo "  • Check config:  cat /etc/ax25/axports"
    echo "  • Check interface: ip link show ax0"
    echo "  • Check AX.25 ports: cat /proc/net/ax25"
    echo "  • Check PTY devices: ls -la /dev/pts/ | head -10"
    echo "  • Check kissattach: ps aux | grep kissattach"
    echo "  • Manual kissattach test: sudo kissattach -s 9600 /dev/pts/X ax0"
    echo ""
}

parse_args "$@"
main
