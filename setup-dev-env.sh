#!/usr/bin/env bash
# Set up development environment for LCTK (LiDAR and Camera Toolkit)
# Usage: ./setup-dev-env.sh [-y] [-v] [--no-cuda] [--no-dev-tools]

set -e

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;36m'
NC='\033[0m' # No Color

# Function to print colored messages
print_info() {
    echo -e "${BLUE}$1${NC}"
}

print_success() {
    echo -e "${GREEN}$1${NC}"
}

print_warning() {
    echo -e "${YELLOW}$1${NC}"
}

print_error() {
    echo -e "${RED}$1${NC}"
}

# Function to print help message
print_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Set up development environment for LCTK (LiDAR and Camera Toolkit)"
    echo ""
    echo "Options:"
    echo "  -h, --help          Display this help message"
    echo "  -y                  Use non-interactive mode (accept all defaults)"
    echo "  -v                  Enable verbose output"
    echo "  --no-cuda           Skip CUDA installation"
    echo "  --no-dev-tools      Skip development tools installation"
    echo "  --minimal           Install minimal dependencies only (no optional packages)"
    echo ""
    echo "Examples:"
    echo "  $0                  # Interactive installation"
    echo "  $0 -y               # Non-interactive with defaults"
    echo "  $0 -y --no-cuda     # Non-interactive without CUDA"
    echo ""
}

# Parse command line arguments
option_yes=false
option_verbose=false
option_no_cuda=false
option_no_dev_tools=false
option_minimal=false

while [ "$1" != "" ]; do
    case "$1" in
    -h | --help)
        print_help
        exit 0
        ;;
    -y)
        option_yes=true
        ;;
    -v)
        option_verbose=true
        ;;
    --no-cuda)
        option_no_cuda=true
        ;;
    --no-dev-tools)
        option_no_dev_tools=true
        ;;
    --minimal)
        option_minimal=true
        option_no_cuda=true
        option_no_dev_tools=true
        ;;
    *)
        print_error "Unknown option: $1"
        print_help
        exit 1
        ;;
    esac
    shift
done

# Initialize ansible args
ansible_args=()

# Confirm to start installation
if [ "$option_yes" = "true" ]; then
    print_info "Running setup in non-interactive mode..."
else
    print_warning "Setting up the LCTK development environment can take 15-30 minutes."
    echo ""
    echo "This will install:"
    echo "  - ROS 2 Humble and related packages"
    echo "  - Rust toolchain and cargo extensions"
    echo "  - OpenCV and GStreamer libraries"
    echo "  - Build tools and dependencies"
    if [ "$option_no_cuda" != "true" ]; then
        echo "  - CUDA toolkit (optional)"
    fi
    if [ "$option_no_dev_tools" != "true" ]; then
        echo "  - Development and debugging tools (optional)"
    fi
    echo ""
    read -rp "Are you sure you want to proceed? [y/N] " answer

    # Check whether to cancel
    if ! [[ ${answer:0:1} =~ y|Y ]]; then
        print_warning "Installation cancelled."
        exit 0
    fi

    ansible_args+=("--ask-become-pass")
fi

# Check verbose option
if [ "$option_verbose" = "true" ]; then
    ansible_args+=("-vvv")
fi

# Handle CUDA installation
if [ "$option_no_cuda" = "true" ]; then
    ansible_args+=("--extra-vars" "prompt_install_cuda=n")
elif [ "$option_yes" = "true" ]; then
    ansible_args+=("--extra-vars" "prompt_install_cuda=n")  # Default to no CUDA in non-interactive
fi

# Handle dev tools installation
if [ "$option_no_dev_tools" = "true" ]; then
    ansible_args+=("--extra-vars" "prompt_install_dev_tools=n")
elif [ "$option_yes" = "true" ]; then
    ansible_args+=("--extra-vars" "prompt_install_dev_tools=y")
fi

# Check OS version
if ! command -v lsb_release &> /dev/null; then
    print_error "lsb_release not found. Installing..."
    sudo apt-get update && sudo apt-get install -y lsb-release
fi

OS_VERSION=$(lsb_release -rs)
if [ "$OS_VERSION" != "22.04" ]; then
    print_error "This script is designed for Ubuntu 22.04 LTS"
    print_error "Your version: Ubuntu $OS_VERSION"
    read -rp "Continue anyway? [y/N] " answer
    if ! [[ ${answer:0:1} =~ y|Y ]]; then
        exit 1
    fi
fi

# Install minimal dependencies for Ansible
print_info "Checking system dependencies..."

# Install sudo if not present
if ! command -v sudo &> /dev/null; then
    print_info "Installing sudo..."
    apt-get -y update
    apt-get -y install sudo
fi

# Install Python and pip
if ! command -v python3 &> /dev/null; then
    print_info "Installing Python 3..."
    sudo apt-get -y update
    sudo apt-get -y install python3 python3-pip python3-venv
fi

# Install pipx for Ansible
if ! python3 -m pipx --version &> /dev/null 2>&1; then
    print_info "Installing pipx..."
    python3 -m pip install --user pipx
    python3 -m pipx ensurepath
fi

# Update PATH to include pipx
export PATH="${HOME}/.local/bin:$PATH"

# Install Ansible
print_info "Installing Ansible..."
pipx install --include-deps --force "ansible==6.*"

# Install Ansible collections
print_info "Installing Ansible Galaxy collections..."
if [ "$option_verbose" = "true" ]; then
    if ! ansible-galaxy collection install -f -r "$SCRIPT_DIR/ansible/ansible-galaxy-requirements.yaml"; then
        print_warning "Failed to install some Ansible collections, but continuing..."
    fi
else
    if ! ansible-galaxy collection install -f -r "$SCRIPT_DIR/ansible/ansible-galaxy-requirements.yaml" > /dev/null 2>&1; then
        print_warning "Failed to install some Ansible collections, but continuing..."
    fi
fi

# Set Ansible configuration path
export ANSIBLE_CONFIG="$SCRIPT_DIR/ansible/ansible.cfg"

# Run the Ansible playbook
print_info "Running LCTK setup playbook..."
print_info "Command: ansible-playbook $SCRIPT_DIR/ansible/playbooks/lctk.dev_env.yaml ${ansible_args[*]}"

# Change to ansible directory for relative paths to work
cd "$SCRIPT_DIR/ansible"

if ansible-playbook "playbooks/lctk.dev_env.yaml" "${ansible_args[@]}"; then
    print_success "================================================"
    print_success "LCTK development environment setup completed!"
    print_success "================================================"
    echo ""
    echo "Next steps:"
    echo "1. Reload your shell or run: source ~/.bashrc"
    echo "2. Build the project: make build"
    echo "3. Test with sample data: make launch_sensor"
    echo ""
    print_info "For more information, see README.md"
    exit 0
else
    print_error "Setup failed. Please check the error messages above."
    exit 1
fi