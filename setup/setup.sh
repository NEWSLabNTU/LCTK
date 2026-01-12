#!/usr/bin/env bash
# LCTK Setup Wrapper
# Interactive setup with optional components

set -e

# Resolve symlinks to find the actual script location
SCRIPT_PATH="$(readlink -f "$0")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
MARKER_DIR="${SCRIPT_DIR}/.markers"
SUDO_PID_FILE="${MARKER_DIR}/.sudo-loop-pid"
SUDO_LOOP_PID=""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Show usage
show_usage() {
    cat << 'EOF'
LCTK Development Environment Setup

Usage:
  ./setup.sh              Run interactive setup
  ./setup.sh status       Show setup status
  ./setup.sh <recipe>     Run specific recipe (ros2, rust, opencv, etc.)
  ./setup.sh --help       Show this help

Examples:
  ./setup.sh              # Interactive full setup
  ./setup.sh status       # Check what's installed
  ./setup.sh ros2         # Install only ROS 2
  ./setup.sh clean-markers # Reset all installation markers

For available recipes, run: just --list --justfile setup/justfile
EOF
}

# Cleanup function - called on exit (normal or abnormal)
cleanup() {
    local exit_code=$?
    if [[ -n "$SUDO_LOOP_PID" ]] && kill -0 "$SUDO_LOOP_PID" 2>/dev/null; then
        kill "$SUDO_LOOP_PID" 2>/dev/null || true
    fi
    rm -f "$SUDO_PID_FILE"

    # Show cancellation message if interrupted (but not on normal exit)
    if [[ $exit_code -eq 130 ]]; then
        printf "\n${YELLOW}Cancelled by user${NC}\n" >&2
    fi
}

# Handle interrupt signal (Ctrl-C)
interrupt_handler() {
    printf "\n${YELLOW}Interrupted${NC}\n" >&2
    exit 130
}

# Set trap for cleanup on any exit
trap cleanup EXIT
trap interrupt_handler INT TERM

# Recipes that don't need sudo
NO_SUDO_RECIPES="status clean-marker clean-markers default"

# Check if recipe needs sudo
needs_sudo() {
    local recipe="${1:-setup}"
    for r in $NO_SUDO_RECIPES; do
        [[ "$recipe" == "$r" ]] && return 1
    done
    return 0
}

# Start sudo keep-alive loop
start_sudo_loop() {
    mkdir -p "$MARKER_DIR"

    # Check if we already have sudo credentials cached
    if ! sudo -n true 2>/dev/null; then
        printf "${YELLOW}->>${NC} Requesting sudo privileges...\n"
        sudo -v || { printf "${RED}x${NC} Failed to obtain sudo credentials\n"; exit 1; }
    fi

    # Start background sudo refresh loop (silent)
    (
        while true; do
            sudo -n true
            sleep 50
        done
    ) </dev/null >/dev/null 2>&1 &
    SUDO_LOOP_PID=$!
    disown $SUDO_LOOP_PID 2>/dev/null || true
    echo "$SUDO_LOOP_PID" > "$SUDO_PID_FILE"
}

# Ask yes/no question
ask_yes_no() {
    local question="$1"
    local default="${2:-y}"
    local prompt

    if [[ "$default" == "y" ]]; then
        prompt="[Y/n/q]"
    else
        prompt="[y/N/q]"
    fi

    while true; do
        printf "${BLUE}?${NC} %s %s " "$question" "$prompt"
        read -r response || {
            # Handle Ctrl-D (EOF)
            printf "\n${YELLOW}Cancelled${NC}\n"
            exit 130
        }
        response="${response:-$default}"
        case "${response,,}" in
            y|yes) return 0 ;;
            n|no) return 1 ;;
            q|quit|exit)
                printf "\n${YELLOW}Cancelled${NC}\n"
                exit 0
                ;;
            *) printf "${RED}Please answer yes, no, or q to quit.${NC}\n" ;;
        esac
    done
}

# Check OS version
check_os() {
    if ! command -v lsb_release &> /dev/null; then
        printf "${RED}Error: lsb_release not found${NC}\n"
        exit 1
    fi

    local os_version
    os_version=$(lsb_release -rs)
    if [[ "$os_version" != "22.04" ]]; then
        printf "${YELLOW}Warning: This script is designed for Ubuntu 22.04 LTS${NC}\n"
        printf "Your version: Ubuntu %s\n\n" "$os_version"
        if ! ask_yes_no "Continue anyway?" "n"; then
            exit 1
        fi
    fi
}

# Interactive setup configuration
interactive_setup() {
    printf "\n${BLUE}LCTK Development Environment Setup${NC}\n\n"

    printf "Core components that will be installed:\n"
    printf "  - ROS 2 Humble and development tools\n"
    printf "  - Rust toolchain (stable and nightly)\n"
    printf "  - OpenCV and GStreamer libraries\n"
    printf "  - Build tools and dependencies\n"
    printf "\n"

    # Optional: Git submodules
    UPDATE_SUBMODULES="n"
    printf "${YELLOW}Git Submodules:${NC} Update git submodules\n"
    printf "${RED}Warning:${NC} This will discard any local changes in submodules.\n"
    if ask_yes_no "Update git submodules?" "n"; then
        UPDATE_SUBMODULES="y"
    fi
    printf "\n"

    # Optional: CUDA
    INSTALL_CUDA="n"
    printf "${YELLOW}Optional:${NC} CUDA toolkit (~2-3 GB)\n"
    printf "Required for GPU acceleration.\n"
    if ask_yes_no "Install CUDA toolkit?" "n"; then
        INSTALL_CUDA="y"
    fi
    printf "\n"

    # Optional: Development tools
    INSTALL_DEV_TOOLS="y"
    printf "${YELLOW}Optional:${NC} Development tools (gdb, valgrind, mdbook, etc.)\n"
    printf "Includes debugging tools, code quality tools, and documentation tools.\n"
    if ask_yes_no "Install development tools?" "y"; then
        INSTALL_DEV_TOOLS="y"
    else
        INSTALL_DEV_TOOLS="n"
    fi
    printf "\n"

    # Export choices for justfile
    export UPDATE_SUBMODULES="$UPDATE_SUBMODULES"
    export INSTALL_CUDA="$INSTALL_CUDA"
    export INSTALL_DEV_TOOLS="$INSTALL_DEV_TOOLS"

    # Summary
    printf "Installing: Core"
    if [[ "$UPDATE_SUBMODULES" == "y" ]]; then
        printf " + Submodules"
    fi
    if [[ "$INSTALL_CUDA" == "y" ]]; then
        printf " + CUDA"
    fi
    if [[ "$INSTALL_DEV_TOOLS" == "y" ]]; then
        printf " + Dev tools"
    fi
    printf "\n\n"

    if ! ask_yes_no "Continue?" "y"; then
        printf "${YELLOW}Cancelled${NC}\n"
        exit 0
    fi
    printf "\n"
}

# Main
main() {
    # Handle --help/-h
    if [[ "$1" == "--help" ]] || [[ "$1" == "-h" ]]; then
        show_usage
        exit 0
    fi

    local recipe="${1:-setup}"

    # Check if just is installed
    if ! command -v just &> /dev/null; then
        printf "${RED}x${NC} 'just' command not found\n"
        printf "Install with: ${BLUE}cargo install just${NC}\n"
        printf "Or: ${BLUE}curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/.local/bin${NC}\n"
        exit 1
    fi

    # Check OS version
    check_os

    # If running full setup, show interactive wizard
    if [[ "$recipe" == "setup" ]]; then
        if [[ ! -t 0 ]]; then
            # Non-interactive mode (piped input)
            printf "${YELLOW}->>${NC} Non-interactive mode\n"
            export UPDATE_SUBMODULES="${UPDATE_SUBMODULES:-n}"
            export INSTALL_CUDA="${INSTALL_CUDA:-n}"
            export INSTALL_DEV_TOOLS="${INSTALL_DEV_TOOLS:-y}"
        else
            # Interactive mode
            interactive_setup
        fi
    fi

    # Start sudo loop if needed (silently)
    if needs_sudo "$recipe"; then
        start_sudo_loop
    fi

    # Run just with all arguments
    cd "$SCRIPT_DIR"
    if [[ $# -eq 0 ]]; then
        # No arguments, run setup
        just setup
    else
        # Pass through all arguments
        just "$@"
    fi

    # After successful setup, show next steps
    if [[ "$recipe" == "setup" ]]; then
        printf "\n${GREEN}Setup complete!${NC}\n\n"
        printf "Next steps:\n"
        printf "1. Reload your shell:    ${BLUE}source ~/.bashrc${NC}\n"
        printf "2. Go to project root:   ${BLUE}cd %s${NC}\n" "$(dirname "$SCRIPT_DIR")"
        printf "3. Build the project:    ${BLUE}just build${NC}\n"
        printf "\n"
        printf "For more commands: ${BLUE}just help${NC}\n"
    fi
}

main "$@"
