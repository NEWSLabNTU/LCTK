#!/usr/bin/env bash
# LCTK development environment setup.
#
# The one documented entry point. It gates on `just`, offers to install it when missing,
# then hands the selected steps to `setup/justfile`, which delegates to `setup/steps.py`
# for dependency resolution, content-hashed markers and verification.

set -uo pipefail

SCRIPT_PATH="$(readlink -f "$0")"
SETUP_DIR="$(dirname "$SCRIPT_PATH")"
PROJECT_ROOT="$(dirname "$SETUP_DIR")"
STEPS="${SETUP_DIR}/steps.py"
SUDO_LOOP_PID=""

GREEN='\033[0;32m'; YELLOW='\033[0;33m'; RED='\033[0;31m'; BLUE='\033[0;34m'; NC='\033[0m'

# Pinned per L-09; override to move the pin deliberately.
JUST_VERSION="${JUST_VERSION:-1.58.0}"
JUST_INSTALL_DIR="${JUST_INSTALL_DIR:-$HOME/.local/bin}"

MODE="install"      # install | status | verify | plan
ASSUME_YES=0
ONLY=""
SKIP=""
LOG_FILE=""

usage() {
    cat <<'EOF'
LCTK development environment setup

Usage:
  ./setup.sh                    Pick steps in a selector, then install
  ./setup.sh --yes              Install the default selection, no prompts
  ./setup.sh --only a,b         Install just these steps (and their dependencies)
  ./setup.sh --skip cuda        Install the default selection minus these
  ./setup.sh --status           Show what is installed, checked against the machine
  ./setup.sh --verify           Run every verifier; non-zero exit if one fails
  ./setup.sh --dry-run          Print the execution order and stop
  ./setup.sh --log FILE         Tee a full transcript to FILE
  ./setup.sh --list             List step ids
  ./setup.sh --clean [step...]  Clear markers so steps re-run

Steps and their dependencies are defined in setup/steps.py. Individual steps also
remain available as `just -f setup/justfile <step>`.
EOF
}

cleanup() {
    local code=$?
    if [[ -n "$SUDO_LOOP_PID" ]] && kill -0 "$SUDO_LOOP_PID" 2>/dev/null; then
        kill "$SUDO_LOOP_PID" 2>/dev/null || true
    fi
    [[ $code -eq 130 ]] && printf "\n${YELLOW}Cancelled by user${NC}\n" >&2
    return 0
}
trap cleanup EXIT
trap 'exit 130' INT TERM

ask_yes_no() {
    local question="$1" default="${2:-y}" prompt response
    [[ "$default" == "y" ]] && prompt="[Y/n]" || prompt="[y/N]"
    if [[ $ASSUME_YES -eq 1 || ! -t 0 ]]; then
        [[ "$default" == "y" ]]
        return
    fi
    while true; do
        printf "${BLUE}?${NC} %s %s " "$question" "$prompt"
        read -r response || { printf "\n"; exit 130; }
        response="${response:-$default}"
        case "${response,,}" in
            y|yes) return 0 ;;
            n|no)  return 1 ;;
            *) printf "${RED}Please answer y or n.${NC}\n" ;;
        esac
    done
}

# `just` has no apt package on jammy, and `cargo install just` needs the `rust` step
# that this script has not run yet -- so the gate offers a prebuilt binary instead. The
# upstream installer resolves x86_64 and aarch64 musl targets, so it covers the
# workstations and the Jetson hosts with no toolchain.
ensure_just() {
    if command -v just >/dev/null 2>&1; then
        return 0
    fi

    printf "${YELLOW}!${NC} 'just' is required and was not found.\n\n"
    printf "  Recommended (prebuilt static binary, no cargo, no apt):\n"
    printf "    ${BLUE}mkdir -p %s${NC}\n" "$JUST_INSTALL_DIR"
    printf "    ${BLUE}curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh \\\\${NC}\n"
    printf "      ${BLUE}| bash -s -- --tag %s --to %s${NC}\n\n" "$JUST_VERSION" "$JUST_INSTALL_DIR"
    printf "  Offline alternative (direct release tarball):\n"
    printf "    ${BLUE}curl -fsSL https://github.com/casey/just/releases/download/%s/just-%s-\$(uname -m)-unknown-linux-musl.tar.gz \\\\${NC}\n" "$JUST_VERSION" "$JUST_VERSION"
    printf "      ${BLUE}| tar -xz -C %s just${NC}\n\n" "$JUST_INSTALL_DIR"
    printf "  Not recommended: 'cargo install just' needs cargo, which this setup\n"
    printf "  installs later; there is no 'just' apt package on Ubuntu 22.04.\n\n"

    if ! ask_yes_no "Install just ${JUST_VERSION} now?" "y"; then
        printf "${RED}x${NC} Cannot continue without just.\n"
        exit 1
    fi

    bash "${SETUP_DIR}/scripts/install-just.sh" || exit 1

    # ~/.profile only adds ~/.local/bin to PATH when the directory existed at login, so
    # a fresh machine needs it on PATH for the rest of this run.
    export PATH="${JUST_INSTALL_DIR}:${PATH}"
    if ! command -v just >/dev/null 2>&1; then
        printf "${RED}x${NC} just still not on PATH after install.\n"
        exit 1
    fi
    printf "${GREEN}v${NC} just %s installed\n\n" "$(just --version)"
}

check_os() {
    command -v lsb_release >/dev/null 2>&1 || return 0
    local version
    version="$(lsb_release -rs)"
    [[ "$version" == "22.04" ]] && return 0
    printf "${YELLOW}Warning:${NC} designed for Ubuntu 22.04 LTS; this is %s\n" "$version"
    ask_yes_no "Continue anyway?" "n" || exit 1
}

start_sudo_loop() {
    if ! sudo -n true 2>/dev/null; then
        printf "${YELLOW}->${NC} Requesting sudo privileges...\n"
        sudo -v || { printf "${RED}x${NC} Failed to obtain sudo credentials\n"; exit 1; }
    fi
    ( while true; do sudo -n true; sleep 50; done ) </dev/null >/dev/null 2>&1 &
    SUDO_LOOP_PID=$!
    disown "$SUDO_LOOP_PID" 2>/dev/null || true
}

select_steps() {
    # --only wins outright; otherwise start from the defaults and subtract --skip.
    if [[ -n "$ONLY" ]]; then
        tr ',' '\n' <<<"$ONLY" | sed '/^$/d'
        return
    fi

    local chosen
    if [[ $ASSUME_YES -eq 0 && -t 0 && -t 1 ]]; then
        chosen="$(mktemp)"
        if ! (cd "$SETUP_DIR" && python3 tui.py --out "$chosen"); then
            rm -f "$chosen"
            printf "${YELLOW}Cancelled${NC}\n" >&2
            exit 0
        fi
        cat "$chosen"
        rm -f "$chosen"
    else
        python3 "$STEPS" plan
    fi | {
        if [[ -n "$SKIP" ]]; then
            grep -vxF -f <(tr ',' '\n' <<<"$SKIP" | sed '/^$/d') || true
        else
            cat
        fi
    }
}

main() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)   usage; exit 0 ;;
            -y|--yes)    ASSUME_YES=1; shift ;;
            --only)      ONLY="$2"; shift 2 ;;
            --skip)      SKIP="$2"; shift 2 ;;
            --status)    MODE="status"; shift ;;
            --verify)    MODE="verify"; shift ;;
            --dry-run)   MODE="plan"; shift ;;
            --log)       LOG_FILE="$2"; shift 2 ;;
            --list)      python3 "$STEPS" list; exit $? ;;
            --clean)     shift; python3 "$STEPS" clean "$@"; exit $? ;;
            *)           printf "${RED}x${NC} unknown option: %s\n\n" "$1"; usage; exit 2 ;;
        esac
    done

    cd "$PROJECT_ROOT"

    if [[ -n "$LOG_FILE" ]]; then
        exec > >(tee -a "$LOG_FILE") 2>&1
        printf "logging to %s\n" "$LOG_FILE"
    fi

    case "$MODE" in
        status) python3 "$STEPS" status; exit $? ;;
        verify) python3 "$STEPS" verify; exit $? ;;
    esac

    check_os
    ensure_just

    local steps
    steps="$(select_steps)"
    if [[ -z "$steps" ]]; then
        printf "${YELLOW}Nothing selected.${NC}\n"
        exit 0
    fi

    local plan
    plan="$(python3 "$STEPS" plan $steps)" || exit 1

    printf "\n${BLUE}Plan${NC} (%s steps, dependencies included):\n" "$(wc -l <<<"$plan")"
    sed 's/^/  /' <<<"$plan"
    printf "\n"

    if [[ "$MODE" == "plan" ]]; then
        exit 0
    fi

    ask_yes_no "Proceed?" "y" || { printf "${YELLOW}Cancelled${NC}\n"; exit 0; }
    start_sudo_loop

    local failed=""
    while read -r step; do
        [[ -z "$step" ]] && continue
        if ! python3 "$STEPS" run "$step"; then
            failed="$step"
            break
        fi
    done <<<"$plan"

    if [[ -n "$failed" ]]; then
        printf "\n${RED}Setup failed at step '%s'.${NC}\n" "$failed"
        printf "Fix the cause and re-run ./setup.sh -- completed steps are skipped.\n"
        exit 1
    fi

    printf "\n${BLUE}Verifying...${NC}\n"
    if ! python3 "$STEPS" verify $steps; then
        exit 1
    fi

    printf "\n${GREEN}Setup complete.${NC}\n\n"
    printf "Next steps:\n"
    printf "  1. Reload your shell:  ${BLUE}source ~/.bashrc${NC}\n"
    printf "  2. Build the project:  ${BLUE}just build${NC}\n"
    printf "  3. Run the tests:      ${BLUE}just test${NC}\n"
}

main "$@"
