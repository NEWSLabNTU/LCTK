#!/bin/bash
# Debug helper for board-fitter tests

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to analyze test failure
analyze_failure() {
    local test_name="$1"
    echo -e "${BLUE}[ANALYZE]${NC} Analyzing failure for $test_name..."
    
    # Run with different log levels to pinpoint issue
    echo -e "${CYAN}Step 1: Checking basic execution...${NC}"
    if ! cargo test --test test_e2e_pipeline "$test_name" -- --quiet 2>&1 >/dev/null; then
        echo -e "${YELLOW}  Test fails at basic level${NC}"
    fi
    
    echo -e "${CYAN}Step 2: Checking with info logs...${NC}"
    RUST_LOG=info cargo test --test test_e2e_pipeline "$test_name" -- --nocapture 2>&1 | grep -E "(INFO|WARN|ERROR|planes|detections|boards)" | head -20
    
    echo -e "${CYAN}Step 3: Checking plane detection...${NC}"
    RUST_LOG=board_fitter=debug cargo test --test test_e2e_pipeline "$test_name" -- --nocapture 2>&1 | grep -i "plane" | head -10
    
    echo -e "${CYAN}Step 4: Checking timeout issues...${NC}"
    RUST_LOG=board_fitter=debug cargo test --test test_e2e_pipeline "$test_name" -- --nocapture 2>&1 | grep -i "timeout" | head -5
}

# Function to trace execution flow
trace_execution() {
    local test_name="$1"
    echo -e "${BLUE}[TRACE]${NC} Tracing execution flow for $test_name..."
    
    RUST_LOG=board_fitter=trace cargo test --test test_e2e_pipeline "$test_name" -- --nocapture 2>&1 | \
        grep -E "(Starting|Processing|Found|Detected|Success|Failed)" | \
        head -50
}

# Function to check memory usage
check_memory() {
    local test_name="$1"
    echo -e "${BLUE}[MEMORY]${NC} Checking memory usage for $test_name..."
    
    # Use /usr/bin/time if available
    if command -v /usr/bin/time &> /dev/null; then
        /usr/bin/time -v cargo test --test test_e2e_pipeline "$test_name" -- --quiet 2>&1 | \
            grep -E "(Maximum resident|User time|System time)"
    else
        echo -e "${YELLOW}  /usr/bin/time not available for detailed memory stats${NC}"
    fi
}

# Function to compare passing vs failing tests
compare_tests() {
    local passing_test="test_perfect_board_detection"
    local failing_test="$1"
    
    echo -e "${BLUE}[COMPARE]${NC} Comparing $passing_test (passing) vs $failing_test (failing)..."
    
    echo -e "${CYAN}Passing test output:${NC}"
    RUST_LOG=info cargo test --test test_e2e_pipeline "$passing_test" -- --nocapture 2>&1 | \
        grep -E "(planes|boards|detections)" | head -5
    
    echo -e "${CYAN}Failing test output:${NC}"
    RUST_LOG=info cargo test --test test_e2e_pipeline "$failing_test" -- --nocapture 2>&1 | \
        grep -E "(planes|boards|detections)" | head -5
}

# Function to run with backtrace
debug_backtrace() {
    local test_name="$1"
    echo -e "${BLUE}[BACKTRACE]${NC} Running $test_name with backtrace..."
    
    RUST_BACKTRACE=1 cargo test --test test_e2e_pipeline "$test_name" -- --nocapture
}

# Function to check specific issue
check_issue() {
    local test_name="$1"
    local issue="$2"
    
    echo -e "${BLUE}[CHECK]${NC} Checking $test_name for issue: $issue"
    
    case "$issue" in
        "planes")
            echo "Checking plane detection..."
            RUST_LOG=board_fitter::plane=debug cargo test --test test_e2e_pipeline "$test_name" -- --nocapture 2>&1 | \
                grep -A 2 -B 2 "detect_planes"
            ;;
        "timeout")
            echo "Checking for timeouts..."
            RUST_LOG=debug cargo test --test test_e2e_pipeline "$test_name" -- --nocapture 2>&1 | \
                grep -i -E "(timeout|elapsed|duration)"
            ;;
        "icp")
            echo "Checking ICP refinement..."
            RUST_LOG=board_fitter::refinement=debug cargo test --test test_e2e_pipeline "$test_name" -- --nocapture 2>&1 | \
                grep -i "icp"
            ;;
        "memory")
            echo "Checking memory allocation..."
            RUST_LOG=debug cargo test --test test_e2e_pipeline "$test_name" -- --nocapture 2>&1 | \
                grep -i -E "(alloc|memory|oom)"
            ;;
        *)
            echo -e "${RED}Unknown issue type: $issue${NC}"
            ;;
    esac
}

# Main script logic
case "${1:-help}" in
    "analyze")
        if [ -z "$2" ]; then
            echo "Usage: $0 analyze <test_name>"
            exit 1
        fi
        analyze_failure "$2"
        ;;
    "trace")
        if [ -z "$2" ]; then
            echo "Usage: $0 trace <test_name>"
            exit 1
        fi
        trace_execution "$2"
        ;;
    "memory")
        if [ -z "$2" ]; then
            echo "Usage: $0 memory <test_name>"
            exit 1
        fi
        check_memory "$2"
        ;;
    "compare")
        if [ -z "$2" ]; then
            echo "Usage: $0 compare <failing_test_name>"
            exit 1
        fi
        compare_tests "$2"
        ;;
    "backtrace")
        if [ -z "$2" ]; then
            echo "Usage: $0 backtrace <test_name>"
            exit 1
        fi
        debug_backtrace "$2"
        ;;
    "check")
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Usage: $0 check <test_name> <issue_type>"
            echo "Issue types: planes, timeout, icp, memory"
            exit 1
        fi
        check_issue "$2" "$3"
        ;;
    "help")
        echo "Board Fitter Test Debug Helper"
        echo ""
        echo "Usage: $0 [command] [options]"
        echo ""
        echo "Commands:"
        echo "  analyze <test>       Analyze test failure with multiple log levels"
        echo "  trace <test>         Trace execution flow"
        echo "  memory <test>        Check memory usage"
        echo "  compare <test>       Compare failing test with passing test"
        echo "  backtrace <test>     Run with Rust backtrace"
        echo "  check <test> <issue> Check for specific issue (planes/timeout/icp/memory)"
        echo "  help                 Show this help message"
        echo ""
        echo "Example:"
        echo "  $0 analyze test_multi_board_scene"
        echo "  $0 check test_varying_distances timeout"
        ;;
    *)
        echo -e "${RED}Unknown command: $1${NC}"
        echo "Run '$0 help' for usage information"
        exit 1
        ;;
esac