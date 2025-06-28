#!/bin/bash
# Integration test runner for board-fitter

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test categories
INTEGRATION_TESTS=(
    "test_perfect_board_detection"
    "test_noisy_board_detection"
    "test_partial_occlusion"
    "test_extreme_poses"
    "test_multi_board_scene"
    "test_varying_distances"
)

# Function to run a single test
run_single_test() {
    local test_name="$1"
    local verbose="$2"
    
    echo -e "${BLUE}[TEST]${NC} Running $test_name..."
    
    if [ "$verbose" = "true" ]; then
        if RUST_LOG=info cargo test --test test_e2e_pipeline "$test_name" -- --nocapture 2>&1; then
            echo -e "${GREEN}[PASS]${NC} $test_name"
            return 0
        else
            echo -e "${RED}[FAIL]${NC} $test_name"
            return 1
        fi
    else
        if cargo test --test test_e2e_pipeline "$test_name" -- --quiet 2>&1 >/dev/null; then
            echo -e "${GREEN}[PASS]${NC} $test_name"
            return 0
        else
            echo -e "${RED}[FAIL]${NC} $test_name"
            return 1
        fi
    fi
}

# Function to run all tests
run_all_tests() {
    local verbose="$1"
    local passed=0
    local failed=0
    
    echo -e "${BLUE}[INFO]${NC} Running all integration tests..."
    echo ""
    
    for test in "${INTEGRATION_TESTS[@]}"; do
        if run_single_test "$test" "$verbose"; then
            ((passed++))
        else
            ((failed++))
        fi
    done
    
    echo ""
    echo -e "${BLUE}[SUMMARY]${NC} Integration Test Results:"
    echo -e "  ${GREEN}Passed:${NC} $passed"
    echo -e "  ${RED}Failed:${NC} $failed"
    echo -e "  ${BLUE}Total:${NC} ${#INTEGRATION_TESTS[@]}"
    echo -e "  ${YELLOW}Pass Rate:${NC} $(( passed * 100 / ${#INTEGRATION_TESTS[@]} ))%"
    
    if [ $failed -eq 0 ]; then
        echo -e "${GREEN}[SUCCESS]${NC} All tests passed!"
        return 0
    else
        echo -e "${RED}[FAILURE]${NC} Some tests failed"
        return 1
    fi
}

# Function to debug a specific test
debug_test() {
    local test_name="$1"
    echo -e "${BLUE}[DEBUG]${NC} Running $test_name with debug logging..."
    RUST_LOG=debug cargo test --test test_e2e_pipeline "$test_name" -- --nocapture
}

# Function to show test status
show_status() {
    echo -e "${BLUE}[STATUS]${NC} Checking integration test status..."
    cargo test --test test_e2e_pipeline -- --list | grep "test " | wc -l | xargs -I {} echo -e "  Total tests: {}"
    
    local passing=0
    for test in "${INTEGRATION_TESTS[@]}"; do
        if cargo test --test test_e2e_pipeline "$test" -- --quiet 2>&1 >/dev/null; then
            ((passing++))
        fi
    done
    
    echo -e "  Passing: $passing/${#INTEGRATION_TESTS[@]}"
    echo -e "  Pass rate: $(( passing * 100 / ${#INTEGRATION_TESTS[@]} ))%"
}

# Function to run tests with performance metrics
perf_test() {
    local test_name="$1"
    echo -e "${BLUE}[PERF]${NC} Running $test_name with performance metrics..."
    
    # Run test and capture timing
    local start_time=$(date +%s.%N)
    if RUST_LOG=info cargo test --test test_e2e_pipeline "$test_name" -- --nocapture 2>&1 | grep -E "(ms|elapsed|time)"; then
        local end_time=$(date +%s.%N)
        local elapsed=$(echo "$end_time - $start_time" | bc)
        echo -e "${GREEN}[PERF]${NC} Test completed in ${elapsed}s"
    else
        echo -e "${RED}[PERF]${NC} Test failed or no timing info available"
    fi
}

# Main script logic
case "${1:-all}" in
    "all")
        run_all_tests "${2:-false}"
        ;;
    "verbose")
        run_all_tests "true"
        ;;
    "debug")
        if [ -z "$2" ]; then
            echo "Usage: $0 debug <test_name>"
            echo "Available tests:"
            for test in "${INTEGRATION_TESTS[@]}"; do
                echo "  - $test"
            done
            exit 1
        fi
        debug_test "$2"
        ;;
    "status")
        show_status
        ;;
    "perf")
        if [ -z "$2" ]; then
            echo "Usage: $0 perf <test_name>"
            exit 1
        fi
        perf_test "$2"
        ;;
    "help")
        echo "Board Fitter Integration Test Runner"
        echo ""
        echo "Usage: $0 [command] [options]"
        echo ""
        echo "Commands:"
        echo "  all              Run all integration tests (default)"
        echo "  verbose          Run all tests with output"
        echo "  debug <test>     Run a specific test with debug logging"
        echo "  status           Show current test status"
        echo "  perf <test>      Run test with performance metrics"
        echo "  help             Show this help message"
        echo ""
        echo "Available tests:"
        for test in "${INTEGRATION_TESTS[@]}"; do
            echo "  - $test"
        done
        ;;
    *)
        # Assume it's a test name
        run_single_test "$1" "${2:-false}"
        ;;
esac