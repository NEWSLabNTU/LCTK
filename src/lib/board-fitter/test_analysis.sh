#!/bin/bash
# Comprehensive test analysis for board-fitter

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Function to get detailed logs for a test
get_test_logs() {
    local test_name="$1"
    local log_level="${2:-info}"
    
    RUST_LOG="$log_level" cargo test --test test_e2e_pipeline "$test_name" -- --nocapture 2>&1
}

# Function to analyze plane detection
analyze_planes() {
    local test_name="$1"
    echo -e "${CYAN}=== Plane Detection Analysis for $test_name ===${NC}"
    
    get_test_logs "$test_name" "board_fitter::plane=debug" | \
        grep -E "(detect_planes|planes_detected|Plane detection|normal=|size=|inliers=)" | \
        head -20
}

# Function to analyze ICP performance
analyze_icp() {
    local test_name="$1"
    echo -e "${CYAN}=== ICP Performance Analysis for $test_name ===${NC}"
    
    get_test_logs "$test_name" "board_fitter::refinement=debug" | \
        grep -E "(ICP|register_advanced|iterations|converged|fitness|refinement)" | \
        head -20
}

# Function to analyze timeouts
analyze_timeouts() {
    local test_name="$1"
    echo -e "${CYAN}=== Timeout Analysis for $test_name ===${NC}"
    
    get_test_logs "$test_name" "info" | \
        grep -E "(timeout|Timeout|elapsed|Duration|ms\)|seconds)" | \
        head -20
}

# Function to analyze detection pipeline
analyze_pipeline() {
    local test_name="$1"
    echo -e "${CYAN}=== Detection Pipeline Analysis for $test_name ===${NC}"
    
    get_test_logs "$test_name" "info" | \
        grep -E "(Starting|Plane detection|Diamond fitting|Hole detection|Pattern|Validation|Detection pipeline)" | \
        head -30
}

# Function to get test assertions
analyze_assertions() {
    local test_name="$1"
    echo -e "${CYAN}=== Test Assertions for $test_name ===${NC}"
    
    get_test_logs "$test_name" "info" | \
        grep -E "(assert|should|expected|actual|error|Error|FAIL|panicked)" | \
        head -10
}

# Function to compare test scenarios
compare_scenarios() {
    echo -e "${MAGENTA}=== Test Scenario Comparison ===${NC}"
    echo ""
    
    # Get timing for each test
    local tests=("test_perfect_board_detection" "test_noisy_board_detection" "test_partial_occlusion" 
                 "test_extreme_poses" "test_multi_board_scene" "test_varying_distances")
    
    for test in "${tests[@]}"; do
        echo -n -e "${BLUE}$test:${NC} "
        
        # Try to run test and get result
        if timeout 15s cargo test --test test_e2e_pipeline "$test" -- --quiet 2>&1 >/dev/null; then
            echo -e "${GREEN}PASS${NC}"
        else
            echo -e "${RED}FAIL${NC}"
        fi
    done
}

# Function to get performance metrics
get_performance_metrics() {
    local test_name="$1"
    echo -e "${CYAN}=== Performance Metrics for $test_name ===${NC}"
    
    get_test_logs "$test_name" "info" | \
        grep -E "([0-9]+\.?[0-9]*\s*ms|processing time|elapsed|duration)" | \
        head -10
}

# Function to analyze specific failure
deep_dive() {
    local test_name="$1"
    
    echo -e "${MAGENTA}=== Deep Dive Analysis: $test_name ===${NC}"
    echo ""
    
    # Check basic info
    echo -e "${YELLOW}1. Basic Test Info:${NC}"
    analyze_assertions "$test_name"
    echo ""
    
    # Check pipeline stages
    echo -e "${YELLOW}2. Pipeline Stages:${NC}"
    analyze_pipeline "$test_name"
    echo ""
    
    # Check plane detection
    echo -e "${YELLOW}3. Plane Detection:${NC}"
    analyze_planes "$test_name"
    echo ""
    
    # Check timeouts
    echo -e "${YELLOW}4. Timing Analysis:${NC}"
    analyze_timeouts "$test_name"
    echo ""
    
    # Check ICP if relevant
    echo -e "${YELLOW}5. ICP Analysis:${NC}"
    analyze_icp "$test_name"
}

# Function to generate report
generate_report() {
    local output_file="${1:-test_report.txt}"
    
    echo -e "${BLUE}[REPORT]${NC} Generating comprehensive test report..."
    
    {
        echo "Board Fitter Test Analysis Report"
        echo "Generated: $(date)"
        echo "========================================"
        echo ""
        
        echo "Test Status Summary:"
        echo "-------------------"
        compare_scenarios
        echo ""
        
        echo "Failing Test Analysis:"
        echo "---------------------"
        
        # Analyze each failing test
        local failing_tests=("test_multi_board_scene" "test_noisy_board_detection" "test_varying_distances")
        
        for test in "${failing_tests[@]}"; do
            echo ""
            echo "### $test ###"
            deep_dive "$test" 2>&1 | head -50
            echo ""
        done
        
    } > "$output_file"
    
    echo -e "${GREEN}[DONE]${NC} Report saved to $output_file"
}

# Main script logic
case "${1:-help}" in
    "planes")
        if [ -z "$2" ]; then
            echo "Usage: $0 planes <test_name>"
            exit 1
        fi
        analyze_planes "$2"
        ;;
    "icp")
        if [ -z "$2" ]; then
            echo "Usage: $0 icp <test_name>"
            exit 1
        fi
        analyze_icp "$2"
        ;;
    "timeout")
        if [ -z "$2" ]; then
            echo "Usage: $0 timeout <test_name>"
            exit 1
        fi
        analyze_timeouts "$2"
        ;;
    "pipeline")
        if [ -z "$2" ]; then
            echo "Usage: $0 pipeline <test_name>"
            exit 1
        fi
        analyze_pipeline "$2"
        ;;
    "assertions")
        if [ -z "$2" ]; then
            echo "Usage: $0 assertions <test_name>"
            exit 1
        fi
        analyze_assertions "$2"
        ;;
    "compare")
        compare_scenarios
        ;;
    "metrics")
        if [ -z "$2" ]; then
            echo "Usage: $0 metrics <test_name>"
            exit 1
        fi
        get_performance_metrics "$2"
        ;;
    "deep")
        if [ -z "$2" ]; then
            echo "Usage: $0 deep <test_name>"
            exit 1
        fi
        deep_dive "$2"
        ;;
    "report")
        generate_report "${2:-test_report.txt}"
        ;;
    "help")
        echo "Board Fitter Test Analysis Tool"
        echo ""
        echo "Usage: $0 [command] [options]"
        echo ""
        echo "Commands:"
        echo "  planes <test>     Analyze plane detection"
        echo "  icp <test>        Analyze ICP performance"
        echo "  timeout <test>    Analyze timeout issues"
        echo "  pipeline <test>   Analyze detection pipeline"
        echo "  assertions <test> Show test assertions/failures"
        echo "  compare           Compare all test scenarios"
        echo "  metrics <test>    Get performance metrics"
        echo "  deep <test>       Deep dive analysis"
        echo "  report [file]     Generate comprehensive report"
        echo "  help              Show this help message"
        echo ""
        echo "Examples:"
        echo "  $0 deep test_multi_board_scene"
        echo "  $0 report analysis.txt"
        ;;
    *)
        echo -e "${RED}Unknown command: $1${NC}"
        echo "Run '$0 help' for usage information"
        exit 1
        ;;
esac