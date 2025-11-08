#!/bin/bash

# Integration test orchestration script for multi_wayside_node
# This script runs comprehensive integration tests with different scenarios

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
TEST_DATA_DIR="$PROJECT_DIR/test_data"
RESULTS_DIR="$PROJECT_DIR/test_results"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test scenarios
SCENARIOS=("scenario_1_perfect_boards" "scenario_2_noisy_data" "scenario_3_partial_occlusion" "scenario_4_multi_boards")

# Initialize
echo -e "${YELLOW}🚀 Starting multi_wayside_node integration tests${NC}"
echo "Project directory: $PROJECT_DIR"
echo "Test data directory: $TEST_DATA_DIR"
echo "Results directory: $RESULTS_DIR"

# Create directories
mkdir -p "$TEST_DATA_DIR"
mkdir -p "$RESULTS_DIR"

# Generate test data if needed
echo -e "${YELLOW}📊 Generating test data...${NC}"
MISSING_SCENARIOS=()
for scenario in "${SCENARIOS[@]}"; do
    if [ ! -d "$TEST_DATA_DIR/$scenario" ]; then
        MISSING_SCENARIOS+=("$scenario")
    fi
done

if [ ${#MISSING_SCENARIOS[@]} -gt 0 ]; then
    echo "Generating missing test scenarios: ${MISSING_SCENARIOS[*]}"
    cd "$PROJECT_DIR"
    
    # Map scenario names to generator arguments
    GENERATOR_ARGS=""
    for scenario in "${MISSING_SCENARIOS[@]}"; do
        case $scenario in
            scenario_1_perfect_boards)
                GENERATOR_ARGS="$GENERATOR_ARGS perfect"
                ;;
            scenario_2_noisy_data)
                GENERATOR_ARGS="$GENERATOR_ARGS noisy"
                ;;
            scenario_3_partial_occlusion)
                GENERATOR_ARGS="$GENERATOR_ARGS occlusion"
                ;;
            scenario_4_multi_boards)
                GENERATOR_ARGS="$GENERATOR_ARGS multi"
                ;;
        esac
    done
    
    python3 scripts/generate_test_data.py --output_dir "$TEST_DATA_DIR" --scenarios $GENERATOR_ARGS
else
    echo "All test data already exists, skipping generation"
fi

# Function to run a single test scenario
run_test_scenario() {
    local scenario=$1
    local test_name=$2
    local timeout=${3:-120}
    
    echo -e "${YELLOW}🧪 Running test: $test_name${NC}"
    
    # Create result file
    local result_file="$RESULTS_DIR/${test_name}_result.log"
    local success_file="$RESULTS_DIR/${test_name}_success.flag"
    
    # Remove old results
    rm -f "$result_file" "$success_file"
    
    # Start test in background
    (
        echo "=== Test: $test_name ===" > "$result_file"
        echo "Scenario: $scenario" >> "$result_file"
        echo "Started: $(date)" >> "$result_file"
        echo "" >> "$result_file"
        
        # Launch the test
        cd "$PROJECT_DIR"
        timeout $timeout ros2 launch launch/test_basic_calibration.launch.py \
            test_bag:="$scenario.bag" \
            use_rviz:=false \
            auto_validate:=true \
            2>&1 | tee -a "$result_file"
        
        # Check if validation passed
        if [ ${PIPESTATUS[0]} -eq 0 ]; then
            echo "SUCCESS" > "$success_file"
            echo "✅ Test completed successfully" >> "$result_file"
        else
            echo "FAILED" > "$success_file"
            echo "❌ Test failed" >> "$result_file"
        fi
        
        echo "Finished: $(date)" >> "$result_file"
    ) &
    
    local test_pid=$!
    
    # Wait for test completion
    if wait $test_pid; then
        if [ -f "$success_file" ] && [ "$(cat "$success_file")" = "SUCCESS" ]; then
            echo -e "${GREEN}✅ $test_name: PASSED${NC}"
            return 0
        else
            echo -e "${RED}❌ $test_name: FAILED${NC}"
            return 1
        fi
    else
        echo -e "${RED}⏰ $test_name: TIMEOUT or ERROR${NC}"
        return 1
    fi
}

# Function to run visual test with RViz
run_visual_test() {
    local scenario=$1
    
    echo -e "${YELLOW}👁️  Running visual test with RViz for $scenario${NC}"
    echo "This will launch RViz for manual inspection of calibration process"
    echo "Press CTRL+C to stop when satisfied with visual validation"
    
    cd "$PROJECT_DIR"
    ros2 launch launch/test_basic_calibration.launch.py \
        test_bag:="$scenario.bag" \
        use_rviz:=true \
        auto_validate:=false \
        || true
}

# Main test execution
main() {
    local run_visual=false
    local selected_scenarios=()
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --visual)
                run_visual=true
                shift
                ;;
            --scenario)
                selected_scenarios+=("$2")
                shift 2
                ;;
            --help)
                echo "Usage: $0 [--visual] [--scenario SCENARIO]"
                echo "Options:"
                echo "  --visual          Run visual tests with RViz"
                echo "  --scenario NAME   Run specific scenario only"
                echo "  --help            Show this help"
                exit 0
                ;;
            *)
                echo "Unknown option: $1"
                exit 1
                ;;
        esac
    done
    
    # Use all scenarios if none specified
    if [ ${#selected_scenarios[@]} -eq 0 ]; then
        selected_scenarios=("${SCENARIOS[@]}")
    fi
    
    local total_tests=0
    local passed_tests=0
    
    # Run automated tests
    if [ "$run_visual" = false ]; then
        echo -e "${YELLOW}🤖 Running automated integration tests${NC}"
        
        for scenario in "${selected_scenarios[@]}"; do
            if [ -d "$TEST_DATA_DIR/$scenario" ]; then
                total_tests=$((total_tests + 1))
                if run_test_scenario "$scenario" "automated_${scenario}" 120; then
                    passed_tests=$((passed_tests + 1))
                fi
            else
                echo -e "${YELLOW}⚠️  Skipping $scenario (test data not found)${NC}"
            fi
        done
        
        # Report results
        echo ""
        echo -e "${YELLOW}📊 Test Results Summary${NC}"
        echo "========================"
        echo "Total tests: $total_tests"
        echo "Passed: $passed_tests"
        echo "Failed: $((total_tests - passed_tests))"
        
        if [ $passed_tests -eq $total_tests ] && [ $total_tests -gt 0 ]; then
            echo -e "${GREEN}🎉 All tests passed!${NC}"
            exit 0
        else
            echo -e "${RED}💥 Some tests failed!${NC}"
            echo ""
            echo "Check individual test logs in $RESULTS_DIR for details"
            exit 1
        fi
    else
        # Run visual tests
        echo -e "${YELLOW}👁️  Running visual tests${NC}"
        for scenario in "${selected_scenarios[@]}"; do
            if [ -d "$TEST_DATA_DIR/$scenario" ]; then
                run_visual_test "$scenario"
            fi
        done
    fi
}

# Run main function with all arguments
main "$@"