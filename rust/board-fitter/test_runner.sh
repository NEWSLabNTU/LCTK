#!/bin/bash
# Test runner utility for board-fitter debugging

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[TEST]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Change to board-fitter directory
cd "$(dirname "$0")"
print_status "Working directory: $(pwd)"

case "${1:-help}" in
    "unit")
        print_status "Running unit tests..."
        cargo test --lib --no-fail-fast
        ;;
    
    "integration") 
        print_status "Running integration tests..."
        cargo test --test test_e2e_pipeline --no-fail-fast
        ;;
    
    "perfect")
        print_status "Running perfect board detection test..."
        timeout 30s cargo test test_perfect_board_detection -- --nocapture
        ;;
    
    "perfect-debug")
        print_status "Running perfect board detection test with debug output..."
        timeout 30s cargo test test_perfect_board_detection -- --nocapture 2>&1 | grep DEBUG
        ;;
    
    "perfect-no-icp")
        print_status "Running perfect board detection test without ICP..."
        # Temporarily modify test to disable ICP
        sed -i.bak 's/BoardDetectorBuilder::new(config)/BoardDetector::new(DetectionConfig::without_icp(config))/' tests/test_e2e_pipeline.rs
        sed -i.bak2 's/use board_fitter::BoardDetectorBuilder;/use board_fitter::{BoardDetectorBuilder, BoardDetector, DetectionConfig};/' tests/test_e2e_pipeline.rs
        timeout 30s cargo test test_perfect_board_detection -- --nocapture 2>&1 | grep DEBUG
        # Restore original
        mv tests/test_e2e_pipeline.rs.bak tests/test_e2e_pipeline.rs
        rm -f tests/test_e2e_pipeline.rs.bak2
        ;;
    
    "build")
        print_status "Building project..."
        cargo build --release
        ;;
    
    "check")
        print_status "Checking compilation..."
        cargo check
        ;;
    
    "lint")
        print_status "Running linter..."
        cargo clippy -- -D warnings
        ;;
    
    "all")
        print_status "Running all tests..."
        cargo test --no-fail-fast
        ;;
    
    "quick")
        print_status "Quick test run (unit + perfect board)..."
        echo "=== Unit Tests ==="
        cargo test --lib --no-fail-fast
        echo "=== Perfect Board Test ==="
        timeout 30s cargo test test_perfect_board_detection -- --nocapture
        ;;
    
    "debug-pipeline")
        print_status "Debugging detection pipeline performance..."
        echo "1. Running without ICP..."
        # Run test without ICP to establish baseline
        sed -i.bak 's/BoardDetectorBuilder::new(config)/BoardDetector::new(DetectionConfig::without_icp(config))/' tests/test_e2e_pipeline.rs
        sed -i.bak2 's/use board_fitter::BoardDetectorBuilder;/use board_fitter::{BoardDetectorBuilder, BoardDetector, DetectionConfig};/' tests/test_e2e_pipeline.rs
        timeout 30s cargo test test_perfect_board_detection -- --nocapture 2>&1 | grep -E "(DEBUG|elapsed|timeout)"
        mv tests/test_e2e_pipeline.rs.bak tests/test_e2e_pipeline.rs
        rm -f tests/test_e2e_pipeline.rs.bak2
        
        echo "2. Running with ICP..."
        timeout 30s cargo test test_perfect_board_detection -- --nocapture 2>&1 | grep -E "(DEBUG|elapsed|timeout)"
        ;;
    
    "icp-test")
        print_status "Testing ICP implementation in isolation..."
        cargo test --lib icp -- --nocapture
        ;;
    
    "clean")
        print_status "Cleaning build artifacts..."
        cargo clean
        ;;
    
    "status")
        print_status "Checking current test status..."
        echo "=== Compilation Check ==="
        if cargo check --quiet; then
            print_success "Compilation: PASS"
        else
            print_error "Compilation: FAIL"
        fi
        
        echo "=== Unit Tests ==="
        if timeout 10s cargo test --lib --quiet; then
            print_success "Unit tests: PASS"
        else
            print_error "Unit tests: FAIL"
        fi
        
        echo "=== Integration Tests ==="
        if timeout 30s cargo test --test test_e2e_pipeline --quiet; then
            print_success "Integration tests: PASS"
        else
            print_error "Integration tests: FAIL"
        fi
        ;;
    
    "progress")
        print_status "Checking progress against PROGRESS.md milestones..."
        echo "=== Test Counts ==="
        TOTAL_TESTS=$(cargo test --no-run 2>&1 | grep -o "[0-9]* test" | head -1 | cut -d' ' -f1 || echo "unknown")
        PASSING_UNIT=$(timeout 10s cargo test --lib --quiet 2>&1 | grep -o "[0-9]* passed" | cut -d' ' -f1 || echo "0")
        echo "Total tests: $TOTAL_TESTS"
        echo "Passing unit tests: $PASSING_UNIT"
        
        echo "=== Critical Milestones ==="
        if timeout 30s cargo test test_perfect_board_detection --quiet 2>/dev/null; then
            print_success "✓ First successful detection achieved"
        else
            print_error "✗ No successful detections yet"
        fi
        ;;
    
    "help"|*)
        echo "Board-fitter test runner utility"
        echo "Usage: $0 <command>"
        echo ""
        echo "Commands:"
        echo "  unit              - Run unit tests only"
        echo "  integration       - Run integration tests only" 
        echo "  perfect           - Run perfect board detection test"
        echo "  perfect-debug     - Run perfect board test with debug output"
        echo "  perfect-no-icp    - Run perfect board test without ICP"
        echo "  build             - Build the project"
        echo "  check             - Check compilation"
        echo "  lint              - Run linter"
        echo "  all               - Run all tests"
        echo "  quick             - Run unit tests + perfect board test"
        echo "  debug-pipeline    - Debug detection pipeline performance"
        echo "  icp-test          - Test ICP implementation in isolation"
        echo "  clean             - Clean build artifacts"
        echo "  status            - Check current test status"
        echo "  progress          - Check progress against PROGRESS.md milestones"
        echo "  help              - Show this help message"
        ;;
esac