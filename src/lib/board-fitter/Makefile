# Board-Fitter Makefile
# Enhanced build system for board-fitter library

# Build configuration
RUST_VERSION ?= stable
CARGO_FLAGS ?= --release
TEST_FLAGS ?= --no-fail-fast

.PHONY: all test test-unit test-integration test-external bench bench-% setup-data download-data clean clean-data docs fmt lint lint-strict check install dev help

# Default target
all: check

# Run all tests
test:
	@echo "Running all tests..."
	cargo nextest run --all-targets $(TEST_FLAGS)

# Run unit tests only
test-unit:
	@echo "Running unit tests..."
	cargo nextest run --lib $(TEST_FLAGS)

# Run integration tests only
test-integration:
	@echo "Running integration tests..."
	cargo nextest run --test '*' $(TEST_FLAGS)

# Run tests with external datasets
test-external: setup-data
	@echo "Running tests with external datasets..."
	cargo nextest run --test 'test_external_*' $(TEST_FLAGS)

# Run all benchmarks
bench: setup-data
	@echo "Running all benchmarks..."
	cargo bench

# Run specific benchmark pattern
bench-%: setup-data
	@echo "Running benchmarks matching pattern: $*"
	cargo bench -- $*

# Setup test data
setup-data:
	@echo "Setting up test data..."
	@if [ ! -d "test_data/external/pcl" ]; then \
		echo "Downloading external test data..."; \
		if [ -x "./scripts/download_test_data.sh" ]; then \
			./scripts/download_test_data.sh; \
		else \
			echo "Warning: download_test_data.sh not found or not executable"; \
			mkdir -p test_data/external/pcl; \
		fi; \
	else \
		echo "Test data already exists"; \
	fi

# Download test data manually
download-data:
	@echo "Downloading test data..."
	@if [ -x "./scripts/download_test_data.sh" ]; then \
		./scripts/download_test_data.sh; \
	else \
		echo "Error: download_test_data.sh not found or not executable"; \
		exit 1; \
	fi

# Clean build artifacts and test data
clean:
	@echo "Cleaning build artifacts..."
	cargo clean
	rm -rf test_data/external/
	rm -rf target/criterion/

# Clean only test data
clean-data:
	@echo "Cleaning test data..."
	rm -rf test_data/external/

# Generate documentation
docs:
	@echo "Generating documentation..."
	cargo doc --open --document-private-items

# Install dependencies for development
dev:
	@echo "Setting up development environment..."
	@command -v cargo-nextest >/dev/null 2>&1 || { echo "Installing cargo-nextest..."; cargo install cargo-nextest; }
	@command -v cargo-criterion >/dev/null 2>&1 || { echo "Installing cargo-criterion..."; cargo install cargo-criterion; }
	@echo "Development environment ready!"

# Install the library (for use in other projects)
install:
	@echo "Installing board-fitter..."
	cargo install --path .

# Format code
fmt:
	@echo "Formatting code..."
	cargo +nightly fmt

# Lint code
lint:
	@echo "Linting code..."
	cargo clippy --all-targets --all-features

# Strict lint (treat warnings as errors)
lint-strict:
	@echo "Running strict linting..."
	cargo clippy --all-targets --all-features -- -D warnings

# Run all quality checks
check: 
	@echo "Running code formatting..."
	@$(MAKE) fmt
	@echo "Running linter..."
	@$(MAKE) lint
	@echo "Running tests..."
	@$(MAKE) test
	@echo "✅ All quality checks passed!"

# Show help
help:
	@echo "Board-Fitter Build System"
	@echo "========================="
	@echo ""
	@echo "Available targets:"
	@echo "  all          - Run all quality checks (default)"
	@echo "  test         - Run unit and integration tests"
	@echo "  test-unit    - Run unit tests only"
	@echo "  test-integration - Run integration tests"
	@echo "  test-external - Run tests with external datasets"
	@echo "  bench        - Run performance benchmarks"
	@echo "  bench-PATTERN - Run specific benchmark (e.g., make bench-detection)"
	@echo "  setup-data   - Download and setup test data"
	@echo "  download-data - Force download test data"
	@echo "  clean        - Clean all build artifacts and test data"
	@echo "  clean-data   - Clean test data only"
	@echo "  docs         - Generate documentation"
	@echo "  fmt          - Format code"
	@echo "  dev          - Setup development environment"
	@echo "  install      - Install library for system-wide use"
	@echo "  lint         - Lint code (warnings allowed)"
	@echo "  lint-strict  - Lint code (warnings as errors)"
	@echo "  check        - Run all quality checks"
	@echo "  help         - Show this help"
	@echo ""
	@echo "Examples:"
	@echo "  make test              # Run all tests"
	@echo "  make bench-detection   # Run detection benchmarks"
	@echo "  make test-external     # Test with real datasets"
	@echo "  make dev               # Setup development tools"
	@echo "  make TEST_FLAGS=--verbose test # Run tests with verbose output"
