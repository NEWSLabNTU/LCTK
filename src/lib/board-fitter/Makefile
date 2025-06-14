# Board-Fitter Makefile

.PHONY: all test bench setup-data clean help

# Default target
all: test

# Run all tests
test: test-unit test-integration

# Run unit tests only
test-unit:
	@echo "Running unit tests..."
	cargo test --lib

# Run integration tests
test-integration: setup-data
	@echo "Running integration tests..."
	cargo test --test test_e2e_pipeline
	cargo test --test test_external_data

# Run benchmarks
bench: setup-data
	@echo "Running benchmarks..."
	cargo bench

# Setup test data
setup-data:
	@echo "Setting up test data..."
	@if [ ! -d "test_data/external/pcl" ]; then \
		echo "Downloading external test data..."; \
		./scripts/download_test_data.sh; \
	else \
		echo "Test data already exists"; \
	fi

# Download test data manually
download-data:
	@echo "Downloading test data..."
	./scripts/download_test_data.sh

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

# Run tests with external data
test-external: setup-data
	@echo "Running tests with external datasets..."
	cargo test --test test_external_data -- --nocapture

# Run specific benchmark
bench-%: setup-data
	@echo "Running benchmark: $*"
	cargo bench --bench detection_benchmark -- --filter $*

# Generate documentation
docs:
	@echo "Generating documentation..."
	cargo doc --open

# Format code
fmt:
	@echo "Formatting code..."
	cargo fmt

# Lint code
lint:
	@echo "Linting code..."
	cargo clippy -- -D warnings

# Run all quality checks
check: fmt lint test

# Show help
help:
	@echo "Board-Fitter Build System"
	@echo "========================="
	@echo ""
	@echo "Available targets:"
	@echo "  all          - Run all tests (default)"
	@echo "  test         - Run unit and integration tests"
	@echo "  test-unit    - Run unit tests only"
	@echo "  test-integration - Run integration tests"
	@echo "  test-external - Run tests with external datasets"
	@echo "  bench        - Run performance benchmarks"
	@echo "  bench-PATTERN - Run specific benchmark"
	@echo "  setup-data   - Download and setup test data"
	@echo "  download-data - Force download test data"
	@echo "  clean        - Clean all build artifacts and test data"
	@echo "  clean-data   - Clean test data only"
	@echo "  docs         - Generate documentation"
	@echo "  fmt          - Format code"
	@echo "  lint         - Lint code"
	@echo "  check        - Run all quality checks"
	@echo "  help         - Show this help"
	@echo ""
	@echo "Examples:"
	@echo "  make test              # Run all tests"
	@echo "  make bench-detection   # Run detection benchmarks"
	@echo "  make test-external     # Test with real datasets"