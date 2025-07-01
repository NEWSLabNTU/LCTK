.PHONY: default build build_ros2_rust build_interface build_packages clean prepare lint format build_cargo

COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
LOG_DIR := build_logs

default: build

prepare:
	pip install -U git+https://github.com/colcon/colcon-cargo.git
	pip install -U git+https://github.com/colcon/colcon-ros-cargo.git

build: build_ros2_rust build_interface build_packages

build_ros2_rust:
	@mkdir -p $(LOG_DIR)
	@echo "Building ROS2 Rust packages... (log: $(LOG_DIR)/ros2_rust.log)"
	. /opt/ros/humble/setup.sh && \
	$(MAKE) -C ros2_rust_ws 2>&1 | tee $(LOG_DIR)/ros2_rust.log

build_interface:
	@mkdir -p $(LOG_DIR)
	@echo "Building interface packages... (log: $(LOG_DIR)/interface.log)"
	. ./ros2_rust_ws/install/setup.sh && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/interface 2>&1 | tee $(LOG_DIR)/interface.log

build_packages:
	@mkdir -p $(LOG_DIR)
	@echo "Building ROS nodes... (log: $(LOG_DIR)/packages.log)"
#	# CARGO_LOG=warn suppresses patch warnings that break colcon-cargo JSON parsing
#	# Without this, cargo metadata outputs patch warnings to stderr before JSON,
#	# causing colcon-cargo to fail with "JSONDecodeError: Expecting value: line 1 column 1"
	. install/setup.sh && \
	CARGO_LOG=warn colcon build $(COLCON_BUILD_FLAGS) --base-paths src/bin 2>&1 | tee $(LOG_DIR)/packages.log

build_cargo:
	cargo build --all-targets

format:
	@echo "Formatting Rust code..."
	cargo +nightly fmt

lint:
	@echo "Checking Rust code formatting and linting..."
	cargo +nightly fmt --check
	cargo clippy --all-targets --all-features -- -D warnings

clean:
	rm -rf build install log target .cargo $(LOG_DIR)
