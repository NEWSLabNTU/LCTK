.PHONY: default build build_ros2_rust build_interface build_packages clean prepare lint format

COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
LOG_DIR := build_logs

default: build

prepare:
	pip install git+https://github.com/colcon/colcon-cargo.git
	pip install git+https://github.com/colcon/colcon-ros-cargo.git

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
	@echo "Building Rust packages without ROS... (log: $(LOG_DIR)/rust_packages.log)"
	env -u CMAKE_PREFIX_PATH -u CMAKE_MODULE_PATH -u PKG_CONFIG_PATH \
	cargo build --release 2>&1 | tee $(LOG_DIR)/rust_packages.log
	@echo "Building Rust ROS nodes... (log: $(LOG_DIR)/rust_ros_nodes.log)"
	. ./ros2_rust_ws/install/setup.sh && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/bin --packages-select calibration_board_locator extrinsic_solver pointcloud_image_overlay synchronizer aruco_generator aruco_locator_service multi_wayside 2>&1 | tee $(LOG_DIR)/rust_ros_nodes.log
	@echo "Building C++ packages... (log: $(LOG_DIR)/cpp_packages.log)"
	. ./ros2_rust_ws/install/setup.sh && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/bin --packages-select rosbag_deck_core rosbag_deck_interface rosbag_deck_node rosbag_deck_python rosbag_deck_tui calib_launch multi_wayside_node 2>&1 | tee $(LOG_DIR)/cpp_packages.log

format:
	@echo "Formatting Rust code..."
	cargo +nightly fmt

lint:
	@echo "Checking Rust code formatting and linting..."
	cargo +nightly fmt --check
	cargo clippy --all-targets --all-features -- -D warnings

clean:
	rm -rf build install log target .cargo $(LOG_DIR)
