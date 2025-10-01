COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --cargo-args --profile=test-release
COLCON_TEST_FLAGS := --ctest-args -C RelWithDebInfo --cargo-args --profile=test-release
LOG_DIR := build_logs
debug_mode := true
enable_icp_iteration_debug := true
enable_evaluator := true
enable_overlay := true
log_level := info
rviz := false
use_best_effort_qos := true
use_advanced_solver := false

## Topics for sample data
# camera_topic := /sensing/camera/front_center/image_raw
# pointcloud_topic := /sensing/lidar/top/pointcloud_raw

## Topics for AutoSDV data
camera_topic := /sensing/camera/zedxm/zed_node/left_raw/image_raw_color
pointcloud_topic := /sensing/lidar/concatenated/pointcloud

.PHONY: default
default: help

.PHONY: help
help:
	@echo "LCTK (LiDAR and Camera Toolkit) - Available targets:"
	@echo ""
	@echo "Setup & Environment:"
	@echo "  make setup                      - Set up development environment (installs all dependencies)"
	@echo "  make prepare                    - Install ROS dependencies with rosdep"
	@echo ""
	@echo "Build Commands:"
	@echo "  make build                      - Build entire project (all 3 passes)"
	@echo "  make build_ros2_rust            - Build ROS2 Rust base packages"
	@echo "  make build_interface            - Build interface types"
	@echo "  make build_packages             - Build ROS nodes"
	@echo ""
	@echo "Launch Commands (using ros2systemd for reliable service management):"
	@echo "  make launch_lidar_camera_sample_data - Create and start LiDAR-camera sample data service"
	@echo "  make stop_lidar_camera_sample_data   - Stop LiDAR-camera sample data service"
	@echo "  make launch_lidar_camera_calibration - Create and start LiDAR-Camera calibration pipeline (add debug_mode=true for debug topics, enable_evaluator=true for IoU metrics, log_level=debug for verbose logs, rviz=true for RViz, use_best_effort_qos=false for rosbag)"
	@echo "  make stop_lidar_camera_calibration   - Stop LiDAR-Camera calibration service"
	@echo "  make launch_two_lidar_calibration    - Create and start two LiDAR calibration service"
	@echo "  make stop_two_lidar_calibration      - Stop two LiDAR calibration service"
	@echo "  make launch_rviz                     - Launch RViz for calibration visualization"
	@echo ""
	@echo "Interactive Tools:"
	@echo "  make tune_filter_box                 - Launch interactive bbox filter tuner (for lidar_board_detector)"
	@echo ""
	@echo "Service Management:"
	@echo "  make service_status             - Show status of all LCTK services"
	@echo "  make service_logs               - Show logs for all LCTK services"
	@echo "  make service_cleanup            - Remove all LCTK systemd services"
	@echo ""
	@echo "Development Tools:"
	@echo "  make format                     - Format all code (Rust, Python, configs)"
	@echo "  make lint                       - Run linters and formatters check"
	@echo "  make test                       - Run all tests including ICP comparison tests"
	@echo "  make clean                      - Clean all build artifacts"
	@echo "  make launch_iou_overlapping        - Launch IoU overlapping evaluator (use extrinsic_json=/path/to/file.json to specify config)"
	@echo ""
	@echo "For more information, see README.md and CLAUDE.md"

.PHONY: setup
setup:
	@echo "Setting up LCTK development environment..."
	@echo "This will install all required dependencies using Ansible."
	@echo ""
	@./setup-dev-env.sh

.PHONY: prepare
prepare:
	@echo "Installing ROS dependencies with rosdep..."
	@. /opt/ros/humble/setup.sh && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -r -y

.PHONY: build
build: build_ros2_rust build_interface build_packages

.PHONY: build_ros2_rust
build_ros2_rust:
	@echo "Building ROS2 Rust packages... (log: $(LOG_DIR)/ros2_rust.log)"
	@mkdir -p $(LOG_DIR)
	@. /opt/ros/humble/setup.sh && \
	export RUST_LOG=debug && \
	$(MAKE) -C src/ros2_rust_ws 2>&1 | tee $(LOG_DIR)/ros2_rust.log

.PHONY: build_interface
build_interface:
	@echo "Building interface packages... (log: $(LOG_DIR)/interface.log)"
	@mkdir -p $(LOG_DIR)
	@. ./src/ros2_rust_ws/install/setup.sh && \
	export OPENCV_PKGCONFIG_NAME=opencv4 && \
	export RUST_LOG=debug && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/interface 2>&1 | tee $(LOG_DIR)/interface.log

.PHONY: build_packages
build_packages:
	@echo "Building ROS nodes... (log: $(LOG_DIR)/packages.log)"
	@mkdir -p $(LOG_DIR)
	@. install/setup.sh && \
	export OPENCV_PKGCONFIG_NAME=opencv4 && \
	export RUST_LOG=debug && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/ros2 2>&1 | tee $(LOG_DIR)/packages.log

.PHONY: format
format:
	@echo "Formatting Rust code..."
	@cargo +nightly fmt
	@echo "Formatting Python code..."
	@find . -name "*.py" -type f -not -path "./build/*" -not -path "./install/*" -not -path "./log/*" -not -path "./.venv/*" -not -path "./src/ros2_rust_ws/*" | xargs -r python3 -m black --quiet 2>/dev/null || true
	@find . -name "*.py" -type f -not -path "./build/*" -not -path "./install/*" -not -path "./log/*" -not -path "./.venv/*" -not -path "./src/ros2_rust_ws/*" | xargs -r python3 -m isort --quiet 2>/dev/null || true
	@echo "Removing trailing spaces in launch and config files..."
	@find . \( -name "*.launch.xml" -o -name "*.launch.py" -o -name "*.yaml" -o -name "*.yml" -o -name "*.json" -o -name "*.json5" \) -type f -not -path "./build/*" -not -path "./install/*" -not -path "./log/*" -not -path "./src/ros2_rust_ws/*" | xargs -r sed -i 's/[[:space:]]*$$//' 2>/dev/null || true
	@echo "Removing trailing spaces in Markdown files..."
	@find . -name "*.md" -type f -not -path "./build/*" -not -path "./install/*" -not -path "./log/*" -not -path "./src/ros2_rust_ws/*" | xargs -r sed -i 's/[[:space:]]*$$//' 2>/dev/null || true

.PHONY: lint
lint:
	@echo "Checking Rust code formatting and linting..."
	@cargo +nightly fmt --check
	@echo "Running clippy on workspace ..."
	@. install/setup.sh && \
	cargo clippy --workspace --all-targets --all-features || true

.PHONY: rust-test
rust-test:
	@echo "Running Rust library tests..."
	@. install/setup.sh && \
	export OPENCV_PKGCONFIG_NAME=opencv4 && \
	export RUST_LOG=debug && \
	cargo nextest run --cargo-profile test-release --no-fail-fast 2>&1 | tee $(LOG_DIR)/rust_tests.log

.PHONY: ros-test
ros-test:
	@echo "Running ROS2 node tests with colcon..."
	@mkdir -p $(LOG_DIR)
	@. install/setup.sh && \
	export OPENCV_PKGCONFIG_NAME=opencv4 && \
	colcon test $(COLCON_TEST_FLAGS) --base-paths src/ros2 2>&1 | tee $(LOG_DIR)/colcon_tests.log && \
	colcon test-result --all --verbose

.PHONY: test
test: rust-test ros-test
	@echo "Running all tests..."

.PHONY: clean
clean:
	rm -rf build install log target .cargo $(LOG_DIR)
	$(MAKE) -C src/ros2_rust_ws clean

.PHONY: launch_lidar_camera_sample_data
launch_lidar_camera_sample_data:
	@. install/setup.sh && \
	ros2 systemd launch \
		--name lctk-lidar-camera-data \
		--replace \
		--env RUST_LOG=debug \
		lctk_sample_data lidar_camera.launch.xml

.PHONY: stop_lidar_camera_sample_data
stop_lidar_camera_sample_data:
	@. install/setup.sh && \
	ros2 systemd stop lctk-lidar-camera-data

.PHONY: launch_lidar_camera_calibration
launch_lidar_camera_calibration:
	@. install/setup.sh && \
	ros2 systemd launch --name lctk-calibration --replace \
		--env RUST_LOG=debug \
		lctk_launch lidar_camera_calibration.launch.xml \
		debug_mode:=$(debug_mode) \
		enable_icp_iteration_debug:=$(enable_icp_iteration_debug) \
		enable_evaluator:=$(enable_evaluator) \
		enable_overlay:=$(enable_overlay) \
		enable_rviz:=$(rviz) \
		log_level:=$(log_level) \
		use_best_effort_qos:=$(use_best_effort_qos) \
		use_advanced_solver:=$(use_advanced_solver) \
		camera_topic:=$(camera_topic) \
		pointcloud_topic:=$(pointcloud_topic)

.PHONY: stop_lidar_camera_calibration
stop_lidar_camera_calibration:
	@. install/setup.sh && \
	ros2 systemd stop lctk-calibration

.PHONY: launch_rviz
launch_rviz:
	@. install/setup.sh && \
	export RUST_LOG=debug && \
	ros2 launch lctk_launch rviz.launch.xml

.PHONY: tune_filter_box
tune_filter_box:
	@. install/setup.sh && \
	export RUST_LOG=info && \
	ros2 run filter_box_tuner filter_box_tuner

.PHONY: interactive_solver_controller
interactive_solver_controller:
	@. install/setup.sh && \
	ros2 run interactive_solver_controller interactive_solver_controller

.PHONY: launch_iou_overlapping
launch_iou_overlapping:
	@. install/setup.sh && \
	export RUST_LOG=debug && \
	ros2 launch iou_overlapping iou_evaluator.launch.xml \
		extrinsic_json:=$(or $(extrinsic_json),$(PWD)/install/iou_overlapping/share/iou_overlapping/config/extrinsic.json) \
		use_best_effort_qos:=$(or $(use_best_effort_qos),true) \
		camera_topic:=$(or $(camera_topic),/sensing/camera/front_center/image_raw) \
		pointcloud_topic:=$(or $(pointcloud_topic),/sensing/lidar/top/pointcloud_raw)

.PHONY: launch_two_lidar_calibration
launch_two_lidar_calibration:
	@. install/setup.sh && \
	ros2 systemd launch \
		--name lctk-two-lidar \
		--replace \
		--env RUST_LOG=debug \
		lctk_launch two_lidar_calibration.launch.xml

.PHONY: stop_two_lidar_calibration
stop_two_lidar_calibration:
	@. install/setup.sh && \
	ros2 systemd stop lctk-two-lidar

# Service Management Utilities

.PHONY: service_status
service_status:
	@echo "LCTK Service Status:"
	@echo "==================="
	. install/setup.sh && \
	ros2 systemd list | grep lctk || echo "No LCTK services found"
	@echo ""
	@echo "Detailed status:"
	. install/setup.sh && \
	(ros2 systemd status lctk-sensor 2>/dev/null || echo "lctk-sensor: not found") && \
	(ros2 systemd status lctk-calibration 2>/dev/null || echo "lctk-calibration: not found") && \
	(ros2 systemd status lctk-two-lidar 2>/dev/null || echo "lctk-two-lidar: not found")

.PHONY: service_logs
service_logs:
	@echo "LCTK Service Logs:"
	@echo "=================="
	@echo "Sensor service logs:"
	. install/setup.sh && \
	(ros2 systemd logs lctk-sensor 2>/dev/null || echo "No logs for lctk-sensor")
	@echo ""
	@echo "Calibration service logs:"
	. install/setup.sh && \
	(ros2 systemd logs lctk-calibration 2>/dev/null || echo "No logs for lctk-calibration")
	@echo ""
	@echo "Two LiDAR service logs:"
	. install/setup.sh && \
	(ros2 systemd logs lctk-two-lidar 2>/dev/null || echo "No logs for lctk-two-lidar")

.PHONY: service_cleanup
service_cleanup:
	@echo "Removing all LCTK systemd services..."
	. install/setup.sh && \
	(ros2 systemd stop lctk-sensor 2>/dev/null || true) && \
	(ros2 systemd stop lctk-calibration 2>/dev/null || true) && \
	(ros2 systemd stop lctk-two-lidar 2>/dev/null || true) && \
	(ros2 systemd remove lctk-sensor 2>/dev/null || true) && \
	(ros2 systemd remove lctk-calibration 2>/dev/null || true) && \
	(ros2 systemd remove lctk-two-lidar 2>/dev/null || true)
	@echo "All LCTK services cleaned up."
