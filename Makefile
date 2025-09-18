COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
LOG_DIR := build_logs
debug_mode := true

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
	@echo "  make build_cargo                - Build with cargo directly (non-ROS)"
	@echo ""
	@echo "Launch Commands (using ros2systemd for reliable service management):"
	@echo "  make launch_sample_data         - Create and start sample data playback service"
	@echo "                                   Optional: pcap_file=path video_file=path pointcloud_topic=name camera_namespace=name loop=true/false"
	@echo "  make stop_sample_data           - Stop sample data playback service"
	@echo "  make launch_lidar_camera_calibration - Create and start LiDAR-Camera calibration pipeline service"
	@echo "                                       Optional: camera_namespace=name pointcloud_topic=name debug_mode=true/false rviz=true/false"
	@echo "  make launch_rviz                    - Launch RViz for calibration visualization"
	@echo "  make stop_lidar_camera_calibration   - Stop LiDAR-Camera calibration service"
	@echo "  make launch_two_lidar_calibration    - Create and start two LiDAR calibration service"
	@echo "  make stop_two_lidar_calibration      - Stop two LiDAR calibration service"
	@echo ""
	@echo "Service Management:"
	@echo "  make service_status             - Show status of all LCTK services"
	@echo "  make service_logs               - Show logs for all LCTK services"
	@echo "  make service_cleanup            - Remove all LCTK systemd services"
	@echo ""
	@echo "Development Tools:"
	@echo "  make format                     - Format all code (Rust, Python, configs)"
	@echo "  make lint                       - Run linters and formatters check"
	@echo "  make clean                      - Clean all build artifacts"
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
	. /opt/ros/humble/setup.sh && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -r -y

.PHONY: build
build: build_ros2_rust build_interface build_packages

.PHONY: build_ros2_rust
build_ros2_rust:
	@mkdir -p $(LOG_DIR)
	@echo "Building ROS2 Rust packages... (log: $(LOG_DIR)/ros2_rust.log)"
	. /opt/ros/humble/setup.sh && \
	$(MAKE) -C src/ros2_rust_ws 2>&1 | tee $(LOG_DIR)/ros2_rust.log

.PHONY: build_interface
build_interface:
	@mkdir -p $(LOG_DIR)
	@echo "Building interface packages... (log: $(LOG_DIR)/interface.log)"
	. ./src/ros2_rust_ws/install/setup.sh && \
	export OPENCV_PKGCONFIG_NAME=opencv4 && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/interface 2>&1 | tee $(LOG_DIR)/interface.log

.PHONY: build_packages
build_packages:
	@mkdir -p $(LOG_DIR)
	@echo "Building ROS nodes... (log: $(LOG_DIR)/packages.log)"
# Fix applied directly to colcon-cargo source to handle JSON parsing issues
	. install/setup.sh && \
	export OPENCV_PKGCONFIG_NAME=opencv4 && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/ros2 2>&1 | tee $(LOG_DIR)/packages.log

.PHONY: build_cargo
build_cargo:
	export OPENCV_PKGCONFIG_NAME=opencv4 && \
	cargo build --all-targets

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
	cargo +nightly fmt --check
	cargo clippy --all-targets --all-features -- -D warnings

.PHONY: clean
clean:
	rm -rf build install log target .cargo $(LOG_DIR)
	$(MAKE) -C src/ros2_rust_ws clean

.PHONY: launch_sample_data
launch_sample_data:
	@echo "Creating and starting sample data playback service with ros2systemd..."
	. install/setup.sh && \
	ros2 systemd remove lctk-sample-data 2>/dev/null || true && \
	ros2 systemd create lctk-sample-data launch lctk_launch sample_data_player.launch.xml \
		pcap_file:=$(or $(pcap_file),$(PWD)/data/sampledata/3/lidar.pcap) \
		video_file:=$(or $(video_file),$(PWD)/data/sampledata/3/video.avi) \
		pointcloud_topic:=$(or $(pointcloud_topic),/sensing/lidar/top/pointcloud_raw) \
		camera_namespace:=$(or $(camera_namespace),/sensing/camera/front_center) \
		loop:=$(or $(loop),true) && \
	ros2 systemd start lctk-sample-data
	@echo "Sample data service started. Use 'make service_status' to check status or 'make stop_sample_data' to stop."

.PHONY: stop_sample_data
stop_sample_data:
	@echo "Stopping sample data service..."
	. install/setup.sh && \
	ros2 systemd stop lctk-sample-data 2>/dev/null || echo "Service not running"

.PHONY: launch_lidar_camera_calibration
launch_lidar_camera_calibration:
	@echo "Creating and starting LiDAR-Camera calibration pipeline service with ros2systemd..."
	@if [ "$(debug_mode)" = "true" ]; then \
		echo "Debug mode enabled - additional debug topics will be published"; \
	fi
	. install/setup.sh && \
	ros2 systemd remove lctk-calibration 2>/dev/null || true && \
	ros2 systemd create --copy-env CYCLONEDDS_URI lctk-calibration launch lctk_launch lidar_camera_calibration.launch.xml \
		camera_namespace:=$(or $(camera_namespace),/sensing/camera/front_center) \
		pointcloud_topic:=$(or $(pointcloud_topic),/sensing/lidar/top/pointcloud_raw) \
		debug_mode:=$(or $(debug_mode),false) \
		enable_rviz:=$(or $(rviz),false) && \
	ros2 systemd start lctk-calibration
	@echo "Calibration service started. Use 'make service_status' to check status or 'make stop_lidar_camera_calibration' to stop."

.PHONY: stop_lidar_camera_calibration
stop_lidar_camera_calibration:
	@echo "Stopping LiDAR-Camera calibration service..."
	. install/setup.sh && \
	ros2 systemd stop lctk-calibration 2>/dev/null || echo "Service not running"

.PHONY: launch_rviz
launch_rviz:
	@echo "Launching RViz for calibration visualization..."
	. install/setup.sh && \
	ros2 launch lctk_launch rviz.launch.xml


.PHONY: launch_two_lidar_calibration
launch_two_lidar_calibration:
	@echo "Creating and starting two LiDAR calibration service with ros2systemd..."
	. install/setup.sh && \
	ros2 systemd remove lctk-two-lidar 2>/dev/null || true && \
	ros2 systemd create lctk-two-lidar launch lctk_launch two_lidar_calibration.launch.xml \
		lidar1_pcap_file:=$(PWD)/data/sampledata/3/lidar.pcap \
		lidar2_pcap_file:=$(PWD)/data/sampledata/4/lidar.pcap && \
	ros2 systemd start lctk-two-lidar
	@echo "Two LiDAR calibration service started. Use 'make service_status' to check status or 'make stop_two_lidar_calibration' to stop."

.PHONY: stop_two_lidar_calibration
stop_two_lidar_calibration:
	@echo "Stopping two LiDAR calibration service..."
	. install/setup.sh && \
	ros2 systemd stop lctk-two-lidar 2>/dev/null || echo "Service not running"

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
	(ros2 systemd status lctk-sample-data 2>/dev/null || echo "lctk-sample-data: not found") && \
	(ros2 systemd status lctk-calibration 2>/dev/null || echo "lctk-calibration: not found") && \
	(ros2 systemd status lctk-two-lidar 2>/dev/null || echo "lctk-two-lidar: not found")

.PHONY: service_logs
service_logs:
	@echo "LCTK Service Logs:"
	@echo "=================="
	@echo "Sample data service logs:"
	. install/setup.sh && \
	(ros2 systemd logs lctk-sample-data 2>/dev/null || echo "No logs for lctk-sample-data")
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
	(ros2 systemd stop lctk-sample-data 2>/dev/null || true) && \
	(ros2 systemd stop lctk-calibration 2>/dev/null || true) && \
	(ros2 systemd stop lctk-two-lidar 2>/dev/null || true) && \
	(ros2 systemd remove lctk-sample-data 2>/dev/null || true) && \
	(ros2 systemd remove lctk-calibration 2>/dev/null || true) && \
	(ros2 systemd remove lctk-two-lidar 2>/dev/null || true)
	@echo "All LCTK services cleaned up."
