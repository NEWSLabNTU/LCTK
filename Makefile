.PHONY: default build build_ros2_rust build_interface build_packages clean prepare lint format build_cargo launch_sensor launch_lidar_camera_calibration launch_two_lidar_calibration

COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
LOG_DIR := build_logs

default: build

prepare:
	@echo "Setting up LCTK development environment..."
	@echo "This will install all required dependencies using Ansible."
	@echo ""
	@./setup-dev-env.sh

prepare-minimal:
	@echo "Setting up minimal LCTK environment (no CUDA or dev tools)..."
	@./setup-dev-env.sh -y --minimal

prepare-ci:
	@echo "Setting up LCTK environment for CI (non-interactive)..."
	@./setup-dev-env.sh -y

build: build_ros2_rust build_interface build_packages

build_ros2_rust:
	@mkdir -p $(LOG_DIR)
	@echo "Building ROS2 Rust packages... (log: $(LOG_DIR)/ros2_rust.log)"
	. /opt/ros/humble/setup.sh && \
	$(MAKE) -C src/ros2_rust_ws 2>&1 | tee $(LOG_DIR)/ros2_rust.log

build_interface:
	@mkdir -p $(LOG_DIR)
	@echo "Building interface packages... (log: $(LOG_DIR)/interface.log)"
	. ./src/ros2_rust_ws/install/setup.sh && \
	export OPENCV_PKGCONFIG_NAME=opencv4 && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/interface 2>&1 | tee $(LOG_DIR)/interface.log

build_packages:
	@mkdir -p $(LOG_DIR)
	@echo "Building ROS nodes... (log: $(LOG_DIR)/packages.log)"
# Fix applied directly to colcon-cargo source to handle JSON parsing issues
	. install/setup.sh && \
	export OPENCV_PKGCONFIG_NAME=opencv4 && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/bin 2>&1 | tee $(LOG_DIR)/packages.log

build_cargo:
	export OPENCV_PKGCONFIG_NAME=opencv4 && \
	cargo build --all-targets

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

lint:
	@echo "Checking Rust code formatting and linting..."
	cargo +nightly fmt --check
	cargo clippy --all-targets --all-features -- -D warnings

clean:
	rm -rf build install log target .cargo $(LOG_DIR)
	$(MAKE) -C src/ros2_rust_ws clean

launch_sensor:
	@echo "Launching sensor publishers with sample data..."
	. /opt/ros/humble/setup.sh && \
	. install/setup.sh && \
	ros2 launch calib_launch sensor.launch.xml \
		pcap_file:=$(PWD)/data/sampledata/3/lidar.pcap \
		video_file:=$(PWD)/data/sampledata/3/video.avi \
		loop:=true

launch_lidar_camera_calibration:
	@echo "Launching LiDAR-Camera calibration with sample data..."
	. /opt/ros/humble/setup.sh && \
	. install/setup.sh && \
	ros2 launch calib_launch lidar_camera_calibration.launch.xml \
		pcap_file:=$(PWD)/data/sampledata/3/lidar.pcap \
		video_file:=$(PWD)/data/sampledata/3/video.avi \
		aruco_config_file:=$(shell ros2 pkg prefix calib_launch)/share/calib_launch/config/aruco/aruco_pattern.json5 \
		board_config_file:=$(shell ros2 pkg prefix calib_launch)/share/calib_launch/config/board/board_pattern.json5 \
		loop:=true

launch_two_lidar_calibration:
	@echo "Launching two LiDAR calibration with sample data..."
	. /opt/ros/humble/setup.sh && \
	. install/setup.sh && \
	ros2 launch calib_launch two_lidar_calibration.launch.xml \
		lidar1_pcap_file:=$(PWD)/data/sampledata/3/lidar.pcap \
		lidar2_pcap_file:=$(PWD)/data/sampledata/4/lidar.pcap \
		board_config_file:=$(shell ros2 pkg prefix calib_launch)/share/calib_launch/config/board/board_pattern.json5
