COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
LOG_DIR := build_logs

.PHONY: default
default: help

.PHONY: help
help:
	@echo "LCTK (LiDAR and Camera Toolkit) - Available targets:"
	@echo ""
	@echo "Setup & Environment:"
	@echo "  make setup                      - Set up development environment (installs all dependencies)"
	@echo "  make rosdep                     - Install ROS dependencies with rosdep"
	@echo ""
	@echo "Build Commands:"
	@echo "  make build                      - Build entire project (all 3 passes)"
	@echo "  make build_ros2_rust            - Build ROS2 Rust base packages"
	@echo "  make build_interface            - Build interface types"
	@echo "  make build_packages             - Build ROS nodes"
	@echo "  make build_cargo                - Build with cargo directly (non-ROS)"
	@echo ""
	@echo "Launch Commands:"
	@echo "  make launch_sensor              - Launch sensor publishers with sample data"
	@echo "  make launch_lidar_camera_calibration - Launch LiDAR-Camera calibration"
	@echo "  make launch_two_lidar_calibration    - Launch two LiDAR calibration"
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

.PHONY: rosdep
rosdep:
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
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/bin 2>&1 | tee $(LOG_DIR)/packages.log

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

.PHONY: launch_sensor
launch_sensor:
	@echo "Launching sensor publishers with sample data..."
	. /opt/ros/humble/setup.sh && \
	. install/setup.sh && \
	ros2 launch calib_launch sensor.launch.xml \
		pcap_file:=$(PWD)/data/sampledata/3/lidar.pcap \
		video_file:=$(PWD)/data/sampledata/3/video.avi \
		loop:=true

.PHONY: launch_lidar_camera_calibration
launch_lidar_camera_calibration:
	@echo "Launching LiDAR-Camera calibration with sample data..."
	. /opt/ros/humble/setup.sh && \
	. install/setup.sh && \
	ros2 launch calib_launch lidar_camera_calibration.launch.xml \
		pcap_file:=$(PWD)/data/sampledata/3/lidar.pcap \
		video_file:=$(PWD)/data/sampledata/3/video.avi \
		loop:=true \
		debug_mode:=$(or $(debug_mode),false)

.PHONY: launch_two_lidar_calibration
launch_two_lidar_calibration:
	@echo "Launching two LiDAR calibration with sample data..."
	. /opt/ros/humble/setup.sh && \
	. install/setup.sh && \
	ros2 launch calib_launch two_lidar_calibration.launch.xml \
		lidar1_pcap_file:=$(PWD)/data/sampledata/3/lidar.pcap \
		lidar2_pcap_file:=$(PWD)/data/sampledata/4/lidar.pcap
