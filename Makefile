.PHONY: default build build_ros2_rust build_interface build_packages clean prepare lint format build_cargo launch_sensor launch_lidar_camera_calibration launch_two_lidar_calibration

COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
LOG_DIR := build_logs

default: build

prepare:
	pip install -U git+https://github.com/jerry73204/colcon-cargo.git
	pip install -U git+https://github.com/colcon/colcon-ros-cargo.git
	rosdep install --from-paths src -y --ignore-src
	cargo install cargo-ament-build

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
	cargo +nightly fmt

lint:
	@echo "Checking Rust code formatting and linting..."
	cargo +nightly fmt --check
	cargo clippy --all-targets --all-features -- -D warnings

clean:
	rm -rf build install log target .cargo $(LOG_DIR)

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
		aruco_config_file:=$(PWD)/configs/aruco_pattern.json5 \
		board_config_file:=$(PWD)/configs/board_pattern.json5 \
		loop:=true

launch_two_lidar_calibration:
	@echo "Launching two LiDAR calibration with sample data..."
	. /opt/ros/humble/setup.sh && \
	. install/setup.sh && \
	ros2 launch calib_launch two_lidar_calibration.launch.xml \
		lidar1_pcap_file:=$(PWD)/data/sampledata/3/lidar.pcap \
		lidar2_pcap_file:=$(PWD)/data/sampledata/4/lidar.pcap \
		board_config_file:=$(PWD)/configs/board_pattern.json5
