.PHONY: default build build_ros2_rust build_packages clean

COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

default: build

build: build_ros2_rust build_packages

build_ros2_rust:
	. /opt/ros/humble/setup.sh && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/ros2_rust_ws src/interface

build_packages:
	. install/setup.sh && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/bin

clean:
	rm -rf build install log target
