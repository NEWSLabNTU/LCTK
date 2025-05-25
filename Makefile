.PHONY: default build build_ros2_rust build_interface build_packages clean

COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

default: build

build: build_ros2_rust build_interface build_packages

build_ros2_rust:
	. /opt/ros/humble/setup.sh && \
	$(MAKE) -C ros2_rust_ws

build_interface:
	. ./ros2_rust_ws/install/setup.sh && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/interface

build_packages:
	. install/setup.sh && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/bin

clean:
	rm -rf build install log target
