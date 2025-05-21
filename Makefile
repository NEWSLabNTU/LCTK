.PHONY: default build clean

COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

default: build

build:
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/interface
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/bin

clean:
	rm -rf build install log
