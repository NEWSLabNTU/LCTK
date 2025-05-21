.PHONY: default build clean

default: build

build:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

clean:
	rm -rf build install log
