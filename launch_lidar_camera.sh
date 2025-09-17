#!/usr/bin/env bash
source install/setup.bash
RUST_LOG=debug ros2 launch lctk_launch lidar_camera_calibration.launch.xml \
		pcap_file:=$PWD/data/sampledata/3/lidar.pcap \
		video_file:=$PWD/data/sampledata/3/video.avi \
		loop:=true \
		debug_mode:=true
