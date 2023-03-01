#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

MANIFEST_FILE=../../rust-bin/solve-extrinsic-params/Cargo.toml

Camera=camera1
Wayside=1
Expname=exp1
DATA_PATH=../../data/${Expname}/wayside${Wayside}/${Camera}

i=0
for file in $( ls $DATA_PATH/*/boards/000001.json5)
do
	board_files[$i]=$file
	echo ${board_files[$i]}
	let i=i+1
done

i=0
for file in $( ls $DATA_PATH/*/aruco/1.json5)
do
	aruco_files[$i]=$file
	echo ${aruco_files[$i]}
	let i=i+1
done

IFS=','
boards_arg="${board_files[*]}"
arucos_arg="${aruco_files[*]}"
unset IFS

cargo run --release \
      --manifest-path "$MANIFEST_FILE" \
      -- \
      --method SQPNP \
      --intrinsics-file ../../config/intrinsics.yaml \
      --output-file ../../config/lidar_to_camera/lidar${Wayside}_${Camera}_extrinsics.json5  \
      --boards "$boards_arg" \
      --arucos "$arucos_arg" 
