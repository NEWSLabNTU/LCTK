#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

MANIFEST_FILE=../rust-bin/solve-extrinsic-params/Cargo.toml
PCD_DATA_PATH=./wayside3/recording/wayside3/pcd
VIDEO_DATA_PATH=./wayside3/recording/wayside3/video

i=0
for file in $( ls $PCD_DATA_PATH/*/boards/000001.json5)
do
	board_files[$i]=$file
	echo ${board_files[$i]}
	let i=i+1
done

i=0
for file in $( ls $VIDEO_DATA_PATH/*/aruco/1.json5)
do
	aruco_files[$i]=$file
	echo ${aruco_files[$i]}
	let i=i+1
done

# board_files=(
#     $DATA_PATH/3/boards/000001.json5
#     $DATA_PATH/3/boards/000002.json5
#     $DATA_PATH/3/boards/000003.json5
#     $DATA_PATH/4/boards/000001.json5
#     $DATA_PATH/4/boards/000002.json5
#     $DATA_PATH/5/boards/000026.json5
# )

# aruco_files=(
#     $DATA_PATH/3/aruco/1.json5
#     $DATA_PATH/3/aruco/2.json5
#     $DATA_PATH/3/aruco/3.json5
#     $DATA_PATH/4/aruco/1.json5
#     $DATA_PATH/4/aruco/2.json5
#     $DATA_PATH/5/aruco/3.json5
# )

IFS=','
boards_arg="${board_files[*]}"
arucos_arg="${aruco_files[*]}"
unset IFS

cargo run --release \
      --manifest-path "$MANIFEST_FILE" \
      -- \
      --method SQPNP \
      --intrinsics-file ../config/intrinsics.yaml \
      --output-file ../config/extrinsics.json5 \
      --boards "$boards_arg" \
      --arucos "$arucos_arg" 
