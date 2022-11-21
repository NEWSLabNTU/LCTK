#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

MANIFEST_FILE=../rust-bin/solve-extrinsic-params/Cargo.toml
DATA_PATH=../data/sampledata
board_files=(
    # ../4m/2/boards/000001.json5
    # ../4m/2/boards/000002.json5
    # ../4m/2/boards/000003.json5
    # ../4m/7/boards/000001.json5
    # ../4m/7/boards/000002.json5
    # ../4m/7/boards/000003.json5
    # ../4m/8/boards/000001.json5
    # ../4m/8/boards/000002.json5
    # ../4m/8/boards/000003.json5
    # ../2m/1/boards/000001.json5
    # ../2m/1/boards/000002.json5
    # ../2m/1/boards/000003.json5
    $DATA_PATH/3/boards/000001.json5
    $DATA_PATH/3/boards/000002.json5
    $DATA_PATH/3/boards/000003.json5
    $DATA_PATH/4/boards/000001.json5
    $DATA_PATH/4/boards/000002.json5
    $DATA_PATH/5/boards/000026.json5
    # ../3m/1/boards/000001.json5
    # ../3m/1/boards/000002.json5
    # ../3m/1/boards/000003.json5
    # ../3m/3/boards/000001.json5
    # ../3m/3/boards/000002.json5
    # ../3m/3/boards/000003.json5
    # ../3m/4/boards/000001.json5
    # ../3m/4/boards/000002.json5
    # ../3m/4/boards/000003.json5
)

aruco_files=(
    # ../4m/2/aruco/1.json5
    # ../4m/2/aruco/2.json5
    # ../4m/2/aruco/3.json5
    # ../4m/7/aruco/1.json5
    # ../4m/7/aruco/2.json5
    # ../4m/7/aruco/3.json5
    # ../4m/8/aruco/1.json5
    # ../4m/8/aruco/2.json5
    # ../4m/8/aruco/3.json5
    $DATA_PATH/3/aruco/1.json5
    $DATA_PATH/3/aruco/2.json5
    $DATA_PATH/3/aruco/3.json5
    $DATA_PATH/4/aruco/1.json5
    $DATA_PATH/4/aruco/2.json5
    $DATA_PATH/5/aruco/3.json5
    # ../2m/5/aruco/1.json5
    # ../2m/5/aruco/2.json5
    # ../2m/5/aruco/3.json5
    # ../3m/1/aruco/1.json5
    # ../3m/1/aruco/2.json5
    # ../3m/1/aruco/3.json5
    # ../3m/3/aruco/1.json5
    # ../3m/3/aruco/2.json5
    # ../3m/3/aruco/3.json5
    # ../3m/4/aruco/1.json5
    # ../3m/4/aruco/2.json5
    # ../3m/4/aruco/3.json5
)

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
