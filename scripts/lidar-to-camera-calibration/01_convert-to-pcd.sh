#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"
DATA_PATH=../data/sampledata
MANIFEST_FILE=../rust-bin/pcd-tool/Cargo.toml

for d in $DATA_PATH/*; do
    echo $d
    rm -rf "$d/pcd"

    ! cargo run --release \
          --manifest-path "$MANIFEST_FILE" \
          -- \
          convert \
          "$d/lidar.pcap" \
          "$d/pcd"
done
