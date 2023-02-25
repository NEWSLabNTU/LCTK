#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"
Camera=camera1
Wayside=1
Expname=exp1
DATA_PATH=../../data/${Expname}/wayside${Wayside}/${Camera}
MANIFEST_FILE=../../rust-bin/pcd-tool/Cargo.toml

for d in $DATA_PATH/*; do
    echo $d
    rm -rf "$d/pcd"

    ! cargo run --release \
          --manifest-path "$MANIFEST_FILE" \
          -- \
          convert \
          "$d/lidar1.pcap" \
          "$d/pcd" \
          10 \
          3
done
