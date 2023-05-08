#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

Camera=camera2
Wayside=2
Expname=w2c2
MANIFEST_FILE=../../rust-bin/project-to-image/Cargo.toml
DATA_PATH=../../data/${Expname}/wayside${Wayside}/${Camera}

rm -rf ../results
mkdir -p ../results

for d in $DATA_PATH/*; do
    cargo run --release \
          --manifest-path "$MANIFEST_FILE" \
          -- \
          --no-gui \
          --min-distance 2 \
          --max-distance 8 \
          --intrinsics-file ../../config/intrinsics.yaml \
          --extrinsics-file ../../config/lidar_to_camera/lidar${Wayside}_${Camera}_extrinsics.json5 \
          --pcd-file "$d/pcd/000002.pcd" \
          --image-file "$d/image/00002.jpg" \
          --output-file "../results/$(basename $d).jpg" &
    
done

wait
