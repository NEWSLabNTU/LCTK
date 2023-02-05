#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

MANIFEST_FILE=../rust-bin/find-aruco-marker/Cargo.toml
DATA_PATH=../data/sampledata
for d in $DATA_PATH/*; do
    echo $d
    rm -rf "$d/aruco"
    ! cargo run --release \
          --manifest-path "$MANIFEST_FILE" \
          -- \
          --gui \
          ../config/intrinsics.yaml \
          $d/video.avi \
          $d/aruco
done
