#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

MANIFEST_FILE=../../rust-bin/find-hollow-board/Cargo.toml

Camera=camera1
Wayside=1
Expname=exp1
DATA_PATH=../../data/${Expname}/wayside${Wayside}/${Camera}

for d in $DATA_PATH/*; do
    echo $d
    rm -rf "$d/boards"

    if [ -f "$d/bbox.json5" ]
    then
        load_bbox_arg="--load-bbox $d/bbox.json5"
    else
        unset load_bbox_arg
    fi
    
    ! cargo run --release \
          --manifest-path "$MANIFEST_FILE" \
          -- \
          --preview \
          $load_bbox_arg \
          --save-bbox "$d/bbox.json5" \
          "$d/pcd" \
          "$d/boards" &
done

wait
