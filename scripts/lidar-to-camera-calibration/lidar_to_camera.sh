#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

Camera=camera1
Wayside=1
Expname=exp1
DATA_PATH=../../data/${Expname}/wayside${Wayside}/${Camera}
MANIFEST_FILE1=../../rust-bin/pcd-tool/Cargo.toml
#01
for d in $DATA_PATH/*; do
    echo $d
    rm -rf "$d/pcd"

    ! cargo run --release \
          --manifest-path "$MANIFEST_FILE1" \
          -- \
          convert \
          "$d/lidar1.pcap" \
          "$d/pcd" \
          10 \
          3
done
#02
for d in $DATA_PATH/* ; do
    rm -rf "$d/image"
    mkdir -p "$d/image"
    echo ffmpeg -i "$d/"${Camera}".mp4" "$d/image/%05d.jpg"
done | parallel
#03
MANIFEST_FILE3=../../rust-bin/find-hollow-board/Cargo.toml
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
          --manifest-path "$MANIFEST_FILE3" \
          -- \
          --preview \
          $load_bbox_arg \
          --save-bbox "$d/bbox.json5" \
          "$d/pcd" \
          "$d/boards" &
done

wait
#04
MANIFEST_FILE4=../../rust-bin/find-aruco-marker/Cargo.toml
for d in $DATA_PATH/*; do
    echo $d
    rm -rf "$d/aruco"
    ! cargo run --release \
          --manifest-path "$MANIFEST_FILE4" \
          -- \
          --gui \
          ../../config/intrinsics.yaml \
          $d/${Camera}".mp4" \
          $d/aruco
done
#05
MANIFEST_FILE5=../../rust-bin/solve-extrinsic-params/Cargo.toml
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
      --manifest-path "$MANIFEST_FILE5" \
      -- \
      --method SQPNP \
      --intrinsics-file ../../config/intrinsics.yaml \
      --output-file ../../config/lidar_to_camera/lidar${Wayside}_${Camera}_extrinsics.json5 \
      --boards "$boards_arg" \
      --arucos "$arucos_arg"
