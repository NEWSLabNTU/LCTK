#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

Camera=camera1
Wayside=1
Expname=exp1
DATA_PATH=../../data/${Expname}/wayside${Wayside}/${Camera}

for d in $DATA_PATH/* ; do
    rm -rf "$d/image"
    mkdir -p "$d/image"
    echo ffmpeg -i "$d/"${Camera}".mp4" "$d/image/%05d.jpg"
done | parallel
