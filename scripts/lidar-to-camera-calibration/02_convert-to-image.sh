#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"
DATA_PATH=../data/sampledata
for d in $DATA_PATH/* ; do
    rm -rf "$d/image"
    mkdir -p "$d/image"
    echo ffmpeg -i "$d/video.mp4" "$d/image/%05d.jpg"
done | parallel