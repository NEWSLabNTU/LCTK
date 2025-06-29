#!/usr/bin/env bash

# This script downloads the LiDAR-camera calibration dataset from Zenodo.
# The dataset is provided by the following Zenodo record:
# https://zenodo.org/records/7541422

set -e

# Check if aria2c is installed, and install if not
if ! command -v aria2c &> /dev/null
then
    echo "aria2c could not be found, attempting to install..."
    sudo apt-get update
    sudo apt-get install -y aria2
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

DATA_DIR="$PROJECT_ROOT/data/lidar_camera_calibration"
LIVOX_TAR_GZ="$DATA_DIR/livox.tar.gz"
OUSTER_TAR_GZ="$DATA_DIR/ouster.tar.gz"
LIVOX_EXTRACTED_DIR="$DATA_DIR/livox"
OUSTER_EXTRACTED_DIR="$DATA_DIR/ouster"
DOWNLOAD_LIST_PATH="$SCRIPT_DIR/download_files.txt"

mkdir -p "$DATA_DIR"

# Check if datasets are already extracted
if [ -d "$LIVOX_EXTRACTED_DIR" ] && [ -d "$OUSTER_EXTRACTED_DIR" ]; then
    echo "Datasets already extracted. Skipping download and extraction."
    exit 0
fi

echo "Downloading datasets..."
aria2c -x 16 -s 16 --allow-overwrite=true --continue=true -i "$DOWNLOAD_LIST_PATH" --dir="$PROJECT_ROOT"

echo "Extracting datasets..."
if [ ! -d "$LIVOX_EXTRACTED_DIR" ]; then
    echo "Extracting Livox dataset..."
    tar -xzf "$LIVOX_TAR_GZ" -C "$DATA_DIR"
fi

if [ ! -d "$OUSTER_EXTRACTED_DIR" ]; then
    echo "Extracting Ouster dataset..."
    tar -xzf "$OUSTER_TAR_GZ" -C "$DATA_DIR"
fi

echo "Cleaning up..."
rm -f "$LIVOX_TAR_GZ"
rm -f "$OUSTER_TAR_GZ"

echo "Dataset downloaded and extracted to $DATA_DIR"
