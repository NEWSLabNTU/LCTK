#!/usr/bin/env bash
# Install GStreamer for video processing
# Converted from ansible/roles/lctk.dev_env.gstreamer/tasks/main.yaml

set -e

echo "Installing GStreamer base packages..."

sudo apt-get update
sudo apt-get install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev

echo "Installing additional GStreamer plugins for video decoding..."
sudo apt-get install -y \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-vaapi

echo "Installing GStreamer Python bindings..."
sudo apt-get install -y python3-gst-1.0

echo "GStreamer installation complete."
