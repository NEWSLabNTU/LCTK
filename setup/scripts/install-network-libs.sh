#!/usr/bin/env bash
# Install network packet capture libraries
# Converted from ansible/roles/lctk.dev_env.network_libs/tasks/main.yaml

set -e

echo "Installing network packet capture libraries..."

sudo apt-get update
sudo apt-get install -y \
    libpcap-dev \
    libpcap0.8-dev \
    tcpdump \
    wireshark-common

echo "Network libraries installation complete."
