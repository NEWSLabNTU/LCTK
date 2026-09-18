#!/usr/bin/env bash
# Install network packet capture libraries
# Converted from ansible/roles/lctk.dev_env.network_libs/tasks/main.yaml

set -e

echo "Installing network packet capture libraries..."

# Only the library and headers: velodyne_driver links libpcap to replay the sample
# data. The capture *tools* (tcpdump, wireshark-common) are analysis conveniences, not
# project dependencies -- and wireshark-common's debconf prompt about non-root dumpcap
# stalls an unattended run behind an invisible dialog, leaving dpkg locked. Install
# them by hand if you want them.
sudo apt-get update
sudo apt-get install -y libpcap-dev

echo "Network libraries installation complete."
