#!/bin/sh
script_dir="$(dirname $(realpath $0))"

sudo cp -v "$script_dir/ipc-lidar.nmconnection" /etc/NetworkManager/system-connections/
sudo systemctl restart NetworkManager.service
