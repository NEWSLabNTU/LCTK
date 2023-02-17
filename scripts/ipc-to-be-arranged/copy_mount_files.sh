#!/usr/bin/env bash

store_dir="mount_files"

sudo cp -v $store_dir/* /etc/systemd/system/

sudo systemctl daemon-reload

for f in $store_dir/*.mount; do
    service=$(basename $f)
    # sudo systemctl enable $service
    sudo systemctl restart $service 
done
