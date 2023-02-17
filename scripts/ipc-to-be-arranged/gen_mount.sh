#!/bin/env bash

if [ $# -ne 2 ]
then
    echo "Usage: $0 device_path mount_dir"
    exit 1
fi

store_dir="mount_files"
mkdir -p $store_dir

device="$(realpath -s $1)"
dir="$(realpath -s $2)"
name="$(tr / - <<< $dir | cut -c 2-)"

cat > "$store_dir/$name.mount" << EOF
[Unit]
Description=External drive

[Mount]
What=$device
Where=$dir
Options=defaults

[Install]
WantedBy=multi-user.target
EOF

cat > "$store_dir/$name.automount" << EOF
Description="Automount External Drive"

[Automount]
Where=$dir

[Install]
WantedBy=multi-user.target

EOF
