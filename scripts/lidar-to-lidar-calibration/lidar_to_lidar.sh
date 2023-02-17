#!/usr/bin/env bash
set -e

function print_usage {
    echo "example usage: $0 --config <lidar-config-path>"
    echo "    you have to checkout your lidar config file before executing this."
}

config="$1"
shift || {
    print_usage
    exit 1
}

MANIFEST_FILE=../../rust-bin/multi_wayside/Cargo.toml

cargo run --release \
          --manifest-path "$MANIFEST_FILE" \
           -- \
           --config ${config}
