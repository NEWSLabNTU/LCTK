#!/usr/bin/env bash
set -e

function print_usage {
    echo "example usage: $0 0900 3600"
    echo "    to record for 1 hour since 09:00 AM"
}


script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"
cd ../rust-bin/multi_wayside/
cargo run --release -- --config $1
