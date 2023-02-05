#!/usr/bin/env bash
set -e

timeout="$1"
shift || {
    echo "example usage: ./$0 00:00:10"
    exit 1
}

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

source ../record.sh
run_recording wayside3 "$timeout" camera-id.txt
