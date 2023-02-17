#!/usr/bin/env bash
set -e

function print_usage {
    echo "example usage: $0 0900 3600"
    echo "    to record for 1 hour since 09:00 AM"
}

since="$1"
shift || {
    print_usage
    exit 1
}
timeout="$1"
shift || {
    print_usage
    exit 1
}

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

source ../record.sh
run_recording wayside2 "$since" "$timeout" camera-id.txt
