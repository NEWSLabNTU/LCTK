#!/usr/bin/env bash
set -e

function print_usage() {
    echo "example usage: $0 14 18 22" 
    echo "    sort ips by wayside 123" 
    echo "    to download data from 10.8.0.14 10.8.0.18 10.8.0.22"
}

#give wayside 123's ip
ip="10.8.0."
ip1=$ip$1
shift || {
    print_usage
    exit 1
}
ip2=$ip$1
shift || {
    print_usage
    exit 1
}
ip3=$ip$1
shift || {
    print_usage
    exit 1
}
mkdir -p ~/2023-camera-calibration/recording
rsync -azvP newslab@"${ip1}":/home/newslab/LCTK/scripts/wayside1/recording/ \
 ~/2023-camera-calibration/recording
 rsync -azvP newslab@"${ip2}":/home/newslab/LCTK/scripts/wayside2/recording/ \
 ~/2023-camera-calibration/recording
 rsync -azvP newslab@"${ip3}":/home/newslab/LCTK/scripts/wayside3/recording/ \
 ~/2023-camera-calibration/recording

