#!/usr/bin/env bash
set -e

function print_usage() {
    echo "example usage: $0 14 18 22 exp1" 
    echo "    sort ips by wayside 123" 
    echo "    to download data from 10.8.0.14 10.8.0.18 10.8.0.22"
    echo "    data path: LCTK/scripts/record-data/waysidei/recname (optional, default is 'recording')"
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
recname="$1"
recname="${recname:-"recording"}"
mkdir -p ~/2023-camera-calibration/${recname}
rsync -azvP newslab@"${ip1}":/home/newslab/LCTK/scripts/record-data/wayside1/${recname}/ \
 ~/2023-camera-calibration/${recname}
 rsync -azvP newslab@"${ip2}":/home/newslab/LCTK/scripts/record-data/wayside2/${recname}/ \
 ~/2023-camera-calibration/${recname}
 rsync -azvP newslab@"${ip3}":/home/newslab/LCTK/scripts/record-data/wayside3/${recname}/ \
 ~/2023-camera-calibration/${recname}

