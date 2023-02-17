#!/usr/bin/env bash
set -e

function print_usage() {
    echo "example usage: $0 ~/2023-camera-calibration" 
    echo "   convert datas  to LCTK data type"
}
dir=$1
shift || {
    print_usage
    exit 1
}
new_dir=~/LCTK/data/newdata/
rm -rf ${new_dir}
mkdir -p ${new_dir}

for i in {1..3}
do
    wi=wayside${i}/
    di=${dir}"/recording/"${wi}
    mkdir -p ${new_dir}/${wi}
    for j in {1..3}
    do 
        cj=camera${j}/
        mkdir -p ${new_dir}/${wi}/${cj}
    done
    for filename in ${di}"/video/"*
    do
        cd $filename
        a=$(basename $filename)
        for j in {1..3}
        do 
            cj=camera${j}
            mkdir -p ${new_dir}/${wi}/${cj}/${a}/
            cp ${filename}/${cj}* ${new_dir}/${wi}/${cj}/${a}/
        done
    done
    
    for filename in ${di}"/pcd/"*
    do
        cd $filename
        a=$(basename $filename)
        for j in {1..3}
        do 
            cj=camera${j}
            cp ${filename}/lidar1.pcap ${new_dir}/${wi}/${cj}/${a}/
        done
    done
    #cd $di
done
