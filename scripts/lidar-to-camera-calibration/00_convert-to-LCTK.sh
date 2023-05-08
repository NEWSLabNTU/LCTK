#!/usr/bin/env bash
set -e

function print_usage() {
    echo "example usage: $0 ~/2023-camera-calibration/exp1 output" 
    echo "   convert datas  to LCTK data type and save to LCTK/data/output"
}
dir=$1
shift || {
    print_usage
    exit 1
}
outname="$1"
outname="${outname:-"output"}"

new_dir=~/LCTK/data/${outname}/
rm -rf ${new_dir}
mkdir -p ${new_dir}

for filename in ${dir}/*
do
    timestamp=$(basename $filename)  
    for i in {1..3}
    do
        wi=wayside${i}
        di=${dir}/${timestamp}/${wi}

	if [ ! -d ${di} ]; then
		continue	
	fi

        mkdir -p ${new_dir}/${wi}

        for j in {1..3}
        do 
            cj=camera${j}
            tar_dir=${new_dir}/${wi}/${cj}/${timestamp}
            mkdir -p $tar_dir
            cp ${di}/pcd/lidar1.pcap ${tar_dir}
            cp ${di}/video/${cj}* ${tar_dir}
        done
    done
done
