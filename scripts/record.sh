function set_camera_env () {
    dev=$1
    shift
    v4l2-ctl -c focus_auto=0 -d ${dev}
    v4l2-ctl -c focus_absolute=5 -d ${dev}
}

function run_recording () {
    hostname="$1"
    shift
    since="$1"
    shift
    timeout_secs="$1"
    shift
    camera_id_file="$1"
    shift


    timeout_spec=$(date "-d@$timeout_secs" -u +%H:%M:%S)

    i=0
    while read line
    do
        array[$i]="/dev/v4l/by-id/$line"
	set_camera_env ${array[$i]}
	let i=i+1
    done < "${camera_id_file}"

    # disk1="/media/newslab/e6d6cda2-f6a3-42ac-af1d-76742bd1a82a"
    disk1="."
    dir="${disk1}/recording/${hostname}"
    time=$(date -Is)
    videodir="$dir/video/$time"
    pcddir="$dir/pcd/$time"

    mkdir -p "$videodir"
    mkdir -p "$pcddir"

    sleepuntil "$since"
    parallel --lb <<EOF
timeout ${timeout_secs} tshark -i enp7s0 -w $pcddir"/lidar1.pcap" udp
echo $(date -Ins) > ${videodir}/camera1.txt; ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1280x720 -i ${array[0]} -t ${timeout_spec} -c:v libx264 -preset fast -vf transpose=2,transpose=2 "${videodir}/camera1.mp4"
echo $(date -Ins) > ${videodir}/camera2.txt; ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1280x720 -i ${array[1]} -t ${timeout_spec} -c:v libx264 -preset fast -vf transpose=2,transpose=2 "${videodir}/camera2.mp4"
echo $(date -Ins) > ${videodir}/camera3.txt; ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1280x720 -i ${array[2]} -t ${timeout_spec} -c:v libx264 -preset fast -vf transpose=2,transpose=2 "${videodir}/camera3.mp4"
EOF
}
