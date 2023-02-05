
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

i=0
while read line
do 
	array[$i]="/dev/v4l/by-id/$line"
	let i=i+1
done < camera-id.txt

# disk1="/media/newslab/e6d6cda2-f6a3-42ac-af1d-76742bd1a82a"
disk1="."
dir="${disk1}/recording/wayside_2"
time=$(date -Ins)
videodir="$dir/video/$time"
pcddir="$dir/pcd/$time"
timeout="00:00:05"

mkdir -p "$videodir"
mkdir -p "$pcddir"

parallel --lb --timeout 10 <<EOF
tshark -i enp7s0 -w $pcddir"/lidar.pcap" udp
echo $(date -Ins) > ${videodir}/camera1.txt; ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1280x720 -i ${array[0]} -t "${timeout}" -c:v libx264 -preset fast -vf transpose=2,transpose=2 "${videodir}/camera1.mp4"
echo $(date -Ins) > ${videodir}/camera2.txt; ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1280x720 -i ${array[1]} -t "${timeout}" -c:v libx264 -preset fast -vf transpose=2,transpose=2 "${videodir}/camera2.mp4"
echo $(date -Ins) > ${videodir}/camera3.txt; ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1280x720 -i ${array[2]} -t "${timeout}" -c:v libx264 -preset fast -vf transpose=2,transpose=2 "${videodir}/camera3.mp4"
EOF
