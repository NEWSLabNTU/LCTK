
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

i=0
while read line
do 
	array[$i]="/dev/v4l/by-id/$line"
	let i=i+1
done < ../config/camera-id.txt
disk1="/media/newslab/e6d6cda2-f6a3-42ac-af1d-76742bd1a82a/"
dir=$disk1"recording/wayside_1/"
time=$(date -Ins)
videodir=$dir"video/"$time
pcddir=$dir"pcd/"$time
mkdir $videodir
mkdir $pcddir
parallel --lb --timeout 10 <<EOF
tshark -i enp7s0 -w $pcddir"/lidar.pcap" udp
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i ${array[0]} -vf transpose=2,transpose=2 $videodir"/camera1.avi"
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i ${array[1]} -vf transpose=2,transpose=2 $videodir"/camera2.avi"
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i ${array[2]} -vf transpose=2,transpose=2 $videodir"/camera3.avi"
EOF
