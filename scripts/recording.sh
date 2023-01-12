
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

i=0
while read line
do 
	array[$i]="/dev/v4l/by-id/$line"
	let i=i+1
done < ../config/camera-id.txt

parallel --lb --timeout 10 <<EOF
tshark -i enp7s0 -w lidar.pcap udp
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i ${array[0]} -vf transpose=2,transpose=2 video1.avi
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i ${array[1]} -vf transpose=2,transpose=2 video2.avi
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i ${array[2]} -vf transpose=2,transpose=2 video3.avi
EOF
