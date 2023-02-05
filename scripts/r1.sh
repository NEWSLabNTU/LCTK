video1=$(readlink -f /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_B2FDD85E-video-index0 )
video2=$(readlink -f /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_13C47C2E-video-index0)
video3=$(readlink -f /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_F7637C2E-video-index0)
#disk1=$(readlink -f /dev/disk/by-id/usb-SanDisk_Cruzer_Force_4C530000100327222540-0:0-part1)
disk1="media/newslab/e6d6cda2-f6a3-42ac-af1d-76742bd1a82a/"
dir=$disk1"recording/wayside_1/"
time=$(date -Ins)
videodir=$dir"video/"$time
pcddir=$dir"pcd/"$time
mkdir $videodir
mkdir $pcddir
parallel --lb --timeout 10 <<EOF
tshark -i enp7s0 -w $pcddir"/lidar.pcap" udp
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i $video1 -vf transpose=2,transpose=2 $videodir"/camera1.avi"
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i $video2 -vf transpose=2,transpose=2 $videodir"/camera2.avi"
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i $video3 -vf transpose=2,transpose=2 $videodir"/camera3.avi" 
EOF
