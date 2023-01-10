video1=$(readlink -f /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_B2FDD85E-video-index0 )
video2=$(readlink -f /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_13C47C2E-video-index0)
video3=$(readlink -f /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_F7637C2E-video-index0)

parallel --lb --timeout 10 <<EOF
tshark -i enp7s0 -w lidar.pcap udp
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i $video1 -vf transpose=2,transpose=2 video1.avi
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i $video2 -vf transpose=2,transpose=2 video2.avi
ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1920x1080 -i $video3 -vf transpose=2,transpose=2 video3.avi
EOF
