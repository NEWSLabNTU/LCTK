#!/usr/bin/env bash

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'


devices=(
    /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_D7F47C2E-video-index0
    /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_13C47C2E-video-index0
    /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_F7637C2E-video-index0
    /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_A9157C2E-video-index0
    /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_6AC47C2E-video-index0
    /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_5C457C2E-video-index0
    /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_DF557C2E-video-index0
    /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_72457C2E-video-index0
    /dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_B2FDD85E-video-index0
)

for dev in ${devices[@]}; do
    echo "# $dev"

    if [ -c $dev ] ; then
        v4l2-ctl -c focus_auto=0 -d $dev && \
        v4l2-ctl -c focus_absolute=5 -d $dev && \
        echo "successfully configured"

        v4l2-ctl -C focus_auto -d $dev
        v4l2-ctl -C focus_absolute -d $dev
    else
        echo "device not found"
    fi
done

echo "############check lidar incoming packets############"
 
#throw packet
timeout 10 tshark -i enp7s0 -c 20 udp
 
#check throw packet function execute resault
if [ "$( echo $? )" = "0" ];then
        echo -e "${GREEN}lidar connection success${NC}"
else
        echo -e "${RED}lidar connection failed!!${NC}"
fi


