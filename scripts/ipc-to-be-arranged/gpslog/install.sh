#!/usr/bin/env bash
sudo apt install moreutils
sudo chmod 744 gpslog.sh
sudo chmod 744 gpslog.service
sudo cp -v gpslog.sh /opt/gpslog.sh
sudo cp -v gpslog.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable gpslog.service
systemctl restart gpslog.service
