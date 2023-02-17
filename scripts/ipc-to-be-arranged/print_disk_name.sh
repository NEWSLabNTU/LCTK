#!/bin/sh
disk_A="33dbc8a9-23ec-4aee-8efd-4c6c3ba08f1d"
disk_B="7c60d6cc-f176-4c41-9824-23dc6d9842e0"
disk_C="6faace49-e036-4c61-a597-ffa0646348e2"
disk_D="8496d57b-cfdf-4ef3-953a-2ac1df899a59"

[ -b "/dev/disk/by-uuid/${disk_A}" ] && echo "/data/disk_A"
[ -b "/dev/disk/by-uuid/${disk_B}" ] && echo "/data/disk_B"
[ -b "/dev/disk/by-uuid/${disk_C}" ] && echo "/data/disk_C"
[ -b "/dev/disk/by-uuid/${disk_D}" ] && echo "/data/disk_D"
