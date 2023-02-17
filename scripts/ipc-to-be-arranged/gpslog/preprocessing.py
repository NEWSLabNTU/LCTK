#!/usr/bin/env python3
import sys
import csv

time_list = list()
filename = sys.argv[1]

with open(filename) as f:
    lines = f.readlines()
    for l in lines:
        l = l[:-1].split(" ")
        gpsdata = l[1].split(",")
        if len(gpsdata) == 13:
            time_list.append([l[0], int(gpsdata[1])])

with open("processed_" + filename.split(".")[0] + ".csv", 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['Unix_time', 'GPS_time'])
    for i in time_list:
        writer.writerow(i)
