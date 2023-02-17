#!/bin/sh
ts '%Y%m%d-%H:%M:%.S' </dev/ttyUSB0 > $(date -Ins).log
