#!/bin/sh
ts '%Y%m%d-%H:%M:%.S' </dev/ttyS0 > $(date -Ins).log
