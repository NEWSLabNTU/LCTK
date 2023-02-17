#!/usr/bin/env bash

root_dir="$(git rev-parse --show-toplevel)"
mkdir -p "${root_dir}/recording/pcd" "${root_dir}/recording/video"

function kill_jobs() {
    [ -z "$VIDEO_PGID" ] || kill -- -$VIDEO_PGID
    [ -z "$LIDAR_PGID" ] || kill -- -$LIDAR_PGID
}

trap "kill_jobs; exit 2" SIGINT SIGTERM EXIT

for i in {1..6}; do
    setsid "${root_dir}/target/release/video-capture" \
               --config "${root_dir}/deploy/ipc/video-capture-recording-set-2.json" &
    VIDEO_PID=$!
    VIDEO_PGID="$(ps -ef -o pgid= -p $VIDEO_PID | tr -d ' ')"

    setsid "${root_dir}/deploy/ipc/capture-velodyne.sh" &
    LIDAR_PID=$!
    LIDAR_PGID="$(ps -ef -o pgid= -p $LIDAR_PID | tr -d ' ')"

    sleep 3600
    kill_jobs
    sleep 5
done
