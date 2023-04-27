import argparse
from pathlib import Path
from datetime import datetime, timedelta
from subprocess import Popen
import time
import signal
import os
from loguru import logger
import json5

from .data import WaysideIndex, CameraIndex, LidarIndex


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-o',
        '--output-dir',
        help='The {video,pcd} files would be stored at OUTPUT_DIR/SAMPLE_NAME/TIMESTAMP/HOSTNAME/{video,pcd}. Absolute path is mandantory.',
        required=True,
    )
    parser.add_argument(
        '-n',
        '--name',
        help='The name of the recording sample',
        default='lidar-to-lidar'
    )
    parser.add_argument(
        '-i',
        '--index',
        help='The index of wayside sensor',
        type=int,
        choices=list(WaysideIndex),
        default=1
    )
    parser.add_argument(
        '-c',
        '--camera',
        help='The camera devices to use.',
        nargs='+',
        type=int,
        choices=list(CameraIndex),
        default=[]
    )
    parser.add_argument(
        '-l',
        '--lidar',
        help='The LiDAR devices to use. Avaible options: [1]',
        nargs='+',
        type=int,
        choices=list(LidarIndex),
        default=[]
    )
    parser.add_argument(
        '-t',
        '--timeout',
        help='Recoding timeout (secs)',
        type=int,
        default=30
    )
    parser.add_argument(
        '-m',
        '--mapping',
        help='Camera id mapping file',
        required=True,
    )
    parser.add_argument(
        '-s',
        '--since',
        help='timestamp in HH:MM:SS',
        default=None
    )
    return parser


def main():
    args = build_parser().parse_args()

    # Output directory
    if not os.path.isabs(args.output_dir):
        logger.error(f'--output-dir: {args.output_dir} needs to be abosulte path!')
        exit(-1)
    output_dir = Path(args.output_dir)

    # Subdirectories under the output directory
    timestamp = datetime.now().strftime('%Y-%m-%dT%H:%M')
    base_dir = output_dir / args.name / str(timestamp) / f'wayside{args.index}'
    vid_dir = base_dir / 'video'
    pcd_dir = base_dir / 'pcd'
    logger.info(f'The recording data would be stored at {base_dir}')
    if args.camera:
        os.makedirs(vid_dir, exist_ok=True)
    if args.lidar:
        os.makedirs(pcd_dir, exist_ok=True)

    # Sanity check
    if not args.camera and not args.lidar:
        logger.error('Neither video nor lidar device specified!')
        exit(-1)

    # Camera id mapping, e.g. 1 ---> usb-046d_Logitech_Webcam_C930e_B2FDD85E-video-index0
    cam_mapping = None
    if args.mapping:
        cam_mapping_file = Path(args.mapping)
        assert cam_mapping_file.exists()
        with open(args.mapping) as f:
            cam_mapping = json5.load(f)[f'wayside{args.index}']  # type: ignore
            assert len(cam_mapping) == 3

    def record_video(idx: int) -> Popen:
        assert cam_mapping
        cam_device = f'/dev/v4l/by-id/{cam_mapping[idx - 1]}'
        video_file = vid_dir / f'camera{idx}.mp4'
        cmd = ' '.join([
            'while true;',
            f'do echo save to {video_file};',
            'sleep 1;',
            'done'
        ])
        test_cmd = ' '.join([
            f'v4l2-ctl -c focus_auto=0 -d {cam_device} &&',
            f'v4l2-ctl -c focus_absolute=5 -d {cam_device} &&'
            f'echo $(date -Ins) > {vid_dir}/camera{idx}.txt &&',
            'ffmpeg -y -f video4linux2 -input_format uyvy422 -framerate 30 -video_size 1280x720',
            f'-i {cam_device} -c:v libx264 -preset fast -vf transpose=2,transpose=2 ${video_file}'
        ])
        print(test_cmd)
        return Popen(['/usr/bin/bash', '-c', cmd])

    def record_lidar(idx: int) -> Popen:
        lidar_file = pcd_dir / f'lidar{idx}.pcap'
        cmd = ' '.join([
            'while true;',
            f'do echo save to {lidar_file};',
            'sleep 1;',
            'done'
        ])
        #  test_cmd = f'tshark -i enp7s0 -w {lidar_file} udp'
        return Popen(['/usr/bin/bash', '-c', cmd])

    # Sleep until the specified since timestamp
    if args.since:
        now = datetime.now()
        since = datetime \
            .strptime(args.since, '%H:%M:%S') \
            .replace(year=now.year, month=now.month, day=now.day)
        delta = since - datetime.now()
        if delta > timedelta(0):
            print(f'The recording will begin at: {since}')
            time.sleep(delta.total_seconds())
        else:
            logger.error(f'The specified since timestamp {args.since} had passed.')
            exit(-1)

    # Begin recording
    process_list = list(map(record_video, args.camera)) + list(map(record_lidar, args.lidar))
    logger.info(f'Begin recording!')

    def clean_up():
        for p in process_list:
            p.terminate()
            p.kill()

    def handler(signum, _):
        if signum == signal.SIGINT.value:
            clean_up()
            logger.info('Terminated.')
            exit(1)

    signal.signal(signal.SIGINT, handler)

    time.sleep(args.timeout)
    clean_up()

    logger.info('Finished.')


if __name__ == '__main__':
    main()
