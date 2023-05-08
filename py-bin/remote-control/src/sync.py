from argparse import ArgumentParser
import os
import subprocess
from loguru import logger
import signal

from .data import load_session


def build_parser() -> ArgumentParser:
    parser = ArgumentParser()
    parser.add_argument(
        '-s',
        '--session',
        help='Session config (json5)',
        required=True,
    )
    parser.add_argument(
        '-p',
        '--parallel',
        action='store_true',
        help='rsync data in parallel'
    )
    parser.add_argument(
        '--source',
        required=True,
        help='source directory'
    )
    parser.add_argument(
        '--target',
        required=True,
        help='target directory'
    )
    return parser


def main():
    args = build_parser().parse_args()
    session_config = load_session(args.session)
    for dir in [args.source, args.target]:
        assert os.path.isabs(dir), 'Absolute path is mandantory!'

    os.makedirs(args.target, exist_ok=True)
    cmd_list = [
        f'rsync -azvP {hosts.address}:{args.source}/ {args.target}'
        for hosts in session_config.hosts
    ]

    logger.info(f'Begin syncing!')

    if args.parallel:
        process_list = [
            subprocess.Popen(['/usr/bin/bash', '-c', cmd])
            for cmd in cmd_list
        ]

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
        for p in process_list:
            p.wait()

        clean_up()

    else:
        for cmd in cmd_list:
            subprocess.call(['/usr/bin/bash', '-c', cmd])

    logger.info('Finished.')
