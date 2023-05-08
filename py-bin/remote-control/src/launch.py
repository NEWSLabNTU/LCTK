import libtmux
from argparse import ArgumentParser

from .data import load_session


WINDOW_NAME = 'Main'

def build_parser() -> ArgumentParser:
    parser = ArgumentParser()
    parser.add_argument(
        '-s',
        '--session',
        help='Session config (json5)',
        required=True,
    )
    parser.add_argument(
        '-w',
        '--working-directory',
        default='/home/newslab/LCTK',
        help='Working directory'
    )
    return parser


def main():
    args = build_parser().parse_args()
    config = load_session(args.session)
    server = libtmux.server.Server()
    session = server.new_session(
        session_name=config.name,
        window_name=WINDOW_NAME,
        kill_session=True,
    )
    window = session.select_window(target_window=WINDOW_NAME)
    for _ in range(len(config.hosts)-1):
        window.split_window(attach=False)
    window.select_layout('even-vertical')
    for (pane, sensor) in zip(window.panes, config.hosts):
        pane.send_keys(f'ssh {sensor.address} cd {args.working_directory}')
    server.attach_session(target_session=config.name)

if __name__ == '__main__':
    main()
