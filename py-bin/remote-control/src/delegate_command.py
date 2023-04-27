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
        '-c',
        '--command',
        default='ip address',
        help='Recipe file path'
    )
    return parser



def main():
    args = build_parser().parse_args()
    session_config = load_session(args.session)

    server = libtmux.server.Server()
    session = server.sessions.filter(session_name=session_config.name)[0]
    assert len(session.panes) == len(session_config.hosts)
    for pane in session.panes:
        pane.send_keys('C-c', enter=False)
        pane.send_keys(args.command)


if __name__ == '__main__':
    main()
