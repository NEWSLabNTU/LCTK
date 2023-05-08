import libtmux
from argparse import ArgumentParser
from loguru import logger

from .data import load_session, load_recipe, display

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
        '-r',
        '--recipe',
        help='Recipe file (json5)',
        required=True,
    )
    parser.add_argument(
        '-o',
        '--output-dir',
        help='The {video,pcd} files would be stored at OUTPUT_DIR/SAMPLE_NAME/TIMESTAMP/HOSTNAME/{video,pcd}. Absolute path is mandantory.',
        default='.'
    )
    parser.add_argument(
        '-m',
        '--mapping',
        help='Camera id mapping file',
        default=None
    )
    return parser



def main():
    args = build_parser().parse_args()
    session_config = load_session(args.session)
    recipe = load_recipe(args.recipe)

    server = libtmux.server.Server()
    session = server.sessions.filter(session_name=session_config.name)[0]
    assert len(session.panes) == len(session_config.hosts)
    logger.info('Session attached.')

    for (pane, lst) in zip(session.panes, recipe.device_lists):
        camera_list = display(lst.camera_list)
        lidar_list = display(lst.lidar_list)

        cmd = ' '.join([
            'pdm run',
            '-p $(git rev-parse --show-toplevel)/py-bin/remote-control',
            'record',
            f'--name {recipe.name}',
            f'--index {int(lst.index)}',
            f'--camera {camera_list}' if camera_list else '',
            f'--lidar {lidar_list}' if lidar_list else '',
            f'--timeout {recipe.timeout_secs}',
            f'--since {recipe.since}' if recipe.since else '',
            f'--output-dir {args.output_dir}',
            f'--mapping {args.mapping}',
        ])

        pane.send_keys('C-c', enter=False)
        pane.send_keys(cmd)
    logger.info('Recipe sent.')


if __name__ == '__main__':
    main()
