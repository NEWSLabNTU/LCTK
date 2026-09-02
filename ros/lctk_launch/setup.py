import os
from glob import glob

from setuptools import find_packages, setup

package_name = "lctk_launch"


def get_data_files():
    """Build list of data files to install, preserving directory structure."""
    data_files = [
        # Ament index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Launch files (XML and Python)
        (os.path.join("share", package_name, "launch"), glob("launch/*.xml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ]

    # Walk config directory to preserve structure
    config_dir = "config"
    for dirpath, dirnames, filenames in os.walk(config_dir):
        if filenames:
            # Get relative path from config/
            rel_dir = os.path.relpath(dirpath, config_dir)
            if rel_dir == ".":
                install_dir = os.path.join("share", package_name, "config")
            else:
                install_dir = os.path.join("share", package_name, "config", rel_dir)

            # Add all files in this directory
            files = [os.path.join(dirpath, f) for f in filenames]
            data_files.append((install_dir, files))

    data_files.extend(get_session_data_files())

    return data_files


def get_session_data_files():
    """Install the shipped sessions from the repo root, preserving structure.

    Sessions live at the repo root rather than under this package because a
    session is a run, not launch machinery -- but they still have to reach the
    install tree so `ros2 run lctk_launch lctk_session list` finds them on a
    machine that only has `install/`.

    A `sessions/` directory need not exist: a fresh clone before the sessions
    land, or a tree pruned to just this package, must still build. Per-session
    `out/` directories are run artifacts and are never installed.
    """
    package_dir = os.path.dirname(os.path.abspath(__file__))
    absolute_dir = os.path.normpath(
        os.path.join(package_dir, os.pardir, os.pardir, "sessions")
    )
    if not os.path.isdir(absolute_dir):
        return []

    # setuptools refuses an absolute source path in `data_files`, so locate the
    # directory from __file__ (which is stable) and then express it relative to
    # the working directory setuptools will copy from.
    sessions_dir = os.path.relpath(absolute_dir, os.getcwd())

    data_files = []
    for dirpath, dirnames, filenames in os.walk(sessions_dir):
        # Prune in place so os.walk does not descend into run outputs at all.
        dirnames[:] = [d for d in dirnames if d != "out"]
        if not filenames:
            continue
        rel_dir = os.path.relpath(dirpath, sessions_dir)
        if rel_dir == os.curdir:
            install_dir = os.path.join("share", package_name, "sessions")
        else:
            install_dir = os.path.join("share", package_name, "sessions", rel_dir)
        files = [os.path.join(dirpath, f) for f in filenames]
        data_files.append((install_dir, files))
    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=get_data_files(),
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="NEWSLAB NTU",
    maintainer_email="lctk@ntu.edu.tw",
    description="Launch files and configurations for LCTK calibration pipelines",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tf_tree_broadcaster = lctk_launch.tf_tree_broadcaster:main",
            "lctk_session = lctk_launch.session_cli:main",
            "lctk_bag_play = lctk_launch.bag_play:main",
        ],
    },
)
