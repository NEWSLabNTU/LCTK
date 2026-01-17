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
        "console_scripts": [],
    },
)
