import os
from glob import glob

from setuptools import find_packages, setup

package_name = "lidar_to_camera_solver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # No local config files
        (os.path.join("lib", package_name), ["scripts/lidar_to_camera_solver"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NEWSLAB NTU",
    maintainer_email="lctk@ntu.edu.tw",
    description="Continuous and manual LiDAR-to-camera extrinsic calibration solver",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar_to_camera_solver = lidar_to_camera_solver.main:main",
            "migrate_detections = lidar_to_camera_solver.migrate_detections:main",
        ],
    },
)
