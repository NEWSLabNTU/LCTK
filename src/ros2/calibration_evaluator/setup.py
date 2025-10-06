import os
from glob import glob

from setuptools import setup

package_name = "calibration_evaluator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/extrinsic.json"]),
        (
            "share/" + package_name + "/launch",
            ["launch/calibration_evaluator.launch.xml"],
        ),
        (os.path.join("lib", package_name), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LCTK",
    maintainer_email="dev@lctk.local",
    description="Evaluates extrinsic calibration quality using IoU metrics between detected board regions and projected LiDAR points.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "calibration_evaluator_node = calibration_evaluator.evaluator_node:main",
        ],
    },
)
