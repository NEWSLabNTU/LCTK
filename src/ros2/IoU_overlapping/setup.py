from setuptools import setup

package_name = "iou_overlapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/extrinsic.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LCTK",
    maintainer_email="dev@lctk.local",
    description="Extrinsic matrix evaluator: IoU between board mask and LiDAR projection.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "evaluator = iou_overlapping.evaluator_node:main",
        ],
    },
)
