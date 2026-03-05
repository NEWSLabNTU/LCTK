from glob import glob

from setuptools import setup

package_name = "pointcloud_image_overlay"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"lib/{package_name}", glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LCTK",
    maintainer_email="dev@lctk.local",
    description="Render point cloud overlay on image with extrinsic from JSON5.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "overlay_node = pointcloud_image_overlay.overlay_node:main",
        ],
    },
)
