from setuptools import setup

package_name = "filter_box_tuner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NEWSLAB NTU",
    maintainer_email="lctk@ntu.edu.tw",
    description="Interactive CLI tool for tuning bounding box filter parameters",
    license="MIT",
    entry_points={
        "console_scripts": [
            "filter_box_tuner = filter_box_tuner.tuner:main",
        ],
    },
)
