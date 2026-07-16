from setuptools import find_packages, setup

package_name = "lctk_autoware_export"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NEWSLAB NTU",
    maintainer_email="lctk@ntu.edu.tw",
    description="Export LCTK LiDAR-camera extrinsics into Autoware sensor_kit_calibration.yaml",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "export = lctk_autoware_export.export:main",
        ],
    },
)
