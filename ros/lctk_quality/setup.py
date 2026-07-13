from setuptools import setup

package_name = "lctk_quality"

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
    description="Quality metrics for the LiDAR-camera extrinsic solve (H-09)",
    license="MIT",
    tests_require=["pytest"],
)
