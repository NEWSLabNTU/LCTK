from setuptools import setup

package_name = "calibration_judge"

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
    maintainer="aeon",
    maintainer_email="aeon@todo.todo",
    description="ROS2 node for evaluating calibration quality by comparing transforms against ground truth",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "judge_node = calibration_judge.judge_node:main",
        ],
    },
)
