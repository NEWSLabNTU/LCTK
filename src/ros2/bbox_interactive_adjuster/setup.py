from setuptools import setup

package_name = "bbox_interactive_adjuster"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Claude",
    maintainer_email="claude@anthropic.com",
    description="Interactive bounding box parameter adjustment tool for calibration_board_locator",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bbox_adjuster = bbox_interactive_adjuster.bbox_adjuster:main",
        ],
    },
)
