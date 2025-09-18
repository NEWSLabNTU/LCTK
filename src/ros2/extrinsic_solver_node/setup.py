from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'extrinsic_solver_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # No local config files
        (os.path.join('lib', package_name), ['scripts/extrinsic_solver_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NEWSLAB NTU',
    maintainer_email='lctk@ntu.edu.tw',
    description='Simple Python ROS 2 node for demonstrating solvePnP with ArUco and board detections',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'extrinsic_solver_node = extrinsic_solver_node_py.main:main',
        ],
    },
)

