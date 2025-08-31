from setuptools import setup, find_packages

package_name = 'aruco_detection_overlay'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LCTK Team',
    maintainer_email='calibration@example.com',
    description='ArUco detection visualization overlay for RViz',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_detection_overlay = aruco_detection_overlay.aruco_detection_overlay:main',
        ],
    },
)