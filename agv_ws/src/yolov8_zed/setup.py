import os
from glob import glob
from setuptools import setup

package_name = 'yolov8_zed'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vrl',
    maintainer_email='nwzantout@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pothole_perception_node = yolov8_zed.pothole_perception_node:main',
            'recording_node = yolov8_zed.recording:main'
        ],
    },
)
