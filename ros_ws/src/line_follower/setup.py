from setuptools import setup
from glob import glob
import os

package_name = 'line_follower'

setup(
    name=package_name,

    version='0.0.0',

    packages=[package_name],

    data_files=[

        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),

        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')
        ),

        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='Youssef',

    maintainer_email='youssefmohamed4004306@gmail.com',

    description='ROS2 Line Follower Robot',

    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [

            'line_sensor_node = line_follower.line_sensor_node:main',

            'line_controller_node = line_follower.line_controller_node:main',

            'motor_driver_node = line_follower.motor_driver_node:main',

            'encoder_odometry_node = line_follower.encoder_odometry_node:main',
        ],
    },
)
