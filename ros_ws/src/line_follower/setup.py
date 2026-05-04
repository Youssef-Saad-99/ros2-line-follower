from setuptools import find_packages, setup

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        # Required by ROS 2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # package.xml
        ('share/' + package_name, ['package.xml']),
        
       #lanuch file
        ('share/' + package_name + '/launch', ['launch/line_follow.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Line follower ROS2 package',
    license='MIT',

    entry_points={
        'console_scripts': [
            'line_sensor_node = line_follower.line_sensor_node:main',
            'encoder_odometry_node = line_follower.encoder_odometry_node:main',
            'line_controller_node = line_follower.line_controller_node:main',
            'motor_driver_node = line_follower.motor_driver_node:main',
        ],
    },
)