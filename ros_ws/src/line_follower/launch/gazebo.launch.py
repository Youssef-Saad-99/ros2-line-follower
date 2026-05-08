from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_path = get_package_share_directory('line_follower')

    urdf_file = os.path.join(
        pkg_path,
        'urdf',
        'robot.urdf'
    )

    world_file = os.path.join(
        pkg_path,
        'worlds',
        'line_world.world'
    )

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_file,
                '-s',
                'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_file],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',

            arguments=[
                '-entity', 'line_robot',
                '-file', urdf_file,

                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.2'
            ],

            output='screen'
        ),
    ])
