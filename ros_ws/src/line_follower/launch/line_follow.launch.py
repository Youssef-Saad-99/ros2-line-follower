from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory('line_follower')

    urdf = os.path.join(pkg, 'urdf', 'robot.urdf')
    world = os.path.join(pkg, 'worlds', 'line_world.world')

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'line_robot', '-file', urdf],
            output='screen'
        ),

        Node(
            package='line_follower',
            executable='line_sensor_node',
            output='screen'
        ),

        Node(
            package='line_follower',
            executable='line_controller_node',
            output='screen'
        ),
        
        Node(
            package='line_follower',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen'
        ),
        
        Node(
            package='line_follower',
            executable='encoder_odometry_node',
            name='encoder_odometry_node',
            output='screen'
        ),

    ])
