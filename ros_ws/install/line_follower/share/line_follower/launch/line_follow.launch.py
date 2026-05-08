from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='line_follower',
            executable='line_sensor_node',
            name='line_sensor_node',
            output='screen'
        ),

        Node(
            package='line_follower',
            executable='encoder_odometry_node',
            name='encoder_odometry_node',
            output='screen'
        ),

        Node(
            package='line_follower',
            executable='line_controller_node',
            name='line_controller_node',
            output='screen'
        ),

        Node(
            package='line_follower',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen'
        ),

    ])