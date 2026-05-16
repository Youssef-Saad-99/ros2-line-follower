from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Serial Communication Node
        # (بدون parameters من هنا لأنها مكتوبة كثوابت داخل كود البايثون)
        Node(
            package='line_follower_pkg',
            executable='serial_comm',
            name='serial_reader_node',
            output='screen'
        ),

        # 2. Line Sensor Node
        Node(
            package='line_follower_pkg',
            executable='line_sensor_node',
            name='line_sensor_node',
            output='screen'
        ),

        # 3. Line Controller Node
        # (مضبوطة على البارامترز اللي إنت كاتبها في الكود)
        Node(
            package='line_follower_pkg',
            executable='line_controller_node',
            name='line_controller_node',
            output='screen',
            parameters=[{
                'Kp': 0.5,
                'Ki': 0.0,
                'Kd': 0.05,
                'base_speed': 0.15,
                'min_speed': 0.05,
                'max_angular': 1.0,
                'timeout_sec': 0.5
            }]
        ),

        # 4. Motor Driver Node
        # (بدون parameters لأنك بتستخدم ثوابت زي MAX_LINEAR_SPEED جوه الكود)
        Node(
            package='line_follower_pkg',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen'
        ),

        # 5. Encoder Odometry Node
        Node(
            package='line_follower_pkg',
            executable='encoder_odom_node',
            name='encoder_odometry_node',
            output='screen',
            parameters=[{
                'wheel_radius': 0.033,
                'wheel_base': 0.160,
                'encoder_ppr': 20,
                'avr_send_interval_s': 0.050,
                'publish_rate_hz': 20.0
            }]
        ),
    ])
