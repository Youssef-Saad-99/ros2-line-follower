#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range


class LineSensorNode(Node):

    def __init__(self):

        super().__init__('line_sensor_node')

        # IR array input (5 sensors)
        self.ir_sub = self.create_subscription(
            Range,
            '/ir_array',
            self.ir_callback,
            10
        )

        self.pub_error = self.create_publisher(
            Float32,
            '/line_error',
            10
        )

        self.get_logger().info("IR Line Sensor Node Started")

        # last known pattern
        self.last_error = 0.0

    def ir_callback(self, msg):

        # Simulated threshold (Gazebo ray distance)
        # closer = black line
        value = 1.0 if msg.range < 0.15 else 0.0

        # NOTE: simple hack if only 1 beam (we expand later)
        error = 0.0

        if value == 1.0:
            error = 0.0
        else:
            error = self.last_error  # hold last direction

        self.last_error = error

        out = Float32()
        out.data = float(error)

        self.pub_error.publish(out)


def main():
    rclpy.init()
    node = LineSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
