#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class ControllerNode(Node):

    def __init__(self):

        super().__init__('line_controller_node')

        self.sub = self.create_subscription(
            Float32,
            '/line_error',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.state = "FOLLOW"   # FOLLOW / LANE_CHANGE
        self.counter = 0

        self.get_logger().info("Controller Node Started")

    def callback(self, msg):

        error = msg.data
        cmd = Twist()

        # =========================
        # LANE CHANGE LOGIC
        # =========================

        # if strong deviation → lane change mode
        if abs(error) > 0.6:
            self.state = "LANE_CHANGE"
            self.counter = 20

        if self.state == "LANE_CHANGE":

            cmd.linear.x = 0.1
            cmd.angular.z = 0.8  # sharp turn

            self.counter -= 1

            if self.counter <= 0:
                self.state = "FOLLOW"

        else:
            # normal line follow
            cmd.linear.x = 0.25
            cmd.angular.z = -error * 1.5

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
