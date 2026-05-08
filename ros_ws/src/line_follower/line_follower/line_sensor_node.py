#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge


class LineSensorNode(Node):

    def __init__(self):

        super().__init__('line_sensor_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32,
            '/line_error',
            10
        )

        self.get_logger().info('Real Line Sensor Node Started')

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, _ = frame.shape

        # bottom region only
        roi = frame[int(height * 0.7):, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        _, thresh = cv2.threshold(
            gray,
            60,
            255,
            cv2.THRESH_BINARY_INV
        )

        moments = cv2.moments(thresh)

        if moments['m00'] > 0:

            cx = int(moments['m10'] / moments['m00'])

            error = (cx - width / 2) / (width / 2)

            msg_out = Float32()
            msg_out.data = float(error)

            self.publisher.publish(msg_out)

            self.get_logger().info(
                f'Line Error: {error:.3f}',
                throttle_duration_sec=0.1
            )

        else:

            self.get_logger().warn(
                'Line Lost',
                throttle_duration_sec=1.0
            )


def main(args=None):

    rclpy.init(args=args)

    node = LineSensorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
