#!/usr/bin/env python3
"""
line_sensor_node.py
====================
PURPOSE:
    Subscribes to '/ir_raw' published by the serial node.
    A weighted centroid formula converts those binary readings into a single
    scalar "error" value that tells the controller how far the robot is from
    the line center.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class LineSensorNode(Node):

    def __init__(self):
        super().__init__('line_sensor_node')

        self.num_sensors = 5
        # weights centred at zero: [-2, -1, 0, +1, +2]
        self.weights = [i - (self.num_sensors - 1) / 2.0 for i in range(self.num_sensors)]
        self.norm_factor = self.weights[-1]   # = (N-1)/2 = 2.0

        # متغير عشان نحفظ فيه آخر Error قبل ما الخط يضيع
        self._last_valid_error = 0.0

        # Publisher for the error value
        self.publisher_ = self.create_publisher(Float32, '/line_error', 10)

        # Subscriber to get raw IR strings from the serial node
        self.subscription = self.create_subscription(
            String,
            '/ir_raw',
            self.ir_callback,
            10
        )
        
        self.get_logger().info('Line Sensor Node is ready, listening to /ir_raw...')

    def ir_callback(self, msg: String):
        line = msg.data
        
        # Format check: "S:XXXXX"
        if line.startswith('S:') and len(line) == 7:
            readings = [float(c) for c in line[2:]]
            error = self._compute_weighted_centroid_error(readings)

            # Publish the error
            out_msg = Float32()
            out_msg.data = error
            self.publisher_.publish(out_msg)

            # Print to terminal for testing
            self.get_logger().info(f'Sensors: {line[2:]} | Error: {error:+.4f}')

    def _compute_weighted_centroid_error(self, readings: list) -> float:
        total_activation = sum(readings)

        if total_activation < 1e-6:
            # لو فقد الخط، هنبعت Max Error بناءً على آخر اتجاه
            self.get_logger().warn('Line lost! All sensors read zero. Sending Max Error to search.')
            if self._last_valid_error >= 0:
                return 1.0  # هيلف يمين ببطء يدور على الخط
            else:
                return -1.0 # هيلف شمال ببطء يدور على الخط

        centroid = sum(w * r for w, r in zip(self.weights, readings)) / total_activation
        error = centroid / self.norm_factor
        clamped_error = float(max(-1.0, min(1.0, error)))
        
        # بنحفظ قيمة الـ error الحالية عشان نستخدمها المرة الجاية لو الخط ضاع
        self._last_valid_error = clamped_error
        
        return clamped_error

def main(args=None):
    rclpy.init(args=args)
    node = LineSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LineSensorNode shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

