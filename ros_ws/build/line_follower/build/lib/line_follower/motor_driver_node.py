#!/usr/bin/env python3
"""
motor_driver_node.py
========================
PURPOSE:
    Translates geometry_msgs/Twist from /cmd_vel into simple string commands
    for the serial_comm_node.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('max_linear', 0.15)
        self.declare_parameter('max_angular', 0.5)
        self.declare_parameter('angular_threshold', 0.05)
        self.declare_parameter('linear_threshold', 0.01)
        
        # زودنا الحد الأدنى لسرعة الدوران لـ 6 لزيادة الاستجابة
        self.declare_parameter('min_turn_speed', 6) 

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.linear_threshold = self.get_parameter('linear_threshold').value
        self.min_turn_speed = self.get_parameter('min_turn_speed').value

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/motor_cmd', 10)

        self.get_logger().info('MotorDriverNode started | Turn speed increased | Max Error safety active')

    def _cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        char = 'S'
        speed = 0

        # 1. Check for Turning (Angular dominates)
        if abs(w) > self.angular_threshold:
            char = 'L' if w > 0 else 'R'
            
            clamped_w = min(abs(w), self.max_angular)
            speed_ratio = clamped_w / self.max_angular
            
            # لو الايرور ماكس (الروبوت على الحافة)، بنقلل السرعة لـ 3 عشان نلحق الخط
            if speed_ratio >= 1.0:
                speed = 3
            else:
                # Mapping من min_turn_speed لحد 9
                speed_range = 9 - self.min_turn_speed
                speed = self.min_turn_speed + int(speed_ratio * speed_range)

        # 2. Check for Forward (Linear dominates)
        elif v > self.linear_threshold:
            char = 'F'
            clamped_v = min(v, self.max_linear)
            speed_ratio = clamped_v / self.max_linear
            speed = int(speed_ratio * 9.0)
            speed = max(1, speed) 

        # 3. Stop
        else:
            char = 'S'
            speed = 0

        # ── Format and Publish ────────────────────────────────────────────
        if char != 'S':
            speed = max(1, min(9, speed)) 
            cmd_str = f"{char}{speed}"
        else:
            cmd_str = "S0" 

        out_msg = String()
        out_msg.data = cmd_str
        self.publisher.publish(out_msg)

        self.get_logger().debug(f'Twist(v={v:.3f}, w={w:.3f}) -> Cmd: {cmd_str}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MotorDriverNode shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

