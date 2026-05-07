#!/usr/bin/env python3

"""
line_controller_node.py
=======================

ROS2 Control Node for Line Follower Robot

This node:
1. Subscribes to /line_error
2. Applies PID control
3. Publishes velocity commands to /cmd_vel

Subscribed Topics:
    /line_error   (std_msgs/msg/Float32)

Published Topics:
    /cmd_vel      (geometry_msgs/msg/Twist)
"""

import time
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


# =========================================================
# PID Controller Class
# =========================================================
class PIDController:
    """
    Simple reusable PID controller.
    """

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        output_min: float = -1.0,
        output_max: float = 1.0,
        integral_windup_limit: float = 1.0
    ):
        """
        Initialize PID controller parameters.

        Args:
            Kp (float): Proportional gain
            Ki (float): Integral gain
            Kd (float): Derivative gain
            output_min (float): Minimum controller output
            output_max (float): Maximum controller output
            integral_windup_limit (float): Integral clamp limit
        """

        # PID gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Output limits
        self.output_min = output_min
        self.output_max = output_max

        # Anti-windup
        self.windup_limit = integral_windup_limit

        # Internal state
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_timestamp = None

    # -----------------------------------------------------
    def reset(self):
        """
        Reset PID internal state.
        """

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_timestamp = None

    # -----------------------------------------------------
    def update(self, error: float, timestamp: float) -> float:
        """
        Compute PID output.

        Args:
            error (float): Current error
            timestamp (float): Current time

        Returns:
            float: PID output
        """

        # Compute dt
        if self._prev_timestamp is None:
            dt = 0.0
        else:
            dt = timestamp - self._prev_timestamp
            dt = max(dt, 1e-6)

        # Proportional term
        p_term = self.Kp * error

        # Integral term
        self._integral += error * dt

        # Clamp integral
        self._integral = max(
            -self.windup_limit,
            min(self.windup_limit, self._integral)
        )

        i_term = self.Ki * self._integral

        # Derivative term
        if dt > 0:
            derivative = (error - self._prev_error) / dt
            d_term = self.Kd * derivative
        else:
            d_term = 0.0

        # PID output
        output = p_term + i_term + d_term

        # Clamp output
        output = max(
            self.output_min,
            min(self.output_max, output)
        )

        # Save state
        self._prev_error = error
        self._prev_timestamp = timestamp

        return output


# =========================================================
# ROS2 Line Controller Node
# =========================================================
class LineControllerNode(Node):
    """
    ROS2 node responsible for line-following control.
    """

    def __init__(self):
        """
        Initialize ROS2 node.
        """

        super().__init__('line_controller_node')

        # =================================================
        # Parameters
        # =================================================
        self.declare_parameter('Kp', 5.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.1)

        self.declare_parameter('base_speed', 0.10)
        self.declare_parameter('min_speed', 0.05)

        self.declare_parameter('max_angular', 2.0)

        self.declare_parameter('timeout_sec', 0.5)


# Read parameters
        Kp = self.get_parameter('Kp').value
        Ki = self.get_parameter('Ki').value
        Kd = self.get_parameter('Kd').value

        self.base_speed = self.get_parameter('base_speed').value
        self.min_speed = self.get_parameter('min_speed').value

        self.max_angular = self.get_parameter('max_angular').value

        self.timeout = self.get_parameter('timeout_sec').value

        # =================================================
        # PID Controller
        # =================================================
        self.pid = PIDController(
            Kp=Kp,
            Ki=Ki,
            Kd=Kd,
            output_min=-self.max_angular,
            output_max=self.max_angular,
            integral_windup_limit=1.0
        )

        # =================================================
        # Internal State
        # =================================================
        self._last_error_time = None
        self._last_error_value = 0.0

        # =================================================
        # Publisher
        # =================================================
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # =================================================
        # Subscriber
        # =================================================
        self.error_sub = self.create_subscription(
            Float32,
            '/line_error',
            self.error_callback,
            10
        )

        # =================================================
        # Timer
        # =================================================
        self.control_rate_hz = 50.0

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop
        )

        self.get_logger().info('Line Controller Node Started')

    # -----------------------------------------------------
    def error_callback(self, msg: Float32):
        """
        Receive line error from sensor node.

        Args:
            msg (Float32): Line error message
        """

        self._last_error_value = msg.data
        self._last_error_time = time.monotonic()

    # -----------------------------------------------------
    def control_loop(self):
        """
        Main control loop.
        """

        now = time.monotonic()

        # =============================================
        # Safety timeout
        # =============================================
        if (
            self._last_error_time is None
            or
            (now - self._last_error_time) > self.timeout
        ):
            self.publish_stop()
            return

        # =============================================
        # Current error
        # =============================================
        error = self._last_error_value

        # =============================================
        # PID control
        # =============================================
        angular_z = self.pid.update(-error, now)

        # =============================================
        # Dynamic speed adjustment
        # =============================================
        abs_error = min(1.0, abs(error))

        linear_x = (
            self.base_speed * (1.0 - abs_error)
            +
            self.min_speed * abs_error
        )

        linear_x = max(self.min_speed, linear_x)

        # =============================================
        # Create Twist message
        # =============================================
        cmd = Twist()

        cmd.linear.x = linear_x
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = angular_z

        # Publish command
        self.cmd_pub.publish(cmd)

        # Debug log
        self.get_logger().info(
            f'Error: {error:.2f} | '
            f'Linear: {linear_x:.2f} | '
            f'Angular: {angular_z:.2f}'
        )


# -----------------------------------------------------
    def publish_stop(self):
        """
        Publish zero velocity command.
        """

        self.cmd_pub.publish(Twist())

        self.pid.reset()


# =========================================================
# Main Function
# =========================================================
def main(args=None):
    """
    ROS2 entry point.
    """

    rclpy.init(args=args)

    node = LineControllerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')

    finally:
        node.destroy_node()
        rclpy.shutdown()


# =========================================================
# Script Entry
# =========================================================
if __name__ == '__main__':
    main()
