#!/usr/bin/env python3
"""
line_controller_node.py
========================
PURPOSE:
    This node is the CONTROL layer of the line-following pipeline.
    It subscribes to /line_error (the lateral deviation of the line from
    robot centre) and computes a corrective velocity command using a
    PID controller.

    The computed command (linear speed + angular correction) is published
    to /cmd_vel as a geometry_msgs/Twist message.

CONTROL ARCHITECTURE  — PID:
    error(t)   = line_error  (received from /line_error)

    P term: Kp * error(t)                         — proportional correction
    I term: Ki * ∫ error dt                        — eliminates steady-state offset
    D term: Kd * d(error)/dt                       — dampens oscillation

    angular_z   = P + I + D                        (rad/s, positive = turn left)
    linear_x    = base_speed * (1 - |error|)       (m/s, slows down on sharp curves)

    The output angular_z is clamped to [−max_angular_vel, +max_angular_vel]
    and linear_x to [min_speed, max_speed] for safety.

SUBSCRIBED TOPICS:
    /line_error  (std_msgs/Float32)

PUBLISHED TOPICS:
    /cmd_vel  (geometry_msgs/Twist)
"""

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class PIDController:
    def __init__(self, Kp: float, Ki: float, Kd: float,
                 output_min: float = -1.0, output_max: float = 1.0,
                 integral_windup_limit: float = 1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max
        self.windup_limit = integral_windup_limit
        self._integral       = 0.0
        self._prev_error     = 0.0
        self._prev_timestamp = None

    def reset(self):
        self._integral       = 0.0
        self._prev_error     = 0.0
        self._prev_timestamp = None

    def update(self, error: float, timestamp: float) -> float:
        if self._prev_timestamp is None:
            dt = 0.0
        else:
            dt = timestamp - self._prev_timestamp
            dt = max(dt, 1e-6)

        p_term = self.Kp * error

        self._integral += error * dt
        self._integral = max(-self.windup_limit,
                             min(self.windup_limit, self._integral))
        i_term = self.Ki * self._integral

        if dt > 0:
            d_term = self.Kd * (error - self._prev_error) / dt
        else:
            d_term = 0.0

        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))

        self._prev_error     = error
        self._prev_timestamp = timestamp

        return output


class LineControllerNode(Node):

    def __init__(self):
        super().__init__('line_controller_node')

        # ── PID gains — lowered Kp so angular doesn't dominate linear ────────
        self.declare_parameter('Kp',           0.5)   # was 5 — too aggressive
        self.declare_parameter('Ki',           0.0)
        self.declare_parameter('Kd',           0.05)  # was 0.1

        # ── Velocity limits ───────────────────────────────────────────────────
        self.declare_parameter('base_speed',   0.15)  # was 0.10 — gives more linear authority
        self.declare_parameter('min_speed',    0.05)
        self.declare_parameter('max_angular',  1.0)   # was 2.0 — caps angular output

        # ── Safety timeout ────────────────────────────────────────────────────
        self.declare_parameter('timeout_sec',  0.5)

        Kp              = self.get_parameter('Kp').value
        Ki              = self.get_parameter('Ki').value
        Kd              = self.get_parameter('Kd').value
        self.base_speed = self.get_parameter('base_speed').value
        self.min_speed  = self.get_parameter('min_speed').value
        self.max_angular= self.get_parameter('max_angular').value
        self.timeout    = self.get_parameter('timeout_sec').value

        self.pid = PIDController(
            Kp=Kp, Ki=Ki, Kd=Kd,
            output_min=-self.max_angular,
            output_max= self.max_angular,
            integral_windup_limit=1.0
        )

        self._last_error_time  = None
        self._last_error_value = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.error_sub = self.create_subscription(
            Float32,
            '/line_error',
            self._error_callback,
            10
        )

        self._control_rate_hz = 50.0
        self.timer = self.create_timer(
            1.0 / self._control_rate_hz,
            self._control_loop
        )

        self.get_logger().info(
            f'LineControllerNode started | '
            f'PID: Kp={Kp}  Ki={Ki}  Kd={Kd} | '
            f'base_speed={self.base_speed} m/s  max_angular={self.max_angular} rad/s'
        )

    # ─────────────────────────────────────────────────────
    def _error_callback(self, msg: Float32):
        self._last_error_value = msg.data
        self._last_error_time  = time.monotonic()

    # ─────────────────────────────────────────────────────
    def _control_loop(self):
        now = time.monotonic()

        if self._last_error_time is None or (now - self._last_error_time) > self.timeout:
            self._publish_stop()
            if self._last_error_time is not None:
                self.get_logger().warn(
                    f'Sensor timeout ({self.timeout}s) — stopping robot.',
                    throttle_duration_sec=1.0
                )
            return

        error      = self._last_error_value
        pid_output = self.pid.update(-error, now)

        abs_error  = min(1.0, abs(error))
        linear_x   = self.base_speed * (1.0 - abs_error) + self.min_speed * abs_error
        linear_x   = max(self.min_speed, linear_x)

        cmd              = Twist()
        cmd.linear.x     = linear_x
        cmd.linear.y     = 0.0
        cmd.linear.z     = 0.0
        cmd.angular.x    = 0.0
        cmd.angular.y    = 0.0
        cmd.angular.z    = pid_output

        self.cmd_pub.publish(cmd)

        self.get_logger().debug(
            f'error={error:+.4f}  PID={pid_output:+.4f}  '
            f'v={linear_x:.3f} m/s  ω={pid_output:.3f} rad/s'
        )

    # ─────────────────────────────────────────────────────
    def _publish_stop(self):
        self.cmd_pub.publish(Twist())
        self.pid.reset()


# ─────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = LineControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LineControllerNode shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

