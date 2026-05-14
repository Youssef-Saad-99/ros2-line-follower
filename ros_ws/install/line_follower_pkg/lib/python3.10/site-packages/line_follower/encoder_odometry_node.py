#!/usr/bin/env python3
"""
encoder_odometry_node.py
========================
Subscribes to /encoder_raw (std_msgs/String, format "E:left,right"),
applies differential-drive kinematics, and publishes nav_msgs/Odometry
on /odom.

This node has NO serial dependency — all hardware I/O is handled by
serial_comm_node.py.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import String

try:
    import tf2_ros
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


class EncoderOdometryNode(Node):

    def __init__(self):
        super().__init__('encoder_odometry_node')

        # ------------------------------------------------------------------ #
        # 1. PARAMETERS
        # ------------------------------------------------------------------ #
        self.declare_parameter('wheel_radius',        0.033)
        self.declare_parameter('wheel_base',          0.160)
        self.declare_parameter('encoder_ppr',         20)
        self.declare_parameter('avr_send_interval_s', 0.050)  # must match AVR firmware
        self.declare_parameter('publish_rate_hz',     20.0)

        self._r      = self.get_parameter('wheel_radius').value
        self._L      = self.get_parameter('wheel_base').value
        self._ppr    = self.get_parameter('encoder_ppr').value
        self._avr_dt = self.get_parameter('avr_send_interval_s').value
        rate_hz      = self.get_parameter('publish_rate_hz').value

        self._dist_per_tick = (2.0 * math.pi * self._r) / self._ppr

        # ------------------------------------------------------------------ #
        # 2. STATE
        # ------------------------------------------------------------------ #
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0

        self._left_count  = 0
        self._right_count = 0
        self._new_data    = False

        self._prev_left  = None
        self._prev_right = None

        # ------------------------------------------------------------------ #
        # 3. SUBSCRIBER
        # ------------------------------------------------------------------ #
        self.create_subscription(
            String, '/encoder_raw', self._encoder_cb, 10
        )

        # ------------------------------------------------------------------ #
        # 4. PUBLISHER + TF
        # ------------------------------------------------------------------ #
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        if TF2_AVAILABLE:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        else:
            self._tf_broadcaster = None
            self.get_logger().warn('tf2_ros not found — TF will not be broadcast.')

        # ------------------------------------------------------------------ #
        # 5. TIMER
        # ------------------------------------------------------------------ #
        self._dt = 1.0 / rate_hz
        self.create_timer(self._dt, self._odom_callback)

        self.get_logger().info(
            f'EncoderOdometryNode ready | '
            f'r={self._r} m  L={self._L} m  PPR={self._ppr}  '
            f'rate={rate_hz} Hz'
        )

    # ---------------------------------------------------------------------- #
    # SUBSCRIBER CALLBACK
    # ---------------------------------------------------------------------- #
    def _encoder_cb(self, msg: String):
        """Parse 'E:left,right' and store latest counts."""
        line = msg.data.strip()

        if not line.startswith('E:'):
            return

        parts = line[2:].split(',')
        if len(parts) != 2:
            self.get_logger().warn(f'Malformed encoder line: "{line}"')
            return

        try:
            self._left_count  = int(parts[0])
            self._right_count = int(parts[1])
            self._new_data    = True
        except ValueError:
            self.get_logger().warn(f'Non-integer encoder values: "{line}"')

    # ---------------------------------------------------------------------- #
    # TIMER CALLBACK
    # ---------------------------------------------------------------------- #
    def _odom_callback(self):
        # Snapshot + reset flag
        left_count  = self._left_count
        right_count = self._right_count
        new_data    = self._new_data
        if new_data:
            self._new_data = False

        # Wait for first real packet
        if self._prev_left is None:
            if new_data:
                self._prev_left  = left_count
                self._prev_right = right_count
            return

        # Tick deltas
        delta_left  = left_count  - self._prev_left
        delta_right = right_count - self._prev_right

        if new_data:
            self._prev_left  = left_count
            self._prev_right = right_count

            # Wheel arc lengths (m)
            d_left  = delta_left  * self._dist_per_tick
            d_right = delta_right * self._dist_per_tick

            # Differential-drive kinematics
            d_centre = (d_left + d_right) / 2.0
            d_theta  = (d_right - d_left) / self._L

            # Midpoint-heading integration
            mid_theta    = self._theta + d_theta / 2.0
            self._x     += d_centre * math.cos(mid_theta)
            self._y     += d_centre * math.sin(mid_theta)
            self._theta += d_theta

            # Wrap theta to [-pi, +pi]
            self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        # Velocity estimates
        v     = 0.0
        omega = 0.0
        if new_data:
            d_c   = (delta_left + delta_right) * self._dist_per_tick / 2.0
            d_th  = (delta_right - delta_left) * self._dist_per_tick / self._L
            v     = d_c  / self._avr_dt
            omega = d_th / self._avr_dt

        # Build Odometry message
        now  = self.get_clock().now().to_msg()
        quat = self._yaw_to_quat(self._theta)

        odom                       = Odometry()
        odom.header.stamp          = now
        odom.header.frame_id       = 'odom'
        odom.child_frame_id        = 'base_link'
        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x  = v
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = omega

        odom.pose.covariance[0]  = 0.005   # x
        odom.pose.covariance[7]  = 0.005   # y
        odom.pose.covariance[35] = 0.010   # yaw
        odom.twist.covariance[0] = 0.005   # vx
        odom.twist.covariance[35]= 0.010   # omega_z

        self._odom_pub.publish(odom)

        # TF broadcast
        if self._tf_broadcaster is not None:
            tf_msg                         = TransformStamped()
            tf_msg.header.stamp            = now
            tf_msg.header.frame_id         = 'odom'
            tf_msg.child_frame_id          = 'base_link'
            tf_msg.transform.translation.x = self._x
            tf_msg.transform.translation.y = self._y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation      = quat
            self._tf_broadcaster.sendTransform(tf_msg)

        self.get_logger().debug(
            f'x={self._x:.3f}  y={self._y:.3f}  '
            f'theta={math.degrees(self._theta):.1f} deg  '
            f'v={v:.3f} m/s  omega={omega:.3f} rad/s'
        )

    # ---------------------------------------------------------------------- #
    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q   = Quaternion()
        q.w = math.cos(yaw * 0.5)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        return q

# =========================================================================== #
def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

