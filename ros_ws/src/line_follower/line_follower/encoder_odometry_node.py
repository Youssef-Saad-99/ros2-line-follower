#!/usr/bin/env python3
"""
encoder_odometry_node.py
========================
PURPOSE:
    This node represents the LOCALISATION layer of the pipeline.
    It reads (or simulates) quadrature wheel encoders and applies the
    differential-drive kinematic model to maintain a running estimate of the
    robot's pose (x, y, θ) in a 2-D plane.

    The result is published as a nav_msgs/Odometry message on /odom,
    which is the de-facto ROS standard for wheel-based dead-reckoning.

WHY THIS NODE EXISTS:
    Odometry is needed for:
      • Logging and monitoring where the robot has been.
      • Higher-level navigation nodes that may subscribe to /odom.
      • Debugging — if the robot drifts, odom reveals systematic encoder errors.

DIFFERENTIAL-DRIVE KINEMATICS (implemented here):
    Given left/right angular velocities ω_L, ω_R (rad/s) and:
        r = wheel radius (m)
        L = wheel base / track width (m)

    Linear  velocity:  v   = (r / 2) * (ω_R + ω_L)
    Angular velocity:  ω   = (r / L) * (ω_R - ω_L)

    Pose integration over time step dt:
        x     += v * cos(θ) * dt
        y     += v * sin(θ) * dt
        θ     += ω * dt

PUBLISHED TOPICS:
    /odom  (nav_msgs/Odometry)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
# tf2_ros is used to broadcast the odom → base_link transform (standard practice)
try:
    import tf2_ros
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


class EncoderOdometryNode(Node):
    """
    ROS 2 node that computes and publishes differential-drive odometry
    from simulated wheel encoder data.
    """

    def __init__(self):
        # ------------------------------------------------------------------ #
        # 1. INITIALISE THE NODE
        # ------------------------------------------------------------------ #
        super().__init__('encoder_odometry_node')

        # ------------------------------------------------------------------ #
        # 2. DECLARE ROS 2 PARAMETERS
        #    These match the physical robot dimensions and must be consistent
        #    with the motor_driver_node and line_controller_node parameters.
        # ------------------------------------------------------------------ #
        self.declare_parameter('wheel_radius',   0.033)   # metres  (r)
        self.declare_parameter('wheel_base',     0.160)   # metres  (L) — distance between wheel centres
        self.declare_parameter('encoder_cpr',    360)     # counts per revolution (simulated encoder resolution)
        self.declare_parameter('publish_rate_hz', 50.0)   # Hz — higher than control loop for smooth odom

        # Read into local variables
        self.r        = self.get_parameter('wheel_radius').value
        self.L        = self.get_parameter('wheel_base').value
        self.cpr      = self.get_parameter('encoder_cpr').value
        publish_rate  = self.get_parameter('publish_rate_hz').value

        # ------------------------------------------------------------------ #
        # 3. ROBOT POSE STATE  (maintained across timer ticks)
        #    x, y    → position in the odom frame (metres)
        #    theta   → heading angle (radians), measured CCW from +x axis
        # ------------------------------------------------------------------ #
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # ------------------------------------------------------------------ #
        # 4. SIMULATION STATE
        #    We simulate encoder ticks that correspond to a robot driving
        #    forward at ~0.2 m/s with a gentle left curve (constant ω_R > ω_L).
        #    This ensures /odom carries realistic, time-varying data.
        # ------------------------------------------------------------------ #
        self._sim_time       = 0.0
        self._base_speed_rps = 0.6      # base wheel speed in revolutions/second
        self._curve_bias     = 0.08     # differential bias for the gentle curve (rps)

        # ------------------------------------------------------------------ #
        # 5. CREATE PUBLISHER
        # ------------------------------------------------------------------ #
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # ------------------------------------------------------------------ #
        # 6. OPTIONAL: TF2 BROADCASTER
        #    Best practice is to also broadcast the odom → base_link transform
        #    so tools like rviz2 can visualise the robot pose without
        #    subscribing to /odom directly.
        # ------------------------------------------------------------------ #
        if TF2_AVAILABLE:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self.get_logger().info('TF2 broadcaster initialised.')
        else:
            self.tf_broadcaster = None
            self.get_logger().warn(
                'tf2_ros not found — TF transform will not be broadcast. '
                'Install tf2_ros for full ROS integration.'
            )

        # ------------------------------------------------------------------ #
        # 7. TIMER
        # ------------------------------------------------------------------ #
        dt           = 1.0 / publish_rate
        self._dt     = dt
        self.timer   = self.create_timer(dt, self._odom_callback)

        self.get_logger().info(
            f'EncoderOdometryNode started | r={self.r} m  L={self.L} m  '
            f'rate={publish_rate} Hz'
        )

    # ---------------------------------------------------------------------- #
    # HELPER: simulate_wheel_speeds
    # ---------------------------------------------------------------------- #
    def _simulate_wheel_speeds(self) -> tuple:
        """
        Simulate left and right wheel angular velocities (rad/s).

        We model a robot that drives forward while gently curving left/right
        in a sinusoidal pattern to represent realistic line-following motion.

        Returns:
            (omega_left, omega_right) in rad/s
        """
        import math

        # Sinusoidal differential to mimic line-following corrections
        bias = self._curve_bias * math.sin(2.0 * math.pi * self._sim_time / 4.0)

        omega_left  = (self._base_speed_rps - bias) * 2.0 * math.pi   # convert rps → rad/s
        omega_right = (self._base_speed_rps + bias) * 2.0 * math.pi

        return omega_left, omega_right

    # ---------------------------------------------------------------------- #
    # HELPER: euler_to_quaternion
    # ---------------------------------------------------------------------- #
    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        Convert Euler angles (roll, pitch, yaw) to a ROS Quaternion message.

        For a ground robot roll=0, pitch=0 always, so only yaw matters.
        Using the ZYX (yaw-pitch-roll) convention:
            q = q_z(yaw) * q_y(pitch) * q_x(roll)

        Args:
            roll, pitch, yaw: Rotation angles in radians.

        Returns:
            geometry_msgs/Quaternion
        """
        cy = math.cos(yaw   * 0.5)
        sy = math.sin(yaw   * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll  * 0.5)
        sr = math.sin(roll  * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    # ---------------------------------------------------------------------- #
    # TIMER CALLBACK
    # ---------------------------------------------------------------------- #
    def _odom_callback(self):
        """
        Called at `publish_rate_hz`.
        1. Read (simulate) wheel angular velocities.
        2. Apply differential-drive kinematics to compute v and ω.
        3. Integrate pose over dt.
        4. Publish Odometry message.
        5. Broadcast TF transform (if tf2 available).
        """
        # --- Advance simulation clock ---
        self._sim_time += self._dt

        # --- Step 1: Get wheel angular velocities (rad/s) ---
        omega_L, omega_R = self._simulate_wheel_speeds()

        # --- Step 2: Differential-drive kinematics ---
        # Linear velocity of the robot centre (m/s)
        #   v = (r/2) * (ω_R + ω_L)
        v = (self.r / 2.0) * (omega_R + omega_L)

        # Angular velocity of the robot (rad/s)
        #   ω = (r/L) * (ω_R - ω_L)
        omega = (self.r / self.L) * (omega_R - omega_L)

        # --- Step 3: Integrate pose ---
        # For small dt, the forward-Euler approximation is accurate enough.
        # For large angular rates, a midpoint or Runge-Kutta integrator
        # would be more appropriate, but is overkill here.
        delta_x     =  v * math.cos(self.theta) * self._dt
        delta_y     =  v * math.sin(self.theta) * self._dt
        delta_theta =  omega * self._dt

        self.x     += delta_x
        self.y     += delta_y
        self.theta += delta_theta

        # Normalise theta to [-π, +π] to prevent unbounded growth
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # --- Step 4: Build and publish Odometry message ---
        now = self.get_clock().now().to_msg()

        odom                        = Odometry()
        odom.header.stamp           = now
        odom.header.frame_id        = 'odom'          # World-fixed odometry frame
        odom.child_frame_id         = 'base_link'     # Robot body frame

        # Pose in the odom frame
        odom.pose.pose.position.x   = self.x
        odom.pose.pose.position.y   = self.y
        odom.pose.pose.position.z   = 0.0
        odom.pose.pose.orientation  = self._euler_to_quaternion(0.0, 0.0, self.theta)

        # Twist in the base_link frame (robot-local velocities)
        odom.twist.twist.linear.x   = v
        odom.twist.twist.linear.y   = 0.0      # No lateral slip assumed
        odom.twist.twist.angular.z  = omega

        # Covariance matrices (6x6 row-major) — use small diagonal values for
        # a simulated "perfect" sensor; real systems tune these from calibration.
        # Indices: [x, y, z, roll, pitch, yaw]
        odom.pose.covariance[0]  = 0.001    # x variance
        odom.pose.covariance[7]  = 0.001    # y variance
        odom.pose.covariance[35] = 0.001    # yaw variance
        odom.twist.covariance[0] = 0.001    # vx variance
        odom.twist.covariance[35]= 0.001    # omega_z variance

        self.odom_pub.publish(odom)

        # --- Step 5: Broadcast TF transform ---
        if self.tf_broadcaster is not None:
            tf_msg                        = TransformStamped()
            tf_msg.header.stamp           = now
            tf_msg.header.frame_id        = 'odom'
            tf_msg.child_frame_id         = 'base_link'
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation     = self._euler_to_quaternion(0.0, 0.0, self.theta)
            self.tf_broadcaster.sendTransform(tf_msg)

        self.get_logger().debug(
            f'Odom | x={self.x:.3f}  y={self.y:.3f}  θ={math.degrees(self.theta):.1f}°  '
            f'v={v:.3f} m/s  ω={omega:.3f} rad/s'
        )


# --------------------------------------------------------------------------- #
# ENTRY POINT
# --------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('EncoderOdometryNode shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
