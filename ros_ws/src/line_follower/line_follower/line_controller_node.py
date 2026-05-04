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
    """
    A simple, reusable discrete PID controller.

    This class is intentionally decoupled from ROS so it can be unit-tested
    independently.  The node instantiates it and feeds it error values.
    """

    def __init__(self, Kp: float, Ki: float, Kd: float,
                 output_min: float = -1.0, output_max: float = 1.0,
                 integral_windup_limit: float = 1.0):
        """
        Args:
            Kp, Ki, Kd        : PID gains.
            output_min/max    : Output saturation limits (anti-windup via clamping).
            integral_windup_limit: Max absolute value of the integral accumulator
                                   to prevent integral windup on prolonged errors.
        """
        # --- Gains ---
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # --- Output limits ---
        self.output_min = output_min
        self.output_max = output_max

        # --- Anti-windup limit on the integral term ---
        self.windup_limit = integral_windup_limit

        # --- State variables (reset on each control session) ---
        self._integral       = 0.0    # Accumulated integral
        self._prev_error     = 0.0    # Error from the previous time step (for derivative)
        self._prev_timestamp = None   # Wall-clock time of the last update call

    def reset(self):
        """Reset controller state — call when the robot re-acquires the line."""
        self._integral       = 0.0
        self._prev_error     = 0.0
        self._prev_timestamp = None

    def update(self, error: float, timestamp: float) -> float:
        """
        Compute the PID output for the current error sample.

        Args:
            error     : Current tracking error (positive = line to the right).
            timestamp : Current time in seconds (monotonic wall clock).

        Returns:
            output (float): PID output, clamped to [output_min, output_max].
        """
        # --- Compute dt ---
        if self._prev_timestamp is None:
            # First call — no derivative available, skip D term
            dt = 0.0
        else:
            dt = timestamp - self._prev_timestamp
            dt = max(dt, 1e-6)   # Guard against zero or negative dt

        # --- Proportional term ---
        p_term = self.Kp * error

        # --- Integral term (with anti-windup clamping) ---
        self._integral += error * dt
        # Clamp integral accumulator to prevent saturation on prolonged errors
        self._integral = max(-self.windup_limit,
                             min(self.windup_limit, self._integral))
        i_term = self.Ki * self._integral

        # --- Derivative term (backward difference) ---
        if dt > 0:
            d_term = self.Kd * (error - self._prev_error) / dt
        else:
            d_term = 0.0

        # --- Sum and clamp ---
        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))

        # --- Save state for next iteration ---
        self._prev_error     = error
        self._prev_timestamp = timestamp

        return output


class LineControllerNode(Node):
    """
    ROS 2 node that runs the PID line-following control loop.

    Receives lateral error on /line_error and publishes corrective
    velocity commands on /cmd_vel.
    """

    def __init__(self):
        # ------------------------------------------------------------------ #
        # 1. INITIALISE THE NODE
        # ------------------------------------------------------------------ #
        super().__init__('line_controller_node')

        # ------------------------------------------------------------------ #
        # 2. DECLARE ROS 2 PARAMETERS
        #    All gains and speed limits are tunable at runtime.
        # ------------------------------------------------------------------ #
        # PID gains — tune these for your physical robot
        self.declare_parameter('Kp',           5)   # Proportional gain
        self.declare_parameter('Ki',           0)  # Integral gain (small to avoid windup)
        self.declare_parameter('Kd',           0.1)   # Derivative gain (dampens oscillation)

        # Velocity limits
        self.declare_parameter('base_speed',   0.10)  # Nominal forward speed (m/s)
        self.declare_parameter('min_speed',    0.05)  # Minimum forward speed (m/s) — prevents stall
        self.declare_parameter('max_angular',  2.0)   # Maximum angular velocity (rad/s)

        # Safety: if no error message received for this many seconds, stop
        self.declare_parameter('timeout_sec',  0.5)

        # Read parameters
        Kp              = self.get_parameter('Kp').value
        Ki              = self.get_parameter('Ki').value
        Kd              = self.get_parameter('Kd').value
        self.base_speed = self.get_parameter('base_speed').value
        self.min_speed  = self.get_parameter('min_speed').value
        self.max_angular= self.get_parameter('max_angular').value
        self.timeout    = self.get_parameter('timeout_sec').value

        # ------------------------------------------------------------------ #
        # 3. INSTANTIATE PID CONTROLLER
        #    Output limits match the max_angular parameter.
        # ------------------------------------------------------------------ #
        self.pid = PIDController(
            Kp=Kp, Ki=Ki, Kd=Kd,
            output_min=-self.max_angular,
            output_max= self.max_angular,
            integral_windup_limit=1.0
        )

        # ------------------------------------------------------------------ #
        # 4. INTERNAL STATE
        # ------------------------------------------------------------------ #
        self._last_error_time  = None    # Time of last /line_error message
        self._last_error_value = 0.0     # Latest error received

        # ------------------------------------------------------------------ #
        # 5. PUBLISHER — sends Twist commands to motor_driver_node
        # ------------------------------------------------------------------ #
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ------------------------------------------------------------------ #
        # 6. SUBSCRIBER — receives lateral error from line_sensor_node
        # ------------------------------------------------------------------ #
        self.error_sub = self.create_subscription(
            Float32,
            '/line_error',
            self._error_callback,
            10                  # QoS queue depth
        )

        # ------------------------------------------------------------------ #
        # 7. CONTROL LOOP TIMER
        #    The control loop runs at its own rate (50 Hz) independent of
        #    the sensor publish rate.  This is a best practice: the controller
        #    always runs and emits zero velocity if the sensor times out,
        #    rather than only running when a message arrives.
        # ------------------------------------------------------------------ #
        self._control_rate_hz = 50.0
        self.timer = self.create_timer(
            1.0 / self._control_rate_hz,
            self._control_loop
        )

        self.get_logger().info(
            f'LineControllerNode started | '
            f'PID: Kp={Kp}  Ki={Ki}  Kd={Kd} | '
            f'base_speed={self.base_speed} m/s'
        )

    # ---------------------------------------------------------------------- #
    # SUBSCRIBER CALLBACK: /line_error
    # ---------------------------------------------------------------------- #
    def _error_callback(self, msg: Float32):
        """
        Called whenever a new Float32 message arrives on /line_error.
        Simply stores the value and timestamp — the actual PID computation
        happens in the separate control loop timer callback.
        """
        self._last_error_value = msg.data
        self._last_error_time  = time.monotonic()

    # ---------------------------------------------------------------------- #
    # CONTROL LOOP CALLBACK
    # ---------------------------------------------------------------------- #
    def _control_loop(self):
        """
        Main control loop, executed at `_control_rate_hz` Hz.

        1. Check sensor timeout — stop robot if sensor data is stale.
        2. Run PID to compute angular velocity correction.
        3. Compute forward speed (reduce speed on sharp curves for stability).
        4. Publish Twist to /cmd_vel.
        """
        now = time.monotonic()

        # --- Safety check: sensor timeout ---
        if self._last_error_time is None or (now - self._last_error_time) > self.timeout:
            # No sensor data — publish zero velocity to halt the robot
            self._publish_stop()
            if self._last_error_time is not None:
                self.get_logger().warn(
                    f'Sensor timeout ({self.timeout}s) — stopping robot.',
                    throttle_duration_sec=1.0
                )
            return

        # --- Step 1: Get the current lateral error ---
        error = self._last_error_value
        # error > 0  → line is to the RIGHT → robot must turn RIGHT → negative angular_z
        # error < 0  → line is to the LEFT  → robot must turn LEFT  → positive angular_z
        # Convention: angular_z positive = CCW = turning left in ROS.
        # So the PID output should be the NEGATIVE of the error for correct steering.
        # We pass -error into PID, or equivalently, negate the output.

        # --- Step 2: PID update ---
        # We pass the raw error; the PID output gives the magnitude of correction.
        # Negating here maps "error to the right" → "turn right" (negative angular_z).
        pid_output = self.pid.update(-error, now)

        # --- Step 3: Compute forward speed ---
        # Intuition: slow down proportionally to |error| so sharp curves are
        # taken carefully.  |error| = 1.0 → drop to min_speed.
        abs_error   = min(1.0, abs(error))
        linear_x    = self.base_speed * (1.0 - abs_error) + self.min_speed * abs_error
        linear_x    = max(self.min_speed, linear_x)   # floor at min_speed

        # --- Step 4: Publish Twist message ---
        cmd              = Twist()
        cmd.linear.x     = linear_x
        cmd.linear.y     = 0.0
        cmd.linear.z     = 0.0
        cmd.angular.x    = 0.0
        cmd.angular.y    = 0.0
        cmd.angular.z    = pid_output    # PID-computed steering correction

        self.cmd_pub.publish(cmd)

        self.get_logger().debug(
            f'Control | error={error:+.4f}  PID_out={pid_output:+.4f}  '
            f'v={linear_x:.3f} m/s  ω={pid_output:.3f} rad/s'
        )

    # ---------------------------------------------------------------------- #
    # HELPER: publish a zero-velocity stop command
    # ---------------------------------------------------------------------- #
    def _publish_stop(self):
        """Send a zero-velocity Twist — safe fallback when line is lost."""
        self.cmd_pub.publish(Twist())
        self.pid.reset()    # Reset integral/derivative state


# --------------------------------------------------------------------------- #
# ENTRY POINT
# --------------------------------------------------------------------------- #
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