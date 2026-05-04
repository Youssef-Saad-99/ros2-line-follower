#!/usr/bin/env python3
"""
motor_driver_node.py
=====================
PURPOSE:
    This node is the ACTUATION layer of the pipeline.
    It subscribes to /cmd_vel (geometry_msgs/Twist) and converts the
    high-level velocity command into per-wheel PWM duty-cycle values
    that would be sent to the physical motor driver (e.g. L298N, TB6612).

    On a real robot these PWM values drive GPIO pins via a library such as
    RPi.GPIO or gpiozero.  In simulation mode (default) they are logged only.

CONVERSION MATHS:
    Given:
        v       = cmd_vel.linear.x   (desired forward speed, m/s)
        omega   = cmd_vel.angular.z  (desired angular velocity, rad/s)
        r       = wheel radius (m)
        L       = wheel base (m)
        PWM_MAX = 255                (8-bit PWM resolution)

    Inverse differential-drive:
        omega_L = (v - (L/2) * omega) / r      → left  wheel angular velocity (rad/s)
        omega_R = (v + (L/2) * omega) / r      → right wheel angular velocity (rad/s)

    Map angular velocity to PWM duty cycle:
        pwm_L = clip(omega_L / omega_max * PWM_MAX, -PWM_MAX, +PWM_MAX)
        pwm_R = clip(omega_R / omega_max * PWM_MAX, -PWM_MAX, +PWM_MAX)

    The sign of pwm encodes direction:
        positive → forward rotation
        negative → reverse rotation (IN1=LOW, IN2=HIGH on L298N)

SUBSCRIBED TOPICS:
    /cmd_vel  (geometry_msgs/Twist)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorDriverNode(Node):
    """
    ROS 2 node that converts Twist commands into left/right PWM signals
    for a differential-drive robot.  Hardware GPIO is simulated via logging.
    """

    def __init__(self):
        # ------------------------------------------------------------------ #
        # 1. INITIALISE THE NODE
        # ------------------------------------------------------------------ #
        super().__init__('motor_driver_node')

        # ------------------------------------------------------------------ #
        # 2. DECLARE ROS 2 PARAMETERS
        # ------------------------------------------------------------------ #
        self.declare_parameter('wheel_radius',    0.033)   # metres  (r)
        self.declare_parameter('wheel_base',      0.160)   # metres  (L)
        self.declare_parameter('max_wheel_speed', 3.0)     # rad/s — max angular speed of a wheel
        self.declare_parameter('pwm_max',         255)     # 8-bit PWM maximum value
        self.declare_parameter('simulate_gpio',   True)    # True  = log only, False = real GPIO

        # Read parameters
        self.r               = self.get_parameter('wheel_radius').value
        self.L               = self.get_parameter('wheel_base').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.pwm_max         = self.get_parameter('pwm_max').value
        self.simulate_gpio   = self.get_parameter('simulate_gpio').value

        # ------------------------------------------------------------------ #
        # 3. GPIO INITIALISATION (hardware path)
        #    Only executed when simulate_gpio is False AND the library is
        #    available.  Falls back to simulation if import fails.
        # ------------------------------------------------------------------ #
        self._gpio_ready = False
        if not self.simulate_gpio:
            self._init_gpio()
        else:
            self.get_logger().info(
                'GPIO simulation mode active — no hardware output will be generated.'
            )

        # ------------------------------------------------------------------ #
        # 4. SUBSCRIBER — receives velocity commands from the controller
        # ------------------------------------------------------------------ #
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10          # QoS queue depth — small because we only care about latest command
        )

        # ------------------------------------------------------------------ #
        # 5. WATCHDOG TIMER
        #    If no /cmd_vel arrives for `watchdog_timeout` seconds, the robot
        #    is stopped immediately.  This prevents run-away if the controller
        #    crashes or the network drops.
        # ------------------------------------------------------------------ #
        self.declare_parameter('watchdog_timeout_sec', 1.0)
        self._watchdog_timeout = self.get_parameter('watchdog_timeout_sec').value
        self._last_cmd_time    = self.get_clock().now()
        self._watchdog_timer   = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info(
            f'MotorDriverNode started | r={self.r} m  L={self.L} m  '
            f'PWM_MAX={self.pwm_max}  simulate={self.simulate_gpio}'
        )

    # ---------------------------------------------------------------------- #
    # GPIO INITIALISATION (hardware path — skipped in simulation)
    # ---------------------------------------------------------------------- #
    def _init_gpio(self):
        """
        Attempt to import RPi.GPIO and configure motor control pins.
        Pin assignments follow a typical L298N wiring:
            Left  motor: IN1=17, IN2=27, ENA=18 (PWM)
            Right motor: IN3=22, IN4=23, ENB=24 (PWM)
        Adjust pin numbers to match your hardware.
        """
        try:
            import RPi.GPIO as GPIO   # type: ignore

            self._GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            # Left motor pins
            self._L_IN1 = 17; self._L_IN2 = 27; self._L_EN = 18
            # Right motor pins
            self._R_IN3 = 22; self._R_IN4 = 23; self._R_EN = 24

            for pin in [self._L_IN1, self._L_IN2, self._L_EN,
                        self._R_IN3, self._R_IN4, self._R_EN]:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)

            # Set up hardware PWM at 1 kHz on the enable pins
            self._pwm_left  = GPIO.PWM(self._L_EN, 1000)
            self._pwm_right = GPIO.PWM(self._R_EN, 1000)
            self._pwm_left.start(0)
            self._pwm_right.start(0)

            self._gpio_ready = True
            self.get_logger().info('RPi.GPIO initialised — hardware mode active.')

        except ImportError:
            self.get_logger().warn(
                'RPi.GPIO not found.  Falling back to GPIO simulation mode.'
            )
            self.simulate_gpio = True

    # ---------------------------------------------------------------------- #
    # SUBSCRIBER CALLBACK: /cmd_vel
    # ---------------------------------------------------------------------- #
    def _cmd_vel_callback(self, msg: Twist):
        """
        Called when a new Twist message arrives on /cmd_vel.

        Steps:
        1. Extract linear and angular velocity from the Twist.
        2. Apply inverse differential-drive kinematics.
        3. Map wheel speeds to PWM duty cycles.
        4. Output PWM (hardware) or log the values (simulation).
        """
        self._last_cmd_time = self.get_clock().now()

        v     = msg.linear.x     # Forward speed (m/s)
        omega = msg.angular.z    # Angular velocity (rad/s), positive = CCW

        # --- Inverse differential-drive kinematics ---
        # Solve for per-wheel angular velocity:
        #   omega_L = (v - (L/2)*omega) / r
        #   omega_R = (v + (L/2)*omega) / r
        omega_L = (v - (self.L / 2.0) * omega) / self.r
        omega_R = (v + (self.L / 2.0) * omega) / self.r

        # --- Map to PWM duty cycle (0 – PWM_MAX) ---
        # Scale by max_wheel_speed so the full PWM range is utilised at peak speed.
        # The sign tells us motor direction.
        raw_L = (omega_L / self.max_wheel_speed) * self.pwm_max
        raw_R = (omega_R / self.max_wheel_speed) * self.pwm_max

        # Clamp to [-PWM_MAX, +PWM_MAX]
        pwm_L = int(max(-self.pwm_max, min(self.pwm_max, raw_L)))
        pwm_R = int(max(-self.pwm_max, min(self.pwm_max, raw_R)))

        # --- Drive motors ---
        self._set_motor_pwm(pwm_L, pwm_R)

        self.get_logger().debug(
            f'CMD_VEL → v={v:.3f} m/s  ω={omega:.3f} rad/s | '
            f'ω_L={omega_L:.3f} rad/s  ω_R={omega_R:.3f} rad/s | '
            f'PWM_L={pwm_L}  PWM_R={pwm_R}'
        )

    # ---------------------------------------------------------------------- #
    # HELPER: set_motor_pwm
    # ---------------------------------------------------------------------- #
    def _set_motor_pwm(self, pwm_left: int, pwm_right: int):
        """
        Apply PWM values to the motors.

        On hardware: sets GPIO direction pins and changes PWM duty cycle.
        In simulation: logs the intended signals.

        Args:
            pwm_left  (int): Left  motor PWM in [-PWM_MAX, +PWM_MAX].
                             Positive = forward, negative = reverse.
            pwm_right (int): Right motor PWM in [-PWM_MAX, +PWM_MAX].
        """
        if self.simulate_gpio or not self._gpio_ready:
            # --- SIMULATION PATH ---
            # Compute the duty cycle percentage for display
            duty_L = abs(pwm_left)  / self.pwm_max * 100.0
            duty_R = abs(pwm_right) / self.pwm_max * 100.0
            dir_L  = 'FWD' if pwm_left  >= 0 else 'REV'
            dir_R  = 'FWD' if pwm_right >= 0 else 'REV'
            self.get_logger().info(
                f'[SIM PWM] LEFT  {dir_L} {duty_L:5.1f}%  |  '
                f'RIGHT {dir_R} {duty_R:5.1f}%',
                throttle_duration_sec=0.2    # Limit log spam to 5 Hz
            )
            return

        # --- HARDWARE PATH (RPi.GPIO) ---
        GPIO = self._GPIO

        # Left motor direction
        if pwm_left >= 0:
            GPIO.output(self._L_IN1, GPIO.HIGH)
            GPIO.output(self._L_IN2, GPIO.LOW)
        else:
            GPIO.output(self._L_IN1, GPIO.LOW)
            GPIO.output(self._L_IN2, GPIO.HIGH)

        # Right motor direction
        if pwm_right >= 0:
            GPIO.output(self._R_IN3, GPIO.HIGH)
            GPIO.output(self._R_IN4, GPIO.LOW)
        else:
            GPIO.output(self._R_IN3, GPIO.LOW)
            GPIO.output(self._R_IN4, GPIO.HIGH)

        # Set duty cycles (RPi.GPIO expects 0.0 – 100.0)
        duty_L = abs(pwm_left)  / self.pwm_max * 100.0
        duty_R = abs(pwm_right) / self.pwm_max * 100.0
        self._pwm_left.ChangeDutyCycle(duty_L)
        self._pwm_right.ChangeDutyCycle(duty_R)

    # ---------------------------------------------------------------------- #
    # WATCHDOG CALLBACK
    # ---------------------------------------------------------------------- #
    def _watchdog_callback(self):
        """
        Periodic check: if no /cmd_vel message has arrived within
        `watchdog_timeout_sec`, stop both motors immediately.
        This prevents uncontrolled movement if the controller crashes.
        """
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > self._watchdog_timeout:
            self._set_motor_pwm(0, 0)
            self.get_logger().warn(
                f'Watchdog: no /cmd_vel for {elapsed:.2f}s — motors stopped.',
                throttle_duration_sec=2.0
            )

    # ---------------------------------------------------------------------- #
    # CLEANUP
    # ---------------------------------------------------------------------- #
    def destroy_node(self):
        """Ensure motors are stopped and GPIO is released on shutdown."""
        self._set_motor_pwm(0, 0)
        if self._gpio_ready:
            self._pwm_left.stop()
            self._pwm_right.stop()
            self._GPIO.cleanup()
            self.get_logger().info('GPIO cleaned up.')
        super().destroy_node()


# --------------------------------------------------------------------------- #
# ENTRY POINT
# --------------------------------------------------------------------------- #
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