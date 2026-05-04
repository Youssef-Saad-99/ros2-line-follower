#!/usr/bin/env python3
"""
line_sensor_node.py
====================
PURPOSE:
    This node represents the SENSING layer of the line-following robot pipeline.
    It reads (or simulates) an IR sensor array mounted on the underside of the robot.
    Each sensor reports whether it detects a dark line (1) or bright surface (0).
    A weighted centroid formula converts those binary readings into a single scalar
    "error" value that tells the controller how far the robot is from the line center.

WHY THIS NODE EXISTS:
    Clean separation of concerns — hardware reading is isolated here so the controller
    never needs to know about sensor indices, noise filtering, or simulation logic.
    If you swap from simulated sensors to real hardware, only this file changes.

PUBLISHED TOPICS:
    /line_error  (std_msgs/Float32)
        Normalised error in range [-1.0, +1.0].
        Negative  → line is to the LEFT  of robot center.
        Positive  → line is to the RIGHT of robot center.
        Zero      → robot is perfectly centered on the line.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class LineSensorNode(Node):
    """
    ROS 2 node that simulates an array of IR line sensors and publishes
    the computed weighted centroid error on /line_error.
    """

    def __init__(self):
        # ------------------------------------------------------------------ #
        # 1. INITIALISE THE NODE
        #    'line_sensor_node' is the unique name used in the ROS graph.
        # ------------------------------------------------------------------ #
        super().__init__('line_sensor_node')

        # ------------------------------------------------------------------ #
        # 2. DECLARE ROS 2 PARAMETERS
        #    Parameters allow runtime tuning without recompiling.
        #    They can be set via the launch file or command line.
        # ------------------------------------------------------------------ #
        self.declare_parameter('num_sensors', 8)          # Number of IR sensors in the array
        self.declare_parameter('publish_rate_hz', 20.0)   # How often to publish (Hz)
        self.declare_parameter('simulate_noise', True)    # Add random noise to simulation

        # Read parameter values into local variables
        self.num_sensors     = self.get_parameter('num_sensors').value
        publish_rate         = self.get_parameter('publish_rate_hz').value
        self.simulate_noise  = self.get_parameter('simulate_noise').value

        # ------------------------------------------------------------------ #
        # 3. BUILD SENSOR WEIGHT ARRAY
        #    Sensors are numbered 0 … (N-1), left to right.
        #    We assign each sensor a "position" weight centred at zero:
        #      weights[i] = i - (N-1)/2
        #    Example for N=8: [-3.5, -2.5, -1.5, -0.5, +0.5, +1.5, +2.5, +3.5]
        #    A weighted centroid over active sensors gives the lateral error.
        # ------------------------------------------------------------------ #
        self.weights = [
            i - (self.num_sensors - 1) / 2.0
            for i in range(self.num_sensors)
        ]
        # Normalisation factor: half of the maximum possible absolute centroid
        # (when only the outermost sensor fires) so the output is in [-1, +1].
        self.norm_factor = self.weights[-1]   # = (N-1)/2

        # ------------------------------------------------------------------ #
        # 4. CREATE THE PUBLISHER
        #    /line_error carries a single Float32 value consumed by the
        #    line_controller_node.  Queue depth 10 handles brief bursts.
        # ------------------------------------------------------------------ #
        self.publisher_ = self.create_publisher(Float32, '/line_error', 10)

        # ------------------------------------------------------------------ #
        # 5. SIMULATION STATE
        #    We simulate a sinusoidal path deviation so the robot has something
        #    non-trivial to correct for, making the demo visually meaningful.
        # ------------------------------------------------------------------ #
        self._sim_time      = 0.0                     # Accumulated simulation time (s)
        self._sim_amplitude = 0.6                     # Peak lateral deviation (sensor units)
        self._sim_period    = 5.0                     # Period of the sinusoidal path (s)

        # ------------------------------------------------------------------ #
        # 6. CREATE A TIMER TO DRIVE THE PUBLISH LOOP
        #    ROS 2 timers fire a callback at a fixed period.
        #    Period = 1.0 / rate_hz.
        # ------------------------------------------------------------------ #
        timer_period = 1.0 / publish_rate
        self.timer   = self.create_timer(timer_period, self._sensor_callback)

        self.get_logger().info(
            f'LineSensorNode started | sensors={self.num_sensors} '
            f'rate={publish_rate} Hz | simulation mode active'
        )

    # ---------------------------------------------------------------------- #
    # HELPER: simulate_sensor_readings
    # ---------------------------------------------------------------------- #
    def _simulate_sensor_readings(self) -> list:
        """
        Simulate a Gaussian "hill" of sensor activations centred at a position
        that drifts sinusoidally over time.  This mimics a robot wandering off
        the line and slowly returning, giving the controller a meaningful signal.

        Returns:
            readings (list[float]): Activation level [0.0, 1.0] for each sensor.
        """
        import math, random

        # Current true lateral offset of the line centre (in sensor-index units)
        true_offset = self._sim_amplitude * math.sin(
            2.0 * math.pi * self._sim_time / self._sim_period
        )

        readings = []
        for i, w in enumerate(self.weights):
            # Gaussian response: peak at true_offset, sigma = 0.8 sensor widths
            activation = math.exp(-((w - true_offset) ** 2) / (2 * 0.8 ** 2))
            # Optional noise to simulate real sensor jitter
            if self.simulate_noise:
                activation += random.gauss(0, 0.02)
            # Clamp to [0, 1]
            activation = max(0.0, min(1.0, activation))
            readings.append(activation)

        return readings

    # ---------------------------------------------------------------------- #
    # HELPER: compute_weighted_centroid_error
    # ---------------------------------------------------------------------- #
    def _compute_weighted_centroid_error(self, readings: list) -> float:
        """
        Compute the weighted centroid of sensor activations to derive a
        single lateral error value.

        Formula:
            centroid = Σ(weight[i] * reading[i]) / Σ(reading[i])
            error    = centroid / norm_factor          ∈ [-1, +1]

        If no sensors are active (total_activation ≈ 0), the line is lost;
        we return the last valid error or 0.0 to avoid division by zero.

        Args:
            readings (list[float]): Per-sensor activation levels.

        Returns:
            error (float): Normalised lateral error in [-1.0, +1.0].
        """
        total_activation = sum(readings)

        if total_activation < 1e-6:
            # Line lost — return 0 (no corrective action) and warn
            self.get_logger().warn('Line lost! All sensor readings are near zero.')
            return 0.0

        # Weighted centroid in raw sensor-position units
        centroid = sum(w * r for w, r in zip(self.weights, readings)) / total_activation

        # Normalise so output is in [-1.0, +1.0]
        error = centroid / self.norm_factor
        return float(max(-1.0, min(1.0, error)))   # safety clamp

    # ---------------------------------------------------------------------- #
    # TIMER CALLBACK
    # ---------------------------------------------------------------------- #
    def _sensor_callback(self):
        """
        Main loop executed at `publish_rate_hz`.
        1. Advance simulation clock.
        2. Obtain sensor readings (simulated).
        3. Compute weighted centroid error.
        4. Publish the error on /line_error.
        """
        # Advance simulation time by one timer period
        # (we derive dt from the declared rate for consistency)
        dt = 1.0 / self.get_parameter('publish_rate_hz').value
        self._sim_time += dt

        # --- Step 1: Read (simulate) the sensor array ---
        readings = self._simulate_sensor_readings()

        # --- Step 2: Compute the scalar error ---
        error = self._compute_weighted_centroid_error(readings)

        # --- Step 3: Build and publish the ROS message ---
        msg       = Float32()
        msg.data  = error
        self.publisher_.publish(msg)

        self.get_logger().debug(
            f'Sensor readings: {[f"{r:.2f}" for r in readings]} | '
            f'Error: {error:+.4f}'
        )


# --------------------------------------------------------------------------- #
# ENTRY POINT
# --------------------------------------------------------------------------- #
def main(args=None):
    """
    Standard ROS 2 entry point.
    Initialises the rclpy context, spins the node, then cleans up.
    """
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