"""
sensor_node.py

This node simulates IR sensor readings for a line follower robot,
computes the line error, and publishes it to a ROS 2 topic.

Topic Published:
    /line_error (std_msgs/Float32)

Author: Your Team
"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class SensorNode(Node):
    """
    ROS2 Node responsible for:
    1. Reading sensor data (simulated for now)
    2. Computing line error
    3. Publishing the error to /line_error topic
    """

    def __init__(self):
        """
        Initialize the SensorNode:
        - Create publisher
        - Create timer loop
        """
        super().__init__('line_sensor_node')

        # Publisher for line error
        self.publisher_ = self.create_publisher(Float32, '/line_error', 10)

        # Timer to run node periodically (every 0.5 seconds)
        self.timer = self.create_timer(0.05, self.run)

    # --------------------------------------------------
    def read_sensors(self):
        """
        Simulate IR sensor readings.

        Returns:
            list[int]: A list of sensor values (0 or 1)
                       Example: [0, 1, 1, 0]
        """
        patterns = [
            [1, 0, 0, 0],  # far left
            [0, 1, 0, 0],  # left
            [0, 1, 1, 0],  # center
            [0, 0, 1, 0],  # right
            [0, 0, 0, 1],  # far right
        ]

        return random.choice(patterns)

    # --------------------------------------------------
    def compute_error(self, sensor_data):
        """
        Compute line error using weighted average method.

        Args:
            sensor_data (list[int]): Sensor readings

        Returns:
            float: Calculated line error
        """
        weights = [-2, -1, 1, 2]

        error = 0
        total = 0

        for i in range(len(sensor_data)):
            error += sensor_data[i] * weights[i]
            total += sensor_data[i]

        if total == 0:
            return 0.0

        return error / total

    # --------------------------------------------------
    def publish_error(self, error):
        """
        Publish the computed line error.

        Args:
            error (float): Line deviation value
        """
        msg = Float32()
        msg.data = float(error)
        self.publisher_.publish(msg)

    # --------------------------------------------------
    def run(self):
        """
        Main loop executed periodically:
        - Read sensors
        - Compute error
        - Publish result
        """
        sensors = self.read_sensors()
        error = self.compute_error(sensors)
        self.publish_error(error)

        self.get_logger().info(f'Line Error: {error}')


# --------------------------------------------------
def main():
    """
    Entry point for the node.
    Initializes ROS2 and starts the SensorNode.
    """
    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()
