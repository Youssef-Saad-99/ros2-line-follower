import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SensorNode(Node):
    def init(self):
        super().init('sensor_node')

        # Publisher
        self.publisher_ = self.create_publisher(Float32, '/line_error', 10)

        # Timer يشغل النود كل 0.5 ثانية
        self.timer = self.create_timer(0.5, self.run)

    # -------------------------
    # 1) قراءة الحساسات
    def read_sensors(self):
        # مثال (simulate)
        # 0 = أبيض / 1 = أسود
        return [0, 1, 1, 0]

    # -------------------------
    # 2) حساب الخطأ
    def compute_error(self, sensor_data):
        weights = [-2, -1, 1, 2]

        error = 0
        total = 0

        for i in range(len(sensor_data)):
            error += sensor_data[i] * weights[i]
            total += sensor_data[i]

        if total == 0:
            return 0.0

        return error / total

    # -------------------------
    # 3) النشر
    def publish_error(self, error):
        msg = Float32()
        msg.data = float(error)
        self.publisher_.publish(msg)

    # -------------------------
    # Main loop
    def run(self):
        sensors = self.read_sensors()
        error = self.compute_error(sensors)
        self.publish_error(error)

        self.get_logger().info(f'Line Error: {error}')


# تشغيل النود
def main():
    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()
