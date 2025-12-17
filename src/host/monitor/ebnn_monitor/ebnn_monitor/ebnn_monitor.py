import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import UInt8
from std_msgs.msg import UInt8MultiArray

from .anomaly_detector import PCAAnomalyDetector


class EBNNMonitor(Node):
    def __init__(self):
        super().__init__('ebnn_monitor')

        self.detector = PCAAnomalyDetector(
            pca_model_path="models/pca_mnist.joblib",
            threshold=0.01
        )

        self.latest_image_flat = None

        self.create_subscription(
            UInt8MultiArray,
            '/mnist_image_to_monitor',
            self.image_callback,
            10
        )

        self.create_subscription(
            UInt8,
            '/mnist_result',
            self.pred_callback,
            10
        )

    def image_callback(self, msg: UInt8MultiArray):
        data = np.asarray(msg.data, dtype=np.uint8)

        if data.size != 28 * 28:
            self.get_logger().error(f"Invalid MNIST image size: {data.size}")
            self.latest_image_flat = None
            return

        img = data.astype(np.float32) / 255.0
        self.latest_image_flat = img.reshape(1, -1)

    def pred_callback(self, msg: UInt8):
        if self.latest_image_flat is None:
            self.get_logger().warn("Prediction received but no image yet")
            return

        pred = msg.data

        is_anomaly, score = self.detector.is_anomaly(
            self.latest_image_flat
        )

        if is_anomaly:
            self.get_logger().warn(f"Anomaly detected | pred={pred} | score={score:.6f}")
        else:
            self.get_logger().info(f"Valid prediction | pred={pred} | score={score:.6f}")

def main(args=None):
    rclpy.init(args=args)

    node = EBNNMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
