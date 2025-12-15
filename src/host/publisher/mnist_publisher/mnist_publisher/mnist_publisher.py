import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, MultiArrayLayout, MultiArrayDimension, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class MnistPublisher(Node):
    def __init__(self):
        super().__init__('mnist_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(UInt8MultiArray, '/mnist_input', qos_profile)

        self.counter = 0
        self.dataset = None
        self.labels = None

    def load_dataset(self):
        from tensorflow.keras.datasets import mnist
        (_, _), (x_test, y_test) = mnist.load_data()
        self.dataset = x_test
        self.labels = y_test
        self.get_logger().info(f"Loaded MNIST test data: {x_test.shape}")

    def send_images(self, img: np.ndarray, label: int):
        msg = UInt8MultiArray()

        img = img.astype(np.uint8).flatten()

        msg.data = img.tolist()

        layout = MultiArrayLayout()
        layout.data_offset = 0

        dim = MultiArrayDimension()
        dim.label = "mnist_image"
        dim.size = len(msg.data) 
        dim.stride = len(msg.data)

        layout.dim = [dim]
        msg.layout = layout

        self.pub.publish(msg)

        self.get_logger().info(
            f"[{self.counter}] Published label=({label}) → /mnist_input, size={len(msg.data)}"
        )

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MnistPublisher()
    node.load_dataset()

    try:
        if node.dataset is not None and len(node.dataset) > 1:
            for i in range(50):
                img = node.dataset[i]
                label = node.labels[i]
                node.send_images(img, label)
                rclpy.spin_once(node, timeout_sec=0.5)
        else:
            node.get_logger().error("Dataset not loaded properly.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()