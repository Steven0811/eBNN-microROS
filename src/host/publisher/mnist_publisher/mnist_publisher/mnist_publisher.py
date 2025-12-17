import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, MultiArrayLayout, MultiArrayDimension
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

        self.pub = self.create_publisher(UInt8MultiArray, '/mnist_image_to_ebnn', qos_profile)
        self.pub_monitor = self.create_publisher(UInt8MultiArray, '/mnist_image_to_monitor', qos_profile)

        self.counter = 0
        self.dataset = None
        self.labels = None

    def load_dataset(self):
        from tensorflow.keras.datasets import mnist
        (_, _), (x_test, y_test) = mnist.load_data()
        self.dataset = x_test
        self.labels = y_test

    def generate_random_image(self):
        return np.random.randint(0, 256, size=(28, 28), dtype=np.uint8)

    def corrupt_mnist(self, img):
        img = img.astype(np.float32)
        x = np.random.randint(0, 20)
        y = np.random.randint(0, 20)
        img[x:x+8, y:y+8] = 255
        return img.astype(np.uint8)

    def send_images(self, img, label):
        if self.counter < 20:
            out_img = img
            out_label = label
        elif self.counter < 35:
            out_img = self.corrupt_mnist(img)
            out_label = -1
        else:
            out_img = self.generate_random_image()
            out_label = -2

        msg = UInt8MultiArray()
        msg.data = out_img.astype(np.uint8).flatten().tolist()

        layout = MultiArrayLayout()
        layout.data_offset = 0

        dim = MultiArrayDimension()
        dim.label = "mnist_image"
        dim.size = len(msg.data)
        dim.stride = len(msg.data)

        layout.dim = [dim]
        msg.layout = layout

        self.pub_monitor.publish(msg)
        self.pub.publish(msg)

        self.get_logger().info(f"[{self.counter}] Published label=({out_label}) size={len(msg.data)}")

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MnistPublisher()
    node.load_dataset()

    try:
        for i in range(50):
            node.send_images(node.dataset[i], node.labels[i])
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
