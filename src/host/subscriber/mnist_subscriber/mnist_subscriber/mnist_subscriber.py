import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

class MNISTSubscriber(Node):
    def __init__(self):
        super().__init__('mnist_subscriber')
        self.subscription = self.create_subscription(
            UInt8,
            'mnist_result',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'Received MNIST result: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    mnist_subscriber = MNISTSubscriber()

    rclpy.spin(mnist_subscriber)

    mnist_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()