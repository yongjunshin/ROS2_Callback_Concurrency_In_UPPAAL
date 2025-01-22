import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import time
from rclpy.executors import SingleThreadedExecutor


class SingleThreadNode(Node):

    def __init__(self):
        super().__init__('single_thread_node')
        qos_profile = QoSProfile(depth=10)
        
        self.msg_A_publisher = self.create_publisher(String, 'msg_A', qos_profile)
        self.msg_B_publisher = self.create_publisher(String, 'msg_B', qos_profile)
        
        self.timer_A = self.create_timer(1, self.publish_msg_A)
        self.timer_B = self.create_timer(1, self.publish_msg_B)

        self.short_msg_subscriber = self.create_subscription(
            String, 
            'msg_C', 
            self.subscribe_msg_C, 
            qos_profile)
        
        self.long_msg_subscriber = self.create_subscription(
            String, 
            'msg_D', 
            self.subscribe_msg_D, 
            qos_profile)
        
    def publish_msg_A(self):
        msg = String()
        msg.data = 'message A'
        self.msg_A_publisher.publish(msg)
        self.get_logger().info('[TMR1] Published message: {0} (processing time: 0s)'.format(msg.data))

    def publish_msg_B(self):
        msg = String()
        msg.data = 'message B'
        self.msg_B_publisher.publish(msg)
        self.get_logger().info('[TMR2] Published message: {0} (processing time: 0s)'.format(msg.data))

        
    def subscribe_msg_C(self, msg):
        self.get_logger().info('  [SUB1] Received message: {0} (processing time: 1s)'.format(msg.data))
        time.sleep(1)

    def subscribe_msg_D(self, msg):
        self.get_logger().info('  [SUB2] Received message: {0} (processing time: 2s)'.format(msg.data))
        time.sleep(2)


def main(args=None):
    rclpy.init(args=args)
    node = SingleThreadNode()
    single_thread_executor = SingleThreadedExecutor()
    single_thread_executor.add_node(node)

    try:
        single_thread_executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
