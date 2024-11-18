import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class ExternalPublisherNode(Node):

    def __init__(self):
        super().__init__('external_publisher_node')
        qos_profile = QoSProfile(depth=10)
        
        self.msg_C_publisher = self.create_publisher(String, 'msg_C', qos_profile)
        self.msg_D_publisher = self.create_publisher(String, 'msg_D', qos_profile)
        
        self.timer_C = self.create_timer(1, self.publish_msg_C)
        self.timer_D = self.create_timer(1, self.publish_msg_D)

    def publish_msg_C(self):
        msg = String()
        msg.data = 'message C'
        self.msg_C_publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))

    def publish_msg_D(self):
        msg = String()
        msg.data = 'message D'
        self.msg_D_publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))
        
    

def main(args=None):
    rclpy.init(args=args)
    node = ExternalPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()