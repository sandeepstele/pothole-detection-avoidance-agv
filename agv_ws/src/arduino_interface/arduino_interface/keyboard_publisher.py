import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Byte
from rclpy.qos import QoSProfile

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.pub = self.create_publisher(Byte, '/arduino_in', 10)
        self.sub = self.create_subscription(Byte, '/arduino_in', self.read, QoSProfile(depth=10))

    # def publish_input(self):
    #     input_str = bytes([int(input('Throttle me: '))])
    #     msg = Byte()
    #     msg.data = input_str
    #     self.pub.publish(msg)

    def read(self, data):
        self.pib.publish(data)

def main(args=None):
    rclpy.init(args=args)

    node = KeyboardPublisher()

    # while rclpy.ok():
        # node.publish_input()

    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
