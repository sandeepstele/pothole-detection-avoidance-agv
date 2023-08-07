import time
from rsl_comm_py import UM7Serial
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Byte
from rclpy.qos import QoSProfile

port = "/dev/ttyUSB0"

class um7(Node):
    def __init__(self):
        super().__init__('um7')
        # Connect to um7 sensor
        self.um7 = UM7Serial(port_name = port)

        rate, *_ = self.um7.creg_com_rates4
        rate.set_field_value(ALL_PROC_RATE=255)
        self.um7.creg_com_rates4 = rate.raw_value

        self.pub = self.create_publisher(Float64MultiArray, '/accel', QoSProfile(depth=10))

        self.timer = self.create_timer(0.01, self.read_data)

        
    def read_data(self):
        for packet in self.um7.recv_euler_broadcast():
            self.get_logger().info(str(packet))
        # for packet in self.um7.recv_all_proc_broadcast():
        #     gyro = [time.time(), packet.gyro_proc_time, packet.gyro_proc_x, packet.gyro_proc_y, packet.gyro_proc_z]
        #     accel = [packet.accel_proc_time, packet.accel_proc_x, packet.accel_proc_y, packet.accel_proc_z]
        #     msg = Float64MultiArray()
        #     msg.data = accel
        #     self.pub.publish(msg)
        #     self.get_logger().info(f"{msg}")

def main(args = None):
    
    try:
        rclpy.init(args=args)
        node = um7()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("i die")
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
    