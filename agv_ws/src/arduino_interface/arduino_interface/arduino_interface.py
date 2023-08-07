import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Byte, Float64
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import QoSProfile
import serial
import time
from collections import deque
import numpy as np
import struct


BAUD_RATE = 57600
port = '/dev/ttyUSB0'

class ArduinoInterface(Node):
    def __init__(self):
        super().__init__('arduino_interface')
        

        self.error_i = 0
        self.error = []

        self.k_p = 5
        self.k_i= 2
        self.k_d = 0

        self.vel_des = 1 # desired vel, m/s

        self.vel_cmd = 0 # pwm, 80-130
        self.brake_cmd = 0 # brake position, 0-100%
        self.steer_cmd = 120 # steer in degrees, 0-255, (-22.5,22.5) degrees, (-0.4,0.4) rad

        self.movingWindowSize = 20
        self.current_vel = deque(maxlen = self.movingWindowSize)
        self.ma_vel = 0

        # set up serial communication with the Arduino
        self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
        
        # set up subscriber
        self.sub = self.create_subscription(Odometry, '/odom', self.get_current_velocity, 10)
        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped, 
            '/pure_pursuit/control', 
            self.get_control_commands, 
            10)
        self.logger = self.get_logger()

    #     self.timer = self.create_timer(0.01, self.attempt_control)

    #     self.vel_vector = np.concatenate([
    #         np.zeros(100), 
    #         np.linspace(0, 1, 300), 
    #         np.ones(500), 
    #         np.linspace(1,0,300)
    #         ])
    #     self.i = 0

    # def attempt_control(self):
    #     if self.i < len(self.vel_vector):
    #         self.vel_des = self.vel_vector[self.i]
    #         self.i += 1

    def get_current_velocity(self, msg: Odometry):
        vel = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        ]
        vel = np.sqrt(vel[0]**2+vel[1]**2)
        self.current_vel.append(vel)
        
        self.ma_vel = sum(self.current_vel)/len(self.current_vel)
        self.logger.info(f'{self.vel_des} {self.vel_cmd}')
        self.compute_control()
        self.write_data()
    
    def get_control_commands(self, msg: AckermannDriveStamped):
        self.vel_des = msg.drive.speed
        self.steer_cmd = int((-msg.drive.steering_angle + 0.4) / 0.8 * 255)
        self.logger.info(f'{self.steer_cmd}')
        
    def compute_control(self):

        if self.vel_des == 0:
            self.vel_cmd = 0
            return
        self.error.append(self.vel_des - self.ma_vel)
        if len(self.error)>=2:
            self.error_i += (self.error[-2]+self.error[-1])/ 2 * 0.01
        ax_des = self.k_p *(self.error[-1]) + self.k_i*(self.error_i)

        if ax_des>0:
            self.vel_cmd = min(75.92156736664518+18.1655985*self.ma_vel + 0.63604731*ax_des - 1.35068359*self.ma_vel**2  - 0.80396293*self.ma_vel*ax_des + 1.00006849*ax_des**2, 130)
        else:
            self.vel_cmd = min(75.92156736664518 + 18.1655985*self.vel_des - 1.35068359*self.vel_des**2, 130)        
        
        self.vel_cmd = int(self.vel_cmd)


    def write_data(self):
        # send command to the Arduino to write digital output
        data = bytes([self.vel_cmd, self.steer_cmd, self.brake_cmd])
        char = 'f'
        self.ser.write(bytes(char.encode()))
        self.ser.write(data)
        data = self.ser.readline().decode()
        self.logger.info(data)

        
def main(args=None):
    try:
        rclpy.init(args=args)
        node = ArduinoInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("i die")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()