# A ROS 2 python implementation of: https://github.com/HemaZ/pure_pursuit

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped

from tf2_ros import TransformListener, Buffer
from typing import List
from math import sqrt, atan2
import numpy as np
from yolov8_zed.transform_conversions import transform_pose

def distance(pt1, pt2):
  return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2))


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.control_pub = self.create_publisher(
            AckermannDriveStamped, '/pure_pursuit/control', 10)
        self.lookahead_pub = self.create_publisher(
            PoseStamped, '/pure_pursuit/lookahead_point', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/pure_pursuit/path', self.path_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(1/60, self.control_loop)

        self.logger = self.get_logger()

        self.map_frame = 'map'
        self.base_frame = 'base_link'
        self.ld_gain = 1.0
        self.min_ld = 1.0
        self.max_ld = 3
        self.car_wheel_base = 1.5

        self.ld = self.min_ld
        self.car_speed = 0
        self.path: List[PoseStamped] = None
        self.got_path = False
        self.path_done = True
        self.loop = False
        self.last_dist = float('inf')
        self.base_location = None
        self.point_idx = 0
        self.last_p_idx = 0
        # self.last_dist = 0
        self.start_end_dist = 0
        self.target_point: PoseStamped = None

    def odom_callback(self, msg: Odometry):
        self.car_speed = np.sqrt(msg.twist.twist.linear.x**2+msg.twist.twist.linear.y**2)
        self.car_current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        # self.ld = min(self.max_ld, max(self.ld_gain * self.car_speed, self.min_ld))

    def path_callback(self, msg: Path):
        self.logger.info(f'New path received')
        self.path = msg.poses
        self.got_path = True
        self.path_done = False
        self.point_idx = 0
        self.start_end_dist = distance(
            self.path[0].pose.position, self.path[-1].pose.position)
        self.logger.info(f"Start to End Distance: {self.start_end_dist}")
        self.logger.info(f"Min lookup distance: {self.min_ld}")
        if (self.start_end_dist <= self.min_ld):
            # self.loop = True
            self.logger.info("Is Loop: True")
        self.logger.info(f'Path: {[[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] for pose in self.path]}')

    def control_loop(self):
        # self.logger.info('am here')
        if self.got_path:
            # self.logger.info('am inside')
            # get the current robot location by tf base_link -> map
            # iterate over the path points
            # if the distance between a point and robot > lookahead break and take
            # this point transform this point to the robot base_link the y component
            # of this point is y_t delta can be computed as atan2(2 * yt * L_, ld_2)
            self.base_location: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())

            while(self.point_idx < len(self.path)):
                self.distance = distance(self.path[self.point_idx].pose.position,
                                    self.base_location.transform.translation)
                self.logger.info(f"Point ID: {self.point_idx}, Distance: {self.distance}")#{self.path[self.point_idx].pose.position.x}, {self.path[self.point_idx].pose.position.y}")
                if (self.distance >= self.ld):
                    self.path[self.point_idx].header.stamp = self.get_clock().now().to_msg()
                    map_to_base_link: TransformStamped = self.tf_buffer.lookup_transform(
                        self.base_frame, self.map_frame, Time()
                    )
                    # Set the timestamp to now for the transform
                    # to work, because it tries to transform the
                    # point at the time stamp of the input point
                    self.target_point = transform_pose(self.path[self.point_idx], map_to_base_link)
                    break
                
                self.point_idx += 1
            # Calculate the steering angle
            y_t = self.target_point.pose.position.y
            # self.logger.info(f"error_y = {y_t}")

            delta = atan2(2 * self.car_wheel_base * y_t, self.ld**2)
            delta = min(max(delta, -0.4), 0.4)
            control_msg = AckermannDriveStamped()
            control_msg.drive.steering_angle = delta
            control_msg.drive.speed = self.path[self.point_idx].pose.position.z
            control_msg.header.stamp = self.get_clock().now().to_msg()
            self.control_pub.publish(control_msg)

            self.last_p_idx = self.point_idx
            self.last_dist = self.distance
            if (self.point_idx == len(self.path) and self.loop): 
                self.point_idx = 0
            elif (self.point_idx == len(self.path)):
                self.logger.info("Reached final point")
                control_msg.drive.steering_angle = 0.
                control_msg.drive.speed = 0.
                control_msg.header.stamp = self.get_clock().now().to_msg()
                self.control_pub.publish(control_msg)
                self.got_path = False
                self.point_idx = 0
            lookahead_p = PoseStamped()
            lookahead_p.pose.position = self.path[self.point_idx].pose.position
            lookahead_p.header = self.path[self.point_idx].header
            self.lookahead_pub.publish(lookahead_p) # Publish the lookahead point
        


def main():
    rclpy.init()
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()

if __name__=="__main__":
    main()