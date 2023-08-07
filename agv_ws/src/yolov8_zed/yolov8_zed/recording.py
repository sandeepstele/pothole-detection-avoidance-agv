import rclpy
from rclpy.node import Node
from typing import List
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
import numpy as np

class RecordingNode(Node):
    def __init__(self):
        super().__init__('recording_node')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.record_odom, 10)
        self.path_subscriber = self.create_subscription(Path, 'pure_pursuit/path', self.record_path, 10)
        self.odom_record_list = []
        self.bb_record_list = []
        self.path_record_list = []

        self.bb_subscriber = self.create_subscription(Float64MultiArray,'pothole_perception_node/bounding_box',
                                                        self.record_bb, 10)

    def record_odom(self, msg):
        self.odom_record_list.append([
            msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

    def record_bb(self, msg):
        self.bb_record_list.append(np.array(msg.data))
    
    def record_path(self, msg: Path):
        poses: List[PoseStamped] = msg.poses
        for pose in poses:
            self.path_record_list.append([
                pose.header.stamp.sec,
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ])


    def save_data(self, path):
        odom_string = [' '.join([str(elt) for elt in datum])+'\n' for datum in self.odom_record_list]
        bb_string = [' '.join([str(elt) for elt in datum])+'\n' for datum in self.bb_record_list]
        path_string = [' '.join([str(elt) for elt in datum])+'\n' for datum in self.path_record_list]
        
        with open(path+'_odom', 'w') as f:
            f.writelines(odom_string)
        with open(path+'_bb', 'w') as f:
            f.writelines(bb_string)
        with open(path+'_path', 'w') as f:
            f.writelines(path_string)


def main():
    rclpy.init()
    node = RecordingNode()
    timestamp = node.get_clock().now().seconds_nanoseconds()[0]
    path = '/home/nader/source/FYP/pothole_detection_avoidance_agv/car_data/newer'
    name = 'avoid_left'
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_data(f'{path}/{name}_{timestamp}')
        node.get_logger().info('Recorded data')



if __name__=="__main__":
    main()
