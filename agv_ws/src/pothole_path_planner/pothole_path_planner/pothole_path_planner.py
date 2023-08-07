import numpy as np
import math
from std_msgs.msg import Float64MultiArray

from geometry_msgs.msg import TransformStamped, Pose, Vector3
from tf2_ros.transform_listener import TransformListener
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from std_msgs.msg import Header, Float64MultiArray

from .helpers.cubic_spline import interpolate_trajectory_with_spline, interpolate_velocity
from .helpers.distance_calculations import get_horiz_dist_to_line


class PotholePathPlanner(Node):

    def __init__(self):
        super().__init__('pothole_path_planner')

        # Constants

        self.TRACK_WIDTH = 0.72  # Width between two wheels
        self.WHEEL_BASE = 1.5
        self.WHEEL_WIDTH = 0.12
        self.CAR_WIDTH = self.TRACK_WIDTH + 2*self.WHEEL_WIDTH

        self.NEW_POTHOLE_THRESHOLD = 0.5  # 20%

        self.GENERAL_VELOCITY = 1
        self.POST_POTHOLE_THRESHOLD = 0.5
        self.POTHOLE_CONTINUE_THRESHOLD = 0.05
        self.POTHOLE_STOP_THRESHOLD = 0.3

        self.RIGHT_SIM_BOUND = -self.CAR_WIDTH/2 - 0.3
        self.LEFT_SIM_BOUND = self.CAR_WIDTH/2

        # Member Variables

        self.car_current_pos = []

        self.pothole_bboxes = []
        self.pothole_centers = []
        self.pothole_diameters = []
        self.pothole_depths = []
        self.pothole_widths = []
        self.prev_pothole_centers = None

        self.road_segments = []
        self.goal_point = []

        self.path = []

        self.new_pothole = True

        # Helper Functions

        self.print = self.get_logger().info

        # Publishers
        self.path_pose_publisher = self.create_publisher(
            Path, 'pure_pursuit/path', 10)

        self.path_visual_publisher = self.create_publisher(
            Marker, 'APF_planner/visualize_path', 10)

        self.sim_visual_publisher = self.create_publisher(
            Marker, 'APF_planner/visualize_sim_bounds', 10)

        # Subscriptions
        self.bb_subscriber = self.create_subscription(Float64MultiArray, 'pothole_perception_node/bounding_box',
                                                      self.bb_callback, 10)
        self.segment_subscriber = self.create_subscription(Float64MultiArray, 'pothole_perception_node/segments',
                                                           self.segment_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom',
                                                        self.get_current_position, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer Callbacks
        self.create_timer(1/20, self.run_planner)

    def generate_decelerate_waypoint(self, cur_pos, pothole_center, bbox, next_x, depth):

        if depth > self.POTHOLE_CONTINUE_THRESHOLD and depth < self.POTHOLE_STOP_THRESHOLD:

            # change speed to 2km/hour and keep moving forward
            speed = 0.5  # takes it in m/s
            second_point = [pothole_center[0], cur_pos[1], speed]
            third_point = [min(bbox[3]+self.POST_POTHOLE_THRESHOLD,
                               (bbox[3]+next_x)/2), cur_pos[1], self.GENERAL_VELOCITY]

            self.path.append(second_point)
            self.path.append(third_point)

            return False, third_point

        elif depth > self.POTHOLE_STOP_THRESHOLD:
            second_point = [bbox[0]-self.WHEEL_BASE/2, cur_pos[1], 0]
            self.path.append(second_point)

            return True, second_point

        else:
            second_point = [pothole_center[0],
                            cur_pos[1], self.GENERAL_VELOCITY]
            # self.print(f'{bbox[3]} {self.post_pothole_threshold} {next_x}')
            third_point = [min(bbox[3]+self.POST_POTHOLE_THRESHOLD,
                               (bbox[3]+next_x)/2), cur_pos[1], self.GENERAL_VELOCITY]

            self.path.append(second_point)
            self.path.append(third_point)

            return False, third_point

    def generate_avoid_waypoint(self, d, bbox, next_x):

        next_point = [min(bbox[3]+self.POST_POTHOLE_THRESHOLD,
                          (bbox[3]+next_x)/2), d[2][1], self.GENERAL_VELOCITY]

        self.path.append([d[2][0], d[2][1], self.GENERAL_VELOCITY])
        self.path.append(next_point)

        return next_point

    def generate_goal_point(self, current_pos):
        goal_x = self.road_segments[np.argmax(self.road_segments[:, 0]), :]
        return np.array([goal_x[0], current_pos[1]])

    def bb_callback(self, msg: Float64MultiArray):

        shape = [msg.layout.dim[0].size, msg.layout.dim[1].size]

        self.pothole_bboxes = np.array(msg.data).reshape(shape)
        self.pothole_bboxes = self.pothole_bboxes[self.pothole_bboxes[:, 0].argsort(
        )]

        self.pothole_centers = (
            self.pothole_bboxes[:, 0:2] + self.pothole_bboxes[:, 3:5]) / 2
        self.pothole_centers[:, 0] -= 1
        self.pothole_diameters = np.linalg.norm(
            self.pothole_bboxes[:, 0:2]-self.pothole_bboxes[:, 3:5], axis=1)
        self.pothole_depths = self.pothole_bboxes[:, 5]-self.pothole_bboxes[:, 2]
        self.pothole_widths = self.pothole_bboxes[:, 4]-self.pothole_bboxes[:, 1]

        if self.prev_pothole_centers is None or len(self.pothole_centers) > len(self.prev_pothole_centers):
            self.new_pothole = True
        elif len(self.pothole_centers) < len(self.prev_pothole_centers):
            self.new_pothole = False
        else:
            self.new_pothole = False
            for old in self.prev_pothole_centers:
                old_pothole = False
                for i, new in enumerate(self.pothole_centers):
                    if np.sqrt((old[0]-new[0])**2+(old[1]-new[1])**2) < self.NEW_POTHOLE_THRESHOLD*self.pothole_diameters[i]:
                        old_pothole = True
                        break
                if not old_pothole:
                    self.new_pothole = True
                    break
        self.prev_pothole_centers = self.pothole_centers.copy()
        self.print("No new potholes" if not self.new_pothole else "New potholes")

    def segment_callback(self, msg: Float64MultiArray):
        shape = [msg.layout.dim[0].size, msg.layout.dim[1].size]
        self.road_segments = np.array(msg.data).reshape(shape)

    def get_current_position(self, msg: Odometry):
        self.car_current_pos = np.array([msg.pose.pose.position.x,
                                         msg.pose.pose.position.y])

    def calculate_pothole_to_edge_distances(self, bb_left, bb_right, bb_near, bb_far):
        right_segments = self.road_segments[self.road_segments[:, 1] < bb_right, :]
        left_segments = self.road_segments[self.road_segments[:, 1] > bb_left, :]

        right_segment = []
        for i in range(1, len(right_segments)):
            if right_segments[i, 0] > bb_near and right_segments[i, 0] < bb_far:
                right_segment.append(right_segments[i])
                break

        left_segment = []
        for i in range(1, len(left_segments)):
            if left_segments[i, 0] > bb_near and left_segments[i, 0] < bb_far:
                left_segment.append(left_segments[i])
                break

        right_segment = np.array(right_segment)
        left_segment = np.array(left_segment)
        if len(right_segment) == 0:
            right_dist = None
        else:
            right_sorted = right_segment[right_segment[:, 1].argsort()]
            right_dist = bb_right - right_sorted[-1, 1]
        if len(left_segment) == 0:
            left_dist = None
        else:
            left_sorted = left_segment[left_segment[:, 1].argsort()]
            left_dist = left_sorted[0, 1] - bb_left

        return right_dist, left_dist

    def calculate_pothole_to_sim_edge_distances(self, bb_left, bb_right, bb_near, bb_far):

        right_dist = bb_right - self.RIGHT_SIM_BOUND
        left_dist = self.LEFT_SIM_BOUND - bb_left
        return right_dist, left_dist

    def calculate_lateral_maneuver_distances(
            self, 
            car_position, 
            pothole_center, 
            car_width, 
            right_pothole_y, 
            left_pothole_y, 
            near_pothole_x):

        epsilon = 0.05

        right_next_point = [near_pothole_x,
                            right_pothole_y - car_width/2 - epsilon]
        left_next_point = [near_pothole_x,
                           left_pothole_y + car_width/2 + epsilon]
        align_next_point = [near_pothole_x, pothole_center[1]]

        return [
            (0, abs(car_position[1]-pothole_center[1]), align_next_point),
            (1, -right_pothole_y +
             (car_position[1]+car_width/2), right_next_point),
            (2, -(car_position[1]-car_width/2) +
             left_pothole_y, left_next_point),
        ]

    def check_kinematic_feasibility(self, cur, next) -> bool:
        diff_y = next[1]-cur[1]
        distance = np.sqrt((next[1]-cur[1])**2+(next[0]-cur[0])**2)
        delta = math.atan2(2 * self.WHEEL_BASE * diff_y, distance**2)
        return abs(delta) < 0.4

    def check_maneuver_feasibility(self, cur_pos, distances, pothole_width, pothole_to_right_seg, pothole_to_left_seg):

        distances.sort(key=lambda x: x[1])

        for d in distances:
            if d[0] == 0:
                if self.TRACK_WIDTH > pothole_width \
                        and self.CAR_WIDTH/2 < (pothole_to_right_seg + pothole_width/2) \
                        and self.CAR_WIDTH/2 < (pothole_to_left_seg + pothole_width/2) \
                        and self.check_kinematic_feasibility(cur_pos, d[2]):
                    return d
            elif d[0] == 1:  # feasibility of right segment
                if self.CAR_WIDTH < pothole_to_right_seg \
                        and self.check_kinematic_feasibility(cur_pos, d[2]):
                    return d
            else:
                if self.CAR_WIDTH < pothole_to_left_seg \
                        and self.check_kinematic_feasibility(cur_pos, d[2]):
                    return d

        return None

    def check_if_pothole_outside_path(self, cur_pos, bbox):
        # check if pothole to the left of vehicle, or to the right, or between wheels
        epsilon = 0.05
        return bbox[1] > cur_pos[1]+self.CAR_WIDTH/2+epsilon or \
            bbox[4] < cur_pos[1]-self.CAR_WIDTH/2-epsilon or \
            (bbox[1] > cur_pos[1]-self.TRACK_WIDTH/2 +
             0.02 and bbox[4] < cur_pos[1]+self.TRACK_WIDTH/2 - 0.02)

    def generate_path(self):
        self.path = []
        if len(self.pothole_bboxes) == 0:
            self.print(f'no potholes')
            # self.print(f'car: {[self.car_current_pos[0], self.car_current_pos[1], self.general_velocity]}')
            self.path = np.array([
                [self.car_current_pos[0], self.car_current_pos[1],
                    self.GENERAL_VELOCITY],
                [self.goal_point[0], self.goal_point[1], self.GENERAL_VELOCITY]
            ])
            return
        else:
            stop = False
            self.print(f'start pothole planning')
            plan_current_pos = self.car_current_pos
            self.path.append(
                [plan_current_pos[0], plan_current_pos[1], self.GENERAL_VELOCITY])
            for i, bbox in enumerate(self.pothole_bboxes):
                # self.print(f'bbox: {bbox}')
                pothole_to_right_seg, pothole_to_left_seg = self.calculate_pothole_to_edge_distances(
                    bbox[4], bbox[1], bbox[0], bbox[3]
                )
                # self.visualize_sim_bounds()
                if pothole_to_left_seg is None or pothole_to_right_seg is None:
                    # self.print(f'segment none')
                    continue
                if bbox[3] < plan_current_pos[0]:
                    # self.print("we are after the pothole")
                    continue
                next_x = self.goal_point[0] if i == len(
                    self.pothole_bboxes)-1 else self.pothole_bboxes[i+1][0]

                if self.check_if_pothole_outside_path(plan_current_pos, bbox) \
                    or self.pothole_depths[i] < self.POTHOLE_CONTINUE_THRESHOLD:
                    # self.print("outside")
                    stop, plan_current_pos = self.generate_decelerate_waypoint(
                        plan_current_pos, self.pothole_centers[i], bbox, next_x, 0)
                    continue

                distances = self.calculate_lateral_maneuver_distances(
                    plan_current_pos, self.pothole_centers[i], self.CAR_WIDTH, bbox[1], bbox[4], bbox[0])
                d = self.check_maneuver_feasibility(
                    plan_current_pos, distances, self.pothole_widths[i], pothole_to_right_seg, pothole_to_left_seg)
                if d is None:
                    # self.print("d is none")
                    stop, plan_current_pos = self.generate_decelerate_waypoint(
                        plan_current_pos, self.pothole_centers[i], bbox, next_x, 0.9)
                    if stop:
                        break
                else:
                    plan_current_pos = self.generate_avoid_waypoint(d, bbox, next_x)

        if not stop:
            self.path.append(
                [self.goal_point[0], self.goal_point[1], self.GENERAL_VELOCITY])
        self.path = np.array(self.path)

    def publish_path(self, path, velocity):
        # self.print(f'{velocity}')
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        self.print(f'Path: {len(path)}\n Velocity: {len(velocity)}')

        if len(path) != len(velocity):
            self.get_logger().warn(
                f'Velocity vector and path vector have different lengths: {len(velocity)} {len(path)}')

        # set the position
        for i in range(min(len(path), len(velocity))):

            pose_stamped = PoseStamped()
            pose_stamped.header = Header(
                frame_id='map', stamp=path_msg.header.stamp)
            pose_stamped.pose.position = Point(
                x=path[i, 0], y=path[i, 1], z=float(velocity[i]))
            pose_stamped.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=0.0, w=1.0)
            path_msg.poses.append(pose_stamped)

        self.path_pose_publisher.publish(path_msg)

    def visualize_path(self, path, color={'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0}, scale=[0.2, 0.2, 0.2]):
        # Create MarkerArray
        # LOGGER.info(f'segment {segment_arr}')
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color['r']
        marker.color.g = color['g']
        marker.color.b = color['b']
        marker.color.a = color['a']
        for point in path:
            if False in np.isfinite(point):
                continue
            p = Point()
            # LOGGER.info(f'{point}')
            p.x, p.y, p.z = [point[0], point[1], 0.]
            marker.points.append(p)

        # Publish MarkerArray
        self.path_visual_publisher.publish(marker)

    def visualize_sim_bounds(self, color={'r': 1.0, 'g': 1.0, 'b': 1.0, 'a': 1.0}, scale=[0.2, 0.2, 0.2]):
        marker1 = Marker()
        marker1.header.frame_id = 'odom'
        marker1.id = 0
        marker1.type = Marker.LINE_STRIP
        marker1.action = Marker.ADD
        marker1.scale.x = scale[0]
        marker1.scale.y = scale[1]
        marker1.scale.z = scale[2]
        marker1.color.r = color['r']
        marker1.color.g = color['g']
        marker1.color.b = color['b']
        marker1.color.a = color['a']
        p1 = Point()
        p1.x, p1.y, p1.z = [0., float(self.RIGHT_SIM_BOUND), 0.]
        p2 = Point()
        p2.x, p2.y, p2.z = [float(self.goal_point[0]),
                            float(self.RIGHT_SIM_BOUND), 0.]
        marker1.points.append(p1)
        marker1.points.append(p2)

        marker2 = Marker()
        marker2.header.frame_id = 'odom'
        marker2.id = 1
        marker2.type = Marker.LINE_STRIP
        marker2.action = Marker.ADD
        marker2.scale.x = scale[0]
        marker2.scale.y = scale[1]
        marker2.scale.z = scale[2]
        marker2.color.r = color['r']
        marker2.color.g = color['g']
        marker2.color.b = color['b']
        marker2.color.a = color['a']
        p1 = Point()
        p1.x, p1.y, p1.z = [0., float(self.LEFT_SIM_BOUND), 0.]
        p2 = Point()
        p2.x, p2.y, p2.z = [float(self.goal_point[0]),
                            float(self.LEFT_SIM_BOUND), 0.]
        marker2.points.append(p1)
        marker2.points.append(p2)

        self.sim_visual_publisher.publish(marker1)
        self.sim_visual_publisher.publish(marker2)

    def run_planner(self):
        if len(self.car_current_pos) == 0:
            self.print(f'waiting for odometry')
        elif len(self.road_segments) == 0:
            self.print(f'waiting for segments')
        else:
            self.goal_point = self.generate_goal_point(self.car_current_pos)
            if self.new_pothole or \
                    np.linalg.norm(self.path[-1, :2] - self.car_current_pos) < 3:

                self.generate_path()
                # self.print(f'for smooth{self.path}')
                if len(self.path) == 0:
                    return
                path_dup = self.path.tolist()
                path = []
                for coord in path_dup:
                    if coord not in path:
                        path.append(coord)
                path = np.array(path)
                # path = self.path
                self.print(f'true path: {path}')

                smooth_path = interpolate_trajectory_with_spline(
                    path[:, 0], path[:, 1])
                smooth_vel = interpolate_velocity(
                    path[:, :2], path[:, 2], smooth_path)
                self.visualize_path(smooth_path)
                self.publish_path(smooth_path, smooth_vel)
                self.new_pothole = False


def main(args=None):

    rclpy.init(args=args)
    pothole_path_planner = PotholePathPlanner()

    rclpy.spin(pothole_path_planner)

    pothole_path_planner.print("Path planner node terminating...")
    pothole_path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
