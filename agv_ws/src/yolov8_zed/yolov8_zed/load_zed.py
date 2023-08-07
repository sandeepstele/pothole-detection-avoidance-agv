from pathlib import Path
import logging
import time
import cv2

import torch
import numpy as np

import pyzed.sl as sl
from threading import Thread
from scipy.spatial.transform import Rotation

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry

from rclpy.time import Time
from rclpy.duration import Duration
from builtin_interfaces.msg import Time as TimeMessage
from rclpy.impl.rcutils_logger import RcutilsLogger

from yolov8_zed.transform_conversions import quaternion_vector_to_rigid_transform, transform_stamped_to_rigid_transform

ZED_IMG_SIZE_DICT = {
    720: sl.RESOLUTION.HD720,
    1080: sl.RESOLUTION.HD1080
}

class LoadZED:
    # YOLOv5 streamloader, i.e. `python detect.py --source 'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP streams`
    def __init__(
        self,
        logger: RcutilsLogger,
        zed_tf_to_base_link: TransformStamped,
        base_link_to_zed_tf: TransformStamped,
        clock,
        odom_publisher,
        tf_broadcaster,
        img_size=720,
        stride=32, 
        vid_stride=1, 
        svo='ZEDLIVE'
        ):
        self.stride = stride
        self.vid_stride = vid_stride  # video frame-rate stride
        self.logger = logger
        self.odom_publisher = odom_publisher
        self.tf_broadcaster = tf_broadcaster
        self.zed_tf_to_base_link = zed_tf_to_base_link
        self.base_link_to_zed_tf = base_link_to_zed_tf
        self.clock = clock

        if img_size not in ZED_IMG_SIZE_DICT:
            self.logger.fatal("Invalid image size")
            exit(-1)

        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        if svo != 'ZEDLIVE':
            self.init_params.set_from_svo_file(svo)
            self.init_params.camera_fps = 60
        else:
            self.init_params.camera_resolution = ZED_IMG_SIZE_DICT[img_size]
            self.init_params.camera_fps = 60
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.logger.fatal(f'Error opening ZED camera: {err}')
            exit(-1)
        
        
        self.tracking_parameters = sl.PositionalTrackingParameters()
        self.initial_position = sl.Transform()
        initial_orientation = sl.Orientation()
        initial_translation = sl.Translation()
        initial_translation.init_vector(
            zed_tf_to_base_link.transform.translation.x,
            zed_tf_to_base_link.transform.translation.y,
            zed_tf_to_base_link.transform.translation.z,
        )
        initial_orientation.init_vector(
            zed_tf_to_base_link.transform.rotation.x,
            zed_tf_to_base_link.transform.rotation.y,
            zed_tf_to_base_link.transform.rotation.z,
            zed_tf_to_base_link.transform.rotation.w            
        )
        self.initial_position.set_translation(initial_translation)
        self.initial_position.set_orientation(initial_orientation)
        self.tracking_parameters.set_initial_world_transform(self.initial_position)

        err_track = self.zed.enable_positional_tracking(self.tracking_parameters)
        if err_track != sl.ERROR_CODE.SUCCESS:
            self.logger.fatal("Error starting positional tracking: {err_track}")
            exit(-1)
        
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            # A new image is available if grab() returns SUCCESS
            image = sl.Mat()
            pcl = sl.Mat()
            pose = sl.Pose()

            # Retrieve the left image
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            self.img = image.get_data().copy()[:, :, :3] # Remove A channel
            self.img = self.img[:, :, ::-1] # Switch R and B channels

            # Retrieve colored point cloud
            self.zed.retrieve_measure(pcl, sl.MEASURE.XYZRGBA)
            self.pcl = pcl.get_data().copy()

            # Retrieve pose of the left eye of the camera
            state = self.zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)
            pose_timestamp = self.clock.now()
            self.handle_pose(pose, pose_timestamp)
            self.pose = pose
            self.pose_timestamp = pose_timestamp

        self.thread = Thread(target=self.update, daemon=True)
        self.logger.info(f"Success")
        self.thread.start()

    def transform_zed_current_pose_to_base_link(self, pose: sl.Pose):
        cur_translation = pose.get_translation().get()
        cur_quaternion = pose.get_orientation().get()

        cur_transform = quaternion_vector_to_rigid_transform(
            cur_translation, cur_quaternion)
        backwards_transform = transform_stamped_to_rigid_transform(
            self.base_link_to_zed_tf)

        base_transform = backwards_transform @ cur_transform
        base_translation = base_transform[:3, 3]
        base_quaternion = Rotation.from_matrix(
            base_transform[:3, :3]).as_quat()
        return base_translation, base_quaternion

    def handle_pose(self, pose: sl.Pose, pose_timestamp: Time):

        translation, orientation = self.transform_zed_current_pose_to_base_link(
            pose)

        t = TransformStamped()
        t.header.stamp = pose_timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]

        self.tf_broadcaster.sendTransform(t)

    def send_odometry_msg(self,
            prev_pose: sl.Pose, 
            prev_pose_timestamp: Time, 
            cur_pose: sl.Pose, 
            cur_pose_timestamp: Time):
        
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.header.stamp = cur_pose_timestamp.to_msg()
        odom_msg.child_frame_id = 'base_link'

        cur_position, cur_quat = self.transform_zed_current_pose_to_base_link(cur_pose)
        prev_position, prev_quat = self.transform_zed_current_pose_to_base_link(prev_pose)

        odom_msg.pose.pose.position.x = cur_position[0]
        odom_msg.pose.pose.position.y = cur_position[1]
        odom_msg.pose.pose.position.z = cur_position[2]

        odom_msg.pose.pose.orientation.x = cur_quat[0]
        odom_msg.pose.pose.orientation.y = cur_quat[1]
        odom_msg.pose.pose.orientation.z = cur_quat[2]
        odom_msg.pose.pose.orientation.w = cur_quat[3]

        cur_rpy = Rotation.from_quat(cur_quat).as_euler('xyz', degrees=False)
        prev_rpy = Rotation.from_quat(prev_quat).as_euler('xyz', degrees=False)

        time = (cur_pose_timestamp - prev_pose_timestamp).nanoseconds / 1e9

        odom_msg.twist.twist.linear.x = (cur_position[0] - prev_position[0]) / time
        odom_msg.twist.twist.linear.y = (cur_position[1] - prev_position[1]) / time
        odom_msg.twist.twist.linear.z = (cur_position[2] - prev_position[2]) / time
        odom_msg.twist.twist.angular.x = 0.
        odom_msg.twist.twist.angular.y = 0.
        odom_msg.twist.twist.angular.z = (cur_rpy[2] - prev_rpy[2]) / time


        self.odom_publisher.publish(odom_msg)


    def update(self):
        # Read stream `i` frames in daemon thread
        n = 0 # frame number, frame array
        image = sl.Mat()
        pcl = sl.Mat()
        pose = sl.Pose()

        while True:
            n += 1
            if n % self.vid_stride == 0:
                if self.zed.grab() == sl.ERROR_CODE.SUCCESS:

                    # Retrieve image
                    self.zed.retrieve_image(image, sl.VIEW.LEFT)
                    self.img = image.get_data().copy()[:, :, :3] # Remove A channel
                    self.img = self.img[:, :, ::-1] # Switch R and B channels

                    # Retrieve colored point cloud
                    self.zed.retrieve_measure(pcl, sl.MEASURE.XYZRGBA) 
                    self.pcl = pcl.get_data().copy()

                    # Publish pose on tf and odom
                    state = self.zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)
                    pose_timestamp = self.clock.now()

                    self.handle_pose(pose, pose_timestamp)
                    self.send_odometry_msg(self.pose, self.pose_timestamp, pose, pose_timestamp)

                    self.pose.init_pose(pose)
                    self.pose_timestamp = pose_timestamp

                elif self.zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                    self.logger.info('End of SVO file reached.')
                    break
                else:
                    self.logger.warning('WARNING! Video stream unresponsive, please check your IP camera connection.')
                    self.img = np.zeros_like(self.img)
                    self.zed.open(self.init_params)
            time.sleep(1/self.init_params.camera_fps)  # wait time

    def __iter__(self):
        self.count = -1
        return self

    def __next__(self):
        self.count += 1        
        return self.img, self.pcl, self.pose

    def __len__(self):
        return 1
