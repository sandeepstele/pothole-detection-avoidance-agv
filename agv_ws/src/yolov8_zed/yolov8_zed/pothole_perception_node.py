import argparse
import os
import platform
import sys
from pathlib import Path
import cv2
from scipy.ndimage import zoom
from scipy.spatial.transform import Rotation
import time

import torch
import numpy as np

import rclpy
from rclpy.node import Node

import pyzed.sl as sl

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float64MultiArray, MultiArrayDimension, ColorRGBA

from geometry_msgs.msg import TransformStamped, Pose, Vector3, Twist
from tf2_ros import TransformListener, TransformBroadcaster, Buffer, Time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

from ultralytics import YOLO
from ultralytics.yolo.engine.results import Results
from ultralytics.yolo.utils.ops import masks2segments
from typing import List

from yolov8_zed.load_zed import LoadZED
from yolov8_zed.transform_conversions import quaternion_vector_to_rigid_transform, transform_stamped_to_rigid_transform


class PotholePerceptionNode(Node):

    def __init__(
        self,
        weights='yolov8s-seg.pt',  # model.pt path(s)
        imgsz=(720, 1280),
        view_img=False,  # show results
        svo='ZEDLIVE',
        send_pointcloud_boolean=True,
    ):
        super().__init__('pothole_perception_node')

        self.pcl_publisher = self.create_publisher(
            PointCloud2, 'pothole_perception_node/pcl', 10)
        self.bb_visual_publisher = self.create_publisher(
            MarkerArray, 'pothole_perception_node/bounding_box_visualize', 10)
        self.segment_visual_publisher = self.create_publisher(
            MarkerArray, 'pothole_perception_node/segments_visualize', 10)

        self.bb_publisher = self.create_publisher(
            Float64MultiArray, 'pothole_perception_node/bounding_box', 10)
        self.segment_publisher = self.create_publisher(
            Float64MultiArray, 'pothole_perception_node/segments', 10)

        self.odom_publisher = self.create_publisher(
            Odometry, '/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.logger = self.get_logger()

        rclpy.spin_once(self)

        # Get transform from vehicle base_link to zed
        while not self.tf_buffer.can_transform('odom', 'map', Time()):
            self.logger.info("Waiting for map to odom")
            rclpy.spin_once(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.
        t.transform.translation.y = 0.
        t.transform.translation.z = 0.
        t.transform.rotation.x = 0.
        t.transform.rotation.y = 0.
        t.transform.rotation.z = 0.
        t.transform.rotation.w = 1.

        self.tf_broadcaster.sendTransform(t)

        while not self.tf_buffer.can_transform('zed_left_camera_frame', 'base_link', Time()):
            self.logger.info("Waiting for base_link to zed_left_camera_frame")
            rclpy.spin_once(self)
        self.base_link_to_zed_tf: TransformStamped = self.tf_buffer.lookup_transform(
            'zed_left_camera_frame',
            'base_link',
            Time())
        self.zed_tf_to_base_link: TransformStamped = self.tf_buffer.lookup_transform(
            'base_link',
            'zed_left_camera_frame',
            Time())

        self.bbox_marker_array = None

        # Initialize run parameters
        self.view_img = view_img
        self.imgsz = imgsz
        self.send_pointcloud_boolean = send_pointcloud_boolean

        # Initialize loader and model
        self.dataloader = LoadZED(
            logger=self.logger,
            svo=svo,
            zed_tf_to_base_link=self.zed_tf_to_base_link,
            base_link_to_zed_tf=self.base_link_to_zed_tf,
            clock=self.get_clock(),
            odom_publisher=self.odom_publisher,
            tf_broadcaster=self.tf_broadcaster,
            img_size=imgsz[0])
        
        torch.backends.cudnn.benchmark = True  # faster for fixed-size inference
        self.model = YOLO(weights)

        if self.view_img and platform.system() == 'Linux':
            # allow window resize (Linux)
            cv2.namedWindow('ZED', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow('ZED', self.imgsz[1], self.imgsz[0])


    def create_odometry_msg(self, posestamped1, posestamped2):
        odom_msg = Odometry()
        odom_msg.header = Header(frame_id = 'base_link', child_frame_id = "odom")
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        pose = Pose()

        odom_msg.pose.pose.position = pose.get_translation().get()
        odom_msg.pose.pose.orientation = pose.get_orientation().get

        x1 = posestamped1.pose.pose.position.x
        y1 = posestamped1.pose.pose.position.y
        x2 = posestamped2.pose.pose.position.x
        y2 = posestamped2.pose.pose.position.y
        distance = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        time = (posestamped2.header.stamp - posestamped1.header.stamp).to_sec()

        quaternion_diff = [
            posestamped2.pose.orientation.x - posestamped1.pose.orientation.x,
            posestamped2.pose.orientation.y - posestamped1.pose.orientation.y,
            posestamped2.pose.orientation.z - posestamped1.pose.orientation.z,
            posestamped2.pose.orientation.w - posestamped1.pose.orientation.w]

        angle_diff = 2 * np.acos(quaternion_diff[3])  #Differentce between two quanernions
        axis_diff = [
            quaternion_diff[2] / np.sin(angle_diff / 2),
            -quaternion_diff[1] / np.sin(angle_diff / 2),
            quaternion_diff[0] / np.sin(angle_diff / 2)]
        angular_velocity = [
            axis_diff[0] * angle_diff / time,
            axis_diff[1] * angle_diff / time,
            axis_diff[2] * angle_diff / time]


        odom_msg.twist.twist.angular = angular_velocity
        odom_msg.twist.twist.linear = distance/time

        return odom_msg   # I tried

    def create_pointcloud2_message(self, points):
        header = Header(frame_id='zed_left_camera_frame')
        header.stamp = self.get_clock().now().to_msg()
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4,
                             datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8,
                             datatype=PointField.FLOAT32, count=1),
                  PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]
        point_step = 16
        # height = points.shape[0]
        width = int(np.prod(points.shape[:-1]))
        row_step = point_step * width
        data = points.tobytes()
        pc2 = PointCloud2(header=header,
                          height=1,
                          width=width,
                          is_dense=False,
                          is_bigendian=False,
                          fields=fields,
                          point_step=point_step,
                          row_step=row_step,
                          data=data)
        return pc2


    def create_bboxes(self, pcl, pothole_masks):
        pothole_masks = pothole_masks.astype(bool)
        pothole_pcl = [pcl[mask] for mask in pothole_masks]
        pothole_pcl = [cur[np.isfinite(cur).all(axis=1)] for cur in pothole_pcl]

        bbox_global = [
            [[cur[:, 0].min(), cur[:, 1].min(), cur[:, 2].min()],
             [cur[:, 0].max(), cur[:, 1].max(), cur[:, 2].max()]]
            for cur in pothole_pcl if 0 not in cur.shape]
        self.get_logger().info(f'bbox: {bbox_global}')

        return bbox_global

    def process_masks_and_segments(self, masks, classifications, pcl,
                                   only_potholes_on_road=True,
                                   process_pothole_segments=False):
        '''
        Separates pothole and road bounding masks and polygons, optionally filtering potholes that
        are constrained within a segmented road.
        '''
        road_masks_flattened = torch.sum(
            masks[classifications == 1, :, :], dim=0, keepdim=True, dtype=bool)
        road_segments = np.array(
            [elt.astype(int) for elt in reversed(masks2segments(road_masks_flattened))])
        pothole_masks = masks[classifications == 0, :, :]
        pothole_segments = list(reversed(masks2segments(pothole_masks)))
        pothole_masks = pothole_masks.cpu().detach().numpy()

        if only_potholes_on_road and len(road_segments) > 0:
            road_contour_mask = np.zeros(
                road_masks_flattened.shape, dtype=np.uint8)
            cv2.drawContours(
                road_contour_mask[0], road_segments, -1, (255, 0, 0), thickness=-1)
            pothole_filtered_masks = np.logical_and(
                pothole_masks.astype(bool),
                road_contour_mask.astype(bool)).astype(int)
            filtered_sums = pothole_filtered_masks.sum(axis=(1, 2))
            full_sums = pothole_masks.sum(axis=(1, 2))
            filtered_indices = filtered_sums > (full_sums//2)
            pothole_masks = pothole_masks[filtered_indices, :, :]
            pothole_segments = [seg for i, seg in enumerate(
                pothole_segments) if filtered_indices[i] == 1]

        road_segments_xyz = [pcl[road_segment[:, 1], road_segment[:, 0], :3].astype(
            float) for road_segment in road_segments]
        if process_pothole_segments:
            pothole_segments_xyz = [pcl[pothole_segment[:, 1], pothole_segment[:, 0], :3].astype(
                float) for pothole_segment in pothole_segments]

        return road_segments_xyz, pothole_masks, pothole_segments_xyz if process_pothole_segments else None

    def globalize_pcl_frame(self, pcl, pose, rescaled_img_size):
        pcl_flattened = pcl.reshape((-1, pcl.shape[-1]))
        pcl_stacked = np.hstack(
            (pcl_flattened[:, :3], np.ones((pcl_flattened.shape[0], 1))))
        # self.logger.info(f'{pcl_stacked[0].shape}')

        cur_trans = pose.get_translation().get()
        cur_rot = pose.get_orientation().get()
        #self.logger.info(f'Rotation {cur_rot}')

        cur_transform = quaternion_vector_to_rigid_transform(
            cur_trans, cur_rot)

        full_transform = cur_transform
        # pcl_global = [cur[np.isfinite(cur).all(axis=1)] for cur in pcl_stacked]
        pcl_global = np.dot(full_transform, pcl_stacked.T).T[:, :3]
        pcl_global = pcl_global.reshape(
            (rescaled_img_size[0], rescaled_img_size[1], 3))

        return pcl_global

    def visualize_segments(self, segment_arr, color={'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 1.0}, scale=[0.03, 0.03, 0.03]):
        # Create MarkerArray
        # LOGGER.info(f'segment {segment_arr}')
        marker_array = MarkerArray()
        for i, segment in enumerate(segment_arr):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = scale[0]
            marker.scale.y = scale[1]
            marker.scale.z = scale[2]
            marker.color.r = color['r']
            marker.color.g = color['g']
            marker.color.b = color['b']
            marker.color.a = color['a']
            for point in segment:
                if False in np.isfinite(point):
                    continue
                p = Point()
                # LOGGER.info(f'{point}')
                p.x, p.y, p.z = point
                marker.points.append(p)
            marker_array.markers.append(marker)

        # Publish MarkerArray
        self.segment_visual_publisher.publish(marker_array)

    def visualize_bboxes(self, bboxes):
        if self.bbox_marker_array is not None:
            for marker in self.bbox_marker_array.markers:
                marker.action = Marker.DELETE
                self.bb_visual_publisher.publish(self.bbox_marker_array)
        self.bbox_marker_array = MarkerArray()
        for i, bbox in enumerate(bboxes):
            if False in np.isfinite(bbox):
                self.logger.info(f"Not finite: {bbox}")
                continue
            corner1, corner2 = bbox[0], bbox[1]
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            # marker.ns = "hollow_rectangular_prism"
            marker.id = 2*i
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.01
            marker.color.r = 1.0
            marker.color.a = 1.0

            # Define the vertices of the hollow rectangular prism
            p1 = Point()
            p1.x, p1.y, p1.z = corner1[0], corner1[1], corner1[2]
            p2 = Point()
            p2.x, p2.y, p2.z = corner2[0], corner1[1], corner1[2]
            p3 = Point()
            p3.x, p3.y, p3.z = corner2[0], corner1[1], corner2[2]
            p4 = Point()
            p4.x, p4.y, p4.z = corner1[0], corner1[1], corner2[2]
            p5 = Point()
            p5.x, p5.y, p5.z = corner1[0], corner2[1], corner1[2]
            p6 = Point()
            p6.x, p6.y, p6.z = corner2[0], corner2[1], corner1[2]
            p7 = Point()
            p7.x, p7.y, p7.z = corner2[0], corner2[1], corner2[2]
            p8 = Point()
            p8.x, p8.y, p8.z = corner1[0], corner2[1], corner2[2]

            # Add the vertices to the marker points list
            marker.points = [
                p1, p2,
                p2, p3,
                p3, p4,
                p4, p1,

                p5, p6,
                p6, p7,
                p7, p8,
                p8, p5,

                p1, p5,
                p2, p6,
                p3, p7,
                p4, p8,
            ]
            header = Header(frame_id='odom')
            position_point = p1
            number_marker = Marker(
                header=header,
                ns='number_display',
                id=2*i+1,
                type=Marker.TEXT_VIEW_FACING,
                action=Marker.ADD,
                pose=Pose(position=position_point),
                scale=Vector3(x=0.1, y=0.1, z=0.1),
                color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
                text=f'l:{corner2[0]-corner1[0]:.3f},w:{corner2[1]-corner1[1]:.3f},d:{corner2[2]-corner1[2]:.3f}'
            )

            self.bbox_marker_array.markers.append(marker)
            self.bbox_marker_array.markers.append(number_marker)

        self.bb_visual_publisher.publish(self.bbox_marker_array)

    def send_pointcloud(self, masks, pcl, pothole_masks, potholes_only=False):
        pothole_masks_flattened = np.sum(pothole_masks, axis=0).astype(bool)
        # Coloring the pothole pixels red
        hex_num = np.int32(0x00FF0000)
        pcl[pothole_masks_flattened, 3] = (
            pcl[pothole_masks_flattened, 3].view(np.int32) | hex_num).view(np.float32)
        if not potholes_only:
            flattened_masks = np.sum(
                masks.cpu().detach().numpy(), axis=0).astype(bool)
        pcl_masked = pcl[pothole_masks_flattened if potholes_only else flattened_masks]
        pcl_msg = self.create_pointcloud2_message(pcl_masked)
        self.pcl_publisher.publish(pcl_msg)

    def send_segments(self, road_segment):
        segments_sent = [
            point for points in road_segment for point in points if False not in np.isfinite(points)]
        
        #rows = len(road_segment)
        rows=len(segments_sent)//3
        self.get_logger().info(f'rows:{rows}')
        columns = 3
        seg_msg = Float64MultiArray()
        seg_msg.data = segments_sent
        seg_msg.layout.dim.append(MultiArrayDimension(
            label='rows', size=rows, stride=rows*columns))
        seg_msg.layout.dim.append(MultiArrayDimension(
            label='columns', size=columns, stride=columns))
        #self.get_logger().info(f'seg msg:{seg_msg}')
        self.segment_publisher.publish(seg_msg)

    def send_bboxes(self, bboxes):
        # bboxes is nx2x3

        bboxes_sent = [np.reshape(item, -1)
                       for item in bboxes if False not in np.isfinite(item)]
        rows = len(bboxes_sent)
        #self.get_logger().info(f'boxes sent pt1:{bboxes_sent}')
        bboxes_sent = [item for sublist in bboxes_sent for item in sublist]
        #self.get_logger().info(f'boxes sent:{bboxes_sent}')
        columns = 6
        bbox_msg = Float64MultiArray()
        bbox_msg.data = bboxes_sent
        bbox_msg.layout.dim.append(MultiArrayDimension(
            label='rows', size=rows, stride=rows*columns))
        bbox_msg.layout.dim.append(MultiArrayDimension(
            label='columns', size=columns, stride=columns))
        

        self.bb_publisher.publish(bbox_msg)
       
    def run(self):

        for img, pcl, pose in self.dataloader:

            results: List[Results] = self.model.predict(
                img, imgsz=self.imgsz, iou=0.7, conf=0.25)
            for result in results:

                if result.masks is None:
                    continue

                if self.view_img:
                    cv2.imshow('ZED', result.plot()[:, :, ::-1])
                    if cv2.waitKey(1) == ord('q'):  # 1 millisecond
                        exit()

                # init_time = time.time()

                classifications = result.boxes.cls
                masks = result.masks.data
                rescaled_img_size = result.masks.shape[1:]
                if rescaled_img_size[0] > self.imgsz[0]:
                    pcl_pad = (rescaled_img_size[0]-self.imgsz[0])//2
                    pcl = np.pad(pcl, ((pcl_pad, pcl_pad), (0, 0), (0, 0)))

                pcl_global = self.globalize_pcl_frame(
                    pcl, pose, rescaled_img_size)
                road_segments_xyz, pothole_masks, _ = self.process_masks_and_segments(
                    masks, classifications, pcl_global, False)

                bbox_global = self.create_bboxes(pcl_global, pothole_masks)

                # Sending segments
                self.visualize_segments(road_segments_xyz)
                self.send_segments(road_segments_xyz[0])

                # Coloring and sending potholes
                if self.send_pointcloud_boolean:
                    self.send_pointcloud(
                        masks, pcl, pothole_masks, potholes_only=True)

                # Sending bounding boxes
                self.visualize_bboxes(bbox_global)
                self.send_bboxes(bbox_global)


    def terminate(self):
        cv2.destroyAllWindows()


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str,
                        default='yolov8s-seg.pt', help='model path(s)')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+',
                        type=int, default=[720, 1280], help='inference size h,w')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--svo', type=str, default='ZEDLIVE', help='open an svo file')

    opt, unknown = parser.parse_known_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    return opt, unknown


def main(args=None):
    opt, _ = parse_opt()

    rclpy.init()

    pothole_perception_node = PotholePerceptionNode(**vars(opt))

    while rclpy.ok():

        pothole_perception_node.run()

    pothole_perception_node.terminate()

    pothole_perception_node.destroy_node()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
