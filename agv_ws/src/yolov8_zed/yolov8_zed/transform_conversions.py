from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Quaternion
from scipy.spatial.transform import Rotation
import numpy as np

def quaternion_vector_to_rigid_transform(translation, quaternion):
    rotation = Rotation.from_quat(quaternion).as_matrix()
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform

def transform_stamped_to_rigid_transform(transform: TransformStamped):
    quaternion = [
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    ]
    translation = [
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ]
    return quaternion_vector_to_rigid_transform(translation, quaternion)

def pose_stamped_to_rigid_transform(pose: PoseStamped):
    quaternion = [
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w
    ]
    translation = [
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
    ]
    return quaternion_vector_to_rigid_transform(translation, quaternion)

def transform_pose(pose: PoseStamped, transform: TransformStamped) -> PoseStamped:
    transform_matrix = transform_stamped_to_rigid_transform(transform)
    pose_matrix = pose_stamped_to_rigid_transform(pose)
    transformed_pose = transform_matrix @ pose_matrix

    transformed_quat = Rotation.from_matrix(transformed_pose[:3,:3]).as_quat()
    transformed_position = transformed_pose[:3,3]

    new_pose = PoseStamped()
    new_pose.header.frame_id = transform.child_frame_id
    new_pose.header.stamp = pose.header.stamp
    new_pose.pose.orientation = Quaternion(
        x=transformed_quat[0],
        y=transformed_quat[1],
        z=transformed_quat[2],
        w=transformed_quat[3]
        )
    new_pose.pose.position = Point(
        x=transformed_position[0],
        y=transformed_position[1],
        z=transformed_position[2],
    )
    return new_pose
    