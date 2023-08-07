from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    roboname = 'catvehicle'

    system_resources_pkg_share = FindPackageShare(package='system_resources').find('system_resources')
    yolov8_zed_pkg_share = FindPackageShare(package='yolov8_zed').find('yolov8_zed')

    default_urdf_path = os.path.join(system_resources_pkg_share, 'urdf', 'catvehicle.xacro')
    default_rviz_config_path = os.path.join(system_resources_pkg_share, 'rviz', 'full_system.rviz')
    default_svo_path = 'ZEDLIVE'
    default_perception_model = os.path.join(system_resources_pkg_share, 'weights', 'yolov8n-pothole.pt')


    pothole_perception_node = Node(
        package='yolov8_zed',
        executable='pothole_perception_node',
        arguments=[
            '--weights', LaunchConfiguration('weights'),
            '--svo', LaunchConfiguration('svo'),
            '--view-img'
        ]
    )
    
    static_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',
            '0', '0', '0', '1',
            'map',
            'odom'
        ]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            default_rviz_config_path
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro ', LaunchConfiguration('urdf'), ' ',
                    'roboname:=', LaunchConfiguration('roboname')
                ])
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    pure_pursuit_node = Node(
        package='pure_pursuit',
        name='pure_pursuit',
        executable='pure_pursuit'
    )

    arduino_interface_node = Node(
        package='arduino_interface',
        name='arduino_interface',
        executable='arduino_interface'
    )

    path_planner_node = Node(
        package='pothole_path_planner',
        name='pothole_path_planner',
        executable='pothole_path_planner'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='svo', default_value=default_svo_path,
                                            description='Absolute path to svo file'),
        DeclareLaunchArgument(name='weights', default_value=default_perception_model,
                                            description='Name of model weights to use'),
        DeclareLaunchArgument(name='urdf', default_value=default_urdf_path,
                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='roboname', default_value=roboname,
                                            description='Robot name'),
        pothole_perception_node,
        static_publisher_node,
        pure_pursuit_node,
        path_planner_node,
        # arduino_interface_node,
        rviz_node,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])