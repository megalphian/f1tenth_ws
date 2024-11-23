from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    kinematic_icp_file = os.path.join(
        get_package_share_directory('kinematic_icp'),
        'launch',
        'online_node.launch.py'
    )

    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinematic_icp_file),
            launch_arguments={
                'lidar_topic': "/autodrive/f1tenth_1/lidar",
                'tf_timeout': 0.4
            }.items()
        )
    ])

    return ld