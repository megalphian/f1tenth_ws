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

    ekf_launch_file = os.path.join(
        get_package_share_directory('robot_localization'),
        'launch',
        'ekf.launch.py'
    )

    ekf_params_file = os.path.join(
        get_package_share_directory('sim_league_bridge'),
        'config',
        'ekf_params.yaml'
    )

    ld = LaunchDescription([
        Node(
            package='sim_league_bridge',
            executable='vehicle_odom',
            name='vehicle_odom',
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file],
           ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinematic_icp_file),
            launch_arguments={
                'lidar_topic': "/autodrive/f1tenth_1/lidar",
                'tf_timeout': '0.4'
            }.items()
        )
    ])

    return ld