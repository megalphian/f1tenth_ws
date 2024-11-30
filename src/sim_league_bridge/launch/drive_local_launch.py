from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():

    # Path to the other launch file
    other_launch_file = os.path.join(
        get_package_share_directory('autodrive_f1tenth'),
        'launch',
        'simulator_bringup_headless.launch.py'
    )

    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(other_launch_file),
        ),
        Node(
            package='sim_league_bridge',
            executable='vehicle_controller.py',
            name='vehicle_controller',
            output='screen',
        ),
        Node(
            package='sim_league_bridge',
            executable='wall_follower',
            name='wall_follower',
            output='screen',
        ),
    ])

    return ld