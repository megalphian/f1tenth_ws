from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the other launch file
    other_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    mapping_params_file = os.path.join(
        get_package_share_directory('sim_league_bridge'),
        'config',
        'mapping_params.yaml'
    )


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(other_launch_file),
            launch_arguments={
                'slam_params_file': mapping_params_file
            }.items()
        ),
    ])