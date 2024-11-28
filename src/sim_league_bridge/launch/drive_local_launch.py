from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription([
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