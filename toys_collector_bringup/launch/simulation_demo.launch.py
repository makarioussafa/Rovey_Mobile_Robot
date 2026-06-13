#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('toys_collector_gazebo')
    bringup_pkg = get_package_share_directory('toys_collector_bringup')

    gazebo_launch = os.path.join(
        gazebo_pkg,
        'launch',
        'toys_collector_gazebo.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        Node(
            package='toys_collector_bringup',
            executable='path_executor_node.py',
            name='path_executor_node',
            output='screen',
        )
    ])