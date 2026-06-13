#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_name = "toys_collector_description"

    xacro_file = "toys_collector.urdf.xacro"
    xacro_path = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "urdf", "robot", xacro_file]
    )

    rviz_file = "toys_collector_description.rviz"
    rviz_path = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "rviz", rviz_file]
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    joint_state_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_gui,
        rviz_node
    ])