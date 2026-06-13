#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = "toys_collector_gazebo"
    pkg_desc = "toys_collector_description"

    pkg_share_gazebo = FindPackageShare(pkg_gazebo).find(pkg_gazebo)
    pkg_share_desc = FindPackageShare(pkg_desc).find(pkg_desc)

    # -------------------------
    # RViz config
    # -------------------------
    rviz = LaunchConfiguration("rviz")

    rviz_config = os.path.join(
        pkg_share_gazebo,
        "rviz",
        "toys_collector.rviz"
    )

    # -------------------------
    # Launch args
    # -------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    load_controllers = LaunchConfiguration("load_controllers")
    robot_name = LaunchConfiguration("robot_name")
    world_file = LaunchConfiguration("world_file")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    # -------------------------
    # Robot description from xacro
    # -------------------------
    xacro_path = os.path.join(pkg_share_desc, "urdf", "robot", "toys_collector.urdf.xacro")

    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_path, " ", "use_gazebo:=true"]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description,
        }],
    )

    # -------------------------
    # Gazebo resource path
    # -------------------------
    gazebo_models_path = os.path.join(pkg_share_gazebo, "models")
    set_env_models = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gazebo_models_path)

    # -------------------------
    # World path
    # -------------------------
    world_path = PathJoinSubstitution([pkg_share_gazebo, "worlds", world_file])

    # -------------------------
    # Start Gazebo Sim
    # -------------------------
    pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find("ros_gz_sim")
    gz_launch = os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")

    start_gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": ["-r -s -v 4 ", world_path]}.items(),
    )

    start_gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": "-g"}.items(),
        condition=IfCondition(PythonExpression(["'", headless, "' == 'false'"])),
    )

    # -------------------------
    # ROS <-> GZ BRIDGE
    # -------------------------
    bridge_yaml = os.path.join(pkg_share_gazebo, "config", "ros_gz_bridge.yaml")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{
            "config_file": bridge_yaml,
            "use_sim_time": use_sim_time,
        }],
    )

    # -------------------------
    # Spawn robot
    # -------------------------
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", robot_name,
            "-allow_renaming", "true",
            "-x", x,
            "-y", y,
            "-z", z,
            "-R", roll,
            "-P", pitch,
            "-Y", yaw,
        ],
    )

    # -------------------------
    # Controllers spawners
    # -------------------------
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    spawner_diff = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    spawner_gripper = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # -------------------------
    # Startup arm pose
    # -------------------------
    arm_start_pose = ExecuteProcess(
        cmd=[
            "ros2",
            "action",
            "send_goal",
            "/arm_controller/follow_joint_trajectory",
            "control_msgs/action/FollowJointTrajectory",
            (
                '{'
                'trajectory: {'
                'joint_names: ["link_1_of_mechanism_joint", "link_2_of_mechanism_joint"], '
                'points: ['
                '{positions: [0.375,0.028], time_from_start: {sec: 2}}'
                ']'
                '}'
                '}'
            ),
        ],
        output="screen",
        shell=False,
    )

    after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=1.0, actions=[spawner_jsb, spawner_diff, spawner_arm, spawner_gripper])
            ],
        ),
        condition=IfCondition(load_controllers),
    )

   
    after_arm_and_gripper_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner_gripper,
            on_exit=[
                TimerAction(period=2.0, actions=[arm_start_pose])
            ],
        ),
        condition=IfCondition(load_controllers),
    )

    # -------------------------
    # RViz node
    # -------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("load_controllers", default_value="true"),
        DeclareLaunchArgument("robot_name", default_value="toys_collector"),
        DeclareLaunchArgument("world_file", default_value="my_world.sdf"),
        DeclareLaunchArgument("rviz", default_value="true"),

        DeclareLaunchArgument("x", default_value="-0.95"),
        DeclareLaunchArgument("y", default_value="-0.80"),
        DeclareLaunchArgument("z", default_value="0.03"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="1.57"),

        set_env_models,
        rsp,
        start_gz_server,
        start_gz_gui,
        bridge,
        spawn_robot,
        after_spawn,
        after_arm_and_gripper_ready,
        rviz_node,
    ])