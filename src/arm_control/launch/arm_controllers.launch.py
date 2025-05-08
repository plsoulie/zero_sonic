#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "left_arm_port",
            default_value="/dev/ttyUSB0",
            description="Port for the left arm Waveshare servo bus",
        ),
        DeclareLaunchArgument(
            "right_arm_port",
            default_value="/dev/ttyUSB1",
            description="Port for the right arm Waveshare servo bus",
        ),
        DeclareLaunchArgument(
            "left_arm_config",
            default_value="left_arm_config.yaml",
            description="Configuration file for left arm controller",
        ),
        DeclareLaunchArgument(
            "right_arm_config",
            default_value="right_arm_config.yaml",
            description="Configuration file for right arm controller",
        ),
        DeclareLaunchArgument(
            "controller_config",
            default_value="controller_config.yaml",
            description="Configuration file for controllers",
        ),
    ]

    # Initialize Arguments
    left_arm_port = LaunchConfiguration("left_arm_port")
    right_arm_port = LaunchConfiguration("right_arm_port")
    left_arm_config = LaunchConfiguration("left_arm_config")
    right_arm_config = LaunchConfiguration("right_arm_config")
    controller_config = LaunchConfiguration("controller_config")
    
    # Get the package directory
    pkg_dir = get_package_share_directory('arm_control')
    
    # Configuration paths
    left_arm_config_path = os.path.join(pkg_dir, 'config', left_arm_config)
    right_arm_config_path = os.path.join(pkg_dir, 'config', right_arm_config)
    controller_config_path = os.path.join(pkg_dir, 'config', controller_config)
    
    # Load the URDF model
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_dir, "urdf", "so100_arm.urdf.xacro"]),
            " ",
        ]
    )
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
    )
    
    # Controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            controller_config_path,
        ],
        output="screen",
    )
    
    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # Spawn left arm controller
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Spawn right arm controller
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Launch left arm controller node
    left_arm_node = Node(
        package="arm_control",
        executable="left_arm_controller",
        name="left_arm_controller",
        namespace="left_arm",
        parameters=[
            {"port": left_arm_port},
            left_arm_config_path,
        ],
        output="screen",
    )
    
    # Launch right arm controller node
    right_arm_node = Node(
        package="arm_control",
        executable="right_arm_controller",
        name="right_arm_controller",
        namespace="right_arm",
        parameters=[
            {"port": right_arm_port},
            right_arm_config_path,
        ],
        output="screen",
    )
    
    # Create the launch description
    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            right_arm_controller_spawner,
            left_arm_node,
            right_arm_node,
        ]
    )
