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
            "arm_side",
            default_value="left",
            description="Which arm to launch: 'left' or 'right'",
        ),
        DeclareLaunchArgument(
            "arm_port",
            default_value="/dev/ttyUSB0",
            description="Port for the arm's Waveshare servo bus",
        ),
        DeclareLaunchArgument(
            "arm_config",
            default_value="",  # Will be set based on arm_side
            description="Configuration file for arm controller",
        ),
        DeclareLaunchArgument(
            "controller_config",
            default_value="controller_config.yaml",
            description="Configuration file for controllers",
        ),
    ]

    # Initialize Arguments
    arm_side = LaunchConfiguration("arm_side")
    arm_port = LaunchConfiguration("arm_port")
    arm_config = LaunchConfiguration("arm_config")
    controller_config = LaunchConfiguration("controller_config")
    
    # Get the package directory
    pkg_dir = get_package_share_directory('arm_control')
    
    # Set config file based on arm side (assuming the LaunchConfiguration is resolved)
    # Note: In practice, you'll need to use a substitution for this
    left_arm_config = os.path.join(pkg_dir, 'config', 'left_arm_config.yaml')
    right_arm_config = os.path.join(pkg_dir, 'config', 'right_arm_config.yaml')
    controller_config_path = os.path.join(pkg_dir, 'config', controller_config)
    
    # Load the URDF model
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_dir, "urdf", "so100_arm.urdf.xacro"]),
            " ",
            "arm_side:=", arm_side,
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
    
    # Determine controller to spawn based on arm_side
    # In practice, this would be done with LaunchConfiguration evaluations
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[arm_side + "_arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Launch arm controller node
    arm_controller_node = Node(
        package="arm_control",
        executable=arm_side + "_arm_controller",
        name=arm_side + "_arm_controller",
        namespace=arm_side + "_arm",
        parameters=[
            {"port": arm_port},
            {"config_path": left_arm_config if arm_side == "left" else right_arm_config},
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
            arm_controller_spawner,
            arm_controller_node,
        ]
    )
