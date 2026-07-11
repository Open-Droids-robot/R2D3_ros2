"""
Display the R2D3 robot in RViz with joint_state_publisher_gui.

Usage:
  ros2 launch dual_rm_description display.launch.py                 # default: 65b
  ros2 launch dual_rm_description display.launch.py arm_model:=75b  # 75b variant
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # -- Arguments --------------------------------------------------------
    declare_arm_model = DeclareLaunchArgument(
        'arm_model',
        default_value='65b',
        choices=['65b', '75b'],
        description='Arm variant: 65b (RM-65B, 6-DOF) or 75b (RM-75B, 7-DOF)',
    )

    # -- Robot description via xacro -------------------------------------
    pkg_share = FindPackageShare('dual_rm_description')
    urdf_file = PathJoinSubstitution(
        [pkg_share, 'urdf', 'r2d3_description.urdf.xacro']
    )

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' arm_model:=', LaunchConfiguration('arm_model'),
        ]),
        value_type=str,
    )

    # -- Nodes ------------------------------------------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'view.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        declare_arm_model,
        rsp_node,
        jsp_gui_node,
        rviz_node,
    ])
