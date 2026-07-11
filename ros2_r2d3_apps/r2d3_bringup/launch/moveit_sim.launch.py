"""
MoveIt2 move_group launch for simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ── Arguments ─────────────────────────────────────────────────
    declare_robot_model = DeclareLaunchArgument(
        "robot_model",
        default_value="65b",
        description="Robot model variant: 65b or 75b",
    )
    robot_model = LaunchConfiguration("robot_model")

    # ── Load MoveIt configs from the 65b package ──────────────────
    moveit_config = MoveItConfigsBuilder(
        "dual_rm_65b_description",
        package_name="dual_rm_65b_moveit_config",
    ).to_moveit_configs()

    # ── Build the full parameter dict for move_group ──────────────
    moveit_params = moveit_config.to_dict()
    moveit_params["use_sim_time"] = True

    # ── move_group node ───────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    return LaunchDescription(
        [
            declare_robot_model,
            move_group_node,
        ]
    )
