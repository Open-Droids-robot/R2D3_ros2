"""
Unified simulation bringup: Nav2 + MoveIt2 for the R2D3 dual-arm mobile robot.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_nav = get_package_share_directory("dual_rm_navigation")
    pkg_sim = get_package_share_directory("dual_rm_simulation")
    pkg_bringup = get_package_share_directory("r2d3_bringup")

    # ── Arguments ─────────────────────────────────────────────────
    declare_robot_model = DeclareLaunchArgument(
        "robot_model",
        default_value="65b",
        description="Robot model variant: 65b or 75b",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_sim, "worlds", "nav_empty.sdf"),
        description="Gz Sim world file (full path)",
    )
    declare_mode = DeclareLaunchArgument(
        "mode",
        default_value="slam",
        description="'slam' for mapping, 'localization' for existing map",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Path to map YAML (required for localization mode)",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 with combined Nav2 + MoveIt view",
    )
    declare_use_moveit = DeclareLaunchArgument(
        "use_moveit",
        default_value="true",
        description="Launch MoveIt2 move_group for arm planning",
    )

    robot_model = LaunchConfiguration("robot_model")
    world = LaunchConfiguration("world")
    mode = LaunchConfiguration("mode")
    map_yaml = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("use_rviz")
    use_moveit = LaunchConfiguration("use_moveit")

    # ── MoveIt parameters for RViz ────────────────────────────────
    moveit_config = MoveItConfigsBuilder(
        "dual_rm_65b_description",
        package_name="dual_rm_65b_moveit_config",
    ).to_moveit_configs()

    # ── 1. Navigation stack (Gz Sim + SLAM/localization + Nav2) ──
    nav_bringup = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, "launch", "bringup_sim.launch.py")
                ),
                launch_arguments={
                    "robot_model": robot_model,
                    "world": world,
                    "mode": mode,
                    "map": map_yaml,
                    "use_rviz": "false",
                }.items(),
            ),
        ],
    )

    # ── 2. RViz2 (combined Nav2 + MoveIt visualisation) ──────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_bringup, "rviz", "nav2_moveit_view.rviz")],
        parameters=[
            {"use_sim_time": True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    # ── 3. MoveIt2 move_group (arm planning) ─────────────────────
    moveit_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "moveit_sim.launch.py")
                ),
                launch_arguments={
                    "robot_model": robot_model,
                }.items(),
                condition=IfCondition(use_moveit),
            ),
        ],
    )

    return LaunchDescription(
        [
            declare_robot_model,
            declare_world,
            declare_mode,
            declare_map,
            declare_use_rviz,
            declare_use_moveit,
            nav_bringup,
            rviz_node,
            moveit_launch,
        ]
    )
