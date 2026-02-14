"""
Full navigation bringup for R2D3 in Gazebo Harmonic simulation.

Orchestrates three layers with timed delays:
  t=0s   — Simulation  (dual_rm_simulation: Gz Sim + robot + controllers + bridge)
  t=20s  — Mapping/Loc  (SLAM Toolbox  OR  AMCL + map_server)
  t=30s  — Navigation   (Nav2 stack)

RViz2 is launched alongside the simulation for immediate visualisation.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_sim = get_package_share_directory('dual_rm_simulation')
    pkg_nav = get_package_share_directory('dual_rm_navigation')

    # ── Arguments ─────────────────────────────────────────────────
    declare_robot_model = DeclareLaunchArgument(
        'robot_model', default_value='65b',
        description='Robot model variant: 65b or 75b',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_sim, 'worlds', 'nav_empty.sdf'),
        description='Gz Sim world file (full path)',
    )
    declare_mode = DeclareLaunchArgument(
        'mode', default_value='slam',
        description="'slam' for mapping, 'localization' for existing map",
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to map YAML (required for localization mode)',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 with navigation view',
    )
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
    )
    declare_slam_params = DeclareLaunchArgument(
        'slam_params',
        default_value=os.path.join(pkg_nav, 'config', 'slam_toolbox_params.yaml'),
    )

    robot_model = LaunchConfiguration('robot_model')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    nav2_params = LaunchConfiguration('nav2_params')
    slam_params = LaunchConfiguration('slam_params')

    # ── 1. Simulation (Gz Sim + robot + controllers + bridge) ────
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'robot_model': robot_model,
            'world': world,
        }.items(),
    )

    # ── 2. RViz2 (navigation visualisation) ──────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_nav, 'rviz', 'nav2_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # ── 3. SLAM Toolbox (mapping mode) ───────────────────────────
    #    Delayed to let Gz Sim, sensors, and controllers start first.
    slam_launch = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, 'launch', 'slam.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': slam_params,
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", mode, "' == 'slam'"]),
                ),
            ),
        ],
    )

    # ── 4. Localization (AMCL + map_server) ──────────────────────
    localization_launch = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, 'launch', 'localization.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params,
                    'map': map_yaml,
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", mode, "' == 'localization'"]),
                ),
            ),
        ],
    )

    # ── 5. Nav2 navigation stack ─────────────────────────────────
    #    Delayed further to let SLAM/localization establish the map→odom TF.
    nav2_launch = TimerAction(
        period=30.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params,
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        declare_robot_model,
        declare_world,
        declare_mode,
        declare_map,
        declare_use_rviz,
        declare_nav2_params,
        declare_slam_params,

        gz_sim_launch,
        rviz_node,
        slam_launch,
        localization_launch,
        nav2_launch,
    ])
