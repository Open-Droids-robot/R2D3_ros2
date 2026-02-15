"""
Full navigation bringup for R2D3 in Gazebo Harmonic simulation.
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
    declare_slam_type = DeclareLaunchArgument(
        'slam_type', default_value='slam_toolbox',
        description="SLAM backend: 'slam_toolbox' (2D LiDAR), "
                    "'rtabmap' (RGB-D + LiDAR), or "
                    "'rtabmap_depth_only' (RGB-D only, no LiDAR)",
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to map YAML (required for localization mode with slam_toolbox)',
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
    declare_rtabmap_params = DeclareLaunchArgument(
        'rtabmap_params',
        default_value=os.path.join(pkg_nav, 'config', 'rtabmap_params.yaml'),
    )

    robot_model = LaunchConfiguration('robot_model')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    slam_type = LaunchConfiguration('slam_type')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    nav2_params = LaunchConfiguration('nav2_params')
    slam_params = LaunchConfiguration('slam_params')
    rtabmap_params = LaunchConfiguration('rtabmap_params')

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

    # ── 3a. SLAM Toolbox (mapping mode, 2D LiDAR-based) ──────────
    slam_toolbox_launch = TimerAction(
        period=10.0,   # controllers ready by ~6s; SLAM needs odom→base_footprint TF
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
                    PythonExpression([
                        "'", mode, "' == 'slam' and '", slam_type, "' == 'slam_toolbox'"
                    ]),
                ),
            ),
        ],
    )

    # ── 3b. RTAB-Map SLAM (mapping mode, RGB-D + LiDAR) ──────────
    rtabmap_slam_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, 'launch', 'rtabmap.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': rtabmap_params,
                    'localization': 'false',
                }.items(),
                condition=IfCondition(
                    PythonExpression([
                        "'", mode, "' == 'slam' and '", slam_type, "' == 'rtabmap'"
                    ]),
                ),
            ),
        ],
    )

    # ── 3c. RTAB-Map Localization (existing map, RGB-D + LiDAR) ────
    rtabmap_loc_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, 'launch', 'rtabmap.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': rtabmap_params,
                    'localization': 'true',
                }.items(),
                condition=IfCondition(
                    PythonExpression([
                        "'", mode, "' == 'localization' and '", slam_type, "' == 'rtabmap'"
                    ]),
                ),
            ),
        ],
    )

    # ── 3d. RTAB-Map Depth-Only SLAM (RGB-D only, no LiDAR) ──────
    rtabmap_depth_slam_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, 'launch', 'rtabmap_depth_only.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'localization': 'false',
                }.items(),
                condition=IfCondition(
                    PythonExpression([
                        "'", mode, "' == 'slam' and '", slam_type, "' == 'rtabmap_depth_only'"
                    ]),
                ),
            ),
        ],
    )

    # ── 3e. RTAB-Map Depth-Only Localization (existing map) ───────
    rtabmap_depth_loc_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, 'launch', 'rtabmap_depth_only.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'localization': 'true',
                }.items(),
                condition=IfCondition(
                    PythonExpression([
                        "'", mode, "' == 'localization' and '", slam_type, "' == 'rtabmap_depth_only'"
                    ]),
                ),
            ),
        ],
    )

    # ── 4. Localization with AMCL + map_server (slam_toolbox backend) ──
    localization_launch = TimerAction(
        period=10.0,
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
                    PythonExpression([
                        "'", mode, "' == 'localization' and '", slam_type, "' == 'slam_toolbox'"
                    ]),
                ),
            ),
        ],
    )

    # ── 5. Nav2 navigation stack ─────────────────────────────────
    nav2_launch = TimerAction(
        period=10.0,  # same as SLAM; lifecycle_manager waits for all services
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
        declare_slam_type,
        declare_map,
        declare_use_rviz,
        declare_nav2_params,
        declare_slam_params,
        declare_rtabmap_params,

        gz_sim_launch,
        rviz_node,
        slam_toolbox_launch,
        rtabmap_slam_launch,
        rtabmap_loc_launch,
        rtabmap_depth_slam_launch,
        rtabmap_depth_loc_launch,
        localization_launch,
        nav2_launch,
    ])
