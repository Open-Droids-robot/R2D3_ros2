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
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav = get_package_share_directory('dual_rm_navigation')

    # ── Arguments ─────────────────────────────────────────────────
    declare_robot_model = DeclareLaunchArgument(
        'robot_model', default_value='65b',
        description='Robot model variant: 65b or 75b',
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value=os.path.join(pkg_nav, 'worlds', 'nav_empty.sdf'),
        description='Gz Sim world file (full path or name)',
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

    # ── 1. Gz Sim + Robot (controllers, sensors, bridge)
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'robot_model': robot_model,
            'world': world,
            'use_rviz': use_rviz,
        }.items(),
    )

    # ── 2. SLAM Toolbox (mapping mode) 
    # Delayed start to allow Gz Sim, sensors, and controllers to initialize.
    # The diff_drive_controller (odom→base_footprint TF) must be active first.
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

    # ── 3. Localization (AMCL + map_server) 
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

    # ── 4. Nav2 navigation stack 
    # Delayed further to let SLAM/localization establish the map→odom TF
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
        slam_launch,
        localization_launch,
        nav2_launch,
    ])
