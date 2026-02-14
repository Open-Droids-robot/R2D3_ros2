"""
Gazebo Harmonic (Gz Sim) launch for the R2D3 dual-arm mobile robot.

Brings up:
  - Gz Sim with the requested world
  - robot_state_publisher  (URDF → TF)
  - ros_gz_bridge          (clock, LiDAR, IMU)
  - ros2_control controllers (diff_drive, arms, platform)

This file lives in dual_rm_simulation and is purely about simulation.
Navigation / SLAM / RViz are launched separately from dual_rm_navigation.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_sim = get_package_share_directory('dual_rm_simulation')
    desc_65b_dir = get_package_share_directory('dual_rm_65b_description')
    desc_75b_dir = get_package_share_directory('dual_rm_75b_description')

    # ── Launch arguments ─────────────────────────────────────────
    declare_robot_model = DeclareLaunchArgument(
        'robot_model', default_value='65b',
        description='Robot model variant: 65b or 75b',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_sim, 'worlds', 'nav_empty.sdf'),
        description='Full path to Gz Sim world SDF file',
    )
    declare_gz_verbosity = DeclareLaunchArgument(
        'gz_verbosity', default_value='1',
        description='Gz Sim verbosity level (0-4)',
    )

    robot_model = LaunchConfiguration('robot_model')
    world = LaunchConfiguration('world')
    gz_verbosity = LaunchConfiguration('gz_verbosity')

    # ── Resolve xacro path based on robot_model ──────────────────
    urdf_xacro_path = PathJoinSubstitution([
        FindPackageShare('dual_rm_simulation'), 'urdf',
        PythonExpression([
            "'r2d3_' + '", robot_model, "' + '_sim.urdf.xacro'"
        ]),
    ])

    # ── Process xacro → robot_description ────────────────────────
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ', urdf_xacro_path,
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    # ── GZ_SIM_RESOURCE_PATH ─────────────────────────────────────
    set_resource_path_65b = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(desc_65b_dir, 'meshes'))
    set_resource_path_75b = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(desc_75b_dir, 'meshes'))
    set_resource_path_desc65 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', str(Path(desc_65b_dir).parent.resolve()))
    set_resource_path_desc75 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', str(Path(desc_75b_dir).parent.resolve()))
    set_resource_path_worlds = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_sim, 'worlds'))

    # ── Launch Gz Sim ────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': ['-r -v ', gz_verbosity, ' ', world],
        }.items(),
    )

    # ── Robot State Publisher ────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            robot_description,
        ],
    )

    # ── Spawn robot into Gz Sim ──────────────────────────────────
    # No spawn yaw needed: the URDF base_footprint_to_base joint already
    # rotates -90° so that base_footprint +X = robot front.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'r2d3_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.05',
            '-allow_renaming', 'true',
        ],
    )

    # ── ros_gz_bridge (YAML config) ──────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_sim, 'config', 'gz_bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ── Controller spawners ──────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_drive_controller'],
    )
    left_arm_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['left_arm_controller'],
    )
    right_arm_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['right_arm_controller'],
    )
    platform_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['platform_controller'],
    )

    # ── Event sequencing: spawn → JSB → other controllers ────────
    evt_spawn_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    evt_jsb_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                diff_drive_controller_spawner,
                left_arm_controller_spawner,
                right_arm_controller_spawner,
                platform_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        declare_robot_model,
        declare_world,
        declare_gz_verbosity,

        set_resource_path_65b,
        set_resource_path_75b,
        set_resource_path_desc65,
        set_resource_path_desc75,
        set_resource_path_worlds,
        gz_sim,
        bridge,
        robot_state_publisher,
        spawn_entity,
        evt_spawn_done,
        evt_jsb_done,
    ])
