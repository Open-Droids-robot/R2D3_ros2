"""
Gz Sim 8 (Gazebo Harmonic) Compatible File 
"""
import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_name_in_model = 'dual_rm_65b_description'

    pkg_gazebo = FindPackageShare(package='dual_rm_gazebo').find('dual_rm_gazebo')
    pkg_description = get_package_share_directory('dual_rm_65b_description')

    # Use the Gz Sim-compatible xacro
    urdf_xacro_path = os.path.join(pkg_gazebo, 'config', 'dual_rm_65b_gz_sim.urdf.xacro')

    # Process xacro → URDF via Command substitution (official Jazzy pattern)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        urdf_xacro_path,
    ])
    robot_description = {'robot_description': robot_description_content}

    # Gz Sim resource path: parent of package share so Gz can resolve
    # file:// URIs and package-relative paths for meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_description, 'meshes')
    )

    # Launch Gz Sim (Harmonic) via ros_gz_sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf',
        }.items(),
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, robot_description, {'publish_frequency': 15.0}],
        output='screen',
    )

    # Spawn entity into Gz Sim via topic (official pattern: avoids CLI length limits)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name_in_model,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.25',
            '-allow_renaming', 'true',
        ],
        output='screen',
    )

    # --- Controller Spawners (Jazzy pattern: Node with 'spawner' executable) ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller'],
    )

    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller'],
    )

    platform_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['platform_controller'],
    )

    # --- Event sequencing ---
    # After spawn completes → load joint_state_broadcaster
    evt_spawn_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    # After joint_state_broadcaster → load arm + platform controllers
    evt_jsb_done_left = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )
    evt_jsb_done_right = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )
    evt_jsb_done_platform = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[platform_controller_spawner],
        )
    )

    # Bridge /clock from Gz to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        gz_resource_path,
        bridge,
        evt_spawn_done,
        evt_jsb_done_left,
        evt_jsb_done_right,
        evt_jsb_done_platform,
        gz_sim,
        node_robot_state_publisher,
        spawn_entity,
    ])
