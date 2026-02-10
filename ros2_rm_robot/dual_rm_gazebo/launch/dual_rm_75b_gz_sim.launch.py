"""
Gazebo Harmonic (gz sim 8) launch file for dual RM-75B robot.
This is the Jazzy-compatible equivalent of dual_rm_75b_gazebo.launch.py.
"""
import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    package_name = 'dual_rm_gazebo'
    robot_name_in_model = 'dual_rm_75b_description'

    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Use the Gz Sim-compatible xacro
    urdf_model_path = os.path.join(pkg_share, 'config/dual_rm_75b_gz_sim.urdf.xacro')

    print("---", urdf_model_path)

    # Parse xacro → URDF string
    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    robot_description = doc.toxml()
    params = {'robot_description': robot_description}

    # --- Gz Sim (Harmonic) ---
    # Set GZ_SIM_RESOURCE_PATH so Gz can find meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(
            get_package_share_directory('dual_rm_75b_description'), 'meshes'
        )
    )

    # Launch Gz Sim via ros_gz_sim
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
        parameters=[{'use_sim_time': True}, params, {'publish_frequency': 15.0}],
        output='screen',
    )

    # Spawn the robot into Gz Sim using ros_gz_sim's create node
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description,
            '-name', robot_name_in_model,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.25',
        ],
        output='screen',
    )

    # --- ros2_control Controllers ---
    # Joint State Broadcaster
    load_joint_state_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster',
        ],
        output='screen',
    )

    # Left Arm Controller
    load_left_arm_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            'left_arm_controller',
        ],
        output='screen',
    )

    # Right Arm Controller
    load_right_arm_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            'right_arm_controller',
        ],
        output='screen',
    )

    # Platform Controller
    load_platform_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            'platform_controller',
        ],
        output='screen',
    )

    # --- Event sequencing ---
    # After spawn completes → load joint_state_broadcaster
    close_evt1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )
    # After joint_state_broadcaster → load arm controllers + platform
    close_evt2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_left_arm_controller],
        )
    )
    close_evt3 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_right_arm_controller],
        )
    )
    close_evt4 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_platform_controller],
        )
    )

    # Bridge /clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        bridge,
        close_evt1,
        close_evt2,
        close_evt3,
        close_evt4,
        gz_sim,
        node_robot_state_publisher,
        spawn_entity,
    ])
