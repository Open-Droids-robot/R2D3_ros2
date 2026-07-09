import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = '4C2'
    urdf_package_path = get_package_share_directory(pkg_name)
    default_xacro_path = os.path.join(urdf_package_path, 'urdf', '4C2.xacro')

    env_gazebo_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(urdf_package_path, '..'))
    env_gazebo_models = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '/opt/ros/humble/share/gazebo_ros/worlds/empty.world',
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_force_system.so'],
        output='screen'
    )

    gzclient = ExecuteProcess(cmd=['gzclient', '--verbose'], output='screen')

    robot_description_content = Command(
        ['xacro ', default_xacro_path, ' sim_gazebo:=true']
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Spawn the robot after 3 seconds (gives gzserver time to start)
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', '4C2_hand', '-z', '0.0'],
                output='screen'
            )
        ]
    )

    # Load joint_state_broadcaster after 6 seconds (robot must be spawned first)
    load_joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    # Load gripper_controller after 7 seconds (after joint_state_broadcaster is up)
    load_gripper_controller = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'gripper_controller'],
                output='screen'
            )
        ]
    )

    # Send initial open position after controllers are active — stops spawn shake
    init_gripper_position = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'action', 'send_goal',
                     '/gripper_controller/gripper_cmd',
                     'control_msgs/action/GripperCommand',
                     '{command: {position: 0.1, max_effort: 0.5}}'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        env_gazebo_model_path,
        env_gazebo_models,
        gzserver,
        gzclient,
        node_robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_gripper_controller,
        init_gripper_position,
    ])