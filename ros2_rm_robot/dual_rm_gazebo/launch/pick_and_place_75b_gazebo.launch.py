import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    # Package and robot names
    package_name = 'dual_rm_gazebo'
    robot_name_in_model = 'dual_rm_75b_description'
    
    # Find package share directory
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    
    # URDF model path
    urdf_model_path = os.path.join(pkg_share, 'config/dual_rm_75b_gazebo.urdf.xacro')

    print("--- Loading URDF: ", urdf_model_path)
    
    # Parse and process URDF file
    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Robot State Publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency": 15.0}],
        output='screen'
    )

    # Spawn the robot entity
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', f'{robot_name_in_model}',
            '-x','0.0', '-y','0.0', '-z','0.25',
        ], 
        output='screen'
    )
    
    table_path = os.path.join(pkg_share, 'models', 'table.sdf')
    cube_path = os.path.join(pkg_share, 'models', 'cube.sdf')

    # Spawn the table 
    spawn_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_table', '-file', table_path, '-x', '0.6', '-y', '-1.0', '-z', '0.0'],
        output='screen'
    )

    # Spawn the target cube 
    spawn_cube = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'target_cube', '-file', cube_path, '-x', '0.0', '-y', '-1.0', '-z', '0.61'],
        output='screen'
    )

    # Controller Loading
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_left_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'left_arm_controller'],
        output='screen'
    )
    load_right_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'right_arm_controller'],
        output='screen'
    )
    load_platform_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'platform_controller'],
        output='screen'
    )
    
    # Added the AGV controller to the automated sequence
    load_agv_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'agv_controller'],
        output='screen'
    )

    # Added Gripper controllers
    load_l_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'l_gripper_controller'],
        output='screen'
    )
    
    load_r_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'r_gripper_controller'],
        output='screen'
    )

    # Event Handlers for Boot Sequence 
    # Start joint state broadcaster after robot spawns
    close_evt1 = RegisterEventHandler( 
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )
    
    # Start all other controllers after joint state broadcaster is ready
    close_evt2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[
                load_left_arm_controller,
                load_right_arm_controller,
                load_platform_controller,
                load_agv_controller,
                load_l_gripper_controller, 
                load_r_gripper_controller
            ]
        )
    )
    
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        spawn_table,
        spawn_cube,
        close_evt1,
        close_evt2
    ])