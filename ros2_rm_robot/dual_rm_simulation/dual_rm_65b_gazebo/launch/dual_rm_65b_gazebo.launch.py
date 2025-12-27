import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

import xacro

def generate_launch_description():
    
    # Package name
    package_name = 'dual_rm_65b_gazebo'
    # Robot model name
    robot_name_in_model = 'dual_rm_65b_description'
    # Package path
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    # URDF model path
    urdf_model_path = os.path.join(pkg_share, f'urdf/dual_rm_65b_gazebo.urdf.xacro')
    # Gazebo world file path
    gazebo_world_path = os.path.join(pkg_share, 'worlds/rm.world')

    print("---", urdf_model_path)
    # Read and process URDF file
    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)

    params = {'robot_description': doc.toxml()}

    # Default RViz configuration file path
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')


    # Launch Gazebo simulation environment
    gazebo =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # Robot state publisher node - publishes robot TF transforms and joint states
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {'publish_frequency':15.0}],
        output='screen'
    )
    
    # Spawn robot entity in Gazebo from robot_description topic
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', f'{robot_name_in_model}',
                                   '-x','0.0',
                                   '-y','0.0',
                                   '-z','0.25',
                                   ], 
                        output='screen')
    
    # Robot localization node using EKF
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': True}]
    )

    # RViz2 visualization node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}],
    )
    
    # Load joint state broadcaster controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Load left arm controller
    load_left_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'left_arm_controller'],
        output='screen'
    )
    
    # Load right arm controller
    load_right_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'right_arm_controller'],
        output='screen'
    )
    
    # Load left hand controller
    load_left_hand_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'left_hand_controller'],
        output='screen'
    )
    
    # Load right hand controller
    load_right_hand_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'right_hand_controller'],
        output='screen'
    )
    
    # Load platform controller
    load_platform_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'platform_controller'],
        output='screen'
    )
    
    # Load differential drive controller (this will be the last controller to load)
    load_diff_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'diff_controller'],
        output='screen'
    )


    # Event handlers to control node startup sequence
    # Listen for spawn_entity completion, then start joint_state_broadcaster
    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
    )
    
    # Listen for joint_state_broadcaster completion, then start left_arm_controller
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_left_arm_controller]
            )
    )
    
    # Listen for joint_state_broadcaster completion, then start right_arm_controller
    close_evt3 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_right_arm_controller]
            )
    )
    
        # Listen for joint_state_broadcaster completion, then start left_hand_controller
    close_evt4 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_left_hand_controller]
            )
    )
    
    # Listen for joint_state_broadcaster completion, then start right_hand_controller
    close_evt5 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_right_hand_controller]
            )
    )
    
    # Listen for joint_state_broadcaster completion, then start platform_controller
    close_evt6 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_platform_controller]
            )
    )
    
    # Listen for joint_state_broadcaster completion, then start diff_controller
    close_evt7 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_controller]
            )
    )
    
    # Listen for diff_controller completion, then start robot_localization_node
    close_evt8 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_diff_controller,
                on_exit=[robot_localization_node]
            )
    )
    
    # Listen for robot_localization_node, then start RViz2
    close_evt9 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_localization_node,
                on_exit=[rviz_node]
            )
    )
    
    # Create launch description with all nodes and event handlers
    ld = LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        close_evt1,
        close_evt2,
        close_evt3,
        close_evt4,
        close_evt5,
        close_evt6,
        close_evt7,
        close_evt8,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])

    return ld
