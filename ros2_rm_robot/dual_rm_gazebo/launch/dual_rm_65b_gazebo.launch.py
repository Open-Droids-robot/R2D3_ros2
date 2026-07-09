import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

import xacro

def generate_launch_description():
    # 功能包名称
    package_name = 'dual_rm_gazebo'
    # 机器人模型名称
    robot_name_in_model = 'dual_rm_65b_description'
    # 功能包路径
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    # urdf模型路径
    urdf_model_path = os.path.join(pkg_share, f'config/dual_rm_65b_gazebo.urdf.xacro')
    # world仿真世界路径
    #gazebo_world_path = os.path.join(pkg_share, 'worlds/rm.world')

    print("---", urdf_model_path)
    # 读取并处理URDF文件
    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    # XML内容转换为字符串存放到字典当中
    params = {'robot_description': doc.toxml()}

    # print("urdf", doc.toxml())


    # 启动gazebo
    gazebo =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容
    # 并且会订阅 /joint_states 话题，获取关节的数据，然后发布tf和tf_static话题. {"publish_frequency":15.0}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency":15.0}],
        output='screen'
    )
    # 通过robot_description话题内生成机器人模型在gazebo中
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', f'{robot_name_in_model}',
                                   '-x','0.0',
                                   '-y','0.0',
                                   '-z','0.25',
                                   ], 
                        output='screen')

    # gazebo在加载urdf时，根据urdf的设定，会启动一个joint_states节点
    # 关节状态发布器
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    # 这个arm_controller需要根据urdf文件里面引用的ros2_controllers.yaml里面的名字确定
    # 加载左臂控制器
    load_left_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'left_arm_controller'],
        output='screen'
    )
    # 加载右臂控制器
    load_right_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'right_arm_controller'],
        output='screen'
    )
    # 加载升降机构控制器
    load_platform_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'platform_controller'],
        output='screen'
    )


    # 用下面这两个估计是想控制好各个节点的启动顺序
    # 监听 spawn_entity_cmd，当其退出（完全启动）时，启动load_joint_state_controller
    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
    )
    # 监听 load_joint_state_controller，当其退出（完全启动）时，启动load_joint_trajectory_controller
    # moveit是怎么和gazebo这里提供的action连接起来的
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_left_arm_controller]
            )
    )
    close_evt3 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_right_arm_controller]
            )
    )
    close_evt4 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_platform_controller]
            )
    )
    
    ld = LaunchDescription([
        close_evt1,
        close_evt2,
        close_evt3,
        close_evt4,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])

    return ld