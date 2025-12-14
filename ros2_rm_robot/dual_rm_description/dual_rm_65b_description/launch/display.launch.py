import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取URDF文件路径
    urdf_file = PathJoinSubstitution(
        [FindPackageShare('dual_rm_65b_description'), 'urdf', 'dual_rm_65b_description.urdf.xacro']
    )

    # 声明URDF文件路径参数
    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=urdf_file,
        description='Full path to the URDF/Xacro file to load'
    )

    # 使用xacro工具转换URDF文件
    robot_description_cmd = Command(['xacro ', LaunchConfiguration('urdf_file')])

    # 创建robot_state_publisher节点
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_cmd}]
    )

    # 创建joint_state_publisher_gui节点
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # 创建rviz节点
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('dual_rm_65b_description'), 'rviz', 'view.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        declare_urdf_cmd,
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])

