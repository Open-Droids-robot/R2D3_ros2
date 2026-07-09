from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    left_control_node = Node(
    package='dual_rm_control', #节点所在的功能包
    executable='dual_rm_control', #表示要运行的可执行文件名或脚本名字.py
    parameters= [
                    {'follow': True},
                    {'arm_type': 65}
                ],             #接入参数文件
    output='screen', #用于将话题信息打印到屏幕
    namespace='left_arm_controller'
    )
    
    right_control_node = Node(
    package='dual_rm_control', #节点所在的功能包
    executable='dual_rm_control', #表示要运行的可执行文件名或脚本名字.py
    parameters= [
                    {'follow': True},
                    {'arm_type': 65}
                ],             #接入参数文件
    output='screen', #用于将话题信息打印到屏幕
    namespace='right_arm_controller'
    )

    ld.add_action(left_control_node)
    ld.add_action(right_control_node)
    return ld

