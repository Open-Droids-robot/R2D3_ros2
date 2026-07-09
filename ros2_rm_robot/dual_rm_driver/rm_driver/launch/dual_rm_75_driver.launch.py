import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    left_arm_config = os.path.join(get_package_share_directory('rm_driver'), 'config', 'dual_75_left_config.yaml')
    right_arm_config = os.path.join(get_package_share_directory('rm_driver'), 'config', 'dual_75_right_config.yaml')

    with open(left_arm_config, 'r') as f:
        left_arm_params = yaml.safe_load(f)["left_arm_controller/rm_driver"]["ros__parameters"]

    with open(right_arm_config, 'r') as f:
        right_arm_params = yaml.safe_load(f)["right_arm_controller/rm_driver"]["ros__parameters"]

    left_arm_driver = Node(
        package="rm_driver",
        executable="rm_driver",
        parameters=[left_arm_config],
        output="screen",
        namespace='left_arm_controller'
    )

    right_arm_driver = Node(
        package="rm_driver",
        executable="rm_driver",
        parameters=[right_arm_config],
        output="screen",
        namespace='right_arm_controller'
    )

    ld.add_action(left_arm_driver)
    ld.add_action(right_arm_driver)

    return ld

