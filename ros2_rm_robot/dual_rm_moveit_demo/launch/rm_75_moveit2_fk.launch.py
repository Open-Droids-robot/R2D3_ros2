import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro  # 导入 xacro 库


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # 读取并处理 xacro 文件
    def load_xacro_file(package_name, file_path):
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        
        doc = xacro.parse(open(absolute_file_path))
        xacro.process_doc(doc)
        return doc.toxml()

    # planning_context
    robot_description_config = load_xacro_file(
        "dual_rm_75b_description", "urdf/dual_rm_75b_description.urdf.xacro"
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(
        "dual_rm_75b_moveit_config", "config/dual_rm_75b_description.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "dual_rm_75b_moveit_config", "config/kinematics.yaml"
    )

    arm_dof = DeclareLaunchArgument("arm_dof",default_value = "7")

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="moveit2_fk_demo",
        package="dual_rm_moveit_demo",
        executable="moveit2_fk_demo",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml,
                     {"arm_dof":LaunchConfiguration("arm_dof")}]
    )

    return LaunchDescription([
        arm_dof,
        move_group_demo])