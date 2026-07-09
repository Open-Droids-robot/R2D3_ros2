from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription  
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from ament_index_python.packages import get_package_share_directory 
import os

def generate_launch_description():


    dual_arm_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("rm_driver"),
                "launch",
                "dual_rm_65_driver.launch.py"                    
            )
        )
    )
    nodes_catch = [ 
    
        Node(
            package='ros2_total_demo',
            executable='detect_object.py',
            name='model_detect',
            output='screen'
        )
    ]
    return LaunchDescription([
    dual_arm_launch
    ]+nodes_catch)
