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
   
    servo_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("servo_driver"),
                "launch",
                "servo_start.launch.py"                    
            )
        )
    )
    
    nodes_servo = [ 
        Node( 
            package='servo_example', 
            executable='send_control_cmd_demo', 
            name='send_control_cmd_demo_node', 
            output='screen' 
            ),
        Node( 
            package='servo_example', 
            executable='get_servo_angle_demo', 
            name='get_servo_angle_demo_node', 
            output='screen' 
            )
    ]
    
    nodes_camera = [ 
        Node( 
            package='rm_camera_demo', 
            executable='camera_0_node', 
            name='camera_0_node', 
            output='screen' 
            ),
        Node( 
            package='rm_camera_demo', 
            executable='camera_1_node', 
            name='camera_1_node', 
            output='screen' 
        ),
        Node( 
            package='ros2_total_demo', 
            executable='camera_0.py', 
            name='camera_0_node', 
            output='screen' 
        ),
        Node( 
            package='ros2_total_demo', 
            executable='camera_1.py', 
            name='camera_1_node', 
            output='screen' 
        ),
        Node( 
            package='rm_camera_demo', 
            executable='realsense_camera_0_node', 
            name='realsense_camera_0_node', 
            output='screen' 
            ),
        Node( 
            package='rm_camera_demo', 
            executable='realsense_camera_1_node', 
            name='realsense_camera_1_node', 
            output='screen' 
            ),
        Node( 
            package='rm_camera_demo', 
            executable='realsense_camera_2_node', 
            name='realsense_camera_2_node', 
            output='screen' 
            ),
        Node( 
            package='ros2_total_demo', 
            executable='realsense_camera_0.py', 
            name='realsense_camera_0_node', 
            output='screen' 
            ),
            
        Node( 
            package='ros2_total_demo', 
            executable='realsense_camera_1.py', 
            name='realsense_camera_1_node', 
            output='screen' 
            ),
        Node( 
            package='ros2_total_demo', 
            executable='realsense_camera_2.py', 
            name='realsense_camera_2_node', 
            output='screen' 
            ),
    ]
    
    
    
    return LaunchDescription([
        dual_arm_launch,
        servo_launch
     ]+ nodes_camera)
