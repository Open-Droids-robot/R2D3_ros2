from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            remappings=[
                ('cmd_vel', '/diff_drive_controller/cmd_vel'),
            ],
            output='screen',
            prefix='xterm -e',  
        ),
    ])
