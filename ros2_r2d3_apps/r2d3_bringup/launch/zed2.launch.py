"""Real-robot ZED 2 bringup.

Requires the vendored wrapper to be enabled (robot only):
    rm ros2_zed/zed-ros2-wrapper/COLCON_IGNORE && colcon build --packages-up-to zed_wrapper

publish_tf/publish_urdf are false: robot_state_publisher owns the ZED frames
via the zed2 macro in dual_rm_description; zed_node must not double-publish.
The resulting topics/frames are byte-for-byte the sim contract
(/zed/zed_node/..., zed_left_camera_frame_optical, ...), so RTAB-Map & co.
run unchanged against sim or hardware.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('r2d3_bringup')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')  # robot only

    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': 'zed2',
            'camera_name': 'zed',
            'publish_tf': 'false',
            'publish_urdf': 'false',
            'ros_params_override_path':
                os.path.join(bringup_dir, 'config', 'zed2_params.yaml'),
        }.items(),
    )

    return LaunchDescription([zed_camera])
