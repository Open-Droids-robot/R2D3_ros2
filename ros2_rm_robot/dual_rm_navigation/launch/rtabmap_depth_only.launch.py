"""
RTAB-Map SLAM launch — depth camera only (no LiDAR).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _launch_rtabmap(context):
    pkg_nav = get_package_share_directory('dual_rm_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    localization = LaunchConfiguration('localization').perform(context).lower()

    # IncrementalMemory: true = SLAM (build map), false = localization (replay)
    incremental_memory = 'false' if localization == 'true' else 'true'

    # Topic remappings: ZED (sim or real wrapper) → RTAB-Map expected names.
    # Pinned to the LEFT eye: depth is registered to the left eye, so image,
    # camera_info and depth all share zed_left_camera_frame_optical. Never
    # feed RTAB-Map the rgb/ alias or the double-width stereo/ image.
    remappings = [
        ('rgb/image', '/zed/zed_node/left/image_rect_color'),
        ('rgb/camera_info', '/zed/zed_node/left/camera_info'),
        ('depth/image', '/zed/zed_node/depth/depth_registered'),
        ('odom', '/diff_drive_controller/odom'),
    ]

    rtabmap_args = ['-d'] if localization != 'true' else []

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time == 'true',
                'Mem/IncrementalMemory': incremental_memory,
                'Mem/InitWMWithAllNodes': 'true' if localization == 'true' else 'false',
            },
        ],
        remappings=remappings,
        arguments=rtabmap_args,
    )

    return [rtabmap_slam]


def generate_launch_description():
    pkg_nav = get_package_share_directory('dual_rm_navigation')

    # ── Arguments ─────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'rtabmap_depth_only_params.yaml'),
        description='Full path to RTAB-Map depth-only parameters YAML',
    )
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='true = localization mode (load existing map), false = SLAM mode',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_localization,

        OpaqueFunction(function=_launch_rtabmap),
    ])
