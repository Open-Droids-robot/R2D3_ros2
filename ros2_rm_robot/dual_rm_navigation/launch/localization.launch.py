import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav = get_package_share_directory('dual_rm_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
    )
    declare_map = DeclareLaunchArgument(
        'map',
        description='Full path to map YAML file',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
    )

    # Include Nav2's localization launch (AMCL + map_server)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'map': LaunchConfiguration('map'),
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_map,
        declare_autostart,
        localization,
    ])
