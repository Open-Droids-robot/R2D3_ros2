from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # Packages
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_dual_rm_navigation = get_package_share_directory('dual_rm_navigation')
    
    # Paths
    map_file = PathJoinSubstitution(
        [pkg_dual_rm_navigation, 'maps', 'my_map.yaml'])
    nav2_params = PathJoinSubstitution(
        [pkg_dual_rm_navigation, 'config', 'nav2_params.yaml'])
    rviz_config = PathJoinSubstitution(
        [pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'])
    
    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file
        }]
    )
    
    # Lifecycle Manager for Map Server
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params
        }.items()
    )
    
    # RTAB-Map in Localization Mode
    rtabmap_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dual_rm_navigation, 'launch', 'agv_slam.launch.py'])
        ),
        launch_arguments={
            'localization': 'true',
            'use_sim_time': 'true'
        }.items()
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', 
            default_value=map_file,
            description='Full path to map yaml file'),
        
        map_server,
        lifecycle_manager_map,
        nav2_bringup,
        rtabmap_localization,
        rviz,
    ])