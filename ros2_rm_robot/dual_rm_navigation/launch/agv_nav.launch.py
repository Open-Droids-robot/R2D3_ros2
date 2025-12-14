from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    
    # Get package directories
    pkg_dual_rm_gazebo = get_package_share_directory('dual_rm_gazebo')
    pkg_dual_rm_nav = get_package_share_directory('dual_rm_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Navigation parameters file - modify this path to your actual nav2 params file
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare('dual_rm_navigation'), 'config', 'nav2_params.yaml']
    )
    
    # Launch file paths
    gazebo_launch = PathJoinSubstitution(
        [pkg_dual_rm_gazebo, 'launch', 'dual_rm_65b_gazebo.launch.py'])
    
    slam_launch = PathJoinSubstitution(
        [pkg_dual_rm_nav, 'launch', 'agv_slam.launch.py'])
    
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    
    rviz_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'rviz_launch.py'])
    
    # Include Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch])
    )
    
    # Include SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_launch]),
        launch_arguments=[
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', 'true')
        ]
    )
    
    # Include Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('params_file', nav2_params_file)
        ]
    )
    
    # Include RViz2
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('use_sim_time', 'true')
        ]
    )
    
    return [
        # Launch nodes in order
        gazebo,
        slam,
        nav2,
        rviz
    ]

def generate_launch_description():
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument(
            'localization', 
            default_value='false',
            description='Launch in localization mode (true) or SLAM mode (false).'),
        
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        OpaqueFunction(function=launch_setup)
    ])