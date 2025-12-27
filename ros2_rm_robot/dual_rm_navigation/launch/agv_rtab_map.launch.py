from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    parameters = [{
        'frame_id': 'base_footprint',     # robot_base
        'odom_frame_id': 'odom',          # EKF odom
        'map_frame_id': 'map',

        'publish_tf': False,               
        'subscribe_depth': True,
        'subscribe_rgbd': False,
        'subscribe_odom_info': False,

        'approx_sync': True,
        'wait_imu_to_init': True,
        'use_sim_time': use_sim_time,

        # 2D SLAM
        'Reg/Force3DoF': 'true',
        'Optimizer/Slam2D': 'true',

        # ICP (depth camera)
        'Reg/Strategy': '1',               # ICP
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1'
    }]


    remappings=[
          ('imu', '/camera/imu'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_rect_raw'),  
          ('odom', '/odometry/filtered')]

    return LaunchDescription([
        
        # Declare use_sim_time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Set RTABMAP database deletion argument
        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='true',
            description='Delete database on start'
        ),

        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),  # -d deletes previous database

        # RTAB-Map visualization
        Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=parameters,
            remappings=remappings),
    ])