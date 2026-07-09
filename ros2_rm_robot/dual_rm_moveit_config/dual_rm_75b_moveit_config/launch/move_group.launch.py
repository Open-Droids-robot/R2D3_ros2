from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load configurations and strictly isolate the pipeline to OMPL
    moveit_config = (
        MoveItConfigsBuilder("dual_rm_75b_description", package_name="dual_rm_75b_moveit_config")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"]) # <--- NUKES CHOMP
        .to_moveit_configs()
    )

    # Explicitly define the MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}, 
        ],
    )

    return LaunchDescription([move_group_node])