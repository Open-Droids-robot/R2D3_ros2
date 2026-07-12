"""
MuJoCo simulation launch for the R2D3 dual-arm mobile robot.
Mirrors dual_rm_simulation/launch/gz_sim.launch.py.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_mujoco = get_package_share_directory("r2d3_mujoco")

    robot_model = LaunchConfiguration("robot_model").perform(context)
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    force_recompile = LaunchConfiguration("force_recompile").perform(context)

    xacro_path = os.path.join(pkg_mujoco, "urdf", "r2d3_mujoco.urdf.xacro")
    robot_description_str = Command([
        FindExecutable(name="xacro"), " ", xacro_path,
        " arm_model:=", robot_model,
        " headless:=", headless,
    ]).perform(context)
    robot_description = {
        "robot_description": ParameterValue(robot_description_str, value_type=str)
    }

    controllers_yaml = os.path.join(pkg_mujoco, "config", f"controllers_{robot_model}.yaml")

    # -- MJCF provider: cached conversion (publishes /mujoco_robot_description) --
    ensure_mjcf_args = [
        "--robot-description", robot_description_str,
        "--world", world,
        "--model", robot_model,
        "--topic", "/mujoco_robot_description",
    ]
    if force_recompile == "true":
        ensure_mjcf_args.append("--force")
    ensure_mjcf = Node(
        package="r2d3_mujoco",
        executable="ensure_mjcf.py",
        name="ensure_mjcf",
        output="both",
        emulate_tty=True,
        arguments=ensure_mjcf_args,
    )

    # -- Robot State Publisher --
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # -- MuJoCo ros2_control node (physics + controller_manager + sensors) --
    remappings = [("/imu_sensor_broadcaster/imu", "/imu")]
    if os.environ.get("ROS_DISTRO") == "humble":
        remappings.append(("~/robot_description", "/robot_description"))
    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}, ParameterFile(controllers_yaml)],
        remappings=remappings,
        on_exit=Shutdown(),
    )

    # -- Controller spawners: JSB first, everything else after it exits --
    def spawner(controller):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--param-file", controllers_yaml],
            output="screen",
        )

    jsb_spawner = spawner("joint_state_broadcaster")
    evt_jsb_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[
                spawner("diff_drive_controller"),
                spawner("left_arm_controller"),
                spawner("right_arm_controller"),
                spawner("platform_controller"),
                spawner("imu_sensor_broadcaster"),
            ],
        )
    )

    # -- /camera/points from depth + camera_info (composable: works on Humble too) --
    pointcloud_container = ComposableNodeContainer(
        name="camera_points_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="point_cloud_xyzrgb",
                parameters=[{"use_sim_time": True}],
                remappings=[
                    ("rgb/camera_info", "/camera/camera_info"),
                    ("rgb/image_rect_color", "/camera/image"),
                    ("depth_registered/image_rect", "/camera/depth_image"),
                    ("points", "/camera/points"),
                ],
            ),
        ],
        output="screen",
    )

    return [
        ensure_mjcf,
        robot_state_publisher,
        control_node,
        jsb_spawner,
        evt_jsb_done,
        pointcloud_container,
    ]


def generate_launch_description():
    pkg_mujoco = get_package_share_directory("r2d3_mujoco")
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_model", default_value="65b",
            description="Robot model variant: 65b or 75b"),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg_mujoco, "worlds", "nav_empty.xml"),
            description="Full path to the MuJoCo scene XML"),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run MuJoCo without the Simulate window"),
        DeclareLaunchArgument(
            "force_recompile", default_value="false",
            description="Force URDF->MJCF recompilation even if cached"),
        OpaqueFunction(function=launch_setup),
    ])
