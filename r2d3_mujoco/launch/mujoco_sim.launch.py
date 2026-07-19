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
        on_exit=Shutdown(),
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
                spawner("neck_controller"),
                spawner("imu_sensor_broadcaster"),
            ],
        )
    )

    # -- Neck servo bridge: real servo contract -> /neck_controller/commands --
    neck_servo_bridge = Node(
        package="servo_sim_bridge",
        executable="neck_servo_bridge",
        name="neck_servo_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # -- Sim-only ZED shim: side-by-side stereo + rgb alias --
    stereo_concat = Node(
        package="dual_rm_simulation",
        executable="stereo_concat.py",
        name="stereo_concat",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # -- /zed/zed_node/point_cloud/cloud_registered from left depth + info --
    pointcloud_container = ComposableNodeContainer(
        name="zed_points_container",
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
                    ("rgb/camera_info", "/zed/zed_node/left/color/rect/camera_info"),
                    ("rgb/image_rect_color", "/zed/zed_node/left/color/rect/image"),
                    ("depth_registered/image_rect", "/zed/zed_node/depth/depth_registered"),
                    ("points", "/zed/zed_node/point_cloud/cloud_registered"),
                ],
            ),
        ],
        output="screen",
    )

    # -- /{side}_wrist/depth/color/points from each wrist D435 --
    # Gz publishes `points` natively from its rgbd_camera; MuJoCo does not, so
    # depth_image_proc completes the same topic contract on this sim.
    def _wrist_points(side):
        return ComposableNodeContainer(
            name=f"{side}_wrist_points_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzrgbNode",
                    name="point_cloud_xyzrgb",
                    # MuJoCo does not stamp colour and depth bit-identically (60-134ms
                    # offsets observed), which starves depth_image_proc's default
                    # exact-time synchroniser to ~85-90% frame loss. Use approximate
                    # sync with a deep enough queue (~3s at ~10Hz) to absorb the jitter.
                    parameters=[{
                        "use_sim_time": True,
                        "approximate_sync": True,
                        "queue_size": 30,
                    }],
                    remappings=[
                        ("rgb/camera_info", f"/{side}_wrist/color/camera_info"),
                        ("rgb/image_rect_color", f"/{side}_wrist/color/image_raw"),
                        ("depth_registered/image_rect", f"/{side}_wrist/depth/image_rect_raw"),
                        ("points", f"/{side}_wrist/depth/color/points"),
                    ],
                ),
            ],
            output="screen",
        )

    left_wrist_points = _wrist_points("left")
    right_wrist_points = _wrist_points("right")

    return [
        ensure_mjcf,
        robot_state_publisher,
        control_node,
        jsb_spawner,
        evt_jsb_done,
        neck_servo_bridge,
        stereo_concat,
        pointcloud_container,
        left_wrist_points,
        right_wrist_points,
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
