"""r2d3_humble_bridge node — runs in Python 3.11 (ros_humble env).

Translates between the custom rm_ros_interfaces / r2d3_model_interfaces
messages that r2d3_model expects and the standard-msg surface Isaac Sim
exposes on `/r2d3/sim/*`.

Architecture
------------

    r2d3_model (Movej/Gripperset/Liftheight)
            │
            ▼ bridge subscribes
        BRIDGE
            │ bridge publishes
            ▼
    isaac sim_adapter (JointState/Float64 cmds)
                                  │ physics
                                  ▼
                            sim_adapter publishes state
            ▲ bridge subscribes
        BRIDGE
            │ bridge publishes Armstate/Liftstate + aggregated Observation
            ▼
    r2d3_model (Observation)
"""
from __future__ import annotations

import threading
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float64

from r2d3_model_interfaces.msg import Observation
from rm_ros_interfaces.msg import (
    Armstate,
    Gripperset,
    Liftheight,
    Liftstate,
    Movej,
)

from . import topics as t


PUBLISH_RATE_HZ = 10.0


class HumbleBridge(Node):
    def __init__(self) -> None:
        super().__init__("r2d3_humble_bridge")
        self._lock = threading.Lock()

        # Cached state — Isaac → bridge.
        self._latest_joint_state: Optional[JointState] = None
        self._latest_lift_m: Optional[float] = None
        self._sim_ready: bool = False
        self._latest_color: Optional[Image] = None
        self._latest_depth: Optional[Image] = None
        self._latest_caminfo: Optional[CameraInfo] = None
        self._latest_wrench_l: Optional[WrenchStamped] = None
        self._latest_wrench_r: Optional[WrenchStamped] = None

        qos = QoSProfile(depth=10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # --- Bridge → Isaac (cmd publishers) ---------------------------------
        self._pub_sim_left_arm  = self.create_publisher(JointState, t.SIM_CMD_LEFT_ARM, qos)
        self._pub_sim_right_arm = self.create_publisher(JointState, t.SIM_CMD_RIGHT_ARM, qos)
        self._pub_sim_lift      = self.create_publisher(Float64, t.SIM_CMD_LIFT, qos)
        self._pub_sim_lfinger   = self.create_publisher(Float64, t.SIM_CMD_LEFT_FINGER, qos)
        self._pub_sim_rfinger   = self.create_publisher(Float64, t.SIM_CMD_RIGHT_FINGER, qos)

        # --- Bridge → r2d3_model (state publishers) --------------------------
        self._pub_armstate_left  = self.create_publisher(Armstate, t.ARMSTATE_LEFT, qos)
        self._pub_armstate_right = self.create_publisher(Armstate, t.ARMSTATE_RIGHT, qos)
        self._pub_liftstate      = self.create_publisher(Liftstate, t.LIFTSTATE, qos)
        self._pub_observations   = self.create_publisher(Observation, t.OBSERVATIONS, qos)

        # --- r2d3_model → bridge (command subscribers) -----------------------
        self.create_subscription(Movej, t.MOVEJ_LEFT, self._on_movej_left, qos)
        self.create_subscription(Movej, t.MOVEJ_RIGHT, self._on_movej_right, qos)
        self.create_subscription(Gripperset, t.GRIPPERSET_LEFT, self._on_gripper_left, qos)
        self.create_subscription(Gripperset, t.GRIPPERSET_RIGHT, self._on_gripper_right, qos)
        self.create_subscription(Liftheight, t.LIFT_HEIGHT, self._on_lift_cmd, qos)

        # --- Isaac → bridge (state subscribers) ------------------------------
        self.create_subscription(JointState, t.SIM_JOINT_STATES, self._on_joint_states, qos)
        self.create_subscription(Float64, t.SIM_LIFT_STATE, self._on_lift_state, qos)
        self.create_subscription(Bool, t.SIM_READY, self._on_sim_ready, latched)
        self.create_subscription(WrenchStamped, t.SIM_WRENCH_LEFT, self._on_wrench_left, qos)
        self.create_subscription(WrenchStamped, t.SIM_WRENCH_RIGHT, self._on_wrench_right, qos)

        # --- Head D435 (sensor-data QoS = best-effort, for image streams) ----
        self.create_subscription(Image, t.CAMERA_COLOR_IMAGE, self._on_color,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, t.CAMERA_DEPTH_IMAGE, self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, t.CAMERA_COLOR_INFO, self._on_caminfo,
                                 qos_profile_sensor_data)

        # --- Aggregator timer ------------------------------------------------
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_state)

        self.get_logger().info("HumbleBridge up. Waiting for Isaac /r2d3/sim/ready.")

    # ============================================================ command in
    def _on_movej_left(self, msg: Movej) -> None:
        self._publish_arm_cmd(self._pub_sim_left_arm, t.LEFT_ARM_JOINTS, msg.joint)

    def _on_movej_right(self, msg: Movej) -> None:
        self._publish_arm_cmd(self._pub_sim_right_arm, t.RIGHT_ARM_JOINTS, msg.joint)

    def _publish_arm_cmd(self, pub, joint_names: List[str], values) -> None:
        if len(values) != len(joint_names):
            self.get_logger().warn(
                f"Movej dropped: expected {len(joint_names)} joints, got {len(values)}"
            )
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(joint_names)
        js.position = [float(v) for v in values]
        pub.publish(js)

    def _on_gripper_left(self, msg: Gripperset) -> None:
        self._pub_sim_lfinger.publish(Float64(data=_gripper_to_m(msg.position)))

    def _on_gripper_right(self, msg: Gripperset) -> None:
        self._pub_sim_rfinger.publish(Float64(data=_gripper_to_m(msg.position)))

    def _on_lift_cmd(self, msg: Liftheight) -> None:
        h_m = min(t.LIFT_MAX_M, max(0.0, msg.height / 1000.0))
        self._pub_sim_lift.publish(Float64(data=h_m))

    # ============================================================ state in
    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            self._latest_joint_state = msg

    def _on_lift_state(self, msg: Float64) -> None:
        with self._lock:
            self._latest_lift_m = float(msg.data)

    def _on_sim_ready(self, msg: Bool) -> None:
        if msg.data and not self._sim_ready:
            self._sim_ready = True
            self.get_logger().info("Isaac sim_adapter reports ready.")

    def _on_color(self, msg: Image) -> None:
        with self._lock:
            self._latest_color = msg

    def _on_depth(self, msg: Image) -> None:
        with self._lock:
            self._latest_depth = msg

    def _on_caminfo(self, msg: CameraInfo) -> None:
        with self._lock:
            self._latest_caminfo = msg

    def _on_wrench_left(self, msg: WrenchStamped) -> None:
        with self._lock:
            self._latest_wrench_l = msg

    def _on_wrench_right(self, msg: WrenchStamped) -> None:
        with self._lock:
            self._latest_wrench_r = msg

    # ============================================================ aggregator
    def _publish_state(self) -> None:
        with self._lock:
            js = self._latest_joint_state
            lift_m = self._latest_lift_m
            color = self._latest_color
            depth = self._latest_depth
            caminfo = self._latest_caminfo
            wrench_l = self._latest_wrench_l
            wrench_r = self._latest_wrench_r

        if js is None:
            return  # haven't heard from Isaac yet

        name_to_pos: Dict[str, float] = dict(zip(js.name, js.position))
        now = self.get_clock().now().to_msg()

        # ---- per-arm Armstate ------------------------------------------------
        left_arm  = self._make_armstate(now, name_to_pos, t.LEFT_ARM_JOINTS)
        right_arm = self._make_armstate(now, name_to_pos, t.RIGHT_ARM_JOINTS)
        self._pub_armstate_left.publish(left_arm)
        self._pub_armstate_right.publish(right_arm)

        # ---- Liftstate -------------------------------------------------------
        lift_msg = Liftstate()
        lift_msg.height = int((lift_m or 0.0) * 1000)   # meters → mm
        lift_msg.current = 0
        lift_msg.err_flag = 0
        lift_msg.mode = 0
        self._pub_liftstate.publish(lift_msg)

        # ---- Aggregated Observation -----------------------------------------
        obs = Observation()
        obs.header.stamp = now
        obs.header.frame_id = "base_link_underpan"
        # Head D435 — latest frames from Isaac's OmniGraph publishers (empty
        # until the first camera frame arrives).
        obs.head_color = color if color is not None else Image()
        obs.head_depth = depth if depth is not None else Image()
        obs.head_camera_info = caminfo if caminfo is not None else CameraInfo()
        obs.joint_states = js
        obs.left_arm_state = left_arm
        obs.right_arm_state = right_arm
        # Wrist 6-axis force-torque (best-effort under the dynamics workaround).
        obs.left_wrench = wrench_l if wrench_l is not None else WrenchStamped()
        obs.right_wrench = wrench_r if wrench_r is not None else WrenchStamped()
        obs.lift_state = lift_msg
        self._pub_observations.publish(obs)

    @staticmethod
    def _make_armstate(stamp, name_to_pos: Dict[str, float], joints: List[str]) -> Armstate:
        msg = Armstate()
        msg.joint = [float(name_to_pos.get(n, 0.0)) for n in joints]
        msg.dof = len(joints)
        msg.arm_err = 0
        msg.sys_err = 0
        # pose left zero — Cartesian EE pose comes via the dedicated PoseStamped
        # topics, not here. Filling Pose requires FK we don't run on this side.
        return msg


def _gripper_to_m(position: int) -> float:
    """Map Gripperset.position [1, 1000] → finger drive distance [0, 0.035] m."""
    p = max(0, min(t.GRIPPER_POSITION_MAX, int(position)))
    return p / t.GRIPPER_POSITION_MAX * t.FINGER_DRIVE_MAX_M


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HumbleBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
