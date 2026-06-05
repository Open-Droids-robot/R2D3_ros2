"""rclpy node living inside the Isaac process.

Publishes joint state + EE poses + ready signal on the ``/r2d3/sim/*``
namespace (standard ROS messages only — the bundled rclpy in Isaac
Sim 6.0 has these built in for Python 3.12). Subscribes target
JointStates and Float64 commands and writes them into a thread-safe
``LatestCommandCache`` that the sim loop drains once per physics step.

Wiring this directly to ``rm_ros_interfaces`` is impossible from
Python 3.12: the custom message bindings only exist for Python 3.11
(the RoboStack ros_humble env). The ``r2d3_humble_bridge`` package
(Py 3.11) does that translation upstream of this node.

Threading model
---------------

This node spins on a ``MultiThreadedExecutor`` in a background thread
started by ``bring_up.py``. All subscriber callbacks write to the
LatestCommandCache under a mutex. The sim-loop thread is the *only*
reader of the cache and the *only* writer of articulation targets.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from . import sim_topics as t


# ---------------------------------------------------------------------------
# LatestCommandCache — written by ROS callbacks, drained by the sim loop.
# ---------------------------------------------------------------------------
@dataclass
class LatestCommandCache:
    """Latest-known-good commands. The sim loop reads, then clears the dirty
    flag; subsequent reads return the same value until a new command arrives.

    All fields are optional — `None` means "no command received yet, sim loop
    should leave that DOF alone."
    """
    lock: threading.Lock = field(default_factory=threading.Lock)

    left_arm_targets:   Optional[List[float]] = None    # length 7, radians
    right_arm_targets:  Optional[List[float]] = None
    head_targets:       Optional[List[float]] = None    # length 2: pan, tilt
    lift_target_m:      Optional[float] = None
    left_finger_m:      Optional[float] = None          # [0, FINGER_DRIVE_MAX_M]
    right_finger_m:     Optional[float] = None


# ---------------------------------------------------------------------------
# SimAdapter — the rclpy node.
# ---------------------------------------------------------------------------
class SimAdapter(Node):
    """rclpy node that owns the Isaac↔ROS standard-msg surface."""

    PUBLISH_RATE_HZ = 30.0

    def __init__(self, cache: LatestCommandCache, joint_names: List[str]) -> None:
        super().__init__("r2d3_sim_adapter")
        self._cache = cache
        self._joint_names = list(joint_names)

        # State publishers (sim → world). Best-effort QoS keeps last messages
        # but won't block the sim loop on a slow subscriber.
        state_qos = QoSProfile(depth=10)
        self._joint_states_pub = self.create_publisher(JointState, t.JOINT_STATES, state_qos)
        self._lift_state_pub   = self.create_publisher(Float64, t.LIFT_STATE, state_qos)
        self._ee_left_pub      = self.create_publisher(PoseStamped, t.EE_POSE_LEFT, state_qos)
        self._ee_right_pub     = self.create_publisher(PoseStamped, t.EE_POSE_RIGHT, state_qos)
        self._wrench_left_pub  = self.create_publisher(WrenchStamped, t.WRENCH_LEFT, state_qos)
        self._wrench_right_pub = self.create_publisher(WrenchStamped, t.WRENCH_RIGHT, state_qos)

        # Ready signal — latched (transient_local) so late subscribers see it.
        ready_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready_pub = self.create_publisher(Bool, t.READY, ready_qos)
        self._ready_msg = Bool(data=False)
        self._ready_pub.publish(self._ready_msg)

        # Command subscribers. Callbacks write into the cache.
        self.create_subscription(JointState, t.CMD_LEFT_ARM,    self._on_cmd_left_arm,   10)
        self.create_subscription(JointState, t.CMD_RIGHT_ARM,   self._on_cmd_right_arm,  10)
        self.create_subscription(JointState, t.CMD_HEAD,        self._on_cmd_head,       10)
        self.create_subscription(Float64,    t.CMD_LIFT,        self._on_cmd_lift,       10)
        self.create_subscription(Float64,    t.CMD_LEFT_FINGER, self._on_cmd_left_finger, 10)
        self.create_subscription(Float64,    t.CMD_RIGHT_FINGER, self._on_cmd_right_finger, 10)

        # State snapshot is populated by the sim loop before each timer fire.
        self._state_snapshot: Optional[Dict[str, float]] = None
        self._ee_left_snapshot: Optional[tuple] = None    # (xyz, quat_xyzw)
        self._ee_right_snapshot: Optional[tuple] = None
        self._wrench_left_snapshot = None                 # 6-vec [fx,fy,fz,tx,ty,tz]
        self._wrench_right_snapshot = None
        self._state_lock = threading.Lock()

        # Publishing timer — fires PUBLISH_RATE_HZ. The actual publish reads
        # the latest snapshot the sim loop posted.
        self.create_timer(1.0 / self.PUBLISH_RATE_HZ, self._publish_state_tick)

    # ----------------------------------------------------------- ready signal
    def mark_ready(self) -> None:
        """Flip /r2d3/sim/ready to True. Called by bring_up after first step."""
        if not self._ready_msg.data:
            self._ready_msg.data = True
            self._ready_pub.publish(self._ready_msg)

    # ---------------------------------------------------------- state ingress
    def update_state(
        self,
        joint_positions: Dict[str, float],
        ee_left: Optional[tuple] = None,
        ee_right: Optional[tuple] = None,
        wrench_left=None,
        wrench_right=None,
    ) -> None:
        """Called by the sim loop after each physics step.

        ee_left / ee_right are (xyz_array, quat_xyzw_array) tuples or None.
        wrench_left / wrench_right are 6-vecs [fx,fy,fz,tx,ty,tz] or None.
        """
        with self._state_lock:
            self._state_snapshot = joint_positions
            self._ee_left_snapshot = ee_left
            self._ee_right_snapshot = ee_right
            if wrench_left is not None:
                self._wrench_left_snapshot = wrench_left
            if wrench_right is not None:
                self._wrench_right_snapshot = wrench_right

    # ---------------------------------------------------------- state publish
    def _publish_state_tick(self) -> None:
        with self._state_lock:
            snap = self._state_snapshot
            ee_l = self._ee_left_snapshot
            ee_r = self._ee_right_snapshot
            wr_l = self._wrench_left_snapshot
            wr_r = self._wrench_right_snapshot
        if snap is None:
            return

        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.header.frame_id = t.BASE_FRAME
        js.name = list(self._joint_names)
        js.position = [snap.get(n, 0.0) for n in self._joint_names]
        self._joint_states_pub.publish(js)

        lift = Float64()
        lift.data = float(snap.get(t.LIFT_JOINT, 0.0))
        self._lift_state_pub.publish(lift)

        if ee_l is not None:
            self._ee_left_pub.publish(self._build_pose(now, t.LEFT_EE_FRAME, *ee_l))
        if ee_r is not None:
            self._ee_right_pub.publish(self._build_pose(now, t.RIGHT_EE_FRAME, *ee_r))

        if wr_l is not None:
            self._wrench_left_pub.publish(self._build_wrench(now, t.LEFT_EE_FRAME, wr_l))
        if wr_r is not None:
            self._wrench_right_pub.publish(self._build_wrench(now, t.RIGHT_EE_FRAME, wr_r))

    @staticmethod
    def _build_wrench(stamp, frame_id, w) -> WrenchStamped:
        ws = WrenchStamped()
        ws.header.stamp = stamp
        ws.header.frame_id = frame_id
        ws.wrench.force.x, ws.wrench.force.y, ws.wrench.force.z = (
            float(w[0]), float(w[1]), float(w[2])
        )
        ws.wrench.torque.x, ws.wrench.torque.y, ws.wrench.torque.z = (
            float(w[3]), float(w[4]), float(w[5])
        )
        return ws

    @staticmethod
    def _build_pose(stamp, frame_id, xyz, quat_xyzw) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = t.BASE_FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
            float(xyz[0]), float(xyz[1]), float(xyz[2])
        )
        # ROS quaternion order is (x, y, z, w). Isaac returns (w, x, y, z) by
        # default, so callers must convert before passing in. We document
        # quat_xyzw to make that obligation explicit.
        ps.pose.orientation.x = float(quat_xyzw[0])
        ps.pose.orientation.y = float(quat_xyzw[1])
        ps.pose.orientation.z = float(quat_xyzw[2])
        ps.pose.orientation.w = float(quat_xyzw[3])
        return ps

    # ---------------------------------------------------------- command subs
    def _on_cmd_left_arm(self, msg: JointState) -> None:
        positions = _select(msg, t.LEFT_ARM_JOINTS)
        if positions is None:
            self.get_logger().warn(
                f"cmd/left_arm dropped: expected joints {t.LEFT_ARM_JOINTS}, "
                f"got name={list(msg.name)}"
            )
            return
        with self._cache.lock:
            self._cache.left_arm_targets = positions

    def _on_cmd_right_arm(self, msg: JointState) -> None:
        positions = _select(msg, t.RIGHT_ARM_JOINTS)
        if positions is None:
            self.get_logger().warn("cmd/right_arm dropped: joint name mismatch")
            return
        with self._cache.lock:
            self._cache.right_arm_targets = positions

    def _on_cmd_head(self, msg: JointState) -> None:
        positions = _select(msg, t.HEAD_JOINTS)
        if positions is None:
            return
        with self._cache.lock:
            self._cache.head_targets = positions

    def _on_cmd_lift(self, msg: Float64) -> None:
        with self._cache.lock:
            self._cache.lift_target_m = float(msg.data)

    def _on_cmd_left_finger(self, msg: Float64) -> None:
        with self._cache.lock:
            self._cache.left_finger_m = float(msg.data)

    def _on_cmd_right_finger(self, msg: Float64) -> None:
        with self._cache.lock:
            self._cache.right_finger_m = float(msg.data)


def _select(msg: JointState, expected: List[str]) -> Optional[List[float]]:
    """Pick out `expected` joints from a JointState, preserving order."""
    name_to_pos = dict(zip(msg.name, msg.position))
    out: List[float] = []
    for n in expected:
        if n not in name_to_pos:
            return None
        out.append(float(name_to_pos[n]))
    return out


def flush_cache_into_robot(cache: LatestCommandCache, robot) -> None:
    """Drain the LatestCommandCache once and push the targets into the robot.

    Idempotent: if no new command has arrived since the last drain the
    relevant cache field is `None` and we leave the corresponding DOFs alone.

    Called from the sim-loop thread every physics step.
    """
    with cache.lock:
        left  = cache.left_arm_targets
        right = cache.right_arm_targets
        head  = cache.head_targets
        lift  = cache.lift_target_m
        lf    = cache.left_finger_m
        rf    = cache.right_finger_m

    if left is not None:
        robot.set_arm_targets("left", left)
    if right is not None:
        robot.set_arm_targets("right", right)
    if head is not None:
        robot.set_head(head[0], head[1])
    if lift is not None:
        robot.set_lift_m(lift)
    if lf is not None:
        robot.set_finger("left", lf)
    if rf is not None:
        robot.set_finger("right", rf)
