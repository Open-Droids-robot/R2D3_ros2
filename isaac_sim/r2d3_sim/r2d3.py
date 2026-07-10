"""R2D3 — the high-level platform SDK.

One object that boots Isaac Sim, loads the robot, and exposes a clean, in-process
control + sensing API (no ROS required). Build RL envs, VLM loops, or teleop on
top of it.

    from isaac_sim.r2d3_sim import R2D3
    with R2D3(end_effector="dexterous") as sim:
        sim.reset()
        sim.set_arm_pose("left", [0.45, -0.21, 0.51], sim.top_down_quat)
        sim.set_gripper("left", 1.0)          # close
        rgb = sim.get_image("head")           # numpy HxWx3, in-process
        sim.step(n=30)

Run it through the Isaac launcher, e.g.:
    scripts/isaacsim_ros2.sh isaac_sim/examples/01_hello_robot.py
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

_USD_DIR = Path(__file__).resolve().parents[1]


@dataclass
class JointState:
    """Snapshot of all articulation DOFs."""
    names: list
    positions: np.ndarray

    def get(self, name: str) -> float:
        return float(self.positions[self.names.index(name)])


@dataclass
class Observation:
    """One observation bundle (everything readable in-process)."""
    images: dict            # {camera_name: rgb uint8 HxWx3}
    depths: dict            # {camera_name: depth float32 HxW (m)}
    joint_positions: dict   # {joint_name: rad | m}
    lift_m: float
    wrench_left: np.ndarray  # [fx,fy,fz,tx,ty,tz]
    wrench_right: np.ndarray
    ee_pose_left: tuple      # (xyz, quat_wxyz)
    ee_pose_right: tuple


class R2D3:
    """High-level handle to the simulated R2D3 robot."""

    def __init__(self, *, end_effector: str = "dexterous", headless: bool = True,
                 mobile: bool = False, usd_path=None, enable_cameras: bool = True,
                 enable_ros: bool = False, camera_resolution: tuple[int, int] = (640, 480),
                 stage_units_m: float = 1.0, setup=None) -> None:
        """Boot Isaac, load the R2D3 USD, bind the articulation + cameras + IK.

        Parameters
        ----------
        end_effector : "dexterous" (Inspire 5-finger hand) or "gripper" (2-finger).
        headless     : run without the Kit UI window.
        mobile       : load the wheels-revolute build and free the base so it can
                       be driven (set_base_pose); otherwise the base is fixed.
        usd_path     : override the auto-selected USD.
        enable_cameras : attach in-process camera render products (get_image).
        enable_ros   : also publish/subscribe on the /r2d3/sim/* ROS topics.
        setup        : optional callable ``setup(world)`` run AFTER the robot is
                       loaded but BEFORE world.reset() — add task objects
                       (tables, cubes via isaacsim.core.api.objects) here so
                       physics initialises them. See examples/07_grasp_cube.py.
        """
        # R2D3_EE is read at IMPORT time by sim_topics/scene, so set it first.
        os.environ["R2D3_EE"] = end_effector
        from . import boot
        boot.launch(headless=headless, enable_ros=enable_ros)

        from . import scene as scene_mod
        from . import sim_topics as t
        from .robot import Robot
        from .cameras import CameraRig
        from .ik import ArmIK
        from isaacsim.core.api import World
        import omni.usd
        from pxr import UsdPhysics

        self._t = t
        self.end_effector = end_effector
        if t.EE_TYPE != end_effector:
            raise RuntimeError(
                f"r2d3_sim.sim_topics was already imported with end_effector="
                f"{t.EE_TYPE!r}, but R2D3(end_effector={end_effector!r}) was requested. "
                f"The end-effector is locked at first import of the r2d3_sim submodules — "
                f"construct R2D3 (or set R2D3_EE) BEFORE importing sim_topics/scene/robot, "
                f"and don't construct two R2D3s with different end-effectors in one process."
            )
        self.mobile = mobile
        self._scene = scene_mod
        self.world = World(stage_units_in_meters=stage_units_m)

        if usd_path is None and mobile:
            # Mobile builds (wheels revolute) are per-end-effector, just like the
            # static ones: usd_mobile = dexterous, usd_gripper_mobile = gripper.
            mdir = "usd_gripper_mobile" if end_effector == "gripper" else "usd_mobile"
            usd_path = _USD_DIR / mdir / "r2d3_v1.usda"
        self._robot_prim = scene_mod.assemble(self.world, usd_path=usd_path)

        if mobile:
            stage = omni.usd.get_context().get_stage()
            for prim in stage.Traverse():
                if prim.GetName() == "root_joint":
                    UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
                    break

        if setup is not None:
            setup(self.world)        # add task objects before physics init

        self.world.reset()
        self.robot = Robot(prim_path=self._robot_prim)
        self.robot.initialize()
        scene_mod._hide_legacy_hand_flanges()

        self.ik = ArmIK(end_effector=end_effector)
        self.cameras = None
        if enable_cameras:
            self.cameras = CameraRig(resolution=camera_resolution)
            self.cameras.attach()

        self._ros = None
        if enable_ros:
            self._start_ros()

    # -------------------------------------------------------------- properties
    @property
    def top_down_quat(self) -> np.ndarray:
        """Convenience: orientation (wxyz) for a top-down grasp."""
        from . import helpers as h
        return h.top_down_quat()

    # ---------------------------------------------------------------- lifecycle
    def reset(self) -> "Observation":
        """Drive to the home pose, settle, warm up the renderer, return obs."""
        self.robot.go_home()
        for _ in range(60):
            self.world.step(render=False)
        if self.cameras is not None and not self.cameras.warmed:
            self.cameras.warmup(self.world, steps=20)
        self.ik.sync_base()
        return self.get_observation()

    def step(self, *, render: bool = True, n: int = 1) -> "Observation":
        """Advance ``n`` physics steps (flushing ROS commands if enabled)."""
        for _ in range(n):
            if self._ros is not None:
                self._ros_flush()
            self.world.step(render=render)
        return self.get_observation()

    def close(self) -> None:
        if self.cameras is not None:
            self.cameras.detach()
        if self._ros is not None:
            self._stop_ros()
        # Isaac's app shutdown can hard-exit and drop buffered stdout — flush
        # first so a script's prints survive.
        import sys
        sys.stdout.flush()
        sys.stderr.flush()
        from . import boot
        boot.close()

    def __enter__(self) -> "R2D3":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------- control: arm
    def set_arm_joints(self, side: str, q: Sequence[float]) -> None:
        """Set the 7 arm joint position targets (radians)."""
        self.robot.set_arm_targets(side, list(q))

    def set_arm_pose(self, side: str, position, quat_wxyz=None, *,
                     pos_tol: float = 0.01, ori_tol: float = 0.2) -> bool:
        """IK to a Cartesian EE pose (world frame); apply if solved. Returns ok.
        Left arm only (right raises NotImplementedError)."""
        if side != "left":
            raise NotImplementedError(
                "IK is left-arm only — right-arm needs r2d3_right_arm_lula.yaml.")
        self.ik.sync_base()
        q, ok = self.ik.ik(position, quat_wxyz, pos_tol=pos_tol, ori_tol=ori_tol)
        if ok:
            self.robot.set_arm_targets(side, list(q))
        return ok

    # -------------------------------------------------- control: head/lift/hand
    def set_head(self, pan: float, tilt: float) -> None:
        self.robot.set_head(pan, tilt)

    def set_lift(self, height_m: float) -> None:
        self.robot.set_lift_m(height_m)

    def set_gripper(self, side: str, frac: float) -> None:
        """End-effector-agnostic open/close: frac 0 = open, 1 = closed."""
        frac = max(0.0, min(1.0, float(frac)))
        self.robot.set_finger(side, (1.0 - frac) * self._t.FINGER_DRIVE_MAX_M)

    def set_joint_targets(self, mapping: dict) -> None:
        """Set arbitrary joint position targets by name."""
        self.robot.set_joint_targets(mapping)

    def set_base_pose(self, position, quat_wxyz) -> None:
        """Kinematically place the mobile base (requires mobile=True)."""
        self.robot.set_base_pose(position, quat_wxyz)

    def go_home(self) -> None:
        self.robot.go_home()

    # ------------------------------------------------------------------ sensing
    def get_image(self, camera: str = "head", *, depth: bool = False):
        """RGB (uint8 HxWx3) for a camera; (rgb, depth) if depth=True."""
        if self.cameras is None:
            raise RuntimeError("cameras are disabled — construct with enable_cameras=True")
        if not self.cameras.warmed:
            self.cameras.warmup(self.world, steps=12)
        rgb = self.cameras.get_rgb(camera)
        return (rgb, self.cameras.get_depth(camera)) if depth else rgb

    def get_images(self, depth: bool = False) -> dict:
        names = self.cameras.names if self.cameras is not None else []
        return {n: self.get_image(n, depth=depth) for n in names}

    def get_joint_positions(self) -> dict:
        return self.robot.get_joint_positions()

    def get_joint_state(self) -> "JointState":
        jp = self.robot.get_joint_positions()
        return JointState(names=list(jp.keys()),
                          positions=np.array(list(jp.values()), dtype=float))

    def get_lift(self) -> float:
        return self.robot.get_lift_m()

    def get_wrench(self, side: str) -> np.ndarray:
        return self.robot.get_wrist_wrench(side)

    def get_ee_pose(self, side: str):
        return self.robot.get_ee_pose(side)

    def get_observation(self) -> "Observation":
        imgs, deps = {}, {}
        if self.cameras is not None and self.cameras.warmed:
            for n in self.cameras.names:
                imgs[n] = self.cameras.get_rgb(n)
                deps[n] = self.cameras.get_depth(n)
        return Observation(
            images=imgs, depths=deps,
            joint_positions=self.robot.get_joint_positions(),
            lift_m=self.robot.get_lift_m(),
            wrench_left=self.robot.get_wrist_wrench("left"),
            wrench_right=self.robot.get_wrist_wrench("right"),
            ee_pose_left=self.robot.get_ee_pose("left"),
            ee_pose_right=self.robot.get_ee_pose("right"),
        )

    # ---------------------------------------------------------- escape hatches
    # (.robot / .world / .ik / .cameras are public attributes for power users)

    # ------------------------------------------------------------ ROS (optional)
    def _start_ros(self) -> None:
        import threading
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from . import sensors as sensors_mod
        from .sim_adapter import SimAdapter, LatestCommandCache, flush_cache_into_robot
        import omni.timeline

        sensors_mod.build_action_graph()
        omni.timeline.get_timeline_interface().play()
        if not rclpy.ok():
            rclpy.init()
        self._cache = LatestCommandCache()
        self._adapter = SimAdapter(cache=self._cache,
                                   joint_names=list(self.robot.joint_index.keys()))
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._adapter)
        self._ros_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._ros_thread.start()
        self._flush = flush_cache_into_robot
        self._ros = True

    def _ros_flush(self) -> None:
        self._flush(self._cache, self.robot)
        self._adapter.update_state(
            joint_positions=self.robot.get_joint_positions(),
            wrench_left=self.robot.get_wrist_wrench("left"),
            wrench_right=self.robot.get_wrist_wrench("right"),
        )

    def _stop_ros(self) -> None:
        import rclpy
        try:
            self._executor.shutdown()
            self._ros_thread.join(timeout=2.0)
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
