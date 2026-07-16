"""Regression guard: both ZED optical frames must bore along the robot's
nav-forward direction, upright, and sit 120 mm apart (ZED 2 baseline).

The sim overlay yaws the whole mechanical tree +90deg at
base_footprint_to_base (mesh -> Nav2). The ZED's physical mount yaw (-90deg
at zed_mount_joint; the ZED body is X-forward, the head is mesh -Y-forward)
cancels it, so the standard optical rotations in the zed2 macro come out
globally correct with NO sim-only compensation (retires the extra -pi/2 the
old depth_camera.urdf.xacro carried; see commit 9a01958 / issue #11). The
zed2 case is wedge-shaped: zed_camera_center carries a +0.05 rad pitch
(bottom_slope, Stereolabs' own model), so the expected bore is nav +X
pitched down by exactly that angle. Any YAW error still fails loudly.

MuJoCo hangs its cameras on these frames' sites (mujoco_inputs.urdf.xacro),
so this guard covers the MuJoCo render orientation the same way
test_gz_camera_bore.py covers Gz.
"""
import math
import shutil
import subprocess
import unittest
from pathlib import Path
from xml.dom import minidom

import numpy as np

XACRO = Path(__file__).resolve().parent.parent / "urdf" / "r2d3_mujoco.urdf.xacro"

SLOPE = 0.05      # zed2 bottom_slope (rad) -- keep equal to zed2.urdf.xacro
BASELINE = 0.12   # ZED 2 stereo baseline (m)


def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF fixed-axis: Rz*Ry*Rx


def _flatten_urdf():
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        raise unittest.SkipTest("xacro not on PATH (source the ROS workspace)")
    out = subprocess.run(
        [xacro_bin, str(XACRO), "arm_model:=65b"],
        capture_output=True, text=True,
    )
    if out.returncode != 0:
        raise unittest.SkipTest(f"xacro failed (workspace not built?): {out.stderr[-400:]}")
    return out.stdout


def _joint_chain(urdf_str, child, root="base_footprint"):
    """Compose the fixed/neutral-config pose (R, p) from root down to `child`.

    Revolute/prismatic joints are evaluated at q=0, so only their <origin>
    contributes. Returns (R, p): child axes / position in root frame.
    """
    dom = minidom.parseString(urdf_str)
    joints = {}  # child_link -> (parent_link, R_origin, p_origin)
    for j in dom.getElementsByTagName("joint"):
        par = j.getElementsByTagName("parent")
        ch = j.getElementsByTagName("child")
        if not par or not ch:
            continue
        o = j.getElementsByTagName("origin")
        rpy, xyz = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        if o:
            if o[0].getAttribute("rpy").strip():
                rpy = tuple(float(v) for v in o[0].getAttribute("rpy").split())
            if o[0].getAttribute("xyz").strip():
                xyz = tuple(float(v) for v in o[0].getAttribute("xyz").split())
        joints[ch[0].getAttribute("link")] = (
            par[0].getAttribute("link"), _rpy_to_R(*rpy), np.array(xyz))

    chain = []
    node = child
    while node != root:
        if node not in joints:
            raise AssertionError(f"no joint chain from {root} to {child} (stuck at {node})")
        parent, R, p = joints[node]
        chain.append((R, p))
        node = parent
    R_total, p_total = np.eye(3), np.zeros(3)
    for R, p in reversed(chain):  # root -> ... -> child
        p_total = p_total + R_total @ p
        R_total = R_total @ R
    return R_total, p_total


# Expected optical axes in base_footprint: only the bottom_slope pitch
# survives (overlay +pi/2 yaw and mount -pi/2 yaw cancel).
_EXPECTED_BORE = np.array([math.cos(SLOPE), 0.0, -math.sin(SLOPE)])   # optical +Z
_EXPECTED_DOWN = np.array([-math.sin(SLOPE), 0.0, -math.cos(SLOPE)])  # optical +Y


class TestZedOpticalFrames(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = _flatten_urdf()

    def test_bores_point_nav_forward_both_eyes(self):
        for side in ("left", "right"):
            R, _ = _joint_chain(self.urdf, f"zed_{side}_camera_frame_optical")
            bore = R[:, 2]   # optical +Z = viewing direction
            down = R[:, 1]   # optical +Y = down in image
            np.testing.assert_allclose(
                bore, _EXPECTED_BORE, atol=1e-6,
                err_msg=f"{side}: optical bore should be nav +X (pitched "
                        f"down bottom_slope={SLOPE}), got {bore}")
            self.assertLess(abs(bore[1]), 1e-6,
                            f"{side}: optical bore has a YAW error: {bore}")
            np.testing.assert_allclose(
                down, _EXPECTED_DOWN, atol=1e-6,
                err_msg=f"{side}: optical down should be nav -Z (pitched by "
                        f"bottom_slope={SLOPE}), got {down}")

    def test_stereo_baseline(self):
        R_left, p_left = _joint_chain(self.urdf, "zed_left_camera_frame_optical")
        _, p_right = _joint_chain(self.urdf, "zed_right_camera_frame_optical")
        # In the LEFT OPTICAL frame (X right, Y down, Z forward) the right
        # eye sits +BASELINE along +X.
        offset_in_left = R_left.T @ (p_right - p_left)
        np.testing.assert_allclose(
            offset_in_left, [BASELINE, 0.0, 0.0], atol=1e-9,
            err_msg=f"stereo baseline wrong: {offset_in_left}")


if __name__ == "__main__":
    unittest.main()
