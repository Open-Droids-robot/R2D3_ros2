"""Regression guard: the camera's optical frame must bore along the robot's
nav-forward direction (base_footprint +X), upright.

The sim overlay yaws the whole mechanical tree +90deg at base_footprint_to_base
to reconcile the mesh frame with the Nav2 convention. The head-mounted camera
rides that yaw, so its optical frame carries a -90deg compensation (in the
sim-only depth_camera.urdf.xacro). If that compensation is dropped -- as it was
in commit 9a01958 -- the camera bores along base_footprint +Y and /camera/image
looks 90deg to the robot's left. This test recomputes the base_footprint ->
camera_optical_frame transform from the flattened URDF and asserts the bore.
"""
import math
import shutil
import subprocess
import unittest
from pathlib import Path
from xml.dom import minidom

import numpy as np

XACRO = Path(__file__).resolve().parent.parent / "urdf" / "r2d3_mujoco.urdf.xacro"


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


def _joint_orientation_chain(urdf_str, child, root="base_footprint"):
    """Compose the fixed/neutral-config rotation from root down to `child`.

    Revolute/prismatic joints are evaluated at q=0, so only their <origin> rpy
    contributes to orientation. Returns R (child axes expressed in root frame).
    """
    dom = minidom.parseString(urdf_str)
    joints = {}  # child_link -> (parent_link, R_origin)
    for j in dom.getElementsByTagName("joint"):
        p = j.getElementsByTagName("parent")
        c = j.getElementsByTagName("child")
        if not p or not c:
            continue
        o = j.getElementsByTagName("origin")
        rpy = (0.0, 0.0, 0.0)
        if o and o[0].getAttribute("rpy").strip():
            rpy = tuple(float(v) for v in o[0].getAttribute("rpy").split())
        joints[c[0].getAttribute("link")] = (p[0].getAttribute("link"), _rpy_to_R(*rpy))

    # Walk up from child to root, collecting rotations, then compose top-down.
    chain = []
    node = child
    while node != root:
        if node not in joints:
            raise AssertionError(f"no joint chain from {root} to {child} (stuck at {node})")
        parent, R = joints[node]
        chain.append(R)
        node = parent
    R_total = np.eye(3)
    for R in reversed(chain):  # root -> ... -> child
        R_total = R_total @ R
    return R_total


class TestCameraOpticalFrame(unittest.TestCase):
    def test_bore_points_nav_forward(self):
        urdf = _flatten_urdf()
        R = _joint_orientation_chain(urdf, "camera_optical_frame")
        bore = R[:, 2]   # optical +Z = viewing direction
        down = R[:, 1]   # optical +Y = down in image
        np.testing.assert_allclose(bore, [1.0, 0.0, 0.0], atol=1e-6,
                                   err_msg=f"optical bore should be nav +X, got {bore}")
        np.testing.assert_allclose(down, [0.0, 0.0, -1.0], atol=1e-6,
                                   err_msg=f"optical down should be nav -Z (upright), got {down}")


if __name__ == "__main__":
    unittest.main()
