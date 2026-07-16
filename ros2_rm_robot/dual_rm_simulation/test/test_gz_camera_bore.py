"""Regression guard: the Gazebo (Gz Sim) camera must render along the robot's
nav-forward direction (base_footprint +X), upright, and agree with the
``camera_optical_frame`` it labels its output with.

A Gz camera renders along the +X of the frame its ``<sensor>`` is mounted on
(+Z up, +Y left -- the SDF camera body convention). ``gz_frame_id`` only
LABELS the published output; it does NOT reorient the render. The sim overlay
yaws the whole mechanical tree +90deg about Z at ``base_footprint_to_base``
(mesh -> Nav2), so ``camera_link`` +X = base_footprint +Y and an
uncompensated camera images 90deg to the robot's left (issue #11).

The fix mounts the sensor on the sim-only ``camera_gz_frame`` -- a massless
fixed link yawed -90deg off ``camera_link`` -- so the render bores nav +X.
This is the Gz analog of the MuJoCo fix (commit c7bbfec), which reoriented
the frame MuJoCo's camera hangs on. Per the issue #11 postmortem the Gz
``<sensor>`` ``<pose>`` must NOT be used for this compensation.

This test recomputes base_footprint -> sensor-mount orientation from the
flattened sim URDF (joint chain at q=0, composed with any direct ``<pose>``
on the sensor, so the guard holds regardless of mechanism) and asserts the
render bore and the render/label consistency.

NOTE: the xacro include resolves via $(find dual_rm_simulation) -> the
INSTALL space. Rebuild (colcon build --packages-select dual_rm_simulation)
after editing the xacro, or this test sees the old file.
"""
import math
import shutil
import subprocess
import unittest
from pathlib import Path
from xml.dom import minidom

import numpy as np

XACRO = Path(__file__).resolve().parent.parent / "urdf" / "r2d3_sim.urdf.xacro"


def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF/SDF fixed-axis: Rz*Ry*Rx


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


def _gz_camera_sensor(urdf_str):
    """Return (reference_link, R_pose) for the Gz camera <sensor>.

    reference_link is the link named by the enclosing <gazebo reference=...>.
    R_pose is the rotation of an optional direct-child <pose> ("x y z r p y",
    identity if absent) so the bore assertion holds no matter which mechanism
    orients the render.
    """
    dom = minidom.parseString(urdf_str)
    for gz in dom.getElementsByTagName("gazebo"):
        for s in gz.getElementsByTagName("sensor"):
            if "camera" not in s.getAttribute("type"):
                continue
            rpy = (0.0, 0.0, 0.0)
            for pose in s.getElementsByTagName("pose"):
                if pose.parentNode is s:
                    vals = [float(v) for v in pose.firstChild.data.split()]
                    if len(vals) == 6:
                        rpy = tuple(vals[3:])
                    break
            return gz.getAttribute("reference"), _rpy_to_R(*rpy)
    raise AssertionError("no camera <sensor> found in any <gazebo> block")


# Gz builds the published optical frame from the sensor body frame by the fixed
# SDF-camera mapping: optical +Z = sensor +X (forward), optical +X = -sensor +Y
# (right), optical +Y = -sensor +Z (down). Columns are (optX, optY, optZ) in
# sensor-body coords.
_SENSOR_TO_OPTICAL = np.array([[0.0, 0.0, 1.0],
                               [-1.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0]])


class TestGzCameraBore(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = _flatten_urdf()
        ref, R_pose = _gz_camera_sensor(cls.urdf)
        cls.sensor_ref = ref
        cls.R_sensor = _joint_orientation_chain(cls.urdf, ref) @ R_pose

    def test_sensor_mounted_on_gz_frame(self):
        """Issue #11 postmortem: compensation must be a mount frame, not a
        sensor <pose>. Guards against silently moving it back."""
        self.assertEqual(self.sensor_ref, "camera_gz_frame")

    def test_render_bore_points_nav_forward(self):
        """The Gz render direction must be nav-forward and upright."""
        bore = self.R_sensor[:, 0]  # sensor +X = Gz viewing direction
        up = self.R_sensor[:, 2]    # sensor +Z = up in image
        np.testing.assert_allclose(bore, [1.0, 0.0, 0.0], atol=1e-6,
                                   err_msg=f"Gz render bore should be nav +X, got {bore}")
        np.testing.assert_allclose(up, [0.0, 0.0, 1.0], atol=1e-6,
                                   err_msg=f"Gz image up should be nav +Z (upright), got {up}")

    def test_render_matches_optical_frame_label(self):
        """The Gz render must agree with the frame it is LABELLED with.

        Gz stamps its output ``gz_frame_id=camera_optical_frame`` but derives
        the actual optical axes from the sensor body frame. If those disagree
        with the URDF's ``camera_optical_frame`` TF, consumers place the cloud
        wrong -- the "potentially worse (inconsistent)" failure issue #11
        flagged.
        """
        R_optical_render = self.R_sensor @ _SENSOR_TO_OPTICAL
        R_optical_label = _joint_orientation_chain(self.urdf, "camera_optical_frame")
        np.testing.assert_allclose(
            R_optical_render, R_optical_label, atol=1e-6,
            err_msg="Gz-rendered optical axes disagree with the camera_optical_frame "
                    f"label:\nrender=\n{R_optical_render}\nlabel=\n{R_optical_label}")


if __name__ == "__main__":
    unittest.main()
