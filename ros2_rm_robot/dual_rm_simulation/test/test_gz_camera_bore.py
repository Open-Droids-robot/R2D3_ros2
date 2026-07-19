"""Regression guard: both simulated ZED eyes must render along the robot's
nav-forward direction, upright, agree with the optical frames they label
their output with, and sit 120 mm apart (the ZED 2 baseline).

A Gz camera renders along the +X of the frame its ``<sensor>`` is mounted on
(+Z up, +Y left -- the SDF camera body convention). ``gz_frame_id`` only
LABELS the published output; it does NOT reorient the render. The sim overlay
yaws the whole mechanical tree +90deg about Z at ``base_footprint_to_base``
(mesh -> Nav2); the ZED's physical mount yaw (-90deg at zed_mount_joint,
the ZED body is X-forward while the head is mesh -Y-forward) cancels it, so
the sensors mount directly on the ZED left/right camera frames with NO
sim-only compensation frames (retires the issue #11 camera_gz_frame
mechanism). Per the issue #11 postmortem the Gz ``<sensor>`` ``<pose>`` must
NOT be used for orientation.

The zed2 case is wedge-shaped: zed_camera_center carries a +0.05 rad pitch
(bottom_slope, from Stereolabs' own model), so the expected bore is nav +X
pitched down by exactly that angle -- the assertions account for it. Any YAW
error (the issue #11 failure mode) still fails loudly.

NOTE: the xacro include resolves via $(find ...) -> the INSTALL space.
Rebuild (colcon build --packages-select dual_rm_description
dual_rm_simulation) after editing xacros, or this test sees old files.
"""
import math
import shutil
import subprocess
import unittest
from pathlib import Path
from xml.dom import minidom

import numpy as np

XACRO = Path(__file__).resolve().parent.parent / "urdf" / "r2d3_sim.urdf.xacro"

SLOPE = 0.05      # zed2 bottom_slope (rad) -- keep equal to zed2.urdf.xacro
BASELINE = 0.12   # ZED 2 stereo baseline (m)


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


def _gz_camera_sensors(urdf_str):
    """Return {reference_link: (sensor_name, R_pose)} for all camera-type
    Gz sensors. R_pose is a direct-child <pose> rotation (identity if absent)
    so the bore assertions hold no matter which mechanism orients the render.
    """
    dom = minidom.parseString(urdf_str)
    found = {}
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
            found[gz.getAttribute("reference")] = (
                s.getAttribute("name"), _rpy_to_R(*rpy))
    if not found:
        raise AssertionError("no camera <sensor> found in any <gazebo> block")
    return found


# Gz builds the published optical frame from the sensor body frame by the fixed
# SDF-camera mapping: optical +Z = sensor +X (forward), optical +X = -sensor +Y
# (right), optical +Y = -sensor +Z (down). Columns are (optX, optY, optZ) in
# sensor-body coords.
_SENSOR_TO_OPTICAL = np.array([[0.0, 0.0, 1.0],
                               [-1.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0]])

# Expected sensor-body orientation in base_footprint: the +pi/2 overlay yaw and
# the -pi/2 physical mount yaw cancel, leaving only the zed2 bottom_slope
# pitch. bore = +X pitched down by SLOPE; up = +Z pitched forward by SLOPE.
_EXPECTED_BORE = np.array([math.cos(SLOPE), 0.0, -math.sin(SLOPE)])
_EXPECTED_UP = np.array([math.sin(SLOPE), 0.0, math.cos(SLOPE)])


class TestGzZedCameraBore(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = _flatten_urdf()
        cls.sensors = _gz_camera_sensors(cls.urdf)

    def test_sensors_mounted_on_zed_camera_frames(self):
        """Issue #11 postmortem: orientation lives in mount frames, never a
        sensor <pose>. Both eyes mount directly on the ZED camera frames."""
        self.assertLessEqual(
            {"zed_left_camera_frame", "zed_right_camera_frame"},
            set(self.sensors.keys()))
        for ref in ("zed_left_camera_frame", "zed_right_camera_frame"):
            _, R_pose = self.sensors[ref]
            np.testing.assert_allclose(
                R_pose, np.eye(3), atol=1e-12,
                err_msg=f"{ref}: sensor <pose> must not rotate the render")

    def _sensor_R(self, ref):
        _, R_pose = self.sensors[ref]
        R_chain, _ = _joint_chain(self.urdf, ref)
        return R_chain @ R_pose

    def test_render_bores_nav_forward_both_eyes(self):
        for ref in ("zed_left_camera_frame", "zed_right_camera_frame"):
            R = self._sensor_R(ref)
            bore, up = R[:, 0], R[:, 2]
            np.testing.assert_allclose(
                bore, _EXPECTED_BORE, atol=1e-6,
                err_msg=f"{ref}: render bore should be nav +X (pitched down "
                        f"bottom_slope={SLOPE}), got {bore}")
            # Yaw error is the issue #11 failure mode -- assert it separately
            # and explicitly.
            self.assertLess(abs(bore[1]), 1e-6,
                            f"{ref}: render bore has a YAW error: {bore}")
            np.testing.assert_allclose(
                up, _EXPECTED_UP, atol=1e-6,
                err_msg=f"{ref}: image up should be nav +Z (pitched by "
                        f"bottom_slope={SLOPE}), got {up}")

    def test_render_matches_optical_frame_label(self):
        """The render axes must agree with the optical frame each sensor is
        LABELLED with (gz_frame_id), or consumers place clouds wrong."""
        for ref, side in (("zed_left_camera_frame", "left"),
                          ("zed_right_camera_frame", "right")):
            R_optical_render = self._sensor_R(ref) @ _SENSOR_TO_OPTICAL
            R_optical_label, _ = _joint_chain(
                self.urdf, f"zed_{side}_camera_frame_optical")
            np.testing.assert_allclose(
                R_optical_render, R_optical_label, atol=1e-6,
                err_msg=f"{side}: rendered optical axes disagree with the "
                        f"zed_{side}_camera_frame_optical label")

    def test_stereo_baseline(self):
        """Right eye sits exactly BASELINE along the left eye's -Y (the ZED
        left camera is the +Y eye of camera_center)."""
        R_left, p_left = _joint_chain(self.urdf, "zed_left_camera_frame")
        _, p_right = _joint_chain(self.urdf, "zed_right_camera_frame")
        offset_in_left = R_left.T @ (p_right - p_left)
        np.testing.assert_allclose(
            offset_in_left, [0.0, -BASELINE, 0.0], atol=1e-9,
            err_msg=f"stereo baseline wrong: {offset_in_left}")


class TestGzWristCameraBore(unittest.TestCase):
    """The wrist cameras must render along the direction their mount frame
    bores, and label their output with an optical frame that agrees.

    Unlike the ZED, the wrists hang off moving arm joints, so there is no
    fixed nav-frame expectation to assert. Instead we assert the two
    invariants that survive any arm pose: the sensor must be mounted on the
    camera colour frame (never oriented by a <pose>, per the issue #11
    postmortem), and the rendered optical axes must equal the optical frame
    named in gz_frame_id.
    """

    @classmethod
    def setUpClass(cls):
        cls.urdf = _flatten_urdf()
        cls.sensors = _gz_camera_sensors(cls.urdf)

    def test_wrist_sensors_mounted_on_colour_frames(self):
        for side in ("left", "right"):
            ref = f"{side}_wrist_camera_color_frame"
            self.assertIn(ref, self.sensors,
                          f"no Gz camera sensor mounted on {ref}")

    def test_wrist_sensor_pose_is_identity(self):
        """Issue #11 postmortem: orientation lives in mount frames, never a
        sensor <pose>. A <pose> here would desync Gz from MuJoCo, which has
        no equivalent override."""
        for side in ("left", "right"):
            _, R_pose = self.sensors[f"{side}_wrist_camera_color_frame"]
            np.testing.assert_allclose(
                R_pose, np.eye(3), atol=1e-12,
                err_msg=f"{side} wrist: sensor <pose> must not rotate the render")

    def test_wrist_render_matches_optical_frame_label(self):
        for side in ("left", "right"):
            ref = f"{side}_wrist_camera_color_frame"
            _, R_pose = self.sensors[ref]
            R_chain, _ = _joint_chain(self.urdf, ref)
            R_optical_render = (R_chain @ R_pose) @ _SENSOR_TO_OPTICAL
            R_optical_label, _ = _joint_chain(
                self.urdf, f"{side}_wrist_camera_color_optical_frame")
            np.testing.assert_allclose(
                R_optical_render, R_optical_label, atol=1e-6,
                err_msg=f"{side} wrist: rendered optical axes disagree with "
                        f"the {side}_wrist_camera_color_optical_frame label")


if __name__ == "__main__":
    unittest.main()
