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


class TestWristCameraOpticalFrames(unittest.TestCase):
    """MuJoCo hangs its cameras on these frames' sites, so the frames must
    exist in the MuJoCo URDF too - the ZED equivalent of this guard is what
    caught the issue #11 yaw error.

    The wrists hang off moving arm joints, so there is no fixed nav-frame
    bore to assert. What must hold in any pose is that the optical frame is a
    proper REP-103 optical frame relative to its camera frame: +Z forward,
    +X right, +Y down.
    """

    @classmethod
    def setUpClass(cls):
        cls.urdf = _flatten_urdf()

    def test_optical_frames_exist(self):
        links = {ln.getAttribute("name")
                 for ln in minidom.parseString(self.urdf).getElementsByTagName("link")}
        for side in ("left", "right"):
            for suffix in ("camera_color_frame", "camera_color_optical_frame"):
                name = f"{side}_wrist_{suffix}"
                self.assertIn(name, links, f"missing link {name}")

    def test_optical_rotation_is_rep103(self):
        for side in ("left", "right"):
            R_cam, _ = _joint_chain(self.urdf, f"{side}_wrist_camera_color_frame")
            R_opt, _ = _joint_chain(self.urdf,
                                    f"{side}_wrist_camera_color_optical_frame")
            # optical axes expressed in the camera frame
            R_rel = R_cam.T @ R_opt
            expected = np.array([[0.0, 0.0, 1.0],
                                 [-1.0, 0.0, 0.0],
                                 [0.0, -1.0, 0.0]])
            np.testing.assert_allclose(
                R_rel, expected, atol=1e-6,
                err_msg=f"{side} wrist: optical frame is not REP-103 "
                        f"(+Z fwd, +X right, +Y down)")


class TestWristCameraMujocoWiring(unittest.TestCase):
    """Regression guard for the MuJoCo-only wiring added alongside the wrist
    D435s: the MJCF <camera> entries in mujoco_inputs.urdf.xacro and the
    ros2_control <sensor> entries in mujoco_ros2_control.urdf.xacro.

    Unlike TestWristCameraOpticalFrames above (which only checks link/frame
    geometry inherited from dual_rm_description), these tests fail if the
    MJCF <camera> / ros2_control <sensor> pairing, site references, topic
    strings, or optics constants added by this task are missing, wrong, or
    name-mismatched. A silent name mismatch between the MJCF camera name and
    the ros2_control sensor name is the most likely way this feature breaks
    (mujoco_ros2_control drops the camera with no error).
    """

    EXPECTED_SIDES = ("left_wrist", "right_wrist")
    EXPECTED_TOPICS = {
        "left_wrist": {
            "info_topic": "/left_wrist/color/camera_info",
            "image_topic": "/left_wrist/color/image_raw",
            "depth_topic": "/left_wrist/depth/image_rect_raw",
        },
        "right_wrist": {
            "info_topic": "/right_wrist/color/camera_info",
            "image_topic": "/right_wrist/color/image_raw",
            "depth_topic": "/right_wrist/depth/image_rect_raw",
        },
    }

    @classmethod
    def setUpClass(cls):
        cls.urdf = _flatten_urdf()
        cls.dom = minidom.parseString(cls.urdf)

    def _mjcf_cameras(self):
        """<camera> elements inside <mujoco_inputs>, keyed by name."""
        cams = {}
        for mi in self.dom.getElementsByTagName("mujoco_inputs"):
            for cam in mi.getElementsByTagName("camera"):
                name = cam.getAttribute("name")
                if name in ("left_wrist", "right_wrist"):
                    cams[name] = cam
        return cams

    def _ros2_control_sensors(self):
        """<sensor> elements inside <ros2_control>, keyed by name."""
        sensors = {}
        for rc in self.dom.getElementsByTagName("ros2_control"):
            for sen in rc.getElementsByTagName("sensor"):
                name = sen.getAttribute("name")
                if name in ("left_wrist", "right_wrist"):
                    sensors[name] = sen
        return sensors

    @staticmethod
    def _sensor_params(sensor_el):
        params = {}
        for p in sensor_el.getElementsByTagName("param"):
            params[p.getAttribute("name")] = "".join(
                n.data for n in p.childNodes if n.nodeType == n.TEXT_NODE
            ).strip()
        return params

    def test_mjcf_and_ros2_control_names_match(self):
        """The MJCF camera names and ros2_control sensor names must be the
        exact same set. If they diverge, mujoco_ros2_control silently
        produces no images for the mismatched camera."""
        mjcf_names = set(self._mjcf_cameras().keys())
        sensor_names = set(self._ros2_control_sensors().keys())
        self.assertEqual(
            mjcf_names, {"left_wrist", "right_wrist"},
            f"expected MJCF wrist camera names {{left_wrist, right_wrist}}, "
            f"got {mjcf_names}")
        self.assertEqual(
            sensor_names, {"left_wrist", "right_wrist"},
            f"expected ros2_control wrist sensor names "
            f"{{left_wrist, right_wrist}}, got {sensor_names}")
        self.assertEqual(
            mjcf_names, sensor_names,
            f"MJCF camera names {mjcf_names} and ros2_control sensor names "
            f"{sensor_names} must match exactly")

    def test_camera_site_references_resolve(self):
        """Each MJCF wrist camera's site= must exist as a link in the
        flattened URDF, or the converter silently drops that camera."""
        links = {ln.getAttribute("name")
                 for ln in self.dom.getElementsByTagName("link")}
        cams = self._mjcf_cameras()
        self.assertEqual(set(cams.keys()), {"left_wrist", "right_wrist"})
        for name, cam in cams.items():
            site = cam.getAttribute("site")
            self.assertTrue(site, f"{name}: MJCF camera has no site=")
            self.assertIn(
                site, links,
                f"{name}: MJCF camera site={site!r} does not exist as a "
                f"link (converter would silently drop this camera)")

    def test_sensor_topics_match_contract(self):
        """ros2_control sensor topic params must match the exact contract
        the Gz sim publishes, so consumers stay sim-agnostic."""
        sensors = self._ros2_control_sensors()
        self.assertEqual(set(sensors.keys()), {"left_wrist", "right_wrist"})
        for name, sensor_el in sensors.items():
            params = self._sensor_params(sensor_el)
            expected = self.EXPECTED_TOPICS[name]
            for key, expected_value in expected.items():
                self.assertEqual(
                    params.get(key), expected_value,
                    f"{name}: param {key} = {params.get(key)!r}, "
                    f"expected {expected_value!r}")

    def test_camera_optics_constants(self):
        """fovy and resolution must match the D435 848x480 contract on both
        wrist cameras."""
        cams = self._mjcf_cameras()
        self.assertEqual(set(cams.keys()), {"left_wrist", "right_wrist"})
        for name, cam in cams.items():
            self.assertEqual(
                cam.getAttribute("fovy"), "56.4",
                f"{name}: fovy = {cam.getAttribute('fovy')!r}, expected '56.4'")
            self.assertEqual(
                cam.getAttribute("resolution"), "848 480",
                f"{name}: resolution = {cam.getAttribute('resolution')!r}, "
                f"expected '848 480'")


if __name__ == "__main__":
    unittest.main()
