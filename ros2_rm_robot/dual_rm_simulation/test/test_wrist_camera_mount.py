"""Guards the wrist D435 mount frames against the two mistakes this geometry
invites.

1. MIRRORED SIGN. The housing X offset is not a left/right rule: 65b-left sits
   at +X while 65b-right, 75b-left and 75b-right all sit at -X. Deriving the
   sign from the side instead of reading it from the config buries a camera
   inside its own wrist link, where it renders the link interior and looks
   merely "dark" rather than obviously broken.
2. AIM APPLIED AT THE WRONG PIVOT. pan/tilt must rotate about the housing
   centroid. If the aim joint is folded into the mount joint, tilting also
   translates the camera out through the housing wall.

The mount pose is asserted against config/wrist_cameras.yaml itself, so the
YAML stays the single source of truth: editing it cannot silently drift from
what the URDF builds.

NOTE: the xacro include resolves via $(find ...) -> the INSTALL space.
Rebuild (colcon build --packages-select dual_rm_description
dual_rm_simulation) after editing xacros or the YAML, or this test sees old
files.
"""
import math
import shutil
import subprocess
import unittest
from pathlib import Path
from xml.dom import minidom

import numpy as np
import yaml

XACRO = Path(__file__).resolve().parent.parent / "urdf" / "r2d3_sim.urdf.xacro"


def _config_path():
    ros2_bin = shutil.which("ros2")
    if ros2_bin is None:
        raise unittest.SkipTest("ros2 not on PATH (source the ROS workspace)")
    out = subprocess.run(
        [ros2_bin, "pkg", "prefix", "--share", "dual_rm_description"],
        capture_output=True, text=True,
    )
    if out.returncode != 0:
        # "package not found" is a legitimately unbuilt workspace, not a
        # regression -- skip rather than fail.
        raise unittest.SkipTest(
            f"dual_rm_description not found (workspace not built?): {out.stderr[-400:]}")
    return Path(out.stdout.strip()) / "config" / "wrist_cameras.yaml"


def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF fixed-axis: Rz*Ry*Rx


def _flatten_urdf(arm_model):
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        raise unittest.SkipTest("xacro not on PATH (source the ROS workspace)")
    out = subprocess.run(
        [xacro_bin, str(XACRO), f"arm_model:={arm_model}"],
        capture_output=True, text=True,
    )
    if out.returncode != 0:
        raise AssertionError(
            f"xacro failed for arm_model={arm_model!r} (exit {out.returncode}): "
            f"{out.stderr}")
    return out.stdout


def _joints(urdf_str):
    """child_link -> (parent_link, rpy, xyz) for every joint."""
    dom = minidom.parseString(urdf_str)
    out = {}
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
        out[ch[0].getAttribute("link")] = (
            par[0].getAttribute("link"), rpy, np.array(xyz))
    return out


def _links(urdf_str):
    dom = minidom.parseString(urdf_str)
    return {ln.getAttribute("name") for ln in dom.getElementsByTagName("link")}


class TestWristCameraMount(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = yaml.safe_load(_config_path().read_text())
        cls.urdf = {m: _flatten_urdf(m) for m in ("65b", "75b")}

    def test_all_frames_exist_for_both_variants(self):
        for model in ("65b", "75b"):
            links = _links(self.urdf[model])
            for side in ("left", "right"):
                for suffix in ("camera_mount", "camera_link",
                               "camera_color_frame", "camera_color_optical_frame",
                               "camera_depth_frame", "camera_depth_optical_frame"):
                    name = f"{side}_wrist_{suffix}"
                    self.assertIn(name, links, f"{model}: missing link {name}")

    def test_mount_matches_config_exactly(self):
        """Catches the mirrored-sign failure: a camera at the wrong X sign is
        inside the wrist link, not outside it."""
        for model in ("65b", "75b"):
            joints = _joints(self.urdf[model])
            for side in ("left", "right"):
                entry = self.cfg[model][side]
                parent, rpy, xyz = joints[f"{side}_wrist_camera_mount"]
                self.assertEqual(parent, entry["parent"],
                                 f"{model}/{side}: wrong parent link")
                np.testing.assert_allclose(
                    xyz, entry["xyz"], atol=1e-9,
                    err_msg=f"{model}/{side}: mount xyz does not match config")
                np.testing.assert_allclose(
                    rpy, entry["rpy"], atol=1e-9,
                    err_msg=f"{model}/{side}: mount rpy does not match config")

    def test_aim_is_a_separate_joint_carrying_only_pan_tilt(self):
        """pan/tilt must live in their own joint so they pivot about the
        housing centroid and never translate the camera."""
        for model in ("65b", "75b"):
            joints = _joints(self.urdf[model])
            for side in ("left", "right"):
                entry = self.cfg[model][side]
                parent, rpy, xyz = joints[f"{side}_wrist_camera_link"]
                self.assertEqual(parent, f"{side}_wrist_camera_mount",
                                 f"{model}/{side}: aim joint must hang off the mount")
                np.testing.assert_allclose(
                    xyz, [0.0, 0.0, 0.0], atol=1e-12,
                    err_msg=f"{model}/{side}: aim joint must not translate")
                np.testing.assert_allclose(
                    rpy, [0.0, entry["tilt"], entry["pan"]], atol=1e-9,
                    err_msg=f"{model}/{side}: aim joint rpy should be (0, tilt, pan)")

    def test_bore_points_away_from_the_wrist_axis(self):
        """At the nominal aim, the camera must look OUTWARD. This is a
        consistency check WITHIN the YAML only: it reads entry["xyz"] from
        the config on both sides of the dot product, so it verifies that
        each entry's nominal yaw and its X offset agree in sign -- it cannot
        fail from a URDF error. test_mount_matches_config_exactly is what
        actually ties the config to the URDF."""
        for model in ("65b", "75b"):
            joints = _joints(self.urdf[model])
            for side in ("left", "right"):
                entry = self.cfg[model][side]
                _, rpy, xyz = joints[f"{side}_wrist_camera_mount"]
                bore = _rpy_to_R(*rpy)[:, 0]
                self.assertGreater(
                    float(np.dot(bore, np.array(entry["xyz"]))), 0.0,
                    f"{model}/{side}: bore points back into the wrist link "
                    f"(bore={bore}, offset={entry['xyz']})")

    def test_color_and_depth_optical_frames_are_coincident(self):
        """Gz rgbd_camera renders colour and depth from ONE viewpoint, and the
        real D435 publishes depth aligned to colour. Modelling a baseline here
        would be a lie the sim cannot honour."""
        for model in ("65b", "75b"):
            joints = _joints(self.urdf[model])
            for side in ("left", "right"):
                _, _, c = joints[f"{side}_wrist_camera_color_frame"]
                _, _, d = joints[f"{side}_wrist_camera_depth_frame"]
                np.testing.assert_allclose(
                    c, d, atol=1e-12,
                    err_msg=f"{model}/{side}: colour and depth frames must coincide")


if __name__ == "__main__":
    unittest.main()
