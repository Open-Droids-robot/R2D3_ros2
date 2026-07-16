import sys
import unittest
from pathlib import Path

# Import the script as a module (it lives in scripts/, not a python package)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import wait_for_sim_ready as gate  # noqa: E402


class TestSignalsReady(unittest.TestCase):
    def test_all_four_required(self):
        self.assertTrue(gate.signals_ready(True, True, True, True))

    def test_any_missing_is_not_ready(self):
        self.assertFalse(gate.signals_ready(False, True, True, True))  # no scan
        self.assertFalse(gate.signals_ready(True, False, True, True))  # no camera
        self.assertFalse(gate.signals_ready(True, True, False, True))  # no odom TF
        self.assertFalse(gate.signals_ready(True, True, True, False))  # no laser TF
        self.assertFalse(gate.signals_ready(False, False, False, False))

    def test_scan_alone_is_not_ready(self):
        # Guard against regressing to a scan-only gate: /scan can flow before
        # the diff-drive odom TF exists, which would start SLAM too early.
        self.assertFalse(gate.signals_ready(True, False, False, False))


class TestMissingSignalNames(unittest.TestCase):
    def test_lists_only_missing(self):
        missing = gate.missing_signal_names(
            got_scan=True, got_camera=True, odom_tf_ok=False, laser_tf_ok=True,
            odom_frame="odom", base_frame="base_footprint", laser_frame="laser_link",
            camera_topic="/zed/zed_node/left/image_rect_color")
        self.assertEqual(missing, ["TF odom->base_footprint"])

    def test_camera_missing_is_named(self):
        missing = gate.missing_signal_names(
            got_scan=True, got_camera=False, odom_tf_ok=True, laser_tf_ok=True,
            odom_frame="odom", base_frame="base_footprint", laser_frame="laser_link",
            camera_topic="/zed/zed_node/left/image_rect_color")
        self.assertEqual(missing, ["/zed/zed_node/left/image_rect_color"])

    def test_all_missing(self):
        missing = gate.missing_signal_names(
            got_scan=False, got_camera=False, odom_tf_ok=False, laser_tf_ok=False,
            odom_frame="odom", base_frame="base_footprint", laser_frame="laser_link",
            camera_topic="/zed/zed_node/left/image_rect_color")
        self.assertEqual(
            missing,
            ["/scan", "/zed/zed_node/left/image_rect_color",
             "TF odom->base_footprint", "TF base_footprint->laser_link"])

    def test_none_missing(self):
        missing = gate.missing_signal_names(
            got_scan=True, got_camera=True, odom_tf_ok=True, laser_tf_ok=True,
            odom_frame="odom", base_frame="base_footprint", laser_frame="laser_link",
            camera_topic="/zed/zed_node/left/image_rect_color")
        self.assertEqual(missing, [])


if __name__ == "__main__":
    unittest.main()
