import sys
import unittest
from pathlib import Path

# Import the script as a module (it lives in scripts/, not a python package)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import wait_for_sim_ready as gate  # noqa: E402


class TestSignalsReady(unittest.TestCase):
    def test_all_three_required(self):
        # Ready only when scan + both TF links are present.
        self.assertTrue(gate.signals_ready(True, True, True))

    def test_any_missing_is_not_ready(self):
        self.assertFalse(gate.signals_ready(False, True, True))   # no scan
        self.assertFalse(gate.signals_ready(True, False, True))   # no odom TF
        self.assertFalse(gate.signals_ready(True, True, False))   # no laser TF
        self.assertFalse(gate.signals_ready(False, False, False))

    def test_scan_alone_is_not_ready(self):
        # Guard against regressing to a scan-only gate: /scan can flow before
        # the diff-drive odom TF exists, which would start SLAM too early.
        self.assertFalse(gate.signals_ready(True, False, False))


class TestMissingSignalNames(unittest.TestCase):
    def test_lists_only_missing(self):
        missing = gate.missing_signal_names(
            got_scan=True, odom_tf_ok=False, laser_tf_ok=True,
            odom_frame="odom", base_frame="base_footprint", laser_frame="laser_link")
        self.assertEqual(missing, ["TF odom->base_footprint"])

    def test_all_missing(self):
        missing = gate.missing_signal_names(
            got_scan=False, odom_tf_ok=False, laser_tf_ok=False,
            odom_frame="odom", base_frame="base_footprint", laser_frame="laser_link")
        self.assertEqual(
            missing,
            ["/scan", "TF odom->base_footprint", "TF base_footprint->laser_link"])

    def test_none_missing(self):
        missing = gate.missing_signal_names(
            got_scan=True, odom_tf_ok=True, laser_tf_ok=True,
            odom_frame="odom", base_frame="base_footprint", laser_frame="laser_link")
        self.assertEqual(missing, [])


if __name__ == "__main__":
    unittest.main()
