"""The seeded DEFAULT_CONFIG (used for pure-unit tests and as an uninstalled
fallback) and the shipped config/neck_servo_bridge.yaml (the overridable file the
node loads when installed) both carry the calibration constants. This test locks
them together so a change to one that isn't mirrored in the other is caught,
rather than silently drifting."""

import unittest
from pathlib import Path

import yaml

from servo_sim_bridge.calibration import DEFAULT_CONFIG, NeckCalibration

CONFIG_YAML = Path(__file__).resolve().parent.parent / "config" / "neck_servo_bridge.yaml"


class TestConfigParity(unittest.TestCase):
    def setUp(self):
        self.from_yaml = NeckCalibration.from_config(yaml.safe_load(CONFIG_YAML.read_text()))
        self.from_default = NeckCalibration.from_config(DEFAULT_CONFIG)

    def test_layout_matches(self):
        self.assertEqual(self.from_yaml.readback_order, self.from_default.readback_order)
        self.assertEqual(self.from_yaml.num_joints, self.from_default.num_joints)

    def test_mappings_match_across_full_band(self):
        for servo_id in DEFAULT_CONFIG["servos"]:
            for units in (200, 350, 500, 650, 800):
                self.assertAlmostEqual(
                    self.from_yaml.units_to_rad(servo_id, units),
                    self.from_default.units_to_rad(servo_id, units),
                    places=9,
                    msg=f"servo {servo_id} at {units} units",
                )


if __name__ == "__main__":
    unittest.main()
