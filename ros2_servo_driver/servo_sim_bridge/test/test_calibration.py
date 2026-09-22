import math
import unittest

from servo_sim_bridge.calibration import NeckCalibration, DEFAULT_CONFIG


# Expected radians recomputed independently from the real-robot calibration the
# spec seeds from (issue #5/#7), so a regression in calibration.py is caught
# rather than mirrored:
#   pan  (id 2 -> head_joint1): rad = (units-500) * (60/300)   * pi/180
#   tilt (id 5 -> head_joint2): rad = ((units-500)*(70.1/300) - 7.766) * pi/180
def pan_rad(units):
    return (units - 500) * (60.0 / 300.0) * math.pi / 180.0


def tilt_rad(units):
    return ((units - 500) * (70.1 / 300.0) - 7.766) * math.pi / 180.0


class TestForwardMapping(unittest.TestCase):
    def setUp(self):
        self.cal = NeckCalibration.from_config(DEFAULT_CONFIG)

    def test_pan_min_center_max(self):
        for units in (200, 500, 800):
            self.assertAlmostEqual(self.cal.units_to_rad(2, units), pan_rad(units), places=9)

    def test_tilt_min_center_max(self):
        for units in (200, 500, 800):
            self.assertAlmostEqual(self.cal.units_to_rad(5, units), tilt_rad(units), places=9)

    def test_tilt_center_offset_honoured(self):
        # At 500 units the tilt is NOT zero: the real neutral sits at -7.766 deg.
        self.assertAlmostEqual(self.cal.units_to_rad(5, 500), math.radians(-7.766), places=9)
        self.assertNotAlmostEqual(self.cal.units_to_rad(5, 500), 0.0, places=3)

    def test_tilt_full_range_matches_widened_urdf_limits(self):
        # The band edges must line up with the widened head_joint2 URDF limits.
        self.assertAlmostEqual(self.cal.units_to_rad(5, 200), -1.359, places=3)
        self.assertAlmostEqual(self.cal.units_to_rad(5, 800), 1.088, places=3)


class TestClamping(unittest.TestCase):
    def setUp(self):
        self.cal = NeckCalibration.from_config(DEFAULT_CONFIG)

    def test_below_band_saturates_to_min(self):
        for sid in (2, 5):
            at_min = self.cal.units_to_rad(sid, 200)
            self.assertAlmostEqual(self.cal.units_to_rad(sid, 100), at_min, places=12)
            self.assertAlmostEqual(self.cal.units_to_rad(sid, 0), at_min, places=12)

    def test_above_band_saturates_to_max(self):
        for sid in (2, 5):
            at_max = self.cal.units_to_rad(sid, 800)
            self.assertAlmostEqual(self.cal.units_to_rad(sid, 900), at_max, places=12)
            self.assertAlmostEqual(self.cal.units_to_rad(sid, 1000), at_max, places=12)


class TestRoundTrip(unittest.TestCase):
    """units -> rad -> units identity within tolerance validates the read-back inverse."""

    def setUp(self):
        self.cal = NeckCalibration.from_config(DEFAULT_CONFIG)

    def test_roundtrip_in_band(self):
        for sid in (2, 5):
            for units in (200, 275, 350, 500, 620, 725, 800):
                rad = self.cal.units_to_rad(sid, units)
                self.assertAlmostEqual(self.cal.rad_to_units(sid, rad), units, places=6)

    def test_rad_to_units_clamps_out_of_range_pose(self):
        # A pose beyond the usable band inverts to a clamped, in-band unit value.
        big = self.cal.units_to_rad(5, 800) + 1.0
        self.assertLessEqual(self.cal.rad_to_units(5, big), 800)
        self.assertGreaterEqual(self.cal.rad_to_units(5, big), 200)


class TestCommandFor(unittest.TestCase):
    def setUp(self):
        self.cal = NeckCalibration.from_config(DEFAULT_CONFIG)

    def test_pan_targets_joint_index_0(self):
        idx, rad = self.cal.command_for(2, 800)
        self.assertEqual(idx, 0)
        self.assertAlmostEqual(rad, pan_rad(800), places=9)

    def test_tilt_targets_joint_index_1(self):
        idx, rad = self.cal.command_for(5, 200)
        self.assertEqual(idx, 1)
        self.assertAlmostEqual(rad, tilt_rad(200), places=9)

    def test_unknown_servo_id_produces_no_command(self):
        for sid in (0, 1, 3, 4, 99, 255):
            self.assertIsNone(self.cal.command_for(sid, 500))


class TestReadback(unittest.TestCase):
    """Read-back array is ordered [id5 tilt units, id2 pan units] to match the real node."""

    def setUp(self):
        self.cal = NeckCalibration.from_config(DEFAULT_CONFIG)

    def test_readback_order_is_tilt_then_pan(self):
        self.assertEqual(self.cal.readback_order, [5, 2])

    def test_readback_units_from_positions(self):
        positions = {"head_joint1": pan_rad(725), "head_joint2": tilt_rad(350)}
        tilt_units, pan_units = self.cal.readback_units(positions)
        self.assertAlmostEqual(tilt_units, 350, places=6)
        self.assertAlmostEqual(pan_units, 725, places=6)

    def test_readback_missing_joint_omitted_or_none(self):
        # Missing joint state must not raise; the entry falls back to the center unit.
        out = self.cal.readback_units({})
        self.assertEqual(len(out), 2)


class TestConfigOverride(unittest.TestCase):
    def test_string_keyed_servo_ids_are_accepted(self):
        # YAML round-trips servo ids as strings; from_config must normalise them.
        cfg = {
            "servos": {
                "2": {"joint_name": "head_joint1", "joint_index": 0, "center_units": 500,
                      "deg_per_unit": 0.2, "offset_deg": 0.0, "min_units": 200, "max_units": 800},
            },
            "readback_order": [2],
            "num_joints": 1,
        }
        cal = NeckCalibration.from_config(cfg)
        self.assertIsNotNone(cal.command_for(2, 800))

    def test_num_joints_exposed(self):
        self.assertEqual(NeckCalibration.from_config(DEFAULT_CONFIG).num_joints, 2)


if __name__ == "__main__":
    unittest.main()
