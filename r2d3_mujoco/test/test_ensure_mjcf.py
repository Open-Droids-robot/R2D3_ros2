import sys
import tempfile
import unittest
from pathlib import Path

# Import the script as a module (it lives in scripts/, not a python package)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import ensure_mjcf  # noqa: E402


class TestChecksum(unittest.TestCase):
    def test_deterministic(self):
        a = ensure_mjcf.compute_checksum("<robot/>", "<mujoco/>", "v1")
        b = ensure_mjcf.compute_checksum("<robot/>", "<mujoco/>", "v1")
        self.assertEqual(a, b)
        self.assertEqual(len(a), 64)  # sha256 hex

    def test_changes_with_any_input(self):
        base = ensure_mjcf.compute_checksum("<robot/>", "<mujoco/>", "v1")
        self.assertNotEqual(base, ensure_mjcf.compute_checksum("<robot2/>", "<mujoco/>", "v1"))
        self.assertNotEqual(base, ensure_mjcf.compute_checksum("<robot/>", "<mujoco2/>", "v1"))
        self.assertNotEqual(base, ensure_mjcf.compute_checksum("<robot/>", "<mujoco/>", "v2"))


class TestCacheValid(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cache = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_empty_cache_invalid(self):
        self.assertFalse(ensure_mjcf.cache_valid(self.cache, "abc"))

    def test_missing_mjcf_invalid(self):
        (self.cache / ensure_mjcf.CHECKSUM_FILENAME).write_text("abc")
        self.assertFalse(ensure_mjcf.cache_valid(self.cache, "abc"))

    def test_wrong_checksum_invalid(self):
        (self.cache / ensure_mjcf.MJCF_FILENAME).write_text("<mujoco/>")
        (self.cache / ensure_mjcf.CHECKSUM_FILENAME).write_text("old")
        self.assertFalse(ensure_mjcf.cache_valid(self.cache, "new"))

    def test_matching_checksum_valid(self):
        (self.cache / ensure_mjcf.MJCF_FILENAME).write_text("<mujoco/>")
        (self.cache / ensure_mjcf.CHECKSUM_FILENAME).write_text("abc\n")
        self.assertTrue(ensure_mjcf.cache_valid(self.cache, "abc"))


class TestConverterCmd(unittest.TestCase):
    def test_flags(self):
        cmd = ensure_mjcf.build_converter_cmd(
            Path("/tmp/r.urdf"), Path("/w/nav.xml"), Path("/c/65b"), "/mujoco_robot_description"
        )
        self.assertEqual(cmd[:4], ["ros2", "run", "mujoco_ros2_control", "robot_description_to_mjcf.sh"])
        self.assertIn("--save_only", cmd)
        self.assertIn("--add_free_joint", cmd)
        self.assertIn("/tmp/r.urdf", cmd)
        self.assertIn("/w/nav.xml", cmd)
        self.assertIn("/c/65b", cmd)
        # --publish_topic is passed, but to a throwaway internal topic, not the real
        # one: this is required to get an absolute <compiler meshdir=...> out of the
        # converter (see build_converter_cmd()'s docstring), not to actually publish
        # on the real topic -- ensure_mjcf.py patches the lidar housing geoms first
        # and is always the one that publishes on the real topic.
        self.assertIn("--publish_topic", cmd)
        self.assertNotIn("/mujoco_robot_description", cmd)
        self.assertIn(ensure_mjcf.internal_convert_topic("/mujoco_robot_description"), cmd)


class TestInternalConvertTopic(unittest.TestCase):
    def test_distinct_from_real_topic(self):
        real = "/mujoco_robot_description"
        internal = ensure_mjcf.internal_convert_topic(real)
        self.assertNotEqual(real, internal)
        self.assertTrue(internal.startswith(real))


class TestRaiseLidarScanPlane(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.mjcf_path = Path(self.tmp.name) / "mujoco_description_formatted.xml"

    def tearDown(self):
        self.tmp.cleanup()

    def _write(self, body: str) -> None:
        self.mjcf_path.write_text(f"<mujoco><worldbody>{body}</worldbody></mujoco>")

    def test_raises_lidar_body_z(self):
        self._write(
            '<body name="laser_link_lidar_body" pos="0.24 0 0.233" quat="0.5 0.5 -0.5 -0.5">'
            '<site name="rf"/></body>'
        )
        patched = ensure_mjcf.raise_lidar_scan_plane(self.mjcf_path)
        self.assertEqual(patched, 1)
        expected_z = round(0.233 + ensure_mjcf.LIDAR_SCAN_RAISE_M, 6)
        self.assertIn(f'pos="0.24 0 {expected_z}"', self.mjcf_path.read_text())

    def test_leaves_same_pos_non_lidar_geoms_untouched(self):
        # A geom sharing the lidar's exact pos (e.g. the housing cylinder) must NOT move:
        # only the named lidar body's pos is raised.
        self._write(
            '<geom size="0.03 0.025" pos="0.24 0 0.233" type="cylinder"/>'
            '<body name="laser_link_lidar_body" pos="0.24 0 0.233"><site name="rf"/></body>'
        )
        ensure_mjcf.raise_lidar_scan_plane(self.mjcf_path)
        text = self.mjcf_path.read_text()
        self.assertIn('<geom size="0.03 0.025" pos="0.24 0 0.233" type="cylinder"/>', text)

    def test_no_match_returns_zero(self):
        self._write('<body name="something_else" pos="0 0 0"><site name="x"/></body>')
        self.assertEqual(ensure_mjcf.raise_lidar_scan_plane(self.mjcf_path), 0)

    def test_no_write_when_nothing_matches(self):
        self._write('<body name="other" pos="0 0 0"><site name="x"/></body>')
        before_mtime = self.mjcf_path.stat().st_mtime_ns
        ensure_mjcf.raise_lidar_scan_plane(self.mjcf_path)
        self.assertEqual(before_mtime, self.mjcf_path.stat().st_mtime_ns)


# A minimal MJCF containing exactly the one lidar rangefinder body the scan-height
# patch must match (EXPECTED_PATCH_COUNT == 1).
LIDAR_BODY_MJCF = (
    '<body name="laser_link_lidar_body" pos="0.24 0 0.233" quat="0.5 0.5 -0.5 -0.5">'
    '<site name="rf"/></body>'
)


class TestValidatePatchCount(unittest.TestCase):
    def test_expected_count_is_valid(self):
        self.assertTrue(ensure_mjcf.validate_patch_count(ensure_mjcf.EXPECTED_PATCH_COUNT))

    def test_any_other_count_is_invalid(self):
        for bad in (0, ensure_mjcf.EXPECTED_PATCH_COUNT + 1, ensure_mjcf.EXPECTED_PATCH_COUNT + 2):
            self.assertFalse(ensure_mjcf.validate_patch_count(bad), bad)


# One drive wheel and one caster body in the format the wheel patch must match.
WHEELS_MJCF = (
    '<body name="link_left_wheel" pos="-0.015 0.148 0.08" quat="0.707107 0 0 0.707107">'
    '<geom type="mesh" mesh="link_left_wheel" class="collision"/>'
    '<geom type="mesh" rgba="0.79 0.82 0.93 1" mesh="link_left_wheel" class="visual"/></body>'
    '<body name="link_swivel_wheel_1_2" pos="-0.0035 -0.022 -0.045">'
    '<geom type="mesh" mesh="link_swivel_wheel_1_2" class="collision"/>'
    '<geom type="mesh" rgba="0.79 0.82 0.93 1" mesh="link_swivel_wheel_1_2" class="visual"/></body>'
)


# Minimal URDF: base_footprint (no inertial of its own) with two symmetric fixed-jointed
# children. Total mass 4 kg, COM at origin, and a hand-verifiable merged inertia:
#   link_a: 2 kg at ( 1,0,0), I=diag(0.1)   link_b: 2 kg at (-1,0,0), I=diag(0.1)
#   -> com=(0,0,0); fullinertia=(0.2, 4.2, 4.2, 0, 0, 0)  (parallel-axis about COM)
MINIMAL_URDF = """<?xml version="1.0"?>
<robot name="mini">
  <link name="base_footprint"/>
  <link name="link_a">
    <inertial>
      <origin xyz="1 0 0" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="link_b">
    <inertial>
      <origin xyz="-1 0 0" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="j_a" type="fixed">
    <parent link="base_footprint"/><child link="link_a"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="j_b" type="fixed">
    <parent link="base_footprint"/><child link="link_b"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>"""

# base_footprint body as the converter emits it: no <inertial>, one nested child body.
BASE_FOOTPRINT_MJCF = (
    '<body name="base_footprint" pos="0 0 0">'
    '<geom type="mesh" mesh="base_footprint" class="collision"/>'
    '<body name="child_link" pos="0 0 0.1"><geom type="mesh" mesh="child"/></body>'
    '</body>'
)


class TestComputeMergedBaseInertial(unittest.TestCase):
    def test_merges_symmetric_children(self):
        result = ensure_mjcf.compute_merged_base_inertial(MINIMAL_URDF)
        self.assertIsNotNone(result)
        mass, com, I = result
        self.assertAlmostEqual(mass, 4.0, places=6)
        for got, exp in zip(com, (0.0, 0.0, 0.0)):
            self.assertAlmostEqual(got, exp, places=6)
        for got, exp in zip(I, (0.2, 4.2, 4.2, 0.0, 0.0, 0.0)):
            self.assertAlmostEqual(got, exp, places=6)

    def test_no_attached_mass_returns_none(self):
        urdf = '<?xml version="1.0"?><robot name="empty"><link name="base_footprint"/></robot>'
        self.assertIsNone(ensure_mjcf.compute_merged_base_inertial(urdf))


class TestInjectBaseFootprintInertial(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.mjcf = Path(self.tmp.name) / "out.xml"

    def tearDown(self):
        self.tmp.cleanup()

    def _write(self, body):
        self.mjcf.write_text(f"<mujoco><worldbody>{body}</worldbody></mujoco>")

    def test_injects_inertial_once(self):
        self._write(BASE_FOOTPRINT_MJCF)
        self.assertTrue(ensure_mjcf.inject_base_footprint_inertial(self.mjcf, MINIMAL_URDF))
        text = self.mjcf.read_text()
        self.assertIn('<inertial pos="0.000000 0.000000 0.000000" mass="4.000000"', text)
        self.assertIn('fullinertia="0.200000 4.200000 4.200000 0.000000 0.000000 0.000000"', text)
        # inertial must sit inside base_footprint, before its nested child body
        base_start = text.index('<body name="base_footprint"')
        child_start = text.index('<body name="child_link"')
        self.assertLess(text.index("<inertial", base_start), child_start)

    def test_refuses_double_injection(self):
        self._write(BASE_FOOTPRINT_MJCF)
        self.assertTrue(ensure_mjcf.inject_base_footprint_inertial(self.mjcf, MINIMAL_URDF))
        # A second pass sees the existing <inertial> and must refuse (no double count).
        self.assertFalse(ensure_mjcf.inject_base_footprint_inertial(self.mjcf, MINIMAL_URDF))

    def test_missing_base_footprint_returns_false(self):
        self._write('<body name="other"><geom type="mesh" mesh="x"/></body>')
        self.assertFalse(ensure_mjcf.inject_base_footprint_inertial(self.mjcf, MINIMAL_URDF))


class TestReplaceWheelCollision(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.mjcf_path = Path(self.tmp.name) / "mujoco_description_formatted.xml"

    def tearDown(self):
        self.tmp.cleanup()

    def _write(self, body):
        self.mjcf_path.write_text(f"<mujoco><worldbody>{body}</worldbody></mujoco>")

    def test_full_wheel_set_patches_all(self):
        # drive wheels + casters (primitive + hull-disable each) + brackets (hull-disable)
        # == EXPECTED_WHEEL_PATCH_COUNT.
        all_bodies = (
            ensure_mjcf._DRIVE_WHEEL_BODIES
            + ensure_mjcf._CASTER_WHEEL_BODIES
            + ensure_mjcf._CASTER_BRACKET_BODIES
        )
        bodies = "".join(
            f'<body name="{b}" pos="0 0 0.08" quat="0.707107 0 0 0.707107">'
            f'<geom type="mesh" mesh="{b}" class="collision"/></body>'
            for b in all_bodies
        )
        self._write(bodies)
        patched = ensure_mjcf.replace_wheel_collision_with_primitives(self.mjcf_path)
        self.assertEqual(patched, ensure_mjcf.EXPECTED_WHEEL_PATCH_COUNT)
        self.assertTrue(ensure_mjcf.validate_wheel_patch_count(patched))

    def test_brackets_get_hull_disabled_no_primitive(self):
        b = ensure_mjcf._CASTER_BRACKET_BODIES[0]
        self._write(f'<body name="{b}" pos="0 0 0.05"><geom type="mesh" mesh="{b}" class="collision"/></body>')
        ensure_mjcf.replace_wheel_collision_with_primitives(self.mjcf_path)
        text = self.mjcf_path.read_text()
        self.assertIn(f'<geom type="mesh" mesh="{b}" class="collision" contype="0"/>', text)
        self.assertNotIn('type="sphere"', text)  # brackets get no primitive
        self.assertNotIn('type="cylinder"', text)

    def test_drive_gets_cylinder_caster_gets_sphere_and_hull_disabled(self):
        self._write(WHEELS_MJCF)
        ensure_mjcf.replace_wheel_collision_with_primitives(self.mjcf_path)
        text = self.mjcf_path.read_text()
        # drive wheel: cylinder inserted, hull collision disabled
        self.assertIn('<geom type="cylinder"', text)
        self.assertIn('<geom type="sphere"', text)
        self.assertIn('<geom type="mesh" mesh="link_left_wheel" class="collision" contype="0"/>', text)
        self.assertIn('<geom type="mesh" mesh="link_swivel_wheel_1_2" class="collision" contype="0"/>', text)
        # the visual mesh geoms are left untouched (still rendered)
        self.assertIn('rgba="0.79 0.82 0.93 1" mesh="link_left_wheel" class="visual"', text)

    def test_partial_wheel_set_fails_validation(self):
        # Only the drive wheels present (e.g. upstream renamed the casters).
        bodies = "".join(
            f'<body name="{b}" pos="0 0 0.08">'
            f'<geom type="mesh" mesh="{b}" class="collision"/></body>'
            for b in ensure_mjcf._DRIVE_WHEEL_BODIES
        )
        self._write(bodies)
        patched = ensure_mjcf.replace_wheel_collision_with_primitives(self.mjcf_path)
        self.assertFalse(ensure_mjcf.validate_wheel_patch_count(patched))


class TestXmlValidity(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "out.xml"

    def tearDown(self):
        self.tmp.cleanup()

    def test_wellformed_xml_accepted(self):
        self.path.write_text("<mujoco><worldbody/></mujoco>")
        self.assertTrue(ensure_mjcf.mjcf_parses_as_xml(self.path))

    def test_truncated_xml_rejected(self):
        self.path.write_text("<mujoco><worldbody><geom type=")
        self.assertFalse(ensure_mjcf.mjcf_parses_as_xml(self.path))

    def test_missing_file_rejected(self):
        self.assertFalse(ensure_mjcf.mjcf_parses_as_xml(self.path))


class TestConverterExitAcceptable(unittest.TestCase):
    """Early converter exits are only trusted on clean exit + valid XML output."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "out.xml"

    def tearDown(self):
        self.tmp.cleanup()

    def test_clean_exit_with_valid_xml_accepted(self):
        self.path.write_text("<mujoco/>")
        self.assertTrue(ensure_mjcf.converter_exit_acceptable(0, self.path))

    def test_nonzero_exit_rejected_even_with_valid_xml(self):
        self.path.write_text("<mujoco/>")
        self.assertFalse(ensure_mjcf.converter_exit_acceptable(1, self.path))
        self.assertFalse(ensure_mjcf.converter_exit_acceptable(-15, self.path))

    def test_clean_exit_with_truncated_xml_rejected(self):
        self.path.write_text("<mujoco><geom ")
        self.assertFalse(ensure_mjcf.converter_exit_acceptable(0, self.path))

    def test_clean_exit_with_missing_or_empty_file_rejected(self):
        self.assertFalse(ensure_mjcf.converter_exit_acceptable(0, self.path))
        self.path.write_text("")
        self.assertFalse(ensure_mjcf.converter_exit_acceptable(0, self.path))


class TestFinalizeConversion(unittest.TestCase):
    """Hard-failure gate for the cache-miss path: no checksum written on any failure,
    so the cache entry stays invalid and the next launch reconverts."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cache = Path(self.tmp.name)
        self.mjcf = self.cache / ensure_mjcf.MJCF_FILENAME
        self.checksum_file = self.cache / ensure_mjcf.CHECKSUM_FILENAME

    def tearDown(self):
        self.tmp.cleanup()

    def _full_worldbody(self):
        wheels = "".join(
            f'<body name="{b}" pos="0 0 0.08" quat="0.707107 0 0 0.707107">'
            f'<geom type="mesh" mesh="{b}" class="collision"/></body>'
            for b in (
                ensure_mjcf._DRIVE_WHEEL_BODIES
                + ensure_mjcf._CASTER_WHEEL_BODIES
                + ensure_mjcf._CASTER_BRACKET_BODIES
            )
        )
        return f"{BASE_FOOTPRINT_MJCF}{LIDAR_BODY_MJCF}{wheels}"

    def test_success_patches_and_writes_checksum(self):
        self.mjcf.write_text(f"<mujoco><worldbody>{self._full_worldbody()}</worldbody></mujoco>")
        self.assertTrue(ensure_mjcf.finalize_conversion(self.mjcf, self.cache, "abc", MINIMAL_URDF))
        self.assertEqual(self.checksum_file.read_text().strip(), "abc")
        text = self.mjcf.read_text()
        expected_z = round(0.233 + ensure_mjcf.LIDAR_SCAN_RAISE_M, 6)
        self.assertIn(f'pos="0.24 0 {expected_z}"', text)
        self.assertIn('<geom type="cylinder"', text)
        self.assertIn('<geom type="sphere"', text)
        self.assertIn('mass="4.000000"', text)

    def test_missing_base_inertial_fails_hard_without_checksum(self):
        # Full wheel + lidar set but no base_footprint body -> inject step must fail hard.
        wheels = "".join(
            f'<body name="{b}" pos="0 0 0.08" quat="0.707107 0 0 0.707107">'
            f'<geom type="mesh" mesh="{b}" class="collision"/></body>'
            for b in (
                ensure_mjcf._DRIVE_WHEEL_BODIES
                + ensure_mjcf._CASTER_WHEEL_BODIES
                + ensure_mjcf._CASTER_BRACKET_BODIES
            )
        )
        self.mjcf.write_text(f"<mujoco><worldbody>{LIDAR_BODY_MJCF}{wheels}</worldbody></mujoco>")
        self.assertFalse(ensure_mjcf.finalize_conversion(self.mjcf, self.cache, "abc", MINIMAL_URDF))
        self.assertFalse(self.checksum_file.exists())

    def test_patch_count_mismatch_fails_hard_without_checksum(self):
        # Well-formed XML, but no lidar rangefinder body present:
        # simulates the converter output no longer matching the patch pattern.
        self.mjcf.write_text('<mujoco><worldbody><geom type="mesh" mesh="l_link1"/></worldbody></mujoco>')
        self.assertFalse(ensure_mjcf.finalize_conversion(self.mjcf, self.cache, "abc", MINIMAL_URDF))
        self.assertFalse(self.checksum_file.exists())

    def test_multiple_lidar_bodies_fails_hard_without_checksum(self):
        # More than one lidar body (unexpected upstream change) must fail hard.
        self.mjcf.write_text(f"<mujoco><worldbody>{LIDAR_BODY_MJCF}{LIDAR_BODY_MJCF}</worldbody></mujoco>")
        self.assertFalse(ensure_mjcf.finalize_conversion(self.mjcf, self.cache, "abc", MINIMAL_URDF))
        self.assertFalse(self.checksum_file.exists())

    def test_invalid_xml_fails_hard_without_checksum(self):
        self.mjcf.write_text("<mujoco><worldbody><geom ")
        self.assertFalse(ensure_mjcf.finalize_conversion(self.mjcf, self.cache, "abc", MINIMAL_URDF))
        self.assertFalse(self.checksum_file.exists())

    def test_missing_file_fails_hard_without_checksum(self):
        self.assertFalse(ensure_mjcf.finalize_conversion(self.mjcf, self.cache, "abc", MINIMAL_URDF))
        self.assertFalse(self.checksum_file.exists())


if __name__ == "__main__":
    unittest.main()
