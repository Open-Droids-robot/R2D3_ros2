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


class TestPatchLidarHousingVisibility(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.mjcf_path = Path(self.tmp.name) / "mujoco_description_formatted.xml"

    def tearDown(self):
        self.tmp.cleanup()

    def _write(self, body: str) -> None:
        self.mjcf_path.write_text(f"<mujoco><worldbody>{body}</worldbody></mujoco>")

    def test_patches_both_collision_and_visual_housing_geoms(self):
        self._write(
            '<geom size="0.03 0.025" pos="0.24 0 0.233" type="cylinder" class="collision"/>'
            '<geom size="0.03 0.025" pos="0.24 0 0.233" type="cylinder" rgba="0.1 0.1 0.1 1" class="visual"/>'
        )
        patched = ensure_mjcf.patch_lidar_housing_visibility(self.mjcf_path)
        self.assertEqual(patched, 2)
        text = self.mjcf_path.read_text()
        self.assertIn('rgba="1 1 1 0"', text)  # collision geom: rgba added
        self.assertIn('rgba="0.1 0.1 0.1 0"', text)  # visual geom: alpha zeroed, color kept

    def test_patches_occluding_chassis_meshes_by_mesh_attribute(self):
        self._write(
            '<geom type="mesh" mesh="base_link_underpan" class="collision"/>'
            '<geom type="mesh" rgba="0.79216 0.81961 0.93333 1" mesh="base_link_underpan" class="visual"/>'
            '<geom type="mesh" mesh="body_base_link" class="collision"/>'
            '<geom type="mesh" rgba="0.79216 0.81961 0.93333 1" mesh="body_base_link" class="visual"/>'
        )
        patched = ensure_mjcf.patch_lidar_housing_visibility(self.mjcf_path)
        self.assertEqual(patched, 4)
        text = self.mjcf_path.read_text()
        self.assertIn('rgba="0.79216 0.81961 0.93333 0"', text)

    def test_leaves_other_cylinder_geoms_untouched(self):
        self._write('<geom size="0.4 0.5" pos="-1 3 0.5" type="cylinder" name="cylinder1" rgba="0.2 0.8 0.2 1"/>')
        patched = ensure_mjcf.patch_lidar_housing_visibility(self.mjcf_path)
        self.assertEqual(patched, 0)
        self.assertIn('rgba="0.2 0.8 0.2 1"', self.mjcf_path.read_text())

    def test_leaves_unrelated_mesh_geoms_untouched(self):
        self._write('<geom type="mesh" mesh="l_link1" class="collision"/>')
        patched = ensure_mjcf.patch_lidar_housing_visibility(self.mjcf_path)
        self.assertEqual(patched, 0)

    def test_no_write_when_nothing_matches(self):
        self._write('<geom type="mesh" mesh="l_link1" class="collision"/>')
        before_mtime = self.mjcf_path.stat().st_mtime_ns
        ensure_mjcf.patch_lidar_housing_visibility(self.mjcf_path)
        self.assertEqual(before_mtime, self.mjcf_path.stat().st_mtime_ns)


if __name__ == "__main__":
    unittest.main()
