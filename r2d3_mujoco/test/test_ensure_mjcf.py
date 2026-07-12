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
        self.assertIn("/mujoco_robot_description", cmd)


if __name__ == "__main__":
    unittest.main()
