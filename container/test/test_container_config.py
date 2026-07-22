"""Seam 2: static consistency guards. The container configuration is spread across
a script, a compose file, a colcon defaults file, ignore markers and a CI workflow,
and the failure mode when they drift is silent. These tests are the drift alarm.
They require no Docker and reach no network."""

import subprocess
import unittest
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CONTAINER_DIR = REPO_ROOT / "container"

# The hardware-only packages. Skipping them is what keeps a multi-gigabyte camera
# SDK out of an image whose job is to run a simulation. Verified safe: the sensor
# xacros are self-contained within the description packages, so no simulation
# package depends on the ZED or RealSense packages.
IGNORED_PACKAGES = [
    "zed_msgs",
    "realsense2_camera",
    "realsense2_camera_msgs",
    "realsense2_description",
    "rm_camera_demo",
    "ros2_agv_robot",
    "ros2_total_demo",
    "rm_driver",
    "woosh_action_msgs",
    "woosh_common_msgs",
    "woosh_nav_msgs",
    "woosh_robot_msgs",
    "woosh_ros_msgs",
    "woosh_task_msgs",
]

# What must remain after the ignore set is applied.
SIMULATION_PACKAGES = [
    "dual_rm_65b_moveit_config",
    "dual_rm_75b_moveit_config",
    "dual_rm_control",
    "dual_rm_description",
    "dual_rm_gazebo",
    "dual_rm_install",
    "dual_rm_moveit_demo",
    "dual_rm_navigation",
    "dual_rm_simulation",
    "r2d3_bringup",
    "r2d3_mujoco",
    "r2d3_test_nodes",
    "rm_ros_interfaces",
    "servo_interfaces",
    "servo_sim_bridge",
]


def parse_ignore_lists(text):
    """Return {verb: [package, ...]} from the colcon defaults YAML.

    Parsed with PyYAML, not a hand-rolled line scanner: the file states the ignore
    set once and aliases it into the other verbs, so anything that does not resolve
    YAML anchors would read `test` and `list` as empty and this guard would fail on
    a correct file. Writing the list out once per verb to suit a weaker parser was
    the alternative, and triplicated lists are exactly the drift this file exists to
    catch. PyYAML is already a dependency of the suite (servo_sim_bridge declares
    python3-yaml and its parity test imports it)."""
    return {
        verb: settings["packages-ignore"]
        for verb, settings in (yaml.safe_load(text) or {}).items()
        if isinstance(settings, dict) and "packages-ignore" in settings
    }


class TestColconDefaults(unittest.TestCase):
    def setUp(self):
        self.lists = parse_ignore_lists(
            (CONTAINER_DIR / "colcon-defaults.yaml").read_text())

    def test_build_and_test_verbs_both_carry_the_ignore_set(self):
        # `build` alone is not enough: a bare `colcon test` inside the container
        # would otherwise try to run the hardware packages' tests.
        for verb in ("build", "test", "list"):
            self.assertIn(verb, self.lists, f"no packages-ignore for verb '{verb}'")

    def test_ignores_exactly_the_intended_set_and_no_others(self):
        for verb, packages in self.lists.items():
            self.assertEqual(sorted(packages), sorted(IGNORED_PACKAGES), verb)


class TestPackageSelection(unittest.TestCase):
    """The mechanism is a defaults file baked into the image, deliberately NOT
    ignore-marker files dropped into the source tree: the tree is bind-mounted, so
    runtime-created markers would pollute the developer's version control status,
    and committed markers would change the HOST's build too."""

    def test_defaults_file_yields_exactly_the_simulation_subset(self):
        proc = subprocess.run(
            ["colcon", "list", "--names-only"],
            cwd=REPO_ROOT,
            env={"PATH": "/usr/bin:/bin",
                 "HOME": str(Path.home()),
                 "COLCON_DEFAULTS_FILE": str(
                     CONTAINER_DIR / "colcon-defaults.yaml")},
            capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        found = sorted(proc.stdout.split())
        self.assertEqual(found, sorted(SIMULATION_PACKAGES))

    def test_applying_the_defaults_file_writes_nothing_into_the_tree(self):
        proc = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=REPO_ROOT, capture_output=True, text=True)
        self.assertEqual(proc.stdout.strip(), "",
                         "package discovery dirtied the working tree")


if __name__ == "__main__":
    unittest.main()
