"""Seam 2: static consistency guards. The container configuration is spread across
a script, a compose file, a colcon defaults file, ignore markers and a CI workflow,
and the failure mode when they drift is silent. These tests are the drift alarm.
They require no Docker and reach no network."""

import re
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

    def test_defaults_file_yields_the_simulation_subset_without_dirtying_the_tree(self):
        # The two assertions are deliberately in ONE test. Split across two,
        # unittest's alphabetical ordering ran the git-status one first -- before
        # the only action that could have dirtied anything -- so it asserted about a
        # tree nothing had touched yet. Here the git status is observed strictly
        # after the colcon invocation, which is the only arrangement that tests the
        # claim: package discovery must not write COLCON_IGNORE markers or any other
        # state into the bind-mounted checkout.
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

        status = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=REPO_ROOT, capture_output=True, text=True)
        self.assertEqual(status.stdout.strip(), "",
                         "package discovery dirtied the working tree")


class TestDockerfileConsumesTheDefaultsFile(unittest.TestCase):
    """Ties the validated YAML to the image that consumes it.

    Everything above validates `container/colcon-defaults.yaml` in isolation. That
    leaves a hole: change the COPY destination, or drop the ENV, and the whole
    suite stays green while the container happily builds all 29 packages and pulls
    the ZED and RealSense SDKs back in. This test closes it by reading the
    Dockerfile and checking the two ends actually meet.
    """

    def setUp(self):
        self.dockerfile = (CONTAINER_DIR / "Dockerfile").read_text()

    def _copy_destination(self):
        match = re.search(
            r"^COPY\s+container/colcon-defaults\.yaml\s+(\S+)\s*$",
            self.dockerfile, re.MULTILINE)
        self.assertIsNotNone(
            match, "the Dockerfile no longer COPYs container/colcon-defaults.yaml")
        return match.group(1)

    def _defaults_file_env(self):
        match = re.search(
            r"^ENV\s+COLCON_DEFAULTS_FILE=(\S+)\s*$", self.dockerfile, re.MULTILINE)
        self.assertIsNotNone(
            match, "the Dockerfile no longer sets ENV COLCON_DEFAULTS_FILE")
        return match.group(1)

    def test_the_copied_defaults_file_is_the_one_colcon_is_told_to_read(self):
        self.assertEqual(self._copy_destination(), self._defaults_file_env())


if __name__ == "__main__":
    unittest.main()
