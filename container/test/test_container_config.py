"""Seam 2: static consistency guards. The container configuration is spread across
a script, a compose file, a colcon defaults file, ignore markers and a CI workflow,
and the failure mode when they drift is silent. These tests are the drift alarm.
They require no Docker and reach no network."""

import json
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


class TestDisplayIsProvidedToExecSessions(unittest.TestCase):
    """The GUI-launching command runs via `docker compose exec`, which inherits
    the image's ENV but NOT the runtime `export DISPLAY` that gui-start.sh makes
    for PID 1. So the display the entrypoint's Xvfb creates must ALSO be declared
    as image ENV, or Gazebo's Qt/xcb GUI finds no display and the whole `gz sim`
    process aborts on launch -- taking the server, /clock and controller_manager
    with it. (Found in hand verification: without the ENV, `./droid up` crashed
    with 'no Qt platform plugin could be initialized' and /clock never advanced.)

    This is a cross-file drift guard, not a self-consistency check: the two values
    come from different files created by different mechanisms -- the Xvfb display
    gui-start.sh actually starts, and the DISPLAY the Dockerfile hands to exec
    sessions. If they drift, the GUI silently fails to render.
    """

    def setUp(self):
        self.dockerfile = (CONTAINER_DIR / "Dockerfile").read_text()
        self.gui_start = (CONTAINER_DIR / "gui-start.sh").read_text()

    def _dockerfile_display(self):
        # Match a standalone `ENV DISPLAY=:1`, not the multi-var ENV block.
        match = re.search(
            r"^ENV\s+DISPLAY=(\S+)\s*$", self.dockerfile, re.MULTILINE)
        self.assertIsNotNone(
            match, "the Dockerfile does not declare ENV DISPLAY -- exec sessions "
                   "(launch-sim.sh, ./droid shell) will have no display and the "
                   "Gazebo GUI will abort")
        return match.group(1)

    def _xvfb_display(self):
        # The display gui-start.sh actually brings up: `Xvfb "$DISPLAY" ...`,
        # with `export DISPLAY=":1"` above it.
        match = re.search(r'export\s+DISPLAY="?(:[0-9]+)"?', self.gui_start)
        self.assertIsNotNone(
            match, "could not find gui-start.sh's DISPLAY assignment")
        self.assertIn(
            'Xvfb "$DISPLAY"', self.gui_start,
            "gui-start.sh no longer starts Xvfb on $DISPLAY")
        return match.group(1)

    def test_dockerfile_display_matches_the_xvfb_display(self):
        self.assertEqual(self._dockerfile_display(), self._xvfb_display())


class TestPrewarmXacroArgsMatchLaunchFile(unittest.TestCase):
    """`container/prewarm-mjcf.py` hand-composes the xacro arguments it feeds
    `Command(...)` rather than importing them from the launch file (it cannot: the
    launch file builds them from `LaunchConfiguration` values resolved at launch
    time). Its final checksum assertion is self-consistent by construction -- both
    `expected` and `stored` derive from the SAME hand-composed description -- so it
    cannot detect that the hand-composed argument list has drifted from
    `mujoco_sim.launch.py`'s. Concretely: ADDING an argument to either file's
    `Command([...])`, or RENAMING one side's `name:=` mapping, slips straight
    through that assertion (xacro silently defaults an unknown/missing arg) and
    ships a build-time cache under a key the launch will never ask for. This test
    is the guard for exactly that gap: it parses both `Command([...])` argument
    lists and asserts the ordered `name:=` tokens, and the xacro file basename,
    match."""

    LAUNCH_PATH = REPO_ROOT / "r2d3_mujoco" / "launch" / "mujoco_sim.launch.py"
    PREWARM_PATH = CONTAINER_DIR / "prewarm-mjcf.py"

    # Matches the actual `Command([FindExecutable(...), ...])` construction
    # (non-greedy, DOTALL), anchored on `FindExecutable` so a mention of
    # `Command([...])` in prose (e.g. a docstring) is not mistaken for it.
    COMMAND_BLOCK_RE = re.compile(r"Command\(\[\s*FindExecutable\(.*?\)(.*?)\]\)", re.DOTALL)
    # Matches a quoted xacro-argument token like `" arm_model:="` -> "arm_model".
    ARG_TOKEN_RE = re.compile(r'"\s+([A-Za-z0-9_]+):="')
    # Matches the xacro file basename wherever it appears as a string literal.
    XACRO_BASENAME_RE = re.compile(r'"([A-Za-z0-9_./]+\.urdf\.xacro)"')

    def _command_block(self, text, path):
        match = self.COMMAND_BLOCK_RE.search(text)
        self.assertIsNotNone(match, f"no Command([...]) construction found in {path}")
        return match.group(1)

    def _xacro_args(self, text, path):
        return self.ARG_TOKEN_RE.findall(self._command_block(text, path))

    def _xacro_basename(self, text, path):
        match = self.XACRO_BASENAME_RE.search(text)
        self.assertIsNotNone(match, f"no *.urdf.xacro literal found in {path}")
        return Path(match.group(1)).name

    def setUp(self):
        self.launch_text = self.LAUNCH_PATH.read_text()
        self.prewarm_text = self.PREWARM_PATH.read_text()

    def test_xacro_argument_names_and_order_match(self):
        launch_args = self._xacro_args(self.launch_text, self.LAUNCH_PATH)
        prewarm_args = self._xacro_args(self.prewarm_text, self.PREWARM_PATH)
        self.assertEqual(
            prewarm_args, launch_args,
            "container/prewarm-mjcf.py's hand-composed xacro arguments have "
            "drifted from mujoco_sim.launch.py's Command([...]) -- the pre-warmed "
            "cache would be keyed on a description the launch never produces")

    def test_xacro_file_basename_matches(self):
        launch_basename = self._xacro_basename(self.launch_text, self.LAUNCH_PATH)
        prewarm_basename = self._xacro_basename(self.prewarm_text, self.PREWARM_PATH)
        self.assertEqual(prewarm_basename, launch_basename)


class TestImageReferenceParity(unittest.TestCase):
    """The entry point prints and reasons about an image the compose file is the
    one actually pulling. If they drift, `droid` reports success against an image
    nobody is running."""

    def test_droid_and_compose_name_the_same_image(self):
        droid = (REPO_ROOT / "droid").read_text()
        compose = (CONTAINER_DIR / "docker-compose.yml").read_text()
        ref = re.search(r'IMAGE_REF="([^"]+)"', droid).group(1)
        # EQUALITY, not substring containment: `assertIn(ref, compose)` would also
        # pass if the compose file's image tag were e.g. "...:jazzy2" -- a tag typo
        # is exactly the drift this class exists to catch, and substring containment
        # lets it through silently.
        compose_match = re.search(r'^\s*image:\s*(\S+)\s*$', compose, re.MULTILINE)
        self.assertIsNotNone(
            compose_match, "no 'image:' line found in container/docker-compose.yml")
        self.assertEqual(ref, compose_match.group(1))
        self.assertEqual(ref, "ghcr.io/open-droids-robot/r2d3-sim:jazzy")


class TestSecretsHygiene(unittest.TestCase):
    """The old Docker/.env was committed and uncovered by ignore rules. It held no
    secrets, but it invited them."""

    def setUp(self):
        self.tracked = subprocess.run(
            ["git", "ls-files"], cwd=REPO_ROOT,
            capture_output=True, text=True).stdout.split()

    def test_env_file_is_git_ignored(self):
        proc = subprocess.run(
            ["git", "check-ignore", "-q", "container/.env"],
            cwd=REPO_ROOT)
        self.assertEqual(proc.returncode, 0, "container/.env is not git-ignored")

    def test_example_counterpart_is_tracked(self):
        self.assertIn("container/env.example", self.tracked)

    def test_no_env_file_is_tracked_anywhere(self):
        offenders = [p for p in self.tracked if Path(p).name == ".env"]
        self.assertEqual(offenders, [])


class TestDevContainer(unittest.TestCase):
    def test_targets_the_same_compose_service(self):
        text = (REPO_ROOT / ".devcontainer" / "devcontainer.json").read_text()
        self.assertIn("../container/docker-compose.yml", text)
        self.assertIn('"service": "sim"', text)

    def test_remote_user_is_droid_not_root(self):
        # The image's final USER is root, and `docker exec` does not run the
        # entrypoint's uid-remapping -- so an editor attaching as root would create
        # root-owned files in the developer's bind-mounted source tree. That's a
        # binding safety constraint, so it needs a guard, not just the service/path
        # checks above.
        #
        # `.devcontainer/devcontainer.json` has no comments today, so plain
        # `json.load` parses it (verified: see task-5-report.md). Using it instead
        # of substring matching means a reformatted or reordered file can't produce
        # a false pass (e.g. `"remoteUser": "root"  // droid` would satisfy
        # assertIn) or a false failure from whitespace/quoting changes.
        text = (REPO_ROOT / ".devcontainer" / "devcontainer.json").read_text()
        config = json.loads(text)
        self.assertEqual(config.get("remoteUser"), "droid")


class TestCiArchitectureParity(unittest.TestCase):
    """CI is the only thing giving arm64 any coverage at all. If the architectures
    it builds drift from the ones the compose file and the docs promise, arm64
    becomes assumed rather than proven."""

    def setUp(self):
        self.workflow = (
            REPO_ROOT / ".github" / "workflows" / "container.yml").read_text()

    def test_builds_exactly_the_promised_architectures(self):
        for platform in ("linux/amd64", "linux/arm64"):
            self.assertIn(platform, self.workflow)

    def test_uses_a_native_runner_per_architecture(self):
        # Emulated arm64 package installation is prohibitively slow, so each
        # architecture must build on its own native runner.
        #
        # Asserted against the PARSED matrix rather than by substring search.
        # A substring check cannot express this: "ubuntu-24.04" is a substring of
        # "ubuntu-24.04-arm" and also appears on other jobs' runs-on, so pointing
        # BOTH matrix entries at the arm runner -- which silently emulates amd64,
        # the exact thing this matrix exists to avoid -- would still satisfy it.
        matrix = yaml.safe_load(self.workflow)["jobs"]["build"]["strategy"]["matrix"]
        pairing = {entry["platform"]: entry["runner"] for entry in matrix["include"]}
        self.assertEqual(
            pairing,
            {"linux/amd64": "ubuntu-24.04", "linux/arm64": "ubuntu-24.04-arm"},
        )

    def test_publishing_is_gated_on_the_default_branch(self):
        self.assertIn("refs/heads/main", self.workflow)

    def test_compiles_the_workspace_inside_the_image(self):
        self.assertIn("colcon build", self.workflow)


if __name__ == "__main__":
    unittest.main()
