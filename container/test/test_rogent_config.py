"""Seam 2 continued: static consistency guards for rogent mode. The mode is
spread across the droid script, a compose overlay, a derived Dockerfile, two
in-container scripts and a vcstool manifest -- and, as with the base config,
the failure mode when they drift is silent: a router nobody gates on, an env
marker nobody sets, a mount target nobody execs into. These tests require no
Docker and reach no network."""

import re
import unittest
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CONTAINER_DIR = REPO_ROOT / "container"


def read(path):
    return path.read_text()


def overlay():
    return yaml.safe_load(read(CONTAINER_DIR / "docker-compose.rogent.yml"))


def overlay_service():
    return overlay()["services"]["sim"]


def volume_targets(service):
    """Map of container target -> full volume entry, for short-syntax mounts."""
    return {entry.rsplit(":", 1)[-1]: entry for entry in service["volumes"]}


class TestImageChain(unittest.TestCase):
    """droid materialises and fingerprints one image ref per mode; the overlay
    names the image compose actually runs; the derived Dockerfile builds FROM
    the base. If any link drifts, `droid` reasons about an image nobody runs."""

    def test_droid_rogent_ref_matches_the_overlay_image(self):
        droid = read(REPO_ROOT / "droid")
        ref = re.search(r'ROGENT_IMAGE_REF="([^"]+)"', droid)
        self.assertIsNotNone(ref, "droid no longer defines ROGENT_IMAGE_REF")
        self.assertEqual(ref.group(1), overlay_service()["image"])

    def test_derived_dockerfile_builds_from_the_base_ref(self):
        droid = read(REPO_ROOT / "droid")
        base_ref = re.search(r'\bIMAGE_REF="([^"]+)"', droid).group(1)
        dockerfile = read(CONTAINER_DIR / "Dockerfile.rogent")
        from_line = re.search(r"^FROM\s+(\S+)\s*$", dockerfile, re.MULTILINE)
        self.assertIsNotNone(from_line, "Dockerfile.rogent has no FROM line")
        self.assertEqual(from_line.group(1), base_ref)

    def test_overlay_build_uses_the_derived_dockerfile(self):
        build = overlay_service()["build"]
        self.assertEqual(build["dockerfile"], "container/Dockerfile.rogent")

    def test_requirements_travel_through_the_named_build_context(self):
        # The Dockerfile COPYs from a named additional context; the overlay must
        # declare a context of exactly that name, fed by DROID_ROGENT_SRC.
        dockerfile = read(CONTAINER_DIR / "Dockerfile.rogent")
        copy_from = re.search(r"^COPY\s+--from=(\S+)\s", dockerfile, re.MULTILINE)
        self.assertIsNotNone(copy_from, "Dockerfile.rogent no longer COPYs from a build context")
        contexts = overlay_service()["build"]["additional_contexts"]
        self.assertIn(copy_from.group(1), contexts)
        self.assertIn("DROID_ROGENT_SRC", contexts[copy_from.group(1)])


class TestZenohTransport(unittest.TestCase):
    def test_base_image_installs_the_zenoh_rmw(self):
        # Unpinned and in the BASE image by decision (rogent-v3#11); it ships
        # rmw_zenohd. Absent, rogent mode fails at runtime with nothing useful.
        self.assertRegex(
            read(CONTAINER_DIR / "Dockerfile"), r"ros-jazzy-rmw-zenoh-cpp")

    def test_overlay_sets_the_zenoh_rmw_container_wide(self):
        self.assertEqual(
            overlay_service()["environment"]["RMW_IMPLEMENTATION"],
            "rmw_zenoh_cpp")

    def test_gui_start_supervises_the_router_behind_the_marker(self):
        # The router must be started by the session supervisor (gui-start), and
        # only in rogent mode -- an unconditional router would put zenoh
        # infrastructure into the default mode's containers.
        gui_start = read(CONTAINER_DIR / "gui-start.sh")
        self.assertIn("rmw_zenohd", gui_start)
        self.assertRegex(gui_start, r'\$\{DROID_ROGENT:-\}.*=.*"1"')

    def test_launch_sim_gates_on_the_router_port(self):
        # rmw_zenohd's default listen endpoint is tcp/[::]:7447 (upstream
        # default; rogent dials tcp/localhost:7447). The gate must probe that
        # port, and only in rogent mode.
        launch = read(CONTAINER_DIR / "launch-sim.sh")
        self.assertIn("/dev/tcp/127.0.0.1/7447", launch)
        self.assertRegex(launch, r'\$\{DROID_ROGENT:-\}.*=.*"1"')


class TestRogentMarkerParity(unittest.TestCase):
    """One in-container marker, three consumers. The overlay is the only thing
    that sets DROID_ROGENT; gui-start.sh (router), launch-sim.sh (gate) and
    `droid rogent` (mode check) all key off it. A rename that misses one file
    silently disables that consumer."""

    def test_overlay_sets_the_marker_to_1(self):
        self.assertEqual(overlay_service()["environment"]["DROID_ROGENT"], "1")

    def test_every_consumer_reads_the_same_marker(self):
        for path in ("gui-start.sh", "launch-sim.sh"):
            self.assertIn("DROID_ROGENT:-", read(CONTAINER_DIR / path), path)
        self.assertIn("printenv DROID_ROGENT", read(REPO_ROOT / "droid"))


class TestRogentSourceMount(unittest.TestCase):
    def test_source_mount_comes_from_droid_rogent_src(self):
        targets = volume_targets(overlay_service())
        self.assertIn("/rogent", targets)
        self.assertIn("DROID_ROGENT_SRC", targets["/rogent"])

    def test_droid_rogent_execs_into_the_mount_target(self):
        # `./droid rogent` runs main.py from the workdir the overlay mounts the
        # checkout at. If the target moves, the subcommand lands in a directory
        # with no main.py and dies confusingly.
        self.assertIn('-w /rogent', read(REPO_ROOT / "droid"))


class TestSpeechPlumbing(unittest.TestCase):
    def test_pulse_server_points_inside_the_pulse_mount(self):
        service = overlay_service()
        targets = volume_targets(service)
        self.assertIn("/run/host-pulse", targets)
        self.assertIn("/run/user/", targets["/run/host-pulse"])
        self.assertEqual(
            service["environment"]["PULSE_SERVER"], "unix:/run/host-pulse/native")

    def test_derived_image_installs_paplay(self):
        self.assertIn("pulseaudio-utils", read(CONTAINER_DIR / "Dockerfile.rogent"))


class TestOllamaReachability(unittest.TestCase):
    def test_host_gateway_alias_is_declared(self):
        self.assertIn(
            "host.docker.internal:host-gateway", overlay_service()["extra_hosts"])


class TestRefPairingManifest(unittest.TestCase):
    """rogent.repos is the record of which rogent ref this workspace is paired
    with, and the remediation `droid` prints tells people to import it."""

    def setUp(self):
        self.manifest = yaml.safe_load(read(REPO_ROOT / "rogent.repos"))

    def test_pins_the_rogent_repo_at_the_sim_ref(self):
        repo = self.manifest["repositories"]["rogent-v3"]
        self.assertEqual(repo["type"], "git")
        self.assertIn("Open-Droids-robot/rogent-v3", repo["url"])
        self.assertEqual(repo["version"], "sim_rogent")

    def test_droid_remediation_names_the_manifest(self):
        self.assertIn("rogent.repos", read(REPO_ROOT / "droid"))


class TestFingerprintCoversTheOverlay(unittest.TestCase):
    def test_overlay_is_folded_into_fingerprint_and_compose_files(self):
        # The overlay file must appear in droid twice: once where the compose
        # -f list is assembled, once where compose_files_content is folded into
        # the drift fingerprint. One occurrence means either invocations don't
        # apply the overlay or toggling it escapes the consent gate.
        droid = read(REPO_ROOT / "droid")
        self.assertGreaterEqual(droid.count("docker-compose.rogent.yml"), 2)

    def test_rogent_src_is_folded_into_the_fingerprint(self):
        # The mount path participates in Compose's recreate decision but is in
        # neither the resolved text nor the compose files' literal text -- the
        # same class of hole host_uid/gid already plug.
        self.assertRegex(
            read(REPO_ROOT / "droid"), r"rogent_src=%s")


if __name__ == "__main__":
    unittest.main()
