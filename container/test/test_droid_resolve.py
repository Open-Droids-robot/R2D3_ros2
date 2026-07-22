"""Seam 1: `./droid resolve` is a pure decision step. It reads probe results from
the environment, prints the resolved configuration, and touches nothing. Driving it
with fabricated environments makes the whole decision table testable offline.

Known blind spot, stated rather than papered over: these cases verify the DECISION,
never the PROBES that populate it. A faulty probe would feed a wrong input to a
correct table and every case here would still pass. TestRealProbe below closes that
as far as it usefully can, and `./droid doctor` prints raw probe values so a human
can inspect what was actually observed."""

import subprocess
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DROID = REPO_ROOT / "droid"

PROBE_KEYS = (
    "DROID_OS", "DROID_ARCH", "DROID_GPU_VENDOR", "DROID_DRI",
    "DROID_JETSON", "DROID_DOCKER_GPU", "DROID_GPU_OVERRIDE",
)


def resolve(**probe):
    """Run `./droid resolve` with a fabricated probe environment.

    Returns (returncode, parsed_stdout_dict, stderr). Every probe key is set
    explicitly -- to the empty string when not named -- so the caller's real
    environment can never leak into a case.
    """
    env = {"PATH": "/usr/bin:/bin", "HOME": "/nonexistent"}
    for key in PROBE_KEYS:
        env[key] = ""
    for key, value in probe.items():
        env["DROID_" + key.upper()] = value
    proc = subprocess.run(
        [str(DROID), "resolve"], env=env, capture_output=True, text=True)
    parsed = dict(
        line.split("=", 1) for line in proc.stdout.splitlines() if "=" in line)
    return proc.returncode, parsed, proc.stderr


class TestAcceleratedTier(unittest.TestCase):
    def test_gpu_present_and_reachable_resolves_to_nvidia(self):
        code, out, _ = resolve(
            os="linux", arch="x86_64", gpu_vendor="nvidia", docker_gpu="ok")
        self.assertEqual(code, 0)
        self.assertEqual(out["tier"], "nvidia")


class TestUnreachableGpuHardFails(unittest.TestCase):
    """The case proven to occur on the reference machine: nvidia-smi reports an
    RTX 4060 and docker info lists a runtime, yet `docker run --gpus all` fails
    with a CDI discovery error. Silently falling back to software rendering here
    is the failure this whole seam exists to prevent."""

    def test_exits_three_and_prints_no_config(self):
        code, out, err = resolve(
            os="linux", arch="x86_64", gpu_vendor="nvidia", docker_gpu="fail")
        self.assertEqual(code, 3)
        self.assertEqual(out, {})
        self.assertIn("--gpu cpu", err)

    def test_override_wins_over_the_hard_failure(self):
        code, out, _ = resolve(
            os="linux", arch="x86_64", gpu_vendor="nvidia", docker_gpu="fail",
            gpu_override="cpu")
        self.assertEqual(code, 0)
        self.assertEqual(out["tier"], "cpu")

    def test_unknown_is_not_a_failure(self):
        # Probe could not run at all (no network to pull the probe image).
        # Degrade to software rendering rather than walling off the developer.
        code, out, _ = resolve(
            os="linux", arch="x86_64", gpu_vendor="nvidia", docker_gpu="unknown")
        self.assertEqual(code, 0)
        self.assertEqual(out["tier"], "cpu")


class TestNoGpuNeverErrors(unittest.TestCase):
    def test_plain_linux_laptop_resolves_to_cpu(self):
        code, out, err = resolve(
            os="linux", arch="x86_64", gpu_vendor="none", dri="yes")
        self.assertEqual(code, 0)
        self.assertEqual(out["tier"], "cpu")
        self.assertEqual(err, "")

    def test_macos_resolves_to_cpu_regardless_of_host_hardware(self):
        code, out, err = resolve(
            os="darwin", arch="arm64", gpu_vendor="nvidia", docker_gpu="fail")
        self.assertEqual(code, 0)
        self.assertEqual(out["tier"], "cpu")
        self.assertEqual(err, "")


class TestJetson(unittest.TestCase):
    def test_jetson_is_identified_and_reported(self):
        code, out, _ = resolve(
            os="linux", arch="aarch64", gpu_vendor="nvidia",
            docker_gpu="ok", jetson="yes")
        self.assertEqual(code, 0)
        self.assertEqual(out["jetson"], "yes")
        self.assertEqual(out["arch"], "arm64")
        self.assertEqual(out["tier"], "nvidia")


class TestArchitectureMapping(unittest.TestCase):
    def test_each_arch_maps_to_its_docker_platform(self):
        for host_arch, expected in (
                ("x86_64", "amd64"), ("amd64", "amd64"),
                ("aarch64", "arm64"), ("arm64", "arm64")):
            code, out, _ = resolve(os="linux", arch=host_arch, gpu_vendor="none")
            self.assertEqual(code, 0, host_arch)
            self.assertEqual(out["arch"], expected, host_arch)
            self.assertEqual(out["platform"], "linux/" + expected, host_arch)

    def test_unknown_arch_is_a_usage_error(self):
        code, _, err = resolve(os="linux", arch="riscv64", gpu_vendor="none")
        self.assertEqual(code, 2)
        self.assertIn("riscv64", err)


class TestOverrideValidation(unittest.TestCase):
    def test_bogus_override_is_rejected(self):
        code, _, err = resolve(os="linux", arch="x86_64", gpu_override="turbo")
        self.assertEqual(code, 2)
        self.assertIn("turbo", err)


class TestOutputShape(unittest.TestCase):
    def test_emits_exactly_the_documented_keys_in_order(self):
        env_keys = ["os", "arch", "platform", "gpu_vendor",
                    "dri", "jetson", "tier", "novnc_port"]
        proc = subprocess.run(
            [str(DROID), "resolve"],
            env={"PATH": "/usr/bin:/bin", "DROID_OS": "linux",
                 "DROID_ARCH": "x86_64", "DROID_GPU_VENDOR": "none"},
            capture_output=True, text=True)
        emitted = [line.split("=", 1)[0] for line in proc.stdout.splitlines()]
        self.assertEqual(emitted, env_keys)

    def test_novnc_port_is_6080(self):
        _, out, _ = resolve(os="linux", arch="x86_64", gpu_vendor="none")
        self.assertEqual(out["novnc_port"], "6080")


if __name__ == "__main__":
    unittest.main()
