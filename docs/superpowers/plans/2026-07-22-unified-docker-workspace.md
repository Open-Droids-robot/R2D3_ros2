# Unified Docker Workspace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the multi-distro Docker tree with a single `./droid` entry point that takes any machine — amd64 Linux, Apple Silicon, Jetson, headless cloud — from a fresh clone to a running R2D3 simulation visible in a browser.

**Architecture:** One ROS 2 Jazzy image built `FROM ros:jazzy-ros-base` (multi-arch), one Compose service, and a bash 3.2 entry point at the repo root. GUI is delivered over noVNC inside the container (Xvfb + fluxbox + x11vnc + websockify) rather than through the host display stack, so the transport is identical everywhere. All branching flows through `droid resolve`, a pure decision step that reads probe results from the environment and prints `key=value` lines with no side effects — that is the seam the test suite drives. Build scope is narrowed by a colcon defaults file baked into the image (outside the bind mount), never by ignore-marker files in the source tree.

**Tech Stack:** bash 3.2, Docker + Compose v2, ROS 2 Jazzy, Gazebo Harmonic (`ros_gz`), `mujoco_ros2_control`, noVNC/websockify, pytest (`unittest.TestCase` style), GitHub Actions.

## Global Constraints

- **Entry point is `./droid` at the repository root**, executable, written in bash and constrained to **bash 3.2** features — no associative arrays (`declare -A`), no `${var^^}` / `${var,,}`, no `mapfile`/`readarray`, no `**` globstar. macOS ships bash 3.2 and the tool must run on it.
- **Nothing may be required beyond bash and Docker.** No python, jq, yq, or make on the host path.
- Subcommands are exactly: `up`, `shell`, `doctor`, `resolve`, `down`, `nuke`, plus `--help`. `up` accepts `--mujoco`, `--gpu <tier>`, `--recreate`.
- **Single ROS distro: Jazzy.** Foxy and Humble are removed, not maintained.
- Base image is **`ros:jazzy-ros-base`**. Target platforms are exactly **`linux/amd64`** and **`linux/arm64`**.
- **The container keeps the workspace's non-symlink install semantics** (`colcon build`, never `--symlink-install`). The stale-install footgun is neutralised by rebuilding the simulation subset on every launch path, not by changing build semantics.
- **Bridge networking with published ports.** Never `network_mode: host`, never `privileged`, never an X11 socket bind-mount, never `/dev/dri` on the committed tiers.
- **noVNC port is `6080`** and the URL printed is `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`.
- **Image reference is `ghcr.io/open-droids-robot/r2d3-sim:jazzy`.** This exact string must appear identically in the compose file and in `droid`.
- **The build/test ignore set is exactly these 14 package names**, in this order wherever a list is written:
  `zed_msgs`, `realsense2_camera`, `realsense2_camera_msgs`, `realsense2_description`, `rm_camera_demo`, `ros2_agv_robot`, `ros2_total_demo`, `rm_driver`, `woosh_action_msgs`, `woosh_common_msgs`, `woosh_nav_msgs`, `woosh_robot_msgs`, `woosh_ros_msgs`, `woosh_task_msgs`
- **The resulting simulation subset is exactly these 15 packages** (verified: `colcon list` drops 29 → 15 with the defaults file, writing nothing to the tree):
  `dual_rm_65b_moveit_config`, `dual_rm_75b_moveit_config`, `dual_rm_control`, `dual_rm_description`, `dual_rm_gazebo`, `dual_rm_install`, `dual_rm_moveit_demo`, `dual_rm_navigation`, `dual_rm_simulation`, `r2d3_bringup`, `r2d3_mujoco`, `r2d3_test_nodes`, `rm_ros_interfaces`, `servo_interfaces`, `servo_sim_bridge`
- **`warehouse_ros_mongo` is the only unresolvable rosdep key** in that subset on jazzy/noble (verified). It must appear in `--skip-keys` and nothing else may be silently skipped.
- **Tests stay static and offline.** No test may invoke Docker, launch a simulator, or reach the network. Existing style: plain `unittest.TestCase` collected by `pytest` from the repo root.
- New container files live under **`container/`** (lowercase, a new directory — *not* a case-rename of the existing `Docker/`, which would break checkout on case-insensitive filesystems).
- **Nothing may be written into the bind-mounted source tree at runtime.** `git status --porcelain` must stay empty after a container session.

---

## File Structure

**Created:**

| Path | Responsibility |
|---|---|
| `droid` | The entire CLI: arg parsing, probe, resolve, compose orchestration, drift check. Single file so it needs no install step. |
| `container/Dockerfile` | The one Jazzy image: apt deps via rosdep, GUI stack, user creation, colcon defaults, pre-built workspace, pre-warmed MuJoCo cache. |
| `container/docker-compose.yml` | Base service definition: image, build, bind mount, volumes, ports, shm, env. |
| `container/docker-compose.nvidia.yml` | Overlay applied only on the `nvidia` tier: GPU device reservation + driver capabilities. |
| `container/colcon-defaults.yaml` | Baked at `/etc/colcon/defaults.yaml`; carries the ignore set for the `build`, `test` and `list` verbs. |
| `container/entrypoint.sh` | Runs as root: remaps the runtime user's uid/gid to the host's, fixes ownership, starts the GUI stack, drops privileges. |
| `container/gui-start.sh` | Xvfb + fluxbox + x11vnc + websockify supervision. |
| `container/launch-sim.sh` | Rebuild the simulation subset, then launch the selected backend with RViz. |
| `container/env.example` | Documented template for the optional, git-ignored `container/.env`. |
| `container/test/test_droid_resolve.py` | Seam 1: drives `droid resolve` with fabricated environments across the decision table; plus one well-formedness test against the real probe. |
| `container/test/test_container_config.py` | Seam 2: static consistency guards across `droid`, compose, colcon defaults, `.gitignore`, CI. |
| `.devcontainer/devcontainer.json` | Optional VS Code / Cursor attachment targeting the same compose service. |
| `.github/workflows/container.yml` | Builds both architectures on native runners and compiles the workspace inside the image. |
| `docs/container.md` | The one canonical container document. |

**Modified:** `.gitignore`, `README.md`, `CLAUDE.md`, `simulation_quickstart_gz.md`, `simulation_quickstart_mujoco.md`

**Deleted:** the entire `Docker/` tree (4 Dockerfiles, 4 run scripts, 3 setup scripts, `Makefile`, `cmds.txt`, `docker-compose.yml`, `.dockerignore`, `.env`, `env.example`, and `readme.md`, `QUICKSTART.md`, `DISTRO_GUIDE.md`, `WORKSPACE_OVERVIEW.md`)

---

## Task 1: The `droid resolve` decision seam

The pure decision step, built test-first. Everything else in the tool calls it.

**Files:**
- Create: `droid`
- Test: `container/test/test_droid_resolve.py`

**Interfaces:**
- Consumes: nothing (first task).
- Produces: the `droid resolve` contract that Tasks 2 and 4 depend on.

  **Input environment variables** (all optional; absent means the empty string):
  | Variable | Values | Meaning |
  |---|---|---|
  | `DROID_OS` | `linux`, `darwin` | host OS |
  | `DROID_ARCH` | `x86_64`, `amd64`, `aarch64`, `arm64` | host machine arch |
  | `DROID_GPU_VENDOR` | `nvidia`, `none` | NVIDIA GPU present on the host? |
  | `DROID_DRI` | `yes`, `no` | is there a direct rendering device? |
  | `DROID_JETSON` | `yes`, `no` | is this a Jetson? |
  | `DROID_DOCKER_GPU` | `ok`, `fail`, `unknown` | could Docker *actually* acquire a GPU? |
  | `DROID_GPU_OVERRIDE` | `cpu`, `nvidia`, empty | explicit tier override |

  **Output** on stdout, exactly these eight lines in exactly this order:
  ```
  os=<linux|darwin>
  arch=<amd64|arm64>
  platform=linux/<amd64|arm64>
  gpu_vendor=<nvidia|none>
  dri=<yes|no>
  jetson=<yes|no>
  tier=<cpu|nvidia>
  novnc_port=6080
  ```

  **Exit codes:** `0` success · `2` usage/validation error · `3` GPU present but unreachable from Docker.

  **Decision rules, applied in this order:**
  1. `DROID_GPU_OVERRIDE` non-empty → it must be `cpu` or `nvidia` (else exit 2); `tier` is that value; **no GPU error is ever raised on this path**.
  2. `DROID_OS=darwin` → `tier=cpu`. Docker Desktop has no GPU passthrough to Linux containers, so macOS never errors.
  3. `DROID_GPU_VENDOR=nvidia` and `DROID_DOCKER_GPU=ok` → `tier=nvidia`.
  4. `DROID_GPU_VENDOR=nvidia` and `DROID_DOCKER_GPU=fail` → **exit 3**, nothing on stdout, remediation on stderr.
  5. Anything else (including `DROID_DOCKER_GPU=unknown`) → `tier=cpu`, exit 0.

- [ ] **Step 1: Write the failing test**

Create `container/test/test_droid_resolve.py`:

```python
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
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd /home/samzpc/code/r2d3/src/R2D3_ros2 && python3 -m pytest container/test/test_droid_resolve.py -v`
Expected: every test ERRORs with `FileNotFoundError` — `droid` does not exist yet.

- [ ] **Step 3: Write the minimal `droid` with `resolve` and `--help`**

Create `droid` at the repository root:

```bash
#!/usr/bin/env bash
# R2D3 containerised simulation workspace.
#
# Constrained to bash 3.2 (macOS ships 3.2): no associative arrays, no ${var^^},
# no mapfile, no globstar. Nothing beyond bash and Docker is required on the host.
set -eu

IMAGE_REF="ghcr.io/open-droids-robot/r2d3-sim:jazzy"
NOVNC_PORT="6080"
REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"

die() { printf '%s\n' "$*" >&2; exit 2; }

usage() {
  cat <<'EOF'
R2D3 containerised simulation workspace.

Usage: ./droid <command> [options]

Commands:
  up [--mujoco] [--gpu <tier>] [--recreate]
                Ensure the image, start the container, rebuild the simulation
                subset, launch the simulation, and print the noVNC URL.
  shell         Open a shell in the running container.
  doctor        Re-run the platform probe and print raw + resolved values.
  resolve       Print the resolved configuration as key=value lines. No side effects.
  down          Stop the container, preserving anything installed inside it.
  nuke          Destroy the container AND its volumes. Container-local installs are lost.

Options for `up`:
  --mujoco      Use the MuJoCo backend instead of Gazebo (the default).
  --gpu <tier>  Override platform detection. <tier> is `cpu` or `nvidia`.
  --recreate    Consent to recreating the container when its configuration has
                drifted. This destroys container-local installations.

The GUI is served over noVNC at http://localhost:6080 on every platform.
See docs/container.md.
EOF
}

# --- resolve ----------------------------------------------------------------
#
# A pure decision step: consumes probe results from the environment, prints the
# resolved configuration, performs no side effects. Every branching decision in
# this tool flows through here, which is what makes the decision table testable
# offline with fabricated environments -- and what makes "why did it pick this?"
# answerable by a human running `./droid resolve`.
cmd_resolve() {
  os="${DROID_OS:-}"
  arch="${DROID_ARCH:-}"
  gpu_vendor="${DROID_GPU_VENDOR:-}"
  dri="${DROID_DRI:-}"
  jetson="${DROID_JETSON:-}"
  docker_gpu="${DROID_DOCKER_GPU:-}"
  override="${DROID_GPU_OVERRIDE:-}"

  [ -n "$os" ] || os="linux"
  [ -n "$gpu_vendor" ] || gpu_vendor="none"
  [ -n "$dri" ] || dri="no"
  [ -n "$jetson" ] || jetson="no"
  [ -n "$docker_gpu" ] || docker_gpu="unknown"

  case "$arch" in
    x86_64|amd64) arch="amd64" ;;
    aarch64|arm64) arch="arm64" ;;
    "") die "droid resolve: DROID_ARCH is not set" ;;
    *) die "droid resolve: unsupported architecture '$arch' (expected x86_64 or aarch64)" ;;
  esac

  if [ -n "$override" ]; then
    case "$override" in
      cpu|nvidia) tier="$override" ;;
      *) die "droid resolve: unsupported --gpu tier '$override' (expected cpu or nvidia)" ;;
    esac
  elif [ "$os" = "darwin" ]; then
    # Docker Desktop has no GPU passthrough to Linux containers, so a Mac's own
    # hardware is irrelevant here and must never produce a GPU error.
    tier="cpu"
  elif [ "$gpu_vendor" = "nvidia" ] && [ "$docker_gpu" = "ok" ]; then
    tier="nvidia"
  elif [ "$gpu_vendor" = "nvidia" ] && [ "$docker_gpu" = "fail" ]; then
    gpu_unreachable_error
  else
    tier="cpu"
  fi

  printf 'os=%s\n' "$os"
  printf 'arch=%s\n' "$arch"
  printf 'platform=linux/%s\n' "$arch"
  printf 'gpu_vendor=%s\n' "$gpu_vendor"
  printf 'dri=%s\n' "$dri"
  printf 'jetson=%s\n' "$jetson"
  printf 'tier=%s\n' "$tier"
  printf 'novnc_port=%s\n' "$NOVNC_PORT"
}

# Hard failure, deliberately. An NVIDIA GPU is present but Docker cannot acquire
# it. Falling back silently means the developer runs software rendering for weeks
# while blaming the container for being slow.
gpu_unreachable_error() {
  cat >&2 <<EOF
droid: an NVIDIA GPU is present, but Docker cannot acquire it.

  nvidia-smi works on this host, but 'docker run --gpus all' fails. That usually
  means the NVIDIA Container Toolkit is missing or has generated no CDI spec.

  To fix it:
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
    sudo systemctl restart docker
  Then re-run:  ./droid doctor

  To proceed on software rendering instead, without fixing this:
    ./droid up --gpu cpu
EOF
  exit 3
}

# --- dispatch ---------------------------------------------------------------
[ $# -gt 0 ] || { usage; exit 2; }
case "$1" in
  resolve) shift; cmd_resolve "$@" ;;
  -h|--help|help) usage ;;
  *) die "droid: unknown command '$1' (try ./droid --help)" ;;
esac
```

Then: `chmod +x droid`

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd /home/samzpc/code/r2d3/src/R2D3_ros2 && python3 -m pytest container/test/test_droid_resolve.py -v`
Expected: PASS, except `TestRealProbe` which does not exist yet (it arrives in Task 2).

- [ ] **Step 5: Verify bash 3.2 compatibility by inspection**

Run: `grep -nE 'declare -A|\$\{[A-Za-z_]+\^\^|\$\{[A-Za-z_]+,,|mapfile|readarray' droid`
Expected: no output. Any hit is a bash 4+ construct and must be rewritten.

- [ ] **Step 6: Commit**

```bash
git add droid container/test/test_droid_resolve.py
git commit -m "feat(droid): add the resolve decision seam with its decision table under test"
```

---

## Task 2: The platform probe and `doctor`

Detection is a probe, not an inspection. It must test what Docker can actually do.

**Files:**
- Modify: `droid` (add `probe_platform`, `cmd_doctor`, wire `up`'s dispatch entry)
- Modify: `container/test/test_droid_resolve.py` (append `TestRealProbe`)

**Interfaces:**
- Consumes: `cmd_resolve` and the `DROID_*` input contract from Task 1.
- Produces: `probe_platform()` — exports `DROID_OS`, `DROID_ARCH`, `DROID_GPU_VENDOR`, `DROID_DRI`, `DROID_JETSON`, `DROID_DOCKER_GPU` into the environment, leaving `DROID_GPU_OVERRIDE` untouched. Tasks 4 calls `probe_platform` then `cmd_resolve`.

- [ ] **Step 1: Write the failing test**

Append to `container/test/test_droid_resolve.py`, above the `if __name__` block:

```python
class TestRealProbe(unittest.TestCase):
    """Closes Seam 1's blind spot as far as it usefully can. This runs the REAL
    probe on whatever machine hosts the suite and asserts only that it emits a
    well-formed result -- never a specific value, which would make the test
    machine-dependent. Whether a probe's reading is *right* on a platform the
    suite has never run on remains an accepted, documented gap."""

    def setUp(self):
        proc = subprocess.run(
            [str(DROID), "doctor", "--probe-only"],
            capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.probe = dict(
            line.split("=", 1)
            for line in proc.stdout.splitlines() if "=" in line)

    def test_emits_every_probe_key(self):
        self.assertEqual(
            sorted(self.probe),
            ["arch", "dri", "docker_gpu", "gpu_vendor", "jetson", "os"])

    def test_every_value_is_in_its_documented_domain(self):
        self.assertIn(self.probe["os"], ("linux", "darwin"))
        self.assertIn(self.probe["arch"], ("x86_64", "amd64", "aarch64", "arm64"))
        self.assertIn(self.probe["gpu_vendor"], ("nvidia", "none"))
        self.assertIn(self.probe["dri"], ("yes", "no"))
        self.assertIn(self.probe["jetson"], ("yes", "no"))
        self.assertIn(self.probe["docker_gpu"], ("ok", "fail", "unknown"))

    def test_probe_output_feeds_resolve_without_translation(self):
        # The probe's vocabulary and resolve's input vocabulary are the same
        # vocabulary. If they drift, `up` silently resolves from defaults.
        code, out, err = resolve(**self.probe)
        self.assertIn(code, (0, 3), err)
        if code == 0:
            self.assertIn(out["tier"], ("cpu", "nvidia"))
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest container/test/test_droid_resolve.py::TestRealProbe -v`
Expected: FAIL — `droid doctor` is not a command yet, so `returncode` is 2.

- [ ] **Step 3: Implement the probe and `doctor`**

In `droid`, insert before the `# --- dispatch ---` block:

```bash
# --- probe ------------------------------------------------------------------
#
# Detection is a probe, not an inspection. Host inspection alone is insufficient:
# on the reference machine nvidia-smi reports an RTX 4060 and `docker info` lists
# an nvidia runtime, yet `docker run --gpus all` fails with a CDI discovery error.
# An inspection-based implementation would generate a broken configuration on the
# very machine this was developed on. So the GPU question is answered by ATTEMPTING
# it.
probe_platform() {
  case "$(uname -s)" in
    Darwin) DROID_OS="darwin" ;;
    *) DROID_OS="linux" ;;
  esac
  DROID_ARCH="$(uname -m)"

  if [ -d /dev/dri ]; then DROID_DRI="yes"; else DROID_DRI="no"; fi

  DROID_JETSON="no"
  if [ -f /etc/nv_tegra_release ]; then
    DROID_JETSON="yes"
  elif [ -r /proc/device-tree/compatible ] &&
       tr -d '\0' < /proc/device-tree/compatible 2>/dev/null | grep -qi tegra; then
    DROID_JETSON="yes"
  fi

  DROID_GPU_VENDOR="none"
  if [ "$DROID_OS" != "darwin" ] && command -v nvidia-smi >/dev/null 2>&1 &&
     nvidia-smi -L >/dev/null 2>&1; then
    DROID_GPU_VENDOR="nvidia"
  fi

  DROID_DOCKER_GPU="unknown"
  if [ "$DROID_GPU_VENDOR" = "nvidia" ]; then
    DROID_DOCKER_GPU="$(probe_docker_gpu)"
  fi

  export DROID_OS DROID_ARCH DROID_DRI DROID_JETSON DROID_GPU_VENDOR DROID_DOCKER_GPU
}

# Answers "can Docker actually acquire a GPU?" by trying. Distinguishes a real
# refusal (`fail`) from an inability to run the probe at all (`unknown`, e.g. no
# network to fetch the probe image) -- because only the former justifies walling
# the developer off.
probe_docker_gpu() {
  command -v docker >/dev/null 2>&1 || { echo "unknown"; return; }
  if ! docker image inspect hello-world >/dev/null 2>&1; then
    docker pull -q hello-world >/dev/null 2>&1 || { echo "unknown"; return; }
  fi
  if docker run --rm --gpus all hello-world >/dev/null 2>&1; then
    echo "ok"
  else
    echo "fail"
  fi
}

cmd_doctor() {
  probe_only="no"
  while [ $# -gt 0 ]; do
    case "$1" in
      --probe-only) probe_only="yes"; shift ;;
      *) die "droid doctor: unknown option '$1'" ;;
    esac
  done

  probe_platform
  printf 'os=%s\n' "$DROID_OS"
  printf 'arch=%s\n' "$DROID_ARCH"
  printf 'gpu_vendor=%s\n' "$DROID_GPU_VENDOR"
  printf 'dri=%s\n' "$DROID_DRI"
  printf 'jetson=%s\n' "$DROID_JETSON"
  printf 'docker_gpu=%s\n' "$DROID_DOCKER_GPU"
  [ "$probe_only" = "yes" ] && return 0

  printf '\n--- resolved ---\n'
  cmd_resolve
}
```

And extend the dispatch `case`:

```bash
  doctor) shift; cmd_doctor "$@" ;;
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `python3 -m pytest container/test/test_droid_resolve.py -v`
Expected: all PASS.

- [ ] **Step 5: Confirm the probe reproduces the known reference-machine state**

Run: `./droid doctor`
Expected on this machine: `gpu_vendor=nvidia`, `docker_gpu=fail`, and the resolved section exiting 3 with the remediation text — the exact condition the seam exists to catch. On a machine without an NVIDIA GPU expect `gpu_vendor=none`, `docker_gpu=unknown`, `tier=cpu`, and no error.

- [ ] **Step 6: Commit**

```bash
git add droid container/test/test_droid_resolve.py
git commit -m "feat(droid): probe the platform by attempting GPU acquisition, not by inspecting it"
```

---

## Task 3: The container image

One Jazzy image, multi-arch, with the GUI stack, the baked colcon defaults, a built workspace and a pre-warmed MuJoCo cache.

**Files:**
- Create: `container/Dockerfile`, `container/colcon-defaults.yaml`, `container/entrypoint.sh`, `container/gui-start.sh`, `container/launch-sim.sh`
- Test: `container/test/test_container_config.py` (the package-selection guard only; the rest of this file arrives in Task 5)

**Interfaces:**
- Consumes: nothing from Tasks 1–2.
- Produces:
  - Image with workspace root `/ws`, source at `/ws/src/R2D3_ros2`, runtime user `droid` (uid 1000 by default, remapped at start), colcon defaults at `/etc/colcon/defaults.yaml`, noVNC on container port `6080`.
  - `/opt/droid/launch-sim.sh <gz|mujoco>` — rebuild then launch. Task 4's `up` invokes exactly this.
  - `/opt/droid/entrypoint.sh` — the image `ENTRYPOINT`; honours `HOST_UID` / `HOST_GID`.

- [ ] **Step 1: Write the failing test**

Create `container/test/test_container_config.py`:

```python
"""Seam 2: static consistency guards. The container configuration is spread across
a script, a compose file, a colcon defaults file, ignore markers and a CI workflow,
and the failure mode when they drift is silent. These tests are the drift alarm.
They require no Docker and reach no network."""

import re
import subprocess
import unittest
from pathlib import Path

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

    Parsed with a small regex rather than PyYAML so the guard has no dependency
    the rest of the suite does not already carry."""
    lists = {}
    verb = None
    for line in text.splitlines():
        stripped = line.strip()
        if re.match(r"^[a-z]+:$", stripped):
            verb = stripped[:-1]
            continue
        if stripped == "packages-ignore:":
            lists[verb] = []
            continue
        if stripped.startswith("- ") and verb in lists:
            lists[verb].append(stripped[2:].strip())
    return lists


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
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest container/test/test_container_config.py -v`
Expected: FAIL with `FileNotFoundError` on `container/colcon-defaults.yaml`.

> Note on `test_applying_the_defaults_file_writes_nothing_into_the_tree`: run it from a clean tree. It will report a false failure if you have uncommitted work in progress — that is the intended sensitivity, since the invariant it guards is "a container session leaves `git status` empty".

- [ ] **Step 3: Write `container/colcon-defaults.yaml`**

```yaml
# Baked into the image at /etc/colcon/defaults.yaml and selected via
# COLCON_DEFAULTS_FILE. This narrows every colcon invocation inside the container
# -- including a bare `colcon build` a developer types by hand -- to the packages
# the simulation can actually reach.
#
# It lives OUTSIDE the bind-mounted source tree on purpose. The obvious
# alternative, dropping COLCON_IGNORE markers into the package directories, is
# wrong here: created at runtime they would appear in the developer's working tree
# as version-control noise, and committed they would change the HOST's build too
# and break anyone legitimately building the vendor packages natively.
#
# The ignored set is hardware-only: the ZED wrapper, the RealSense packages, the
# AGV robot package, the Woosh message packages, the arm driver, and the
# object-detection demo. Skipping them is verified safe -- the sensor xacros are
# self-contained within the description packages, so no simulation package
# depends on any of these.
build:
  packages-ignore: &hardware_packages
    - zed_msgs
    - realsense2_camera
    - realsense2_camera_msgs
    - realsense2_description
    - rm_camera_demo
    - ros2_agv_robot
    - ros2_total_demo
    - rm_driver
    - woosh_action_msgs
    - woosh_common_msgs
    - woosh_nav_msgs
    - woosh_robot_msgs
    - woosh_ros_msgs
    - woosh_task_msgs
test:
  packages-ignore: *hardware_packages
list:
  packages-ignore: *hardware_packages
```

> If the regex parser in the test does not follow the YAML anchor, write the list out three times verbatim instead — the test asserts on all three verbs and is the arbiter. Prefer whichever form makes the test pass without weakening it.

- [ ] **Step 4: Run the package-selection test to verify it passes**

Run: `python3 -m pytest container/test/test_container_config.py -v`
Expected: PASS — 29 packages discovered without the defaults file, exactly the 15 simulation packages with it, and `git status --porcelain` empty.

- [ ] **Step 5: Write `container/entrypoint.sh`**

```bash
#!/usr/bin/env bash
# Runs as root. Remaps the runtime user's numeric id to the host user's so files
# created in the bind mount are owned by the developer on the host, fixes up
# ownership, starts the GUI stack, then drops privileges.
#
# Running the container as an unmapped host uid was rejected: it provides no
# passwd entry, and therefore no sudo -- and developers need to install packages
# mid-session.
set -eu

USER_NAME="droid"
HOST_UID="${HOST_UID:-1000}"
HOST_GID="${HOST_GID:-1000}"

current_uid="$(id -u "$USER_NAME")"
current_gid="$(id -g "$USER_NAME")"

if [ "$current_gid" != "$HOST_GID" ]; then
  groupmod -o -g "$HOST_GID" "$USER_NAME"
fi
if [ "$current_uid" != "$HOST_UID" ]; then
  usermod -o -u "$HOST_UID" "$USER_NAME"
fi

# Volumes are created root-owned by the daemon; the home directory needs fixing
# whenever the ids moved. The bind-mounted source tree is deliberately NOT
# chowned -- it already belongs to the host user.
chown "$HOST_UID:$HOST_GID" /home/"$USER_NAME"
for d in /home/"$USER_NAME"/.ros /home/"$USER_NAME"/.cache /ws /ws/build /ws/install /ws/log; do
  [ -d "$d" ] || mkdir -p "$d"
  chown -R "$HOST_UID:$HOST_GID" "$d" 2>/dev/null || true
done

exec setpriv --reuid "$HOST_UID" --regid "$HOST_GID" --init-groups \
  /opt/droid/gui-start.sh "$@"
```

- [ ] **Step 6: Write `container/gui-start.sh`**

```bash
#!/usr/bin/env bash
# Starts the in-container GUI stack, then execs the requested command.
#
# A virtual X server plus a window manager plus noVNC is the single universal GUI
# path: identical on macOS, Jetson, cloud and Linux, with no X11 socket bind-mount,
# no DISPLAY plumbing and no VNC client to install. It also gives Gazebo a GLX
# context rather than requiring headless EGL, which has been unreliable here --
# the symptom of EGL failure is /clock silently stalling rather than a clean error.
set -eu

export DISPLAY=":1"
GEOMETRY="${DROID_GEOMETRY:-1920x1080x24}"

Xvfb "$DISPLAY" -screen 0 "$GEOMETRY" +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &
for _ in $(seq 1 50); do
  xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 && break
  sleep 0.2
done
xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 || {
  echo "droid: virtual X server failed to start; see /tmp/xvfb.log" >&2
  exit 1
}

fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display "$DISPLAY" -forever -shared -nopw -quiet -rfbport 5900 \
  >/tmp/x11vnc.log 2>&1 &
websockify --web /usr/share/novnc 6080 localhost:5900 \
  >/tmp/websockify.log 2>&1 &

exec "$@"
```

- [ ] **Step 7: Write `container/launch-sim.sh`**

```bash
#!/usr/bin/env bash
# Rebuild the simulation subset, then launch. Every launch path rebuilds first.
#
# The workspace is built WITHOUT --symlink-install, here exactly as on the host,
# so that every existing document stays true and there is one mental model. That
# makes install/ plain copies, which means an edit under src/ does not exist until
# colcon build copies it -- and the symptom of forgetting is "this setting does
# nothing", which sends people debugging code that was never wrong. Rebuilding on
# every launch makes that impossible to trigger. It is cheap: the whole subset is
# data packages, with C++ only in r2d3_test_nodes (four files) and servo_interfaces
# (two message definitions).
set -eu

BACKEND="${1:-gz}"

# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.sh
cd /ws
colcon build
# shellcheck disable=SC1091
. /ws/install/setup.sh

case "$BACKEND" in
  gz)
    ros2 launch dual_rm_simulation gz_sim.launch.py &
    sim_pid=$!
    rviz_config="$(ros2 pkg prefix dual_rm_description)/share/dual_rm_description/rviz/view.rviz"
    rviz2 -d "$rviz_config" &
    wait "$sim_pid"
    ;;
  mujoco)
    # First launch on an UNMODIFIED tree hits the cache baked into the image and
    # starts promptly. A feature branch or local description edit changes the
    # generated robot description, so the content-addressed cache misses and the
    # full multi-minute reconversion runs. That is correct behaviour, not a hang.
    ros2 launch r2d3_mujoco mujoco_sim.launch.py &
    sim_pid=$!
    rviz_config="$(ros2 pkg prefix dual_rm_description)/share/dual_rm_description/rviz/view.rviz"
    rviz2 -d "$rviz_config" &
    wait "$sim_pid"
    ;;
  *)
    echo "launch-sim: unknown backend '$BACKEND' (expected gz or mujoco)" >&2
    exit 2
    ;;
esac
```

- [ ] **Step 8: Write `container/Dockerfile`**

```dockerfile
# One image, ROS 2 Jazzy, for linux/amd64 and linux/arm64.
#
# FROM ros:jazzy-ros-base rather than osrf/ros:jazzy-desktop-full: the osrf images
# are published for amd64 only and therefore cannot run natively on Apple Silicon
# or a Jetson. Jazzy is the only distro here -- Gazebo Harmonic pairs with Jazzy,
# mujoco_ros2_control has no Humble binary, and Foxy is end-of-life.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# GUI stack, software GL, and the handful of tools the entry point and the
# developer need. Image size is explicitly not a constraint here; functionality
# takes priority.
RUN apt-get update && apt-get install -y --no-install-recommends \
      xvfb x11vnc novnc websockify fluxbox x11-utils x11-xserver-utils \
      libgl1-mesa-dri mesa-utils libglx-mesa0 \
      python3-colcon-common-extensions python3-vcstool \
      sudo git curl wget nano less bash-completion util-linux \
    && rm -rf /var/lib/apt/lists/*

# The runtime user. Starts at 1000 and is remapped to the host's ids by the
# entrypoint, so a developer whose uid is not 1000 needs no local image rebuild.
# noble's base image ships an 'ubuntu' user at 1000 that must go first.
RUN if getent passwd ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi && \
    groupadd --gid 1000 droid && \
    useradd --uid 1000 --gid 1000 -m -s /bin/bash droid && \
    echo 'droid ALL=(root) NOPASSWD:ALL' > /etc/sudoers.d/droid && \
    chmod 0440 /etc/sudoers.d/droid

# Narrow every colcon invocation to the simulation subset. See the file's own
# comment for why this is a defaults file and not COLCON_IGNORE markers.
COPY container/colcon-defaults.yaml /etc/colcon/defaults.yaml
ENV COLCON_DEFAULTS_FILE=/etc/colcon/defaults.yaml

WORKDIR /ws
COPY --chown=droid:droid . /ws/src/R2D3_ros2

# Dependencies come from rosdep over the package manifests, so the manifests stay
# the single source of truth rather than a hand-maintained apt list.
# warehouse_ros_mongo is the one key with no jazzy/noble binary; it is a MoveIt
# warehouse backend the simulation never loads.
RUN apt-get update && \
    rosdep update --rosdistro jazzy && \
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
      --skip-keys "warehouse_ros_mongo" \
    && rm -rf /var/lib/apt/lists/*

# A built workspace is required, not incidental: pre-warming the MuJoCo cache
# means running the description through xacro and the converter at image build
# time, which needs the workspace installed first.
RUN source /opt/ros/jazzy/setup.bash && colcon build

# Pre-warm the MuJoCo converter venv and the converted scene into the ROS user
# state directory, which is exposed as a volume seeded from the image.
#
# The benefit is deliberately scoped: the conversion is content-addressed over the
# generated robot description plus the world file, so this is a cache HIT only for
# an unmodified checkout at this revision. A developer on a feature branch pays the
# full reconversion, correctly.
USER droid
RUN source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
    ros2 launch r2d3_mujoco mujoco_sim.launch.py headless:=true & \
    sleep 240; kill %1 2>/dev/null; \
    test -f /home/droid/.ros/r2d3_mujoco/65b/checksum
USER root

COPY container/entrypoint.sh container/gui-start.sh container/launch-sim.sh /opt/droid/
RUN chmod +x /opt/droid/*.sh

# Bridge networking with published ports, never host networking -- host networking
# is Linux-only and would break macOS. Outbound HTTPS is all that is required;
# distributed ROS across machines is explicitly out of scope, so the graph stays
# inside the container.
ENV ROS_LOCALHOST_ONLY=1 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    XDG_CACHE_HOME=/home/droid/.cache \
    HF_HOME=/home/droid/.cache/huggingface \
    TORCH_HOME=/home/droid/.cache/torch

EXPOSE 6080
ENTRYPOINT ["/opt/droid/entrypoint.sh"]
CMD ["sleep", "infinity"]
```

- [ ] **Step 9: Build the image and verify the workspace compiles inside it**

Run: `docker build -f container/Dockerfile -t ghcr.io/open-droids-robot/r2d3-sim:jazzy .`
Expected: a successful build. This is slow on a first run.

If the `rosdep install` step fails on an unexpected key, **do not widen `--skip-keys` reflexively** — resolve the key first, and only skip it with a comment naming why it is safe.

If the MuJoCo pre-warm step fails, verify by hand inside a shell what the converter reported before adjusting the timeout; the cache directory is `~/.ros/r2d3_mujoco/65b/` and a valid cache has both `mujoco_description_formatted.xml` and `checksum`.

- [ ] **Step 10: Verify the ignore set actually took effect in the image**

Run: `docker run --rm --entrypoint bash ghcr.io/open-droids-robot/r2d3-sim:jazzy -c 'cd /ws && colcon list --names-only | sort | tr "\n" " "'`
Expected: exactly the 15 simulation package names from the Global Constraints, and none of the 14 ignored ones.

- [ ] **Step 11: Commit**

```bash
git add container/Dockerfile container/colcon-defaults.yaml container/entrypoint.sh \
        container/gui-start.sh container/launch-sim.sh container/test/test_container_config.py
git commit -m "feat(container): add the single Jazzy image with noVNC GUI and scoped build"
```

---

## Task 4: Compose definition and the lifecycle commands

`up`, `shell`, `down`, `nuke` — and the consent gate that stops a config change from silently destroying container-local installs.

**Files:**
- Create: `container/docker-compose.yml`, `container/docker-compose.nvidia.yml`
- Modify: `droid`

**Interfaces:**
- Consumes: `probe_platform` and `cmd_resolve` (Tasks 1–2); `/opt/droid/launch-sim.sh` and the image reference (Task 3).
- Produces: the compose project name `r2d3`, service name `sim`, container name `r2d3-sim`, and the drift label `com.r2d3.droid.config`.

- [ ] **Step 1: Write `container/docker-compose.yml`**

```yaml
# One service. Bridge networking with a published port -- never host networking,
# which is Linux-only and would break macOS. The image is named first and a build
# definition carried alongside it, so a pull is attempted and a local build is the
# fallback: the tool works with no registry access and works offline.
name: r2d3

services:
  sim:
    image: ghcr.io/open-droids-robot/r2d3-sim:jazzy
    build:
      context: ..
      dockerfile: container/Dockerfile
    container_name: r2d3-sim
    platform: linux/${DROID_COMPOSE_ARCH:-amd64}
    # The default shared-memory allocation is small enough to cause cryptic
    # transport failures on a many-node launch. The repository already carries an
    # unreferenced FastDDS profile written for that failure; sizing it properly
    # here is the better fix.
    shm_size: "2gb"
    ports:
      - "${DROID_NOVNC_PORT:-6080}:6080"
    environment:
      HOST_UID: "${DROID_HOST_UID:-1000}"
      HOST_GID: "${DROID_HOST_GID:-1000}"
      LIBGL_ALWAYS_SOFTWARE: "${DROID_SOFTWARE_GL:-1}"
    env_file:
      - path: .env
        required: false
    labels:
      com.r2d3.droid.config: "${DROID_CONFIG_FINGERPRINT:-unset}"
    volumes:
      # The developer's actual working tree, so edits in their own editor take
      # effect with no image rebuild.
      - ..:/ws/src/R2D3_ros2
      # Build artifacts as named volumes: strictly separate from any native build
      # on the host, and heavy build I/O kept off the bind mount, which matters
      # on macOS.
      - r2d3_build:/ws/build
      - r2d3_install:/ws/install
      - r2d3_log:/ws/log
      # Seeded from the image, which is what makes the MuJoCo cache warm on a
      # first launch of an unmodified tree.
      - r2d3_ros_home:/home/droid/.ros
      # Downloaded model weights survive restarts.
      - r2d3_cache:/home/droid/.cache
    stdin_open: true
    tty: true

volumes:
  r2d3_build:
  r2d3_install:
  r2d3_log:
  r2d3_ros_home:
  r2d3_cache:
```

- [ ] **Step 2: Write `container/docker-compose.nvidia.yml`**

```yaml
# Applied as an overlay only when `droid resolve` lands on tier=nvidia -- that is,
# only when `docker run --gpus all` was PROVEN to work by the probe.
services:
  sim:
    environment:
      LIBGL_ALWAYS_SOFTWARE: "0"
      NVIDIA_VISIBLE_DEVICES: "all"
      NVIDIA_DRIVER_CAPABILITIES: "graphics,compute,utility"
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu, graphics]
```

- [ ] **Step 3: Implement the lifecycle commands in `droid`**

Insert before `# --- dispatch ---`:

```bash
COMPOSE_DIR="$REPO_ROOT/container"

require_docker() {
  command -v docker >/dev/null 2>&1 ||
    die "droid: docker is not installed or not on PATH. See docs/container.md."
  docker info >/dev/null 2>&1 ||
    die "droid: the Docker daemon is not reachable. Is Docker running?"
}

# Loads `droid resolve`'s output as shell variables prefixed with r_.
load_resolved() {
  probe_platform
  resolved="$(cmd_resolve)" || exit $?
  r_arch="$(printf '%s\n' "$resolved" | sed -n 's/^arch=//p')"
  r_tier="$(printf '%s\n' "$resolved" | sed -n 's/^tier=//p')"
  r_port="$(printf '%s\n' "$resolved" | sed -n 's/^novnc_port=//p')"
  # The fingerprint is the whole resolved configuration. Any change to any
  # decision changes it, which is exactly the drift the consent gate must catch.
  r_fingerprint="$(printf '%s' "$resolved" | cksum | tr -d ' ')"

  # Deliberately a DIFFERENT variable from DROID_ARCH, which is the probe's raw
  # `uname -m` reading and an INPUT to resolve. Reusing the name here would feed
  # resolve's own output back in as its input on any second call.
  export DROID_COMPOSE_ARCH="$r_arch"
  export DROID_NOVNC_PORT="$r_port"
  export DROID_CONFIG_FINGERPRINT="$r_fingerprint"
  export DROID_HOST_UID="$(id -u)"
  export DROID_HOST_GID="$(id -g)"
  if [ "$r_tier" = "nvidia" ]; then
    export DROID_SOFTWARE_GL="0"
  else
    export DROID_SOFTWARE_GL="1"
  fi
}

compose() {
  if [ "$r_tier" = "nvidia" ]; then
    docker compose -f "$COMPOSE_DIR/docker-compose.yml" \
                   -f "$COMPOSE_DIR/docker-compose.nvidia.yml" "$@"
  else
    docker compose -f "$COMPOSE_DIR/docker-compose.yml" "$@"
  fi
}

container_fingerprint() {
  docker inspect -f '{{index .Config.Labels "com.r2d3.droid.config"}}' \
    r2d3-sim 2>/dev/null || true
}

# The container orchestrator recreates a container whenever its resolved
# configuration changes -- which would silently destroy the packages a developer
# installed with sudo inside it. Re-running the platform probe and landing on a
# different tier is exactly such a change, so the documented recovery path would
# otherwise be the thing that wipes their tools. Refuse, and require consent.
check_drift() {
  existing="$(container_fingerprint)"
  [ -n "$existing" ] || return 0
  [ "$existing" = "unset" ] && return 0
  [ "$existing" = "$r_fingerprint" ] && return 0
  cat >&2 <<EOF
droid: the resolved configuration has changed since this container was created.

  Applying it requires recreating the container, which DESTROYS anything you
  installed inside it with sudo, along with your shell history and scratch files.
  Your source tree, build artifacts and caches are on volumes and are unaffected.

  Resolved now:
$(printf '%s\n' "$resolved" | sed 's/^/    /')

  To keep the existing container, run it unchanged:
    ./droid shell

  To accept the new configuration and recreate:
    ./droid up --recreate

  Anything you want to keep permanently belongs in container/Dockerfile, so your
  teammates get it too.
EOF
  exit 4
}

cmd_up() {
  backend="gz"
  recreate="no"
  while [ $# -gt 0 ]; do
    case "$1" in
      --mujoco) backend="mujoco"; shift ;;
      --gpu) [ $# -ge 2 ] || die "droid up: --gpu needs a tier"
             DROID_GPU_OVERRIDE="$2"; export DROID_GPU_OVERRIDE; shift 2 ;;
      --recreate) recreate="yes"; shift ;;
      *) die "droid up: unknown option '$1' (try ./droid --help)" ;;
    esac
  done

  require_docker
  load_resolved
  [ "$recreate" = "yes" ] || check_drift

  if [ "$r_tier" = "cpu" ]; then
    cat <<'EOF'
droid: running on the software-rendering tier.

  The goal of this tier is to be ALIVE, not fast. The simulation carries four
  RGBD cameras and a GPU lidar, so software rendering will be slow -- that is
  expected, not a defect. The GPU tier is the recommended path for real work.
EOF
  fi

  if [ "$recreate" = "yes" ]; then
    compose up -d --force-recreate
  else
    compose up -d
  fi

  cat <<EOF

droid: open the simulation at
  http://localhost:$r_port/vnc.html?autoconnect=1&resize=scale

EOF
  compose exec sim /opt/droid/launch-sim.sh "$backend"
}

cmd_shell() {
  require_docker
  load_resolved
  compose exec sim bash
}

cmd_down() {
  require_docker
  load_resolved
  compose stop
  echo "droid: container stopped. Anything you installed inside it is preserved."
  echo "       Start again with ./droid up"
}

cmd_nuke() {
  require_docker
  load_resolved
  cat >&2 <<'EOF'
droid: this DESTROYS the container and all of its volumes.

  You will lose: packages you installed inside the container, the build and
  install spaces, the MuJoCo cache, and any downloaded model weights.
  You will NOT lose: anything in your source tree, which lives on your host.
EOF
  printf 'Type "nuke" to confirm: '
  read -r confirm
  [ "$confirm" = "nuke" ] || die "droid: aborted; nothing was destroyed."
  compose down --volumes --remove-orphans
  echo "droid: destroyed. ./droid up will start clean."
}
```

Extend the dispatch `case`:

```bash
  up) shift; cmd_up "$@" ;;
  shell) shift; cmd_shell "$@" ;;
  down) shift; cmd_down "$@" ;;
  nuke) shift; cmd_nuke "$@" ;;
```

- [ ] **Step 4: Verify the compose files parse and resolve identically to the tool**

Run:
```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2
DROID_COMPOSE_ARCH=amd64 docker compose -f container/docker-compose.yml config >/dev/null && echo "base OK"
DROID_COMPOSE_ARCH=amd64 docker compose -f container/docker-compose.yml \
  -f container/docker-compose.nvidia.yml config >/dev/null && echo "nvidia overlay OK"
```
Expected: `base OK` and `nvidia overlay OK`.

- [ ] **Step 5: Re-run the bash 3.2 guard and the resolve suite**

Run:
```bash
grep -nE 'declare -A|\$\{[A-Za-z_]+\^\^|\$\{[A-Za-z_]+,,|mapfile|readarray' droid; echo "exit=$?"
python3 -m pytest container/test/ -v
```
Expected: the grep prints nothing (`exit=1`), and every test passes.

- [ ] **Step 6: Commit**

```bash
git add container/docker-compose.yml container/docker-compose.nvidia.yml droid
git commit -m "feat(droid): add compose definition and lifecycle commands with a recreate consent gate"
```

---

## Task 5: Repository hygiene, editor attachment, and the remaining drift guards

**Files:**
- Create: `container/env.example`, `.devcontainer/devcontainer.json`
- Modify: `.gitignore`, `container/test/test_container_config.py`
- Delete from tracking: `Docker/.env`

**Interfaces:**
- Consumes: the compose service name `sim` and file path `container/docker-compose.yml` (Task 4).
- Produces: nothing later tasks depend on.

- [ ] **Step 1: Write the failing tests**

Append to `container/test/test_container_config.py`, above the `if __name__` block:

```python
class TestImageReferenceParity(unittest.TestCase):
    """The entry point prints and reasons about an image the compose file is the
    one actually pulling. If they drift, `droid` reports success against an image
    nobody is running."""

    def test_droid_and_compose_name_the_same_image(self):
        droid = (REPO_ROOT / "droid").read_text()
        compose = (CONTAINER_DIR / "docker-compose.yml").read_text()
        ref = re.search(r'IMAGE_REF="([^"]+)"', droid).group(1)
        self.assertIn(ref, compose)
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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `python3 -m pytest container/test/test_container_config.py -v`
Expected: `TestSecretsHygiene` and `TestDevContainer` fail; `TestImageReferenceParity` may already pass from Tasks 3–4.

- [ ] **Step 3: Write `container/env.example`**

```bash
# Optional. Copy to container/.env to set environment variables inside the
# container. This file is read if present and is NOT generated for you.
#
# container/.env is git-ignored. Credential plumbing is deliberately yours: this
# tooling mounts no secrets and configures no cloud CLI.
#
# Downloaded model weights are already cached across restarts on a volume, so
# nothing here is needed to make caching work.

# Example: an API key a node needs to call a remote inference endpoint.
# OPENAI_API_KEY=

# Example: raise the virtual display resolution.
# DROID_GEOMETRY=2560x1440x24
```

- [ ] **Step 4: Update `.gitignore`**

Add these lines to `.gitignore`:

```
container/.env
.env
```

- [ ] **Step 5: Remove the tracked environment file from version control**

Run:
```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2
git rm --cached Docker/.env
```
(`Docker/.env` on disk is deleted wholesale in Task 7 along with the rest of the legacy tree; this step removes it from tracking now so the hygiene test passes.)

- [ ] **Step 6: Write `.devcontainer/devcontainer.json`**

```json
{
  "name": "R2D3 simulation workspace",
  "dockerComposeFile": ["../container/docker-compose.yml"],
  "service": "sim",
  "workspaceFolder": "/ws/src/R2D3_ros2",
  "remoteUser": "droid",
  "shutdownAction": "none",
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-python.python",
        "ms-vscode.cpptools",
        "ms-iot.vscode-ros"
      ]
    }
  }
}
```

> This is strictly optional. `./droid shell` is equivalent, and the host keeps the real git tree either way — the tooling must not force an IDE on anyone.

- [ ] **Step 7: Run the tests to verify they pass**

Run: `python3 -m pytest container/test/ -v`
Expected: all PASS.

- [ ] **Step 8: Commit**

```bash
git add .gitignore container/env.example .devcontainer/devcontainer.json \
        container/test/test_container_config.py
git rm --cached Docker/.env
git commit -m "chore(container): untrack the committed env file, add an example and dev container"
```

---

## Task 6: CI on both architectures

**Files:**
- Create: `.github/workflows/container.yml`
- Modify: `container/test/test_container_config.py`

**Interfaces:**
- Consumes: `container/Dockerfile`, the image reference, the simulation package list.
- Produces: nothing later tasks depend on.

- [ ] **Step 1: Write the failing test**

Append to `container/test/test_container_config.py`, above the `if __name__` block:

```python
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
        self.assertIn("ubuntu-24.04-arm", self.workflow)
        self.assertIn("ubuntu-24.04", self.workflow)

    def test_publishing_is_gated_on_the_default_branch(self):
        self.assertIn("refs/heads/main", self.workflow)

    def test_compiles_the_workspace_inside_the_image(self):
        self.assertIn("colcon build", self.workflow)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest container/test/test_container_config.py::TestCiArchitectureParity -v`
Expected: FAIL with `FileNotFoundError`.

- [ ] **Step 3: Write `.github/workflows/container.yml`**

```yaml
# CI scope is deliberately minimal: it proves the image BUILDS and the workspace
# COMPILES inside it, on both architectures. It does not boot the simulation.
#
# That is a conscious trade. The simulation carries four RGBD cameras and a GPU
# lidar, hosted runners are small and have no GPU, and a software-rendered boot
# test on that payload would be slow and chronically flaky -- on the only job
# giving arm64 any coverage at all. The honest consequence, which the docs state:
# the arm64 image is proven to build, not proven to boot.
name: container

on:
  push:
    branches: ["**"]
  pull_request:
  workflow_dispatch:

env:
  IMAGE: ghcr.io/open-droids-robot/r2d3-sim

jobs:
  build:
    strategy:
      fail-fast: false
      matrix:
        include:
          - runner: ubuntu-24.04
            platform: linux/amd64
            arch: amd64
          - runner: ubuntu-24.04-arm
            platform: linux/arm64
            arch: arm64
    runs-on: ${{ matrix.runner }}
    permissions:
      contents: read
      packages: write
    steps:
      - uses: actions/checkout@v4

      - name: Build the image natively
        run: |
          docker build \
            --platform ${{ matrix.platform }} \
            -f container/Dockerfile \
            -t "$IMAGE:ci-${{ matrix.arch }}" \
            .

      - name: Verify the workspace compiles inside the image
        run: |
          docker run --rm --entrypoint bash "$IMAGE:ci-${{ matrix.arch }}" -c '
            set -e
            source /opt/ros/jazzy/setup.bash
            cd /ws
            colcon build
            source /ws/install/setup.bash
            ros2 pkg prefix dual_rm_simulation
            ros2 pkg prefix r2d3_mujoco
          '

      - name: Verify the build scope stayed inside the simulation subset
        run: |
          docker run --rm --entrypoint bash "$IMAGE:ci-${{ matrix.arch }}" -c '
            cd /ws && colcon list --names-only | sort
          ' > found.txt
          cat > expected.txt <<'EOF'
          dual_rm_65b_moveit_config
          dual_rm_75b_moveit_config
          dual_rm_control
          dual_rm_description
          dual_rm_gazebo
          dual_rm_install
          dual_rm_moveit_demo
          dual_rm_navigation
          dual_rm_simulation
          r2d3_bringup
          r2d3_mujoco
          r2d3_test_nodes
          rm_ros_interfaces
          servo_interfaces
          servo_sim_bridge
          EOF
          sed 's/^          //' expected.txt | sed '/^$/d' > expected.clean
          diff -u expected.clean found.txt

      - name: Log in to the registry
        if: github.ref == 'refs/heads/main'
        uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}

      - name: Push the per-architecture image
        if: github.ref == 'refs/heads/main'
        run: |
          docker tag "$IMAGE:ci-${{ matrix.arch }}" "$IMAGE:jazzy-${{ matrix.arch }}"
          docker push "$IMAGE:jazzy-${{ matrix.arch }}"

  manifest:
    needs: build
    if: github.ref == 'refs/heads/main'
    runs-on: ubuntu-24.04
    permissions:
      contents: read
      packages: write
    steps:
      - uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}
      - name: Publish the multi-architecture manifest
        run: |
          docker buildx imagetools create \
            -t "$IMAGE:jazzy" \
            "$IMAGE:jazzy-amd64" \
            "$IMAGE:jazzy-arm64"

  tests:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: "3.12"
      - run: pip install pytest pyyaml
      - name: Run the static guards
        # These require no Docker and reach no network, which is the point.
        run: python -m pytest container/test/test_droid_resolve.py -v
```

> Note: `container/test/test_container_config.py` is not run in the `tests` job because its package-selection case needs `colcon`, which is not on a bare runner. If you want it in CI, add it to the in-image verification step instead of installing colcon on the runner.

- [ ] **Step 4: Run the test to verify it passes**

Run: `python3 -m pytest container/test/ -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add .github/workflows/container.yml container/test/test_container_config.py
git commit -m "ci(container): build and compile on native amd64 and arm64 runners"
```

---

## Task 7: Documentation, and deleting the superseded tree

One canonical document replaces five overlapping ones.

**Files:**
- Create: `docs/container.md`
- Modify: `README.md`, `CLAUDE.md`, `simulation_quickstart_gz.md`, `simulation_quickstart_mujoco.md`
- Delete: the entire `Docker/` directory

**Interfaces:**
- Consumes: everything from Tasks 1–6.
- Produces: nothing.

- [ ] **Step 1: Write `docs/container.md`**

The one canonical container document. It must cover, in this order:

1. **What this is and the one command** — `./droid up`, then `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`. Prerequisites are bash and Docker, nothing else.
2. **The two tiers, and their different ambitions.** State plainly that the GPU tier is the recommended path and where real work happens; that the software tier's goal is *to be alive, not fast*, because the simulation carries four RGBD cameras and a GPU lidar, and that its slowness is expected rather than a defect. Its success criterion is: the simulation runs, the clock advances, the robot is visible.
3. **Commands** — `up` (with `--mujoco`, `--gpu <tier>`, `--recreate`), `shell`, `doctor`, `resolve`, `down`, `nuke`. Each with what it does and what it preserves or destroys.
4. **Platform detection** — that it is a probe, not an inspection; that `docker run --gpus all` is actually attempted; the hard-fail case with the remediation commands and the `--gpu cpu` escape hatch; and that `./droid resolve` answers "why did it pick this?".
5. **Editing code** — the working tree is bind-mounted, edits take effect immediately, no image rebuild. Build artifacts are on volumes, strictly separate from any native build on the host. Files you create are owned by you on the host, whatever your uid.
6. **The rebuild guarantee** — every launch path rebuilds the simulation subset first, so a YAML or xacro edit takes effect on the next launch. State that the container keeps the host's non-symlink install semantics deliberately, so there is one mental model and the existing docs stay true, and that the rebuild is cheap because the subset is almost entirely data packages.
7. **Persistence and destruction** — `down` preserves container-local `sudo apt install`s, shell history and scratch files; `nuke` is the explicit destructive reset; a configuration change refuses to act without `--recreate`. Direct developers to promote anything permanent into `container/Dockerfile` so teammates receive it too.
8. **MuJoCo cold start** — the cache is warm *only* for an unmodified checkout at the image's revision, because the conversion is content-addressed over the generated description plus the world file. On a feature branch or with local description edits, expect a multi-minute reconversion on first launch. That is correct behaviour and not a hang.
9. **What is built and what is not** — the 15 simulation packages, the 14 ignored hardware packages, and why (no camera SDK to run a simulation). Nav2 and MoveIt are in the image and launchable by hand from `./droid shell`, but are not subcommands.
10. **Verification status, stated honestly.** A table:

    | Path | Status |
    |---|---|
    | amd64, software-rendering tier, Gazebo and MuJoCo | Verified by hand end-to-end |
    | amd64 and arm64 image build + workspace compile | Verified by CI |
    | NVIDIA accelerated tier | **Not verified** |
    | Jetson | **Not verified** |
    | arm64 at runtime | **Built by CI, not booted** |
    | macOS desktop experience | **Not verified** |
    | Windows / WSL2 | Not supported — deliberately out of scope |

11. **Optional dev container** — "Reopen in Container" in VS Code or Cursor; strictly optional, `./droid shell` is equivalent.
12. **Credentials and caching** — `container/.env` is read if present, is git-ignored, and is not generated. Model weights cache on a volume across restarts.
13. **Naming** — Factory AI ships a CLI also called `droid`. `./droid` from the repo root is unambiguous; putting it on `PATH` is optional and carries this caveat.
14. **Troubleshooting** — GPU unreachable, port 6080 already in use, drift refusal, MuJoCo reconversion wait, software-rendering slowness.

- [ ] **Step 2: Rewrite the README's Docker section**

In `README.md`, replace the whole `## 🐳 Docker Implementation` section (through the "For comprehensive Docker documentation, see:" list at line ~127) with a short section describing the command that actually exists:

```markdown
## 🐳 Containerised simulation

One command, on any machine, from a fresh clone:

```bash
./droid up
```

This detects your platform, starts a single container, builds the simulation
subset, launches Gazebo with RViz, and prints a URL. Open
<http://localhost:6080/vnc.html?autoconnect=1&resize=scale> and you will see the
robot. The same command and the same URL work on an amd64 Linux desktop, an Apple
Silicon Mac, a Jetson and a headless cloud instance, because the GUI is delivered
over noVNC rather than through the host's display stack.

```bash
./droid up --mujoco   # switch the simulation backend
./droid shell         # a shell inside the container
./droid doctor        # re-run the platform probe
./droid down          # stop, preserving anything you installed inside
```

Requires only bash and Docker. ROS 2 Jazzy only. See **[docs/container.md](docs/container.md)**
for the two rendering tiers, GPU setup, and what has and has not been verified.
```

Also fix the "Docker Troubleshooting" section (~line 336) to stop referencing `xhost +local:docker`, the docker group workflow for the old compose flow, and the deleted guides. Point at `docs/container.md`.

Update the changelog table's Docker row to note the consolidation to a single Jazzy container workflow.

- [ ] **Step 3: Add a pointer to both simulation quickstarts**

Near the top of `simulation_quickstart_gz.md`, immediately before the existing `colcon build` instructions, insert:

```markdown
> **Prefer not to install ROS 2 locally?** `./droid up` from the repository root
> runs this whole simulation in a container and serves the GUI to your browser,
> on Linux, macOS, Jetson or a headless cloud instance. See
> [docs/container.md](docs/container.md).
```

And in `simulation_quickstart_mujoco.md`, in the same position:

```markdown
> **Prefer not to install ROS 2 locally?** `./droid up --mujoco` from the
> repository root runs this simulation in a container and serves the GUI to your
> browser. The image ships a pre-warmed converter venv and MJCF cache, so an
> unmodified checkout skips the cold-start conversion described below. See
> [docs/container.md](docs/container.md).
```

- [ ] **Step 4: Note the container in `CLAUDE.md`**

Append to the "Build model: rebuild before trusting anything" section of `CLAUDE.md`:

```markdown
## The container

`./droid up` runs the whole simulation in a container (ROS 2 Jazzy, GUI over
noVNC at `http://localhost:6080`). The repository is bind-mounted, so it is the
developer's working tree that runs — but **the container's build semantics are
identical to the host's**: no `--symlink-install`, `install/` holds plain copies,
and the same stale-install trap applies. The container neutralises it by
rebuilding the simulation subset on every launch path rather than by changing
build semantics, so the rule above is unchanged wherever code runs. Build
artifacts live on volumes, separate from any native `build/` and `install/` in
the working tree. See `docs/container.md`.
```

- [ ] **Step 5: Delete the superseded tree**

```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2
git rm -r Docker
```

This removes the four Dockerfiles (Foxy, Humble, Jazzy and the unsuffixed one), the four run scripts, the three setup scripts, `Makefile`, `cmds.txt`, `docker-compose.yml`, `.dockerignore`, `env.example`, and the four superseded documents (`readme.md`, `QUICKSTART.md`, `DISTRO_GUIDE.md`, `WORKSPACE_OVERVIEW.md`). Git history retains all of them.

- [ ] **Step 6: Verify no dangling references to the deleted tree remain**

Run:
```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2
grep -rn --exclude-dir=.git --exclude-dir=docs/superpowers \
  -e 'Docker/' -e 'setup_r2d3' -e 'Dockerfile.humble' -e 'Dockerfile.foxy' \
  -e 'DISTRO_GUIDE' -e 'WORKSPACE_OVERVIEW' . || echo "no dangling references"
```
Expected: `no dangling references`. Any hit outside `docs/superpowers/` (which archives historical plans) must be fixed.

- [ ] **Step 7: Run the whole test suite**

Run: `cd /home/samzpc/code/r2d3/src/R2D3_ros2 && python3 -m pytest -v`
Expected: every test passes, including the pre-existing package tests.

- [ ] **Step 8: Commit**

```bash
git add docs/container.md README.md CLAUDE.md simulation_quickstart_gz.md simulation_quickstart_mujoco.md
git rm -r Docker
git commit -m "docs(container): add the canonical container guide and delete the superseded Docker tree"
```

---

## Task 8: Hand verification on the software-rendering tier

The completion bar. Nothing here is optional, and nothing may be claimed that was not observed.

**Files:** none — this task produces evidence, and fixes whatever it uncovers.

- [ ] **Step 1: Start clean and bring up the Gazebo backend**

```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2
./droid up --gpu cpu
```

Expected: the software-rendering-tier notice, then the noVNC URL, then a colcon build of the 15 simulation packages, then Gazebo and RViz starting.

`--gpu cpu` is required on this machine: the probe correctly hard-fails here because `docker run --gpus all` cannot acquire the RTX 4060.

- [ ] **Step 2: Confirm the simulation is actually running, not merely launched**

From a second terminal:

```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2
./droid shell
# inside the container:
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 topic echo /clock --once
ros2 control list_controllers
```

Expected: `/clock` produces a message and, sampled twice, advances. `list_controllers` shows the joint state broadcaster, diff drive, both arm controllers, the platform and neck controllers, all `active`.

Assert on simulation state, not on rendering — headless rendering here has a known intermittent failure whose symptom is the clock stalling rather than a clean error, so **if `/clock` does not advance, that is the failure, and it must be diagnosed rather than retried past.**

- [ ] **Step 3: Capture the browser screenshot**

Open `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`, confirm Gazebo and RViz are both visible with the robot rendered, and capture a screenshot as the evidence the verification bar requires.

- [ ] **Step 4: Verify the source tree stayed clean**

```bash
git status --porcelain
```
Expected: empty. A container session must not write into the bind-mounted tree.

- [ ] **Step 5: Verify the rebuild guarantee end-to-end**

Edit a value in a simulation YAML on the host, relaunch with `./droid up --gpu cpu`, and confirm the new value is in force inside the container. This is the footgun the pre-launch rebuild exists to neutralise; verify it rather than assuming it.

- [ ] **Step 6: Repeat for the MuJoCo backend**

```bash
./droid down
./droid up --mujoco --gpu cpu
```

Expected: on an unmodified checkout at the image's revision, the MJCF cache hits and the simulation starts without a multi-minute conversion. Verify `/clock` advances and controllers are active as in Step 2.

- [ ] **Step 7: Verify persistence and the consent gate**

```bash
./droid shell   # inside: sudo apt-get install -y htop; exit
./droid down
./droid up --gpu cpu   # inside a shell: `which htop` must still find it
```

Then confirm the drift gate fires: run `./droid up` with no `--gpu` override so resolution changes, and check that it **refuses**, names what changed, warns that container-local installs will be lost, and points at `--recreate`.

- [ ] **Step 8: Commit any fixes this task uncovered**

```bash
git add -A
git commit -m "fix(container): <what hand verification uncovered>"
```

If nothing needed fixing, skip the commit and say so plainly rather than manufacturing one.

---

## Verification bar for completion

Do not claim this work is complete without having run and observed each of these:

- `python3 -m pytest -v` from the repo root — full suite green, including the pre-existing package tests.
- `./droid doctor` — reproduces the known reference-machine state (`gpu_vendor=nvidia`, `docker_gpu=fail`, hard fail with remediation).
- `./droid up --gpu cpu` and `./droid up --mujoco --gpu cpu` — both reaching a running simulation with `/clock` advancing and controllers active, with a browser screenshot captured.
- `git status --porcelain` empty after a container session.
- `grep` for bash 4+ constructs in `droid` — no hits.

**What is explicitly NOT verified, and must be labelled as such rather than claimed:** the NVIDIA accelerated tier, Jetson, arm64 at runtime, and the macOS desktop experience. CI proves both architectures **build**; it does not prove arm64 **boots**.
