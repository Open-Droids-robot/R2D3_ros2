#!/usr/bin/env python3
"""Pre-warm the MuJoCo URDF->MJCF cache under the EXACT key `mujoco_sim.launch.py`
will look up at runtime, and fail the image build if it is not.

Why this is a Python driver and not a few lines of shell
--------------------------------------------------------
The cache is content-addressed over the xacro-generated robot description
(`ensure_mjcf.compute_checksum(description, world_text, CONVERTER_ARGS_VERSION)`),
so the pre-warm is a HIT only if the description it hands the converter is
byte-identical to the one the launch file hands it. Two things kept breaking that,
both silently -- the cache still existed, so every build-time assertion still
passed, and the only symptom was a slow first launch:

  1. `description="$(xacro ...)"` in bash. Command substitution strips ALL trailing
     newlines, so the description ended `</robot>`. `launch.substitutions.Command`
     returns `subprocess.run(..., stdout=PIPE, universal_newlines=True).stdout`
     with no strip at all, so the launch's description ends `</robot>\\n`. One byte,
     a completely different sha256, a guaranteed miss. Measured in the shipped
     image: 73510 bytes vs 73511, checksum 33f7ba16... vs 2c302a63... . This file
     therefore uses the real `Command` substitution object; there is no shell
     anywhere in the path that produces the description.

  2. The launch file's argument defaults were hand-copied into the Dockerfile.
     Change `robot_model`, `world` or `headless` in the launch file and the
     pre-warm quietly warms a key nobody asks for. This file reads the defaults
     out of the launch file's own `DeclareLaunchArgument` entities instead.

The final assertion is the guard for both, and for anything else that could ever
drift (a `CONVERTER_ARGS_VERSION` bump, a changed world file, a converter flag):
the checksum recomputed from the launch-form description MUST equal the checksum
file the converter wrote. If it does not, the layer fails loudly rather than
shipping an image whose "pre-warmed" cache is dead weight.

Why it drives `ensure_mjcf.py` directly instead of running the launch file
-------------------------------------------------------------------------
`headless` is an argument to the xacro, not just a runtime flag, so passing
`headless:=true` to dodge the MuJoCo window would itself change the description
and therefore the cache key. With the real default (`false`) the launch brings up
`mujoco_ros2_control`, which wants a display no build layer has. Driving the
converter directly with the launch's own defaults warms the right key with none of
that. It also skips `controller_manager`, six spawners and three composable
containers, none of which contribute to the cache.

The converter is invoked as the installed script rather than through `ros2 run`:
`ros2 run` Popen()s the script as a child, so the PID we hold would be the
launcher and SIGTERM would stop the wrong process. `ensure_mjcf.py` restores
default signal handling before it latches the model and spins, so SIGTERM ends it
cleanly once the cache is on disk.
"""

import importlib.util
import os
import subprocess
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch.substitutions import Command, FindExecutable
from launch.utilities import perform_substitutions

PACKAGE = "r2d3_mujoco"
LAUNCH_FILE = "mujoco_sim.launch.py"
XACRO_FILE = "r2d3_mujoco.urdf.xacro"
TOPIC = "/mujoco_robot_description"

# Backstop only. The mechanism is the poll below; a fixed sleep would be either
# wasted build time or a truncated conversion.
POLL_INTERVAL_S = 5.0
TIMEOUT_S = 1200.0


def fail(message):
    print(f"prewarm: {message}", file=sys.stderr, flush=True)
    sys.exit(1)


def load_ensure_mjcf(prefix):
    """Import the INSTALLED ensure_mjcf.py as a module.

    Imported rather than re-implemented so `compute_checksum`,
    `CONVERTER_ARGS_VERSION`, `default_cache_root()` and the cache filenames are
    the converter's own, not a second copy that can disagree with it.
    """
    script = Path(prefix) / "lib" / PACKAGE / "ensure_mjcf.py"
    if not script.is_file():
        fail(f"installed converter script not found at {script}")
    spec = importlib.util.spec_from_file_location("ensure_mjcf", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return script, module


def launch_argument_defaults(launch_path):
    """{name: default_value} read from the launch file's DeclareLaunchArgument entities.

    Loading the launch file and walking its entities, rather than copying the
    defaults into this file, is what keeps the pre-warm and the launch in step.
    """
    description = get_launch_description_from_python_launch_file(str(launch_path))
    context = LaunchContext()
    defaults = {}
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
            defaults[entity.name] = perform_substitutions(context, list(entity.default_value))
    return defaults


def launch_form_robot_description(xacro_path, robot_model, headless):
    """The robot description exactly as `mujoco_sim.launch.py` produces it.

    Uses the real `launch.substitutions.Command`, so the capture semantics that
    decide the cache key -- `stdout=PIPE`, `universal_newlines=True`, and above all
    NO strip of the trailing newline -- are not mirrored here but literally the
    same code the launch runs.
    """
    return Command([
        FindExecutable(name="xacro"), " ", str(xacro_path),
        " arm_model:=", robot_model,
        " headless:=", headless,
    ]).perform(LaunchContext())


def run_converter(script, description, world, model, cache_dir, mjcf_name, checksum_name):
    """Run the converter until the cache files appear, then stop it."""
    child = subprocess.Popen([
        str(script),
        "--robot-description", description,
        "--world", str(world),
        "--model", model,
        "--topic", TOPIC,
    ])
    checksum_file = cache_dir / checksum_name
    mjcf_file = cache_dir / mjcf_name
    deadline = time.monotonic() + TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            if checksum_file.is_file() and mjcf_file.is_file():
                break
            if child.poll() is not None:
                break
            time.sleep(POLL_INTERVAL_S)
    finally:
        if child.poll() is None:
            child.terminate()
        try:
            child.wait(timeout=30)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=30)


def main():
    prefix = get_package_prefix(PACKAGE)
    share = Path(get_package_share_directory(PACKAGE))
    script, ensure_mjcf = load_ensure_mjcf(prefix)

    defaults = launch_argument_defaults(share / "launch" / LAUNCH_FILE)
    missing = [k for k in ("robot_model", "world", "headless") if k not in defaults]
    if missing:
        fail(f"{LAUNCH_FILE} no longer declares {missing}; the pre-warm cannot "
             f"derive the cache key it must warm")
    robot_model = defaults["robot_model"]
    world = Path(defaults["world"])
    headless = defaults["headless"]
    print(f"prewarm: launch defaults -> robot_model={robot_model} headless={headless} "
          f"world={world}", flush=True)

    description = launch_form_robot_description(share / "urdf" / XACRO_FILE,
                                                robot_model, headless)
    expected = ensure_mjcf.compute_checksum(
        description, world.read_text(), ensure_mjcf.CONVERTER_ARGS_VERSION)
    print(f"prewarm: description {len(description)} chars, "
          f"ends {description[-9:]!r}; target key {expected}", flush=True)

    cache_dir = ensure_mjcf.default_cache_root() / robot_model
    checksum_file = cache_dir / ensure_mjcf.CHECKSUM_FILENAME

    # A checksum file for a DIFFERENT key must go before the converter starts. The
    # poll below breaks as soon as the cache files exist, and `ensure_mjcf.py` only
    # unlinks a stale checksum a moment after it starts -- so a pre-existing wrong
    # key could be observed and accepted in that window, and the assertion at the
    # end would then be comparing against a file this run never wrote. Empty in the
    # image build; this matters when the driver is re-run by hand over a warm
    # ~/.ros (a developer warming a feature branch).
    if checksum_file.is_file() and checksum_file.read_text().strip() != expected:
        print("prewarm: discarding a cache entry under a different key", flush=True)
        checksum_file.unlink()

    run_converter(script, description, world, robot_model, cache_dir,
                  ensure_mjcf.MJCF_FILENAME, ensure_mjcf.CHECKSUM_FILENAME)

    mjcf_file = cache_dir / ensure_mjcf.MJCF_FILENAME
    if not checksum_file.is_file():
        fail("no checksum -- the MuJoCo conversion did not complete")
    if not mjcf_file.is_file():
        fail("no MJCF -- the MuJoCo conversion did not complete")

    venv = Path(os.path.expanduser("~")) / ".ros/ros2_control/.venv/bin/python3"
    if not os.access(venv, os.X_OK):
        fail(f"converter venv missing at {venv} -- first launch would rebuild it")

    stored = checksum_file.read_text().strip()
    print(f"prewarm: stored key   {stored}", flush=True)
    if stored != expected:
        fail(f"cache key mismatch -- the pre-warmed cache would MISS on the first "
             f"launch.\n  expected (launch-form description): {expected}\n"
             f"  stored   (what the converter wrote)  : {stored}\n"
             f"Something changed the description the launch produces, the world "
             f"file, or ensure_mjcf.CONVERTER_ARGS_VERSION.")

    print(f"prewarm: OK -- cached key matches the key mujoco_sim.launch.py will "
          f"look up ({stored})", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
