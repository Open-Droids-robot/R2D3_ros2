"""R2D3 Isaac Sim bring-up entry point.

Launches a SimulationApp, loads the R2D3 USD, binds the articulation,
and runs the sim loop forever. Later milestones layer OmniGraph wiring
(sensors.py) and the rclpy sim_adapter on top — for M1 this script
just verifies the robot loads and stands.

Usage
-----

    scripts/isaacsim_ros2.sh isaac_sim/r2d3_sim/bring_up.py [--headless] [--no-ros]

Flags
-----

    --headless   Run without the Kit UI window (default: GUI). Required on
                 hosts without an X display unless TurboVNC is set up.
    --no-ros    Skip rclpy / OmniGraph ROS wiring (M1 verification only).
                 Currently the *default* for M1 since the ROS wiring is
                 implemented in M2 / M3.
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
from pathlib import Path

# Make `isaac_sim.r2d3_sim.*` importable when this script is invoked directly
# via `scripts/isaacsim_ros2.sh isaac_sim/r2d3_sim/bring_up.py` (the launcher
# exec's Python with the script path, not `-m`). When invoked as a module, this
# is a no-op since the repo root is already on sys.path.
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# Default head pose — D435 should see a table at lift = 1.0 m. Locked here as a
# bring-up constant; tasks override via their YAML once the engine lands.
DEFAULT_HEAD_PAN_RAD = 0.0
DEFAULT_HEAD_TILT_RAD = -0.20

DEFAULT_LIFT_M = 0.5


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="bring_up",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--headless", action="store_true")
    p.add_argument(
        "--no-ros", action="store_true",
        help="Skip rclpy adapter (M1 viewer-only verification).",
    )
    p.add_argument(
        "--max-steps", type=int, default=0,
        help="Stop after this many physics steps (0 = run forever).",
    )
    p.add_argument(
        "--demo-workspace", action="store_true",
        help="Place a table+cube in front + a front fill light and tilt the "
             "head down, for capturing the live head-D435 view of a workspace.",
    )
    return p


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s %(levelname)s] %(message)s",
    )
    log = logging.getLogger("bring_up")

    args = _build_arg_parser().parse_args()

    # SimulationApp must be the first heavy import — it patches sys.path so the
    # rest of the omni.* / isaacsim.* modules are findable.
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    os.environ.setdefault("PRIVACY_CONSENT", "Y")

    from isaacsim import SimulationApp
    sim_app = SimulationApp({"headless": args.headless})

    # Enable the ROS 2 bridge extension — base.python.kit doesn't list it,
    # so OmniGraph node types like ROS2PublishClock aren't registered until
    # we ask for it explicitly. Doing this BEFORE world.reset() ensures
    # the OmniGraph TF / clock / camera nodes are available at graph build
    # time.
    import omni.kit.app
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in ("isaacsim.ros2.core", "isaacsim.ros2.nodes",
                "isaacsim.ros2.bridge"):
        if not ext_manager.is_extension_enabled(ext):
            ext_manager.set_extension_enabled_immediate(ext, True)

    executor = None
    executor_thread = None
    try:
        # Local imports must come AFTER SimulationApp(), or omni.* isn't loaded.
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot, HOME_LIFT_M

        world = World(stage_units_in_meters=1.0)
        robot_prim_path = scene_mod.assemble(world)

        # First reset initializes physics handles for the loaded USD.
        world.reset()

        robot = Robot(prim_path=robot_prim_path)
        robot.initialize()
        print(f"[bring_up] M1 ready: {robot.num_dof} DOFs; "
              f"initial lift={robot.get_lift_m():.3f} m", flush=True)

        # --- M2: OmniGraph fast path (clock / TF / D435) -------------------
        if not args.no_ros:
            try:
                from isaac_sim.r2d3_sim import sensors as sensors_mod
                sensors_mod.build_action_graph()
                # Action graph nodes only fire when the timeline is playing.
                import omni.timeline
                omni.timeline.get_timeline_interface().play()
                print("[bring_up] OmniGraph wired (clock, tf, camera) "
                      "and timeline playing", flush=True)
            except Exception as e:
                import traceback
                print(f"[bring_up] WARN: OmniGraph wiring failed: {e}",
                      flush=True)
                traceback.print_exc()

        # --- Optional: stand up the rclpy adapter (M3 / M4) ----------------
        adapter = None
        cache = None
        if not args.no_ros:
            try:
                # Make Isaac's bundled humble rclpy importable. The isaacsim.ros2
                # bridge extension would normally add this, but it isn't enabled
                # in the base.python.kit experience.
                _BUNDLED_HUMBLE = Path(
                    "/usr1/home/semathew/miniforge3/envs/isaac/lib/python3.12/"
                    "site-packages/isaacsim/exts/isaacsim.ros2.core/humble/rclpy"
                )
                if _BUNDLED_HUMBLE.is_dir() and str(_BUNDLED_HUMBLE) not in sys.path:
                    sys.path.insert(0, str(_BUNDLED_HUMBLE))
                # And the LD path so the .so files resolve.
                _BUNDLED_LIB = _BUNDLED_HUMBLE.parent / "lib"
                cur_ld = os.environ.get("LD_LIBRARY_PATH", "")
                if str(_BUNDLED_LIB) not in cur_ld:
                    os.environ["LD_LIBRARY_PATH"] = f"{_BUNDLED_LIB}:{cur_ld}"

                print("[bring_up] importing rclpy...", flush=True)
                import rclpy
                from rclpy.executors import MultiThreadedExecutor
                from isaac_sim.r2d3_sim.sim_adapter import (
                    SimAdapter, LatestCommandCache, flush_cache_into_robot,
                )
                print(f"[bring_up] rclpy from: {rclpy.__file__}", flush=True)
                rclpy.init()
                cache = LatestCommandCache()
                joint_names_for_publish = list(robot.joint_index.keys())
                adapter = SimAdapter(cache=cache, joint_names=joint_names_for_publish)

                executor = MultiThreadedExecutor()
                executor.add_node(adapter)
                executor_thread = threading.Thread(
                    target=executor.spin, name="r2d3-rclpy-spin", daemon=True,
                )
                executor_thread.start()
                print("[bring_up] rclpy adapter spinning on background thread.",
                      flush=True)
            except Exception as e:
                import traceback
                print(f"[bring_up] FATAL rclpy setup failed: {e}", flush=True)
                traceback.print_exc()
                raise
        else:
            print("[bring_up] --no-ros: skipping rclpy adapter.", flush=True)

        # --- Sim loop -------------------------------------------------------
        print(f"[bring_up] entering sim loop; max_steps={args.max_steps}; "
              f"is_running={sim_app.is_running()}", flush=True)
        # Command the home pose once — the position drives hold it (and the
        # AGV wheels are welded to fixed in the USD, so no per-step holding or
        # velocity-zeroing is needed anymore; gravity is disabled on the
        # articulation in Robot._configure_drives()).
        robot.go_home()
        print(f"[bring_up] commanded home; position drives converging "
              f"(lift target={HOME_LIFT_M:.3f} m)", flush=True)

        if args.demo_workspace:
            # Table+cube in front (clear of the body), a front fill light so the
            # camera-facing faces aren't black, and head tilted down to look at
            # it. Rendered via the LIVE ROS camera (correct tonemapping), unlike
            # the offline replicator path which washes out.
            import numpy as np
            from pxr import UsdLux, UsdGeom, Sdf, Gf
            import omni.usd as _ou
            _stage = _ou.get_context().get_stage()
            ground_z = float(scene_mod.world_range(robot_prim_path)[0][2])
            # Darken the scene lights so the bright dome background doesn't drive
            # the camera's auto-exposure to wash out the lit workspace.
            for _ln, _i in (("/DomeLight", 80.0), ("/KeyLight", 600.0), ("/FillLight", 300.0)):
                _lp = _stage.GetPrimAtPath(_ln)
                if _lp:
                    UsdLux.LightAPI(_lp).GetIntensityAttr().Set(_i)
            _cube = (1.05, -0.22, 0.46)        # forward of the robot, low
            _top = _cube[2] - 0.03
            scene_mod.add_visual_box("/Preview/ground", (0.6, -0.22, ground_z - 0.01),
                                     (8.0, 8.0, 0.02), (0.32, 0.36, 0.44))
            scene_mod.add_visual_box("/Preview/bench",
                                     (_cube[0], _cube[1], ground_z + (_top - ground_z) / 2),
                                     (0.34, 0.34, _top - ground_z), (0.62, 0.46, 0.30))
            scene_mod.add_visual_box("/Preview/cube", _cube, (0.06, 0.06, 0.06),
                                     (0.90, 0.12, 0.12))
            _fl = UsdLux.SphereLight.Define(_stage, Sdf.Path("/Preview/WorkFront"))
            _fl.CreateIntensityAttr(6.0e4); _fl.CreateRadiusAttr(0.18)
            _fx = UsdGeom.Xformable(_fl.GetPrim()); _fx.ClearXformOpOrder()
            _fx.AddTranslateOp().Set(Gf.Vec3d(_cube[0] - 0.45, _cube[1], _top + 0.30))
            _tl = UsdLux.SphereLight.Define(_stage, Sdf.Path("/Preview/WorkTop"))
            _tl.CreateIntensityAttr(3.0e4); _tl.CreateRadiusAttr(0.18)
            _tx = UsdGeom.Xformable(_tl.GetPrim()); _tx.ClearXformOpOrder()
            _tx.AddTranslateOp().Set(Gf.Vec3d(_cube[0], _cube[1], _top + 0.6))
            robot.set_head(0.0, -0.45)
            print("[bring_up] --demo-workspace: props + front light placed, head down.",
                  flush=True)

        step = 0
        while sim_app.is_running():
            if cache is not None:
                flush_cache_into_robot(cache, robot)
            world.step(render=True)
            step += 1
            if adapter is not None:
                adapter.update_state(
                    joint_positions=robot.get_joint_positions(),
                    wrench_left=robot.get_wrist_wrench("left"),
                    wrench_right=robot.get_wrist_wrench("right"),
                )
                if step == 1:
                    adapter.mark_ready()
            if step in (1, 2, 3, 5, 8, 13, 21, 34, 55) or step % 100 == 0:
                positions = robot.get_joint_positions()
                # Find first joint with NaN, with the step it appeared.
                import math
                nans = {n: v for n, v in positions.items() if math.isnan(v)}
                if step == 1 or (step <= 100 and nans):
                    summary = ", ".join(f"{n}={v:.4f}" for n, v in
                                        list(positions.items())[:5])
                    nan_count = len(nans)
                    nan_names = list(nans.keys())[:3]
                    print(f"[bring_up] step {step}: nan_count={nan_count} "
                          f"first_nans={nan_names} sample={summary}",
                          flush=True)
                elif step % 100 == 0:
                    print(f"[bring_up] step {step}: all-nan={len(nans)==len(positions)}",
                          flush=True)
            if args.max_steps and step >= args.max_steps:
                print(f"[bring_up] reached --max-steps={args.max_steps}, exiting cleanly.",
                      flush=True)
                break

        return 0

    finally:
        if executor is not None:
            executor.shutdown()
        if executor_thread is not None:
            executor_thread.join(timeout=2.0)
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
