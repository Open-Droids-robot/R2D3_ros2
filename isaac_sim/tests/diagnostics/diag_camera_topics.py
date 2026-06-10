"""Verify the 3-camera ROS wiring: build the action graph and confirm the head +
both wrist camera publisher nodes + topics exist."""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import omni.usd
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim import sensors as sensors_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        import omni.kit.app
        em = omni.kit.app.get_app().get_extension_manager()
        for ext in ("isaacsim.ros2.core", "isaacsim.ros2.nodes", "isaacsim.ros2.bridge"):
            if not em.is_extension_enabled(ext):
                em.set_extension_enabled_immediate(ext, True)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        print("[topics] reset+init OK; building graph...", flush=True)
        try:
            sensors_mod.build_action_graph(publish_clock=True, publish_tf=True, publish_camera=True)
            print("[topics] build_action_graph OK", flush=True)
        except Exception:
            import traceback; traceback.print_exc()
            print("[topics] BUILD FAILED", flush=True); raise
        for _ in range(40):
            world.step(render=True)
        print("[topics] stepped 40", flush=True)

        stage = omni.usd.get_context().get_stage()
        nodes = sorted(p.GetName() for p in stage.Traverse()
                       if p.GetPath().pathString.startswith(sensors_mod.ACTION_GRAPH_PATH)
                       and p.GetName().startswith(("Rgb_", "Depth_", "InfoColor_", "InfoDepth_")))
        print(f"[topics] camera graph nodes ({len(nodes)}): {nodes}", flush=True)

        # ROS topic list via bundled rclpy
        try:
            import rclpy
            from rclpy.node import Node
            if not rclpy.ok():
                rclpy.init()
            n = Node("cam_topic_probe")
            import time; time.sleep(1.0)
            names = [t for t, _ in n.get_topic_names_and_types()
                     if "camera" in t and ("image_raw" in t or "camera_info" in t)]
            print(f"[topics] camera ROS topics ({len(names)}):", flush=True)
            for t in sorted(names):
                print(f"[topics]   {t}", flush=True)
            n.destroy_node()
        except Exception as e:  # noqa: BLE001
            print(f"[topics] rclpy topic list skipped: {e}", flush=True)
        print("[topics] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
