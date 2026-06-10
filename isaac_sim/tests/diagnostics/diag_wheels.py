"""Verify + show the AGV wheels turning (mobile build). Adds a red rim marker to
the left drive wheel, spins the drive wheels in place, and renders a close-up
down the axle (the marker orbits => the wheel is turning). Prints the wheel DOF
positions before/after to confirm they actually moved."""
from __future__ import annotations
import os, sys, math
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")
os.environ["R2D3_EE"] = "dexterous"
OUT = _REPO / "isaac_sim/tests/captures"
USD_MOBILE = _REPO / "isaac_sim/usd_mobile/r2d3_v1.usda"
DRIVE_WHEELS = ["joint_left_wheel", "joint_right_wheel"]


def main() -> int:
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image
        from pxr import UsdGeom, Gf
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world, usd_path=USD_MOBILE)
        stage = omni.usd.get_context().get_stage()
        # red rim marker as a child of the left drive wheel link (rotates with it)
        lw = next((p for p in stage.Traverse()
                   if p.GetName() == "link_left_wheel" and p.GetTypeName() == "Xform"), None)
        marker_path = lw.GetPath().pathString + "/rim_marker"
        scene_mod.add_visual_box(marker_path, (0, 0, 0), (0.02, 0.02, 0.02), (0.9, 0.1, 0.1))
        # offset it to the rim, in the wheel plane (axle is local X -> rim in local Y)
        mk = UsdGeom.Xformable(stage.GetPrimAtPath(marker_path)); mk.ClearXformOpOrder()
        mk.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.045, 0.0)); mk.AddScaleOp().Set(Gf.Vec3f(0.03, 0.03, 0.03))

        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize(); robot.go_home()
        for _ in range(40):
            world.step(render=False)
        idx = robot.joint_index
        wj = [j for j in DRIVE_WHEELS if j in idx]
        def wpos():
            p = robot._art.get_joint_positions()
            return {j: round(float(p[idx[j]]), 3) for j in wj}
        print(f"[wheel] drive-wheel DOFs found: {wj}", flush=True)
        print(f"[wheel] positions BEFORE spin: {wpos()}", flush=True)

        def wp(name):
            o = UsdGeom.XformCache().GetLocalToWorldTransform(
                next(p for p in stage.Traverse() if p.GetName()==name and p.GetTypeName()=="Xform")
            ).Transform(Gf.Vec3d(0,0,0)); return np.array([o[0],o[1],o[2]])
        W = wp("link_left_wheel")
        print(f"[wheel] left wheel world pos = {W.round(3)}", flush=True)
        # camera along +X axle but from ABOVE (downward angle — rep look_at needs
        # it) so the rim marker's orbit reads as the wheel turning
        cam = rep.functional.create.camera(position=tuple(float(v) for v in (W + np.array([0.32, 0.0, 0.26]))),
                                            look_at=tuple(float(v) for v in W))
        rp = rep.create.render_product(str(cam.GetPath()), (700, 700))
        ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach(rp)
        frames = []
        def grab():
            a = np.asarray(ann.get_data(do_array_copy=True))
            if a.ndim == 3 and a.shape[2] == 4: a = a[:, :, :3]
            frames.append(Image.fromarray(a.astype(np.uint8)).resize((360, 360)))
        for _ in range(12):
            world.step(render=True)

        # spin the drive wheels ~2.5 turns in place
        N = 60
        total = 2.5 * 2 * math.pi
        for i in range(N):
            a = total * (i + 1) / N
            robot._apply(wj, [a if "left" in j else -a for j in wj])  # both roll same world dir
            world.step(render=True)
            grab()
        print(f"[wheel] positions AFTER spin: {wpos()}", flush=True)

        ann.detach(); rp.destroy()
        if frames:
            frames[0].save(OUT/"wheel_spin.gif", save_all=True, append_images=frames[1:], duration=60, loop=0)
            frames[len(frames)//4].save(OUT/"wheel_spin_a.png"); frames[len(frames)//2].save(OUT/"wheel_spin_b.png")
            print(f"[wheel] wrote wheel_spin.gif ({len(frames)} frames)", flush=True)
        print("[wheel] DONE", flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
