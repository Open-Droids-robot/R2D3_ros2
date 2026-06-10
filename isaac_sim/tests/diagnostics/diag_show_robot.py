"""Quick scene-only render to check the dexterous hands (model + orientation).

Loads the converted USD, disables gravity on the articulation, renders a
third-person frame + a close-up of the left hand. No Robot control needed.
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

OUT = _REPO / "isaac_sim/tests/captures"


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image
        from pxr import UsdGeom, Gf, PhysxSchema
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        stage = omni.usd.get_context().get_stage()
        # disable gravity on every rigid body so it holds pose without drives/Robot
        for prim in stage.Traverse():
            if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI(prim).CreateDisableGravityAttr(True)
        world.reset()
        for _ in range(8):
            world.step(render=True)

        def find(n):
            return next((p.GetPath().pathString for p in stage.Traverse()
                         if p.GetName() == n and p.GetTypeName() == "Xform"), None)
        lhand = find("l_dex_hand_base_link")
        m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(lhand))
        ho = np.array(m.Transform(Gf.Vec3d(0, 0, 0)))
        print(f"[show] l_dex_hand_base_link world={ho.round(3)}", flush=True)

        def render(name, eye, look, res=(1100, 750)):
            cam = rep.functional.create.camera(position=tuple(float(v) for v in eye),
                                               look_at=tuple(float(v) for v in look))
            rp = rep.create.render_product(str(cam.GetPath()), res)
            a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
            for _ in range(14):
                world.step(render=True)
            img = np.asarray(a.get_data(do_array_copy=True))[:, :, :3].astype(np.uint8)
            Image.fromarray(img, "RGB").save(OUT / name)
            a.detach(); rp.destroy()
            print(f"[show] wrote {name}", flush=True)

        rmin, rmax = scene_mod.world_range(rpath)
        ctr = (rmin + rmax) / 2; rad = 0.5 * float(np.linalg.norm(rmax - rmin))
        d = np.array([0.7, -0.7, 0.2]); d /= np.linalg.norm(d)
        render("dex_overview.png", ctr + 2.2 * rad * d, ctr)
        # close-up on the left hand
        d2 = np.array([0.6, -0.7, 0.25]); d2 /= np.linalg.norm(d2)
        render("dex_lhand.png", ho + 0.45 * d2, ho)
        print("[show] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
