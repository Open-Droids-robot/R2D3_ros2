"""Mobile-base move task: drive FORWARD then ROTATE 90 degrees, wheels rolling.

Uses the MOBILE build (isaac_sim/usd_mobile/, AGV wheels kept revolute by
scripts/build_robot.sh / render.sh dexterous 0). The base is moved kinematically
(set_world_pose, gravity off, root_joint freed); the wheels are spun through the
physics API (robot._apply drive targets) so it's stable (editing meshes mid-sim
invalidates the physics view). The 2 drive wheels roll about their lateral X
axle, so the AGV forward is +Y; we drive +Y (wheels roll) then yaw 90 about Z
(drive wheels counter-rotate, casters steer).

Run: scripts/isaacsim_ros2.sh isaac_sim/tests/move_task.py
"""
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

FORWARD_M = 1.0
TURN_DEG = 90.0
WHEEL_R = 0.05
HALF_TRACK = 0.148

DRIVE_WHEELS = ["joint_left_wheel", "joint_right_wheel"]
CASTER_ROLL = [f"joint_swivel_wheel_{i}_2" for i in (1, 2, 3, 4)]
CASTER_STEER = [f"joint_swivel_wheel_{i}_1" for i in (1, 2, 3, 4)]


def qmul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz, aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx, aw*bz + ax*by - ay*bx + az*bw)


def yaw_quat(deg):
    h = math.radians(deg) / 2.0
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image
        from pxr import UsdPhysics
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world, usd_path=USD_MOBILE)
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.GetName() == "root_joint":
                UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
                print("[move] disabled root_joint (base is now free)", flush=True)
                break

        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize(); robot.go_home()
        # which wheel joints exist as DOFs?
        idx = robot.joint_index
        rolls = [j for j in (DRIVE_WHEELS + CASTER_ROLL) if j in idx]
        steers = [j for j in CASTER_STEER if j in idx]
        print(f"[move] wheel DOFs: {len(rolls)} roll, {len(steers)} steer", flush=True)
        for _ in range(60):
            world.step(render=False)

        p0, _ = robot._art.get_world_pose()
        p0 = np.asarray(p0, float); q_up = (1.0, 0.0, 0.0, 0.0)
        # PHYSICS/PLACEMENT: the loaded base sits at z~-0.495 with the wheels
        # ~0.20 m below it, so it ends up below a z=0 floor. Raise the robot so its
        # LOWEST point (the wheels) rests on the floor at z=0 — unambiguous, no
        # negative-z. (Base is kinematic + gravity off, so it can't sink/fall.)
        rmin, _rmax = scene_mod.world_range(rpath)
        raise_z = -float(rmin[2]) + 0.005
        BASE = p0 + np.array([0.0, 0.0, raise_z])
        robot._art.set_world_pose(position=BASE, orientation=np.array(q_up, float))
        for _ in range(8):
            world.step(render=False)
        print(f"[move] start={p0.round(3)} raised {raise_z:.3f} -> wheels on floor z=0", flush=True)

        # Coloured studio: slate floor + dark walls so the WHITE robot stands out;
        # darken the dome so the background isn't a washed-out white.
        from pxr import UsdLux
        for ln, inten in (("/DomeLight", 130.0), ("/KeyLight", 3400.0), ("/FillLight", 2400.0)):
            lp = stage.GetPrimAtPath(ln)
            if lp: UsdLux.LightAPI(lp).GetIntensityAttr().Set(inten)
        scene_mod.add_visual_box("/Preview/floor", (BASE[0], BASE[1]+0.6, -0.01),
                                 (16.0, 16.0, 0.02), (0.22, 0.36, 0.50))      # slate-blue floor
        scene_mod.add_visual_box("/Preview/wall_back", (BASE[0]-2.3, BASE[1]+0.6, 1.4),
                                 (0.1, 16.0, 5.0), (0.13, 0.16, 0.24))        # dark wall behind (-X)
        scene_mod.add_visual_box("/Preview/wall_end", (BASE[0], BASE[1]+3.4, 1.4),
                                 (16.0, 0.1, 5.0), (0.17, 0.13, 0.22))        # plum end wall (+Y)

        ctr = np.array([BASE[0], BASE[1]+FORWARD_M*0.5, 0.55])
        eye = ctr + np.array([3.2, -1.4, 0.7])
        cam = rep.functional.create.camera(position=tuple(float(v) for v in eye),
                                            look_at=tuple(float(v) for v in ctr))
        rp = rep.create.render_product(str(cam.GetPath()), (960, 540))
        ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach(rp)
        frames = []
        def grab():
            a = np.asarray(ann.get_data(do_array_copy=True))
            if a.ndim == 3 and a.shape[2] == 4: a = a[:, :, :3]
            frames.append(Image.fromarray(a.astype(np.uint8)).resize((480, 270)))

        def drive(pos, quat, roll=0.0, diff=0.0, steer=0.0):
            robot._art.set_world_pose(position=np.array(pos, float), orientation=np.array(quat, float))
            robot.go_home()
            # wheel drive targets (rad). diff = extra counter-rotation for turn-in-place.
            names, vals = [], []
            for j in rolls:
                s = -1.0 if "right" in j else 1.0
                d = diff * (-1.0 if "right" in j else 1.0) if j in DRIVE_WHEELS else 0.0
                names.append(j); vals.append(s * roll + d)
            for j in steers:
                names.append(j); vals.append(steer)
            if names:
                robot._apply(names, vals)
            world.step(render=True)

        for _ in range(12):
            drive(BASE, q_up); world.step(render=True)
        for _ in range(6):
            drive(BASE, q_up); grab()

        print("[move] drive forward (+Y), wheels roll", flush=True)
        N1 = 48
        for i in range(N1):
            a = (i + 1) / N1
            dist = FORWARD_M * a
            drive(BASE + np.array([0.0, dist, 0.0]), q_up, roll=dist / WHEEL_R)
            grab()
        p_mid = BASE + np.array([0.0, FORWARD_M, 0.0])
        roll0 = FORWARD_M / WHEEL_R

        print("[move] rotate 90 (drive wheels counter-rotate, casters steer)", flush=True)
        N2 = 40
        for i in range(N2):
            a = (i + 1) / N2
            yaw = TURN_DEG * a
            arc = math.radians(yaw) * HALF_TRACK / WHEEL_R
            drive(p_mid, qmul(yaw_quat(yaw), q_up), roll=roll0, diff=arc, steer=math.radians(yaw))
            grab()
        for _ in range(8):
            drive(p_mid, qmul(yaw_quat(TURN_DEG), q_up), roll=roll0,
                  diff=math.radians(TURN_DEG)*HALF_TRACK/WHEEL_R, steer=math.radians(TURN_DEG))
            grab()

        ann.detach(); rp.destroy()
        if frames:
            frames[0].save(OUT/"move_task.gif", save_all=True, append_images=frames[1:], duration=70, loop=0)
            frames[0].save(OUT/"move_task_first.png"); frames[-1].save(OUT/"move_task_last.png")
            frames[len(frames)//2].save(OUT/"move_task_mid.png")
            frames[int(len(frames)*0.30)].save(OUT/"move_task_q.png")
            print(f"[move] wrote move_task.gif ({len(frames)} frames)", flush=True)
        print("[move] DONE", flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
