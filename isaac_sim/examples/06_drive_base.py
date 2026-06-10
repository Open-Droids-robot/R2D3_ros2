"""Drive the mobile base forward then rotate 90 degrees, wheels rolling.

Needs the MOBILE build (wheels kept revolute):
    bash isaac_sim/urdf/render.sh dexterous 0
    scripts/urdf_to_usd.py --urdf isaac_sim/urdf/r2d3_v1_dexterous_mobile.urdf --usd-dir isaac_sim/usd_mobile
Then:
    scripts/isaacsim_ros2.sh isaac_sim/examples/06_drive_base.py

The AGV drive wheels are on a lateral X axle, so the rolling direction is +Y.
The base is moved kinematically (set_base_pose); the wheels are spun to match
(set_joint_targets). See docs/platform.md for why driving is kinematic in V1.
"""
import numpy as np
from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim import helpers as h

WHEEL_R = 0.05


def main() -> int:
    with R2D3(end_effector="dexterous", headless=True, mobile=True) as sim:
        sim.reset()
        p0, _ = sim.robot.get_base_pose()
        up = np.array([1.0, 0.0, 0.0, 0.0])

        # drive forward +1 m along +Y, spinning the drive + caster wheels
        for i in range(40):
            d = 1.0 * (i + 1) / 40
            sim.set_base_pose(p0 + np.array([0.0, d, 0.0]), up)
            ang = d / WHEEL_R
            sim.set_joint_targets({
                "joint_left_wheel": ang, "joint_right_wheel": -ang,
                **{f"joint_swivel_wheel_{k}_2": ang for k in (1, 2, 3, 4)},
            })
            sim.step()

        # rotate 90 deg about +Z in place
        p_mid = p0 + np.array([0.0, 1.0, 0.0])
        for i in range(30):
            sim.set_base_pose(p_mid, h.yaw_quat(90.0 * (i + 1) / 30))
            sim.step()

        end_p, end_q = sim.robot.get_base_pose()
        print(f"[drive] base end pos={end_p.round(3)} quat={end_q.round(3)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
