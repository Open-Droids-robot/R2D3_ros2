"""Hello, R2D3 — minimal control + sensing with the platform SDK.

    scripts/isaacsim_ros2.sh isaac_sim/examples/01_hello_robot.py
"""
from isaac_sim.r2d3_sim import R2D3


def main() -> int:
    # Boot Isaac + load the robot (dexterous hand). enable_cameras=True by default.
    with R2D3(end_effector="dexterous", headless=True) as sim:
        sim.reset()
        print(f"[hello] booted: {sim.robot.num_dof} DOFs, end-effector={sim.end_effector}")

        # Move the LEFT arm to a Cartesian pose via IK (top-down), close the hand,
        # tilt the head down, raise the body lift.
        ok = sim.set_arm_pose("left", [0.45, -0.21, 0.55], sim.top_down_quat)
        sim.set_gripper("left", 1.0)          # 0 = open, 1 = closed
        sim.set_head(pan=0.0, tilt=-0.3)
        sim.set_lift(0.6)
        sim.step(n=60)

        pos, quat = sim.get_ee_pose("left")
        print(f"[hello] IK ok={ok}  left EE pos={pos.round(3)}")
        print(f"[hello] lift={sim.get_lift():.3f}  wrench_left={sim.get_wrench('left').round(2)}")
        print(f"[hello] joints tracked: {len(sim.get_joint_positions())}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
