"""Teleoperation demo — stream target poses into the sim with TeleopServer.

    scripts/isaacsim_ros2.sh isaac_sim/examples/03_teleop.py

Replace the scripted submit() calls with your input source (keyboard, gamepad,
VR controller, leader arm, network socket). Use use_ros=True to drive it from
the /r2d3/sim/cmd/* ROS topics instead.
"""
from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim.envs.teleop import TeleopServer


def main() -> int:
    with R2D3(end_effector="dexterous", headless=True) as sim:
        sim.reset()
        tele = TeleopServer(sim, use_ros=False)

        # reach to a pose (Cartesian, solved by IK each tick), hand open
        tele.submit(left_ee=([0.45, -0.21, 0.58], sim.top_down_quat), gripper_left=0.0)
        tele.spin(40)
        print(f"[teleop] reached: {sim.get_ee_pose('left')[0].round(3)}")

        # descend + close the hand
        tele.submit(left_ee=([0.45, -0.21, 0.50], sim.top_down_quat))
        tele.spin(30)
        tele.submit(gripper_left=1.0)
        tele.spin(30)
        print("[teleop] closed gripper, done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
