"""r2d3_sim — Python platform SDK for the R2D3 Isaac Sim robot.

Run inside the Isaac launcher (``scripts/isaacsim_ros2.sh``): omni.* / isaacsim.*
/ rclpy only exist after SimulationApp boots. The pure-Python parts of this
package (``sim_topics``, the dataclasses, helper math) import fine anywhere; the
Isaac-touching code is deferred until ``R2D3(...)`` / function calls.

High-level SDK (start here):
    R2D3        One handle: boot + control + sensing, in-process (no ROS needed).
                Build RL / VLM / teleop on top of it.  ``from isaac_sim.r2d3_sim import R2D3``

SDK modules:
    r2d3        The R2D3 facade + Observation / JointState dataclasses
    boot        SimulationApp launch + ROS-bridge enable
    helpers     Quaternions, prim lookups, world poses, RGBA, lighting, GIF
    cameras     CameraRig — in-process RGB/depth as numpy
    ik          ArmIK — Lula IK/FK for the left arm
    scenes      Training environments (warehouse/kitchen/living_room) + manipulable objects
    envs/       RL env, VLM loop, teleop built on R2D3

Building blocks (used by the facade; also usable directly):
    sim_topics  Topic-name + joint-name contract (no Isaac imports)
    scene       Ground plane + USD loading (per-end-effector)
    robot       Articulation wrapper (joint/EE control, FT, FK)
    sensors     OmniGraph wiring (clock, TF, D435 cameras) + Camera prims
    sim_adapter rclpy node, std-msg state/cmd surface
    bring_up    ROS entry point (the eval/bridge path)
"""
__version__ = "0.1.0"

# R2D3 is safe to import here: r2d3.py's module level is pure-Python (numpy +
# dataclasses); the omni/isaacsim imports are deferred into R2D3.__init__.
from .r2d3 import R2D3, Observation, JointState  # noqa: E402,F401

__all__ = ["R2D3", "Observation", "JointState"]
