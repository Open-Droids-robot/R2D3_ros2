"""r2d3_sim — Python wrappers for the R2D3 Isaac Sim platform.

Container-only: every submodule imports omni.* / isaacsim.* / rclpy that
only exist after SimulationApp boots inside Isaac Sim 6.0. Do not import
``r2d3_sim`` outside the launcher (``scripts/isaacsim_ros2.sh``).

V1 modules:
    sim_topics  Topic-name + joint-name contract (no Isaac imports)
    scene       Ground plane + USD loading
    robot       Articulation wrapper
    sensors     OmniGraph wiring (clock, TF, D435 camera) — M2
    sim_adapter rclpy node, std-msg state/cmd surface           — M3 / M4
    bring_up    Entry point
"""
__version__ = "0.0.1"
