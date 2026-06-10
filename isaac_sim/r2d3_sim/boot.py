"""Boot the Isaac Sim app for the R2D3 SDK.

The SimulationApp launch + ROS2-bridge extension enable, factored out of
bring_up.py so every entry point (the R2D3 facade, examples) boots identically.
SimulationApp must be created exactly once per process and BEFORE importing any
omni/isaacsim modules — that ordering is the whole point of this module.
"""
from __future__ import annotations

import os

_APP = None

_ROS_EXTENSIONS = ("isaacsim.ros2.core", "isaacsim.ros2.nodes", "isaacsim.ros2.bridge")


def launch(headless: bool = True, *, enable_ros: bool = False):
    """Create the SimulationApp (idempotent) and return the handle.

    Sets the EULA / privacy-consent env. If ``enable_ros`` is set, also enables
    the bundled ROS2 bridge extensions (needed before sensors.build_action_graph
    and before importing the bundled rclpy).
    """
    global _APP
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    os.environ.setdefault("PRIVACY_CONSENT", "Y")
    if _APP is None:
        from isaacsim import SimulationApp
        _APP = SimulationApp({"headless": bool(headless)})
    if enable_ros:
        enable_ros_bridge()
    return _APP


def enable_ros_bridge() -> None:
    """Enable the isaacsim ROS2 bridge extensions. Safe to call repeatedly."""
    import omni.kit.app
    em = omni.kit.app.get_app().get_extension_manager()
    for ext in _ROS_EXTENSIONS:
        if not em.is_extension_enabled(ext):
            em.set_extension_enabled_immediate(ext, True)


def app():
    """The live SimulationApp handle (or None if not launched)."""
    return _APP


def close() -> None:
    global _APP
    if _APP is not None:
        _APP.close()
        _APP = None
