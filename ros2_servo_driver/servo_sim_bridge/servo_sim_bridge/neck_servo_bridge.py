"""Sim-side stand-in for the real ``servo_driver`` neck node.

Speaks the exact same servo contract as the real robot so identical application
code drives the neck in MuJoCo, Gazebo, and on hardware:

* Subscribes ``/servo_control/move`` (``servo_interfaces/msg/ServoMove``). On each
  message it dispatches on ``servo_id``, clamps the units to the servo's usable
  band, converts to radians via :mod:`servo_sim_bridge.calibration`, and publishes
  the full ``[head_joint1, head_joint2]`` setpoint on ``/neck_controller/commands``
  (``std_msgs/msg/Float64MultiArray``). Unknown ``servo_id`` values are ignored.
* Publishes read-back at a fixed rate on ``/servo_both_angles``
  (``Float64MultiArray``), computed from the latest ``/joint_states`` by inverting
  the calibration, ordered ``[id5/tilt units, id2/pan units]`` to match the real
  ``get_states_publishers`` order.

The low-level ``get_angle`` / ``/servo_angle`` request-response polling loop is
deliberately not reproduced (there is no serial device to poll). A slew-rate
limit is stubbed behind a single parameter, disabled by default.

All unit<->radian maths live in the pure, unit-tested
:class:`servo_sim_bridge.calibration.NeckCalibration`; this module is a thin ROS
shell around it.
"""

import os

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from servo_interfaces.msg import ServoMove
from std_msgs.msg import Float64MultiArray

from servo_sim_bridge.calibration import DEFAULT_CONFIG, NeckCalibration

DEFAULT_CONFIG_RELPATH = os.path.join("config", "neck_servo_bridge.yaml")


def load_config(config_file: str, logger=None) -> dict:
    """Load the calibration config from an explicit path, else the installed
    default, else the seeded :data:`DEFAULT_CONFIG` (keeps the node usable when
    run uninstalled, e.g. in tests)."""
    path = config_file
    if not path:
        try:
            from ament_index_python.packages import get_package_share_directory

            path = os.path.join(
                get_package_share_directory("servo_sim_bridge"), DEFAULT_CONFIG_RELPATH
            )
        except Exception:  # pragma: no cover - only when share dir is unavailable
            path = ""
    if path and os.path.isfile(path):
        with open(path, "r") as handle:
            return yaml.safe_load(handle)
    if logger is not None:
        logger.warning(
            f"config file '{path or config_file}' not found; using built-in defaults"
        )
    return DEFAULT_CONFIG


class NeckServoBridge(Node):
    def __init__(self):
        super().__init__("neck_servo_bridge")

        self.declare_parameter("config_file", "")
        self.declare_parameter("servo_move_topic", "/servo_control/move")
        self.declare_parameter("command_topic", "/neck_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("readback_topic", "/servo_both_angles")
        self.declare_parameter("readback_rate_hz", 4.0)
        # Slew-rate emulation is stubbed for later enablement (see _on_servo_move):
        # 0 disables it and commands pass straight through to the physics actuators.
        # Declared but not yet read, so the hook exists without dead runtime state.
        self.declare_parameter("slew_rate_units_per_s", 0.0)

        config_file = self.get_parameter("config_file").value
        self.calibration = NeckCalibration.from_config(
            load_config(config_file, self.get_logger())
        )

        # Seed the setpoint at each servo's centre so the first published command
        # is a complete, well-defined pose even before any ServoMove arrives.
        self._joint_targets = [0.0] * self.calibration.num_joints
        for servo_id in self.calibration.readback_order:
            idx, rad = self.calibration.command_for(
                servo_id, self.calibration.center_units(servo_id)
            )
            self._joint_targets[idx] = rad

        self._latest_positions = {}

        command_topic = self.get_parameter("command_topic").value
        readback_topic = self.get_parameter("readback_topic").value
        self._command_pub = self.create_publisher(Float64MultiArray, command_topic, 10)
        self._readback_pub = self.create_publisher(Float64MultiArray, readback_topic, 10)

        self.create_subscription(
            ServoMove,
            self.get_parameter("servo_move_topic").value,
            self._on_servo_move,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self._on_joint_states,
            10,
        )

        rate = float(self.get_parameter("readback_rate_hz").value)
        self.create_timer(1.0 / rate, self._publish_readback)

        self.get_logger().info(
            f"neck_servo_bridge up: commands -> {command_topic}, "
            f"read-back -> {readback_topic} at {rate} Hz"
        )

    def _on_servo_move(self, msg: ServoMove):
        command = self.calibration.command_for(msg.servo_id, msg.angle)
        if command is None:
            self.get_logger().debug(f"ignoring unknown servo_id {msg.servo_id}")
            return
        idx, rad = command
        # Slew hook: to emulate the real servo's finite slew rate, interpolate
        # toward `rad` here at slew_rate_units_per_s instead of jumping to it.
        self._joint_targets[idx] = rad
        self._command_pub.publish(Float64MultiArray(data=list(self._joint_targets)))

    def _on_joint_states(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            self._latest_positions[name] = position

    def _publish_readback(self):
        units = self.calibration.readback_units(self._latest_positions)
        self._readback_pub.publish(Float64MultiArray(data=[float(u) for u in units]))


def main(args=None):
    rclpy.init(args=args)
    node = NeckServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
