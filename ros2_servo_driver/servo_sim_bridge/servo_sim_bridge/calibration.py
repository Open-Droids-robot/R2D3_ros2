"""Pure servo-unit <-> joint-radian calibration for the simulated R2D3 neck.

This module has no ROS dependency on purpose: it is the single tested seam behind
``neck_servo_bridge``. The node is a thin ROS shell that owns subscriptions,
publishers and timers; all unit<->radian maths live here so they can be unit
tested without a simulator or ``rclpy``.

Calibration is seeded from the real robot's ``get_states_publishers.py`` and is
fully overridable via config (see ``config/neck_servo_bridge.yaml``). Each servo
maps linearly, in degrees, then to radians::

    deg   = (units - center_units) * deg_per_unit + offset_deg
    rad   = radians(deg)
    units = center_units + (degrees(rad) - offset_deg) / deg_per_unit   # inverse

Servo units are clamped to each servo's usable band before conversion so a
command can never drive the joint past its mechanical limits.
"""

import math
from typing import Dict, List, Optional, Tuple

# Seeded defaults (issue #5). ``deg_per_unit`` is expressed as the real
# calibration's span ratio so the constants stay legible next to the source:
#   pan  id 2 -> head_joint1: 60 deg   over 300 units, no offset  -> ~ +/-60 deg
#   tilt id 5 -> head_joint2: 70.1 deg over 300 units, -7.766 deg offset
DEFAULT_CONFIG = {
    "servos": {
        2: {
            "joint_name": "head_joint1",
            "joint_index": 0,
            "center_units": 500,
            "deg_per_unit": 60.0 / 300.0,
            "offset_deg": 0.0,
            "min_units": 200,
            "max_units": 800,
        },
        5: {
            "joint_name": "head_joint2",
            "joint_index": 1,
            "center_units": 500,
            "deg_per_unit": 70.1 / 300.0,
            "offset_deg": -7.766,
            "min_units": 200,
            "max_units": 800,
        },
    },
    # Read-back array order is load-bearing: index 0 = tilt servo (id 5) units,
    # index 1 = pan servo (id 2) units, matching the real get_states_publishers.
    "readback_order": [5, 2],
    "num_joints": 2,
}


class ServoCalibration:
    """Linear unit<->radian mapping for a single servo, with band clamping."""

    def __init__(
        self,
        servo_id: int,
        joint_name: str,
        joint_index: int,
        center_units: float,
        deg_per_unit: float,
        offset_deg: float,
        min_units: float,
        max_units: float,
    ):
        if deg_per_unit == 0:
            raise ValueError(f"servo {servo_id}: deg_per_unit must be non-zero")
        if min_units > max_units:
            raise ValueError(f"servo {servo_id}: min_units must be <= max_units")
        self.servo_id = servo_id
        self.joint_name = joint_name
        self.joint_index = joint_index
        self.center_units = center_units
        self.deg_per_unit = deg_per_unit
        self.offset_deg = offset_deg
        self.min_units = min_units
        self.max_units = max_units

    def clamp_units(self, units: float) -> float:
        return max(self.min_units, min(self.max_units, units))

    def units_to_rad(self, units: float) -> float:
        clamped = self.clamp_units(units)
        deg = (clamped - self.center_units) * self.deg_per_unit + self.offset_deg
        return math.radians(deg)

    def rad_to_units(self, rad: float) -> float:
        deg = math.degrees(rad)
        units = self.center_units + (deg - self.offset_deg) / self.deg_per_unit
        return self.clamp_units(units)


class NeckCalibration:
    """Collection of per-servo calibrations plus the neck's joint/read-back layout."""

    def __init__(
        self,
        servos: Dict[int, ServoCalibration],
        readback_order: List[int],
        num_joints: int,
    ):
        self._servos = servos
        self.readback_order = list(readback_order)
        self.num_joints = num_joints

    @classmethod
    def from_config(cls, config: dict) -> "NeckCalibration":
        servos: Dict[int, ServoCalibration] = {}
        for raw_id, params in config["servos"].items():
            servo_id = int(raw_id)  # YAML may key servo ids as strings
            servos[servo_id] = ServoCalibration(
                servo_id=servo_id,
                joint_name=params["joint_name"],
                joint_index=int(params["joint_index"]),
                center_units=float(params["center_units"]),
                deg_per_unit=float(params["deg_per_unit"]),
                offset_deg=float(params["offset_deg"]),
                min_units=float(params["min_units"]),
                max_units=float(params["max_units"]),
            )
        readback_order = [int(x) for x in config.get("readback_order", [])]
        num_joints = int(config.get("num_joints", len(servos)))
        return cls(servos, readback_order, num_joints)

    def has_servo(self, servo_id: int) -> bool:
        return servo_id in self._servos

    def center_units(self, servo_id: int) -> float:
        return self._servos[servo_id].center_units

    def units_to_rad(self, servo_id: int, units: float) -> float:
        return self._servos[servo_id].units_to_rad(units)

    def rad_to_units(self, servo_id: int, rad: float) -> float:
        return self._servos[servo_id].rad_to_units(rad)

    def command_for(self, servo_id: int, units: float) -> Optional[Tuple[int, float]]:
        """Return ``(joint_index, radians)`` for a servo command, or ``None`` for an
        unknown ``servo_id`` (unrelated servo traffic must not destabilise the sim)."""
        servo = self._servos.get(servo_id)
        if servo is None:
            return None
        return servo.joint_index, servo.units_to_rad(units)

    def readback_units(self, positions_by_joint: Dict[str, float]) -> List[float]:
        """Map current joint radians back to servo units, ordered per ``readback_order``.

        A joint missing from ``positions_by_joint`` (e.g. no ``/joint_states`` yet)
        falls back to the servo's centre unit so the array stays well-formed.
        """
        out: List[float] = []
        for servo_id in self.readback_order:
            servo = self._servos[servo_id]
            rad = positions_by_joint.get(servo.joint_name)
            if rad is None:
                out.append(servo.center_units)
            else:
                out.append(servo.rad_to_units(rad))
        return out
