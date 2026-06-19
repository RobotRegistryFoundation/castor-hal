"""castor-hal — the transport hardware-abstraction layer for robot-md-gateway.

One actuator interface fronts every physical link (serial bus servos, CAN/J1939,
ISOBUS, ROS 2, MAVLink). Implement ``Transport`` (open/close/set_goal/read_state/
estop), declare its ``capabilities``, and wrap it in ``TransportActuator`` — the
gateway's trust pipeline (ROBOT.md verify, tier/confidence/HiTL default-deny,
signing, RCAN→Atlas) is then free and transport-independent.

  from castor_hal import Transport, TransportActuator, GoalKind, JointPositions
"""

from __future__ import annotations

from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import (
    GOAL_TYPES,
    BodyTwist,
    FlightSetpoint,
    GoalKind,
    Gripper,
    Home,
    JointPositions,
    JointTrajectory,
    JointWaypoint,
    RateSetpoint,
    SectionState,
    parse_goal,
)
from castor_hal.state import TransportState, serialize_state
from castor_hal.transport import GoalResult, Transport

# The gateway bridge imports robot-md-gateway; keep it lazy so the pure transport
# core (goal/state/errors/transport) is importable without the gateway present.
try:  # pragma: no cover - exercised by environments with/without the gateway
    from castor_hal.actuator import TransportActuator
except ModuleNotFoundError:  # robot-md-gateway not installed
    TransportActuator = None  # type: ignore[assignment]

__version__ = "0.1.0"

__all__ = [
    "Transport",
    "GoalResult",
    "TransportActuator",
    "TransportState",
    "serialize_state",
    "TransportError",
    "TransportErrorCode",
    "GoalKind",
    "parse_goal",
    "GOAL_TYPES",
    "JointPositions",
    "JointTrajectory",
    "JointWaypoint",
    "BodyTwist",
    "FlightSetpoint",
    "Gripper",
    "RateSetpoint",
    "SectionState",
    "Home",
    "__version__",
]
