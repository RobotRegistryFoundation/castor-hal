"""The normalized GOAL model — the transport-independent vocabulary of intent.

Every actuation the gateway might hand a transport is expressed as one of a
small, closed set of typed goals. A 6-DOF arm, a mobile base, a drone, and an
ISOBUS rate controller speak utterly different wire protocols — but the *intent*
("hold these joint angles", "drive at this twist", "apply at this rate") is a
finite set. Normalizing it here means:

  * every adapter parses the gateway envelope the SAME way (``parse_goal``), and
  * the gateway's tier/confidence/allowlist gates run against a stable shape.

A transport declares which ``GoalKind``s it accepts (``Transport.capabilities``);
asking it for one it doesn't support is a structured ``UNSUPPORTED_GOAL`` error,
never a crash.

PURE module: no I/O, no gateway import — safe to import anywhere.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from castor_hal.errors import TransportError, TransportErrorCode


class GoalKind(str, Enum):
    """The closed set of normalized intents a transport may accept."""

    JOINT_POSITIONS = "joint_positions"      # hold a set of joint angles (arms)
    JOINT_TRAJECTORY = "joint_trajectory"    # follow a timed sequence of joint poses
    BODY_TWIST = "body_twist"                # linear+angular velocity (mobile bases / cmd_vel)
    FLIGHT_SETPOINT = "flight_setpoint"      # NED position + yaw (drones / MAVLink offboard)
    GRIPPER = "gripper"                      # open/close an end-effector
    RATE_SETPOINT = "rate_setpoint"          # application rate (ISOBUS: seed/spray/spread)
    SECTION_STATE = "section_state"          # implement section on/off (ISOBUS section control)
    HOME = "home"                            # go to the device's configured home/safe pose


@dataclass(frozen=True)
class JointPositions:
    """Hold each named joint at a target value. ``unit`` defaults to radians.
    A 'gripper' joint, if the device has one, is just another key here."""

    positions: dict[str, float]
    unit: str = "rad"
    kind: GoalKind = field(default=GoalKind.JOINT_POSITIONS, init=False)


@dataclass(frozen=True)
class JointWaypoint:
    positions: dict[str, float]
    time_from_start_s: float


@dataclass(frozen=True)
class JointTrajectory:
    """A timed sequence of joint poses."""

    points: tuple[JointWaypoint, ...]
    unit: str = "rad"
    kind: GoalKind = field(default=GoalKind.JOINT_TRAJECTORY, init=False)


@dataclass(frozen=True)
class BodyTwist:
    """Linear + angular velocity in the robot body frame (ROS cmd_vel shape).
    Units: m/s and rad/s."""

    linear: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular: tuple[float, float, float] = (0.0, 0.0, 0.0)
    kind: GoalKind = field(default=GoalKind.BODY_TWIST, init=False)


@dataclass(frozen=True)
class FlightSetpoint:
    """A drone offboard setpoint: North/East/Down metres + yaw radians."""

    north_m: float
    east_m: float
    down_m: float
    yaw_rad: float = 0.0
    kind: GoalKind = field(default=GoalKind.FLIGHT_SETPOINT, init=False)


@dataclass(frozen=True)
class Gripper:
    """Open/close an end-effector. ``position`` 0..1 (0=closed) unless the
    adapter documents otherwise; ``effort`` optional (N or device units)."""

    position: float
    effort: float | None = None
    kind: GoalKind = field(default=GoalKind.GRIPPER, init=False)


@dataclass(frozen=True)
class RateSetpoint:
    """An application rate setpoint (ISOBUS Task Controller). ``product`` names
    what is metered (e.g. 'seed', 'liquid'); ``unit`` e.g. 'kg/ha', 'l/ha'."""

    rate: float
    unit: str
    product: str | None = None
    kind: GoalKind = field(default=GoalKind.RATE_SETPOINT, init=False)


@dataclass(frozen=True)
class SectionState:
    """Implement section on/off map (ISOBUS section control). Keys are 1-based
    section indices; values are booleans."""

    sections: dict[int, bool]
    kind: GoalKind = field(default=GoalKind.SECTION_STATE, init=False)


@dataclass(frozen=True)
class Home:
    """Go to the device's configured home/safe pose (the adapter knows it)."""

    kind: GoalKind = field(default=GoalKind.HOME, init=False)


# A Goal is any one of the variants above. (Kept as a comment rather than a
# typing.Union alias so the module stays import-cheap and 3.10-clean.)
GOAL_TYPES = (
    JointPositions, JointTrajectory, BodyTwist, FlightSetpoint,
    Gripper, RateSetpoint, SectionState, Home,
)


def _require(args: dict, key: str, tool: str):
    if key not in args:
        raise TransportError(
            TransportErrorCode.INVALID_GOAL,
            f"{tool!r} requires {key!r}",
            detail={"tool": tool, "missing": key},
        )
    return args[key]


def _num(v, field_name: str, tool: str) -> float:
    """Coerce a goal field to a finite float, or raise INVALID_GOAL. Untrusted
    input (None, 'abc', inf, nan) must surface as a *structured* invalid-goal
    error — never a raw TypeError/ValueError that the bridge would mislabel
    INTERNAL (review finding #1)."""
    try:
        out = float(v)
    except (TypeError, ValueError) as e:
        raise TransportError(
            TransportErrorCode.INVALID_GOAL,
            f"{tool!r}: {field_name!r} must be numeric (got {v!r})",
            detail={"tool": tool, "field": field_name},
        ) from e
    if not math.isfinite(out):
        raise TransportError(
            TransportErrorCode.INVALID_GOAL,
            f"{tool!r}: {field_name!r} must be finite (got {out!r})",
            detail={"tool": tool, "field": field_name},
        )
    return out


def parse_goal(tool_name: str, tool_args: dict | None):
    """Map a gateway RCAN-envelope (``tool_name`` + ``tool_args``) to a Goal.

    This is the ONE canonical envelope→goal mapping every adapter shares. It
    handles only *goal-producing* tools; ``read_state`` and ``estop`` are
    transport methods, not goals, and are dispatched by the actuator bridge
    BEFORE this is called.

    Raises:
        TransportError(UNSUPPORTED_GOAL): ``tool_name`` isn't a known goal tool.
        TransportError(INVALID_GOAL): required args missing/malformed.
    """
    args = tool_args or {}
    name = (tool_name or "").strip().lower()

    if name in ("move", "set_joint_positions", "joint_positions"):
        positions = args.get("joint_positions", args.get("positions"))
        if not isinstance(positions, dict) or not positions:
            raise TransportError(
                TransportErrorCode.INVALID_GOAL,
                f"{name!r} requires a non-empty 'joint_positions' object",
                detail={"tool": name},
            )
        return JointPositions(
            positions={str(k): _num(v, str(k), name) for k, v in positions.items()},
            unit=str(args.get("unit", "rad")),
        )

    if name in ("home", "go_home"):
        return Home()

    if name in ("set_velocity", "cmd_vel", "drive", "body_twist"):
        raw_lin = args.get("linear", (0.0, 0.0, 0.0))
        raw_ang = args.get("angular", (0.0, 0.0, 0.0))
        if (not isinstance(raw_lin, (list, tuple)) or len(raw_lin) != 3
                or not isinstance(raw_ang, (list, tuple)) or len(raw_ang) != 3):
            raise TransportError(TransportErrorCode.INVALID_GOAL,
                                 f"{name!r} linear/angular must be 3-vectors", detail={"tool": name})
        lin = tuple(_num(x, "linear", name) for x in raw_lin)
        ang = tuple(_num(x, "angular", name) for x in raw_ang)
        return BodyTwist(linear=lin, angular=ang)  # type: ignore[arg-type]

    if name in ("set_gripper", "gripper"):
        pos = _num(_require(args, "position", name), "position", name)
        effort = args.get("effort")
        return Gripper(position=pos, effort=None if effort is None else _num(effort, "effort", name))

    if name in ("set_rate", "rate_setpoint"):
        return RateSetpoint(
            rate=_num(_require(args, "rate", name), "rate", name),
            unit=str(_require(args, "unit", name)),
            product=args.get("product"),
        )

    if name in ("set_sections", "section_state"):
        raw = _require(args, "sections", name)
        try:
            if isinstance(raw, dict):
                sections = {int(k): bool(v) for k, v in raw.items()}
            elif isinstance(raw, (list, tuple)):
                sections = {i + 1: bool(v) for i, v in enumerate(raw)}
            else:
                raise TransportError(TransportErrorCode.INVALID_GOAL,
                                     f"{name!r} 'sections' must be an object or array", detail={"tool": name})
        except (TypeError, ValueError) as e:
            raise TransportError(TransportErrorCode.INVALID_GOAL,
                                 f"{name!r} 'sections' keys must be integer indices", detail={"tool": name}) from e
        return SectionState(sections=sections)

    if name in ("flight_setpoint", "goto"):
        return FlightSetpoint(
            north_m=_num(_require(args, "north_m", name), "north_m", name),
            east_m=_num(_require(args, "east_m", name), "east_m", name),
            down_m=_num(_require(args, "down_m", name), "down_m", name),
            yaw_rad=_num(args.get("yaw_rad", 0.0), "yaw_rad", name),
        )

    if name in ("set_trajectory", "joint_trajectory"):
        pts_raw = _require(args, "points", name)
        if not isinstance(pts_raw, (list, tuple)) or not pts_raw:
            raise TransportError(TransportErrorCode.INVALID_GOAL,
                                 f"{name!r} 'points' must be a non-empty array", detail={"tool": name})
        points = []
        for i, p in enumerate(pts_raw):
            if not isinstance(p, dict) or not isinstance(p.get("positions"), dict):
                raise TransportError(TransportErrorCode.INVALID_GOAL,
                                     f"{name!r} points[{i}] needs a 'positions' object",
                                     detail={"tool": name, "index": i})
            points.append(JointWaypoint(
                positions={str(k): _num(v, str(k), name) for k, v in p["positions"].items()},
                time_from_start_s=_num(p.get("time_from_start_s", 0.0), "time_from_start_s", name),
            ))
        return JointTrajectory(points=tuple(points), unit=str(args.get("unit", "rad")))

    raise TransportError(
        TransportErrorCode.UNSUPPORTED_GOAL,
        f"no goal mapping for tool {tool_name!r}",
        detail={"tool": tool_name},
    )
