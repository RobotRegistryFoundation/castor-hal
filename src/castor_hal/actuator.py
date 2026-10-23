"""TransportActuator — the bridge that makes ANY Transport a gateway actuator.

This is the payoff of the HAL: the robot-md-gateway already does the hard,
transport-independent work (verify ROBOT.md, tier/tool/confidence/HiTL
default-deny gating, sign the outcome, ship RCAN to Atlas). It delegates the
final actuation step to an object implementing its ``Actuator`` Protocol
(``execute(*, envelope, manifest_path, tier, config) -> ActuatorOutcome``).

``TransportActuator`` implements that Protocol ONCE, for every transport: it
parses the envelope into a normalized ``Goal`` (``parse_goal``), routes
``read_state``/``estop`` to the transport's own methods, calls ``set_goal`` for
the goal tools it declares (``motion_capabilities``) and refuses any other, and
turns transport success/``TransportError``/unexpected exceptions into a
structured ``ActuatorOutcome``. Writing a new device driver is now: implement
``Transport`` (5 methods) — the gateway plumbing is free.

The gateway discovers actuator CLASSES via entry-points and instantiates them
with NO args, then passes its config to ``execute``. So a transport that needs
runtime config (a serial port, a CAN channel) is built LAZILY from a
``factory(config) -> Transport`` on first ``execute``. Tests pass a ready
transport instance directly.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

from robot_md_gateway.actuator import ActuatorOutcome

from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import GoalKind, goal_tool_names, parse_goal
from castor_hal.state import serialize_state
from castor_hal.transport import Transport

# Tool names that are transport METHODS, not goals — dispatched before parse_goal.
_READ_TOOLS = frozenset({"read_state", "get_state", "read"})
_ESTOP_TOOLS = frozenset({"estop", "stop", "emergency_stop", "halt"})

#: Every tool name ``parse_goal`` maps to a goal, whatever the transport: what a
#: transport whose goal kinds are not known yet is declared to accept.
_GOAL_TOOLS = goal_tool_names(GoalKind)


class TransportActuator:
    """Adapt a ``Transport`` to the gateway's ``Actuator`` Protocol.

    Two declarations tell robot-md-gateway which of its tools stop the machine
    and which move it. The gateway reads them off the instance and matches tool
    names EXACTLY, so they list exact spellings:

    ``stop_capabilities``: ``estop``, ``stop``, ``emergency_stop`` and
        ``halt``, the tools routed to ``Transport.estop``, when the transport
        can stop (``Transport.can_estop``); none when it cannot (a read-only
        tap, whose ``estop`` refuses). From robot-md-gateway#35 on, a tripped
        gateway still lets a declared stop through, and sends this actuator
        the first one, sorted (``emergency_stop``), when it trips. The stop is
        routed whatever its case (``ESTOP`` stops too), but only these
        spellings are declared.

    ``motion_capabilities``: every tool name that can reach ``set_goal``: the
        names ``parse_goal`` maps to the goal kinds the transport accepts
        (``GOAL_TOOL_NAMES``). Every goal actuates the machine, so the gateway
        treats all of them as motion: an actuation scope, the motion HiTL and
        confidence gates, and a stop on the allowlist. ``execute`` refuses a
        goal tool that is not in this set, spelled exactly as it is here, so
        the set is never smaller than what can move the machine.

    The two never overlap. A transport built lazily (``factory=``) is not
    known until the first ``execute``, long after the gateway has read these.
    Pass ``transport_type=`` to declare from its class; without it the
    declarations fail safe: every goal tool is declared as motion, and the
    stop tools are declared.

    Args:
        transport: a ready ``Transport`` instance (tests / direct use), OR
        factory: ``Callable[[dict], Transport]`` built lazily from the gateway
            ``config`` on first ``execute`` (the entry-point path). Exactly one
            of ``transport``/``factory`` must be given.
        transport_type: with ``factory`` only: the ``Transport`` class it
            builds, read for ``capabilities`` and ``can_estop`` (class
            attributes) to make the declarations above.
        name/description: actuator identity surfaced to the gateway.
        config_schema: JSON-schema-ish dict the gateway may surface.
    """

    #: The fail-safe declarations, for a lazily built transport of unknown
    #: type. ``__init__`` replaces them whenever the transport's class is known.
    stop_capabilities: frozenset[str] = _ESTOP_TOOLS
    motion_capabilities: frozenset[str] = _GOAL_TOOLS

    def __init__(
        self,
        transport: Transport | None = None,
        *,
        factory: Callable[[dict], Transport] | None = None,
        transport_type: type[Transport] | None = None,
        name: str | None = None,
        description: str | None = None,
        config_schema: dict | None = None,
    ) -> None:
        if (transport is None) == (factory is None):
            raise ValueError("provide exactly one of transport= or factory=")
        if transport_type is not None and factory is None:
            raise ValueError("transport_type= describes what factory= builds; "
                             "with transport=, the instance is read instead")
        described = transport if transport is not None else transport_type
        if described is not None:
            self.motion_capabilities = goal_tool_names(described.capabilities)
            self.stop_capabilities = (
                _ESTOP_TOOLS if getattr(described, "can_estop", True) else frozenset())
        self._transport = transport
        self._factory = factory
        self._name = name or (transport.name if transport is not None else "castor-transport")
        self._description = description or (
            transport.description if transport is not None else "castor-hal transport actuator"
        )
        self._config_schema = config_schema or {}

    @property
    def name(self) -> str:
        return self._name

    @property
    def description(self) -> str:
        return self._description

    @property
    def config_schema(self) -> dict:
        return self._config_schema

    def _ensure_transport(self, config: dict) -> Transport:
        if self._transport is None:
            assert self._factory is not None  # guaranteed by __init__
            self._transport = self._factory(config or {})
        self._transport.open()
        return self._transport

    def execute(
        self,
        *,
        envelope: dict,
        manifest_path: Path,
        tier: str,
        config: dict,
    ) -> ActuatorOutcome:
        tool_name = (envelope or {}).get("tool_name")
        tool_args = (envelope or {}).get("tool_args", {}) or {}
        name = (tool_name or "").strip().lower()
        try:
            transport = self._ensure_transport(config or {})

            if name in _READ_TOOLS:
                state = transport.read_state()
                return ActuatorOutcome(
                    success=True, outcome_kind="executed", telemetry=serialize_state(state)
                )

            if name in _ESTOP_TOOLS:
                transport.estop()
                return ActuatorOutcome(
                    success=True, outcome_kind="executed", telemetry={"estopped": True}
                )

            # Only a goal tool declared in motion_capabilities, spelled exactly
            # as declared, goes on to the transport. parse_goal ignores case and
            # surrounding space, and the gateway does not: "MOVE" used to move
            # the machine while the gateway's motion gates (actuation scope,
            # HiTL, confidence) did not count it as motion.
            if tool_name not in self.motion_capabilities:
                hint = f"; send it as {name!r}" if name in self.motion_capabilities else ""
                raise TransportError(
                    TransportErrorCode.UNSUPPORTED_GOAL,
                    f"{self.name!r} declares no goal tool {tool_name!r}{hint}",
                    detail={"tool": tool_name,
                            "motion_capabilities": sorted(self.motion_capabilities)},
                )

            # parse_goal's contract is "a Goal or a TransportError, never raw".
            # Belt-and-suspenders so a future parser slip surfaces as INVALID_GOAL
            # (operator-actionable) rather than INTERNAL (review finding #2).
            try:
                goal = parse_goal(tool_name, tool_args)
            except TransportError:
                raise
            except Exception as exc:  # noqa: BLE001
                raise TransportError(
                    TransportErrorCode.INVALID_GOAL,
                    f"could not parse goal for {tool_name!r}: {exc}",
                    detail={"tool": tool_name},
                ) from exc
            # Checked here as well as by the adapter, which is asked to (see
            # Transport.ensure_supported): motion_capabilities is derived from
            # the transport's capabilities, and this keeps that declaration
            # true for an adapter whose set_goal forgets to check.
            transport.ensure_supported(goal)
            result = transport.set_goal(goal)
            telemetry: dict = {"reached": bool(result.reached), "goal_kind": goal.kind.value}
            if result.detail:
                telemetry.update(result.detail)
            if result.state is not None:
                telemetry["state"] = serialize_state(result.state)
            # `success` reflects whether the goal was actually achieved; the
            # action still EXECUTED (outcome_kind) even if it fell short — the
            # gateway records both, honestly.
            return ActuatorOutcome(
                success=bool(result.reached), outcome_kind="executed", telemetry=telemetry
            )

        except TransportError as exc:
            return ActuatorOutcome(
                success=False,
                outcome_kind="error",
                error_message=str(exc),
                telemetry=exc.as_dict(),
            )
        except Exception as exc:  # noqa: BLE001 — adapters are operator code; never crash the gateway
            return ActuatorOutcome(
                success=False,
                outcome_kind="error",
                error_message=f"{type(exc).__name__}: {exc}",
                telemetry={"error_code": TransportErrorCode.INTERNAL.value},
            )
