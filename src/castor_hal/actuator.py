"""TransportActuator — the bridge that makes ANY Transport a gateway actuator.

This is the payoff of the HAL: the robot-md-gateway already does the hard,
transport-independent work (verify ROBOT.md, tier/tool/confidence/HiTL
default-deny gating, sign the outcome, ship RCAN to Atlas). It delegates the
final actuation step to an object implementing its ``Actuator`` Protocol
(``execute(*, envelope, manifest_path, tier, config) -> ActuatorOutcome``).

``TransportActuator`` implements that Protocol ONCE, for every transport: it
parses the envelope into a normalized ``Goal`` (``parse_goal``), routes
``read_state``/``estop`` to the transport's own methods, calls ``set_goal`` for
everything else, and turns transport success/``TransportError``/unexpected
exceptions into a structured ``ActuatorOutcome``. Writing a new device driver is
now: implement ``Transport`` (5 methods) — the gateway plumbing is free.

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
from castor_hal.goal import parse_goal
from castor_hal.state import serialize_state
from castor_hal.transport import Transport

# Tool names that are transport METHODS, not goals — dispatched before parse_goal.
_READ_TOOLS = frozenset({"read_state", "get_state", "read"})
_ESTOP_TOOLS = frozenset({"estop", "stop", "emergency_stop", "halt"})


class TransportActuator:
    """Adapt a ``Transport`` to the gateway's ``Actuator`` Protocol.

    Args:
        transport: a ready ``Transport`` instance (tests / direct use), OR
        factory: ``Callable[[dict], Transport]`` built lazily from the gateway
            ``config`` on first ``execute`` (the entry-point path). Exactly one
            of ``transport``/``factory`` must be given.
        name/description: actuator identity surfaced to the gateway.
        config_schema: JSON-schema-ish dict the gateway may surface.
    """

    def __init__(
        self,
        transport: Transport | None = None,
        *,
        factory: Callable[[dict], Transport] | None = None,
        name: str | None = None,
        description: str | None = None,
        config_schema: dict | None = None,
    ) -> None:
        if (transport is None) == (factory is None):
            raise ValueError("provide exactly one of transport= or factory=")
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
