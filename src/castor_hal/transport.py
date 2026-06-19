"""The Transport base class — the one seam every adapter implements.

A ``Transport`` is the thin, device-specific layer that turns a normalized
``Goal`` into wire traffic and reads device ``TransportState`` back. It is the
ONLY thing that differs between an SO-ARM101 (serial bus servos), a tractor
(CAN/J1939, ISOBUS), a mobile base (ROS 2 ``/cmd_vel``), or a drone (MAVLink).
Everything above it — gateway policy/tier gating, signing, RCAN, Atlas — is
transport-independent and already exists.

Five methods, by design: ``open / close / set_goal / read_state / estop``.
A transport also declares ``capabilities`` (the ``GoalKind``s it accepts) so the
bridge can refuse an unsupported goal as a structured error instead of crashing.

SAFETY (binding): ``estop()`` is a BEST-EFFORT software stop — it commands the
device to halt/hold as fast as the link allows. It is NOT a safety-rated
emergency stop and is NOT a substitute for a hardware e-stop wired to a
safety-rated controller (ISO 13850). The transport can refuse or command a stop;
it cannot guarantee the machine physically stopped, and the network/bus latency
means it is never in the real-time safety loop. Adapters must document their
estop's real behavior and limits.

PURE of the gateway: this module imports only goal/state/errors. The gateway
bridge lives in ``castor_hal.actuator``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import GoalKind
from castor_hal.state import TransportState


@dataclass
class GoalResult:
    """The outcome of a single ``set_goal``. ``reached`` is the adapter's
    honest read on whether the goal was achieved (within tolerance/ack);
    ``state`` is an optional post-goal snapshot; ``detail`` is structured
    extras (elapsed, final positions, …)."""

    reached: bool
    state: TransportState | None = None
    detail: dict = field(default_factory=dict)


class Transport(ABC):
    """Abstract device transport. Subclass per hardware family.

    A subclass MUST set ``capabilities`` to the frozenset of ``GoalKind``s it
    accepts and implement ``open/close/set_goal/read_state/estop``. The default
    ``set_goal`` precheck (call ``ensure_supported(goal)`` first) gives a uniform
    ``UNSUPPORTED_GOAL`` error across every adapter.
    """

    #: GoalKinds this transport accepts. Subclasses override.
    capabilities: frozenset[GoalKind] = frozenset()

    #: Human-facing identity (override or set in __init__).
    name: str = "transport"
    description: str = "castor-hal transport"

    # -- lifecycle ---------------------------------------------------------
    @abstractmethod
    def open(self) -> None:
        """Open the link (idempotent — a second call on an open link is a no-op)."""

    @abstractmethod
    def close(self) -> None:
        """Close the link (idempotent). Safe to call on an unopened transport."""

    # -- actuation / sensing ----------------------------------------------
    @abstractmethod
    def set_goal(self, goal) -> GoalResult:
        """Drive the device toward ``goal``. Raise ``TransportError`` on failure."""

    @abstractmethod
    def read_state(self) -> TransportState:
        """Read a normalized state snapshot. Raise ``TransportError`` on failure."""

    @abstractmethod
    def estop(self) -> None:
        """Best-effort immediate stop/hold. MUST NOT raise for a routine 'already
        stopped' condition; MAY raise ``TransportError(IO_ERROR)`` only if it
        cannot even command the stop. See the module SAFETY note: this is not a
        hardware e-stop."""

    # -- helpers (concrete) -----------------------------------------------
    def supports(self, goal) -> bool:
        """True iff this transport accepts ``goal``'s kind."""
        return getattr(goal, "kind", None) in self.capabilities

    def ensure_supported(self, goal) -> None:
        """Raise a structured ``UNSUPPORTED_GOAL`` if the goal isn't accepted.
        Adapters should call this at the top of ``set_goal``."""
        if not self.supports(goal):
            kind = getattr(goal, "kind", None)
            raise TransportError(
                TransportErrorCode.UNSUPPORTED_GOAL,
                f"{self.name!r} does not accept goal kind "
                f"{getattr(kind, 'value', kind)!r}",
                detail={
                    "transport": self.name,
                    "goal_kind": getattr(kind, "value", str(kind)),
                    "capabilities": sorted(k.value for k in self.capabilities),
                },
            )

    # -- context manager ---------------------------------------------------
    def __enter__(self) -> "Transport":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()
