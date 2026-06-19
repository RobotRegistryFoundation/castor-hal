"""Structured error taxonomy for castor-hal transports.

Every transport failure is a single ``TransportError`` carrying a typed
``TransportErrorCode`` (plus an optional ``detail`` dict), so the gateway
bridge can turn it into a *structured* ``ActuatorOutcome`` instead of a bare
string — and so two different transports (a servo bus, a CAN/J1939 link, an
ISOBUS rate controller) report the *same* failure the same way. This is the
"written once" half of the HAL: adapters raise these; nobody re-invents an
ad-hoc error string.

HONESTY/SAFETY: a ``TransportError`` (including ``ESTOPPED``) describes what
the SOFTWARE transport observed or refused. It is never a safety attestation —
a refused or stopped command at this layer does NOT prove the machine
physically stopped; that is the hardware e-stop's job. See ``transport.estop``.
"""

from __future__ import annotations

from enum import Enum


class TransportErrorCode(str, Enum):
    """The closed set of transport failure modes. String-valued so the code
    rides telemetry/JSON verbatim."""

    NOT_CONNECTED = "not_connected"      # open() not called / link down
    UNSUPPORTED_GOAL = "unsupported_goal"  # this transport can't accept this GoalKind
    INVALID_GOAL = "invalid_goal"        # goal malformed (missing/garbage fields)
    UNKNOWN_TARGET = "unknown_target"    # a named joint/section/axis the device doesn't have
    OUT_OF_RANGE = "out_of_range"        # a value outside the device's declared limits
    TIMEOUT = "timeout"                  # the device didn't reach/ack in time
    ESTOPPED = "estopped"                # a command refused because the transport is stopped
    IO_ERROR = "io_error"               # serial/CAN/socket wire failure
    INTERNAL = "internal"                # an unexpected adapter bug


class TransportError(Exception):
    """A typed transport failure.

    Args:
        code: the typed failure mode.
        message: a human-readable description.
        detail: optional structured context (e.g. ``{"joint": "elbow_flex",
            "value": 3.1, "limit": [−1.8, 1.8]}``) that rides into telemetry.
    """

    def __init__(
        self,
        code: TransportErrorCode,
        message: str,
        *,
        detail: dict | None = None,
    ) -> None:
        self.code = code
        self.message = message
        self.detail: dict = detail or {}
        super().__init__(f"{code.value}: {message}")

    def as_dict(self) -> dict:
        """Stable wire form for an ``ActuatorOutcome.telemetry`` payload."""
        out: dict = {"error_code": self.code.value, "error_message": self.message}
        if self.detail:
            out["error_detail"] = self.detail
        return out
