"""Normalized transport STATE + a stable telemetry serializer.

Every transport reports state in the same shape, so Atlas (and any operator
console) reads "what the machine is doing" identically whether the source is a
servo bus, a J1939 ECU, or an ISOBUS task controller. ``serialize_state`` emits
the wire dict that rides an ``ActuatorOutcome.telemetry`` payload — absent
fields are dropped (honest-empty, never ``null`` noise).

PURE module: no I/O, no gateway import.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class TransportState:
    """A normalized snapshot of a device's state. All fields optional — a
    telemetry-only ISOBUS tap fills ``measurements``; an arm fills
    ``positions``/``temperatures_c``. ``timestamp_s`` is a monotonic-ish
    reading stamp supplied by the adapter."""

    connected: bool = False
    estopped: bool = False
    positions: dict[str, float] | None = None         # joint → rad (or device unit)
    velocities: dict[str, float] | None = None        # joint/axis → unit/s
    temperatures_c: dict[str, float] | None = None     # motor/component → °C
    measurements: dict[str, float] | None = None       # named scalar telemetry (speed, rpm, rate…)
    unit: str = "rad"
    timestamp_s: float | None = None
    extra: dict = field(default_factory=dict)           # adapter-specific extras


def serialize_state(state: TransportState) -> dict:
    """Stable wire dict for telemetry. Drops empty/None fields so the payload
    reflects only what the transport actually observed."""
    out: dict = {"connected": bool(state.connected), "estopped": bool(state.estopped)}
    if state.positions:
        out["positions"] = dict(state.positions)
        out["unit"] = state.unit
    if state.velocities:
        out["velocities"] = dict(state.velocities)
    if state.temperatures_c:
        out["temperatures_c"] = dict(state.temperatures_c)
    if state.measurements:
        out["measurements"] = dict(state.measurements)
    if state.timestamp_s is not None:
        out["timestamp_s"] = state.timestamp_s
    if state.extra:
        out["extra"] = dict(state.extra)
    return out
