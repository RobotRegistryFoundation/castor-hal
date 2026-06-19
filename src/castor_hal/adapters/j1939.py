"""J1939 read-only telemetry transport — the tractor/heavy-equipment wedge.

A modern tractor exposes a standardized, no-key set of broadcast telemetry on its
J1939 CAN bus (engine speed, wheel-based speed, coolant temp, fuel rate, …). This
transport TAPS that bus read-only and normalizes it into ``TransportState.
measurements`` — signed and shipped to Atlas by the gateway above it, exactly
like every other transport. It is the lowest-risk, near-universal first rung of
the tractor stack: data flowing, zero actuation.

It is deliberately a TELEMETRY-ONLY transport — ``capabilities`` is EMPTY, so any
``set_goal`` is a structured ``UNSUPPORTED_GOAL`` and ``estop`` is refused
honestly (a read tap cannot stop a machine — that's a hardware e-stop's job, and
pretending otherwise would be a safety lie). This also validates the HAL's
read-only-adapter case.

DRIVING a tractor is NOT this: steering/throttle/brake are not open over CAN and
require an OEM API or a physical retrofit kit (see
docs/strategy/2026-06-19-gateway-onramp-strategy.md). This adapter never writes
to the bus.

Install the bus driver with ``pip install castor-hal[j1939]`` (python-can). For
tests, inject any object with ``recv(timeout) -> frame | None`` and ``shutdown()``
where ``frame`` has ``.arbitration_id`` (29-bit) and ``.data`` (bytes).

CONFIDENCE: the PGN/SPN set + scalings below are the STANDARD J1939 definitions
(SAE J1939-71) and are a starter set — verify against the target ECU's documented
SPNs before relying on a specific value externally; extend ``PGN_DECODERS`` per
machine. Unknown PGNs are ignored.
"""

from __future__ import annotations

import time
from typing import Callable

from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.state import TransportState
from castor_hal.transport import GoalResult, Transport


def _u16le(data: bytes, lo: int) -> int | None:
    """Little-endian uint16 at byte offset ``lo``, or None if data too short."""
    if len(data) < lo + 2:
        return None
    return data[lo] | (data[lo + 1] << 8)


def _u8(data: bytes, i: int) -> int | None:
    return data[i] if len(data) > i else None


# --- standard J1939 PGN decoders: data(bytes) -> {measurement: value} ---------
def _eec1(d: bytes) -> dict:        # PGN 61444 (0xF004) — Electronic Engine Controller 1
    rpm = _u16le(d, 3)               # SPN 190 Engine Speed, 0.125 rpm/bit, bytes 4-5
    return {} if rpm is None else {"engine_speed_rpm": round(rpm * 0.125, 3)}


def _ccvs1(d: bytes) -> dict:       # PGN 65265 (0xFEF1) — Cruise Control/Vehicle Speed
    spd = _u16le(d, 1)               # SPN 84 Wheel-Based Vehicle Speed, 1/256 km/h/bit, bytes 2-3
    return {} if spd is None else {"wheel_speed_kmh": round(spd / 256.0, 3)}


def _et1(d: bytes) -> dict:         # PGN 65262 (0xFEEE) — Engine Temperature 1
    t = _u8(d, 0)                    # SPN 110 Coolant Temp, 1 °C/bit, offset -40, byte 1
    return {} if t is None else {"coolant_temp_c": t - 40}


def _lfe1(d: bytes) -> dict:        # PGN 65266 (0xFEF2) — Fuel Economy (Liquid)
    fr = _u16le(d, 0)                # SPN 183 Fuel Rate, 0.05 L/h/bit, bytes 1-2
    return {} if fr is None else {"fuel_rate_lph": round(fr * 0.05, 3)}


def _eec2(d: bytes) -> dict:        # PGN 61443 (0xF003) — Electronic Engine Controller 2
    ap = _u8(d, 1)                   # SPN 91 Accelerator Pedal Position 1, 0.4 %/bit, byte 2
    return {} if ap is None else {"accel_pedal_pct": round(ap * 0.4, 2)}


#: PGN → (name, decoder). The standard starter set; extend per machine.
PGN_DECODERS: dict[int, tuple[str, Callable[[bytes], dict]]] = {
    61444: ("EEC1", _eec1),
    65265: ("CCVS1", _ccvs1),
    65262: ("ET1", _et1),
    65266: ("LFE1", _lfe1),
    61443: ("EEC2", _eec2),
}


def pgn_of(arbitration_id: int) -> int:
    """Extract the J1939 PGN from a 29-bit CAN arbitration id.

    Layout: PRIO(3) EDP(1) DP(1) PF(8) PS(8) SA(8). PDU1 (PF<240) is
    destination-specific — PS is the destination address and is NOT part of the
    PGN; PDU2 (PF>=240) is broadcast — PS belongs to the PGN.
    """
    pf = (arbitration_id >> 16) & 0xFF
    top = (arbitration_id >> 8) & 0x3FFFF  # EDP|DP|PF|PS (18 bits)
    return top & 0x3FF00 if pf < 240 else top


class J1939ReadTransport(Transport):
    """Read-only J1939/CAN telemetry tap. ``capabilities`` is empty by design."""

    capabilities = frozenset()  # telemetry-only: accepts NO goals
    name = "j1939-read"
    description = "read-only J1939/CAN telemetry tap (no actuation)"

    def __init__(
        self,
        bus=None,  # noqa: ANN001 — duck-typed python-can Bus (or a test fake)
        *,
        channel: str = "can0",
        interface: str = "socketcan",
        drain_window_s: float = 0.5,
        max_frames: int = 200,
        recv_timeout_s: float = 0.05,
    ) -> None:
        self._bus = bus
        self._channel = channel
        self._interface = interface
        self._drain_window_s = drain_window_s
        self._max_frames = max_frames
        self._recv_timeout_s = recv_timeout_s

    @classmethod
    def from_config(cls, config_dict: dict | None) -> "J1939ReadTransport":
        cfg = config_dict or {}
        return cls(
            channel=str(cfg.get("channel", "can0")),
            interface=str(cfg.get("interface", "socketcan")),
            drain_window_s=float(cfg.get("drain_window_s", 0.5)),
        )

    def open(self) -> None:
        if self._bus is not None:
            return
        try:
            import can  # lazily imported: only needed on real hardware
        except ModuleNotFoundError as exc:
            raise TransportError(
                TransportErrorCode.NOT_CONNECTED,
                "python-can not installed — `pip install castor-hal[j1939]`",
            ) from exc
        try:
            self._bus = can.Bus(channel=self._channel, interface=self._interface)
        except Exception as exc:  # noqa: BLE001 — surface any bring-up failure structurally
            raise TransportError(
                TransportErrorCode.NOT_CONNECTED,
                f"cannot open CAN {self._interface}:{self._channel}: {exc}",
                detail={"channel": self._channel, "interface": self._interface},
            ) from exc

    def close(self) -> None:
        bus, self._bus = self._bus, None
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:  # noqa: BLE001 — close is best-effort
                pass

    def set_goal(self, goal) -> GoalResult:
        # Telemetry-only: ensure_supported always refuses (capabilities is empty).
        self.ensure_supported(goal)
        raise AssertionError("unreachable: empty capabilities refuse every goal")  # pragma: no cover

    def read_state(self) -> TransportState:
        if self._bus is None:
            raise TransportError(TransportErrorCode.NOT_CONNECTED, "CAN bus not open")
        measurements: dict[str, float] = {}
        seen_pgns: set[int] = set()
        deadline = time.monotonic() + self._drain_window_s
        frames = 0
        frames_invalid = 0  # known PGN whose payload was too short to decode
        try:
            while frames < self._max_frames and time.monotonic() < deadline:
                msg = self._bus.recv(timeout=self._recv_timeout_s)
                if msg is None:
                    break
                frames += 1
                pgn = pgn_of(int(msg.arbitration_id))
                dec = PGN_DECODERS.get(pgn)
                if dec is None:
                    continue  # unrecognized PGN — not invalid, just not decoded here
                seen_pgns.add(pgn)
                decoded = dec[1](bytes(msg.data))
                if decoded:
                    measurements.update(decoded)
                else:
                    # a known PGN that decoded to nothing = a truncated/corrupt
                    # payload; surface it so silent empty telemetry is detectable.
                    frames_invalid += 1
        except TransportError:
            raise
        except Exception as exc:  # noqa: BLE001 — bus read failure → structured IO_ERROR
            raise TransportError(TransportErrorCode.IO_ERROR, f"CAN read failed: {exc}") from exc

        return TransportState(
            connected=True,
            estopped=False,
            measurements=measurements,
            timestamp_s=time.monotonic(),
            extra={"frames_read": frames, "frames_invalid": frames_invalid,
                   "pgns_decoded": sorted(seen_pgns)},
        )

    def estop(self) -> None:
        # HONEST SAFETY: a read-only tap physically cannot stop the machine.
        # Refuse loudly so the caller never believes a stop occurred — the stop
        # must come from a hardware e-stop, not this transport.
        raise TransportError(
            TransportErrorCode.UNSUPPORTED_GOAL,
            "j1939-read is telemetry-only and cannot command a stop; "
            "wire a hardware e-stop (this transport never writes to the bus)",
        )


def make_hal_actuator(config_dict: dict | None = None):
    """An SO-ARM101-style convenience: a gateway actuator that exposes the J1939
    tap's ``read_state`` (and refuses actuation) through ``TransportActuator``."""
    from castor_hal.actuator import TransportActuator

    return TransportActuator(
        factory=J1939ReadTransport.from_config,
        name="j1939-read",
        description=J1939ReadTransport.description,
    )
