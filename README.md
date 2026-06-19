# castor-hal

**The transport hardware-abstraction layer for `robot-md-gateway`.** One actuator
interface fronts every physical link — serial bus servos, CAN/J1939, ISOBUS,
ROS 2, MAVLink — so the gateway's trust pipeline (ROBOT.md verification,
tier/confidence/HiTL default-deny gating, Ed25519 signing, RCAN → Atlas) is
written once and reused unchanged across every machine.

## Why

`robot-md-gateway` already does the hard, transport-independent work and exposes
a pluggable `Actuator` Protocol. What was missing was a shared seam so each new
driver didn't re-implement envelope parsing, error reporting, and telemetry.
`castor-hal` is that seam.

Writing a new device driver becomes: **implement `Transport` (5 methods),
declare its `capabilities`, wrap it in `TransportActuator`.** The gateway plumbing
is free.

## The model

- **`Goal`** — a small, closed set of normalized intents (`JointPositions`,
  `JointTrajectory`, `BodyTwist`, `FlightSetpoint`, `Gripper`, `RateSetpoint`,
  `SectionState`, `Home`). `parse_goal(tool_name, tool_args)` is the one canonical
  mapping from a gateway RCAN envelope to a `Goal`.
- **`Transport`** — the device-specific seam: `open / close / set_goal /
  read_state / estop`, plus `capabilities` (the `GoalKind`s it accepts).
- **`TransportState` / `serialize_state`** — one normalized telemetry shape.
- **`TransportError` / `TransportErrorCode`** — one typed failure taxonomy.
- **`TransportActuator`** — adapts any `Transport` to the gateway's `Actuator`
  Protocol, turning success / `TransportError` / unexpected exceptions into a
  structured `ActuatorOutcome`.

```python
from castor_hal import Transport, TransportActuator, GoalKind, GoalResult, TransportState

class MyTransport(Transport):
    capabilities = frozenset({GoalKind.JOINT_POSITIONS, GoalKind.HOME})
    def open(self): ...
    def close(self): ...
    def set_goal(self, goal) -> GoalResult: self.ensure_supported(goal); ...
    def read_state(self) -> TransportState: ...
    def estop(self): ...   # best-effort software stop — see SAFETY below

actuator = TransportActuator(MyTransport())   # now a valid gateway actuator
```

## SAFETY (binding)

`estop()` is a **best-effort software stop** — it commands the device to halt as
fast as the link allows. It is **not** a safety-rated emergency stop and **not** a
substitute for a hardware e-stop wired to a safety-rated controller (ISO 13850).
The transport can refuse or command a stop; it **cannot guarantee the machine
physically stopped**, and link latency means it is never in the real-time safety
loop. A signed refusal proves a command was blocked at the gate — nothing more.
Never market the gateway or the HAL as "safety."

## Status

Alpha. Core (`goal` / `state` / `errors` / `transport`) is pure and
gateway-independent; the `TransportActuator` bridge depends on
`robot-md-gateway`. First reference adapter: SO-ARM101 (serial bus servos), in
the `so-arm101-actuator` package.
