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
  read_state / estop`, plus `capabilities` (the `GoalKind`s it accepts) and
  `can_estop` (False only for a transport that cannot stop the machine at all,
  such as the read-only J1939 tap).
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

### What the gateway is told

`TransportActuator` declares which tools stop the machine and which move it.
robot-md-gateway reads both, and matches tool names exactly:

- **`stop_capabilities`** — `estop`, `stop`, `emergency_stop`, `halt`, or none
  when `can_estop` is False. From robot-md-gateway#35 on, a declared stop gets
  through a tripped safety monitor, and the gateway sends the first one
  (`emergency_stop`) itself when it trips.
- **`motion_capabilities`** — every tool name `parse_goal` maps to a goal kind
  the transport accepts (`GOAL_TOOL_NAMES`). The gateway treats them all as
  motion: it requires an actuation scope, runs the motion HiTL and confidence
  gates, and checks for a stop on the allowlist.

The two never overlap. A goal tool reaches the transport only when it is in
`motion_capabilities` and spelled exactly as declared (`MOVE` is refused; send
`move`), and only when the transport accepts its goal kind.

Only the four declared stop spellings, sent exactly, get through a tripped
gateway: `estop`, `stop`, `emergency_stop` and `halt`. castor-hal still routes
other cases and spacing (`ESTOP`, `" halt"`) to `estop`, but the gateway matches
declarations exactly: such a spelling needs its own allowlist entry, and a
tripped gateway refuses it. Allowlist the four and send them exactly. With more
than one castor-hal actuator on a gateway, each declares the same four, so a
stop must name its actuator (`actuator_name`); the gateway refuses one that
does not. The gateway's own stop on a trip goes to every actuator that declares
one.

A transport built lazily with `factory=` declares from `transport_type=` if you
pass it. Without it, its type is not known when the gateway reads the
declarations, so they fail safe: every goal tool is declared as motion, and the
stops are declared. If the factory builds something else, or a transport whose
`can_estop` disagrees with what was declared, the first build logs it at ERROR
(`transport_type_mismatch`, `stop_declared_but_refused`, `stop_not_declared`)
and carries on; it never refuses the call. Keep I/O out of the factory and do
it in `open()`: the build and the first `open()` run under one lock, and every
first call waits for them, the gateway's stop included.

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
