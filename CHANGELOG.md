# Changelog

All notable changes to `castor-hal`. Format loosely follows
[Keep a Changelog](https://keepachangelog.com/); versions follow semver.

The 0.1.1 entry is reconstructed from the git history, which begins there.

## [0.2.0] - 2026-10-23

`TransportActuator` now tells robot-md-gateway which of its tools stop the
machine and which move it. Before, it declared neither, so the gateway's
motion gates (actuation scope, motion HiTL, motion confidence) did not count
castor-hal motion as motion: a read-tier bearer could move a goal-accepting
transport by writing scope `OBSERVE`, and an actuate bearer could pass the
HiTL and motion-confidence gates by labelling a move `NAVIGATE`. Its startup
audit could not see the actuator, and its stop path could neither let a
castor-hal stop through a tripped safety monitor nor send one on a trip. From
the review of robot-md-gateway#35 (Q5).

### Added

- **`TransportActuator.stop_capabilities`**: `estop`, `stop`,
  `emergency_stop` and `halt`, the exact spellings routed to
  `Transport.estop`; none for a transport that cannot stop.
- **`TransportActuator.motion_capabilities`**: every tool name `parse_goal`
  maps to a goal kind the transport accepts. The two sets never overlap.
- **`Transport.can_estop`** (default True). False on the J1939 read-only tap,
  whose `estop()` always refuses, so it declares no stop.
- **`TransportActuator(factory=..., transport_type=...)`**: the `Transport`
  subclass a lazy factory builds, so the declarations can be exact before the
  transport exists. Without it they fail safe: every goal tool is declared as
  motion, and the stops are declared. A `transport_type` that is not a
  `Transport` subclass raises `TypeError`. J1939's `make_hal_actuator` passes
  it.
- **`GOAL_TOOL_NAMES`** and **`goal_tool_names()`**: the one table of goal tool
  names, which `parse_goal` now reads too. The names themselves are unchanged.

### Changed (behaviour)

- **A goal tool must be spelled exactly as declared.** `parse_goal` ignores
  case and surrounding space, and the gateway does not, so `MOVE` and `" move"`
  used to move the machine past the gateway's motion gates. They are now
  refused as `unsupported_goal`, with a hint naming the declared spelling.
  Stops are still routed whatever their case, but only the four exact
  spellings are declared, so only they pass a tripped gateway.
- **The bridge checks the goal kind before `set_goal`**
  (`Transport.ensure_supported`). An adapter whose `set_goal` skipped that
  check could be handed a goal kind outside its `capabilities`: `goto`, a
  flight setpoint, reached an adapter that accepts joint, home and drive goals.

### Fixed

- **A lazily built transport is built once.** `_ensure_transport` checked and
  built with no lock. robot-md-gateway#35 sends the declared stop from its own
  thread while requests run on its worker pool, so on first use a stop and a
  request could each build a transport, and the stop could land on the copy
  that was thrown away. A double-checked lock now builds one.

### Known

- The bridge still reports its own refusals (`unsupported_goal`,
  `invalid_goal`) as `outcome_kind="error"`, which robot-md-gateway answers
  with HTTP 500 on an allow record. Reporting them as `denied` is a separate
  change, planned for before this version is tagged.

## [0.1.1] - 2026-06-19

- The transport HAL (`Goal`, `Transport`, `TransportState`, `TransportError`,
  `TransportActuator`) and the J1939 read-only telemetry adapter
  (`castor_hal.adapters.j1939`): standard PGNs normalized into
  `TransportState.measurements`, no goals accepted, and an `estop()` that
  refuses, because a read-only tap cannot stop a machine.
