"""TransportActuator — the gateway bridge. Routes an RCAN envelope to a
Transport and returns a structured ActuatorOutcome. This is the contract the
whole HAL exists to satisfy, so it's tested hardest."""

import threading
import time
from pathlib import Path

import pytest

from castor_hal.actuator import TransportActuator
from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import GoalKind, goal_tool_names
from castor_hal.state import TransportState
from castor_hal.transport import GoalResult, Transport

MANIFEST = Path("ROBOT.md")

#: The tools the bridge routes to Transport.estop and Transport.read_state.
STOPS = frozenset({"estop", "stop", "emergency_stop", "halt"})
READS = frozenset({"read_state", "get_state", "read"})


def _exec(actuator, tool_name, tool_args=None, *, tier="actuate", config=None):
    return actuator.execute(
        envelope={"tool_name": tool_name, "tool_args": tool_args or {}, "msg_id": "m1"},
        manifest_path=MANIFEST,
        tier=tier,
        config=config or {},
    )


def test_move_executes_and_reports_reached(fake):
    a = TransportActuator(fake)
    out = _exec(a, "move", {"joint_positions": {"j1": 0.5}})
    assert out.success is True
    assert out.outcome_kind == "executed"
    assert out.telemetry["reached"] is True
    assert out.telemetry["goal_kind"] == "joint_positions"
    assert out.telemetry["state"]["positions"] == {"j1": 0.5}
    assert fake.open_calls >= 1  # lazily opened


def test_read_state_routes_to_transport(fake):
    a = TransportActuator(fake)
    out = _exec(a, "read_state")
    assert out.success is True and out.outcome_kind == "executed"
    assert out.telemetry["connected"] is True
    assert "positions" in out.telemetry


def test_estop_routes_to_transport(fake):
    a = TransportActuator(fake)
    out = _exec(a, "estop")
    assert out.success is True
    assert out.telemetry == {"estopped": True}
    assert fake.estop_calls == 1
    assert fake.estopped is True


def test_unsupported_goal_is_structured_error_not_a_crash(fake):
    a = TransportActuator(fake)  # fake accepts only joint_positions/home
    out = _exec(a, "cmd_vel", {"linear": [1, 0, 0]})
    assert out.success is False
    assert out.outcome_kind == "error"
    assert out.telemetry["error_code"] == TransportErrorCode.UNSUPPORTED_GOAL.value


def test_unknown_tool_is_unsupported_goal_error(fake):
    a = TransportActuator(fake)
    out = _exec(a, "frobnicate")
    assert out.success is False
    assert out.telemetry["error_code"] == TransportErrorCode.UNSUPPORTED_GOAL.value


def test_invalid_goal_is_structured_error(fake):
    a = TransportActuator(fake)
    out = _exec(a, "move", {})  # missing joint_positions
    assert out.success is False
    assert out.telemetry["error_code"] == TransportErrorCode.INVALID_GOAL.value


def test_malformed_numeric_goal_is_invalid_not_internal(fake):
    # review #1/#2: a non-numeric joint value must surface as INVALID_GOAL
    # (operator-actionable), never INTERNAL (looks like an adapter bug).
    a = TransportActuator(fake)
    out = _exec(a, "move", {"joint_positions": {"j1": "not-a-number"}})
    assert out.success is False
    assert out.telemetry["error_code"] == TransportErrorCode.INVALID_GOAL.value
    assert out.telemetry["error_code"] != TransportErrorCode.INTERNAL.value


def test_not_reached_yields_success_false_but_still_executed(make_fake):
    a = TransportActuator(make_fake(reach=False))
    out = _exec(a, "move", {"joint_positions": {"j1": 0.5}})
    assert out.outcome_kind == "executed"
    assert out.success is False
    assert out.telemetry["reached"] is False


def test_transport_error_on_read_becomes_structured_outcome(make_fake):
    a = TransportActuator(make_fake(fail_read=True))
    out = _exec(a, "read_state")
    assert out.success is False
    assert out.outcome_kind == "error"
    assert out.telemetry["error_code"] == TransportErrorCode.IO_ERROR.value


def test_open_failure_becomes_structured_outcome(make_fake):
    a = TransportActuator(make_fake(fail_open=True))
    out = _exec(a, "read_state")
    assert out.success is False
    assert out.telemetry["error_code"] == TransportErrorCode.IO_ERROR.value


def test_unexpected_exception_is_caught_as_internal(make_fake):
    t = make_fake()

    def boom():
        raise RuntimeError("driver bug")

    t.read_state = boom  # simulate an adapter bug (non-TransportError)
    a = TransportActuator(t)
    out = _exec(a, "read_state")
    assert out.success is False
    assert out.outcome_kind == "error"
    assert out.telemetry["error_code"] == TransportErrorCode.INTERNAL.value
    assert "RuntimeError" in out.error_message


def test_lazy_factory_builds_transport_from_config(make_fake):
    built = {}

    def factory(config):
        built["config"] = config
        return make_fake()

    a = TransportActuator(factory=factory, name="lazy", description="d")
    assert a.name == "lazy"
    out = _exec(a, "read_state", config={"port": "/dev/ttyACM0"})
    assert out.success is True
    assert built["config"] == {"port": "/dev/ttyACM0"}  # config threaded to the factory


def test_requires_exactly_one_of_transport_or_factory(fake):
    with pytest.raises(ValueError):
        TransportActuator()  # neither
    with pytest.raises(ValueError):
        TransportActuator(fake, factory=lambda c: fake)  # both


def test_name_defaults_to_transport_name(fake):
    a = TransportActuator(fake)
    assert a.name == "fake"
    assert a.config_schema == {}


# -- stop_capabilities / motion_capabilities: what the gateway is told -------
#
# robot-md-gateway reads both off the actuator and matches tool names exactly.
# With neither declared it could not let a castor-hal stop through a tripped
# safety monitor, could not send one on a trip, and its motion gates (actuation
# scope, HiTL, confidence) did not see castor-hal motion as motion (review of
# robot-md-gateway#35, Q5).


def test_the_declared_stops_are_the_spellings_routed_to_estop(fake):
    a = TransportActuator(fake)
    assert a.stop_capabilities == STOPS
    for count, tool in enumerate(sorted(a.stop_capabilities), start=1):
        out = _exec(a, tool, tier="read")
        assert out.success is True and out.telemetry == {"estopped": True}
        assert fake.estop_calls == count
    assert fake.last_goal is None                  # no stop reached set_goal


def test_the_declared_motion_is_every_goal_tool_the_transport_accepts(fake):
    a = TransportActuator(fake)                    # accepts joint_positions and home
    assert a.motion_capabilities == {"move", "set_joint_positions", "joint_positions",
                                     "home", "go_home"}
    for tool in sorted(a.motion_capabilities):
        fake.last_goal = None
        out = _exec(a, tool, {"joint_positions": {"j1": 0.25}})
        assert out.outcome_kind == "executed", out.error_message
        assert fake.last_goal is not None          # it reached set_goal


def test_stops_and_motion_never_overlap(make_fake):
    """For every way an actuator is built, and for any transport: no tool is
    both a stop and a motion. A gateway lets a declared stop through a tripped
    monitor and sends it unprompted on a trip, so a motion tool in both sets
    would get both."""
    from castor_hal.adapters.j1939 import J1939ReadTransport

    class EveryGoal(make_fake):
        capabilities = frozenset(GoalKind)

    built = [
        TransportActuator(make_fake()),
        TransportActuator(EveryGoal()),
        TransportActuator(factory=lambda config: EveryGoal()),            # fail-safe sets
        TransportActuator(factory=lambda config: EveryGoal(), transport_type=EveryGoal),
        TransportActuator(J1939ReadTransport()),
    ]
    for a in built:
        assert a.stop_capabilities.isdisjoint(a.motion_capabilities), a.name
    assert TransportActuator.stop_capabilities.isdisjoint(TransportActuator.motion_capabilities)
    # Nor could they ever: no stop or read spelling is a goal tool at all.
    assert STOPS.isdisjoint(goal_tool_names(GoalKind))
    assert READS.isdisjoint(goal_tool_names(GoalKind))
    assert STOPS.isdisjoint(READS)


@pytest.mark.parametrize("spelling", ["MOVE", "Move", " move", "move ", "HOME"])
def test_a_goal_tool_spelled_otherwise_than_declared_is_refused(fake, spelling):
    """parse_goal ignores case and surrounding space; the gateway does not. A
    spelling the gateway cannot see as motion must not move the machine."""
    a = TransportActuator(fake)
    out = _exec(a, spelling, {"joint_positions": {"j1": 0.5}})
    assert out.success is False and out.outcome_kind == "error"
    assert out.telemetry["error_code"] == TransportErrorCode.UNSUPPORTED_GOAL.value
    assert "send it as" in out.error_message
    assert fake.last_goal is None
    assert fake.read_state().positions == {"j1": 0.0}


def test_a_stop_spelled_otherwise_still_stops_but_is_not_declared(fake):
    """The stop stays generous: refusing a stop over its spelling is the wrong
    failure. Only the exact spellings are declared, because they are all the
    gateway can match."""
    a = TransportActuator(fake)
    for spelling in ("ESTOP", " Halt ", "Emergency_Stop"):
        assert spelling not in a.stop_capabilities
        assert _exec(a, spelling).telemetry == {"estopped": True}
    assert fake.estop_calls == 3


def test_the_bridge_checks_the_goal_kind_when_the_adapter_does_not(make_fake):
    """motion_capabilities is derived from the transport's capabilities. An
    adapter whose set_goal forgets ensure_supported must not make that
    declaration false."""

    class Careless(make_fake):
        capabilities = frozenset({GoalKind.HOME})

        def set_goal(self, goal) -> GoalResult:       # no ensure_supported
            self.last_goal = goal
            return GoalResult(reached=True)

    t = Careless()
    a = TransportActuator(factory=lambda config: t)   # declares every goal tool
    assert "move" in a.motion_capabilities
    out = _exec(a, "move", {"joint_positions": {"j1": 0.5}})
    assert out.telemetry["error_code"] == TransportErrorCode.UNSUPPORTED_GOAL.value
    assert t.last_goal is None
    assert _exec(a, "home").outcome_kind == "executed"


def test_a_lazy_actuator_fails_safe_until_given_its_transport_type(make_fake):
    """The gateway reads the declarations at startup, before a factory has
    built anything. Unknown, every goal tool is motion and the stops are
    declared; with transport_type=, they are exact."""
    lazy = TransportActuator(factory=lambda config: make_fake())
    assert lazy.motion_capabilities == goal_tool_names(GoalKind)
    assert lazy.stop_capabilities == STOPS
    typed = TransportActuator(factory=lambda config: make_fake(), transport_type=make_fake)
    assert typed.motion_capabilities == TransportActuator(make_fake()).motion_capabilities
    assert typed.stop_capabilities == STOPS


def test_transport_type_goes_only_with_a_factory(fake):
    with pytest.raises(ValueError):
        TransportActuator(fake, transport_type=type(fake))


@pytest.mark.parametrize("wrong", [object, "FakeTransport", 3])
def test_transport_type_must_be_a_transport_subclass(make_fake, wrong):
    """The declarations are read off it, so it has to be a Transport class: not
    another class, a name or an instance."""
    with pytest.raises(TypeError):
        TransportActuator(factory=lambda config: make_fake(), transport_type=wrong)
    with pytest.raises(TypeError):
        TransportActuator(factory=lambda config: make_fake(), transport_type=make_fake())


def test_a_transport_that_cannot_stop_declares_no_stop(make_fake):
    class Tap(make_fake):
        capabilities = frozenset()
        can_estop = False

    a = TransportActuator(Tap())
    assert a.stop_capabilities == frozenset()
    assert a.motion_capabilities == frozenset()


# -- a lazily built transport, built once ----------------------------------


def test_a_stop_and_a_goal_on_first_use_reach_the_same_transport(make_fake):
    """robot-md-gateway#35 sends castor-hal's declared stop from its own
    watchdog thread, while requests run on its worker pool. On first use both
    could build a transport: the stop went to one, the goal to the other, and
    the one kept was never stopped (review of castor-hal#1, note 1). Now one
    transport is built, and the one kept is the one stopped."""
    built = []

    def slow_factory(config):
        time.sleep(0.2)                            # a serial or CAN bring-up
        transport = make_fake()
        built.append(transport)
        return transport

    a = TransportActuator(factory=slow_factory, transport_type=make_fake)
    start = threading.Barrier(2)
    outcomes = {}

    def call(tool, args=None):
        start.wait()
        outcomes[tool] = _exec(a, tool, args)

    threads = [threading.Thread(target=call, args=("emergency_stop",)),
               threading.Thread(target=call, args=("move", {"joint_positions": {"j1": 0.5}}))]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(5)

    assert len(built) == 1, f"{len(built)} transports were built"
    [transport] = built
    assert a._transport is transport
    assert transport.estop_calls == 1                       # the stop reached the one kept
    assert transport.last_goal is not None                  # and so did the goal
    assert outcomes["emergency_stop"].success and outcomes["move"].success


class _Exclusive(Transport):
    """open() as most adapters write it: check, then a slow exclusive open that
    refuses a second holder (open(2), flock, TIOCEXCL)."""

    capabilities = frozenset({GoalKind.JOINT_POSITIONS})
    holders = 0

    def __init__(self):
        self.handle = None
        self.estop_calls = 0

    def open(self):
        if self.handle is not None:
            return
        time.sleep(0.2)
        if _Exclusive.holders:
            raise TransportError(TransportErrorCode.IO_ERROR, "Device or resource busy")
        _Exclusive.holders += 1
        self.handle = object()

    def close(self):
        pass

    def set_goal(self, goal):
        return GoalResult(reached=True)

    def read_state(self):
        return TransportState(connected=True, estopped=False, positions={}, timestamp_s=1.0)

    def estop(self):
        self.estop_calls += 1


def test_a_stop_and_a_read_on_first_use_open_an_exclusive_transport_once():
    """Building once is not enough: the first open() must be once too. With the
    build locked and the open not, a stop and a request released together each
    opened the one transport, the second got EBUSY, and when that was the stop,
    nothing stopped (review of castor-hal#1, round 2, note 1: 5 of 10 trials)."""
    _Exclusive.holders = 0
    a = TransportActuator(factory=lambda config: _Exclusive(), transport_type=_Exclusive)
    start = threading.Barrier(2)
    outcomes = {}

    def call(tool):
        start.wait()
        outcomes[tool] = _exec(a, tool, tier="read")

    threads = [threading.Thread(target=call, args=(tool,))
               for tool in ("emergency_stop", "read_state")]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(5)

    assert outcomes["emergency_stop"].success, outcomes["emergency_stop"].error_message
    assert outcomes["read_state"].success, outcomes["read_state"].error_message
    assert a._transport.estop_calls == 1
    assert _Exclusive.holders == 1
