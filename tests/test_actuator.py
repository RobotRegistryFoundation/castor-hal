"""TransportActuator — the gateway bridge. Routes an RCAN envelope to a
Transport and returns a structured ActuatorOutcome. This is the contract the
whole HAL exists to satisfy, so it's tested hardest."""

from pathlib import Path

import pytest

from castor_hal.actuator import TransportActuator
from castor_hal.errors import TransportErrorCode

MANIFEST = Path("ROBOT.md")


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
