"""Tests for castor_hal.adapters.ros2 (ROS2Transport)."""

from __future__ import annotations

import pytest

from castor_hal.adapters.ros2 import ROS2Config, ROS2Transport
from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import BodyTwist, FlightSetpoint, Home, JointPositions
from castor_hal.state import serialize_state


class _FakeROS2Bridge:
    def __init__(self) -> None:
        self.twists: list[tuple[str, tuple, tuple]] = []
        self.joint_cmds: list[tuple[str, dict[str, float]]] = []
        self.positions: dict[str, float] = {"shoulder_pan": 0.1, "elbow_flex": -0.2}
        self.velocities: dict[str, float] = {"shoulder_pan": 0.0, "elbow_flex": 0.0}
        self.odom: dict[str, float] = {"vx_mps": 0.2, "vy_mps": 0.0, "wz_rps": 0.05}
        self.ages: dict[str, float | None] = {"/joint_states": 0.05, "/odom": 0.04}
        self.closed = False

    def publish_twist(self, topic, linear, angular) -> None:
        self.twists.append((topic, linear, angular))

    def publish_joint_positions(self, topic, positions) -> None:
        self.joint_cmds.append((topic, dict(positions)))

    def read_joint_states(self, topic):
        return dict(self.positions), dict(self.velocities)

    def read_odom_twist(self, topic):
        return dict(self.odom)

    def topic_heartbeat_age_s(self, topic):
        return self.ages.get(topic)

    def close(self) -> None:
        self.closed = True


def test_ros2_body_twist_in_bounds():
    fake = _FakeROS2Bridge()
    with ROS2Transport(ROS2Config(max_linear_mps=0.5, max_angular_rps=1.0), backend=fake) as t:
        res = t.set_goal(BodyTwist(linear=(0.3, 0.0, 0.0), angular=(0.0, 0.0, 0.4)))
        assert res.reached is True
        assert fake.twists[-1] == ("/cmd_vel", (0.3, 0.0, 0.0), (0.0, 0.0, 0.4))


def test_ros2_body_twist_exceeds_linear_bound():
    fake = _FakeROS2Bridge()
    with ROS2Transport(ROS2Config(max_linear_mps=0.5), backend=fake) as t:
        with pytest.raises(TransportError) as exc:
            t.set_goal(BodyTwist(linear=(0.9, 0.0, 0.0)))
        assert exc.value.code is TransportErrorCode.OUT_OF_RANGE
        assert fake.twists == []


def test_ros2_body_twist_exceeds_angular_bound():
    fake = _FakeROS2Bridge()
    with ROS2Transport(ROS2Config(max_angular_rps=1.0), backend=fake) as t:
        with pytest.raises(TransportError) as exc:
            t.set_goal(BodyTwist(angular=(0.0, 0.0, 2.0)))
        assert exc.value.code is TransportErrorCode.OUT_OF_RANGE


def test_ros2_joint_positions_and_home():
    fake = _FakeROS2Bridge()
    cfg = ROS2Config(home_positions={"shoulder_pan": 0.0, "elbow_flex": 0.0})
    with ROS2Transport(cfg, backend=fake) as t:
        t.set_goal(JointPositions(positions={"shoulder_pan": 0.25}))
        t.set_goal(Home())
        assert fake.joint_cmds[0] == ("/joint_commands", {"shoulder_pan": 0.25})
        assert fake.joint_cmds[1] == ("/joint_commands", {"shoulder_pan": 0.0, "elbow_flex": 0.0})


def test_ros2_estop_publishes_zero_twist_and_latches():
    fake = _FakeROS2Bridge()
    with ROS2Transport(backend=fake) as t:
        t.estop()
        assert fake.twists[-1] == ("/cmd_vel", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        with pytest.raises(TransportError) as exc:
            t.set_goal(BodyTwist(linear=(0.1, 0.0, 0.0)))
        assert exc.value.code is TransportErrorCode.ESTOPPED
        t.clear_estop()
        res = t.set_goal(BodyTwist(linear=(0.1, 0.0, 0.0)))
        assert res.reached is True


def test_ros2_dds_stale_fault_isolation():
    fake = _FakeROS2Bridge()
    fake.ages = {"/joint_states": 5.5, "/odom": 6.0}
    with ROS2Transport(ROS2Config(stale_topic_s=2.0), backend=fake) as t:
        st = t.read_state()
        assert st.connected is False
        assert st.extra["dds_healthy"] is False
        assert st.extra["fault_domain"] == "ros2_dds_stale"
        wire = serialize_state(st)
        assert wire["connected"] is False


def test_ros2_read_only_refuses_all_goals():
    fake = _FakeROS2Bridge()
    with ROS2Transport(ROS2Config(read_only=True), backend=fake) as t:
        with pytest.raises(TransportError) as exc:
            t.set_goal(BodyTwist(linear=(0.1, 0.0, 0.0)))
        assert exc.value.code is TransportErrorCode.UNSUPPORTED_GOAL


def test_ros2_unsupported_goal_rejected():
    fake = _FakeROS2Bridge()
    with ROS2Transport(backend=fake) as t:
        with pytest.raises(TransportError) as exc:
            t.set_goal(FlightSetpoint(north_m=1.0, east_m=0.0, down_m=-1.0))
        assert exc.value.code is TransportErrorCode.UNSUPPORTED_GOAL


def test_ros2_not_connected_raises():
    t = ROS2Transport(backend=_FakeROS2Bridge())
    with pytest.raises(TransportError) as exc:
        t.read_state()
    assert exc.value.code is TransportErrorCode.NOT_CONNECTED


def test_ros2_class_level_capabilities_work_with_lazy_transport_actuator():
    """TransportActuator(factory=..., transport_type=ROS2Transport) reads
    class-level capabilities and can_estop before the instance exists."""
    from castor_hal.actuator import TransportActuator

    fake = _FakeROS2Bridge()
    a = TransportActuator(
        factory=lambda _cfg: ROS2Transport(backend=fake),
        transport_type=ROS2Transport,
        name="ros2-lazy",
    )
    assert "cmd_vel" in a.motion_capabilities
    assert "move" in a.motion_capabilities
    assert "home" in a.motion_capabilities
    assert "estop" in a.stop_capabilities

    out = a.execute(
        envelope={"tool_name": "cmd_vel", "tool_args": {"linear": [0.2, 0.0, 0.0]}},
        manifest_path=None,
        tier="actuate",
        config={},
    )
    assert out.success is True and out.outcome_kind == "executed"
    assert fake.twists[-1] == ("/cmd_vel", (0.2, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_ros2_read_only_does_not_advertise_estop_or_publish_cmd_vel():
    from castor_hal.actuator import TransportActuator

    fake = _FakeROS2Bridge()
    t = ROS2Transport(ROS2Config(read_only=True), backend=fake)
    assert t.capabilities == frozenset()
    assert t.can_estop is False

    a = TransportActuator(t)
    assert a.motion_capabilities == frozenset()
    assert a.stop_capabilities == frozenset()

    with t:
        with pytest.raises(TransportError) as exc:
            t.estop()
        assert exc.value.code is TransportErrorCode.UNSUPPORTED_GOAL
    assert fake.twists == []


def test_ros2_estop_and_clear_estop_through_transport_actuator():
    from castor_hal.actuator import TransportActuator

    fake = _FakeROS2Bridge()
    t = ROS2Transport(backend=fake)
    a = TransportActuator(t)

    # Estop latches and blocks subsequent motion as a signed refusal (`denied`).
    stop_out = a.execute(envelope={"tool_name": "estop", "tool_args": {}}, manifest_path=None, tier="read", config={})
    assert stop_out.success is True and stop_out.telemetry["estopped"] is True

    refused_move = a.execute(
        envelope={"tool_name": "cmd_vel", "tool_args": {"linear": [0.2, 0.0, 0.0]}},
        manifest_path=None,
        tier="actuate",
        config={},
    )
    assert refused_move.success is False
    assert refused_move.outcome_kind == "denied"
    assert refused_move.telemetry["deny"] == "estopped"

    # Non-commission tier cannot clear estop.
    bad_clear = a.execute(
        envelope={"tool_name": "clear_estop", "tool_args": {}},
        manifest_path=None,
        tier="actuate",
        config={},
    )
    assert bad_clear.success is False
    assert bad_clear.outcome_kind == "denied"
    assert bad_clear.telemetry["deny"] == "tool_tier"

    # Commission tier clears estop and allows bounded motion again.
    ok_clear = a.execute(
        envelope={"tool_name": "clear_estop", "tool_args": {}},
        manifest_path=None,
        tier="commission",
        config={},
    )
    assert ok_clear.success is True and ok_clear.telemetry["estopped"] is False

    # Out-of-range velocity is refused (`denied`), not an error (500).
    too_fast = a.execute(
        envelope={"tool_name": "cmd_vel", "tool_args": {"linear": [5.0, 0.0, 0.0]}},
        manifest_path=None,
        tier="actuate",
        config={},
    )
    assert too_fast.success is False
    assert too_fast.outcome_kind == "denied"
    assert too_fast.telemetry["deny"] == "out_of_range"

    a.close()
    assert t._open is False
