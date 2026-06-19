"""J1939ReadTransport — read-only CAN telemetry tap (the tractor wedge).

No python-can needed: a FakeCanBus injects scripted frames. Also exercises the
telemetry-only contract (no goals, honest estop refusal) end-to-end through the
gateway bridge."""

from pathlib import Path

import pytest

from castor_hal.actuator import TransportActuator
from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import JointPositions
from castor_hal.adapters.j1939 import J1939ReadTransport, pgn_of


def j1939_id(pgn: int, sa: int = 0x00, prio: int = 3) -> int:
    """Broadcast (PDU2) arbitration id for a PGN. Valid for the PF>=240 PGNs used here."""
    return (prio << 26) | (pgn << 8) | sa


class FakeMsg:
    def __init__(self, arbitration_id: int, data: bytes) -> None:
        self.arbitration_id = arbitration_id
        self.data = data


class FakeCanBus:
    def __init__(self, frames: list[FakeMsg]) -> None:
        self._frames = list(frames)
        self.shutdown_called = False

    def recv(self, timeout=None):
        return self._frames.pop(0) if self._frames else None

    def shutdown(self) -> None:
        self.shutdown_called = True


# Standard J1939 frames: 800 rpm, 10 km/h, 85 °C coolant.
EEC1 = FakeMsg(j1939_id(61444), bytes([0, 0, 0, 0x00, 0x19, 0, 0, 0]))   # 6400*0.125 = 800 rpm
CCVS1 = FakeMsg(j1939_id(65265), bytes([0, 0x00, 0x0A, 0, 0, 0, 0, 0]))   # 2560/256 = 10 km/h
ET1 = FakeMsg(j1939_id(65262), bytes([0x7D, 0, 0, 0, 0, 0, 0, 0]))        # 125-40 = 85 °C


def test_pgn_extraction_pdu2():
    assert pgn_of(j1939_id(61444)) == 61444   # EEC1
    assert pgn_of(j1939_id(65265)) == 65265   # CCVS1
    # priority/source-address bits must not leak into the PGN
    assert pgn_of(j1939_id(61444, sa=0xEE, prio=6)) == 61444


def test_read_state_decodes_standard_telemetry():
    t = J1939ReadTransport(bus=FakeCanBus([EEC1, CCVS1, ET1]))
    st = t.read_state()
    assert st.connected is True
    assert st.measurements["engine_speed_rpm"] == pytest.approx(800.0)
    assert st.measurements["wheel_speed_kmh"] == pytest.approx(10.0)
    assert st.measurements["coolant_temp_c"] == 85
    assert st.extra["frames_read"] == 3
    assert 61444 in st.extra["pgns_decoded"]


def test_unknown_pgn_ignored_and_short_data_safe():
    unknown = FakeMsg(j1939_id(65300), bytes([1, 2, 3, 4]))
    short_eec1 = FakeMsg(j1939_id(61444), bytes([0, 0]))  # too short for SPN 190
    t = J1939ReadTransport(bus=FakeCanBus([unknown, short_eec1]))
    st = t.read_state()
    assert "engine_speed_rpm" not in st.measurements  # short frame decoded to {}
    assert st.measurements == {}
    # a KNOWN PGN with a truncated payload is counted invalid (observability);
    # an unknown PGN is not "invalid", just not decoded here.
    assert st.extra["frames_invalid"] == 1
    assert st.extra["frames_read"] == 2


class _OneThenNone:
    """recv() yields one frame, then None forever — exercises the early break."""
    def __init__(self, frame):
        self._frame = frame
        self.recv_calls = 0

    def recv(self, timeout=None):
        self.recv_calls += 1
        f, self._frame = self._frame, None
        return f

    def shutdown(self):
        pass


def test_drain_loop_exits_cleanly_on_recv_none():
    bus = _OneThenNone(EEC1)
    t = J1939ReadTransport(bus=bus)
    st = t.read_state()
    assert st.measurements["engine_speed_rpm"] == pytest.approx(800.0)
    assert st.extra["frames_read"] == 1
    # exactly 2 recv() calls: the frame, then the None that breaks the loop —
    # the loop does NOT spin to the deadline or max_frames.
    assert bus.recv_calls == 2


def test_read_state_without_open_is_not_connected():
    t = J1939ReadTransport()  # no bus injected, never opened
    with pytest.raises(TransportError) as ei:
        t.read_state()
    assert ei.value.code is TransportErrorCode.NOT_CONNECTED


def test_telemetry_only_refuses_goals():
    t = J1939ReadTransport(bus=FakeCanBus([]))
    with pytest.raises(TransportError) as ei:
        t.set_goal(JointPositions({"j1": 0.0}))
    assert ei.value.code is TransportErrorCode.UNSUPPORTED_GOAL


def test_estop_is_refused_honestly():
    # A read-only tap cannot stop the machine — it must NOT pretend to.
    t = J1939ReadTransport(bus=FakeCanBus([]))
    with pytest.raises(TransportError) as ei:
        t.estop()
    assert ei.value.code is TransportErrorCode.UNSUPPORTED_GOAL
    assert "hardware e-stop" in ei.value.message


def test_close_shuts_down_bus():
    bus = FakeCanBus([])
    t = J1939ReadTransport(bus=bus)
    t.close()
    assert bus.shutdown_called is True


def test_open_without_python_can_is_structured_not_connected():
    # python-can isn't installed in the test env; open() with no injected bus
    # must surface a structured NOT_CONNECTED, not a raw ImportError.
    t = J1939ReadTransport()
    with pytest.raises(TransportError) as ei:
        t.open()
    assert ei.value.code is TransportErrorCode.NOT_CONNECTED


# -- end-to-end through the gateway bridge ---------------------------------
def test_bridge_read_state_returns_measurements():
    a = TransportActuator(J1939ReadTransport(bus=FakeCanBus([EEC1, CCVS1])))
    out = a.execute(
        envelope={"tool_name": "read_state", "tool_args": {}},
        manifest_path=Path("ROBOT.md"), tier="read", config={},
    )
    assert out.success is True
    assert out.telemetry["measurements"]["engine_speed_rpm"] == pytest.approx(800.0)


def test_bridge_refuses_actuation_as_structured_error():
    a = TransportActuator(J1939ReadTransport(bus=FakeCanBus([])))
    out = a.execute(
        envelope={"tool_name": "move", "tool_args": {"joint_positions": {"j1": 0.0}}},
        manifest_path=Path("ROBOT.md"), tier="actuate", config={},
    )
    assert out.success is False
    assert out.telemetry["error_code"] == TransportErrorCode.UNSUPPORTED_GOAL.value


def test_bridge_estop_is_structured_error():
    a = TransportActuator(J1939ReadTransport(bus=FakeCanBus([])))
    out = a.execute(
        envelope={"tool_name": "estop", "tool_args": {}},
        manifest_path=Path("ROBOT.md"), tier="actuate", config={},
    )
    assert out.success is False
    assert out.telemetry["error_code"] == TransportErrorCode.UNSUPPORTED_GOAL.value
