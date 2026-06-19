"""serialize_state — stable telemetry wire form, honest-empty."""

from castor_hal.state import TransportState, serialize_state


def test_empty_state_only_has_flags():
    out = serialize_state(TransportState())
    assert out == {"connected": False, "estopped": False}


def test_present_fields_ride_the_wire():
    s = TransportState(
        connected=True,
        estopped=False,
        positions={"j1": 0.5},
        temperatures_c={"j1": 31.0},
        measurements={"ground_speed_kmh": 7.2},
        timestamp_s=12.5,
        unit="rad",
        extra={"port": "/dev/ttyACM0"},
    )
    out = serialize_state(s)
    assert out["connected"] is True
    assert out["positions"] == {"j1": 0.5}
    assert out["unit"] == "rad"
    assert out["temperatures_c"] == {"j1": 31.0}
    assert out["measurements"] == {"ground_speed_kmh": 7.2}
    assert out["timestamp_s"] == 12.5
    assert out["extra"] == {"port": "/dev/ttyACM0"}


def test_unit_only_emitted_with_positions():
    # No positions → no 'unit' key (it would be meaningless noise).
    out = serialize_state(TransportState(measurements={"rpm": 540.0}))
    assert "unit" not in out
    assert out["measurements"] == {"rpm": 540.0}


def test_serialize_is_a_copy_not_a_reference():
    s = TransportState(positions={"j1": 0.0})
    out = serialize_state(s)
    out["positions"]["j1"] = 9.9
    assert s.positions == {"j1": 0.0}  # source untouched
