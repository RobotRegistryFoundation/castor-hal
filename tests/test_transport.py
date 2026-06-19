"""Transport base — capability gating + context manager, via FakeTransport."""

import pytest

from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import BodyTwist, Home, JointPositions
from castor_hal.transport import GoalResult, Transport


def test_supports_and_ensure_supported(fake):
    assert fake.supports(JointPositions({"j1": 0.0})) is True
    assert fake.supports(Home()) is True
    assert fake.supports(BodyTwist()) is False
    # ensure_supported is a no-op for accepted goals…
    fake.ensure_supported(JointPositions({"j1": 0.0}))
    # …and a structured UNSUPPORTED_GOAL for the rest.
    with pytest.raises(TransportError) as ei:
        fake.ensure_supported(BodyTwist())
    assert ei.value.code is TransportErrorCode.UNSUPPORTED_GOAL
    assert ei.value.detail["goal_kind"] == "body_twist"
    assert "joint_positions" in ei.value.detail["capabilities"]


def test_set_goal_rejects_unsupported_via_ensure(fake):
    with pytest.raises(TransportError) as ei:
        fake.set_goal(BodyTwist())
    assert ei.value.code is TransportErrorCode.UNSUPPORTED_GOAL


def test_context_manager_opens_and_closes(fake):
    with fake as opened:
        assert opened is fake
        assert fake.open_calls == 1
    assert fake.close_calls == 1


def test_set_goal_records_and_returns_result(make_fake):
    t = make_fake(reach=True)
    res = t.set_goal(JointPositions({"j1": 0.5}))
    assert isinstance(res, GoalResult)
    assert res.reached is True
    assert res.detail["applied"] is True
    assert res.state.positions == {"j1": 0.5}


def test_not_reached_is_honest(make_fake):
    t = make_fake(reach=False)
    res = t.set_goal(JointPositions({"j1": 0.5}))
    assert res.reached is False


def test_transport_is_abstract():
    with pytest.raises(TypeError):
        Transport()  # cannot instantiate the ABC directly
