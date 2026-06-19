"""Shared test doubles + fixtures for castor-hal."""

from __future__ import annotations

import pytest

from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import GoalKind, JointPositions
from castor_hal.state import TransportState
from castor_hal.transport import GoalResult, Transport


class FakeTransport(Transport):
    """An in-memory Transport for testing the base + the gateway bridge. Accepts
    JOINT_POSITIONS + HOME; records lifecycle + last goal; configurable reach."""

    capabilities = frozenset({GoalKind.JOINT_POSITIONS, GoalKind.HOME})
    name = "fake"
    description = "in-memory test transport"

    def __init__(self, *, reach: bool = True, fail_read: bool = False,
                 fail_open: bool = False) -> None:
        self.open_calls = 0
        self.close_calls = 0
        self.estop_calls = 0
        self.estopped = False
        self.last_goal = None
        self._reach = reach
        self._fail_read = fail_read
        self._fail_open = fail_open
        self._positions: dict[str, float] = {"j1": 0.0}

    def open(self) -> None:
        if self._fail_open:
            raise TransportError(TransportErrorCode.IO_ERROR, "cannot open link")
        self.open_calls += 1

    def close(self) -> None:
        self.close_calls += 1

    def set_goal(self, goal) -> GoalResult:
        self.ensure_supported(goal)
        self.last_goal = goal
        if isinstance(goal, JointPositions):
            self._positions = dict(goal.positions)
        return GoalResult(reached=self._reach, state=self.read_state(),
                          detail={"applied": True})

    def read_state(self) -> TransportState:
        if self._fail_read:
            raise TransportError(TransportErrorCode.IO_ERROR, "read failed")
        return TransportState(
            connected=self.open_calls > 0,
            estopped=self.estopped,
            positions=dict(self._positions),
            timestamp_s=1.0,
        )

    def estop(self) -> None:
        self.estop_calls += 1
        self.estopped = True


@pytest.fixture
def make_fake():
    """Return the FakeTransport CLASS so tests can build parameterized instances:
    ``t = make_fake(reach=False)``."""
    return FakeTransport


@pytest.fixture
def fake(make_fake):
    """A default (reaching, healthy) FakeTransport instance."""
    return make_fake()
