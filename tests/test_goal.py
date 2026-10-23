"""parse_goal — the one canonical envelope→Goal mapping every adapter shares."""

import pytest

from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import (
    GOAL_TOOL_NAMES,
    BodyTwist,
    FlightSetpoint,
    GoalKind,
    Gripper,
    Home,
    JointPositions,
    JointTrajectory,
    RateSetpoint,
    SectionState,
    goal_tool_names,
    parse_goal,
)


def test_move_to_joint_positions():
    g = parse_goal("move", {"joint_positions": {"elbow_flex": 0.5, "wrist_roll": -1.0}})
    assert isinstance(g, JointPositions)
    assert g.kind is GoalKind.JOINT_POSITIONS
    assert g.positions == {"elbow_flex": 0.5, "wrist_roll": -1.0}
    assert g.unit == "rad"


def test_move_accepts_positions_alias_and_is_case_insensitive():
    g = parse_goal("SET_JOINT_POSITIONS", {"positions": {"j1": 1}})
    assert isinstance(g, JointPositions)
    assert g.positions == {"j1": 1.0}  # coerced to float


def test_home():
    assert isinstance(parse_goal("home", {}), Home)
    assert isinstance(parse_goal("go_home", None), Home)


def test_body_twist():
    g = parse_goal("cmd_vel", {"linear": [1, 0, 0], "angular": [0, 0, 0.5]})
    assert isinstance(g, BodyTwist)
    assert g.linear == (1.0, 0.0, 0.0)
    assert g.angular == (0.0, 0.0, 0.5)


def test_body_twist_defaults_to_zero():
    g = parse_goal("drive", {})
    assert g.linear == (0.0, 0.0, 0.0) and g.angular == (0.0, 0.0, 0.0)


def test_gripper():
    g = parse_goal("set_gripper", {"position": 0.8, "effort": 2.0})
    assert isinstance(g, Gripper) and g.position == 0.8 and g.effort == 2.0


def test_rate_setpoint():
    g = parse_goal("set_rate", {"rate": 120, "unit": "kg/ha", "product": "seed"})
    assert isinstance(g, RateSetpoint)
    assert g.rate == 120.0 and g.unit == "kg/ha" and g.product == "seed"


def test_section_state_from_object_and_array():
    g1 = parse_goal("set_sections", {"sections": {"1": True, "2": False}})
    assert isinstance(g1, SectionState) and g1.sections == {1: True, 2: False}
    g2 = parse_goal("section_state", {"sections": [True, False, True]})
    assert g2.sections == {1: True, 2: False, 3: True}  # 1-based


def test_flight_setpoint():
    g = parse_goal("goto", {"north_m": 10, "east_m": -5, "down_m": -2, "yaw_rad": 1.57})
    assert isinstance(g, FlightSetpoint)
    assert (g.north_m, g.east_m, g.down_m, g.yaw_rad) == (10.0, -5.0, -2.0, 1.57)


def test_joint_trajectory():
    g = parse_goal("set_trajectory", {"points": [
        {"positions": {"j1": 0.0}, "time_from_start_s": 0.0},
        {"positions": {"j1": 1.0}, "time_from_start_s": 2.0},
    ]})
    assert isinstance(g, JointTrajectory)
    assert len(g.points) == 2 and g.points[1].time_from_start_s == 2.0


def test_unknown_tool_is_unsupported_goal():
    with pytest.raises(TransportError) as ei:
        parse_goal("frobnicate", {})
    assert ei.value.code is TransportErrorCode.UNSUPPORTED_GOAL
    assert ei.value.detail["tool"] == "frobnicate"


def test_move_without_positions_is_invalid_goal():
    with pytest.raises(TransportError) as ei:
        parse_goal("move", {})
    assert ei.value.code is TransportErrorCode.INVALID_GOAL


def test_rate_missing_unit_is_invalid_goal():
    with pytest.raises(TransportError) as ei:
        parse_goal("set_rate", {"rate": 1})
    assert ei.value.code is TransportErrorCode.INVALID_GOAL
    assert ei.value.detail["missing"] == "unit"


def test_goals_are_frozen():
    g = parse_goal("move", {"joint_positions": {"j1": 0.0}})
    with pytest.raises(Exception):
        g.positions = {}  # type: ignore[misc]  # frozen dataclass


# ── malformed input surfaces as INVALID_GOAL, never a raw exception (review #1) ──
@pytest.mark.parametrize("tool,args", [
    ("move", {"joint_positions": {"j1": None}}),       # None → not numeric
    ("move", {"joint_positions": {"j1": "abc"}}),      # non-numeric string
    ("move", {"joint_positions": {"j1": float("inf")}}),  # non-finite
    ("move", {"joint_positions": {"j1": float("nan")}}),  # nan
    ("set_gripper", {"position": None}),
    ("set_rate", {"rate": "x", "unit": "kg/ha"}),
    ("cmd_vel", {"linear": [1, 0]}),                    # wrong-length vector
    ("cmd_vel", {"linear": [1, "y", 0]}),              # non-numeric element
    ("cmd_vel", {"angular": [0, 0, float("inf")]}),    # non-finite element
    ("goto", {"north_m": "n", "east_m": 0, "down_m": 0}),
    ("set_sections", {"sections": {"notanint": True}}),  # bad key
    ("set_trajectory", {"points": [{"time_from_start_s": 0.0}]}),  # missing positions
    ("set_trajectory", {"points": ["nope"]}),          # point not an object
])
def test_malformed_numeric_or_key_input_is_invalid_goal(tool, args):
    with pytest.raises(TransportError) as ei:
        parse_goal(tool, args)
    assert ei.value.code is TransportErrorCode.INVALID_GOAL, f"{tool} {args} → {ei.value.code}"


def test_int_strings_coerce_fine():
    # well-formed numeric strings are still accepted (coercion, not rejection)
    g = parse_goal("move", {"joint_positions": {"j1": "1.5"}})
    assert g.positions["j1"] == 1.5


# -- GOAL_TOOL_NAMES: the one table of goal tool names -----------------------
#
# parse_goal reads it, and so does TransportActuator.motion_capabilities, the
# set robot-md-gateway treats as motion. A name the parser accepts and the
# declaration leaves out would move the machine unseen by the gateway's gates.

#: The smallest valid arguments for each goal kind.
_VALID_ARGS = {
    GoalKind.JOINT_POSITIONS: {"joint_positions": {"j1": 0.0}},
    GoalKind.HOME: {},
    GoalKind.BODY_TWIST: {},
    GoalKind.GRIPPER: {"position": 0.5},
    GoalKind.RATE_SETPOINT: {"rate": 1.0, "unit": "kg/ha"},
    GoalKind.SECTION_STATE: {"sections": [True]},
    GoalKind.FLIGHT_SETPOINT: {"north_m": 0.0, "east_m": 0.0, "down_m": 0.0},
    GoalKind.JOINT_TRAJECTORY: {"points": [{"positions": {"j1": 0.0}}]},
}


def test_every_goal_kind_has_tool_names_and_no_name_has_two_kinds():
    assert set(GOAL_TOOL_NAMES) == set(GoalKind) == set(_VALID_ARGS)
    names = [name for tools in GOAL_TOOL_NAMES.values() for name in tools]
    assert names and len(names) == len(set(names))
    # Stored as parse_goal compares them, so a declared spelling is one it accepts.
    assert all(name == name.strip().lower() for name in names)


@pytest.mark.parametrize("kind,tool", sorted(
    ((kind, tool) for kind, tools in GOAL_TOOL_NAMES.items() for tool in tools),
    key=lambda pair: (pair[0].value, pair[1])))
def test_every_tool_name_in_the_table_parses_to_its_kind(kind, tool):
    assert parse_goal(tool, _VALID_ARGS[kind]).kind is kind


def test_goal_tool_names_is_the_union_for_the_kinds_asked():
    assert goal_tool_names([GoalKind.HOME]) == {"home", "go_home"}
    assert goal_tool_names(["home"]) == {"home", "go_home"}      # a kind's value works too
    assert goal_tool_names(frozenset()) == frozenset()
    assert goal_tool_names(GoalKind) == frozenset().union(*GOAL_TOOL_NAMES.values())
    with pytest.raises(ValueError):
        goal_tool_names(["teleport"])
