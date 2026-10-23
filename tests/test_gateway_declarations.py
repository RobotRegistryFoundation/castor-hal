"""What robot-md-gateway sees of a TransportActuator, through its own code.

The gateway reads `stop_capabilities` and `motion_capabilities` off each
actuator with `robot_md_gateway.cert.policy.as_tool_set` and matches tool names
exactly. With neither declared (review of robot-md-gateway#35, Q5):

- its startup audit could not see castor-hal at all;
- its scope gate did not count castor-hal motion as motion, so a read-tier
  bearer could move a transport by writing scope OBSERVE;
- from robot-md-gateway#35 on, a tripped gateway refused the transport's own
  stop and had nothing to send it on a trip.

In memory only: the real gateway app, a fake transport, a manifest signed at
test time with a throwaway key.
"""

from __future__ import annotations

import base64
import time

import pytest
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from fastapi.testclient import TestClient
from robot_md_gateway.cert.audit import AuditChain
from robot_md_gateway.cert.policy import (
    ToolAllowlist,
    as_tool_set,
    audit_allowlist_for_stop,
)
from robot_md_gateway.cert.safety import GatewayState, SafetyMonitor
from robot_md_gateway.receiver import make_app

from castor_hal.actuator import TransportActuator

KID = "castor-hal-test-manifest"
BEARERS = {"read-token": "read", "actuate-token": "actuate"}
STOPS = {"estop", "stop", "emergency_stop", "halt"}
FAKE_MOTION = {"move", "set_joint_positions", "joint_positions", "home", "go_home"}


@pytest.fixture
def signed():
    """(signing key, resolver): a throwaway Ed25519 key, and the resolver the
    gateway asks for its public half."""
    key = Ed25519PrivateKey.generate()
    pub_pem = key.public_key().public_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PublicFormat.SubjectPublicKeyInfo,
    )

    class _Resolver:
        def resolve_public_key_pem(self, kid: str) -> bytes | None:
            return pub_pem if kid == KID else None

    return key, _Resolver()


@pytest.fixture
def manifest(tmp_path, signed):
    """A minimal ROBOT.md signed the way the gateway verifies one: the body,
    then a ROBOT-MD-SIG footer."""
    key, _ = signed
    body = "---\nmetadata:\n  robot_name: castor-hal-test\n---\n\n# castor-hal test\n"
    signature = base64.b64encode(key.sign(body.encode("utf-8"))).decode("ascii")
    path = tmp_path / "ROBOT.md"
    path.write_text(f"{body}\n<!-- ROBOT-MD-SIG kid={KID} sig={signature} -->\n")
    return str(path)


def _invoke(http, manifest, tool, *, scope, token, args=None):
    return http.post(
        "/v1/invoke",
        json={"msg_id": f"m-{tool}-{time.monotonic_ns()}", "type": "rcan/v1/invoke",
              "ruri": "rcan://RRN-000000000000/robot", "scope": scope, "tool_name": tool,
              "tool_args": args or {}, "manifest_path": manifest},
        headers={"Authorization": f"Bearer {token}"},
    )


def _wait_for(predicate, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def test_the_gateway_reads_the_declarations_as_declared(fake):
    a = TransportActuator(fake)
    assert as_tool_set(getattr(a, "stop_capabilities", None)) == STOPS
    assert as_tool_set(getattr(a, "motion_capabilities", None)) == FAKE_MOTION


def test_the_gateway_startup_audit_sees_the_actuator(fake):
    """An allowlist carrying motion and no stop is the misconfiguration the
    audit exists to name. It skips an actuator that declares no motion."""
    a = TransportActuator(fake)
    [finding] = audit_allowlist_for_stop(ToolAllowlist(allowed_tools=("move",)), [("fake", a)])
    assert finding["reason"] == "allowlist_has_no_stop"
    assert finding["allowed_motion_tools"] == ["move"]
    assert finding["stop_tools_declared"] == sorted(STOPS)
    assert audit_allowlist_for_stop(
        ToolAllowlist(allowed_tools=("move", "estop")), [("fake", a)]) == []


def test_a_read_bearer_cannot_move_a_transport_by_writing_scope_observe(fake, signed, manifest):
    """No tier is bound to `move` at the gateway here, so its scope gate is what
    stands between a read bearer and the machine. It used to let this through:
    HTTP 200, and the transport moved."""
    _, resolver = signed
    app = make_app(resolver=resolver, tool_allowlist=ToolAllowlist(allowed_tools=("move", "estop")),
                   bearer_tiers=BEARERS, actuator=TransportActuator(fake))
    with TestClient(app) as http:
        response = _invoke(http, manifest, "move", scope="OBSERVE", token="read-token",
                           args={"joint_positions": {"j1": 0.7}})
    assert response.status_code == 403, response.text
    assert response.json()["detail"]["deny"] == "tool_scope"
    assert fake.last_goal is None
    assert fake.read_state().positions == {"j1": 0.0}


def _skip_without_the_safety_watchdog(resolver) -> None:
    """robot-md-gateway#35 added the watchdog that sends each actuator its
    declared stop on a trip, and lets declared stops through a tripped
    monitor, with /v1/safety/heartbeat. Earlier gateways have neither."""
    routes = {getattr(route, "path", None) for route in make_app(resolver=resolver).routes}
    if "/v1/safety/heartbeat" not in routes:
        pytest.skip("this robot-md-gateway does not stop its actuators on a trip "
                    "(no /v1/safety/heartbeat; robot-md-gateway#35)")


def test_a_trip_stops_the_transport_and_its_stop_passes_the_tripped_gateway(
        fake, signed, manifest):
    _, resolver = signed
    _skip_without_the_safety_watchdog(resolver)
    chain = AuditChain()
    monitor = SafetyMonitor(state=GatewayState.READY, heartbeat_staleness_s=0.3)
    app = make_app(resolver=resolver, tool_allowlist=ToolAllowlist(allowed_tools=("move", "estop")),
                   bearer_tiers=BEARERS, safety_monitor=monitor, audit_chain=chain,
                   actuator=TransportActuator(fake), actuator_config={})

    with TestClient(app) as http:
        # No heartbeat ever arrives: the gateway trips and stops the transport
        # itself, with the first declared stop, sorted.
        assert _wait_for(lambda: fake.estop_calls == 1), "the gateway sent no stop"
        assert monitor.state is GatewayState.SAFE_STOP
        assert _wait_for(lambda: [e for e in chain.entries
                                  if e.decision_reason.startswith("safety_stop:")])
        [stop] = [e for e in chain.entries if e.decision_reason.startswith("safety_stop:")]
        assert "sent emergency_stop to fake" in stop.decision_reason
        assert stop.actuator_outcome_kind == "executed"

        # An operator's stop gets through the tripped gateway; motion does not.
        operator = _invoke(http, manifest, "estop", scope="SAFETY", token="read-token")
        assert operator.status_code == 200, operator.text
        assert fake.estop_calls == 2
        refused = _invoke(http, manifest, "move", scope="MANIPULATE", token="actuate-token",
                          args={"joint_positions": {"j1": 0.7}})
        assert refused.status_code == 403
        assert refused.json()["detail"]["deny"] == "safety_state"
    assert fake.last_goal is None
