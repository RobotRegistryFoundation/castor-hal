"""ROS 2 Transport adapter for castor-hal (Humble / Jazzy / rclpy).

Bridges normalized castor-hal Goals (``BodyTwist``, ``JointPositions``, ``Home``)
to standard ROS 2 topics (``geometry_msgs/Twist`` on ``/cmd_vel``,
``sensor_msgs/JointState`` on ``/joint_states`` & ``/joint_commands``) and
exposes DDS topic heartbeat health for ``robot_doctor`` fault isolation.

Core remains dependency-free: ``rclpy`` is imported lazily only when no custom
``backend`` is injected. Tests and simulation harnesses inject any object
implementing ``ROS2Bridge``.

SAFETY (binding): ``estop()`` publishes a zero ``Twist`` on ``cmd_vel_topic``
and latches a software stop in this transport. It is a BEST-EFFORT software
stop, NOT a safety-rated hardware e-stop (ISO 13850).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import BodyTwist, GoalKind, Home, JointPositions
from castor_hal.state import TransportState
from castor_hal.transport import GoalResult, Transport


@runtime_checkable
class ROS2Bridge(Protocol):
    """Minimal ROS 2 I/O seam (implemented by RclpyBridge or a test fake)."""

    def publish_twist(
        self,
        topic: str,
        linear: tuple[float, float, float],
        angular: tuple[float, float, float],
    ) -> None: ...

    def publish_joint_positions(self, topic: str, positions: dict[str, float]) -> None: ...

    def read_joint_states(self, topic: str) -> tuple[dict[str, float], dict[str, float]]: ...

    def read_odom_twist(self, topic: str) -> dict[str, float]: ...

    def topic_heartbeat_age_s(self, topic: str) -> float | None: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class ROS2Config:
    """Configuration and velocity/joint bounds for ``ROS2Transport``."""

    node_name: str = "castor_hal_bridge"
    cmd_vel_topic: str = "/cmd_vel"
    joint_cmd_topic: str = "/joint_commands"
    joint_state_topic: str = "/joint_states"
    odom_topic: str = "/odom"
    max_linear_mps: float = 1.0
    max_angular_rps: float = 1.5
    stale_topic_s: float = 2.0
    home_positions: dict[str, float] | None = None
    read_only: bool = False


class RclpyBridge:
    """Live ROS 2 bridge backed by ``rclpy`` (lazy-imported on construction)."""

    def __init__(self, cfg: ROS2Config) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import JointState
        except ImportError as exc:
            raise TransportError(
                TransportErrorCode.IO_ERROR,
                "rclpy / ROS 2 message packages are not installed in this environment; "
                "source your ROS 2 setup.bash or inject a ROS2Bridge backend",
                detail={"missing": str(exc)},
            ) from exc

        self._rclpy = rclpy
        self._Twist = Twist
        self._JointState = JointState
        self._read_only = cfg.read_only
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(cfg.node_name)
        if cfg.read_only:
            self._vel_pub = None
            self._joint_pub = None
        else:
            self._vel_pub = self._node.create_publisher(Twist, cfg.cmd_vel_topic, 10)
            self._joint_pub = self._node.create_publisher(JointState, cfg.joint_cmd_topic, 10)
        self._last_joints: tuple[dict[str, float], dict[str, float]] = ({}, {})
        self._last_odom: dict[str, float] = {}
        self._last_rx: dict[str, float] = {}

        def _on_js(msg) -> None:
            pos = {n: float(p) for n, p in zip(msg.name, msg.position)}
            vel = {n: float(v) for n, v in zip(msg.name, msg.velocity)} if msg.velocity else {}
            self._last_joints = (pos, vel)
            self._last_rx[cfg.joint_state_topic] = time.monotonic()

        def _on_odom(msg) -> None:
            t = msg.twist.twist
            self._last_odom = {
                "vx_mps": float(t.linear.x),
                "vy_mps": float(t.linear.y),
                "wz_rps": float(t.angular.z),
            }
            self._last_rx[cfg.odom_topic] = time.monotonic()

        self._node.create_subscription(JointState, cfg.joint_state_topic, _on_js, 10)
        self._node.create_subscription(Odometry, cfg.odom_topic, _on_odom, 10)

    def _spin_once(self) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=0.01)

    def publish_twist(self, topic: str, linear: tuple[float, float, float], angular: tuple[float, float, float]) -> None:
        if self._vel_pub is None:
            raise TransportError(TransportErrorCode.UNSUPPORTED_GOAL, "read-only ROS2 bridge cannot publish twist")
        msg = self._Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = linear
        msg.angular.x, msg.angular.y, msg.angular.z = angular
        self._vel_pub.publish(msg)

    def publish_joint_positions(self, topic: str, positions: dict[str, float]) -> None:
        if self._joint_pub is None:
            raise TransportError(TransportErrorCode.UNSUPPORTED_GOAL, "read-only ROS2 bridge cannot publish joint positions")
        msg = self._JointState()
        msg.name = list(positions.keys())
        msg.position = [float(v) for v in positions.values()]
        self._joint_pub.publish(msg)

    def read_joint_states(self, topic: str) -> tuple[dict[str, float], dict[str, float]]:
        self._spin_once()
        return self._last_joints

    def read_odom_twist(self, topic: str) -> dict[str, float]:
        self._spin_once()
        return dict(self._last_odom)

    def topic_heartbeat_age_s(self, topic: str) -> float | None:
        ts = self._last_rx.get(topic)
        return None if ts is None else max(0.0, time.monotonic() - ts)

    def close(self) -> None:
        self._node.destroy_node()


class ROS2Transport(Transport):
    """Bounded ROS 2 transport with velocity limits, DDS health, and latching software estop."""

    name = "ros2"
    description = "ROS 2 (rclpy) bounded cmd_vel / joint_states transport"
    capabilities: frozenset[GoalKind] = frozenset({
        GoalKind.BODY_TWIST,
        GoalKind.JOINT_POSITIONS,
        GoalKind.HOME,
    })
    can_estop: bool = True

    def __init__(
        self,
        config: ROS2Config | None = None,
        *,
        backend: ROS2Bridge | None = None,
    ) -> None:
        self._cfg = config or ROS2Config()
        self._injected_backend = backend
        self._backend: ROS2Bridge | None = None
        self._open = False
        self._estopped = False
        if self._cfg.read_only:
            self.capabilities = frozenset()
            self.can_estop = False
        else:
            self.capabilities = type(self).capabilities
            self.can_estop = type(self).can_estop

    def open(self) -> None:
        if self._open:
            return
        self._backend = self._injected_backend if self._injected_backend is not None else RclpyBridge(self._cfg)
        self._open = True

    def close(self) -> None:
        if not self._open:
            return
        if self._backend is not None and self._injected_backend is None:
            self._backend.close()
        self._backend = None
        self._open = False

    def _require_open(self) -> ROS2Bridge:
        if not self._open or self._backend is None:
            raise TransportError(TransportErrorCode.NOT_CONNECTED, "ROS2Transport is not open")
        return self._backend

    def clear_estop(self) -> None:
        """Clear the software estop latch (operator / commission tier)."""
        self._estopped = False

    def estop(self) -> None:
        """Best-effort software stop: publish zero twist and latch estop.

        A read-only ROS 2 tap refuses estop honestly without writing to /cmd_vel,
        matching J1939ReadTransport (not a hardware e-stop, ISO 13850).
        """
        if self._cfg.read_only:
            raise TransportError(
                TransportErrorCode.UNSUPPORTED_GOAL,
                "read-only ROS2Transport cannot command a stop; "
                "wire a hardware e-stop (this transport never publishes to /cmd_vel)",
            )
        self._estopped = True
        if self._open and self._backend is not None:
            try:
                self._backend.publish_twist(self._cfg.cmd_vel_topic, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            except Exception as exc:
                raise TransportError(
                    TransportErrorCode.IO_ERROR,
                    f"failed to publish zero twist during estop: {exc}",
                ) from exc

    def set_goal(self, goal) -> GoalResult:
        self.ensure_supported(goal)
        backend = self._require_open()
        if self._estopped:
            raise TransportError(
                TransportErrorCode.ESTOPPED,
                "ROS2Transport is e-stopped; call clear_estop() before commanding motion",
            )

        if isinstance(goal, BodyTwist):
            lin_mag = math.sqrt(sum(v * v for v in goal.linear))
            ang_mag = math.sqrt(sum(v * v for v in goal.angular))
            if lin_mag > self._cfg.max_linear_mps:
                raise TransportError(
                    TransportErrorCode.OUT_OF_RANGE,
                    f"linear speed {lin_mag:.3f} m/s exceeds max_linear_mps={self._cfg.max_linear_mps}",
                    detail={"linear_mps": lin_mag, "limit": self._cfg.max_linear_mps},
                )
            if ang_mag > self._cfg.max_angular_rps:
                raise TransportError(
                    TransportErrorCode.OUT_OF_RANGE,
                    f"angular speed {ang_mag:.3f} rad/s exceeds max_angular_rps={self._cfg.max_angular_rps}",
                    detail={"angular_rps": ang_mag, "limit": self._cfg.max_angular_rps},
                )
            try:
                backend.publish_twist(self._cfg.cmd_vel_topic, goal.linear, goal.angular)
            except Exception as exc:
                raise TransportError(TransportErrorCode.IO_ERROR, f"publish_twist failed: {exc}") from exc
            return GoalResult(
                reached=True,
                state=self.read_state(),
                detail={"topic": self._cfg.cmd_vel_topic, "linear": goal.linear, "angular": goal.angular},
            )

        if isinstance(goal, JointPositions):
            try:
                backend.publish_joint_positions(self._cfg.joint_cmd_topic, goal.positions)
            except Exception as exc:
                raise TransportError(TransportErrorCode.IO_ERROR, f"publish_joint_positions failed: {exc}") from exc
            return GoalResult(
                reached=True,
                state=self.read_state(),
                detail={"topic": self._cfg.joint_cmd_topic, "joints": sorted(goal.positions)},
            )

        if isinstance(goal, Home):
            home = self._cfg.home_positions
            if not home:
                # Mobile-only ROS 2 base: home commands zero twist
                try:
                    backend.publish_twist(self._cfg.cmd_vel_topic, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                except Exception as exc:
                    raise TransportError(TransportErrorCode.IO_ERROR, f"publish_twist failed: {exc}") from exc
                return GoalResult(reached=True, state=self.read_state(), detail={"action": "zero_twist"})
            try:
                backend.publish_joint_positions(self._cfg.joint_cmd_topic, home)
            except Exception as exc:
                raise TransportError(TransportErrorCode.IO_ERROR, f"publish_joint_positions failed: {exc}") from exc
            return GoalResult(reached=True, state=self.read_state(), detail={"topic": self._cfg.joint_cmd_topic, "home": True})

        raise TransportError(TransportErrorCode.UNSUPPORTED_GOAL, f"unhandled goal {goal!r}")

    def read_state(self) -> TransportState:
        backend = self._require_open()
        try:
            positions, velocities = backend.read_joint_states(self._cfg.joint_state_topic)
            odom = backend.read_odom_twist(self._cfg.odom_topic)
            js_age = backend.topic_heartbeat_age_s(self._cfg.joint_state_topic)
            odom_age = backend.topic_heartbeat_age_s(self._cfg.odom_topic)
        except Exception as exc:
            raise TransportError(TransportErrorCode.IO_ERROR, f"ROS 2 read_state failed: {exc}") from exc

        ages = [a for a in (js_age, odom_age) if a is not None]
        stale = bool(ages) and all(a > self._cfg.stale_topic_s for a in ages)
        extra: dict = {
            "node_name": self._cfg.node_name,
            "dds_healthy": not stale,
            "joint_state_age_s": js_age,
            "odom_age_s": odom_age,
        }
        if stale:
            extra["fault_domain"] = "ros2_dds_stale"

        return TransportState(
            connected=self._open and not stale,
            estopped=self._estopped,
            positions=positions or None,
            velocities=velocities or None,
            measurements=odom or None,
            timestamp_s=time.time(),
            extra=extra,
        )
