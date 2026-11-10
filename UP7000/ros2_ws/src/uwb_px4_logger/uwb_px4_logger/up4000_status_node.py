"""ROS 2 node `up4000_status`: send UP4000 + logger + PX4 status to the ground.

Collects /uwb_relay/status, /px4/status and /px4/snapshot (JSON strings from
the other two nodes) plus host health, and sends one JSON datagram
`status_hz` times per second to <ground>:15557, where New_GUI's "UP4000" tab
shows it. The ground address follows /uwb_relay/ground_address unless
`ground_host` is set. Read only: nothing is sent back to the drone.
"""
from __future__ import annotations

import json
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .session import AlreadyRunning, bind_or_explain, single_instance, session_dir
from .status_reporter import HostHealth, StatusSender, session_files


class Up4000StatusNode(Node):
    def __init__(self) -> None:
        self._instance_lock = single_instance("up4000_status")
        super().__init__("up4000_status")
        p = self.declare_parameter
        log_root = p("log_root", "~/flight_logs").value
        session = p("session", "").value
        self.ground_host = p("ground_host", "").value
        port = p("status_port", 15557).value
        status_hz = float(p("status_hz", 4.0).value)
        health_hz = float(p("health_hz", 1.0).value)
        self.broadcast = p("broadcast_host", "255.255.255.255").value  # "" disables
        self.slow_every = max(1, round(status_hz / max(float(p("slow_hz", 1.0).value), 0.01)))
        self.part_gap_s = max(0.0, float(p("part_gap_ms", 2.0).value)) / 1000.0
        self._tick = 0

        self.folder = session_dir(log_root, session)
        self.health = HostHealth(log_dir=str(self.folder))
        self.sender = StatusSender(port)
        self._host: dict = {}
        self._parts: dict[str, dict | None] = {"uwb_relay": None, "px4_logger": None, "px4": None}
        self._learned_ground = ""

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/uwb_relay/ground_address", self._on_ground, latched)
        for topic, key in (("/uwb_relay/status", "uwb_relay"), ("/px4/status", "px4_logger"),
                           ("/px4/snapshot", "px4")):
            self.create_subscription(String, topic, lambda m, k=key: self._on_part(k, m), qos)
        self._sample_health()
        self.create_timer(1.0 / health_hz, self._sample_health)
        self.create_timer(1.0 / status_hz, self._send)
        self.get_logger().info(f"UP4000 status -> ground UDP :{port} at {status_hz:g} Hz "
                               f"({self.ground_host or 'ground address from uwb_relay'})")

    def _on_ground(self, msg: String) -> None:
        self._learned_ground = msg.data

    def _on_part(self, key: str, msg: String) -> None:
        try:
            self._parts[key] = json.loads(msg.data)
        except ValueError:
            pass

    def _sample_health(self) -> None:
        self._host = self.health.sample()

    def _send(self) -> None:
        """PX4 values every tick; slower-changing sections once per second.
        A broadcast (no ground known yet) always carries everything."""
        ground = self.ground_host or self._learned_ground
        target = self.sender.target(ground, broadcast=self.broadcast)
        if target is None:
            return
        self._tick += 1
        px4 = self._parts["px4"]
        if isinstance(px4, dict):
            px4 = dict(px4, message_types=sorted(px4.get("messages") or {}))
        sections = {"px4": px4}
        if not ground or self._tick % self.slow_every == 1 or self.slow_every == 1:
            sections.update(host=self._host, session={"folder": str(self.folder),
                                                      "files": session_files(self.folder)},
                            uwb_relay=self._parts["uwb_relay"], px4_logger=self._parts["px4_logger"])
        self.sender.send(sections, target, part_gap_s=self.part_gap_s)

    def shutdown(self) -> None:
        self.sender.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = Up4000StatusNode()
    except AlreadyRunning as exc:
        rclpy.logging.get_logger("up4000_status").error(str(exc))
        rclpy.try_shutdown()
        raise SystemExit(1) from None
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
