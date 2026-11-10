"""ROS 2 node `uwb_relay`: runs UwbRelay and publishes the UWB topics.

Topics (same names as Software/UWB_UART_GUI/ros2/uwb_ros_bridge.py):
  /uwb/anchor_<N>/range             sensor_msgs/Range, calibrated+valid DS only
  /uwb/anchor_<N>/diagnostic_range  sensor_msgs/Range, every DS range
  /uwb/burst_metadata               std_msgs/String JSON per Burst
  /uwb/cycle_seq, /uwb/tag_time_us  TAG cycle identity and clock
  /uwb/device_info                  std_msgs/String JSON
  /uwb_relay/ground_address         std_msgs/String (latched) for px4_logger
  /uwb_relay/status                 std_msgs/String JSON, 1 Hz
Header stamps are the UP4000 receive time (CLOCK_REALTIME).
"""
from __future__ import annotations

import json
import signal
import threading

import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String, UInt32, UInt64

from .session import AlreadyRunning, bind_or_explain, single_instance, TAG_PORT_PATTERNS, find_port, session_dir, write_json
from .uwb_relay import UwbRelay
from .vendor import telemetry_protocol as tp
from .vendor.burst_metadata import BurstMetadata


def stamp_from_ns(realtime_ns: int) -> Time:
    return Time(sec=realtime_ns // 1_000_000_000, nanosec=realtime_ns % 1_000_000_000)


class UwbRelayNode(Node):
    def __init__(self) -> None:
        self._instance_lock = single_instance("uwb_relay")
        super().__init__("uwb_relay")
        p = self.declare_parameter
        log_root = p("log_root", "~/flight_logs").value
        session = p("session", "").value
        tag_port = p("tag_port", "auto").value
        baud = p("baud", 460800).value
        ground_host = p("ground_host", "").value
        ground_port = p("ground_port", 15555).value
        local_port = p("local_port", 15556).value
        batch_ms = p("batch_ms", 10.0).value
        beacon_host = p("beacon_host", "255.255.255.255").value
        self.publish_ros = p("publish_ros", True).value
        # JSON per Burst (~320/s) costs CPU on the UP4000; only for tools that need it.
        self.publish_metadata = p("publish_burst_metadata", False).value
        # rclpy publishing of every anchor at ~320 Burst/s saturates a UP4000 core
        # and starves the PX4 logger (07/11). Log + WiFi keep the full rate.
        self.publish_hz = float(p("ros_publish_hz", 50.0).value)
        self._min_period_ns = int(1e9 / self.publish_hz) if self.publish_hz > 0 else 0
        self._last_burst_pub_ns = 0

        out = session_dir(log_root, session)
        write_json(out / "uwb_config.json", dict(
            log_root=log_root, session=session, tag_port=tag_port, baud=baud, ground_host=ground_host,
            ground_port=ground_port, local_port=local_port, batch_ms=batch_ms, beacon_host=beacon_host,
            publish_ros=self.publish_ros, publish_burst_metadata=self.publish_metadata,
            ros_publish_hz=self.publish_hz))
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._qos = qos
        self._range_pubs: dict[str, object] = {}
        self._metadata = self.create_publisher(String, "/uwb/burst_metadata", qos)
        self._cycle = self.create_publisher(UInt32, "/uwb/cycle_seq", qos)
        self._tag_time = self.create_publisher(UInt64, "/uwb/tag_time_us", qos)
        self._device = self.create_publisher(String, "/uwb/device_info", latched)
        self._ground = self.create_publisher(String, "/uwb_relay/ground_address", latched)
        self._status = self.create_publisher(String, "/uwb_relay/status", qos)
        self._builder = BurstMetadata()

        self.relay = UwbRelay(
            port_resolver=lambda: find_port(tag_port, TAG_PORT_PATTERNS),
            out_dir=out, baud=baud, ground_host=ground_host, ground_port=ground_port,
            local_port=local_port, batch_s=batch_ms / 1000.0, beacon_host=beacon_host,
            on_message=self._on_message if self.publish_ros else None,
            on_ground=lambda host: self._ground.publish(String(data=host)),
            log=self.get_logger().info,
        )
        if ground_host:
            self._ground.publish(String(data=ground_host))
        bind_or_explain(self.relay.open_socket, local_port, "uwb_relay")
        self.get_logger().info(f"UWB session folder: {out}")
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self.relay.run, args=(self._stop,), name="uwb-relay", daemon=True)
        self._thread.start()
        self.create_timer(1.0, self._publish_status)

    def _range(self, topic: str, anchor: int, range_mm: int, stamp: Time) -> None:
        pub = self._range_pubs.get(topic)
        if pub is None:
            pub = self._range_pubs[topic] = self.create_publisher(Range, topic, self._qos)
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = f"uwb_anchor_{anchor}"
        msg.field_of_view = 0.0
        msg.min_range = 0.0
        msg.max_range = 65.534
        msg.range = range_mm / 1000.0
        pub.publish(msg)

    def _on_message(self, msg: object, realtime_ns: int, _monotonic_ns: int) -> None:
        self._builder.accept(msg)
        stamp = stamp_from_ns(realtime_ns)
        if isinstance(msg, tp.DeviceInfoMessage):
            self._device.publish(String(data=json.dumps({
                "boot_id": msg.boot_id, "config_hash": hex(msg.build_config_hash),
                "features": msg.telemetry_features, "calibrated_mask": msg.calibrated_mask,
                "uart_baud": msg.uart_baud})))
        elif isinstance(msg, tp.RangeBurstMessage):
            if realtime_ns - self._last_burst_pub_ns < self._min_period_ns:
                return  # rate limit; uwb_frames.bin and the WiFi stream keep every Burst
            self._last_burst_pub_ns = realtime_ns
            self._cycle.publish(UInt32(data=msg.cycle_seq))
            self._tag_time.publish(UInt64(data=msg.t_us))
            for a in msg.anchors:
                if not a.radio_ok or a.mode_name != "DS" or a.range_mm is None:
                    continue
                self._range(f"/uwb/anchor_{a.anchor_id}/diagnostic_range", a.anchor_id, a.range_mm, stamp)
                if a.flags & tp.BURST_FLAG_CAL_OK and a.flags & tp.BURST_FLAG_VALID:
                    self._range(f"/uwb/anchor_{a.anchor_id}/range", a.anchor_id, a.range_mm, stamp)
            if self.publish_metadata:
                self._metadata.publish(String(data=json.dumps(
                    self._builder.build(msg), separators=(",", ":"), allow_nan=False)))

    def _publish_status(self) -> None:
        self._status.publish(String(data=json.dumps(self.relay.status(), default=str)))

    def shutdown(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = UwbRelayNode()
    except AlreadyRunning as exc:
        rclpy.logging.get_logger("uwb_relay").error(str(exc))
        rclpy.try_shutdown()
        raise SystemExit(1) from None
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch and systemd/timeout can each deliver SIGINT; a second
        # one must not interrupt closing the log files.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
