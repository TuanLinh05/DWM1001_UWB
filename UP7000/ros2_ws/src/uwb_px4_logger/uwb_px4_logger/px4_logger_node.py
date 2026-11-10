"""ROS 2 node `px4_logger`: runs Px4Link (tlog + ULog stream + timesync + QGC forward).

Topics:
  /px4/odometry       nav_msgs/Odometry from MAVLink ODOMETRY (PX4 EKF2 output,
                      local NED/FRD frames as sent by PX4; no ENU conversion)
  /px4/status         std_msgs/String JSON, 1 Hz (link, ULog stream, counters)
Subscribes /uwb_relay/ground_address to forward MAVLink to the discovered
ground station unless `ground_host` is set.
"""
from __future__ import annotations

import json
import signal
import math
import threading

import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt64

from .px4_link import DEFAULT_RATES, Px4Link, parse_rates
from .imu_bridge import ImuSampleClock, highres_imu_sample
from .session import AlreadyRunning, bind_or_explain, single_instance, PX4_PORT_PATTERNS, find_port, session_dir, write_json


def stamp_from_ns(realtime_ns: int) -> Time:
    return Time(sec=realtime_ns // 1_000_000_000, nanosec=realtime_ns % 1_000_000_000)


class Px4LoggerNode(Node):
    def __init__(self) -> None:
        self._instance_lock = single_instance("px4_logger")
        super().__init__("px4_logger")
        p = self.declare_parameter
        config = dict(
            log_root=p("log_root", "~/flight_logs").value,
            session=p("session", "").value,
            px4_port=p("px4_port", "auto").value,
            px4_baud=p("px4_baud", 921600).value,
            ulog_stream=p("ulog_stream", True).value,
            message_rates=list(p("message_rates", list(DEFAULT_RATES)).value),
            timesync_hz=p("timesync_hz", 2.0).value,
            forward_mavlink=p("forward_mavlink", True).value,
            ground_host=p("ground_host", "").value,
            ground_mavlink_port=p("ground_mavlink_port", 14550).value,
            local_mavlink_port=p("local_mavlink_port", 14555).value,
            publish_ros=p("publish_ros", True).value,
            snapshot_hz=p("snapshot_hz", 4.0).value,
        )
        out = session_dir(config["log_root"], config["session"])
        write_json(out / "px4_config.json", config)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._odom = self.create_publisher(Odometry, "/px4/odometry", qos)
        self._status = self.create_publisher(String, "/px4/status", qos)
        self._snapshot = self.create_publisher(String, "/px4/snapshot", qos)
        self._imu = self.create_publisher(Imu, "/px4/imu_frd", qos)
        self._imu_device_time = self.create_publisher(UInt64, "/px4/imu_device_time_us", qos)
        self._imu_clock = ImuSampleClock()
        self._imu_samples = self._imu_rejected = 0

        self.link = Px4Link(
            device_resolver=lambda: find_port(config["px4_port"], PX4_PORT_PATTERNS),
            out_dir=out, baud=config["px4_baud"], ulog_stream=config["ulog_stream"],
            message_rates=parse_rates(config["message_rates"]), timesync_hz=config["timesync_hz"],
            forward_mavlink=config["forward_mavlink"], ground_host=config["ground_host"],
            ground_port=config["ground_mavlink_port"], local_port=config["local_mavlink_port"],
            on_message=self._on_message if config["publish_ros"] else None,
            log=self.get_logger().info,
        )
        self.create_subscription(String, "/uwb_relay/ground_address",
                                 lambda m: self.link.set_ground_host(m.data), latched)
        bind_or_explain(self.link.open_socket, config["local_mavlink_port"], "px4_logger")
        self.get_logger().info(f"PX4 session folder: {out}")
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self.link.run, args=(self._stop,), name="px4-link", daemon=True)
        self._thread.start()
        self.create_timer(1.0, self._publish_status)
        if config["snapshot_hz"] > 0:  # latest PX4 values for the ground UP4000 tab
            self.create_timer(1.0 / config["snapshot_hz"], self._publish_snapshot)

    def _on_message(self, msg, realtime_ns: int, _monotonic_ns: int) -> None:
        if msg.get_type() == "HIGHRES_IMU":
            sample = highres_imu_sample(msg, realtime_ns, _monotonic_ns, self._imu_clock)
            if sample is None:
                self._imu_rejected += 1
                return
            stamp, acceleration, angular_velocity = sample
            imu = Imu()
            imu.header.stamp = stamp_from_ns(stamp)
            imu.header.frame_id = "px4_imu_frd"
            imu.orientation_covariance[0] = -1.0  # orientation is not measured here
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acceleration
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = angular_velocity
            self._imu.publish(imu)
            self._imu_device_time.publish(UInt64(data=int(msg.time_usec)))
            self._imu_samples += 1
            return
        if msg.get_type() != "ODOMETRY":
            return
        odom = Odometry()
        odom.header.stamp = stamp_from_ns(realtime_ns)
        odom.header.frame_id = f"px4_frame_{msg.frame_id}"
        odom.child_frame_id = f"px4_frame_{msg.child_frame_id}"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = msg.x, msg.y, msg.z
        w, x, y, z = msg.q  # MAVLink quaternion order is w, x, y, z
        o = odom.pose.pose.orientation
        o.w, o.x, o.y, o.z = w, x, y, z
        t = odom.twist.twist
        t.linear.x, t.linear.y, t.linear.z = msg.vx, msg.vy, msg.vz
        t.angular.x, t.angular.y, t.angular.z = msg.rollspeed, msg.pitchspeed, msg.yawspeed
        # MAVLink sends the upper triangle of the 6x6 covariance (21 values).
        for cov, dst in ((msg.pose_covariance, odom.pose.covariance), (msg.velocity_covariance, odom.twist.covariance)):
            k = 0
            for r in range(6):
                for c in range(r, 6):
                    v = float(cov[k]) if not math.isnan(cov[k]) else 0.0
                    dst[r * 6 + c] = dst[c * 6 + r] = v
                    k += 1
        self._odom.publish(odom)

    def _publish_snapshot(self) -> None:
        self._snapshot.publish(String(data=json.dumps(self.link.snapshot(), separators=(",", ":"), default=str)))

    def _publish_status(self) -> None:
        status = self.link.status()
        status["imu_bridge"] = {"samples": self._imu_samples, "rejected": self._imu_rejected,
                                "clock_resets": self._imu_clock.resets, "frame": "px4_imu_frd",
                                "clock": "device intervals anchored to UP7000 receive time"}
        self._status.publish(String(data=json.dumps(status, default=str)))

    def shutdown(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = Px4LoggerNode()
    except AlreadyRunning as exc:
        rclpy.logging.get_logger("px4_logger").error(str(exc))
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
