"""ROS 2 CDR/MCAP session recording without a ROS installation on Windows.

Only standard ROS message types are used. Bag timestamps are laptop receive
times, anchored to the wall clock once per session; device clocks stay in the
payload. PX4 input is the received status stream, not the full MAVLink stream.
"""
from __future__ import annotations

from collections import Counter, deque
from copy import deepcopy
from dataclasses import asdict, is_dataclass
import json
import math
from pathlib import Path
import queue
import re
import threading
import time

import telemetry_protocol as tp


_HEADER = """\n================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""
SCHEMAS = {
    "std_msgs/msg/String": "string data\n",
    "std_msgs/msg/Float64": "float64 data\n",
    "std_msgs/msg/UInt32": "uint32 data\n",
    "std_msgs/msg/UInt64": "uint64 data\n",
    "std_msgs/msg/Float64MultiArray": """std_msgs/MultiArrayLayout layout
float64[] data
================================================================================
MSG: std_msgs/MultiArrayLayout
std_msgs/MultiArrayDimension[] dim
uint32 data_offset
================================================================================
MSG: std_msgs/MultiArrayDimension
string label
uint32 size
uint32 stride
""",
    "geometry_msgs/msg/PointStamped": """std_msgs/Header header
geometry_msgs/Point point
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
""" + _HEADER,
    "geometry_msgs/msg/Vector3Stamped": """std_msgs/Header header
geometry_msgs/Vector3 vector
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
""" + _HEADER,
}


def _json(value):
    return json.dumps(value, separators=(",", ":"), ensure_ascii=False,
                      default=lambda v: v.hex() if isinstance(v, bytes) else asdict(v))


def _name(value):
    return re.sub(r"[^a-zA-Z0-9_]", "_", str(value)).lower()


class Ros2BagRecorder:
    """Bounded, non-blocking producer queue; one thread owns the MCAP file.

    Overflow and I/O failures are exposed to the GUI and recording_status.json.
    stop() drains accepted events and finalizes the index and metadata.yaml.
    """

    def __init__(self, session_dir: Path, *, queue_size: int = 8192):
        try:
            from mcap_ros2.writer import Writer
            import yaml
        except ImportError as exc:
            raise RuntimeError("Install New_GUI requirements.txt to enable ROS 2 MCAP logging") from exc
        self._writer_class, self._yaml = Writer, yaml
        self.bag_dir = Path(session_dir) / "rosbag2"
        self.bag_dir.mkdir(exist_ok=False)
        self.mcap_path = self.bag_dir / "data_0.mcap"
        self._wall_ns, self._mono_s = time.time_ns(), time.monotonic()
        self._queue = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._ready = threading.Event()
        self.closed = False
        self.error = ""
        self.dropped_events = 0
        self.accepted_events = 0
        self.messages_written = 0
        self._topics = {}
        self._schemas = {}
        self._counts = Counter()
        self._first_ns = self._last_ns = None
        self._groups = {}
        self._finished_groups = deque(maxlen=256)
        self._last_px4 = {}
        self._thread = threading.Thread(target=self._run, name="ros2-mcap-writer", daemon=True)
        self._thread.start()
        if not self._ready.wait(5):
            self._stop.set()
            raise RuntimeError("MCAP writer did not start within 5 seconds")
        if self.error:
            self._thread.join(5)
            raise RuntimeError(self.error)

    def _enqueue(self, kind, data, received_s=None):
        if self._stop.is_set() or self.error:
            return False
        stamp = self._wall_ns + round(((time.monotonic() if received_s is None else received_s)
                                      - self._mono_s) * 1e9)
        try:
            self._queue.put_nowait((kind, data, stamp))
            self.accepted_events += 1
            return True
        except queue.Full:
            self.dropped_events += 1
            return False

    def log_telemetry(self, msg, received_s, solution=None):
        # Telemetry dataclasses are frozen; copy the engine's mutable mappings.
        solved = asdict(solution) if solution is not None else None
        return self._enqueue("telemetry", (msg, solved), received_s)

    def log_status(self, part, sender, received_s=None):
        # Listener state and reconstructed fragment dictionaries can change later.
        return self._enqueue("status", (deepcopy(part), sender), received_s)

    def log_metadata(self, metadata):
        return self._enqueue("metadata", deepcopy(metadata))

    def _write(self, topic, msgtype, message, ns):
        if msgtype not in self._schemas:
            self._schemas[msgtype] = self._writer.register_msgdef(msgtype, SCHEMAS[msgtype])
        if topic in self._topics and self._topics[topic] != msgtype:
            raise ValueError(f"Topic type changed: {topic}")
        self._topics[topic] = msgtype
        self._writer.write_message(topic, self._schemas[msgtype], message,
                                   log_time=ns, publish_time=ns, sequence=self._counts[topic])
        self._counts[topic] += 1
        self.messages_written += 1
        self._first_ns = ns if self._first_ns is None else min(ns, self._first_ns)
        self._last_ns = ns if self._last_ns is None else max(ns, self._last_ns)

    def _scalar(self, topic, value, ns, msgtype="Float64"):
        self._write(topic, f"std_msgs/msg/{msgtype}", {"data": value}, ns)

    def _text(self, topic, value, ns):
        self._scalar(topic, _json(value), ns, "String")

    def _vector(self, topic, values, ns, *, point=False):
        if not all(isinstance(v, (float, int)) and math.isfinite(v) for v in values):
            return
        header = {"stamp": {"sec": ns // 10**9, "nanosec": ns % 10**9}, "frame_id": "uwb_map"}
        key = "point" if point else "vector"
        msgtype = "PointStamped" if point else "Vector3Stamped"
        self._write(topic, f"geometry_msgs/msg/{msgtype}",
                    {"header": header, key: dict(zip(("x", "y", "z"), values))}, ns)

    def _telemetry(self, msg, solution, ns):
        if not is_dataclass(msg):
            return
        self._text(f"/uwb/telemetry/{_name(type(msg).__name__)}", asdict(msg), ns)
        if isinstance(msg, tp.RangeBurstMessage):
            self._scalar("/uwb/cycle_seq", msg.cycle_seq, ns, "UInt32")
            self._scalar("/uwb/tag_time_us", msg.t_us, ns, "UInt64")
            self._scalar("/uwb/boot_id", msg.boot_id, ns, "UInt32")
            self._scalar("/uwb/period_us", msg.period_us, ns, "UInt32")
            for a in msg.anchors:
                prefix = f"/uwb/anchor_{a.anchor_id}"
                self._scalar(prefix + "/flags", a.flags, ns, "UInt32")
                self._scalar(prefix + "/status", a.status, ns, "UInt32")
                if not a.radio_ok or a.range_mm is None:
                    continue
                self._scalar(prefix + "/diagnostic_range_m", a.range_mm / 1000.0, ns)
                if a.flags & tp.BURST_FLAG_CAL_OK and a.flags & tp.BURST_FLAG_VALID:
                    self._scalar(prefix + "/range_m", a.range_mm / 1000.0, ns)
                if a.fp_cdbm != tp.INT16_MIN:
                    self._scalar(prefix + "/fp_dbm", a.fp_cdbm / 100.0, ns)
                if a.rx_cdbm != tp.INT16_MIN:
                    self._scalar(prefix + "/rx_dbm", a.rx_cdbm / 100.0, ns)
            if solution is not None:
                self._text("/uwb/solution", solution, ns)
                self._vector("/uwb/position", [solution[k] for k in ("x", "y", "z")], ns, point=True)
                self._vector("/uwb/velocity", [solution[k] for k in ("vx_mps", "vy_mps", "vz_mps")], ns)
                self._scalar("/uwb/rms_m", solution["rms_m"], ns)

    def _status(self, packet, sender, ns):
        self._text("/up7000/status_parts", {"sender": sender, "packet": packet}, ns)
        path = packet.get("part", "")
        if path and path != "px4" and not path.startswith("px4."):
            return
        key = (sender, packet.get("seq"), packet.get("time_unix"))
        if key in self._finished_groups:
            return
        # Flush the preceding snapshot when a newer one starts. Never combine
        # message fields from different snapshots after packet loss.
        for old in list(self._groups):
            newer = (key[2] > old[2] if isinstance(key[2], (int, float)) and isinstance(old[2], (int, float))
                     else key[1] != old[1])
            if old[0] == sender and old != key and newer:
                self._flush_group(old)
        if len(self._groups) >= 8 and key not in self._groups:
            self._flush_group(next(iter(self._groups)))
        if key not in self._groups:
            self._groups[key] = {"data": {}, "ns": ns, "arrival": time.monotonic()}
        group = self._groups[key]
        if not path:  # v1: complete status in one packet
            group["data"] = packet.get("px4") or {}
            self._flush_group(key)
            return
        node = group["data"]
        for part in path.split(".")[1:]:
            node = node.setdefault(part, {})
        node.update(packet.get("data") or {})

    def _flush_group(self, key):
        group = self._groups.pop(key)
        self._finished_groups.append(key)
        data, ns = group["data"], group["ns"]
        if data.get("connected") is False:
            self._last_px4.clear()
            return
        for name, entry in (data.get("messages") or {}).items():
            age = entry.get("age_s")
            fields = entry.get("fields")
            if not isinstance(age, (int, float)) or not 0 <= age < 2 or not isinstance(fields, dict):
                continue
            signature = _json(fields)
            last_key = (key[0], name)
            if self._last_px4.get(last_key) == signature:
                continue
            self._last_px4[last_key] = signature
            prefix = f"/px4/{_name(name)}"
            self._text(prefix + "/message", {"age_s": age, "fields": fields}, ns)
            for field, value in fields.items():
                topic = prefix + "/" + _name(field)
                if isinstance(value, (int, float)) and math.isfinite(value):
                    self._scalar(topic, float(value), ns)
                elif isinstance(value, list) and value and all(
                    isinstance(v, (int, float)) and math.isfinite(v) for v in value
                ):
                    self._write(topic, "std_msgs/msg/Float64MultiArray",
                                {"layout": {"dim": [], "data_offset": 0}, "data": value}, ns)

    def _save_status(self):
        payload = {"closed": self.closed, "error": self.error, "dropped_events": self.dropped_events,
                   "accepted_events": self.accepted_events, "messages_written": self.messages_written,
                   "queue_events": self._queue.qsize(), "mcap": self.mcap_path.name}
        path = self.bag_dir / "recording_status.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        temporary.replace(path)

    def _finalize_metadata(self):
        start = self._first_ns or self._wall_ns
        duration = (self._last_ns or start) - start
        timing = {"starting_time": {"nanoseconds_since_epoch": start},
                  "duration": {"nanoseconds": duration}, "message_count": self.messages_written}
        info = {"version": 5, "storage_identifier": "mcap", **timing,
                "topics_with_message_count": [
                    {"topic_metadata": {"name": topic, "type": msgtype, "serialization_format": "cdr",
                                        "offered_qos_profiles": ""}, "message_count": self._counts[topic]}
                    for topic, msgtype in sorted(self._topics.items())],
                "compression_format": "", "compression_mode": "",
                "relative_file_paths": [self.mcap_path.name],
                "files": [{"path": self.mcap_path.name, **timing}]}
        (self.bag_dir / "metadata.yaml").write_text(
            self._yaml.safe_dump({"rosbag2_bagfile_information": info}, sort_keys=False), encoding="utf-8")
        (self.bag_dir / "topics.json").write_text(json.dumps(info["topics_with_message_count"], indent=2), encoding="utf-8")
        (self.bag_dir / "README_ROS2.md").write_text(
            "# New_GUI ROS 2 recording\n\n"
            "From a terminal with ROS 2 sourced and its MCAP storage plugin installed:\n\n"
            "```sh\nros2 bag info /path/to/rosbag2\nros2 bag play /path/to/rosbag2\n"
            "ros2 run plotjuggler plotjuggler --datafile /path/to/rosbag2/data_0.mcap\n```\n\n"
            "UWB position: /uwb/position/point/x (also y/z). Ranges: /uwb/anchor_1/range_m/data.\n"
            "PX4: /px4/attitude/roll/data, /px4/local_position_ned/x/data; see topics.json.\n\n"
            "Times are laptop receive times. TAG boot ID/cycle/device time are separate topics.\n"
            "uwb_map uses the configured GUI rig axes. PX4 fields retain MAVLink units and NED/FRD axes.\n"
            "PX4 is the received ~4 Hz status snapshot stream, with unchanged payloads deduplicated; "
            "it is not full-rate onboard MAVLink. Raw received status parts and UWB messages are JSON String topics.\n"
            "Only calibrated valid UWB results enter range_m. Other radio results enter diagnostic_range_m; "
            "missing measurements are not replaced by zeros. A position topic exists only when a solver fix exists.\n"
            "Check recording_status.json for queue drops, errors and finalized state.\n", encoding="utf-8")

    def _run(self):
        try:
            with self.mcap_path.open("xb") as stream:
                self._writer = self._writer_class(stream, enable_crcs=True)
                self._writer.add_metadata("new_gui", {"clock": "laptop_receive_unix_ns", "source": "New_GUI"})
                self._save_status()
                self._ready.set()
                last_status = time.monotonic()
                try:
                    while not self._stop.is_set() or not self._queue.empty():
                        try:
                            kind, payload, ns = self._queue.get(timeout=0.1)
                        except queue.Empty:
                            kind = None
                        if kind == "telemetry":
                            self._telemetry(*payload, ns)
                        elif kind == "status":
                            self._status(*payload, ns)
                        elif kind == "metadata":
                            self._text("/session/metadata", payload, ns)
                        for key, group in list(self._groups.items()):
                            if time.monotonic() - group["arrival"] > 0.6:
                                self._flush_group(key)
                        if time.monotonic() - last_status > 1:
                            stream.flush()
                            self._save_status()
                            last_status = time.monotonic()
                    for key in list(self._groups):
                        self._flush_group(key)
                finally:
                    self._writer.finish()
            self._finalize_metadata()
            self.closed = True
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            self._ready.set()
            try:
                self._save_status()
            except OSError as exc:
                self.error = self.error or f"Cannot save recording status: {exc}"

    def stop(self, timeout_s=10):
        self._stop.set()
        self._thread.join(timeout_s)
        if self._thread.is_alive():
            raise TimeoutError(f"MCAP is still finalizing: {self.bag_dir}")
        if self.error:
            raise RuntimeError(self.error)
        return self.bag_dir


def plotjuggler_command(mcap_path: Path):
    """Argument vector, never shell interpolation of user-selected paths."""
    import sys
    script = Path(__file__).resolve().parent.parent / "UWB_UART_GUI/ros2/start_plotjuggler.sh"
    if sys.platform == "win32":
        def wsl_path(path):
            value = path.resolve().as_posix()
            if not re.match(r"^[A-Za-z]:/", value):
                raise ValueError("PlotJuggler via WSL needs a file on a local Windows drive")
            return "/mnt/" + value[0].lower() + value[2:]
        return "wsl.exe", ["-d", "Ubuntu-22.04", "--", "bash", wsl_path(script),
                           "--datafile", wsl_path(mcap_path)]
    return "bash", [str(script), "--datafile", str(mcap_path.resolve())]
