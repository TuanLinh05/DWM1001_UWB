"""Pixhawk (PX4) USB MAVLink -> .tlog + streamed .ulg + time sync + QGC forward.

- px4.tlog: every MAVLink packet received, standard tlog framing (uint64
  big-endian UNIX microseconds of the UP4000, then the raw packet). Opens in
  QGroundControl, MAVExplorer and pymavlink.
- px4_stream_<n>.ulg: the full PX4 ULog streamed over MAVLink
  (MAV_CMD_LOGGING_START). With SDLOG_PROFILE including "Estimator replay"
  it holds every EKF2 topic (estimator_states, innovations, test ratios,
  sensor inputs). A new file starts after each USB reconnect.
- timesync.csv: MAVLink TIMESYNC round trips, mapping PX4 boot time (ULog
  timestamps) onto the UP4000 clocks used by uwb_frames.bin.
- MAVLink is forwarded to the ground (QGroundControl UDP 14550) both ways.
  Log streaming packets are never forwarded: they would saturate the WiFi.
"""
from __future__ import annotations

from collections import Counter, deque
import math
import os
from pathlib import Path
import socket
import struct
import threading
import time
from typing import Callable

from .session import LossLog, PortUserWarning, describe_port_loss, now_ns, write_json
from .ulog_stream import UlogStreamWriter

# ODOMETRY (331) and LOGGING_DATA (266) only exist in MAVLink 2; select the
# v2 dialect before pymavlink is imported anywhere in this process.
os.environ.setdefault("MAVLINK20", "1")

MAV_CMD_LOGGING_START = 2510
ULOG_MAGIC = b"ULog"
MAV_CMD_LOGGING_STOP = 2511
MAV_CMD_SET_MESSAGE_INTERVAL = 511
ONBOARD_COMPUTER_COMPID = 191
NOT_FORWARDED = {"LOGGING_DATA", "LOGGING_DATA_ACKED", "LOGGING_ACK"}
# Latest values kept for the ground "UP4000" tab (snapshot()).
SNAPSHOT_TYPES = (
    "HEARTBEAT", "SYS_STATUS", "BATTERY_STATUS", "EXTENDED_SYS_STATE", "ODOMETRY", "ATTITUDE",
    "ATTITUDE_QUATERNION", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "ESTIMATOR_STATUS",
    "VIBRATION", "HIGHRES_IMU", "GPS_RAW_INT", "ALTITUDE", "DISTANCE_SENSOR", "TIMESYNC",
)
DEFAULT_RATES = (
    "ODOMETRY:50", "ATTITUDE_QUATERNION:50", "LOCAL_POSITION_NED:50", "HIGHRES_IMU:200",
    "ESTIMATOR_STATUS:10", "VIBRATION:5", "EXTENDED_SYS_STATE:2",
    "OPTICAL_FLOW_RAD:50", "DISTANCE_SENSOR:50",
)


def _plain(value):
    """JSON-safe copy: NaN/inf -> None, bytes -> hex, tuples -> lists."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (bytes, bytearray)):
        return value.hex()
    if isinstance(value, dict):
        return {k: _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    return value


def parse_rates(entries) -> dict[str, float]:
    """["ODOMETRY:50", ...] -> {"ODOMETRY": 50.0}; bad entries are ignored."""
    rates: dict[str, float] = {}
    for item in entries or ():
        name, _, hz = str(item).partition(":")
        try:
            rates[name.strip().upper()] = float(hz)
        except ValueError:
            continue
    return rates


def _default_connect(device: str, baud: int):
    from pymavlink import mavutil

    return mavutil.mavlink_connection(
        device, baud=baud, source_system=1, source_component=ONBOARD_COMPUTER_COMPID,
        autoreconnect=False, dialect="common",
    )


class Px4Link:
    def __init__(
        self,
        *,
        device_resolver: Callable[[], str | None],
        out_dir: Path,
        baud: int = 921600,
        ulog_stream: bool = True,
        message_rates: dict[str, float] | None = None,
        timesync_hz: float = 2.0,
        forward_mavlink: bool = True,
        ground_host: str = "",
        ground_port: int = 14550,
        local_port: int = 14555,
        on_message: Callable[[object, int, int], None] | None = None,
        log: Callable[[str], None] = print,
        connect: Callable[[str, int], object] = _default_connect,
        no_data_timeout_s: float = 5.0,
    ) -> None:
        self.device_resolver = device_resolver
        self.out_dir = Path(out_dir)
        self.baud = baud
        self.ulog_enabled = ulog_stream
        self.message_rates = message_rates if message_rates is not None else parse_rates(DEFAULT_RATES)
        self.timesync_period = 1.0 / timesync_hz if timesync_hz > 0 else 0.0
        self.forward = forward_mavlink
        self.ground_port = ground_port
        self.local_port = local_port
        self.on_message = on_message
        self.log = log
        self.connect = connect
        self._loss_log = LossLog(log)
        self._port_users = PortUserWarning(log, "PX4")
        self.no_data_timeout_s = no_data_timeout_s
        self._last_rx = 0.0
        self._serial_port = None

        self._lock = threading.Lock()
        self._latest: dict[str, tuple[float, object]] = {}
        self._type_counts: Counter = Counter()
        self._snap_prev: tuple[float, Counter] = (time.monotonic(), Counter())  # first rates: since start
        self._statustext: deque = deque(maxlen=10)  # fits one 1200-byte status part
        self._ground_host = ground_host.strip()
        self._ground_addr: tuple[str, int] | None = None
        self.counts: Counter = Counter()
        self.mav = None
        self.device: str | None = None
        self.target: tuple[int, int] | None = None
        self.vehicle: dict = {}
        self._sock: socket.socket | None = None
        self._ulog: UlogStreamWriter | None = None
        self._ulog_fh = None
        self._ulog_parts = 0
        self._last_ulog_stats: dict | None = None
        self._ulog_requested_at = -1e9
        self._ulog_last_data = -1e9
        self._last_open_try = -1e9
        self._open_problem: str | None = None
        self._last_timesync = -1e9
        self._last_heartbeat = -1e9
        self._last_status = 0.0
        self._started_unix = time.time()
        self._tlog = (self.out_dir / "px4.tlog").open("wb")
        self._timesync = (self.out_dir / "timesync.csv").open("w", encoding="utf-8")
        self._timesync.write("host_send_realtime_ns,host_send_monotonic_ns,host_recv_realtime_ns,"
                             "host_recv_monotonic_ns,px4_time_ns,rtt_ns,px4_minus_host_monotonic_ns\n")
        self._timesync_pending: dict[int, tuple[int, int]] = {}

    # -------------------------------------------------------------- ground
    def set_ground_host(self, host: str) -> None:
        """Called when the UWB relay discovers the ground station (thread-safe)."""
        with self._lock:
            if not self._ground_host and host and (self._ground_addr is None or self._ground_addr[0] != host):
                self._ground_addr = (host, self.ground_port)
                self.log(f"MAVLink forward -> {host}:{self.ground_port}")

    def open_socket(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.local_port))
        sock.setblocking(False)
        self._sock = sock
        if self._ground_host:
            self._ground_addr = (self._ground_host, self.ground_port)

    def _forward_to_ground(self, raw: bytes) -> None:
        with self._lock:
            addr = self._ground_addr
        if not self.forward or addr is None or self._sock is None:
            return
        try:
            self._sock.sendto(raw, addr)
            self.counts["forward_bytes"] += len(raw)
        except OSError:
            self.counts["forward_errors"] += 1

    def _poll_ground(self) -> None:
        if self._sock is None:
            return
        while True:
            try:
                data, addr = self._sock.recvfrom(4096)
            except (BlockingIOError, InterruptedError, OSError):
                return
            with self._lock:
                if self._ground_host and addr[0] != self._ground_host:
                    self.counts["ground_rejected"] += 1
                    continue
                self._ground_addr = addr  # QGC answers from its own port
            if self.mav is not None:
                try:
                    self.mav.write(data)  # raw passthrough: QGC talks to PX4
                    self.counts["ground_to_px4_bytes"] += len(data)
                except Exception as exc:
                    self._drop_link(exc)

    # ---------------------------------------------------------------- link
    def _ensure_link(self, now: float) -> bool:
        if self.mav is not None:
            return True
        if now - self._last_open_try < 1.0:
            return False
        self._last_open_try = now
        device = self.device_resolver()
        if not device:
            self.counts["px4_port_missing"] += 1
            self._log_open_problem("PX4 not found (no /dev/pixhawk or PX4 by-id link)")
            return False
        self._port_users.check(device)
        try:
            self.mav = self.connect(device, self.baud)
        except Exception as exc:
            self.counts["px4_open_errors"] += 1
            self._log_open_problem(f"PX4 port {device}: {exc}")
            return False
        self._open_problem = None
        self.device = device
        self._last_rx = now
        self.target = None
        mavlink = getattr(self.mav, "mav", None)
        if mavlink is not None:
            mavlink.robust_parsing = True  # corrupt bytes -> BAD_DATA, not an exception
        port = getattr(self.mav, "port", None)
        self._serial_port = port if hasattr(port, "in_waiting") else None
        if self._serial_port is not None:
            self._serial_port.timeout = 0.01  # pymavlink opens it non-blocking
        self.log(f"PX4 MAVLink open: {device}")
        return True

    def _read_raw(self) -> bytes:
        """Everything the OS has buffered (serial), or one datagram (UDP)."""
        port = self._serial_port
        if port is not None:
            return port.read(port.in_waiting or 1)
        data = self.mav.recv()  # UDP etc.: keeps pymavlink's reply address
        if not data:
            time.sleep(0.002)
        return data

    def _log_open_problem(self, text: str) -> None:
        if text != self._open_problem:  # once per distinct problem, not every retry
            self._open_problem = text
            self.log(text + " (retrying every second)")

    def _drop_link(self, exc: Exception) -> None:
        self.counts["px4_io_errors"] += 1
        self._loss_log(f"PX4 link lost: {describe_port_loss(exc)}")
        try:
            self.mav.close()
        except Exception:
            pass
        self.mav = None
        self.target = None
        self._close_ulog()

    def _command_long(self, command: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self.mav.mav.command_long_send(self.target[0], self.target[1], command, 0, *p)

    def _on_autopilot_found(self, msg) -> None:
        self.target = (msg.get_srcSystem(), msg.get_srcComponent())
        self.vehicle = {"system": self.target[0], "component": self.target[1],
                        "autopilot": getattr(msg, "autopilot", None), "type": getattr(msg, "type", None)}
        self.log(f"PX4 heartbeat: system {self.target[0]} component {self.target[1]}")
        from pymavlink.dialects.v20 import common as mavlink

        for name, hz in self.message_rates.items():
            msg_id = getattr(mavlink, f"MAVLINK_MSG_ID_{name}", None)
            if msg_id is None:
                self.log(f"Unknown MAVLink message for rate request: {name}")
                continue
            interval_us = 1e6 / hz if hz > 0 else -1  # -1 disables the stream
            self._command_long(MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, interval_us)
        if self.ulog_enabled:
            self._start_ulog(time.monotonic())

    # ---------------------------------------------------------------- ulog
    def _start_ulog(self, now: float) -> None:
        self._ulog_requested_at = now
        self._command_long(MAV_CMD_LOGGING_START, 0)  # param1 0 = ULog format
        self.counts["ulog_start_requests"] += 1

    @staticmethod
    def _ulog_stats(ulog: UlogStreamWriter) -> dict:
        return {"chunks": ulog.chunks, "bytes": ulog.bytes_written, "dropped_chunks": ulog.dropped_chunks,
                "dropout_messages": ulog.dropout_messages, "duplicates": ulog.duplicates}

    def _close_ulog(self) -> None:
        if self._ulog is not None:
            self._last_ulog_stats = self._ulog_stats(self._ulog)
        if self._ulog_fh is not None:
            self._ulog_fh.close()
            self._ulog_fh = None
            self._ulog = None

    def _open_ulog_part(self) -> UlogStreamWriter:
        self._close_ulog()
        self._ulog_parts += 1
        path = self.out_dir / f"px4_stream_{self._ulog_parts}.ulg"
        self._ulog_fh = path.open("wb")
        self._ulog = UlogStreamWriter(self._ulog_fh)
        self.log(f"ULog stream -> {path.name}")
        return self._ulog

    def _handle_logging_data(self, msg, acked: bool) -> None:
        if acked:
            self.mav.mav.logging_ack_send(self.target[0], self.target[1], msg.sequence)
        self._ulog_last_data = time.monotonic()
        data = bytes(msg.data[:msg.length])
        if msg.first_message_offset == 0 and data[:4] == ULOG_MAGIC:
            # A stream starts with the ULog file header. Chunks before it are
            # left over from an earlier session that PX4 was still streaming
            # (07/11: one stale chunk made the whole new stream look "old").
            self._open_ulog_part()
        elif self._ulog is None:
            self.counts["ulog_chunks_before_header"] += 1
            return
        try:
            self._ulog.handle(msg.sequence, data, msg.first_message_offset, acked)
        except ValueError as exc:
            self.counts["ulog_errors"] += 1
            self.log(f"ULog stream: {exc}")

    # ------------------------------------------------------------ timesync
    def _send_timesync(self, now: float) -> None:
        realtime_ns, monotonic_ns = now_ns()
        self._timesync_pending[monotonic_ns] = (realtime_ns, monotonic_ns)
        if len(self._timesync_pending) > 32:
            self._timesync_pending.pop(next(iter(self._timesync_pending)))
        self.mav.mav.timesync_send(0, monotonic_ns)
        self._last_timesync = now

    def _handle_timesync(self, msg, recv_rt: int, recv_mono: int) -> None:
        if msg.tc1 == 0:
            return  # a request from PX4, not our reply
        sent = self._timesync_pending.pop(msg.ts1, None)
        if sent is None:
            return
        rtt = recv_mono - sent[1]
        offset = msg.tc1 - (sent[1] + rtt // 2)
        self._timesync.write(f"{sent[0]},{sent[1]},{recv_rt},{recv_mono},{msg.tc1},{rtt},{offset}\n")
        self.counts["timesync_samples"] += 1

    # ---------------------------------------------------------------- loop
    def _write_tlog(self, raw: bytes, realtime_ns: int) -> None:
        self._tlog.write(struct.pack(">Q", realtime_ns // 1000) + raw)

    def step(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        self._poll_ground()
        if not self._ensure_link(now):
            time.sleep(0.01)
            self._status(now)
            return
        try:
            data = self._read_raw()
        except Exception as exc:
            self._drop_link(exc)
            return
        if data:
            self._last_rx = now
            self.counts["px4_bytes"] += len(data)
            # One parse call for everything buffered. pymavlink's recv_match
            # reads only bytes_needed() per call, several syscalls per message;
            # under CPU load on the UP4000 that let the tty buffer overflow.
            for msg in self.mav.mav.parse_buffer(data) or ():
                self._handle(msg)
        elif now - self._last_rx > self.no_data_timeout_s:
            # PX4 sends HEARTBEAT at 1 Hz: a silent port is dead, reopen it.
            self.counts["px4_silent_reopens"] += 1
            self._drop_link(TimeoutError(f"no MAVLink for {self.no_data_timeout_s:.0f} s, reopening"))
            return
        if now - self._last_heartbeat >= 1.0:
            self._last_heartbeat = now
            try:  # MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, MAV_STATE_ACTIVE
                self.mav.mav.heartbeat_send(18, 8, 0, 0, 4)
            except Exception as exc:
                self._drop_link(exc)
                return
        if self.target is not None:
            if self.timesync_period and now - self._last_timesync >= self.timesync_period:
                self._send_timesync(now)
            if self.ulog_enabled and now - self._ulog_requested_at > 3.0 and (
                    self._ulog is None or now - self._ulog_last_data > 3.0):
                # No header yet (PX4 may still be streaming an old session, and
                # then ignores START) or the stream stopped: stop, then start.
                if self.counts["ulog_start_requests"] == 3:
                    self.log("No ULog stream after 3 requests: check PX4 SDLOG_BACKEND includes MAVLink (2)")
                self._command_long(MAV_CMD_LOGGING_STOP)
                self._start_ulog(now)
        self._status(now)

    def _handle(self, msg) -> None:
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            self.counts["bad_data"] += 1
            return
        realtime_ns, monotonic_ns = now_ns()
        raw = bytes(msg.get_msgbuf())
        self._write_tlog(raw, realtime_ns)
        self.counts["messages"] += 1
        if self.target is None and mtype == "HEARTBEAT" and msg.get_srcComponent() == 1:
            self._on_autopilot_found(msg)
        if mtype in ("LOGGING_DATA", "LOGGING_DATA_ACKED"):
            if self.target is not None:
                self._handle_logging_data(msg, mtype == "LOGGING_DATA_ACKED")
        elif mtype == "TIMESYNC":
            self._handle_timesync(msg, realtime_ns, monotonic_ns)
        elif mtype == "COMMAND_ACK" and msg.command == MAV_CMD_LOGGING_START:
            self.counts[f"ulog_start_ack_{msg.result}"] += 1
        if mtype not in NOT_FORWARDED:
            self._forward_to_ground(raw)
        self._remember(msg, mtype, monotonic_ns)
        if self.on_message is not None:
            self._callback(self.on_message, msg, realtime_ns, monotonic_ns)

    def _remember(self, msg, mtype: str, monotonic_ns: int) -> None:
        """Keep the newest message of each snapshot type and count every type.
        Conversion to plain values happens in snapshot(), a few times a second."""
        with self._lock:
            self._type_counts[mtype] += 1
            if mtype == "STATUSTEXT":
                self._statustext.append((time.time(), msg.severity, str(msg.text)))
            elif mtype in SNAPSHOT_TYPES and (mtype != "HEARTBEAT" or msg.get_srcComponent() == 1):
                self._latest[mtype] = (monotonic_ns / 1e9, msg)

    def snapshot(self) -> dict:
        """Latest PX4 values (SNAPSHOT_TYPES), per-type rates since the last
        call, and recent STATUSTEXT. Safe to call from another thread."""
        now = time.monotonic()
        with self._lock:
            latest = dict(self._latest)
            counts = Counter(self._type_counts)
            texts = list(self._statustext)
        prev_t, prev_counts = self._snap_prev
        dt = now - prev_t
        self._snap_prev = (now, counts)
        rates = {t: round((n - prev_counts.get(t, 0)) / dt, 1) for t, n in sorted(counts.items())} if dt > 0 else {}
        messages = {}
        for mtype, (t, msg) in sorted(latest.items()):
            fields = msg.to_dict()
            fields.pop("mavpackettype", None)
            messages[mtype] = {"age_s": round(now - t, 3), "fields": _plain(fields)}
        return {
            "connected": self.mav is not None,
            "port": self.device,
            "vehicle": self.vehicle,
            "rates_hz": rates,
            "messages": messages,
            "statustext": [{"time_unix": ts, "severity": sev, "text": txt} for ts, sev, txt in texts],
        }

    def _status(self, now: float) -> None:
        if now - self._last_status < 1.0:
            return
        self._last_status = now
        self._tlog.flush()
        self._timesync.flush()
        if self._ulog_fh is not None:
            self._ulog_fh.flush()
        write_json(self.out_dir / "px4_status.json", self.status())

    def _callback(self, fn, *args) -> None:
        """ROS publishing must never stop the logging thread (e.g. a publish
        racing rclpy shutdown, or a malformed message)."""
        try:
            fn(*args)
        except Exception as exc:
            self.counts["callback_errors"] += 1
            if self.counts["callback_errors"] == 1:
                self.log(f"Callback failed, logging continues: {exc!r}")

    def status(self) -> dict:
        ulog = self._ulog
        ulog_stats = self._ulog_stats(ulog) if ulog is not None else self._last_ulog_stats
        return {
            "started_unix_s": self._started_unix,
            "updated_unix_s": time.time(),
            "px4_port": self.device,
            "vehicle": self.vehicle,
            "ground": f"{self._ground_addr[0]}:{self._ground_addr[1]}" if self._ground_addr else None,
            "ulog_part": self._ulog_parts,
            "ulog": ulog_stats,
            "counts": dict(self.counts),
        }

    def run(self, stop: threading.Event) -> None:
        if self._sock is None:
            self.open_socket()
        try:
            while not stop.is_set():
                self.step()
        finally:
            self.close()

    def close(self) -> None:
        if self.mav is not None and self.target is not None and self.ulog_enabled:
            try:
                self._command_long(MAV_CMD_LOGGING_STOP)
                if self._serial_port is not None:
                    self._serial_port.flush()  # let STOP leave before the port closes
                time.sleep(0.05)
            except Exception:
                pass
        if self.mav is not None:
            try:
                self.mav.close()
            except Exception:
                pass
            self.mav = None
        self._close_ulog()
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        self._tlog.close()
        self._timesync.close()
        write_json(self.out_dir / "px4_status.json", self.status())
