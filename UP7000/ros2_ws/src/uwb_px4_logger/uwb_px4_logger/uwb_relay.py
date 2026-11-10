"""TAG DevKit UART -> timestamped log + WiFi UDP to the ground New_GUI.

Two paths, so a busy CPU can never cost logged data:

  hot  (UART reader, run()):  bytes as read -> uwb_frames.bin record
                                            -> UDP datagrams to the ground GUI
       No parsing, no CRC, no ROS: on 07/11 per-frame Python work plus ROS
       publishing at ~320 Burst/s starved the reader on the 2-core UP4000 and
       the J-Link CDC buffer overflowed (CRC errors, 78 % of cycles lost).
  cold (parser thread):       CRC-checked frames -> TAG housekeeping, command
                              ACK log, discovery beacon, on_message (ROS).
       Fed through a bounded queue; when it cannot keep up it drops chunks
       for itself only (counted), never for the file or the WiFi stream.

Ground UDP -> only CRC-valid CMD frames -> UART, so the GUI can still
calibrate, set the burst rate, etc. over WiFi. The GUI parses the byte stream,
so frames may be split across datagrams.

Ground discovery: with ground_host empty, the relay broadcasts the TAG's
DEVICE_INFO/INFO frame once per second. New_GUI answers with PING from its
listening socket; the relay then unicasts the stream to that address. A fixed
ground_host skips discovery and accepts commands only from that host.

The relay keeps the TAG configured by itself (PING, DEVICE_INFO, telemetry
0x07 = snapshot + per-cycle Burst + DIAG), so the log is complete even when
no ground station is connected.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import queue
import socket
import threading
import time
from typing import Callable

from .session import FrameRecordWriter, LossLog, PortUserWarning, describe_port_loss, now_ns, write_json
from .vendor import telemetry_protocol as tp

TELEMETRY_FEATURES = 0x07  # RANGE_SNAPSHOT | RANGE_MEAS (enables Burst records) | DIAG
_RESET = object()          # parser reset marker after a reopen


def _open_serial(port: str, baud: int):
    import serial  # pyserial; imported lazily so tests can inject fakes

    # A stalled J-Link command must not block the shared UART/UDP reader forever.
    ser = serial.Serial(port, 115200, timeout=0.005, write_timeout=0.2)
    ser.baudrate = baud  # force the J-Link VCOM line-coding transition
    ser.reset_input_buffer()
    return ser


class UwbRelay:
    def __init__(
        self,
        *,
        port_resolver: Callable[[], str | None],
        out_dir: Path,
        baud: int = 460800,
        ground_host: str = "",
        ground_port: int = 15555,
        local_port: int = 15556,
        batch_s: float = 0.010,
        max_datagram: int = 1200,
        beacon_host: str = "255.255.255.255",
        beacon_s: float = 1.0,
        ground_timeout_s: float = 5.0,
        on_message: Callable[[object, int, int], None] | None = None,
        on_ground: Callable[[str], None] | None = None,
        log: Callable[[str], None] = print,
        serial_factory: Callable[[str, int], object] = _open_serial,
        no_data_timeout_s: float = 3.0,
        parse_queue_chunks: int = 4096,
    ) -> None:
        self.port_resolver = port_resolver
        self.out_dir = Path(out_dir)
        self.baud = baud
        self.fixed_ground = ground_host.strip()
        self.ground_port = ground_port
        self.local_port = local_port
        self.batch_s = batch_s
        self.max_datagram = max_datagram
        self.beacon_addr = (beacon_host, ground_port)
        self.beacon_s = beacon_s
        self.ground_timeout_s = ground_timeout_s
        self.on_message = on_message
        self.on_ground = on_ground
        self.log = log
        self.serial_factory = serial_factory
        self._loss_log = LossLog(log)
        self._port_users = PortUserWarning(log, "TAG")
        self.no_data_timeout_s = no_data_timeout_s

        self.ground_addr: tuple[str, int] | None = (
            (self.fixed_ground, ground_port) if self.fixed_ground else None
        )
        self.counts: Counter = Counter()        # hot path only
        self.parse_counts: Counter = Counter()  # cold path only
        self.identity: dict | None = None
        self.port: str | None = None
        self._ser = None
        self._sock: socket.socket | None = None
        self._parser = tp.TelemetryStreamParser()
        self._ground_parser = tp.TelemetryStreamParser()
        self._parse_q: queue.Queue = queue.Queue(maxsize=parse_queue_chunks)
        self._parse_thread: threading.Thread | None = None
        self._write_lock = threading.Lock()
        self._log_lock = threading.Lock()
        self._batch: list[bytes] = []
        self._batch_bytes = 0
        self._batch_started = 0.0
        self._beacon_frame: bytes | None = None
        self._last_rx = 0.0
        self._last_ground_rx = -1e9
        self._last_beacon = -1e9
        self._last_ping = -1e9
        self._last_identity_req = -1e9
        self._last_status = 0.0
        self._last_open_try = -1e9
        self._open_problem: str | None = None
        self._cmd_seq = int(time.time_ns()) & 0xFFFFFFFF
        self._started_unix = time.time()
        self.writer = FrameRecordWriter(self.out_dir / "uwb_frames.bin")
        self._commands = (self.out_dir / "uwb_commands.jsonl").open("w", encoding="utf-8")

    # ------------------------------------------------------------------ setup
    def open_socket(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(("0.0.0.0", self.local_port))
        sock.setblocking(False)
        self._sock = sock

    def _ensure_serial(self, now: float) -> bool:
        if self._ser is not None:
            return True
        if now - self._last_open_try < 1.0:
            return False
        self._last_open_try = now
        port = self.port_resolver()
        if not port:
            self.counts["tag_port_missing"] += 1
            self._log_open_problem("TAG not found (no /dev/uwb_tag or SEGGER by-id link)")
            return False
        self._port_users.check(port)
        try:
            self._ser = self.serial_factory(port, self.baud)
        except Exception as exc:  # unplugged, permission, busy
            self.counts["tag_open_errors"] += 1
            self._log_open_problem(f"TAG port {port}: {exc}")
            return False
        self._open_problem = None
        self.port = port
        self._enqueue_parse(_RESET, 0, 0)
        self._last_ping = self._last_identity_req = -1e9
        self._last_rx = now
        self.log(f"TAG UART open: {port} @ {self.baud}")
        return True

    def _check_silence(self, now: float) -> None:
        """The TAG streams continuously and ACKs a PING every second. A port
        that stays silent is dead (seen on 07/11 after a USB re-enumeration:
        the old handle neither errored nor delivered data): reopen it."""
        if self._ser is not None and now - self._last_rx > self.no_data_timeout_s:
            self._drop_serial(TimeoutError(f"no data for {self.no_data_timeout_s:.0f} s, reopening"),
                              counter="tag_silent_reopens")

    def _log_open_problem(self, text: str) -> None:
        if text != self._open_problem:  # once per distinct problem, not every retry
            self._open_problem = text
            self.log(text + " (retrying every second)")

    def _drop_serial(self, exc: Exception, counter: str = "tag_io_errors") -> None:
        """Hot path only: the reader owns opening and closing the port."""
        self.counts[counter] += 1
        self._loss_log(f"TAG UART lost: {describe_port_loss(exc)}")
        ser, self._ser = self._ser, None
        try:
            ser.close()
        except Exception:
            pass

    # -------------------------------------------------------------- TAG side
    def _send_tag(self, command: int, payload: bytes = b"") -> None:
        with self._write_lock:
            self._cmd_seq = (self._cmd_seq + 1) & 0xFFFFFFFF
            frame = tp.encode_command(self._cmd_seq, command, payload)
        self._write_tag(frame)

    def _write_tag(self, frame: bytes) -> None:
        """Both paths write; a failure is only counted here. The reader detects
        a dead port (read error or silence) and reopens it."""
        with self._write_lock:
            ser = self._ser
            if ser is None:
                return
            try:
                ser.write(frame)
            except Exception:
                self.counts["tag_write_errors"] += 1

    def _housekeeping(self, now: float) -> None:
        if now - self._last_ping >= 1.0:
            self._send_tag(tp.CMD_PING)
            self._last_ping = now
        if self.identity is None and now - self._last_identity_req >= 2.0:
            self._send_tag(tp.CMD_GET_DEVICE_INFO)
            self._last_identity_req = now

    def _poll_serial(self, now: float) -> None:
        try:
            data = self._ser.read(self._ser.in_waiting or 1)
        except Exception as exc:
            self._drop_serial(exc)
            return
        if not data:
            return
        self._last_rx = now
        realtime_ns, monotonic_ns = now_ns()
        self.counts["tag_bytes"] += len(data)
        self.writer.write(data, realtime_ns, monotonic_ns)
        self._queue_udp(data, now)
        self._enqueue_parse(data, realtime_ns, monotonic_ns)

    # ------------------------------------------------------------ cold path
    def _enqueue_parse(self, data, realtime_ns: int, monotonic_ns: int) -> None:
        try:
            self._parse_q.put_nowait((data, realtime_ns, monotonic_ns))
        except queue.Full:
            self.counts["parse_queue_dropped_chunks"] += 1

    def process_parse_queue(self) -> None:
        """Parse everything queued (the parser thread, or step() in tests)."""
        while True:
            try:
                item = self._parse_q.get_nowait()
            except queue.Empty:
                return
            self._parse_chunk(*item)

    def _parse_loop(self, stop: threading.Event) -> None:
        while True:
            try:
                item = self._parse_q.get(timeout=0.1)
            except queue.Empty:
                if stop.is_set():
                    return
                continue
            self._parse_chunk(*item)

    def _parse_chunk(self, data, realtime_ns: int, monotonic_ns: int) -> None:
        if data is _RESET:
            self._parser = tp.TelemetryStreamParser()
            return
        for frame in self._parser.feed(data):
            self.parse_counts[f"frame_0x{frame.type:02X}"] += 1
            try:
                msg = tp.decode_frame(frame)
            except tp.ProtocolError:
                self.parse_counts["decode_errors"] += 1
                continue
            self._handle_tag_message(msg, frame)
            if self.on_message is not None:
                self._callback(self.on_message, msg, realtime_ns, monotonic_ns)

    def _handle_tag_message(self, msg: object, frame: tp.Frame) -> None:
        if isinstance(msg, tp.DeviceInfoMessage):
            new_boot = self.identity is None or self.identity.get("boot_id") != msg.boot_id
            self.identity = asdict(msg)
            self._beacon_frame = tp.encode_frame(frame.type, frame.sequence, frame.time_ms, frame.payload)
            if new_boot:
                self.log(f"TAG config=0x{msg.build_config_hash:08X} boot={msg.boot_id} "
                         f"features=0x{msg.telemetry_features:02X}")
                if (msg.telemetry_features & TELEMETRY_FEATURES) != TELEMETRY_FEATURES:
                    self._send_tag(tp.CMD_SET_TELEMETRY, bytes((msg.telemetry_features | TELEMETRY_FEATURES,)))
        elif isinstance(msg, tp.InfoMessage) and self._beacon_frame is None:
            self._beacon_frame = tp.encode_frame(frame.type, frame.sequence, frame.time_ms, frame.payload)
        elif isinstance(msg, tp.CmdAckMessage):
            event = asdict(msg)
            event["data"] = msg.data.hex()
            self._log_command("tag_ack", event)

    def _log_command(self, kind: str, event: dict) -> None:
        realtime_ns, monotonic_ns = now_ns()
        line = json.dumps({"kind": kind, "realtime_ns": realtime_ns, "monotonic_ns": monotonic_ns, **event})
        with self._log_lock:
            if not self._commands.closed:
                self._commands.write(line + "\n")

    def _callback(self, fn, *args) -> None:
        """ROS publishing must never stop logging (e.g. a publish racing rclpy
        shutdown, or a malformed message)."""
        try:
            fn(*args)
        except Exception as exc:
            self.parse_counts["callback_errors"] += 1
            if self.parse_counts["callback_errors"] == 1:
                self.log(f"Callback failed, logging continues: {exc!r}")

    # ----------------------------------------------------------- ground side
    def _queue_udp(self, data: bytes, now: float) -> None:
        if not self._batch:
            self._batch_started = now
        self._batch.append(data)
        self._batch_bytes += len(data)
        if self._batch_bytes >= self.max_datagram:
            self._flush_udp(now)

    def _flush_udp(self, now: float) -> None:
        if not self._batch:
            return
        packet = b"".join(self._batch)
        self._batch.clear()
        self._batch_bytes = 0
        if self.ground_addr is None:
            self.counts["udp_no_ground_bytes"] += len(packet)
            return
        for start in range(0, len(packet), self.max_datagram):
            self._sendto(packet[start:start + self.max_datagram], self.ground_addr, "udp")

    def _sendto(self, packet: bytes, addr: tuple[str, int], kind: str) -> None:
        try:
            self._sock.sendto(packet, addr)
            self.counts[f"{kind}_datagrams"] += 1
            self.counts[f"{kind}_bytes"] += len(packet)
        except OSError:  # WiFi down / no route: drop, the file log is complete
            self.counts[f"{kind}_send_errors"] += 1

    def _poll_ground(self, now: float) -> None:
        if self._sock is None:
            return
        while True:
            try:
                data, addr = self._sock.recvfrom(4096)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:  # e.g. Windows ICMP port-unreachable echo
                return
            if self.fixed_ground and addr[0] != self.fixed_ground:
                self.counts["ground_rejected"] += 1
                continue
            self._last_ground_rx = now
            if not self.fixed_ground and self.ground_addr != addr:
                self.ground_addr = addr
                self.log(f"Ground station: {addr[0]}:{addr[1]}")
                if self.on_ground is not None:
                    self._callback(self.on_ground, addr[0])
            for frame in self._ground_parser.feed(data):
                if frame.type != tp.TYPE_CMD:
                    self.counts["ground_non_command"] += 1
                    continue
                encoded = tp.encode_frame(frame.type, frame.sequence, frame.time_ms, frame.payload)
                self._write_tag(encoded)
                self.counts["ground_commands"] += 1
                self._log_command("ground_cmd", {"command_id": frame.payload[0] if frame.payload else None,
                                                 "frame": encoded.hex()})

    def _beacon(self, now: float) -> None:
        stale = self.ground_addr is None or (
            not self.fixed_ground and now - self._last_ground_rx > self.ground_timeout_s
        )
        beacon = self._beacon_frame
        if not stale or beacon is None or now - self._last_beacon < self.beacon_s:
            return
        self._last_beacon = now
        self._sendto(beacon, self.beacon_addr, "beacon")

    # ------------------------------------------------------------------ loop
    def step(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        if self._ensure_serial(now):
            self._housekeeping(now)
            if self._ser is not None:
                self._poll_serial(now)
            self._check_silence(now)
        else:
            time.sleep(0.01)
        if self._parse_thread is None:
            self.process_parse_queue()  # no parser thread (tests drive step())
        if self._batch and now - self._batch_started >= self.batch_s:
            self._flush_udp(now)
        self._poll_ground(now)
        self._beacon(now)
        if now - self._last_status >= 1.0:
            self._last_status = now
            self.writer.flush()
            with self._log_lock:
                self._commands.flush()
            write_json(self.out_dir / "uwb_status.json", self.status())

    def run(self, stop: threading.Event) -> None:
        if self._sock is None:
            self.open_socket()
        self.log(f"UWB relay: UDP local :{self.local_port}, ground "
                 f"{self.fixed_ground or 'auto-discover'}:{self.ground_port}")
        self._parse_thread = threading.Thread(target=self._parse_loop, args=(stop,),
                                              name="uwb-parse", daemon=True)
        self._parse_thread.start()
        try:
            while not stop.is_set():
                self.step()
        finally:
            self._parse_thread.join(timeout=2.0)
            self.close()

    def status(self) -> dict:
        return {
            "started_unix_s": self._started_unix,
            "updated_unix_s": time.time(),
            "tag_port": self.port,
            "tag_baud": self.baud,
            "ground": f"{self.ground_addr[0]}:{self.ground_addr[1]}" if self.ground_addr else None,
            "ground_fixed": bool(self.fixed_ground),
            "records_logged": self.writer.records,
            "log_bytes": self.writer.bytes,
            "parse_queue": self._parse_q.qsize(),
            "parser": asdict(self._parser.counters),
            "counts": dict(self.counts),
            "parsed": dict(self.parse_counts),
            "device_info": self.identity,
        }

    def close(self) -> None:
        self._flush_udp(time.monotonic())
        if self._parse_thread is None:
            self.process_parse_queue()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        self.writer.close()
        with self._log_lock:
            self._commands.close()
        write_json(self.out_dir / "uwb_status.json", self.status())
