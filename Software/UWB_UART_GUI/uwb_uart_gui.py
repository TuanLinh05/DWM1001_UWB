"""Tkinter GUI for live DWM1001 Tag telemetry over a serial COM port."""

from __future__ import annotations

import argparse
from collections import deque
import math
import os
from pathlib import Path
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - exercised only on missing dependency
    raise SystemExit(
        "PySerial chưa được cài. Chạy: py -3.12 -m pip install -r requirements.txt"
    ) from exc

from telemetry_protocol import (
    AnchorSample,
    CmdAckMessage,
    DeviceInfoMessage,
    DiagSystemMessage,
    InfoMessage,
    MEAS_FLAG_ANCHOR_DIAG,
    MEAS_FLAG_CAL_OK,
    MEAS_FLAG_FILTER_OK,
    MEAS_FLAG_RADIO_OK,
    ParserCounters,
    ProtocolError,
    RangeMeasMessage,
    RangeMessage,
    StatsMessage,
    STATUS_CALIBRATION_MISSING,
    STATUS_RANGE_REJECT,
    TelemetryStreamParser,
    decode_frame,
    encode_frame,
    encode_range_meas_payload,
    fpp_to_legacy_scale_cdbm,
    status_names,
    TYPE_CMD_ACK,
    TYPE_DEVICE_INFO,
    TYPE_INFO,
    TYPE_RANGE,
    TYPE_RANGE_MEAS,
    TYPE_STATS,
    CMD_GET_DEVICE_INFO,
    CMD_PING,
    CMD_SET_TELEMETRY,
    TELEM_FEATURE_DIAG,
    TELEM_FEATURE_RANGE_MEAS,
    TELEM_FEATURE_RANGE_SNAPSHOT,
    TELEM_RANGE_MEAS_MIN_BAUD,
    encode_command,
)
from session_recorder import SessionRecorder
from host_range_filter import HostRangeFilter
from gui_analysis import MAX_ANCHORS, TelemetryPoint, save_layout
from gui_views import AnalysisPanel, AnchorMapPanel, CalibrationPanel, RangeMeasPanel


APP_TITLE = "DWM1001 UWB Ground Control"
# Tag_DevKit runs its UART at 1 Mbaud (RANGE_MEAS). The ESP32-C3 gateway is a
# USB device and ignores the baud; only a Tag PCB on a plain USB-UART adapter
# still needs 115200 selected by hand.
DEFAULT_BAUD = 1000000
UI_POLL_MS = 40
HISTORY_LENGTH = 300
ANALYSIS_HISTORY_LENGTH = 15000
HOST_HEARTBEAT_S = 1.0
DEVICE_INFO_RETRY_S = 2.0
LINK_HINT_AFTER_S = 3.0
MEAS_REFRESH_S = 0.25


def application_directory() -> Path:
    """Return a persistent directory for source and PyInstaller one-file builds."""
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


class SerialWorker(threading.Thread):
    def __init__(
        self, port: str, baud: int, events: queue.Queue, auto_enable_meas: bool = True
    ) -> None:
        super().__init__(name="uwb-serial-reader", daemon=True)
        self.port = port
        self.baud = baud
        self.events = events
        self.auto_enable_meas = auto_enable_meas
        self.stop_event = threading.Event()
        self._serial: serial.Serial | None = None
        self._command_sequence = int(time.time_ns()) & 0xFFFFFFFF
        self._requests: queue.Queue[tuple[int, bytes]] = queue.Queue()
        self._tag_online_reported = False
        self._snapshot_requested = False
        self._device_info_seen = False
        self._link_hint_sent = False

    def _send_command(self, command_id: int, arguments: bytes = b"") -> int:
        """Send one command from the reader thread (the sole serial owner)."""
        assert self._serial is not None
        self._command_sequence = (self._command_sequence + 1) & 0xFFFFFFFF
        self._serial.write(encode_command(self._command_sequence, command_id, arguments))
        return self._command_sequence

    def request_command(self, command_id: int, arguments: bytes = b"") -> None:
        """Queue a command from the GUI thread; the reader thread sends it."""
        self._requests.put((command_id, bytes(arguments)))

    def _send_requested_commands(self) -> None:
        while True:
            try:
                command_id, arguments = self._requests.get_nowait()
            except queue.Empty:
                return
            self._send_command(command_id, arguments)

    def _configure_session_telemetry(self, message: DeviceInfoMessage) -> None:
        """Snapshot always (the Live tab needs it) and RANGE_MEAS when the
        UART can carry it. Session only: settings saved on the TAG stay."""
        if self._snapshot_requested:
            return
        current = message.telemetry_features
        features = current | TELEM_FEATURE_RANGE_SNAPSHOT
        if self.auto_enable_meas and message.uart_baud >= TELEM_RANGE_MEAS_MIN_BAUD:
            features |= TELEM_FEATURE_RANGE_MEAS
        if features == current:
            return
        self._send_command(CMD_SET_TELEMETRY, bytes((features,)))
        self._snapshot_requested = True
        if not current & TELEM_FEATURE_RANGE_SNAPSHOT:
            self.events.put((
                "notice",
                "TAG đang tắt RANGE_SNAPSHOT; GUI đã bật lại cho phiên này.",
            ))
        if features & ~current & TELEM_FEATURE_RANGE_MEAS:
            self.events.put((
                "notice",
                f"TAG đang tắt RANGE_MEAS (thường do settings đã lưu từ firmware cũ); "
                f"UART {message.uart_baud} baud đủ nhanh nên GUI đã bật cho phiên này. "
                "Để TAG tự bật sau reboot: uwb_command.py set-telemetry "
                "--snapshot --meas --diag --save.",
            ))

    def _link_hint(self, received_bytes: int) -> str:
        if received_bytes == 0:
            return (
                f"Chưa nhận byte nào sau {LINK_HINT_AFTER_S:.0f} s: kiểm tra đúng cổng COM "
                "và TAG đã được nạp firmware, đang chạy."
            )
        return (
            f"Nhận {received_bytes} byte nhưng không có frame hợp lệ ở {self.baud} baud: "
            "kiểm tra baud. Tag_DevKit chạy 1000000; Tag PCB nối USB-UART trực tiếp "
            "chạy 115200; qua gateway ESP32-C3 thì baud nào cũng được."
        )

    def stop(self) -> None:
        self.stop_event.set()
        if self._serial is not None:
            try:
                self._serial.cancel_read()
            except (AttributeError, OSError, serial.SerialException):
                pass

    def run(self) -> None:
        parser = TelemetryStreamParser()
        last_report = time.monotonic()
        report_bytes = 0
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.10)
            self._serial.reset_input_buffer()
            self.events.put(("connected", self.port))
            self._send_command(CMD_GET_DEVICE_INFO)
            last_ping = 0.0
            opened_at = last_device_info_request = time.monotonic()

            while not self.stop_event.is_set():
                now = time.monotonic()
                if now - last_ping >= HOST_HEARTBEAT_S:
                    self._send_command(CMD_PING)
                    last_ping = now
                if (not self._device_info_seen
                        and now - last_device_info_request >= DEVICE_INFO_RETRY_S):
                    self._send_command(CMD_GET_DEVICE_INFO)
                    last_device_info_request = now
                self._send_requested_commands()
                if (not self._link_hint_sent and parser.counters.valid_frames == 0
                        and now - opened_at >= LINK_HINT_AFTER_S):
                    self._link_hint_sent = True
                    self.events.put(("notice", self._link_hint(parser.counters.bytes_received)))

                data = self._serial.read(self._serial.in_waiting or 1)
                report_bytes += len(data)
                for frame in parser.feed(data):
                    self.events.put((
                        "raw_frame",
                        encode_frame(frame.type, frame.sequence, frame.time_ms, frame.payload),
                    ))
                    try:
                        message = decode_frame(frame)
                    except ProtocolError as exc:
                        parser.counters.decode_errors += 1
                        self.events.put(("protocol_error", str(exc)))
                    else:
                        if not self._tag_online_reported:
                            self._tag_online_reported = True
                            self.events.put(("tag_online", self.port))
                        if (isinstance(message, DeviceInfoMessage)
                                and not self._device_info_seen):
                            self._device_info_seen = True
                            self._configure_session_telemetry(message)
                        self.events.put(("message", message))

                now = time.monotonic()
                if now - last_report >= 1.0:
                    byte_rate = report_bytes / (now - last_report)
                    self.events.put(("parser", (parser.counters.snapshot(), byte_rate)))
                    report_bytes = 0
                    last_report = now
        except (OSError, serial.SerialException) as exc:
            self.events.put(("error", str(exc)))
        finally:
            if self._serial is not None:
                try:
                    self._serial.close()
                except (OSError, serial.SerialException):
                    pass
            self.events.put(("disconnected", self.port))


DEMO_BOOT_ID = 0x0DE0
DEMO_UART_BAUD = 1000000
DEMO_PRE_OFFSET_MM = 30      # demo "raw" range before the calibration offset


class DemoWorker(threading.Thread):
    """End-to-end protocol demo used when hardware is not connected.

    It speaks the TAG protocol: INFO, DEVICE_INFO, 50 Hz RANGE snapshots,
    one RANGE_MEAS per measurement (A3 goes through a simulated NLOS episode
    every 20 s) and a CMD_ACK for SET_TELEMETRY from the RANGE_MEAS tab.
    """

    def __init__(self, events: queue.Queue) -> None:
        super().__init__(name="uwb-demo-source", daemon=True)
        self.events = events
        self.stop_event = threading.Event()
        self.features = (TELEM_FEATURE_RANGE_SNAPSHOT | TELEM_FEATURE_RANGE_MEAS
                         | TELEM_FEATURE_DIAG)
        self._requests: queue.Queue[tuple[int, bytes]] = queue.Queue()

    def stop(self) -> None:
        self.stop_event.set()

    def request_command(self, command_id: int, arguments: bytes = b"") -> None:
        self._requests.put((command_id, bytes(arguments)))

    @staticmethod
    def _device_info_payload(features: int) -> bytes:
        return struct.pack(
            "<BBHIBBBBIHHIBBHHIBBIIIBBBBBBBIBBII",
            1, 1, 0, 0, 0, 2, features, 0, 0xDE000001, DEMO_BOOT_ID, 1, 0,
            1, 0, 16436, 16436, 0x0E082848, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            DEMO_UART_BAUD, 0x0F, 0x0F, 0, 0,
        )

    def _answer_commands(self, parser: TelemetryStreamParser, tag_ms: int) -> None:
        while True:
            try:
                command_id, arguments = self._requests.get_nowait()
            except queue.Empty:
                return
            if command_id == CMD_SET_TELEMETRY and len(arguments) == 1:
                self.features = arguments[0] & (TELEM_FEATURE_RANGE_SNAPSHOT
                                                | TELEM_FEATURE_RANGE_MEAS
                                                | TELEM_FEATURE_DIAG)
                ack = bytes((command_id, 0x00, self.features))
            else:
                ack = bytes((command_id, 0x08))          # UNSUPPORTED in the demo
            self._emit(parser, encode_frame(TYPE_CMD_ACK, 0, tag_ms, ack))

    def run(self) -> None:
        parser = TelemetryStreamParser()
        sequence = 0
        meas_seq = 0
        start = last_report = time.monotonic()
        last_report_bytes = 0
        self.events.put(("connected", "DEMO"))

        info_payload = bytes((2, 0x05, 1, 4, 0x0F, 1, 1, 8, 0, 0))
        for anchor_id in range(1, 5):
            info_payload += struct.pack("<Hi", anchor_id, 0)
        self._emit(parser, encode_frame(TYPE_INFO, 0, 0, info_payload))
        self._emit(parser, encode_frame(
            TYPE_DEVICE_INFO, 0, 0, self._device_info_payload(self.features)))

        while not self.stop_event.wait(0.02):
            sequence += 1
            elapsed = time.monotonic() - start
            tag_ms = int(elapsed * 1000)
            self._answer_commands(parser, tag_ms)
            payload = bytearray((4,))
            tag_x = 2.5 + 1.5 * math.cos(elapsed * 0.35)
            tag_y = 2.0 + 1.1 * math.sin(elapsed * 0.48)
            demo_anchors = ((0.0, 0.0), (5.0, 0.0), (2.5, 4.0), (5.0, 4.0))
            measurements = []
            for anchor_id, (anchor_x, anchor_y) in enumerate(demo_anchors, start=1):
                ideal_mm = math.hypot(tag_x - anchor_x, tag_y - anchor_y) * 1000.0
                # A3 is shadowed 6 s out of every 20 s: longer path, weaker first path.
                nlos = anchor_id == 3 and elapsed % 20.0 >= 14.0
                raw_mm = int(ideal_mm + 12.0 * math.sin(elapsed * 3.1 + anchor_id)
                             + (350.0 + 40.0 * math.sin(elapsed * 17.0) if nlos else 0.0))
                filtered_mm = int(ideal_mm + 3.0 * math.sin(elapsed * 1.4 + anchor_id))
                fp_cdbm = -7850 - anchor_id * 35 - (900 if nlos else 0)
                rx_cdbm = fp_cdbm + (1150 if nlos else 250) + int(
                    80 * math.sin(elapsed * 2.3 + anchor_id))
                valid = 1
                status = 0
                payload.extend(
                    struct.pack("<HBBHiih", anchor_id, valid, status, 5, raw_mm, filtered_mm, fp_cdbm)
                )
                measurements.append((anchor_id, raw_mm, filtered_mm, fp_cdbm, rx_cdbm))
            frame = encode_frame(TYPE_RANGE, sequence, tag_ms, bytes(payload))
            self._emit(parser, frame[:7])
            self._emit(parser, frame[7:])

            if self.features & TELEM_FEATURE_RANGE_MEAS:
                for anchor_id, raw_mm, filtered_mm, fp_cdbm, rx_cdbm in measurements:
                    meas_seq += 1
                    message = RangeMeasMessage(
                        sequence=meas_seq, time_ms=tag_ms, boot_id=DEMO_BOOT_ID,
                        meas_seq=meas_seq,
                        meas_time_us=int(elapsed * 1e6) + (anchor_id - 1) * 2400,
                        anchor_id=anchor_id, txn=meas_seq & 0xFF, mode=0,
                        flags=(MEAS_FLAG_RADIO_OK | MEAS_FLAG_CAL_OK | MEAS_FLAG_FILTER_OK
                               | MEAS_FLAG_ANCHOR_DIAG),
                        status=0, raw_mm=raw_mm + DEMO_PRE_OFFSET_MM, corrected_mm=raw_mm,
                        filtered_mm=filtered_mm, fp_cdbm=fp_cdbm, rx_cdbm=rx_cdbm,
                        anchor_fp_cdbm=fp_cdbm - 40, anchor_rx_cdbm=rx_cdbm - 25,
                        std_noise=22 + 3 * anchor_id, fp_index=(745 + anchor_id) * 64 + 17,
                        ci_ppm_x100=(anchor_id * 2 - 5) * 90, slot_us=2150 + 45 * anchor_id,
                    )
                    self._emit(parser, encode_frame(
                        TYPE_RANGE_MEAS, meas_seq, tag_ms, encode_range_meas_payload(message)))

            if sequence % 50 == 0:
                stats_payload = struct.pack(
                    "<IIIIIIHH", sequence * 4, sequence, 0, 0, 0, 0, 50, 50
                )
                self._emit(
                    parser,
                    encode_frame(TYPE_STATS, sequence, tag_ms, stats_payload),
                )
                now = time.monotonic()
                byte_rate = (parser.counters.bytes_received - last_report_bytes) / max(
                    now - last_report, 1e-3)
                last_report, last_report_bytes = now, parser.counters.bytes_received
                self.events.put(("parser", (parser.counters.snapshot(), byte_rate)))

        self.events.put(("disconnected", "DEMO"))

    def _emit(self, parser: TelemetryStreamParser, data: bytes) -> None:
        for frame in parser.feed(data):
            self.events.put((
                "raw_frame",
                encode_frame(frame.type, frame.sequence, frame.time_ms, frame.payload),
            ))
            try:
                self.events.put(("message", decode_frame(frame)))
            except ProtocolError as exc:
                parser.counters.decode_errors += 1
                self.events.put(("protocol_error", str(exc)))


class UwbGui:
    def __init__(self, root: tk.Tk, demo: bool = False) -> None:
        self.root = root
        self.demo = demo
        self.events: queue.Queue = queue.Queue()
        self.worker: SerialWorker | DemoWorker | None = None
        self.port_labels: dict[str, str] = {}
        self.history: dict[int, deque[tuple[int, int | None, int]]] = {
            anchor_id: deque(maxlen=HISTORY_LENGTH) for anchor_id in range(1, MAX_ANCHORS + 1)
        }
        self.detailed_history: dict[int, deque[TelemetryPoint]] = {
            anchor_id: deque(maxlen=ANALYSIS_HISTORY_LENGTH)
            for anchor_id in range(1, MAX_ANCHORS + 1)
        }
        self.frame_times: deque[float] = deque(maxlen=ANALYSIS_HISTORY_LENGTH)
        self.latest_samples: dict[int, AnchorSample] = {}
        self.host_filters: dict[int, HostRangeFilter] = {}
        # INFO flags of the connected firmware; tells whether FPP uses the
        # corrected RXPACC scale (host filter thresholds use the old scale).
        self.info_flags: int | None = None
        self.last_diag_log = 0.0
        self.latest_host_filtered: dict[int, int] = {}
        self.anchor_items: dict[int, str] = {}
        self.last_range_sequence: int | None = None
        self.sequence_loss = 0
        self.range_frame_count = 0
        self.last_log_sequence = -1
        self.last_range_received: float | None = None
        self.data_stale = False
        self.recorder: SessionRecorder | None = None
        self.last_session_directory: Path | None = None
        self.recording_error_shown = False
        self.last_advanced_refresh = 0.0
        self.last_meas_refresh = 0.0
        self._last_ui_error: str | None = None

        self._configure_window()
        self._create_variables()
        self._build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(UI_POLL_MS, self._poll_events)
        if demo:
            self.root.after(250, self.connect)

    def _configure_window(self) -> None:
        self.root.title(APP_TITLE)
        self.root.geometry("1280x920")
        self.root.minsize(1040, 760)
        try:
            self.root.tk.call("tk", "scaling", 1.15)
        except tk.TclError:
            pass

        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("Title.TLabel", font=("Segoe UI Semibold", 16))
        style.configure("Heading.TLabel", font=("Segoe UI Semibold", 10))
        style.configure("Value.TLabel", font=("Consolas", 10))
        style.configure("Connected.TLabel", foreground="#147d3f", font=("Segoe UI Semibold", 10))
        style.configure("Pending.TLabel", foreground="#9a6700", font=("Segoe UI Semibold", 10))
        style.configure("Disconnected.TLabel", foreground="#a33a2b", font=("Segoe UI Semibold", 10))

    def _create_variables(self) -> None:
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.connection_var = tk.StringVar(value="Chưa kết nối")
        self.frame_var = tk.StringVar(value="0")
        self.crc_var = tk.StringVar(value="0")
        self.discarded_var = tk.StringVar(value="0")
        self.rate_var = tk.StringVar(value="0 B/s")
        self.loss_var = tk.StringVar(value="0")
        self.info_var = tk.StringVar(value="Chưa nhận INFO")
        self.stats_var = tk.StringVar(value="Chưa nhận STATS")
        self.sequence_var = tk.StringVar(value="-")
        self.uptime_var = tk.StringVar(value="-")
        self.graph_anchor_var = tk.StringVar(value="A1")
        default_logs = application_directory() / "data_logs"
        self.log_directory_var = tk.StringVar(value=str(default_logs))
        self.recording_var = tk.StringVar(value="Chưa ghi dữ liệu")
        self.record_count_var = tk.StringVar(value="0 frame / 0 mẫu")

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(outer)
        header.pack(fill=tk.X)
        ttk.Label(header, text=APP_TITLE, style="Title.TLabel").pack(side=tk.LEFT)
        self.connection_label = ttk.Label(
            header, textvariable=self.connection_var, style="Disconnected.TLabel"
        )
        self.connection_label.pack(side=tk.RIGHT)

        connection = ttk.LabelFrame(outer, text="Kết nối UART", padding=9)
        connection.pack(fill=tk.X, pady=(10, 8))
        ttk.Label(connection, text="COM:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(
            connection, textvariable=self.port_var, state="readonly", width=46
        )
        self.port_combo.pack(side=tk.LEFT, padx=(6, 8))
        ttk.Button(connection, text="Làm mới", command=self.refresh_ports).pack(side=tk.LEFT)
        ttk.Label(connection, text="Baud:").pack(side=tk.LEFT, padx=(16, 4))
        self.baud_combo = ttk.Combobox(
            connection,
            textvariable=self.baud_var,
            state="readonly",
            values=("115200", "230400", "460800", "921600", "1000000"),
            width=10,
        )
        self.baud_combo.pack(side=tk.LEFT)
        self.connect_button = ttk.Button(connection, text="Kết nối", command=self.connect)
        self.connect_button.pack(side=tk.RIGHT)

        # Recording and parser diagnostics are secondary to the plots.  Keep
        # the actions that matter during a run on one compact row and reveal
        # the directory/firmware details only when the operator asks for them.
        telemetry_section = ttk.Frame(outer)
        telemetry_section.pack(fill=tk.X, pady=(0, 8))
        telemetry_bar = ttk.Frame(telemetry_section)
        telemetry_bar.pack(fill=tk.X)
        ttk.Label(telemetry_bar, text="Thu dữ liệu", style="Heading.TLabel").pack(
            side=tk.LEFT
        )
        self.details_button = ttk.Button(
            telemetry_bar,
            text="Hiện chi tiết ▾",
            command=self._toggle_telemetry_details,
        )
        self.details_button.pack(side=tk.RIGHT, padx=(6, 0))
        self.record_button = ttk.Button(
            telemetry_bar, text="Bắt đầu ghi", command=self.toggle_recording
        )
        self.record_button.pack(side=tk.RIGHT)
        ttk.Label(
            telemetry_bar, textvariable=self.record_count_var, style="Value.TLabel"
        ).pack(side=tk.RIGHT, padx=12)
        ttk.Label(
            telemetry_bar, textvariable=self.recording_var, anchor=tk.W
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(10, 0))

        self.telemetry_details = ttk.Frame(telemetry_section)
        recording_options = ttk.Frame(self.telemetry_details)
        recording_options.pack(fill=tk.X, pady=(5, 4))
        ttk.Label(recording_options, text="Thư mục:").pack(side=tk.LEFT)
        ttk.Entry(
            recording_options, textvariable=self.log_directory_var, width=54
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 6))
        ttk.Button(
            recording_options, text="Chọn...", command=self.choose_log_directory
        ).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(
            recording_options, text="Mở thư mục", command=self.open_log_directory
        ).pack(side=tk.LEFT)

        summary = ttk.LabelFrame(
            self.telemetry_details, text="Luồng UART", padding=(8, 4)
        )
        summary.pack(fill=tk.X, pady=(0, 4))
        metrics = (
            ("Frame", self.frame_var),
            ("CRC lỗi", self.crc_var),
            ("Byte bỏ qua", self.discarded_var),
            ("Tốc độ", self.rate_var),
            ("Mất sequence", self.loss_var),
        )
        for index, (title, variable) in enumerate(metrics):
            if index:
                ttk.Separator(summary, orient=tk.VERTICAL).pack(
                    side=tk.LEFT, fill=tk.Y, padx=12
                )
            ttk.Label(summary, text=f"{title}:").pack(side=tk.LEFT)
            ttk.Label(summary, textvariable=variable, style="Value.TLabel").pack(
                side=tk.LEFT, padx=(4, 0)
            )

        info = ttk.LabelFrame(
            self.telemetry_details, text="Firmware / thống kê", padding=(8, 4)
        )
        info.pack(fill=tk.X)
        ttk.Label(info, textvariable=self.info_var, style="Value.TLabel").pack(
            anchor=tk.W
        )
        info_state_line = ttk.Frame(info)
        info_state_line.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(
            info_state_line, textvariable=self.stats_var, style="Value.TLabel"
        ).pack(side=tk.LEFT)
        ttk.Label(info_state_line, text="Sequence:").pack(side=tk.LEFT, padx=(20, 0))
        ttk.Label(
            info_state_line, textvariable=self.sequence_var, style="Value.TLabel"
        ).pack(side=tk.LEFT, padx=(4, 16))
        ttk.Label(info_state_line, text="Tag uptime:").pack(side=tk.LEFT)
        ttk.Label(
            info_state_line, textvariable=self.uptime_var, style="Value.TLabel"
        ).pack(side=tk.LEFT, padx=4)

        self.notebook = ttk.Notebook(outer)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        live_tab = ttk.Frame(self.notebook, padding=6)
        self.notebook.add(live_tab, text="  Live  ")

        table_frame = ttk.LabelFrame(live_tab, text="Khoảng cách theo Anchor", padding=6)
        table_frame.pack(fill=tk.BOTH, expand=True)
        columns = (
            "anchor", "valid", "status", "age", "raw", "filtered", "host_filtered", "fpp"
        )
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=5)
        headings = {
            "anchor": "Anchor",
            "valid": "Valid",
            "status": "Status",
            "age": "Age (ms)",
            "raw": "Raw (m)",
            "filtered": "FW Filter (m)",
            "host_filtered": "Host Filter (m)",
            "fpp": "FPP (dBm)",
        }
        widths = {
            "anchor": 65, "valid": 55, "status": 175, "age": 75,
            "raw": 95, "filtered": 95, "host_filtered": 110, "fpp": 90,
        }
        for column in columns:
            self.tree.heading(column, text=headings[column])
            self.tree.column(column, width=widths[column], anchor=tk.CENTER)
        self.tree.tag_configure("valid", background="#e7f5e9")
        self.tree.tag_configure("cal", background="#fff3cd")
        self.tree.tag_configure("error", background="#fde8e7")
        self._insert_anchor_placeholders()
        self.tree.pack(fill=tk.BOTH, expand=True)

        log_frame = ttk.LabelFrame(live_tab, text="Sự kiện gần nhất", padding=5)
        log_frame.pack(fill=tk.X, pady=(8, 0))
        self.log_text = tk.Text(log_frame, height=5, wrap=tk.NONE, font=("Consolas", 9), state=tk.DISABLED)
        self.log_text.pack(fill=tk.X)

        graph_tab = ttk.Frame(self.notebook, padding=6)
        self.notebook.add(graph_tab, text="  Biểu đồ lớn  ")
        graph_frame = ttk.LabelFrame(
            graph_tab, text="Biểu đồ khoảng cách Raw / Host Filter / Firmware Filter", padding=8
        )
        graph_frame.pack(fill=tk.BOTH, expand=True)
        graph_toolbar = ttk.Frame(graph_frame)
        graph_toolbar.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(graph_toolbar, text="Anchor:").pack(side=tk.LEFT)
        graph_combo = ttk.Combobox(
            graph_toolbar,
            textvariable=self.graph_anchor_var,
            state="readonly",
            values=tuple(f"A{anchor_id}" for anchor_id in range(1, MAX_ANCHORS + 1)),
            width=6,
        )
        graph_combo.pack(side=tk.LEFT, padx=(5, 14))
        graph_combo.bind("<<ComboboxSelected>>", lambda _event: self._draw_graph())
        ttk.Label(
            graph_toolbar,
            text=f"Tối đa {HISTORY_LENGTH} mẫu gần nhất · tự động scale theo dữ liệu",
            foreground="#64748b",
        ).pack(side=tk.LEFT)
        ttk.Button(
            graph_toolbar, text="Xóa lịch sử anchor này", command=self._clear_graph_history
        ).pack(side=tk.RIGHT)
        self.graph = tk.Canvas(
            graph_frame, background="#111827", highlightthickness=0
        )
        self.graph.pack(fill=tk.BOTH, expand=True)
        self.graph.bind("<Configure>", lambda _event: self._draw_graph())

        self.meas_panel = RangeMeasPanel(self.notebook, self._request_range_meas)
        self.notebook.add(self.meas_panel, text="  RANGE_MEAS  ")

        app_directory = application_directory()
        self.map_panel = AnchorMapPanel(self.notebook, app_directory)
        self.analysis_panel = AnalysisPanel(self.notebook)
        self.calibration_panel = CalibrationPanel(self.notebook)
        self.notebook.add(self.map_panel, text="  Bản đồ 2D / 3D  ")
        self.notebook.add(self.analysis_panel, text="  Phân tích  ")
        self.notebook.add(self.calibration_panel, text="  Calibration  ")

    def _toggle_telemetry_details(self) -> None:
        if self.telemetry_details.winfo_manager():
            self.telemetry_details.pack_forget()
            self.details_button.configure(text="Hiện chi tiết ▾")
        else:
            self.telemetry_details.pack(fill=tk.X)
            self.details_button.configure(text="Ẩn chi tiết ▴")

    def _clear_graph_history(self) -> None:
        anchor_id = int(self.graph_anchor_var.get()[1:])
        self.history.setdefault(anchor_id, deque(maxlen=HISTORY_LENGTH)).clear()
        self._draw_graph()

    def _insert_anchor_placeholders(self) -> None:
        for anchor_id in range(1, MAX_ANCHORS + 1):
            item = self.tree.insert(
                "", tk.END,
                values=(f"A{anchor_id}", "-", "NO DATA", "-", "-", "-", "-", "-"),
            )
            self.anchor_items[anchor_id] = item

    def refresh_ports(self) -> None:
        if self.worker is not None:
            return
        ports = sorted(list_ports.comports(), key=lambda item: item.device)
        self.port_labels = {
            f"{item.device} — {item.description}": item.device for item in ports
        }
        labels = list(self.port_labels)
        self.port_combo["values"] = labels
        if labels:
            current = self.port_var.get()
            if current not in labels:
                preferred = next(
                    (label for label in labels if "Bluetooth" not in label), labels[0]
                )
                self.port_var.set(preferred)
        else:
            self.port_var.set("")

    def connect(self) -> None:
        if self.worker is not None:
            self.disconnect()
            return

        if self.demo:
            self.worker = DemoWorker(self.events)
        else:
            label = self.port_var.get()
            port = self.port_labels.get(label, label.split(" ", 1)[0] if label else "")
            if not port:
                messagebox.showwarning(APP_TITLE, "Không tìm thấy cổng COM. Hãy cắm ESP32-C3 hoặc USB-UART rồi bấm Làm mới.")
                return
            try:
                baud = int(self.baud_var.get())
            except ValueError:
                messagebox.showerror(APP_TITLE, "Baud không hợp lệ")
                return
            self.worker = SerialWorker(
                port, baud, self.events,
                auto_enable_meas=self.meas_panel.auto_enable_var.get(),
            )

        self.last_range_sequence = None
        self.last_log_sequence = -1
        self.last_range_received = time.monotonic()
        self.data_stale = False
        self.sequence_loss = 0
        self.range_frame_count = 0
        self.host_filters.clear()
        self.latest_host_filtered.clear()
        for history in self.history.values():
            history.clear()
        for history in self.detailed_history.values():
            history.clear()
        self.frame_times.clear()
        self.latest_samples.clear()
        self.tree.delete(*self.tree.get_children())
        self.anchor_items.clear()
        self._insert_anchor_placeholders()
        self.info_var.set("Chưa nhận INFO")
        self.stats_var.set("Chưa nhận STATS")
        self.sequence_var.set("-")
        self.uptime_var.set("-")
        self.loss_var.set("0")
        self._update_parser_counters(ParserCounters(), 0.0)
        self._draw_graph()
        self.map_panel.reset_session()
        self.analysis_panel.reset_session()
        self.calibration_panel.discard_capture()
        self.meas_panel.reset_session()
        self.connect_button.configure(text="Ngắt kết nối")
        self.port_combo.configure(state="disabled")
        self.baud_combo.configure(state="disabled")
        self.worker.start()

    def disconnect(self) -> None:
        if self.worker is not None:
            self.worker.stop()

    def close(self) -> None:
        if self.worker is not None:
            self.worker.stop()
        self.stop_recording()
        self.root.after(80, self.root.destroy)

    def choose_log_directory(self) -> None:
        directory = filedialog.askdirectory(
            title="Chọn thư mục lưu phiên đo", initialdir=self.log_directory_var.get()
        )
        if directory:
            self.log_directory_var.set(directory)

    def open_log_directory(self) -> None:
        directory = self.last_session_directory or Path(self.log_directory_var.get())
        try:
            directory.mkdir(parents=True, exist_ok=True)
            os.startfile(directory)  # type: ignore[attr-defined]
        except OSError as exc:
            messagebox.showerror(APP_TITLE, f"Không mở được thư mục:\n{exc}")

    def toggle_recording(self) -> None:
        if self.recorder is None:
            self.start_recording()
        else:
            self.stop_recording(show_result=True)

    def start_recording(self) -> None:
        if self.worker is None:
            messagebox.showwarning(APP_TITLE, "Hãy kết nối UART trước khi bắt đầu ghi.")
            return
        source = "DEMO" if self.demo else self.port_labels.get(
            self.port_var.get(), self.port_var.get().split(" ", 1)[0]
        )
        try:
            recorder = SessionRecorder(
                Path(self.log_directory_var.get()), source, int(self.baud_var.get())
            )
        except (OSError, ValueError) as exc:
            messagebox.showerror(APP_TITLE, f"Không tạo được phiên ghi:\n{exc}")
            return
        self.recorder = recorder
        self.last_session_directory = recorder.session_directory
        try:
            save_layout(recorder.session_directory / "anchor_layout.json", self.map_panel.layout)
        except OSError as exc:
            recorder.record_event(f"ANCHOR LAYOUT SNAPSHOT ERROR: {exc}")
        self.recording_error_shown = False
        self.record_button.configure(text="Dừng và lưu")
        self.recording_var.set(f"Đang ghi: {recorder.session_directory.name}")
        self.record_count_var.set("0 frame / 0 mẫu")
        self._append_log(f"RECORD START {recorder.session_directory}")

    def stop_recording(self, show_result: bool = False) -> None:
        recorder = self.recorder
        if recorder is None:
            return
        self._append_log("RECORD STOP")
        self.recorder = None
        recorder.stop()
        self.record_button.configure(text="Bắt đầu ghi")
        self.last_session_directory = recorder.session_directory
        self.record_count_var.set(self._recording_counts(recorder))
        if recorder.error:
            self.recording_var.set(f"Lỗi ghi: {recorder.error}")
            if not self.recording_error_shown:
                self.recording_error_shown = True
                messagebox.showerror(APP_TITLE, f"Lỗi ghi dữ liệu:\n{recorder.error}")
        else:
            self.recording_var.set(f"Đã lưu: {recorder.session_directory}")
            if show_result:
                meas = (f"\n{recorder.meas_records} bản ghi RANGE_MEAS (meas.csv)."
                        if recorder.meas_records else "")
                messagebox.showinfo(
                    APP_TITLE,
                    f"Đã lưu {recorder.range_frames} frame RANGE "
                    f"({recorder.range_samples} mẫu).{meas}\n\n{recorder.session_directory}",
                )

    def _poll_events(self) -> None:
        """Keep the Tk polling heartbeat alive even if one view fails."""
        try:
            self._poll_events_once()
        except Exception as exc:  # Tk callbacks otherwise stop permanently
            message = f"GUI UPDATE ERROR: {type(exc).__name__}: {exc}"
            if message != self._last_ui_error:
                self._last_ui_error = message
                self._append_log(message)
        finally:
            try:
                if self.root.winfo_exists():
                    self.root.after(UI_POLL_MS, self._poll_events)
            except tk.TclError:
                pass

    def _poll_events_once(self) -> None:
        latest_range: RangeMessage | None = None
        try:
            for _ in range(500):
                event, payload = self.events.get_nowait()
                if event == "connected":
                    self.connection_var.set(f"Đã mở {payload}; đang chờ TAG trả lời")
                    self.connection_label.configure(style="Pending.TLabel")
                    self._append_log(f"OPEN {payload}")
                elif event == "tag_online":
                    self.connection_var.set(f"TAG online: {payload}")
                    self.connection_label.configure(style="Connected.TLabel")
                elif event == "notice":
                    self._append_log(str(payload))
                elif event == "disconnected":
                    latest_range = None
                    self.analysis_panel.cancel_capture(
                        "Mất kết nối; phiên lấy mẫu đã được hủy."
                    )
                    self._mark_stale()
                    self.connection_var.set("Đã ngắt kết nối")
                    self.connection_label.configure(style="Disconnected.TLabel")
                    self.worker = None
                    self.connect_button.configure(text="Kết nối")
                    self.port_combo.configure(state="readonly")
                    self.baud_combo.configure(state="readonly")
                    self._append_log(f"CLOSE {payload}")
                    if self.recorder is not None:
                        self.stop_recording()
                elif event == "error":
                    self._append_log(f"SERIAL ERROR: {payload}")
                    messagebox.showerror(APP_TITLE, f"Lỗi UART:\n{payload}")
                elif event == "protocol_error":
                    self._append_log(f"PROTOCOL: {payload}")
                elif event == "raw_frame":
                    if self.recorder is not None:
                        self.recorder.record_raw(payload)
                elif event == "parser":
                    counters, byte_rate = payload
                    if self.recorder is not None:
                        self.recorder.record_uart_stats(counters, byte_rate)
                    self._update_parser_counters(counters, byte_rate)
                elif event == "message":
                    # RANGE_MEAS is the most frequent message (one per measurement).
                    if isinstance(payload, RangeMeasMessage):
                        self.meas_panel.ingest(payload, time.monotonic())
                        if self.recorder is not None:
                            self.recorder.record_message(payload)
                    elif isinstance(payload, RangeMessage):
                        latest_range = payload
                        host_filtered = self._record_history(payload)
                        if self.recorder is not None:
                            self.recorder.record_message(payload, host_filtered)
                    elif isinstance(payload, InfoMessage):
                        if self.recorder is not None:
                            self.recorder.record_message(payload)
                        self._update_info(payload)
                    elif isinstance(payload, StatsMessage):
                        if self.recorder is not None:
                            self.recorder.record_message(payload)
                        self._update_stats(payload)
                    elif isinstance(payload, DeviceInfoMessage):
                        self._update_device_info(payload)
                    elif isinstance(payload, DiagSystemMessage):
                        self._update_diag_system(payload)
                    elif isinstance(payload, CmdAckMessage):
                        if payload.command_id == CMD_SET_TELEMETRY:
                            self._handle_telemetry_ack(payload)
        except queue.Empty:
            pass

        if latest_range is not None:
            self._update_range(latest_range)
            self._draw_graph()
            now = time.monotonic()
            if now - self.last_advanced_refresh >= 0.20:
                self.map_panel.update_data(self.latest_samples, self.latest_host_filtered)
                self.last_advanced_refresh = now
        if (self.worker is not None and self.last_range_received is not None
                and time.monotonic() - self.last_range_received > 1.0):
            self._mark_stale()
        now = time.monotonic()
        if ((self.worker is not None or self.meas_panel.tracker.total)
                and now - self.last_meas_refresh >= MEAS_REFRESH_S):
            self.meas_panel.refresh(now)
            self.last_meas_refresh = now
        self._refresh_recording_status()

    @staticmethod
    def _recording_counts(recorder: SessionRecorder) -> str:
        text = f"{recorder.range_frames} frame / {recorder.range_samples} mẫu"
        if recorder.meas_records:
            text += f" / {recorder.meas_records} meas"
        return text

    def _refresh_recording_status(self) -> None:
        recorder = self.recorder
        if recorder is None:
            return
        self.record_count_var.set(self._recording_counts(recorder))
        if recorder.error is not None or not recorder.active:
            self.stop_recording()

    def _mark_stale(self) -> None:
        if self.data_stale:
            return
        self.data_stale = True
        for anchor_id, item in self.anchor_items.items():
            self.tree.item(
                item,
                values=(f"A{anchor_id}", "NO", "STALE", "-", "-", "-", "-", "-"),
                tags=("error",),
            )
        self._append_log("STALE: không có RANGE mới; các giá trị trước đó đã hết hạn.")
        self.map_panel.update_data({}, {})

    def _update_parser_counters(self, counters: ParserCounters, byte_rate: float) -> None:
        self.frame_var.set(str(counters.valid_frames))
        self.crc_var.set(str(counters.crc_errors))
        self.discarded_var.set(str(counters.discarded_bytes))
        self.rate_var.set(f"{byte_rate:,.0f} B/s")

    def _update_device_info(self, message: DeviceInfoMessage) -> None:
        self.meas_panel.set_device_info(message.telemetry_features, message.uart_baud)
        dirty = "+dirty" if message.git_dirty else ""
        self._append_log(
            f"DEVICE git={message.git_hash:08x}{dirty} cfg={message.build_config_hash:08x} "
            f"cal_profile={message.calibration_profile_id:08x} proto=v{message.frame_version} "
            f"boot={message.boot_count} txpwr=0x{message.tx_power_register:08X} "
            f"ant={message.tx_antenna_delay}/{message.rx_antenna_delay} "
            f"xtal_otp=0x{message.otp_xtal_trim:02X} baud={message.uart_baud} "
            f"telem=0x{message.telemetry_features:02X}"
        )

    def _request_range_meas(self, enable: bool) -> None:
        """RANGE_MEAS on/off for this session (the RANGE_MEAS tab button)."""
        worker = self.worker
        if worker is None:
            messagebox.showwarning(APP_TITLE, "Hãy kết nối UART trước khi bật/tắt RANGE_MEAS.")
            return
        features = self.meas_panel.features
        if features is None:
            features = TELEM_FEATURE_RANGE_SNAPSHOT | TELEM_FEATURE_DIAG
        # The Live tab and range.csv are built from the snapshot: keep it on.
        features |= TELEM_FEATURE_RANGE_SNAPSHOT
        if enable:
            features |= TELEM_FEATURE_RANGE_MEAS
            self.meas_panel.tracker.restart_sequence()
        else:
            features &= ~TELEM_FEATURE_RANGE_MEAS
        worker.request_command(CMD_SET_TELEMETRY, bytes((features,)))
        self._append_log(
            f"SET_TELEMETRY 0x{features:02X}: RANGE_MEAS {'ON' if enable else 'OFF'} "
            "cho phiên này"
        )

    def _handle_telemetry_ack(self, ack: CmdAckMessage) -> None:
        features = f"0x{ack.data[0]:02X}" if ack.data else "-"
        if ack.data:
            self.meas_panel.set_features(ack.data[0])
        if ack.ok:
            self._append_log(f"TELEMETRY features={features}")
        else:
            self._append_log(
                f"SET_TELEMETRY bị từ chối: {ack.result_name} (features={features})"
            )

    def _update_diag_system(self, message: DiagSystemMessage) -> None:
        self.meas_panel.set_tag_counters(
            message.meas_queue_drops, message.uart_tx_overflow, message.uart_high_water)
        now = time.monotonic()
        problems = (message.fault_hold or message.spi_errors or message.cycle_overruns
                    or message.config_mismatches or message.uart_tx_overflow)
        if not problems and now - self.last_diag_log < 10.0:
            return
        self.last_diag_log = now
        temp = "-" if message.temperature_c is None else f"{message.temperature_c:.1f}C"
        self._append_log(
            f"DIAG cycle={message.cycle_us}us max={message.cycle_max_us}us "
            f"overrun={message.cycle_overruns} recover={message.radio_recoveries} "
            f"spi_err={message.spi_errors} cfg_mismatch={message.config_mismatches} "
            f"rx_lde={message.rx_errors.get('lde', 0)} temp={temp} "
            f"fault_hold={int(message.fault_hold)} locked={int(message.locked)}"
        )

    def _update_info(self, message: InfoMessage) -> None:
        self.info_flags = message.flags
        mode = "DS-TWR" if message.ranging_mode else "SS-TWR"
        self.info_var.set(
            f"Schema {message.schema} | {mode} | anchors={message.anchor_count} | "
            f"cal_mask=0x{message.calibrated_mask:02X} | filter={message.filter_mode} | "
            f"PHY={message.phy_profile_id} | SPI={message.spi_clock_mhz} MHz"
        )
        self._append_log(
            f"INFO mode={mode} anchors={message.anchor_count} cal=0x{message.calibrated_mask:02X}"
        )

    def _update_stats(self, message: StatsMessage) -> None:
        self.stats_var.set(
            f"poll={message.poll_count} ok={message.response_ok_count} "
            f"timeout={message.rx_timeout_count} rxerr={message.rx_error_count} "
            f"overrun={message.cycle_overrun_count} uart_ovf={message.uart_overflow_count} "
            f"cycle={message.cycle_hz} Hz ops={message.operation_hz} Hz"
        )
        self._append_log(
            f"STATS ok={message.response_ok_count} timeout={message.rx_timeout_count} "
            f"cycle={message.cycle_hz}Hz uart_ovf={message.uart_overflow_count}"
        )

    def _record_history(self, message: RangeMessage) -> dict[int, int]:
        received_s = time.monotonic()
        self.last_range_received = received_s
        self.data_stale = False
        if self.last_range_sequence is not None:
            delta = (message.sequence - self.last_range_sequence) & 0xFFFFFFFF
            if 1 < delta < 0x80000000:
                self.sequence_loss += delta - 1
            elif delta >= 0x80000000:
                self._append_log("TAG sequence đã quay lại: reset lịch sử phiên đo.")
                self.last_log_sequence = -1
                for history in self.history.values():
                    history.clear()
                for history in self.detailed_history.values():
                    history.clear()
                self.frame_times.clear()
                self.host_filters.clear()
                self.map_panel.reset_session()
                self.analysis_panel.cancel_capture(
                    "TAG đã reset; phiên lấy mẫu đã được hủy để không trộn hai phiên."
                )
        self.last_range_sequence = message.sequence
        self.range_frame_count += 1
        self.loss_var.set(str(self.sequence_loss))
        self.frame_times.append(received_s)

        host_filtered: dict[int, int] = {}
        for sample in message.samples:
            diagnostic_range = sample.status & (
                STATUS_CALIBRATION_MISSING | STATUS_RANGE_REJECT
            )
            if sample.valid or diagnostic_range:
                host_value = self.host_filters.setdefault(
                    sample.anchor_id, HostRangeFilter()
                ).update(
                    sample.raw_mm,
                    fpp_to_legacy_scale_cdbm(sample.fpp_cdbm, self.info_flags),
                    message.time_ms,
                )
                host_filtered[sample.anchor_id] = host_value
                self.history.setdefault(sample.anchor_id, deque(maxlen=HISTORY_LENGTH)).append(
                    (sample.raw_mm, sample.filtered_mm if sample.valid else None, host_value)
                )
            host_value = host_filtered.get(sample.anchor_id)
            point = TelemetryPoint(
                received_s=received_s,
                tag_time_ms=message.time_ms,
                valid=sample.valid,
                status=sample.status,
                age_ms=sample.age_ms,
                raw_mm=sample.raw_mm,
                firmware_mm=sample.filtered_mm if sample.valid and sample.filtered_mm > 0 else None,
                host_mm=host_value,
                fpp_dbm=(sample.fpp_cdbm / 100.0) if (sample.valid or diagnostic_range) else None,
            )
            self.detailed_history.setdefault(
                sample.anchor_id, deque(maxlen=ANALYSIS_HISTORY_LENGTH)
            ).append(point)
            self.analysis_panel.ingest(sample, point)
            self.calibration_panel.ingest(sample, point)
        self.latest_host_filtered = host_filtered
        self.latest_samples = {sample.anchor_id: sample for sample in message.samples}
        return host_filtered

    def _update_range(self, message: RangeMessage) -> None:
        self.sequence_var.set(str(message.sequence))
        self.uptime_var.set(self._format_uptime(message.time_ms))
        present = {sample.anchor_id for sample in message.samples}
        for anchor_id, item in self.anchor_items.items():
            if anchor_id not in present:
                self.tree.item(
                    item,
                    values=(f"A{anchor_id}", "NO", "MISSING", "-", "-", "-", "-", "-"),
                    tags=("error",),
                )
        for sample in message.samples:
            self._update_anchor_row(sample)

        if message.sequence - self.last_log_sequence >= 25:
            a1 = next((sample for sample in message.samples if sample.anchor_id == 1), None)
            if a1 is not None:
                self._append_log(
                    f"RANGE seq={message.sequence} A1={a1.raw_mm / 1000.0:.3f}m "
                    f"valid={int(a1.valid)} status={'|'.join(status_names(a1.status))}"
                )
            self.last_log_sequence = message.sequence

    def _update_anchor_row(self, sample: AnchorSample) -> None:
        item = self.anchor_items.get(sample.anchor_id)
        if item is None:
            item = self.tree.insert("", tk.END)
            self.anchor_items[sample.anchor_id] = item

        status_text = " | ".join(status_names(sample.status))
        calibration_missing = bool(sample.status & STATUS_CALIBRATION_MISSING)
        diagnostic = bool(
            sample.status & (STATUS_CALIBRATION_MISSING | STATUS_RANGE_REJECT)
        )
        raw_text = f"{sample.raw_mm / 1000.0:.3f}" if sample.valid or diagnostic else "-"
        filtered_text = f"{sample.filtered_mm / 1000.0:.3f}" if sample.valid else "-"
        host_value = self.latest_host_filtered.get(sample.anchor_id)
        host_filtered_text = f"{host_value / 1000.0:.3f}" if host_value is not None else "-"
        age_text = "N/A" if calibration_missing and not sample.valid else str(sample.age_ms)
        fpp_text = (
            f"{sample.fpp_cdbm / 100.0:.2f}" if sample.valid or diagnostic else "-"
        )
        tag = "valid" if sample.valid else "cal" if diagnostic else "error"
        self.tree.item(
            item,
            values=(
                f"A{sample.anchor_id}",
                "YES" if sample.valid else "NO",
                status_text,
                age_text,
                raw_text,
                filtered_text,
                host_filtered_text,
                fpp_text,
            ),
            tags=(tag,),
        )

    def _draw_graph(self) -> None:
        canvas = self.graph
        canvas.delete("all")
        width = max(canvas.winfo_width(), 100)
        height = max(canvas.winfo_height(), 100)
        left, top, right, bottom = 58, 14, width - 16, height - 28
        anchor_id = int(self.graph_anchor_var.get()[1:])
        values = list(self.history.get(anchor_id, ()))

        canvas.create_rectangle(left, top, right, bottom, outline="#374151")
        if len(values) < 2:
            canvas.create_text(
                width / 2,
                height / 2,
                text=f"Đang chờ dữ liệu A{anchor_id}",
                fill="#9ca3af",
                font=("Segoe UI", 11),
            )
            return

        raw_values = [raw for raw, _firmware, _host in values]
        firmware_values = [
            firmware for _raw, firmware, _host in values if firmware is not None
        ]
        host_values = [host for _raw, _firmware, host in values]
        scale_values = raw_values + firmware_values + host_values
        minimum = min(scale_values)
        maximum = max(scale_values)
        padding = max((maximum - minimum) * 0.15, 50)
        y_min = minimum - padding
        y_max = maximum + padding
        span = max(y_max - y_min, 1)

        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = bottom - fraction * (bottom - top)
            value_m = (y_min + fraction * span) / 1000.0
            canvas.create_line(left, y, right, y, fill="#263244")
            canvas.create_text(left - 7, y, text=f"{value_m:.2f}", fill="#9ca3af", anchor=tk.E)

        count = len(values)

        def draw_series(series: list[int | None], color: str, width_px: float, tag: str) -> None:
            segment: list[float] = []
            for index, value in enumerate(series):
                if value is None:
                    if len(segment) >= 4:
                        canvas.create_line(
                            *segment, fill=color, width=width_px, smooth=False, tags=(tag,)
                        )
                    segment = []
                    continue
                x = left + index * (right - left) / max(count - 1, 1)
                y = bottom - (value - y_min) * (bottom - top) / span
                segment.extend((x, y))
            if len(segment) >= 4:
                canvas.create_line(
                    *segment, fill=color, width=width_px, smooth=False, tags=(tag,)
                )

        draw_series(
            [raw for raw, _firmware, _host in values], "#94a3b8", 1.5, "raw_series"
        )
        draw_series(
            [host for _raw, _firmware, host in values], "#22c55e", 2.5, "host_series"
        )
        draw_series(
            [firmware for _raw, firmware, _host in values],
            "#38bdf8", 2.0, "firmware_series",
        )

        canvas.create_line(left + 8, top + 10, left + 28, top + 10, fill="#94a3b8", width=2)
        canvas.create_text(left + 34, top + 10, text="Raw", fill="#cbd5e1", anchor=tk.W)
        canvas.create_line(left + 78, top + 10, left + 98, top + 10, fill="#22c55e", width=3)
        canvas.create_text(left + 104, top + 10, text="Host Filter", fill="#86efac", anchor=tk.W)
        canvas.create_line(left + 180, top + 10, left + 200, top + 10, fill="#38bdf8", width=2)
        canvas.create_text(left + 206, top + 10, text="FW Filter", fill="#7dd3fc", anchor=tk.W)

        latest_raw, latest_firmware, latest_host = values[-1]
        latest_text = f"Raw {latest_raw / 1000.0:.3f} m"
        latest_text += f" | Host {latest_host / 1000.0:.3f} m"
        latest_text += (
            f" | FW {latest_firmware / 1000.0:.3f} m"
            if latest_firmware is not None else " | FW --"
        )
        canvas.create_text(
            right,
            top + 4,
            text=f"A{anchor_id}: {latest_text}",
            fill="#e5e7eb",
            anchor=tk.NE,
            font=("Consolas", 10, "bold"),
        )
        canvas.create_text(left, bottom + 16, text=f"{count} mẫu gần nhất", fill="#9ca3af", anchor=tk.W)

    def _append_log(self, line: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {line}\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > 100:
            self.log_text.delete("1.0", f"{line_count - 100}.0")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)
        if self.recorder is not None:
            self.recorder.record_event(line)

    @staticmethod
    def _format_uptime(time_ms: int) -> str:
        seconds = time_ms // 1000
        hours, remainder = divmod(seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}.{time_ms % 1000:03d}"


def main() -> int:
    parser = argparse.ArgumentParser(description=APP_TITLE)
    parser.add_argument("--demo", action="store_true", help="run with simulated telemetry")
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    args = parser.parse_args()

    root = tk.Tk()
    if args.smoke_test:
        root.withdraw()
    app = UwbGui(root, demo=args.demo or args.smoke_test)
    if args.smoke_test:
        root.after(1500, app.close)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
