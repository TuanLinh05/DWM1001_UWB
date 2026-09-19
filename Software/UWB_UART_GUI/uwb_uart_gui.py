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
    InfoMessage,
    ParserCounters,
    ProtocolError,
    RangeMessage,
    StatsMessage,
    STATUS_CALIBRATION_MISSING,
    STATUS_RANGE_REJECT,
    TelemetryStreamParser,
    decode_frame,
    encode_frame,
    status_names,
    TYPE_INFO,
    TYPE_RANGE,
    TYPE_STATS,
)
from session_recorder import SessionRecorder
from host_range_filter import HostRangeFilter
from gui_analysis import MAX_ANCHORS, TelemetryPoint, save_layout
from gui_views import AnalysisPanel, AnchorMapPanel, CalibrationPanel


APP_TITLE = "DWM1001 UWB Ground Control"
DEFAULT_BAUD = 115200
UI_POLL_MS = 40
HISTORY_LENGTH = 300
ANALYSIS_HISTORY_LENGTH = 15000


def application_directory() -> Path:
    """Return a persistent directory for source and PyInstaller one-file builds."""
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


class SerialWorker(threading.Thread):
    def __init__(self, port: str, baud: int, events: queue.Queue) -> None:
        super().__init__(name="uwb-serial-reader", daemon=True)
        self.port = port
        self.baud = baud
        self.events = events
        self.stop_event = threading.Event()
        self._serial: serial.Serial | None = None

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

            while not self.stop_event.is_set():
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


class DemoWorker(threading.Thread):
    """End-to-end protocol demo used when hardware is not connected."""

    def __init__(self, events: queue.Queue) -> None:
        super().__init__(name="uwb-demo-source", daemon=True)
        self.events = events
        self.stop_event = threading.Event()

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        parser = TelemetryStreamParser()
        sequence = 0
        start = time.monotonic()
        self.events.put(("connected", "DEMO"))

        info_payload = bytes((2, 0x05, 1, 4, 0x0F, 1, 1, 8, 0, 0))
        for anchor_id in range(1, 5):
            info_payload += struct.pack("<Hi", anchor_id, 0)
        self._emit(parser, encode_frame(TYPE_INFO, 0, 0, info_payload))

        while not self.stop_event.wait(0.02):
            sequence += 1
            elapsed = time.monotonic() - start
            payload = bytearray((4,))
            tag_x = 2.5 + 1.5 * math.cos(elapsed * 0.35)
            tag_y = 2.0 + 1.1 * math.sin(elapsed * 0.48)
            demo_anchors = ((0.0, 0.0), (5.0, 0.0), (2.5, 4.0), (5.0, 4.0))
            for anchor_id, (anchor_x, anchor_y) in enumerate(demo_anchors, start=1):
                ideal_mm = math.hypot(tag_x - anchor_x, tag_y - anchor_y) * 1000.0
                raw_mm = int(ideal_mm + 12.0 * math.sin(elapsed * 3.1 + anchor_id))
                filtered_mm = int(ideal_mm + 3.0 * math.sin(elapsed * 1.4 + anchor_id))
                valid = 1
                status = 0
                payload.extend(
                    struct.pack("<HBBHiih", anchor_id, valid, status, 5, raw_mm, filtered_mm, -7850-anchor_id*35)
                )
            frame = encode_frame(TYPE_RANGE, sequence, int(elapsed * 1000), bytes(payload))
            self._emit(parser, frame[:7])
            self._emit(parser, frame[7:])

            if sequence % 50 == 0:
                stats_payload = struct.pack(
                    "<IIIIIIHH", sequence * 4, sequence, 0, 0, 0, 0, 50, 50
                )
                self._emit(
                    parser,
                    encode_frame(TYPE_STATS, sequence, int(elapsed * 1000), stats_payload),
                )
                self.events.put(("parser", (parser.counters.snapshot(), 4050.0)))

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
        self.last_analysis_refresh = 0.0
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

        recording = ttk.LabelFrame(outer, text="Thu và lưu dữ liệu", padding=9)
        recording.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(recording, text="Thư mục:").pack(side=tk.LEFT)
        ttk.Entry(recording, textvariable=self.log_directory_var, width=54).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 6)
        )
        ttk.Button(recording, text="Chọn...", command=self.choose_log_directory).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(recording, text="Mở thư mục", command=self.open_log_directory).pack(
            side=tk.LEFT, padx=(0, 10)
        )
        self.record_button = ttk.Button(
            recording, text="Bắt đầu ghi", command=self.toggle_recording
        )
        self.record_button.pack(side=tk.RIGHT)
        recording_state = ttk.Frame(outer)
        recording_state.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(recording_state, textvariable=self.recording_var).pack(side=tk.LEFT)
        ttk.Label(
            recording_state, textvariable=self.record_count_var, style="Value.TLabel"
        ).pack(side=tk.RIGHT)

        summary = ttk.Frame(outer)
        summary.pack(fill=tk.X, pady=(0, 8))
        for title, variable in (
            ("Frame hợp lệ", self.frame_var),
            ("CRC lỗi", self.crc_var),
            ("Byte bỏ qua", self.discarded_var),
            ("Tốc độ UART", self.rate_var),
            ("Mất sequence", self.loss_var),
        ):
            card = ttk.LabelFrame(summary, text=title, padding=(10, 5))
            card.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
            ttk.Label(card, textvariable=variable, style="Value.TLabel").pack()

        info = ttk.LabelFrame(outer, text="Firmware / thống kê", padding=8)
        info.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(info, textvariable=self.info_var, style="Value.TLabel").pack(anchor=tk.W)
        ttk.Label(info, textvariable=self.stats_var, style="Value.TLabel").pack(anchor=tk.W, pady=(3, 0))
        state_line = ttk.Frame(info)
        state_line.pack(fill=tk.X, pady=(3, 0))
        ttk.Label(state_line, text="Sequence:").pack(side=tk.LEFT)
        ttk.Label(state_line, textvariable=self.sequence_var, style="Value.TLabel").pack(side=tk.LEFT, padx=(4, 18))
        ttk.Label(state_line, text="Tag uptime:").pack(side=tk.LEFT)
        ttk.Label(state_line, textvariable=self.uptime_var, style="Value.TLabel").pack(side=tk.LEFT, padx=4)

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

        app_directory = application_directory()
        self.map_panel = AnchorMapPanel(self.notebook, app_directory)
        self.analysis_panel = AnalysisPanel(self.notebook)
        self.calibration_panel = CalibrationPanel(self.notebook)
        self.notebook.add(self.map_panel, text="  Bản đồ 2D / 3D  ")
        self.notebook.add(self.analysis_panel, text="  Phân tích  ")
        self.notebook.add(self.calibration_panel, text="  Calibration  ")

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
            self.worker = SerialWorker(port, baud, self.events)

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
        self.analysis_panel.refresh(self.detailed_history, self.frame_times)
        self.calibration_panel.discard_capture()
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
        self.record_count_var.set(
            f"{recorder.range_frames} frame / {recorder.range_samples} mẫu"
        )
        if recorder.error:
            self.recording_var.set(f"Lỗi ghi: {recorder.error}")
            if not self.recording_error_shown:
                self.recording_error_shown = True
                messagebox.showerror(APP_TITLE, f"Lỗi ghi dữ liệu:\n{recorder.error}")
        else:
            self.recording_var.set(f"Đã lưu: {recorder.session_directory}")
            if show_result:
                messagebox.showinfo(
                    APP_TITLE,
                    f"Đã lưu {recorder.range_frames} frame RANGE "
                    f"({recorder.range_samples} mẫu).\n\n{recorder.session_directory}",
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
                    self.connection_var.set(f"Đã kết nối: {payload}")
                    self.connection_label.configure(style="Connected.TLabel")
                    self._append_log(f"OPEN {payload}")
                elif event == "disconnected":
                    latest_range = None
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
                    if isinstance(payload, RangeMessage):
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
        except queue.Empty:
            pass

        if latest_range is not None:
            self._update_range(latest_range)
            self._draw_graph()
            now = time.monotonic()
            if now - self.last_advanced_refresh >= 0.20:
                self.map_panel.update_data(self.latest_samples, self.latest_host_filtered)
                self.last_advanced_refresh = now
            if now - self.last_analysis_refresh >= 0.50:
                self.analysis_panel.refresh(self.detailed_history, self.frame_times)
                self.last_analysis_refresh = now
        if (self.worker is not None and self.last_range_received is not None
                and time.monotonic() - self.last_range_received > 1.0):
            self._mark_stale()
        self._refresh_recording_status()

    def _refresh_recording_status(self) -> None:
        recorder = self.recorder
        if recorder is None:
            return
        self.record_count_var.set(
            f"{recorder.range_frames} frame / {recorder.range_samples} mẫu"
        )
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

    def _update_info(self, message: InfoMessage) -> None:
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
                ).update(sample.raw_mm, sample.fpp_cdbm, message.time_ms)
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
