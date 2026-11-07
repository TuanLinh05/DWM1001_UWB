"""High-Speed PyQt6 UWB Positioning & Calibration Studio (`New_GUI`).

Designed specifically to eliminate the visual clutter, clipped labels, and
redraw lag of the legacy Tkinter GUI while adding full support for current
firmware (`RANGE_BURST` 0x18 at 50-300 Hz, uncalibrated `CAL_MISSING` fallback,
1D & 2D/3D spot calibration, `SET_DS_CAL`/`GET_DS_CAL`/`SAVE_SETTINGS`, and
2D/3D accuracy evaluation).
"""

from __future__ import annotations

from collections import deque
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QDoubleValidator, QFont, QFontDatabase, QIntValidator
from PyQt6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSlider,
    QSplitter,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

NEW_GUI_DIR = Path(__file__).resolve().parent
OLD_GUI_DIR = NEW_GUI_DIR.parent / "UWB_UART_GUI"
if str(NEW_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(NEW_GUI_DIR))
if str(OLD_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(OLD_GUI_DIR))

import telemetry_protocol as tp  # noqa: E402

from diag_rates import format_rates  # noqa: E402
from up4000_panel import Up4000Panel, Up4000StatusListener  # noqa: E402
from core_state import (  # noqa: E402
    AnchorConfig,
    CalibrationAnchorResult,
    DEFAULT_PRESET_BOX8,
    DEFAULT_PRESET_FLAT4,
    DEFAULT_PRESET_RIG40,
    DEFAULT_PRESET_ROOM_66_36,
    DopMetrics,
    LoadedLogSession,
    PositionResult,
    UwbStateEngine,
    load_anchor_layout_json,
    load_anchors_rig_txt,
    load_log_session,
    save_anchor_layout_json,
    save_anchors_rig_txt,
)

LAST_RIG_PATH = NEW_GUI_DIR / "last_rig_config.json"
from design_tokens import (  # noqa: E402
    ANCHOR_PALETTE,
    FONT_MONO,
    SPACING_MD,
    SPACING_SM,
    SPACING_XS,
    ThemeManager,
)
from serial_worker import (  # noqa: E402
    BURST_PRESETS,
    DemoBurstWorker,
    FastCsvLogger,
    LogReplayWorker,
    SerialTelemetryWorker,
    UdpTelemetryWorker,
    UDP_RELAY_PREFIX,
    DEFAULT_UDP_RELAY_PORT,
    available_serial_ports,
)
from widgets_canvas import FastRollingChart, PositionMapCanvas  # noqa: E402
from multi_spot_calibration import suggest_calibration_spots  # noqa: E402


class FloatInput(QLineEdit):
    """Clean numeric float input box that renders crisply across all QSS themes without spinbox clipping."""

    valueChanged = pyqtSignal(float)

    def __init__(self, min_val: float, max_val: float, value: float, decimals: int = 3, min_width: int = 68) -> None:
        super().__init__()
        self._min = min_val
        self._max = max_val
        self._decimals = decimals
        self._val = float(value)
        self.setValidator(QDoubleValidator(min_val, max_val, decimals, self))
        self.setMinimumWidth(min_width)
        self.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.setFont(QFont(FONT_MONO, 9))
        self.setText(f"{self._val:.{self._decimals}f}")
        self.editingFinished.connect(self._on_commit)

    def _on_commit(self) -> None:
        try:
            v = float(self.text().strip())
        except ValueError:
            v = self._val
        v = max(self._min, min(self._max, v))
        self._val = v
        self.setText(f"{self._val:.{self._decimals}f}")
        self.valueChanged.emit(self._val)

    def value(self) -> float:
        try:
            return max(self._min, min(self._max, float(self.text().strip())))
        except ValueError:
            return self._val

    def setValue(self, val: float) -> None:  # noqa: N802
        self._val = max(self._min, min(self._max, float(val)))
        self.setText(f"{self._val:.{self._decimals}f}")
        self.valueChanged.emit(self._val)


class IntInput(QLineEdit):
    """Clean integer input box that renders crisply across all QSS themes."""

    valueChanged = pyqtSignal(int)

    def __init__(self, min_val: int, max_val: int, value: int, min_width: int = 56) -> None:
        super().__init__()
        self._min = int(min_val)
        self._max = int(max_val)
        self._val = int(value)
        self.setValidator(QIntValidator(self._min, self._max, self))
        self.setMinimumWidth(min_width)
        self.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.setFont(QFont(FONT_MONO, 9))
        self.setText(str(self._val))
        self.editingFinished.connect(self._on_commit)

    def _on_commit(self) -> None:
        try:
            v = int(self.text().strip())
        except ValueError:
            v = self._val
        v = max(self._min, min(self._max, v))
        self._val = v
        self.setText(str(self._val))
        self.valueChanged.emit(self._val)

    def value(self) -> int:
        try:
            return max(self._min, min(self._max, int(self.text().strip())))
        except ValueError:
            return self._val

    def setValue(self, val: int) -> None:  # noqa: N802
        self._val = max(self._min, min(self._max, int(val)))
        self.setText(str(self._val))
        self.valueChanged.emit(self._val)


def _make_double_spin(
    min_val: float,
    max_val: float,
    value: float,
    decimals: int = 3,
    min_width: int = 68,
) -> FloatInput:
    return FloatInput(min_val, max_val, value, decimals=decimals, min_width=min_width)


def _make_int_spin(min_val: int, max_val: int, value: int, min_width: int = 56) -> IntInput:
    return IntInput(min_val, max_val, value, min_width=min_width)


class StatCard(QFrame):
    """Compact telemetry KPI card following the 4px design system."""

    def __init__(self, title: str, initial_value: str = "--", subtext: str = "", parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("stat_card")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(3)

        self._title_lbl = QLabel(title.upper())
        self._title_lbl.setObjectName("card_title")

        self._value_lbl = QLabel(initial_value)
        self._value_lbl.setObjectName("metric_value")

        self._sub_lbl = QLabel(subtext)
        self._sub_lbl.setObjectName("metric_sub")

        layout.addWidget(self._title_lbl)
        layout.addWidget(self._value_lbl)
        layout.addWidget(self._sub_lbl)

    def update_metric(self, value: str, subtext: Optional[str] = None) -> None:
        if self._value_lbl.text() != value:
            self._value_lbl.setText(value)
        if subtext is not None and self._sub_lbl.text() != subtext:
            self._sub_lbl.setText(subtext)


class UwbFastGuiWindow(QMainWindow):
    """Streamlined, high-speed UWB 2D/3D Positioning & Calibration Studio."""

    def __init__(
        self,
        start_demo: bool = False,
        initial_port: Optional[str] = None,
        initial_baud: int = 460800,
    ) -> None:
        super().__init__()
        self.setWindowTitle("UWB Fast Studio — 2D/3D Positioning & High-Speed Calibration")
        self.resize(1540, 920)
        self.setMinimumSize(1260, 760)

        app = QApplication.instance()
        self.theme_manager = ThemeManager(app) if app is not None else None
        if self.theme_manager is not None:
            self.theme_manager.apply_light()

        initial_rig = DEFAULT_PRESET_BOX8 if start_demo else DEFAULT_PRESET_ROOM_66_36
        if not start_demo and LAST_RIG_PATH.is_file():
            try:
                loaded = load_anchor_layout_json(LAST_RIG_PATH)
                if any(a.enabled for a in loaded):
                    initial_rig = loaded
            except Exception:
                pass

        self.engine = UwbStateEngine(initial_anchors=initial_rig)
        self.replay_engine = UwbStateEngine(initial_anchors=initial_rig)
        self.loaded_session: Optional[LoadedLogSession] = None
        self.replay_playing: bool = False
        self.replay_frame_idx: int = 0
        self.replay_speed: float = 1.0
        self._replay_accum_f: float = 0.0
        self.serial_worker: Optional[SerialTelemetryWorker] = None
        self.demo_worker: Optional[DemoBurstWorker] = None
        self.csv_logger: Optional[FastCsvLogger] = None
        self._replay_loader: Optional[LogReplayWorker] = None
        self._event_log: List[str] = []

        self._editing_anchor_table = False
        self._build_ui()
        self._populate_anchor_config_table()
        self.map_canvas.reset_camera()
        self._refresh_ports(select_port=initial_port)

        idx_baud = self.baud_combo.findText(str(initial_baud))
        if idx_baud >= 0:
            self.baud_combo.setCurrentIndex(idx_baud)

        # Decoupled 30 Hz UI refresh timer (33 ms)
        self._ui_timer = QTimer(self)
        self._ui_timer.setInterval(33)
        self._ui_timer.timeout.connect(self._on_ui_refresh_tick)
        self._ui_timer.start()

        if start_demo:
            self.start_demo_simulator()

    def _log_event(self, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._event_log.append(f"[{stamp}] {text}")
        if len(self._event_log) > 400:
            del self._event_log[:100]

    # ------------------------------------------------------------------
    # UI Construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget(self)
        central.setObjectName("central_root")
        central.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setCentralWidget(central)
        root_vbox = QVBoxLayout(central)
        root_vbox.setContentsMargins(0, 0, 0, 0)
        root_vbox.setSpacing(0)

        root_vbox.addWidget(self._build_top_bar())

        body = QWidget(central)
        body.setObjectName("central_root")
        body.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        body_hbox = QHBoxLayout(body)
        body_hbox.setContentsMargins(0, 0, 0, 0)
        body_hbox.setSpacing(0)

        body_hbox.addWidget(self._build_sidebar())

        self.pages = QStackedWidget(body)
        self.pages.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.pages.addWidget(self._build_live_positioning_page())
        self.pages.addWidget(self._build_calibration_page())
        self.pages.addWidget(self._build_layout_and_diag_page())
        self.pages.addWidget(self._build_replay_and_analysis_page())
        # Read-only view of the drone computer (UP4000 up4000_status node, UDP 15557)
        self.up4000_listener = Up4000StatusListener(parent=self)
        self.up4000_panel = Up4000Panel(self.up4000_listener, self.pages)
        self.pages.addWidget(self.up4000_panel)
        body_hbox.addWidget(self.pages, 1)

        root_vbox.addWidget(body, 1)

    def _build_top_bar(self) -> QFrame:
        bar = QFrame(self)
        bar.setObjectName("top_bar")
        bar.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        bar.setFixedHeight(54)
        hbox = QHBoxLayout(bar)
        hbox.setContentsMargins(SPACING_MD, SPACING_SM, SPACING_MD, SPACING_SM)
        hbox.setSpacing(SPACING_SM)

        brand = QLabel("UWB FAST STUDIO")
        brand.setObjectName("brand_title")
        hbox.addWidget(brand)

        self.conn_badge = QLabel("DISCONNECTED")
        self.conn_badge.setObjectName("badge_neutral")
        hbox.addWidget(self.conn_badge)

        hbox.addSpacing(SPACING_SM)

        hbox.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        self.port_combo.setMaximumWidth(210)
        hbox.addWidget(self.port_combo)

        btn_refresh = QPushButton("Refresh")
        btn_refresh.clicked.connect(lambda: self._refresh_ports())
        hbox.addWidget(btn_refresh)

        hbox.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["460800", "921600", "1000000", "230400", "115200"])
        self.baud_combo.setCurrentText("460800")
        self.baud_combo.setMinimumWidth(86)
        hbox.addWidget(self.baud_combo)

        self.btn_connect = QPushButton("Connect UART")
        self.btn_connect.setObjectName("primary_btn")
        self.btn_connect.clicked.connect(self._toggle_serial_connection)
        hbox.addWidget(self.btn_connect)

        hbox.addSpacing(SPACING_SM)

        self.btn_demo = QPushButton("Demo 3D Sim")
        self.btn_demo.setCheckable(True)
        self.btn_demo.clicked.connect(self._on_demo_button_toggled)
        hbox.addWidget(self.btn_demo)

        self.demo_profile_combo = QComboBox()
        self.demo_profile_combo.addItems(["helix_3d", "circle_2d", "stationary"])
        self.demo_profile_combo.setMinimumWidth(104)
        self.demo_profile_combo.setToolTip("Simulator motion profile")
        self.demo_profile_combo.currentTextChanged.connect(self._on_demo_profile_changed)
        hbox.addWidget(self.demo_profile_combo)

        hbox.addStretch(1)

        self.btn_csv_log = QPushButton("Log CSV: OFF")
        self.btn_csv_log.setCheckable(True)
        self.btn_csv_log.clicked.connect(self._toggle_csv_logging)
        hbox.addWidget(self.btn_csv_log)

        self.btn_theme = QPushButton("Theme: Light")
        self.btn_theme.clicked.connect(self._toggle_theme)
        hbox.addWidget(self.btn_theme)

        return bar

    def _build_sidebar(self) -> QFrame:
        sidebar = QFrame(self)
        sidebar.setObjectName("sidebar")
        sidebar.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        sidebar.setFixedWidth(224)
        vbox = QVBoxLayout(sidebar)
        vbox.setContentsMargins(10, 12, 10, 12)
        vbox.setSpacing(6)

        hdr = QLabel("WORKSPACES")
        hdr.setObjectName("section_header")
        hdr.setContentsMargins(4, 0, 4, 2)
        vbox.addWidget(hdr)

        self.nav_group = QButtonGroup(self)
        self.nav_group.setExclusive(True)

        nav_items = [
            ("1. 2D / 3D Positioning", 0),
            ("2. Fast Calibration", 1),
            ("3. Rig Layout && Diag", 2),
            ("4. Log Replay && Analysis", 3),
            ("5. UP4000 Onboard", 4),
        ]
        for label, page_idx in nav_items:
            btn = QPushButton(label)
            btn.setObjectName("nav_btn")
            btn.setCheckable(True)
            if page_idx == 0:
                btn.setChecked(True)
            btn.clicked.connect(lambda _checked, idx=page_idx: self.pages.setCurrentIndex(idx))
            self.nav_group.addButton(btn, page_idx)
            vbox.addWidget(btn)

        vbox.addSpacing(8)
        quick_hdr = QLabel("SOLVER CONFIGURATION")
        quick_hdr.setObjectName("section_header")
        quick_hdr.setContentsMargins(4, 4, 4, 2)
        vbox.addWidget(quick_hdr)

        dim_row = QHBoxLayout()
        dim_row.setSpacing(6)
        self.btn_dim_2d = QPushButton("2D Fixed-Z")
        self.btn_dim_2d.setCheckable(True)
        self.btn_dim_3d = QPushButton("3D XYZ")
        self.btn_dim_3d.setCheckable(True)
        self.btn_dim_3d.setChecked(True)
        self.btn_dim_2d.clicked.connect(lambda: self.set_dimension_mode(2))
        self.btn_dim_3d.clicked.connect(lambda: self.set_dimension_mode(3))
        dim_row.addWidget(self.btn_dim_2d, 1)
        dim_row.addWidget(self.btn_dim_3d, 1)
        vbox.addLayout(dim_row)

        z_row = QHBoxLayout()
        z_row.addWidget(QLabel("2D Fixed Z (m):"))
        self.spin_fixed_z = _make_double_spin(-20.0, 20.0, 0.000, decimals=3, min_width=76)
        self.spin_fixed_z.valueChanged.connect(self._on_fixed_z_changed)
        z_row.addWidget(self.spin_fixed_z)
        vbox.addLayout(z_row)

        self.chk_use_uncal = QCheckBox("Use Uncal + Host Bias")
        self.chk_use_uncal.setChecked(True)
        self.chk_use_uncal.setToolTip(
            "Allow positioning even when firmware reports CAL_MISSING (0x20), applying host calibration bias."
        )
        self.chk_use_uncal.toggled.connect(self._on_solver_flags_changed)
        vbox.addWidget(self.chk_use_uncal)

        self.filter_mode_combo = QComboBox()
        self.filter_mode_combo.addItem("Robust Adaptive EKF", "robust_ekf")
        self.filter_mode_combo.addItem("Legacy CV + IRLS", "legacy")
        self.filter_mode_combo.setToolTip("Position estimator; also used when re-solving a recorded log.")
        vbox.addWidget(self.filter_mode_combo)

        self.chk_use_filtered = QCheckBox("Legacy: Host CV Range Filter")
        self.chk_use_filtered.setChecked(True)
        self.chk_use_filtered.setEnabled(False)
        self.chk_use_filtered.toggled.connect(self._on_solver_flags_changed)
        vbox.addWidget(self.chk_use_filtered)

        self.chk_nlos_weight = QCheckBox("NLOS Adaptive Weight")
        self.chk_nlos_weight.setChecked(True)
        self.chk_nlos_weight.toggled.connect(self._on_solver_flags_changed)
        vbox.addWidget(self.chk_nlos_weight)
        self.chk_signal_quality = QCheckBox("Signal Quality Weight")
        self.chk_signal_quality.setChecked(True)
        self.chk_signal_quality.setToolTip("EKF: reduce trust using power, first-path/noise, link diagnostics and clock stability. Score is heuristic, not LOS probability.")
        self.chk_signal_quality.toggled.connect(self._on_solver_flags_changed)
        vbox.addWidget(self.chk_signal_quality)
        self.filter_mode_combo.currentIndexChanged.connect(self._on_solver_flags_changed)

        vbox.addSpacing(8)
        rate_hdr = QLabel("TAG BURST RATE (0x08)")
        rate_hdr.setObjectName("section_header")
        rate_hdr.setContentsMargins(4, 4, 4, 2)
        vbox.addWidget(rate_hdr)

        self.burst_preset_combo = QComboBox()
        preset_labels = {
            "stable50": "50 Hz Stable",
            "hz100": "100 Hz Fast",
            "hz200": "200 Hz High-Speed",
            "sequential": "Sequential Legacy",
        }
        for key in BURST_PRESETS:
            self.burst_preset_combo.addItem(preset_labels.get(key, key), key)
        self.burst_preset_combo.setCurrentIndex(2)
        vbox.addWidget(self.burst_preset_combo)

        btn_apply_burst = QPushButton("Apply Rate to TAG")
        btn_apply_burst.clicked.connect(self._send_selected_burst_preset)
        vbox.addWidget(btn_apply_burst)

        vbox.addStretch(1)

        fw_card = QFrame(sidebar)
        fw_card.setObjectName("sub_card")
        fw_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        fw_v = QVBoxLayout(fw_card)
        fw_v.setContentsMargins(10, 8, 10, 8)
        fw_v.setSpacing(3)
        fw_title = QLabel("FIRMWARE LINK")
        fw_title.setObjectName("card_title")
        self.lbl_sidebar_fw = QLabel("Idle — Connect UART or start Demo 3D Sim.")
        self.lbl_sidebar_fw.setWordWrap(True)
        self.lbl_sidebar_fw.setObjectName("metric_sub")
        fw_v.addWidget(fw_title)
        fw_v.addWidget(self.lbl_sidebar_fw)
        vbox.addWidget(fw_card)

        return sidebar

    # ------------------------------------------------------------------
    # Page 1: 2D / 3D Live Positioning & Measurement
    # ------------------------------------------------------------------

    def _build_live_positioning_page(self) -> QWidget:
        page = QWidget(self)
        page.setObjectName("workspace_page")
        page.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        vbox = QVBoxLayout(page)
        vbox.setContentsMargins(SPACING_MD, SPACING_MD, SPACING_MD, SPACING_MD)
        vbox.setSpacing(SPACING_SM)

        # Clean 4-Card KPI Strip (spacious, never clipped)
        kpi_row = QHBoxLayout()
        kpi_row.setSpacing(SPACING_SM)
        self.card_pos = StatCard("TAG POSITION (X, Y, Z)", "-- , -- , -- m", "Mode: 3D | Used: 0 anchors")
        self.card_residual = StatCard("SOLVER RESIDUAL", "RMS: -- mm", "Max: -- mm | Excluded: none")
        self.card_dop = StatCard("GEOMETRY DOP", "PDOP: --", "HDOP: -- | VDOP: -- | GDOP: --")
        self.card_rate = StatCard("TELEMETRY & HEALTH", "0.0 Hz", "Bursts: 0 | Lost: 0 | CalMask: 0x00")
        for card in (self.card_pos, self.card_residual, self.card_dop, self.card_rate):
            kpi_row.addWidget(card, 1)
        vbox.addLayout(kpi_row)

        main_split = QSplitter(Qt.Orientation.Horizontal, page)

        # Left Pane: 2D/3D Interactive Viewport + Accuracy Bar
        left_frame = QFrame(main_split)
        left_frame.setObjectName("panel_card")
        left_frame.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        left_vbox = QVBoxLayout(left_frame)
        left_vbox.setContentsMargins(10, 10, 10, 10)
        left_vbox.setSpacing(8)

        canvas_bar = QHBoxLayout()
        canvas_bar.setSpacing(6)

        self.view_mode_group = QButtonGroup(self)
        self.view_mode_group.setExclusive(True)
        self.btn_view_2d = QPushButton("2D XY")
        self.btn_view_2d.setCheckable(True)
        self.btn_view_3d = QPushButton("3D Orbit")
        self.btn_view_3d.setCheckable(True)
        self.btn_view_split = QPushButton("Split 2D + 3D")
        self.btn_view_split.setCheckable(True)
        self.btn_view_split.setChecked(True)

        for idx, (b, mode_name) in enumerate(
            ((self.btn_view_2d, "2D"), (self.btn_view_3d, "3D"), (self.btn_view_split, "SPLIT"))
        ):
            self.view_mode_group.addButton(b, idx)
            b.clicked.connect(lambda _c, m=mode_name: self._set_canvas_view_mode(m))
            canvas_bar.addWidget(b)

        canvas_bar.addStretch(1)

        self.chk_show_rays = QCheckBox("Rays")
        self.chk_show_rays.setChecked(True)
        self.chk_show_rays.toggled.connect(self._on_canvas_overlay_changed)
        canvas_bar.addWidget(self.chk_show_rays)

        self.chk_show_ranges = QCheckBox("2D Circles")
        self.chk_show_ranges.setChecked(False)
        self.chk_show_ranges.toggled.connect(self._on_canvas_overlay_changed)
        canvas_bar.addWidget(self.chk_show_ranges)

        self.chk_show_trail = QCheckBox("Trail")
        self.chk_show_trail.setChecked(True)
        self.chk_show_trail.toggled.connect(self._on_canvas_overlay_changed)
        canvas_bar.addWidget(self.chk_show_trail)

        self.chk_stability_colors = QCheckBox("Stability Colors")
        self.chk_stability_colors.setChecked(True)
        self.chk_stability_colors.setToolTip("Green=Stable, Amber=Warn, Red=Unstable along the full 3D loop")
        self.chk_stability_colors.toggled.connect(self._on_canvas_overlay_changed)
        canvas_bar.addWidget(self.chk_stability_colors)

        self.edit_waypoint_label = QLineEdit()
        self.edit_waypoint_label.setPlaceholderText("label (e.g. P1_start)")
        self.edit_waypoint_label.setMaximumWidth(180)
        canvas_bar.addWidget(self.edit_waypoint_label)
        btn_mark_wp = QPushButton("Mark Point")
        btn_mark_wp.setToolTip("Drop a numbered waypoint marker (W1, W2...) on the 2D/3D map and record it in CSV log")
        btn_mark_wp.clicked.connect(self._mark_live_waypoint)
        canvas_bar.addWidget(btn_mark_wp)

        btn_fit = QPushButton("Reset Cam")
        btn_fit.clicked.connect(lambda: self.map_canvas.reset_camera())
        canvas_bar.addWidget(btn_fit)

        btn_clr_trail = QPushButton("Clear Trail")
        btn_clr_trail.clicked.connect(self._clear_trail)
        canvas_bar.addWidget(btn_clr_trail)

        left_vbox.addLayout(canvas_bar)

        self.map_canvas = PositionMapCanvas(self.engine, left_frame)
        self.map_canvas.view_mode = "SPLIT"
        left_vbox.addWidget(self.map_canvas, 1)

        # Clean, Unclipped 2D/3D Accuracy Evaluation Bar
        eval_card = QFrame(left_frame)
        eval_card.setObjectName("sub_card")
        eval_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        eval_vbox = QVBoxLayout(eval_card)
        eval_vbox.setContentsMargins(10, 8, 10, 8)
        eval_vbox.setSpacing(4)

        eval_top = QHBoxLayout()
        eval_top.setSpacing(6)
        eval_title = QLabel("BENCHMARK:")
        eval_title.setObjectName("card_title")
        eval_top.addWidget(eval_title)

        eval_top.addWidget(QLabel("X:"))
        self.spin_eval_x = _make_double_spin(-100.0, 100.0, 2.500, decimals=3, min_width=62)
        eval_top.addWidget(self.spin_eval_x)

        eval_top.addWidget(QLabel("Y:"))
        self.spin_eval_y = _make_double_spin(-100.0, 100.0, 2.000, decimals=3, min_width=62)
        eval_top.addWidget(self.spin_eval_y)

        eval_top.addWidget(QLabel("Z:"))
        self.spin_eval_z = _make_double_spin(-100.0, 100.0, 1.200, decimals=3, min_width=62)
        eval_top.addWidget(self.spin_eval_z)

        eval_top.addWidget(QLabel("N:"))
        self.spin_eval_n = _make_int_spin(10, 5000, 200, min_width=52)
        eval_top.addWidget(self.spin_eval_n)

        eval_top.addStretch(1)

        self.btn_eval_run = QPushButton("Measure Accuracy")
        self.btn_eval_run.setObjectName("primary_btn")
        self.btn_eval_run.setMinimumWidth(128)
        self.btn_eval_run.clicked.connect(self._start_or_stop_position_eval)
        eval_top.addWidget(self.btn_eval_run)

        btn_eval_export = QPushButton("Export JSON")
        btn_eval_export.setMinimumWidth(96)
        btn_eval_export.clicked.connect(self._export_position_eval_report)
        eval_top.addWidget(btn_eval_export)

        eval_vbox.addLayout(eval_top)

        self.lbl_eval_summary = QLabel(
            "Enter known reference (X, Y, Z) and click 'Measure Accuracy' to compute 2D CEP50/CEP95, 3D SEP50/SEP95, and XYZ bias."
        )
        self.lbl_eval_summary.setObjectName("metric_sub")
        self.lbl_eval_summary.setWordWrap(True)
        eval_vbox.addWidget(self.lbl_eval_summary)

        left_vbox.addWidget(eval_card)

        # Right Pane: 7-Column Clean Live Anchor Table + Rolling Waveform
        right_split = QSplitter(Qt.Orientation.Vertical, main_split)

        table_card = QFrame(right_split)
        table_card.setObjectName("panel_card")
        table_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        t_vbox = QVBoxLayout(table_card)
        t_vbox.setContentsMargins(10, 10, 10, 10)
        t_vbox.setSpacing(6)

        t_hdr_row = QHBoxLayout()
        t_hdr = QLabel("LIVE 8-ANCHOR RANGING & RESIDUALS")
        t_hdr.setObjectName("section_header")
        t_hdr_row.addWidget(t_hdr)
        t_hdr_row.addStretch(1)
        self.lbl_burst_meta = QLabel("Burst Seq: -- | Reachable: 0/8")
        self.lbl_burst_meta.setObjectName("metric_sub")
        t_hdr_row.addWidget(self.lbl_burst_meta)
        t_vbox.addLayout(t_hdr_row)

        self.live_anchor_table = QTableWidget(8, 9, table_card)
        self.live_anchor_table.setHorizontalHeaderLabels(
            [
                "ID",
                "State",
                "Hz",
                "Raw (m)",
                "Filt (m)",
                "Res (mm)",
                "NLOS (dB)",
                "Quality",
                "R x",
            ]
        )
        self.live_anchor_table.verticalHeader().setVisible(False)
        self.live_anchor_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.live_anchor_table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.live_anchor_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.live_anchor_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.live_anchor_table.setAlternatingRowColors(True)
        hdr_view = self.live_anchor_table.horizontalHeader()
        hdr_view.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        hdr_view.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        hdr_view.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        hdr_view.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self.live_anchor_table.horizontalHeaderItem(7).setToolTip("Radio quality 0-100 from available indicators; not a LOS probability. Hover each score for reasons and diagnostic age.")
        self.live_anchor_table.horizontalHeaderItem(8).setToolTip("Applied radio variance multiplier. Larger means less trust; EKF innovation and health weights also apply.")
        self._init_live_anchor_table_rows()
        t_vbox.addWidget(self.live_anchor_table, 1)

        chart_card = QFrame(right_split)
        chart_card.setObjectName("panel_card")
        chart_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        c_vbox = QVBoxLayout(chart_card)
        c_vbox.setContentsMargins(10, 10, 10, 10)
        c_vbox.setSpacing(6)

        c_bar = QHBoxLayout()
        c_hdr = QLabel("ROLLING WAVEFORM")
        c_hdr.setObjectName("section_header")
        c_bar.addWidget(c_hdr)
        c_bar.addStretch(1)

        self.chart_target_combo = QComboBox()
        self.chart_target_combo.addItem("Solved Position X / Y / Z (m)", 9)
        self.chart_target_combo.addItem("All 8 Anchors (Filtered m)", 0)
        for aid in range(1, 9):
            self.chart_target_combo.addItem(f"Anchor A{aid} (Raw vs Cal vs Filt)", aid)
        self.chart_target_combo.currentIndexChanged.connect(self._on_chart_controls_changed)
        c_bar.addWidget(self.chart_target_combo)

        self.chart_win_combo = QComboBox()
        self.chart_win_combo.addItem("5 s", 5.0)
        self.chart_win_combo.addItem("10 s", 10.0)
        self.chart_win_combo.addItem("20 s", 20.0)
        self.chart_win_combo.addItem("30 s", 30.0)
        self.chart_win_combo.setCurrentIndex(1)
        self.chart_win_combo.currentIndexChanged.connect(self._on_chart_controls_changed)
        c_bar.addWidget(self.chart_win_combo)
        c_vbox.addLayout(c_bar)

        self.rolling_chart = FastRollingChart(self.engine, chart_card)
        c_vbox.addWidget(self.rolling_chart, 1)

        right_split.setSizes([360, 340])
        main_split.setSizes([720, 580])
        vbox.addWidget(main_split, 1)

        return page

    def _init_live_anchor_table_rows(self) -> None:
        mono_font = QFont(FONT_MONO, 9)
        for row in range(8):
            aid = row + 1
            for col in range(self.live_anchor_table.columnCount()):
                item = QTableWidgetItem("--")
                item.setFont(mono_font)
                if col == 0:
                    item.setText(f"A{aid}")
                    item.setForeground(QColor(ANCHOR_PALETTE[row]))
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                elif col == 1:
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                else:
                    item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.live_anchor_table.setItem(row, col, item)
            self.live_anchor_table.setRowHeight(row, 29)

    # ------------------------------------------------------------------
    # Page 2: Fast Calibration Studio (1D Single Anchor & 2D/3D Multi-Spot)
    # ------------------------------------------------------------------

    def _build_calibration_page(self) -> QWidget:
        page = QWidget(self)
        page.setObjectName("workspace_page")
        page.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        vbox = QVBoxLayout(page)
        vbox.setContentsMargins(SPACING_MD, SPACING_MD, SPACING_MD, SPACING_MD)
        vbox.setSpacing(SPACING_SM)

        method_row = QHBoxLayout()
        method_row.addWidget(QLabel("Calibration method:"))
        self.combo_cal_method = QComboBox()
        self.combo_cal_method.addItem("Joint multi-spot (recommended)", "joint")
        self.combo_cal_method.addItem("Median per spot (legacy)", "median")
        self.combo_cal_method.setCurrentIndex(self.combo_cal_method.findData(self.engine.calibration_method))
        self.combo_cal_method.currentIndexChanged.connect(self._on_calibration_method_changed)
        method_row.addWidget(self.combo_cal_method)
        method_row.addStretch(1)
        vbox.addLayout(method_row)

        modes_hbox = QHBoxLayout()
        modes_hbox.setSpacing(SPACING_SM)

        # Mode A: 2D/3D Known-Spot Simultaneous All-8-Anchor Calibration
        spot_card = QFrame(page)
        spot_card.setObjectName("panel_card")
        spot_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        s_vbox = QVBoxLayout(spot_card)
        s_vbox.setContentsMargins(14, 12, 14, 12)
        s_vbox.setSpacing(8)

        s_title = QLabel("MODE A — 2D/3D KNOWN-SPOT CALIBRATION (ALL 8 ANCHORS AT ONCE)")
        s_title.setObjectName("section_header")
        s_vbox.addWidget(s_title)

        s_desc = QLabel(
            "Place TAG at a known (X, Y, Z) spot in your 2D/3D rig. "
            "Captures raw burst distances for all active anchors simultaneously (bias = raw_median - true_dist)."
        )
        s_desc.setObjectName("metric_sub")
        s_desc.setWordWrap(True)
        s_vbox.addWidget(s_desc)

        s_form = QHBoxLayout()
        s_form.setSpacing(8)
        s_form.addWidget(QLabel("Spot X (m):"))
        self.spin_cal_x = _make_double_spin(-100.0, 100.0, 2.5000, decimals=4, min_width=80)
        s_form.addWidget(self.spin_cal_x)

        s_form.addWidget(QLabel("Y (m):"))
        self.spin_cal_y = _make_double_spin(-100.0, 100.0, 2.0000, decimals=4, min_width=80)
        s_form.addWidget(self.spin_cal_y)

        s_form.addWidget(QLabel("Z (m):"))
        self.spin_cal_z = _make_double_spin(-100.0, 100.0, 1.2000, decimals=4, min_width=80)
        s_form.addWidget(self.spin_cal_z)

        s_form.addWidget(QLabel("Samples:"))
        self.spin_cal_samples = _make_int_spin(10, 5000, 200, min_width=64)
        s_form.addWidget(self.spin_cal_samples)
        s_form.addStretch(1)
        s_vbox.addLayout(s_form)

        s_btns = QHBoxLayout()
        s_btns.setSpacing(8)
        self.btn_capture_spot = QPushButton("Capture Spot (All Anchors)")
        self.btn_capture_spot.setObjectName("primary_btn")
        self.btn_capture_spot.clicked.connect(self._start_spot_calibration)
        s_btns.addWidget(self.btn_capture_spot)

        self.btn_suggest_spots = QPushButton("Suggest spots")
        self.btn_suggest_spots.clicked.connect(self._suggest_calibration_spots)
        s_btns.addWidget(self.btn_suggest_spots)

        btn_room_center = QPushButton("Preset: Room (3.6, 1.8, 0.48)")
        btn_room_center.clicked.connect(lambda: self._set_spot_xyz(3.60, 1.80, 0.48))
        s_btns.addWidget(btn_room_center)

        btn_rig_center = QPushButton("Preset: Rig40 (0.2, 0.2, 1.0)")
        btn_rig_center.clicked.connect(lambda: self._set_spot_xyz(0.20, 0.20, 1.00))
        s_btns.addWidget(btn_rig_center)

        btn_box_center = QPushButton("Preset: Box (2.5, 2.0, 1.2)")
        btn_box_center.clicked.connect(lambda: self._set_spot_xyz(2.50, 2.00, 1.20))
        s_btns.addWidget(btn_box_center)
        s_btns.addStretch(1)
        s_vbox.addLayout(s_btns)

        self.lbl_spot_plan = QLabel("")
        self.lbl_spot_plan.setWordWrap(True)
        s_vbox.addWidget(self.lbl_spot_plan)

        modes_hbox.addWidget(spot_card, 3)

        # Mode B: 1D Single-Anchor Tape Distance Calibration
        tape_card = QFrame(page)
        tape_card.setObjectName("panel_card")
        tape_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        t_vbox = QVBoxLayout(tape_card)
        t_vbox.setContentsMargins(14, 12, 14, 12)
        t_vbox.setSpacing(8)

        t_title = QLabel("MODE B — 1D SINGLE-ANCHOR TAPE CALIBRATION")
        t_title.setObjectName("section_header")
        t_vbox.addWidget(t_title)

        t_desc = QLabel(
            "Place TAG at a known 1D tape/laser distance from a single anchor (A1..A8) to calibrate its bias individually."
        )
        t_desc.setObjectName("metric_sub")
        t_desc.setWordWrap(True)
        t_vbox.addWidget(t_desc)

        t_form = QHBoxLayout()
        t_form.setSpacing(8)
        t_form.addWidget(QLabel("Anchor:"))
        self.combo_1d_anchor = QComboBox()
        self.combo_1d_anchor.setMinimumWidth(68)
        for aid in range(1, 9):
            self.combo_1d_anchor.addItem(f"A{aid}", aid)
        t_form.addWidget(self.combo_1d_anchor)

        t_form.addWidget(QLabel("True Dist (m):"))
        self.spin_1d_dist = _make_double_spin(0.05, 500.0, 1.0000, decimals=4, min_width=84)
        t_form.addWidget(self.spin_1d_dist)

        self.btn_capture_1d = QPushButton("Capture 1D Anchor")
        self.btn_capture_1d.setObjectName("primary_btn")
        self.btn_capture_1d.clicked.connect(self._start_1d_calibration)
        t_form.addWidget(self.btn_capture_1d)
        t_form.addStretch(1)
        t_vbox.addLayout(t_form)

        quick_row = QHBoxLayout()
        quick_row.setSpacing(6)
        quick_row.addWidget(QLabel("Quick:"))
        for d_val in (0.4, 0.5, 1.0, 2.0, 3.0, 5.0):
            b = QPushButton(f"{d_val:g}m")
            b.clicked.connect(lambda _c, v=d_val: self.spin_1d_dist.setValue(v))
            quick_row.addWidget(b)
        quick_row.addStretch(1)
        t_vbox.addLayout(quick_row)

        modes_hbox.addWidget(tape_card, 2)
        vbox.addLayout(modes_hbox)

        # Progress & Action Ribbon
        prog_card = QFrame(page)
        prog_card.setObjectName("sub_card")
        prog_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        prog_hbox = QHBoxLayout(prog_card)
        prog_hbox.setContentsMargins(12, 8, 12, 8)
        prog_hbox.setSpacing(8)

        self.lbl_cal_status = QLabel("Ready for calibration capture.")
        prog_hbox.addWidget(self.lbl_cal_status, 1)

        self.cal_progress = QProgressBar()
        self.cal_progress.setRange(0, 100)
        self.cal_progress.setValue(0)
        self.cal_progress.setFixedWidth(160)
        prog_hbox.addWidget(self.cal_progress)

        btn_finish_cal = QPushButton("Finish now")
        btn_finish_cal.setToolTip("Stop the capture and use the samples collected so far; "
                                  "anchors below the minimum are marked LOW_SAMPLES.")
        btn_finish_cal.clicked.connect(lambda: self.engine.finish_calibration_now())
        prog_hbox.addWidget(btn_finish_cal)

        btn_cancel_cal = QPushButton("Cancel")
        btn_cancel_cal.clicked.connect(lambda: self.engine.cancel_calibration())
        prog_hbox.addWidget(btn_cancel_cal)

        btn_apply_host = QPushButton("1. Apply to Host Solver")
        btn_apply_host.setObjectName("success_btn")
        btn_apply_host.clicked.connect(self._confirm_apply_calibration_to_host)
        prog_hbox.addWidget(btn_apply_host)

        btn_write_tag = QPushButton("2. Write to TAG (SET_DS_CAL)")
        btn_write_tag.setObjectName("primary_btn")
        btn_write_tag.clicked.connect(self._confirm_write_calibrated_biases_to_tag)
        prog_hbox.addWidget(btn_write_tag)

        btn_save_flash = QPushButton("3. Save Flash")
        btn_save_flash.clicked.connect(self._save_tag_settings_to_flash)
        prog_hbox.addWidget(btn_save_flash)

        btn_read_tag = QPushButton("Read TAG")
        btn_read_tag.clicked.connect(self._request_tag_ds_cal)
        prog_hbox.addWidget(btn_read_tag)

        btn_clear_cal = QPushButton("Clear")
        btn_clear_cal.setObjectName("danger_btn")
        btn_clear_cal.clicked.connect(self._clear_calibration_captures)
        prog_hbox.addWidget(btn_clear_cal)

        vbox.addWidget(prog_card)

        # Spot sanity warnings (wrong spot coordinates, implausible bias, NLOS spot)
        self.lbl_cal_warning = QLabel("")
        self.lbl_cal_warning.setObjectName("badge_err")
        self.lbl_cal_warning.setWordWrap(True)
        self.lbl_cal_warning.setVisible(False)
        vbox.addWidget(self.lbl_cal_warning)

        joint_row = QHBoxLayout()
        self.lbl_joint_status = QLabel("Joint solve needs at least 2 spot captures (have 0) — using per-spot median")
        self.lbl_joint_status.setWordWrap(True)
        joint_row.addWidget(self.lbl_joint_status, 1)
        self.btn_remove_spot = QPushButton("Remove selected spot")
        self.btn_remove_spot.clicked.connect(self._remove_selected_calibration_spot)
        joint_row.addWidget(self.btn_remove_spot)
        vbox.addLayout(joint_row)
        self.cal_spot_table = QTableWidget(0, 6, page)
        self.cal_spot_table.setHorizontalHeaderLabels(
            ["#", "Entered XYZ (m)", "Estimated XYZ (m)", "Offset (cm)", "Resid RMS (mm)", "Outliers"])
        self.cal_spot_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.cal_spot_table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.cal_spot_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.cal_spot_table.verticalHeader().setVisible(False)
        self.cal_spot_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.cal_spot_table.setMinimumHeight(120)
        self.cal_spot_table.setMaximumHeight(200)
        vbox.addWidget(self.cal_spot_table, 1)
        self._cal_spot_table_key = None

        # Calibration Results Table (9 spacious columns — zero truncation)
        res_card = QFrame(page)
        res_card.setObjectName("panel_card")
        res_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        res_vbox = QVBoxLayout(res_card)
        res_vbox.setContentsMargins(10, 10, 10, 10)
        res_vbox.setSpacing(6)

        res_top = QHBoxLayout()
        res_hdr = QLabel("CALIBRATION RESULTS & PER-ANCHOR BIAS SUMMARY")
        res_hdr.setObjectName("section_header")
        res_top.addWidget(res_hdr)
        res_top.addStretch(1)

        btn_exp_rig = QPushButton("Export anchors_rig40.txt...")
        btn_exp_rig.clicked.connect(self._export_rig_txt_dialog)
        res_top.addWidget(btn_exp_rig)

        btn_exp_cal_csv = QPushButton("Export Calibration CSV / JSON...")
        btn_exp_cal_csv.clicked.connect(self._export_calibration_records_dialog)
        res_top.addWidget(btn_exp_cal_csv)
        res_vbox.addLayout(res_top)

        self.cal_table = QTableWidget(8, 9, res_card)
        self.cal_table.setHorizontalHeaderLabels(
            [
                "Anchor",
                "Spot / Mode",
                "Samples",
                "True (m)",
                "Raw Med (m)",
                "Bias (mm)",
                "Agg Bias (mm)",
                "Noise σ (mm)",
                "Active Host / FW State",
            ]
        )
        self.cal_table.verticalHeader().setVisible(False)
        self.cal_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.cal_table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.cal_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.cal_table.setAlternatingRowColors(True)
        cal_hdr = self.cal_table.horizontalHeader()
        cal_hdr.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        cal_hdr.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        cal_hdr.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        cal_hdr.setSectionResizeMode(8, QHeaderView.ResizeMode.ResizeToContents)
        self._init_cal_table_rows()
        res_vbox.addWidget(self.cal_table, 1)

        vbox.addWidget(res_card, 1)
        return page

    def _init_cal_table_rows(self) -> None:
        mono_font = QFont(FONT_MONO, 9)
        for row in range(8):
            aid = row + 1
            for col in range(9):
                item = QTableWidgetItem("--")
                item.setFont(mono_font)
                if col == 0:
                    item.setText(f"A{aid}")
                    item.setForeground(QColor(ANCHOR_PALETTE[row]))
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                elif col in (2, 3, 4, 5, 6, 7):
                    item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                else:
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.cal_table.setItem(row, col, item)
            self.cal_table.setRowHeight(row, 30)

    # ------------------------------------------------------------------
    # Page 3: Anchor Rig Layout & Firmware Diagnostics
    # ------------------------------------------------------------------

    def _build_layout_and_diag_page(self) -> QWidget:
        page = QWidget(self)
        page.setObjectName("workspace_page")
        page.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        vbox = QVBoxLayout(page)
        vbox.setContentsMargins(SPACING_MD, SPACING_MD, SPACING_MD, SPACING_MD)
        vbox.setSpacing(SPACING_SM)

        split = QSplitter(Qt.Orientation.Horizontal, page)

        layout_card = QFrame(split)
        layout_card.setObjectName("panel_card")
        layout_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        l_vbox = QVBoxLayout(layout_card)
        l_vbox.setContentsMargins(10, 10, 10, 10)
        l_vbox.setSpacing(8)

        l_hdr = QLabel("ANCHOR RIG COORDINATES & BIAS CONFIGURATION (A1..A8)")
        l_hdr.setObjectName("section_header")
        l_vbox.addWidget(l_hdr)

        l_top = QHBoxLayout()
        l_top.setSpacing(8)
        btn_preset_room = QPushButton("Preset: Room (6.6×3.6×1.58m)")
        btn_preset_room.clicked.connect(lambda: self._apply_anchor_preset(DEFAULT_PRESET_ROOM_66_36))
        l_top.addWidget(btn_preset_room)

        btn_preset_box = QPushButton("Preset: 3D Box (5×4×3m)")
        btn_preset_box.clicked.connect(lambda: self._apply_anchor_preset(DEFAULT_PRESET_BOX8))
        l_top.addWidget(btn_preset_box)

        btn_preset_rig40 = QPushButton("Preset: Rig 40cm Cube")
        btn_preset_rig40.clicked.connect(lambda: self._apply_anchor_preset(DEFAULT_PRESET_RIG40))
        l_top.addWidget(btn_preset_rig40)

        btn_preset_2d = QPushButton("Preset: 2D Flat (4A)")
        btn_preset_2d.clicked.connect(lambda: self._apply_anchor_preset(DEFAULT_PRESET_FLAT4))
        l_top.addWidget(btn_preset_2d)
        l_top.addStretch(1)

        l_vbox.addLayout(l_top)

        self.anchor_cfg_table = QTableWidget(8, 8, layout_card)
        self.anchor_cfg_table.setHorizontalHeaderLabels(
            [
                "ID",
                "On",
                "X (m)",
                "Y (m)",
                "Z (m)",
                "Bias (mm)",
                "σ (mm)",
                "FW State",
            ]
        )
        self.anchor_cfg_table.verticalHeader().setVisible(False)
        self.anchor_cfg_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.anchor_cfg_table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        cfg_hdr = self.anchor_cfg_table.horizontalHeader()
        cfg_hdr.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        cfg_hdr.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        cfg_hdr.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        cfg_hdr.setSectionResizeMode(7, QHeaderView.ResizeMode.ResizeToContents)
        self.anchor_cfg_table.itemChanged.connect(self._on_anchor_cfg_cell_edited)
        l_vbox.addWidget(self.anchor_cfg_table, 1)

        l_bot = QHBoxLayout()
        l_bot.setSpacing(8)
        btn_import_rig = QPushButton("Import Rig (.txt / .json)")
        btn_import_rig.clicked.connect(self._import_rig_file_dialog)
        l_bot.addWidget(btn_import_rig)

        btn_export_rig = QPushButton("Export Rig (.txt / .json)")
        btn_export_rig.clicked.connect(self._export_rig_txt_dialog)
        l_bot.addWidget(btn_export_rig)

        l_bot.addStretch(1)

        btn_push_mask = QPushButton("Push Mask to TAG (0x0B)")
        btn_push_mask.setObjectName("primary_btn")
        btn_push_mask.clicked.connect(self._push_enabled_anchor_mask_to_tag)
        l_bot.addWidget(btn_push_mask)

        l_vbox.addLayout(l_bot)

        diag_card = QFrame(split)
        diag_card.setObjectName("panel_card")
        diag_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        d_vbox = QVBoxLayout(diag_card)
        d_vbox.setContentsMargins(10, 10, 10, 10)
        d_vbox.setSpacing(8)

        d_hdr = QLabel("FIRMWARE DIAGNOSTICS & EVENT LOG")
        d_hdr.setObjectName("section_header")
        d_vbox.addWidget(d_hdr)

        d_top = QHBoxLayout()
        d_top.setSpacing(8)
        btn_get_info = QPushButton("GET_DEVICE_INFO")
        btn_get_info.clicked.connect(lambda: self._send_raw_cmd(tp.CMD_GET_DEVICE_INFO))
        d_top.addWidget(btn_get_info)

        btn_get_cfg = QPushButton("GET_DS_CAL")
        btn_get_cfg.clicked.connect(lambda: self._send_raw_cmd(tp.CMD_GET_DS_CAL))
        d_top.addWidget(btn_get_cfg)

        btn_rxq_on = QPushButton("Enable RX_QUALITY")
        btn_rxq_on.clicked.connect(self._enable_rx_quality_telemetry)
        d_top.addWidget(btn_rxq_on)
        d_top.addStretch(1)

        d_vbox.addLayout(d_top)

        self.lbl_diag_summary = QLabel("System Diag: waiting for frames...")
        self.lbl_diag_summary.setObjectName("metric_sub")
        self.lbl_diag_summary.setWordWrap(True)
        d_vbox.addWidget(self.lbl_diag_summary)

        # Per-second rates of the TAG's DIAG counters (last ~1 s) and likely cause
        self.lbl_diag_rates = QLabel("Radio counters: waiting for DIAG frames...")
        self.lbl_diag_rates.setObjectName("metric_sub")
        self.lbl_diag_rates.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
        self.lbl_diag_rates.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        d_vbox.addWidget(self.lbl_diag_rates)

        self.lbl_diag_hints = QLabel("")
        self.lbl_diag_hints.setObjectName("badge_err")
        self.lbl_diag_hints.setWordWrap(True)
        self.lbl_diag_hints.setVisible(False)
        d_vbox.addWidget(self.lbl_diag_hints)

        self.event_console = QPlainTextEdit(diag_card)
        self.event_console.setReadOnly(True)
        self.event_console.setMaximumBlockCount(400)
        d_vbox.addWidget(self.event_console, 1)

        split.setSizes([820, 480])
        vbox.addWidget(split, 1)
        return page

    # ------------------------------------------------------------------
    # Page 4: Log Replay, 3D Loop Trajectory & Instability Inspector
    # ------------------------------------------------------------------

    def _build_replay_and_analysis_page(self) -> QWidget:
        page = QWidget(self)
        page.setObjectName("workspace_page")
        page.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        vbox = QVBoxLayout(page)
        vbox.setContentsMargins(SPACING_MD, SPACING_MD, SPACING_MD, SPACING_MD)
        vbox.setSpacing(SPACING_SM)

        # Top Control & KPI Row
        ctrl_bar = QHBoxLayout()
        ctrl_bar.setSpacing(6)

        btn_load_latest = QPushButton("Load Latest Log")
        btn_load_latest.setObjectName("primary_btn")
        btn_load_latest.clicked.connect(self._load_latest_log_session)
        ctrl_bar.addWidget(btn_load_latest)

        btn_open_log = QPushButton("Open Log CSV...")
        btn_open_log.clicked.connect(self._open_log_session_dialog)
        ctrl_bar.addWidget(btn_open_log)

        self.chk_replay_resolve = QCheckBox("Re-Solve 3D with Current Rig && Filter")
        self.chk_replay_resolve.setChecked(False)
        self.chk_replay_resolve.setToolTip(
            "Re-run the 3D position solver on raw anchor distances using the current Rig Layout & biases"
        )
        self.chk_replay_resolve.toggled.connect(self._reload_current_log_session)
        ctrl_bar.addWidget(self.chk_replay_resolve)

        ctrl_bar.addStretch(1)

        self.lbl_replay_file = QLabel("No log loaded — Click 'Load Latest Log' or 'Open Log CSV...'")
        self.lbl_replay_file.setObjectName("metric_sub")
        ctrl_bar.addWidget(self.lbl_replay_file)

        vbox.addLayout(ctrl_bar)

        kpi_row = QHBoxLayout()
        kpi_row.setSpacing(SPACING_SM)
        self.card_rep_session = StatCard("SESSION OVERVIEW", "--", "Duration: -- | Frames: 0")
        self.card_rep_loop = StatCard("3D LOOP & DISTANCE", "0.00 m", "2D Path: 0.00 m | Waypoints: 0")
        self.card_rep_stability = StatCard("STABILITY BREAKDOWN", "-- % Stable", "Warn: --% | Unstable: --% | Segments: 0")
        self.card_rep_cursor = StatCard("REPLAY CURSOR (X, Y, Z)", "-- , -- , -- m", "t = 0.00s | RMS: -- mm | Speed: -- m/s")
        for c in (self.card_rep_session, self.card_rep_loop, self.card_rep_stability, self.card_rep_cursor):
            kpi_row.addWidget(c, 1)
        vbox.addLayout(kpi_row)

        main_split = QSplitter(Qt.Orientation.Horizontal, page)

        # Left Pane: 2D/3D Replay Map + Timeline Scrubber
        left_card = QFrame(main_split)
        left_card.setObjectName("panel_card")
        left_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        l_vbox = QVBoxLayout(left_card)
        l_vbox.setContentsMargins(10, 10, 10, 10)
        l_vbox.setSpacing(8)

        map_bar = QHBoxLayout()
        map_bar.setSpacing(6)
        lbl_m = QLabel("REPLAY 2D + 3D TRAJECTORY (Green=Stable · Amber=Warn · Red=Unstable · Pink=Selected Segment)")
        lbl_m.setObjectName("section_header")
        map_bar.addWidget(lbl_m)
        map_bar.addStretch(1)

        btn_rep_reset = QPushButton("Reset Cam")
        btn_rep_reset.clicked.connect(lambda: self.replay_canvas.reset_camera())
        map_bar.addWidget(btn_rep_reset)
        l_vbox.addLayout(map_bar)

        self.replay_canvas = PositionMapCanvas(self.replay_engine, left_card)
        self.replay_canvas.view_mode = "SPLIT"
        self.replay_canvas.color_by_stability = True
        l_vbox.addWidget(self.replay_canvas, 1)

        # Timeline Scrubber Bar
        scrub_card = QFrame(left_card)
        scrub_card.setObjectName("sub_card")
        scrub_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        s_hbox = QHBoxLayout(scrub_card)
        s_hbox.setContentsMargins(10, 6, 10, 6)
        s_hbox.setSpacing(6)

        self.btn_replay_play = QPushButton("Play")
        self.btn_replay_play.setObjectName("primary_btn")
        self.btn_replay_play.setFixedWidth(72)
        self.btn_replay_play.clicked.connect(self._toggle_replay_play)
        s_hbox.addWidget(self.btn_replay_play)

        self.combo_replay_speed = QComboBox()
        for spd_lbl, spd_val in (("0.25x", 0.25), ("0.5x", 0.5), ("1.0x", 1.0), ("2.0x", 2.0), ("5.0x", 5.0), ("10x", 10.0)):
            self.combo_replay_speed.addItem(spd_lbl, spd_val)
        self.combo_replay_speed.setCurrentIndex(2)
        self.combo_replay_speed.currentIndexChanged.connect(
            lambda _i: setattr(self, "replay_speed", float(self.combo_replay_speed.currentData() or 1.0))
        )
        s_hbox.addWidget(self.combo_replay_speed)

        self.replay_slider = QSlider(Qt.Orientation.Horizontal, scrub_card)
        self.replay_slider.setRange(0, 0)
        self.replay_slider.valueChanged.connect(self._on_replay_slider_changed)
        s_hbox.addWidget(self.replay_slider, 1)

        self.lbl_replay_time = QLabel("0.00s / 0.00s")
        self.lbl_replay_time.setFont(QFont(FONT_MONO, 9))
        s_hbox.addWidget(self.lbl_replay_time)

        btn_prev_uns = QPushButton("<< Prev Unstable")
        btn_prev_uns.clicked.connect(lambda: self._step_unstable_segment(-1))
        s_hbox.addWidget(btn_prev_uns)

        btn_next_uns = QPushButton("Next Unstable >>")
        btn_next_uns.clicked.connect(lambda: self._step_unstable_segment(1))
        s_hbox.addWidget(btn_next_uns)

        l_vbox.addWidget(scrub_card)

        # Right Pane: Unstable Segments Table + 8-Anchor Snapshot at Cursor
        right_split = QSplitter(Qt.Orientation.Vertical, main_split)

        uns_card = QFrame(right_split)
        uns_card.setObjectName("panel_card")
        uns_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        u_vbox = QVBoxLayout(uns_card)
        u_vbox.setContentsMargins(10, 10, 10, 10)
        u_vbox.setSpacing(6)

        u_hdr = QLabel("UNSTABLE SEGMENTS (CLICK ROW TO INSPECT ON 2D/3D MAP)")
        u_hdr.setObjectName("section_header")
        u_vbox.addWidget(u_hdr)

        self.unstable_table = QTableWidget(0, 7, uns_card)
        self.unstable_table.setHorizontalHeaderLabels(
            ["#", "Severity", "Time (s)", "Pos (X,Y,Z)", "Max RMS", "Jump", "Cause / Anchor Diagnosis"]
        )
        self.unstable_table.verticalHeader().setVisible(False)
        self.unstable_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.unstable_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.unstable_table.setAlternatingRowColors(True)
        u_hview = self.unstable_table.horizontalHeader()
        u_hview.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        u_hview.setSectionResizeMode(6, QHeaderView.ResizeMode.Stretch)
        self.unstable_table.cellClicked.connect(self._on_unstable_table_row_clicked)
        u_vbox.addWidget(self.unstable_table, 1)

        snap_card = QFrame(right_split)
        snap_card.setObjectName("panel_card")
        snap_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        sn_vbox = QVBoxLayout(snap_card)
        sn_vbox.setContentsMargins(10, 10, 10, 10)
        sn_vbox.setSpacing(6)

        sn_hdr = QLabel("8-ANCHOR TELEMETRY AT REPLAY CURSOR")
        sn_hdr.setObjectName("section_header")
        sn_vbox.addWidget(sn_hdr)

        self.replay_anchor_table = QTableWidget(8, 8, snap_card)
        self.replay_anchor_table.setHorizontalHeaderLabels(
            ["ID", "State", "Hz", "Raw (m)", "Filt (m)", "Res (mm)", "NLOS (dB)", "σ (mm)"]
        )
        self.replay_anchor_table.verticalHeader().setVisible(False)
        self.replay_anchor_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.replay_anchor_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.replay_anchor_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.replay_anchor_table.setAlternatingRowColors(True)
        sn_hview = self.replay_anchor_table.horizontalHeader()
        sn_hview.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        sn_hview.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        sn_hview.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        sn_hview.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        mono = QFont(FONT_MONO, 9)
        for row in range(8):
            aid = row + 1
            for col in range(8):
                it = QTableWidgetItem(f"A{aid}" if col == 0 else "--")
                it.setFont(mono)
                if col == 0:
                    it.setForeground(QColor(ANCHOR_PALETTE[row]))
                else:
                    it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.replay_anchor_table.setItem(row, col, it)
            self.replay_anchor_table.setRowHeight(row, 26)
        sn_vbox.addWidget(self.replay_anchor_table, 1)

        right_split.setSizes([280, 290])
        main_split.setSizes([780, 520])
        vbox.addWidget(main_split, 1)
        return page

    # ------------------------------------------------------------------
    # Serial & Demo Worker Lifecycle
    # ------------------------------------------------------------------

    def _refresh_ports(self, select_port: Optional[str] = None) -> None:
        current = select_port or self.port_combo.currentData() or self.port_combo.currentText()
        ports = available_serial_ports()
        self.port_combo.clear()
        if ports:
            for dev, label in ports:
                self.port_combo.addItem(label, dev)
            for idx in range(self.port_combo.count()):
                if self.port_combo.itemData(idx) == current:
                    self.port_combo.setCurrentIndex(idx)
                    break
        else:
            self.port_combo.addItem("COM3", "COM3")
        # UP4000 on the drone relays the TAG over WiFi (UP4000/ros2_ws uwb_relay).
        udp_key = f"{UDP_RELAY_PREFIX}{DEFAULT_UDP_RELAY_PORT}"
        self.port_combo.addItem(f"WiFi UDP :{DEFAULT_UDP_RELAY_PORT} (UP4000 relay)", udp_key)
        if current == udp_key:
            self.port_combo.setCurrentIndex(self.port_combo.count() - 1)

    def _toggle_serial_connection(self) -> None:
        if self.serial_worker is not None:
            self._stop_serial_worker()
            return

        if self.demo_worker is not None:
            self.stop_demo_simulator()

        port = str(self.port_combo.currentData() or self.port_combo.currentText()).split(" ")[0].strip()
        baud = int(self.baud_combo.currentText())
        if not port:
            return

        if port.startswith(UDP_RELAY_PREFIX):
            worker = UdpTelemetryWorker(listen_port=int(port[len(UDP_RELAY_PREFIX):]), parent=self)
        else:
            worker = SerialTelemetryWorker(port=port, baud=baud, parent=self)
        worker.batch_ready.connect(self._on_telemetry_batch)
        worker.link_status.connect(self._on_link_status)
        worker.log_event.connect(self._log_event)
        self.serial_worker = worker
        worker.start()

        QTimer.singleShot(250, lambda: self._send_raw_cmd(tp.CMD_GET_DEVICE_INFO))
        QTimer.singleShot(400, lambda: self._send_raw_cmd(tp.CMD_GET_DS_CAL))

    def _stop_serial_worker(self) -> None:
        if self.serial_worker is not None:
            self.serial_worker.stop()
            self.serial_worker = None
        self.btn_connect.setText("Connect UART")
        self._set_conn_badge("DISCONNECTED", "badge_neutral")

    def _on_demo_button_toggled(self, checked: bool) -> None:
        if checked:
            self.start_demo_simulator()
        else:
            self.stop_demo_simulator()

    def start_demo_simulator(self) -> None:
        if self.serial_worker is not None:
            self._stop_serial_worker()
        if self.demo_worker is not None:
            return

        worker = DemoBurstWorker(
            rate_hz=120,
            anchors=list(self.engine.anchors.values()),
            parent=self,
        )
        worker.motion_profile = self.demo_profile_combo.currentText()
        worker.batch_ready.connect(self._on_telemetry_batch)
        worker.link_status.connect(self._on_link_status)
        worker.log_event.connect(self._log_event)
        self.demo_worker = worker
        self.btn_demo.setChecked(True)
        worker.start()

    def stop_demo_simulator(self) -> None:
        if self.demo_worker is not None:
            self.demo_worker.stop()
            self.demo_worker = None
        self.btn_demo.setChecked(False)
        if self.serial_worker is None:
            self._set_conn_badge("DISCONNECTED", "badge_neutral")

    def _on_demo_profile_changed(self, profile: str) -> None:
        if self.demo_worker is not None:
            self.demo_worker.motion_profile = profile

    def _active_worker(self):
        return self.serial_worker or self.demo_worker

    def _on_link_status(self, state: str, detail: str) -> None:
        if state in ("connected", "online"):
            is_demo = "Simulator" in detail or "DEMO" in detail.upper()
            self._set_conn_badge(detail, "badge_info" if is_demo else "badge_ok")
            if not is_demo:
                self.btn_connect.setText("Disconnect")
        elif state == "warn":
            self._set_conn_badge("UART WARN", "badge_err")
        else:
            self._set_conn_badge("DISCONNECTED", "badge_neutral")
            self.btn_connect.setText("Connect UART")

    def _set_conn_badge(self, text: str, object_name: str) -> None:
        self.conn_badge.setText(text)
        self.conn_badge.setObjectName(object_name)
        self.conn_badge.style().unpolish(self.conn_badge)
        self.conn_badge.style().polish(self.conn_badge)

    # ------------------------------------------------------------------
    # High-Rate Telemetry Ingestion & 30 Hz UI Refresh
    # ------------------------------------------------------------------

    def _on_telemetry_batch(self, batch: List[Tuple[object, float]]) -> None:
        for msg, rx_ts in batch:
            try:
                self.engine.ingest_message(msg, rx_ts)
                if self.csv_logger is not None and isinstance(msg, tp.RangeBurstMessage):
                    sol = self.engine.latest_solution
                    pos_xyz = (sol.x, sol.y, sol.z) if sol is not None else None
                    rms_m = sol.rms_m if sol is not None else None
                    self.csv_logger.log_burst(msg, pos_xyz=pos_xyz, rms_m=rms_m, engine=self.engine)
                elif self.csv_logger is not None and isinstance(msg, tp.RxQualityMessage):
                    self.csv_logger.log_rx_quality(msg)
                elif self.csv_logger is not None and isinstance(
                    msg, (tp.DiagSystemMessage, tp.DiagAnchorMessage, tp.StatsMessage)
                ):
                    self.csv_logger.log_diag(msg, rx_ts)
                elif isinstance(msg, tp.CmdAckMessage):
                    self._log_event(
                        f"CMD_ACK cmd=0x{msg.command_id:02X} result={msg.result_name} ({'OK' if msg.ok else 'ERR'})"
                    )
                    if msg.command_id in (tp.CMD_GET_DS_CAL, tp.CMD_SET_DS_CAL):
                        self._refresh_anchor_cfg_fw_column()
            except Exception as exc:
                self._log_event(f"Telemetry ingest warning ({type(msg).__name__}): {exc}")

    def _on_ui_refresh_tick(self) -> None:
        try:
            eng = self.engine
            sol = eng.latest_solution

            # 1. Update Top KPI Cards
            if sol is not None:
                self.card_pos.update_metric(
                    f"{sol.x:+.3f}, {sol.y:+.3f}, {sol.z:+.3f} m",
                    f"{sol.filter_mode} | {sol.dimension}D ({sol.motion_state}) | Anchors: {len(sol.used_anchor_ids)} ({','.join(f'A{a}' for a in sol.used_anchor_ids)})",
                )
                excl = ",".join(f"A{a}" for a in sol.excluded_anchor_ids) if sol.excluded_anchor_ids else "none"
                self.card_residual.update_metric(
                    f"RMS: {sol.rms_m * 1000.0:.1f} mm [{sol.stability_state}]",
                    f"Max: {sol.max_residual_m * 1000.0:.1f} mm | Jump: {sol.step_jump_mm:.0f} mm | Excl: {excl}",
                )
                dop = sol.dop
                coplanar_tag = " [COPLANAR Z]" if dop.coplanar_warning else ""
                self.card_dop.update_metric(
                    f"PDOP: {dop.pdop:.2f}{coplanar_tag}",
                    f"HDOP: {dop.hdop:.2f} | VDOP: {dop.vdop:.2f} | GDOP: {dop.gdop:.2f}",
                )
            else:
                self.card_pos.update_metric("-- , -- , -- m", f"{eng.position_filter_mode} | Waiting for >={eng.solver_dimension + 1} anchors")
                self.card_residual.update_metric("No current fix", "Waiting for reliable ranges")

            temp_str = f"{eng.temperature_c:.1f}°C" if eng.temperature_c is not None else "--°C"
            log_str = f" | Log:{self.csv_logger.rows_written}" if self.csv_logger is not None else ""
            self.card_rate.update_metric(
                f"{eng.cycle_rate_hz:.1f} Hz",
                f"Bursts: {eng.total_bursts} | Lost: {eng.lost_cycles} | {temp_str}{log_str}",
            )

            if eng.device_info is not None:
                di = eng.device_info
                role_str = "TAG" if di.role == 1 else f"Role {di.role}"
                self.lbl_sidebar_fw.setText(
                    f"FW git:{di.git_hash:08X} (v{di.frame_version}) | {role_str} #{di.node_address}\n"
                    f"Mode: {eng.ranging_mode_name} | Baud: {di.uart_baud}"
                )

            # 2. Repaint active page widgets only
            page_idx = self.pages.currentIndex()
            if page_idx == 0:
                self.map_canvas.update()
                self.rolling_chart.update()
                self._update_live_anchor_table()
                self._update_eval_status_bar()
            elif page_idx == 1:
                self._update_calibration_page()
            elif page_idx == 2:
                self._update_diagnostics_page()
            elif page_idx == 3:
                self._update_replay_playback_tick()
            elif page_idx == 4:
                self.up4000_panel.refresh()
        except Exception as exc:
            self._log_event(f"UI refresh warning: {exc}")

    def _update_live_anchor_table(self) -> None:
        eng = self.engine
        valid_cnt = 0
        now_s = time.monotonic()

        for row in range(8):
            aid = row + 1
            st = eng.live[aid]
            fresh = st.is_fresh(now_s) and st.raw_mm > 0
            if fresh:
                valid_cnt += 1

            if not fresh:
                status_txt = "OFF"
            elif st.valid or st.fw_calibrated:
                status_txt = "FW_CAL"
            elif eng.allow_uncalibrated_positioning:
                status_txt = "HOST"
            else:
                status_txt = "UNCAL"
            self.live_anchor_table.item(row, 1).setText(status_txt)
            self.live_anchor_table.item(row, 2).setText(f"{st.rate_hz:.0f}")

            raw_txt = f"{st.raw_mm / 1000.0:.3f}" if st.raw_mm > 0 else "--"
            cal_mm = st.host_filtered_mm if (eng.use_host_filter_for_pos and st.host_filtered_mm is not None) else st.corrected_mm
            cal_txt = f"{cal_mm / 1000.0:.3f}" if st.raw_mm > 0 else "--"
            self.live_anchor_table.item(row, 3).setText(raw_txt)
            self.live_anchor_table.item(row, 4).setText(cal_txt)

            self.live_anchor_table.item(row, 5).setText(
                f"{st.residual_mm:+.1f}" if st.residual_mm is not None else "--"
            )
            self.live_anchor_table.item(row, 5).setToolTip(f"Residual: {st.residual_mm:+.1f} mm" if st.residual_mm is not None else "No residual")
            self.live_anchor_table.item(row, 6).setText(
                f"{st.nlos_db:.1f}" if st.nlos_db is not None else "--"
            )
            q = st.quality
            quality_cell = self.live_anchor_table.item(row, 7)
            quality_cell.setText(f"{q.score:.0f}" if fresh and q.score is not None else "--")
            quality_cell.setForeground(QColor({"GOOD": "#10b981", "WARN": "#f59e0b", "POOR": "#ef4444"}.get(q.label, "#94a3b8")))
            applied = eng.position_filter_mode == "robust_ekf" and eng.use_signal_quality
            details = f"{q.label}; heuristic score, not LOS probability. Extra weighting: {'ON' if applied else 'OFF'}.\n"
            details += " | ".join(q.reasons) or "No adverse indicators"
            details += "\nFeatures: " + ", ".join(q.features)
            if q.diagnostic_age_s is not None:
                details += f"\nRaw diagnostics age: {q.diagnostic_age_s * 1000:.0f} ms"
            else:
                details += "\nRaw diagnostics unavailable/stale; using available indicators."
            quality_cell.setToolTip(details)
            radio_scale = eng.applied_radio_variance_scale(aid)
            self.live_anchor_table.item(row, 8).setText(f"{radio_scale:.1f}" if fresh and radio_scale is not None else "--")
            self.live_anchor_table.item(row, 8).setToolTip(details + "\nRadio variance factor; innovation and EKF health weights also apply.")

        self.lbl_burst_meta.setText(
            f"Burst Seq: {eng.last_cycle_seq if eng.last_cycle_seq is not None else '--'} | Reachable: {valid_cnt}/8"
        )

    def _update_eval_status_bar(self) -> None:
        eng = self.engine
        if eng.accuracy_eval_active:
            done = len(eng._accuracy_fixes)
            target = eng.accuracy_target_samples
            self.btn_eval_run.setText(f"Stop ({done}/{target})")
            self.lbl_eval_summary.setText(f"Sampling live 2D/3D positions... {done}/{target} fixes collected.")
            return

        self.btn_eval_run.setText("Measure Accuracy")
        r = eng.latest_accuracy_report
        if r is not None:
            bx, by, bz = (v * 1000.0 for v in r.bias_xyz_m)
            self.lbl_eval_summary.setText(
                f"[{r.dimension}D N={r.samples}] 2D RMSE: {r.rmse_2d_m * 1000.0:.1f} mm | CEP50: {r.cep50_m * 1000.0:.1f} mm | CEP95: {r.cep95_m * 1000.0:.1f} mm  ||  "
                f"3D RMSE: {r.rmse_3d_m * 1000.0:.1f} mm | SEP50: {r.sep50_m * 1000.0:.1f} mm | SEP95: {r.sep95_m * 1000.0:.1f} mm  ||  "
                f"Bias (X,Y,Z): ({bx:+.1f}, {by:+.1f}, {bz:+.1f}) mm"
            )

    def _update_calibration_page(self) -> None:
        eng = self.engine
        self._update_joint_calibration_table()
        done = eng.cal_collected
        target = eng.cal_target_samples
        if eng.calibration_active:
            pct = int(round(100.0 * min(done, target) / max(1, target)))
            self.cal_progress.setValue(pct)
            slow = sorted(
                (c, aid) for aid, c in eng.cal_anchor_counts.items() if c < target
            )[:3]
            slow_txt = (
                " · slowest " + ", ".join(f"A{aid} {c}/{target}" for c, aid in slow) if slow else ""
            )
            self.lbl_cal_status.setText(
                f"Capturing {pct}% · {eng.cal_elapsed_s:.0f}/{eng.cal_timeout_s:.0f} s{slow_txt}"
            )
        else:
            self.cal_progress.setValue(100 if eng.spot_captures else 0)
            self.lbl_cal_status.setText(
                eng.cal_last_message or f"Ready — {len(eng.spot_captures)} spot(s) captured."
            )

        cal_warnings = eng.calibration_warnings()
        if cal_warnings:
            shown = cal_warnings[-4:]
            more = len(cal_warnings) - len(shown)
            text = "⚠ " + "\n⚠ ".join(shown) + (f"\n(+{more} earlier warnings)" if more > 0 else "")
            if self.lbl_cal_warning.text() != text:
                self.lbl_cal_warning.setText(text)
            self.lbl_cal_warning.setVisible(True)
        elif not self.lbl_cal_warning.isHidden():
            self.lbl_cal_warning.setText("")
            self.lbl_cal_warning.setVisible(False)

        agg = eng.aggregated_calibration()
        latest_by_anchor: Dict[int, Tuple[str, CalibrationAnchorResult]] = {}
        for cap in eng.spot_captures:
            spot_label = (
                f"1D ({cap.reference_distance_m:.2f}m)"
                if cap.single_anchor_id is not None and cap.reference_distance_m is not None
                else (
                    f"({cap.spot_xyz[0]:.2f},{cap.spot_xyz[1]:.2f},{cap.spot_xyz[2]:.2f})"
                    if cap.spot_xyz is not None
                    else f"Spot #{cap.spot_index}"
                )
            )
            for aid, res in cap.anchor_results.items():
                latest_by_anchor[aid] = (spot_label, res)

        for row in range(8):
            aid = row + 1
            cfg = eng.anchors[aid]
            entry = latest_by_anchor.get(aid)
            agg_entry = agg.get(aid)

            if entry is not None:
                spot_str, rec = entry
                self.cal_table.item(row, 1).setText(spot_str)
                self.cal_table.item(row, 2).setText(str(rec.samples))
                self.cal_table.item(row, 3).setText(f"{rec.true_distance_mm / 1000.0:.4f}")
                self.cal_table.item(row, 4).setText(f"{rec.median_raw_mm / 1000.0:.4f}")
                self.cal_table.item(row, 5).setText(f"{rec.bias_mm:+.2f}")
                self.cal_table.item(row, 6).setText(
                    (f"{agg_entry[0]:+.1f} ±{eng.joint_calibration().biases[aid].std_mm:.0f} J"
                     if eng.active_calibration_method() == "joint" else f"{agg_entry[0]:+.1f} ({agg_entry[2]}x) M")
                    if agg_entry is not None else "--"
                )
                self.cal_table.item(row, 7).setText(f"{rec.sigma_mm:.2f}")
                self.cal_table.item(row, 8).setText(
                    f"{rec.quality} | Host:{cfg.bias_mm:+.1f}mm | FW:{cfg.fw_bias_mm:+.1f}mm"
                )
            else:
                self.cal_table.item(row, 1).setText("--")
                self.cal_table.item(row, 2).setText("0")
                self.cal_table.item(row, 3).setText("--")
                self.cal_table.item(row, 4).setText("--")
                self.cal_table.item(row, 5).setText("--")
                self.cal_table.item(row, 6).setText("--")
                self.cal_table.item(row, 7).setText("--")
                self.cal_table.item(row, 8).setText(
                    f"Host:{cfg.bias_mm:+.1f}mm | FW:{cfg.fw_bias_mm:+.1f}mm ({'CAL' if cfg.fw_calibrated else 'UNCAL'})"
                )

    def _on_calibration_method_changed(self) -> None:
        self.engine.set_calibration_method(self.combo_cal_method.currentData())
        self._update_calibration_page()

    def _suggest_calibration_spots(self) -> None:
        try:
            points = suggest_calibration_spots(list(self.engine.anchors.values()), 4)
        except ValueError as exc:
            self.lbl_spot_plan.setText(str(exc))
            return
        self.lbl_spot_plan.setText(" · ".join(f"P{i}: ({x:.3f}, {y:.3f}, {z:.3f}) m"
                                               for i, (x, y, z) in enumerate(points, 1)))
        captured = [c.spot_xyz for c in self.engine.spot_captures if c.spot_xyz is not None]
        first = next((p for p in points if not any(sum((p[k] - c[k]) ** 2 for k in range(3)) < 0.01 ** 2
                                                 for c in captured)), points[0])
        self._set_spot_xyz(*first)

    def _remove_selected_calibration_spot(self) -> None:
        row = self.cal_spot_table.currentRow()
        if row >= 0:
            item = self.cal_spot_table.item(row, 0)
            if item is not None:
                self.engine.remove_capture(item.data(Qt.ItemDataRole.UserRole))
                self._update_calibration_page()

    def _update_joint_calibration_table(self) -> None:
        eng = self.engine
        result = eng.joint_calibration()
        captures = [c for c in eng.spot_captures if c.spot_xyz is not None and c.single_anchor_id is None]
        if result is not None and result.method == "joint":
            rms = f"{result.residual_rms_mm:.1f}" if result.residual_rms_mm is not None else "--"
            cond = f"{result.condition_number:.1f}" if result.identifiable else "infinite"
            self.lbl_joint_status.setText(f"Joint: {len(result.spots)} spots · rank {result.rank}/{result.unknowns} · "
                                          f"cond {cond} · RMS {rms} mm · applying {eng.active_calibration_method()}")
        elif result is not None and result.method == "failed":
            self.lbl_joint_status.setText("Joint solve unavailable — using per-spot median. " +
                                          " ".join(w.message for w in result.warnings))
        else:
            self.lbl_joint_status.setText(f"Joint solve needs at least 2 spot captures (have {len(captures)}) — using per-spot median")
        dark = self.theme_manager is not None and self.theme_manager.is_dark
        key = (tuple(id(c) for c in captures), id(result), dark)
        if key == self._cal_spot_table_key:
            return
        selected = self.cal_spot_table.item(self.cal_spot_table.currentRow(), 0)
        selected_id = selected.data(Qt.ItemDataRole.UserRole) if selected else None
        self.cal_spot_table.setRowCount(len(captures))
        solved = {s.spot_index: s for s in result.spots} if result is not None else {}
        flagged = {w.spot_index for w in result.warnings if w.code == "SPOT_OFFSET"} if result is not None else set()
        for row, capture in enumerate(captures):
            s = solved.get(capture.spot_index)
            xyz = lambda p: ", ".join(f"{v:.3f}" for v in p)
            texts = [str(capture.spot_index), xyz(capture.spot_xyz), xyz(s.estimated_xyz) if s else "--",
                     f"{s.offset_m * 100:.1f}" if s else "--", f"{s.residual_rms_mm:.1f}" if s else "--",
                     ", ".join(f"A{a}" for a in s.outlier_anchor_ids) if s else "--"]
            for col, value in enumerate(texts):
                item = QTableWidgetItem(value)
                if col == 0:
                    item.setData(Qt.ItemDataRole.UserRole, capture.spot_index)
                if capture.spot_index in flagged:
                    item.setBackground(QColor("#7F1D1D" if dark else "#FEE2E2"))
                    item.setForeground(QColor("#991B1B"))
                self.cal_spot_table.setItem(row, col, item)
            if capture.spot_index == selected_id:
                self.cal_spot_table.selectRow(row)
        self._cal_spot_table_key = key

    def _update_diagnostics_page(self) -> None:
        eng = self.engine
        ds = eng.diag_system
        if ds is not None:
            uptime_s = ds.uptime_ms / 1000.0
            temp_txt = f"{ds.temperature_c:.1f}" if ds.temperature_c is not None else "--"
            vbat_txt = f"{ds.vbat_v:.2f}" if ds.vbat_v is not None else "--"
            self.lbl_diag_summary.setText(
                f"System Diag — Uptime: {uptime_s:.1f}s | ActiveMask: 0x{ds.active_mask:02X} | "
                f"Temp: {temp_txt} °C | Vbat: {vbat_txt} V"
            )
        tracker = eng.diag_tracker
        rates_text = format_rates(tracker.system_rates(), tracker.anchor_rates())
        if self.lbl_diag_rates.text() != rates_text:
            self.lbl_diag_rates.setText(rates_text)
        hints = tracker.interpret(session=True)
        hints_text = "⚠ " + "\n⚠ ".join(hints) if hints else ""
        if self.lbl_diag_hints.text() != hints_text:
            self.lbl_diag_hints.setText(hints_text)
        self.lbl_diag_hints.setVisible(bool(hints))
        log_text = "\n".join(self._event_log[-100:])
        if self.event_console.toPlainText() != log_text:
            self.event_console.setPlainText(log_text)

    # ------------------------------------------------------------------
    # Solver & Viewport Actions
    # ------------------------------------------------------------------

    def set_dimension_mode(self, dim: int) -> None:
        self.engine.solver_dimension = 2 if dim == 2 else 3
        self.btn_dim_2d.setChecked(self.engine.solver_dimension == 2)
        self.btn_dim_3d.setChecked(self.engine.solver_dimension == 3)
        now_s = time.monotonic()
        self.engine._solve_current_position(now_s, max(s.tag_time_s for s in self.engine.live.values()))

    def _set_canvas_view_mode(self, mode: str) -> None:
        self.map_canvas.view_mode = mode
        self.map_canvas.update()

    def _on_fixed_z_changed(self, val: float) -> None:
        self.engine.fixed_z_m = float(val)

    def _on_solver_flags_changed(self) -> None:
        self.engine.set_position_filter_mode(self.filter_mode_combo.currentData())
        self.chk_use_filtered.setEnabled(self.engine.position_filter_mode == "legacy")
        self.engine.allow_uncalibrated_positioning = self.chk_use_uncal.isChecked()
        self.engine.use_host_filter_for_pos = self.chk_use_filtered.isChecked()
        self.engine.use_nlos_downweight = self.chk_nlos_weight.isChecked()
        self.engine.use_signal_quality = self.chk_signal_quality.isChecked()
        self.chk_signal_quality.setEnabled(self.engine.position_filter_mode == "robust_ekf")

    def _on_canvas_overlay_changed(self) -> None:
        self.map_canvas.show_rays = self.chk_show_rays.isChecked()
        self.map_canvas.show_ranges = self.chk_show_ranges.isChecked()
        self.map_canvas.show_trail = self.chk_show_trail.isChecked()
        self.map_canvas.color_by_stability = self.chk_stability_colors.isChecked()
        self.map_canvas.update()

    def _mark_live_waypoint(self) -> None:
        label = self.edit_waypoint_label.text().strip()
        wp = self.engine.mark_waypoint(label)
        if wp is not None:
            if label.endswith("_start"):
                self.edit_waypoint_label.setText(label[:-6] + "_end")
            self._log_event(f"Marked Waypoint {wp[1]} at ({wp[2]:+.3f}, {wp[3]:+.3f}, {wp[4]:+.3f}) m")
            if self.csv_logger is not None:
                self.csv_logger._write_metadata_snapshot()
            self.map_canvas.update()

    def _clear_trail(self) -> None:
        self.engine.clear_trail_and_waypoints()
        self.map_canvas.update()

    def _on_chart_controls_changed(self) -> None:
        target = int(self.chart_target_combo.currentData() or 0)
        win_s = float(self.chart_win_combo.currentData() or 10.0)
        self.rolling_chart.selected_anchor_id = target
        self.rolling_chart.window_s = win_s
        self.rolling_chart.update()

    # ------------------------------------------------------------------
    # Position Accuracy Benchmark Actions
    # ------------------------------------------------------------------

    def _start_or_stop_position_eval(self) -> None:
        if self.engine.accuracy_eval_active:
            self.engine.stop_accuracy_eval()
            self.map_canvas.show_reference = False
            return
        ref_xyz = (
            float(self.spin_eval_x.value()),
            float(self.spin_eval_y.value()),
            float(self.spin_eval_z.value()),
        )
        self.map_canvas.reference_xyz = ref_xyz
        self.map_canvas.show_reference = True
        self.engine.start_accuracy_eval(reference_xyz=ref_xyz, target_samples=int(self.spin_eval_n.value()))

    def _export_position_eval_report(self) -> None:
        r = self.engine.latest_accuracy_report
        if r is None:
            QMessageBox.information(self, "No Accuracy Report", "Run 'Measure Accuracy' first to generate a report.")
            return
        path_str, _ = QFileDialog.getSaveFileName(
            self,
            "Export 2D/3D Accuracy Report",
            str(NEW_GUI_DIR / f"accuracy_report_{r.dimension}d.json"),
            "JSON Files (*.json)",
        )
        if not path_str:
            return
        payload = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "dimension": r.dimension,
            "samples": r.samples,
            "reference_xyz_m": list(r.reference_xyz),
            "mean_xyz_m": list(r.mean_xyz),
            "bias_xyz_m": list(r.bias_xyz_m),
            "std_xyz_m": list(r.std_xyz_m),
            "rmse_2d_m": r.rmse_2d_m,
            "cep50_m": r.cep50_m,
            "cep95_m": r.cep95_m,
            "rmse_3d_m": r.rmse_3d_m,
            "sep50_m": r.sep50_m,
            "sep95_m": r.sep95_m,
            "mean_pdop": r.mean_pdop,
        }
        Path(path_str).write_text(json.dumps(payload, indent=2), encoding="utf-8")

    # ------------------------------------------------------------------
    # Calibration Actions (1D & 2D/3D Spot + TAG SET_DS_CAL)
    # ------------------------------------------------------------------

    def _set_spot_xyz(self, x: float, y: float, z: float) -> None:
        self.spin_cal_x.setValue(x)
        self.spin_cal_y.setValue(y)
        self.spin_cal_z.setValue(z)

    def _start_spot_calibration(self) -> None:
        spot = (
            float(self.spin_cal_x.value()),
            float(self.spin_cal_y.value()),
            float(self.spin_cal_z.value()),
        )
        self.engine.start_spot_calibration(spot_xyz=spot, target_samples=int(self.spin_cal_samples.value()))

    def _start_1d_calibration(self) -> None:
        aid = int(self.combo_1d_anchor.currentData() or 1)
        dist = float(self.spin_1d_dist.value())
        self.engine.start_single_anchor_calibration(
            anchor_id=aid,
            reference_m=dist,
            target_samples=int(self.spin_cal_samples.value()),
        )

    def _persist_rig_config(self) -> None:
        if self.demo_worker is not None:
            return
        try:
            save_anchor_layout_json(LAST_RIG_PATH, list(self.engine.anchors.values()))
        except Exception:
            pass

    def _calibration_warnings_accepted(self, action: str) -> bool:
        """Ask before using captures that failed the spot sanity checks."""
        warnings = self.engine.calibration_warnings()
        if not warnings:
            return True
        answer = QMessageBox.warning(
            self,
            "Calibration looks inconsistent",
            "\n\n".join(warnings[-6:]) + f"\n\n{action} anyway?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return answer == QMessageBox.StandardButton.Yes

    def _confirm_apply_calibration_to_host(self) -> None:
        if self._calibration_warnings_accepted("Apply these biases to the host solver"):
            self.apply_calibrated_biases_to_host()

    def _confirm_write_calibrated_biases_to_tag(self) -> None:
        if self._calibration_warnings_accepted("Write these biases to the TAG"):
            self._write_calibrated_biases_to_tag()

    def apply_calibrated_biases_to_host(self) -> Dict[int, float]:
        applied = self.engine.apply_calibration_to_host()
        self._populate_anchor_config_table()
        self._persist_rig_config()
        return applied

    def _write_calibrated_biases_to_tag(self) -> None:
        worker = self._active_worker()
        if worker is None:
            QMessageBox.warning(self, "Not Connected", "Connect to TAG UART or start Demo Simulator first.")
            return
        agg = {aid: vals[0] for aid, vals in self.engine.aggregated_calibration().items()}
        if not agg:
            agg = {aid: cfg.bias_mm for aid, cfg in self.engine.anchors.items() if cfg.enabled}

        for idx, (aid, bias_mm) in enumerate(sorted(agg.items())):
            QTimer.singleShot(
                idx * 45,
                lambda a=aid, b=bias_mm: worker.send_set_ds_cal(anchor_id=a, bias_mm=b, calibrated=True),
            )
        QTimer.singleShot(len(agg) * 45 + 100, lambda: worker.send_command(tp.CMD_GET_DS_CAL))

    def _save_tag_settings_to_flash(self) -> None:
        worker = self._active_worker()
        if worker is not None:
            worker.send_save_settings_sequence()

    def _request_tag_ds_cal(self) -> None:
        self._send_raw_cmd(tp.CMD_GET_DS_CAL)

    def _clear_calibration_captures(self) -> None:
        self.engine.spot_captures.clear()
        self._update_calibration_page()

    def _export_calibration_records_dialog(self) -> None:
        if not self.engine.spot_captures:
            QMessageBox.information(self, "No Captures", "Capture at least one calibration spot first.")
            return
        path_str, _ = QFileDialog.getSaveFileName(
            self,
            "Export Calibration Records",
            str(NEW_GUI_DIR / "calibration_summary.csv"),
            "CSV Files (*.csv);;JSON Files (*.json)",
        )
        if not path_str:
            return
        path = Path(path_str)
        if path.suffix.lower() == ".json":
            data = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "calibration_summary": self.engine.calibration_summary(),
                "aggregated_biases_mm": {
                    str(k): {"median_bias_mm": v[0], "mean_sigma_mm": v[1], "spots": v[2]}
                    for k, v in self.engine.aggregated_calibration().items()
                },
                "captures": [
                    {
                        "spot_index": c.spot_index,
                        "spot_xyz": list(c.spot_xyz) if c.spot_xyz else None,
                        "single_anchor_id": c.single_anchor_id,
                        "reference_distance_m": c.reference_distance_m,
                        "anchors": {
                            str(aid): {
                                "samples": r.samples,
                                "true_distance_mm": r.true_distance_mm,
                                "median_raw_mm": r.median_raw_mm,
                                "bias_mm": r.bias_mm,
                                "offset_mm": r.offset_mm,
                                "sigma_mm": r.sigma_mm,
                                "drift_mm": r.drift_mm,
                                "quality": r.quality,
                            }
                            for aid, r in c.anchor_results.items()
                        },
                    }
                    for c in self.engine.spot_captures
                ],
            }
            path.write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")
        else:
            with path.open("w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(
                    [
                        "spot_index",
                        "anchor_id",
                        "samples",
                        "true_distance_m",
                        "raw_median_m",
                        "bias_mm",
                        "offset_mm",
                        "sigma_mm",
                        "drift_mm",
                        "quality",
                    ]
                )
                for c in self.engine.spot_captures:
                    for aid, r in sorted(c.anchor_results.items()):
                        w.writerow(
                            [
                                c.spot_index,
                                f"A{aid}",
                                r.samples,
                                f"{r.true_distance_mm / 1000.0:.4f}",
                                f"{r.median_raw_mm / 1000.0:.4f}",
                                f"{r.bias_mm:.3f}",
                                f"{r.offset_mm:.3f}",
                                f"{r.sigma_mm:.3f}",
                                f"{r.drift_mm:.3f}",
                                r.quality,
                            ]
                        )

    # ------------------------------------------------------------------
    # Anchor Rig Layout & Command Helpers
    # ------------------------------------------------------------------

    def _populate_anchor_config_table(self) -> None:
        self._editing_anchor_table = True
        try:
            mono = QFont(FONT_MONO, 9)
            for row in range(8):
                aid = row + 1
                cfg = self.engine.anchors[aid]

                it_id = QTableWidgetItem(f"A{aid}")
                it_id.setFlags(it_id.flags() & ~Qt.ItemFlag.ItemIsEditable)
                it_id.setForeground(QColor(ANCHOR_PALETTE[row]))
                it_id.setFont(mono)
                self.anchor_cfg_table.setItem(row, 0, it_id)

                it_en = QTableWidgetItem("1" if cfg.enabled else "0")
                it_en.setFont(mono)
                self.anchor_cfg_table.setItem(row, 1, it_en)

                for col, val, fmt in (
                    (2, cfg.x, "{:.4f}"),
                    (3, cfg.y, "{:.4f}"),
                    (4, cfg.z, "{:.4f}"),
                    (5, cfg.bias_mm, "{:+.2f}"),
                    (6, cfg.sigma_mm, "{:.1f}"),
                ):
                    it = QTableWidgetItem(fmt.format(val))
                    it.setFont(mono)
                    self.anchor_cfg_table.setItem(row, col, it)

                it_fw = QTableWidgetItem(
                    f"{cfg.fw_bias_mm:+.1f} mm ({'CAL' if cfg.fw_calibrated else 'UNCAL'})"
                )
                it_fw.setFlags(it_fw.flags() & ~Qt.ItemFlag.ItemIsEditable)
                it_fw.setFont(mono)
                self.anchor_cfg_table.setItem(row, 7, it_fw)
                self.anchor_cfg_table.setRowHeight(row, 30)
        finally:
            self._editing_anchor_table = False

    def _refresh_anchor_cfg_fw_column(self) -> None:
        self._editing_anchor_table = True
        try:
            for row in range(8):
                aid = row + 1
                cfg = self.engine.anchors[aid]
                it_fw = self.anchor_cfg_table.item(row, 7)
                if it_fw is not None:
                    it_fw.setText(
                        f"{cfg.fw_bias_mm:+.1f} mm ({'CAL' if cfg.fw_calibrated else 'UNCAL'})"
                    )
        finally:
            self._editing_anchor_table = False

    def _on_anchor_cfg_cell_edited(self, item: QTableWidgetItem) -> None:
        if self._editing_anchor_table:
            return
        row = item.row()
        aid = row + 1
        cfg = self.engine.anchors.get(aid)
        if cfg is None:
            return
        try:
            en_txt = self.anchor_cfg_table.item(row, 1).text().strip().lower()
            cfg.enabled = en_txt in ("1", "true", "yes", "on")
            cfg.x = float(self.anchor_cfg_table.item(row, 2).text())
            cfg.y = float(self.anchor_cfg_table.item(row, 3).text())
            cfg.z = float(self.anchor_cfg_table.item(row, 4).text())
            cfg.bias_mm = float(self.anchor_cfg_table.item(row, 5).text())
            cfg.sigma_mm = max(1.0, float(self.anchor_cfg_table.item(row, 6).text()))
        except (ValueError, AttributeError):
            self._populate_anchor_config_table()
            return

        self.engine.set_anchors(list(self.engine.anchors.values()))
        if self.demo_worker is not None:
            self.demo_worker.set_anchors(list(self.engine.anchors.values()))
        self._persist_rig_config()
        self.map_canvas.reset_camera()

    def _apply_anchor_preset(self, configs: Sequence[AnchorConfig]) -> None:
        self.engine.set_anchors(configs)
        self._populate_anchor_config_table()
        if self.demo_worker is not None:
            self.demo_worker.set_anchors(list(self.engine.anchors.values()))
        self._persist_rig_config()
        self.map_canvas.reset_camera()

    def _import_rig_file_dialog(self) -> None:
        path_str, _ = QFileDialog.getOpenFileName(
            self,
            "Import Anchor Rig Layout",
            str(NEW_GUI_DIR),
            "Rig Text / JSON (*.txt *.json);;All Files (*)",
        )
        if not path_str:
            return
        path = Path(path_str)
        if path.suffix.lower() == ".json":
            configs = load_anchor_layout_json(path)
        else:
            configs = load_anchors_rig_txt(path)
        if configs:
            self._apply_anchor_preset(configs)

    def _export_rig_txt_dialog(self) -> None:
        path_str, _ = QFileDialog.getSaveFileName(
            self,
            "Export anchors_rig40.txt",
            str(NEW_GUI_DIR / "anchors_rig40.txt"),
            "Rig Text Files (*.txt);;JSON Files (*.json)",
        )
        if not path_str:
            return
        path = Path(path_str)
        if path.suffix.lower() == ".json":
            save_anchor_layout_json(path, list(self.engine.anchors.values()))
        else:
            save_anchors_rig_txt(path, list(self.engine.anchors.values()))

    def _push_enabled_anchor_mask_to_tag(self) -> None:
        worker = self._active_worker()
        if worker is None:
            return
        mask = 0
        for aid, cfg in self.engine.anchors.items():
            if cfg.enabled:
                mask |= 1 << (aid - 1)
        worker.send_command(tp.CMD_SET_ANCHOR_MASK, bytes((mask & 0xFF,)))

    def _send_selected_burst_preset(self) -> None:
        worker = self._active_worker()
        if worker is None:
            return
        key = str(self.burst_preset_combo.currentData() or "hz200")
        preset = BURST_PRESETS.get(key)
        if preset is not None:
            worker.send_command(tp.CMD_SET_BURST, preset.encode())

    def _enable_rx_quality_telemetry(self) -> None:
        worker = self._active_worker()
        if worker is None:
            return
        wanted = (
            tp.TELEM_FEATURE_RANGE_SNAPSHOT
            | tp.TELEM_FEATURE_RANGE_MEAS
            | tp.TELEM_FEATURE_DIAG
        )
        cur = self.engine.telemetry_features or 0
        worker.send_command(tp.CMD_SET_TELEMETRY, bytes(((cur | wanted) & 0xFF,)))

    def _send_raw_cmd(self, cmd_id: int, payload: bytes = b"") -> None:
        worker = self._active_worker()
        if worker is not None:
            worker.send_command(cmd_id, payload)

    # ------------------------------------------------------------------
    # CSV Logging & Theme Toggle
    # ------------------------------------------------------------------

    def _toggle_csv_logging(self, checked: bool) -> None:
        if checked:
            log_root = NEW_GUI_DIR / "logs"
            port_label = "DEMO" if self.demo_worker is not None else str(self.port_combo.currentData() or "UART")
            self.csv_logger = FastCsvLogger(log_root, port_label, engine=self.engine)
            self.btn_csv_log.setText("Log CSV: ON")
            self._log_event(f"Full 3D & Anchor CSV logging started -> {self.csv_logger.session_dir}")
        else:
            saved_csv: Optional[Path] = None
            if self.csv_logger is not None:
                saved_csv = self.csv_logger.csv_path
                rows = self.csv_logger.rows_written
                self.csv_logger.stop()
                self.csv_logger = None
                self._log_event(f"CSV logging stopped ({rows} rows) -> {saved_csv}")
            self.btn_csv_log.setText("Log CSV: OFF")
            if saved_csv is not None and saved_csv.is_file():
                try:
                    self._load_log_from_path(saved_csv)
                except Exception as exc:
                    self._log_event(f"Auto-load replay warning: {exc}")

    # ------------------------------------------------------------------
    # Log Replay & Instability Inspector Actions
    # ------------------------------------------------------------------

    def _load_latest_log_session(self) -> None:
        log_root = NEW_GUI_DIR / "logs"
        if not log_root.is_dir():
            QMessageBox.information(self, "No Logs Found", f"No logs directory found at {log_root}")
            return
        candidates = sorted(
            log_root.glob("**/uwb_fast_log.csv"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        if not candidates:
            QMessageBox.information(self, "No Logs Found", f"No uwb_fast_log.csv files found inside {log_root}")
            return
        self._load_log_from_path(candidates[0])

    def _open_log_session_dialog(self) -> None:
        log_root = NEW_GUI_DIR / "logs"
        start_dir = str(log_root if log_root.is_dir() else NEW_GUI_DIR)
        path_str, _ = QFileDialog.getOpenFileName(
            self,
            "Open UWB Session CSV Log",
            start_dir,
            "CSV Log Files (*.csv);;All Files (*)",
        )
        if not path_str:
            return
        self._load_log_from_path(Path(path_str))

    def _reload_current_log_session(self) -> None:
        if self.loaded_session is not None and self.loaded_session.csv_path.is_file():
            self._load_log_from_path(self.loaded_session.csv_path)

    def _load_log_from_path(self, csv_path: Path) -> None:
        if self._replay_loader is not None and self._replay_loader.isRunning():
            self._log_event("Replay is already loading")
            return
        self.replay_playing = False
        self.btn_replay_play.setText("Play")
        re_solve = self.chk_replay_resolve.isChecked()
        fallback = [AnchorConfig(**vars(a)) for a in self.engine.anchors.values()]
        self.lbl_replay_file.setText(f"Loading {csv_path.parent.name}...")
        worker = LogReplayWorker(
            csv_path=csv_path,
            fallback_anchors=fallback,
            re_solve=re_solve,
            max_frames=12000,
            filter_mode=self.engine.position_filter_mode,
            signal_quality=self.engine.use_signal_quality,
            parent=self,
        )
        worker.progress.connect(lambda done, total: self.lbl_replay_file.setText(
            f"Loading {csv_path.parent.name}: {done:,}/{total:,} rows"))
        worker.failed.connect(self._on_replay_load_failed)
        worker.session_ready.connect(self._on_replay_session_loaded)
        self._replay_loader = worker
        worker.start()

    def _on_replay_load_failed(self, reason: str) -> None:
        self.lbl_replay_file.setText("Replay load failed")
        self._log_event(f"Replay load failed: {reason}")

    def _on_replay_session_loaded(self, session: LoadedLogSession) -> None:
        csv_path = session.csv_path
        self.loaded_session = session
        self.replay_engine.set_anchors(session.anchors)
        self.replay_engine.clear_trail_and_waypoints()
        self.replay_engine.map_path = deque(session.map_path)
        for pt in session.map_path:
            self.replay_engine.position_trail.append((pt[0], pt[1], pt[2], pt[3]))
        self.replay_engine.waypoints = list(session.waypoints)
        self.replay_canvas.highlight_time_range = None
        self.replay_canvas.reset_camera()

        self.lbl_replay_file.setText(f"{csv_path.parent.name}/{csv_path.name} ({session.total_csv_rows:,} raw rows)")
        self.card_rep_session.update_metric(
            f"{session.duration_s:.1f} s ({len(session.frames):,} frames)",
            f"Folder: {csv_path.parent.name} | Raw CSV: {session.total_csv_rows:,} rows",
        )
        self.card_rep_loop.update_metric(
            f"3D Path: {session.path_length_3d_m:.2f} m",
            f"2D Loop: {session.path_length_2d_m:.2f} m | Waypoints: {len(session.waypoints)}",
        )
        self.card_rep_stability.update_metric(
            f"{session.stable_pct:.1f}% Stable",
            f"Warn: {session.warn_pct:.1f}% | Unstable: {session.unstable_pct:.1f}% | Segments: {len(session.unstable_segments)}",
        )

        # Populate Unstable Segments table
        mono = QFont(FONT_MONO, 8)
        self.unstable_table.setRowCount(len(session.unstable_segments))
        for r_idx, seg in enumerate(session.unstable_segments):
            vals = (
                f"#{seg.segment_id}",
                seg.severity,
                f"{seg.start_rel_s:.1f}–{seg.end_rel_s:.1f}s ({seg.duration_s:.1f}s)",
                f"({seg.xyz_m[0]:.2f}, {seg.xyz_m[1]:.2f}, {seg.xyz_m[2]:.2f})",
                f"{seg.max_rms_mm:.0f} mm",
                f"{seg.max_jump_mm:.0f} mm",
                seg.cause_summary,
            )
            for c_idx, txt in enumerate(vals):
                it = QTableWidgetItem(txt)
                it.setFont(mono)
                if c_idx == 1:
                    it.setForeground(QColor("#ef4444" if seg.severity == "UNSTABLE" else "#f59e0b"))
                self.unstable_table.setItem(r_idx, c_idx, it)
            self.unstable_table.setRowHeight(r_idx, 25)

        max_idx = max(0, len(session.frames) - 1)
        self.replay_slider.blockSignals(True)
        self.replay_slider.setRange(0, max_idx)
        self.replay_slider.setValue(max_idx)
        self.replay_slider.blockSignals(False)
        self._apply_replay_frame(max_idx)

    def _on_replay_slider_changed(self, idx: int) -> None:
        self._apply_replay_frame(idx)

    def _toggle_replay_play(self) -> None:
        if self.loaded_session is None or not self.loaded_session.frames:
            self._load_latest_log_session()
            if self.loaded_session is None or not self.loaded_session.frames:
                return
        self.replay_playing = not self.replay_playing
        if self.replay_playing and self.replay_frame_idx >= len(self.loaded_session.frames) - 1:
            self.replay_frame_idx = 0
        self.btn_replay_play.setText("Pause" if self.replay_playing else "Play")

    def _update_replay_playback_tick(self) -> None:
        if not self.replay_playing or self.loaded_session is None or not self.loaded_session.frames:
            return
        n = len(self.loaded_session.frames)
        dur = max(self.loaded_session.duration_s, 1.0)
        fps_log = n / dur
        step_f = max(0.25, (fps_log * 0.033) * self.replay_speed)
        self._replay_accum_f += step_f
        inc = int(self._replay_accum_f)
        if inc >= 1:
            self._replay_accum_f -= inc
            nxt = self.replay_frame_idx + inc
            if nxt >= n - 1:
                nxt = n - 1
                self.replay_playing = False
                self.btn_replay_play.setText("Play")
            self.replay_slider.blockSignals(True)
            self.replay_slider.setValue(nxt)
            self.replay_slider.blockSignals(False)
            self._apply_replay_frame(nxt)

    def _on_unstable_table_row_clicked(self, row: int, _col: int) -> None:
        if self.loaded_session is None or not (0 <= row < len(self.loaded_session.unstable_segments)):
            return
        seg = self.loaded_session.unstable_segments[row]
        self.replay_playing = False
        self.btn_replay_play.setText("Play")
        self.replay_canvas.highlight_time_range = (seg.start_rel_s, seg.end_rel_s)
        self.replay_slider.blockSignals(True)
        self.replay_slider.setValue(seg.peak_frame_idx)
        self.replay_slider.blockSignals(False)
        self._apply_replay_frame(seg.peak_frame_idx)

    def _step_unstable_segment(self, direction: int) -> None:
        if self.loaded_session is None or not self.loaded_session.unstable_segments:
            return
        segs = self.loaded_session.unstable_segments
        cur_row = self.unstable_table.currentRow()
        if cur_row < 0:
            nxt_row = 0 if direction >= 0 else len(segs) - 1
        else:
            nxt_row = (cur_row + direction) % len(segs)
        self.unstable_table.selectRow(nxt_row)
        self._on_unstable_table_row_clicked(nxt_row, 0)

    def _apply_replay_frame(self, idx: int) -> None:
        if self.loaded_session is None or not self.loaded_session.frames:
            return
        idx = max(0, min(idx, len(self.loaded_session.frames) - 1))
        self.replay_frame_idx = idx
        fr = self.loaded_session.frames[idx]
        dur = self.loaded_session.duration_s
        self.lbl_replay_time.setText(f"{fr.rel_time_s:.2f}s / {dur:.2f}s")

        res_m = {
            aid: (sn.res_mm / 1000.0)
            for aid, sn in fr.anchors.items()
            if sn.res_mm is not None
        }
        self.replay_engine.latest_solution = PositionResult(
            x=fr.x,
            y=fr.y,
            z=fr.z,
            rms_m=fr.rms_mm / 1000.0,
            max_residual_m=fr.max_res_mm / 1000.0,
            residuals_m=res_m,
            used_anchor_ids=fr.used_anchor_ids,
            excluded_anchor_ids=fr.excluded_anchor_ids,
            dimension=3,
            dop=DopMetrics(
                gdop=fr.pdop,
                pdop=fr.pdop,
                hdop=fr.hdop,
                vdop=fr.vdop,
                condition_number=1.5,
                coplanar_warning=False,
            ),
            tag_time_s=fr.rel_time_s,
            raw_x=fr.raw_x,
            raw_y=fr.raw_y,
            raw_z=fr.raw_z,
            speed_mps=fr.speed_mps,
            step_jump_mm=fr.step_jump_mm,
            motion_state=fr.motion_state,
            stability_state=fr.stability_state,
            instability_reason=fr.instability_reason,
        )

        wp_tag = f" | {fr.waypoint_label}" if fr.waypoint_label else ""
        reason_tag = f" | {fr.instability_reason}" if fr.instability_reason else ""
        self.card_rep_cursor.update_metric(
            f"{fr.x:+.3f}, {fr.y:+.3f}, {fr.z:+.3f} m [{fr.stability_state}]",
            f"t={fr.rel_time_s:.2f}s | RMS:{fr.rms_mm:.1f}mm | Jump:{fr.step_jump_mm:.0f}mm | {fr.speed_mps:.2f}m/s{wp_tag}{reason_tag}",
        )

        for row in range(8):
            aid = row + 1
            sn = fr.anchors.get(aid)
            if sn is None:
                continue
            self.replay_anchor_table.item(row, 1).setText(sn.status_str)
            self.replay_anchor_table.item(row, 2).setText(f"{sn.rate_hz:.0f}")
            self.replay_anchor_table.item(row, 3).setText(f"{sn.raw_mm / 1000.0:.3f}" if sn.raw_mm else "--")
            self.replay_anchor_table.item(row, 4).setText(f"{sn.filt_mm / 1000.0:.3f}" if sn.filt_mm else "--")
            self.replay_anchor_table.item(row, 5).setText(f"{sn.res_mm:+.1f}" if sn.res_mm is not None else "--")
            self.replay_anchor_table.item(row, 6).setText(f"{sn.nlos_db:.1f}" if sn.nlos_db is not None else "--")
            self.replay_anchor_table.item(row, 7).setText(f"{sn.sigma_mm:.1f}" if sn.sigma_mm is not None else "--")

        self.replay_canvas.update()

    def _toggle_theme(self) -> None:
        if self.theme_manager is None:
            return
        is_dark = bool(self.theme_manager.toggle())
        self.btn_theme.setText("Theme: Dark" if is_dark else "Theme: Light")
        self.map_canvas.is_dark = is_dark
        self.replay_canvas.is_dark = is_dark
        self.rolling_chart.is_dark = is_dark
        self.up4000_panel.set_dark_mode(is_dark)
        self.map_canvas.update()
        self.replay_canvas.update()
        self.rolling_chart.update()

    def closeEvent(self, event) -> None:  # noqa: N802
        self.up4000_listener.close()
        if self._replay_loader is not None:
            self._replay_loader.requestInterruption()
            self._replay_loader.wait()
        self._ui_timer.stop()
        if self.csv_logger is not None:
            self.csv_logger.stop()
            self.csv_logger = None
        if self.serial_worker is not None:
            self.serial_worker.stop()
            self.serial_worker = None
        if self.demo_worker is not None:
            self.demo_worker.stop()
            self.demo_worker = None
        super().closeEvent(event)


def main() -> int:
    parser = argparse.ArgumentParser(description="UWB Fast Studio — 2D/3D Positioning & Calibration GUI")
    parser.add_argument("--port", type=str, default=None, help="Initial serial COM port (e.g. COM6)")
    parser.add_argument("--baud", type=int, default=460800, help="UART baudrate (default: 460800)")
    parser.add_argument("--demo", action="store_true", help="Launch with built-in 120 Hz 3D burst simulator")
    parser.add_argument("--smoke-test", action="store_true", help="Auto-close after 1.2s for automated verification")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    win = UwbFastGuiWindow(start_demo=args.demo, initial_port=args.port, initial_baud=args.baud)
    win.show()

    if args.smoke_test:
        QTimer.singleShot(1200, win.close)

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
