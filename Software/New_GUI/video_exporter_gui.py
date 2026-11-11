"""Standalone & Integrated UWB Log-to-MP4 Video Export Studio (`video_exporter_gui.py`).

Can be launched:
1. Directly from `main_gui.py` Tab 4 ("Export MP4 Video...") as an independent process
   so video rendering never adds load or lag to the main UWB studio GUI.
2. Standalone as its own desktop GUI:
   `python video_exporter_gui.py`
3. Headlessly via CLI for batch export:
   `python video_exporter_gui.py --csv path/to/uwb_fast_log.csv --out replay.mp4 --speed 10 --headless`
"""

from __future__ import annotations

import argparse
from collections import deque
import glob
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Callable, Optional, Sequence

import numpy as np
from PyQt6.QtCore import QPointF, QRectF, Qt, QThread, QUrl, pyqtSignal
from PyQt6.QtGui import (
    QColor,
    QDesktopServices,
    QDoubleValidator,
    QFont,
    QImage,
    QMouseEvent,
    QPainter,
    QPaintEvent,
    QPen,
    QWheelEvent,
)
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

NEW_GUI_DIR = Path(__file__).resolve().parent
OLD_GUI_DIR = NEW_GUI_DIR.parent / "UWB_UART_GUI"
if str(NEW_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(NEW_GUI_DIR))
if str(OLD_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(OLD_GUI_DIR))

from core_state import (  # noqa: E402
    AnchorConfig,
    DEFAULT_PRESET_ROOM_66_36,
    DopMetrics,
    LoadedLogSession,
    PositionResult,
    ReplayFrame,
    UwbStateEngine,
    load_anchor_layout_json,
    load_log_session,
)
from design_tokens import (  # noqa: E402
    ANCHOR_PALETTE,
    DARK_QSS,
    FONT_MONO,
    LIGHT_QSS,
    SPACING_MD,
    SPACING_SM,
    ThemeManager,
)
from serial_worker import LogReplayWorker  # noqa: E402
from widgets_canvas import PositionMapCanvas  # noqa: E402

LAST_RIG_PATH = NEW_GUI_DIR / "last_rig_config.json"


def find_ffmpeg_binary() -> Optional[str]:
    """Locate ffmpeg executable in PATH or WinGet packages."""
    found = shutil.which("ffmpeg")
    if found:
        return found
    local_appdata = os.environ.get("LOCALAPPDATA", "")
    if local_appdata:
        pattern = str(Path(local_appdata) / "Microsoft" / "WinGet" / "Packages" / "*FFmpeg*" / "**" / "ffmpeg.exe")
        matches = glob.glob(pattern, recursive=True)
        if matches:
            return matches[0]
    return None


class Mp4VideoWriter:
    """Streams RGB24 frames to an H.264 `.mp4` file via FFmpeg pipe or OpenCV fallback."""

    def __init__(self, output_path: Path, width: int, height: int, fps: float = 30.0) -> None:
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.width = int(width) - (int(width) % 2)
        self.height = int(height) - (int(height) % 2)
        self.fps = float(fps)
        self._proc: Optional[subprocess.Popen] = None
        self._cv_writer = None
        self.backend = "none"

        ffmpeg_bin = find_ffmpeg_binary()
        if ffmpeg_bin:
            cmd = [
                ffmpeg_bin,
                "-y",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-vcodec",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                f"{self.width}x{self.height}",
                "-r",
                f"{self.fps:g}",
                "-i",
                "-",
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                "-preset",
                "veryfast",
                "-crf",
                "20",
                "-movflags",
                "+faststart",
                str(self.output_path),
            ]
            creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0) if os.name == "nt" else 0
            self._proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                creationflags=creationflags,
            )
            self.backend = "ffmpeg_h264"
        else:
            import cv2  # type: ignore

            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._cv_writer = cv2.VideoWriter(
                str(self.output_path), fourcc, self.fps, (self.width, self.height)
            )
            if not self._cv_writer.isOpened():
                raise RuntimeError(f"Could not open OpenCV VideoWriter for {self.output_path}")
            self.backend = "opencv_mp4v"

    def write_qimage(self, qimg: QImage) -> None:
        if qimg.width() != self.width or qimg.height() != self.height:
            qimg = qimg.scaled(self.width, self.height, Qt.AspectRatioMode.IgnoreAspectRatio)
        if qimg.format() != QImage.Format.Format_RGB888:
            qimg = qimg.convertToFormat(QImage.Format.Format_RGB888)

        ptr = qimg.constBits()
        ptr.setsize(qimg.sizeInBytes())
        arr = np.frombuffer(ptr, dtype=np.uint8).reshape((self.height, qimg.bytesPerLine()))
        rgb = np.ascontiguousarray(arr[:, : self.width * 3].reshape((self.height, self.width, 3)))

        if self._proc is not None and self._proc.stdin is not None:
            self._proc.stdin.write(rgb.tobytes())
        elif self._cv_writer is not None:
            bgr = np.ascontiguousarray(rgb[:, :, ::-1])
            self._cv_writer.write(bgr)

    def close(self) -> None:
        if self._proc is not None:
            try:
                if self._proc.stdin is not None:
                    self._proc.stdin.close()
                stderr_bytes = b""
                if self._proc.stderr is not None:
                    stderr_bytes = self._proc.stderr.read()
                    self._proc.stderr.close()
                rc = self._proc.wait(timeout=15)
                if rc != 0:
                    err_txt = stderr_bytes.decode("utf-8", errors="replace").strip()
                    raise RuntimeError(f"FFmpeg exited with code {rc}: {err_txt}")
            finally:
                self._proc = None
        if self._cv_writer is not None:
            self._cv_writer.release()
            self._cv_writer = None


class ReplayVideoFrameRenderer:
    """Offscreen QPainter renderer for UWB Replay Dashboard & 2D/3D Trajectory frames."""

    def __init__(self) -> None:
        self.engine = UwbStateEngine(initial_anchors=DEFAULT_PRESET_ROOM_66_36)
        self.canvas = PositionMapCanvas(self.engine)
        self.canvas.view_mode = "SPLIT"
        self.canvas.color_by_stability = True
        self.canvas.lock_map_path_bounds = True
        self.canvas.trail_mode = "PROGRESSIVE_GHOST"

        self.layout_mode: str = "DASHBOARD"  # "DASHBOARD" | "MAP_ONLY"
        self.is_dark: bool = False
        self.playback_speed: float = 10.0
        self._session: Optional[LoadedLogSession] = None

    def set_session(self, session: LoadedLogSession) -> None:
        self._session = session
        self.engine.set_anchors(session.anchors)
        self.engine.clear_trail_and_waypoints()
        self.engine.map_path = deque(session.map_path)
        for pt in session.map_path:
            self.engine.position_trail.append((pt[0], pt[1], pt[2], pt[3]))
        self.engine.waypoints = list(session.waypoints)
        self.canvas.highlight_time_range = None

    def apply_frame_index(self, idx: int) -> Optional[ReplayFrame]:
        if self._session is None or not self._session.frames:
            return None
        idx = max(0, min(idx, len(self._session.frames) - 1))
        fr = self._session.frames[idx]
        res_m = {
            aid: (sn.res_mm / 1000.0)
            for aid, sn in fr.anchors.items()
            if sn.res_mm is not None
        }
        self.engine.latest_solution = PositionResult(
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
        self.canvas.cursor_time_s = fr.rel_time_s
        self.canvas.is_dark = self.is_dark
        return fr

    def render_qimage(self, frame_idx: int, width: int, height: int) -> QImage:
        width = int(width) - (int(width) % 2)
        height = int(height) - (int(height) % 2)
        qimg = QImage(width, height, QImage.Format.Format_RGB888)
        painter = QPainter(qimg)
        self.paint_dashboard(painter, QRectF(0, 0, width, height), frame_idx)
        painter.end()
        return qimg

    def paint_dashboard(self, painter: QPainter, rect: QRectF, frame_idx: int) -> None:
        painter.save()
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing, True)

        dark = self.is_dark
        bg_col = QColor("#0B1120" if dark else "#F1F5F9")
        card_bg = QColor("#111827" if dark else "#FFFFFF")
        sub_bg = QColor("#1E293B" if dark else "#F8FAFC")
        border_col = QColor("#1E293B" if dark else "#CBD5E1")
        fg_col = QColor("#F8FAFC" if dark else "#0F172A")
        muted_col = QColor("#94A3B8" if dark else "#475569")
        primary_col = QColor("#0EA5E9" if dark else "#0284C7")

        painter.fillRect(rect, bg_col)

        session = self._session
        fr = self.apply_frame_index(frame_idx)
        pad = 12.0
        x0 = rect.left() + pad
        y0 = rect.top() + pad
        avail_w = rect.width() - 2 * pad
        avail_h = rect.height() - 2 * pad

        # 1. Top Header Bar (32px)
        hdr_rect = QRectF(x0, y0, avail_w, 32.0)
        painter.setPen(QPen(border_col, 1))
        painter.setBrush(card_bg)
        painter.drawRoundedRect(hdr_rect, 6, 6)

        painter.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        painter.setPen(primary_col)
        painter.drawText(
            QRectF(hdr_rect.left() + 12, hdr_rect.top(), 420, hdr_rect.height()),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            "UWB FAST STUDIO — LOG REPLAY & 3D TRAJECTORY ANALYSIS",
        )

        if session is not None and fr is not None:
            meta_txt = (
                f"{session.csv_path.parent.name}/{session.csv_path.name}  ·  "
                f"Playback {self.playback_speed:g}x  ·  "
                f"Frame {frame_idx + 1:,}/{len(session.frames):,}"
            )
        else:
            meta_txt = "No log session loaded"
        painter.setFont(QFont("Consolas", 9))
        painter.setPen(muted_col)
        painter.drawText(
            QRectF(hdr_rect.left() + 430, hdr_rect.top(), hdr_rect.width() - 442, hdr_rect.height()),
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            meta_txt,
        )

        # 2. Four KPI Cards Row (68px)
        kpi_y = hdr_rect.bottom() + 8.0
        kpi_h = 68.0
        kpi_gap = 8.0
        kpi_w = (avail_w - 3 * kpi_gap) / 4.0

        if session is not None and fr is not None:
            stab_color = (
                QColor("#EF4444")
                if fr.stability_state == "UNSTABLE"
                else (QColor("#F59E0B") if fr.stability_state == "WARN" else QColor("#10B981"))
            )
            reason_suffix = f" | {fr.instability_reason}" if fr.instability_reason else ""
            kpi_data = [
                (
                    "SESSION OVERVIEW",
                    f"{session.duration_s:.1f} s ({len(session.frames):,} frames)",
                    f"Folder: {session.csv_path.parent.name} | Raw CSV: {session.total_csv_rows:,} rows",
                    fg_col,
                ),
                (
                    "3D LOOP & DISTANCE",
                    f"3D Path: {session.path_length_3d_m:.2f} m",
                    f"2D Loop: {session.path_length_2d_m:.2f} m | Waypoints: {len(session.waypoints)}",
                    fg_col,
                ),
                (
                    "STABILITY BREAKDOWN",
                    f"{session.stable_pct:.1f}% Stable",
                    f"Warn: {session.warn_pct:.1f}% | Unstable: {session.unstable_pct:.1f}% | Segments: {len(session.unstable_segments)}",
                    fg_col,
                ),
                (
                    "REPLAY CURSOR (X, Y, Z)",
                    f"{fr.x:+.3f}, {fr.y:+.3f}, {fr.z:+.3f} m [{fr.stability_state}]",
                    f"t={fr.rel_time_s:.2f}s | RMS:{fr.rms_mm:.1f}mm | Jump:{fr.step_jump_mm:.0f}mm | {fr.speed_mps:.2f}m/s{reason_suffix}",
                    stab_color,
                ),
            ]
        else:
            kpi_data = [
                ("SESSION OVERVIEW", "--", "Duration: -- | Frames: 0", fg_col),
                ("3D LOOP & DISTANCE", "0.00 m", "2D Path: 0.00 m | Waypoints: 0", fg_col),
                ("STABILITY BREAKDOWN", "-- % Stable", "Warn: --% | Unstable: --%", fg_col),
                ("REPLAY CURSOR (X, Y, Z)", "-- , -- , -- m", "t = 0.00s | RMS: -- mm", fg_col),
            ]

        for idx, (k_title, k_val, k_sub, k_col) in enumerate(kpi_data):
            kx = x0 + idx * (kpi_w + kpi_gap)
            k_rect = QRectF(kx, kpi_y, kpi_w, kpi_h)
            painter.setPen(QPen(border_col, 1))
            painter.setBrush(card_bg)
            painter.drawRoundedRect(k_rect, 8, 8)

            painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
            painter.setPen(muted_col)
            painter.drawText(
                QRectF(kx + 12, kpi_y + 6, kpi_w - 24, 14),
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                k_title,
            )

            painter.setFont(QFont("Consolas", 12, QFont.Weight.Bold))
            painter.setPen(k_col)
            painter.drawText(
                QRectF(kx + 12, kpi_y + 22, kpi_w - 24, 22),
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                k_val,
            )

            painter.setFont(QFont("Segoe UI", 8))
            painter.setPen(muted_col)
            elided_sub = painter.fontMetrics().elidedText(k_sub, Qt.TextElideMode.ElideRight, int(kpi_w - 24))
            painter.drawText(
                QRectF(kx + 12, kpi_y + 45, kpi_w - 24, 16),
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                elided_sub,
            )

        # 3. Main Body Area
        body_y = kpi_y + kpi_h + 8.0
        body_h = max(200.0, (y0 + avail_h) - body_y)

        if self.layout_mode == "MAP_ONLY":
            left_w = avail_w
            right_w = 0.0
        else:
            left_w = round(avail_w * 0.60)
            right_w = avail_w - left_w - 8.0

        # Left Card: 2D + 3D Trajectory + Timeline Scrubber
        left_card = QRectF(x0, body_y, left_w, body_h)
        painter.setPen(QPen(border_col, 1))
        painter.setBrush(card_bg)
        painter.drawRoundedRect(left_card, 8, 8)

        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.setPen(muted_col)
        painter.drawText(
            QRectF(left_card.left() + 12, left_card.top() + 8, left_card.width() - 24, 18),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            "REPLAY 2D + 3D TRAJECTORY (Green=Stable · Amber=Warn · Red=Unstable)",
        )

        scrub_h = 34.0
        canvas_rect = QRectF(
            left_card.left() + 10,
            left_card.top() + 30,
            left_card.width() - 20,
            max(120.0, left_card.height() - 42 - scrub_h - 6),
        )
        self.canvas.paint_onto(painter, canvas_rect)

        # Scrubber / Progress Bar inside Left Card
        scrub_rect = QRectF(
            left_card.left() + 10,
            canvas_rect.bottom() + 6,
            left_card.width() - 20,
            scrub_h,
        )
        self._paint_timeline_bar(painter, scrub_rect, session, fr, dark)

        # Right Column: Unstable Segments Table + 8-Anchor Telemetry Table
        if right_w > 120.0:
            rx0 = left_card.right() + 8.0
            top_rh = round((body_h - 8.0) * 0.45)
            bot_rh = body_h - 8.0 - top_rh

            seg_card = QRectF(rx0, body_y, right_w, top_rh)
            anc_card = QRectF(rx0, body_y + top_rh + 8.0, right_w, bot_rh)
            self._paint_unstable_segments_card(painter, seg_card, session, fr, dark)
            self._paint_anchor_telemetry_card(painter, anc_card, fr, dark)

        painter.restore()

    def _paint_timeline_bar(
        self,
        painter: QPainter,
        rect: QRectF,
        session: Optional[LoadedLogSession],
        fr: Optional[ReplayFrame],
        dark: bool,
    ) -> None:
        sub_bg = QColor("#1E293B" if dark else "#F8FAFC")
        border_col = QColor("#334155" if dark else "#CBD5E1")
        fg_col = QColor("#F8FAFC" if dark else "#0F172A")
        primary_col = QColor("#0EA5E9" if dark else "#0284C7")

        painter.setPen(QPen(border_col, 1))
        painter.setBrush(sub_bg)
        painter.drawRoundedRect(rect, 6, 6)

        # Speed badge
        badge_rect = QRectF(rect.left() + 8, rect.top() + 5, 58, rect.height() - 10)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(primary_col)
        painter.drawRoundedRect(badge_rect, 4, 4)
        painter.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        painter.setPen(QColor("#FFFFFF"))
        painter.drawText(badge_rect, Qt.AlignmentFlag.AlignCenter, f"{self.playback_speed:g}x")

        dur = max(session.duration_s if session else 1.0, 0.01)
        cur_t = fr.rel_time_s if fr else 0.0
        frac = min(1.0, max(0.0, cur_t / dur))

        time_lbl_w = 135.0
        bar_x = badge_rect.right() + 12.0
        bar_w = max(60.0, rect.right() - time_lbl_w - 12.0 - bar_x)
        bar_cy = rect.center().y()
        track_rect = QRectF(bar_x, bar_cy - 4.0, bar_w, 8.0)

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor("#334155" if dark else "#E2E8F0"))
        painter.drawRoundedRect(track_rect, 4, 4)

        # Draw unstable segment markers on timeline track
        if session and session.unstable_segments:
            for seg in session.unstable_segments:
                sx0 = bar_x + min(1.0, max(0.0, seg.start_rel_s / dur)) * bar_w
                sx1 = bar_x + min(1.0, max(0.0, seg.end_rel_s / dur)) * bar_w
                s_col = QColor("#EF4444" if seg.severity == "UNSTABLE" else "#F59E0B")
                s_col.setAlpha(140)
                painter.setBrush(s_col)
                painter.drawRect(QRectF(sx0, bar_cy - 4.0, max(2.0, sx1 - sx0), 8.0))

        fill_rect = QRectF(bar_x, bar_cy - 4.0, bar_w * frac, 8.0)
        painter.setBrush(primary_col)
        painter.drawRoundedRect(fill_rect, 4, 4)

        knob_x = bar_x + bar_w * frac
        painter.setPen(QPen(QColor("#FFFFFF"), 1.5))
        painter.setBrush(QColor("#DC2626"))
        painter.drawEllipse(QPointF(knob_x, bar_cy), 6.5, 6.5)

        painter.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        painter.setPen(fg_col)
        painter.drawText(
            QRectF(rect.right() - time_lbl_w - 8, rect.top(), time_lbl_w, rect.height()),
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            f"{cur_t:6.2f}s / {dur:.2f}s",
        )

    def _paint_unstable_segments_card(
        self,
        painter: QPainter,
        rect: QRectF,
        session: Optional[LoadedLogSession],
        fr: Optional[ReplayFrame],
        dark: bool,
    ) -> None:
        card_bg = QColor("#111827" if dark else "#FFFFFF")
        hdr_bg = QColor("#1E293B" if dark else "#F8FAFC")
        alt_bg = QColor("#172033" if dark else "#F8FAFC")
        border_col = QColor("#1E293B" if dark else "#CBD5E1")
        fg_col = QColor("#F8FAFC" if dark else "#0F172A")
        muted_col = QColor("#94A3B8" if dark else "#475569")

        painter.setPen(QPen(border_col, 1))
        painter.setBrush(card_bg)
        painter.drawRoundedRect(rect, 8, 8)

        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.setPen(muted_col)
        painter.drawText(
            QRectF(rect.left() + 10, rect.top() + 6, rect.width() - 20, 18),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            "UNSTABLE SEGMENTS (HIGHLIGHTED AT REPLAY CURSOR)",
        )

        tbl_rect = QRectF(rect.left() + 8, rect.top() + 28, rect.width() - 16, rect.height() - 36)
        painter.save()
        painter.setClipRect(tbl_rect)

        painter.setPen(QPen(border_col, 1))
        painter.setBrush(card_bg)
        painter.drawRoundedRect(tbl_rect, 4, 4)

        cols = [
            ("#", 0.09),
            ("Severity", 0.17),
            ("Time (s)", 0.27),
            ("Pos (X,Y,Z)", 0.25),
            ("Max RMS", 0.12),
            ("Jump", 0.10),
        ]
        row_h = 22.0
        hdr_r = QRectF(tbl_rect.left(), tbl_rect.top(), tbl_rect.width(), row_h)
        painter.fillRect(hdr_r, hdr_bg)
        painter.setPen(QPen(border_col, 1))
        painter.drawLine(hdr_r.bottomLeft(), hdr_r.bottomRight())

        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.setPen(muted_col)
        cx = tbl_rect.left()
        for c_title, c_frac in cols:
            cw = tbl_rect.width() * c_frac
            painter.drawText(QRectF(cx + 4, hdr_r.top(), cw - 8, row_h), Qt.AlignmentFlag.AlignCenter, c_title)
            cx += cw

        segs = session.unstable_segments if session else []
        max_rows = max(1, int((tbl_rect.height() - row_h) // row_h))
        cur_t = fr.rel_time_s if fr else 0.0

        # Auto-scroll segment list so active or upcoming segment is visible
        active_idx = 0
        for i, s in enumerate(segs):
            if s.start_rel_s <= cur_t + 0.2:
                active_idx = i
        start_idx = max(0, min(active_idx - max_rows // 2, max(0, len(segs) - max_rows)))

        painter.setFont(QFont("Consolas", 8))
        for r_i in range(max_rows):
            s_idx = start_idx + r_i
            if s_idx >= len(segs):
                break
            seg = segs[s_idx]
            ry = tbl_rect.top() + (r_i + 1) * row_h
            r_rect = QRectF(tbl_rect.left(), ry, tbl_rect.width(), row_h)
            is_current = (seg.start_rel_s - 0.15) <= cur_t <= (seg.end_rel_s + 0.15)
            if is_current:
                painter.fillRect(r_rect, QColor("#7F1D1D" if dark else "#FEE2E2"))
            elif r_i % 2 == 1:
                painter.fillRect(r_rect, alt_bg)

            painter.setPen(QPen(border_col, 1))
            painter.drawLine(r_rect.bottomLeft(), r_rect.bottomRight())

            vals = [
                f"#{seg.segment_id}",
                seg.severity,
                f"{seg.start_rel_s:.1f}-{seg.end_rel_s:.1f}s ({seg.duration_s:.1f}s)",
                f"({seg.xyz_m[0]:.2f}, {seg.xyz_m[1]:.2f}, {seg.xyz_m[2]:.2f})",
                f"{seg.max_rms_mm:.0f} mm",
                f"{seg.max_jump_mm:.0f}",
            ]
            cx = tbl_rect.left()
            for c_i, ((_, c_frac), txt) in enumerate(zip(cols, vals)):
                cw = tbl_rect.width() * c_frac
                if c_i == 1:
                    painter.setPen(QColor("#EF4444" if seg.severity == "UNSTABLE" else "#F59E0B"))
                else:
                    painter.setPen(fg_col)
                painter.drawText(
                    QRectF(cx + 3, ry, cw - 6, row_h),
                    Qt.AlignmentFlag.AlignCenter,
                    txt,
                )
                cx += cw

        painter.restore()

    def _paint_anchor_telemetry_card(
        self,
        painter: QPainter,
        rect: QRectF,
        fr: Optional[ReplayFrame],
        dark: bool,
    ) -> None:
        card_bg = QColor("#111827" if dark else "#FFFFFF")
        hdr_bg = QColor("#1E293B" if dark else "#F8FAFC")
        alt_bg = QColor("#172033" if dark else "#F8FAFC")
        border_col = QColor("#1E293B" if dark else "#CBD5E1")
        fg_col = QColor("#F8FAFC" if dark else "#0F172A")
        muted_col = QColor("#94A3B8" if dark else "#475569")

        painter.setPen(QPen(border_col, 1))
        painter.setBrush(card_bg)
        painter.drawRoundedRect(rect, 8, 8)

        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.setPen(muted_col)
        painter.drawText(
            QRectF(rect.left() + 10, rect.top() + 6, rect.width() - 20, 18),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            "8-ANCHOR TELEMETRY AT REPLAY CURSOR",
        )

        tbl_rect = QRectF(rect.left() + 8, rect.top() + 28, rect.width() - 16, rect.height() - 36)
        painter.save()
        painter.setClipRect(tbl_rect)

        painter.setPen(QPen(border_col, 1))
        painter.setBrush(card_bg)
        painter.drawRoundedRect(tbl_rect, 4, 4)

        cols = [
            ("ID", 0.09),
            ("State", 0.11),
            ("Hz", 0.11),
            ("Raw (m)", 0.15),
            ("Filt (m)", 0.15),
            ("Res (mm)", 0.15),
            ("NLOS (dB)", 0.12),
            ("σ (mm)", 0.12),
        ]
        row_h = min(26.0, max(18.0, tbl_rect.height() / 9.0))
        hdr_r = QRectF(tbl_rect.left(), tbl_rect.top(), tbl_rect.width(), row_h)
        painter.fillRect(hdr_r, hdr_bg)
        painter.setPen(QPen(border_col, 1))
        painter.drawLine(hdr_r.bottomLeft(), hdr_r.bottomRight())

        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.setPen(muted_col)
        cx = tbl_rect.left()
        for c_title, c_frac in cols:
            cw = tbl_rect.width() * c_frac
            painter.drawText(QRectF(cx + 2, hdr_r.top(), cw - 4, row_h), Qt.AlignmentFlag.AlignCenter, c_title)
            cx += cw

        for row in range(8):
            aid = row + 1
            ry = tbl_rect.top() + (row + 1) * row_h
            r_rect = QRectF(tbl_rect.left(), ry, tbl_rect.width(), row_h)
            if row % 2 == 1:
                painter.fillRect(r_rect, alt_bg)
            painter.setPen(QPen(border_col, 1))
            painter.drawLine(r_rect.bottomLeft(), r_rect.bottomRight())

            sn = fr.anchors.get(aid) if fr else None
            if sn is not None:
                vals = [
                    f"A{aid}",
                    sn.status_str,
                    f"{sn.rate_hz:.0f}",
                    f"{sn.raw_mm / 1000.0:.3f}" if sn.raw_mm else "--",
                    f"{sn.filt_mm / 1000.0:.3f}" if sn.filt_mm else "--",
                    f"{sn.res_mm:+.1f}" if sn.res_mm is not None else "--",
                    f"{sn.nlos_db:.1f}" if sn.nlos_db is not None else "--",
                    f"{sn.sigma_mm:.1f}" if sn.sigma_mm is not None else "--",
                ]
            else:
                vals = [f"A{aid}", "--", "--", "--", "--", "--", "--", "--"]

            cx = tbl_rect.left()
            for c_i, ((_, c_frac), txt) in enumerate(zip(cols, vals)):
                cw = tbl_rect.width() * c_frac
                if c_i == 0:
                    painter.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
                    painter.setPen(QColor(ANCHOR_PALETTE[row]))
                else:
                    painter.setFont(QFont("Consolas", 8))
                    painter.setPen(fg_col)
                painter.drawText(
                    QRectF(cx + 3, ry, cw - 6, row_h),
                    Qt.AlignmentFlag.AlignCenter,
                    txt,
                )
                cx += cw

        painter.restore()


def select_video_frame_indices(
    frames: Sequence[ReplayFrame],
    start_time_s: float,
    end_time_s: float,
    playback_speed: float,
    fps: float,
) -> list[int]:
    """Compute the exact frame indices for each video frame at target FPS and playback speed."""
    if not frames:
        return []
    t0 = max(frames[0].rel_time_s, float(start_time_s))
    t1 = min(frames[-1].rel_time_s, float(end_time_s))
    if t1 <= t0:
        t0 = frames[0].rel_time_s
        t1 = frames[-1].rel_time_s

    dt_log = max(0.001, (float(playback_speed) / max(1.0, float(fps))))
    times = [f.rel_time_s for f in frames]
    out_indices: list[int] = []
    cur_t = t0
    idx = 0
    n = len(frames)
    while cur_t <= t1 + 1e-6:
        while idx + 1 < n and times[idx + 1] <= cur_t:
            idx += 1
        out_indices.append(idx)
        cur_t += dt_log
    if out_indices and out_indices[-1] != n - 1 and abs(t1 - frames[-1].rel_time_s) < 0.05:
        out_indices.append(n - 1)
    return out_indices


def export_session_to_mp4(
    session: LoadedLogSession,
    output_path: Path,
    *,
    width: int = 1920,
    height: int = 1080,
    fps: float = 30.0,
    playback_speed: float = 10.0,
    start_time_s: Optional[float] = None,
    end_time_s: Optional[float] = None,
    layout_mode: str = "DASHBOARD",
    trail_mode: str = "PROGRESSIVE_GHOST",
    is_dark: bool = False,
    azimuth_deg: float = -42.0,
    elevation_deg: float = 26.0,
    orbit_deg_per_s: float = 0.0,
    progress_cb: Optional[Callable[[int, int, float], bool]] = None,
) -> dict:
    """Render a `LoadedLogSession` to an `.mp4` video file and return summary stats."""
    if not session.frames:
        raise ValueError("LoadedLogSession has no frames to export.")

    t0 = session.frames[0].rel_time_s if start_time_s is None else float(start_time_s)
    t1 = session.frames[-1].rel_time_s if end_time_s is None else float(end_time_s)
    frame_indices = select_video_frame_indices(session.frames, t0, t1, playback_speed, fps)
    if not frame_indices:
        frame_indices = [0]

    renderer = ReplayVideoFrameRenderer()
    renderer.layout_mode = layout_mode
    renderer.is_dark = is_dark
    renderer.playback_speed = playback_speed
    renderer.canvas.trail_mode = trail_mode
    renderer.canvas.azimuth_deg = azimuth_deg
    renderer.canvas.elevation_deg = elevation_deg
    renderer.set_session(session)

    writer = Mp4VideoWriter(output_path, width=width, height=height, fps=fps)
    t_start = time.perf_counter()
    rendered = 0
    total = len(frame_indices)
    cancelled = False
    try:
        for step_i, f_idx in enumerate(frame_indices):
            if orbit_deg_per_s != 0.0:
                renderer.canvas.azimuth_deg = (azimuth_deg + (step_i / max(1.0, fps)) * orbit_deg_per_s) % 360.0
            qimg = renderer.render_qimage(f_idx, width, height)
            writer.write_qimage(qimg)
            rendered += 1
            if progress_cb is not None:
                elapsed = max(1e-6, time.perf_counter() - t_start)
                if not progress_cb(rendered, total, rendered / elapsed):
                    cancelled = True
                    break
    finally:
        writer.close()

    elapsed_s = max(1e-6, time.perf_counter() - t_start)
    return {
        "output_path": str(output_path),
        "frames_rendered": rendered,
        "total_frames": total,
        "video_duration_s": rendered / max(1.0, fps),
        "elapsed_s": elapsed_s,
        "render_fps": rendered / elapsed_s,
        "backend": writer.backend,
        "cancelled": cancelled,
    }


class VideoRenderWorker(QThread):
    """Background thread that renders and encodes the MP4 video without blocking the GUI."""

    progress = pyqtSignal(int, int, float)
    finished_ok = pyqtSignal(dict)
    failed = pyqtSignal(str)

    def __init__(
        self,
        session: LoadedLogSession,
        output_path: Path,
        *,
        width: int,
        height: int,
        fps: float,
        playback_speed: float,
        start_time_s: float,
        end_time_s: float,
        layout_mode: str,
        trail_mode: str,
        is_dark: bool,
        azimuth_deg: float,
        elevation_deg: float,
        orbit_deg_per_s: float,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.session = session
        self.output_path = output_path
        self.width = width
        self.height = height
        self.fps = fps
        self.playback_speed = playback_speed
        self.start_time_s = start_time_s
        self.end_time_s = end_time_s
        self.layout_mode = layout_mode
        self.trail_mode = trail_mode
        self.is_dark = is_dark
        self.azimuth_deg = azimuth_deg
        self.elevation_deg = elevation_deg
        self.orbit_deg_per_s = orbit_deg_per_s

    def run(self) -> None:
        try:
            def _on_prog(done: int, total: int, r_fps: float) -> bool:
                self.progress.emit(done, total, r_fps)
                return not self.isInterruptionRequested()

            res = export_session_to_mp4(
                self.session,
                self.output_path,
                width=self.width,
                height=self.height,
                fps=self.fps,
                playback_speed=self.playback_speed,
                start_time_s=self.start_time_s,
                end_time_s=self.end_time_s,
                layout_mode=self.layout_mode,
                trail_mode=self.trail_mode,
                is_dark=self.is_dark,
                azimuth_deg=self.azimuth_deg,
                elevation_deg=self.elevation_deg,
                orbit_deg_per_s=self.orbit_deg_per_s,
                progress_cb=_on_prog,
            )
            self.finished_ok.emit(res)
        except Exception as exc:
            self.failed.emit(str(exc))


class VideoPreviewWidget(QWidget):
    """Interactive WYSIWYG preview of the video frame with mouse drag for 3D camera orbit."""

    camera_changed = pyqtSignal(float, float)

    def __init__(self, renderer: ReplayVideoFrameRenderer, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.renderer = renderer
        self.frame_idx: int = 0
        self.setMinimumSize(720, 420)
        self._last_pos: Optional[QPointF] = None

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        self.renderer.paint_dashboard(painter, QRectF(0, 0, self.width(), self.height()), self.frame_idx)
        painter.end()

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() in (Qt.MouseButton.LeftButton, Qt.MouseButton.RightButton):
            self._last_pos = event.position()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self._last_pos is None:
            return
        delta = event.position() - self._last_pos
        self._last_pos = event.position()
        c = self.renderer.canvas
        c.azimuth_deg = (c.azimuth_deg + delta.x() * 0.45) % 360.0
        c.elevation_deg = max(6.0, min(84.0, c.elevation_deg + delta.y() * 0.35))
        self.camera_changed.emit(c.azimuth_deg, c.elevation_deg)
        self.update()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        self._last_pos = None

    def wheelEvent(self, event: QWheelEvent) -> None:
        steps = event.angleDelta().y() / 120.0
        if steps != 0:
            c = self.renderer.canvas
            c.zoom_3d = max(0.5, min(3.0, c.zoom_3d * (1.12 ** steps)))
            c.zoom_2d = max(0.5, min(4.0, c.zoom_2d * (1.12 ** steps)))
            self.update()


class VideoExportStudioWindow(QMainWindow):
    """Dedicated GUI for exporting UWB Replay Logs to MP4 Video."""

    def __init__(
        self,
        initial_csv: Optional[Path] = None,
        initial_speed: float = 10.0,
        initial_az: float = -42.0,
        initial_el: float = 26.0,
        initial_dark: bool = False,
        initial_resolve: bool = False,
        auto_load: bool = True,
    ) -> None:
        super().__init__()
        self.setWindowTitle("UWB Fast Studio — MP4 Video Export Studio")
        self.resize(1380, 860)
        self.setMinimumSize(1080, 680)

        app = QApplication.instance()
        self.theme_manager = ThemeManager(app) if app is not None else None
        if self.theme_manager is not None:
            if initial_dark:
                self.theme_manager.apply_dark()
            else:
                self.theme_manager.apply_light()

        self.renderer = ReplayVideoFrameRenderer()
        self.renderer.is_dark = initial_dark
        self.renderer.playback_speed = initial_speed
        self.renderer.canvas.azimuth_deg = initial_az
        self.renderer.canvas.elevation_deg = initial_el

        self.loaded_session: Optional[LoadedLogSession] = None
        self._loader_worker: Optional[LogReplayWorker] = None
        self._render_worker: Optional[VideoRenderWorker] = None
        self._initial_resolve = initial_resolve

        self._build_ui(initial_speed=initial_speed, initial_dark=initial_dark)

        if initial_csv is not None and initial_csv.is_file():
            self.load_csv_file(initial_csv)
        elif auto_load:
            self.load_latest_log()

    def _build_ui(self, initial_speed: float, initial_dark: bool) -> None:
        central = QWidget(self)
        central.setObjectName("central_root")
        central.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setCentralWidget(central)
        vbox = QVBoxLayout(central)
        vbox.setContentsMargins(SPACING_MD, SPACING_MD, SPACING_MD, SPACING_MD)
        vbox.setSpacing(SPACING_SM)

        # 1. Top Control Bar: Log Selection & Output Path
        top_card = QFrame(central)
        top_card.setObjectName("panel_card")
        top_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        top_lay = QVBoxLayout(top_card)
        top_lay.setContentsMargins(12, 10, 12, 10)
        top_lay.setSpacing(8)

        row1 = QHBoxLayout()
        row1.setSpacing(8)
        lbl_brand = QLabel("MP4 VIDEO EXPORT STUDIO")
        lbl_brand.setObjectName("brand_title")
        row1.addWidget(lbl_brand)

        btn_latest = QPushButton("Load Latest Log")
        btn_latest.setObjectName("primary_btn")
        btn_latest.clicked.connect(self.load_latest_log)
        row1.addWidget(btn_latest)

        btn_browse_csv = QPushButton("Open Log CSV...")
        btn_browse_csv.clicked.connect(self._browse_csv_dialog)
        row1.addWidget(btn_browse_csv)

        self.chk_resolve = QCheckBox("Re-Solve 3D with Rig Layout")
        self.chk_resolve.setChecked(self._initial_resolve)
        self.chk_resolve.toggled.connect(self._reload_current_csv)
        row1.addWidget(self.chk_resolve)

        row1.addStretch(1)

        self.lbl_source_status = QLabel("Waiting for log session...")
        self.lbl_source_status.setObjectName("metric_sub")
        row1.addWidget(self.lbl_source_status)

        self.btn_theme = QPushButton("Theme: Dark" if initial_dark else "Theme: Light")
        self.btn_theme.clicked.connect(self._toggle_theme)
        row1.addWidget(self.btn_theme)
        top_lay.addLayout(row1)

        # Row 2: Video Settings (Speed, Resolution, FPS, Layout, Trail Mode, Time Range)
        row2 = QHBoxLayout()
        row2.setSpacing(8)

        row2.addWidget(QLabel("Layout:"))
        self.combo_layout = QComboBox()
        self.combo_layout.addItem("Full Dashboard (2D/3D + Tables)", "DASHBOARD")
        self.combo_layout.addItem("Map Only (Fullscreen 2D + 3D)", "MAP_ONLY")
        self.combo_layout.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.combo_layout.currentIndexChanged.connect(self._on_settings_changed)
        row2.addWidget(self.combo_layout)

        row2.addWidget(QLabel("Trail:"))
        self.combo_trail = QComboBox()
        self.combo_trail.addItem("Progressive + Ghost Loop", "PROGRESSIVE_GHOST")
        self.combo_trail.addItem("Progressive Trail Only", "PROGRESSIVE_ONLY")
        self.combo_trail.addItem("Full Static Loop", "FULL")
        self.combo_trail.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.combo_trail.currentIndexChanged.connect(self._on_settings_changed)
        row2.addWidget(self.combo_trail)

        row2.addWidget(QLabel("Speed:"))
        self.combo_speed = QComboBox()
        speeds = [("1x (Real-Time)", 1.0), ("2x", 2.0), ("4x", 4.0), ("5x", 5.0), ("10x", 10.0), ("15x", 15.0), ("20x", 20.0)]
        sel_idx = 4
        for i, (lbl, val) in enumerate(speeds):
            self.combo_speed.addItem(lbl, val)
            if abs(val - initial_speed) < 0.1:
                sel_idx = i
        self.combo_speed.setCurrentIndex(sel_idx)
        self.combo_speed.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.combo_speed.currentIndexChanged.connect(self._on_settings_changed)
        row2.addWidget(self.combo_speed)

        row2.addWidget(QLabel("Resolution:"))
        self.combo_res = QComboBox()
        self.combo_res.addItem("1920×1080 (1080p)", (1920, 1080))
        self.combo_res.addItem("1280×720 (720p Fast)", (1280, 720))
        self.combo_res.addItem("1600×900 (HD+)", (1600, 900))
        self.combo_res.setCurrentIndex(1)
        self.combo_res.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        row2.addWidget(self.combo_res)

        row2.addWidget(QLabel("FPS:"))
        self.combo_fps = QComboBox()
        for f_val in (24, 30, 60):
            self.combo_fps.addItem(f"{f_val} FPS", float(f_val))
        self.combo_fps.setCurrentIndex(1)
        self.combo_fps.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.combo_fps.currentIndexChanged.connect(self._on_settings_changed)
        row2.addWidget(self.combo_fps)

        row2.addWidget(QLabel("Start (s):"))
        self.edit_t0 = QLineEdit("0.0")
        self.edit_t0.setValidator(QDoubleValidator(0.0, 99999.0, 2, self))
        self.edit_t0.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.edit_t0.setFont(QFont(FONT_MONO, 9))
        self.edit_t0.setFixedWidth(64)
        self.edit_t0.editingFinished.connect(self._on_settings_changed)
        row2.addWidget(self.edit_t0)

        row2.addWidget(QLabel("End (s):"))
        self.edit_t1 = QLineEdit("0.0")
        self.edit_t1.setValidator(QDoubleValidator(0.0, 99999.0, 2, self))
        self.edit_t1.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.edit_t1.setFont(QFont(FONT_MONO, 9))
        self.edit_t1.setFixedWidth(64)
        self.edit_t1.editingFinished.connect(self._on_settings_changed)
        row2.addWidget(self.edit_t1)

        self.chk_orbit = QCheckBox("Auto-Orbit 3D")
        self.chk_orbit.setToolTip("Slowly rotate the 3D Orbit camera (8°/s) during the exported video")
        row2.addWidget(self.chk_orbit)

        row2.addStretch(1)
        top_lay.addLayout(row2)

        # Row 3: Output File & Export Action
        row3 = QHBoxLayout()
        row3.setSpacing(8)
        row3.addWidget(QLabel("Output MP4:"))
        self.edit_out_path = QLineEdit()
        self.edit_out_path.setPlaceholderText("Select output .mp4 path...")
        row3.addWidget(self.edit_out_path, 1)

        btn_browse_out = QPushButton("Browse Save Path...")
        btn_browse_out.clicked.connect(self._browse_output_dialog)
        row3.addWidget(btn_browse_out)

        self.lbl_est_info = QLabel("Est. Video: -- s (-- frames)")
        self.lbl_est_info.setObjectName("badge_info")
        row3.addWidget(self.lbl_est_info)

        self.btn_export = QPushButton("Render && Export .MP4")
        self.btn_export.setObjectName("success_btn")
        self.btn_export.setMinimumWidth(165)
        self.btn_export.clicked.connect(self._toggle_export)
        row3.addWidget(self.btn_export)

        self.btn_open_video = QPushButton("Play Video")
        self.btn_open_video.setEnabled(False)
        self.btn_open_video.clicked.connect(self._open_exported_video)
        row3.addWidget(self.btn_open_video)

        self.btn_open_folder = QPushButton("Open Folder")
        self.btn_open_folder.clicked.connect(self._open_output_folder)
        row3.addWidget(self.btn_open_folder)

        top_lay.addLayout(row3)

        # Progress Bar Row
        prog_row = QHBoxLayout()
        prog_row.setSpacing(8)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        prog_row.addWidget(self.progress_bar, 1)

        self.lbl_render_status = QLabel(
            f"Encoder ready ({'FFmpeg H.264' if find_ffmpeg_binary() else 'OpenCV MP4V'}) — Drag 3D preview to adjust camera angle before exporting"
        )
        self.lbl_render_status.setObjectName("metric_sub")
        prog_row.addWidget(self.lbl_render_status)
        top_lay.addLayout(prog_row)

        vbox.addWidget(top_card)

        # 2. Interactive WYSIWYG Frame Preview Card
        prev_card = QFrame(central)
        prev_card.setObjectName("panel_card")
        prev_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        p_lay = QVBoxLayout(prev_card)
        p_lay.setContentsMargins(10, 10, 10, 10)
        p_lay.setSpacing(6)

        p_hdr = QHBoxLayout()
        lbl_p = QLabel("LIVE WYSIWYG VIDEO FRAME PREVIEW (DRAG MOUSE TO ROTATE 3D CAMERA · SCROLL TO ZOOM)")
        lbl_p.setObjectName("section_header")
        p_hdr.addWidget(lbl_p)
        p_hdr.addStretch(1)
        btn_reset_cam = QPushButton("Reset 3D Camera")
        btn_reset_cam.clicked.connect(self._reset_preview_camera)
        p_hdr.addWidget(btn_reset_cam)
        p_lay.addLayout(p_hdr)

        self.preview_widget = VideoPreviewWidget(self.renderer, prev_card)
        p_lay.addWidget(self.preview_widget, 1)

        # Preview Scrubber Slider
        s_row = QHBoxLayout()
        s_row.setSpacing(8)
        s_row.addWidget(QLabel("Preview Scrubber:"))
        self.preview_slider = QSlider(Qt.Orientation.Horizontal)
        self.preview_slider.setRange(0, 0)
        self.preview_slider.valueChanged.connect(self._on_preview_slider_changed)
        s_row.addWidget(self.preview_slider, 1)
        self.lbl_preview_frame = QLabel("0.00s / 0.00s")
        self.lbl_preview_frame.setFont(QFont(FONT_MONO, 9, QFont.Weight.Bold))
        s_row.addWidget(self.lbl_preview_frame)
        p_lay.addLayout(s_row)

        vbox.addWidget(prev_card, 1)

    def _fallback_anchors(self) -> list[AnchorConfig]:
        if LAST_RIG_PATH.is_file():
            try:
                loaded = load_anchor_layout_json(LAST_RIG_PATH)
                if any(a.enabled for a in loaded):
                    return loaded
            except Exception:
                pass
        return list(DEFAULT_PRESET_ROOM_66_36)

    def load_latest_log(self) -> None:
        log_root = NEW_GUI_DIR / "logs"
        if not log_root.is_dir():
            self.lbl_source_status.setText(f"No logs folder at {log_root}")
            return
        uwb_candidates = sorted(
            log_root.glob("uwb_fast_*/uwb_fast_log.csv"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        if not uwb_candidates:
            uwb_candidates = sorted(
                log_root.glob("**/uwb_fast_log.csv"),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
        if not uwb_candidates:
            self.lbl_source_status.setText("No uwb_fast_log.csv found in logs/")
            return
        self.load_csv_file(uwb_candidates[0])

    def _browse_csv_dialog(self) -> None:
        log_root = NEW_GUI_DIR / "logs"
        start_dir = str(log_root if log_root.is_dir() else NEW_GUI_DIR)
        path_str, _ = QFileDialog.getOpenFileName(
            self,
            "Select UWB Session CSV Log",
            start_dir,
            "CSV Log Files (*.csv);;All Files (*)",
        )
        if path_str:
            self.load_csv_file(Path(path_str))

    def _reload_current_csv(self) -> None:
        if self.loaded_session is not None and self.loaded_session.csv_path.is_file():
            self.load_csv_file(self.loaded_session.csv_path)

    def load_csv_file(self, csv_path: Path) -> None:
        if self._loader_worker is not None and self._loader_worker.isRunning():
            return
        self.lbl_source_status.setText(f"Loading {csv_path.parent.name}...")
        worker = LogReplayWorker(
            csv_path=csv_path,
            fallback_anchors=self._fallback_anchors(),
            re_solve=self.chk_resolve.isChecked(),
            max_frames=15000,
            parent=self,
        )
        worker.progress.connect(
            lambda done, total: self.lbl_source_status.setText(f"Loading {csv_path.parent.name}: {done:,}/{total:,} rows")
        )
        worker.failed.connect(lambda err: self.lbl_source_status.setText(f"Load failed: {err}"))
        worker.session_ready.connect(self._on_session_loaded)
        self._loader_worker = worker
        worker.start()

    def _on_session_loaded(self, session: LoadedLogSession) -> None:
        self.loaded_session = session
        self.renderer.set_session(session)
        csv_path = session.csv_path
        self.lbl_source_status.setText(
            f"{csv_path.parent.name}/{csv_path.name} ({len(session.frames):,} frames · {session.duration_s:.1f}s)"
        )
        default_mp4 = csv_path.parent / f"{csv_path.parent.name}_replay.mp4"
        self.edit_out_path.setText(str(default_mp4))

        self.edit_t0.setText("0.0")
        self.edit_t1.setText(f"{session.duration_s:.1f}")

        max_idx = max(0, len(session.frames) - 1)
        self.preview_slider.blockSignals(True)
        self.preview_slider.setRange(0, max_idx)
        self.preview_slider.setValue(max_idx // 2 if max_idx > 0 else 0)
        self.preview_slider.blockSignals(False)
        self._on_preview_slider_changed(self.preview_slider.value())
        self._update_estimate_badge()

    def _on_preview_slider_changed(self, idx: int) -> None:
        self.preview_widget.frame_idx = idx
        if self.loaded_session and self.loaded_session.frames:
            fr = self.loaded_session.frames[max(0, min(idx, len(self.loaded_session.frames) - 1))]
            self.lbl_preview_frame.setText(f"{fr.rel_time_s:.2f}s / {self.loaded_session.duration_s:.2f}s")
        self.preview_widget.update()

    def _reset_preview_camera(self) -> None:
        self.renderer.canvas.reset_camera()
        self.preview_widget.update()

    def _on_settings_changed(self) -> None:
        self.renderer.layout_mode = str(self.combo_layout.currentData() or "DASHBOARD")
        self.renderer.canvas.trail_mode = str(self.combo_trail.currentData() or "PROGRESSIVE_GHOST")
        self.renderer.playback_speed = float(self.combo_speed.currentData() or 10.0)
        self._update_estimate_badge()
        self.preview_widget.update()

    def _time_range_values(self) -> tuple[float, float]:
        dur = self.loaded_session.duration_s if self.loaded_session else 0.0
        try:
            t0 = max(0.0, float(self.edit_t0.text().strip()))
        except ValueError:
            t0 = 0.0
        try:
            t1 = max(t0, float(self.edit_t1.text().strip()))
        except ValueError:
            t1 = dur
        return t0, t1

    def _update_estimate_badge(self) -> None:
        if self.loaded_session is None or not self.loaded_session.frames:
            self.lbl_est_info.setText("Est. Video: -- s (-- frames)")
            return
        spd = float(self.combo_speed.currentData() or 10.0)
        fps = float(self.combo_fps.currentData() or 30.0)
        t0, t1 = self._time_range_values()
        indices = select_video_frame_indices(
            self.loaded_session.frames,
            t0,
            t1,
            spd,
            fps,
        )
        vid_s = len(indices) / max(1.0, fps)
        self.lbl_est_info.setText(f"Est. Video: {vid_s:.1f} s ({len(indices):,} frames)")

    def _browse_output_dialog(self) -> None:
        cur = self.edit_out_path.text().strip() or str(NEW_GUI_DIR / "replay_export.mp4")
        path_str, _ = QFileDialog.getSaveFileName(
            self,
            "Save Exported MP4 Video",
            cur,
            "MP4 Video (*.mp4);;All Files (*)",
        )
        if path_str:
            if not path_str.lower().endswith(".mp4"):
                path_str += ".mp4"
            self.edit_out_path.setText(path_str)

    def _toggle_export(self) -> None:
        if self._render_worker is not None and self._render_worker.isRunning():
            self._render_worker.requestInterruption()
            self.lbl_render_status.setText("Cancelling export...")
            return

        if self.loaded_session is None or not self.loaded_session.frames:
            QMessageBox.information(self, "No Log Loaded", "Please load a UWB log CSV first.")
            return

        out_str = self.edit_out_path.text().strip()
        if not out_str:
            self._browse_output_dialog()
            out_str = self.edit_out_path.text().strip()
            if not out_str:
                return

        out_path = Path(out_str)
        w, h = self.combo_res.currentData() or (1280, 720)
        fps = float(self.combo_fps.currentData() or 30.0)
        spd = float(self.combo_speed.currentData() or 10.0)
        layout_mode = str(self.combo_layout.currentData() or "DASHBOARD")
        trail_mode = str(self.combo_trail.currentData() or "PROGRESSIVE_GHOST")
        orbit_rate = 8.0 if self.chk_orbit.isChecked() else 0.0

        self.btn_export.setText("Cancel Export")
        self.btn_export.setObjectName("danger_btn")
        self.btn_export.style().unpolish(self.btn_export)
        self.btn_export.style().polish(self.btn_export)
        self.btn_open_video.setEnabled(False)
        self.progress_bar.setValue(0)

        t0, t1 = self._time_range_values()
        worker = VideoRenderWorker(
            self.loaded_session,
            out_path,
            width=w,
            height=h,
            fps=fps,
            playback_speed=spd,
            start_time_s=t0,
            end_time_s=t1,
            layout_mode=layout_mode,
            trail_mode=trail_mode,
            is_dark=self.renderer.is_dark,
            azimuth_deg=self.renderer.canvas.azimuth_deg,
            elevation_deg=self.renderer.canvas.elevation_deg,
            orbit_deg_per_s=orbit_rate,
            parent=self,
        )
        worker.progress.connect(self._on_render_progress)
        worker.finished_ok.connect(self._on_render_finished)
        worker.failed.connect(self._on_render_failed)
        self._render_worker = worker
        worker.start()

    def _on_render_progress(self, done: int, total: int, r_fps: float) -> None:
        pct = int(100 * done / max(1, total))
        self.progress_bar.setValue(pct)
        rem_s = (total - done) / max(0.1, r_fps)
        self.lbl_render_status.setText(
            f"Rendering frame {done:,} / {total:,} ({pct}%)  ·  {r_fps:.1f} fps  ·  ETA {rem_s:.1f}s"
        )

    def _on_render_finished(self, result: dict) -> None:
        self._reset_export_button()
        if result.get("cancelled"):
            self.lbl_render_status.setText("Export cancelled by user.")
            return
        self.progress_bar.setValue(100)
        out_path = result.get("output_path", "")
        self.btn_open_video.setEnabled(True)
        self.lbl_render_status.setText(
            f"Export complete: {result['frames_rendered']:,} frames ({result['video_duration_s']:.1f}s MP4) in {result['elapsed_s']:.1f}s ({result['render_fps']:.1f} fps) -> {out_path}"
        )

    def _on_render_failed(self, err: str) -> None:
        self._reset_export_button()
        self.lbl_render_status.setText(f"Export failed: {err}")
        QMessageBox.warning(self, "MP4 Export Failed", err)

    def _reset_export_button(self) -> None:
        self.btn_export.setText("Render && Export .MP4")
        self.btn_export.setObjectName("success_btn")
        self.btn_export.style().unpolish(self.btn_export)
        self.btn_export.style().polish(self.btn_export)

    def _open_exported_video(self) -> None:
        out_path = Path(self.edit_out_path.text().strip())
        if out_path.is_file():
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(out_path)))

    def _open_output_folder(self) -> None:
        out_str = self.edit_out_path.text().strip()
        folder = Path(out_str).parent if out_str else (NEW_GUI_DIR / "logs")
        if folder.is_dir():
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(folder)))

    def _toggle_theme(self) -> None:
        if self.theme_manager is None:
            return
        dark = bool(self.theme_manager.toggle())
        self.btn_theme.setText("Theme: Dark" if dark else "Theme: Light")
        self.renderer.is_dark = dark
        self.preview_widget.update()

    def closeEvent(self, event) -> None:  # noqa: N802
        if self._render_worker is not None and self._render_worker.isRunning():
            self._render_worker.requestInterruption()
            self._render_worker.wait(5000)
        if self._loader_worker is not None and self._loader_worker.isRunning():
            self._loader_worker.requestInterruption()
            self._loader_worker.wait(2000)
        super().closeEvent(event)


def parse_resolution(res_str: str) -> tuple[int, int]:
    parts = res_str.lower().replace("×", "x").split("x")
    if len(parts) == 2:
        return int(parts[0]), int(parts[1])
    return 1280, 720


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="UWB Fast Studio — Log-to-MP4 Video Export Studio")
    parser.add_argument("--csv", type=str, default=None, help="Path to uwb_fast_log.csv")
    parser.add_argument("--out", type=str, default=None, help="Output .mp4 file path (for --headless mode)")
    parser.add_argument("--speed", type=float, default=10.0, help="Playback speed multiplier (default: 10.0)")
    parser.add_argument("--fps", type=float, default=30.0, help="Output video FPS (default: 30)")
    parser.add_argument("--resolution", type=str, default="1280x720", help="Video resolution WxH (default: 1280x720)")
    parser.add_argument("--layout", type=str, default="DASHBOARD", choices=("DASHBOARD", "MAP_ONLY"))
    parser.add_argument(
        "--trail",
        type=str,
        default="PROGRESSIVE_GHOST",
        choices=("PROGRESSIVE_GHOST", "PROGRESSIVE_ONLY", "FULL"),
    )
    parser.add_argument("--az", type=float, default=-42.0, help="3D camera azimuth degrees")
    parser.add_argument("--el", type=float, default=26.0, help="3D camera elevation degrees")
    parser.add_argument("--orbit", type=float, default=0.0, help="3D camera auto-orbit deg/s")
    parser.add_argument("--start-time", type=float, default=None, help="Start time in seconds")
    parser.add_argument("--end-time", type=float, default=None, help="End time in seconds")
    parser.add_argument("--dark", action="store_true", help="Use Dark theme")
    parser.add_argument("--re-solve", action="store_true", help="Re-solve 3D trajectory from raw anchor ranges")
    parser.add_argument("--headless", action="store_true", help="Render MP4 directly without opening the GUI window")
    args = parser.parse_args(argv)

    if args.headless:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        if os.name == "nt":
            os.environ.setdefault("QT_QPA_FONTDIR", r"C:\Windows\Fonts")

    app = QApplication.instance() or QApplication(sys.argv)

    if args.headless:
        if not args.csv:
            raise SystemExit("--headless requires --csv <path/to/uwb_fast_log.csv>")
        csv_path = Path(args.csv)
        out_path = Path(args.out) if args.out else csv_path.parent / f"{csv_path.parent.name}_replay.mp4"
        w, h = parse_resolution(args.resolution)
        session = load_log_session(
            csv_path,
            fallback_anchors=list(DEFAULT_PRESET_ROOM_66_36),
            re_solve=args.re_solve,
            max_frames=15000,
        )
        res = export_session_to_mp4(
            session,
            out_path,
            width=w,
            height=h,
            fps=args.fps,
            playback_speed=args.speed,
            start_time_s=args.start_time,
            end_time_s=args.end_time,
            layout_mode=args.layout,
            trail_mode=args.trail,
            is_dark=args.dark,
            azimuth_deg=args.az,
            elevation_deg=args.el,
            orbit_deg_per_s=args.orbit,
        )
        print(
            f"Exported {res['frames_rendered']} frames ({res['video_duration_s']:.1f}s) "
            f"in {res['elapsed_s']:.2f}s ({res['render_fps']:.1f} fps) -> {res['output_path']}"
        )
        return 0

    win = VideoExportStudioWindow(
        initial_csv=Path(args.csv) if args.csv else None,
        initial_speed=args.speed,
        initial_az=args.az,
        initial_el=args.el,
        initial_dark=args.dark,
        initial_resolve=args.re_solve,
    )
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
