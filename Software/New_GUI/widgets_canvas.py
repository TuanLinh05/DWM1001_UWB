"""High-FPS QPainter Canvas Widgets for 2D/3D Positioning Map and Rolling Telemetry Charts.

Key visual guarantees:
1. Every sub-viewport (2D XY, 3D Orbit, Z Altitude Bar) is drawn inside its own
   bordered panel with strict hardware clipping (`painter.setClipRect`), so
   range circles, 3D wireframes, and labels NEVER bleed outside their panel or
   overlap adjacent views.
2. Independent 2D zoom/pan and 3D orbit/zoom controls based on mouse position.
3. Separate toggles for clean Anchor-to-Tag Rays (default ON) vs 2D Range
   Circles (default OFF to avoid 8-circle clutter).
"""

from __future__ import annotations

import math

from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import (
    QColor,
    QFont,
    QMouseEvent,
    QPainter,
    QPaintEvent,
    QPen,
    QPolygonF,
    QWheelEvent,
)
from PyQt6.QtWidgets import QWidget

from core_state import MAP_TRAIL_LEN, UwbStateEngine
from design_tokens import ANCHOR_PALETTE


class PositionMapCanvas(QWidget):
    """Hardware-clipped QPainter 2D XY & 3D Orbit Map with Z-altitude bar."""

    def __init__(self, engine: UwbStateEngine, parent=None) -> None:
        super().__init__(parent)
        self.engine = engine
        self.setMinimumSize(480, 360)
        self.view_mode: str = "SPLIT"       # "2D" | "3D" | "SPLIT"
        self.show_rays: bool = True         # Clean lines from active anchors to TAG
        self.show_ranges: bool = False      # 2D circles (optional so 8 circles don't clutter)
        self.show_trail: bool = True
        self.color_by_stability: bool = True
        self.highlight_time_range: tuple[float, float] | None = None
        self.show_reference: bool = False
        self.reference_xyz: tuple[float, float, float] = (2.5, 2.0, 1.2)
        self.is_dark: bool = False
        self.grid_step_m: float = 0.6

        # Independent 2D & 3D camera states
        self.zoom_2d: float = 1.0
        self.pan_2d_x: float = 0.0
        self.pan_2d_y: float = 0.0

        self.azimuth_deg: float = -42.0
        self.elevation_deg: float = 26.0
        self.zoom_3d: float = 1.0

        self._last_mouse_pos: QPointF | None = None
        self._active_drag_pane: str = "3D"  # "2D" or "3D"
        self._rect_2d = QRectF()
        self._rect_3d = QRectF()

    @property
    def zoom(self) -> float:
        return self.zoom_3d

    @zoom.setter
    def zoom(self, val: float) -> None:
        self.zoom_3d = float(val)
        self.zoom_2d = float(val)

    def reset_camera(self) -> None:
        self.zoom_2d = 1.0
        self.pan_2d_x = 0.0
        self.pan_2d_y = 0.0
        self.azimuth_deg = -42.0
        self.elevation_deg = 26.0
        self.zoom_3d = 1.0
        self.update()

    def _pane_at(self, pos: QPointF) -> str:
        if self.view_mode == "2D":
            return "2D"
        if self.view_mode == "3D":
            return "3D"
        return "2D" if self._rect_2d.contains(pos) else "3D"

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() in (Qt.MouseButton.LeftButton, Qt.MouseButton.RightButton):
            self._last_mouse_pos = event.position()
            self._active_drag_pane = self._pane_at(event.position())

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self._last_mouse_pos is None:
            return
        delta = event.position() - self._last_mouse_pos
        self._last_mouse_pos = event.position()
        if self._active_drag_pane == "3D":
            self.azimuth_deg = (self.azimuth_deg + delta.x() * 0.45) % 360.0
            self.elevation_deg = max(6.0, min(84.0, self.elevation_deg + delta.y() * 0.35))
        else:
            # Smooth 2D pan in meters
            min_x, max_x, min_y, max_y, _, _ = self._scene_bounds()
            span = max(max_x - min_x, max_y - min_y, 1.0)
            m_per_px = span / max(self._rect_2d.width(), 120.0) / self.zoom_2d
            self.pan_2d_x -= delta.x() * m_per_px
            self.pan_2d_y += delta.y() * m_per_px
        self.update()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        self._last_mouse_pos = None

    def wheelEvent(self, event: QWheelEvent) -> None:
        steps = event.angleDelta().y() / 120.0
        if steps == 0:
            return
        pane = self._pane_at(event.position())
        if pane == "2D":
            self.zoom_2d = max(0.5, min(4.0, self.zoom_2d * (1.12 ** steps)))
        else:
            self.zoom_3d = max(0.5, min(3.0, self.zoom_3d * (1.12 ** steps)))
        self.update()

    def _scene_bounds(self) -> tuple[float, float, float, float, float, float]:
        pts: list[tuple[float, float, float]] = [
            (a.x, a.y, a.z) for a in self.engine.anchors.values() if a.enabled
        ]
        if self.engine.latest_solution is not None:
            s = self.engine.latest_solution
            pts.append((s.x, s.y, s.z))
        if self.show_reference:
            pts.append(self.reference_xyz)
        if not pts:
            return (0.0, 5.0, 0.0, 4.0, 0.0, 3.0)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        min_z, max_z = min(0.0, min(zs)), max(zs)
        if max_x - min_x < 0.2:
            max_x = min_x + 1.0
        if max_y - min_y < 0.2:
            max_y = min_y + 1.0
        if max_z - min_z < 0.2:
            max_z = min_z + 1.5
        return (min_x, max_x, min_y, max_y, min_z, max_z)

    def _draw_panel_box(self, painter: QPainter, box: QRectF, title: str) -> QRectF:
        """Draw a distinct sub-viewport card with a header bar and return the inner clipped plot area."""
        bg_col = QColor("#0d1526" if self.is_dark else "#ffffff")
        hdr_bg = QColor("#131d33" if self.is_dark else "#f1f5f9")
        border_col = QColor("#253248" if self.is_dark else "#cbd5e1")
        title_col = QColor("#94a3b8" if self.is_dark else "#475569")

        painter.setPen(QPen(border_col, 1))
        painter.setBrush(bg_col)
        painter.drawRoundedRect(box, 6, 6)

        hdr_rect = QRectF(box.left() + 1, box.top() + 1, box.width() - 2, 24)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(hdr_bg)
        painter.drawRoundedRect(hdr_rect, 5, 5)
        painter.setPen(QPen(border_col, 1))
        painter.drawLine(QPointF(box.left(), box.top() + 25), QPointF(box.right(), box.top() + 25))

        painter.setPen(title_col)
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.drawText(
            QRectF(box.left() + 10, box.top() + 2, box.width() - 16, 22),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            title,
        )
        return QRectF(box.left() + 2, box.top() + 26, box.width() - 4, box.height() - 28)

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing, True)

        rect = QRectF(0, 0, self.width(), self.height())
        outer_bg = QColor("#090e1a" if self.is_dark else "#f8fafc")
        outer_border = QColor("#1e293b" if self.is_dark else "#cbd5e1")
        painter.fillRect(rect, outer_bg)
        painter.setPen(QPen(outer_border, 1))
        painter.drawRoundedRect(rect.adjusted(0.5, 0.5, -0.5, -0.5), 8, 8)

        # Reserve bottom 26px for the clean HUD status strip
        content_h = max(120.0, rect.height() - 36.0)
        z_bar_w = 56.0
        z_box = QRectF(rect.right() - z_bar_w - 8, 8, z_bar_w, content_h)
        main_w = rect.width() - z_bar_w - 24.0

        if self.view_mode == "2D":
            self._rect_2d = QRectF(8, 8, main_w, content_h)
            self._rect_3d = QRectF()
            self._paint_2d(painter, self._rect_2d)
        elif self.view_mode == "3D":
            self._rect_2d = QRectF()
            self._rect_3d = QRectF(8, 8, main_w, content_h)
            self._paint_3d(painter, self._rect_3d)
        else:
            gap = 8.0
            half_w = (main_w - gap) * 0.46
            right_w = main_w - gap - half_w
            self._rect_2d = QRectF(8, 8, half_w, content_h)
            self._rect_3d = QRectF(8 + half_w + gap, 8, right_w, content_h)
            self._paint_2d(painter, self._rect_2d)
            self._paint_3d(painter, self._rect_3d)

        self._paint_z_bar(painter, z_box)
        self._paint_hud_overlay(painter, QRectF(8, rect.bottom() - 24, rect.width() - 16, 20))
        painter.end()

    def _paint_2d(self, painter: QPainter, box: QRectF) -> None:
        step = max(self.grid_step_m, 0.05)
        inner = self._draw_panel_box(painter, box, f"2D TOP-DOWN (X → , Y ↑ · Grid {step:g}m)")
        if inner.width() < 50 or inner.height() < 50:
            return

        painter.save()
        painter.setClipRect(inner)

        min_x, max_x, min_y, max_y, _min_z, _max_z = self._scene_bounds()
        span_x = max(max_x - min_x, 0.4)
        span_y = max(max_y - min_y, 0.4)
        pad = max(span_x, span_y) * 0.22 / self.zoom_2d
        cx = (min_x + max_x) * 0.5 + self.pan_2d_x
        cy = (min_y + max_y) * 0.5 + self.pan_2d_y
        half_span_x = span_x * 0.5 / self.zoom_2d + pad
        half_span_y = span_y * 0.5 / self.zoom_2d + pad
        min_x, max_x = cx - half_span_x, cx + half_span_x
        min_y, max_y = cy - half_span_y, cy + half_span_y

        plot = inner.adjusted(32, 10, -12, -24)
        scale = min(plot.width() / max(max_x - min_x, 0.1), plot.height() / max(max_y - min_y, 0.1))
        offset_x = plot.left() + (plot.width() - (max_x - min_x) * scale) * 0.5
        offset_y = plot.bottom() - (plot.height() - (max_y - min_y) * scale) * 0.5

        def proj(x: float, y: float) -> QPointF:
            return QPointF(offset_x + (x - min_x) * scale, offset_y - (y - min_y) * scale)

        grid_color = QColor("#1e293b" if self.is_dark else "#e2e8f0")
        text_muted = QColor("#94a3b8" if self.is_dark else "#64748b")
        painter.setFont(QFont("Segoe UI", 8))

        # Clip grid lines inside plot rect (exact 0.6 m grid spacing without float drift)
        painter.save()
        painter.setClipRect(plot)
        kx = math.ceil((min_x - 1e-9) / step)
        while True:
            gx = round(kx * step, 3)
            if gx > max_x + 1e-9:
                break
            p1 = proj(gx, min_y)
            p2 = proj(gx, max_y)
            painter.setPen(QPen(grid_color, 1))
            painter.drawLine(p1, p2)
            kx += 1

        ky = math.ceil((min_y - 1e-9) / step)
        while True:
            gy = round(ky * step, 3)
            if gy > max_y + 1e-9:
                break
            p1 = proj(min_x, gy)
            p2 = proj(max_x, gy)
            painter.setPen(QPen(grid_color, 1))
            painter.drawLine(p1, p2)
            ky += 1

        sol = self.engine.latest_solution
        tag_z = sol.z if sol is not None else self.engine.fixed_z_m

        # Optional 2D Range circles (strictly clipped inside `plot` so they never bleed out!)
        if self.show_ranges:
            for aid, cfg in self.engine.anchors.items():
                if not cfg.enabled:
                    continue
                live = self.engine.live[aid]
                r_mm = live.host_filtered_mm or live.corrected_mm
                if r_mm is None or r_mm <= 0 or not live.radio_ok:
                    continue
                r_m = r_mm / 1000.0
                dz = tag_z - cfg.z
                horiz_m = math.sqrt(max(r_m * r_m - dz * dz, 0.0))
                if horiz_m <= 0.01:
                    continue
                center = proj(cfg.x, cfg.y)
                rad_px = horiz_m * scale
                col = QColor(ANCHOR_PALETTE[(aid - 1) % len(ANCHOR_PALETTE)])
                col.setAlpha(75)
                painter.setPen(QPen(col, 1.1, Qt.PenStyle.DashLine))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawEllipse(center, rad_px, rad_px)

        # Clean Rays from active anchors to TAG in 2D
        if sol is not None and self.show_rays:
            tp_pt = proj(sol.x, sol.y)
            for aid in sol.used_anchor_ids:
                cfg = self.engine.anchors.get(aid)
                if cfg and cfg.enabled:
                    r_col = QColor(ANCHOR_PALETTE[(aid - 1) % len(ANCHOR_PALETTE)])
                    r_col.setAlpha(90)
                    painter.setPen(QPen(r_col, 1.1, Qt.PenStyle.DotLine))
                    painter.drawLine(proj(cfg.x, cfg.y), tp_pt)

        # Full-loop trajectory trail (color-coded by stability: Green=STABLE, Amber=WARN, Red=UNSTABLE)
        if self.show_trail:
            self._paint_trajectory_2d(painter, proj)

        # User-marked waypoints (W1, W2, ...)
        if self.engine.waypoints:
            painter.setFont(QFont("Segoe UI", 7, QFont.Weight.Bold))
            wp_col = QColor("#8b5cf6" if not self.is_dark else "#a78bfa")
            for _idx, wlbl, wx, wy, _wz, _wt in self.engine.waypoints:
                wp_pt = proj(wx, wy)
                painter.setPen(QPen(QColor("#ffffff"), 1.2))
                painter.setBrush(wp_col)
                diamond = QPolygonF(
                    [
                        QPointF(wp_pt.x(), wp_pt.y() - 5.5),
                        QPointF(wp_pt.x() + 5.5, wp_pt.y()),
                        QPointF(wp_pt.x(), wp_pt.y() + 5.5),
                        QPointF(wp_pt.x() - 5.5, wp_pt.y()),
                    ]
                )
                painter.drawPolygon(diamond)
                painter.setPen(wp_col)
                painter.drawText(QPointF(wp_pt.x() + 7, wp_pt.y() + 3), wlbl.split()[0])

        # Reference waypoint
        if self.show_reference:
            rp = proj(self.reference_xyz[0], self.reference_xyz[1])
            painter.setPen(QPen(QColor("#10b981"), 2.0))
            painter.drawLine(QPointF(rp.x() - 7, rp.y()), QPointF(rp.x() + 7, rp.y()))
            painter.drawLine(QPointF(rp.x(), rp.y() - 7), QPointF(rp.x(), rp.y() + 7))
            painter.drawEllipse(rp, 4.5, 4.5)

        # Anchors (group co-located XY anchors so A1/A5 labels never overwrite each other)
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        xy_groups: dict[tuple[float, float], list[int]] = {}
        for aid, cfg in sorted(self.engine.anchors.items()):
            if not cfg.enabled:
                continue
            key = (round(cfg.x, 3), round(cfg.y, 3))
            xy_groups.setdefault(key, []).append(aid)

        lbl_col = QColor("#e2e8f0" if self.is_dark else "#0f172a")
        for (gx_a, gy_a), aids in xy_groups.items():
            ap = proj(gx_a, gy_a)
            col = QColor(ANCHOR_PALETTE[(aids[0] - 1) % len(ANCHOR_PALETTE)])
            painter.setPen(QPen(QColor("#ffffff"), 1.5))
            painter.setBrush(col)
            painter.drawEllipse(ap, 6.0, 6.0)
            painter.setPen(lbl_col)
            label_txt = "/".join(f"A{a}" for a in aids)
            # Place label inward if near right edge of plot
            tx = ap.x() - 36 if ap.x() > plot.right() - 42 else ap.x() + 8
            ty = ap.y() + 13 if ap.y() < plot.top() + 16 else ap.y() - 5
            painter.drawText(QPointF(tx, ty), label_txt)

        # Estimated TAG
        if sol is not None:
            tp_pt = proj(sol.x, sol.y)
            glow = QColor("#ef4444")
            glow.setAlpha(65)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(glow)
            painter.drawEllipse(tp_pt, 12, 12)
            painter.setPen(QPen(QColor("#ffffff"), 1.8))
            painter.setBrush(QColor("#ef4444"))
            painter.drawEllipse(tp_pt, 6.5, 6.5)
            painter.setPen(QColor("#f87171" if self.is_dark else "#b91c1c"))
            painter.drawText(QPointF(tp_pt.x() + 9, tp_pt.y() - 6), "TAG")

        painter.restore()  # End plot clip

        # Axis tick labels outside `plot` but inside `inner`
        painter.setFont(QFont("Segoe UI", 7))
        painter.setPen(text_muted)
        last_lbl_px = -1e9
        kx = math.ceil((min_x - 1e-9) / step)
        while True:
            gx = round(kx * step, 3)
            if gx > max_x + 1e-9:
                break
            px = proj(gx, min_y).x()
            if plot.left() <= px <= plot.right() and (px - last_lbl_px) >= 22.0:
                painter.drawText(
                    QRectF(px - 18, plot.bottom() + 3, 36, 15),
                    Qt.AlignmentFlag.AlignCenter,
                    f"{gx:g}",
                )
                last_lbl_px = px
            kx += 1

        last_lbl_py = 1e9
        ky = math.ceil((min_y - 1e-9) / step)
        while True:
            gy = round(ky * step, 3)
            if gy > max_y + 1e-9:
                break
            py = proj(min_x, gy).y()
            if plot.top() <= py <= plot.bottom() and (last_lbl_py - py) >= 14.0:
                painter.drawText(
                    QRectF(inner.left() + 2, py - 7, 26, 14),
                    Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                    f"{gy:g}",
                )
                last_lbl_py = py
            ky += 1

        painter.restore()  # End inner clip

    def _paint_3d(self, painter: QPainter, box: QRectF) -> None:
        step = max(self.grid_step_m, 0.05)
        title = f"3D ORBIT (Az {self.azimuth_deg:.0f}° · El {self.elevation_deg:.0f}° · Grid {step:g}m)"
        inner = self._draw_panel_box(painter, box, title)
        if inner.width() < 50 or inner.height() < 50:
            return

        painter.save()
        painter.setClipRect(inner)

        min_x, max_x, min_y, max_y, min_z, max_z = self._scene_bounds()
        cx = (min_x + max_x) * 0.5
        cy = (min_y + max_y) * 0.5
        cz = (min_z + max_z) * 0.5
        az = math.radians(self.azimuth_deg)
        el = math.radians(self.elevation_deg)

        def raw_proj(pt: tuple[float, float, float]) -> tuple[float, float, float]:
            dx, dy, dz = pt[0] - cx, pt[1] - cy, pt[2] - cz
            hx = -math.sin(az) * dx + math.cos(az) * dy
            depth = math.cos(az) * dx + math.sin(az) * dy
            vy = -math.sin(el) * depth + math.cos(el) * dz
            return hx, vy, depth

        corners = [
            (x, y, z)
            for x in (min_x, max_x)
            for y in (min_y, max_y)
            for z in (min_z, max_z)
        ]
        proj_corners = [raw_proj(c) for c in corners]
        span_x = max(p[0] for p in proj_corners) - min(p[0] for p in proj_corners)
        span_y = max(p[1] for p in proj_corners) - min(p[1] for p in proj_corners)
        scale = min(
            (inner.width() - 56) / max(span_x, 0.2),
            (inner.height() - 44) / max(span_y, 0.2),
        ) * self.zoom_3d
        mid_x = inner.center().x()
        mid_y = inner.center().y() + 4

        def proj3(pt: tuple[float, float, float]) -> QPointF:
            hx, vy, _d = raw_proj(pt)
            return QPointF(mid_x + hx * scale, mid_y - vy * scale)

        # Draw 0.6 m floor grid on the z = min_z plane
        floor_pen = QPen(QColor("#172033" if self.is_dark else "#e2e8f0"), 1.0)
        painter.setPen(floor_pen)
        kx = math.ceil((min_x - 1e-9) / step)
        while True:
            gx = round(kx * step, 3)
            if gx > max_x + 1e-9:
                break
            painter.drawLine(proj3((gx, min_y, min_z)), proj3((gx, max_y, min_z)))
            kx += 1

        ky = math.ceil((min_y - 1e-9) / step)
        while True:
            gy = round(ky * step, 3)
            if gy > max_y + 1e-9:
                break
            painter.drawLine(proj3((min_x, gy, min_z)), proj3((max_x, gy, min_z)))
            ky += 1

        # Draw bounding box wireframe
        box_pen = QPen(QColor("#1e293b" if self.is_dark else "#cbd5e1"), 1.0)
        painter.setPen(box_pen)
        for i, c1 in enumerate(corners):
            for c2 in corners[i + 1 :]:
                if sum(abs(a - b) > 1e-6 for a, b in zip(c1, c2)) == 1:
                    painter.drawLine(proj3(c1), proj3(c2))

        # Coordinate triad at (min_x, min_y, min_z)
        axis_len = max(max_x - min_x, max_y - min_y) * 0.16
        orig = (min_x, min_y, min_z)
        painter.setFont(QFont("Segoe UI", 7, QFont.Weight.Bold))
        for end_pt, col, lbl in (
            ((min_x + axis_len, min_y, min_z), "#ef4444", "X"),
            ((min_x, min_y + axis_len, min_z), "#22c55e", "Y"),
            ((min_x, min_y, min_z + axis_len), "#3b82f6", "Z"),
        ):
            painter.setPen(QPen(QColor(col), 1.8))
            p0, p1 = proj3(orig), proj3(end_pt)
            painter.drawLine(p0, p1)
            painter.drawText(QPointF(p1.x() + 3, p1.y() - 2), lbl)

        sol = self.engine.latest_solution
        if sol is not None and self.show_rays:
            tag_3d = (sol.x, sol.y, sol.z)
            tag_2d = proj3(tag_3d)
            for aid in sol.used_anchor_ids:
                cfg = self.engine.anchors.get(aid)
                if cfg and cfg.enabled:
                    ray_col = QColor(ANCHOR_PALETTE[(aid - 1) % len(ANCHOR_PALETTE)])
                    ray_col.setAlpha(105)
                    painter.setPen(QPen(ray_col, 1.1, Qt.PenStyle.DashLine))
                    painter.drawLine(proj3((cfg.x, cfg.y, cfg.z)), tag_2d)

        # 3D Full-Loop Trajectory Trail (color-coded by stability)
        if self.show_trail:
            self._paint_trajectory_3d(painter, proj3)

        # User-marked waypoints in 3D
        if self.engine.waypoints:
            painter.setFont(QFont("Segoe UI", 7, QFont.Weight.Bold))
            wp_col = QColor("#8b5cf6" if not self.is_dark else "#a78bfa")
            for _idx, wlbl, wx, wy, wz, _wt in self.engine.waypoints:
                wp_pt = proj3((wx, wy, wz))
                painter.setPen(QPen(QColor("#ffffff"), 1.2))
                painter.setBrush(wp_col)
                diamond = QPolygonF(
                    [
                        QPointF(wp_pt.x(), wp_pt.y() - 5.0),
                        QPointF(wp_pt.x() + 5.0, wp_pt.y()),
                        QPointF(wp_pt.x(), wp_pt.y() + 5.0),
                        QPointF(wp_pt.x() - 5.0, wp_pt.y()),
                    ]
                )
                painter.drawPolygon(diamond)
                painter.setPen(wp_col)
                painter.drawText(QPointF(wp_pt.x() + 6, wp_pt.y() + 3), wlbl.split()[0])

        # Reference target in 3D
        if self.show_reference:
            rp = proj3(self.reference_xyz)
            rf = proj3((self.reference_xyz[0], self.reference_xyz[1], min_z))
            painter.setPen(QPen(QColor("#10b981"), 1.0, Qt.PenStyle.DotLine))
            painter.drawLine(rp, rf)
            painter.setPen(QPen(QColor("#10b981"), 2.0))
            painter.drawEllipse(rp, 5, 5)

        # Draw anchors sorted back-to-front
        enabled_anchors = [a for a in self.engine.anchors.values() if a.enabled]
        enabled_anchors.sort(key=lambda a: raw_proj((a.x, a.y, a.z))[2], reverse=True)
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        for cfg in enabled_anchors:
            aid = cfg.anchor_id
            ap = proj3((cfg.x, cfg.y, cfg.z))
            fp = proj3((cfg.x, cfg.y, min_z))
            col = QColor(ANCHOR_PALETTE[(aid - 1) % len(ANCHOR_PALETTE)])
            drop_col = QColor(col)
            drop_col.setAlpha(60)
            painter.setPen(QPen(drop_col, 1.0, Qt.PenStyle.DotLine))
            painter.drawLine(ap, fp)

            painter.setPen(QPen(QColor("#ffffff"), 1.5))
            painter.setBrush(col)
            painter.drawEllipse(ap, 6.0, 6.0)
            painter.setPen(QColor("#e2e8f0" if self.is_dark else "#0f172a"))
            painter.drawText(QPointF(ap.x() + 8, ap.y() - 4), f"A{aid}")

        if sol is not None:
            tp_pt = proj3((sol.x, sol.y, sol.z))
            fp_pt = proj3((sol.x, sol.y, min_z))
            painter.setPen(QPen(QColor("#ef4444"), 1.2, Qt.PenStyle.DashDotLine))
            painter.drawLine(tp_pt, fp_pt)
            painter.drawEllipse(fp_pt, 3.5, 2.0)

            glow = QColor("#ef4444")
            glow.setAlpha(70)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(glow)
            painter.drawEllipse(tp_pt, 13, 13)
            painter.setPen(QPen(QColor("#ffffff"), 2.0))
            painter.setBrush(QColor("#ef4444"))
            painter.drawEllipse(tp_pt, 7.0, 7.0)
            painter.setPen(QColor("#f87171" if self.is_dark else "#b91c1c"))
            painter.drawText(
                QPointF(tp_pt.x() + 9, tp_pt.y() - 7),
                "TAG",
            )

        painter.restore()  # End inner clip

    def _stability_color(self, state: str) -> QColor:
        if not self.color_by_stability:
            return QColor("#0ea5e9" if not self.is_dark else "#38bdf8")
        if state == "UNSTABLE":
            return QColor("#ef4444")
        if state == "WARN":
            return QColor("#f59e0b")
        return QColor("#10b981" if not self.is_dark else "#34d399")

    def _paint_trajectory_2d(self, painter: QPainter, proj) -> None:
        mpath = list(self.engine.map_path)
        if len(mpath) < 2:
            if len(self.engine.position_trail) >= 2:
                poly = QPolygonF([proj(x, y) for x, y, _z, _t in self.engine.position_trail])
                painter.setPen(QPen(self._stability_color("STABLE"), 2.0))
                painter.drawPolyline(poly)
            return

        cur_state = mpath[0][4]
        cur_pts = [proj(mpath[0][0], mpath[0][1])]
        for x, y, _z, _t, st, _rms in mpath[1:]:
            pt = proj(x, y)
            cur_pts.append(pt)
            if st != cur_state:
                w = 2.6 if cur_state == "UNSTABLE" else (2.2 if cur_state == "WARN" else 1.8)
                painter.setPen(QPen(self._stability_color(cur_state), w))
                painter.drawPolyline(QPolygonF(cur_pts))
                cur_state = st
                cur_pts = [pt]
        if len(cur_pts) >= 2:
            w = 2.6 if cur_state == "UNSTABLE" else (2.2 if cur_state == "WARN" else 1.8)
            painter.setPen(QPen(self._stability_color(cur_state), w))
            painter.drawPolyline(QPolygonF(cur_pts))

        if self.highlight_time_range is not None:
            t0, t1 = self.highlight_time_range
            hl_pts = [proj(x, y) for x, y, _z, t, _st, _rms in mpath if (t0 - 0.1) <= t <= (t1 + 0.1)]
            if len(hl_pts) >= 2:
                painter.setPen(QPen(QColor("#ec4899"), 4.2))
                painter.drawPolyline(QPolygonF(hl_pts))
            elif len(hl_pts) == 1:
                painter.setPen(QPen(QColor("#ec4899"), 2.5))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawEllipse(hl_pts[0], 9.0, 9.0)

    def _paint_trajectory_3d(self, painter: QPainter, proj3) -> None:
        mpath = list(self.engine.map_path)
        if len(mpath) < 2:
            if len(self.engine.position_trail) >= 2:
                poly = QPolygonF([proj3((x, y, z)) for x, y, z, _t in self.engine.position_trail])
                painter.setPen(QPen(self._stability_color("STABLE"), 2.0))
                painter.drawPolyline(poly)
            return

        cur_state = mpath[0][4]
        cur_pts = [proj3((mpath[0][0], mpath[0][1], mpath[0][2]))]
        for x, y, z, _t, st, _rms in mpath[1:]:
            pt = proj3((x, y, z))
            cur_pts.append(pt)
            if st != cur_state:
                w = 2.6 if cur_state == "UNSTABLE" else (2.2 if cur_state == "WARN" else 1.8)
                painter.setPen(QPen(self._stability_color(cur_state), w))
                painter.drawPolyline(QPolygonF(cur_pts))
                cur_state = st
                cur_pts = [pt]
        if len(cur_pts) >= 2:
            w = 2.6 if cur_state == "UNSTABLE" else (2.2 if cur_state == "WARN" else 1.8)
            painter.setPen(QPen(self._stability_color(cur_state), w))
            painter.drawPolyline(QPolygonF(cur_pts))

        if self.highlight_time_range is not None:
            t0, t1 = self.highlight_time_range
            hl_pts = [proj3((x, y, z)) for x, y, z, t, _st, _rms in mpath if (t0 - 0.1) <= t <= (t1 + 0.1)]
            if len(hl_pts) >= 2:
                painter.setPen(QPen(QColor("#ec4899"), 4.2))
                painter.drawPolyline(QPolygonF(hl_pts))
            elif len(hl_pts) == 1:
                painter.setPen(QPen(QColor("#ec4899"), 2.5))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawEllipse(hl_pts[0], 9.0, 9.0)

    def _paint_z_bar(self, painter: QPainter, box: QRectF) -> None:
        inner = self._draw_panel_box(painter, box, "Z (m)")
        painter.save()
        painter.setClipRect(inner)

        _min_x, _max_x, _min_y, _max_y, min_z, max_z = self._scene_bounds()
        span_z = max(max_z - min_z, 0.5)
        top = inner.top() + 12
        bottom = inner.bottom() - 12
        track_x = inner.left() + 16

        text_muted = QColor("#94a3b8" if self.is_dark else "#64748b")
        painter.setPen(QPen(QColor("#334155" if self.is_dark else "#cbd5e1"), 2))
        painter.drawLine(QPointF(track_x, top), QPointF(track_x, bottom))

        def z_to_y(z: float) -> float:
            frac = min(max((z - min_z) / span_z, 0.0), 1.0)
            return bottom - frac * (bottom - top)

        painter.setFont(QFont("Segoe UI", 7))
        for frac in (0.0, 0.5, 1.0):
            zv = min_z + frac * span_z
            yv = z_to_y(zv)
            painter.setPen(text_muted)
            painter.drawLine(QPointF(track_x - 4, yv), QPointF(track_x + 4, yv))
            painter.drawText(QPointF(track_x + 6, yv + 3), f"{zv:.1f}")

        for aid, cfg in self.engine.anchors.items():
            if not cfg.enabled:
                continue
            yv = z_to_y(cfg.z)
            col = QColor(ANCHOR_PALETTE[(aid - 1) % len(ANCHOR_PALETTE)])
            painter.setPen(QPen(col, 2.0))
            painter.drawLine(QPointF(track_x - 6, yv), QPointF(track_x + 2, yv))

        sol = self.engine.latest_solution
        if sol is not None:
            yv = z_to_y(sol.z)
            painter.setPen(QPen(QColor("#ffffff"), 1.5))
            painter.setBrush(QColor("#ef4444"))
            painter.drawEllipse(QPointF(track_x, yv), 5.5, 5.5)

        painter.restore()

    def _paint_hud_overlay(self, painter: QPainter, hud_rect: QRectF) -> None:
        sol = self.engine.latest_solution
        dop = sol.dop if sol is not None else self.engine.current_dop()
        painter.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        if sol is not None:
            hud = (
                f"POS ({sol.x:+.3f}, {sol.y:+.3f}, {sol.z:+.3f}) m  |  "
                f"RMS {sol.rms_m * 1000:.0f} mm  |  "
                f"Used {len(sol.used_anchor_ids)}A  |  "
                f"PDOP {dop.pdop:.2f} (HDOP {dop.hdop:.2f}, VDOP {dop.vdop:.2f})"
            )
            if self.is_dark:
                col = QColor("#4ade80" if sol.rms_m < 0.08 else "#facc15")
            else:
                col = QColor("#059669" if sol.rms_m < 0.08 else "#b45309")
        else:
            need = self.engine.solver_dimension + 1
            hud = f"Waiting for >= {need} fresh anchors ({self.engine.solver_dimension}D solver)..."
            col = QColor("#94a3b8" if self.is_dark else "#475569")

        painter.setPen(col)
        painter.drawText(
            hud_rect,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            hud,
        )


class FastRollingChart(QWidget):
    """High-speed QPainter rolling strip chart (Position XYZ or Anchor Ranges)."""

    def __init__(self, engine: UwbStateEngine, parent=None) -> None:
        super().__init__(parent)
        self.engine = engine
        self.setMinimumHeight(180)
        # Default to Position XYZ (9) so the user sees 3 clean X/Y/Z curves instead of 8 tangled sine waves
        self.selected_anchor_id: int = 9   # 9 = Position XYZ, 0 = All Anchors, 1..8 = specific Anchor
        self.window_s: float = 10.0
        self.is_dark: bool = False

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(0, 0, self.width(), self.height())
        bg = QColor("#0d1526" if self.is_dark else "#ffffff")
        border = QColor("#253248" if self.is_dark else "#cbd5e1")
        painter.fillRect(rect, bg)
        painter.setPen(QPen(border, 1))
        painter.drawRoundedRect(rect.adjusted(0.5, 0.5, -0.5, -0.5), 8, 8)

        plot = rect.adjusted(54, 26, -16, -18)
        if plot.width() < 40 or plot.height() < 30:
            painter.end()
            return

        if self.selected_anchor_id == 9:
            self._paint_xyz_series(painter, plot)
        elif self.selected_anchor_id == 0:
            self._paint_all_anchors(painter, plot)
        else:
            self._paint_single_anchor(painter, plot, self.selected_anchor_id)
        painter.end()

    def _draw_grid(
        self,
        painter: QPainter,
        plot: QRectF,
        y_min: float,
        y_max: float,
        unit_fmt: str = "{:.2f}m",
        title: str = "",
        span_t: float | None = None,
    ) -> None:
        grid_col = QColor("#1e293b" if self.is_dark else "#e2e8f0")
        txt_col = QColor("#94a3b8" if self.is_dark else "#64748b")
        painter.setFont(QFont("Consolas", 8))
        span = max(y_max - y_min, 1e-6)
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = plot.bottom() - frac * plot.height()
            val = y_min + frac * span
            painter.setPen(QPen(grid_col, 1))
            painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            painter.setPen(txt_col)
            painter.drawText(
                QRectF(2, y - 8, plot.left() - 6, 16),
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                unit_fmt.format(val),
            )
        if span_t is not None and span_t > 0:
            for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
                x = plot.left() + frac * plot.width()
                rel_t = -(1.0 - frac) * span_t
                lbl = "0.0s" if abs(rel_t) < 1e-3 else f"{rel_t:.1f}s"
                painter.setPen(txt_col)
                painter.drawText(
                    QRectF(x - 24, plot.bottom() + 2, 48, 15),
                    Qt.AlignmentFlag.AlignCenter,
                    lbl,
                )
        if title:
            painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
            painter.drawText(QPointF(plot.left() + 2, plot.top() - 8), title)

    def _paint_all_anchors(self, painter: QPainter, plot: QRectF) -> None:
        latest_t = max(
            (s.tag_time_s for s in self.engine.live.values() if s.rx_count > 0),
            default=0.0,
        )
        t_cutoff = latest_t - self.window_s
        vals: list[float] = []
        series_by_aid: dict[int, list[tuple[float, float]]] = {}
        earliest_t = latest_t
        for aid in range(1, 9):
            pts = [
                (t, host_mm / 1000.0)
                for (t, _raw, _corr, host_mm, _nlos) in self.engine.anchor_history[aid]
                if t >= t_cutoff
            ]
            if len(pts) >= 2:
                earliest_t = min(earliest_t, pts[0][0])
                step = max(1, len(pts) // 320)
                pts = pts[::step]
                series_by_aid[aid] = pts
                vals.extend(v for _t, v in pts)

        if not vals:
            self._draw_grid(painter, plot, 0.0, 5.0, "{:.2f}m", "ALL ANCHORS HOST-FILTERED RANGE (m)", self.window_s)
            return

        span_t = max(latest_t - earliest_t, 0.2)
        y_min, y_max = min(vals), max(vals)
        pad = max((y_max - y_min) * 0.12, 0.05)
        y_min -= pad
        y_max += pad
        self._draw_grid(painter, plot, y_min, y_max, "{:.2f}m", "ALL ANCHORS HOST-FILTERED RANGE (m)", span_t)

        painter.save()
        painter.setClipRect(plot)
        span_y = max(y_max - y_min, 1e-6)
        for aid, pts in series_by_aid.items():
            poly = QPolygonF(
                [
                    QPointF(
                        plot.left() + (t - earliest_t) / span_t * plot.width(),
                        plot.bottom() - (v - y_min) / span_y * plot.height(),
                    )
                    for t, v in pts
                ]
            )
            col = QColor(ANCHOR_PALETTE[(aid - 1) % len(ANCHOR_PALETTE)])
            painter.setPen(QPen(col, 1.5))
            painter.drawPolyline(poly)
        painter.restore()

    def _paint_single_anchor(self, painter: QPainter, plot: QRectF, aid: int) -> None:
        hist = list(self.engine.anchor_history.get(aid, ()))
        if len(hist) < 2:
            self._draw_grid(painter, plot, 0.0, 5.0, "{:.2f}m", f"A{aid} RAW vs CALIBRATED vs HOST FILTER", self.window_s)
            return
        latest_t = hist[-1][0]
        t_cutoff = latest_t - self.window_s
        pts = [p for p in hist if p[0] >= t_cutoff]
        if len(pts) < 2:
            pts = hist[-100:]
        earliest_t = pts[0][0]
        span_t = max(latest_t - earliest_t, 0.2)
        step = max(1, len(pts) // 450)
        pts = pts[::step]

        vals = [p[1] / 1000.0 for p in pts] + [p[2] / 1000.0 for p in pts] + [p[3] / 1000.0 for p in pts]
        y_min, y_max = min(vals), max(vals)
        pad = max((y_max - y_min) * 0.15, 0.02)
        y_min -= pad
        y_max += pad
        self._draw_grid(
            painter,
            plot,
            y_min,
            y_max,
            "{:.3f}m",
            f"A{aid} — Grey: Raw | Amber: Calibrated | Cyan: Host Filter ({len(pts)} pts)",
            span_t,
        )
        painter.save()
        painter.setClipRect(plot)
        span_y = max(y_max - y_min, 1e-6)

        for idx, color_hex, width_px in ((1, "#64748b", 1.1), (2, "#f59e0b", 1.4), (3, "#0ea5e9", 2.0)):
            poly = QPolygonF(
                [
                    QPointF(
                        plot.left() + (p[0] - earliest_t) / span_t * plot.width(),
                        plot.bottom() - (p[idx] / 1000.0 - y_min) / span_y * plot.height(),
                    )
                    for p in pts
                ]
            )
            painter.setPen(QPen(QColor(color_hex), width_px))
            painter.drawPolyline(poly)
        painter.restore()

    def _paint_xyz_series(self, painter: QPainter, plot: QRectF) -> None:
        trail = list(self.engine.position_trail)
        if len(trail) < 2:
            self._draw_grid(painter, plot, 0.0, 5.0, "{:.2f}m", "TAG POSITION — X (Red) · Y (Green) · Z (Blue)", self.window_s)
            return
        latest_t = trail[-1][3]
        t_cutoff = latest_t - self.window_s
        pts = [p for p in trail if p[3] >= t_cutoff]
        if len(pts) < 2:
            pts = trail[-100:]
        earliest_t = pts[0][3]
        span_t = max(latest_t - earliest_t, 0.2)
        step = max(1, len(pts) // 400)
        pts = pts[::step]

        vals = [v for p in pts for v in (p[0], p[1], p[2])]
        y_min, y_max = min(vals), max(vals)
        pad = max((y_max - y_min) * 0.15, 0.08)
        y_min -= pad
        y_max += pad
        self._draw_grid(painter, plot, y_min, y_max, "{:.2f}m", "TAG POSITION — X (Red) · Y (Green) · Z (Blue)", span_t)

        painter.save()
        painter.setClipRect(plot)
        span_y = max(y_max - y_min, 1e-6)

        for axis_idx, col_hex in ((0, "#ef4444"), (1, "#10b981"), (2, "#3b82f6")):
            poly = QPolygonF(
                [
                    QPointF(
                        plot.left() + (p[3] - earliest_t) / span_t * plot.width(),
                        plot.bottom() - (p[axis_idx] - y_min) / span_y * plot.height(),
                    )
                    for p in pts
                ]
            )
            painter.setPen(QPen(QColor(col_hex), 1.9))
            painter.drawPolyline(poly)
        painter.restore()
