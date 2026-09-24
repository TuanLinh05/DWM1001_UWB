"""Advanced Tkinter panels for mapping, analysis and UWB calibration."""

from __future__ import annotations

import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import statistics
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Sequence

from gui_analysis import (
    AnchorMetrics,
    AnchorPosition,
    CalibrationResult,
    DEFAULT_LAYOUT_4,
    DEFAULT_LAYOUT_8,
    FRESH_AGE_MS,
    MAX_ANCHORS,
    PositionSolution,
    RegularSeries,
    SignalStatistics,
    TelemetryPoint,
    analyze_anchor,
    calibration_result,
    histogram_density,
    load_layout,
    overlapping_allan_deviation,
    regularize_time_series,
    save_layout,
    signal_statistics,
    solve_position,
    welch_psd,
)
from telemetry_protocol import (
    AnchorSample,
    STATUS_CALIBRATION_MISSING,
    STATUS_DS_FALLBACK,
    STATUS_RANGE_REJECT,
)


COLORS = ("#2563eb", "#7c3aed", "#0891b2", "#ea580c", "#db2777", "#059669", "#dc2626", "#4f46e5")


def _fmt(value: float | None, digits: int = 1, suffix: str = "") -> str:
    if value is None or not math.isfinite(value):
        return "—"
    return f"{value:.{digits}f}{suffix}"


class AnchorMapPanel(ttk.Frame):
    def __init__(self, parent: tk.Misc, app_directory: Path) -> None:
        super().__init__(parent, padding=8)
        self.layout_path = app_directory / "anchor_layout.json"
        self.layout = self._load_initial_layout()
        self.enabled_vars: dict[int, tk.BooleanVar] = {}
        self.coordinate_vars: dict[int, tuple[tk.StringVar, tk.StringVar, tk.StringVar]] = {}
        self.mode_var = tk.StringVar(value="2D")
        self.fixed_z_var = tk.StringVar(value="0.0")
        self.show_ranges_var = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(value="Đang chờ tối thiểu 3 anchor valid cho nghiệm 2D.")
        self.geometry_var = tk.StringVar()
        self.azimuth_deg = 40.0
        self.latest_samples: dict[int, AnchorSample] = {}
        self.latest_host: dict[int, int] = {}
        self.solution: PositionSolution | None = None
        self.trail: list[tuple[float, float, float]] = []
        self._build()
        self._populate_layout(self.layout)

    def _load_initial_layout(self) -> tuple[AnchorPosition, ...]:
        if self.layout_path.exists():
            try:
                return load_layout(self.layout_path)
            except (OSError, ValueError, json.JSONDecodeError):
                pass
        return DEFAULT_LAYOUT_4

    def _build(self) -> None:
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)
        editor = ttk.LabelFrame(self, text="Anchor layout · ENU (m)", padding=8)
        editor.grid(row=0, column=0, sticky="nsw", padx=(0, 8))
        ttk.Label(editor, text="Dùng tọa độ tâm anten; Z hướng lên.", foreground="#64748b").grid(
            row=0, column=0, columnspan=5, sticky="w", pady=(0, 6)
        )
        ttk.Label(editor, text="Use").grid(row=1, column=0)
        for column, title in enumerate(("Anchor", "X", "Y", "Z"), start=1):
            ttk.Label(editor, text=title, style="Heading.TLabel").grid(row=1, column=column, padx=3)
        for anchor_id in range(1, MAX_ANCHORS + 1):
            enabled = tk.BooleanVar(value=anchor_id <= 4)
            coordinates = tuple(tk.StringVar(value="0") for _ in range(3))
            self.enabled_vars[anchor_id] = enabled
            self.coordinate_vars[anchor_id] = coordinates  # type: ignore[assignment]
            ttk.Checkbutton(editor, variable=enabled, command=self._preview_layout).grid(row=anchor_id + 1, column=0)
            ttk.Label(editor, text=f"A{anchor_id}", width=5).grid(row=anchor_id + 1, column=1)
            for axis, variable in enumerate(coordinates):
                entry = ttk.Entry(editor, textvariable=variable, width=7, justify=tk.RIGHT)
                entry.grid(row=anchor_id + 1, column=axis + 2, padx=2, pady=1)
                entry.bind("<FocusOut>", lambda _event: self._preview_layout())

        preset_row = ttk.Frame(editor)
        preset_row.grid(row=10, column=0, columnspan=5, sticky="ew", pady=(7, 3))
        ttk.Button(preset_row, text="Preset 4A · 2D", command=lambda: self._populate_layout(DEFAULT_LAYOUT_4)).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(preset_row, text="Preset 8A · 3D", command=lambda: self._populate_layout(DEFAULT_LAYOUT_8)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(5, 0))
        ttk.Label(editor, textvariable=self.geometry_var, wraplength=330, foreground="#475569").grid(
            row=11, column=0, columnspan=5, sticky="w", pady=(3, 6)
        )
        action_row = ttk.Frame(editor)
        action_row.grid(row=12, column=0, columnspan=5, sticky="ew")
        ttk.Button(action_row, text="Lưu & áp dụng", command=self.apply_layout).pack(side=tk.LEFT)
        ttk.Button(action_row, text="Nhập JSON", command=self.import_layout).pack(side=tk.LEFT, padx=4)
        ttk.Button(action_row, text="Xuất JSON", command=self.export_layout).pack(side=tk.LEFT)

        display = ttk.LabelFrame(editor, text="Hiển thị / solver", padding=7)
        display.grid(row=13, column=0, columnspan=5, sticky="ew", pady=(8, 0))
        ttk.Label(display, text="Chế độ:").grid(row=0, column=0, sticky="w")
        mode = ttk.Combobox(display, textvariable=self.mode_var, values=("2D", "3D"), state="readonly", width=5)
        mode.grid(row=0, column=1, sticky="w", padx=(4, 10))
        mode.bind("<<ComboboxSelected>>", lambda _event: self._recompute())
        ttk.Label(display, text="TAG Z cố định (2D):").grid(row=0, column=2, sticky="e")
        fixed = ttk.Entry(display, textvariable=self.fixed_z_var, width=7)
        fixed.grid(row=0, column=3, padx=(4, 0))
        fixed.bind("<FocusOut>", lambda _event: self._recompute())
        ttk.Checkbutton(display, text="Vẽ range", variable=self.show_ranges_var, command=self.draw).grid(row=1, column=0, columnspan=2, sticky="w", pady=(5, 0))
        ttk.Button(display, text="Xoay −", command=lambda: self._rotate(-10)).grid(row=1, column=2, pady=(5, 0))
        ttk.Button(display, text="Xoay +", command=lambda: self._rotate(10)).grid(row=1, column=3, pady=(5, 0))

        view = ttk.LabelFrame(self, text="Bản đồ vị trí", padding=6)
        view.grid(row=0, column=1, sticky="nsew")
        view.columnconfigure(0, weight=1)
        view.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(view, background="#f8fafc", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", lambda _event: self.draw())
        ttk.Label(view, textvariable=self.status_var, style="Value.TLabel", wraplength=760).grid(
            row=1, column=0, sticky="ew", pady=(5, 0)
        )

    def _rotate(self, amount: float) -> None:
        self.azimuth_deg = (self.azimuth_deg + amount) % 360
        self.draw()

    def _populate_layout(self, layout: Sequence[AnchorPosition]) -> None:
        by_id = {anchor.anchor_id: anchor for anchor in layout}
        for anchor_id in range(1, MAX_ANCHORS + 1):
            anchor = by_id.get(anchor_id)
            self.enabled_vars[anchor_id].set(anchor is not None)
            values = (anchor.x, anchor.y, anchor.z) if anchor else (0.0, 0.0, 0.0)
            for variable, value in zip(self.coordinate_vars[anchor_id], values):
                variable.set(f"{value:g}")
        self._preview_layout()

    def _read_layout(self) -> tuple[AnchorPosition, ...]:
        result = []
        for anchor_id in range(1, MAX_ANCHORS + 1):
            if not self.enabled_vars[anchor_id].get():
                continue
            try:
                x, y, z = (float(variable.get()) for variable in self.coordinate_vars[anchor_id])
            except ValueError as exc:
                raise ValueError(f"Tọa độ A{anchor_id} không hợp lệ") from exc
            if not all(math.isfinite(value) for value in (x, y, z)):
                raise ValueError(f"Tọa độ A{anchor_id} không hữu hạn")
            result.append(AnchorPosition(anchor_id, x, y, z))
        if len(result) < 3:
            raise ValueError("Cần bật ít nhất 3 anchor")
        return tuple(result)

    def _preview_layout(self) -> None:
        try:
            layout = self._read_layout()
        except ValueError as exc:
            self.geometry_var.set(str(exc))
            return
        xs, ys, zs = ([getattr(anchor, axis) for anchor in layout] for axis in ("x", "y", "z"))
        self.geometry_var.set(
            f"{len(layout)} anchor · {max(xs)-min(xs):.1f} × {max(ys)-min(ys):.1f} × {max(zs)-min(zs):.1f} m · "
            + ("3D volume" if max(zs) - min(zs) > 0.05 else "coplanar / 2D")
        )

    def apply_layout(self, notify: bool = True) -> None:
        try:
            layout = self._read_layout()
            save_layout(self.layout_path, layout)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Anchor layout", str(exc), parent=self)
            return
        self.layout = layout
        self.trail.clear()
        self._recompute()
        if notify:
            messagebox.showinfo("Anchor layout", f"Đã lưu {len(layout)} anchor:\n{self.layout_path}", parent=self)

    def import_layout(self) -> None:
        filename = filedialog.askopenfilename(parent=self, title="Nhập anchor layout", filetypes=(("JSON", "*.json"), ("All files", "*.*")))
        if not filename:
            return
        try:
            layout = load_layout(Path(filename))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            messagebox.showerror("Anchor layout", f"Không đọc được file:\n{exc}", parent=self)
            return
        self._populate_layout(layout)
        self.layout = layout
        self._recompute()

    def export_layout(self) -> None:
        filename = filedialog.asksaveasfilename(parent=self, title="Xuất anchor layout", defaultextension=".json", initialfile="anchor_layout.json", filetypes=(("JSON", "*.json"),))
        if not filename:
            return
        try:
            save_layout(Path(filename), self._read_layout())
        except (OSError, ValueError) as exc:
            messagebox.showerror("Anchor layout", str(exc), parent=self)

    def update_data(self, samples: dict[int, AnchorSample], host_values: dict[int, int]) -> None:
        self.latest_samples = samples.copy()
        self.latest_host = host_values.copy()
        self._recompute()

    def reset_session(self) -> None:
        self.latest_samples.clear()
        self.latest_host.clear()
        self.solution = None
        self.trail.clear()
        self.status_var.set("Đang chờ dữ liệu position.")
        self.draw()

    def _recompute(self) -> None:
        ranges = {}
        for anchor_id, sample in self.latest_samples.items():
            if not sample.valid or sample.age_ms > FRESH_AGE_MS:
                continue
            value = self.latest_host.get(anchor_id) or sample.filtered_mm or sample.raw_mm
            if value > 0:
                ranges[anchor_id] = value
        dimension = 3 if self.mode_var.get() == "3D" else 2
        try:
            fixed_z = float(self.fixed_z_var.get())
        except ValueError:
            fixed_z = 0.0
        self.solution = solve_position(self.layout, ranges, dimension=dimension, fixed_z=fixed_z)
        if self.solution is None:
            need = dimension + 1
            self.status_var.set(f"Chờ ≥ {need} anchor valid có hình học {dimension}D · hiện có {len(ranges)}.")
        else:
            solution = self.solution
            self.trail.append((solution.x, solution.y, solution.z))
            self.trail = self.trail[-250:]
            used = ", ".join(f"A{item}" for item in solution.used_anchor_ids)
            excluded = ", ".join(f"A{item}" for item in solution.excluded_anchor_ids) or "none"
            self.status_var.set(
                f"TAG ({solution.x:.3f}, {solution.y:.3f}, {solution.z:.3f}) m · RMS {solution.rms_m*1000:.0f} mm · "
                f"max residual {solution.max_residual_m*1000:.0f} mm · used [{used}] · excluded {excluded} · geom {solution.geometry_condition:.1f}"
            )
        self.draw()

    def _bounds(self) -> tuple[float, float, float, float, float, float]:
        points = [(anchor.x, anchor.y, anchor.z) for anchor in self.layout]
        if self.solution:
            points.append((self.solution.x, self.solution.y, self.solution.z))
        xs, ys, zs = zip(*points)
        return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)

    def draw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width, height = max(canvas.winfo_width(), 300), max(canvas.winfo_height(), 240)
        if self.mode_var.get() == "3D":
            self._draw_3d(canvas, width, height)
        else:
            self._draw_2d(canvas, width, height)

    def _draw_2d(self, canvas: tk.Canvas, width: int, height: int) -> None:
        min_x, max_x, min_y, max_y, _min_z, _max_z = self._bounds()
        pad_m = max(max_x - min_x, max_y - min_y, 1.0) * 0.12 + 0.25
        min_x, max_x, min_y, max_y = min_x-pad_m, max_x+pad_m, min_y-pad_m, max_y+pad_m
        left, top, right, bottom = 48, 24, width-24, height-42
        scale = min((right-left)/max(max_x-min_x, 0.1), (bottom-top)/max(max_y-min_y, 0.1))
        def project(x: float, y: float) -> tuple[float, float]:
            return left+(x-min_x)*scale, bottom-(y-min_y)*scale
        step = 0.5 if max(max_x-min_x, max_y-min_y) < 5 else 1.0
        grid_x = math.floor(min_x/step)*step
        while grid_x <= max_x:
            x, _ = project(grid_x, min_y)
            canvas.create_line(x, top, x, bottom, fill="#e2e8f0")
            canvas.create_text(x, bottom+14, text=f"{grid_x:g}", fill="#64748b")
            grid_x += step
        grid_y = math.floor(min_y/step)*step
        while grid_y <= max_y:
            _, y = project(min_x, grid_y)
            canvas.create_line(left, y, right, y, fill="#e2e8f0")
            canvas.create_text(left-8, y, text=f"{grid_y:g}", fill="#64748b", anchor=tk.E)
            grid_y += step
        if self.show_ranges_var.get():
            for anchor in self.layout:
                sample = self.latest_samples.get(anchor.anchor_id)
                if sample is None or not sample.valid:
                    continue
                value = self.latest_host.get(anchor.anchor_id) or sample.filtered_mm or sample.raw_mm
                center = project(anchor.x, anchor.y)
                radius = value/1000.0*scale
                canvas.create_oval(center[0]-radius, center[1]-radius, center[0]+radius, center[1]+radius, outline=COLORS[(anchor.anchor_id-1)%len(COLORS)], dash=(4, 4), width=1)
        if len(self.trail) > 1:
            coordinates = [coordinate for point in self.trail for coordinate in project(point[0], point[1])]
            canvas.create_line(*coordinates, fill="#0ea5e9", width=2)
        for anchor in self.layout:
            x, y = project(anchor.x, anchor.y)
            color = COLORS[(anchor.anchor_id-1)%len(COLORS)]
            canvas.create_oval(x-8, y-8, x+8, y+8, fill=color, outline="white", width=2)
            canvas.create_text(x+11, y-10, text=f"A{anchor.anchor_id}\n({anchor.x:g}, {anchor.y:g}, {anchor.z:g})", fill="#0f172a", anchor=tk.SW, font=("Segoe UI", 8, "bold"))
        if self.solution:
            x, y = project(self.solution.x, self.solution.y)
            canvas.create_oval(x-9, y-9, x+9, y+9, fill="#ef4444", outline="white", width=3)
            canvas.create_text(x+12, y+12, text="TAG", fill="#b91c1c", anchor=tk.NW, font=("Segoe UI Semibold", 10))
        canvas.create_text(right, top, text="ENU · nhìn từ trên · X→ / Y↑", fill="#475569", anchor=tk.NE)

    def _draw_3d(self, canvas: tk.Canvas, width: int, height: int) -> None:
        min_x, max_x, min_y, max_y, min_z, max_z = self._bounds()
        if max_z-min_z < 0.05:
            max_z = min_z + 1.0
        center = ((min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2)
        angle, elevation = math.radians(self.azimuth_deg), math.radians(28)
        def raw_project(point: tuple[float, float, float]) -> tuple[float, float]:
            x, y, z = point[0]-center[0], point[1]-center[1], point[2]-center[2]
            horizontal = -math.sin(angle)*x + math.cos(angle)*y
            depth = math.cos(angle)*x + math.sin(angle)*y
            return horizontal, -math.sin(elevation)*depth + math.cos(elevation)*z
        corners = [(x, y, z) for x in (min_x, max_x) for y in (min_y, max_y) for z in (min_z, max_z)]
        projected = [raw_project(point) for point in corners]
        span_x = max(value[0] for value in projected)-min(value[0] for value in projected)
        span_y = max(value[1] for value in projected)-min(value[1] for value in projected)
        scale = min((width-90)/max(span_x, .1), (height-90)/max(span_y, .1))
        def project(point: tuple[float, float, float]) -> tuple[float, float]:
            x, y = raw_project(point)
            return width/2+x*scale, height/2-y*scale
        edges = []
        for index, first in enumerate(corners):
            for second in corners[index+1:]:
                differences = sum(abs(a-b) > 1e-9 for a, b in zip(first, second))
                if differences == 1:
                    edges.append((first, second))
        for first, second in edges:
            canvas.create_line(*project(first), *project(second), fill="#cbd5e1", width=1)
        if len(self.trail) > 1:
            coordinates = [coordinate for point in self.trail for coordinate in project(point)]
            canvas.create_line(*coordinates, fill="#0ea5e9", width=2)
        if self.solution and self.show_ranges_var.get():
            tag_xy = project((self.solution.x, self.solution.y, self.solution.z))
            for anchor in self.layout:
                if anchor.anchor_id in self.solution.used_anchor_ids:
                    canvas.create_line(*project((anchor.x, anchor.y, anchor.z)), *tag_xy, fill="#94a3b8", dash=(4, 3))
        for anchor in sorted(self.layout, key=lambda item: raw_project((item.x, item.y, item.z))[1]):
            x, y = project((anchor.x, anchor.y, anchor.z))
            color = COLORS[(anchor.anchor_id-1)%len(COLORS)]
            canvas.create_oval(x-8, y-8, x+8, y+8, fill=color, outline="white", width=2)
            canvas.create_text(x+10, y-8, text=f"A{anchor.anchor_id} ({anchor.x:g},{anchor.y:g},{anchor.z:g})", fill="#0f172a", anchor=tk.SW, font=("Segoe UI", 8, "bold"))
        if self.solution:
            x, y = project((self.solution.x, self.solution.y, self.solution.z))
            canvas.create_oval(x-10, y-10, x+10, y+10, fill="#ef4444", outline="white", width=3)
            canvas.create_text(x+12, y+12, text="TAG", fill="#b91c1c", anchor=tk.NW, font=("Segoe UI Semibold", 10))
        canvas.create_text(width-16, 16, text=f"ENU 3D · azimuth {self.azimuth_deg:.0f}°", fill="#475569", anchor=tk.NE)


class _LegacyAnalysisPanel(ttk.Frame):
    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent, padding=8)
        self.anchor_var = tk.StringVar(value="A1")
        self.metric_var = tk.StringVar(value="Range")
        self.window_var = tk.StringVar(value="60 s")
        self.summary_var = tk.StringVar(value="Chưa có telemetry trong cửa sổ phân tích.")
        self.cached_points: list[TelemetryPoint] = []
        self.latest_metrics: list[AnchorMetrics] = []
        self._build()

    def _build(self) -> None:
        toolbar = ttk.Frame(self)
        toolbar.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(toolbar, text="Anchor:").pack(side=tk.LEFT)
        anchor = ttk.Combobox(toolbar, textvariable=self.anchor_var, values=tuple(f"A{i}" for i in range(1, MAX_ANCHORS+1)), state="readonly", width=5)
        anchor.pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(toolbar, text="Plot:").pack(side=tk.LEFT)
        metric = ttk.Combobox(toolbar, textvariable=self.metric_var, values=("Range", "FPP", "Age", "Filter Δ"), state="readonly", width=11)
        metric.pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(toolbar, text="Cửa sổ:").pack(side=tk.LEFT)
        window = ttk.Combobox(toolbar, textvariable=self.window_var, values=("15 s", "60 s", "5 min", "Toàn phiên"), state="readonly", width=10)
        window.pack(side=tk.LEFT, padx=(4, 12))
        ttk.Button(toolbar, text="Xuất CSV", command=self.export_csv).pack(side=tk.RIGHT)
        for widget in (anchor, metric, window):
            widget.bind("<<ComboboxSelected>>", lambda _event: self.draw())

        self.canvas = tk.Canvas(self, height=280, background="#111827", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.draw())
        ttk.Label(self, textvariable=self.summary_var, style="Value.TLabel").pack(fill=tk.X, pady=(5, 6))

        columns = ("anchor", "availability", "fresh", "mean", "noise", "range", "delta", "fpp", "age", "problems")
        self.tree = ttk.Treeview(self, columns=columns, show="headings", height=8)
        headings = {
            "anchor": "Anchor", "availability": "Availability", "fresh": "Fresh / total",
            "mean": "Host mean", "noise": "Noise σ", "range": "P05–P95",
            "delta": "|Filter Δ| p95", "fpp": "FPP median", "age": "Age p95",
            "problems": "Invalid / stale / missing",
        }
        widths = {"anchor": 55, "availability": 85, "fresh": 90, "mean": 90, "noise": 75, "range": 125, "delta": 100, "fpp": 90, "age": 75, "problems": 145}
        for column in columns:
            self.tree.heading(column, text=headings[column])
            self.tree.column(column, width=widths[column], anchor=tk.CENTER)
        self.tree.tag_configure("good", background="#e7f5e9")
        self.tree.tag_configure("warn", background="#fff3cd")
        self.tree.tag_configure("bad", background="#fde8e7")
        self.tree.pack(fill=tk.X)

    def _window_seconds(self) -> int:
        return {"15 s": 15, "60 s": 60, "5 min": 300, "Toàn phiên": 0}[self.window_var.get()]

    def refresh(self, histories: dict[int, Sequence[TelemetryPoint]], frame_times: Sequence[float]) -> None:
        seconds = self._window_seconds()
        cutoff = time.monotonic()-seconds if seconds else -math.inf
        selected_id = int(self.anchor_var.get()[1:])
        points_by_id = {anchor_id: [point for point in histories.get(anchor_id, ()) if point.received_s >= cutoff] for anchor_id in range(1, MAX_ANCHORS+1)}
        selected_frames = [value for value in frame_times if value >= cutoff]
        total_frames = len(selected_frames)
        self.latest_metrics = [analyze_anchor(anchor_id, points_by_id[anchor_id], total_frames) for anchor_id in range(1, MAX_ANCHORS+1)]
        self.cached_points = points_by_id[selected_id]
        self.tree.delete(*self.tree.get_children())
        for metrics in self.latest_metrics:
            host = metrics.host
            delta95 = metrics.filter_delta.p95
            tag = "good" if metrics.availability_pct >= 95 else "warn" if metrics.availability_pct >= 70 else "bad"
            self.tree.insert("", tk.END, values=(
                f"A{metrics.anchor_id}", f"{metrics.availability_pct:.1f}%",
                f"{metrics.fresh} / {metrics.total_frames}", _fmt(host.mean, 1, " mm"),
                _fmt(host.stddev, 1, " mm"), f"{_fmt(host.p05, 0)}–{_fmt(host.p95, 0)} mm",
                _fmt(abs(delta95) if delta95 is not None else None, 1, " mm"),
                _fmt(metrics.fpp.median, 1, " dBm"), _fmt(metrics.age.p95, 0, " ms"),
                f"{metrics.invalid} / {metrics.stale} / {metrics.missing}",
            ), tags=(tag,))
        if len(selected_frames) > 1:
            duration = max(selected_frames[-1]-selected_frames[0], 1e-6)
            rate = (len(selected_frames)-1)/duration
        else:
            rate = 0.0
        warnings = sum(item.availability_pct < 95 for item in self.latest_metrics if item.observed)
        self.summary_var.set(f"{total_frames} frame · {rate:.1f} Hz quan sát · {warnings} anchor cảnh báo · fresh age ≤ {FRESH_AGE_MS} ms")
        self.draw()

    def draw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width, height = max(canvas.winfo_width(), 300), max(canvas.winfo_height(), 180)
        left, top, right, bottom = 62, 20, width-20, height-35
        metric = self.metric_var.get()
        points = self.cached_points[-1200:]
        if len(points) < 2:
            canvas.create_text(width/2, height/2, text="Đang chờ dữ liệu cho plot", fill="#94a3b8", font=("Segoe UI", 11))
            return
        if metric == "Range":
            series = (
                ("Raw", [point.raw_mm if point.raw_mm > 0 else None for point in points], "#94a3b8", 1),
                ("Host", [point.host_mm for point in points], "#22c55e", 2),
                ("FW", [point.firmware_mm for point in points], "#38bdf8", 2),
            )
            unit, divisor = "mm", 1.0
        elif metric == "FPP":
            series = (("FPP", [point.fpp_dbm for point in points], "#c084fc", 2),)
            unit, divisor = "dBm", 1.0
        elif metric == "Age":
            series = (("Age", [float(point.age_ms) for point in points], "#fb923c", 2),)
            unit, divisor = "ms", 1.0
        else:
            series = (("Raw − Host", [point.raw_mm-point.host_mm if point.host_mm else None for point in points], "#f472b6", 2),)
            unit, divisor = "mm", 1.0
        values = [float(value) for _name, items, _color, _width in series for value in items if value is not None and math.isfinite(float(value))]
        if not values:
            canvas.create_text(width/2, height/2, text="Không có giá trị phù hợp", fill="#94a3b8")
            return
        minimum, maximum = min(values), max(values)
        padding = max((maximum-minimum)*.12, 1.0)
        minimum, maximum = minimum-padding, maximum+padding
        span = max(maximum-minimum, 1e-9)
        for fraction in (0, .25, .5, .75, 1):
            y = bottom-fraction*(bottom-top)
            value = (minimum+fraction*span)/divisor
            canvas.create_line(left, y, right, y, fill="#263244")
            canvas.create_text(left-7, y, text=f"{value:.1f}", fill="#9ca3af", anchor=tk.E)
        for series_index, (name, items, color, line_width) in enumerate(series):
            segment = []
            for index, value in enumerate(items):
                if value is None:
                    if len(segment) >= 4:
                        canvas.create_line(*segment, fill=color, width=line_width)
                    segment = []
                    continue
                x = left+index*(right-left)/max(len(items)-1, 1)
                y = bottom-(float(value)-minimum)*(bottom-top)/span
                segment.extend((x, y))
            if len(segment) >= 4:
                canvas.create_line(*segment, fill=color, width=line_width)
            legend_x = left+series_index*110
            canvas.create_line(legend_x, 10, legend_x+20, 10, fill=color, width=line_width)
            canvas.create_text(legend_x+25, 10, text=name, fill="#cbd5e1", anchor=tk.W)
        canvas.create_text(right, bottom+18, text=f"{len(points)} mẫu · {unit}", fill="#9ca3af", anchor=tk.E)

    def export_csv(self) -> None:
        if not self.latest_metrics:
            return
        filename = filedialog.asksaveasfilename(parent=self, title="Xuất UWB analysis", defaultextension=".csv", initialfile=time.strftime("uwb_analysis_%Y%m%d_%H%M%S.csv"), filetypes=(("CSV", "*.csv"),))
        if not filename:
            return
        try:
            with Path(filename).open("w", newline="", encoding="utf-8-sig") as handle:
                writer = csv.writer(handle)
                writer.writerow(("anchor", "availability_pct", "fresh", "total_frames", "host_mean_mm", "host_std_mm", "host_p05_mm", "host_p95_mm", "filter_delta_p95_mm", "fpp_median_dbm", "age_p95_ms", "invalid", "stale", "missing"))
                for item in self.latest_metrics:
                    writer.writerow((item.anchor_id, item.availability_pct, item.fresh, item.total_frames, item.host.mean, item.host.stddev, item.host.p05, item.host.p95, item.filter_delta.p95, item.fpp.median, item.age.p95, item.invalid, item.stale, item.missing))
        except OSError as exc:
            messagebox.showerror("Analysis", f"Không xuất được CSV:\n{exc}", parent=self)


class AnalysisPanel(ttk.Frame):
    """Capture-first 2x2 scientific dashboard for one selected UWB anchor.

    Unlike the compact live graph, this panel deliberately freezes its plots
    while a capture is running.  A report is calculated only after the target
    number of usable samples has arrived, which keeps the UART UI responsive
    and makes every exported figure describe one well-defined measurement.
    """

    SOURCE_LABELS = ("Raw", "Host Filter", "Firmware Filter")
    SOURCE_COLORS = {
        "Raw": "#64748b",
        "Host Filter": "#16a34a",
        "Firmware Filter": "#0284c7",
    }

    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent, padding=7)
        self.anchor_var = tk.StringVar(value="A1")
        self.source_var = tk.StringVar(value="Host Filter")
        # Kept for backwards-compatible programmatic refreshes.  Interactive
        # captures always use the complete, frozen sample set.
        self.window_var = tk.StringVar(value="Toàn bộ bộ nhớ")
        self.target_var = tk.StringVar(value="3000")
        self.reference_var = tk.StringVar(value="")
        self.include_diagnostic_var = tk.BooleanVar(value=True)
        self.exclude_fallback_var = tk.BooleanVar(value=True)
        self.capture_status_var = tk.StringVar(
            value="Sẵn sàng. Chọn cấu hình rồi bấm “Bắt đầu lấy mẫu”."
        )
        self.summary_var = tk.StringVar(value="Chưa có phiên đo để phân tích.")
        self.note_var = tk.StringVar(
            value="Trong lúc lấy mẫu, giữ TAG/anchor đứng yên. Đồ thị chỉ xuất hiện sau khi phiên đo kết thúc."
        )
        self.capture_active = False
        self.capture_points: list[TelemetryPoint] = []
        self.capture_target = 3000
        self.capture_accepted = 0
        self.capture_rejected = 0
        self.capture_anchor_id = 1
        self.capture_source = "Host Filter"
        self.capture_started_s: float | None = None
        self.capture_finished_s: float | None = None
        self._capture_lock_widgets: list[tuple[tk.Widget, str]] = []
        self.cached_points: list[TelemetryPoint] = []
        self.latest_metrics: list[AnchorMetrics] = []
        self.signal_stats: SignalStatistics | None = None
        self._histories: dict[int, tuple[TelemetryPoint, ...]] = {}
        self._frame_times: tuple[float, ...] = ()
        self._series: dict[str, tuple[tuple[float, ...], tuple[float, ...]]] = {}
        self._histogram = histogram_density(())
        self._allan: tuple[tuple[float, ...], tuple[float, ...]] = ((), ())
        self._psd: dict[str, tuple[tuple[float, ...], tuple[float, ...]]] = {}
        self._regular: RegularSeries | None = None
        self._last_science_refresh = 0.0
        self._build()

    def _build(self) -> None:
        capture_box = ttk.LabelFrame(self, text="Phiên lấy mẫu", padding=(8, 6))
        capture_box.pack(fill=tk.X, pady=(0, 5))
        toolbar = ttk.Frame(capture_box)
        toolbar.pack(fill=tk.X)
        ttk.Label(toolbar, text="Anchor:").pack(side=tk.LEFT)
        self.anchor_combo = ttk.Combobox(
            toolbar, textvariable=self.anchor_var,
            values=tuple(f"A{i}" for i in range(1, MAX_ANCHORS + 1)),
            state="readonly", width=5,
        )
        self.anchor_combo.pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(toolbar, text="Nguồn phân tích:").pack(side=tk.LEFT)
        self.source_combo = ttk.Combobox(
            toolbar, textvariable=self.source_var, values=self.SOURCE_LABELS,
            state="readonly", width=15,
        )
        self.source_combo.pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(toolbar, text="Số mẫu:").pack(side=tk.LEFT)
        self.target_entry = ttk.Entry(toolbar, textvariable=self.target_var, width=8)
        self.target_entry.pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(toolbar, text="Khoảng cách chuẩn (m):").pack(side=tk.LEFT)
        self.reference_entry = ttk.Entry(toolbar, textvariable=self.reference_var, width=9)
        self.reference_entry.pack(side=tk.LEFT, padx=(4, 4))
        ttk.Label(toolbar, text="để trống = so với trung bình", foreground="#64748b").pack(side=tk.LEFT)

        self.start_button = ttk.Button(
            toolbar, text="Bắt đầu lấy mẫu", command=self.toggle_capture
        )
        self.start_button.pack(side=tk.RIGHT, padx=(6, 0))
        self.cancel_button = ttk.Button(
            toolbar, text="Hủy", command=self.cancel_capture, state=tk.DISABLED
        )
        self.cancel_button.pack(side=tk.RIGHT)

        options = ttk.Frame(capture_box)
        options.pack(fill=tk.X, pady=(6, 0))
        self.diagnostic_check = ttk.Checkbutton(
            options, text="Gồm raw/host diagnostic CAL_MISSING",
            variable=self.include_diagnostic_var, command=self._controls_changed,
        )
        self.diagnostic_check.pack(side=tk.LEFT)
        self.fallback_check = ttk.Checkbutton(
            options, text="Tách DS fallback khỏi thống kê chính",
            variable=self.exclude_fallback_var, command=self._controls_changed,
        )
        self.fallback_check.pack(side=tk.LEFT, padx=(16, 0))
        self.save_plot_button = ttk.Button(
            options, text="Lưu biểu đồ…", command=self.save_plot, state=tk.DISABLED
        )
        self.save_plot_button.pack(side=tk.RIGHT)
        self.export_button = ttk.Button(
            options, text="Lưu dữ liệu CSV…", command=self.export_csv, state=tk.DISABLED
        )
        self.export_button.pack(side=tk.RIGHT, padx=(0, 5))
        for widget in (self.anchor_combo, self.source_combo):
            widget.bind("<<ComboboxSelected>>", lambda _event: self._controls_changed())
        self.reference_entry.bind("<Return>", lambda _event: self._controls_changed())
        self.reference_entry.bind("<FocusOut>", lambda _event: self._controls_changed())
        self._capture_lock_widgets = [
            (self.anchor_combo, "readonly"),
            (self.source_combo, "readonly"),
            (self.target_entry, "normal"),
            (self.reference_entry, "normal"),
            (self.diagnostic_check, "normal"),
            (self.fallback_check, "normal"),
        ]

        self.capture_progress = ttk.Progressbar(capture_box, maximum=100.0)
        self.capture_progress.pack(fill=tk.X, pady=(6, 2))
        ttk.Label(
            capture_box, textvariable=self.capture_status_var, style="Value.TLabel",
        ).pack(fill=tk.X)

        ttk.Label(
            self, textvariable=self.summary_var, style="Value.TLabel", wraplength=1220,
        ).pack(fill=tk.X, pady=(1, 3))
        ttk.Label(
            self, textvariable=self.note_var, foreground="#92400e", wraplength=1220,
        ).pack(fill=tk.X, pady=(0, 4))

        plots = ttk.Frame(self)
        plots.pack(fill=tk.BOTH, expand=True)
        plots.grid_columnconfigure(0, weight=1, uniform="plot")
        plots.grid_columnconfigure(1, weight=1, uniform="plot")
        plots.grid_rowconfigure(0, weight=1, uniform="plot")
        plots.grid_rowconfigure(1, weight=1, uniform="plot")
        self.time_canvas = self._make_plot_canvas(plots, 0, 0)
        self.hist_canvas = self._make_plot_canvas(plots, 0, 1)
        self.allan_canvas = self._make_plot_canvas(plots, 1, 0)
        self.psd_canvas = self._make_plot_canvas(plots, 1, 1)
        self.canvas = self.time_canvas  # compatibility with older integration tests

        columns = (
            "anchor", "availability", "fresh", "mean", "noise", "range",
            "delta", "fpp", "age", "problems",
        )
        self.tree = ttk.Treeview(self, columns=columns, show="headings", height=2)
        headings = {
            "anchor": "Anchor", "availability": "Availability", "fresh": "Fresh / total",
            "mean": "Host mean", "noise": "Noise σ", "range": "P05–P95",
            "delta": "|Filter Δ| p95", "fpp": "FPP median", "age": "Age p95",
            "problems": "Invalid / stale / missing",
        }
        widths = {
            "anchor": 55, "availability": 85, "fresh": 90, "mean": 90,
            "noise": 75, "range": 125, "delta": 100, "fpp": 90,
            "age": 75, "problems": 145,
        }
        for column in columns:
            self.tree.heading(column, text=headings[column])
            self.tree.column(column, width=widths[column], anchor=tk.CENTER)
        self.tree.tag_configure("good", background="#e7f5e9")
        self.tree.tag_configure("warn", background="#fff3cd")
        self.tree.tag_configure("bad", background="#fde8e7")
        self.tree.pack(fill=tk.X, pady=(5, 0))

    def _set_capture_controls_locked(self, locked: bool) -> None:
        for widget, normal_state in self._capture_lock_widgets:
            widget.configure(state=tk.DISABLED if locked else normal_state)
        self.cancel_button.configure(state=tk.NORMAL if locked else tk.DISABLED)

    def toggle_capture(self) -> None:
        if self.capture_active:
            self.finish_capture(automatic=False)
            return
        self.start_capture()

    def start_capture(self) -> None:
        try:
            target = int(self.target_var.get().strip())
        except ValueError:
            messagebox.showerror("Phân tích", "Số mẫu phải là một số nguyên.", parent=self)
            return
        _reference, reference_ok = self._reference_mm()
        if not reference_ok:
            messagebox.showerror(
                "Phân tích",
                "Khoảng cách chuẩn phải là số dương theo mét hoặc để trống.",
                parent=self,
            )
            return
        if not 100 <= target <= 50000:
            messagebox.showerror(
                "Phân tích", "Hãy chọn từ 100 đến 50.000 mẫu.", parent=self
            )
            return

        self.capture_target = target
        self.capture_anchor_id = int(self.anchor_var.get()[1:])
        self.capture_source = self.source_var.get()
        self.capture_points = []
        self.capture_accepted = 0
        self.capture_rejected = 0
        self.capture_started_s = time.monotonic()
        self.capture_finished_s = None
        self.capture_active = True
        self._histories = {}
        self._frame_times = ()
        self.cached_points = []
        self.latest_metrics = []
        self.signal_stats = None
        self._series = {}
        self._histogram = histogram_density(())
        self._allan = ((), ())
        self._psd = {}
        self._regular = None
        self.start_button.configure(text="Dừng và phân tích")
        self.save_plot_button.configure(state=tk.DISABLED)
        self.export_button.configure(state=tk.DISABLED)
        self._set_capture_controls_locked(True)
        self.capture_progress.configure(value=0.0)
        self.capture_status_var.set(
            f"Đang lấy mẫu A{self.capture_anchor_id} · {self.capture_source} · 0/{target}"
        )
        self.summary_var.set("Đang thu dữ liệu; đồ thị được giữ trống cho tới khi kết thúc.")
        self.note_var.set(
            "Không di chuyển TAG/anchor và không đổi cấu hình radio trong suốt phiên đo."
        )
        self.tree.delete(*self.tree.get_children())
        self.draw()

    def ingest(self, sample: AnchorSample, point: TelemetryPoint) -> None:
        """Collect one selected-anchor point while a batch capture is active."""
        if not self.capture_active or sample.anchor_id != self.capture_anchor_id:
            return
        self.capture_points.append(point)
        value = self._point_value(point, self.capture_source)
        if value is not None and self._point_usable(point, self.capture_source):
            self.capture_accepted += 1
        else:
            self.capture_rejected += 1
        self.capture_progress.configure(
            value=min(100.0, self.capture_accepted * 100.0 / self.capture_target)
        )
        self.capture_status_var.set(
            f"Đang lấy mẫu A{self.capture_anchor_id} · "
            f"{self.capture_accepted}/{self.capture_target} hợp lệ · "
            f"loại {self.capture_rejected}"
        )
        if self.capture_accepted >= self.capture_target:
            self.finish_capture(automatic=True)

    def finish_capture(self, *, automatic: bool) -> None:
        if not self.capture_active:
            return
        self.capture_active = False
        self.capture_finished_s = time.monotonic()
        self.start_button.configure(text="Bắt đầu lấy mẫu")
        self._set_capture_controls_locked(False)
        if self.capture_accepted < 20:
            self.capture_status_var.set(
                f"Phiên đo dừng với {self.capture_accepted} mẫu hợp lệ; cần ít nhất 20 mẫu."
            )
            self.capture_progress.configure(value=0.0)
            self.note_var.set("Hãy bắt đầu lại một phiên đo dài hơn.")
            return

        anchor_id = self.capture_anchor_id
        self.anchor_var.set(f"A{anchor_id}")
        self.source_var.set(self.capture_source)
        histories = {
            item: tuple(self.capture_points) if item == anchor_id else ()
            for item in range(1, MAX_ANCHORS + 1)
        }
        frame_times = tuple(point.received_s for point in self.capture_points)
        self.refresh(histories, frame_times)
        self.capture_progress.configure(value=100.0)
        duration = (
            self.capture_finished_s - self.capture_started_s
            if self.capture_started_s is not None else 0.0
        )
        completion = "Đã đủ mẫu" if automatic else "Đã dừng sớm"
        self.capture_status_var.set(
            f"{completion} · {self.capture_accepted} mẫu hợp lệ · "
            f"loại {self.capture_rejected} · {duration:.1f} s"
        )
        self.note_var.set(
            "Phiên đo đã đóng băng. Có thể đổi nguồn/chuẩn để tính lại, rồi lưu PNG/PDF/SVG hoặc CSV."
        )
        self.save_plot_button.configure(state=tk.NORMAL)
        self.export_button.configure(state=tk.NORMAL)

    def cancel_capture(self, message: str = "Đã hủy phiên lấy mẫu.") -> None:
        if not self.capture_active:
            return
        self.capture_active = False
        self.capture_points = []
        self.capture_accepted = 0
        self.capture_rejected = 0
        self.capture_started_s = None
        self.start_button.configure(text="Bắt đầu lấy mẫu")
        self._set_capture_controls_locked(False)
        self.capture_progress.configure(value=0.0)
        self.capture_status_var.set(message)
        self.summary_var.set("Chưa có phiên đo để phân tích.")
        self.note_var.set(
            "Trong lúc lấy mẫu, giữ TAG/anchor đứng yên. Đồ thị chỉ xuất hiện sau khi phiên đo kết thúc."
        )

    def reset_session(self) -> None:
        if self.capture_active:
            self.cancel_capture("Kết nối thay đổi; phiên lấy mẫu đã được hủy.")
        self.capture_points = []
        self.capture_accepted = 0
        self.capture_rejected = 0
        self.capture_started_s = None
        self.capture_finished_s = None
        self._histories = {}
        self._frame_times = ()
        self.cached_points = []
        self.latest_metrics = []
        self.signal_stats = None
        self._series = {}
        self._histogram = histogram_density(())
        self._allan = ((), ())
        self._psd = {}
        self._regular = None
        self.capture_progress.configure(value=0.0)
        self.capture_status_var.set(
            "Sẵn sàng. Chọn cấu hình rồi bấm “Bắt đầu lấy mẫu”."
        )
        self.summary_var.set("Chưa có phiên đo để phân tích.")
        self.save_plot_button.configure(state=tk.DISABLED)
        self.export_button.configure(state=tk.DISABLED)
        self.tree.delete(*self.tree.get_children())
        self.draw()

    def _make_plot_canvas(self, parent: ttk.Frame, row: int, column: int) -> tk.Canvas:
        canvas = tk.Canvas(
            parent, background="#ffffff", highlightthickness=1,
            highlightbackground="#cbd5e1", height=205,
        )
        canvas.grid(row=row, column=column, sticky="nsew", padx=3, pady=3)
        canvas.bind("<Configure>", lambda _event: self.draw())
        return canvas

    def _window_seconds(self) -> int:
        return {"15 s": 15, "60 s": 60, "5 min": 300, "Toàn bộ bộ nhớ": 0}.get(
            self.window_var.get(), 60
        )

    def _controls_changed(self) -> None:
        if self.capture_active:
            return
        self._recompute(force_science=True)

    def refresh(
        self,
        histories: dict[int, Sequence[TelemetryPoint]],
        frame_times: Sequence[float],
    ) -> None:
        self._histories = {
            anchor_id: tuple(histories.get(anchor_id, ()))
            for anchor_id in range(1, MAX_ANCHORS + 1)
        }
        self._frame_times = tuple(frame_times)
        self._recompute(force_science=False)

    def _reference_mm(self) -> tuple[float | None, bool]:
        text = self.reference_var.get().strip().replace(",", ".")
        if not text:
            return None, True
        try:
            value = float(text) * 1000.0
        except ValueError:
            return None, False
        return (value, True) if math.isfinite(value) and value > 0.0 else (None, False)

    @staticmethod
    def _point_value(point: TelemetryPoint, source: str) -> float | None:
        if source == "Raw":
            return float(point.raw_mm) if point.raw_mm > 0 else None
        if source == "Host Filter":
            return float(point.host_mm) if point.host_mm is not None and point.host_mm > 0 else None
        return (
            float(point.firmware_mm)
            if point.firmware_mm is not None and point.firmware_mm > 0 else None
        )

    def _point_usable(self, point: TelemetryPoint, source: str) -> bool:
        if self.exclude_fallback_var.get() and point.status & STATUS_DS_FALLBACK:
            return False
        if point.valid and point.age_ms <= FRESH_AGE_MS:
            return True
        diagnostic = bool(point.status & (STATUS_CALIBRATION_MISSING | STATUS_RANGE_REJECT))
        return (
            self.include_diagnostic_var.get()
            and diagnostic
            and source in ("Raw", "Host Filter")
        )

    def _select_series(
        self, points: Sequence[TelemetryPoint], source: str
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        selected: list[tuple[TelemetryPoint, float]] = []
        for point in points:
            value = self._point_value(point, source)
            if value is not None and self._point_usable(point, source):
                selected.append((point, value))
        return self._unwrap_times([item[0] for item in selected]), tuple(
            item[1] for item in selected
        )

    @staticmethod
    def _unwrap_times(points: Sequence[TelemetryPoint]) -> tuple[float, ...]:
        """Use the TAG clock for spectral timing while surviving wrap/reset.

        Every RANGE frame contains one record per anchor, so the frame clock is
        the consistent cadence for Raw/Firmware/Host comparisons. ``age_ms``
        remains a quality/latency metric; it cannot be used as a timestamp for
        diagnostic raw because it refers to the last production-valid range.
        """
        if not points:
            return ()
        result = [points[0].received_s]
        elapsed = 0.0
        previous_frame = points[0].tag_time_ms & 0xFFFFFFFF
        previous_received = points[0].received_s
        for point in points[1:]:
            frame_time = point.tag_time_ms & 0xFFFFFFFF
            frame_delta_ms = (frame_time - previous_frame) & 0xFFFFFFFF
            if 0 < frame_delta_ms < 0x80000000:
                elapsed += frame_delta_ms / 1000.0
            else:
                # Do not invent a 1 us interval for duplicate frames; that
                # previously produced a false ~1 MHz sample rate. Duplicate
                # timestamps are collapsed by clean_time_series().
                host_delta = point.received_s - previous_received
                if host_delta >= 0.001:
                    elapsed += host_delta
            result.append(result[0] + elapsed)
            previous_frame = frame_time
            previous_received = point.received_s
        return tuple(result)

    def _recompute(self, force_science: bool) -> None:
        latest_time = self._frame_times[-1] if self._frame_times else time.monotonic()
        seconds = self._window_seconds()
        cutoff = latest_time - seconds if seconds else -math.inf
        selected_frames = [value for value in self._frame_times if value >= cutoff]
        points_by_id = {
            anchor_id: [
                point for point in self._histories.get(anchor_id, ())
                if point.received_s >= cutoff
            ]
            for anchor_id in range(1, MAX_ANCHORS + 1)
        }
        self.latest_metrics = [
            analyze_anchor(anchor_id, points_by_id[anchor_id], len(selected_frames))
            for anchor_id in range(1, MAX_ANCHORS + 1)
        ]
        self._update_table()

        selected_id = int(self.anchor_var.get()[1:])
        self.cached_points = points_by_id[selected_id]
        self._series = {
            source: self._select_series(self.cached_points, source)
            for source in self.SOURCE_LABELS
        }
        selected_times, selected_values = self._series[self.source_var.get()]
        reference_mm, reference_ok = self._reference_mm()
        if reference_ok:
            self.signal_stats = signal_statistics(
                selected_times, selected_values, reference_mm=reference_mm
            )
        else:
            self.signal_stats = None

        if self.signal_stats is not None:
            errors = [value - self.signal_stats.reference_mm for value in selected_values]
            self._histogram = histogram_density(errors)
        else:
            self._histogram = histogram_density(())

        now = time.monotonic()
        if force_science or now - self._last_science_refresh >= 1.0:
            self._last_science_refresh = now
            self._regular = regularize_time_series(selected_times, selected_values)
            if self._regular is not None:
                self._allan = overlapping_allan_deviation(
                    self._regular.values, self._regular.sample_rate_hz
                )
            else:
                self._allan = ((), ())
            self._psd = {}
            for source, (times_s, values) in self._series.items():
                regular = regularize_time_series(times_s, values)
                if regular is not None:
                    self._psd[source] = welch_psd(
                        regular.values, regular.sample_rate_hz,
                        maximum_fft=1024, maximum_segments=8,
                    )
        self._update_summary(reference_ok)
        self.draw()

    def _update_table(self) -> None:
        self.tree.delete(*self.tree.get_children())
        for metrics in self.latest_metrics:
            if metrics.total_frames == 0:
                continue
            host = metrics.host
            delta95 = metrics.filter_delta.p95
            tag = (
                "good" if metrics.availability_pct >= 95.0
                else "warn" if metrics.availability_pct >= 70.0 else "bad"
            )
            self.tree.insert("", tk.END, values=(
                f"A{metrics.anchor_id}", f"{metrics.availability_pct:.1f}%",
                f"{metrics.fresh} / {metrics.total_frames}", _fmt(host.mean, 1, " mm"),
                _fmt(host.stddev, 1, " mm"),
                f"{_fmt(host.p05, 0)}–{_fmt(host.p95, 0)} mm",
                _fmt(abs(delta95) if delta95 is not None else None, 1, " mm"),
                _fmt(metrics.fpp.median, 1, " dBm"), _fmt(metrics.age.p95, 0, " ms"),
                f"{metrics.invalid} / {metrics.stale} / {metrics.missing}",
            ), tags=(tag,))

    def _update_summary(self, reference_ok: bool) -> None:
        if not reference_ok:
            self.summary_var.set("Khoảng cách chuẩn không hợp lệ; hãy nhập số dương theo mét hoặc để trống.")
            return
        stats = self.signal_stats
        if stats is None:
            self.summary_var.set(
                f"{self.anchor_var.get()} · {self.source_var.get()} · chưa đủ mẫu phù hợp."
            )
            return
        reference_text = (
            "trung bình của phiên đo" if stats.reference_is_mean
            else f"chuẩn {stats.reference_mm / 1000.0:.4f} m"
        )
        allan_tau, allan_values = self._allan
        if allan_values:
            index = min(range(len(allan_values)), key=lambda item: allan_values[item])
            allan_text = f"Allan min {allan_values[index]:.2f} mm @ {allan_tau[index]:.2f} s"
        else:
            allan_text = "Allan: cần thêm mẫu liên tục"
        fallback_count = sum(bool(point.status & STATUS_DS_FALLBACK) for point in self.cached_points)
        gap_text = f" · gap {self._regular.gap_count}" if self._regular is not None else ""
        self.summary_var.set(
            f"{self.anchor_var.get()} · {self.source_var.get()} · N={stats.sample_count} · "
            f"{_fmt(stats.sample_rate_hz, 2, ' Hz')} · {stats.duration_s:.1f} s{gap_text} · "
            f"mean {stats.mean_mm / 1000.0:.4f} m · σ {stats.stddev_mm:.1f} mm · "
            f"bias {stats.bias_mm:+.1f} mm · RMSE {stats.rmse_mm:.1f} mm · "
            f"P95 |e| {stats.p95_abs_error_mm:.1f} mm · "
            f"drift {_fmt(stats.drift_mm_per_min, 1, ' mm/min')} · {allan_text} · "
            f"tham chiếu: {reference_text} · DS fallback thấy {fallback_count}"
        )

    def draw(self) -> None:
        self._draw_time_series()
        self._draw_histogram()
        self._draw_allan()
        self._draw_psd()

    @staticmethod
    def _format_tick(value: float, logarithmic: bool = False) -> str:
        if logarithmic:
            exponent = math.log10(value)
            if abs(exponent - round(exponent)) < 1e-6:
                return f"10^{round(exponent)}"
        absolute = abs(value)
        if absolute >= 1000.0 or (0.0 < absolute < 0.01):
            return f"{value:.1e}"
        if absolute >= 10.0:
            return f"{value:.1f}"
        return f"{value:.2f}"

    def _axes(
        self,
        canvas: tk.Canvas,
        title: str,
        x_values: Sequence[float],
        y_values: Sequence[float],
        x_label: str,
        y_label: str,
        *,
        log_x: bool = False,
        log_y: bool = False,
        zero_y_baseline: bool = False,
        right_margin: int = 18,
    ):
        canvas.delete("all")
        width, height = max(canvas.winfo_width(), 280), max(canvas.winfo_height(), 160)
        finite_x = [float(value) for value in x_values if math.isfinite(float(value)) and (not log_x or value > 0)]
        finite_y = [float(value) for value in y_values if math.isfinite(float(value)) and (not log_y or value > 0)]
        canvas.create_text(width / 2, 13, text=title, fill="#0f172a", font=("Segoe UI Semibold", 10))
        if not finite_x or not finite_y:
            canvas.create_text(width / 2, height / 2, text="Chưa đủ dữ liệu", fill="#64748b", font=("Segoe UI", 10))
            return None
        x_min, x_max = min(finite_x), max(finite_x)
        y_min, y_max = min(finite_y), max(finite_y)
        if x_min == x_max:
            if log_x:
                x_min, x_max = x_min / math.sqrt(10.0), x_max * math.sqrt(10.0)
            else:
                x_min, x_max = x_min - 0.5, x_max + 0.5
        if y_min == y_max:
            if log_y:
                y_min, y_max = y_min / math.sqrt(10.0), y_max * math.sqrt(10.0)
            else:
                padding = max(abs(y_min) * 0.02, 1.0)
                y_min, y_max = y_min - padding, y_max + padding
        if zero_y_baseline and not log_y:
            y_min = 0.0
            y_max = max(y_max, 1e-12)
            y_max += max(y_max * 0.08, 1e-12)
        elif not log_y:
            padding = max((y_max - y_min) * 0.08, 1e-9)
            y_min, y_max = y_min - padding, y_max + padding
        left, top, right, bottom = 59, 39, width - right_margin, height - 38
        tx_min, tx_max = (math.log10(x_min), math.log10(x_max)) if log_x else (x_min, x_max)
        ty_min, ty_max = (math.log10(y_min), math.log10(y_max)) if log_y else (y_min, y_max)

        def transform(x_value: float, y_value: float) -> tuple[float, float]:
            tx = math.log10(x_value) if log_x else x_value
            ty = math.log10(y_value) if log_y else y_value
            x = left + (tx - tx_min) * (right - left) / max(tx_max - tx_min, 1e-12)
            y = bottom - (ty - ty_min) * (bottom - top) / max(ty_max - ty_min, 1e-12)
            return x, y

        transform.log_x = log_x
        transform.log_y = log_y

        for index in range(5):
            fraction = index / 4.0
            x_tick_transformed = tx_min + fraction * (tx_max - tx_min)
            y_tick_transformed = ty_min + fraction * (ty_max - ty_min)
            x_tick = 10 ** x_tick_transformed if log_x else x_tick_transformed
            y_tick = 10 ** y_tick_transformed if log_y else y_tick_transformed
            x = left + fraction * (right - left)
            y = bottom - fraction * (bottom - top)
            canvas.create_line(x, top, x, bottom, fill="#e2e8f0")
            canvas.create_line(left, y, right, y, fill="#e2e8f0")
            canvas.create_text(x, bottom + 12, text=self._format_tick(x_tick, log_x), fill="#475569", font=("Segoe UI", 7))
            canvas.create_text(left - 5, y, text=self._format_tick(y_tick, log_y), fill="#475569", anchor=tk.E, font=("Segoe UI", 7))
        canvas.create_line(left, bottom, right, bottom, fill="#334155")
        canvas.create_line(left, top, left, bottom, fill="#334155")
        canvas.create_text((left + right) / 2, height - 8, text=x_label, fill="#334155", font=("Segoe UI", 8))
        canvas.create_text(10, (top + bottom) / 2, text=y_label, fill="#334155", angle=90, font=("Segoe UI", 8))
        return transform, (left, top, right, bottom), (x_min, x_max, y_min, y_max)

    @staticmethod
    def _decimate_pairs(
        x_values: Sequence[float], y_values: Sequence[float], maximum_points: int
    ) -> list[tuple[float, float]]:
        pairs = [(float(x), float(y)) for x, y in zip(x_values, y_values) if math.isfinite(float(x)) and math.isfinite(float(y))]
        if len(pairs) <= maximum_points:
            return pairs
        bucket = max(1, math.ceil(len(pairs) / max(maximum_points // 2, 1)))
        result: list[tuple[float, float]] = []
        for start in range(0, len(pairs), bucket):
            group = pairs[start:start + bucket]
            minimum = min(group, key=lambda item: item[1])
            maximum = max(group, key=lambda item: item[1])
            result.extend(sorted((minimum, maximum), key=lambda item: item[0]))
        return result

    def _draw_curve(
        self, canvas: tk.Canvas, transform, x_values: Sequence[float], y_values: Sequence[float],
        color: str, width: float = 1.5, dash=None,
    ) -> None:
        pairs = self._decimate_pairs(x_values, y_values, max(canvas.winfo_width() * 2, 300))
        coordinates: list[float] = []
        for x_value, y_value in pairs:
            if x_value <= 0 and getattr(transform, "log_x", False):
                continue
            if y_value <= 0 and getattr(transform, "log_y", False):
                continue
            x, y = transform(x_value, y_value)
            coordinates.extend((x, y))
        if len(coordinates) >= 4:
            canvas.create_line(*coordinates, fill=color, width=width, dash=dash)

    def _draw_curve_with_gaps(
        self, canvas: tk.Canvas, transform, x_values: Sequence[float], y_values: Sequence[float],
        color: str, width: float = 1.5,
    ) -> None:
        if len(x_values) < 2:
            return
        intervals = [later - earlier for earlier, later in zip(x_values, x_values[1:]) if later > earlier]
        nominal = statistics.median(intervals) if intervals else 0.0
        threshold = nominal * 3.0 if nominal > 0.0 else math.inf
        start = 0
        for index, interval in enumerate(
            [later - earlier for earlier, later in zip(x_values, x_values[1:])]
        ):
            if interval > threshold:
                self._draw_curve(
                    canvas, transform, x_values[start:index + 1], y_values[start:index + 1],
                    color, width,
                )
                start = index + 1
        self._draw_curve(canvas, transform, x_values[start:], y_values[start:], color, width)

    @staticmethod
    def _legend(canvas: tk.Canvas, items: Sequence[tuple[str, str]]) -> None:
        x = 66
        for label, color in items:
            canvas.create_line(x, 29, x + 17, 29, fill=color, width=2)
            canvas.create_text(x + 21, 29, text=label, fill="#334155", anchor=tk.W, font=("Segoe UI", 7))
            x += 37 + len(label) * 5

    def _draw_time_series(self) -> None:
        available = []
        all_times: list[float] = []
        all_values_m: list[float] = []
        first_time = min(
            (times[0] for times, values in self._series.values() if times and values),
            default=0.0,
        )
        for source in self.SOURCE_LABELS:
            times, values = self._series.get(source, ((), ()))
            if not values:
                continue
            relative = tuple(value - first_time for value in times)
            metres = tuple(value / 1000.0 for value in values)
            available.append((source, relative, metres, self.SOURCE_COLORS[source]))
            all_times.extend(relative)
            all_values_m.extend(metres)
        if self.signal_stats is not None:
            all_values_m.append(self.signal_stats.reference_mm / 1000.0)
        rate_text = _fmt(self.signal_stats.sample_rate_hz, 2, " Hz") if self.signal_stats else "—"
        axes = self._axes(
            self.time_canvas,
            f"Chuỗi thời gian · {self.anchor_var.get()} · {rate_text}",
            all_times, all_values_m, "Thời gian (s)", "Khoảng cách (m)", right_margin=47,
        )
        if axes is None:
            return
        transform, bounds, _limits = axes
        for _source, times, values, color in available:
            self._draw_curve_with_gaps(self.time_canvas, transform, times, values, color)
        legend = [(source, color) for source, _times, _values, color in available]
        if self.signal_stats is not None:
            y = self.signal_stats.reference_mm / 1000.0
            x_min, x_max = min(all_times), max(all_times)
            self._draw_curve(self.time_canvas, transform, (x_min, x_max), (y, y), "#dc2626", 1, (4, 3))
            legend.append(("Mean/Ref", "#dc2626"))
        self._legend(self.time_canvas, legend)

        fpp_points = [
            point for point in self.cached_points
            if point.fpp_dbm is not None and math.isfinite(point.fpp_dbm)
            and not (self.exclude_fallback_var.get() and point.status & STATUS_DS_FALLBACK)
            and (
                (point.valid and point.age_ms <= FRESH_AGE_MS)
                or (
                    self.include_diagnostic_var.get()
                    and point.status & (STATUS_CALIBRATION_MISSING | STATUS_RANGE_REJECT)
                )
            )
        ]
        fpp_times = self._unwrap_times(fpp_points)
        fpp_samples = [
            (timestamp - first_time, float(point.fpp_dbm))
            for timestamp, point in zip(fpp_times, fpp_points)
            if point.fpp_dbm is not None
        ]
        if len(fpp_samples) >= 2:
            fpp_min = min(value for _timestamp, value in fpp_samples)
            fpp_max = max(value for _timestamp, value in fpp_samples)
            if fpp_max == fpp_min:
                fpp_min -= 1.0
                fpp_max += 1.0
            left, top, right, bottom = bounds
            coordinates = []
            for timestamp, value in self._decimate_pairs(
                [item[0] for item in fpp_samples], [item[1] for item in fpp_samples],
                max(self.time_canvas.winfo_width(), 300),
            ):
                x, _ = transform(timestamp, all_values_m[0])
                y = bottom - (value - fpp_min) * (bottom - top) / (fpp_max - fpp_min)
                coordinates.extend((x, y))
            if len(coordinates) >= 4:
                self.time_canvas.create_line(*coordinates, fill="#a855f7", width=1, dash=(2, 3))
                self.time_canvas.create_text(right + 4, top, text=f"{fpp_max:.0f}", fill="#7e22ce", anchor=tk.W, font=("Segoe UI", 7))
                self.time_canvas.create_text(right + 4, bottom, text=f"{fpp_min:.0f}", fill="#7e22ce", anchor=tk.W, font=("Segoe UI", 7))
                self.time_canvas.create_text(right + 23, (top + bottom) / 2, text="FPP dBm", fill="#7e22ce", angle=90, font=("Segoe UI", 7))

    def _draw_histogram(self) -> None:
        histogram = self._histogram
        reference_is_mean = self.signal_stats.reference_is_mean if self.signal_stats else True
        x_label = "Độ lệch so với trung bình (mm)" if reference_is_mean else "Sai số so với khoảng cách chuẩn (mm)"
        # Include the physical zero baseline so bars always end on the X axis.
        y_values = [0.0] + list(histogram.density) + list(histogram.normal_density)
        if histogram.centers:
            half_width = histogram.bin_width / 2.0
            x_limits = (
                histogram.centers[0] - half_width,
                histogram.centers[-1] + half_width,
            )
        else:
            x_limits = ()
        axes = self._axes(
            self.hist_canvas,
            f"Phân bố sai số / độ lệch · {len(histogram.centers)} cột",
            x_limits,
            y_values, x_label, "Mật độ", zero_y_baseline=True,
        )
        if axes is None:
            return
        transform, _bounds, _limits = axes
        for center, density in zip(histogram.centers, histogram.density):
            x0, y0 = transform(center - histogram.bin_width * 0.46, 0.0)
            x1, y1 = transform(center + histogram.bin_width * 0.46, density)
            self.hist_canvas.create_rectangle(x0, y1, x1, y0, fill="#38bdf8", outline="#0369a1")
        self._draw_curve(
            self.hist_canvas, transform, histogram.centers, histogram.normal_density,
            "#ef4444", 1.5,
        )
        self._legend(self.hist_canvas, (("Dữ liệu", "#0284c7"), ("Chuẩn khớp", "#ef4444")))

    def _draw_allan(self) -> None:
        taus, deviations = self._allan
        title = "Allan deviation"
        ideal: tuple[float, ...] = ()
        if deviations:
            index = min(range(len(deviations)), key=lambda item: deviations[item])
            title += f" · min {deviations[index]:.2f} mm @ {taus[index]:.2f} s"
            if deviations[0] > 0.0 and taus[0] > 0.0:
                ideal = tuple(deviations[0] * math.sqrt(taus[0] / tau) for tau in taus)
        axes = self._axes(
            self.allan_canvas, title, taus, tuple(deviations) + ideal,
            "τ (s)", "σ Allan (mm)",
            log_x=True, log_y=True,
        )
        if axes is None:
            return
        transform, _bounds, _limits = axes
        self._draw_curve(self.allan_canvas, transform, taus, deviations, "#0369a1", 2)
        if ideal:
            self._draw_curve(self.allan_canvas, transform, taus, ideal, "#f97316", 1, (5, 4))
            self._legend(self.allan_canvas, (("Allan", "#0369a1"), ("White noise −1/2", "#f97316")))

    def _draw_psd(self) -> None:
        series = []
        all_frequencies: list[float] = []
        all_density: list[float] = []
        for source in self.SOURCE_LABELS:
            frequencies, density = self._psd.get(source, ((), ()))
            pairs = [(frequency, value) for frequency, value in zip(frequencies, density) if frequency > 0.0 and value > 0.0]
            if not pairs:
                continue
            x_values = tuple(item[0] for item in pairs)
            y_values = tuple(item[1] for item in pairs)
            series.append((source, x_values, y_values, self.SOURCE_COLORS[source]))
            all_frequencies.extend(x_values)
            all_density.extend(y_values)
        axes = self._axes(
            self.psd_canvas, "Mật độ phổ Welch · kiểm tra bộ lọc",
            all_frequencies, all_density, "Tần số (Hz)", "PSD (mm²/Hz)",
            log_x=True, log_y=True,
        )
        if axes is None:
            return
        transform, _bounds, _limits = axes
        for _source, frequencies, density, color in series:
            self._draw_curve(self.psd_canvas, transform, frequencies, density, color, 1.3)
        self._legend(self.psd_canvas, tuple((source, color) for source, _x, _y, color in series))

    def _build_export_figure(self):
        """Build a publication-style Matplotlib figure for the frozen capture."""
        try:
            from matplotlib.figure import Figure
        except ImportError as exc:  # pragma: no cover - depends on field PC setup
            raise RuntimeError(
                "Thiếu Matplotlib. Hãy chạy: py -3.12 -m pip install -r requirements.txt"
            ) from exc

        if self.signal_stats is None:
            raise RuntimeError("Chưa có phiên đo hợp lệ để vẽ.")

        stats = self.signal_stats
        figure = Figure(figsize=(16, 10), dpi=120, facecolor="#f8fafc")
        axes = figure.subplots(2, 2)
        figure.subplots_adjust(
            left=0.065, right=0.96, bottom=0.075, top=0.90, wspace=0.28, hspace=0.34
        )
        figure.suptitle(
            f"Báo cáo UWB · {self.anchor_var.get()} · {self.source_var.get()} · "
            f"N={stats.sample_count}",
            fontsize=18, fontweight="bold", color="#0f172a",
        )
        for axis in axes.flat:
            axis.set_facecolor("#ffffff")
            axis.grid(True, color="#cbd5e1", alpha=0.55, linewidth=0.7)
            axis.tick_params(colors="#334155", labelsize=9)
            axis.spines["top"].set_visible(False)
            axis.spines["right"].set_visible(False)

        # Time series: keep all three distance sources and place FPP on the
        # secondary axis, matching the information density of the reference.
        time_axis = axes[0, 0]
        first_time = min(
            (times[0] for times, values in self._series.values() if times and values),
            default=0.0,
        )
        for source in self.SOURCE_LABELS:
            times_s, values = self._series.get(source, ((), ()))
            if not times_s or not values:
                continue
            selected_source = source == self.source_var.get()
            pairs = self._decimate_pairs(
                [value - first_time for value in times_s],
                [value / 1000.0 for value in values],
                12000,
            )
            time_axis.plot(
                [item[0] for item in pairs], [item[1] for item in pairs],
                color="#111827" if selected_source else self.SOURCE_COLORS[source],
                linewidth=1.65 if selected_source else 0.8,
                alpha=1.0 if selected_source else 0.28, label=source,
            )
        if self._series.get(self.source_var.get(), ((), ()))[0]:
            selected_times = self._series[self.source_var.get()][0]
            x_min = selected_times[0] - first_time
            x_max = selected_times[-1] - first_time
            time_axis.plot(
                (x_min, x_max),
                (stats.reference_mm / 1000.0, stats.reference_mm / 1000.0),
                color="#15803d", linewidth=1.4, linestyle=(0, (7, 5)),
                label="Trung bình/chuẩn",
            )
        time_axis.set_title(
            f"Chuỗi thời gian · trung vị {stats.median_mm / 1000.0:.4f} m",
            fontsize=13, color="#0f172a",
        )
        time_axis.set_xlabel("Thời gian (s)")
        time_axis.set_ylabel("Khoảng cách (m)")
        time_axis.legend(loc="best", fontsize=8, framealpha=0.92)

        fpp_points = [
            point for point in self.cached_points
            if point.fpp_dbm is not None and math.isfinite(point.fpp_dbm)
            and not (self.exclude_fallback_var.get() and point.status & STATUS_DS_FALLBACK)
        ]
        if len(fpp_points) >= 2:
            fpp_times = self._unwrap_times(fpp_points)
            fpp_axis = time_axis.twinx()
            pairs = self._decimate_pairs(
                [value - first_time for value in fpp_times],
                [float(point.fpp_dbm) for point in fpp_points if point.fpp_dbm is not None],
                6000,
            )
            fpp_axis.plot(
                [item[0] for item in pairs], [item[1] for item in pairs],
                color="#ea580c", linewidth=0.9, alpha=0.72, label="FPP",
            )
            fpp_axis.set_ylabel("FPP (dBm)", color="#c2410c")
            fpp_axis.tick_params(axis="y", colors="#c2410c", labelsize=8)
            fpp_axis.spines["top"].set_visible(False)

        histogram_axis = axes[0, 1]
        histogram = self._histogram
        histogram_axis.bar(
            histogram.centers, histogram.density,
            width=histogram.bin_width * 0.88, color="#ecfeff",
            edgecolor="#06b6d4", linewidth=1.0, label="Dữ liệu",
        )
        histogram_axis.plot(
            histogram.centers, histogram.normal_density,
            color="#ef4444", linewidth=2.0, label="Chuẩn khớp",
        )
        histogram_axis.set_title(
            f"Phân bố · σ {stats.stddev_mm:.1f} mm", fontsize=13, color="#0f172a"
        )
        histogram_axis.set_xlabel(
            "Độ lệch so với trung bình (mm)"
            if stats.reference_is_mean else "Sai số so với khoảng cách chuẩn (mm)"
        )
        histogram_axis.set_ylabel("Mật độ")
        histogram_axis.legend(loc="best", fontsize=8)

        allan_axis = axes[1, 0]
        taus, deviations = self._allan
        positive = [(tau, value) for tau, value in zip(taus, deviations) if tau > 0 and value > 0]
        if positive:
            plot_taus = [item[0] for item in positive]
            plot_deviations = [item[1] for item in positive]
            allan_axis.loglog(
                plot_taus, plot_deviations, "o-", color="#0891b2",
                markerfacecolor="#0f172a", markersize=4, linewidth=1.2,
                label="Allan deviation",
            )
            ideal = [
                plot_deviations[0] * math.sqrt(plot_taus[0] / tau)
                for tau in plot_taus
            ]
            allan_axis.loglog(
                plot_taus, ideal, color="#f97316", linewidth=1.1,
                linestyle=(0, (7, 5)), label="Nhiễu trắng lý tưởng",
            )
            minimum_index = min(range(len(plot_deviations)), key=plot_deviations.__getitem__)
            allan_title = (
                f"Lấy trung bình bao lâu? · min {plot_deviations[minimum_index]:.2f} mm "
                f"@ {plot_taus[minimum_index]:.2f} s"
            )
        else:
            allan_title = "Lấy trung bình bao lâu? · chưa đủ dữ liệu liên tục"
        allan_axis.set_title(allan_title, fontsize=13, color="#0f172a")
        allan_axis.set_xlabel("τ (s)")
        allan_axis.set_ylabel("σ Allan (mm)")
        if positive:
            allan_axis.legend(loc="best", fontsize=8)

        psd_axis = axes[1, 1]
        plotted_psd = False
        for source in self.SOURCE_LABELS:
            frequencies, density = self._psd.get(source, ((), ()))
            pairs = [
                (frequency, value) for frequency, value in zip(frequencies, density)
                if frequency > 0.0 and value > 0.0
            ]
            if not pairs:
                continue
            plotted_psd = True
            psd_axis.loglog(
                [item[0] for item in pairs], [item[1] for item in pairs],
                color=self.SOURCE_COLORS[source], linewidth=1.15, label=source,
            )
        psd_axis.set_title("Mật độ phổ Welch · kiểm tra bộ lọc", fontsize=13, color="#0f172a")
        psd_axis.set_xlabel("Tần số (Hz)")
        psd_axis.set_ylabel("PSD (mm²/Hz)")
        if plotted_psd:
            psd_axis.legend(loc="best", fontsize=8)

        figure.text(
            0.5, 0.025,
            f"mean {stats.mean_mm / 1000.0:.4f} m  ·  RMSE {stats.rmse_mm:.1f} mm  ·  "
            f"P95 |e| {stats.p95_abs_error_mm:.1f} mm  ·  "
            f"drift {_fmt(stats.drift_mm_per_min, 1, ' mm/min')}",
            ha="center", color="#334155", fontsize=10,
        )
        return figure

    def save_plot(self) -> None:
        if self.signal_stats is None:
            messagebox.showwarning("Phân tích", "Chưa có phiên đo để lưu.", parent=self)
            return
        filename = filedialog.asksaveasfilename(
            parent=self, title="Lưu biểu đồ phân tích",
            defaultextension=".png",
            initialfile=time.strftime(
                f"uwb_analysis_{self.anchor_var.get()}_%Y%m%d_%H%M%S.png"
            ),
            filetypes=(("PNG", "*.png"), ("PDF", "*.pdf"), ("SVG", "*.svg")),
        )
        if not filename:
            return
        try:
            figure = self._build_export_figure()
            figure.savefig(filename, dpi=180, facecolor=figure.get_facecolor())
        except (OSError, RuntimeError, ValueError) as exc:
            messagebox.showerror("Phân tích", f"Không lưu được biểu đồ:\n{exc}", parent=self)
            return
        messagebox.showinfo("Phân tích", f"Đã lưu biểu đồ:\n{filename}", parent=self)

    def export_csv(self) -> None:
        if not self.capture_points or self.signal_stats is None:
            messagebox.showwarning("Phân tích", "Chưa có phiên đo để lưu.", parent=self)
            return
        filename = filedialog.asksaveasfilename(
            parent=self, title="Lưu dữ liệu phiên phân tích", defaultextension=".csv",
            initialfile=time.strftime(
                f"uwb_analysis_{self.anchor_var.get()}_%Y%m%d_%H%M%S.csv"
            ),
            filetypes=(("CSV", "*.csv"),),
        )
        if not filename:
            return
        try:
            with Path(filename).open("w", newline="", encoding="utf-8-sig") as handle:
                writer = csv.writer(handle)
                writer.writerow((
                    "sample", "elapsed_s", "tag_time_ms", "anchor", "accepted",
                    "valid", "status_hex", "age_ms", "raw_mm", "firmware_mm",
                    "host_mm", "fpp_dbm",
                ))
                unwrapped = self._unwrap_times(self.capture_points)
                start = unwrapped[0] if unwrapped else 0.0
                for index, (timestamp, point) in enumerate(
                    zip(unwrapped, self.capture_points), start=1
                ):
                    value = self._point_value(point, self.source_var.get())
                    accepted = value is not None and self._point_usable(
                        point, self.source_var.get()
                    )
                    writer.writerow((
                        index, f"{timestamp - start:.6f}", point.tag_time_ms,
                        self.capture_anchor_id, int(accepted), int(point.valid),
                        f"0x{point.status:02X}", point.age_ms, point.raw_mm,
                        "" if point.firmware_mm is None else point.firmware_mm,
                        "" if point.host_mm is None else point.host_mm,
                        "" if point.fpp_dbm is None else f"{point.fpp_dbm:.2f}",
                    ))
                writer.writerow(())
                writer.writerow(("metric", "value"))
                writer.writerow(("selected_anchor", self.anchor_var.get()))
                writer.writerow(("selected_source", self.source_var.get()))
                writer.writerow(("capture_source", self.capture_source))
                writer.writerow(("target_samples", self.capture_target))
                writer.writerow(("accepted_samples", self.capture_accepted))
                writer.writerow(("rejected_samples", self.capture_rejected))
                for key, value in asdict(self.signal_stats).items():
                    writer.writerow((key, value))
        except OSError as exc:
            messagebox.showerror("Phân tích", f"Không lưu được CSV:\n{exc}", parent=self)
            return
        messagebox.showinfo("Phân tích", f"Đã lưu dữ liệu:\n{filename}", parent=self)


class CalibrationPanel(ttk.Frame):
    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent, padding=8)
        self.anchor_var = tk.StringVar(value="A1")
        self.reference_var = tk.StringVar(value="1.000")
        self.target_var = tk.StringVar(value="500")
        self.live_var = tk.StringVar(value="Chưa bắt đầu capture.")
        self.offset_var = tk.StringVar(value="Offset tổng hợp sẽ xuất hiện sau khi có capture.")
        self.capture_active = False
        self.capture_points: list[TelemetryPoint] = []
        self.capture_anchor_id = 1
        self.capture_reference_mm = 1000.0
        self.capture_target = 500
        self.rejected = 0
        self.results: list[CalibrationResult] = []
        self._build()

    def _build(self) -> None:
        notice = ttk.LabelFrame(self, text="Calibration an toàn", padding=8)
        notice.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(notice, text="Thu raw range khi TAG và anchor đứng yên. Offset = khoảng cách chuẩn − raw mean. Không trộn SS-TWR và DS-TWR; luôn validation ở khoảng cách chưa dùng để fit.", wraplength=1080, foreground="#92400e").pack(anchor=tk.W)
        controls = ttk.Frame(notice)
        controls.pack(fill=tk.X, pady=(7, 0))
        ttk.Label(controls, text="Anchor:").pack(side=tk.LEFT)
        ttk.Combobox(controls, textvariable=self.anchor_var, values=tuple(f"A{i}" for i in range(1, MAX_ANCHORS+1)), state="readonly", width=5).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(controls, text="Khoảng cách chuẩn (m):").pack(side=tk.LEFT)
        ttk.Entry(controls, textvariable=self.reference_var, width=9).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Label(controls, text="Mẫu:").pack(side=tk.LEFT)
        ttk.Entry(controls, textvariable=self.target_var, width=7).pack(side=tk.LEFT, padx=(4, 8))
        self.start_button = ttk.Button(controls, text="Bắt đầu capture", command=self.toggle_capture)
        self.start_button.pack(side=tk.LEFT, padx=(3, 4))
        ttk.Button(controls, text="Bỏ capture", command=self.discard_capture).pack(side=tk.LEFT)
        recommendations = ttk.Frame(notice)
        recommendations.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(recommendations, text="Điểm gợi ý:").pack(side=tk.LEFT)
        for distance in (0.5, 1.0, 2.0, 3.0, 5.0, 8.0):
            ttk.Button(recommendations, text=f"{distance:g} m", width=6, command=lambda value=distance: self.reference_var.set(f"{value:.3f}")).pack(side=tk.LEFT, padx=2)
        self.progress = ttk.Progressbar(notice, maximum=100)
        self.progress.pack(fill=tk.X, pady=(7, 3))
        ttk.Label(notice, textvariable=self.live_var, style="Value.TLabel").pack(anchor=tk.W)

        columns = ("anchor", "reference", "samples", "mean", "offset", "noise", "range", "drift", "fpp", "quality")
        self.tree = ttk.Treeview(self, columns=columns, show="headings", height=10)
        headings = {"anchor":"Anchor", "reference":"Reference", "samples":"Samples", "mean":"Raw mean", "offset":"Offset", "noise":"Noise σ", "range":"P05–P95", "drift":"Drift", "fpp":"FPP median", "quality":"Quality"}
        widths = {"anchor":55, "reference":90, "samples":70, "mean":90, "offset":90, "noise":80, "range":120, "drift":80, "fpp":90, "quality":80}
        for column in columns:
            self.tree.heading(column, text=headings[column])
            self.tree.column(column, width=widths[column], anchor=tk.CENTER)
        self.tree.tag_configure("good", background="#e7f5e9")
        self.tree.tag_configure("warn", background="#fff3cd")
        self.tree.pack(fill=tk.BOTH, expand=True)
        footer = ttk.Frame(self)
        footer.pack(fill=tk.X, pady=(7, 0))
        ttk.Label(footer, textvariable=self.offset_var, style="Value.TLabel", wraplength=800).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(footer, text="Xuất JSON", command=lambda: self.export("json")).pack(side=tk.RIGHT)
        ttk.Button(footer, text="Xuất CSV", command=lambda: self.export("csv")).pack(side=tk.RIGHT, padx=4)

    def toggle_capture(self) -> None:
        if self.capture_active:
            self.finish_capture()
            return
        try:
            reference = float(self.reference_var.get())
            target = int(self.target_var.get())
        except ValueError:
            messagebox.showerror("Calibration", "Khoảng cách hoặc số mẫu không hợp lệ.", parent=self)
            return
        if reference <= 0 or target < 20:
            messagebox.showerror("Calibration", "Khoảng cách phải > 0 và cần ít nhất 20 mẫu.", parent=self)
            return
        self.capture_points.clear()
        self.rejected = 0
        self.capture_anchor_id = int(self.anchor_var.get()[1:])
        self.capture_reference_mm = reference * 1000.0
        self.capture_target = target
        self.capture_active = True
        self.start_button.configure(text="Kết thúc capture")
        self.progress.configure(value=0)
        self.live_var.set(f"Đang thu {self.anchor_var.get()} tại {reference:.3f} m…")

    def discard_capture(self) -> None:
        self.capture_active = False
        self.capture_points.clear()
        self.rejected = 0
        self.progress.configure(value=0)
        self.start_button.configure(text="Bắt đầu capture")
        self.live_var.set("Capture đã được bỏ.")

    def ingest(self, sample: AnchorSample, point: TelemetryPoint) -> None:
        if not self.capture_active or sample.anchor_id != self.capture_anchor_id:
            return
        diagnostic = bool(sample.status & (STATUS_CALIBRATION_MISSING | STATUS_RANGE_REJECT))
        if point.raw_mm <= 0 or not (point.valid or diagnostic):
            self.rejected += 1
        else:
            self.capture_points.append(point)
        target = self.capture_target
        self.progress.configure(value=min(100, len(self.capture_points)*100/target))
        if self.capture_points:
            values = [item.raw_mm for item in self.capture_points]
            mean = statistics.fmean(values)
            stddev = statistics.pstdev(values) if len(values) > 1 else 0.0
            accepted = len(values)*100/max(len(values)+self.rejected, 1)
            self.live_var.set(f"{len(values)}/{target} accepted · rejected {self.rejected} · mean {mean:.1f} mm · σ {stddev:.1f} mm · acceptance {accepted:.1f}%")
        if len(self.capture_points) >= target:
            self.finish_capture()

    def finish_capture(self) -> None:
        self.capture_active = False
        self.start_button.configure(text="Bắt đầu capture")
        if not self.capture_points:
            self.live_var.set("Không có mẫu hợp lệ trong capture.")
            return
        result = calibration_result(self.capture_anchor_id, self.capture_reference_mm, self.capture_points)
        self.results.append(result)
        accepted = result.samples*100/max(result.samples+self.rejected, 1)
        quality = "GOOD" if result.stddev_mm <= 50 and abs(result.drift_mm) <= 40 and accepted >= 75 else "CHECK"
        self.tree.insert("", tk.END, values=(
            f"A{result.anchor_id}", f"{result.reference_mm/1000:.3f} m", result.samples,
            f"{result.mean_raw_mm:.1f} mm", f"{result.offset_mm:+.1f} mm", f"{result.stddev_mm:.1f} mm",
            f"{result.p05_mm:.0f}–{result.p95_mm:.0f} mm", f"{result.drift_mm:+.1f} mm",
            _fmt(result.median_fpp_dbm, 1, " dBm"), quality,
        ), tags=("good" if quality == "GOOD" else "warn",))
        self.live_var.set(f"Đã lưu capture {self.anchor_var.get()} · {quality} · acceptance {accepted:.1f}%")
        self.capture_points = []
        self.rejected = 0
        self.progress.configure(value=0)
        self._update_offsets()

    def _update_offsets(self) -> None:
        summaries = []
        for anchor_id in range(1, MAX_ANCHORS+1):
            captures = [item for item in self.results if item.anchor_id == anchor_id]
            if not captures:
                continue
            total = sum(item.samples for item in captures)
            offset = sum(item.offset_mm*item.samples for item in captures)/total
            summaries.append(f"A{anchor_id} {offset:+.1f} mm ({len(captures)} điểm)")
        self.offset_var.set("Offset tổng hợp: " + " · ".join(summaries) if summaries else "Chưa có kết quả.")

    def export(self, file_type: str) -> None:
        if not self.results:
            messagebox.showwarning("Calibration", "Chưa có capture để xuất.", parent=self)
            return
        extension = f".{file_type}"
        filename = filedialog.asksaveasfilename(parent=self, title="Xuất calibration", defaultextension=extension, initialfile=time.strftime(f"uwb_calibration_%Y%m%d_%H%M%S{extension}"), filetypes=((file_type.upper(), f"*{extension}"),))
        if not filename:
            return
        try:
            if file_type == "json":
                payload = {"schema": 1, "created_local": time.strftime("%Y-%m-%dT%H:%M:%S"), "offset_definition": "reference_mm - mean_raw_mm", "captures": [asdict(item) for item in self.results]}
                Path(filename).write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
            else:
                with Path(filename).open("w", newline="", encoding="utf-8-sig") as handle:
                    writer = csv.DictWriter(handle, fieldnames=tuple(asdict(self.results[0])))
                    writer.writeheader()
                    writer.writerows(asdict(item) for item in self.results)
        except OSError as exc:
            messagebox.showerror("Calibration", f"Không xuất được file:\n{exc}", parent=self)
