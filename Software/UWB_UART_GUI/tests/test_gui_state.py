"""Widget-state regression checks; no serial hardware or visible window needed."""
import sys
import time
import math
import queue
from pathlib import Path
import tkinter as tk
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uwb_uart_gui import SerialWorker, UwbGui, application_directory
from telemetry_protocol import (
    AnchorSample,
    CMD_PING,
    RangeMessage,
    STATUS_CALIBRATION_MISSING,
    TYPE_CMD,
    TelemetryStreamParser,
)
from gui_analysis import (
    DEFAULT_LAYOUT_4,
    TelemetryPoint,
    overlapping_allan_deviation,
    regularize_time_series,
    signal_statistics,
    welch_psd,
)
from gui_views import AnalysisPanel


class GuiStateTests(unittest.TestCase):
    @staticmethod
    def close_app(root):
        for callback in root.tk.call('after', 'info'):
            root.after_cancel(callback)
        root.destroy()

    def test_frozen_application_directory_is_executable_parent(self):
        executable = Path("D:/Demo/UWB/DWM1001_UWB_Ground_Control.exe")
        with patch("uwb_uart_gui.sys.frozen", True, create=True), patch(
            "uwb_uart_gui.sys.executable", str(executable)
        ):
            self.assertEqual(application_directory(), executable.parent)

    def test_serial_worker_encodes_bidirectional_ping(self):
        class CaptureSerial:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(data)
                return len(data)

        worker = SerialWorker("COM_TEST", 115200, queue.Queue())
        capture = CaptureSerial()
        worker._serial = capture
        worker._command_sequence = 40

        self.assertEqual(worker._send_command(CMD_PING), 41)
        frame = TelemetryStreamParser().feed(capture.writes[0])[0]
        self.assertEqual(frame.type, TYPE_CMD)
        self.assertEqual(frame.sequence, 41)
        self.assertEqual(frame.payload, bytes((CMD_PING,)))

    def test_stale_and_reboot(self):
        root = tk.Tk()
        root.withdraw()
        try:
            app = UwbGui(root)
            sample = AnchorSample(1, True, 0, 0, 1234, 1230, -8000)
            app._record_history(RangeMessage(100, 2000, (sample,)))
            app._update_range(RangeMessage(100, 2000, (sample,)))
            item = app.anchor_items[1]
            app._mark_stale()
            self.assertEqual(app.tree.item(item, 'values')[1:3], ('NO', 'STALE'))
            app._record_history(RangeMessage(1, 20, (sample,)))
            self.assertFalse(app.data_stale)
            self.assertEqual(len(app.history[1]), 1)
            self.assertEqual(app.sequence_loss, 0)
            app._update_range(RangeMessage(1, 20, (sample,)))
            self.assertEqual(app.tree.item(item, 'values')[1], 'YES')
        finally:
            self.close_app(root)

    def test_graph_contains_raw_and_filtered_series(self):
        root = tk.Tk()
        root.withdraw()
        try:
            app = UwbGui(root)
            app._record_history(RangeMessage(
                1, 20, (AnchorSample(1, True, 0, 0, 1200, 1180, -8000),)
            ))
            app._record_history(RangeMessage(
                2, 40, (AnchorSample(1, True, 0, 0, 1250, 1200, -8000),)
            ))
            app._draw_graph()
            self.assertTrue(app.graph.find_withtag("raw_series"))
            self.assertTrue(app.graph.find_withtag("host_series"))
            self.assertTrue(app.graph.find_withtag("firmware_series"))
            app._clear_graph_history()
            self.assertEqual(len(app.history[1]), 0)
        finally:
            self.close_app(root)

    def test_secondary_telemetry_panel_starts_collapsed(self):
        root = tk.Tk()
        root.withdraw()
        try:
            app = UwbGui(root)
            self.assertEqual(app.telemetry_details.winfo_manager(), "")
            self.assertEqual(app.record_button.winfo_manager(), "pack")
            self.assertEqual(app.details_button.cget("text"), "Hiện chi tiết ▾")

            app._toggle_telemetry_details()
            self.assertEqual(app.telemetry_details.winfo_manager(), "pack")
            self.assertEqual(app.details_button.cget("text"), "Ẩn chi tiết ▴")

            app._toggle_telemetry_details()
            self.assertEqual(app.telemetry_details.winfo_manager(), "")
        finally:
            self.close_app(root)

    def test_host_filter_accepts_uncalibrated_diagnostic_raw(self):
        root = tk.Tk()
        root.withdraw()
        try:
            app = UwbGui(root)
            sample = AnchorSample(
                1, False, STATUS_CALIBRATION_MISSING, 0xFFFF, 1270, 0, -9607
            )
            filtered = app._record_history(RangeMessage(1, 20, (sample,)))
            self.assertEqual(filtered[1], 1270)
            raw, firmware, host = app.history[1][-1]
            self.assertEqual(raw, 1270)
            self.assertIsNone(firmware)
            self.assertEqual(host, 1270)
        finally:
            self.close_app(root)

    def test_advanced_tabs_and_2d_solution(self):
        root = tk.Tk()
        root.withdraw()
        try:
            app = UwbGui(root)
            self.assertEqual(len(app.notebook.tabs()), 5)
            tab_labels = [app.notebook.tab(tab_id, "text").strip() for tab_id in app.notebook.tabs()]
            self.assertEqual(
                tab_labels,
                ["Live", "Biểu đồ lớn", "Bản đồ 2D / 3D", "Phân tích", "Calibration"],
            )
            app.map_panel.layout = DEFAULT_LAYOUT_4
            target = (2.0, 1.5, 0.0)
            samples = {}
            host = {}
            for anchor in DEFAULT_LAYOUT_4:
                distance = round(math.dist(target, (anchor.x, anchor.y, anchor.z)) * 1000)
                samples[anchor.anchor_id] = AnchorSample(anchor.anchor_id, True, 0, 5, distance, distance, -7800)
                host[anchor.anchor_id] = distance
            app.map_panel.update_data(samples, host)
            self.assertIsNotNone(app.map_panel.solution)
            self.assertAlmostEqual(app.map_panel.solution.x, 2.0, places=3)
            self.assertAlmostEqual(app.map_panel.solution.y, 1.5, places=3)
        finally:
            self.close_app(root)

    def test_analysis_and_calibration_receive_live_history(self):
        root = tk.Tk()
        root.withdraw()
        try:
            app = UwbGui(root)
            app.calibration_panel.target_var.set("20")
            app.calibration_panel.reference_var.set("1.100")
            app.calibration_panel.toggle_capture()
            for sequence in range(1, 21):
                sample = AnchorSample(1, False, STATUS_CALIBRATION_MISSING, 0, 1000 + sequence, 0, -7600)
                app._record_history(RangeMessage(sequence, sequence * 20, (sample,)))
            self.assertEqual(len(app.calibration_panel.results), 1)
            self.assertFalse(app.calibration_panel.capture_active)
            app.analysis_panel.refresh(app.detailed_history, app.frame_times)
            a1 = app.analysis_panel.latest_metrics[0]
            self.assertEqual(a1.observed, 20)
            self.assertEqual(a1.invalid, 20)
            self.assertEqual(a1.missing, 0)
        finally:
            self.close_app(root)

    def test_analysis_waits_for_explicit_batch_capture(self):
        root = tk.Tk()
        root.withdraw()
        try:
            app = UwbGui(root)
            for sequence in range(1, 21):
                sample = AnchorSample(1, True, 0, 5, 2000 + sequence % 5, 2000, -7800)
                app._record_history(RangeMessage(sequence, sequence * 20, (sample,)))
            self.assertIsNone(app.analysis_panel.signal_stats)

            app.analysis_panel.target_var.set("100")
            app.analysis_panel.start_capture()
            self.assertTrue(app.analysis_panel.capture_active)
            for sequence in range(21, 120):
                sample = AnchorSample(1, True, 0, 5, 2000 + sequence % 7, 2001, -7800)
                app._record_history(RangeMessage(sequence, sequence * 20, (sample,)))
            self.assertTrue(app.analysis_panel.capture_active)
            self.assertIsNone(app.analysis_panel.signal_stats)

            sequence = 120
            sample = AnchorSample(1, True, 0, 5, 2000 + sequence % 7, 2001, -7800)
            app._record_history(RangeMessage(sequence, sequence * 20, (sample,)))
            self.assertFalse(app.analysis_panel.capture_active)
            self.assertIsNotNone(app.analysis_panel.signal_stats)
            self.assertEqual(app.analysis_panel.signal_stats.sample_count, 100)
            self.assertEqual(str(app.analysis_panel.save_plot_button["state"]), "normal")
            self.assertIn("Đã đủ mẫu", app.analysis_panel.capture_status_var.get())
        finally:
            self.close_app(root)

    def test_scientific_dashboard_recomputes_for_anchor_and_source(self):
        root = tk.Tk()
        root.withdraw()
        try:
            app = UwbGui(root)
            for sequence in range(1, 129):
                samples = (
                    AnchorSample(1, True, 0, 5, 1000 + sequence % 9, 998 + sequence % 5, -7800),
                    AnchorSample(2, True, 0, 5, 2000 + sequence % 13, 1997 + sequence % 7, -8000),
                )
                app._record_history(RangeMessage(sequence, sequence * 20, samples))
            app.analysis_panel.refresh(app.detailed_history, app.frame_times)

            self.assertTrue(app.analysis_panel.time_canvas.winfo_exists())
            self.assertTrue(app.analysis_panel.hist_canvas.winfo_exists())
            self.assertTrue(app.analysis_panel.allan_canvas.winfo_exists())
            self.assertTrue(app.analysis_panel.psd_canvas.winfo_exists())
            self.assertIsNotNone(app.analysis_panel.signal_stats)
            self.assertLess(app.analysis_panel.signal_stats.mean_mm, 1100)

            app.analysis_panel.anchor_var.set("A2")
            app.analysis_panel.source_var.set("Raw")
            app.analysis_panel._controls_changed()
            self.assertIsNotNone(app.analysis_panel.signal_stats)
            self.assertGreater(app.analysis_panel.signal_stats.mean_mm, 1900)
            self.assertTrue(app.analysis_panel._allan[0])
            self.assertTrue(app.analysis_panel._psd["Raw"][0])
        finally:
            self.close_app(root)

    def test_analysis_tag_clock_handles_uint32_wrap_and_range_age(self):
        points = (
            TelemetryPoint(10.00, 0xFFFFFFF5, True, 0, 5, 1000, 1000, 1000, -78.0),
            TelemetryPoint(10.02, 0x00000009, True, 0, 5, 1001, 1001, 1001, -78.0),
            TelemetryPoint(10.04, 0x0000001D, True, 0, 5, 1002, 1002, 1002, -78.0),
        )
        unwrapped = AnalysisPanel._unwrap_times(points)
        self.assertEqual(len(unwrapped), 3)
        self.assertAlmostEqual(unwrapped[1] - unwrapped[0], 0.020, places=9)
        self.assertAlmostEqual(unwrapped[2] - unwrapped[1], 0.020, places=9)

    def test_diagnostic_clock_uses_frame_time_instead_of_stale_age(self):
        points = tuple(
            TelemetryPoint(
                10.0,
                index * 20,
                False,
                STATUS_CALIBRATION_MISSING,
                index * 20,
                1000 + index % 11,
                None,
                1000 + index % 7,
                -78.0,
            )
            for index in range(1, 129)
        )
        times = AnalysisPanel._unwrap_times(points)
        self.assertAlmostEqual(times[1] - times[0], 0.020, places=9)
        stats = signal_statistics(times, [point.host_mm for point in points])
        self.assertIsNotNone(stats)
        self.assertAlmostEqual(stats.sample_rate_hz, 50.0, places=6)
        regular = regularize_time_series(times, [point.host_mm for point in points])
        self.assertIsNotNone(regular)
        self.assertAlmostEqual(regular.sample_rate_hz, 50.0, places=6)
        self.assertGreater(len(overlapping_allan_deviation(regular.values, regular.sample_rate_hz)[0]), 1)
        self.assertTrue(welch_psd(regular.values, regular.sample_rate_hz)[0])

        duplicate = (
            TelemetryPoint(20.000000, 1000, False, STATUS_CALIBRATION_MISSING, 1000, 1, None, 1, -78.0),
            TelemetryPoint(20.000001, 1000, False, STATUS_CALIBRATION_MISSING, 1000, 2, None, 2, -78.0),
        )
        duplicate_times = AnalysisPanel._unwrap_times(duplicate)
        self.assertEqual(duplicate_times[0], duplicate_times[1])

        mixed = tuple(
            TelemetryPoint(
                30.0 + index * 0.000001,
                2000 + index * 20,
                index % 2 == 0,
                0 if index % 2 == 0 else STATUS_CALIBRATION_MISSING,
                5 if index % 2 == 0 else 2000 + index * 20,
                900 + index % 5,
                900 if index % 2 == 0 else None,
                900 + index % 3,
                -79.0,
            )
            for index in range(560)
        )
        mixed_times = AnalysisPanel._unwrap_times(mixed)
        mixed_stats = signal_statistics(mixed_times, [point.host_mm for point in mixed])
        self.assertIsNotNone(mixed_stats)
        self.assertAlmostEqual(mixed_stats.sample_rate_hz, 50.0, places=6)

    def test_singleton_log_axes_and_histogram_zero_baseline_are_safe(self):
        root = tk.Tk()
        root.withdraw()
        try:
            panel = AnalysisPanel(root)
            log_axes = panel._axes(
                panel.allan_canvas,
                "singleton",
                [0.02],
                [0.5],
                "tau",
                "adev",
                log_x=True,
                log_y=True,
            )
            self.assertIsNotNone(log_axes)
            self.assertGreater(log_axes[2][0], 0.0)
            self.assertGreater(log_axes[2][2], 0.0)

            linear_axes = panel._axes(
                panel.hist_canvas,
                "histogram",
                [-1.0, 1.0],
                [0.0, 1.0],
                "error",
                "density",
                zero_y_baseline=True,
            )
            transform, bounds, limits = linear_axes
            self.assertEqual(limits[2], 0.0)
            self.assertAlmostEqual(transform(0.0, 0.0)[1], bounds[3])
        finally:
            self.close_app(root)

    def test_polling_reschedules_after_view_exception(self):
        root = tk.Tk()
        root.withdraw()
        try:
            app = UwbGui(root)
            with patch.object(app, "_poll_events_once", side_effect=ValueError("plot failed")), patch.object(
                root, "after"
            ) as schedule:
                app._poll_events()
                schedule.assert_called_once()
            self.assertIn("ValueError: plot failed", app._last_ui_error)
        finally:
            self.close_app(root)
