"""Widget-state regression checks; no serial hardware or visible window needed."""
import sys
import time
import math
import queue
from dataclasses import replace
from pathlib import Path
import tkinter as tk
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uwb_uart_gui import DemoWorker, SerialWorker, UwbGui, application_directory
from telemetry_protocol import (
    AnchorSample,
    CMD_PING,
    CMD_SET_TELEMETRY,
    CmdAckMessage,
    DeviceInfoMessage,
    MEAS_FLAG_CAL_OK,
    MEAS_FLAG_FILTER_OK,
    MEAS_FLAG_RADIO_OK,
    RangeMeasMessage,
    RangeMessage,
    STATUS_CALIBRATION_MISSING,
    TYPE_CMD,
    TYPE_DEVICE_INFO,
    TelemetryStreamParser,
    decode_frame,
    encode_frame,
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


class CaptureSerial:
    def __init__(self):
        self.writes = []

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def frames(self):
        parser = TelemetryStreamParser()
        return [frame for data in self.writes for frame in parser.feed(data)]


class FakeWorker:
    """Stands in for SerialWorker when the GUI asks for a command."""

    def __init__(self):
        self.requests = []

    def request_command(self, command_id, arguments=b""):
        self.requests.append((command_id, bytes(arguments)))

    def stop(self):
        pass


def demo_device_info(**fields) -> DeviceInfoMessage:
    frame = encode_frame(TYPE_DEVICE_INFO, 0, 0, DemoWorker._device_info_payload(0x07))
    return replace(decode_frame(TelemetryStreamParser().feed(frame)[0]), **fields)


MEAS_A2 = RangeMeasMessage(
    sequence=0, time_ms=0, boot_id=0x1234, meas_seq=0, meas_time_us=0, anchor_id=2, txn=0,
    mode=0, flags=MEAS_FLAG_RADIO_OK | MEAS_FLAG_CAL_OK | MEAS_FLAG_FILTER_OK, status=0,
    raw_mm=4210, corrected_mm=4100, filtered_mm=4095, fp_cdbm=-8100, rx_cdbm=-7300,
    anchor_fp_cdbm=-8200, anchor_rx_cdbm=-7900, std_noise=20, fp_index=0x2000,
    ci_ppm_x100=150, slot_us=3200,
)


class GuiStateTests(unittest.TestCase):
    @staticmethod
    def close_app(root):
        for callback in root.tk.call('after', 'info'):
            root.after_cancel(callback)
        root.destroy()

    def test_serial_worker_enables_range_meas_only_on_a_fast_uart(self):
        # (auto enable, TAG UART baud, TAG features) -> features the GUI requests
        cases = (
            (True, 1_000_000, 0x05, 0x07),
            (True, 1_000_000, 0x07, None),
            (True, 460_800, 0x04, 0x07),
            (True, 115_200, 0x05, None),
            (True, 115_200, 0x04, 0x05),
            (False, 1_000_000, 0x05, None),
            (False, 1_000_000, 0x04, 0x05),
        )
        for auto, baud, features, expected in cases:
            with self.subTest(auto=auto, baud=baud, features=features):
                events = queue.Queue()
                worker = SerialWorker("COM_TEST", 1_000_000, events, auto_enable_meas=auto)
                capture = CaptureSerial()
                worker._serial = capture
                worker._configure_session_telemetry(
                    demo_device_info(telemetry_features=features, uart_baud=baud))
                frames = capture.frames()
                if expected is None:
                    self.assertEqual(frames, [])
                    self.assertTrue(events.empty())
                else:
                    self.assertEqual([frame.payload for frame in frames],
                                     [bytes((CMD_SET_TELEMETRY, expected))])
                    notices = [events.get_nowait()[1] for _ in range(events.qsize())]
                    self.assertEqual(any("RANGE_MEAS" in text for text in notices),
                                     bool(expected & ~features & 0x02))

    def test_serial_worker_sends_gui_requests_in_order(self):
        worker = SerialWorker("COM_TEST", 1_000_000, queue.Queue())
        capture = CaptureSerial()
        worker._serial = capture
        worker.request_command(CMD_SET_TELEMETRY, bytes((0x07,)))
        worker.request_command(CMD_PING)
        self.assertEqual(capture.writes, [])          # only the reader thread writes
        worker._send_requested_commands()
        frames = capture.frames()
        self.assertEqual([frame.payload for frame in frames],
                         [bytes((CMD_SET_TELEMETRY, 0x07)), bytes((CMD_PING,))])
        self.assertEqual((frames[1].sequence - frames[0].sequence) & 0xFFFFFFFF, 1)

    def test_link_hint_names_both_baud_rates(self):
        worker = SerialWorker("COM_TEST", 115200, queue.Queue())
        garbage = worker._link_hint(500)
        self.assertIn("115200 baud", garbage)
        self.assertIn("Tag_DevKit chạy 1000000", garbage)
        self.assertIn("cổng COM", worker._link_hint(0))

    def test_demo_answers_set_telemetry(self):
        events = queue.Queue()
        demo = DemoWorker(events)
        self.assertEqual(demo_device_info().uart_baud, 1_000_000)
        demo.request_command(CMD_SET_TELEMETRY, bytes((0x05,)))
        demo._answer_commands(TelemetryStreamParser(), 0)
        self.assertEqual(demo.features, 0x05)
        messages = [payload for event, payload in
                    (events.get_nowait() for _ in range(events.qsize())) if event == "message"]
        self.assertEqual(len(messages), 1)
        self.assertIsInstance(messages[0], CmdAckMessage)
        self.assertTrue(messages[0].ok)
        self.assertEqual(messages[0].data, bytes((0x05,)))

    def test_demo_stream_carries_range_meas(self):
        events = queue.Queue()
        demo = DemoWorker(events)
        demo.start()
        time.sleep(0.5)
        demo.stop()
        demo.join(timeout=2.0)
        self.assertFalse(demo.is_alive())
        items = [events.get_nowait() for _ in range(events.qsize())]
        self.assertFalse([payload for event, payload in items if event == "protocol_error"])
        messages = [payload for event, payload in items if event == "message"]
        info = [item for item in messages if isinstance(item, DeviceInfoMessage)]
        self.assertEqual((info[0].telemetry_features, info[0].uart_baud), (0x07, 1_000_000))
        meas = [item for item in messages if isinstance(item, RangeMeasMessage)]
        self.assertEqual({item.anchor_id for item in meas}, {1, 2, 3, 4})
        self.assertEqual([item.meas_seq for item in meas], list(range(1, len(meas) + 1)))

    def test_range_meas_tab_tracks_records_and_toggles_telemetry(self):
        root = tk.Tk()
        root.withdraw()
        try:
            app = UwbGui(root)
            worker = FakeWorker()
            app.worker = worker
            panel = app.meas_panel
            self.assertIsNone(panel.meas_enabled)

            app.events.put(("message", demo_device_info(telemetry_features=0x05)))
            app._poll_events_once()
            self.assertIs(panel.meas_enabled, False)
            self.assertIn("RANGE_MEAS TẮT", panel.state_var.get())
            self.assertEqual(panel.toggle_button.cget("text"), "Bật RANGE_MEAS")

            panel.toggle_button.invoke()
            self.assertEqual(worker.requests, [(CMD_SET_TELEMETRY, bytes((0x07,)))])
            app.events.put(("message", CmdAckMessage(9, 0, CMD_SET_TELEMETRY, 0, bytes((0x07,)))))
            for seq in [*range(1, 30), *range(31, 62)]:          # record 30 is lost
                app.events.put(("message", replace(
                    MEAS_A2, meas_seq=seq, meas_time_us=seq * 20_000,
                    raw_mm=4210 + (3 if seq % 2 else -3))))
            app._poll_events_once()
            self.assertIs(panel.meas_enabled, True)
            self.assertEqual(panel.toggle_button.cget("text"), "Tắt RANGE_MEAS")

            panel.refresh(time.monotonic(), draw=True)
            values = panel.tree.item(panel.rows[2], "values")
            self.assertEqual(values[:4], ("A2", "—", "DS", "OK"))  # rate needs 0.5 s
            self.assertEqual(values[4:7], ("4.213", "4.100", "4.095"))
            self.assertEqual(values[10:13], ("8.0", "3.0", "+1.50"))
            self.assertEqual(panel.tree.item(panel.rows[2], "tags"), ("warn",))  # 8 dB
            self.assertEqual(panel.tree.item(panel.rows[1], "values")[3], "NO DATA")
            self.assertEqual(panel.tracker.lost, 1)
            self.assertIn("mất 1", panel.summary_var.get())
            self.assertIn("queue drop TAG —", panel.summary_var.get())
            panel.set_tag_counters(3, 0, 512)
            panel.refresh(time.monotonic(), draw=False)
            self.assertIn("queue drop TAG 3", panel.summary_var.get())
            self.assertIn("đỉnh TX 512 B (50 %)", panel.summary_var.get())

            panel.plot_anchor_var.set("A2")
            panel.draw()
            for tag in ("meas_raw", "meas_corrected", "meas_filtered", "meas_nlos"):
                self.assertTrue(panel.canvas.find_withtag(tag), tag)

            panel.toggle_button.invoke()                           # snapshot + diag stay on
            self.assertEqual(worker.requests[-1], (CMD_SET_TELEMETRY, bytes((0x05,))))
            panel.set_features(0x02)          # RANGE_MEAS alone: the Live tab would starve
            panel.toggle_button.invoke()
            self.assertEqual(worker.requests[-1], (CMD_SET_TELEMETRY, bytes((0x01,))))

            panel.set_device_info(0x05, 115200)
            self.assertEqual(str(panel.toggle_button["state"]), "disabled")
            self.assertIn("115200", panel.state_var.get())
        finally:
            self.close_app(root)

    def test_frozen_application_directory_is_executable_parent(self):
        executable = Path("D:/Demo/UWB/DWM1001_UWB_Ground_Control.exe")
        with patch("uwb_uart_gui.sys.frozen", True, create=True), patch(
            "uwb_uart_gui.sys.executable", str(executable)
        ):
            self.assertEqual(application_directory(), executable.parent)

    def test_serial_worker_encodes_bidirectional_ping(self):
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
            self.assertEqual(len(app.notebook.tabs()), 6)
            tab_labels = [app.notebook.tab(tab_id, "text").strip() for tab_id in app.notebook.tabs()]
            self.assertEqual(
                tab_labels,
                ["Live", "Biểu đồ lớn", "RANGE_MEAS", "Bản đồ 2D / 3D", "Phân tích", "Calibration"],
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
