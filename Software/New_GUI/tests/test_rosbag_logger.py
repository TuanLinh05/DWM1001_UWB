"""Real MCAP/CDR round trips, split status handling and recording lifecycle."""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT.parent / "UWB_UART_GUI")]

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from PyQt6.QtWidgets import QApplication
import telemetry_protocol as tp
from core_state import UwbStateEngine
from rosbag_logger import Ros2BagRecorder
from serial_worker import FastCsvLogger

APP = QApplication.instance() or QApplication([])


def read_ros2_messages(filename):
    from types import SimpleNamespace
    with open(filename, "rb") as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        for _schema, channel, message, decoded in reader.iter_decoded_messages():
            yield SimpleNamespace(channel=channel, log_time_ns=message.log_time, ros_msg=decoded)


def burst(cycle=1):
    flags = tp.BURST_FLAG_RADIO_OK | tp.BURST_FLAG_CAL_OK | tp.BURST_FLAG_VALID
    return tp.RangeBurstMessage(cycle, cycle * 10, 7, cycle, cycle * 8, cycle * 10000, 10000, (
        tp.BurstAnchor(1, 0, flags, 2500, -8000, -7500, 0),
        tp.BurstAnchor(2, 0, tp.BURST_FLAG_RADIO_OK, 3000, tp.INT16_MIN, tp.INT16_MIN, 0),
        tp.BurstAnchor(3, 1, 0, None, tp.INT16_MIN, tp.INT16_MIN, 0)))


class Ros2BagTests(unittest.TestCase):
    def test_cdr_roundtrip_and_metadata_match_every_message(self):
        from core_state import DopMetrics, PositionResult
        import yaml
        with tempfile.TemporaryDirectory() as temp:
            bag = Ros2BagRecorder(Path(temp))
            sol = PositionResult(1., 2., 3., .01, .02, {1: .01}, (1, 2, 3, 4), (), 3,
                                 DopMetrics(1., 1., 1., 1., 1., False), .01)
            t0 = time.monotonic()
            for i in range(1, 301):
                bag.log_telemetry(burst(i), t0 + i / 300, sol)
            bag.log_metadata({"source": "TEST", "simulated": True})
            bag.stop()
            messages = list(read_ros2_messages(str(bag.mcap_path)))
            self.assertEqual(len(messages), bag.messages_written)
            self.assertEqual(bag.dropped_events, 0)
            self.assertTrue(bag.closed)
            poses = [m for m in messages if m.channel.topic == "/uwb/position"]
            self.assertEqual(len(poses), 300)
            self.assertEqual(poses[0].ros_msg.point.y, 2.)
            self.assertEqual(poses[0].ros_msg.header.frame_id, "uwb_map")
            self.assertEqual(poses[0].ros_msg.header.stamp.sec, poses[0].log_time_ns // 10**9)
            ranges = [m for m in messages if m.channel.topic == "/uwb/anchor_1/range_m"]
            self.assertEqual(ranges[0].ros_msg.data, 2.5)
            topics = {m.channel.topic for m in messages}
            self.assertIn("/uwb/anchor_2/diagnostic_range_m", topics)
            self.assertNotIn("/uwb/anchor_2/range_m", topics)
            self.assertNotIn("/uwb/anchor_3/diagnostic_range_m", topics)
            info = yaml.safe_load((bag.bag_dir / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
            self.assertEqual(info["message_count"], len(messages))
            self.assertEqual(sum(t["message_count"] for t in info["topics_with_message_count"]), len(messages))
            self.assertEqual(info["starting_time"]["nanoseconds_since_epoch"], min(m.log_time_ns for m in messages))
            self.assertFalse(bag.log_telemetry(burst(), time.monotonic()))

    def test_split_px4_preserves_fields_without_repeating_stale_snapshots(self):
        def part(seq, path, data):
            return {"kind": "up4000_status", "seq": seq, "time_unix": seq * .25,
                    "part": path, "data": data}
        with tempfile.TemporaryDirectory() as temp:
            bag = Ros2BagRecorder(Path(temp))
            packets = [
                part(1, "px4.messages.ATTITUDE", {"age_s": .01}),
                part(1, "px4.messages.ATTITUDE.fields", {"time_boot_ms": 10, "roll": .3}),
                part(1, "px4.messages.ATTITUDE.fields", {"pitch": .2}),
                part(1, "px4.messages.ODOMETRY", {"age_s": .01, "fields": {"q": [1., 0., 0., 0.]}}),
                part(2, "px4.messages.ATTITUDE", {"age_s": .1, "fields": {"time_boot_ms": 10, "roll": .3, "pitch": .2}}),
                part(3, "px4.messages.ATTITUDE", {"age_s": 3, "fields": {"roll": 88.}}),
                part(4, "px4.messages.ATTITUDE.fields", {"roll": 99.}),  # age packet lost
            ]
            for p in packets:
                bag.log_status(p, "100.123.216.14")
                p["data"].clear()  # prove queued dictionaries are independent
            bag.stop()
            messages = list(read_ros2_messages(str(bag.mcap_path)))
            rolls = [m.ros_msg.data for m in messages if m.channel.topic == "/px4/attitude/roll"]
            self.assertEqual(rolls, [.3])
            q = next(m.ros_msg.data for m in messages if m.channel.topic == "/px4/odometry/q")
            self.assertEqual(list(q), [1., 0., 0., 0.])
            raw = [m for m in messages if m.channel.topic == "/up7000/status_parts"]
            self.assertEqual(len(raw), len(packets))
            self.assertEqual(json.loads(raw[0].ros_msg.data)["sender"], "100.123.216.14")

    def test_disk_failure_is_reported_and_never_marked_complete(self):
        with tempfile.TemporaryDirectory() as temp:
            bag = Ros2BagRecorder(Path(temp))
            with patch.object(bag, "_telemetry", side_effect=OSError("disk full")):
                bag.log_telemetry(burst(), time.monotonic())
                with self.assertRaisesRegex(RuntimeError, "disk full"):
                    bag.stop()
            self.assertFalse(bag.closed)
            status = json.loads((bag.bag_dir / "recording_status.json").read_text())
            self.assertIn("disk full", status["error"])

    def test_queue_overflow_is_counted_without_blocking(self):
        import threading
        with tempfile.TemporaryDirectory() as temp:
            bag = Ros2BagRecorder(Path(temp), queue_size=1)
            entered, release = threading.Event(), threading.Event()
            original = bag._telemetry
            def wait_then_write(*args):
                entered.set()
                release.wait(3)
                original(*args)
            with patch.object(bag, "_telemetry", side_effect=wait_then_write):
                bag.log_telemetry(burst(), time.monotonic())
                self.assertTrue(entered.wait(2))
                self.assertTrue(bag.log_telemetry(burst(2), time.monotonic()))
                self.assertFalse(bag.log_telemetry(burst(3), time.monotonic()))
                release.set()
                bag.stop()
            self.assertEqual(bag.dropped_events, 1)
            self.assertEqual(bag.accepted_events, 2)

    def test_rapid_sessions_do_not_overwrite_csv_or_bags(self):
        with tempfile.TemporaryDirectory() as temp, patch("serial_worker.time.strftime", return_value="SAME"):
            a, b = FastCsvLogger(Path(temp), "UDP"), FastCsvLogger(Path(temp), "UDP")
            a.stop()
            b.stop()
            self.assertNotEqual(a.session_dir, b.session_dir)

    def test_gui_records_status_and_uwb_with_same_log_button(self):
        from main_gui import UwbFastGuiWindow
        from up4000_panel import Up4000StatusListener
        with tempfile.TemporaryDirectory() as temp, \
                patch("main_gui.NEW_GUI_DIR", Path(temp)), \
                patch("main_gui.Up4000StatusListener", side_effect=lambda parent: Up4000StatusListener(port=0, parent=parent)):
            win = UwbFastGuiWindow(start_demo=False, enable_onboard_control=False)
            try:
                self.assertTrue(win.log_format_combo.currentData())
                win.btn_csv_log.click()
                bag = win.rosbag_logger
                self.assertIsNotNone(bag)
                self.assertFalse(win.log_format_combo.isEnabled())
                win._on_telemetry_batch([(burst(), time.monotonic())])
                packet = {"kind": "up4000_status", "version": 1, "seq": 1, "px4": {"messages": {
                    "ATTITUDE": {"age_s": .01, "fields": {"roll": .6}}}}}
                win.up4000_listener.handle_datagram(json.dumps(packet).encode(), "TEST")
                win.btn_csv_log.click()
                self.assertIsNone(win.rosbag_logger)
                self.assertTrue(win.log_format_combo.isEnabled())
                topics = {m.channel.topic for m in read_ros2_messages(str(bag.mcap_path))}
                self.assertIn("/uwb/cycle_seq", topics)
                self.assertIn("/px4/attitude/roll", topics)
                # CSV-only remains available and does not create a misleading bag.
                win.log_format_combo.setCurrentIndex(1)
                win.btn_csv_log.click()
                self.assertIsNone(win.rosbag_logger)
                win.btn_csv_log.click()
            finally:
                win.close()

    def test_gui_demo_bag_excludes_live_px4_and_finishes_on_close(self):
        from main_gui import UwbFastGuiWindow
        from up4000_panel import Up4000StatusListener
        with tempfile.TemporaryDirectory() as temp, \
                patch("main_gui.NEW_GUI_DIR", Path(temp)), \
                patch("main_gui.Up4000StatusListener", side_effect=lambda parent: Up4000StatusListener(port=0, parent=parent)):
            win = UwbFastGuiWindow(start_demo=True)
            win.btn_csv_log.click()
            bag = win.rosbag_logger
            try:
                self.assertFalse(win.btn_demo.isEnabled())
                self.assertFalse(win.btn_connect.isEnabled())
                win._on_telemetry_batch([(burst(), time.monotonic())])
                packet = {"kind": "up4000_status", "version": 1, "seq": 1, "px4": {"messages": {
                    "ATTITUDE": {"age_s": .01, "fields": {"roll": .6}}}}}
                win.up4000_listener.handle_datagram(json.dumps(packet).encode(), "REAL_PX4")
            finally:
                win.close()
            self.assertTrue(bag.closed)
            messages = list(read_ros2_messages(str(bag.mcap_path)))
            self.assertFalse(any(m.channel.topic.startswith(("/px4/", "/up7000/")) for m in messages))
            metadata = next(m for m in messages if m.channel.topic == "/session/metadata")
            self.assertTrue(json.loads(metadata.ros_msg.data)["simulated"])


if __name__ == "__main__":
    unittest.main()
