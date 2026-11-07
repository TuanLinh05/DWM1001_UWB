"""WiFi relay input: New_GUI UdpTelemetryWorker with the real UP4000 relay core."""
from __future__ import annotations

import csv
import math
import os
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
NEW_GUI_DIR = Path(__file__).resolve().parents[1]
REPO = NEW_GUI_DIR.parents[1]
sys.path.insert(0, str(NEW_GUI_DIR))
sys.path.insert(0, str(NEW_GUI_DIR.parent / "UWB_UART_GUI"))
sys.path.insert(0, str(REPO / "UP4000" / "ros2_ws" / "src" / "uwb_px4_logger"))

from PyQt6.QtWidgets import QApplication  # noqa: E402
import telemetry_protocol as tp  # noqa: E402
from core_state import AnchorConfig, save_anchor_layout_json  # noqa: E402
from serial_worker import UdpTelemetryWorker  # noqa: E402
from uwb_px4_logger.session import FrameRecordWriter  # noqa: E402
from uwb_px4_logger.uwb_relay import UwbRelay  # noqa: E402

_APP = QApplication.instance() or QApplication([])
ANCHORS = [AnchorConfig(1, 0, 0, .48), AnchorConfig(2, 5.4, 0, .48), AnchorConfig(3, 5.4, 3.6, .48),
           AnchorConfig(4, 0, 3.6, .48), AnchorConfig(5, 0, 0, 2.208), AnchorConfig(6, 5.4, 0, 2.208),
           AnchorConfig(7, 5.4, 3.6, 2.208), AnchorConfig(8, 0, 3.6, 2.208)]


def burst_frame(cycle: int, xyz=(2.0, 1.5, 1.0)) -> bytes:
    anchors = tuple(
        tp.BurstAnchor(a.anchor_id, 0, tp.BURST_FLAG_RADIO_OK | tp.BURST_FLAG_CAL_OK | tp.BURST_FLAG_VALID,
                       round(math.dist(xyz, (a.x, a.y, a.z)) * 1000), -8400, -8000, -250)
        for a in ANCHORS)
    msg = tp.RangeBurstMessage(cycle, cycle * 3, 77, cycle, cycle * 8, cycle * 3300, 3300, anchors)
    return tp.encode_frame(tp.TYPE_RANGE_BURST, cycle, cycle * 3, tp.encode_range_burst_payload(msg))


def device_info_frame() -> bytes:
    payload = struct.pack("<BBHIBBBBIHHIBBHHIBBIIIBBBBBBBIBB",
                          1, 1, 0, 0x2D19AC75, 0, 3, 0x07, 1, 0x1234, 77, 1, 0,
                          0, 0, 16436, 16436, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 460800, 0xFF, 0xFF)
    return tp.encode_frame(tp.TYPE_DEVICE_INFO, 1, 10, payload + struct.pack("<II", 0xB5F8839D, 0))


class FakeSerial:
    def __init__(self):
        self.chunks: list[bytes] = []
        self.writes: list[bytes] = []
        self.lock = threading.Lock()

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.chunks[0]) if self.chunks else 0

    def read(self, n):
        with self.lock:
            if self.chunks:
                return self.chunks.pop(0)
        time.sleep(0.001)
        return b""

    def write(self, data):
        with self.lock:
            self.writes.append(bytes(data))

    def close(self):
        pass


def pump_until(cond, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        _APP.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    return False


class UdpRelayInputTests(unittest.TestCase):
    def test_gui_discovers_relay_receives_bursts_and_sends_commands(self):
        worker = UdpTelemetryWorker(listen_port=0)
        received, states = [], []
        worker.batch_ready.connect(lambda batch: received.extend(m for m, _ in batch))
        worker.link_status.connect(lambda s, d: states.append(s))
        worker.start()
        self.assertTrue(pump_until(lambda: worker.bound_port is not None))
        serial = FakeSerial()
        with tempfile.TemporaryDirectory() as tmp:
            relay = UwbRelay(port_resolver=lambda: "FAKE", out_dir=Path(tmp), ground_port=worker.bound_port,
                             local_port=0, beacon_host="127.0.0.1", serial_factory=lambda p, b: serial,
                             log=lambda s: None)
            relay.open_socket()
            stop = threading.Event()
            runner = threading.Thread(target=relay.run, args=(stop,), daemon=True)
            serial.chunks.append(device_info_frame())
            runner.start()
            try:
                # Beacon -> GUI learns the relay and PINGs it -> relay unicasts the stream.
                self.assertTrue(pump_until(lambda: relay.ground_addr is not None))
                with serial.lock:
                    serial.chunks.extend(burst_frame(c) for c in range(1, 101))
                self.assertTrue(pump_until(
                    lambda: sum(isinstance(m, tp.RangeBurstMessage) for m in received) == 100))
                self.assertIn("online", states)
                worker.send_set_ds_cal(3, -12.5)
                expected = struct.pack("<HiB", 3, -12500, 1)

                def tag_got_cal():
                    with serial.lock:
                        frames = [f for w in serial.writes for f in tp.TelemetryStreamParser().feed(w)]
                    return any(f.payload[:1] == bytes((tp.CMD_SET_DS_CAL,)) and f.payload[1:] == expected
                               for f in frames)

                self.assertTrue(pump_until(tag_got_cal))
            finally:
                stop.set()
                runner.join(timeout=3)
                worker.stop()
        cycles = [m.cycle_seq for m in received if isinstance(m, tp.RangeBurstMessage)]
        self.assertEqual(cycles, list(range(1, 101)))

    def test_up4000_frames_convert_to_a_replayable_session(self):
        from tools.uwb_frames_to_session import convert

        with tempfile.TemporaryDirectory() as tmp:
            flight = Path(tmp) / "20261107_150000"
            flight.mkdir()
            writer = FrameRecordWriter(flight / "uwb_frames.bin")
            t0 = time.monotonic_ns()
            writer.write(device_info_frame(), time.time_ns(), t0)
            for c in range(1, 301):
                writer.write(burst_frame(c, (2.0 + c * 0.002, 1.5, 1.0)), time.time_ns(), t0 + c * 3_300_000)
            writer.close()
            anchors = Path(tmp) / "rig.json"
            save_anchor_layout_json(anchors, ANCHORS)
            session = convert(flight / "uwb_frames.bin", Path(tmp) / "logs", anchors)
            with (session / "uwb_fast_log.csv").open(newline="", encoding="utf-8") as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(len(rows), 300)
            last = rows[-1]
            self.assertAlmostEqual(float(last["pos_x_m"]), 2.6, delta=0.05)
            self.assertIn("UP4000_20261107_150000", session.name)


if __name__ == "__main__":
    unittest.main()
