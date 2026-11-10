"""Px4Link against a simulated PX4 over MAVLink (pymavlink, UDP loopback)."""
from __future__ import annotations

import json
import os
from pathlib import Path
import random
import socket
import sys
import tempfile
import threading
import time
import unittest

os.environ.setdefault("MAVLINK20", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from pymavlink import mavutil  # noqa: E402
from test_core import chunk_stream, synthetic_ulog  # noqa: E402
from uwb_px4_logger.px4_link import MAV_CMD_LOGGING_START, Px4Link  # noqa: E402


def free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakePx4(threading.Thread):
    """Heartbeat, ODOMETRY, TIMESYNC replies and a ULog stream on LOGGING_START."""

    def __init__(self, port: int, chunks, stale=()) -> None:
        super().__init__(daemon=True)
        self.stale = list(stale)  # an earlier session still streaming
        self.mav = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=1,
                                              source_component=1, dialect="common")
        self.chunks = chunks
        self.stop = threading.Event()
        self.received: list[str] = []
        self.acks: list[int] = []
        self.intervals: dict[int, float] = {}
        self.streamed = False

    def run(self) -> None:
        last_hb = 0.0
        told = False
        while not self.stop.is_set():
            now = time.monotonic()
            if now - last_hb > 0.1:
                self.mav.mav.heartbeat_send(2, 12, 0, 0, 4)  # quadrotor, PX4
                self.mav.mav.odometry_send(int(now * 1e6), 1, 12, 1.0, 2.0, -3.0, [1, 0, 0, 0],
                                           0.1, 0.2, 0.3, 0, 0, 0, [float("nan")] * 21, [float("nan")] * 21)
                last_hb = now
            if self.stale and not self.streamed:
                seq, data, offset, _acked = self.stale.pop(0)
                self.mav.mav.logging_data_send(0, 0, seq, len(data), offset, list(data) + [0] * (249 - len(data)))
            try:
                msg = self.mav.recv_match(blocking=True, timeout=0.01)
            except ConnectionResetError:  # Windows: ICMP unreachable before the logger bound
                continue
            if msg is None:
                continue
            if not told:  # the logger is listening now
                self.mav.mav.statustext_send(4, b"Preflight Fail: test warning")
                told = True
            mtype = msg.get_type()
            self.received.append(mtype)
            if mtype == "COMMAND_LONG" and msg.command == MAV_CMD_LOGGING_START and not self.streamed:
                self.mav.mav.command_ack_send(MAV_CMD_LOGGING_START, 0)
                self.streamed = True
                for seq, data, offset, acked in self.chunks:
                    payload = list(data) + [0] * (249 - len(data))
                    send = self.mav.mav.logging_data_acked_send if acked else self.mav.mav.logging_data_send
                    send(0, 0, seq, len(data), offset, payload)
                    time.sleep(0.0005)
            elif mtype == "COMMAND_LONG" and msg.command == 511:
                self.intervals[int(msg.param1)] = msg.param2
            elif mtype == "LOGGING_ACK":
                self.acks.append(msg.sequence)
            elif mtype == "TIMESYNC" and msg.tc1 == 0:
                self.mav.mav.timesync_send(int(time.monotonic_ns() + 5_000_000_000), msg.ts1)


class Px4LinkTests(unittest.TestCase):
    def test_tlog_ulog_timesync_and_ground_forwarding(self):
        self._run_link(stale=False)

    def test_stale_chunks_from_an_old_session_are_ignored(self):
        # 07/11 run 8: PX4 was still streaming the previous session; its first
        # chunk (sequence ~9100) made the whole new stream look like duplicates.
        link = self._run_link(stale=True)
        self.assertGreater(link.counts["ulog_chunks_before_header"], 0)

    def _run_link(self, stale):
        header, defs, data = synthetic_ulog(random.Random(7), 120)
        chunks = chunk_stream(header, defs, data)
        old = chunk_stream(*synthetic_ulog(random.Random(8), 60), start_seq=9000)
        old = [c for c in old if not c[3]][:20] if stale else []
        link_port, qgc_port = free_udp_port(), free_udp_port()
        qgc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        qgc.bind(("127.0.0.1", qgc_port))
        qgc.settimeout(0.2)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            px4 = FakePx4(link_port, chunks, old)
            link = Px4Link(device_resolver=lambda: f"udpin:127.0.0.1:{link_port}", out_dir=out,
                           ground_host="127.0.0.1", ground_port=qgc_port, local_port=0,
                           message_rates={"ODOMETRY": 50.0}, timesync_hz=10.0, log=lambda s: None)
            stop = threading.Event()
            runner = threading.Thread(target=link.run, args=(stop,), daemon=True)
            runner.start()
            px4.start()
            parser = mavutil.mavlink.MAVLink(None)
            parser.robust_parsing = True
            forwarded: list[str] = []
            deadline = time.monotonic() + 8.0
            sent_param_request = False
            while time.monotonic() < deadline:
                try:
                    data_in, addr = qgc.recvfrom(65535)
                except socket.timeout:
                    continue
                for m in parser.parse_buffer(data_in) or []:
                    forwarded.append(m.get_type())
                if not sent_param_request and "HEARTBEAT" in forwarded:
                    # QGC -> UP4000 -> PX4 passthrough
                    req = mavutil.mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
                    qgc.sendto(req.param_request_list_encode(1, 1).pack(req), addr)
                    sent_param_request = True
                if (px4.streamed and link._ulog is not None and link._ulog.chunks == len(chunks)
                        and link.counts["timesync_samples"] >= 3 and "PARAM_REQUEST_LIST" in px4.received):
                    break
            snap = link.snapshot()                                   # what the ground tab receives
            stop.set()
            runner.join(timeout=3)
            px4.stop.set()
            px4.join(timeout=3)
            qgc.close()

            self.assertIn("ODOMETRY", forwarded)
            self.assertNotIn("LOGGING_DATA", forwarded)
            self.assertNotIn("LOGGING_DATA_ACKED", forwarded)
            self.assertIn("PARAM_REQUEST_LIST", px4.received)
            self.assertIn(331, px4.intervals)                       # ODOMETRY rate request
            self.assertAlmostEqual(px4.intervals[331], 20000.0)      # 50 Hz
            acked = [c[0] for c in chunks if c[3]]
            self.assertTrue(set(acked) <= set(px4.acks))             # every ACKED chunk acknowledged
            ulog = (out / "px4_stream_1.ulg").read_bytes()
            self.assertEqual(ulog, header + b"".join(defs) + b"".join(data))
            rows = (out / "timesync.csv").read_text().splitlines()[1:]
            self.assertGreaterEqual(len(rows), 3)
            offset_s = int(rows[-1].split(",")[-1]) / 1e9
            self.assertAlmostEqual(offset_s, 5.0, delta=0.05)        # simulated PX4 clock offset
            tlog = mavutil.mavlink_connection(str(out / "px4.tlog"), dialect="common")
            types = []
            while True:
                m = tlog.recv_match()
                if m is None:
                    break
                types.append(m.get_type())
            tlog.close()
            self.assertIn("ODOMETRY", types)
            self.assertIn("LOGGING_DATA", types)  # the tlog keeps everything received
            self.assertEqual(snap["messages"]["ODOMETRY"]["fields"]["x"], 1.0)
            self.assertIsNone(snap["messages"]["ODOMETRY"]["fields"]["pose_covariance"][0])  # NaN -> None
            self.assertIn("HEARTBEAT", snap["messages"])
            self.assertIn("ODOMETRY", snap["rates_hz"])
            json.dumps(snap, allow_nan=False)                         # JSON-safe
            status = json.loads((out / "px4_status.json").read_text())
            self.assertEqual(status["ulog"]["chunks"], len(chunks))  # kept after the file closed
        return link


class SilentPx4Tests(unittest.TestCase):
    def test_silent_mavlink_port_is_reopened(self):
        class Silent:
            def __init__(self):
                self.closed = False
                self.mav = self

            def recv(self, n=None):
                return b""

            def heartbeat_send(self, *a):
                pass

            def close(self):
                self.closed = True

        links = []

        def connect(device, baud):
            links.append(Silent())
            return links[-1]

        with tempfile.TemporaryDirectory() as tmp:
            link = Px4Link(device_resolver=lambda: "FAKE", out_dir=Path(tmp), local_port=0,
                           log=lambda s: None, connect=connect)
            link.open_socket()
            t = 0.0
            while t < 7.0:
                link.step(now=t)
                t += 0.1
            link.close()
        self.assertEqual(link.counts["px4_silent_reopens"], 1)
        self.assertTrue(links[0].closed)
        self.assertEqual(len(links), 2)


if __name__ == "__main__":
    unittest.main()
