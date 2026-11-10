"""ROS-free tests for the UP4000 logger core (run with pytest or unittest).

python -m pytest UP4000/ros2_ws/src/uwb_px4_logger/test
"""
from __future__ import annotations

import io
import os
from pathlib import Path
import random
import socket
import struct
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uwb_px4_logger.session import FRAME_FILE_MAGIC, FrameRecordWriter, find_port, read_frame_records  # noqa: E402
from uwb_px4_logger.ulog_stream import UlogStreamWriter  # noqa: E402
from uwb_px4_logger.uwb_relay import UwbRelay  # noqa: E402
from uwb_px4_logger.vendor import telemetry_protocol as tp  # noqa: E402

REPO = Path(__file__).resolve().parents[5]


# --------------------------------------------------------------------- ULog
def ulog_message(msg_type: str, payload: bytes) -> bytes:
    return struct.pack("<HB", len(payload), ord(msg_type)) + payload


def synthetic_ulog(rng: random.Random, data_messages: int = 400) -> tuple[bytes, list[bytes], list[bytes]]:
    header = b"ULog\x01\x12\x35\x01" + struct.pack("<Q", 123456)
    definitions = [ulog_message("B", bytes(40)),
                   ulog_message("F", b"vehicle_odometry:uint64_t timestamp;float[3] position;"),
                   ulog_message("A", b"\x00\x01\x00vehicle_odometry")]
    data = [ulog_message("D", b"\x01\x00" + bytes(rng.randrange(256) for _ in range(rng.randrange(4, 600))))
            for _ in range(data_messages)]
    return header, definitions, data


class PxLogWriterModel:
    """Model of PX4 LogWriterMavlink: 249-byte chunks, first_message_offset set
    when a write starts in a chunk without one; definitions need ACK."""

    def __init__(self) -> None:
        self.chunks: list[tuple[int, bytes, int, bool]] = []
        self.buf = bytearray()
        self.offset = 255
        self.seq = 0
        self.acked = True

    def write(self, message: bytes) -> None:
        if self.offset == 255:
            self.offset = len(self.buf)
        data = memoryview(message)
        while data:
            n = min(len(data), 249 - len(self.buf))
            self.buf += data[:n]
            data = data[n:]
            if len(self.buf) == 249:
                self.publish()

    def publish(self) -> None:
        if self.buf:
            self.chunks.append((self.seq & 0xFFFF, bytes(self.buf), self.offset, self.acked))
            self.seq += 1
            self.buf = bytearray()
            self.offset = 255


def chunk_stream(header, definitions, data, start_seq=0):
    model = PxLogWriterModel()
    model.seq = start_seq
    model.write(header)
    for d in definitions:
        model.write(d)
    model.publish()
    model.acked = False
    for d in data:
        model.write(d)
    model.publish()
    return model.chunks


def parse_ulog(blob: bytes) -> list[bytes]:
    assert blob[:7] == b"ULog\x01\x12\x35"
    pos, out = 16, []
    while pos < len(blob):
        size = blob[pos] | blob[pos + 1] << 8
        out.append(blob[pos:pos + 3 + size])
        pos += 3 + size
    assert pos == len(blob), "trailing partial ULog message"
    return out


class UlogStreamTests(unittest.TestCase):
    def test_lossless_stream_is_byte_identical(self):
        header, defs, data = synthetic_ulog(random.Random(1))
        out = io.BytesIO()
        w = UlogStreamWriter(out)
        for seq, chunk, offset, acked in chunk_stream(header, defs, data):
            w.handle(seq, chunk, offset, acked)
        self.assertEqual(out.getvalue(), header + b"".join(defs) + b"".join(data))
        self.assertEqual(w.dropped_chunks, 0)

    def test_lost_chunks_keep_whole_messages_and_mark_dropouts(self):
        for seed in range(20):
            loss = (0.02, 0.08, 0.25)[seed % 3]
            with self.subTest(seed=seed, loss=loss):
                header, defs, data = synthetic_ulog(random.Random(seed), 600)
                chunks = chunk_stream(header, defs, data)
                rng = random.Random(1000 + seed)
                first_data = next(i for i, c in enumerate(chunks) if not c[3])
                kept = [c for i, c in enumerate(chunks) if i < first_data + 2 or rng.random() > loss]
                out = io.BytesIO()
                w = UlogStreamWriter(out)
                for seq, chunk, offset, acked in kept:
                    w.handle(seq, chunk, offset, acked)
                messages = parse_ulog(out.getvalue())
                self.assertEqual(messages[:3], defs)
                originals = set(data)
                dropouts = [m for m in messages if m[2] == ord("O")]
                rest = [m for m in messages[3:] if m[2] != ord("O")]
                self.assertTrue(all(m in originals for m in rest))       # never a spliced message
                self.assertEqual(rest, [d for d in data if d in set(rest)])  # order kept
                self.assertGreater(len(dropouts), 0)
                self.assertEqual(len(dropouts), w.dropout_messages)
                self.assertGreater(len(rest), (0.75 - 2 * loss) * len(data))

    def test_duplicates_and_sequence_wrap(self):
        header, defs, data = synthetic_ulog(random.Random(4), 300)
        chunks = chunk_stream(header, defs, data, start_seq=65530)
        self.assertTrue(any(c[0] < 10 for c in chunks))
        out = io.BytesIO()
        w = UlogStreamWriter(out)
        for i, (seq, chunk, offset, acked) in enumerate(chunks):
            self.assertTrue(w.handle(seq, chunk, offset, acked))
            if i % 5 == 0:  # PX4 retransmits an ACKED chunk whose ACK it missed
                self.assertFalse(w.handle(seq, chunk, offset, acked))
        self.assertEqual(out.getvalue(), header + b"".join(defs) + b"".join(data))
        self.assertGreater(w.duplicates, 0)


# --------------------------------------------------------------------- UWB
def device_info_frame(boot_id=77, features=0x05) -> bytes:
    payload = struct.pack("<BBHIBBBBIHHIBBHHIBBIIIBBBBBBBIBB",
                          1, 1, 0, 0x2D19AC75, 0, 3, features, 1, 0x1234, boot_id, 1, 0,
                          0, 0, 16436, 16436, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 460800, 0xFF, 0xFF)
    payload += struct.pack("<II", 0xB5F8839D, 0)
    return tp.encode_frame(tp.TYPE_DEVICE_INFO, 1, 10, payload)


def burst_frame(cycle: int) -> bytes:
    anchors = tuple(tp.BurstAnchor(a, 0, tp.BURST_FLAG_RADIO_OK | tp.BURST_FLAG_CAL_OK | tp.BURST_FLAG_VALID,
                                   2000 + a * 100 + cycle % 7, -8400, -8000, -250) for a in range(1, 9))
    msg = tp.RangeBurstMessage(cycle, cycle * 3, 77, cycle, cycle * 8, cycle * 3300, 3300, anchors)
    return tp.encode_frame(tp.TYPE_RANGE_BURST, cycle, cycle * 3, tp.encode_range_burst_payload(msg))


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.writes: list[bytes] = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, n):
        if not self.chunks:
            time.sleep(0.001)
            return b""
        return self.chunks.pop(0)

    def write(self, data):
        self.writes.append(bytes(data))

    def close(self):
        self.closed = True


def drain(sock: socket.socket) -> list[bytes]:
    out = []
    sock.settimeout(0.05)
    while True:
        try:
            out.append(sock.recvfrom(65535)[0])
        except (socket.timeout, OSError):
            return out


class UwbRelayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.ground = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ground.bind(("127.0.0.1", 0))
        self.ground_port = self.ground.getsockname()[1]

    def tearDown(self):
        self.ground.close()
        self.tmp.cleanup()

    def make_relay(self, serial, **kw):
        relay = UwbRelay(port_resolver=lambda: "FAKE", out_dir=Path(self.tmp.name),
                         ground_port=self.ground_port, local_port=0,
                         serial_factory=lambda port, baud: serial, log=lambda s: None, **kw)
        relay.open_socket()
        return relay

    def run_steps(self, relay, n=200):
        for _ in range(n):
            relay.step()
        relay._flush_udp(time.monotonic())

    def test_frames_logged_and_forwarded_exactly(self):
        frames = [device_info_frame()] + [burst_frame(c) for c in range(1, 60)]
        stream = b"".join(frames)
        serial = FakeSerial([stream[i:i + 97] for i in range(0, len(stream), 97)])  # split mid-frame
        relay = self.make_relay(serial, ground_host="127.0.0.1")
        self.run_steps(relay)
        relay.close()
        datagrams = drain(self.ground)
        self.assertLess(len(datagrams), len(frames))  # several frames per datagram
        self.assertTrue(all(len(d) <= 1200 for d in datagrams))
        self.assertEqual(b"".join(datagrams), stream)  # byte stream forwarded unchanged
        records = list(read_frame_records(Path(self.tmp.name) / "uwb_frames.bin"))
        self.assertEqual(b"".join(r[2] for r in records), stream)  # every byte logged as read
        self.assertTrue(all(b[1] >= a[1] for a, b in zip(records, records[1:])))
        self.assertEqual(relay.parse_counts["frame_0x18"], 59)
        # The relay configures the TAG itself: PING, DEVICE_INFO, then telemetry 0x07.
        sent = [f for w in serial.writes for f in tp.TelemetryStreamParser().feed(w)]
        commands = [f.payload[0] for f in sent]
        self.assertIn(tp.CMD_PING, commands)
        self.assertIn(tp.CMD_GET_DEVICE_INFO, commands)
        telem = [f for f in sent if f.payload[0] == tp.CMD_SET_TELEMETRY]
        self.assertEqual(telem[0].payload[1], 0x07)

    def test_ground_commands_reach_the_tag_only_as_valid_command_frames(self):
        serial = FakeSerial([])
        relay = self.make_relay(serial, ground_host="127.0.0.1")
        relay_addr = ("127.0.0.1", relay._sock.getsockname()[1])
        cal = tp.encode_command(9, tp.CMD_SET_DS_CAL, struct.pack("<HiB", 3, -12000, 1))
        corrupt = bytearray(tp.encode_command(10, tp.CMD_SAVE_SETTINGS))
        corrupt[-1] ^= 0xFF
        self.ground.sendto(burst_frame(1) + bytes(corrupt) + cal, relay_addr)
        time.sleep(0.05)
        self.run_steps(relay, 20)
        relay.close()
        self.assertIn(cal, serial.writes)
        self.assertFalse(any(w == bytes(corrupt) for w in serial.writes))
        self.assertEqual(relay.counts["ground_commands"], 1)
        self.assertEqual(relay.counts["ground_non_command"], 1)

    def test_discovery_beacon_then_unicast_to_the_answering_gui(self):
        serial = FakeSerial([device_info_frame()])
        found = []
        relay = self.make_relay(serial, beacon_host="127.0.0.1", on_ground=found.append)
        self.run_steps(relay, 30)
        beacons = drain(self.ground)
        self.assertTrue(beacons)
        self.assertEqual(beacons[0], device_info_frame())
        # The GUI answers from its listening socket; the relay learns that address.
        relay_addr = ("127.0.0.1", relay._sock.getsockname()[1])
        self.ground.sendto(tp.encode_command(1, tp.CMD_PING), relay_addr)
        time.sleep(0.05)
        serial.chunks = [burst_frame(c) for c in range(1, 20)]
        self.run_steps(relay, 60)
        relay.close()
        self.assertEqual(found, ["127.0.0.1"])
        parser = tp.TelemetryStreamParser()
        types = [f.type for d in drain(self.ground) for f in parser.feed(d)]
        self.assertEqual(types.count(tp.TYPE_RANGE_BURST), 19)

    def test_failing_ros_callback_does_not_stop_logging(self):
        frames = [burst_frame(c) for c in range(1, 30)]

        def broken_publish(msg, realtime_ns, monotonic_ns):
            raise RuntimeError("publisher's context is invalid")  # rclpy after shutdown

        relay = self.make_relay(FakeSerial(frames), ground_host="127.0.0.1", on_message=broken_publish)
        self.run_steps(relay, 60)
        relay.close()
        logged = b"".join(r[2] for r in read_frame_records(Path(self.tmp.name) / "uwb_frames.bin"))
        self.assertEqual(logged, b"".join(frames))
        self.assertEqual(relay.parse_counts["callback_errors"], len(frames))
        self.assertEqual(len(drain(self.ground)) > 0, True)

    def test_silent_port_is_reopened(self):
        # 07/11: after a USB re-enumeration the open handle gave no data and no error.
        opened = []

        def factory(port, baud):
            ser = FakeSerial([] if not opened else [burst_frame(len(opened))])
            opened.append(ser)
            return ser

        relay = UwbRelay(port_resolver=lambda: "FAKE", out_dir=Path(self.tmp.name), ground_host="127.0.0.1",
                         ground_port=self.ground_port, local_port=0, serial_factory=factory, log=lambda s: None)
        relay.open_socket()
        t = 0.0
        while t < 4.5:
            relay.step(now=t)
            t += 0.1
        relay.close()
        self.assertEqual(relay.counts["tag_silent_reopens"], 1)
        self.assertGreaterEqual(len(opened), 2)
        self.assertTrue(opened[0].closed)
        self.assertEqual(relay.writer.records, 1)

    def test_serial_reconnects_after_unplug(self):
        attempts = []

        class Unplugged(FakeSerial):
            def read(self, n):
                raise OSError("device disconnected")

        def factory(port, baud):
            attempts.append(port)
            if len(attempts) == 1:
                raise OSError("not yet enumerated")
            return Unplugged([]) if len(attempts) == 2 else FakeSerial([burst_frame(5)])

        relay = UwbRelay(port_resolver=lambda: "FAKE", out_dir=Path(self.tmp.name), ground_host="127.0.0.1",
                         ground_port=self.ground_port, local_port=0, serial_factory=factory, log=lambda s: None)
        relay.open_socket()
        t = 0.0
        for _ in range(40):
            relay.step(now=t)
            t += 0.6
        relay.close()
        self.assertGreaterEqual(len(attempts), 3)
        self.assertEqual(relay.counts["tag_io_errors"], 1)      # the unplug
        self.assertGreaterEqual(relay.writer.records, 1)         # data after reconnecting


class SessionTests(unittest.TestCase):
    def test_truncated_record_file_reads_complete_records(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "uwb_frames.bin"
            w = FrameRecordWriter(path)
            w.write(burst_frame(1), 10, 20)
            w.write(burst_frame(2), 11, 21)
            w.close()
            blob = path.read_bytes()
            self.assertTrue(blob.startswith(FRAME_FILE_MAGIC))
            path.write_bytes(blob[:-5])  # power cut mid-record
            self.assertEqual([r[:2] for r in read_frame_records(path)], [(10, 20)])

    def test_other_port_users_finds_xrce_agent_and_launch_args(self):
        from uwb_px4_logger.session import other_port_users

        with tempfile.TemporaryDirectory() as proc:
            def fake(pid, *argv):
                (Path(proc) / str(pid)).mkdir()
                (Path(proc) / str(pid) / "cmdline").write_bytes(b"\0".join(a.encode() for a in argv) + b"\0")

            fake(65807, "MicroXRCEAgent", "serial", "--dev", "/dev/ttyACM0", "-b", "921600")
            fake(66844, "/usr/bin/python3", "ros2", "launch", "collect.launch.py", "uwb_port:=/dev/ttyACM0")
            fake(1234, "vim", "/dev/ttyACM01-notes.txt")      # different name, no match
            fake(os.getpid(), "python3", "/dev/ttyACM0")       # ourselves
            found = other_port_users("/dev/ttyACM0", proc_root=proc)
        self.assertEqual(sorted(f.split(":")[0] for f in found), ["pid 65807", "pid 66844"])

    @unittest.skipUnless(sys.platform.startswith("linux"), "flock lock files are Linux-only")
    def test_second_instance_is_refused(self):
        from uwb_px4_logger.session import AlreadyRunning, single_instance

        with tempfile.TemporaryDirectory() as tmp:
            first = single_instance("unit_test", lock_dir=tmp)
            with self.assertRaises(AlreadyRunning) as ctx:
                single_instance("unit_test", lock_dir=tmp)
            self.assertIn(f"pid {os.getpid()}", str(ctx.exception))
            self.assertIn("systemctl stop uwb-px4-logger", str(ctx.exception))
            first.close()
            single_instance("unit_test", lock_dir=tmp).close()  # free again after release

    def test_busy_udp_port_is_explained(self):
        from uwb_px4_logger.session import AlreadyRunning, bind_or_explain

        busy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        busy.bind(("0.0.0.0", 0))
        port = busy.getsockname()[1]
        second = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            with self.assertRaises(AlreadyRunning) as ctx:
                bind_or_explain(lambda: second.bind(("0.0.0.0", port)), port, "uwb_relay")
            self.assertIn(f"UDP port {port} is busy", str(ctx.exception))
        finally:
            busy.close()
            second.close()

    def test_find_port_prefers_explicit_setting(self):
        self.assertEqual(find_port("/dev/ttyACM3", ("/nonexistent/*",)), "/dev/ttyACM3")
        self.assertIsNone(find_port("auto", ("/nonexistent/*",)))


class VendorSyncTests(unittest.TestCase):
    @unittest.skipUnless((REPO / "Software" / "UWB_UART_GUI").is_dir(),
                         "GUI source not present (UP4000 folder copied alone)")
    def test_vendored_protocol_matches_gui_source(self):
        sys.path.insert(0, str(REPO / "UP4000" / "scripts"))
        import sync_vendor

        vendor = Path(__file__).resolve().parents[1] / "uwb_px4_logger" / "vendor"
        for name in sync_vendor.FILES:
            self.assertEqual((vendor / name).read_text(encoding="utf-8"), sync_vendor.vendored_text(name),
                             f"{name} is stale: run UP4000/scripts/sync_vendor.py")


if __name__ == "__main__":
    unittest.main()
