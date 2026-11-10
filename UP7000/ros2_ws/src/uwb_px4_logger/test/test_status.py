"""Host health from a fake /proc + /sys tree, and the status datagram."""
from __future__ import annotations

import json
from pathlib import Path
import socket
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uwb_px4_logger.status_reporter import (  # noqa: E402
    STATUS_KIND, HostHealth, StatusSender, session_files,
)


def write(root: Path, rel: str, text: str) -> None:
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


class HostHealthTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.proc, self.sys = root / "proc", root / "sys"
        write(self.proc, "stat", "cpu  100 0 100 800 0 0 0 0 0 0\ncpu0 50 0 50 400 0 0 0 0 0 0\n")
        write(self.proc, "loadavg", "3.59 3.84 3.62 3/512 70000\n")
        write(self.proc, "meminfo", "MemTotal:        1886000 kB\nMemFree:  74000 kB\nMemAvailable:     702000 kB\n")
        write(self.proc, "uptime", "11280.55 20000.00\n")
        write(self.proc, "net/wireless",
              "Inter-| sta-|   Quality        |   Discarded packets\n"
              " face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n"
              "wlx30169df9a262: 0000   58.  -52.  -256        0      0      0     12      0        0\n")
        write(self.sys, "class/thermal/thermal_zone0/type", "acpitz\n")
        write(self.sys, "class/thermal/thermal_zone0/temp", "40000\n")
        write(self.sys, "class/thermal/thermal_zone1/type", "x86_pkg_temp\n")
        write(self.sys, "class/thermal/thermal_zone1/temp", "61500\n")
        cmd = "/usr/bin/python3\0/home/dronepif/UP4000/ros2_ws/install/uwb_px4_logger/lib/uwb_px4_logger/uwb_relay\0"
        write(self.proc, "4242/cmdline", cmd)
        self.stat_line = "4242 (uwb_relay) S 1 1 1 0 -1 0 0 0 0 0 {ut} {st} 0 0 20 0 5 0 100 777888 14650 0\n"
        write(self.proc, "4242/stat", self.stat_line.format(ut=1000, st=200))
        self.iw = {  # `iw` output from the UP4000 (rtl8821cu) on 07/11
            ("iw", "dev", "wlx30169df9a262", "link"):
                "Connected to 11:22:33:44:55:66 (on wlx30169df9a262)\n\tSSID: PIF_CLUB_5G\n"
                "\tfreq: 5500\n\tsignal: -48 dBm\n\ttx bitrate: 434.0 MBit/s\n",
            ("iw", "dev", "wlx30169df9a262", "get", "power_save"): "Power save: on\n",
        }
        self.health = HostHealth(proc=str(self.proc), sys=str(self.sys), log_dir=self.tmp.name,
                                 run=lambda argv: self.iw.get(tuple(argv)))
        self.health._ticks = 100

    def tearDown(self):
        self.tmp.cleanup()

    def test_sample_reads_every_source(self):
        first = self.health.sample()
        self.assertIsNone(first["cpu_percent"])          # needs two samples
        self.assertEqual(first["load"], [3.59, 3.84, 3.62])
        self.assertEqual(first["memory"]["total_mb"], 1842)
        self.assertAlmostEqual(first["memory"]["used_percent"], 62.8, delta=0.1)
        self.assertEqual(first["cpu_temp_c"], 61.5)
        self.assertEqual(first["temperatures_c"], {"acpitz": 40.0, "x86_pkg_temp": 61.5})
        self.assertEqual(first["wifi"][0], {"interface": "wlx30169df9a262", "link_quality": 58.0,
                                            "signal_dbm": -48.0, "noise_dbm": None, "ssid": "PIF_CLUB_5G",
                                            "freq_mhz": 5500.0, "tx_mbit": 434.0, "power_save": True})
        self.assertAlmostEqual(first["uptime_s"], 11280.55)
        self.assertIsNotNone(first["disk"]["free_gb"])
        self.assertEqual(first["processes"]["uwb_relay"]["pid"], 4242)

        time.sleep(0.05)  # Windows monotonic clock ticks ~15 ms
        write(self.proc, "stat", "cpu  160 0 140 900 0 0 0 0 0 0\n")       # +100 busy, +100 idle
        write(self.proc, "4242/stat", self.stat_line.format(ut=1030, st=210))
        second = self.health.sample()
        self.assertAlmostEqual(second["cpu_percent"], 50.0)
        self.assertGreater(second["processes"]["uwb_relay"]["cpu_percent"], 0.0)

    def test_missing_sources_give_none_not_errors(self):
        empty = HostHealth(proc=str(Path(self.tmp.name) / "nope"), sys=str(Path(self.tmp.name) / "nope"))
        s = empty.sample()
        self.assertIsNone(s["cpu_percent"])
        self.assertIsNone(s["memory"])
        self.assertEqual(s["wifi"], [])
        self.assertEqual(s["processes"], {})


    def test_positive_wifi_level_is_not_dbm(self):
        # rtl8821cu prints the level as a positive number; without iw there is no dBm value.
        write(self.proc, "net/wireless",
              "Inter-| sta-|   Quality        |   Discarded packets\n"
              " face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n"
              "wlx30169df9a262: 0000   95.   52.    0.       0      0      0      0      0        0\n")
        self.iw.clear()
        wifi = self.health.sample()["wifi"][0]
        self.assertIsNone(wifi["signal_dbm"])
        self.assertEqual(wifi["signal_level"], 52.0)
        self.assertNotIn("power_save", wifi)


def merge(state: dict, path: str, data: dict) -> None:
    """Receiver side (same as New_GUI up4000_panel): update the dict at path."""
    node = state
    for key in path.split("."):
        node = node.setdefault(key, {})
    node.update(data)


def big_px4() -> dict:
    """Roughly the size of the real 07/11 snapshot (14 message types, ~5.5 KB)."""
    messages = {f"TYPE_{i}": {"age_s": 0.01 * i, "fields": {f"field_{j}": j * 1.234567 for j in range(25)}}
                for i in range(14)}
    messages["ODOMETRY"] = {"age_s": 0.02, "fields": {"x": 1.0, "pose_covariance": [None] * 21,
                                                      "velocity_covariance": [None] * 21}}
    return {"connected": True, "port": "/dev/pixhawk", "rates_hz": {f"T{i}": 50.0 for i in range(40)},
            "statustext": [{"time_unix": 1.0, "severity": 4, "text": "x" * 50}] * 10, "messages": messages}


class StatusSenderTests(unittest.TestCase):
    def test_parts_fit_one_datagram_and_merge_back(self):
        sections = {"host": {"cpu_percent": 12.5, "wifi": [{"signal_dbm": -50.0}]},
                    "px4": big_px4(), "uwb_relay": {"counts": {f"k{i}": i for i in range(150)}},
                    "px4_logger": None}
        sender = StatusSender()
        datagrams = sender.datagrams(sections)
        sender.close()
        self.assertGreater(len(datagrams), 5)
        self.assertTrue(all(len(d) <= 1200 for d in datagrams), [len(d) for d in datagrams])
        state: dict = {}
        for d in datagrams:
            msg = json.loads(d)
            self.assertEqual((msg["kind"], msg["version"], msg["seq"]), (STATUS_KIND, 2, 1))
            merge(state, msg["part"], msg["data"])
        expected = {k: v for k, v in sections.items() if v is not None}
        self.assertEqual(json.loads(json.dumps(state)), json.loads(json.dumps(expected)))

    def test_byte_limit_includes_envelope_for_long_unicode_values(self):
        import base64
        sender = StatusSender(max_part_bytes=400)
        sender.seq = 999999999
        expected = {"statustext": [{"text": "cảm biến 📡" * 90}] * 4}
        packets = sender.datagrams({"px4": expected})
        sender.close()
        self.assertTrue(all(len(p) <= 400 for p in packets), [len(p) for p in packets])
        fragments = [json.loads(p) for p in packets]
        self.assertGreater(len(fragments), 1)
        fragments.sort(key=lambda p: p["fragment"]["index"])
        decoded = json.loads(b"".join(base64.b64decode(p["data"]) for p in fragments))
        self.assertEqual(decoded, expected)

    def test_unicast_and_rate_limited_broadcast(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(1.0)
        sender = StatusSender(rx.getsockname()[1])
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "px4.tlog").write_bytes(b"x" * 10)
            sections = {"session": {"files": session_files(Path(tmp))}, "host": {"cpu_percent": 1.0}}
        self.assertEqual(sender.send(sections, ""), 0)                          # no ground, no broadcast
        self.assertEqual(sender.send(sections, "", broadcast="127.0.0.1"), 2)   # discovery broadcast
        self.assertEqual(sender.send(sections, "", broadcast="127.0.0.1"), 0)   # at most 1 per second
        self.assertEqual(sender.send(sections, "127.0.0.1"), 2)                 # unicast: every call
        start = time.monotonic()
        self.assertEqual(sender.send(sections, "127.0.0.1", part_gap_s=0.05), 2)  # parts spaced
        self.assertGreaterEqual(time.monotonic() - start, 0.04)
        got = [json.loads(rx.recvfrom(65535)[0]) for _ in range(6)]
        rx.close()
        sender.close()
        self.assertEqual({g["part"] for g in got}, {"session", "host"})
        self.assertIn({"files": {"px4.tlog": 10}}, [g["data"] for g in got])


if __name__ == "__main__":
    unittest.main()
