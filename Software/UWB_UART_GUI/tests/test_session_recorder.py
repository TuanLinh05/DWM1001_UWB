"""Session-recorder regression tests; no UART hardware is required."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import shutil
import sys
import unittest
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from session_recorder import SessionRecorder  # noqa: E402
from telemetry_protocol import (  # noqa: E402
    AnchorSample,
    INT16_MIN,
    InfoMessage,
    ParserCounters,
    RangeMeasMessage,
    RangeMessage,
    StatsMessage,
)


class SessionRecorderTests(unittest.TestCase):
    def test_writes_complete_session(self) -> None:
        test_root = Path(__file__).resolve().parents[1]
        temporary = test_root / f".recorder-test-{uuid.uuid4().hex}"
        temporary.mkdir()
        self.addCleanup(shutil.rmtree, temporary, True)
        recorder = SessionRecorder(temporary, "COM7", 115200)
        try:
            recorder.record_event("OPEN COM7")
            recorder.record_raw(b"\x55\xAA\x01\x02")
            recorder.record_message(
                InfoMessage(1, 10, 2, 5, 1, 1, 0, 0, 2, 8, 0, 0, ((1, 0),))
            )
            recorder.record_message(
                RangeMessage(
                    2,
                    20,
                    (AnchorSample(1, True, 0, 0, 2345, 2300, -8125),),
                ),
                {1: 2320},
            )
            recorder.record_message(
                StatsMessage(3, 30, 10, 8, 2, 0, 0, 0, 50, 40)
            )
            recorder.record_message(
                RangeMeasMessage(
                    41, 25, 0x1234, 41, 5_000_123, 2, 0x10, 2, 0x0005, 0x30,
                    4210, 4100, 0, -8100, -7300, INT16_MIN, INT16_MIN, 20, 0x2000,
                    -150, 3200,
                )
            )
            recorder.record_uart_stats(
                ParserCounters(4000, 100, 3, 1, 2, 4, 5), 4123.5
            )
        finally:
            recorder.stop()

        session = recorder.session_directory
        self.assertEqual((session / "raw_telemetry.bin").read_bytes(), b"\x55\xAA\x01\x02")
        self.assertIn("OPEN COM7", (session / "events.log").read_text(encoding="utf-8"))

        with (session / "range.csv").open(encoding="utf-8", newline="") as handle:
            ranges = list(csv.DictReader(handle))
        self.assertEqual(len(ranges), 1)
        self.assertEqual(ranges[0]["anchor_id"], "1")
        self.assertEqual(ranges[0]["raw_mm"], "2345")
        self.assertEqual(ranges[0]["host_filtered_mm"], "2320")
        self.assertEqual(ranges[0]["status_text"], "OK")

        with (session / "meas.csv").open(encoding="utf-8", newline="") as handle:
            meas = list(csv.DictReader(handle))
        self.assertEqual(len(meas), 1)
        self.assertEqual(meas[0]["meas_seq"], "41")
        self.assertEqual(meas[0]["meas_time_us"], "5000123")
        self.assertEqual(meas[0]["anchor_id"], "2")
        self.assertEqual((meas[0]["mode"], meas[0]["mode_text"]), ("2", "SS_FALLBACK"))
        self.assertEqual((meas[0]["flags_hex"], meas[0]["status_hex"]), ("0x0005", "0x30"))
        self.assertEqual(meas[0]["status_text"], "DS_FALLBACK|CAL_MISSING")
        self.assertEqual((meas[0]["raw_mm"], meas[0]["corrected_mm"]), ("4210", "4100"))
        self.assertEqual((meas[0]["fp_cdbm"], meas[0]["rx_cdbm"]), ("-8100", "-7300"))
        self.assertEqual(meas[0]["nlos_db"], "8.00")
        # INT16_MIN = the anchor-side power was not reported: an empty cell.
        self.assertEqual((meas[0]["anchor_fp_cdbm"], meas[0]["anchor_rx_cdbm"]), ("", ""))
        self.assertEqual((meas[0]["ci_ppm_x100"], meas[0]["slot_us"]), ("-150", "3200"))

        with (session / "stats.csv").open(encoding="utf-8", newline="") as handle:
            stats = list(csv.DictReader(handle))
        self.assertEqual(stats[0]["poll_count"], "10")
        self.assertEqual(stats[0]["operation_hz"], "40")

        with (session / "uart.csv").open(encoding="utf-8", newline="") as handle:
            uart = list(csv.DictReader(handle))
        self.assertEqual(uart[0]["byte_rate"], "4123.500")
        self.assertEqual(uart[0]["valid_frames"], "100")
        self.assertEqual(uart[0]["decode_errors"], "5")

        info = json.loads((session / "info.jsonl").read_text(encoding="utf-8"))
        self.assertEqual(info["schema"], 2)
        self.assertEqual(info["active_offsets_um"], [[1, 0]])

        metadata = json.loads((session / "session.json").read_text(encoding="utf-8"))
        self.assertTrue(metadata["completed"])
        self.assertIsNone(metadata["error"])
        self.assertEqual(metadata["range_frames"], 1)
        self.assertEqual(metadata["range_samples"], 1)
        self.assertEqual(metadata["meas_records"], 1)
        self.assertEqual(metadata["info_frames"], 1)
        self.assertEqual(metadata["stats_frames"], 1)
        self.assertEqual(metadata["uart_snapshots"], 1)
        self.assertEqual(metadata["raw_frames"], 1)
        self.assertEqual(metadata["raw_bytes"], 4)
        self.assertEqual(metadata["event_lines"], 1)
        self.assertGreaterEqual(metadata["duration_s"], 0)
        self.assertEqual(
            metadata["host_filter"]["version"],
            "median3-cv-alpha-beta-v3",
        )
        self.assertEqual(
            metadata["host_filter"]["model"],
            "constant_velocity_alpha_beta",
        )
        self.assertIsNone(metadata["host_filter"]["maximum_range_mm"])
        self.assertIsNone(metadata["host_filter"]["snap_after_samples"])


if __name__ == "__main__":
    unittest.main()
