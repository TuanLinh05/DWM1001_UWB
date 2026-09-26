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
    AnchorInfoMessage,
    AnchorSample,
    CmdAckMessage,
    DiagAnchorMessage,
    DiagSystemMessage,
    INT16_MIN,
    InfoMessage,
    ParserCounters,
    RX_ERROR_FIELDS,
    RangeMeasMessage,
    RangeMessage,
    StatsMessage,
)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


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
            # One second later: 2000 new bytes, 200 of them discarded.
            recorder.record_uart_stats(
                ParserCounters(6000, 150, 203, 11, 2, 4, 5), 2000.0
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
        self.assertEqual(metadata["uart_snapshots"], 2)
        # The first snapshot is only the baseline of the cumulative counters.
        self.assertEqual((metadata["uart_bytes_received"], metadata["uart_discarded_bytes"],
                          metadata["uart_crc_errors"]), (2000, 200, 10))
        self.assertEqual(metadata["uart_discarded_pct"], 10.0)
        self.assertEqual((metadata["meas_seq_expected"], metadata["meas_delivered_pct"]),
                         (1, 100.0))
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

        # The whole session is summarised per anchor, A1..A8 even without data.
        summary = read_csv(session / "summary.csv")
        self.assertEqual([row["anchor_id"] for row in summary], [str(n) for n in range(1, 9)])
        self.assertEqual(summary[1]["meas_count"], "1")
        self.assertEqual(summary[1]["ss_fallback_count"], "1")
        self.assertEqual(summary[0]["snap_frames"], "1")
        timeline = read_csv(session / "anchor_timeline.csv")
        self.assertEqual(len(timeline) % 8, 0)
        self.assertGreaterEqual(len(timeline), 8)
        self.assertEqual(metadata["anchor_timeline_rows"], len(timeline))
        self.assertTrue(metadata["summary_written"])

    def test_every_other_message_type_gets_its_own_csv(self) -> None:
        test_root = Path(__file__).resolve().parents[1]
        temporary = test_root / f".recorder-test-{uuid.uuid4().hex}"
        temporary.mkdir()
        self.addCleanup(shutil.rmtree, temporary, True)
        recorder = SessionRecorder(temporary, "COM7", 1_000_000)
        try:
            recorder.record_message(DiagAnchorMessage(
                7, 1000, 5, True, True, False, 120, 340, 0, 12, 55, 120, 0, 60, 60, 1, 22,
                3, 4, 3100, 5600, 2650, 310, 420,
            ))
            recorder.record_message(AnchorInfoMessage(
                8, 1100, 6, 0x02, False, (1000, -2000, 2500), 0x202F8AE9, True, 0,
                16436, 16436, 4, 0x4C2C721F,
            ))
            recorder.record_message(CmdAckMessage(9, 1200, 0x0F, 0x08, bytes((0x07,))))
            recorder.record_message(DiagSystemMessage(
                10, 1300, 0xBEEF, 2, 0x10, 60_000, 2400, 25_100, 31_000, 2300, 0, 0, 0,
                59, 0, 0, False, 0, False, 0xFF, 0, 0, 700, 3, 60, 0, 0, False, 1,
                {name: index for index, name in enumerate(RX_ERROR_FIELDS)},
                900, 300, 280, 0, 31.5, None,
            ))
        finally:
            recorder.stop()

        session = recorder.session_directory
        diag = read_csv(session / "diag_anchor.csv")
        self.assertEqual(len(diag), 1)
        self.assertEqual((diag[0]["anchor_id"], diag[0]["backed_off"]), ("5", "1"))
        self.assertEqual((diag[0]["resp_timeouts"], diag[0]["report_timeouts"]), ("340", "60"))
        self.assertIn("host_elapsed_s", diag[0])

        info = read_csv(session / "anchor_info.csv")[0]
        self.assertEqual((info["build_hash"], info["build_config_hash"]),
                         ("0x202F8AE9", "0x4C2C721F"))
        self.assertEqual((info["position_x_mm"], info["position_z_mm"]), ("1000", "2500"))
        self.assertEqual((info["build_dirty"], info["anchor_status"]), ("1", "0x2"))

        ack = read_csv(session / "cmd_ack.csv")[0]
        self.assertEqual((ack["result_name"], ack["data"]), ("UNSUPPORTED", "07"))

        system = read_csv(session / "diag_system.csv")[0]
        self.assertEqual((system["active_mask"], system["reset_cause"]), ("0xFF", "0x10"))
        self.assertEqual((system["rx_errors_fcs"], system["rx_errors_soft_resets"]), ("1", "10"))
        self.assertEqual((system["temperature_c"], system["vbat_v"]), ("31.500", ""))

        metadata = json.loads((session / "session.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["message_counts"], {
            "anchor_info.csv": 1, "cmd_ack.csv": 1,
            "diag_anchor.csv": 1, "diag_system.csv": 1,
        })
        summary = {row["anchor_id"]: row for row in read_csv(session / "summary.csv")}
        self.assertEqual((summary["5"]["resp_timeouts_total"], summary["5"]["diag_frames"]),
                         ("340", "1"))
        self.assertEqual(summary["6"]["anchor_boot_last"], "4")


if __name__ == "__main__":
    unittest.main()
