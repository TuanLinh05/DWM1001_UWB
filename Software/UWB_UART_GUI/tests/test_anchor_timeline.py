"""Per-anchor timeline and session summary; no UART hardware is required."""

from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from anchor_timeline import (  # noqa: E402
    ANCHOR_IDS,
    SUMMARY_COLUMNS,
    TIMELINE_COLUMNS,
    AnchorTimeline,
)
from telemetry_protocol import (  # noqa: E402
    INT16_MIN,
    AnchorInfoMessage,
    AnchorSample,
    DiagAnchorMessage,
    RangeMeasMessage,
    RangeMessage,
    STATUS_CALIBRATION_MISSING,
    STATUS_DS_FALLBACK,
    STATUS_TIMEOUT,
)


MEAS_A5 = RangeMeasMessage(
    sequence=0, time_ms=0, boot_id=1, meas_seq=0, meas_time_us=0, anchor_id=5, txn=0,
    mode=0, flags=0x0001, status=STATUS_CALIBRATION_MISSING, raw_mm=3000,
    corrected_mm=0, filtered_mm=0, fp_cdbm=-8500, rx_cdbm=-8000,
    anchor_fp_cdbm=-8600, anchor_rx_cdbm=-8100, std_noise=30, fp_index=0x2000,
    ci_ppm_x100=-1250, slot_us=3100,
)


def diag(anchor_id: int, success: int, resp_timeouts: int, backed_off: bool) -> DiagAnchorMessage:
    return DiagAnchorMessage(
        sequence=0, time_ms=0, anchor_id=anchor_id, active=True, backed_off=backed_off,
        calibrated=False, success=success, resp_timeouts=resp_timeouts, poll_tx_timeouts=0,
        rx_errors=2, poll_skipped=40, cal_missing=success, ds_ok=success // 2,
        report_timeouts=7, ds_fallbacks=7, txn_mismatch=0, probes=9, resp_streak=0,
        ds_streak=0, slot_us=3100, slot_max_us=5400, resp_wait_max_us=2610,
        processing_max_us=300, poll_tx_max_us=400,
    )


class AnchorTimelineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.rows: list[dict[str, object]] = []
        self.timeline = AnchorTimeline(
            datetime(2026, 9, 25, 8, 0, 0, tzinfo=timezone.utc), self.rows.append)

    def rows_for(self, anchor_id: int) -> list[dict[str, object]]:
        return [row for row in self.rows if row["anchor_id"] == anchor_id]

    def test_every_anchor_gets_a_row_every_second(self) -> None:
        timeline = self.timeline
        timeline.add_message(MEAS_A5, 0.10)
        timeline.add_message(replace(MEAS_A5, raw_mm=3010), 0.50)
        timeline.add_message(
            replace(MEAS_A5, mode=2, status=STATUS_DS_FALLBACK, raw_mm=3020,
                    anchor_fp_cdbm=INT16_MIN, anchor_rx_cdbm=INT16_MIN), 1.20)
        timeline.add_message(RangeMessage(1, 20, (
            AnchorSample(5, False, STATUS_CALIBRATION_MISSING, 0, 3020, 0, -8500),
            AnchorSample(8, False, STATUS_TIMEOUT, 65535, 0, 0, INT16_MIN),
        )), 1.30)
        timeline.advance(3.05)                       # 2.0 s .. 3.0 s had no traffic

        self.assertEqual(len(self.rows), 3 * len(ANCHOR_IDS))
        self.assertTrue(all(set(row) == set(TIMELINE_COLUMNS) for row in self.rows))
        a5 = self.rows_for(5)
        self.assertEqual([row["meas_count"] for row in a5], [2, 1, 0])
        self.assertEqual((a5[0]["ds_count"], a5[1]["ss_fallback_count"]), (2, 1))
        self.assertEqual((a5[0]["raw_mean_mm"], a5[0]["raw_min_mm"], a5[0]["raw_max_mm"]),
                         ("3005.0", "3000", "3010"))
        self.assertEqual((a5[0]["fp_dbm_mean"], a5[0]["nlos_db_mean"]), ("-85.00", "5.00"))
        self.assertEqual((a5[0]["ci_ppm_mean"], a5[1]["anchor_fp_dbm_mean"]), ("-12.50", ""))
        self.assertEqual(a5[1]["snap_frames"], 1)
        self.assertEqual(a5[2]["raw_mean_mm"], "")
        self.assertEqual(a5[0]["window_start_s"], "0.000")
        self.assertEqual(a5[2]["window_end_s"], "3.000")
        self.assertTrue(str(a5[0]["host_time_iso"]).startswith("2026-09-25T08:00:01.000"))
        a8 = self.rows_for(8)
        self.assertEqual([row["meas_count"] for row in a8], [0, 0, 0])
        self.assertEqual((a8[1]["snap_timeout"], a8[1]["last_status_text"]), (1, "TIMEOUT"))
        self.assertEqual(a8[0]["diag_success"], "")  # no DIAG_ANCHOR yet

    def test_summary_reports_rates_gaps_and_counter_deltas(self) -> None:
        timeline = self.timeline
        timeline.add_message(diag(5, 100, 10, False), 0.2)
        timeline.add_message(AnchorInfoMessage(
            0, 0, 5, 0, False, (0, 0, 0), 0x202F8AE9, True, 0, 16436, 16436, 3, 0x6AE7796E,
        ), 0.3)
        timeline.add_message(MEAS_A5, 0.5)
        timeline.add_message(replace(MEAS_A5, mode=2, raw_mm=3100), 2.5)
        timeline.add_message(diag(5, 160, 45, True), 3.0)
        timeline.add_message(diag(6, 50, 5, False), 1.0)
        timeline.add_message(diag(6, 20, 1, False), 2.0)   # the TAG rebooted
        timeline.finish(4.0)

        summary = {row["anchor_id"]: row for row in timeline.summary_rows()}
        self.assertEqual(sorted(summary), list(ANCHOR_IDS))
        self.assertTrue(all(set(row) == set(SUMMARY_COLUMNS) for row in summary.values()))
        a5 = summary[5]
        self.assertEqual((a5["meas_count"], a5["meas_hz"], a5["ds_pct"]), (2, "0.50", "50.0"))
        self.assertEqual((a5["longest_gap_s"], a5["ss_fallback_count"]), ("2.000", 1))
        self.assertEqual((a5["success_total"], a5["success_delta"]), (160, 60))
        self.assertEqual((a5["resp_timeouts_delta"], a5["diag_backed_off_last"]), (35, 1))
        self.assertEqual((a5["anchor_build_hash"], a5["anchor_config_hash"]),
                         ("0x202F8AE9", "0x6AE7796E"))
        self.assertEqual((a5["anchor_build_dirty"], a5["anchor_boot_last"]), (1, 3))
        self.assertEqual(summary[6]["success_delta"], "")   # negative: not a delta
        a8 = summary[8]
        self.assertEqual((a8["meas_count"], a8["meas_hz"], a8["success_total"]), (0, "0.00", ""))
        # Four whole seconds: the final partial window is empty and not repeated.
        self.assertEqual(len(self.rows), 4 * len(ANCHOR_IDS))


if __name__ == "__main__":
    unittest.main()
