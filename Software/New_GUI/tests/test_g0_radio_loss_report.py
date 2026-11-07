"""G0.2 — tools/radio_loss_report.py: why and when ranges go missing.

Hand-off contract for the radio-loss analysis tool. Synthetic sessions are
written with tests/g0_fixtures.write_session (schema-5 column names).
"""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from g0_fixtures import write_session
import radio_loss_report as rlr  # noqa: E402

DT = 0.004


class RadioLossReportTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def test_anchor_specific_loss(self) -> None:
        folder = write_session(self.tmp / "s1", 2000,
                               range_mm=lambda row, aid: None if (aid == 3 and row % 2 == 0) else 3000.0)
        data = rlr.load_session(folder)
        self.assertEqual(len(data.tag_time_s), 2000)
        self.assertTrue(data.has_status_columns)
        stats = rlr.anchor_stats(data)
        self.assertAlmostEqual(stats[3].range_ok_given_polled_pct, 50.0, delta=0.5)
        self.assertAlmostEqual(stats[1].range_ok_given_polled_pct, 100.0, delta=0.01)
        hist = rlr.anchors_per_cycle_histogram(data)
        self.assertEqual(len(hist), 9)
        self.assertEqual(sum(hist), 2000)
        self.assertEqual((hist[7], hist[8]), (1000, 1000))
        pattern = rlr.classify_loss(data)
        self.assertEqual(pattern.pattern, "anchor_specific")
        self.assertEqual(pattern.worst_anchor_ids, (3,))

    def test_common_cause_loss(self) -> None:
        # All anchors drop together in 30 % of the cycles, in bursts of 10 cycles.
        folder = write_session(self.tmp / "s2", 3000,
                               range_mm=lambda row, aid: None if (row // 10) % 10 < 3 else 3000.0)
        pattern = rlr.classify_loss(rlr.load_session(folder))
        self.assertEqual(pattern.pattern, "common_cause")
        self.assertGreater(pattern.co_failure_lift, 2.0)

    def test_independent_loss(self) -> None:
        fail = np.random.default_rng(11).random((3000, 9)) < 0.3
        folder = write_session(self.tmp / "s3", 3000,
                               range_mm=lambda row, aid: None if fail[row, aid] else 3000.0)
        pattern = rlr.classify_loss(rlr.load_session(folder))
        self.assertEqual(pattern.pattern, "independent")
        self.assertLess(pattern.co_failure_lift, 1.2)

    def test_no_loss(self) -> None:
        folder = write_session(self.tmp / "s4", 500, range_mm=lambda row, aid: 3000.0)
        self.assertEqual(rlr.classify_loss(rlr.load_session(folder)).pattern, "none")

    def test_outage_polled_and_lost_cycles(self) -> None:
        folder = write_session(
            self.tmp / "s5", 2000,
            range_mm=lambda row, aid: None if (aid == 5 and 2.0 <= row * DT < 3.5) else 3000.0,
            polled=lambda row, aid: not (aid == 8 and row >= 1500),
            cycle_seq=lambda row: 1000 + row + (5 if row >= 1000 else 0),
        )
        data = rlr.load_session(folder)
        stats = rlr.anchor_stats(data, outage_threshold_s=0.2)
        self.assertEqual(stats[5].outages, 1)
        self.assertAlmostEqual(stats[5].longest_outage_s, 1.504, delta=0.01)
        self.assertEqual(stats[1].outages, 0)
        self.assertAlmostEqual(stats[8].polled_pct, 75.0, delta=0.1)
        self.assertAlmostEqual(stats[8].range_ok_given_polled_pct, 100.0, delta=0.01)
        self.assertEqual(rlr.lost_cycles(data), 5)

    def test_time_bins_follow_temperature(self) -> None:
        n = 5000  # 20 s
        # Failure probability grows with time while the TAG warms from 40 to 50 degC.
        fail = np.random.default_rng(5).random((n, 9)) < np.linspace(0.0, 0.6, n)[:, None]
        folder = write_session(self.tmp / "s6", n,
                               range_mm=lambda row, aid: None if fail[row, aid] else 3000.0,
                               temperature_c=lambda t_s: 40.0 + 0.5 * t_s)
        data = rlr.load_session(folder)
        bins = rlr.time_bins(data, bin_s=5.0)
        self.assertEqual(len(bins), 4)
        self.assertEqual(sum(b.cycles for b in bins), n)
        temps = [b.temperature_c for b in bins]
        self.assertTrue(all(t2 > t1 for t1, t2 in zip(temps, temps[1:])), temps)
        means = [b.mean_anchors_per_cycle for b in bins]
        self.assertTrue(all(m2 < m1 for m1, m2 in zip(means, means[1:])), means)
        self.assertAlmostEqual(bins[0].mean_period_ms, 4.0, delta=0.01)
        self.assertLess(rlr.temperature_correlation(bins), -0.9)

    def test_report_is_strict_json_and_cli_writes_files(self) -> None:
        folder = write_session(self.tmp / "s7", 1500,
                               range_mm=lambda row, aid: None if (aid == 3 and row % 2 == 0) else 3000.0)
        rep = rlr.build_report(folder)
        json.dumps(rep, allow_nan=False)
        for key in ("session", "cycles", "duration_s", "mean_anchors_per_cycle", "histogram",
                    "lost_cycles", "anchors", "bins", "pattern", "temperature_correlation",
                    "has_status_columns", "has_rx_quality"):
            self.assertIn(key, rep)
        self.assertEqual(rep["cycles"], 1500)
        self.assertAlmostEqual(rep["mean_anchors_per_cycle"], 7.5, delta=0.01)
        self.assertEqual(rep["pattern"]["pattern"], "anchor_specific")
        self.assertIn("3", rep["anchors"])

        out = self.tmp / "out"
        self.assertEqual(rlr.main([str(folder), "--out", str(out)]), 0)
        self.assertTrue((out / f"{folder.name}_radio_loss_report.json").is_file())
        self.assertTrue((out / f"{folder.name}_radio_loss_bins.csv").is_file())

    def test_session_without_rx_quality_still_reports(self) -> None:
        folder = write_session(self.tmp / "s8", 300, range_mm=lambda row, aid: 3000.0)
        (folder / "rx_quality.jsonl").unlink()
        rep = rlr.build_report(folder)
        self.assertFalse(rep["has_rx_quality"])
        self.assertIsNone(rep["anchors"]["1"]["resp_ok_given_polled_pct"])
        self.assertIsNone(rep["temperature_correlation"])


if __name__ == "__main__":
    unittest.main()
