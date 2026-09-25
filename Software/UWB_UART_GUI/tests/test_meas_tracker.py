"""RANGE_MEAS tracker: rate, loss, noise, NLOS and plot history (no Tk)."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from meas_tracker import (  # noqa: E402
    RangeMeasTracker,
    anchor_nlos_db,
    difference_noise_mm,
    power_dbm,
)
from telemetry_protocol import (  # noqa: E402
    INT16_MIN,
    MEAS_FLAG_CAL_OK,
    MEAS_FLAG_FILTER_OK,
    MEAS_FLAG_RADIO_OK,
    RangeMeasMessage,
)

BASE = RangeMeasMessage(
    sequence=0, time_ms=0, boot_id=0x1234, meas_seq=0, meas_time_us=0, anchor_id=1,
    txn=0, mode=0, flags=MEAS_FLAG_RADIO_OK | MEAS_FLAG_CAL_OK | MEAS_FLAG_FILTER_OK,
    status=0, raw_mm=2000, corrected_mm=1980, filtered_mm=1985, fp_cdbm=-8000,
    rx_cdbm=-7700, anchor_fp_cdbm=-8100, anchor_rx_cdbm=-7600, std_noise=30,
    fp_index=746 * 64, ci_ppm_x100=120, slot_us=2200,
)


def meas(seq: int, anchor_id: int = 1, **fields) -> RangeMeasMessage:
    fields.setdefault("meas_time_us", seq * 20_000)
    return replace(BASE, sequence=seq, meas_seq=seq, anchor_id=anchor_id, **fields)


class RangeMeasTrackerTests(unittest.TestCase):
    def test_rate_counts_records_per_anchor_over_the_window(self) -> None:
        tracker = RangeMeasTracker()
        # Two anchors, 50 records/s each, for 3 s of host time.
        for index in range(300):
            tracker.ingest(meas(index, anchor_id=1 + index % 2), index * 0.01)
        summaries = tracker.summaries(2.99)
        self.assertEqual([item.anchor_id for item in summaries], [1, 2])
        for item in summaries:
            self.assertAlmostEqual(item.rate_hz, 50.0, delta=1.0)
            self.assertEqual(item.received, 150)
            self.assertFalse(item.stale)
        self.assertAlmostEqual(tracker.total_rate(summaries), 100.0, delta=2.0)
        self.assertEqual(tracker.lost, 0)

    def test_rate_needs_half_a_second_and_decays_when_records_stop(self) -> None:
        tracker = RangeMeasTracker()
        tracker.ingest(meas(1), 10.0)
        self.assertIsNone(tracker.summaries(10.2)[0].rate_hz)
        for index in range(2, 60):
            tracker.ingest(meas(index), 10.0 + index * 0.02)
        self.assertGreater(tracker.summaries(11.2)[0].rate_hz, 40.0)
        stale = tracker.summaries(15.0)[0]
        self.assertEqual(stale.rate_hz, 0.0)
        self.assertTrue(stale.stale)
        self.assertAlmostEqual(stale.age_s, 15.0 - (10.0 + 59 * 0.02))

    def test_meas_seq_gaps_count_lost_records_across_uint32_wrap(self) -> None:
        tracker = RangeMeasTracker()
        # 0xFFFFFFFF and 0 are lost across the wrap, then 3 and 4.
        for seq in (0xFFFFFFFD, 0xFFFFFFFE, 1, 2, 5, 6):
            tracker.ingest(meas(seq, meas_time_us=0), 1.0)
        self.assertEqual(tracker.lost, 2 + 2)
        self.assertEqual(tracker.total, 6)

    def test_re_enabling_range_meas_is_not_a_loss(self) -> None:
        tracker = RangeMeasTracker()
        tracker.ingest(meas(10), 1.0)
        tracker.restart_sequence()          # the TAG counted on while it was off
        tracker.ingest(meas(5000), 2.0)
        tracker.ingest(meas(5002), 2.1)
        self.assertEqual(tracker.lost, 1)

    def test_tag_reboot_restarts_sequence_and_plot_history(self) -> None:
        tracker = RangeMeasTracker()
        for seq in range(100, 110):
            tracker.ingest(meas(seq), seq * 0.02)
        # The new boot already counted meas_seq while RANGE_MEAS was off.
        tracker.ingest(meas(500, boot_id=0x9999), 3.0)
        tracker.ingest(meas(501, boot_id=0x9999), 3.02)
        self.assertEqual(tracker.lost, 0)
        self.assertEqual(tracker.restarts, 1)
        self.assertEqual(len(tracker.points(1, 60.0)), 2)

    def test_noise_ignores_steady_motion(self) -> None:
        ramp = [1000 + 20 * index for index in range(41)]      # 1 m/s at 50 Hz
        self.assertEqual(difference_noise_mm(ramp), 0.0)
        noisy = [value + (5 if index % 2 else -5) for index, value in enumerate(ramp)]
        # Alternating +/-5 mm: 40 differences, half 30 mm and half 10 mm.
        self.assertAlmostEqual(difference_noise_mm(noisy), 10.0 / math.sqrt(2.0))
        self.assertIsNone(difference_noise_mm(ramp[:5]))

        tracker = RangeMeasTracker()
        for index, value in enumerate(noisy):
            tracker.ingest(meas(index, raw_mm=value), index * 0.02)
        self.assertAlmostEqual(tracker.summaries(0.8)[0].noise_mm, 10.0 / math.sqrt(2.0))

    def test_nlos_mean_and_unknown_powers(self) -> None:
        tracker = RangeMeasTracker()
        tracker.ingest(meas(1, fp_cdbm=-9000, rx_cdbm=-7800), 1.0)      # 12 dB
        tracker.ingest(meas(2, fp_cdbm=-8000, rx_cdbm=-7800), 1.02)     # 2 dB
        tracker.ingest(meas(3, fp_cdbm=INT16_MIN, rx_cdbm=INT16_MIN), 1.04)
        summary = tracker.summaries(1.05)[0]
        self.assertAlmostEqual(summary.nlos_mean_db, 7.0)
        self.assertIsNone(tracker.points(1, 10.0)[-1].nlos_db)
        self.assertIsNone(power_dbm(INT16_MIN))
        self.assertEqual(power_dbm(-8123), -81.23)
        self.assertAlmostEqual(anchor_nlos_db(BASE), 5.0)
        self.assertIsNone(anchor_nlos_db(replace(BASE, anchor_fp_cdbm=INT16_MIN)))

    def test_plot_points_follow_flags_and_tag_time(self) -> None:
        tracker = RangeMeasTracker()
        for seq in range(1, 501):                                # 10 s at 50 Hz
            tracker.ingest(meas(seq), seq * 0.02)
        tracker.ingest(meas(501, flags=MEAS_FLAG_RADIO_OK), 10.1)
        points = tracker.points(1, 2.0)
        self.assertEqual(len(points), 101)                       # 2 s + the end point
        self.assertAlmostEqual(points[-1].tag_time_s - points[0].tag_time_s, 2.0)
        self.assertEqual((points[-2].corrected_mm, points[-2].filtered_mm), (1980, 1985))
        self.assertEqual((points[-1].corrected_mm, points[-1].filtered_mm), (None, None))
        self.assertEqual(tracker.points(7, 2.0), [])


if __name__ == "__main__":
    unittest.main()
