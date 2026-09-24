import json
import math
import sys
import tempfile
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gui_analysis import (
    AnchorPosition,
    TelemetryPoint,
    analyze_anchor,
    calibration_result,
    load_layout,
    histogram_density,
    overlapping_allan_deviation,
    percentile,
    regularize_time_series,
    save_layout,
    signal_statistics,
    solve_position,
    welch_psd,
)


class GuiAnalysisTests(unittest.TestCase):
    def point(self, *, valid=True, age=5, raw=1000, host=990, fpp=-78.0):
        return TelemetryPoint(1.0, 20, valid, 0, age, raw, raw - 5, host, fpp)

    def test_percentile_and_anchor_quality(self):
        self.assertEqual(percentile([0, 10, 20], 0.5), 10)
        points = [
            self.point(raw=1000),
            self.point(valid=False),
            self.point(age=250),
        ]
        metrics = analyze_anchor(1, points, total_frames=5)
        self.assertEqual(metrics.fresh, 1)
        self.assertEqual(metrics.invalid, 1)
        self.assertEqual(metrics.stale, 1)
        self.assertEqual(metrics.missing, 2)
        self.assertAlmostEqual(metrics.availability_pct, 20.0)

    def test_calibration_reports_offset_noise_and_drift(self):
        result = calibration_result(
            2,
            2000,
            [self.point(raw=value, fpp=-75.0) for value in (1900, 1910, 1920, 1930, 1940)],
        )
        self.assertEqual(result.samples, 5)
        self.assertAlmostEqual(result.mean_raw_mm, 1920)
        self.assertAlmostEqual(result.offset_mm, 80)
        self.assertAlmostEqual(result.drift_mm, 40)
        self.assertEqual(result.median_fpp_dbm, -75.0)

    def test_layout_json_roundtrip_and_legacy_id(self):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as temporary:
            path = Path(temporary.name)
        try:
            original = (AnchorPosition(1, 1.0, 2.0, 3.0), AnchorPosition(2, 4.0, 5.0, 6.0), AnchorPosition(3, 7.0, 8.0, 9.0))
            save_layout(path, original)
            self.assertEqual(load_layout(path), original)
            path.write_text(json.dumps({"anchors": [{"id": i, "x": i, "y": 0} for i in range(1, 4)]}), encoding="utf-8")
            self.assertEqual([item.anchor_id for item in load_layout(path)], [1, 2, 3])
        finally:
            path.unlink(missing_ok=True)

    def test_solver_2d_and_3d(self):
        layout_2d = (
            AnchorPosition(1, 0, 0), AnchorPosition(2, 5, 0),
            AnchorPosition(3, 0, 4), AnchorPosition(4, 5, 4),
        )
        target_2d = (2.0, 1.5, 0.0)
        ranges_2d = {
            anchor.anchor_id: round(math.dist(target_2d, (anchor.x, anchor.y, anchor.z)) * 1000)
            for anchor in layout_2d
        }
        solution_2d = solve_position(layout_2d, ranges_2d, dimension=2)
        self.assertIsNotNone(solution_2d)
        self.assertAlmostEqual(solution_2d.x, target_2d[0], places=3)
        self.assertAlmostEqual(solution_2d.y, target_2d[1], places=3)

        layout_3d = (
            AnchorPosition(1, 0, 0, 0), AnchorPosition(2, 5, 0, 0),
            AnchorPosition(3, 0, 4, 0), AnchorPosition(4, 0, 0, 3),
            AnchorPosition(5, 5, 4, 3),
        )
        target_3d = (2.0, 1.5, 1.2)
        ranges_3d = {
            anchor.anchor_id: round(math.dist(target_3d, (anchor.x, anchor.y, anchor.z)) * 1000)
            for anchor in layout_3d
        }
        solution_3d = solve_position(layout_3d, ranges_3d, dimension=3)
        self.assertIsNotNone(solution_3d)
        self.assertAlmostEqual(solution_3d.x, target_3d[0], places=3)
        self.assertAlmostEqual(solution_3d.y, target_3d[1], places=3)
        self.assertAlmostEqual(solution_3d.z, target_3d[2], places=3)

    def test_solver_rejects_one_large_outlier(self):
        layout = (
            AnchorPosition(1, 0, 0), AnchorPosition(2, 5, 0),
            AnchorPosition(3, 0, 4), AnchorPosition(4, 5, 4),
            AnchorPosition(5, 2.5, -2),
        )
        target = (2.0, 1.5, 0.0)
        ranges = {
            anchor.anchor_id: round(math.dist(target, (anchor.x, anchor.y, anchor.z)) * 1000)
            for anchor in layout
        }
        ranges[5] += 2500
        solution = solve_position(layout, ranges, dimension=2)
        self.assertIsNotNone(solution)
        self.assertIn(5, solution.excluded_anchor_ids)
        self.assertAlmostEqual(solution.x, target[0], places=2)
        self.assertAlmostEqual(solution.y, target[1], places=2)

    def test_signal_statistics_reference_and_histogram_density(self):
        values = [990.0, 1000.0, 1010.0]
        stats = signal_statistics([0.0, 0.02, 0.04], values, reference_mm=1000.0)
        self.assertIsNotNone(stats)
        self.assertEqual(stats.sample_count, 3)
        self.assertAlmostEqual(stats.sample_rate_hz, 50.0)
        self.assertAlmostEqual(stats.bias_mm, 0.0)
        self.assertAlmostEqual(stats.mae_mm, 20.0 / 3.0)
        self.assertAlmostEqual(stats.rmse_mm, math.sqrt(200.0 / 3.0))
        histogram = histogram_density([value - 1000.0 for value in values], bins=3)
        self.assertEqual(len(histogram.centers), 3)
        self.assertAlmostEqual(
            sum(value * histogram.bin_width for value in histogram.density), 1.0
        )

    def test_histogram_auto_resolution_scales_for_live_capture(self):
        values = [
            120.0 * math.sin(index * 0.071) + 8.0 * math.sin(index * 0.53)
            for index in range(300)
        ]
        histogram = histogram_density(values)
        self.assertGreaterEqual(len(histogram.centers), 34)
        self.assertLessEqual(len(histogram.centers), 80)
        left_edge = histogram.centers[0] - histogram.bin_width / 2.0
        right_edge = histogram.centers[-1] + histogram.bin_width / 2.0
        self.assertLessEqual(left_edge, min(values) + 1e-9)
        self.assertGreaterEqual(right_edge + 1e-9, max(values))
        self.assertAlmostEqual(
            sum(value * histogram.bin_width for value in histogram.density), 1.0
        )

    def test_histogram_does_not_claim_more_bins_than_samples(self):
        histogram = histogram_density([-2.0, 0.0, 3.0])
        self.assertLessEqual(len(histogram.centers), 3)

    def test_regularization_does_not_bridge_large_gap(self):
        regular = regularize_time_series(
            [0.00, 0.02, 0.04, 2.00, 2.02, 2.04, 2.06, 2.08],
            [1, 1, 1, 2, 2, 2, 2, 2],
        )
        self.assertIsNotNone(regular)
        self.assertEqual(regular.gap_count, 1)
        self.assertEqual(regular.source_samples, 5)
        self.assertTrue(all(abs(value - 2.0) < 1e-9 for value in regular.values))

    def test_regularization_preserves_uniform_grid_and_splits_one_missing_sample(self):
        values = list(range(21))
        regular = regularize_time_series(
            [index * 0.02 for index in range(len(values))], values
        )
        self.assertIsNotNone(regular)
        self.assertEqual(len(regular.values), len(values))
        self.assertAlmostEqual(regular.values[0], values[0])
        self.assertAlmostEqual(regular.values[-1], values[-1])
        self.assertAlmostEqual(regular.sample_rate_hz, 50.0)

        split = regularize_time_series(
            [0.00, 0.02, 0.04, 0.08, 0.10, 0.12, 0.14],
            [1, 1, 1, 2, 2, 2, 2],
        )
        self.assertIsNotNone(split)
        self.assertEqual(split.gap_count, 1)
        self.assertEqual(split.source_samples, 4)
        self.assertTrue(all(abs(value - 2.0) < 1e-9 for value in split.values))

    def test_allan_deviation_for_constant_and_linear_signal(self):
        taus, deviations = overlapping_allan_deviation([5.0] * 128, 10.0)
        self.assertTrue(taus)
        self.assertTrue(all(value == 0.0 for value in deviations))
        rate = 20.0
        slope = 3.0
        signal = [slope * index / rate for index in range(512)]
        taus, deviations = overlapping_allan_deviation(signal, rate)
        self.assertTrue(taus)
        for tau, deviation in zip(taus[:5], deviations[:5]):
            self.assertAlmostEqual(deviation, slope * tau / math.sqrt(2.0), places=9)

    def test_welch_psd_finds_sine_frequency_and_power(self):
        rate = 64.0
        frequency = 8.0
        values = [math.sin(2.0 * math.pi * frequency * index / rate) for index in range(1024)]
        frequencies, density = welch_psd(values, rate, maximum_fft=256)
        self.assertTrue(frequencies)
        peak = max(range(1, len(density)), key=lambda index: density[index])
        self.assertAlmostEqual(frequencies[peak], frequency, places=9)
        bin_width = frequencies[1] - frequencies[0]
        self.assertAlmostEqual(sum(density) * bin_width, 0.5, delta=0.02)


if __name__ == "__main__":
    unittest.main()
