"""E2.1-A — in-flight IMU noise estimation and parameter recommendation (eskf_tuning.py).

Hand-off contract for Plan/E2_1_YEU_CAU_TINH_CHINH_ESKF_THEO_RUNG_KHI_BAY.md (section 3).
Synthetic data: tests/e21_fixtures.py. Reviewer reference results are noted next to thresholds.
"""

from __future__ import annotations

import math
import unittest

import numpy as np

import e21_fixtures as F
import eskf_replay as er
import eskf_tuning as tu  # noqa: E402


def _hover(imu_hz, vibrating=True, seed=1):
    sim = F.simulate_flight(duration_s=30.0, hover_s=40.0, imu_hz=imu_hz, seed=seed)   # hovers the whole time
    return F.add_vibration(sim, seed=seed + 4) if vibrating else sim


def _estimate(sim, intervals=((1.0, 29.0),), **kw):
    return tu.estimate_imu_noise(sim["imu_t"], sim["imu_acc"], sim["imu_gyro"], list(intervals), **kw)


class ImuNoiseTests(unittest.TestCase):
    def test_recovers_vibration_std_at_200_and_50_hz(self) -> None:
        for hz in (200.0, 50.0):
            with self.subTest(imu_hz=hz):
                noise = _estimate(_hover(hz))
                self.assertEqual(noise.version, tu.TUNING_VERSION)
                self.assertAlmostEqual(noise.stream_hz, hz, delta=0.01 * hz)
                ratio_acc = np.asarray(noise.acc_std) / np.asarray(F.VIB_ACC_STD)
                ratio_gyro = np.degrees(noise.gyro_std) / np.asarray(F.VIB_GYRO_STD_DPS)
                self.assertTrue(np.all(np.abs(ratio_acc - 1) < 0.10), ratio_acc)        # reference: within 4 %
                self.assertTrue(np.all(np.abs(ratio_gyro - 1) < 0.10), ratio_gyro)
                # density = worst-axis per-sample std / sqrt(stream rate)
                self.assertAlmostEqual(noise.acc_density, max(noise.acc_std) / math.sqrt(noise.stream_hz),
                                       delta=1e-9 + 0.01 * noise.acc_density)
                self.assertAlmostEqual(noise.gyro_density, max(noise.gyro_std) / math.sqrt(noise.stream_hz),
                                       delta=1e-12 + 0.01 * noise.gyro_density)
                self.assertIn("VIBRATION_HIGH", noise.warnings)
                self.assertGreater(noise.samples, 1000 if hz == 200 else 1300)
                self.assertGreater(len(noise.acc_peaks_hz), 0)
                self.assertTrue(all(2.0 <= f <= hz / 2 for f in noise.acc_peaks_hz))

    def test_clean_imu_is_not_flagged(self) -> None:
        noise = _estimate(_hover(200.0, vibrating=False))
        self.assertTrue(np.allclose(noise.acc_std, 0.03, rtol=0.15), noise.acc_std)       # e20 default noise
        self.assertTrue(np.allclose(noise.gyro_std, 0.002, rtol=0.15), noise.gyro_std)
        self.assertNotIn("VIBRATION_HIGH", noise.warnings)

    def test_slow_flight_motion_is_not_counted_as_noise(self) -> None:
        hover = _estimate(_hover(50.0))
        circle = _estimate(F.add_vibration(F.simulate_flight(duration_s=30.0, imu_hz=50.0, seed=1), seed=5))
        self.assertLess(abs(circle.acc_density / hover.acc_density - 1), 0.05)
        self.assertLess(abs(circle.gyro_density / hover.gyro_density - 1), 0.05)
        self.assertGreater(float(np.linalg.norm(circle.acc_trend_std)), float(np.linalg.norm(hover.acc_trend_std)) + 0.05)  # motion -> trend

    def test_decimate_stream_pick_and_mean(self) -> None:
        sim = _hover(200.0)
        t, acc, gyro = sim["imu_t"], sim["imu_acc"], sim["imu_gyro"]
        tp, ap, gp = tu.decimate_stream(t, acc, gyro, 50.0, "pick")
        self.assertLess(abs(len(tp) - 50 * (t[-1] - t[0])), 2)
        self.assertTrue(np.all(np.isin(tp, t)))                                          # picked samples, unchanged
        k = np.searchsorted(t, tp)
        self.assertTrue(np.array_equal(ap, acc[k]) and np.array_equal(gp, gyro[k]))
        tm, am, gm = tu.decimate_stream(t, acc, gyro, 50.0, "mean")
        self.assertEqual(len(tm), len(tp))
        self.assertTrue(np.all(np.diff(tm) > 0))
        white = np.random.default_rng(3).normal(0.0, 1.0, (len(t), 3))
        _, wm, _ = tu.decimate_stream(t, white, white, 50.0, "mean")
        self.assertTrue(np.allclose(wm.std(0), 0.5, rtol=0.1))                          # 4 samples averaged
        with self.assertRaises(ValueError):
            tu.decimate_stream(t, acc, gyro, 50.0, "median")
        with self.assertRaises(ValueError):
            tu.decimate_stream(t, acc, gyro, 0.0, "pick")

    def test_recommendation_scales_with_the_stream_actually_used(self) -> None:
        sim = _hover(200.0)
        full = _estimate(sim)
        t50, a50, g50 = tu.decimate_stream(sim["imu_t"], sim["imu_acc"], sim["imu_gyro"], 50.0, "pick")
        picked = tu.estimate_imu_noise(t50, a50, g50, [(1.0, 29.0)])
        p_full, p_pick = tu.recommend_params(full), tu.recommend_params(picked)
        self.assertAlmostEqual(p_pick.na / p_full.na, 2.0, delta=0.2)                     # same std, 4x fewer samples
        self.assertAlmostEqual(p_pick.ng / p_full.ng, 2.0, delta=0.2)
        self.assertAlmostEqual(p_full.na, full.acc_density, delta=1e-12)
        self.assertEqual(p_full.min_anchor_interval_s, 0.02)
        self.assertTrue(p_full.reinit_keep_attitude)
        base = er.EskfParams()
        for name in ("sig_r", "gate", "k_pos", "k_neg", "nba", "nbg", "init_static_s", "reinit_after_rejects"):
            self.assertEqual(getattr(p_full, name), getattr(base, name), name)
        wide = tu.recommend_params(full, margin=1.5, min_anchor_interval_s=0.05, reinit_keep_attitude=False)
        self.assertAlmostEqual(wide.na, 1.5 * full.acc_density, delta=1e-12)
        self.assertEqual((wide.min_anchor_interval_s, wide.reinit_keep_attitude), (0.05, False))
        quiet = tu.recommend_params(_estimate(_hover(200.0, vibrating=False)))
        self.assertGreaterEqual(quiet.na, base.na)                                        # never below the datasheet
        self.assertGreaterEqual(quiet.ng, base.ng)
        with self.assertRaises(ValueError):
            tu.recommend_params(full, margin=0.5)

    def test_intervals_gaps_and_invalid_samples(self) -> None:
        sim = _hover(200.0)
        t, acc, gyro = sim["imu_t"].copy(), sim["imu_acc"].copy(), sim["imu_gyro"].copy()
        acc[1000:1010] = np.nan                                                          # inside (1, 10)
        gyro[3000] = np.inf                                                              # inside (12, 20)
        noise = tu.estimate_imu_noise(t, acc, gyro, [(1.0, 10.0), (12.0, 20.0)])
        self.assertTrue(np.all(np.isfinite(noise.acc_std)) and np.all(np.isfinite(noise.gyro_std)))
        self.assertLess(noise.duration_s, 17.5)
        self.assertGreater(noise.duration_s, 16.0)
        # a 2 s hole must not be bridged by the detrending (no step artefact)
        hole = (t < 14.0) | (t > 16.0)
        holed = tu.estimate_imu_noise(t[hole], acc[hole], gyro[hole], [(1.0, 29.0)])
        self.assertTrue(np.allclose(holed.acc_std, noise.acc_std, rtol=0.1))
        empty = tu.estimate_imu_noise(t, acc, gyro, [(100.0, 200.0)])
        self.assertIn("TOO_FEW_SAMPLES", empty.warnings)
        self.assertTrue(math.isnan(empty.acc_density))
        with self.assertRaises(ValueError):
            tu.recommend_params(empty)
        json_like = noise.to_dict()
        self.assertEqual(json_like["version"], tu.TUNING_VERSION)
        self.assertEqual(len(json_like["acc_std"]), 3)


if __name__ == "__main__":
    unittest.main()
