"""E2.0-A — sensor-aided UWB range calibration (aided_calibration.py).

Hand-off contract for Plan/E2_0_YEU_CAU_HIEU_CHUAN_CO_CAM_BIEN_VA_REPLAY_ESKF_A.md.
Synthetic data: tests/e20_fixtures.py. Reference results (reviewer prototype)
are written next to the thresholds. Do not change the thresholds.
"""

from __future__ import annotations

import json
import math
import unittest

import numpy as np

import e20_fixtures as F
import aided_calibration as ac  # noqa: E402


def _fit(data, **kw):
    return ac.fit_aided_calibration(F.ROOM_48, data["ranges"], data["z_ref"], data["yaw"], data["xy_seed"],
                                    ac.AidedSettings(**kw))


def _bias_err_mm(cal):
    return max(abs(cal.corrections[i + 1].bias_m - F.TRUE_BIAS_M[i]) * 1000 for i in range(8))


class AidedCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.bias_only = F.survey(F.GRID_POINTS, F.HEADINGS_8, per=6, harmonics=False, seed=1)
        cls.heading = F.survey(F.GRID_POINTS, F.HEADINGS_8, per=6, harmonics=True, seed=2)
        cls.heading_test = F.survey(F.TEST_POINTS, F.HEADINGS_8, per=8, harmonics=True, seed=3)

    def test_constant_bias_is_recovered_with_height_fixed(self) -> None:
        cal, xy = _fit(self.bias_only)
        self.assertTrue(cal.ok)
        self.assertEqual(cal.version, ac.AIDED_VERSION)
        self.assertEqual((cal.harmonics_requested, cal.harmonics_used), (0, 0))
        self.assertLess(_bias_err_mm(cal), 12.0)                       # reference: 6.4 mm
        self.assertEqual(xy.shape, (len(self.bias_only["ranges"]), 2))
        self.assertLess(float(np.median(np.linalg.norm(xy - self.bias_only["xy_true"], axis=1))), 0.03)
        for c in cal.corrections.values():
            self.assertEqual((c.cos_m, c.sin_m), ((), ()))

    def test_heading_terms_improve_held_out_height(self) -> None:
        cal0, _ = _fit(self.heading, harmonics=0)
        cal1, _ = _fit(self.heading, harmonics=1)
        self.assertEqual(cal1.harmonics_used, 1)
        self.assertEqual(cal1.yaw_sectors, 8)
        self.assertLess(cal1.residual_rms_m, cal0.residual_rms_m)      # reference: 17.5 vs 36.7 mm
        for i in range(8):
            c = cal1.corrections[i + 1]
            self.assertLess(abs(c.cos_m[0] - F.TRUE_COS_M[i]), 0.025, i + 1)   # reference: <= 13.8 mm
            self.assertLess(abs(c.sin_m[0] - F.TRUE_SIN_M[i]), 0.025, i + 1)
        t = self.heading_test
        rep0, _, _ = ac.holdout_report(F.ROOM_48, cal0, t["ranges"], t["z_ref"], t["yaw"], t["xy_seed"])
        rep1, _, _ = ac.holdout_report(F.ROOM_48, cal1, t["ranges"], t["z_ref"], t["yaw"], t["xy_seed"])
        self.assertGreater(rep0["dz_median_before_m"], 0.10)            # reference: +14.2 cm without correction
        self.assertGreater(rep0["dz_p95_abs_before_m"], 0.18)           # reference: 22.4 cm
        self.assertLess(abs(rep0["dz_median_after_m"]), 0.02)           # reference: -0.6 cm
        self.assertLess(rep0["dz_p95_abs_after_m"], 0.14)               # reference: 9.4 cm
        self.assertLess(abs(rep1["dz_median_after_m"]), 0.02)           # reference: 0.0 cm
        self.assertLess(rep1["dz_p95_abs_after_m"], 0.08)               # reference: 5.4 cm
        self.assertLess(rep1["dz_p95_abs_after_m"], rep0["dz_p95_abs_after_m"])
        self.assertEqual(rep1["solved_after"], len(t["ranges"]))

    def test_two_headings_do_not_fit_heading_terms(self) -> None:
        # Test_1 had only two headings: heading terms would just fit position-dependent errors.
        d = F.survey(F.GRID_POINTS, (math.radians(-34.0), math.radians(64.5)), per=10, harmonics=True, seed=4)
        cal, _ = _fit(d, harmonics=1)
        self.assertTrue(cal.ok)
        self.assertEqual((cal.harmonics_requested, cal.harmonics_used, cal.yaw_sectors), (1, 0, 2))
        self.assertIn("HEADING_COVERAGE", cal.warning_codes())
        self.assertTrue(all(c.cos_m == () for c in cal.corrections.values()))

    def test_gross_outliers_are_down_weighted(self) -> None:
        d = F.survey(F.GRID_POINTS, F.HEADINGS_8, per=6, harmonics=False, outlier_frac=0.05, seed=5)
        cal, _ = _fit(d)
        self.assertLess(_bias_err_mm(cal), 12.0)                       # reference: 7.4 mm

    def test_invalid_and_insufficient_inputs(self) -> None:
        d = F.survey(F.GRID_POINTS[:1], F.HEADINGS_8[:2], per=10, harmonics=False, seed=6)   # 20 epochs
        cal, _ = _fit(d)
        self.assertFalse(cal.ok)
        self.assertIn("FEW_EPOCHS", cal.warning_codes())
        self.assertEqual(cal.corrections, {})
        nan_z = dict(self.bias_only, z_ref=np.full(len(self.bias_only["z_ref"]), np.nan))
        cal, _ = _fit(nan_z)
        self.assertFalse(cal.ok)
        with self.assertRaises(ValueError):
            _fit(self.bias_only, harmonics=3)

    def test_correction_and_serialization(self) -> None:
        truth = ac.AidedCalibration(
            ok=True, harmonics_requested=1, harmonics_used=1, yaw_sectors=8,
            corrections={i + 1: ac.AnchorCorrection(i + 1, float(F.TRUE_BIAS_M[i]), (float(F.TRUE_COS_M[i]),),
                                                    (float(F.TRUE_SIN_M[i]),), 8, 0, None) for i in range(8)},
            residual_rms_m=None, epochs=0, warnings=())
        d = F.survey(F.TEST_POINTS, F.HEADINGS_8, per=2, noise_m=0.0, harmonics=True, seed=7)
        corrected = ac.correct_ranges(truth, F.ROOM_48, d["ranges"], d["xy_true"], d["yaw"])
        geometric = np.linalg.norm(d["xyz_true"][:, None, :] - F.ANCHOR_XYZ[None], axis=2)
        self.assertLess(float(np.max(np.abs(corrected - geometric))), 1e-9)
        th = ac.body_bearing(F.ROOM_48, d["xy_true"], d["yaw"])
        self.assertLess(float(np.max(np.abs(th - F.body_bearing(d["xy_true"], d["yaw"])))), 1e-12)
        self.assertTrue(np.all((th >= -math.pi) & (th < math.pi)))
        cal, _ = _fit(self.heading, harmonics=1)
        text = json.dumps(cal.to_dict(), allow_nan=False)
        self.assertIn(ac.AIDED_VERSION, text)
        self.assertEqual(json.loads(text)["harmonics_used"], 1)


if __name__ == "__main__":
    unittest.main()
