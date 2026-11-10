"""G0.1b — second solve pass that releases the prior of mistyped spots.

Hand-off contract for Plan/G0_1B_YEU_CAU_GIAI_LUOT_HAI.md. Do not change the
thresholds. Reference results (reviewer prototype) are noted next to them.
"""

from __future__ import annotations

from dataclasses import replace
import json
import math
import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from g0_fixtures import ROOM_54, SPOTS_4, TRUE_BIAS_MM, feed_spot_capture, spot_observation  # noqa: E402
from core_state import UwbStateEngine  # noqa: E402
import multi_spot_calibration as msc  # noqa: E402


def _off():
    """Single-pass settings (the G0.1 behaviour)."""
    return msc.JointCalibrationSettings(release_flagged_spot_priors=False)


WRONG = {
    2: (SPOTS_4[1][0] - 0.65, SPOTS_4[1][1], SPOTS_4[1][2]),   # x measured from the wrong origin
    3: (SPOTS_4[2][0], SPOTS_4[2][1] + 0.50, SPOTS_4[2][2]),
    4: (SPOTS_4[3][0] - 0.60, SPOTS_4[3][1], SPOTS_4[3][2]),
}


def _err(result) -> float:
    return max(abs(result.biases[aid].bias_mm - TRUE_BIAS_MM[aid]) for aid in TRUE_BIAS_MM)


def _spots(wrong=(), noise=True, keep=(1, 2, 3, 4)):
    return [spot_observation(j, true, WRONG[j] if j in wrong else true, noise=noise)
            for j, true in enumerate(SPOTS_4, start=1) if j in keep]


def _spot(result, index):
    return next(s for s in result.spots if s.spot_index == index)


class SolverPriorReleaseTests(unittest.TestCase):
    def test_settings_defaults(self) -> None:
        s = msc.JointCalibrationSettings()
        self.assertIs(s.release_flagged_spot_priors, True)
        self.assertEqual(s.max_released_fraction, 0.5)
        self.assertEqual(s.max_passes, 3)
        self.assertIn("MANY_SPOT_OFFSETS", msc.WARNING_CODES)

    def test_exact_data_with_mistyped_spot_is_recovered(self) -> None:
        obs = _spots(wrong=(2,), noise=False)
        released = msc.solve_joint_calibration(ROOM_54, obs)
        single = msc.solve_joint_calibration(ROOM_54, obs, settings=_off())
        self.assertGreater(_err(single), 5.0)          # one pass: 10.5 mm (prior still pulls)
        self.assertLess(_err(released), 1.0)           # reference: 0.02 mm
        self.assertEqual(released.passes, 2)
        self.assertEqual(single.passes, 1)
        self.assertEqual(released.released_spot_indices, (2,))
        self.assertEqual(single.released_spot_indices, ())
        s2 = _spot(released, 2)
        self.assertTrue(s2.prior_released)
        self.assertEqual([s.prior_released for s in released.spots], [False, True, False, False])
        self.assertLess(math.dist(s2.estimated_xyz, SPOTS_4[1]), 0.005)
        self.assertGreater(s2.offset_m, 0.6)
        self.assertEqual([w.spot_index for w in released.warnings if w.code == "SPOT_OFFSET"], [2])
        self.assertTrue(released.identifiable)

    def test_noisy_data_one_and_two_mistyped_spots(self) -> None:
        for wrong, limit in (((2,), 10.0), ((2, 4), 10.0)):        # reference: 4.0 and 4.5 mm
            obs = _spots(wrong=wrong)
            r = msc.solve_joint_calibration(ROOM_54, obs)
            single = msc.solve_joint_calibration(ROOM_54, obs, settings=_off())
            self.assertEqual(r.released_spot_indices, tuple(wrong))
            self.assertLess(_err(r), limit, wrong)
            self.assertLess(_err(r), _err(single), wrong)  # one pass: 14.5 and 27.7 mm

    def test_two_and_three_spot_captures(self) -> None:
        for keep, limit in (((1, 2), 15.0), ((1, 2, 3), 12.0)):   # reference: 6.1 and 5.2 mm
            r = msc.solve_joint_calibration(ROOM_54, _spots(wrong=(2,), keep=keep))
            self.assertEqual(r.released_spot_indices, (2,), keep)
            self.assertLess(_err(r), limit, keep)

    def test_clean_spots_need_one_pass_and_match_single_pass(self) -> None:
        obs = _spots()
        r = msc.solve_joint_calibration(ROOM_54, obs)
        single = msc.solve_joint_calibration(ROOM_54, obs, settings=_off())
        self.assertEqual((r.passes, r.released_spot_indices), (1, ()))
        for aid in TRUE_BIAS_MM:
            self.assertAlmostEqual(r.biases[aid].bias_mm, single.biases[aid].bias_mm, delta=1e-9)
        self.assertFalse(any(s.prior_released for s in r.spots))

    def test_majority_of_spots_offset_releases_nothing(self) -> None:
        r = msc.solve_joint_calibration(ROOM_54, _spots(wrong=(2, 3, 4)))
        self.assertEqual((r.passes, r.released_spot_indices), (1, ()))
        self.assertIn("MANY_SPOT_OFFSETS", r.warning_codes())
        self.assertEqual(sorted(w.spot_index for w in r.warnings if w.code == "SPOT_OFFSET"), [2, 3, 4])
        many = next(w for w in r.warnings if w.code == "MANY_SPOT_OFFSETS")
        self.assertIn("anchor coordinates", many.message)

    def test_wrong_rig_origin_is_not_hidden(self) -> None:
        # Every anchor typed 0.5 m off in x: all spots disagree with their typed coordinates.
        shifted = [replace(a, x=a.x + 0.5) for a in ROOM_54]
        r = msc.solve_joint_calibration(shifted, _spots())
        self.assertIn("MANY_SPOT_OFFSETS", r.warning_codes())
        self.assertEqual(r.released_spot_indices, ())

    def test_pass_and_fraction_limits(self) -> None:
        obs = _spots(wrong=(2,))
        one = msc.solve_joint_calibration(ROOM_54, obs, settings=msc.JointCalibrationSettings(max_passes=1))
        self.assertEqual((one.passes, one.released_spot_indices), (1, ()))
        self.assertNotIn("MANY_SPOT_OFFSETS", one.warning_codes())
        for bad in (dict(max_passes=0), dict(max_released_fraction=1.5), dict(max_released_fraction=-0.1),
                    dict(release_flagged_spot_priors="yes")):
            r = msc.solve_joint_calibration(ROOM_54, obs, settings=msc.JointCalibrationSettings(**bad))
            self.assertEqual(r.method, "failed", bad)
            self.assertIn("NOT_IDENTIFIABLE", r.warning_codes(), bad)

    def test_two_dimensional_release_keeps_heights(self) -> None:
        r = msc.solve_joint_calibration(ROOM_54, _spots(wrong=(2,)), dimension=2)
        self.assertEqual(r.released_spot_indices, (2,))
        self.assertTrue(all(s.estimated_xyz[2] == s.entered_xyz[2] for s in r.spots))
        self.assertLess(_err(r), 10.0)                 # reference: 3.7 mm

    def test_spot_with_only_outliers_has_no_rms(self) -> None:
        settings = msc.JointCalibrationSettings(residual_warn_sigma=1e-9)   # every row is an outlier
        r = msc.solve_joint_calibration(ROOM_54, _spots(), settings=settings)
        self.assertEqual([s.residual_rms_mm for s in r.spots], [None] * 4)
        self.assertIsNone(r.residual_rms_mm)
        d = r.to_dict()
        json.dumps(d, allow_nan=False)
        self.assertIsNone(d["spots"][0]["residual_rms_mm"])

    def test_serialization_includes_release_fields(self) -> None:
        d = msc.solve_joint_calibration(ROOM_54, _spots(wrong=(2,))).to_dict()
        json.dumps(d, allow_nan=False)
        self.assertEqual(d["passes"], 2)
        self.assertEqual(d["released_spot_indices"], [2])
        self.assertEqual([s["prior_released"] for s in d["spots"]], [False, True, False, False])
        self.assertIs(d["settings"]["release_flagged_spot_priors"], True)


def _engine(wrong=()) -> UwbStateEngine:
    engine = UwbStateEngine(ROOM_54)
    cycle = 1
    for j, true in enumerate(SPOTS_4, start=1):
        cycle = feed_spot_capture(engine, true, WRONG[j] if j in wrong else true, first_cycle=cycle)
    return engine


class EnginePriorReleaseTests(unittest.TestCase):
    def test_release_reaches_applied_biases_and_metadata(self) -> None:
        engine = _engine(wrong=(2,))
        agg = engine.aggregated_calibration()
        self.assertLess(max(abs(agg[a][0] - TRUE_BIAS_MM[a]) for a in TRUE_BIAS_MM), 5.0)  # reference: < 1 mm
        engine.apply_calibration_to_host()
        joint = engine.last_applied_calibration["joint"]
        self.assertEqual(joint["released_spot_indices"], [2])
        self.assertEqual(joint["passes"], 2)

    def test_many_offsets_warning_reaches_the_user(self) -> None:
        engine = _engine(wrong=(2, 3, 4))
        self.assertIn("MANY_SPOT_OFFSETS", engine.joint_calibration().warning_codes())
        warnings = engine.calibration_warnings()
        self.assertTrue(any(w.startswith("Joint solve:") and "anchor coordinates" in w for w in warnings), warnings)


class GuiPriorReleaseTests(unittest.TestCase):
    def setUp(self) -> None:
        from PyQt6.QtWidgets import QApplication
        from main_gui import UwbFastGuiWindow

        self.app = QApplication.instance() or QApplication([])
        self.win = UwbFastGuiWindow(start_demo=False)
        self.engine = self.win.engine
        self.engine.set_anchors(ROOM_54)

    def tearDown(self) -> None:
        self.win.close()

    def _feed(self, wrong=()) -> None:
        cycle = 1
        for j, true in enumerate(SPOTS_4, start=1):
            cycle = feed_spot_capture(self.engine, true, WRONG[j] if j in wrong else true, first_cycle=cycle)

    def _row_of(self, spot_index: int) -> int:
        table = self.win.cal_spot_table
        return next(r for r in range(table.rowCount()) if table.item(r, 0).text() == str(spot_index))

    def test_released_spot_is_shown(self) -> None:
        self._feed(wrong=(2,))
        self.win._update_calibration_page()
        status = self.win.lbl_joint_status.text()
        self.assertIn("rank", status.lower())
        self.assertIn("released", status.lower())
        self.assertIn("#2", status)
        offset_text = self.win.cal_spot_table.item(self._row_of(2), 3).text()
        self.assertIn("released", offset_text)
        self.assertNotIn("released", self.win.cal_spot_table.item(self._row_of(1), 3).text())

    def test_missing_residual_rms_is_shown_as_dashes(self) -> None:
        self._feed()
        self.engine.joint_settings = msc.JointCalibrationSettings(residual_warn_sigma=1e-9)
        self.win._update_calibration_page()
        table = self.win.cal_spot_table
        self.assertEqual([table.item(r, 4).text() for r in range(table.rowCount())], ["--"] * 4)


if __name__ == "__main__":
    unittest.main()
