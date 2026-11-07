"""G0.1 — joint multi-spot bias calibration solver (multi_spot_calibration.py).

These tests are the hand-off contract for G0.1. They must pass without
changing the thresholds. Synthetic data: tests/g0_fixtures.py.
"""

from __future__ import annotations

from dataclasses import replace
import json
import math
import unittest

from g0_fixtures import (
    ROOM_54,
    SPOTS_4,
    TRUE_BIAS_MM,
    feed_spot_capture,
    feed_tape_capture,
    spot_observation,
)
from core_state import UwbStateEngine  # noqa: E402
import multi_spot_calibration as msc  # noqa: E402


def _max_bias_error(result) -> float:
    return max(abs(result.biases[aid].bias_mm - TRUE_BIAS_MM[aid]) for aid in TRUE_BIAS_MM)


def _spots(entered: dict | None = None, extra: dict | None = None, noise: bool = True):
    return [
        spot_observation(j, true, (entered or {}).get(j, true), extra_mm=(extra or {}).get(j), noise=noise)
        for j, true in enumerate(SPOTS_4, start=1)
    ]


def _spot(result, index: int):
    return next(s for s in result.spots if s.spot_index == index)


class JointSolverTests(unittest.TestCase):
    def test_correct_spots_recover_biases(self) -> None:
        r = msc.solve_joint_calibration(ROOM_54, _spots())
        self.assertTrue(r.ok)
        self.assertEqual(r.method, "joint")
        self.assertEqual(r.version, msc.SOLVER_VERSION)
        self.assertTrue(r.identifiable)
        self.assertEqual((r.rank, r.unknowns), (20, 20))
        self.assertLess(_max_bias_error(r), 15.0)
        self.assertNotIn("SPOT_OFFSET", r.warning_codes())
        for s in r.spots:
            self.assertLess(s.offset_m, 0.03, s.spot_index)
        self.assertLess(r.residual_rms_mm, 10.0)
        for b in r.biases.values():
            self.assertTrue(0.5 < b.std_mm < 40.0, (b.anchor_id, b.std_mm))
            self.assertEqual(b.spots_used, 4)

    def test_wrong_spot_is_located_and_does_not_corrupt_biases(self) -> None:
        # 07/11: the TAG stood 0.65 m from the coordinates typed into the GUI.
        true2 = SPOTS_4[1]
        entered2 = (true2[0] - 0.65, true2[1], true2[2])
        r = msc.solve_joint_calibration(ROOM_54, _spots({2: entered2}))
        self.assertEqual(r.method, "joint")
        self.assertLess(_max_bias_error(r), 25.0)
        s2 = _spot(r, 2)
        self.assertLess(math.dist(s2.estimated_xyz, true2), 0.05)
        self.assertGreater(s2.offset_m, 0.5)
        offsets = [w.spot_index for w in r.warnings if w.code == "SPOT_OFFSET"]
        self.assertEqual(offsets, [2])

    def test_nlos_anchor_at_one_spot_is_flagged_not_absorbed(self) -> None:
        r = msc.solve_joint_calibration(ROOM_54, _spots(extra={3: {6: 400.0}}))
        self.assertIn(6, _spot(r, 3).outlier_anchor_ids)
        self.assertTrue(any(w.code == "RESIDUAL_OUTLIER" and w.spot_index == 3 and w.anchor_id == 6
                            for w in r.warnings))
        self.assertNotIn(3, [w.spot_index for w in r.warnings if w.code == "SPOT_OFFSET"])
        self.assertLess(_max_bias_error(r), 30.0)

    def test_single_spot_falls_back_to_per_spot_bias(self) -> None:
        spot = spot_observation(1, SPOTS_4[0], SPOTS_4[0], noise=False)
        r = msc.solve_joint_calibration(ROOM_54, [spot])
        self.assertTrue(r.ok)
        self.assertEqual(r.method, "single_spot_fallback")
        self.assertFalse(r.identifiable)
        self.assertIn("SINGLE_SPOT", r.warning_codes())
        self.assertEqual((r.rank, r.unknowns), (8, 11))
        for aid, b in TRUE_BIAS_MM.items():
            self.assertAlmostEqual(r.biases[aid].bias_mm, b, delta=0.01)

    def test_identifiability_needs_two_spots(self) -> None:
        spots = _spots(noise=False)
        expected = {1: (8, 11), 2: (14, 14), 3: (17, 17), 4: (20, 20)}
        for k, want in expected.items():
            rank, unknowns, cond = msc.data_identifiability(ROOM_54, spots[:k], dimension=3)
            self.assertEqual((rank, unknowns), want, k)
            self.assertEqual(math.isfinite(cond), k >= 2, (k, cond))
        self.assertEqual(msc.data_identifiability(ROOM_54, spots[:1], dimension=2)[:2], (8, 10))
        self.assertEqual(msc.data_identifiability(ROOM_54, spots[:2], dimension=2)[:2], (12, 12))

    def test_two_dimensional_mode_keeps_entered_height(self) -> None:
        r = msc.solve_joint_calibration(ROOM_54, _spots(), dimension=2)
        self.assertEqual(r.method, "joint")
        self.assertEqual(r.dimension, 2)
        for s in r.spots:
            self.assertEqual(s.estimated_xyz[2], s.entered_xyz[2])
        self.assertLess(_max_bias_error(r), 15.0)

    def test_flat_spot_heights_are_reported(self) -> None:
        flat = ((1.35, 0.90, 1.00), (4.05, 0.90, 1.00), (4.05, 2.70, 1.00))
        r = msc.solve_joint_calibration(ROOM_54, [spot_observation(j, p, p) for j, p in enumerate(flat, 1)])
        self.assertTrue(r.identifiable)
        self.assertIn("FLAT_HEIGHTS", r.warning_codes())

    def test_tape_capture_supplies_anchor_missing_from_spots(self) -> None:
        spots = []
        for s in _spots():
            keep = {aid: v for aid, v in s.median_raw_mm.items() if aid != 4}
            spots.append(replace(s, median_raw_mm=keep,
                                 sigma_mm={a: s.sigma_mm[a] for a in keep}, samples={a: s.samples[a] for a in keep}))
        tape = msc.TapeObservation(capture_index=9, anchor_id=4, reference_mm=2000.0,
                                   median_raw_mm=2000.0 + TRUE_BIAS_MM[4], sigma_mm=10.0, samples=100)
        r = msc.solve_joint_calibration(ROOM_54, spots, [tape])
        self.assertEqual(r.method, "joint")
        self.assertAlmostEqual(r.biases[4].bias_mm, TRUE_BIAS_MM[4], delta=0.5)
        self.assertEqual((r.biases[4].spots_used, r.biases[4].tape_used), (0, 1))
        self.assertEqual(r.biases[1].tape_used, 0)

    def test_degenerate_inputs_do_not_raise(self) -> None:
        r = msc.solve_joint_calibration(ROOM_54, [])
        self.assertFalse(r.ok)
        self.assertEqual(r.method, "failed")
        self.assertIn("NO_DATA", r.warning_codes())
        self.assertEqual(r.biases, {})

        few = spot_observation(5, SPOTS_4[0], SPOTS_4[0])
        few = replace(few, median_raw_mm={a: few.median_raw_mm[a] for a in (1, 2, 3)})
        r = msc.solve_joint_calibration(ROOM_54, _spots()[:2] + [few])
        self.assertTrue(any(w.code == "SPOT_FEW_ANCHORS" and w.spot_index == 5 for w in r.warnings))
        self.assertEqual(sorted(s.spot_index for s in r.spots), [1, 2])

        with self.assertRaises(ValueError):
            msc.solve_joint_calibration(ROOM_54, _spots(), dimension=4)

    def test_result_serializes_to_strict_json(self) -> None:
        entered2 = (SPOTS_4[1][0] - 0.65, SPOTS_4[1][1], SPOTS_4[1][2])
        r = msc.solve_joint_calibration(ROOM_54, _spots({2: entered2}))
        d = r.to_dict()
        text = json.dumps(d, allow_nan=False)
        self.assertIn(msc.SOLVER_VERSION, text)
        self.assertEqual(d["method"], "joint")
        self.assertEqual(len(d["spots"]), 4)
        self.assertIsInstance(d["biases"]["1"]["bias_mm"], float)
        self.assertTrue(any(w["code"] == "SPOT_OFFSET" for w in d["warnings"]))
        # A failed result must serialize as well.
        json.dumps(msc.solve_joint_calibration(ROOM_54, []).to_dict(), allow_nan=False)

    def test_suggested_spots_are_inside_rig_and_identifiable(self) -> None:
        pts = msc.suggest_calibration_spots(ROOM_54, count=4)
        self.assertEqual(len(pts), 4)
        for x, y, z in pts:
            self.assertTrue(0.5 <= x <= 4.9 and 0.5 <= y <= 3.1, (x, y))
            self.assertTrue(0.6 <= z <= 2.1, z)
        heights = sorted({round(p[2], 2) for p in pts})
        self.assertEqual(len(heights), 2)
        self.assertGreaterEqual(heights[1] - heights[0], 0.5)
        obs = [spot_observation(j, p, p, noise=False) for j, p in enumerate(pts, 1)]
        rank, unknowns, _ = msc.data_identifiability(ROOM_54, obs[:2])
        self.assertEqual(rank, unknowns)  # the first two suggestions alone are enough
        with self.assertRaises(ValueError):
            msc.suggest_calibration_spots(ROOM_54, count=1)


class CaptureConversionTests(unittest.TestCase):
    def test_observations_from_captures_uses_usable_results_only(self) -> None:
        engine = UwbStateEngine(ROOM_54)
        cycle = feed_spot_capture(engine, SPOTS_4[0], SPOTS_4[0])
        cycle = feed_spot_capture(engine, SPOTS_4[1], SPOTS_4[1], first_cycle=cycle)
        feed_tape_capture(engine, 2, 1.5, first_cycle=cycle)
        self.assertEqual(len(engine.spot_captures), 3)

        # Mark A5 of the first capture LOW_SAMPLES: it must not be used.
        first = engine.spot_captures[0]
        low = dict(first.anchor_results)
        low[5] = replace(low[5], quality="LOW_SAMPLES")
        engine.spot_captures[0] = replace(first, anchor_results=low)

        spots, tapes = msc.observations_from_captures(engine.spot_captures, enabled_anchor_ids=range(1, 8))
        self.assertEqual([s.spot_index for s in spots], [1, 2])
        self.assertEqual(spots[0].entered_xyz, tuple(SPOTS_4[0]))
        self.assertNotIn(5, spots[0].median_raw_mm)
        self.assertIn(5, spots[1].median_raw_mm)
        self.assertNotIn(8, spots[1].median_raw_mm)          # disabled anchor
        self.assertGreaterEqual(spots[1].samples[1], 20)
        self.assertEqual(len(tapes), 1)
        self.assertEqual(tapes[0].anchor_id, 2)
        self.assertAlmostEqual(tapes[0].reference_mm, 1500.0)
        self.assertAlmostEqual(tapes[0].median_raw_mm - tapes[0].reference_mm, TRUE_BIAS_MM[2], delta=1.0)


if __name__ == "__main__":
    unittest.main()
