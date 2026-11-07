"""G0.1 — joint calibration inside UwbStateEngine and the session metadata.

Hand-off contract: core_state.UwbStateEngine and serial_worker.FastCsvLogger.
"""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

from g0_fixtures import ROOM_54, SPOTS_4, TRUE_BIAS_MM, feed_spot_capture, feed_tape_capture
from core_state import UwbStateEngine  # noqa: E402
import multi_spot_calibration as msc  # noqa: E402


def _engine(entered: dict | None = None, spots=SPOTS_4) -> UwbStateEngine:
    engine = UwbStateEngine(ROOM_54)
    cycle = 1
    for j, true in enumerate(spots, start=1):
        cycle = feed_spot_capture(engine, true, (entered or {}).get(j, true), first_cycle=cycle)
    return engine


def _max_error(agg) -> float:
    return max(abs(agg[aid][0] - TRUE_BIAS_MM[aid]) for aid in TRUE_BIAS_MM)


# Two of four spots typed in with the same mistake (x measured from the wrong
# origin): the per-spot median can no longer reject them.
WRONG_2_AND_4 = {
    2: (SPOTS_4[1][0] - 0.65, SPOTS_4[1][1], SPOTS_4[1][2]),
    4: (SPOTS_4[3][0] - 0.60, SPOTS_4[3][1], SPOTS_4[3][2]),
}


class EngineJointCalibrationTests(unittest.TestCase):
    def test_default_method_is_joint_and_validated(self) -> None:
        engine = UwbStateEngine(ROOM_54)
        self.assertEqual(engine.calibration_method, "joint")
        engine.set_calibration_method("median")
        self.assertEqual(engine.calibration_method, "median")
        with self.assertRaises(ValueError):
            engine.set_calibration_method("mean")
        self.assertIsNone(engine.joint_calibration())  # no capture yet

    def test_joint_corrects_wrong_spots_where_median_fails(self) -> None:
        engine = _engine(WRONG_2_AND_4)
        self.assertEqual(engine.active_calibration_method(), "joint")
        self.assertLess(_max_error(engine.aggregated_calibration()), 40.0)   # reference: 23 mm
        engine.set_calibration_method("median")
        self.assertEqual(engine.active_calibration_method(), "median")
        self.assertGreater(_max_error(engine.aggregated_calibration()), 150.0)  # reference: 276 mm

    def test_single_spot_keeps_legacy_behaviour(self) -> None:
        engine = _engine(spots=SPOTS_4[:1])
        self.assertEqual(engine.joint_calibration().method, "single_spot_fallback")
        self.assertEqual(engine.active_calibration_method(), "median")
        agg = engine.aggregated_calibration()
        self.assertEqual(len(agg), 8)
        self.assertLess(_max_error(agg), 1.5)
        self.assertEqual(engine.calibration_warnings(), [])

    def test_joint_result_is_cached_and_invalidated(self) -> None:
        engine = _engine()
        r1 = engine.joint_calibration()
        self.assertIs(engine.joint_calibration(), r1)
        self.assertTrue(engine.remove_capture(2))
        self.assertFalse(engine.remove_capture(99))
        r2 = engine.joint_calibration()
        self.assertIsNot(r2, r1)
        self.assertEqual(sorted(s.spot_index for s in r2.spots), [1, 3, 4])
        engine.solver_dimension = 2
        self.assertEqual(engine.joint_calibration().dimension, 2)
        engine.solver_dimension = 3
        r3 = engine.joint_calibration()
        moved = [replace(a) for a in engine.anchors.values()]
        moved[0].x += 0.01
        engine.set_anchors(moved)
        self.assertIsNot(engine.joint_calibration(), r3)

    def test_wrong_spot_warning_reaches_calibration_warnings(self) -> None:
        engine = _engine({2: WRONG_2_AND_4[2]})
        warnings = engine.calibration_warnings()
        self.assertTrue(any(w.startswith("Joint solve:") and "Spot #2" in w for w in warnings), warnings)
        engine.set_calibration_method("median")
        self.assertFalse(any(w.startswith("Joint solve:") for w in engine.calibration_warnings()))

    def test_apply_records_method_and_version(self) -> None:
        engine = _engine({2: WRONG_2_AND_4[2]})
        applied = engine.apply_calibration_to_host()
        self.assertEqual(sorted(applied), list(range(1, 9)))
        self.assertAlmostEqual(engine.anchors[4].bias_mm, TRUE_BIAS_MM[4], delta=30.0)
        info = engine.last_applied_calibration
        self.assertEqual(info["method"], "joint")
        self.assertEqual(info["version"], msc.SOLVER_VERSION)
        self.assertEqual(sorted(info["biases_mm"]), [str(a) for a in range(1, 9)])
        self.assertEqual(info["spots"], 4)
        json.dumps(info, allow_nan=False)

    def test_tape_and_spot_captures_combine(self) -> None:
        engine = _engine(spots=SPOTS_4[:2])
        feed_tape_capture(engine, 1, 2.0, first_cycle=500)
        self.assertEqual(engine.active_calibration_method(), "joint")
        agg = engine.aggregated_calibration()
        self.assertEqual(agg[1][2], 3)   # two spots + one tape capture
        self.assertEqual(agg[2][2], 2)
        self.assertLess(_max_error(agg), 30.0)

    def test_calibration_summary_is_strict_json(self) -> None:
        engine = _engine()
        summary = engine.calibration_summary()
        json.dumps(summary, allow_nan=False)
        self.assertEqual(summary["method_selected"], "joint")
        self.assertEqual(summary["method_applied"], "joint")
        self.assertEqual(summary["joint"]["version"], msc.SOLVER_VERSION)
        self.assertEqual(len(summary["aggregated"]), 8)

    def test_session_metadata_records_applied_calibration(self) -> None:
        from serial_worker import FastCsvLogger

        engine = _engine()
        engine.apply_calibration_to_host()
        with tempfile.TemporaryDirectory() as tmp:
            logger = FastCsvLogger(Path(tmp), "G0", engine=engine)
            logger.stop()
            meta = json.loads((logger.session_dir / "session_metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(meta["calibration"]["method"], "joint")
        self.assertEqual(meta["calibration"]["version"], msc.SOLVER_VERSION)


if __name__ == "__main__":
    unittest.main()
