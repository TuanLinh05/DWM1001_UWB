"""G0.3 — tools/static_dataset_report.py: statistics of the standard dataset.

Hand-off contract. A session holds static segments at surveyed points, marked
in the GUI with waypoint labels "<point>_start" / "<point>_end".
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from g0_fixtures import ROOM_54, SPOTS_4, TRUE_BIAS_MM, ar1_noise, true_range_mm, write_session
import static_dataset_report as sdr  # noqa: E402

DT = 0.004
P1, P2 = SPOTS_4[0], SPOTS_4[1]
# Rows: P1 static 0..7500 (30 s), walk 7500..7600, P2 static 7600..15100 (30 s).
LABELS = {0: "P1_start", 7500: "P1_end", 7600: "P2_start", 15100: "P2_end"}
N_ROWS = 15101
TAU_S = 0.4
SIGMA_MM = 20.0


def _where(row: int):
    if row <= 7500:
        return P1
    if row >= 7600:
        return P2
    f = (row - 7500) / 100.0
    return tuple(a + f * (b - a) for a, b in zip(P1, P2))


def _write(folder: Path, host_bias=None) -> Path:
    noise = {a.anchor_id: ar1_noise(N_ROWS, TAU_S, DT, SIGMA_MM, seed=100 + a.anchor_id) for a in ROOM_54}
    anchors = {a.anchor_id: a for a in ROOM_54}
    return write_session(
        folder, N_ROWS,
        range_mm=lambda row, aid: true_range_mm(_where(row), anchors[aid]) + TRUE_BIAS_MM[aid] + noise[aid][row],
        position=_where,
        labels=LABELS,
        host_bias_mm=host_bias or TRUE_BIAS_MM,
    )


MANIFEST = {"points": {"P1": list(P1), "P2": list(P2)}, "trim_s": 2.0}


class StaticDatasetReportTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def test_autocorrelation_time(self) -> None:
        x = ar1_noise(15000, TAU_S, DT, 20.0, seed=3)
        self.assertAlmostEqual(sdr.autocorrelation_time(x, DT), TAU_S, delta=0.25 * TAU_S)
        white = np.random.default_rng(4).normal(0.0, 1.0, 5000)
        self.assertLess(sdr.autocorrelation_time(white, DT), 2 * DT)
        self.assertEqual(sdr.autocorrelation_time(np.full(100, 7.0), DT), 0.0)

    def test_manifest_validation(self) -> None:
        path = self.tmp / "m.json"
        path.write_text(json.dumps(MANIFEST), encoding="utf-8")
        self.assertEqual(sdr.load_manifest(path)["trim_s"], 2.0)
        path.write_text(json.dumps({"points": {"P1": [1, 2]}}), encoding="utf-8")
        with self.assertRaises(ValueError):
            sdr.load_manifest(path)

    def test_find_segments_with_trim(self) -> None:
        folder = _write(self.tmp / "s1")
        rows = sdr.load_rows(folder)
        segs = sdr.find_segments(rows, MANIFEST)
        self.assertEqual([s.point for s in segs], ["P1", "P2"])
        self.assertEqual((segs[0].start_row, segs[0].end_row), (500, 7000))
        self.assertEqual((segs[1].start_row, segs[1].end_row), (8100, 14600))
        self.assertAlmostEqual(segs[0].t_end_s - segs[0].t_start_s, 26.0, delta=0.01)
        with self.assertRaises(ValueError) as ctx:
            sdr.find_segments(rows, {"points": {"P9": [1.0, 1.0, 1.0]}})
        self.assertIn("P9_start", str(ctx.exception))

    def test_segment_statistics(self) -> None:
        folder = _write(self.tmp / "s2")
        rows = sdr.load_rows(folder)
        anchors = sdr.anchors_from_metadata(folder)
        self.assertEqual(sorted(anchors), list(range(1, 9)))
        seg = sdr.find_segments(rows, MANIFEST)[0]
        stats = sdr.segment_anchor_stats(rows, seg, anchors)
        for aid, st in stats.items():
            self.assertEqual(st.samples, 6501)
            self.assertAlmostEqual(st.success_pct, 100.0, delta=0.01)
            self.assertAlmostEqual(st.sigma_mm, SIGMA_MM, delta=6.0)
            self.assertAlmostEqual(st.tau_s, TAU_S, delta=0.4 * TAU_S)
            self.assertAlmostEqual(st.true_mm, true_range_mm(P1, ROOM_54[aid - 1]), delta=1e-6)
            self.assertAlmostEqual(st.raw_bias_mm, TRUE_BIAS_MM[aid], delta=15.0)
            self.assertAlmostEqual(st.residual_after_cal_mm, 0.0, delta=15.0)
        pos = sdr.segment_position_stats(rows, seg)
        self.assertEqual(pos.fixes, 6501)
        self.assertLess(pos.error_3d_m, 1e-3)

    def test_acceptance_and_cli(self) -> None:
        good = _write(self.tmp / "good")
        manifest_path = self.tmp / "manifest.json"
        manifest_path.write_text(json.dumps(MANIFEST), encoding="utf-8")
        rep = sdr.build_report(good, sdr.load_manifest(manifest_path))
        json.dumps(rep, allow_nan=False)
        self.assertTrue(rep["acceptance"]["pass"], rep["acceptance"])
        self.assertEqual(sdr.main([str(good), str(manifest_path)]), 0)
        self.assertTrue((good / "static_dataset_report.json").is_file())
        self.assertTrue((good / "static_dataset_anchors.csv").is_file())

        wrong = dict(TRUE_BIAS_MM)
        wrong[4] += 120.0   # host bias off by 120 mm for A4
        bad = _write(self.tmp / "bad", host_bias=wrong)
        rep = sdr.build_report(bad, MANIFEST)
        self.assertFalse(rep["acceptance"]["pass"])
        reasons = " ".join(rep["acceptance"]["points"]["P1"]["reasons"])
        self.assertIn("A4", reasons)
        self.assertEqual(sdr.main([str(bad), str(manifest_path)]), 1)


if __name__ == "__main__":
    unittest.main()
