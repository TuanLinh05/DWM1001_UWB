"""E2.1 IO boundaries and estimator edge cases beyond the immutable contracts."""

from dataclasses import asdict, replace
from contextlib import redirect_stderr
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import e21_fixtures as F
import eskf_replay as er
import eskf_tuning as tuning
import eskf_tuning_report as report
import imu_noise_report
from flight_log_io import rig_attitude_rotations
from imu_noise_report import e21_output


class OfflineE21ToolsTests(unittest.TestCase):
    def test_parameter_file_rejects_unknown_missing_and_wrong_types(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve()
            document = dict(params=asdict(er.EskfParams()), provenance=dict(flight=str(root), windows_s=[[75., 195.]],
                            stream_hz=50., decimate="mean", measured_stream_hz=50., margin=1., version=tuning.TUNING_VERSION))
            path = root / "params.json"
            def load(value):
                path.write_text(json.dumps(value), encoding="utf-8")
                return report.load_params(path, root, [(75., 195.)], 50., "mean")
            self.assertEqual(load(document)[0], er.EskfParams())
            for key, value in (("unknown", 1), ("na", float('nan')), ("reinit_keep_attitude", 1),
                               ("init_min_anchors", 6.), ("att_sig_deg", [2, 2]), ("gate", "25")):
                with self.subTest(key=key), self.assertRaises(ValueError):
                    load({**document, "params": {**document["params"], key: value}})
            missing = dict(document["params"])
            missing.pop("ng")
            with self.assertRaises(ValueError):
                load({**document, "params": missing})
            with self.assertRaises(ValueError):
                load({**document, "extra": 1})

    def test_parameter_provenance_cannot_use_holdout_or_wrong_stream(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve()
            provenance = dict(flight=str(root), windows_s=[[75., 195.]], stream_hz=50., decimate="mean",
                              measured_stream_hz=50., margin=1., version=tuning.TUNING_VERSION)
            path = root / "params.json"
            for key, value in (("windows_s", [[198., 218.]]), ("decimate", "pick"), ("stream_hz", 100.), ("margin", .5)):
                path.write_text(json.dumps(dict(params=asdict(er.EskfParams()), provenance={**provenance, key: value})), encoding="utf-8")
                with self.subTest(key=key), self.assertRaises(ValueError):
                    report.load_params(path, root, [(75., 195.)], 50., "mean")

    def test_e21_writer_refuses_e20_and_paths_outside_analysis(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve()
            for path in (root / "analysis/e20_existing", root / "other/e21_output", root / "analysis"):
                with self.assertRaises(ValueError):
                    e21_output(root, path, "e21_test")
                self.assertFalse(path.exists())
            expected = root / "analysis/e21_test"
            self.assertEqual(e21_output(root, None, "e21_test"), expected)

    def test_rejected_attitude_preserves_every_state_and_covariance(self):
        params = er.EskfParams()
        f = er.ESKF(np.ones(3), np.eye(3), np.zeros(3), params)
        f.P = np.eye(15) * .001
        before = [getattr(f, key).copy() for key in ("p", "v", "R", "ba", "bg", "P")]
        accepted, nis = f.update_attitude(er.so3_exp([0, 0, math.pi/2]), params)
        self.assertFalse(accepted)
        self.assertGreater(nis, params.att_gate)
        for value, key in zip(before, ("p", "v", "R", "ba", "bg", "P")):
            np.testing.assert_array_equal(getattr(f, key), value)

    def test_attitude_joseph_injects_cross_correlated_position_and_bias(self):
        params = er.EskfParams()
        f = er.ESKF(np.zeros(3), np.eye(3), np.zeros(3), params)
        f.P = np.eye(15) * .1
        for axis in range(3):
            f.P[axis, axis+6] = f.P[axis+6, axis] = .02
            f.P[axis+12, axis+6] = f.P[axis+6, axis+12] = .01
        accepted, _ = f.update_attitude(er.so3_exp([.01, -.02, .03]), params)
        self.assertTrue(accepted)
        self.assertGreater(np.linalg.norm(f.p), 0)
        self.assertGreater(np.linalg.norm(f.bg), 0)
        np.testing.assert_allclose(f.P, f.P.T, atol=1e-15)
        self.assertGreater(np.linalg.eigvalsh(f.P).min(), 0)
        np.testing.assert_allclose(f.R.T @ f.R, np.eye(3), atol=1e-14)

    def test_sorting_invalid_attitudes_and_offered_interval(self):
        sim = F.simulate_flight(duration_s=1., hover_s=2., imu_hz=50., uwb_hz=100., seed=7)
        imu = [er.ImuSample(float(t), tuple(a), tuple(g)) for t, a, g in zip(sim["imu_t"], sim["imu_acc"], sim["imu_gyro"])]
        ranges = [er.RangeSample(t, aid, r) for t, aid, r in sim["ranges"]]
        R = sim["truth_R"][0]
        attitude = [er.AttitudeSample(.5, R @ er.so3_exp([0, 0, math.pi/2])), er.AttitudeSample(.52, R),
                    er.AttitudeSample(.6, R), er.AttitudeSample(.7, np.eye(3)*2),
                    er.AttitudeSample(.8, np.diag([1, 1, -1])), er.AttitudeSample(.9, np.full((3, 3), np.nan))]
        params = replace(er.EskfParams(), init_static_s=.1, att_min_interval_s=.1, min_anchor_interval_s=.02,
                         sig_th0_deg=(1., 1., 1.))
        a = er.run_eskf_replay(F.ROOM_48, imu, ranges, sim["lever"], sim["yaw0"], params, attitude)
        b = er.run_eskf_replay(F.ROOM_48, imu[::-1], ranges[::-1], sim["lever"], sim["yaw0"], params, attitude[::-1])
        np.testing.assert_array_equal(a.p, b.p)
        np.testing.assert_array_equal(a.update_t, b.update_t)
        np.testing.assert_allclose(a.att_update_t, [.5, .6])
        np.testing.assert_array_equal(a.att_accepted, [False, True])
        self.assertEqual(a.reinits, 0)

    def test_log_so3_at_pi_reconstructs_rotation(self):
        for axis in ([1, 0, 0], [0, -1, 0], [-1, 2, 3]):
            axis = np.asarray(axis, float)
            axis /= np.linalg.norm(axis)
            for angle in (math.pi, math.pi-1e-10, 1e-12):
                R = er.so3_exp(axis * angle)
                np.testing.assert_allclose(er.so3_exp(er.log_so3(R)), R, atol=1e-12)

    def test_rig_attitude_formula_yaw_and_frame_handedness(self):
        ned_frd = er.attitude_from_accel([.1, .2, 9.81], .7)
        offset = 1.1
        with patch("flight_log_io.attitude_rotations", return_value=np.array([ned_frd])):
            R = rig_attitude_rotations([0.], {}, {}, offset)[0]
        self.assertAlmostEqual(math.atan2(R[1, 0], R[0, 0]), offset-.7)
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-15)
        self.assertAlmostEqual(np.linalg.det(R), 1.)

    def test_noise_detrending_does_not_bridge_step_at_gap(self):
        t = np.r_[np.arange(0., 2., .01), np.arange(4., 6., .01)]
        acc = np.zeros((len(t), 3))
        acc[t >= 4] = 100
        noise = tuning.estimate_imu_noise(t, acc, np.zeros_like(acc))
        np.testing.assert_allclose(noise.acc_std, 0., atol=1e-10)
        self.assertLess(noise.duration_s, 4.)

    def test_decimation_is_order_independent_and_preserves_first_sample(self):
        t = np.arange(0., 1., .005)
        x = np.c_[t, t*2, t*3]
        for mode in ("pick", "mean"):
            a = tuning.decimate_stream(t, x, x, 50., mode)
            b = tuning.decimate_stream(t[::-1], x[::-1], x[::-1], 50., mode)
            for first, second in zip(a, b):
                np.testing.assert_array_equal(first, second)
        self.assertEqual(tuning.decimate_stream(t, x, x, 50.)[0][0], t[0])

    def test_missing_optional_dependency_fails_without_writing_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "flight.ulg").touch()
            error = io.StringIO()
            with patch.dict(sys.modules, {"pyulog": None}), redirect_stderr(error):
                code = imu_noise_report.main([str(root), "--windows", "75:195"])
            self.assertEqual(code, 2)
            self.assertIn("python -m pip install pyulog", error.getvalue())
            self.assertFalse((root / "analysis").exists())

    def test_cli_rejects_bad_params_before_loading_flight_or_replaying(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            params = root / "bad.json"
            params.write_text('{"unknown": 1}', encoding="utf-8")
            with patch.object(report, "load_flight") as loader, redirect_stderr(io.StringIO()):
                code = report.main([str(root), "--train", "75:195", "--test", "198:218,235:300",
                                    "--imu-stream-hz", "50", "--imu-decimate", "mean", "--params-json", str(params)])
            self.assertEqual(code, 2)
            loader.assert_not_called()
            self.assertFalse((root / "analysis").exists())

    def test_npz_roundtrip_preserves_range_and_attitude_times_in_both_variants(self):
        keys = ("t", "p", "v", "R", "tag", "p_std", "update_t", "update_anchor", "update_accepted", "update_nis",
                "update_innovation_m", "att_update_t", "att_accepted", "att_nis")
        results = {name: SimpleNamespace(**{key: np.array([i, k], dtype=float) for k, key in enumerate(keys)})
                   for i, name in enumerate(("tuned", "tuned_att"))}
        results["tuned"].att_update_t = np.empty(0)
        results["tuned_att"].update_t = np.arange(8.)
        results["tuned_att"].att_update_t = np.arange(3.)
        refs = {name: np.eye(3)[None] for name in results}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "tracks.npz"
            np.savez_compressed(path, **report.trajectory_arrays(results, refs))
            with np.load(path) as arrays:
                for name, result in results.items():
                    for key in keys:
                        np.testing.assert_array_equal(arrays[name+"__"+key], getattr(result, key))


if __name__ == "__main__":
    unittest.main()
