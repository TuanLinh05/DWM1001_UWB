"""E2.1-B — ESKF-A replay under in-flight vibration: tuning, per-anchor rate limit, attitude
carry-over at re-init, optional flight-controller attitude aiding, consistency diagnostics.

Hand-off contract for Plan/E2_1_YEU_CAU_TINH_CHINH_ESKF_THEO_RUNG_KHI_BAY.md (section 4).
Synthetic flights: tests/e21_fixtures.py. Reviewer reference results are noted next to thresholds.
"""

from __future__ import annotations

from dataclasses import replace
import math
import unittest

import numpy as np

import e21_fixtures as F
import eskf_replay as er
import eskf_tuning as tu  # noqa: E402


def _inputs(sim):
    imu = [er.ImuSample(float(t), tuple(map(float, a)), tuple(map(float, g)))
           for t, a, g in zip(sim["imu_t"], sim["imu_acc"], sim["imu_gyro"])]
    ranges = [er.RangeSample(float(t), int(aid), float(r)) for t, aid, r in sim["ranges"]]
    return imu, ranges


def _recommended(sim, **kw):
    noise = tu.estimate_imu_noise(sim["imu_t"], sim["imu_acc"], sim["imu_gyro"], [(0.5, 30.0)])
    return tu.recommend_params(noise, **kw)


def _run(sim, params, attitude=()):
    imu, ranges = _inputs(sim)
    res = er.run_eskf_replay(F.ROOM_48, imu, ranges, sim["lever"], sim["yaw0"], params, attitude=attitude)
    truth_p = np.column_stack([np.interp(res.t, sim["truth_t"], sim["truth_p"][:, k]) for k in range(3)])
    return res, np.linalg.norm(res.p - truth_p, axis=1)


def _yaw_error_deg(res, sim):
    j = np.clip(np.searchsorted(sim["truth_t"], res.t), 0, len(sim["truth_t"]) - 1)
    d = np.einsum("nji,njk->nik", sim["truth_R"][j], res.R)
    return np.degrees(np.abs(np.arctan2(d[:, 1, 0], d[:, 0, 0])))


def _rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


class EskfTuningTests(unittest.TestCase):
    _cache: dict = {}

    def _correlated(self, limit):
        key = ("corr", limit)
        if key not in self._cache:
            sim = F.vibrating_flight(uwb_hz=250.0, correlated=True, seed=31)
            res, err = _run(sim, _recommended(sim, min_anchor_interval_s=limit))
            tz = np.interp(res.t, sim["truth_t"], sim["truth_tag"][:, 2])
            late = res.t > 6.0
            cons = tu.normalized_errors(res.t, res.tag[:, 2], res.p_std[:, 2], res.t[late], tz[late])
            self._cache[key] = (res, err, cons)
        return self._cache[key]

    def test_recommended_noise_model_tracks_a_vibrating_flight(self) -> None:
        sim = F.vibrating_flight(seed=11)                                   # 50 Hz IMU, Test_1-like vibration
        default, err_d = _run(sim, er.EskfParams())
        tuned, err_t = _run(sim, _recommended(sim))
        fly_d, fly_t = default.t > 6.0, tuned.t > 6.0
        self.assertEqual(er.REPLAY_VERSION, "eskf-a-replay-v2")
        self.assertGreater(_rms(err_d[fly_d]), 0.15)                        # reference: 35.1 cm (datasheet noise)
        self.assertLess(_rms(err_t[fly_t]), 0.05)                           # reference: 2.3 cm
        self.assertLess(_rms(err_t[fly_t]), 0.25 * _rms(err_d[fly_d]))
        self.assertEqual(tuned.reinits, 0)
        self.assertGreater(tuned.acceptance_ratio(), 0.99)

    def test_per_anchor_rate_limit(self) -> None:
        sim = F.simulate_flight(duration_s=8.0, seed=7)                     # 100 Hz per anchor, clean
        base, _ = _run(sim, er.EskfParams())
        tiny, _ = _run(sim, replace(er.EskfParams(), min_anchor_interval_s=1e-4))
        self.assertEqual(tiny.skipped_ranges, 0)                            # below the 10 ms spacing: no change
        self.assertTrue(np.array_equal(base.t, tiny.t))
        self.assertLess(float(np.max(np.abs(base.p - tiny.p))), 1e-12)
        limited, _ = _run(sim, replace(er.EskfParams(), min_anchor_interval_s=0.02))
        self.assertGreater(limited.skipped_ranges, 0.45 * len(sim["ranges"]))
        self.assertLess(len(limited.update_t), 0.55 * len(base.update_t))
        for aid in range(1, 9):
            times = limited.update_t[limited.update_anchor == aid]
            self.assertGreaterEqual(float(np.min(np.diff(times))), 0.02 - 1e-9)
        self.assertEqual(base.skipped_ranges, 0)
        with self.assertRaises(ValueError):
            _run(sim, replace(er.EskfParams(), min_anchor_interval_s=-0.01))

    def test_rate_limit_restores_consistency_with_correlated_ranges(self) -> None:
        res_all, err_all, cons_all = self._correlated(0.0)
        res_lim, err_lim, cons_lim = self._correlated(0.02)                 # 250 Hz -> 50 Hz per anchor
        self.assertLess(cons_all["frac_within_2sigma"], 0.90)               # reference: 0.83 (overconfident)
        self.assertGreaterEqual(cons_lim["frac_within_2sigma"], 0.90)       # reference: 0.997
        self.assertGreater(cons_lim["rms_normalized"], 0.6)                 # reference: 0.81
        self.assertLess(cons_lim["rms_normalized"], 1.3)
        self.assertLess(_rms(err_lim[res_lim.t > 6.0]), 1.2 * _rms(err_all[res_all.t > 6.0]))   # ref: 7.3 vs 7.0 cm
        self.assertLess(len(res_lim.update_t), 0.25 * len(res_all.update_t))                    # ref: 0.20

    def test_innovation_autocorrelation_flags_correlated_ranges(self) -> None:
        res_corr, _, _ = self._correlated(0.02)
        white_sim = F.vibrating_flight(uwb_hz=250.0, correlated=False, seed=31)
        res_white, _ = _run(white_sim, _recommended(white_sim))
        corr = tu.innovation_autocorrelation(res_corr, lags_s=(0.1, 0.5, 1.0))
        white = tu.innovation_autocorrelation(res_white, lags_s=(0.1, 0.5, 1.0))
        self.assertEqual(sorted(corr), list(range(1, 9)))
        self.assertGreater(min(v["autocorr"][1] for v in corr.values()), 0.3)     # reference: >= 0.39 at 0.5 s
        self.assertLess(max(v["autocorr"][1] for v in white.values()), 0.2)       # reference: <= 0.10
        for v in corr.values():
            self.assertAlmostEqual(v["rate_hz"], 50.0, delta=3.0)
            self.assertEqual(tuple(v["lags_s"]), (0.1, 0.5, 1.0))

    def test_reinit_keeps_the_propagated_attitude(self) -> None:
        # Yaw turns 0.15 rad/s; every range is 2 m long during [14, 15) s (forced drop), clean re-init after 15.3 s.
        offsets = {a.anchor_id: (14.0, 15.0, 2.0) for a in F.ROOM_48}
        sim = F.vibrating_flight(yaw_rate=0.15, anchor_offsets=offsets, dropouts=[(15.0, 15.3)], seed=21)
        keep = _recommended(sim)
        results = {}
        for name, params in (("keep", keep), ("boot", replace(keep, reinit_keep_attitude=False))):
            res, err = _run(sim, params)
            yaw = _yaw_error_deg(res, sim)
            after = (res.t >= 15.3) & (res.t < 18.0)
            settle = (res.t >= 16.0) & (res.t < 18.0)
            self.assertEqual(res.reinits, 1, name)
            results[name] = (float(np.median(yaw[after])), _rms(err[settle]))
        self.assertLess(results["keep"][0], 25.0)        # reference: ~6 deg
        self.assertLess(results["keep"][1], 0.06)        # reference: 2.4 cm
        self.assertGreater(results["boot"][0], 60.0)     # reference: ~105 deg (boot yaw, 13 s of turning ago)
        self.assertGreater(results["boot"][1], 0.10)     # reference: 14.2 cm

    def test_flight_controller_attitude_aiding(self) -> None:
        sim = F.vibrating_flight(seed=11)
        params = _recommended(sim)
        rng = np.random.default_rng(11)
        times = np.arange(0.0, 30.0, 0.02)
        j = np.clip(np.searchsorted(sim["truth_t"], times), 0, len(sim["truth_t"]) - 1)
        good = [er.AttitudeSample(float(sim["truth_t"][k]),
                                  sim["truth_R"][k] @ er.so3_exp(np.radians(rng.normal(0.0, (1.0, 1.0, 3.0)))))
                for k in j]
        bad = [er.AttitudeSample(a.t, a.R @ er.so3_exp([0.0, 0.0, math.pi / 2])) if 20.0 <= a.t < 21.0 else a
               for a in good]
        none, err_n = _run(sim, params)
        aided, err_a = _run(sim, params, good)
        hit, err_h = _run(sim, params, bad)
        fly = none.t > 6.0
        self.assertGreater(float(np.percentile(_yaw_error_deg(none, sim)[fly], 95)), 3.0)    # reference: 6.7 deg
        self.assertLess(float(np.percentile(_yaw_error_deg(aided, sim)[aided.t > 6.0], 95)), 2.0)  # ref: 1.1 deg
        self.assertEqual(len(none.att_update_t), 0)
        self.assertGreater(len(aided.att_update_t), 1400)
        self.assertTrue(np.all(aided.att_accepted))
        self.assertLess(_rms(err_a[aided.t > 6.0]), 1.1 * _rms(err_n[fly]) + 0.002)
        wrong = (hit.att_update_t >= 20.0) & (hit.att_update_t < 21.0)
        self.assertGreater(int(wrong.sum()), 40)
        self.assertFalse(np.any(hit.att_accepted[wrong]))                   # 90 deg yaw jump is gated
        self.assertTrue(np.all(hit.att_nis[wrong] > params.att_gate))
        self.assertLess(_rms(err_h[hit.t > 6.0]), 1.1 * _rms(err_a[aided.t > 6.0]) + 0.002)
        noisy_params = replace(params, att_min_interval_s=0.1)
        sparse, _ = _run(sim, noisy_params, good)
        self.assertLess(len(sparse.att_update_t), 0.25 * len(aided.att_update_t))

    def test_normalized_errors_on_known_gaussian(self) -> None:
        rng = np.random.default_rng(5)
        t = np.arange(0.0, 100.0, 0.002)
        truth = np.sin(t)
        sigma = np.full(len(t), 0.05)
        est = truth + rng.normal(0.0, 0.05, len(t))
        ok = tu.normalized_errors(t, est, sigma, t[::3], truth[::3])
        self.assertAlmostEqual(ok["frac_within_2sigma"], 0.954, delta=0.02)
        self.assertAlmostEqual(ok["rms_normalized"], 1.0, delta=0.05)
        self.assertAlmostEqual(ok["frac_within_1sigma"], 0.683, delta=0.03)
        over = tu.normalized_errors(t, est, sigma / 3, t[::3], truth[::3])
        self.assertAlmostEqual(over["rms_normalized"], 3.0, delta=0.15)
        self.assertAlmostEqual(over["frac_within_2sigma"], 0.495, delta=0.03)
        gap = (t < 40.0) | (t > 41.0)                                        # 1 s hole in the estimate
        holed = tu.normalized_errors(t[gap], est[gap], sigma[gap], t[::3], truth[::3], max_gap_s=0.1)
        self.assertLess(holed["n"], ok["n"] - 10)
        self.assertEqual(tu.normalized_errors(t[:1], est[:1], sigma[:1], t, truth)["n"], 0)

    def test_log_so3_is_the_inverse_of_so3_exp(self) -> None:
        rng = np.random.default_rng(9)
        for _ in range(200):
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            phi = axis * rng.uniform(0.0, math.pi - 1e-3)
            self.assertLess(float(np.max(np.abs(er.log_so3(er.so3_exp(phi)) - phi))), 1e-7)
        self.assertLess(float(np.max(np.abs(er.log_so3(np.eye(3))))), 1e-12)


if __name__ == "__main__":
    unittest.main()
