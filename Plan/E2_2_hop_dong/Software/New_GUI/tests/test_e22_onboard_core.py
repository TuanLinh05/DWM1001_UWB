"""E2.2-A — Python reference for the onboard core: IMU averaging, PX4 attitude init/aiding,
the node's static-start rule and IMU-gap recovery (eskf_replay.py, still REPLAY_VERSION v2).

Hand-off contract for Plan/E2_2_YEU_CAU_ESKF_ONBOARD_CHONG_RUNG_BANG_PHAN_MEM.md (section 3).
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


def _imu(sim, keep=None):
    rows = zip(sim["imu_t"], sim["imu_acc"], sim["imu_gyro"])
    return [er.ImuSample(float(t), tuple(map(float, a)), tuple(map(float, g)))
            for t, a, g in rows if keep is None or keep(t)]


def _ranges(sim):
    return [er.RangeSample(float(t), int(aid), float(r)) for t, aid, r in sim["ranges"]]


def _attitude(sim, hz=50.0, seed=0, sigma_deg=(1.0, 1.0, 3.0)):
    rng = np.random.default_rng(seed)
    times = np.arange(0.0, sim["truth_t"][-1], 1.0 / hz)
    j = np.clip(np.searchsorted(sim["truth_t"], times), 0, len(sim["truth_t"]) - 1)
    return [er.AttitudeSample(float(sim["truth_t"][k]),
                              sim["truth_R"][k] @ er.so3_exp(np.radians(rng.normal(0.0, sigma_deg)))) for k in j]


def _errors(res, sim):
    tp = np.column_stack([np.interp(res.t, sim["truth_t"], sim["truth_p"][:, k]) for k in range(3)])
    return np.linalg.norm(res.p - tp, axis=1)


def _bin_means(t, acc, gyro, width):
    """Offline form of the core averager: bins floor((t - t0) / width + 1e-9), mean of t/acc/gyro."""
    t, acc, gyro = np.asarray(t, float), np.asarray(acc, float), np.asarray(gyro, float)
    bins = np.floor((t - t[0]) / width + 1e-9).astype(np.int64)
    starts = np.r_[0, np.flatnonzero(np.diff(bins)) + 1]
    counts = np.diff(np.r_[starts, len(t)])
    mean = lambda x: np.add.reduceat(x, starts, axis=0) / (counts[:, None] if x.ndim == 2 else counts)
    return mean(t), mean(acc), mean(gyro)


def _rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def _params(sim, **kw):
    noise = tu.estimate_imu_noise(sim["imu_t"], sim["imu_acc"], sim["imu_gyro"], [(0.5, 30.0)])
    return replace(tu.recommend_params(noise), **kw)


class OnboardCoreTests(unittest.TestCase):
    def test_online_imu_averaging_matches_pre_averaged_stream(self) -> None:
        sim = F.vibrating_flight(imu_hz=200.0, seed=41)                  # raw 200 Hz like HIGHRES_IMU at 200 Hz
        t50, a50, g50 = _bin_means(sim["imu_t"], sim["imu_acc"], sim["imu_gyro"], 0.02)
        mean_noise = tu.estimate_imu_noise(t50, a50, g50, [(0.5, 30.0)])
        params = tu.recommend_params(mean_noise)
        online = er.run_eskf_replay(F.ROOM_48, _imu(sim), _ranges(sim), sim["lever"], sim["yaw0"],
                                    replace(params, imu_average_s=0.02))
        pre = er.run_eskf_replay(F.ROOM_48, [er.ImuSample(float(t), tuple(a), tuple(g)) for t, a, g in zip(t50, a50, g50)],
                                 _ranges(sim), sim["lever"], sim["yaw0"], params)
        rate = (len(online.t) - 1) / (online.t[-1] - online.t[0])
        self.assertAlmostEqual(rate, 50.0, delta=1.0)
        common = np.intersect1d(np.round(online.t, 9), np.round(t50, 9))
        self.assertGreater(len(common), 0.99 * len(online.t))            # outputs at the bin-mean times (last bin open)
        err_on, err_pre = _errors(online, sim), _errors(pre, sim)
        self.assertLess(_rms(err_on[online.t > 6.0]), 0.05)              # reference: 3.1 cm (bin closes 0-20 ms later)
        self.assertLess(_rms(err_on[online.t > 6.0]), 1.5 * _rms(err_pre[pre.t > 6.0]) + 0.002)   # ref: 3.1 vs 2.4 cm
        with self.assertRaises(ValueError):
            er.run_eskf_replay(F.ROOM_48, _imu(sim), _ranges(sim), sim["lever"], sim["yaw0"],
                               replace(params, imu_average_s=-0.01))

    def test_static_rule_blocks_a_vibrating_start_and_external_attitude_does_not(self) -> None:
        sim = F.vibrating_flight(seed=42)                                # motors already spinning at t = 0
        params = _params(sim, static_check=True)
        static = er.run_eskf_replay(F.ROOM_48, _imu(sim), _ranges(sim), sim["lever"], sim["yaw0"], params)
        self.assertIsNone(static.init_time)                              # the node would wait forever
        self.assertEqual(len(static.t), 0)
        self.assertEqual(static.attitude_inits, 0)
        loose = er.run_eskf_replay(F.ROOM_48, _imu(sim), _ranges(sim), sim["lever"], sim["yaw0"],
                                   replace(params, static_check=False))
        self.assertIsNotNone(loose.init_time)                            # E2.0/E2.1 replay behaviour is kept
        ext = er.run_eskf_replay(F.ROOM_48, _imu(sim), _ranges(sim), sim["lever"], sim["yaw0"],
                                 replace(params, attitude_init="external"), attitude=_attitude(sim, seed=1))
        self.assertIsNotNone(ext.init_time)
        self.assertLess(ext.init_time, 0.2)                              # no 1 s static wait
        self.assertEqual(ext.attitude_inits, 1)
        self.assertLess(_rms(_errors(ext, sim)[ext.t > 6.0]), 0.05)      # reference: 2.7 cm
        with self.assertRaises(ValueError):
            er.run_eskf_replay(F.ROOM_48, _imu(sim), _ranges(sim), sim["lever"], sim["yaw0"],
                               replace(params, attitude_init="px4"))

    def test_imu_gap_in_flight_recovers_only_with_external_attitude(self) -> None:
        sim = F.vibrating_flight(seed=43)
        keep = lambda t: not 15.0 <= t < 15.5                           # 0.5 s USB/MAVLink hiccup
        base = _params(sim, static_check=True, imu_gap_reset_s=0.2)
        clean = F.simulate_flight(duration_s=30.0, imu_hz=50.0, seed=43)  # quiet start so static init can happen
        start_quiet = lambda t: True
        imu_static = [s if s.t < 2.0 else v for s, v in zip(_imu(clean, start_quiet), _imu(sim))]
        imu_static = [s for s in imu_static if keep(s.t)]
        static = er.run_eskf_replay(F.ROOM_48, imu_static, _ranges(sim), sim["lever"], sim["yaw0"], base)
        self.assertIsNotNone(static.init_time)
        self.assertEqual(static.imu_gap_resets, 1)
        self.assertFalse(np.any(static.t > 15.5))                       # flying: never static again -> stuck
        ext = er.run_eskf_replay(F.ROOM_48, _imu(sim, keep), _ranges(sim), sim["lever"], sim["yaw0"],
                                 replace(base, attitude_init="external"), attitude=_attitude(sim, seed=2))
        self.assertEqual(ext.imu_gap_resets, 1)
        self.assertEqual(ext.attitude_inits, 2)
        resumed = ext.t[ext.t >= 15.5]
        self.assertGreater(len(resumed), 0)
        self.assertLess(float(resumed[0]), 15.7)                         # reference: ~15.52 s
        err = _errors(ext, sim)
        self.assertLess(_rms(err[(ext.t >= 17.0)]), 0.05)               # reference: 2.6 cm
        self.assertFalse(np.any((ext.t >= 15.0) & (ext.t < 15.5)))      # nothing invented inside the gap

    def test_external_reinit_uses_the_latest_flight_controller_attitude(self) -> None:
        offsets = {a.anchor_id: (14.0, 15.0, 2.0) for a in F.ROOM_48}
        sim = F.vibrating_flight(yaw_rate=0.15, anchor_offsets=offsets, dropouts=[(15.0, 15.3)], seed=21)
        params = _params(sim, attitude_init="external", reinit_keep_attitude=False)
        res = er.run_eskf_replay(F.ROOM_48, _imu(sim), _ranges(sim), sim["lever"], sim["yaw0"], params,
                                 attitude=_attitude(sim, seed=3))
        self.assertEqual(res.reinits, 1)
        j = np.clip(np.searchsorted(sim["truth_t"], res.t), 0, len(sim["truth_t"]) - 1)
        d = np.einsum("nji,njk->nik", sim["truth_R"][j], res.R)
        yaw = np.degrees(np.abs(np.arctan2(d[:, 1, 0], d[:, 0, 0])))
        after = (res.t >= 15.3) & (res.t < 18.0)
        self.assertLess(float(np.median(yaw[after])), 3.0)              # reference: 0.3 deg (boot yaw would be ~105)
        self.assertLess(_rms(_errors(res, sim)[(res.t >= 16.0) & (res.t < 18.0)]), 0.05)

    def test_px4_attitude_conversion_to_the_rig_frame(self) -> None:
        offset = math.radians(70.0)

        def q_from_euler(roll, pitch, yaw):                              # PX4 FRD->NED, ZYX
            cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2),
                                      math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2))
            return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                    cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)

        def yaw_of(R):
            return math.atan2(R[1, 0], R[0, 0])

        level = er.px4_attitude_to_rig(q_from_euler(0, 0, 0), offset)
        self.assertTrue(np.allclose(level @ level.T, np.eye(3), atol=1e-12))
        self.assertAlmostEqual(float(np.linalg.det(level)), 1.0, places=12)
        self.assertTrue(np.allclose(level[:, 2], [0, 0, 1], atol=1e-12))  # body up = rig up
        self.assertAlmostEqual(yaw_of(level), offset, places=12)          # yaw_rig = offset - yaw_ned
        turned = er.px4_attitude_to_rig(q_from_euler(0, 0, math.radians(30)), offset)
        self.assertAlmostEqual(yaw_of(turned), offset - math.radians(30), places=12)
        nose_up = er.px4_attitude_to_rig(q_from_euler(0, math.radians(10), 0), offset)
        self.assertAlmostEqual(nose_up[2, 0], math.sin(math.radians(10)), places=12)  # body +X climbs
        right_down = er.px4_attitude_to_rig(q_from_euler(math.radians(10), 0, 0), offset)
        self.assertAlmostEqual(right_down[2, 1], math.sin(math.radians(10)), places=12)  # body +Y (left) climbs
        scaled = er.px4_attitude_to_rig(tuple(1.2 * v for v in q_from_euler(0.1, -0.2, 0.3)), offset)
        self.assertTrue(np.allclose(scaled, er.px4_attitude_to_rig(q_from_euler(0.1, -0.2, 0.3), offset), atol=1e-12))
        q = q_from_euler(0.05, -0.03, math.radians(123))
        derived = er.rig_yaw_offset_from_yaw0(q, math.radians(-15))
        self.assertAlmostEqual(yaw_of(er.px4_attitude_to_rig(q, derived)), math.radians(-15), places=9)
        for bad in ((0, 0, 0, 0), (float("nan"), 0, 0, 1), (1, 0, 0)):
            with self.assertRaises(ValueError):
                er.px4_attitude_to_rig(bad, offset)


if __name__ == "__main__":
    unittest.main()
