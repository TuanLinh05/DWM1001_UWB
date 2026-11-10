"""E2.0-B — offline ESKF-A replay (eskf_replay.py), a Python mirror of
UP4000/estimator/uwb_eskf/include/uwb_eskf/filter.hpp plus the node's init logic.

Hand-off contract for Plan/E2_0_YEU_CAU_HIEU_CHUAN_CO_CAM_BIEN_VA_REPLAY_ESKF_A.md.
Synthetic flights: tests/e20_fixtures.py. Reference results noted next to thresholds.
"""

from __future__ import annotations

import math
import random
import unittest

import numpy as np

import e20_fixtures as F
import eskf_replay as er  # noqa: E402


def _inputs(sim):
    imu = [er.ImuSample(float(t), tuple(map(float, a)), tuple(map(float, g)))
           for t, a, g in zip(sim["imu_t"], sim["imu_acc"], sim["imu_gyro"])]
    ranges = [er.RangeSample(float(t), int(aid), float(r)) for t, aid, r in sim["ranges"]]
    return imu, ranges


def _run(sim, lever=None, params=er.EskfParams()):
    imu, ranges = _inputs(sim)
    res = er.run_eskf_replay(F.ROOM_48, imu, ranges, sim["lever"] if lever is None else lever, sim["yaw0"], params)
    truth_p = np.column_stack([np.interp(res.t, sim["truth_t"], sim["truth_p"][:, k]) for k in range(3)])
    truth_tag = np.column_stack([np.interp(res.t, sim["truth_t"], sim["truth_tag"][:, k]) for k in range(3)])
    return res, np.linalg.norm(res.p - truth_p, axis=1), np.linalg.norm(res.tag - truth_tag, axis=1)


def _ls_error(sim, t0, t1):
    grid = np.arange(t0, t1, 0.01)
    est = F.epoch_least_squares(sim["ranges"], grid)
    truth = np.column_stack([np.interp(grid, sim["truth_t"], sim["truth_tag"][:, k]) for k in range(3)])
    return np.linalg.norm(est - truth, axis=1)


def _rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


class EskfReplayTests(unittest.TestCase):
    _circle_cache = None

    @classmethod
    def setUpClass(cls) -> None:
        cls.circle = F.simulate_flight(duration_s=30.0, seed=2)

    def _circle_run(self):
        if EskfReplayTests._circle_cache is None:
            EskfReplayTests._circle_cache = _run(self.circle)
        return EskfReplayTests._circle_cache

    def test_static_start_initializes_and_converges(self) -> None:
        sim = F.simulate_flight(duration_s=20.0, hover_s=25.0, seed=1)
        res, err_p, err_tag = _run(sim)
        self.assertEqual(res.version, er.REPLAY_VERSION)
        self.assertGreaterEqual(res.init_time, 1.0)          # waits init_static_s of IMU
        self.assertLess(res.init_time, 1.2)
        late = res.t > 5.0
        self.assertLess(_rms(err_tag[late]), 0.02)            # reference: 0.5 cm
        self.assertGreater(res.acceptance_ratio(), 0.95)
        self.assertEqual(res.reinits, 0)
        self.assertLess(float(err_p[0]), 0.10)                # init removes the lever arm (C++ node does not yet)

    def test_circle_flight_beats_per_epoch_least_squares(self) -> None:
        res, err_p, _ = self._circle_run()
        flying = res.t > 6.0
        ls = _ls_error(self.circle, 6.0, 29.9)
        self.assertLess(_rms(err_p[flying]), 0.03)             # reference: 0.6 cm
        self.assertLess(_rms(err_p[flying]), 0.5 * _rms(ls[np.isfinite(ls)]))   # LS reference: 4.2 cm
        self.assertTrue(np.all(np.diff(res.t) > 0))
        self.assertEqual(res.p.shape, (len(res.t), 3))
        self.assertEqual(res.R.shape, (len(res.t), 3, 3))

    def test_lever_arm_is_required(self) -> None:
        res, err_p, _ = _run(self.circle, lever=np.zeros(3))
        self.assertGreater(_rms(err_p[res.t > 6.0]), 0.20)     # reference: 32.8 cm without the lever

    def test_nlos_spikes_are_rejected(self) -> None:
        sim = F.simulate_flight(duration_s=30.0, spikes_frac=0.02, seed=3)
        res, _, err_tag = _run(sim)
        flying = res.t > 6.0
        ls = _ls_error(sim, 6.0, 29.9)
        self.assertLess(float(np.percentile(err_tag[flying], 99)), 0.05)       # reference: 1.0 cm
        self.assertGreater(float(np.nanpercentile(ls, 99)), 0.20)               # reference: 83 cm
        self.assertLess(res.acceptance_ratio(), 0.99)                            # spikes were refused
        self.assertGreater(res.acceptance_ratio(), 0.95)
        refused = ~res.update_accepted
        self.assertTrue(np.all(res.update_nis[refused] > er.EskfParams().gate))

    def test_one_second_uwb_outage_is_bridged(self) -> None:
        sim = F.simulate_flight(duration_s=30.0, dropouts=[(15.0, 16.0)], seed=4)
        res, err_p, _ = _run(sim)
        gap = (res.t >= 15.0) & (res.t < 16.0)
        self.assertTrue(gap.any())                            # output continues from IMU prediction
        self.assertLess(float(np.max(err_p[gap])), 0.10)       # reference: 1.3 cm (clean synthetic IMU)
        self.assertLess(float(np.interp(17.0, res.t, err_p)), 0.03)
        self.assertEqual(res.reinits, 0)

    def test_persistent_anchor_bias_is_not_removed(self) -> None:
        # Documented limitation of the 15-state filter: it smooths, it does not calibrate.
        sim = F.simulate_flight(duration_s=30.0, anchor_offsets={3: (12.0, 17.0, 0.25)}, seed=5)
        res, _, err_tag = _run(sim)
        window = (res.t >= 12.0) & (res.t < 17.0)
        eskf_p95 = float(np.percentile(err_tag[window], 95))
        ls_p95 = float(np.nanpercentile(_ls_error(sim, 12.0, 17.0), 95))
        self.assertGreater(eskf_p95, 0.10)                    # reference: 18.3 cm
        self.assertLess(abs(eskf_p95 - ls_p95), 0.5 * ls_p95)  # about the same as LS (18.8 cm)

    def test_fifty_hertz_imu_like_the_onboard_stream(self) -> None:
        sim = F.simulate_flight(duration_s=30.0, imu_hz=50.0, seed=6)
        res, err_p, _ = _run(sim)
        self.assertLess(_rms(err_p[res.t > 6.0]), 0.03)

    def test_input_order_and_invalid_samples(self) -> None:
        sim = F.simulate_flight(duration_s=8.0, seed=7)
        imu, ranges = _inputs(sim)
        base = er.run_eskf_replay(F.ROOM_48, imu, ranges, sim["lever"], sim["yaw0"])
        shuffled_imu, shuffled_ranges = imu[:], ranges[:]
        random.Random(1).shuffle(shuffled_imu)
        random.Random(2).shuffle(shuffled_ranges)
        noisy = shuffled_ranges + [er.RangeSample(4.0, 3, math.nan), er.RangeSample(4.0, 99, 2.0),
                                   er.RangeSample(math.nan, 2, 2.0)]
        again = er.run_eskf_replay(F.ROOM_48, shuffled_imu, noisy, sim["lever"], sim["yaw0"])
        self.assertTrue(np.array_equal(base.t, again.t))
        self.assertLess(float(np.max(np.abs(base.p - again.p))), 1e-12)
        self.assertEqual(len(base.update_t), len(again.update_t))


if __name__ == "__main__":
    unittest.main()
