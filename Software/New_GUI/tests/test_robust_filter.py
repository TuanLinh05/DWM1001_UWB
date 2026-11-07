"""Behavioral regressions for high-rate filtering, timing and replay."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import random
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core_state import AnchorConfig, EkfGuardSettings, UwbStateEngine, load_log_session
import telemetry_protocol as tp
from robust_range_ekf import EkfSettings, RangeObservation, RobustRangeEkf


def burst(engine, cycle, xyz, *, dt=.004, boot=1, noise=None, missing=(), bias=None):
    entries = []
    for a in engine.anchors.values():
        if a.anchor_id in missing:
            continue
        distance = math.dist(xyz, (a.x, a.y, a.z)) * 1000
        if noise:
            distance += noise.gauss(0, 20)
        distance += (bias or {}).get(a.anchor_id, 0)
        entries.append(tp.BurstAnchor(a.anchor_id, 0,
            tp.BURST_FLAG_RADIO_OK | tp.BURST_FLAG_CAL_OK | tp.BURST_FLAG_VALID,
            round(distance), -8200, -7800, 0))
    return tp.RangeBurstMessage(cycle, round(cycle * dt * 1000), boot, cycle,
        cycle * 8, round(cycle * dt * 1e6), round(dt * 1e6), tuple(entries))


class FilterBehaviorTests(unittest.TestCase):
    def test_legacy_cv_route_still_tracks_motion(self):
        engine = UwbStateEngine()
        engine.set_position_filter_mode('legacy')
        errors = []
        for i in range(1, 501):
            t = i * .01
            xyz = (2 + .5 * math.sin(t), 2 + .3 * math.cos(t), 1.2)
            engine.ingest_message(burst(engine, i, xyz, dt=.01), t)
            if i > 50:
                sol = engine.latest_solution
                errors.append(math.dist((sol.x, sol.y, sol.z), xyz))
        self.assertLess(sum(errors) / len(errors), .04)

    def test_batch_timing_does_not_change_positions_or_velocities(self):
        realtime, batched = UwbStateEngine(), UwbStateEngine()
        rng = random.Random(12)
        for i in range(1, 600):
            xyz = (2 + i * .002, 1.7, 1.1 + .1 * math.sin(i * .01))
            msg = burst(realtime, i, xyz, noise=rng)
            realtime.ingest_message(msg, i * .004)
            batched.ingest_message(msg, math.floor(i / 10) * .04)
        self.assertEqual(realtime.latest_solution, batched.latest_solution)
        self.assertEqual(realtime.live[1].host_filtered_mm, batched.live[1].host_filtered_mm)

    def test_moving_target_noise_spikes_nlos_and_anchor_dropout(self):
        engine = UwbStateEngine()
        rng = random.Random(77)
        errors, jumps = [], []
        for i in range(1, 1501):
            t = i * .004
            xyz = (2 + .6 * math.sin(t), 2 + .5 * math.cos(t), 1.3 + .2 * math.sin(t * 1.5))
            bad = {1: 1000} if 300 < i < 900 else {}
            if i % 100 == 0:
                bad[2] = 27000
            msg = burst(engine, i, xyz, noise=rng, bias=bad, missing=(5, 8) if 500 < i < 800 else ())
            engine.ingest_message(msg, i * .004)
            sol = engine.latest_solution
            self.assertIsNotNone(sol)
            if i > 100:
                errors.append(math.dist((sol.x, sol.y, sol.z), xyz))
                jumps.append(sol.step_jump_mm)
        self.assertLess(float(np.sqrt(np.mean(np.square(errors)))), .065)
        self.assertLess(max(jumps), 40)
        self.assertGreater(engine.range_ekf.health[1], .9)  # link recovers
        self.assertGreater(float(np.linalg.eigvalsh(engine.range_ekf.p).min()), -1e-10)
        # Spikes, a 1 m NLOS anchor and dropouts are not divergence.
        self.assertEqual(engine.ekf_guard_resets, 0)

    def _stuck_ekf_run(self, guard_enabled):
        """Mechanism test: a stiff EKF is displaced 0.4 m and every range is then gated."""
        engine = UwbStateEngine()
        engine.ekf_guard = EkfGuardSettings(enabled=guard_enabled)
        engine.range_ekf = RobustRangeEkf(EkfSettings(acceleration_noise=1e-7,
                                                      max_prediction_s=10.0, reacquire_after_s=10.0))
        rng, truth = random.Random(5), (2.0, 2.0, 1.2)
        for i in range(1, 101):
            engine.ingest_message(burst(engine, i, truth, noise=rng), i * .004)
        engine.range_ekf.x[:3] += np.array([0.4, 0.0, 0.0])
        engine.range_ekf.p[:3, :3] = np.eye(3) * 1e-6
        reasons, errors = [], []
        for i in range(101, 601):
            engine.ingest_message(burst(engine, i, truth, noise=rng), i * .004)
            sol = engine.latest_solution
            reasons.append(sol.instability_reason)
            errors.append(math.dist((sol.x, sol.y, sol.z), truth))
        return engine, reasons, errors

    def test_divergence_guard_reanchors_a_stuck_ekf(self):
        engine, _reasons, errors = self._stuck_ekf_run(False)
        # Without the guard it stays 0.4 m off for over a second (0.6-1.0 s shown).
        self.assertGreater(min(errors[150:250]), .3)
        engine, reasons, errors = self._stuck_ekf_run(True)
        self.assertEqual(engine.ekf_guard_resets, 1)
        fired = next(k for k, r in enumerate(reasons) if 'EKF re-anchored to IRLS' in r)
        self.assertLess(fired * .004, .6)  # persist_s plus a few cycles
        self.assertLess(max(errors[fired + 5:]), .05)

    def _lossy_run(self, window_s):
        # Like the 13:16 log: ~1 range per Burst (random anchor), TAG walking slowly.
        engine = UwbStateEngine()
        engine.range_ekf = RobustRangeEkf(EkfSettings(full_fix_window_s=window_s))
        rng = random.Random(9)
        fixes, errors = 0, []
        for i in range(1, 2501):
            t = i * .0031
            xyz = (2.5 + .4 * math.sin(.5 * t), 2 + .3 * math.cos(.5 * t), 1.2)
            keep = {rng.randint(1, 8)} if rng.random() < .75 else set()
            if rng.random() < .3:
                keep.add(rng.randint(1, 8))
            missing = tuple(a for a in range(1, 9) if a not in keep)
            engine.ingest_message(burst(engine, i, xyz, dt=.0031, noise=rng, missing=missing), t)
            sol = engine.latest_solution
            if i > 300:
                fixes += sol is not None
                if sol is not None:
                    errors.append(math.dist((sol.x, sol.y, sol.z), xyz))
        return fixes / 2200, errors

    def test_lossy_bursts_keep_a_fix_from_distinct_recent_anchors(self):
        single_cycle_rate, _ = self._lossy_run(0.0)   # old rule: 4 anchors in one Burst
        rate, errors = self._lossy_run(0.2)
        self.assertLess(single_cycle_rate, .5)
        self.assertGreater(rate, .99)
        self.assertLess(float(np.sqrt(np.mean(np.square(errors)))), .05)

    def test_divergence_guard_ignores_fast_motion_with_partial_anchor_sets(self):
        # Like the 07/11 log: ~half the anchors missing per cycle, so IRLS mixes
        # range ages while the TAG moves at up to ~1.6 m/s.
        engine = UwbStateEngine()
        rng = random.Random(21)
        for i in range(1, 2001):
            t = i * .004
            xyz = (2.5 + 1.2 * math.sin(1.3 * t), 2 + 1.0 * math.cos(1.3 * t), 1.2 + .2 * math.sin(2 * t))
            missing = tuple(a for a in range(1, 9) if rng.random() < .5)
            engine.ingest_message(burst(engine, i, xyz, noise=rng, missing=missing), t)
        self.assertEqual(engine.ekf_guard_resets, 0)
        sol = engine.latest_solution
        self.assertLess(math.dist((sol.x, sol.y, sol.z), xyz), .1)

    def test_slow_motion_is_not_frozen_by_stationary_detection(self):
        engine = UwbStateEngine()
        for i in range(1, 1001):
            xyz = (2 + .02 * i * .01, 2, 1.2)
            engine.ingest_message(burst(engine, i, xyz, dt=.01), i * .01)
        sol = engine.latest_solution
        self.assertAlmostEqual(sol.x, 2.2, delta=.015)

    def test_noise_window_recovers_after_large_range_change(self):
        engine = UwbStateEngine()
        st = engine.live[4]
        for i in range(160):
            st.rx_count += 1
            # One-metre step then quiet, after an initially noisy segment.
            mm = (2000 + (100 if i % 2 else -100)) if i < 60 else (3000 + (10 if i % 2 else -10))
            engine._update_anchor_rate_and_noise(4, i * .004, mm)
        self.assertLess(st.noise_sigma_mm, 35)
        self.assertEqual(engine._noise_window[4][-1], 3010)

    def test_cv_checkbox_path_returns_actual_cv_estimate(self):
        engine = UwbStateEngine()
        for i in range(30):
            result = engine._update_filtered_range(1, 2000 + (20 if i % 2 else -20), -8200, i * 4, i * .004)
        self.assertEqual(result, round(engine.host_filters[1].x))

    def test_duplicates_out_of_order_and_reboot(self):
        engine = UwbStateEngine()
        last = burst(engine, 10, (2, 2, 1.2))
        engine.ingest_message(last, .04)
        sol, count = engine.latest_solution, engine.live[1].rx_count
        engine.ingest_message(last, .08)
        engine.ingest_message(burst(engine, 9, (4, 3, 1.8)), .09)
        self.assertEqual(engine.live[1].rx_count, count)
        self.assertEqual(engine.latest_solution, sol)
        engine.ingest_message(burst(engine, 1, (3, 2, 1.3), boot=2), .1)
        self.assertAlmostEqual(engine.latest_solution.x, 3, delta=.01)
        self.assertEqual(engine.latest_solution.step_jump_mm, 0)

    def test_dropout_expires_and_reacquires_at_new_location(self):
        engine = UwbStateEngine()
        for i in range(1, 301):
            missing = tuple(range(1, 9)) if 30 <= i < 280 else ()
            xyz = (2, 2, 1.2) if i < 30 else (3, 2, 1.2)
            engine.ingest_message(burst(engine, i, xyz, missing=missing), i * .004)
            if i == 200:
                self.assertIsNone(engine.latest_solution)
        self.assertAlmostEqual(engine.latest_solution.x, 3, delta=.015)

    def test_2d_and_dimension_changes_reset_covariance(self):
        engine = UwbStateEngine()
        engine.ingest_message(burst(engine, 1, (2, 2, 1.2)), .004)
        engine.solver_dimension, engine.fixed_z_m = 2, 1.2
        for i in range(2, 40):
            engine.ingest_message(burst(engine, i, (2 + i * .001, 2, 1.2)), i * .004)
        self.assertEqual(engine.latest_solution.z, 1.2)
        self.assertEqual(engine.latest_solution.vz_mps, 0)
        self.assertEqual(engine.latest_solution.position_std_m[2], 0)
        engine.solver_dimension = 3
        engine.ingest_message(burst(engine, 40, (2.04, 2, 1.2)), .16)
        self.assertGreater(engine.latest_solution.position_std_m[2], 0)

    def test_range_meas_is_ignored_while_burst_is_active(self):
        engine = UwbStateEngine()
        engine.ingest_message(burst(engine, 10, (2, 2, 1.2)), .04)
        count = engine.live[1].rx_count
        # RANGE_MEAS records can repeat observations of the same ranging cycle.
        msg = burst(engine, 10, (2, 2, 1.2))
        meas = msg.to_range_meas()[0]
        engine.ingest_message(meas, .041)
        self.assertEqual(engine.live[1].rx_count, count)

    def test_standalone_range_meas_tracks_and_recovers_from_reboot(self):
        engine = UwbStateEngine()
        for i in range(1, 50):
            for meas in burst(engine, i, (2 + i * .001, 2, 1.2)).to_range_meas():
                engine.ingest_message(meas, i * .004)
        self.assertAlmostEqual(engine.latest_solution.x, 2.049, delta=.02)
        for meas in burst(engine, 1, (3, 2, 1.2), boot=2).to_range_meas():
            engine.ingest_message(meas, .2)
        self.assertAlmostEqual(engine.latest_solution.x, 3, delta=.02)

    def test_anchor_order_does_not_change_ekf_update(self):
        a, b = RobustRangeEkf(), RobustRangeEkf()
        obs = [RangeObservation(i, xyz, math.dist((2.03, 2, 1.2), xyz), .03)
               for i, xyz in enumerate([(0, 0, 0), (5, 0, 0), (0, 4, 0), (0, 0, 3)], 1)]
        for f in (a, b):
            f.initialize((2, 2, 1.2), 0, 3, 0)
            f.predict(.004)
        a.update(obs, .004)
        b.update(list(reversed(obs)), .004)
        np.testing.assert_allclose(a.x, b.x, atol=1e-12)

    def test_replay_decimates_after_filtering(self):
        from serial_worker import FastCsvLogger
        engine = UwbStateEngine()
        with tempfile.TemporaryDirectory() as tmp:
            logger = FastCsvLogger(Path(tmp), 'REPLAY_TEST', engine)
            try:
                for i in range(1, 81):
                    msg = burst(engine, i, (2 + i * .001, 2, 1.2))
                    engine.ingest_message(msg, i * .004)
                    logger.log_burst(msg, None, None, engine)
            finally:
                logger.stop()
            with logger.csv_path.open(newline='', encoding='utf-8') as handle:
                reader = csv.reader(handle)
                widths = [len(r) for r in reader]
            self.assertEqual(len(set(widths)), 1)
            with patch.object(UwbStateEngine, 'ingest_message', autospec=True,
                              side_effect=UwbStateEngine.ingest_message) as ingest:
                replay = load_log_session(logger.csv_path, re_solve=True, max_frames=8)
            self.assertEqual(ingest.call_count, 80)
            self.assertAlmostEqual(replay.frames[-1].x, engine.latest_solution.x, places=4)
            metadata = json.loads(logger.meta_path.read_text())
            self.assertEqual(metadata['solver']['position_filter_mode'], 'robust_ekf')

    def test_replay_preserves_distinct_host_and_firmware_bias(self):
        from serial_worker import FastCsvLogger
        engine = UwbStateEngine()
        a = engine.anchors[1]
        a.bias_mm, a.fw_bias_mm, a.fw_calibrated = 100, 40, True
        with tempfile.TemporaryDirectory() as tmp:
            logger = FastCsvLogger(Path(tmp), 'BIAS_REPLAY', engine)
            try:
                for i in range(1, 25):
                    msg = burst(engine, i, (2, 2, 1.2), bias={1: 60})
                    engine.ingest_message(msg, i * .004)
                    logger.log_burst(msg, None, None, engine)
            finally:
                logger.stop()
            replay = load_log_session(logger.csv_path, re_solve=True)
            self.assertAlmostEqual(replay.frames[-1].x, engine.latest_solution.x, places=4)
            self.assertAlmostEqual(replay.frames[-1].z, engine.latest_solution.z, places=4)
            with self.assertRaises(InterruptedError):
                load_log_session(logger.csv_path, re_solve=True, cancel_requested=lambda: True)

    def test_log_width_matches_for_missing_fix(self):
        from serial_worker import FastCsvLogger
        engine = UwbStateEngine()
        with tempfile.TemporaryDirectory() as tmp:
            logger = FastCsvLogger(Path(tmp), 'NO_FIX', engine)
            try:
                msg = burst(engine, 1, (2, 2, 1.2), missing=range(1, 9))
                engine.ingest_message(msg, .004)
                logger.log_burst(msg, None, None, engine)
                msg = burst(engine, 2, (2, 2, 1.2))
                engine.ingest_message(msg, .008)
                logger.log_burst(msg, None, None, engine)
            finally:
                logger.stop()
            with logger.csv_path.open(newline='', encoding='utf-8') as f:
                rows = list(csv.reader(f))
            self.assertEqual(len({len(row) for row in rows}), 1)


if __name__ == '__main__':
    unittest.main()
