"""Radio heuristics, chronology, covariance gating and accuracy with known truth."""
from dataclasses import replace
import csv
import json
import math
from pathlib import Path
import random
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core_state import UwbStateEngine, load_log_session
from radio_quality import RadioQualityModel
from robust_range_ekf import RangeObservation, RobustRangeEkf
from serial_worker import FastCsvLogger
from test_robust_filter import burst
import telemetry_protocol as tp


def diagnostics(cycle, *, bad=(), boot=1, flags=1, time_s=None):
    entries = tuple(tp.RxQualityAnchor(
        aid, flags, 100, 2000, 2000, 2000, 288,
        1500 if aid in bad else 20, 47000, -1234, -8300, -7900,
        -8300, -7900) for aid in range(1, 9))
    t = cycle * .004 if time_s is None else time_s
    return tp.RxQualityMessage(cycle, round(t * 1000), boot, cycle,
        round(t * 1e6), 16, 3, 28.0, 3.3, 0, 100, 120, 180, entries)


def assess(model, t=0, **kwargs):
    options = dict(fp_dbm=-83, nlos_db=4, ci_ppm=-20)
    options.update(kwargs)
    return model.assess(1, 1, t, **options)


class RadioQualityTests(unittest.TestCase):
    def test_missing_information_is_neutral_and_not_claimed_good(self):
        q = assess(RadioQualityModel(), fp_dbm=None, nlos_db=None, ci_ppm=None)
        self.assertEqual(q.variance_scale, 1)
        self.assertEqual(q.label, 'UNKNOWN')
        self.assertIsNone(q.score)

    def test_weak_power_reduces_trust_and_recovers_in_device_seconds(self):
        model = RadioQualityModel()
        bad = assess(model, 0, fp_dbm=-104)
        recovering = assess(model, .004)
        good = assess(model, 2)
        self.assertGreater(bad.extra_variance_scale, 5)
        self.assertGreater(recovering.variance_scale, 5)
        self.assertLess(good.variance_scale, 1.01)
        self.assertIn('Recovering', recovering.reasons)

    def test_correlated_gap_is_not_multiplied_twice(self):
        q = assess(RadioQualityModel(), nlos_db=12)
        self.assertAlmostEqual(q.variance_scale, 1 + .15 * 6**2)
        self.assertAlmostEqual(q.extra_variance_scale, 1)

    def test_legacy_power_scale_does_not_use_absolute_thresholds(self):
        q = assess(RadioQualityModel(), fp_dbm=-104, corrected_power=False)
        self.assertEqual(q.variance_scale, 1)

    def test_raw_noise_features_validity_age_and_boot(self):
        bad = diagnostics(0, bad=(1,))
        for mode in ('fresh', 'future', 'stale', 'wrong_boot', 'invalid'):
            model = RadioQualityModel()
            msg = bad
            if mode == 'wrong_boot':
                msg = replace(bad, boot_id=2)
            if mode == 'invalid':
                msg = diagnostics(0, bad=(1,), flags=0)
            model.ingest(msg, 1)
            t = -.01 if mode == 'future' else .31 if mode == 'stale' else 0
            q = assess(model, t)
            if mode == 'fresh':
                self.assertGreater(q.extra_variance_scale, 5)
                self.assertIn('fp_noise_proxy', q.features)
                self.assertIsNotNone(q.fp_noise_db)
            else:
                self.assertEqual(q.extra_variance_scale, 1, mode)
                self.assertIsNone(q.diagnostic_age_s, mode)

    def test_out_of_order_diagnostics_do_not_replace_newer_sample(self):
        model = RadioQualityModel()
        model.ingest(diagnostics(10), 1)
        model.ingest(diagnostics(9, bad=(1,)), 1)
        self.assertEqual(assess(model, .044).extra_variance_scale, 1)

    def test_anchor_final_power_can_detect_bad_reverse_link(self):
        msg = diagnostics(0, flags=3)
        anchor = replace(msg.anchors[0], anchor_fp_cdbm=-10500, anchor_rx_cdbm=-9000)
        msg = replace(msg, anchors=(anchor,))
        model = RadioQualityModel()
        model.ingest(msg, 1)
        q = assess(model)
        self.assertGreater(q.extra_variance_scale, 10)
        self.assertIn('anchor_rx_fp_gap', q.features)

    def test_stable_clock_offset_is_not_mistaken_for_error(self):
        model = RadioQualityModel()
        assess(model, 0, ci_ppm=-25)
        stable = assess(model, .004, ci_ppm=-25)
        jump = assess(model, .008, ci_ppm=-10)
        self.assertEqual(stable.extra_variance_scale, 1)
        self.assertGreater(jump.extra_variance_scale, 1)

    def test_quality_never_opens_innovation_gate_for_huge_outlier(self):
        f = RobustRangeEkf()
        f.initialize((2, 2, 1.2), 0, 3, 0)
        f.predict(.004)
        f.update([RangeObservation(1, (0, 0, 0), 30, .035, 4, 36)], .004)
        self.assertEqual(f.rejected, {1: 'innovation gate'})

    def test_healthy_radio_has_identical_output_with_weighting_disabled(self):
        on, off = UwbStateEngine(), UwbStateEngine()
        off.use_signal_quality = False
        for i in range(1, 120):
            msg = burst(on, i, (2 + i * .001, 2, 1.2))
            on.ingest_message(msg, i * .004)
            off.ingest_message(msg, i * .004)
        self.assertEqual(on.latest_solution, off.latest_solution)

    def test_new_boot_discards_cached_bad_radio_diagnostics(self):
        engine = UwbStateEngine()
        engine.ingest_message(burst(engine, 10, (2, 2, 1.2)), .04)
        engine.ingest_message(diagnostics(10, bad=(1,)), .041)
        engine.ingest_message(burst(engine, 11, (2, 2, 1.2)), .044)
        self.assertGreater(engine.live[1].quality.extra_variance_scale, 1)
        engine.ingest_message(burst(engine, 1, (2, 2, 1.2), boot=2), .1)
        self.assertEqual(engine.live[1].quality.extra_variance_scale, 1)
        self.assertIsNone(engine.rx_quality)

    def test_weak_biased_links_improve_known_position_accuracy(self):
        enabled, disabled = UwbStateEngine(), UwbStateEngine()
        disabled.use_signal_quality = False
        rng = random.Random(312)
        errors = [[], []]
        for i in range(1, 701):
            xyz = (2.0 + .12 * math.sin(i * .004), 2, 1.2)
            bad = 100 <= i < 600
            msg = burst(enabled, i, xyz, noise=rng, bias={1: 110, 5: 110} if bad else {})
            if bad:
                msg = replace(msg, anchors=tuple(replace(a, fp_cdbm=-10400, rx_cdbm=-10000)
                    if a.anchor_id in (1, 5) else a for a in msg.anchors))
            for index, engine in enumerate((enabled, disabled)):
                engine.ingest_message(msg, i * .004)
                sol = engine.latest_solution
                self.assertIsNotNone(sol)
                if 150 <= i < 600:
                    errors[index].append(math.dist((sol.x, sol.y, sol.z), xyz))
        rmse = [math.sqrt(float(np.mean(np.square(v)))) for v in errors]
        self.assertLess(rmse[0], .75 * rmse[1], str(rmse))

    def test_low_first_path_to_noise_is_used_even_when_rx_fp_gap_is_small(self):
        enabled, disabled = UwbStateEngine(), UwbStateEngine()
        disabled.use_signal_quality = False
        rng = random.Random(313)
        errors = [[], []]
        for i in range(1, 601):
            xyz = (2, 2, 1.2)
            bad = i > 100
            msg = burst(enabled, i, xyz, noise=rng, bias={1: 110, 5: 110} if bad else {})
            for index, engine in enumerate((enabled, disabled)):
                if i % 25 == 0:
                    engine.ingest_message(diagnostics(i - 1, bad=(1, 5) if bad else ()), i * .004)
                engine.ingest_message(msg, i * .004)
                sol = engine.latest_solution
                self.assertIsNotNone(sol)
                if i > 150:
                    errors[index].append(math.dist((sol.x, sol.y, sol.z), xyz))
        rmse = [math.sqrt(float(np.mean(np.square(v)))) for v in errors]
        self.assertLess(rmse[0], .8 * rmse[1], str(rmse))

    def test_raw_quality_sidecar_replays_received_order_exactly(self):
        engine = UwbStateEngine()
        with tempfile.TemporaryDirectory() as tmp:
            logger = FastCsvLogger(Path(tmp), 'QUALITY_ORDER', engine)
            live_x = []
            try:
                for i in range(1, 90):
                    msg = burst(engine, i, (2, 2, 1.2), bias={1: 80})
                    engine.ingest_message(msg, i * .004)
                    live_x.append(engine.latest_solution.x)
                    logger.log_burst(msg, None, None, engine)
                    # Same-cycle diagnostics arrive AFTER the Burst was solved.
                    if i % 10 == 0:
                        quality = diagnostics(i, bad=(1,))
                        engine.ingest_message(quality, i * .004)
                        logger.log_rx_quality(quality)
            finally:
                logger.stop()
            events = [json.loads(s) for s in logger.quality_path.read_text().splitlines()]
            self.assertEqual([e['after_burst_rows'] for e in events], list(range(10, 90, 10)))
            replay = load_log_session(logger.csv_path, re_solve=True)
            np.testing.assert_allclose([f.x for f in replay.frames], live_x, atol=1e-12, rtol=0)
            self.assertEqual(logger.quality_records_written, 8)
            with logger.csv_path.open(newline='', encoding='utf-8') as f:
                rows = list(csv.DictReader(f))
            self.assertIn('a1_quality_score', rows[-1])
            self.assertGreater(float(rows[-1]['a1_quality_extra_r_scale']), 1)
            metadata = json.loads(logger.meta_path.read_text())
            self.assertFalse(metadata['solver']['radio_quality']['score_is_probability'])


if __name__ == '__main__':
    unittest.main()
