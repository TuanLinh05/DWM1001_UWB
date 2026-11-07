"""Additional calibration/report regressions beyond the unchanged G0 contracts."""

from dataclasses import replace
import csv
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from g0_fixtures import ROOM_54, SPOTS_4, TRUE_BIAS_MM, feed_spot_capture, spot_observation, true_range_mm, write_session
from core_state import UwbStateEngine
import multi_spot_calibration as msc
import radio_loss_report as radio
import static_dataset_report as static


class CalibrationRegressionTests(unittest.TestCase):
    def test_rank_deficient_geometry_uses_legacy_and_serializes(self):
        engine = UwbStateEngine(ROOM_54)
        cycle = feed_spot_capture(engine, SPOTS_4[0], SPOTS_4[0])
        feed_spot_capture(engine, SPOTS_4[0], SPOTS_4[0], first_cycle=cycle)
        result = engine.joint_calibration()
        self.assertFalse(result.identifiable)
        self.assertIn("NOT_IDENTIFIABLE", result.warning_codes())
        self.assertEqual(engine.active_calibration_method(), "median")
        self.assertEqual(engine.aggregated_calibration(), engine._median_aggregated_calibration())
        self.assertIsNone(result.to_dict()["condition_number"])
        json.dumps(engine.calibration_summary(), allow_nan=False)

    def test_cache_tracks_settings_geometry_enabled_and_capture_identity(self):
        engine = UwbStateEngine(ROOM_54)
        cycle = feed_spot_capture(engine, SPOTS_4[0], SPOTS_4[0])
        feed_spot_capture(engine, SPOTS_4[1], SPOTS_4[1], first_cycle=cycle)
        old = engine.joint_calibration()
        engine.joint_settings = replace(engine.joint_settings, systematic_sigma_mm=25.0)
        self.assertIsNot(engine.joint_calibration(), old)
        old = engine.joint_calibration()
        engine.anchors[1].x += 0.01
        self.assertIsNot(engine.joint_calibration(), old)
        old = engine.joint_calibration()
        engine.anchors[8].enabled = False
        self.assertIsNot(engine.joint_calibration(), old)
        self.assertNotIn(8, engine.joint_calibration().biases)
        old = engine.joint_calibration()
        engine.spot_captures[0] = replace(engine.spot_captures[0])
        self.assertIsNot(engine.joint_calibration(), old)
        old = engine.joint_calibration()
        engine.reset_live_state()
        self.assertIs(engine.joint_calibration(), old)

    def test_removing_middle_capture_keeps_unique_ids_and_clear_restarts(self):
        engine = UwbStateEngine(ROOM_54)
        cycle = 1
        for p in SPOTS_4[:3]:
            cycle = feed_spot_capture(engine, p, p, first_cycle=cycle)
        engine.remove_capture(2)
        feed_spot_capture(engine, SPOTS_4[3], SPOTS_4[3], first_cycle=cycle)
        self.assertEqual([c.spot_index for c in engine.spot_captures], [1, 3, 4])
        engine.spot_captures.clear()
        feed_spot_capture(engine, SPOTS_4[0], SPOTS_4[0], first_cycle=1000)
        self.assertEqual(engine.spot_captures[0].spot_index, 1)

    def test_tape_only_and_nonfinite_ranges_have_safe_fallbacks(self):
        tape = msc.TapeObservation(1, 4, 2000.0, 2243.0)
        result = msc.solve_joint_calibration(ROOM_54, [], [tape])
        self.assertEqual(result.method, "single_spot_fallback")
        self.assertEqual(result.biases[4].bias_mm, 243.0)
        s = spot_observation(1, SPOTS_4[0], SPOTS_4[0])
        result = msc.solve_joint_calibration(ROOM_54, [replace(s, median_raw_mm={1: math.nan, 2: math.inf, 3: -1})])
        self.assertFalse(result.ok)
        json.dumps(result.to_dict(), allow_nan=False)

    def test_invalid_settings_return_diagnostics_instead_of_raising(self):
        spots = [spot_observation(j, p, p) for j, p in enumerate(SPOTS_4, 1)]
        for changes in ({"systematic_sigma_mm": None}, {"huber_k": 0}, {"min_spots": None},
                        {"max_iterations": 1.5}, {"spot_offset_warn_m": math.nan}):
            result = msc.solve_joint_calibration(ROOM_54, spots, settings=replace(msc.JointCalibrationSettings(), **changes))
            self.assertEqual(result.method, "failed")
            self.assertIn("NOT_IDENTIFIABLE", result.warning_codes())
            json.dumps(result.to_dict(), allow_nan=False)


class ReportRegressionTests(unittest.TestCase):
    def test_old_csv_without_status_and_with_late_flag(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = write_session(Path(tmp), 100, range_mm=lambda row, aid: 3000.0,
                                   late=lambda row, aid: aid == 1 and row % 2 == 0)
            csv_path = folder / "uwb_fast_log.csv"
            with csv_path.open(encoding="utf-8", newline="") as fh:
                reader = csv.DictReader(fh)
                fields = [f for f in reader.fieldnames if not f.endswith("_status_code")]
                rows = list(reader)
            with csv_path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
                writer.writeheader()
                writer.writerows(rows)
            data = radio.load_session(folder)
            self.assertFalse(data.has_status_columns)
            self.assertTrue(data.polled.all())
            self.assertEqual(radio.anchor_stats(data)[1].late_pct, 50.0)

    def test_sequence_wrap_and_boot_boundary_do_not_create_false_gaps_or_outages(self):
        times = np.array([1.0, 1.1, 1.2, 1.0, 1.1, 1.2])
        data = radio.SessionData("boots", times, np.array([1, 1, 1, 2, 2, 2]),
                 np.array([2 ** 32 - 2, 2 ** 32 - 1, 2, 0, 1, 2]), np.full(6, 100000.0),
                 np.zeros((6, 8), dtype=bool), np.ones((6, 8), dtype=bool),
                 np.zeros((6, 8), dtype=bool), True)
        self.assertEqual(radio.lost_cycles(data), 2)
        stats = radio.anchor_stats(data, outage_threshold_s=0.15)
        self.assertEqual(stats[1].outages, 2)
        self.assertAlmostEqual(stats[1].longest_outage_s, 0.2)
        self.assertEqual(sum(b.cycles for b in radio.time_bins(data)), 6)

    def test_missing_anchor_and_position_fail_acceptance(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = write_session(Path(tmp), 101, range_mm=lambda row, aid: None if aid == 4 else 3000.0,
                                   labels={0: "P_start", 100: "P_end"})
            report = static.build_report(folder, {"points": {"P": [1, 1, 1]}, "trim_s": 0})
            reasons = report["acceptance"]["points"]["P"]["reasons"]
            self.assertFalse(report["acceptance"]["pass"])
            self.assertTrue(any("A4" in r for r in reasons))
            self.assertTrue(any("No usable position" in r for r in reasons))
            json.dumps(report, allow_nan=False)

    def test_static_correction_uses_bias_at_each_sample(self):
        rows = [{"tag_time_us": str(i * 4000), "a1_mm": str(1000 + (500 if i % 2 else 0)),
                 "a1_bias_mm": str(500 if i % 2 else 0)} for i in range(100)]
        segment = static.Segment("P", (1, 0, 0), 0, 99, 0, 0.396)
        result = static.segment_anchor_stats(rows, segment, {1: (0, 0, 0, 120.0)})[1]
        self.assertEqual(result.host_bias_mm, 250.0)
        self.assertEqual(result.residual_after_cal_mm, 0.0)

    def test_static_report_restores_firmware_offsets_before_host_correction(self):
        with tempfile.TemporaryDirectory() as tmp:
            point = SPOTS_4[0]
            folder = write_session(Path(tmp), 101,
                 range_mm=lambda row, aid: true_range_mm(point, ROOM_54[aid - 1]),
                 host_bias_mm=TRUE_BIAS_MM, position=lambda row: point, labels={0: "P_start", 100: "P_end"})
            meta_path = folder / "session_metadata.json"
            with meta_path.open(encoding="utf-8") as fh:
                meta = json.load(fh)
            for a in meta["anchors"]:
                a["fw_calibrated"] = True
                a["fw_bias_mm"] = TRUE_BIAS_MM[a["anchor_id"]]
            with meta_path.open("w", encoding="utf-8") as fh:
                json.dump(meta, fh)
            csv_path = folder / "uwb_fast_log.csv"
            with csv_path.open(encoding="utf-8", newline="") as fh:
                reader = csv.DictReader(fh)
                fields = reader.fieldnames
                rows = list(reader)
            for row in rows:
                for aid in range(1, 9):
                    row[f"a{aid}_flags"] = str(0x24)
            with csv_path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
            before = csv_path.read_bytes()
            report = static.build_report(folder, {"points": {"P": list(point)}, "trim_s": 0})
            self.assertTrue(report["acceptance"]["pass"], report["acceptance"])
            self.assertEqual(csv_path.read_bytes(), before)
            self.assertEqual(report["input_warnings"], [])
            for a in report["segments"][0]["anchors"].values():
                self.assertLess(abs(a["residual_after_cal_mm"]), 0.6)
            for row in rows:
                for aid in range(1, 9):
                    row[f"a{aid}_flags"] = ""
            with csv_path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
            report = static.build_report(folder, {"points": {"P": list(point)}, "trim_s": 0})
            self.assertTrue(report["acceptance"]["pass"])
            for a in meta["anchors"]:
                del a["fw_bias_mm"]
            with meta_path.open("w", encoding="utf-8") as fh:
                json.dump(meta, fh)
            report = static.build_report(folder, {"points": {"P": list(point)}, "trim_s": 0})
            self.assertFalse(report["acceptance"]["pass"])
            self.assertEqual(len(report["input_warnings"]), 8)

    def test_empty_radio_session_has_strict_json_and_no_fake_environment(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = write_session(Path(tmp), 0, range_mm=lambda row, aid: 3000.0)
            report = radio.build_report(folder)
            self.assertEqual(report["cycles"], 0)
            self.assertEqual(sum(report["histogram"]), 0)
            self.assertIsNone(report["temperature_correlation"])
            self.assertIsNone(report["anchors"]["1"]["range_ok_given_polled_pct"])
            json.dumps(report, allow_nan=False)


if __name__ == "__main__":
    unittest.main()
