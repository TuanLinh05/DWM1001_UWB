"""Offline IO safeguards and numerical edge cases outside the supplied E2.0 contracts."""

import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

GUI = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(GUI), str(GUI / "tools")]
import flight_log_io as io
import aided_calibration as ac


class OfflineFlightIoTests(unittest.TestCase):
    def test_overlapping_train_and_holdout_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "overlap"):
            io.validate_intervals([(75, 195)], [(190, 218)])
        io.validate_intervals([(75, 195)], [(195, 218), (235, 300)])
        for bad in ("75:75", "nan:100", "1", "3:2"):
            with self.subTest(interval=bad), self.assertRaises(ValueError):
                io.parse_intervals(bad)

    def test_only_fully_calibrated_wire_ranges_are_read_without_double_bias(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            with (path / "session_metadata.json").open("w", encoding="utf-8") as fh:
                json.dump({"anchors": [{"anchor_id": i, "x": i % 2, "y": i // 2, "z": i / 3,
                                        "enabled": True, "bias_mm": 700} for i in range(1, 5)]}, fh)
            fields = ["tag_time_us", "cycle_seq", "boot_id"] + [f"a{i}_{s}" for i in range(1, 5) for s in ("mm", "flags")]
            with (path / "uwb_fast_log.csv").open("w", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=fields)
                writer.writeheader()
                writer.writerow({"tag_time_us": 1000, "cycle_seq": 1, "boot_id": 1,
                                  **{f"a{i}_mm": 3200 for i in range(1, 5)},
                                  **{f"a{i}_flags": flag for i, flag in enumerate((0x2C, 0x28, 0x24, 0x0C), 1)}})
            _, _, ranges = io.read_csv(path)
            self.assertEqual(ranges[0, 0], 3.2)
            self.assertTrue(np.isnan(ranges[0, 1:]).all())

    def test_rangefinder_freshness_uses_previous_sample_not_a_future_one(self):
        t = np.array([40., 40.4, 40.6, 41., 41.4, 41.6])
        bridge = {"distance_sensor": [{"fields": {"time_boot_ms": ts, "current_distance": 20, "orientation": 25}}
                                       for ts in (40000, 41000, 42000)]}
        rotations = np.repeat(np.eye(3)[None], len(t), axis=0)
        z, age, audit = io.height_reference(t, rotations, bridge, {}, {}, (-.09, 0, .31), ground_window=(40, 40.4))
        self.assertTrue(np.allclose(age, [0, .4, .6, 0, .4, .6]))
        self.assertTrue(np.isnan(z[[2, 5]]).all())
        self.assertTrue(np.allclose(z[[0, 1, 3, 4]], .51))
        self.assertEqual(audit["stale_epochs"], 2)

    def test_imu_timestamps_and_frame_flip_are_applied_once(self):
        d = {"timestamp": np.array([1000000, 1010000, 1022000]),
             "accelerometer_timestamp_relative": np.array([0, 1000, 2000])}
        for i, value in enumerate((1., 2., -9.81)):
            d[f"accelerometer_m_s2[{i}]"] = np.full(3, value)
        for i, value in enumerate((.1, .2, .3)):
            d[f"gyro_rad[{i}]"] = np.full(3, value)
        t, acc, gyro = io.read_imu({"sensor_combined": d})
        self.assertTrue(np.allclose(t, [1, 1.011, 1.024]))
        self.assertTrue(np.allclose(acc, [1, -2, 9.81]))
        self.assertTrue(np.allclose(gyro, [.1, -.2, -.3]))

    def test_yaw_alignment_uses_training_only_and_correct_ned_to_rig_sign(self):
        angle = .4
        rotation = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
        source = np.array([[0., 0.], [1., 0.], [0., 1.], [1., 1.]])
        target = source @ rotation + [2, 3]
        target[-1] += 100
        offset, audit = io.estimate_rig_yaw_offset(source, target, np.array([True, True, True, False]))
        self.assertAlmostEqual(offset, np.pi / 2 + angle)
        self.assertLess(audit["rmse_m"], 1e-12)

    def test_clock_fit_preserves_time_order_and_checks_cache_identity(self):
        t = np.arange(3000) * .01
        columns = {"tag_time_us": 100000000 + t * 1e6, "cycle_seq": np.arange(len(t))}
        receives = np.column_stack((columns["cycle_seq"], columns["tag_time_us"] / 1e6, 1700000000 + 100 + t))
        bridge = {"attitude": [{"host_s": 1700000100.02, "age_s": .02, "fields": {"time_boot_ms": 100000}}]}
        mapped, audit = io.map_tag_to_px4_clock(columns, bridge, receives)
        self.assertLess(np.max(np.abs(mapped - (100 + t))), 1e-6)
        self.assertTrue(np.all(np.diff(mapped) > 0))
        self.assertGreaterEqual(audit["envelope_bins"], 3)
        receives[2, 0] += 1
        with self.assertRaisesRegex(ValueError, "sequence"):
            io.map_tag_to_px4_clock(columns, bridge, receives)

    def test_optional_dependency_error_and_negative_lever_syntax(self):
        with patch.object(io.importlib, "import_module", side_effect=ImportError("missing")):
            with self.assertRaisesRegex(RuntimeError, "pip install pyulog"):
                io.dependency("pyulog")
        args = io.vector_arguments(["flight", "--lever", "-0.09,0,0.31", "--train", "75:195"])
        self.assertIn("--lever=-0.09,0,0.31", args)
        self.assertEqual(io.parse_lever("-0.09,0,0.31"), (-.09, 0., .31))

    def test_one_degenerate_epoch_does_not_break_other_position_solves(self):
        anchors = [io.FlightAnchor(i + 1, x, y, 0) for i, (x, y) in enumerate(((0, 0), (3, 0), (3, 3), (0, 3)))]
        xyz = np.array([[a.x, a.y, a.z] for a in anchors])
        true = np.array([1.1, 1.2, 1.])
        ranges = np.tile(np.linalg.norm(true - xyz, axis=1), (2, 1))
        result = ac.solve_positions_3d(anchors, ranges, [[1.3, 1.4, .8], [1.1, 1.2, 0]])
        self.assertLess(np.linalg.norm(result[0] - true), 1e-7)
        self.assertTrue(np.isnan(result[1]).all())


if __name__ == "__main__":
    unittest.main()
