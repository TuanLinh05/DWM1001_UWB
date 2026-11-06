"""TDD test suite for New_GUI core state, firmware compatibility, 2D/3D solver, and calibration."""

from __future__ import annotations

from pathlib import Path
import math
import re
import struct
import sys
import tempfile
import unittest

NEW_GUI_DIR = Path(__file__).resolve().parents[1]
OLD_GUI_DIR = NEW_GUI_DIR.parent / "UWB_UART_GUI"
FIRMWARE_COMMON_DIR = NEW_GUI_DIR.parents[1] / "Firmware" / "common"

sys.path.insert(0, str(NEW_GUI_DIR))
sys.path.insert(0, str(OLD_GUI_DIR))

import telemetry_protocol as tp  # noqa: E402

from core_state import (  # noqa: E402
    AnchorConfig,
    AccuracyReport,
    CalibrationSpotCapture,
    DopMetrics,
    Pose3D,
    UwbStateEngine,
    compute_dop,
    encode_set_ds_cal_command,
    load_anchors_rig_txt,
    save_anchors_rig_txt,
    solve_position_robust,
)


class FirmwareCompatibilityTests(unittest.TestCase):
    """Verify New_GUI protocol & command constants match current Firmware/common headers."""

    def _read_define(self, rel_path: str, macro: str) -> int:
        header_path = FIRMWARE_COMMON_DIR / rel_path
        self.assertTrue(header_path.is_file(), f"Missing firmware header: {header_path}")
        text = header_path.read_text(encoding="utf-8")
        match = re.search(rf"^#define\s+{macro}\s+(0x[0-9A-Fa-f]+|\d+)U?\b", text, re.MULTILINE)
        self.assertIsNotNone(match, f"{macro} not found in {rel_path}")
        return int(match.group(1), 0)

    def test_firmware_headers_match_protocol_constants(self) -> None:
        self.assertEqual(
            self._read_define("telemetry/telemetry_frame.h", "TELEM_TYPE_RANGE_BURST"),
            tp.TYPE_RANGE_BURST,
        )
        self.assertEqual(
            self._read_define("telemetry/telemetry_frame.h", "TELEM_TYPE_RX_QUALITY"),
            tp.TYPE_RX_QUALITY,
        )
        self.assertEqual(
            self._read_define("telemetry/telemetry.h", "TELEM_INFO_FLAG_BURST"),
            tp.INFO_FLAG_BURST,
        )
        self.assertEqual(
            self._read_define("telemetry/telemetry.h", "TELEM_RANGE_MEAS_MIN_BAUD"),
            tp.TELEM_RANGE_MEAS_MIN_BAUD,
        )
        self.assertEqual(
            self._read_define("services/uwb_cmd.h", "UWB_CMD_SET_DS_CAL"),
            tp.CMD_SET_DS_CAL,
        )
        self.assertEqual(
            self._read_define("services/uwb_cmd.h", "UWB_CMD_GET_DS_CAL"),
            tp.CMD_GET_DS_CAL,
        )
        self.assertEqual(
            self._read_define("services/uwb_cmd.h", "UWB_CMD_SAVE_SETTINGS"),
            tp.CMD_SAVE_SETTINGS,
        )
        self.assertEqual(
            self._read_define("services/uwb_cmd.h", "UWB_CMD_SET_BURST"),
            tp.CMD_SET_BURST,
        )
        self.assertEqual(
            self._read_define("ranging/tag_ranging.h", "TAG_BURST_REC_CAL_OK"),
            tp.BURST_FLAG_CAL_OK,
        )
        self.assertEqual(
            self._read_define("ranging/tag_ranging.h", "TAG_BURST_REC_VALID"),
            tp.BURST_FLAG_VALID,
        )
        self.assertEqual(
            self._read_define("ranging/tag_ranging.h", "TAG_BURST_REC_RADIO_OK"),
            tp.BURST_FLAG_RADIO_OK,
        )


class SolverAndDopTests(unittest.TestCase):
    def setUp(self) -> None:
        # 8-anchor 3D box (5 x 4 x 3 m)
        self.anchors_3d = [
            AnchorConfig(1, 0.0, 0.0, 0.0),
            AnchorConfig(2, 5.0, 0.0, 0.0),
            AnchorConfig(3, 0.0, 4.0, 0.0),
            AnchorConfig(4, 5.0, 4.0, 0.0),
            AnchorConfig(5, 0.0, 0.0, 3.0),
            AnchorConfig(6, 5.0, 0.0, 3.0),
            AnchorConfig(7, 0.0, 4.0, 3.0),
            AnchorConfig(8, 5.0, 4.0, 3.0),
        ]

    def test_solve_2d_and_3d_exact_and_outlier_rejection(self) -> None:
        true_pos = (2.1, 1.7, 1.35)
        ranges_mm = {
            a.anchor_id: round(math.dist(true_pos, (a.x, a.y, a.z)) * 1000.0)
            for a in self.anchors_3d
        }
        # Inject a 1.5 m NLOS spike on Anchor 6
        corrupted = dict(ranges_mm)
        corrupted[6] += 1500

        sol_3d = solve_position_robust(self.anchors_3d, corrupted, dimension=3)
        self.assertIsNotNone(sol_3d)
        assert sol_3d is not None
        self.assertAlmostEqual(sol_3d.x, true_pos[0], places=2)
        self.assertAlmostEqual(sol_3d.y, true_pos[1], places=2)
        self.assertAlmostEqual(sol_3d.z, true_pos[2], places=2)
        self.assertIn(6, sol_3d.excluded_anchor_ids)

        sol_2d = solve_position_robust(
            self.anchors_3d[:4],
            {k: ranges_mm[k] for k in range(1, 5)},
            dimension=2,
            fixed_z=true_pos[2],
        )
        self.assertIsNotNone(sol_2d)
        assert sol_2d is not None
        self.assertAlmostEqual(sol_2d.x, true_pos[0], places=2)
        self.assertAlmostEqual(sol_2d.y, true_pos[1], places=2)
        self.assertAlmostEqual(sol_2d.z, true_pos[2], places=2)

    def test_compute_dop_warns_on_coplanar_3d(self) -> None:
        dop_good = compute_dop(self.anchors_3d, (2.5, 2.0, 1.5), dimension=3)
        self.assertIsInstance(dop_good, DopMetrics)
        self.assertLess(dop_good.pdop, 3.0)
        self.assertFalse(dop_good.coplanar_warning)

        coplanar = [AnchorConfig(i, a.x, a.y, 0.48) for i, a in enumerate(self.anchors_3d[:4], 1)]
        dop_flat = compute_dop(coplanar, (2.5, 2.0, 1.0), dimension=3)
        self.assertTrue(dop_flat.coplanar_warning)


class BurstIngestionAndCalibrationTests(unittest.TestCase):
    def test_burst_stream_updates_live_position_even_with_cal_missing_when_allowed(self) -> None:
        engine = UwbStateEngine()
        true_tag = (1.8, 2.2, 1.2)
        # Apply a known host bias of +50 mm on A1 (meaning raw measures 50 mm too long)
        anchors = list(engine.anchors.values())
        anchors[0] = AnchorConfig(1, anchors[0].x, anchors[0].y, anchors[0].z, bias_mm=50.0)
        engine.set_anchors(anchors)
        engine.allow_uncalibrated_positioning = True

        for cycle in range(1, 15):
            burst_entries = []
            for a in engine.anchors.values():
                true_mm = math.dist(true_tag, (a.x, a.y, a.z)) * 1000.0
                raw_mm = round(true_mm + a.bias_mm)
                # Simulate firmware with UWB_DS_CALIBRATED_MASK=0: RADIO_OK set, CAL_OK=0, VALID=0
                flags = tp.BURST_FLAG_RADIO_OK
                burst_entries.append(
                    tp.BurstAnchor(
                        anchor_id=a.anchor_id,
                        status=tp.STATUS_CALIBRATION_MISSING,
                        flags=flags,
                        range_mm=raw_mm,
                        fp_cdbm=-8100,
                        rx_cdbm=-7700,
                        ci_ppm_x100=-150,
                    )
                )
            msg = tp.RangeBurstMessage(
                sequence=cycle,
                time_ms=cycle * 5,
                boot_id=0x1234,
                cycle_seq=cycle,
                meas_seq_first=cycle * 8,
                t_us=cycle * 5000,
                period_us=5000,
                anchors=tuple(burst_entries),
            )
            engine.ingest_message(msg, received_s=cycle * 0.005)

        self.assertIsNotNone(engine.latest_solution)
        sol = engine.latest_solution
        assert sol is not None
        self.assertAlmostEqual(sol.x, true_tag[0], delta=0.02)
        self.assertAlmostEqual(sol.y, true_tag[1], delta=0.02)
        self.assertAlmostEqual(sol.z, true_tag[2], delta=0.02)
        self.assertGreater(engine.cycle_rate_hz, 150.0)

    def test_multi_anchor_3d_spot_calibration_and_cmd_encoding(self) -> None:
        engine = UwbStateEngine()
        spot_xyz = (2.0, 1.5, 1.1)
        injected_biases_mm = {i: (-40.0 + i * 15.0) for i in range(1, 9)}

        engine.start_spot_calibration(spot_xyz=spot_xyz, target_samples=30)
        for cycle in range(1, 35):
            entries = []
            for a in engine.anchors.values():
                d_mm = math.dist(spot_xyz, (a.x, a.y, a.z)) * 1000.0
                raw_mm = round(d_mm + injected_biases_mm[a.anchor_id])
                entries.append(
                    tp.BurstAnchor(
                        anchor_id=a.anchor_id,
                        status=tp.STATUS_CALIBRATION_MISSING,
                        flags=tp.BURST_FLAG_RADIO_OK,
                        range_mm=raw_mm,
                        fp_cdbm=-8000,
                        rx_cdbm=-7600,
                        ci_ppm_x100=100,
                    )
                )
            msg = tp.RangeBurstMessage(
                sequence=cycle,
                time_ms=cycle * 5,
                boot_id=1,
                cycle_seq=cycle,
                meas_seq_first=cycle * 8,
                t_us=cycle * 5000,
                period_us=5000,
                anchors=tuple(entries),
            )
            engine.ingest_message(msg, received_s=cycle * 0.005)

        self.assertFalse(engine.calibration_active)
        self.assertEqual(len(engine.spot_captures), 1)
        capture: CalibrationSpotCapture = engine.spot_captures[0]
        for aid, true_bias in injected_biases_mm.items():
            res = capture.anchor_results[aid]
            self.assertAlmostEqual(res.bias_mm, true_bias, delta=1.5)
            self.assertAlmostEqual(res.offset_mm, -true_bias, delta=1.5)

        # Verify SET_DS_CAL payload matches Firmware/common/services/uwb_cmd.c (<HiB, bias_um)
        frame = encode_set_ds_cal_command(sequence=42, anchor_id=3, bias_mm=injected_biases_mm[3], calibrated=True)
        parsed = tp.TelemetryStreamParser().feed(frame)[0]
        self.assertEqual(parsed.type, tp.TYPE_CMD)
        self.assertEqual(parsed.payload[0], tp.CMD_SET_DS_CAL)
        aid, bias_um, cal_flag = struct.unpack("<HiB", parsed.payload[1:])
        self.assertEqual(aid, 3)
        self.assertEqual(bias_um, round(injected_biases_mm[3] * 1000.0))
        self.assertEqual(cal_flag, 1)

    def test_accuracy_hold_evaluation_2d_and_3d(self) -> None:
        engine = UwbStateEngine()
        engine.allow_uncalibrated_positioning = True
        ref_xyz = (2.5, 2.0, 1.5)
        engine.start_accuracy_eval(reference_xyz=ref_xyz, target_samples=25)

        for cycle in range(1, 30):
            entries = []
            for a in engine.anchors.values():
                d_mm = round(math.dist(ref_xyz, (a.x, a.y, a.z)) * 1000.0)
                entries.append(
                    tp.BurstAnchor(
                        a.anchor_id, 0, tp.BURST_FLAG_RADIO_OK | tp.BURST_FLAG_CAL_OK | tp.BURST_FLAG_VALID,
                        d_mm, -8000, -7600, 0
                    )
                )
            msg = tp.RangeBurstMessage(
                cycle, cycle * 10, 1, cycle, cycle * 8, cycle * 10000, 10000, tuple(entries)
            )
            engine.ingest_message(msg, received_s=cycle * 0.01)

        self.assertFalse(engine.accuracy_eval_active)
        report: AccuracyReport | None = engine.latest_accuracy_report
        self.assertIsNotNone(report)
        assert report is not None
        self.assertEqual(report.samples, 25)
        self.assertLess(report.rmse_2d_m, 0.01)
        self.assertLess(report.rmse_3d_m, 0.01)
        self.assertLess(report.cep95_m, 0.01)
        self.assertLess(report.sep95_m, 0.01)

    def test_anchors_rig40_txt_roundtrip(self) -> None:
        from core_state import DEFAULT_PRESET_RIG40
        rig_path = NEW_GUI_DIR.parents[1] / "Reference" / "IMGUI_UWB_CALIB" / "anchors_rig40.txt"
        if rig_path.is_file():
            loaded = load_anchors_rig_txt(rig_path)
            self.assertEqual(len(loaded), 8)
            self.assertAlmostEqual(loaded[0].z, 0.48)
            self.assertAlmostEqual(loaded[4].z, 1.58)
            self.assertAlmostEqual(loaded[2].bias_mm, -178.0)
        else:
            loaded = list(DEFAULT_PRESET_RIG40)

        with tempfile.TemporaryDirectory() as tmpdir:
            out_path = Path(tmpdir) / "anchors_out.txt"
            save_anchors_rig_txt(out_path, loaded)
            reloaded = load_anchors_rig_txt(out_path)
            for a, b in zip(loaded, reloaded):
                self.assertEqual(a.anchor_id, b.anchor_id)
                self.assertAlmostEqual(a.x, b.x, places=3)
                self.assertAlmostEqual(a.y, b.y, places=3)
                self.assertAlmostEqual(a.z, b.z, places=3)
                self.assertAlmostEqual(a.bias_mm, b.bias_mm, places=1)

    def test_stationary_anti_jump_and_glitch_rejection(self) -> None:
        from core_state import DEFAULT_PRESET_ROOM_66_36
        engine = UwbStateEngine(initial_anchors=DEFAULT_PRESET_ROOM_66_36)
        true_xyz = (4.10, 1.05, 0.43)
        zs: list[float] = []
        for cycle in range(1, 120):
            t_s = cycle * 0.025
            entries = []
            for a in engine.anchors.values():
                if a.anchor_id == 3:
                    # Simulate A3 offline
                    entries.append(tp.BurstAnchor(a.anchor_id, tp.STATUS_TIMEOUT, 0, None, tp.INT16_MIN, tp.INT16_MIN, tp.INT16_MIN))
                    continue
                if a.anchor_id == 6 and 60 <= cycle <= 80:
                    # Simulate 0.5s RF fade on A6
                    entries.append(tp.BurstAnchor(a.anchor_id, tp.STATUS_TIMEOUT, 0, None, tp.INT16_MIN, tp.INT16_MIN, tp.INT16_MIN))
                    continue
                d_mm = round(math.dist(true_xyz, (a.x, a.y, a.z)) * 1000.0)
                if a.anchor_id == 1 and cycle == 70:
                    d_mm = 33685  # Single-packet hardware glitch
                entries.append(
                    tp.BurstAnchor(
                        a.anchor_id, 0, tp.BURST_FLAG_RADIO_OK | tp.BURST_FLAG_CAL_OK | tp.BURST_FLAG_VALID,
                        d_mm, -8000, -7600, 0
                    )
                )
            msg = tp.RangeBurstMessage(cycle, int(t_s * 1000), 1, cycle, cycle * 8, int(t_s * 1e6), 25000, tuple(entries))
            engine.ingest_message(msg, received_s=t_s)
            if engine.latest_solution is not None and cycle >= 20:
                zs.append(engine.latest_solution.z)

        self.assertGreater(len(zs), 50)
        self.assertLess(max(zs) - min(zs), 0.025)
        self.assertAlmostEqual(zs[-1], true_xyz[2], delta=0.02)

    def test_3d_loop_motion_tracking_and_waypoints(self) -> None:
        from core_state import DEFAULT_PRESET_ROOM_66_36
        engine = UwbStateEngine(initial_anchors=DEFAULT_PRESET_ROOM_66_36)
        # Walk a full 3D loop around the 6.6 x 3.6 m room over 20 seconds (at 50 Hz = 1000 steps)
        errors_3d: list[float] = []
        total_steps = 400
        dt = 0.05  # 20 seconds total
        for step in range(1, total_steps + 1):
            t_s = step * dt
            theta = 2.0 * math.pi * (step / total_steps)
            true_x = 3.30 + 2.20 * math.cos(theta)
            true_y = 1.80 + 1.10 * math.sin(theta)
            true_z = 0.85 + 0.25 * math.sin(2.0 * theta)
            true_xyz = (true_x, true_y, true_z)

            entries = []
            for a in engine.anchors.values():
                # Simulate realistic asymmetric anchor rates (A2 only updates every 15th frame, A3 offline)
                if a.anchor_id == 3:
                    entries.append(tp.BurstAnchor(a.anchor_id, tp.STATUS_TIMEOUT, 0, None, tp.INT16_MIN, tp.INT16_MIN, tp.INT16_MIN))
                    continue
                if a.anchor_id == 2 and (step % 15 != 0):
                    entries.append(tp.BurstAnchor(a.anchor_id, tp.STATUS_TIMEOUT, 0, None, tp.INT16_MIN, tp.INT16_MIN, tp.INT16_MIN))
                    continue
                d_mm = round(math.dist(true_xyz, (a.x, a.y, a.z)) * 1000.0 + a.bias_mm)
                entries.append(
                    tp.BurstAnchor(
                        a.anchor_id, 0, tp.BURST_FLAG_RADIO_OK | tp.BURST_FLAG_CAL_OK | tp.BURST_FLAG_VALID,
                        d_mm, -8000, -7600, 0
                    )
                )
            msg = tp.RangeBurstMessage(step, int(t_s * 1000), 1, step, step * 8, int(t_s * 1e6), 50000, tuple(entries))
            engine.ingest_message(msg, received_s=t_s)

            if step in (100, 200, 300, 400):
                wp = engine.mark_waypoint(f"Corner_{step // 100}")
                self.assertIsNotNone(wp)

            if engine.latest_solution is not None and step >= 15:
                sol = engine.latest_solution
                errors_3d.append(math.dist((sol.x, sol.y, sol.z), true_xyz))

        self.assertGreater(len(errors_3d), 350)
        mean_err_m = sum(errors_3d) / len(errors_3d)
        self.assertLess(mean_err_m, 0.045)
        # Ensure full loop path is preserved in map_path and 4 waypoints were marked
        self.assertGreater(len(engine.map_path), 100)
        self.assertEqual(len(engine.waypoints), 4)
        xs = [p[0] for p in engine.map_path]
        ys = [p[1] for p in engine.map_path]
        self.assertGreater(max(xs) - min(xs), 4.0)
        self.assertGreater(max(ys) - min(ys), 2.0)


if __name__ == "__main__":
    unittest.main()
