"""TDD test suite for New_GUI design tokens, background workers, and PyQt6 main window."""

from __future__ import annotations

import os
from pathlib import Path
import struct
import sys
import tempfile
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

NEW_GUI_DIR = Path(__file__).resolve().parents[1]
OLD_GUI_DIR = NEW_GUI_DIR.parent / "UWB_UART_GUI"
if str(NEW_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(NEW_GUI_DIR))
if str(OLD_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(OLD_GUI_DIR))

from PyQt6.QtWidgets import QApplication  # noqa: E402
import telemetry_protocol as tp  # noqa: E402

from design_tokens import (  # noqa: E402
    DARK_QSS,
    LIGHT_QSS,
    SPACING_MD,
    SPACING_SM,
    SPACING_UNIT,
    ThemeManager,
)
from serial_worker import DemoBurstWorker, FastCsvLogger  # noqa: E402
from main_gui import UwbFastGuiWindow  # noqa: E402


_APP = QApplication.instance() or QApplication([])


class DesignTokensAndWorkerTests(unittest.TestCase):
    def test_design_tokens_4px_grid_and_theme_manager(self) -> None:
        self.assertEqual(SPACING_UNIT, 4)
        self.assertEqual(SPACING_SM % 4, 0)
        self.assertEqual(SPACING_MD % 4, 0)
        self.assertIn("QMainWindow", LIGHT_QSS)
        self.assertIn("QMainWindow", DARK_QSS)

        tm = ThemeManager(_APP)
        tm.apply_dark()
        self.assertTrue(tm.is_dark)
        tm.toggle()
        self.assertFalse(tm.is_dark)

    def test_demo_burst_worker_and_command_roundtrip(self) -> None:
        received: list[object] = []
        worker = DemoBurstWorker(rate_hz=100)
        worker.batch_ready.connect(lambda batch: received.extend(batch))
        worker.start()
        try:
            worker.send_command(tp.CMD_GET_DS_CAL)
            worker.send_set_ds_cal(anchor_id=2, bias_mm=34.5, calibrated=True)
            deadline = time.monotonic() + 1.2
            while time.monotonic() < deadline and len(received) < 8:
                _APP.processEvents()
                time.sleep(0.02)
        finally:
            worker.stop()

        bursts = [m for m, _t in received if isinstance(m, tp.RangeBurstMessage)]
        acks = [m for m, _t in received if isinstance(m, tp.CmdAckMessage)]
        self.assertGreater(len(bursts), 2)
        self.assertTrue(any(a.command_id == tp.CMD_SET_DS_CAL and a.ok for a in acks))

    def test_fast_csv_logger_writes_burst_and_position_rows(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            logger = FastCsvLogger(Path(tmpdir), "TEST_DEMO")
            try:
                anchor = tp.BurstAnchor(1, 0, tp.BURST_FLAG_RADIO_OK | tp.BURST_FLAG_VALID, 2500, -8000, -7600, 0)
                msg = tp.RangeBurstMessage(1, 10, 1, 1, 1, 10000, 5000, (anchor,))
                logger.log_burst(msg, pos_xyz=(1.2, 2.3, 0.9), rms_m=0.015)
            finally:
                logger.stop()
            self.assertTrue((logger.session_dir / "uwb_fast_log.csv").is_file())
            content = (logger.session_dir / "uwb_fast_log.csv").read_text(encoding="utf-8")
            self.assertIn("cycle_seq", content)
            self.assertIn("2500", content)


class MainWindowIntegrationTests(unittest.TestCase):
    def test_main_window_lifecycle_2d_3d_and_calibration(self) -> None:
        win = UwbFastGuiWindow(start_demo=True)
        try:
            # Let demo feed some burst cycles
            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline and win.engine.total_bursts < 5:
                _APP.processEvents()
                time.sleep(0.02)

            self.assertGreaterEqual(win.engine.total_bursts, 3)
            self.assertIsNotNone(win.engine.latest_solution)

            # Switch between 2D and 3D modes
            win.set_dimension_mode(2)
            self.assertEqual(win.engine.solver_dimension, 2)
            win.set_dimension_mode(3)
            self.assertEqual(win.engine.solver_dimension, 3)

            # Trigger fast spot calibration
            win.engine.start_spot_calibration((2.5, 2.0, 1.2), target_samples=10)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and win.engine.calibration_active:
                _APP.processEvents()
                time.sleep(0.02)

            self.assertFalse(win.engine.calibration_active)
            self.assertGreaterEqual(len(win.engine.spot_captures), 1)
            applied = win.apply_calibrated_biases_to_host()
            self.assertEqual(len(applied), 8)

            # Exercise 1D single-anchor calibration, accuracy benchmark, theme toggle, and all 3 pages
            win.engine.start_single_anchor_calibration(anchor_id=1, reference_m=2.5, target_samples=10)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and win.engine.calibration_active:
                _APP.processEvents()
                time.sleep(0.02)
            self.assertGreaterEqual(len(win.engine.spot_captures), 2)

            win.engine.start_accuracy_eval((2.5, 2.0, 1.2), target_samples=10)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and win.engine.accuracy_eval_active:
                _APP.processEvents()
                time.sleep(0.02)
            self.assertIsNotNone(win.engine.latest_accuracy_report)

            for page_idx in (0, 1, 2, 0):
                win.pages.setCurrentIndex(page_idx)
                win._on_ui_refresh_tick()
                _APP.processEvents()

            win._toggle_theme()
            win._on_ui_refresh_tick()
            _APP.processEvents()
        finally:
            win.close()

    def test_real_hardware_messages_do_not_crash_gui(self) -> None:
        """Verify DeviceInfoMessage, CmdAckMessage, DiagSystemMessage, RxQualityMessage from real TAG do not crash UI."""
        win = UwbFastGuiWindow(start_demo=False)
        try:
            now = time.monotonic()
            dev_info = tp.DeviceInfoMessage(
                sequence=3693,
                time_ms=405971,
                role=1,
                node_address=0,
                git_hash=0x2D19AC75,
                git_dirty=True,
                frame_version=3,
                telemetry_features=7,
                wait4resp=True,
                device_id=2963110252,
                boot_id=41082,
                boot_count=1,
                reset_cause=0,
                tx_power_mode=0,
                reference_tuning=False,
                tx_antenna_delay=16436,
                rx_antenna_delay=16436,
                tx_power_register=0x1F1F1F1F,
                spi_clock_mhz=8,
                otp_valid=True,
                otp_part_id=1,
                otp_lot_id=2,
                otp_ldotune=0,
                otp_xtal_trim=16,
                otp_revision=1,
                otp_vbat_cal=170,
                otp_vtemp_cal=120,
                applied_xtal_trim=16,
                ldo_kicked=True,
                settings_status=0,
                uart_baud=460800,
                active_mask=0xFF,
                calibrated_mask=0x00,
                build_config_hash=0x12345678,
                calibration_profile_id=1,
            )
            ack_ping = tp.CmdAckMessage(sequence=1, time_ms=405971, command_id=tp.CMD_PING, result=0, data=b"")
            cal_bytes = b"".join(struct.pack("<HiB", aid, 0, 0) for aid in range(1, 9))
            ack_cal = tp.CmdAckMessage(sequence=2, time_ms=405972, command_id=tp.CMD_GET_DS_CAL, result=0, data=cal_bytes)
            diag_sys = tp.DiagSystemMessage(
                sequence=3694,
                time_ms=406471,
                boot_id=41082,
                boot_count=1,
                reset_cause=0,
                uptime_ms=406471,
                cycle_count=130711,
                cycle_us=3119,
                cycle_max_us=3628,
                cycle_overruns=123,
                spi_errors=0,
                radio_recoveries=0,
                radio_recovery_failures=0,
                config_checks=356,
                config_mismatches=0,
                last_config_mismatch=0,
                fault_hold=False,
                last_fault_cause=0,
                paused=False,
                active_mask=0xFF,
                uart_tx_overflow=0,
                uart_rx_overflow=0,
                uart_high_water=128,
                meas_queue_drops=0,
                cmd_ok=3,
                cmd_rejected=0,
                cmd_crc_errors=0,
                locked=False,
                settings_status=0,
                rx_errors={},
                ds_ok=100,
                ds_fallback=0,
                ds_report_timeout=0,
                ds_final_tx_timeout=0,
                temperature_c=29.84,
                vbat_v=3.421,
            )
            rx_qual = tp.RxQualityMessage(
                sequence=41777,
                time_ms=405999,
                boot_id=41082,
                cycle_seq=41777,
                t_us=407366242,
                prf_mhz=16,
                env_flags=3,
                temperature_c=29.84,
                vbat_v=3.421,
                env_age_ms=497,
                sample_period_ms=100,
                temp_raw=131,
                vbat_raw=181,
                anchors=(
                    tp.RxQualityAnchor(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -32768, -32768, -32768, -32768),
                ),
            )
            win._on_telemetry_batch([(dev_info, now), (ack_ping, now), (ack_cal, now), (diag_sys, now), (rx_qual, now)])
            for page_idx in (0, 1, 2):
                win.pages.setCurrentIndex(page_idx)
                win._on_ui_refresh_tick()
                _APP.processEvents()
            self.assertIn("460800", win.lbl_sidebar_fw.text())
            self.assertIn("406.5s", win.lbl_diag_summary.text())
        finally:
            win.close()


if __name__ == "__main__":
    unittest.main()
