"""TDD test suite for New_GUI design tokens, background workers, and PyQt6 main window."""

from __future__ import annotations

import os
from pathlib import Path
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
        with tempfile.TemporaryDirectory(dir="D:/Antigravity/2026-11-05/tmp") as tmpdir:
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


if __name__ == "__main__":
    unittest.main()
