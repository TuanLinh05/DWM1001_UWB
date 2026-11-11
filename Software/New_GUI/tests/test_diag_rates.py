"""DIAG counter logging, per-second rates and cause hints."""
from __future__ import annotations

from dataclasses import fields
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
NEW_GUI_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NEW_GUI_DIR))
sys.path.insert(0, str(NEW_GUI_DIR.parent / "UWB_UART_GUI"))

import telemetry_protocol as tp  # noqa: E402
from core_state import UwbStateEngine  # noqa: E402
from diag_rates import DiagTracker, format_rates  # noqa: E402

RX_ZERO = {k: 0 for k in tp.RX_ERROR_FIELDS}


def make(cls, **values):
    """Build a protocol dataclass with zero/None defaults for unspecified fields."""
    kwargs = {}
    for f in fields(cls):
        if f.name in values:
            kwargs[f.name] = values[f.name]
        elif f.default is not f.default_factory:  # has a default -> leave it
            continue
        else:
            kwargs[f.name] = {"rx_errors": dict(RX_ZERO)}.get(f.name, 0)
    return cls(**kwargs)


def system(time_ms, cycles, boot=7, final_late=0, **rx):
    return make(tp.DiagSystemMessage, time_ms=time_ms, boot_id=boot, cycle_count=cycles,
                temperature_c=None, vbat_v=None, burst_final_late=final_late,
                rx_errors={**RX_ZERO, **rx})


def anchor(aid, time_ms, success, resp_to, skipped=0, report_to=0, ds_ok=0, txn=0):
    return make(tp.DiagAnchorMessage, anchor_id=aid, time_ms=time_ms, active=True, backed_off=False, rx_errors=0,
                calibrated=True, success=success, resp_timeouts=resp_to, poll_skipped=skipped,
                report_timeouts=report_to, ds_ok=ds_ok, txn_mismatch=txn)


class DiagRateTests(unittest.TestCase):
    def test_rates_and_never_detected_hint(self):
        tr = DiagTracker()
        tr.ingest(system(1000, 1000))
        tr.ingest(anchor(5, 1000, 1000, 0))
        # One second later: 300 cycles; A5 polled in 180 (120 skipped), RESP lost in 120.
        tr.ingest(system(2000, 1300, preamble_timeout=40, fcs=2))
        tr.ingest(anchor(5, 2000, 1060, 120, skipped=120, ds_ok=8))
        sys_r, anc_r = tr.system_rates(), tr.anchor_rates()
        self.assertAlmostEqual(sys_r["cycle_count"], 300)
        self.assertAlmostEqual(sys_r["rx_fcs"], 2)
        self.assertAlmostEqual(anc_r[5]["resp_timeouts"], 120)
        hints = " ".join(tr.interpret())
        self.assertIn("never detected", hints)
        self.assertIn("A5 40%", hints)
        text = format_rates(sys_r, anc_r)
        self.assertIn("A5", text)
        self.assertIn("67%", text)  # 120 lost of 180 polled

    def test_corrupted_frames_and_final_late_hints(self):
        tr = DiagTracker()
        tr.ingest(system(0, 0))
        tr.ingest(anchor(1, 0, 0, 0))
        tr.ingest(system(1000, 300, final_late=30, fcs=80, phy_header=20))
        tr.ingest(anchor(1, 1000, 150, 150))
        hints = " ".join(tr.interpret())
        self.assertIn("damaged", hints)
        self.assertIn("FINAL late in 10%", hints)

    def test_reboot_restarts_baseline_instead_of_negative_rates(self):
        tr = DiagTracker()
        tr.ingest(system(5000, 9000))
        tr.ingest(anchor(2, 5000, 9000, 50))
        tr.ingest(system(200, 10, boot=8))
        tr.ingest(anchor(2, 200, 10, 0))
        self.assertIsNone(tr.system_rates())
        self.assertEqual(tr.anchor_rates(), {})
        tr.ingest(system(1200, 310, boot=8))
        tr.ingest(anchor(2, 1200, 300, 10))
        self.assertAlmostEqual(tr.system_rates(session=True)["cycle_count"], 300)
        self.assertAlmostEqual(tr.anchor_rates(session=True)[2]["resp_timeouts"], 10)

    def test_engine_tracks_diag_anchor_messages(self):
        engine = UwbStateEngine()
        engine.ingest_message(system(0, 0), 0.0)
        engine.ingest_message(anchor(3, 0, 0, 0), 0.0)
        engine.ingest_message(system(1000, 250), 1.0)
        engine.ingest_message(anchor(3, 1000, 200, 50), 1.0)
        self.assertIn(3, engine.diag_anchor)
        self.assertAlmostEqual(engine.diag_tracker.anchor_rates()[3]["resp_timeouts"], 50)

    def test_logger_writes_diag_sidecar_and_summary_tool_reads_it(self):
        from serial_worker import FastCsvLogger
        with tempfile.TemporaryDirectory() as tmp:
            logger = FastCsvLogger(Path(tmp), "TEST", engine=UwbStateEngine())
            try:
                logger.log_diag(system(0, 0), 0.0)
                logger.log_diag(anchor(4, 0, 0, 0), 0.0)
                logger.log_diag(make(tp.StatsMessage, time_ms=500), 0.5)
                logger.log_diag(system(2000, 600, preamble_timeout=100), 2.0)
                logger.log_diag(anchor(4, 2000, 300, 300), 2.0)
            finally:
                logger.stop()
            lines = (logger.session_dir / "diag.jsonl").read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(lines), 5)
            self.assertEqual(json.loads(lines[2])["type"], "StatsMessage")
            meta = json.loads((logger.session_dir / "session_metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(meta["diag_records_written"], 5)
            out = subprocess.run(
                [sys.executable, str(NEW_GUI_DIR / "tools" / "diag_summary.py"), str(logger.session_dir)],
                capture_output=True, text=True, check=True,
            ).stdout
            self.assertIn("TAG counters over 2.0 s", out)
            self.assertIn("RESP lost in 50%", out)


class DiagnosticsPageTests(unittest.TestCase):
    def test_page_shows_rates_and_hints(self):
        from PyQt6.QtWidgets import QApplication
        from main_gui import UwbFastGuiWindow
        app = QApplication.instance() or QApplication([])
        win = UwbFastGuiWindow(start_demo=False)
        try:
            eng = win.engine
            for t, cyc, to in ((0, 0, 0), (1000, 300, 200)):
                eng.ingest_message(system(t, cyc, preamble_timeout=to // 4), t / 1000)
                eng.ingest_message(anchor(3, t, 300 - to if t else 0, to), t / 1000)
            win._update_diagnostics_page()
            app.processEvents()
            self.assertIn("A3", win.lbl_diag_rates.text())
            self.assertIn("67%", win.lbl_diag_rates.text())
            self.assertEqual(win.diag_kpi_cells["cycles"].text(), "300 /s")
            self.assertEqual(win.diag_rates_table.item(2, 0).text(), "A3")
            self.assertEqual(win.diag_rates_table.item(2, 2).text(), "67%")
            self.assertFalse(win.lbl_diag_hints.isHidden())
            self.assertIn("never detected", win.lbl_diag_hints.text())
        finally:
            win.close()


if __name__ == "__main__":
    unittest.main()
