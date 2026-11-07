"""G0.1 / G0.3 — calibration page and labelled waypoints in UwbFastGuiWindow.

Hand-off contract for main_gui.py. Never calls "Apply to Host" here: with
start_demo=False that would overwrite the user's last_rig_config.json.
"""

from __future__ import annotations

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from g0_fixtures import ROOM_54, SPOTS_4, feed_spot_capture  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

from main_gui import UwbFastGuiWindow  # noqa: E402
import multi_spot_calibration as msc  # noqa: E402

_APP = QApplication.instance() or QApplication([])


class CalibrationPageG0Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.win = UwbFastGuiWindow(start_demo=False)
        self.engine = self.win.engine
        self.engine.set_anchors(ROOM_54)

    def tearDown(self) -> None:
        self.win.close()

    def _capture(self, count: int) -> None:
        cycle = 1
        for j, true in enumerate(SPOTS_4[:count], start=1):
            cycle = feed_spot_capture(self.engine, true, true, first_cycle=cycle)

    def test_method_selector_drives_engine(self) -> None:
        combo = self.win.combo_cal_method
        self.assertEqual([combo.itemData(i) for i in range(combo.count())], ["joint", "median"])
        self.assertEqual(combo.currentData(), "joint")
        combo.setCurrentIndex(1)
        self.assertEqual(self.engine.calibration_method, "median")
        combo.setCurrentIndex(0)
        self.assertEqual(self.engine.calibration_method, "joint")

    def test_spot_table_and_joint_status(self) -> None:
        self._capture(1)
        self.win._update_calibration_page()
        self.assertEqual(self.win.cal_spot_table.rowCount(), 1)
        self.assertIn("2", self.win.lbl_joint_status.text())   # asks for at least 2 spots
        self._capture_more()
        self.win._update_calibration_page()
        self.assertEqual(self.win.cal_spot_table.rowCount(), 3)
        self.assertIn("rank", self.win.lbl_joint_status.text().lower())

    def _capture_more(self) -> None:
        cycle = 1000
        for true in SPOTS_4[1:3]:
            cycle = feed_spot_capture(self.engine, true, true, first_cycle=cycle)

    def test_remove_selected_spot(self) -> None:
        self._capture(2)
        self.win._update_calibration_page()
        self.win.cal_spot_table.selectRow(0)
        self.win.btn_remove_spot.click()
        self.assertEqual([c.spot_index for c in self.engine.spot_captures], [2])
        self.win._update_calibration_page()
        self.assertEqual(self.win.cal_spot_table.rowCount(), 1)

    def test_suggest_spots_fills_the_spot_inputs(self) -> None:
        self.win.btn_suggest_spots.click()
        enabled = [a for a in self.engine.anchors.values() if a.enabled]
        first = msc.suggest_calibration_spots(enabled, count=4)[0]
        self.assertAlmostEqual(self.win.spin_cal_x.value(), first[0], places=3)
        self.assertAlmostEqual(self.win.spin_cal_y.value(), first[1], places=3)
        self.assertAlmostEqual(self.win.spin_cal_z.value(), first[2], places=3)
        self.assertTrue(self.win.lbl_spot_plan.text())

    def test_waypoint_label_is_used(self) -> None:
        from test_robust_filter import burst

        for i in range(1, 40):
            self.engine.ingest_message(burst(self.engine, i, (2.7, 1.8, 1.2)), i * 0.004)
        self.assertIsNotNone(self.engine.latest_solution)
        self.win.edit_waypoint_label.setText("P1_start")
        self.win._mark_live_waypoint()
        self.assertEqual(self.engine.waypoints[-1][1], "P1_start")
        self.win.edit_waypoint_label.setText("")
        self.win._mark_live_waypoint()
        self.assertTrue(self.engine.waypoints[-1][1].startswith("W"))


if __name__ == "__main__":
    unittest.main()
