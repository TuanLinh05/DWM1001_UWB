"""Offscreen aided-import and nonblocking recording warnings with fake onboard IO."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import test_onboard_recording as fixture
from eskf_config import config_hash


class E23GuiTests(unittest.TestCase):
    setUp=fixture.GuiControlTests.setUp
    tearDown=fixture.GuiControlTests.tearDown

    def calibration_file(self):
        path=Path(self.temp.name)/'aided_calibration.json'
        path.write_text(json.dumps(dict(ok=True,harmonics_used=0,corrections={
            '1':dict(anchor_id=1,bias_m=.021,cos_m=[],sin_m=[]),
            '8':dict(anchor_id=8,bias_m=-.019,cos_m=[],sin_m=[])})))
        return path

    def test_import_button_shows_bias_changes_hash_and_clear_restores_hash(self):
        before=config_hash(self.win._eskf_payload())
        path=self.calibration_file()
        with patch('main_gui.QFileDialog.getOpenFileName',return_value=(str(path),'JSON')):
            self.win.btn_eskf_import_aided.click()
        self.assertEqual(self.win._eskf_payload()['aided_bias_m'],[[1,.021],[8,-.019]])
        self.assertIn('A1: +21.0 mm',self.win.lbl_eskf_aided_bias.text())
        self.assertNotEqual(config_hash(self.win._eskf_payload()),before)
        self.win.btn_eskf_clear_aided.click()
        self.assertNotIn('aided_bias_m',self.win._eskf_payload())
        self.assertEqual(config_hash(self.win._eskf_payload()),before)

    def test_invalid_import_leaves_current_bias_untouched(self):
        path=self.calibration_file();self.win._eskf_aided_bias_m=[[1,.01]]
        document=json.loads(path.read_text());document['corrections']['8']['bias_m']=.6
        path.write_text(json.dumps(document))
        with patch('main_gui.QFileDialog.getOpenFileName',return_value=(str(path),'JSON')),patch('main_gui.QMessageBox.warning') as warning:
            self.win.btn_eskf_import_aided.click()
        warning.assert_called_once()
        self.assertEqual(self.win._eskf_aided_bias_m,[[1,.01]])

    def test_zero_lever_unconfirmed_warnings_do_not_block_logging_and_are_manifested(self):
        with patch('main_gui.QMessageBox.warning') as modal:
            self.win.btn_csv_log.click()
            self.assertTrue(fixture.pump(lambda:self.win._remote_worker is None))
            modal.assert_not_called()
        self.assertIsNotNone(self.win.csv_logger)
        self.assertEqual({w['code'] for w in self.win._eskf_recording_warnings},{'ZERO_LEVER','UNCONFIRMED'})
        self.assertIn('WAIT_CONFIGURATION',self.win.lbl_eskf_recording_warning.text())
        for path in (self.win.last_log_dir/'onboard_recording.json',self.win.csv_logger.meta_path):
            data=json.loads(path.read_text())
            self.assertEqual(len(data['eskf_configuration_warnings']),2)
        self.win._refresh_recording_status()
        self.assertEqual(self.win.btn_csv_log.text(),'Log: WARN')

    def test_measured_confirmed_setup_has_no_start_warnings(self):
        self.win.eskf_inputs['lever_z'].setValue(.31)
        self.win.chk_eskf_confirmed.setChecked(True)
        self.assertEqual(self.win._eskf_start_warnings(self.win._eskf_payload()),[])


if __name__=='__main__':unittest.main()
