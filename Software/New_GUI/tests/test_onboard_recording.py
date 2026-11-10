"""Acknowledged GUI/HDD recording lifecycle and UDP retry behavior."""
import importlib.util
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
GUI = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(GUI), str(GUI.parent / 'UWB_UART_GUI')]
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtWidgets import QApplication
from onboard_recording import request_recording, KIND
from eskf_config import config_hash
from test_eskf_config import configuration
from main_gui import UwbFastGuiWindow
from up4000_panel import Up4000StatusListener

spec = importlib.util.spec_from_file_location('recording_server', GUI.parents[1] / 'UP7000/estimator/recording_control_server.py')
server_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server_module)
APP = QApplication.instance() or QApplication([])


def pump(predicate, timeout=3):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        APP.processEvents()
        if predicate():
            return True
        time.sleep(.005)
    return False


class UdpControlTests(unittest.TestCase):
    def test_large_configuration_fragments_survive_a_missing_piece(self):
        captured=[]
        server=server_module.ControlServer('127.0.0.1',0,['127.0.0.1'],
            lambda req: captured.append(req) or {'ok':True,'config_sha256':req['config_sha256']})
        assemble=server.assemble; dropped=[]; lengths=[]
        def lose_one(request,address):
            lengths.append(len(json.dumps(request,separators=(',',':')).encode()))
            if request.get('action')=='fragment' and request['index']==1 and not dropped:
                dropped.append(True);return None
            return assemble(request,address)
        server.assemble=lose_one
        thread=threading.Thread(target=server.serve);thread.start()
        try:
            value=configuration()
            for a in value['anchors']:
                a[1]+=0.12345678;a[2]+=0.98765432;a[3]+=0.01234567
                a[5]=1234.12345678;a[6]=987.12345678
            response=request_recording('127.0.0.1','start','f'*32,'long_session_'+'x'*110,
                config=value,port=server.address[1],timeout_s=4)
            self.assertTrue(response['ok'])
            self.assertTrue(dropped)
            self.assertTrue(all(n<=1200 for n in lengths))
            self.assertEqual(len(captured),1)
            self.assertEqual(captured[0]['config_sha256'],config_hash(value))
        finally:server.stopping.set();thread.join(3)
    def test_retry_keeps_one_action_and_rejects_request_id_reuse(self):
        calls = []
        def action(request):
            calls.append(request)
            return {'ok': True, 'recording_id': request['recording_id'], 'state': 'recording'}
        server = server_module.ControlServer('127.0.0.1', 0, ['127.0.0.1'], action)
        original = server.send
        dropped = []
        def lose_first(*args):
            if not dropped:
                dropped.append(True)
                return
            original(*args)
        server.send = lose_first
        thread = threading.Thread(target=server.serve)
        thread.start()
        try:
            response = request_recording('127.0.0.1', 'start', 'b'*32, port=server.address[1], timeout_s=3, request_id='a'*32)
            self.assertTrue(response['ok'])
            self.assertEqual(len(calls), 1)
            collision = request_recording('127.0.0.1', 'finish', 'b'*32, port=server.address[1], timeout_s=2, request_id='a'*32)
            self.assertFalse(collision['ok'])
            self.assertEqual(len(calls), 1)
        finally:
            server.stopping.set(); thread.join(3)

    def test_unapproved_source_cannot_control_recording(self):
        calls = []
        server = server_module.ControlServer('127.0.0.1', 0, [], lambda req: calls.append(req))
        thread = threading.Thread(target=server.serve); thread.start()
        try:
            result = request_recording('127.0.0.1', 'start', 'c'*32, port=server.address[1], timeout_s=.3)
            self.assertFalse(result['ok'])
            self.assertFalse(calls)
        finally:
            server.stopping.set(); thread.join(3)

    def test_status_and_malformed_packets_do_not_run_actions(self):
        calls = []
        server = server_module.ControlServer('127.0.0.1', 0, ['127.0.0.1'], lambda r: calls.append(r),
                                             lambda: {'ok': True, 'active': True, 'recording_id': 'e'*32})
        thread = threading.Thread(target=server.serve); thread.start()
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.sendto(b'not json', server.address)
                sock.sendto(b'[]', server.address)
                sock.sendto(b'x'*1300, server.address)
            result = request_recording('127.0.0.1', 'status', port=server.address[1], timeout_s=2)
            self.assertTrue(result['active'])
            self.assertFalse(calls)
        finally:
            server.stopping.set(); thread.join(3)


class FakeWorker(QThread):
    result_ready = pyqtSignal(dict)
    calls = []
    fail = False
    bad_hash = False
    def __init__(self, host, action, recording_id='', gui_session='', parent=None, config=None):
        super().__init__(parent)
        self.action, self.recording_id = action, recording_id
        self.config = config
    def run(self):
        self.msleep(50)
        self.calls.append(self.action)
        result = {'ok': not self.fail, 'action': self.action, 'recording_id': self.recording_id,
                  'state': 'saved' if self.action == 'finish' else 'recording', 'active': True,
                  'closed': self.action == 'finish', 'background_recording': self.action == 'finish',
                  'session': '/HDD/test_session', 'bag': '/HDD/test_session/bag', 'estimator_state': 'WAIT_CONFIGURATION'}
        if self.config is not None:result['config_sha256'] = config_hash(self.config)
        if self.bad_hash:result['config_sha256']='0'*64
        if self.fail:
            result['error'] = 'UP7000 unavailable'
        self.result_ready.emit(result)


class StubTelemetry:
    relay_addr = ('127.0.0.1', 15556)
    commands = []
    def send_command(self, *args):
        self.commands.append(args)
    def stop(self):
        pass


class GuiControlTests(unittest.TestCase):
    def setUp(self):
        FakeWorker.calls = []; FakeWorker.fail = False; FakeWorker.bad_hash = False
        self.temp = tempfile.TemporaryDirectory()
        self.patches = [patch('main_gui.NEW_GUI_DIR', Path(self.temp.name)),
                        patch('main_gui.OnboardRecordingWorker', FakeWorker),
                        patch('main_gui.Up4000StatusListener', side_effect=lambda parent: Up4000StatusListener(port=0, parent=parent))]
        for p in self.patches:p.start()
        self.win = UwbFastGuiWindow(initial_port='udp:15555')
        self.win.serial_worker = StubTelemetry()
    def tearDown(self):
        if self.win.csv_logger is not None:
            self.win.btn_csv_log.click()
        pump(lambda: self.win._remote_worker is None)
        self.win.close()
        for p in reversed(self.patches):p.stop()
        self.temp.cleanup()

    def test_one_button_starts_both_and_saves_hdd_then_continues_background(self):
        self.win.btn_csv_log.click()
        self.assertFalse(self.win.btn_csv_log.isEnabled())
        self.assertTrue(pump(lambda: self.win._remote_worker is None))
        self.assertEqual(self.win._remote_state, 'recording')
        self.assertIsNotNone(self.win.csv_logger)
        self.assertIsNotNone(self.win.rosbag_logger)
        self.assertIn('UP7000 HDD', self.win.statusBar().currentMessage())
        self.win.btn_csv_log.click()
        self.assertTrue(pump(lambda: self.win._remote_worker is None))
        manifest = json.loads((self.win.last_log_dir/'onboard_recording.json').read_text())
        self.assertEqual(manifest['state'], 'saved')
        self.assertTrue(manifest['onboard']['closed'])
        self.assertTrue(manifest['onboard']['background_recording'])
        self.assertEqual(FakeWorker.calls, ['start', 'finish'])
        self.assertFalse(self.win._remote_recording_id)

    def test_config_hash_mismatch_is_partial_and_editing_invalidates_confirmation(self):
        self.win.chk_eskf_confirmed.setChecked(True)
        self.assertTrue(self.win.chk_eskf_confirmed.isChecked())
        self.win.engine.anchors[1].x+=.1
        self.win._eskf_settings_changed()
        self.assertFalse(self.win.chk_eskf_confirmed.isChecked())
        FakeWorker.bad_hash=True
        self.win.btn_csv_log.click()
        self.assertFalse(self.win.eskf_config_panel.isEnabled())
        self.assertTrue(pump(lambda:self.win._remote_worker is None))
        self.win._refresh_recording_status()
        self.assertEqual(self.win.btn_csv_log.text(),'Log: PARTIAL')
        manifest=json.loads((self.win.last_log_dir/'onboard_recording.json').read_text())
        self.assertEqual(manifest['eskf_config']['anchors'][0][1],self.win.engine.anchors[1].x)

    def test_failure_stays_visible_and_does_not_claim_complete_recording(self):
        FakeWorker.fail = True
        self.win.btn_csv_log.click()
        self.assertTrue(pump(lambda: self.win._remote_worker is None))
        self.win._refresh_recording_status()
        self.assertEqual(self.win.btn_csv_log.text(), 'Log: PARTIAL')
        self.assertIsNotNone(self.win.csv_logger)
        self.assertIn('unavailable', self.win._remote_error)
        self.win.btn_csv_log.click()
        self.assertTrue(pump(lambda: self.win._remote_worker is None))
        self.assertEqual(self.win.btn_csv_log.text(), 'Log: PARTIAL')

    def test_closing_during_start_finishes_remote_before_window_closes(self):
        self.win.show()
        self.win.btn_csv_log.click()
        self.win.close()
        self.assertTrue(pump(lambda: self.win._remote_worker is None and not self.win.isVisible()))
        self.assertEqual(FakeWorker.calls, ['start', 'finish'])

    def test_demo_never_controls_real_onboard_service(self):
        self.win.serial_worker = None
        self.win.start_demo_simulator()
        self.win.btn_csv_log.click()
        pump(lambda: False, .15)
        self.assertFalse(FakeWorker.calls)
        self.win.btn_csv_log.click()


if __name__ == '__main__':unittest.main()
