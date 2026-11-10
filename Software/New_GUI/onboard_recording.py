"""Asynchronous acknowledged control of UP7000's HDD recording sessions."""
import json
import socket
import time
import uuid
import base64
import hashlib
from eskf_config import normalize_config, config_hash
from PyQt6.QtCore import QThread, pyqtSignal

KIND = 'up7000_recording_control'
DEFAULT_HOST = '100.123.216.14'
CONTROL_PORT = 15558


def request_recording(host, action, recording_id='', gui_session='', *, port=CONTROL_PORT,
                      timeout_s=35, request_id=None, config=None):
    request_id = request_id or uuid.uuid4().hex
    request = {'kind': KIND, 'version': 1, 'request_id': request_id, 'action': action,
               'recording_id': recording_id, 'gui_session': gui_session[:128]}
    if config is not None and action == 'start':
        request['config'] = normalize_config(config)
        request['config_sha256'] = config_hash(config)
    raw = json.dumps(request, separators=(',', ':'), sort_keys=True, allow_nan=False).encode()
    if len(raw) > 16384:
        raise ValueError('Recording configuration exceeds 16 KiB')
    packets = [raw]
    if len(raw) > 1200:
        chunks = [raw[i:i+600] for i in range(0, len(raw), 600)]
        packets = [json.dumps({'kind': KIND, 'version': 1, 'action': 'fragment', 'request_id': request_id,
                               'index': i, 'count': len(chunks), 'sha256': hashlib.sha256(raw).hexdigest(),
                               'data': base64.b64encode(chunk).decode()}, separators=(',', ':')).encode()
                   for i, chunk in enumerate(chunks)]
    address = (socket.gethostbyname(host), port)
    deadline, last_send = time.monotonic() + timeout_s, 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(.2)
        while time.monotonic() < deadline:
            if time.monotonic() - last_send > 1:
                for packet in packets:
                    sock.sendto(packet, address)
                last_send = time.monotonic()
            try:
                raw, source = sock.recvfrom(4096)
            except (socket.timeout, ConnectionResetError):
                continue
            if source != address or len(raw) > 1200:
                continue
            try:
                result = json.loads(raw)
                if (result.get('kind') == KIND and result.get('request_id') == request_id
                        and result.get('version') == 1 and result.get('action') == action):
                    return result
            except (ValueError, AttributeError):
                continue
    return {'ok': False, 'action': action, 'request_id': request_id, 'recording_id': recording_id,
            'error': f'UP7000 did not confirm {action}; check Tailscale, HDD and recording-control service'}


class OnboardRecordingWorker(QThread):
    result_ready = pyqtSignal(dict)

    def __init__(self, host, action, recording_id='', gui_session='', parent=None, config=None):
        super().__init__(parent)
        self.host, self.action = host, action
        self.recording_id, self.gui_session = recording_id, gui_session
        self.config = config

    def run(self):
        try:
            result = request_recording(self.host, self.action, self.recording_id, self.gui_session,
                                       timeout_s=3 if self.action == 'status' else 35, config=self.config)
        except Exception as exc:
            result = {'ok': False, 'action': self.action, 'recording_id': self.recording_id, 'error': str(exc)}
        self.result_ready.emit(result)
