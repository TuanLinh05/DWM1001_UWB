#!/usr/bin/env python3
"""Small UDP control channel, separate from the high-rate telemetry reader."""
import argparse
import json
from pathlib import Path
import queue
import re
import signal
import socket
import subprocess
import threading
import time
import base64
import hashlib

KIND = 'up7000_recording_control'
PORT = 15558
BASE = Path('/mnt/droneup/onboard_20261107/thoa/drone_tools/eskf')


def inspect_status():
    folder = (BASE / 'latest_session').resolve()
    state = json.loads((folder / 'session.json').read_text())
    estimator = state.get('last_estimator_status') or {}
    return {'ok': True, 'state': 'recording' if not state.get('closed') else 'saved',
            'recording_id': (state.get('recording_context') or {}).get('recording_id', ''),
            'session': str(folder), 'bag': str(folder / 'bag'), 'closed': bool(state.get('closed')),
            'active': not state.get('closed') and time.time() - state.get('updated_unix_s', 0) < 5,
            'error': str(state.get('error', ''))[:160], 'estimator_state': estimator.get('state', 'WAIT_INPUTS'),
            'config_sha256': estimator.get('config_sha256', ''),
            'imu_samples': estimator.get('imu_samples', 0), 'range_samples': estimator.get('range_samples', 0)}


def privileged_action(request):
    result = subprocess.run(['/usr/bin/sudo', '-n', '/usr/local/libexec/up7000-recording-action'],
                            input=json.dumps(request), capture_output=True, text=True, timeout=100)
    try:
        return json.loads(result.stdout)
    except ValueError:
        raise RuntimeError((result.stderr or 'Recording helper returned no JSON')[:200])


class ControlServer:
    def __init__(self, bind='0.0.0.0', port=PORT, allowed=(), action_runner=privileged_action, status_reader=inspect_status):
        self.allowed = set(allowed)
        self.action_runner, self.status_reader = action_runner, status_reader
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind, port)); self.sock.settimeout(.2)
        self.address = self.sock.getsockname()
        self.jobs = queue.Queue(maxsize=16)
        self.stopping = threading.Event()
        self.fragments = {}

    def assemble(self, request, address):
        if request.get('action') != 'fragment':
            return request
        now = time.monotonic()
        self.fragments = {key: group for key, group in self.fragments.items() if now - group['time'] < 10}
        index, count, digest = request.get('index'), request.get('count'), request.get('sha256')
        if (type(index) is not int or type(count) is not int or not 0 <= index < count <= 28
                or not re.fullmatch('[a-f0-9]{64}', str(digest))):
            raise ValueError('Invalid configuration fragment')
        chunk = base64.b64decode(request.get('data', ''), validate=True)
        if len(chunk) > 600:
            raise ValueError('Configuration fragment too large')
        key = (address[0], request['request_id'], digest)
        if key not in self.fragments:
            if len(self.fragments) >= 16:
                self.fragments.pop(next(iter(self.fragments)))
            self.fragments[key] = {'time': now, 'count': count, 'parts': {}}
        group = self.fragments[key]
        if group['count'] != count:
            raise ValueError('Configuration fragment count changed')
        group['parts'][index] = chunk
        if len(group['parts']) < count:
            return None
        raw = b''.join(group['parts'][i] for i in range(count))
        del self.fragments[key]
        if len(raw) > 16384 or hashlib.sha256(raw).hexdigest() != digest:
            raise ValueError('Configuration fragment checksum mismatch')
        value = json.loads(raw)
        if (not isinstance(value, dict) or value.get('request_id') != request['request_id']
                or value.get('kind') != KIND or value.get('version') != 1 or value.get('action') != 'start'):
            raise ValueError('Invalid reconstructed recording request')
        return value

    def send(self, result, request, address):
        reply = {**result, 'kind': KIND, 'version': 1, 'request_id': request.get('request_id'), 'action': request.get('action')}
        data = json.dumps(reply, separators=(',', ':')).encode()
        if len(data) > 1200:
            data = json.dumps({'kind': KIND, 'version': 1, 'request_id': request.get('request_id'),
                               'action': request.get('action'), 'ok': False, 'error': 'Control response too large'}).encode()
        self.sock.sendto(data, address)

    def actions(self):
        cache = {}
        while not self.stopping.is_set() or not self.jobs.empty():
            try:
                request, address = self.jobs.get(timeout=.2)
            except queue.Empty:
                continue
            key = request['request_id']
            signature = (request.get('action'), request.get('recording_id'), request.get('config_sha256'))
            if key not in cache:
                try:
                    result = self.action_runner(request)
                except Exception as exc:
                    result = {'ok': False, 'error': str(exc)[:240]}
                cache[key] = (signature, result)
                if len(cache) > 128:
                    cache.pop(next(iter(cache)))
            try:
                result = cache[key][1] if cache[key][0] == signature else {'ok': False, 'error': 'Request ID reused'}
                self.send(result, request, address)
            except OSError:
                pass

    def serve(self):
        worker = threading.Thread(target=self.actions, name='recording-actions', daemon=True)
        worker.start()
        try:
            while not self.stopping.is_set():
                try:
                    data, address = self.sock.recvfrom(4096)
                except (socket.timeout, ConnectionResetError, InterruptedError):
                    continue
                if address[0] not in self.allowed or len(data) > 1200:
                    continue
                request = {}
                try:
                    request = json.loads(data)
                    if request.get('kind') != KIND or request.get('version') != 1:
                        continue
                    if not re.fullmatch('[a-f0-9]{32}', str(request.get('request_id', ''))):
                        continue
                    request = self.assemble(request, address)
                    if request is None:
                        continue
                    if request.get('action') == 'status':
                        self.send(self.status_reader(), request, address)
                    elif request.get('action') in ('start', 'finish'):
                        if not re.fullmatch('[a-f0-9]{32}', str(request.get('recording_id', ''))):
                            raise ValueError('Invalid recording ID')
                        self.jobs.put_nowait((request, address))
                    else:
                        raise ValueError('Unsupported action')
                except (ValueError, TypeError, AttributeError, OSError, queue.Full) as exc:
                    self.send({'ok': False, 'error': str(exc)[:180]}, request if isinstance(request, dict) else {}, address)
        finally:
            self.stopping.set(); worker.join(105); self.sock.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bind', default='0.0.0.0'); ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--allow-source', action='append', required=True)
    args = ap.parse_args()
    server = ControlServer(args.bind, args.port, args.allow_source)
    signal.signal(signal.SIGINT, lambda *_: server.stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: server.stopping.set())
    server.serve()
