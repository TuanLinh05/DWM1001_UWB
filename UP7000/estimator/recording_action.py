#!/usr/bin/python3
"""Narrow privileged action: begin/finish a GUI-owned onboard recording.

Install root-owned at /usr/local/libexec/up7000-recording-action. No arbitrary
commands, paths or unit names are accepted from the caller.
"""
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import uuid
sys.dont_write_bytecode = True
from up7000_eskf_config import normalize_config, config_hash, render_files

BASE = Path('/mnt/droneup/onboard_20261107/thoa/drone_tools/eskf')
LOGS = Path('/mnt/droneup/onboard_20261107/thoa/flight_logs/eskf')
UUID = '896b17fa-8fe0-4111-853b-57b93ec87c60'
UNIT = 'uwb-eskf-recorder.service'


def read_json(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(fd) as stream:
        return json.load(stream)


def save(path, data):
    temporary = path.parent / ('.' + path.name + '.' + uuid.uuid4().hex)
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o644)
    with os.fdopen(fd, 'w') as stream:
        json.dump(data, stream)
    os.replace(temporary, path)


def unit(action, name=UNIT):
    subprocess.run(['/usr/bin/systemctl', action, name], check=True, capture_output=True, text=True, timeout=40)


def active():
    return subprocess.run(['/usr/bin/systemctl', 'is-active', '--quiet', UNIT]).returncode == 0


def latest():
    folder = (BASE / 'latest_session').resolve()
    if not folder.is_relative_to(LOGS):
        raise RuntimeError('Invalid HDD session path')
    return folder, read_json(folder / 'session.json')


def wait_ready(recording_id, config_sha=''):
    end = time.monotonic() + 12
    while time.monotonic() < end:
        try:
            folder, state = latest()
            context = state.get('recording_context') or {}
            recorder_log = (folder / 'recorder.log').read_text(errors='replace')
            subscribed = all("Subscribed to topic '" + topic + "'" in recorder_log
                             for topic in ('/uwb_eskf/status', '/uwb_eskf/odom', '/px4/imu_frd'))
            if (context.get('recording_id', '') == recording_id and not state.get('closed')
                    and not state.get('error') and state.get('last_estimator_status')
                    and (not config_sha or state['last_estimator_status'].get('config_sha256') == config_sha)
                    and subscribed and time.time() - state.get('updated_unix_s', 0) < 3):
                return folder, state
        except (OSError, ValueError, RuntimeError):
            pass
        time.sleep(.2)
    raise RuntimeError('Onboard recorder did not confirm the requested session')


def summary(folder, state):
    estimator = state.get('last_estimator_status') or {}
    return {'session': str(folder), 'bag': str(folder / 'bag'), 'closed': bool(state.get('closed')),
            'estimator_state': estimator.get('state', 'WAIT_INPUTS'), 'config_sha256': estimator.get('config_sha256', ''),
            'imu_samples': estimator.get('imu_samples', 0), 'range_samples': estimator.get('range_samples', 0)}


def apply_config(value, digest):
    versions = BASE / 'config_versions'
    versions.mkdir(exist_ok=True)
    target = versions / digest
    if not target.exists():
        temporary = versions / ('.pending_' + uuid.uuid4().hex)
        temporary.mkdir(mode=0o755)
        for name, content in render_files(value, target).items():
            path = temporary / name
            path.write_text(content); path.chmod(0o644)
        temporary.rename(target)
    if config_hash(read_json(target / 'gui_config.json')) != digest:
        raise RuntimeError('Stored ESKF configuration checksum mismatch')
    save(BASE / 'active_config.json', {'directory': str(target), 'config_sha256': digest})


def perform(request):
    operation = request.get('action')
    if operation not in ('start', 'finish'):
        raise ValueError('Unsupported recording action')
    for key in ('request_id', 'recording_id'):
        if not isinstance(request.get(key), str) or not re.fullmatch('[a-f0-9]{32}', request[key]):
            raise ValueError('Invalid ' + key)
    configuration = normalize_config(request['config']) if 'config' in request else None
    digest = config_hash(configuration) if configuration is not None else ''
    if configuration is not None and request.get('config_sha256') != digest:
        raise ValueError('GUI ESKF configuration checksum mismatch')
    actual = subprocess.check_output(['/usr/bin/findmnt', '-n', '-o', 'UUID', '--target', '/mnt/droneup'], text=True).strip()
    if actual != UUID:
        raise RuntimeError('Required HDD is not mounted')
    control = BASE / 'control'
    control.mkdir(exist_ok=True)
    if control.resolve() != control:
        raise RuntimeError('Control directory must be a physical HDD directory')
    with (control / 'action.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        result_path = control / (request['request_id'] + '.json')
        if result_path.exists():
            result = read_json(result_path)
            if (result['action'] != operation or result['recording_id'] != request['recording_id']
                    or operation == 'start' and digest != result.get('requested_config_sha256', '')):
                raise ValueError('Request ID reused for a different recording')
            return result
        context_path = BASE / 'recording_context.json'
        try:
            context = read_json(context_path)
        except FileNotFoundError:
            context = {}
        result = {'ok': True, 'action': operation, 'request_id': request['request_id'],
                  'recording_id': request['recording_id']}
        if operation == 'start':
            if context.get('recording_id') == request['recording_id'] and context.get('config_sha256', '') != digest:
                raise ValueError('Recording ID reused with a different ESKF configuration')
            if configuration is not None:
                apply_config(configuration, digest)
            result['requested_config_sha256'] = digest
            unit('start', 'uwb-px4-logger.service')
            if context.get('recording_id') != request['recording_id'] or not active():
                context = {'mode': 'gui', 'recording_id': request['recording_id'],
                           'gui_session': str(request.get('gui_session', ''))[:128],
                           'config_sha256': digest,
                           'requested_unix_s': time.time()}
                save(context_path, context)
                unit('restart')
            folder, state = wait_ready(request['recording_id'], digest)
            result.update(summary(folder, state), state='recording', background_recording=False)
        else:
            if context.get('recording_id') != request['recording_id']:
                raise RuntimeError('This GUI does not own the current onboard recording')
            folder, _ = latest()
            unit('stop')  # waits for both ESKF and native MCAP finalization
            state = read_json(folder / 'session.json')
            if not state.get('closed') or state.get('error'):
                raise RuntimeError('MCAP did not finalize cleanly: ' + str(state.get('error')))
            result.update(summary(folder, state))
            save(context_path, {'mode': 'autonomous', 'recording_id': '', 'after_gui_recording': request['recording_id']})
            unit('start')
            background, _ = wait_ready('')
            result.update(state='saved', background_recording=True, background_session=str(background))
        save(result_path, result)
        return result


if __name__ == '__main__':
    try:
        raw = sys.stdin.buffer.read(16385)
        if len(raw) > 16384:
            raise ValueError('Request too large')
        print(json.dumps(perform(json.loads(raw))))
    except Exception as exc:
        print(json.dumps({'ok': False, 'error': str(exc)[:240]}))
        sys.exit(1)
