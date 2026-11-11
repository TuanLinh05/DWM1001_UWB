"""Enroll a qualified card, with backups and rollback to the previous backend."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid

from common import (BASE, CONFIG, DATA, HDD_UUID, KIT, MOUNT, PREFIX, RECORDS, RUN, UNITS,
                    atomic_text, check_kit, command, current_uuid, fingerprint, mount_info,
                    mount_temporary, progress, require, root_only, save_json,
                    services, verify_mount, volume_for_uuid)
from seed import capture, populate

PATHS = ['/etc/fstab', '/etc/up7000-storage.json',
         '/usr/local/libexec/up7000-onboard-storage-check',
         '/usr/local/libexec/up7000-recording-action',
         '/home/thoa/drone_tools/eskf/run_onboard.py',
         '/etc/systemd/system/mnt-droneup.mount.d/usb2.conf']


def patch_uuid_source(text):
    replacement = ("CARD_STORAGE_CONFIG = Path('/etc/up7000-storage.json')\n"
                   "UUID = (json.loads(CARD_STORAGE_CONFIG.read_text())['uuid']\n"
                   "        if CARD_STORAGE_CONFIG.exists() else '" + HDD_UUID + "')")
    old = "UUID = '" + HDD_UUID + "'"
    if replacement in text:
        return text
    require(text.count(old) == 1, 'Unrecognized source UUID declaration; do not overwrite changed code')
    return text.replace(old, replacement)


def new_fstab(text, value):
    lines = [];found = 0
    for line in text.splitlines():
        fields = line.split()
        if fields and not fields[0].startswith('#') and len(fields) > 1 and fields[1] == str(MOUNT):
            lines.append('# Saved by UP7000 card kit: ' + line);found += 1
        else:
            lines.append(line)
    require(found == 1, 'Expected exactly one previous data mount entry')
    lines.append(f'UUID={value} {MOUNT} ext4 defaults,nofail,noatime,nosuid,nodev,x-systemd.device-timeout=20s 0 2')
    return '\n'.join(lines) + '\n'


GUARD = '''#!/usr/bin/python3 -B
import json,os,re,subprocess,sys
from pathlib import Path
p=Path('/etc/up7000-storage.json')
expected='896b17fa-8fe0-4111-853b-57b93ec87c60'
if p.exists():
    s=p.lstat()
    if p.is_symlink() or s.st_uid!=0 or s.st_mode & 0o022:
        sys.exit('Untrusted storage configuration')
    cfg=json.loads(p.read_text())
    if cfg.get('schema')!=1 or cfg.get('kind')!='sdcard_ext4' or cfg.get('mount')!='/mnt/droneup':
        sys.exit('Unsupported storage configuration')
    expected=cfg['uuid']
    if not re.fullmatch(r'[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}',expected):
        sys.exit('Invalid storage UUID')
q=subprocess.run(['findmnt','-n','--mountpoint','/mnt/droneup','-o','UUID,FSTYPE'],capture_output=True,text=True)
if q.returncode or q.stdout.split()!=[expected,'ext4'] or Path('/mnt/droneup').stat().st_dev==Path('/').stat().st_dev:
    sys.exit('Correct external data card is missing; refusing eMMC fallback')
'''


def wait_ready(value, timeout=55):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        verify_mount(MOUNT, value)
        state = None
        try:
            folder = (BASE / 'latest_session').resolve()
            state = json.loads((folder / 'session.json').read_text())
            if (all(v == 'active' for v in services().values()) and state.get('recorder_topics_verified')
                    and state.get('last_estimator_status') and not state.get('error') and not state.get('closed')
                    and state.get('hdd_uuid') == value and time.time() - state.get('updated_unix_s', 0) < 4):
                require(folder.stat().st_dev == MOUNT.stat().st_dev, 'Session escaped the data card')
                return {'session': str(folder), 'estimator_status': state['last_estimator_status']}
        except (OSError, ValueError):
            pass
        time.sleep(.5)
    raise RuntimeError(f'Recorder did not become ready: {services()}, {state}')


def backup_files(destination):
    destination.mkdir(mode=0o700, exist_ok=False)
    saved = {}
    for i, name in enumerate(PATHS):
        p = Path(name);require(not p.is_symlink(), 'Unexpected OS/source symlink: ' + name)
        saved[name] = p.exists()
        if p.exists():shutil.copy2(p, destination / str(i))
    save_json(destination / 'paths.json', saved, 0o600)
    enabled = subprocess.run(['systemctl', 'is-enabled', 'up7000-hdd-usb2.service'], capture_output=True, text=True).stdout.strip()
    service_flags = {u: subprocess.run(['systemctl', 'is-enabled', u], capture_output=True,
                                      text=True).stdout.strip() for u in UNITS}
    save_json(destination / 'old_policy.json', {'enabled': enabled, 'services': service_flags}, 0o600)


def restore_files(destination):
    saved = json.loads((destination / 'paths.json').read_text())
    for i, name in enumerate(PATHS):
        p = Path(name)
        if saved[name]:
            p.parent.mkdir(parents=True, exist_ok=True);atomic_text(p, (destination / str(i)).read_text())
        elif p.exists():
            require(name == str(CONFIG), 'Refusing removal outside new config')
            p.unlink()
    policy = json.loads((destination / 'old_policy.json').read_text())
    if policy['enabled'] == 'enabled':command(['systemctl', 'enable', 'up7000-hdd-usb2.service'])
    for unit in UNITS:
        flag = policy.get('services', {}).get(unit)
        if flag in ('enabled', 'disabled'):
            command(['systemctl', 'enable' if flag == 'enabled' else 'disable', unit])
    command(['systemctl', 'daemon-reload'])


def validate_existing_data_tree(volume):
    """Allow retry after rollback without replacing configs or prior test logs."""
    tree = volume / PREFIX
    data = tree / 'thoa'
    required_dirs = [tree, data, data / 'flight_logs/eskf', data / '.ros/log',
                     data / 'cache/python', data / 'service_logs', data / 'tmp',
                     data / 'drone_tools/eskf/config_versions',
                     data / 'drone_tools/eskf/control']
    for path in required_dirs:
        require(path.is_dir() and not path.is_symlink(),
                'Existing onboard tree is incomplete or unsafe: ' + str(path))
    base = data / 'drone_tools/eskf'
    pointer_path = base / 'active_config.json'
    context_path = base / 'recording_context.json'
    require(pointer_path.is_file() and not pointer_path.is_symlink(),
            'Existing onboard config pointer missing or unsafe')
    require(context_path.is_file() and not context_path.is_symlink(),
            'Existing recording context missing or unsafe')
    pointer = json.loads(pointer_path.read_text())
    raw_config_dir = Path(pointer.get('directory', ''))
    if raw_config_dir.is_absolute() and raw_config_dir.is_relative_to(MOUNT):
        config_dir = volume / raw_config_dir.relative_to(MOUNT)
    else:
        config_dir = raw_config_dir
    config_dir = config_dir.resolve()
    require(config_dir.is_relative_to((base / 'config_versions').resolve()),
            'Existing onboard config pointer escapes the card config tree')
    for name in ['gui_config.json', 'eskf.yaml', 'rig_onboard.txt', 'physical_setup.json']:
        path = config_dir / name
        require(path.is_file() and not path.is_symlink(),
                'Existing onboard config file missing or unsafe: ' + str(path))
    context = json.loads(context_path.read_text())
    require(context.get('mode') == 'autonomous' and context.get('recording_id') == '',
            'Existing data tree has an active GUI recording')
    latest = base / 'latest_session'
    if latest.is_symlink():
        raw_target = Path(os.readlink(latest))
        if raw_target.is_absolute() and raw_target.is_relative_to(MOUNT):
            target = volume / raw_target.relative_to(MOUNT)
        else:
            target = raw_target if raw_target.is_absolute() else latest.parent / raw_target
        target = target.resolve(strict=True)
        require(target.is_relative_to((data / 'flight_logs/eskf').resolve()),
                'Existing latest-session link escapes the card log tree')
        session = target / 'session.json'
        if session.is_file():
            state = json.loads(session.read_text())
            require(not (state.get('recording_context') or {}).get('recording_id'),
                    'Existing latest session is GUI-owned')
    return True


def activate(value, waive_full_card_test=False):
    root_only();manifest = check_kit()
    row = volume_for_uuid(value)
    if current_uuid() == value and mount_info() and mount_info()['uuid'] == value:
        return {'phase': 'already_active', **wait_ready(value)}
    report_path = RECORDS / value / 'qualification.json'
    report = None
    if report_path.exists():
        require(not report_path.is_symlink(), 'Untrusted qualification report')
        st = report_path.stat()
        require(st.st_uid == 0 and not st.st_mode & 0o022, 'Untrusted qualification report')
        report = json.loads(report_path.read_text())
    full_qualification = bool(report and report.get('phase') == 'qualified'
                              and report.get('sha256_match') is True)
    if full_qualification:
        require(report.get('identity') == fingerprint(row), 'Card identity changed since qualification')
        require(0 <= time.time() - report.get('completed_unix_s', 0) <= 7 * 86400,
                'Qualification is older than seven days')
        media_validation = {'phase': 'full_card_test_passed', 'full_capacity_verified': True,
                            'completed_unix_s': report['completed_unix_s']}
    else:
        require(waive_full_card_test,
                'Full card qualification has not passed; explicitly waive with --waive-full-card-test')
        media_validation = {
            'phase': 'full_card_test_waived_by_user',
            'full_capacity_verified': False,
            'waived_unix_s': time.time(),
            'identity': fingerprint(row),
            'reason': 'User requested GUI and ROS logger setup without full-capacity media testing',
        }
        save_json(RECORDS / value / 'qualification_waiver.json', media_validation, 0o600)
    rootdev = Path('/').stat().st_dev
    for p in ['/opt/ros/humble', '/home/thoa/UP4000', '/home/thoa/ros2_px4_v2', '/home/thoa/drone_tools']:
        require(Path(p).stat().st_dev == rootdev, 'Runtime is not on eMMC: ' + p)
    # Refuse an active GUI session; the caller is performing maintenance.
    if (BASE / 'latest_session').exists():
        current = json.loads(((BASE / 'latest_session').resolve() / 'session.json').read_text())
        require(not (current.get('recording_context') or {}).get('recording_id'), 'Finish GUI logging before switching media')
    volume = mount_temporary(row)
    work = RUN / ('activation_' + uuid.uuid4().hex);work.mkdir()
    backup = Path('/etc/up7000-card-backups') / time.strftime('%Y%m%d_%H%M%S')
    stopped = False;changed = False;seed = KIT / 'seed.tar.gz'
    old = mount_info();old_uuid = old['uuid'] if old else None
    try:
        reuse_data_tree = (volume / PREFIX).exists()
        if reuse_data_tree:
            require(waive_full_card_test,
                    'Existing onboard data tree on card; reuse only with explicit operator waiver')
            validate_existing_data_tree(volume)
        # Closing the current bag also freezes configuration during capture.
        stopped = True;command(['systemctl', 'stop', *UNITS], timeout=120)
        if not reuse_data_tree:
            if old_uuid == HDD_UUID:
                fresh = work / 'seed.tar.gz';capture(fresh);seed = fresh
            else:
                require(seed.exists() and manifest.get('seed_config_sha256'), 'Prepared configuration seed missing')
            populate(volume, seed)
        backup.parent.mkdir(parents=True, exist_ok=True);backup_files(backup)
        changed = True
        for name in ['/usr/local/libexec/up7000-recording-action', '/home/thoa/drone_tools/eskf/run_onboard.py']:
            p = Path(name);atomic_text(p, patch_uuid_source(p.read_text()))
        guard = Path('/usr/local/libexec/up7000-onboard-storage-check');atomic_text(guard, GUARD);guard.chmod(0o755)
        save_json(CONFIG, {'schema': 1, 'kind': 'sdcard_ext4', 'uuid': value, 'mount': str(MOUNT),
                           'media_validation': {k: media_validation[k] for k in
                                                ('phase', 'full_capacity_verified', 'waived_unix_s', 'completed_unix_s')
                                                if k in media_validation}})
        dropin = Path('/etc/systemd/system/mnt-droneup.mount.d/usb2.conf')
        if dropin.exists():dropin.unlink()  # Exact backed-up legacy HDD-only dependency.
        command(['systemctl', 'disable', '--now', 'up7000-hdd-usb2.service'])
        fstab = Path('/etc/fstab');atomic_text(fstab, new_fstab(fstab.read_text(), value))
        command(['sync', '-f', '/'])
        if os.path.ismount(MOUNT):command(['systemctl', 'stop', 'mnt-droneup.mount'])
        command(['umount', str(volume)])
        command(['systemctl', 'daemon-reload'])
        command(['systemctl', 'reset-failed', 'mnt-droneup.mount', *UNITS])
        command(['systemctl', 'start', 'mnt-droneup.mount'])
        verify_mount(MOUNT, value)
        command(['/usr/local/libexec/up7000-onboard-storage-check'])
        command(['systemctl', 'start', 'uwb-px4-logger', 'uwb-eskf-recorder', 'uwb-recording-control'])
        ready = wait_ready(value)
        command(['systemctl', 'enable', 'uwb-px4-logger', 'uwb-eskf-recorder', 'uwb-recording-control'])
        result = {'phase': 'card_active', 'uuid': value, 'identity': fingerprint(row),
                  'services': services(), 'runtime_on_emmc': True, 'logs_on_card': True,
                  'data_tree_reused': reuse_data_tree,
                  'media_validation': media_validation,
                  'os_backup': str(backup), 'activated_unix_s': time.time(), **ready}
        save_json(RECORDS / value / 'activation.json', result)
        out = DATA / 'flight_logs/card_setup';out.mkdir(exist_ok=True)
        os.chown(out, 1000, 1000)
        save_json(out / 'activation.json', result)
        command(['sync', '-f', str(MOUNT)])
        progress('card_active', **{k: v for k, v in result.items() if k != 'phase'})
        return result
    except BaseException as exc:
        failure = {'phase': 'activation_failed', 'error': str(exc), 'os_backup': str(backup)}
        if changed:
            subprocess.run(['systemctl', 'stop', *UNITS], capture_output=True)
            if mount_info() and mount_info()['uuid'] == value:
                subprocess.run(['systemctl', 'stop', 'mnt-droneup.mount'], capture_output=True)
            restore_files(backup)
        if stopped:
            # Original mount may still be present when failure happened early.
            if not os.path.ismount(MOUNT):
                subprocess.run(['systemctl', 'start', 'mnt-droneup.mount'], capture_output=True)
            subprocess.run(['systemctl', 'start', 'uwb-px4-logger', 'uwb-eskf-recorder', 'uwb-recording-control'], capture_output=True)
        failure['restored_services'] = services()
        save_json(RECORDS / value / 'activation_failure.json', failure)
        raise
    finally:
        if os.path.ismount(volume):command(['umount', str(volume)])
