#!/usr/bin/env python3
"""Supervise the read-only estimator and native rosbag2 MCAP recorder on HDD."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

UUID = '896b17fa-8fe0-4111-853b-57b93ec87c60'
MOUNT = Path('/mnt/droneup')
DATA = MOUNT / 'onboard_20261107/thoa'
BASE = DATA / 'drone_tools/eskf'
LOGS = DATA / 'flight_logs/eskf'
EXECUTABLE = Path('/home/thoa/ros2_px4_v2/install/uwb_eskf/lib/uwb_eskf/uwb_eskf_node')
TOPICS = ['/px4/imu_frd', '/px4/imu_device_time_us', '/px4/odometry', '/px4/status',
          '/px4/snapshot',
          '/uwb/cycle_seq', '/uwb/tag_time_us', '/uwb/device_info', '/uwb/burst_metadata',
          '/uwb_relay/status', '/uwb_eskf/odom', '/uwb_eskf/status', '/uwb_eskf/session']
TOPICS += [f'/uwb/anchor_{i}/{kind}' for i in range(1, 9) for kind in ('range', 'diagnostic_range')]


def storage_check():
    actual = subprocess.check_output(['findmnt', '-n', '-o', 'UUID', '--target', str(MOUNT)], text=True).strip()
    if actual != UUID or not os.path.ismount(MOUNT):
        raise RuntimeError('Required onboard HDD is not mounted with the expected UUID')
    if not str(BASE.resolve()).startswith(str(DATA) + '/'):
        raise RuntimeError('Estimator storage escaped HDD root')
    if shutil.disk_usage(MOUNT).free < 2 * 1024**3:
        raise RuntimeError('HDD has less than 2 GiB free; stopping recording')


def atomic_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value, indent=2), encoding='utf-8')
    tmp.replace(path)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def discovery_profile():
    path = Path(os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'])
    values = ET.parse(path).findall('.//{*}maxInitialPeersRange')
    if not values or max(int(v.text) for v in values) < 32:
        raise RuntimeError('Fast DDS UDP profile requires maxInitialPeersRange >= 32 for the onboard process graph')
    return path


def stop_children(children):
    for child in children.values():
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
    deadline = time.monotonic() + 22
    for child in children.values():
        try:
            child.wait(max(.1, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()
    storage_check()
    dds_profile = discovery_profile()
    config_dir = BASE / 'config'
    pointer = BASE / 'active_config.json'
    selected = json.loads(pointer.read_text()) if pointer.exists() else {}
    if selected:
        config_dir = Path(selected['directory']).resolve()
        if not config_dir.is_relative_to(BASE / 'config_versions'):
            raise RuntimeError('Invalid active ESKF configuration directory')
        import hashlib
        raw = json.loads((config_dir / 'gui_config.json').read_text())
        digest = hashlib.sha256(json.dumps(raw, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        if digest != selected['config_sha256']:
            raise RuntimeError('Active ESKF configuration checksum mismatch')
    config = config_dir / 'eskf.yaml'
    rig = config_dir / 'rig_onboard.txt'
    survey = config_dir / 'physical_setup.json'
    for path in (EXECUTABLE, config, rig, survey):
        if not path.is_file():
            raise RuntimeError(f'Missing required file: {path}')
    if args.check:
        print(json.dumps({'storage_uuid': UUID, 'config': str(config), 'log_root': str(LOGS),
                          'executable': str(EXECUTABLE), 'topics': TOPICS}))
        return 0

    lock = (BASE / 'supervisor.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    LOGS.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S', time.gmtime())
    folder = LOGS / (stamp + '_' + str(os.getpid()))
    folder.mkdir()
    snapshot = folder / 'config'
    snapshot.mkdir()
    for path in (config, rig, survey, dds_profile, Path(__file__).resolve()):
        shutil.copy2(path, snapshot / path.name)
    if (config_dir / 'gui_config.json').exists():
        shutil.copy2(config_dir / 'gui_config.json', snapshot / 'gui_config.json')
    commands = {
        'estimator': [str(EXECUTABLE), '--ros-args', '--params-file', str(config)],
        'recorder': ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(folder / 'bag'),
                     '--max-bag-duration', '600', '--max-bag-size', '1073741824',
                     '--max-cache-size', '33554432', *TOPICS],
    }
    context_path = BASE / 'recording_context.json'
    context = json.loads(context_path.read_text()) if context_path.exists() else {'mode': 'autonomous', 'recording_id': ''}
    info = {'started_unix_s': time.time(), 'folder': str(folder), 'clock': 'UP7000 receive clock; IMU device intervals',
            'hdd_uuid': UUID, 'commands': commands, 'sha256': {str(p): sha(p) for p in (EXECUTABLE, config, rig, survey, dds_profile, Path(__file__).resolve())},
            'physical_setup': json.loads(survey.read_text()), 'config_sha256': selected.get('config_sha256', ''),
            'recording_context': context, 'closed': False, 'error': '',
            'role': 'local_estimation_and_recording_only', 'last_estimator_status': None}
    atomic_json(folder / 'session.json', info)
    link = BASE / 'latest_session.tmp'
    if link.is_symlink():
        link.unlink()
    link.symlink_to(folder)
    link.replace(BASE / 'latest_session')

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('onboard_eskf_supervisor')
    def status(msg):
        try:
            info['last_estimator_status'] = json.loads(msg.data)
            info['last_status_received_unix_s'] = time.time()
        except ValueError:
            pass
    subscription = node.create_subscription(String, '/uwb_eskf/status', status, 10)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, '/uwb_eskf/session', latched)
    metadata = String(data=json.dumps(info, separators=(',', ':')))
    stopping = False
    def handle_stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)
    children, handles = {}, []
    exit_code = 0
    try:
        for name, command in commands.items():
            handle = (folder / (name + '.log')).open('ab', buffering=0)
            handles.append(handle)
            children[name] = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        publisher.publish(metadata)
        print('Onboard ESKF session: ' + str(folder), flush=True)
        last = 0
        while not stopping:
            rclpy.spin_once(node, timeout_sec=.2)
            for name, child in children.items():
                if child.poll() is not None:
                    raise RuntimeError(f'{name} exited unexpectedly: {child.returncode}')
            if time.monotonic() - last > 1:
                storage_check()
                if time.time() - info['started_unix_s'] > 20:
                    missing = [topic for topic in ('/uwb_eskf/odom', '/uwb_eskf/status', '/px4/imu_frd')
                               if not any(endpoint.node_name == 'rosbag2_recorder'
                                          for endpoint in node.get_subscriptions_info_by_topic(topic))]
                    if missing:
                        raise RuntimeError('Recorder did not subscribe to: ' + ', '.join(missing))
                    if time.time() - info.get('last_status_received_unix_s', 0) > 5:
                        raise RuntimeError('Estimator status stopped arriving')
                    info['recorder_topics_verified'] = True
                info['updated_unix_s'] = time.time()
                info['processes'] = {name: child.pid for name, child in children.items()}
                atomic_json(folder / 'session.json', info)
                publisher.publish(metadata)
                last = time.monotonic()
    except Exception as exc:
        info['error'] = str(exc)
        exit_code = 1
    finally:
        stop_children(children)
        info['exit_codes'] = {name: child.returncode for name, child in children.items()}
        info['ended_unix_s'] = time.time()
        # Native rosbag2 writes metadata.yaml when SIGINT finalization completes.
        info['closed'] = (folder / 'bag/metadata.yaml').is_file() and children.get('recorder') is not None and children['recorder'].returncode == 0
        if not info['closed']:
            info['error'] = info['error'] or 'Bag was not finalized successfully'
            exit_code = 1
        try:
            atomic_json(folder / 'session.json', info)
        except OSError as exc:
            print('Cannot write HDD session metadata: ' + str(exc), file=sys.stderr)
            exit_code = 1
        for handle in handles:
            handle.close()
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
