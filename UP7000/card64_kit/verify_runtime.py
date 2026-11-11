#!/usr/bin/python3 -B
"""Check card runtime and optionally exercise GUI Log/Save on the ground."""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import uuid

from common import BASE, DATA, MOUNT, command, current_uuid, require, services, verify_mount

KIND = 'up7000_recording_control'


def request(action, recording_id='', config=None):
    rid = uuid.uuid4().hex
    value = {'kind': KIND, 'version': 1, 'request_id': rid, 'action': action,
             'recording_id': recording_id, 'gui_session': '64 GB card validation'}
    if config is not None:
        value['config'] = config
        value['config_sha256'] = hashlib.sha256(json.dumps(config, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    raw = json.dumps(value, separators=(',', ':'), sort_keys=True, allow_nan=False).encode()
    packets = [raw]
    if len(raw) > 1200:
        chunks = [raw[i:i+600] for i in range(0, len(raw), 600)]
        packets = [json.dumps({'kind': KIND, 'version': 1, 'request_id': rid, 'action': 'fragment',
            'index': i, 'count': len(chunks), 'sha256': hashlib.sha256(raw).hexdigest(),
            'data': base64.b64encode(c).decode()}, separators=(',', ':')).encode() for i, c in enumerate(chunks)]
    end, last = time.monotonic() + 50, 0
    address = ('127.0.0.1', 15558)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(.2)
        while time.monotonic() < end:
            if time.monotonic() - last > 1:
                for packet in packets:sock.sendto(packet, address)
                last = time.monotonic()
            try:data, source = sock.recvfrom(4096)
            except socket.timeout:continue
            if source != address or len(data) > 1200:continue
            reply = json.loads(data)
            if reply.get('request_id') == rid and reply.get('action') == action:
                require(reply.get('ok'), str(reply));return reply
    raise RuntimeError('Recording-control reply timed out')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--rotate-test', action='store_true', help='Maintenance only: start/save test session, then resume background')
    args = ap.parse_args()
    value = current_uuid();verify_mount(MOUNT, value)
    status = request('status')
    require(status['active'] and not status.get('error'), 'Recorder is not healthy')
    rootdev = Path('/').stat().st_dev
    for name in ['/opt/ros/humble','/home/thoa/UP4000','/home/thoa/ros2_px4_v2','/home/thoa/drone_tools']:
        require(Path(name).stat().st_dev == rootdev, 'Runtime escaped eMMC')
    result = {'uuid': value, 'services': services(), 'status': status, 'checked_unix_s': time.time()}
    if args.rotate_test:
        require(not status['recording_id'], 'A user GUI owns the current recording')
        # Operator runs activation/rotation on the ground; no vehicle commands are sent.
        pointer = json.loads((BASE / 'active_config.json').read_text())
        config = json.loads((Path(pointer['directory']) / 'gui_config.json').read_text())
        recording_id = uuid.uuid4().hex
        start = request('start', recording_id, config)
        try:time.sleep(5)
        finally:finish = request('finish', recording_id)
        require(finish['closed'] and finish['background_recording'], 'Save did not finalize/resume')
        folder = Path(start['session'])
        require(folder.is_relative_to(DATA / 'flight_logs/eskf') and folder.stat().st_dev == MOUNT.stat().st_dev,
                'Test bag was not written on selected data volume')
        # Source environment in a child shell, then decode every native CDR message.
        decoder = r'''
import collections,json,sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
types={t.name:t.type for t in r.get_all_topics_and_types()};counts=collections.Counter()
while r.has_next():
 topic,data,stamp=r.read_next();deserialize_message(data,get_message(types[topic]));counts[topic]+=1
if not counts['/uwb_eskf/status']:raise RuntimeError('Missing estimator status in bag')
print(json.dumps(dict(counts)))
'''
        shell = 'source /home/thoa/onboard_env.sh && source /home/thoa/ros2_px4_v2/install/setup.bash && exec python3 -B -c "$1" "$2"'
        counts = json.loads(command(['bash', '-c', shell, 'card-decoder', decoder, str(folder / 'bag')]))
        result.update(start=start, finish=finish, native_cdr_message_counts=counts, background=request('status'))
    destination = DATA / 'flight_logs/card_setup'
    destination.mkdir(exist_ok=True)
    out = destination / ('runtime_%s.json' % int(time.time()))
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':main()
