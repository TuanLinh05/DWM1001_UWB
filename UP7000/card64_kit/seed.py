"""Capture estimator configuration; never copy flight archives or raw devices."""
import io
import json
import os
from pathlib import Path, PurePosixPath
import pwd
import tarfile

from common import BASE, DATA, HDD_UUID, MOUNT, PREFIX, command, digest, require, verify_mount


def capture(output):
    verify_mount(MOUNT, HDD_UUID)
    selected = [BASE / 'config', BASE / 'config_versions', BASE / 'active_config.json']
    require(all(p.exists() for p in selected), 'Live estimator configuration missing')
    pointer = json.loads((BASE / 'active_config.json').read_text())
    directory = Path(pointer['directory'])
    require(directory.is_relative_to(BASE / 'config_versions'), 'Invalid config pointer')
    gui = json.loads((directory / 'gui_config.json').read_text())
    import hashlib
    actual = hashlib.sha256(json.dumps(gui, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    require(actual == pointer['config_sha256'], 'Configuration checksum mismatch')
    output = Path(output)
    require(not output.exists(), 'Seed output already exists')
    files = []
    for path in selected:
        files += [path, *path.rglob('*')] if path.is_dir() else [path]
    require(not any(p.is_symlink() for p in files), 'Unexpected configuration symlink')
    require(sum(p.stat().st_size for p in files if p.is_file()) <= 32 * 1024**2, 'Configuration seed too large')
    with tarfile.open(output, 'w:gz', format=tarfile.PAX_FORMAT, dereference=True) as archive:
        for path in selected:
            archive.add(path, arcname=str(path.relative_to(MOUNT)), recursive=True)
    return {'seed_sha256': digest(output), 'config_sha256': actual,
            'archive_bytes': output.stat().st_size,
            'flight_archives_copied': False}


def safe_unpack(archive_path, destination):
    """Configuration archive contains directories and regular files only."""
    destination = Path(destination)
    with tarfile.open(archive_path, 'r:gz') as archive:
        members = archive.getmembers()
        total = 0
        for member in members:
            p = PurePosixPath(member.name)
            require(not p.is_absolute() and '..' not in p.parts and p.parts[0] == str(PREFIX),
                    'Archive path escapes the new data tree')
            require(member.isdir() or member.isfile(), 'Links and special files are not allowed in seed')
            total += member.size
        require(total <= 32 * 1024**2, 'Seed exceeds size limit')
        for member in members:
            path = destination / member.name
            require(not any(p.is_symlink() for p in [path, *path.parents] if p != destination.parent),
                    'Unexpected symlink in extraction path')
            if member.isdir():
                path.mkdir(parents=True, exist_ok=True)
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as source, path.open('xb') as target:
                    __import__('shutil').copyfileobj(source, target)
            os.chmod(path, member.mode & 0o777)
            os.chown(path, member.uid, member.gid)


def populate(volume, archive_path):
    require(not (volume / PREFIX).exists(), 'Card already has an onboard data tree; inspect before reuse')
    safe_unpack(archive_path, volume)
    data = volume / PREFIX / 'thoa'
    dirs = ['flight_logs/eskf', '.ros/log', 'cache/python', 'service_logs', 'tmp',
            'UP4000/ros2_ws/log', 'ros2_px4_v2/log', 'drone_tools/eskf/control',
            'drone_tools/eskf/backups', 'drone_tools/eskf/tests', 'drone_tools/eskf/usb2_validation']
    data.mkdir(parents=True, exist_ok=True)
    os.chown(data, 1000, 1000)
    for name in dirs:
        path = data / name
        path.mkdir(parents=True, exist_ok=True)
        # All intermediate user-data directories belong to the flight user.
        for parent in [path, *path.parents]:
            if parent == data.parent: break
            os.chown(parent, 1000, 1000)
    state = data / 'drone_tools/eskf/recording_context.json'
    state.write_text(json.dumps({'mode': 'autonomous', 'recording_id': ''}))
    os.chown(state, 1000, 1000)
    # Existing apt-cache symlinks can generate their disposable data on the card.
    apt = volume / PREFIX / 'system/apt'
    uid = pwd.getpwnam('_apt').pw_uid
    for name in ['archives', 'lists']:
        parent = apt / name;parent.mkdir(parents=True, exist_ok=True)
        part = parent / 'partial';part.mkdir(exist_ok=True)
        os.chmod(part, 0o700);os.chown(part, uid, 0)
    command(['sync', '-f', str(volume)])
    return {'data_directory': str(data), 'old_flight_logs_remain_on_hdd': True}
