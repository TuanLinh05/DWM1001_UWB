"""Fixed paths and non-destructive checks for the UP7000 data-card kit."""
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import time
import uuid

HDD_UUID = '896b17fa-8fe0-4111-853b-57b93ec87c60'
MOUNT = Path('/mnt/droneup')
PREFIX = Path('onboard_20261107')
DATA = MOUNT / PREFIX / 'thoa'
BASE = DATA / 'drone_tools/eskf'
CONFIG = Path('/etc/up7000-storage.json')
RUN = Path('/run/up7000-card64')
RECORDS = Path('/var/lib/up7000-card64')
UNITS = ['uwb-recording-control', 'uwb-eskf-recorder', 'uwb-px4-logger']
GIB = 1024**3
KIT = Path(__file__).resolve().parent


class CardError(RuntimeError):
    pass


def require(value, message):
    if not value:
        raise CardError(message)


def command(args, timeout=90):
    p = subprocess.run([str(x) for x in args], capture_output=True, text=True, timeout=timeout)
    require(p.returncode == 0, f'{args[0]} failed: {(p.stderr or p.stdout)[-1600:]}')
    return p.stdout


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024**2), b''):
            result.update(block)
    return result.hexdigest()


def save_json(path, value, mode=0o644):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name('.' + path.name + '.tmp')
    require(not temp.is_symlink(), 'Unexpected temporary symlink')
    fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, mode)
    with os.fdopen(fd, 'w') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)


def atomic_text(path, text):
    """Replace a small OS/source file without leaving a truncated version."""
    path = Path(path)
    require(not path.is_symlink(), 'Refusing OS/source symlink')
    before = path.stat() if path.exists() else None
    temp = path.with_name('.card64-' + uuid.uuid4().hex + '.tmp')
    try:
        fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write(text);stream.flush()
            os.fchmod(stream.fileno(), (before.st_mode & 0o777) if before else 0o644)
            if before and (before.st_uid, before.st_gid) != (os.geteuid(), os.getegid()):
                os.fchown(stream.fileno(), before.st_uid, before.st_gid)
            os.fsync(stream.fileno())
        os.replace(temp, path)
        parent = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:os.fsync(parent)
        finally:os.close(parent)
    finally:
        if temp.exists():temp.unlink()


def root_only():
    require(os.geteuid() == 0, 'Run this operation with sudo')


def uuid_checked(value):
    require(isinstance(value, str) and bool(re.fullmatch(r'[0-9a-fA-F]{8}(?:-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}', value)),
            'Use the ext4 filesystem UUID, not an sdX guess')
    return value.lower()


def all_volumes():
    tree = json.loads(command(['lsblk', '-b', '-J', '-p', '-o',
                              'NAME,TYPE,SIZE,FSTYPE,UUID,MOUNTPOINTS,RO,TRAN,MODEL,SERIAL']))
    result = []

    def visit(row, parent=None):
        item = {k: v for k, v in row.items() if k != 'children'}
        item['disk_name'] = parent['disk_name'] if parent else row['name']
        item['transport'] = row.get('tran') or (parent or {}).get('transport')
        item['disk_model'] = (parent or {}).get('disk_model') or row.get('model')
        item['disk_serial'] = (parent or {}).get('disk_serial') or row.get('serial')
        item['disk_size'] = (parent or {}).get('disk_size') or int(row['size'])
        result.append(item)
        for child in row.get('children', []):
            visit(child, item)

    for row in tree['blockdevices']:
        visit(row)
    return result


def validate_volume(row, rows):
    require(row['type'] in ('disk', 'part'), 'Not a direct disk/partition')
    require(50_000_000_000 <= int(row['size']) <= 70_000_000_000,
            'Expected a roughly 64 GB data volume (50–70 billion bytes)')
    require(50_000_000_000 <= row['disk_size'] <= 70_000_000_000,
            'Parent is not the expected 64 GB card')
    require(row.get('fstype') == 'ext4', 'Card must be ext4. This kit never formats a disk.')
    uuid_checked(row.get('uuid'))
    require(not row.get('ro'), 'Card is write-protected')
    require(row.get('transport') in ('usb', 'mmc'), 'Expected a USB reader or native SD device')
    siblings = [r for r in rows if r['disk_name'] == row['disk_name']]
    require(not any(p in ('/', '/boot', '/boot/efi', '[SWAP]')
                    for r in siblings for p in (r.get('mountpoints') or [])),
            'Refusing the operating-system disk or swap')
    require(row.get('uuid') != HDD_UUID, 'Refusing the source HDD')
    return row


def volume_for_uuid(value):
    value = uuid_checked(value)
    rows = all_volumes()
    found = [r for r in rows if str(r.get('uuid', '')).lower() == value]
    require(len(found) == 1, 'UUID is missing or duplicated; inspect devices first')
    row = validate_volume(found[0], rows)
    device = Path('/dev/disk/by-uuid') / value
    require(device.exists() and stat.S_ISBLK(device.stat().st_mode), 'Expected block device missing')
    require(device.resolve() == Path(row['name']).resolve(), 'Device identity changed')
    disk = Path(row['disk_name']).resolve().name
    sysdevice = (Path('/sys/class/block') / disk / 'device').resolve()
    usb = next((p for p in [sysdevice, *sysdevice.parents] if (p / 'idVendor').exists()), None)
    if row['transport'] == 'usb':
        require(usb is not None, 'USB reader not found in sysfs')
        vendor, product = [(usb / p).read_text().strip() for p in ('idVendor', 'idProduct')]
        require((vendor, product) != ('048d', '1234'), 'Refusing the previously unreliable SSD controller')
        speed = (usb / 'speed').read_text().strip()
        require(speed == '480', 'Use a USB 2.0 reader/cable (480 Mb/s); USB3 requires separate RF testing')
        row.update(usb_speed_mbps=480, usb_vendor=vendor, usb_product=product)
    row['uuid'] = value
    row['by_uuid'] = str(device)
    return row


def fingerprint(row):
    return {k: row.get(k) for k in ['uuid', 'size', 'disk_size', 'fstype', 'disk_model',
                                   'disk_serial', 'transport', 'usb_vendor', 'usb_product']}


def mount_info(path=MOUNT):
    p = subprocess.run(['findmnt', '-J', '--mountpoint', str(path), '-o', 'SOURCE,UUID,FSTYPE,OPTIONS'],
                       capture_output=True, text=True)
    return json.loads(p.stdout)['filesystems'][0] if p.returncode == 0 else None


def verify_mount(path, value):
    info = mount_info(path)
    require(info and info['uuid'] == value and info['fstype'] == 'ext4', 'Wrong or missing data mount')
    require('rw' in info['options'].split(','), 'Data volume is not writable')
    require(Path(path).stat().st_dev != Path('/').stat().st_dev, 'Refusing eMMC fallback')
    return info


def mount_temporary(row):
    require(not any(row.get('mountpoints') or []), 'Unmount this card before checking/activation')
    path = RUN / ('volume_' + row['uuid'])
    path.mkdir(parents=True, exist_ok=True)
    require(not os.path.ismount(path), 'Temporary mount is already in use')
    command(['mount', '-t', 'ext4', '-o', 'rw,noatime,nosuid,nodev', row['by_uuid'], str(path)])
    verify_mount(path, row['uuid'])
    return path


def current_uuid():
    return json.loads(CONFIG.read_text())['uuid'] if CONFIG.exists() else HDD_UUID


def services():
    return {u: subprocess.run(['systemctl', 'is-active', u], capture_output=True,
                              text=True).stdout.strip() for u in UNITS}


def progress(phase, **values):
    value = {'phase': phase, 'updated_unix_s': time.time(), **values}
    save_json(RUN / 'status.json', value)
    print(json.dumps(value), flush=True)


def check_kit():
    manifest = json.loads((KIT / 'manifest.json').read_text())
    for name, expected in manifest['sha256'].items():
        p = KIT / name
        require(not p.is_symlink() and p.is_file(), f'Missing kit file: {name}')
        require(digest(p) == expected, f'Kit checksum mismatch: {name}')
    return manifest
