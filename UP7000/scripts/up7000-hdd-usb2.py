#!/usr/bin/python3 -B
"""Keep the tested HDD connector on USB2; run before mounting its filesystem."""
import json
from pathlib import Path
import subprocess
import time

PCI = Path('/sys/bus/pci/devices/0000:00:14.0')
UUID = '896b17fa-8fe0-4111-853b-57b93ec87c60'
SERIAL = 'DD564198838A2'


def device():
    for path in PCI.glob('usb*/*'):
        try:
            if (path / 'serial').read_text().strip() == SERIAL:
                return path, (path / 'speed').read_text().strip()
        except OSError:
            pass
    return None, None


def main():
    if (PCI / 'vendor').read_text().strip() != '0x8086' or (PCI / 'device').read_text().strip() != '0x54ed':
        raise RuntimeError('Unexpected USB controller; refusing to change ports')
    end = time.monotonic() + 12
    port = None
    while time.monotonic() < end:
        for root in PCI.glob('usb*'):
            try:
                if float((root / 'speed').read_text()) >= 5000:
                    name = root.name[3:]
                    candidate = root / (name+'-0:1.0') / ('usb'+name+'-port1')
                    if (candidate / 'disable').is_file() and (candidate / 'peer').exists():
                        port = candidate
                        break
            except OSError:
                pass
        if port:
            break
        time.sleep(.1)
    if port is None:
        raise RuntimeError('Expected USB3 port1/peer interface is missing')
    peer = (port / 'peer').resolve()
    for node in (port, peer):
        child = node / 'device'
        if child.exists() and (child / 'serial').read_text().strip() != SERIAL:
            raise RuntimeError('HDD connector occupied by another device; refusing to disconnect it')
    disk, speed = device()
    if speed == '480' and (port / 'disable').read_text().strip() == '1':
        print(json.dumps({'mode': 'USB2', 'speed_mbps': 480, 'device': disk.name, 'unchanged': True}))
        return
    if disk is not None:
        fsck = subprocess.check_output(['systemd-escape', '--path', '--template=systemd-fsck@.service',
                                       '/dev/disk/by-uuid/' + UUID], text=True).strip()
        if subprocess.run(['systemctl', 'is-active', '--quiet', fsck]).returncode == 0:
            raise RuntimeError('Refusing to disconnect HDD during filesystem check')
        block = Path('/dev/disk/by-uuid') / UUID
        end = time.monotonic() + 8
        while not block.exists() and time.monotonic() < end:
            time.sleep(.1)
        if not block.exists():
            raise RuntimeError('Expected HDD filesystem UUID is missing')
        parts = json.loads(subprocess.check_output(['lsblk', '-J', '-o', 'NAME,MOUNTPOINTS'], text=True))
        name = block.resolve().name.rstrip('0123456789')
        def mounted(rows):
            return any(any(r.get('mountpoints') or []) or mounted(r.get('children', [])) for r in rows)
        if mounted([r for r in parts['blockdevices'] if r['name'] == name]):
            raise RuntimeError('Refusing to disconnect a mounted HDD')
    original = (port / 'disable').read_text().strip()
    try:
        (port / 'disable').write_text('1')
        if disk is not None:
            end = time.monotonic() + 8
            while time.monotonic() < end:
                new, speed = device()
                if speed == '480' and (Path('/dev/disk/by-uuid') / UUID).exists():
                    print(json.dumps({'mode': 'USB2', 'speed_mbps': 480, 'device': new.name}))
                    return
                time.sleep(.2)
            raise RuntimeError('HDD did not reappear at USB2 speed')
        print(json.dumps({'mode': 'USB2', 'connector': str(port), 'waiting_for_hdd': True}))
    except Exception:
        (port / 'disable').write_text(original)
        raise


if __name__ == '__main__':
    main()
