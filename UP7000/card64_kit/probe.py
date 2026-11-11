"""Filesystem-only write/read qualification; no raw write or formatting."""
import hashlib
import os
from pathlib import Path
import shutil
import time
import uuid

from common import (CardError, GIB, RECORDS, RUN, command, fingerprint, mount_temporary,
                    progress, require, root_only, save_json, volume_for_uuid)

BLOCK = 4 * 1024**2


def hash_uncached(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        if hasattr(os, 'posix_fadvise'):
            os.posix_fadvise(stream.fileno(), 0, 0, os.POSIX_FADV_DONTNEED)
        for data in iter(lambda: stream.read(BLOCK), b''):
            h.update(data)
    return h.hexdigest()


def test_filesystem(root, budget, *, file_size=GIB, emit=lambda **_: None):
    """Write distinct data in new files, verify all of it, clean only on pass."""
    root = Path(root)
    require(not root.is_symlink(), 'Probe root must not be a symlink')
    folder = root / ('.up7000_probe_' + uuid.uuid4().hex)
    folder.mkdir(exist_ok=False)
    files = []
    total, write_seconds = 0, 0.0
    while total < budget:
        amount = min(file_size, budget - total)
        path = folder / ('part_%06d.bin' % len(files))
        h = hashlib.sha256(); first = None; tail = None
        start = time.monotonic()
        with path.open('xb') as stream:
            remaining = amount
            while remaining:
                data = os.urandom(min(BLOCK, remaining))
                if first is None: first = data
                tail = data
                stream.write(data); h.update(data); remaining -= len(data)
            stream.flush(); os.fsync(stream.fileno())
            if hasattr(os, 'posix_fadvise'):
                os.posix_fadvise(stream.fileno(), 0, 0, os.POSIX_FADV_DONTNEED)
        elapsed = time.monotonic() - start
        write_seconds += elapsed; total += amount
        entry = {'name': path.name, 'bytes': amount, 'sha256': h.hexdigest()}
        files.append(entry)
        # Catch ignored writes early, then still re-read every byte at the end.
        with path.open('rb') as stream:
            require(stream.read(len(first)) == first, 'First-block readback mismatch')
            stream.seek(amount - len(tail))
            require(stream.read(len(tail)) == tail, 'Last-block readback mismatch')
        emit(phase='writing_probe',written_bytes=total,total_bytes=budget)
    read_start = time.monotonic(); verified = 0
    for entry in files:
        path = folder / entry['name']
        require(hash_uncached(path) == entry['sha256'], f'Checksum mismatch: {entry["name"]}')
        verified += entry['bytes']
        emit(phase='verifying_probe',verified_bytes=verified,total_bytes=budget)
    read_seconds = time.monotonic() - read_start
    # This namespace was created exclusively by this call. Never recurse/delete.
    allowed = {e['name'] for e in files}
    require({p.name for p in folder.iterdir()} == allowed, 'Unexpected file in test directory')
    for entry in files:
        path = folder / entry['name']
        require(path.is_file() and not path.is_symlink() and path.parent == folder,
                'Unexpected probe path')
        path.unlink()
    folder.rmdir()
    return {'written_bytes': total, 'verified_bytes': verified, 'sha256_match': True,
            'write_mib_s': round(total / 1024**2 / max(write_seconds, .001), 2),
            'read_mib_s': round(total / 1024**2 / max(read_seconds, .001), 2),
            'file_sha256': files, 'probe_files_removed_after_success': True}


def qualify(value, mode='full', minimum_write=5.0):
    root_only()
    require(minimum_write >= 5, 'Qualification floor is at least 5 MiB/s')
    row = volume_for_uuid(value)
    volume = mount_temporary(row)
    result = {'phase': 'failed', 'identity': fingerprint(row), 'mode': mode,
              'started_unix_s': time.time(), 'minimum_write_mib_s': minimum_write}
    try:
        entries = list(volume.iterdir())
        require(all(p.name == 'lost+found' and p.is_dir() and not p.is_symlink()
                    and not any(p.iterdir()) for p in entries),
                'Qualification requires an empty ext4 card. Back up existing files first; no format/delete is performed.')
        free = shutil.disk_usage(volume).free
        budget = ((free - 2 * GIB) // GIB) * GIB if mode == 'full' else min(2 * GIB, free - 2 * GIB)
        require(budget >= GIB, 'Not enough writable free space')
        result.update(test_filesystem(volume, budget, emit=lambda **v: progress(**v)))
        require(result['write_mib_s'] >= minimum_write, 'Sustained write speed is below qualification floor')
        command(['sync', '-f', str(volume)])
        result.update(phase='qualified' if mode == 'full' else 'quick_check_only',
                      completed_unix_s=time.time(), tested_usable_bytes=budget,
                      note='Tests writable free space minus reserve; not a guarantee of RF performance or card lifetime')
    except BaseException as exc:
        result['error'] = str(exc)
        raise
    finally:
        save_json(RECORDS / row['uuid'] / 'qualification.json', result)
        command(['umount', str(volume)])
    print(__import__('json').dumps(result, indent=2), flush=True)
    return result
