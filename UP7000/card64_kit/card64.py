#!/usr/bin/python3 -B
"""UP7000: qualify a 64 GB ext4 card and use it for data, keeping code on eMMC."""
import argparse
import fcntl
import json
import os
import sys
from pathlib import Path

from common import (CardError, KIT, RUN, all_volumes, check_kit, require, root_only,
                    save_json, uuid_checked, validate_volume)


def inspect():
    rows = all_volumes();result = []
    for row in rows:
        if row['type'] not in ('part', 'disk'):continue
        try:
            validate_volume(row, rows);row['candidate_ext4_64gb'] = True
        except CardError as exc:
            row.update(candidate_ext4_64gb=False, reason=str(exc))
        result.append(row)
    return {'phase': 'inventory', 'volumes': result,
            'note': 'No disk is formatted or selected automatically; USB speed is checked during qualification.'}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='operation', required=True)
    sub.add_parser('inspect', help='Read-only device inventory')
    sub.add_parser('check-kit', help='Check packaged file SHA-256')
    for name in ['qualify', 'activate']:
        command = sub.add_parser(name)
        command.add_argument('--uuid', required=True, type=uuid_checked)
        if name == 'qualify':
            command.add_argument('--mode', choices=['full', 'quick'], default='full')
            command.add_argument('--minimum-write-mib-s', type=float, default=5.0)
        else:
            command.add_argument('--waive-full-card-test', action='store_true',
                                 help='Record an explicit user waiver of the full-capacity write/read test')
    args = ap.parse_args()
    if args.operation == 'inspect':print(json.dumps(inspect(), indent=2));return
    if args.operation == 'check-kit':print(json.dumps(check_kit(), indent=2));return
    root_only();RUN.mkdir(parents=True, exist_ok=True)
    with (RUN / 'operation.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        check_kit()
        if args.operation == 'qualify':
            from probe import qualify
            qualify(args.uuid, args.mode, args.minimum_write_mib_s)
        elif args.operation == 'activate':
            from activate import activate
            print(json.dumps(activate(args.uuid, waive_full_card_test=args.waive_full_card_test), indent=2))


if __name__ == '__main__':
    try:main()
    except (CardError, OSError, ValueError) as exc:
        print(json.dumps({'phase': 'refused_or_failed', 'error': str(exc)}), file=sys.stderr)
        sys.exit(1)
