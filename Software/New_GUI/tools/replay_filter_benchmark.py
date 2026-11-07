"""Replay every Burst in a recorded CSV and compare with the recorded output.

python tools/replay_filter_benchmark.py logs/<session> --output <directory>
Output includes missing-fix counts; a smoother frozen track is not success.
This is a continuity benchmark, not a ground-truth position accuracy test.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core_state import AnchorConfig, UwbStateEngine
from radio_quality import QualityReplay
import telemetry_protocol as tp


def optional(row, key):
    value = row.get(key, '')
    return float(value) if value else None


def trajectory_metrics(xyz, times):
    valid = np.all(np.isfinite(xyz), axis=1)
    pairs = valid[1:] & valid[:-1]
    dt = np.diff(times)
    steps = np.linalg.norm(np.diff(xyz, axis=0), axis=1) * 1000
    steps = steps[pairs]
    # Show both all consecutive samples and samples within a normal radio cycle.
    normal = pairs & (dt > 0) & (dt < .02)
    small_steps = np.linalg.norm(np.diff(xyz, axis=0)[normal], axis=1) * 1000
    return {
        'valid_fixes': int(valid.sum()), 'missing_fixes': int((~valid).sum()),
        'jumps_gt90mm': int(np.sum(steps > 90)),
        'normal_cycle_jumps_gt90mm': int(np.sum(small_steps > 90)),
        'max_step_mm': round(float(np.max(steps)), 3) if len(steps) else None,
        'p99_step_mm': round(float(np.quantile(steps, .99)), 3) if len(steps) else None,
        'xyz_span_m': np.ptp(xyz[valid], axis=0).round(3).tolist() if valid.any() else [],
    }


def benchmark(session: Path, output: Path, mode: str, signal_quality: bool = True):
    metadata = json.loads((session / 'session_metadata.json').read_text(encoding='utf-8'))
    engine = UwbStateEngine([AnchorConfig(**a) for a in metadata['anchors']])
    config = metadata.get('solver', {})
    engine.solver_dimension = config.get('dimension', 3)
    engine.fixed_z_m = config.get('fixed_z_m', 0)
    engine.allow_uncalibrated_positioning = config.get('allow_uncalibrated_positioning', True)
    engine.use_nlos_downweight = config.get('use_nlos_downweight', True)
    engine.use_signal_quality = signal_quality
    engine.info_flags = metadata.get('power_info_flags', engine.info_flags)
    engine.set_position_filter_mode(mode)
    baseline, trajectory, times, used, duration_ms, stds = [], [], [], [], [], []
    accepted = rejected = 0
    start = time.perf_counter()
    csv_path = session / 'uwb_fast_log.csv'
    with csv_path.open(newline='', encoding='utf-8') as handle, QualityReplay(csv_path) as quality_replay:
        for index, row in enumerate(csv.DictReader(handle)):
            entries = []
            for aid in range(1, 9):
                mm = optional(row, f'a{aid}_mm')
                if mm is None:
                    continue
                fp = optional(row, f'a{aid}_fp_dbm')
                nlos = optional(row, f'a{aid}_nlos_db')
                ci = optional(row, f'a{aid}_ci_ppm')
                flags = optional(row, f'a{aid}_flags')
                if flags is None:
                    flags = tp.BURST_FLAG_RADIO_OK
                    if engine.anchors[aid].fw_calibrated:
                        flags |= tp.BURST_FLAG_VALID | tp.BURST_FLAG_CAL_OK
                entries.append(tp.BurstAnchor(aid, int(optional(row, f'a{aid}_status_code') or 0),
                    int(flags), round(mm), round(fp * 100) if fp is not None else tp.INT16_MIN,
                    round((fp + nlos) * 100) if fp is not None and nlos is not None else tp.INT16_MIN,
                    round(ci * 100) if ci is not None else tp.INT16_MIN))
            tag_us = int(row['tag_time_us'])
            seq = int(row['cycle_seq'])
            msg = tp.RangeBurstMessage(seq, tag_us // 1000, int(row['boot_id']), seq,
                seq * 8, tag_us, int(row['period_us']), tuple(entries))
            before = time.perf_counter()
            quality_replay.feed_before_row(engine, index, float(row['host_time_s']))
            engine.info_flags = int(optional(row, 'power_info_flags') if row.get('power_info_flags') else engine.info_flags)
            engine.ingest_message(msg, float(row['host_time_s']))
            duration_ms.append((time.perf_counter() - before) * 1000)
            sol = engine.latest_solution
            times.append(tag_us / 1e6)
            recorded = [optional(row, f'pos_{a}_m') for a in 'xyz']
            baseline.append([float('nan') if v is None else v for v in recorded])
            trajectory.append([sol.x, sol.y, sol.z] if sol else [float('nan')] * 3)
            used.append(len(sol.used_anchor_ids) if sol else 0)
            stds.append(sol.position_std_m if sol else [float('nan')] * 3)
            accepted += len(engine.range_ekf.accepted_ids)
            rejected += len(engine.range_ekf.rejected)
            if (index + 1) % 10000 == 0:
                print(f'{session.name}: {index + 1} samples, {time.perf_counter()-start:.1f}s', flush=True)
    trajectory, baseline, times = np.asarray(trajectory), np.asarray(baseline), np.asarray(times)
    result = {
        'session': session.name, 'filter': mode, 'signal_quality': signal_quality, 'samples': len(times),
        'quality_model': engine.radio_quality.metadata(),
        'raw_quality_sidecar': (session / 'rx_quality.jsonl').is_file(),
        'recorded': trajectory_metrics(baseline, times), 'replayed': trajectory_metrics(trajectory, times),
        'accepted_observations': accepted, 'rejected_observations': rejected,
        'initializations': engine.range_ekf.initialization_count,
        'processing_ms_p50_p95_p99_max': np.quantile(duration_ms, [.5, .95, .99, 1]).round(3).tolist(),
        'elapsed_s': round(time.perf_counter()-start, 3),
        'note': 'No ground truth: continuity metrics do not establish absolute position accuracy.',
    }
    output.mkdir(parents=True, exist_ok=True)
    stem = session.name + '_' + mode + ('_quality_on' if signal_quality else '_quality_off')
    (output / (stem + '.json')).write_text(json.dumps(result, indent=2), encoding='utf-8')
    np.savez_compressed(output / (stem + '.npz'), times=times, baseline=baseline,
        trajectory=trajectory, used=used, stds=stds)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sessions', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=['legacy', 'robust_ekf'], default='robust_ekf')
    parser.add_argument('--signal-quality', choices=['on', 'off'], default='on')
    args = parser.parse_args()
    for path in args.sessions:
        benchmark(path, args.output, args.mode, args.signal_quality == 'on')
