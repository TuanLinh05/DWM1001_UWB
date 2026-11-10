"""Export chronological ESKF events and full-state Python references for C++ parity.

Example: python tools/export_core_events.py --out logs/e22_validation/parity --flight Logs/10112026/Test_1
Synthetic fixtures are imported only for this validation tool, never by the onboard core.
"""
import argparse
import csv
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

from flight_log_io import (GUI_DIR, interval_mask, load_flight, read_mcap_cache, read_ulog,
                           resolve_input, rig_attitude_rotations, usable_epochs, write_json)
from eskf_replay import (AttitudeSample, EskfParams, EstimatorCore, ImuSample, RangeSample,
                         px4_attitude_to_rig, px4_yaw_ned, rig_yaw_offset_from_yaw0, sorted_events)
from eskf_tuning import estimate_imu_noise, normalized_errors, recommend_params


def summary(core):
    return dict(reinits=core.reinits, skipped_ranges=core.skipped_ranges, imu_gap_resets=core.imu_gap_resets,
                attitude_inits=core.attitude_inits, initializations=core.initializations,
                offered_ranges=len(core.updates), accepted_ranges=core.accepted_ranges,
                accepted_attitudes=core.accepted_attitudes, rejected_attitudes=core.rejected_attitudes,
                output_rows=len(core.rows), init_time=core.init_time)


def export_bundle(out, anchors, imu, ranges, attitude, lever, yaw0, params):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    write_json(out / "metadata.json", dict(params=asdict(params), lever=lever, yaw0=yaw0,
               state_columns="t,p[3],v[3],R[row-major 9],ba[3],bg[3],P[row-major 225]",
               update_columns="kind(1=range,2=attitude),t,id,accepted,NIS,innovation(range only)",
               event_order="time, IMU/range/attitude, anchor ID, payload"))
    with (out / "config.csv").open("w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        for key, value in asdict(params).items():
            writer.writerow([key, *(value if isinstance(value, tuple) else [int(value) if isinstance(value, bool) else value])])
        writer.writerow(["lever", *lever])
        writer.writerow(["yaw0", yaw0])
    np.savetxt(out / "anchors.csv", [[a.anchor_id, a.x, a.y, a.z] for a in anchors if a.enabled], delimiter=",", fmt="%.17g")
    core = EstimatorCore(anchors, lever, yaw0, params)
    start = perf_counter()
    with (out / "events.csv").open("w", newline="") as ef, (out / "python_states.csv").open("w", newline="") as sf, \
            (out / "python_updates.csv").open("w", newline="") as uf:
        events, states, updates = [csv.writer(f, lineterminator="\n") for f in (ef, sf, uf)]
        event_count = 0
        for t, kind, aid, _, sample in sorted_events(imu, ranges, attitude, core.anchors):
            before = len(core.rows)
            if kind == 0:
                events.writerow([kind, t, 0, *sample.acc, *sample.gyro])
                core.on_imu(t, sample.acc, sample.gyro)
            elif kind == 1:
                events.writerow([kind, t, aid, sample.range_m])
                core.on_range(aid, t, sample.range_m)
            else:
                events.writerow([kind, t, 0, *np.asarray(sample.R).ravel()])
                core.on_attitude(t, sample.R)
            if len(core.rows) != before:
                f = core.filter
                states.writerow([core.rows[-1][0], *f.p, *f.v, *f.R.ravel(), *f.ba, *f.bg, *f.P.ravel()])
            if core.last_update is not None:
                k, stamp, id_, accepted, nis, innovation = core.last_update
                updates.writerow([k, stamp, id_, int(accepted), nis, innovation])
            event_count += 1
    write_json(out / "python_summary.json", summary(core))
    print(f"{out.name}: {event_count} events, {len(core.rows)} outputs, {len(core.updates)} range updates, "
          f"{perf_counter()-start:.2f} s", flush=True)
    return core


def _samples(sim, keep=lambda t: True):
    imu = [ImuSample(float(t), tuple(a), tuple(g)) for t, a, g in zip(sim['imu_t'], sim['imu_acc'], sim['imu_gyro']) if keep(t)]
    ranges = [RangeSample(float(t), int(aid), float(r)) for t, aid, r in sim['ranges']]
    return imu, ranges


def synthetic_cases(out):
    sys.path.insert(0, str(GUI_DIR / "tests"))
    import e21_fixtures as F
    from test_e22_onboard_core import _attitude, _bin_means, _params
    groups = {}
    def case(group, name, sim, params, attitude=(), imu=None):
        original, ranges = _samples(sim)
        export_bundle(out / group / name, F.ROOM_48, original if imu is None else imu, ranges,
                      attitude, sim['lever'], sim['yaw0'], params)
        groups.setdefault(group, []).append(name)
    sim = F.vibrating_flight(imu_hz=200., seed=41)
    t, a, g = _bin_means(sim['imu_t'], sim['imu_acc'], sim['imu_gyro'], .02)
    params = recommend_params(estimate_imu_noise(t, a, g, [(0.5, 30.)]))
    case("01_online_average", "online", sim, replace(params, imu_average_s=.02))
    case("01_online_average", "preaveraged", sim, params, imu=[ImuSample(float(ti), tuple(ai), tuple(gi)) for ti, ai, gi in zip(t,a,g)])
    sim = F.vibrating_flight(seed=42)
    params = _params(sim, static_check=True)
    case("02_static_start", "blocked", sim, params)
    case("02_static_start", "legacy", sim, replace(params, static_check=False))
    case("02_static_start", "external", sim, replace(params, attitude_init="external"), _attitude(sim, seed=1))
    sim = F.vibrating_flight(seed=43)
    params = _params(sim, static_check=True, imu_gap_reset_s=.2)
    raw, _ = _samples(sim)
    clean, _ = _samples(F.simulate_flight(duration_s=30., imu_hz=50., seed=43))
    quiet_start = [s if s.t < 2 else v for s, v in zip(clean, raw) if not 15 <= s.t < 15.5]
    case("03_imu_gap", "static", sim, params, imu=quiet_start)
    case("03_imu_gap", "external", sim, replace(params, attitude_init="external"), _attitude(sim, seed=2),
         imu=[s for s in raw if not 15 <= s.t < 15.5])
    sim = F.vibrating_flight(yaw_rate=.15, anchor_offsets={a.anchor_id:(14.,15.,2.) for a in F.ROOM_48},
                            dropouts=[(15.,15.3)], seed=21)
    case("04_external_reinit", "external", sim, _params(sim, attitude_init="external", reinit_keep_attitude=False), _attitude(sim, seed=3))
    case("06_carry_attitude", "carry", sim, _params(sim))
    case("06_carry_attitude", "boot", sim, _params(sim, reinit_keep_attitude=False))
    frames = out / "05_frame_conversion"
    frames.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(49)
    with (frames / 'frames.csv').open('w', newline='') as f:
        writer = csv.writer(f, lineterminator='\n')
        for _ in range(100):
            q = rng.normal(size=4);q *= rng.uniform(.51, 1.99)/np.linalg.norm(q)
            offset, yaw0 = rng.uniform(-math.pi, math.pi, 2)
            writer.writerow([*q, offset, yaw0, *px4_attitude_to_rig(q,offset).ravel(), px4_yaw_ned(q), rig_yaw_offset_from_yaw0(q,yaw0)])
    groups['05_frame_conversion'] = ['frames']
    return groups


def test1_case(out, flight):
    from eskf_a_replay import fit_training, _metrics, _sample_track
    from eskf_tuning_report import _attitude_errors, _attitude_stats
    from aided_calibration import correct_ranges
    data = load_flight(flight, [(75.,195.)], (-.09,0.,.31))
    params = replace(EskfParams(), na=.293, ng=.0107, min_anchor_interval_s=.02, reinit_keep_attitude=True,
                     imu_average_s=.02, attitude_init='external', static_check=True, imu_gap_reset_s=.2)
    cal, _ = fit_training(data, [(75.,195.)], 0)
    calibrated = correct_ranges(cal, data.anchors, data.ranges, data.gui_xyz[:,:2], data.yaw_rig)
    valid = np.isfinite(calibrated) & (calibrated>.01) & (data.t[:,None]>=data.imu_t[0]) & (data.t[:,None]<=data.imu_t[-1])
    rows, cols = np.nonzero(valid)
    ranges = [RangeSample(float(data.t[r]), data.anchors[c].anchor_id, float(calibrated[r,c])) for r,c in zip(rows,cols)]
    imu = [ImuSample(float(t),tuple(a),tuple(g)) for t,a,g in zip(data.imu_t,data.imu_acc_flu,data.imu_gyro_flu)]
    bridge, _ = read_mcap_cache(data.root, data.session)
    datasets, _ = read_ulog(data.audit['sources']['ulog'])
    offset = data.audit['yaw_alignment']['rig_yaw_offset_rad']
    att_t = np.arange(data.imu_t[0], data.imu_t[-1], .02)
    attitude = [AttitudeSample(float(t),R) for t,R in zip(att_t,rig_attitude_rotations(att_t,bridge,datasets,offset))]
    yaw0 = float(np.interp(data.imu_t[0],data.t,np.unwrap(data.yaw_rig)))
    core = export_bundle(out/'07_test1'/'onboard',data.anchors,imu,ranges,attitude,(-.09,0.,.31),yaw0,params)
    res = core.result()
    indices = np.unique(np.searchsorted(data.t,np.arange(np.ceil(data.t[0]*10)/10,np.floor(data.t[-1]*10)/10,.1)))
    indices = indices[indices<len(data.t)]
    query = data.t[indices]
    positions = _sample_track(query,res)
    reference_R = rig_attitude_rotations(res.t,bridge,datasets,offset)
    yaw,tilt = _attitude_errors(res,reference_R)
    segments = []
    checks = {}
    for i,(lo,hi) in enumerate(((75.,195.),(198.,218.),(235.,300.))):
        mask = interval_mask(query,[(lo,hi)]) & usable_epochs(data)[indices]
        metrics = _metrics(positions[mask],data.z_ref[indices][mask],data.gps_tag_xy[indices][mask])
        metrics['attitude'] = _attitude_stats(yaw,tilt,interval_mask(res.t,[(lo,hi)]))
        segments.append(dict(interval_s=[lo,hi],**metrics))
        for key,value,limit in [('xy_rmse',metrics['xy']['rmse_m'],(.125,.185,.255)[i]),
                                ('xy_p95',metrics['xy']['p95_m'],(.24,.34,.34)[i]),
                                ('z_median_abs',abs(metrics['z']['median_m']),.03),('z_p95',metrics['z']['p95_abs_m'],.15),
                                ('yaw_p95',metrics['attitude']['yaw_p95_abs_deg'],3.),('tilt_p95',metrics['attitude']['tilt_p95_deg'],2.)]:
            checks[f'{lo:g}:{hi:g}.{key}'] = dict(value=value,limit=limit,passed=value<=limit)
    mask = interval_mask(data.t,[(75.,300.)]) & usable_epochs(data)
    normalized = normalized_errors(res.t,res.tag[:,2],res.p_std[:,2],data.t[mask],data.z_ref[mask])
    for key,value,ok in [('reinits',res.reinits,res.reinits==0),('range_acceptance',res.acceptance_ratio(),res.acceptance_ratio()>=.99),
                         ('attitude_acceptance',float(res.att_accepted.mean()),res.att_accepted.mean()>=.99),
                         ('coverage',normalized['frac_within_2sigma'],normalized['frac_within_2sigma']>=.9),
                         ('rms_normalized',normalized['rms_normalized'],.6<=normalized['rms_normalized']<=1.4)]:
        checks[key] = dict(value=value,passed=bool(ok))
    report = dict(summary=summary(core),params=asdict(params),segments=segments,normalized_z=normalized,
                  calibration=cal.to_dict(),input_audit=data.audit,checks=checks,all_passed=all(c['passed'] for c in checks.values()),
                  selection='Noise fixed to specified mean50 values; aided K=0 and alignment trained only on 75:195; no holdout tuning',
                  limitations=['RF is the calibration source, not independent ground truth','GNSS aligned on training windows, not surveyed truth',
                               'Body position sigma omits lever-arm attitude covariance','PX4 attitude/IMU are correlated'])
    write_json(out/'07_test1'/'test1_report.json',report)
    print('Test_1 acceptance:',report['all_passed'],flush=True)
    return ['onboard']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--flight', help='Include Test_1 as the seventh scenario group')
    parser.add_argument('--golden', action='store_true', help='Export a small deterministic fixture for colcon/gtest')
    args = parser.parse_args(argv)
    try:
        out = args.out.resolve()
        if args.flight and out.is_relative_to(resolve_input(args.flight)):
            raise ValueError('Parity output must not be inside original flight Logs')
        if args.golden:
            sys.path.insert(0,str(GUI_DIR/'tests'))
            import e21_fixtures as F
            from test_e22_onboard_core import _attitude
            sim = F.simulate_flight(duration_s=2.,imu_hz=200.,seed=81)
            imu,ranges = _samples(sim)
            export_bundle(out,F.ROOM_48,imu,ranges,_attitude(sim,seed=81),sim['lever'],sim['yaw0'],
                          replace(EskfParams(),imu_average_s=.02,attitude_init='external',min_anchor_interval_s=.02))
        else:
            groups = synthetic_cases(out)
            if args.flight:groups['07_test1'] = test1_case(out,args.flight)
            write_json(out/'suite.json',groups)
        return 0
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print(f'export_core_events: {exc}',file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
