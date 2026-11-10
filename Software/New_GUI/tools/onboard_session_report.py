"""Read-only E2.3 onboard session inventory, replay and shadow-flight evaluation.

Outputs are confined to analysis/e23_* under the flight (or synthetic session).
Replay does not model live input arrival, wall-clock freshness or timeouts.
RF/GNSS are sensor references, not surveyed ground truth. No holdout tuning.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import csv
import json
import math
import os
from pathlib import Path
import sys
from time import perf_counter
from types import SimpleNamespace

import numpy as np

from flight_log_io import (dependency, interval_mask, load_flight, output_directory,
                           parse_intervals, parse_lever, resolve_input, usable_epochs,
                           validate_intervals, vector_arguments, write_json, interp_vectors)
from onboard_session_io import read_session, read_tlog
from onboard_replay import (compare_tracks, node_config, node_events, parse_rig,
                            replay_node, stream_stats)
from eskf_config import PARAMETERS, aided_bias_rows
from eskf_replay import px4_yaw_ned
from eskf_tuning import estimate_imu_noise, normalized_errors
from eskf_a_replay import _metrics, _sample_track
from eskf_tuning_report import _attitude_errors, _attitude_stats


def replay_session(session, params, rig_text):
    config = node_config(params, rig_text)
    events = node_events(config, session.imu_ns, session.acc, session.gyro,
                         ranges=session.ranges, att_ns=session.att_ns, att_q=session.att_q)
    start = perf_counter()
    result = replay_node(config, events)
    origin_boot = (events.origin_ns - int(session.clock['offset_ns'])) * 1e-9
    result.t += origin_boot
    result.update_t += origin_boot
    result.att_update_t += origin_boot
    diagnostic = dict(outputs=len(result.t), dropped=events.dropped, input_ranges=len(events.ranges),
                      origin_ns=events.origin_ns, origin_boot_s=origin_boot,
                      rig_yaw_offset_rad=events.rig_yaw_offset_rad,
                      init_boot_s=None if result.init_time is None else result.init_time + origin_boot,
                      reinits=result.reinits, imu_gap_resets=result.imu_gap_resets,
                      range_acceptance=result.acceptance_ratio(),
                      attitude_acceptance=float(np.mean(result.att_accepted)) if len(result.att_accepted) else 0.,
                      skipped_ranges=result.skipped_ranges, elapsed_s=perf_counter()-start,
                      node_parameters=params, mapped_core_parameters=asdict(config.params))
    return result, diagnostic, events


def status_timeline(session):
    rows = session.statuses.get('/uwb_eskf/status', [])
    counts = Counter(str(value.get('state', 'UNKNOWN')) for _, value in rows)
    durations = Counter()
    transitions = []
    previous = None
    end = max((s['last_log_ns'] or 0 for s in session.streams.values()), default=0)
    for i, (log, value) in enumerate(rows):
        state = str(value.get('state', 'UNKNOWN'))
        durations[state] += max(0, (rows[i+1][0] if i+1<len(rows) else end)-log)*1e-9
        signature = (state, value.get('reason'), value.get('reinits'), value.get('imu_gap_resets'))
        if signature != previous:
            transitions.append(dict(log_time_ns=log, boot_s=(log-int(session.clock['offset_ns']))*1e-9,
                                    state=state, status=value))
        previous = signature
    total = sum(counts.values())
    return dict(samples=total, counts=dict(counts), fraction={s:n/total for s,n in counts.items()},
                duration_s=dict(durations), transitions=transitions,
                duration_rule='Left-held status until next sample or final MCAP message; no assertion before first status')


def static_window(session, flight, requested):
    if requested:
        return requested, 'Explicit --static-window'
    if flight:
        # vehicle_status.armed_time preserves the ARM event even if the ULog starts at ARM.
        ulog = dependency('pyulog').ULog(str(flight.audit['sources']['ulog']), ['vehicle_status'])
        values = ulog.get_dataset('vehicle_status').data
        armed = np.asarray(values.get('armed_time', []), float)*1e-6
        armed = armed[np.isfinite(armed) & (armed>0)]
        if len(armed):
            start = math.ceil(session.boot_seconds(session.imu_ns)[0])
            stop = math.floor(float(armed.min()))
            if stop>start:
                return [(start, stop)], 'ULog vehicle_status.armed_time; full seconds recorded before ARM'
    return [], 'No reliable pre-ARM interval; specify --static-window (no noise recommendation for static)'


def noise_report(session, config, train, windows, source):
    t = session.boot_seconds(session.imu_ns, session.imu_log_ns)
    rotation = np.asarray(config.imu_R).reshape(3, 3)
    acc, gyro = session.acc@rotation.T, session.gyro@rotation.T
    def estimate(intervals):
        if not intervals:
            return None
        noise = estimate_imu_noise(t, acc, gyro, intervals)
        return dict(intervals_s=intervals, **noise.to_dict(),
                    suggested_na=noise.acc_density, suggested_ng=noise.gyro_density)
    return dict(frame='Body FLU; imu_R applied once to recorded samples',
                method='eskf_tuning detrend 0.2 s; worst-axis residual std / sqrt(measured Hz); no resampling',
                static_source=source, static=estimate(windows), train=estimate(train),
                warning='Recommendations describe this recorded stream only; they are not applied to replay or holdout')


def evaluate(result, data, train, tests, yaw_offset):
    grid = np.arange(np.ceil(data.t[0]*10)/10, np.floor(data.t[-1]*10)/10, .1)
    indices = np.unique(np.searchsorted(data.t, grid))
    indices = indices[indices<len(data.t)]
    t = data.t[indices]
    eligible = usable_epochs(data)[indices] & (data.rangefinder_age_s[indices]<=.5)
    positions = _sample_track(t, result)
    segments = []
    for role, intervals in (('train', train), ('test', tests)):
        for lo, hi in intervals:
            mask = (t>=lo) & (t<hi) & eligible
            segments.append(dict(role=role, interval_s=[lo, hi],
                                 **_metrics(positions[mask], data.z_ref[indices][mask], data.gps_tag_xy[indices][mask])))
    # Same convention as E2.1: body-position sigma with TAG height error, stated explicitly.
    normalized_mask = usable_epochs(data) & (data.rangefinder_age_s<=.5) & interval_mask(data.t, [(75.,300.)])
    consistency = normalized_errors(result.t, result.tag[:,2], result.p_std[:,2],
                                    data.t[normalized_mask], data.z_ref[normalized_mask])
    from flight_log_io import read_mcap_cache, read_ulog, rig_attitude_rotations
    bridge, _ = read_mcap_cache(data.root, data.session)
    datasets, _ = read_ulog(data.audit['sources']['ulog'])
    reference = rig_attitude_rotations(result.t, bridge, datasets, yaw_offset)
    yaw, tilt = _attitude_errors(result, reference)
    selected = interval_mask(result.t, train+tests) & np.isfinite(yaw) & np.isfinite(tilt)
    return dict(segments=segments, normalized_z_75_300=consistency,
                attitude=_attitude_stats(yaw, tilt, selected),
                scoring='Actual CSV epochs near 10 Hz; RF age <=0.5 s; interpolation gaps <=0.1 s; TAG coordinates',
                sigma_convention='Body position covariance diagonal; attitude/lever uncertainty not added to TAG sigma')


def summary_error(values, reference):
    good = np.isfinite(values) & np.isfinite(reference)
    a,b = np.asarray(values)[good],np.asarray(reference)[good]
    return dict(samples=len(a), median_error=float(np.median(a-b)) if len(a) else None,
                rmse=float(np.sqrt(np.mean((a-b)**2))) if len(a) else None,
                correlation=float(np.corrcoef(a,b)[0,1]) if len(a)>2 and min(a.std(),b.std())>0 else None)


def flow_velocity(fields):
    """MAVLink OPTICAL_FLOW_RAD (body FRD), downward sensor, flat ground.

    MAVLink RH sensor-rotation convention: +body X -> +integrated_y;
    +body Y -> -integrated_x. PX4's internal EKF flow has both signs reversed
    on ingress, unlike this MAVLink stream. No sign fitted from the data.
    Returns average body-FLU XY velocity; requires gyro, distance and duration.
    """
    keys=('integrated_x','integrated_y','integrated_xgyro','integrated_ygyro','distance','integration_time_us')
    try:
        x,y,gx,gy,d,us=(float(fields[k]) for k in keys)
    except (KeyError,TypeError,ValueError):
        return None
    if not np.isfinite([x,y,gx,gy,d,us]).all() or d<=0 or us<=0 or fields.get('quality',0)<=0:
        return None
    return np.array([(y-gy)*d/(us*1e-6), (x-gx)*d/(us*1e-6), 0.])


def tlog_report(raw, data, results, arrays):
    messages = raw['selected']['messages']
    report = {k:v for k,v in raw.items() if k not in ('selected','boots')}
    report['boots'] = [{k:v for k,v in b.items() if k!='messages'} for b in raw['boots']]
    report['streams'] = {}
    if data:
        from flight_log_io import read_ulog
        _, parameters=read_ulog(data.audit['sources']['ulog'])
        report['recorded_flow_parameters']={k:v for k,v in parameters.items() if k.startswith(('SENS_FLOW','EKF2_OF'))}
    for kind, rows in messages.items():
        t = np.array([r['boot_s'] for r in rows])
        selected = (t>=75) & (t<300) if data else np.ones(len(t),bool)
        stats = stream_stats(np.rint(t*1e9).astype(np.int64))
        fields = [r['fields'] for r,yes in zip(rows,selected) if yes]
        entry = dict(**stats, evaluation_interval_s=[75.,300.] if data else None)
        if kind=='OPTICAL_FLOW_RAD':
            quality = np.array([f.get('quality',np.nan) for f in fields],float)
            dt = np.array([f.get('integration_time_us',np.nan) for f in fields],float)*1e-6
            finite=quality[np.isfinite(quality)]
            entry.update(quality_p10_p50_p90=np.percentile(finite,[10,50,90]) if len(finite) else [],
                         integration_s_median=float(np.nanmedian(dt)) if len(dt) else None,
                         integration_duty_fraction=float(np.nanmedian(dt)*stats['rate_hz']) if len(dt) else None)
        if kind=='DISTANCE_SENSOR':
            entry['missing_signal_quality'] = sum(f.get('signal_quality') is None for f in fields)
            if data:
                chosen=[r for r,yes in zip(rows,selected) if yes and r['fields'].get('orientation')==25
                        and r['fields'].get('min_distance',0)<r['fields'].get('current_distance',0)<r['fields'].get('max_distance',math.inf)]
                qt=np.array([r['boot_s'] for r in chosen])
                dist=np.array([r['fields']['current_distance']*.01 for r in chosen])
                ref=interp_vectors(qt,data.t,data.z_ref[:,None])[:,0]
                age=interp_vectors(qt,data.t,data.rangefinder_age_s[:,None])[:,0]
                ref[age>.5]=np.nan
                entry['raw_down_distance_vs_tag_z_ref'] = summary_error(dist,ref)
                entry['comparison_caveat'] = 'Raw slant distance at sensor vs TAG z_ref: lever, tilt and sensor origin differ; shared RF is not independent truth'
                arrays.update(distance_t=qt,distance_m=dist,distance_z_ref=ref)
        report['streams'][kind]=entry
    flow=messages.get('OPTICAL_FLOW_RAD',[])
    ft, fv=[],[]
    for row in flow:
        velocity=flow_velocity(row['fields'])
        if velocity is not None:
            # Average integration interval represented by its midpoint.
            ft.append(row['boot_s']-row['fields']['integration_time_us']*.5e-6);fv.append(velocity)
    ft, fv=np.array(ft), np.asarray(fv).reshape(-1,3)
    arrays['flow_t']=ft;arrays['flow_body_flu_velocity']=fv
    comparisons={}
    for name,result in results.items():
        if len(result.t)<2 or not len(ft):continue
        rotations=interp_vectors(ft,result.t,result.R.reshape(-1,9)).reshape(-1,3,3)
        # Interpolated rotation entries suffice for this diagnostic projection only.
        velocity=np.einsum('nij,nj->ni',rotations,fv)
        reference=interp_vectors(ft,result.t,result.v)
        right=np.clip(np.searchsorted(result.t,ft),1,len(result.t)-1)
        valid=(result.t[right]-result.t[right-1]<=.1)
        if data:valid &= (ft>=75)&(ft<300)
        velocity[~valid]=np.nan
        comparisons[name]={axis:summary_error(velocity[:,k],reference[:,k]) for k,axis in enumerate(('rig_x','rig_y'))}
        arrays[name+'__flow_rig_velocity']=velocity
        arrays[name+'__flow_eskf_velocity']=reference
    report['flow_velocity'] = dict(comparisons=comparisons,
        convention='MAVLink RH integrated flow minus integrated gyro; body FLU [+(y-gy), +(x-gx),0]*distance/dt; rig rotation from replay',
        assumptions='PX4 body-aligned downward sensor, flat ground; no vertical velocity, sensor lever or gyro-bias correction; no fitting or fusion',
        sources=['https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD',
                 'https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/streams/OPTICAL_FLOW_RAD.hpp',
                 'https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF2.cpp'],
        mounting='PX4 vehicle_optical_flow already applies SENS_FLOW_ROT; do not apply that mounting rotation again',
        conclusion='Feasibility only: sparse integration duty, moderate quality, mounting/clock/gyro and distance calibration require validation before fusion')
    return report


def acceptance_checks(report):
    """Frozen Test_1 gates; never used for parameter selection."""
    if report['train']!=[(75.,195.)] or report['test']!=[(198.,218.),(235.,300.)] or 'what_if_aided' not in report['replays']:
        return dict(applicable=False)
    checks={}
    def add(name,value,requirement,ok):checks[name]=dict(value=value,requirement=requirement,passed=bool(ok))
    imu=report['streams']['/px4/imu_frd']
    add('imu_count',imu['samples'],'==14890',imu['samples']==14890)
    add('imu_dt_ms',imu['dt_ms_p50'],'19.55 +/-0.2',abs(imu['dt_ms_p50']-19.55)<=.2)
    add('imu_gaps',imu['gaps'],'==0',imu['gaps']==0)
    ranges=[v['samples'] for k,v in report['streams'].items() if k.startswith('/uwb/anchor_') and k.endswith('/range')]
    add('anchors',ranges,'8 streams; each 12522..12936',len(ranges)==8 and all(12522<=n<=12936 for n in ranges))
    odom=report['streams'].get('/px4/odometry',{}).get('samples',0)
    add('px4_odom',odom,'==14887',odom==14887)
    spread=report['clock']['spread_ns']
    add('clock_spread_ns',spread,'<1000000',spread is not None and spread<1e6)
    fraction=report['status']['fraction'].get('WAIT_CONFIGURATION',0)
    add('status_wait',fraction,'==1',fraction==1)
    recorded=report['replays']['as_recorded']
    add('recorded_outputs',recorded['outputs'],'==0',recorded['outputs']==0)
    rejected=recorded['dropped']['ranges_unconfirmed']
    add('unconfirmed_ranges',rejected,'== all recorded ranges',rejected==sum(ranges))
    add('actual_node_outputs',report['recorded_odom_samples'],'==0',report['recorded_odom_samples']==0)
    aided=report['replays']['what_if_aided']
    add('init_boot_s',aided['init_boot_s'],'<=47',aided['init_boot_s'] is not None and aided['init_boot_s']<=47)
    add('reinits',aided['reinits'],'==0',aided['reinits']==0)
    for key in ('range_acceptance','attitude_acceptance'):
        add(key,aided[key],'>=0.99',aided[key]>=.99)
    for i,segment in enumerate(aided['evaluation']['segments']):
        for label,value,limit in [('xy_rmse',segment['xy']['rmse_m'],(.125,.18,.25)[i]),
                                  ('xy_p95',segment['xy']['p95_m'],(.24,.32,.33)[i]),
                                  ('z_median',segment['z']['median_m'],.03),('z_p95',segment['z']['p95_abs_m'],.15)]:
            add(f'{i}.{label}',value,f'absolute <= {limit} m',value is not None and abs(value)<=limit)
    consistency=aided['evaluation']['normalized_z_75_300']
    add('z_2sigma',consistency['frac_within_2sigma'],'>=0.90',consistency['frac_within_2sigma']>=.9)
    add('z_rms_normalized',consistency['rms_normalized'],'0.6..1.4',.6<=consistency['rms_normalized']<=1.4)
    yaw=aided['evaluation']['attitude']['yaw_p95_abs_deg']
    add('yaw_p95_deg',yaw,'<=3',yaw is not None and yaw<=3)
    for label,na,ng,tolerance in [('static',.036,.0017,.2),('train',.948,.0317,.15)]:
        noise=report['noise'].get(label) or {}
        for key,expected in [('acc_density',na),('gyro_density',ng)]:
            value=noise.get(key)
            add(label+'.'+key,value,f'{expected} +/- {tolerance*100:g}%',value is not None and abs(value-expected)<=expected*tolerance)
    tlog=report.get('tlog')
    if tlog:
        add('tlog_boot_count',len(tlog['boots']),'==2',len(tlog['boots'])==2)
        add('tlog_boot_selection',tlog['selected_boot'],'==2',tlog['selected_boot']==2)
        for kind in ('OPTICAL_FLOW_RAD','DISTANCE_SENSOR'):
            hz=tlog['streams'].get(kind,{}).get('rate_hz',0)
            add(kind+'.rate_hz',hz,'approximately 10 (9..11)',9<=hz<=11)
        quality=tlog['streams'].get('OPTICAL_FLOW_RAD',{}).get('quality_p10_p50_p90',[])
        median=quality[1] if len(quality)==3 else None
        add('flow_quality',median,'70 +/-3',median is not None and abs(median-70)<=3)
    else:add('tlog',None,'required',False)
    return dict(applicable=True,all_passed=all(v['passed'] for v in checks.values()),checks=checks)


def plots(out,results,data,arrays):
    os.environ.setdefault('MPLCONFIGDIR',str(out/'.matplotlib'))
    plt=dependency('matplotlib.pyplot','matplotlib');plt.switch_backend('Agg')
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True,constrained_layout=True)
    for name,r in results.items():
        for k,ax in enumerate(axes):ax.plot(r.t,r.tag[:,k],lw=.7,label=name)
    if data:
        for k in range(2):axes[k].plot(data.t,data.gps_tag_xy[:,k],'k',lw=.8,label='aligned GNSS')
        axes[2].plot(data.t,data.z_ref,'k',lw=.8,label='fresh RF TAG reference')
    for axis,label in zip(axes,'XYZ'):
        axis.set_ylabel('TAG '+label+' (m)');axis.grid(alpha=.2);axis.legend(loc='upper left')
    axes[-1].set_xlabel('PX4 boot time (s)' if data else 'Device elapsed time (s)')
    fig.savefig(out/'trajectories.png',dpi=130);plt.close(fig)
    if 'flow_t' in arrays:
        fig,axes=plt.subplots(2,1,figsize=(12,6),sharex=True,constrained_layout=True)
        name='what_if_aided' if 'what_if_aided' in results else 'as_recorded'
        if name+'__flow_rig_velocity' in arrays:
            for k,ax in enumerate(axes):
                ax.plot(arrays['flow_t'],arrays[name+'__flow_rig_velocity'][:,k],lw=.6,label='flow, feasibility only')
                ax.plot(arrays['flow_t'],arrays[name+'__flow_eskf_velocity'][:,k],lw=.6,label='ESKF')
                ax.set_ylabel(('rig X','rig Y')[k]+' m/s');ax.legend();ax.grid(alpha=.2)
        fig.savefig(out/'flow_velocity.png',dpi=130);plt.close(fig)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session');parser.add_argument('--flight');parser.add_argument('--tlog')
    parser.add_argument('--boot',type=int);parser.add_argument('--out')
    parser.add_argument('--train',type=parse_intervals,default=[]);parser.add_argument('--test',type=parse_intervals,default=[])
    parser.add_argument('--static-window',type=parse_intervals)
    parser.add_argument('--what-if-confirm',action='store_true');parser.add_argument('--lever',type=parse_lever)
    yaw=parser.add_mutually_exclusive_group();yaw.add_argument('--yaw0-deg',type=float);yaw.add_argument('--yaw0-from-flight',action='store_true')
    parser.add_argument('--aided-bias-json')
    args=parser.parse_args(vector_arguments(argv))
    try:
        validate_intervals(args.train,args.test)
        if args.what_if_confirm and (args.lever is None or (args.yaw0_deg is None and not args.yaw0_from_flight)):
            raise ValueError('What-if requires an explicit --lever and --yaw0-deg or --yaw0-from-flight')
        if args.yaw0_from_flight and (not args.flight or not args.train):
            raise ValueError('--yaw0-from-flight requires --flight and training intervals')
        if args.aided_bias_json and not args.what_if_confirm:
            raise ValueError('--aided-bias-json requires --what-if-confirm')
        session=read_session(args.session)
        data=load_flight(args.flight,args.train,args.lever or (-.09,0.,.31)) if args.flight else None
        if data and (not session.clock['available'] or session.clock['resets']):
            raise ValueError('Flight scoring requires one stable IMU/device clock epoch; split reset sessions first')
        owner=data or SimpleNamespace(root=session.root)
        # Restrict this new tool further than the shared E20/E21 allowlist.
        requested=args.out or str(owner.root/'analysis/e23_onboard_session')
        candidate=Path(requested)
        if not candidate.is_absolute():
            from flight_log_io import REPO_DIR
            candidate=(REPO_DIR/candidate) if candidate.parts[0].lower()=='logs' else Path.cwd()/candidate
        relative=candidate.resolve().relative_to((owner.root/'analysis').resolve())
        if not relative.parts or not relative.parts[0].startswith('e23_'):
            raise ValueError('This tool writes only analysis/e23_*')
        out=output_directory(owner,requested,'e23_onboard_session')
        report=dict(tool='onboard_session_report',version='e23-v1',session=str(session.root),
                    train=args.train,test=args.test,session_metadata=session.metadata,
                    recorded_parameters=session.ros_params,streams=session.streams,clock=session.clock,
                    audit=session.audit,status=status_timeline(session),attitude_source=session.attitude_source,
                    recorded_odom_samples=len(session.odom_ns),replays={},
                    limitations=session.audit['limitations']+['RF/GNSS are aligned sensor references, not surveyed truth',
                    'What-if replay cannot reconstruct live freshness, timeouts, message delivery order or publication timing'],
                    parameter_selection='GUI E2.2 defaults fixed; bias loaded from training-only E2.0 artifact; no holdout tuning')
        if data:report['flight_reference_audit']=data.audit
        variants={'as_recorded':(dict(session.ros_params),session.rig_text)}
        if args.what_if_confirm:
            params={**session.ros_params,**{k:v[0] for k,v in PARAMETERS.items()},
                    'configuration_confirmed':True,'lever':list(args.lever),'ignore_rig_bias':True}
            if args.yaw0_from_flight:
                first_valid=None
                for i,q in enumerate(session.att_q):
                    try:px4_yaw_ned(q)
                    except ValueError:continue
                    first_valid=i;break
                if first_valid is None:raise ValueError('No valid recorded attitude for --yaw0-from-flight')
                t0=session.boot_seconds(session.att_ns,session.att_log_ns)[first_valid]
                finite=np.isfinite(data.yaw_rig)&np.isfinite(data.t)
                params['yaw0_deg']=float(np.degrees(np.interp(t0,data.t[finite],np.unwrap(data.yaw_rig[finite]))))
                report['yaw0_source']=dict(method='Training-only GNSS-aligned rig yaw at first recorded attitude',boot_s=t0,fit_intervals_s=args.train)
            else:
                if not math.isfinite(args.yaw0_deg):raise ValueError('yaw0 must be finite')
                params['yaw0_deg']=args.yaw0_deg
            variants['what_if_no_bias']=(params,session.rig_text)
            if args.aided_bias_json:
                path=resolve_input(args.aided_bias_json)
                document=json.loads(path.read_text(encoding='utf-8-sig'))
                provenance=document.get('run',{})
                if provenance and (provenance.get('training_intervals_s')!=[list(w) for w in args.train]
                                   or provenance.get('lever_flu_m')!=list(args.lever)):
                    raise ValueError('Aided calibration provenance must match training intervals and lever')
                biases=aided_bias_rows(document,set(parse_rig(session.rig_text)[0]))
                text=session.rig_text+'\n# E2.0 aided extra bias, measured=true+bias\n'+''.join(f'an{aid}_bias, {bias}\n' for aid,bias in biases)
                variants['what_if_aided']=({**params,'ignore_rig_bias':False},text)
                report['aided_source']=dict(path=str(path),biases_m=biases,run=document.get('run'))
        results,arrays={},{}
        for name,(params,rig) in variants.items():
            print('Replay '+name+'...',flush=True)
            result,diagnostic,events=replay_session(session,params,rig)
            results[name]=result;report['replays'][name]=diagnostic
            for field in ('t','p','v','R','tag','p_std','update_t','update_anchor','update_accepted','update_nis','att_update_t','att_accepted'):
                arrays[name+'__'+field]=getattr(result,field)
            if data and len(result.t)>1:
                diagnostic['evaluation']=evaluate(result,data,args.train,args.test,events.rig_yaw_offset_rad)
            if name=='as_recorded':
                odom_time=(session.odom_ns-events.origin_ns)*1e-9+diagnostic['origin_boot_s']
                diagnostic['recorded_node_comparison']=compare_tracks(odom_time,session.odom_p,result.t,result.p)
                if len(session.odom_ns):
                    c=diagnostic['recorded_node_comparison']
                    diagnostic['recorded_node_comparison_acceptance']=dict(
                        passed=c['n']>0 and c['median_m']<=.01 and c['p95_m']<=.03,
                        requirement='Median <=0.01 m, p95 <=0.03 m; arrival/freshness/publication differences remain possible')
                diagnostic['comparison_position']='Body/IMU, not TAG; int ns offset removed first; gaps >0.1 s excluded'
        arrays.update(recorded_odom_t=session.boot_seconds(session.odom_ns,session.odom_log_ns),recorded_odom_p=session.odom_p)
        windows,source=static_window(session,data,args.static_window)
        report['noise']=noise_report(session,node_config(session.ros_params,session.rig_text),args.train,windows,source)
        if args.tlog:
            span=(min(v['first_log_ns'] for v in session.streams.values() if v['first_log_ns'] is not None),
                  max(v['last_log_ns'] for v in session.streams.values() if v['last_log_ns'] is not None))
            report['tlog']=tlog_report(read_tlog(args.tlog,span,args.boot),data,results,arrays)
        report['acceptance']=acceptance_checks(report)
        if 'what_if_no_bias' in report['replays'] and data:
            med=[m['z']['median_m'] for m in report['replays']['what_if_no_bias']['evaluation']['segments']]
            report['finding_without_aided_bias']=dict(z_median_m=med,conclusion='Report residual TAG height bias explicitly; supported bias is opt-in, never auto-enabled')
        write_json(out/'report.json',report)
        np.savez_compressed(out/'trajectories.npz',**arrays)
        with (out/'streams.csv').open('w',encoding='utf-8',newline='') as f:
            fields=['topic','samples','duration_s','rate_hz','dt_ms_p50','dt_ms_p95','dt_ms_max','gaps','gap_time_s','non_increasing','clock']
            writer=csv.DictWriter(f,fields,extrasaction='ignore');writer.writeheader()
            writer.writerows(dict(topic=k,**v) for k,v in session.streams.items())
        lines=['# E2.3 onboard session report','',f'Session: `{session.root}`','',
               'Replay excludes live wall-clock freshness, arrival timeouts and message delivery order. RF/GNSS are sensor references, not surveyed truth.','',
               f"Recorded outputs: {len(session.odom_ns)}. States: {report['status']['counts']}.",'',
               '| Replay | Window (s) | XY RMSE cm | XY p95 cm | Z median cm | Z p95 cm |','|---|---|---:|---:|---:|---:|']
        for name,diagnostic in report['replays'].items():
            for m in diagnostic.get('evaluation',{}).get('segments',[]):
                values=[m['xy']['rmse_m'],m['xy']['p95_m'],m['z']['median_m'],m['z']['p95_abs_m']]
                cells=['—' if v is None else f'{v*100:.2f}' for v in values]
                lines.append('| '+name+' | '+str(m['interval_s'])+' | '+' | '.join(cells)+' |')
        lines+=['','## Noise of the recorded IMU stream','', '| Window | Rate Hz | na | ng |','|---|---:|---:|---:|']
        for label in ('static','train'):
            noise=report['noise'][label]
            if noise:lines.append(f"| {label} {noise['intervals_s']} | {noise['stream_hz']:.2f} | {noise['acc_density']:.6f} | {noise['gyro_density']:.6f} |")
        lines+=['',report['noise']['static_source'], '', '## Node vs replay','',
                str(report['replays']['as_recorded']['recorded_node_comparison']), '', '## Acceptance','',
                '| Check | Value | Requirement | Result |','|---|---|---|---|']
        for name,c in report['acceptance'].get('checks',{}).items():
            value=c['value']
            lines.append(f"| {name} | {value:.6g} | {c['requirement']} | {'PASS' if c['passed'] else 'FAIL'} |" if isinstance(value,float) else
                         f"| {name} | {value} | {c['requirement']} | {'PASS' if c['passed'] else 'FAIL'} |")
        if report.get('tlog'):
            lines+=['','## Optical flow feasibility','',f"Selected PX4 boot: {report['tlog']['selected_boot']} of {len(report['tlog']['boots'])}.",
                    '',report['tlog']['flow_velocity']['conclusion'], '',report['tlog']['flow_velocity']['assumptions']]
        lines+=['', 'Complete configuration, status timeline, stream/noise/flow diagnostics and assumptions: `report.json`.']
        (out/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
        plots(out,results,data,arrays)
        print(json.dumps({'out':str(out),'acceptance':report['acceptance']},default=str),flush=True)
        comparison=report['replays']['as_recorded'].get('recorded_node_comparison_acceptance',{})
        return 0 if report['acceptance'].get('all_passed',True) and comparison.get('passed',True) else 1
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('onboard_session_report: '+str(exc),file=sys.stderr);return 2


if __name__=='__main__':sys.exit(main())
