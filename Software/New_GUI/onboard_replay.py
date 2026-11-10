"""E2.3-A — replay a recorded onboard uwb_eskf session the way the E2.2 node consumes it.

Plan: Plan/E2_3_YEU_CAU_CONG_CU_PHIEN_ONBOARD_VA_BAY_SHADOW.md (section 3).
Contract: tests/test_e23_onboard_replay.py. Do not loosen the tests.

Purpose: turn a session (eskf.yaml ros__parameters, rig_onboard.txt, recorded IMU/attitude/range
messages with int64 ns header stamps) into exactly the inputs the node hands to its core, then
replay them with eskf_replay.run_eskf_replay. The live node additionally checks wall-clock freshness
and timeouts; those cannot be reproduced offline and are NOT modelled here (document it in reports).
Pure numpy + stdlib; no MCAP/pyulog/ROS here (tools/onboard_session_io.py does the IO).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import re
from typing import Any, Mapping

import numpy as np  # noqa: F401

from eskf_replay import (AttitudeSample, EskfParams, ImuSample, RangeSample,
                         px4_attitude_to_rig, rig_yaw_offset_from_yaw0, run_eskf_replay)

ONBOARD_REPLAY_VERSION = "onboard-replay-v1"

# Defaults declared by UP7000/estimator/uwb_eskf/src/uwb_eskf_node.cpp (E2.2). Keep in sync with the node.
NODE_DEFAULTS: dict[str, Any] = {
    "configuration_confirmed": False, "ignore_rig_bias": False, "imu": "none", "yaw0_deg": 0.0,
    "sig_r": .15, "q_cv": 1., "na": .45, "ng": .016, "nba": .003, "nbg": .0001, "gate": 25., "k_pos": 1.5,
    "k_neg": 3., "init_window_s": .05, "init_static_s": 1., "imu_timeout_s": .2, "range_timeout_s": .5,
    "max_input_age_s": .25, "min_anchor_interval_s": .02, "imu_average_s": .02, "reinit_keep_attitude": True,
    "reinit_after_rejects": 200, "static_acc_tol": .8, "static_gyro_tol": .15, "use_px4_attitude": 1,
    "att_sig_tilt_deg": 2., "att_sig_yaw_deg": 5., "att_gate": 25., "att_min_interval_s": .02,
    "config_sha256": "",
}
MAVLINK_IMU_R = (1., 0., 0., 0., -1., 0., 0., 0., -1.)      # node default imu_R for imu=mavlink (FRD -> FLU)


@dataclass(frozen=True)
class RigAnchor:
    anchor_id: int
    x: float
    y: float
    z: float
    enabled: bool = True


@dataclass(frozen=True)
class NodeConfig:
    confirmed: bool
    imu_mode: str                          # "mavlink" | "px4" | "d435i" | "none"
    anchors: tuple[RigAnchor, ...]         # sorted by anchor_id
    bias_m: Mapping[int, float]            # from "anN_bias, b" lines (measured = true + b)
    ignore_rig_bias: bool
    lever_flu: tuple[float, float, float]
    yaw0_rad: float
    imu_R: tuple[float, ...]               # 9 values, row-major, applied to raw acc/gyro
    use_px4_attitude: bool
    params: EskfParams                     # mapped exactly like the node (see node_config)
    imu_timeout_s: float
    range_timeout_s: float
    max_input_age_s: float
    config_sha256: str = ""


@dataclass
class NodeEvents:
    imu: list                              # eskf_replay.ImuSample, seconds since origin_ns, FLU
    ranges: list                           # eskf_replay.RangeSample after node guards and bias handling
    attitude: list                         # eskf_replay.AttitudeSample (body FLU -> rig)
    origin_ns: int                         # int64 time origin subtracted before converting to seconds
    rig_yaw_offset_rad: float | None       # derived from yaw0 at the first valid attitude sample
    dropped: dict = field(default_factory=dict)   # imu, ranges_unconfirmed, ranges_invalid, ranges_unknown_anchor, attitude


def parse_rig(text: str):
    """Same rules as filter.hpp load_rig/load_bias. Only lines whose stripped text starts with "an";
    commas are separators; anchor id = digits between "an" and the first "_".
    "anN_p, x, y, z" (>= 3 numbers) -> anchors[N] = (x, y, z); "anN_bias, b" -> bias[N] = b;
    anything else (comments, short lines) is ignored. Returns (anchors dict, bias dict)."""
    anchors, biases = {}, {}
    for line in text.splitlines():
        fields = line.strip().replace(',', ' ').split()
        if not fields:
            continue
        match = re.fullmatch(r'an(\d+)_(p|bias)', fields[0])
        if not match:
            continue
        aid, kind = int(match[1]), match[2]
        count = 3 if kind == 'p' else 1
        if len(fields) <= count:
            continue
        try:
            values = tuple(float(x) for x in fields[1:count+1])
        except ValueError:
            continue
        if not all(math.isfinite(v) for v in values):
            continue
        if kind == 'p':
            anchors[aid] = values
        else:
            biases[aid] = values[0]
    return anchors, biases


def node_config(ros_params: Mapping[str, Any], rig_text: str) -> NodeConfig:
    """Mirror of the node's parameter handling (E2.2 uwb_eskf_node.cpp).

    Missing keys take NODE_DEFAULTS; min_init_anchors defaults to max(4, min(6, n_anchors)).
    EskfParams: sig_r, gate, k_pos, k_neg, na, ng, nba, nbg, init_static_s, init_window_s,
    init_min_anchors, reinit_after_rejects, min_anchor_interval_s, reinit_keep_attitude,
    att_sig_deg = (tilt, tilt, yaw), att_gate, att_min_interval_s, imu_average_s,
    attitude_init = "external" if use_px4_attitude == 1 else "static", static_check = True,
    static_acc_tol, static_gyro_tol, imu_gap_reset_s = imu_timeout_s; other fields keep EskfParams defaults.
    ValueError: use_px4_attitude not 0/1; lever missing/empty or not 3 values (the URDF fallback is not
    replayed offline); imu_R not 9 values; confirmed with < 4 anchors.
    """
    if not isinstance(ros_params, Mapping):
        raise ValueError('Expected ROS parameter mapping')
    p = {**NODE_DEFAULTS, **ros_params}
    xyz, bias = parse_rig(rig_text)
    anchors = tuple(RigAnchor(aid, *point) for aid, point in sorted(xyz.items()))
    lever = p.get('lever', ())
    if not isinstance(lever, (list, tuple, np.ndarray)) or np.shape(lever) != (3,) or not np.isfinite(lever).all():
        raise ValueError('Offline replay needs an explicit finite 3-value lever; URDF fallback is not replayed')
    if type(p['use_px4_attitude']) is not int or p['use_px4_attitude'] not in (0, 1):
        raise ValueError('use_px4_attitude must be integer 0 or 1')
    for key in ('configuration_confirmed', 'ignore_rig_bias', 'reinit_keep_attitude'):
        if type(p[key]) is not bool:
            raise ValueError(key + ' must be boolean')
    minimum = p.get('min_init_anchors', max(4, min(6, len(anchors))))
    if type(minimum) is not int or minimum < 4 or (p['configuration_confirmed'] and minimum > len(anchors)):
        raise ValueError('Invalid min_init_anchors or too few confirmed anchors')
    mode = p['imu']
    if mode not in ('none', 'mavlink', 'px4', 'd435i'):
        raise ValueError('Unknown IMU mode')
    default_R = (0.,0.,1.,-1.,0.,0.,0.,-1.,0.) if mode == 'd435i' else MAVLINK_IMU_R
    rotation = np.asarray(p.get('imu_R', default_R) if mode != 'px4' else MAVLINK_IMU_R, float)
    if rotation.shape != (9,) or not np.isfinite(rotation).all():
        raise ValueError('imu_R must contain 9 finite values')
    R = rotation.reshape(3,3)
    if np.max(np.abs(R.T @ R - np.eye(3))) > 1e-6 or np.linalg.det(R) <= 0:
        raise ValueError('imu_R must be a proper rotation')
    names = ('sig_r','gate','k_pos','k_neg','na','ng','nba','nbg','init_static_s','init_window_s',
             'reinit_after_rejects','min_anchor_interval_s','reinit_keep_attitude','att_gate','att_min_interval_s',
             'imu_average_s','static_acc_tol','static_gyro_tol')
    kwargs = {name:p[name] for name in names}
    kwargs.update(init_min_anchors=minimum, att_sig_deg=(p['att_sig_tilt_deg'],p['att_sig_tilt_deg'],p['att_sig_yaw_deg']),
                  attitude_init='external' if p['use_px4_attitude'] else 'static', static_check=True,
                  imu_gap_reset_s=p['imu_timeout_s'])
    if not math.isfinite(p['yaw0_deg']):
        raise ValueError('Initial yaw must be finite')
    for key in ('imu_timeout_s','range_timeout_s','max_input_age_s','init_static_s','init_window_s'):
        if not math.isfinite(p[key]) or p[key] <= 0:
            raise ValueError(key + ' must be finite and positive')
    # Reuse the numerical core's parameter validation without replaying any inputs.
    from eskf_replay import EstimatorCore
    params = EskfParams(**kwargs)
    EstimatorCore(anchors, lever, math.radians(p['yaw0_deg']), params)
    return NodeConfig(p['configuration_confirmed'],mode,anchors,bias,p['ignore_rig_bias'],tuple(map(float,lever)),
                      math.radians(p['yaw0_deg']),tuple(rotation),bool(p['use_px4_attitude']),params,
                      p['imu_timeout_s'],p['range_timeout_s'],p['max_input_age_s'],str(p['config_sha256']))


def _stamp(value):
    """Keep integer ns precision; invalid header stamps do not enter the driver."""
    if isinstance(value, (bool, np.bool_)):
        return None
    if isinstance(value, (int, np.integer)):
        value = int(value)
    elif isinstance(value, (float, np.floating)) and math.isfinite(value) and value.is_integer():
        value = int(value)
    else:
        return None
    return value if 0 <= value <= np.iinfo(np.int64).max else None


def node_events(config: NodeConfig, imu_ns, acc, gyro, att_ns=(), att_q=(), ranges: Mapping[int, Any] | None = None,
                origin_ns: int | None = None) -> NodeEvents:
    """Apply the node's deterministic input handling.

    origin_ns: given, or the minimum first stamp over IMU, attitude and all range streams; seconds are
    (int64 stamp - origin) * 1e-9 (integer subtraction first, like the node's core_time).
    IMU: drop non-finite or non-increasing stamps; acc/gyro multiplied by imu_R.
    Ranges ({anchor_id: (ns array, value array)}): if not confirmed, drop all (ranges_unconfirmed);
    unknown anchor ids dropped (ranges_unknown_anchor); per anchor drop non-finite, <= 0, > 100 m or
    non-increasing stamps (ranges_invalid); then subtract bias_m[id] unless ignore_rig_bias; drop if <= 0.
    Attitude (q = w, x, y, z, body FRD -> local NED) only when use_px4_attitude and imu_mode != "none":
    rig yaw offset = rig_yaw_offset_from_yaw0(first valid q, yaw0); R = px4_attitude_to_rig(q, offset);
    invalid quaternions and non-increasing stamps are dropped (attitude).
    """
    ranges = ranges or {}
    imu_ns, acc, gyro = np.asarray(imu_ns), np.asarray(acc,float), np.asarray(gyro,float)
    att_ns, att_q = np.asarray(att_ns), np.asarray(att_q,float)
    if imu_ns.ndim != 1 or acc.shape != (len(imu_ns),3) or gyro.shape != acc.shape:
        raise ValueError('Expected IMU ns (N,), acc/gyro (N,3)')
    if att_ns.ndim != 1 or att_q.size and att_q.shape != (len(att_ns),4) or not att_q.size and len(att_ns):
        raise ValueError('Expected attitude ns (N,), quaternion (N,4)')
    if origin_ns is None:
        first = []
        for values in [imu_ns, att_ns, *(pair[0] for pair in ranges.values())]:
            for value in values:
                stamp = _stamp(value)
                if stamp is not None:
                    first.append(stamp)
                    break
        origin_ns = min(first) if first else 0
    origin_ns = _stamp(origin_ns)
    if origin_ns is None:
        raise ValueError('Expected an int64 nanosecond origin')
    dropped = dict(imu=0,ranges_unconfirmed=0,ranges_invalid=0,ranges_unknown_anchor=0,attitude=0)
    result = NodeEvents([],[],[],origin_ns,None,dropped)
    rotation = np.array(config.imu_R).reshape(3,3)
    previous = None
    for value,a,g in zip(imu_ns,acc,gyro):
        stamp = _stamp(value)
        if stamp is None or not np.isfinite(a).all() or not np.isfinite(g).all() or previous is not None and stamp<=previous:
            dropped['imu'] += 1
            continue
        previous = stamp
        result.imu.append(ImuSample((stamp-origin_ns)*1e-9,tuple(rotation@a),tuple(rotation@g)))
    known = {a.anchor_id for a in config.anchors}
    for aid,pair in sorted(ranges.items()):
        stamps, values = pair
        if len(stamps)!=len(values):
            raise ValueError('Range stamp/value length mismatch')
        if not config.confirmed:
            dropped['ranges_unconfirmed'] += len(stamps)
            continue
        if aid not in known:
            dropped['ranges_unknown_anchor'] += len(stamps)
            continue
        previous = None
        for value, measured in zip(stamps,values):
            stamp = _stamp(value)
            if stamp is None or not math.isfinite(measured) or measured<=0 or measured>100 or previous is not None and stamp<=previous:
                dropped['ranges_invalid'] += 1
                continue
            previous = stamp
            corrected = measured if config.ignore_rig_bias else measured-config.bias_m.get(aid,0.)
            if corrected<=0:
                dropped['ranges_invalid'] += 1
                continue
            result.ranges.append(RangeSample((stamp-origin_ns)*1e-9,int(aid),float(corrected)))
    result.ranges.sort(key=lambda s:(s.t,s.anchor_id,s.range_m))
    if config.use_px4_attitude and config.imu_mode!='none':
        previous = None
        for value,q in zip(att_ns,att_q):
            stamp = _stamp(value)
            try:
                if stamp is None or previous is not None and stamp<=previous:
                    raise ValueError('Non-increasing attitude stamp')
                offset = result.rig_yaw_offset_rad
                if offset is None:
                    offset = rig_yaw_offset_from_yaw0(q,config.yaw0_rad)
                R = px4_attitude_to_rig(q,offset)
            except (ValueError,TypeError):
                dropped['attitude'] += 1
                continue
            previous = stamp
            result.rig_yaw_offset_rad = offset
            result.attitude.append(AttitudeSample((stamp-origin_ns)*1e-9,R))
    else:
        dropped['attitude_disabled'] = len(att_ns)
    return result


def replay_node(config: NodeConfig, events: NodeEvents):
    """run_eskf_replay(config.anchors, events.imu, events.ranges, lever, yaw0, config.params,
    attitude=events.attitude). ValueError for imu_mode "none" (the UWB-only CV branch is not replayed)."""
    if config.imu_mode=='none':
        raise ValueError('The UWB-only CV branch is not replayed by this tool')
    return run_eskf_replay(config.anchors,events.imu,events.ranges,config.lever_flu,config.yaw0_rad,
                           config.params,attitude=events.attitude)


def compare_tracks(t_a, p_a, t_b, p_b, max_gap_s: float = 0.1) -> dict[str, float]:
    """Distance between track a and track b interpolated at a's times (inside b's span, not across
    b gaps > max_gap_s). Returns {"n", "coverage" (fraction of a used), "median_m", "p95_m", "max_m"};
    NaN stats and n = 0 when nothing overlaps."""
    t_a,p_a,t_b,p_b = map(lambda x:np.asarray(x,float),(t_a,p_a,t_b,p_b))
    if t_a.ndim!=1 or t_b.ndim!=1 or p_a.shape!=(len(t_a),3) or p_b.shape!=(len(t_b),3):
        raise ValueError('Expected times (N,) and XYZ positions (N,3)')
    if not math.isfinite(max_gap_s) or max_gap_s<=0:
        raise ValueError('Interpolation gap must be finite and positive')
    empty = dict(n=0,coverage=0.,median_m=float('nan'),p95_m=float('nan'),max_m=float('nan'))
    good = np.isfinite(t_b) & np.isfinite(p_b).all(axis=1)
    order = np.argsort(t_b[good],kind='stable')
    t_b,p_b = t_b[good][order],p_b[good][order]
    if len(t_b)<2 or not len(t_a):
        return empty
    unique = np.r_[True,np.diff(t_b)>0]
    t_b,p_b = t_b[unique],p_b[unique]
    if len(t_b)<2:
        return empty
    right = np.clip(np.searchsorted(t_b,t_a),1,len(t_b)-1)
    exact = (t_a==t_b[right]) | (t_a==t_b[right-1])
    selected = np.isfinite(t_a) & np.isfinite(p_a).all(axis=1) & (t_a>=t_b[0]) & (t_a<=t_b[-1]) \
        & ((t_b[right]-t_b[right-1]<=max_gap_s) | exact)
    interp = np.column_stack([np.interp(t_a[selected],t_b,p_b[:,k]) for k in range(3)])
    distance = np.linalg.norm(p_a[selected]-interp,axis=1)
    if not len(distance):
        return empty
    return dict(n=len(distance),coverage=len(distance)/len(t_a),median_m=float(np.median(distance)),
                p95_m=float(np.percentile(distance,95)),max_m=float(distance.max()))


def stream_stats(stamps_ns, gap_s: float = 0.2) -> dict[str, float]:
    """{"samples", "duration_s", "rate_hz" (1 / median positive dt), "dt_ms_p50", "dt_ms_p95",
    "dt_ms_max", "gaps" (dt > gap_s), "gap_time_s", "non_increasing" (dt <= 0)} of an int64 ns stream."""
    stamps = np.asarray(stamps_ns,dtype=np.int64)
    if stamps.ndim!=1 or not math.isfinite(gap_s) or gap_s<=0:
        raise ValueError('Expected a 1-D ns stream and finite positive gap threshold')
    dt = np.diff(stamps)*1e-9
    positive = dt[dt>0]
    median = float(np.median(positive)) if len(positive) else float('nan')
    gaps = dt>gap_s
    return dict(samples=len(stamps),duration_s=float((int(stamps[-1])-int(stamps[0]))*1e-9) if len(stamps)>1 else 0.,
                rate_hz=1/median,dt_ms_p50=median*1000,
                dt_ms_p95=float(np.percentile(positive,95)*1000) if len(positive) else float('nan'),
                dt_ms_max=float(positive.max()*1000) if len(positive) else float('nan'),gaps=int(gaps.sum()),
                gap_time_s=float(dt[gaps].sum()),non_increasing=int((dt<=0).sum()))
