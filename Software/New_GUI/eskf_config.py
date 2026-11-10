"""Shared GUI/onboard ESKF configuration schema; stdlib only."""
import hashlib
import itertools
import json
import math

PARAMETERS = {
    'sig_r': (.15, .005, 5.), 'q_cv': (1., .0001, 100.),
    'init_static_s': (1., .2, 30.), 'init_window_s': (.05, .01, .5),
    'imu_timeout_s': (.2, .05, 2.), 'range_timeout_s': (.5, .1, 5.),
    'max_input_age_s': (.25, .02, 2.), 'min_init_anchors': (6, 4, 8),
    'na': (.45, 1e-4, 20.), 'ng': (.016, 1e-5, 1.), 'nba': (3e-3, 0., 1.), 'nbg': (1e-4, 0., .1),
    'min_anchor_interval_s': (.02, 0., .5), 'imu_average_s': (.02, 0., .1),
    'use_px4_attitude': (1, 0, 1), 'att_sig_tilt_deg': (2., .1, 30.),
    'att_sig_yaw_deg': (5., .1, 90.), 'att_gate': (25., 1., 1000.),
}


def number(value, low, high, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{label} must be a finite number between {low} and {high}')
    return round(float(value), 8)


def has_3d_geometry(points):
    if len(points) < 4:
        return False
    span = max(max(p[k] for p in points) - min(p[k] for p in points) for k in range(3))
    for a, b, c, d in itertools.combinations(points, 4):
        u, v, w = [[p[k] - a[k] for k in range(3)] for p in (b, c, d)]
        determinant = u[0]*(v[1]*w[2]-v[2]*w[1])-u[1]*(v[0]*w[2]-v[2]*w[0])+u[2]*(v[0]*w[1]-v[1]*w[0])
        if abs(determinant) > max(1e-9, span**3 * 1e-7):
            return True
    return False


def normalize_config(value):
    if not isinstance(value, dict) or value.get('schema') != 1:
        raise ValueError('Unsupported ESKF configuration schema')
    if type(value.get('confirmed')) is not bool:
        raise ValueError('ESKF configuration confirmation must be true or false')
    rows = value.get('anchors')
    if not isinstance(rows, list) or len(rows) > 8:
        raise ValueError('Provide at most 8 anchor rows')
    anchors, seen = [], set()
    for row in rows:
        if not isinstance(row, list) or len(row) != 7:
            raise ValueError('Anchor row must be [id,x,y,z,enabled,bias_mm,sigma_mm]')
        aid = row[0]
        if type(aid) is not int or not 1 <= aid <= 8 or aid in seen or type(row[4]) is not bool:
            raise ValueError('Anchor IDs must be unique 1..8, with a boolean enabled flag')
        seen.add(aid)
        xyz = [number(row[k], -10000, 10000, f'A{aid} XYZ') for k in (1, 2, 3)]
        anchors.append([aid, *xyz, row[4], number(row[5], -10000, 10000, 'Host bias mm'),
                        number(row[6], .01, 10000, 'Host sigma mm')])
    anchors.sort(key=lambda a: a[0])
    lever = value.get('lever_flu_m')
    if not isinstance(lever, list) or len(lever) != 3:
        raise ValueError('TAG minus IMU lever requires X/Y/Z in body FLU, metres')
    lever = [number(v, -5, 5, 'TAG-IMU offset') for v in lever]
    yaw = number(value.get('yaw0_deg'), -360, 360, 'Initial yaw degrees')
    supplied = value.get('parameters', {})
    if not isinstance(supplied, dict) or set(supplied) - set(PARAMETERS):
        raise ValueError('Unknown ESKF parameter')
    parameters = {key: number(supplied.get(key, default), low, high, key)
                  for key, (default, low, high) in PARAMETERS.items()}
    for key in ('min_init_anchors', 'use_px4_attitude'):
        if parameters[key] != int(parameters[key]):
            raise ValueError(key + ' must be an integer')
        parameters[key] = int(parameters[key])
    minimum = parameters['min_init_anchors']
    enabled = [a for a in anchors if a[4]]
    if value['confirmed']:
        if len(enabled) < minimum:
            raise ValueError(f'Enable at least {int(minimum)} anchors for ESKF, or reduce min init anchors')
        if not has_3d_geometry([a[1:4] for a in enabled]):
            raise ValueError('3D ESKF requires enabled anchors with non-coplanar coordinates')
        if any(math.dist(a[1:4], b[1:4]) < 1e-6 for a, b in itertools.combinations(enabled, 2)):
            raise ValueError('Enabled anchors cannot have identical coordinates')
    aided = value.get('aided_bias_m', [])
    if not isinstance(aided,list):
        raise ValueError('aided_bias_m must be a list of [anchor_id, bias_m]')
    aided_rows, aided_seen = [], set()
    for row in aided:
        if not isinstance(row,list) or len(row)!=2 or type(row[0]) is not int or row[0] not in seen or row[0] in aided_seen:
            raise ValueError('Aided bias IDs must be unique integers present in the rig')
        aided_seen.add(row[0])
        aided_rows.append([row[0],number(row[1],-.5,.5,'Aided bias m')])
    result = {'schema': 1, 'confirmed': value['confirmed'], 'anchors': anchors,
              'lever_flu_m': lever, 'yaw0_deg': yaw, 'parameters': parameters}
    if aided_rows:
        result['aided_bias_m'] = sorted(aided_rows)
    return result


def config_hash(value):
    normalized = normalize_config(value)
    return hashlib.sha256(json.dumps(normalized, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def aided_bias_rows(document, known_ids):
    """Import the constant E2.0 correction only; never discard directional terms."""
    if not isinstance(document, dict) or document.get('ok') is not True or document.get('harmonics_used') != 0:
        raise ValueError('Require a successful aided_calibration.json with harmonics_used = 0')
    corrections = document.get('corrections')
    if not isinstance(corrections, dict) or not corrections:
        raise ValueError('Aided calibration has no corrections')
    rows, seen = [], set()
    for key, correction in corrections.items():
        if not isinstance(correction, dict):
            raise ValueError('Invalid aided correction')
        aid = correction.get('anchor_id')
        if type(aid) is not int or aid not in known_ids or str(aid) != key or aid in seen:
            raise ValueError('Aided anchor IDs must match the current rig and JSON keys')
        if correction.get('cos_m') != [] or correction.get('sin_m') != []:
            raise ValueError('Directional corrections cannot be represented by constant onboard bias')
        seen.add(aid)
        rows.append([aid, number(correction.get('bias_m'), -.5, .5, 'Aided bias m')])
    return sorted(rows)


def render_files(value, directory):
    value = normalize_config(value)
    digest = config_hash(value)
    rig = '# GUI configuration ' + digest + '\n# Calibrated /range input; host bias is NOT subtracted again.\n'
    rig += ''.join(f'an{a[0]}_p, {a[1]}, {a[2]}, {a[3]}\n' for a in value['anchors'] if a[4])
    enabled = {a[0] for a in value['anchors'] if a[4]}
    aided = [(aid,bias) for aid,bias in value.get('aided_bias_m',[]) if aid in enabled]
    if aided:
        rig += '# Sensor-aided EXTRA bias: measured = true + bias; node subtracts once.\n'
        rig += ''.join(f'an{aid}_bias, {bias}\n' for aid,bias in aided)
    params = {'imu': 'mavlink', 'imu_topic': '/px4/imu_frd', 'attitude_topic': '/px4/attitude_frd', 'imu_link': 'px4_imu_flu',
              'imu_R': [1., 0., 0., 0., -1., 0., 0., 0., -1.], 'frame_id': 'uwb_rig',
              'rig_file': str(directory) + '/rig_onboard.txt', 'configuration_confirmed': value['confirmed'],
              'ignore_rig_bias': not bool(aided), 'lever': value['lever_flu_m'], 'yaw0_deg': value['yaw0_deg'],
              'config_sha256': digest, **value['parameters']}
    physical = {'configuration_confirmed': value['confirmed'], 'source': 'GUI', 'config_sha256': digest,
                'lever_flu_m': value['lever_flu_m'], 'yaw0_deg': value['yaw0_deg'],
                'coordinate_convention': 'rig Z up; body FLU; lever = TAG position minus IMU position',
                'host_bias_and_sigma': 'retained in gui_config.json; ESKF uses calibrated ranges and global sig_r'}
    return {'rig_onboard.txt': rig, 'eskf.yaml': json.dumps({'uwb_eskf': {'ros__parameters': params}}, indent=2),
            'physical_setup.json': json.dumps(physical, indent=2), 'gui_config.json': json.dumps(value, indent=2)}
