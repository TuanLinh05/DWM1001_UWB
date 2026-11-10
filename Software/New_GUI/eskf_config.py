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
    minimum = parameters['min_init_anchors']
    if minimum != int(minimum):
        raise ValueError('min_init_anchors must be an integer')
    parameters['min_init_anchors'] = int(minimum)
    enabled = [a for a in anchors if a[4]]
    if value['confirmed']:
        if len(enabled) < minimum:
            raise ValueError(f'Enable at least {int(minimum)} anchors for ESKF, or reduce min init anchors')
        if not has_3d_geometry([a[1:4] for a in enabled]):
            raise ValueError('3D ESKF requires enabled anchors with non-coplanar coordinates')
        if any(math.dist(a[1:4], b[1:4]) < 1e-6 for a, b in itertools.combinations(enabled, 2)):
            raise ValueError('Enabled anchors cannot have identical coordinates')
    return {'schema': 1, 'confirmed': value['confirmed'], 'anchors': anchors,
            'lever_flu_m': lever, 'yaw0_deg': yaw, 'parameters': parameters}


def config_hash(value):
    normalized = normalize_config(value)
    return hashlib.sha256(json.dumps(normalized, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def render_files(value, directory):
    value = normalize_config(value)
    digest = config_hash(value)
    rig = '# GUI configuration ' + digest + '\n# Calibrated /range input; host bias is NOT subtracted again.\n'
    rig += ''.join(f'an{a[0]}_p, {a[1]}, {a[2]}, {a[3]}\n' for a in value['anchors'] if a[4])
    params = {'imu': 'mavlink', 'imu_topic': '/px4/imu_frd', 'imu_link': 'px4_imu_flu',
              'imu_R': [1., 0., 0., 0., -1., 0., 0., 0., -1.], 'frame_id': 'uwb_rig',
              'rig_file': str(directory) + '/rig_onboard.txt', 'configuration_confirmed': value['confirmed'],
              'ignore_rig_bias': True, 'lever': value['lever_flu_m'], 'yaw0_deg': value['yaw0_deg'],
              'config_sha256': digest, **value['parameters']}
    physical = {'configuration_confirmed': value['confirmed'], 'source': 'GUI', 'config_sha256': digest,
                'lever_flu_m': value['lever_flu_m'], 'yaw0_deg': value['yaw0_deg'],
                'coordinate_convention': 'rig Z up; body FLU; lever = TAG position minus IMU position',
                'host_bias_and_sigma': 'retained in gui_config.json; ESKF uses calibrated ranges and global sig_r'}
    return {'rig_onboard.txt': rig, 'eskf.yaml': json.dumps({'uwb_eskf': {'ros__parameters': params}}, indent=2),
            'physical_setup.json': json.dumps(physical, indent=2), 'gui_config.json': json.dumps(value, indent=2)}
