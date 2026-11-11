"""E2.2-D contract: GUI -> onboard ESKF configuration for the vibration-aware estimator.

Plan/E2_2_YEU_CAU_ESKF_ONBOARD_CHONG_RUNG_BANG_PHAN_MEM.md, section 6. Stdlib only.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import eskf_config as ec  # noqa: E402

NEW = {   # key: (default, low, high)
    'na': (.45, 1e-4, 20.), 'ng': (.016, 1e-5, 1.), 'nba': (3e-3, 0., 1.), 'nbg': (1e-4, 0., .1),
    'min_anchor_interval_s': (.02, 0., .5), 'imu_average_s': (.02, 0., .1), 'use_px4_attitude': (1, 0, 1),
    'att_sig_tilt_deg': (2., .1, 30.), 'att_sig_yaw_deg': (5., .1, 90.), 'att_gate': (25., 1., 1000.),
}
OLD = ('sig_r', 'q_cv', 'init_static_s', 'init_window_s', 'imu_timeout_s', 'range_timeout_s',
       'max_input_age_s', 'min_init_anchors')


def config(**parameters):
    anchors = [[i + 1, x, y, z, True, 0.0, 10.0] for i, (x, y, z) in enumerate(
        [(0, 0, .48), (4.8, 0, .48), (4.8, 3.6, .48), (0, 3.6, .48),
         (0, 0, 2.208), (4.8, 0, 2.208), (4.8, 3.6, 2.208), (0, 3.6, 2.208)])]
    return {'schema': 1, 'confirmed': True, 'anchors': anchors, 'lever_flu_m': [-.09, 0., .31],
            'yaw0_deg': 0.0, 'parameters': parameters}


class EskfConfigE22Tests(unittest.TestCase):
    def test_new_parameters_and_bounds(self) -> None:
        for key, spec in NEW.items():
            self.assertIn(key, ec.PARAMETERS, key)
            self.assertEqual(tuple(ec.PARAMETERS[key]), spec, key)
        for key in OLD:
            self.assertIn(key, ec.PARAMETERS, key)                   # nothing removed

    def test_defaults_are_filled_and_validated(self) -> None:
        normalized = ec.normalize_config(config())                    # an older saved config without new keys
        for key, (default, _, _) in NEW.items():
            self.assertEqual(normalized['parameters'][key], default, key)
        self.assertIsInstance(normalized['parameters']['use_px4_attitude'], int)
        self.assertEqual(ec.normalize_config(config(use_px4_attitude=0))['parameters']['use_px4_attitude'], 0)
        self.assertEqual(ec.normalize_config(config(min_anchor_interval_s=0.0, imu_average_s=0.0))['parameters']
                         ['min_anchor_interval_s'], 0.0)              # 0 disables rate limit / averaging
        for bad in ({'use_px4_attitude': .5}, {'use_px4_attitude': 2}, {'na': 0.0}, {'ng': 5.0},
                    {'imu_average_s': .5}, {'att_gate': float('nan')}, {'na': True}, {'unknown_key': 1.0}):
            with self.assertRaises(ValueError, msg=str(bad)):
                ec.normalize_config(config(**bad))
        self.assertNotEqual(ec.config_hash(config()), ec.config_hash(config(na=.5)))

    def test_rendered_node_parameters(self) -> None:
        files = ec.render_files(config(na=.6, use_px4_attitude=0), '/tmp/eskf')
        params = json.loads(files['eskf.yaml'])['uwb_eskf']['ros__parameters']
        self.assertEqual(params['attitude_topic'], '/px4/attitude_frd')
        self.assertEqual(params['imu_topic'], '/px4/imu_frd')
        self.assertEqual(params['na'], .6)
        self.assertEqual(params['use_px4_attitude'], 0)
        for key in NEW:
            self.assertIn(key, params, key)
        for key in OLD:
            self.assertIn(key, params, key)
        self.assertEqual(params['ignore_rig_bias'], True)              # unchanged behaviour


if __name__ == "__main__":
    unittest.main()
