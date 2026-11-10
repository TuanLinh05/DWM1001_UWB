"""Type-sensitive release settings must be validated even after a cached solve."""

from dataclasses import replace
import json
import unittest

import numpy as np

from g0_fixtures import ROOM_54, SPOTS_4, feed_spot_capture
from core_state import UwbStateEngine
from multi_spot_calibration import JointCalibrationSettings


class PriorReleaseCacheTests(unittest.TestCase):
    def test_equal_values_with_invalid_types_do_not_reuse_valid_results(self):
        engine = UwbStateEngine(ROOM_54)
        cycle = 1
        for point in SPOTS_4[:2]:
            cycle = feed_spot_capture(engine, point, point, first_cycle=cycle)
        cases = (({}, {"release_flagged_spot_priors": 1}),
                 ({"max_passes": 1}, {"max_passes": True}),
                 ({}, {"max_passes": 3.0}),
                 ({}, {"release_flagged_spot_priors": np.bool_(True)}))
        for valid, invalid in cases:
            with self.subTest(invalid=invalid):
                engine.joint_settings = JointCalibrationSettings(**valid)
                previous = engine.joint_calibration()
                self.assertEqual(previous.method, "joint")
                engine.joint_settings = replace(engine.joint_settings, **invalid)
                current = engine.joint_calibration()
                self.assertIsNot(current, previous)
                self.assertEqual(current.method, "failed")
                self.assertIn("NOT_IDENTIFIABLE", current.warning_codes())
                self.assertEqual(current.passes, 0)
                self.assertEqual(current.released_spot_indices, ())
                self.assertEqual(engine.active_calibration_method(), "median")
                json.dumps(engine.calibration_summary(), allow_nan=False)


if __name__ == "__main__":
    unittest.main()
