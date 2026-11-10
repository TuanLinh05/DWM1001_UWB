"""E2.3-A contract: replay an onboard uwb_eskf session the way the E2.2 node consumes it
(onboard_replay.py), plus the opt-in sensor-aided anchor bias in the GUI->onboard config.

Plan/E2_3_YEU_CAU_CONG_CU_PHIEN_ONBOARD_VA_BAY_SHADOW.md (section 3). Synthetic data: e21_fixtures.
Reviewer reference results are noted next to thresholds.
"""

from __future__ import annotations

import json
import math
import unittest

import numpy as np

import e21_fixtures as F
import eskf_config as ec
import eskf_replay as er
import onboard_replay as ob  # noqa: E402

# eskf.yaml (ros__parameters) recorded onboard during Test_1, before E2.2 keys existed.
TEST1_ESKF_YAML = {
    "imu": "mavlink", "imu_topic": "/px4/imu_frd", "imu_link": "px4_imu_flu",
    "imu_R": [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0], "frame_id": "uwb_rig",
    "configuration_confirmed": False, "ignore_rig_bias": True, "lever": [0.0, 0.0, 0.0], "yaw0_deg": 0.0,
    "config_sha256": "f30674ee", "sig_r": 0.15, "q_cv": 1.0, "init_static_s": 1.0, "init_window_s": 0.05,
    "imu_timeout_s": 0.2, "range_timeout_s": 0.5, "max_input_age_s": 0.25, "min_init_anchors": 6,
}
RIG_TEXT = "# GUI configuration x\n# comment an9_p, 1, 2, 3\n" + "".join(
    f"an{a.anchor_id}_p, {a.x}, {a.y}, {a.z}\n" for a in F.ROOM_48)
NED_TO_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FLU_TO_FRD = np.diag([1., -1., -1.])


def gui_config(**parameters):
    anchors = [[a.anchor_id, a.x, a.y, a.z, True, 0.0, 10.0] for a in F.ROOM_48]
    return {"schema": 1, "confirmed": True, "anchors": anchors, "lever_flu_m": [-.09, 0., .31],
            "yaw0_deg": 30.0, "parameters": parameters}


def rendered(value):
    files = ec.render_files(value, "/tmp/eskf")
    return json.loads(files["eskf.yaml"])["uwb_eskf"]["ros__parameters"], files["rig_onboard.txt"]


def matrix_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([w, x, y, z])


def recorded_session(sim, rig_offset, origin_ns=1_794_305_934_213_750_800, att_hz=50.0):
    """Express a synthetic flight the way the onboard bag stores it: int64 ns stamps, FRD IMU,
    PX4 quaternions body FRD -> local NED, per-anchor range topics."""
    t = sim["imu_t"]
    imu_ns = origin_ns + np.round(t * 1e9).astype(np.int64)
    acc_frd, gyro_frd = sim["imu_acc"] * [1, -1, -1], sim["imu_gyro"] * [1, -1, -1]
    a = rig_offset - math.pi / 2
    Rz = np.array([[math.cos(a), -math.sin(a), 0.], [math.sin(a), math.cos(a), 0.], [0., 0., 1.]])
    att_t = np.arange(0.0, t[-1], 1.0 / att_hz)
    j = np.clip(np.searchsorted(sim["truth_t"], att_t), 0, len(sim["truth_t"]) - 1)
    q = np.array([matrix_to_quat(NED_TO_ENU.T @ Rz.T @ sim["truth_R"][k] @ FLU_TO_FRD.T) for k in j])
    att_ns = origin_ns + np.round(sim["truth_t"][j] * 1e9).astype(np.int64)
    ranges = {}
    for tt, aid, r in sim["ranges"]:
        ranges.setdefault(int(aid), ([], []))
        ranges[int(aid)][0].append(origin_ns + int(round(tt * 1e9)))
        ranges[int(aid)][1].append(r)
    ranges = {k: (np.array(v[0], dtype=np.int64), np.array(v[1])) for k, v in ranges.items()}
    return imu_ns, acc_frd, gyro_frd, att_ns, q, ranges


class OnboardReplayTests(unittest.TestCase):
    def test_node_parameter_mapping(self) -> None:
        params, rig = rendered(gui_config(na=.6, ng=.02, min_anchor_interval_s=.03, imu_average_s=.01))
        cfg = ob.node_config(params, rig)
        self.assertEqual(ob.ONBOARD_REPLAY_VERSION, "onboard-replay-v1")
        p = cfg.params
        self.assertTrue(cfg.confirmed and cfg.use_px4_attitude and cfg.ignore_rig_bias)
        self.assertEqual((p.na, p.ng, p.min_anchor_interval_s, p.imu_average_s), (.6, .02, .03, .01))
        self.assertEqual(p.attitude_init, "external")
        self.assertTrue(p.static_check)
        self.assertEqual(p.imu_gap_reset_s, cfg.imu_timeout_s)
        self.assertEqual(tuple(p.att_sig_deg), (2.0, 2.0, 5.0))
        self.assertTrue(p.reinit_keep_attitude)
        self.assertEqual(p.init_min_anchors, 6)
        self.assertEqual(cfg.lever_flu, (-.09, 0.0, .31))
        self.assertAlmostEqual(cfg.yaw0_rad, math.radians(30.0), places=12)
        self.assertEqual([a.anchor_id for a in cfg.anchors], list(range(1, 9)))
        static_params, _ = rendered(gui_config(use_px4_attitude=0))
        self.assertEqual(ob.node_config(static_params, rig).params.attitude_init, "static")
        old = ob.node_config(TEST1_ESKF_YAML, rig)                        # pre-E2.2 file -> node defaults
        self.assertFalse(old.confirmed)
        self.assertEqual((old.params.na, old.params.ng, old.params.imu_average_s), (.45, .016, .02))
        self.assertEqual(old.params.attitude_init, "external")             # node default use_px4_attitude = 1
        self.assertEqual(old.lever_flu, (0.0, 0.0, 0.0))
        for bad in ({"use_px4_attitude": 2}, {"lever": [0.1, 0.2]}, {"imu_R": [1, 0, 0]}):
            with self.assertRaises(ValueError, msg=str(bad)):
                ob.node_config({**TEST1_ESKF_YAML, **bad}, rig)
        without_lever = {k: v for k, v in TEST1_ESKF_YAML.items() if k != "lever"}
        with self.assertRaises(ValueError):                                # URDF lookup is not replayed offline
            ob.node_config(without_lever, rig)

    def test_rig_text_rules_match_filter_hpp(self) -> None:
        text = ("# an7_p, 9, 9, 9\nan1_p, 0.0, 0.0, 0.48\n an2_p,4.8,0,0.48\nan3_bias, 0.043\nxx an4_p, 1, 1, 1\n"
                "an12_p, 1.5, 2.5, 3.5\nan5_p, 1, 2\nan6_bias\n")
        anchors, bias = ob.parse_rig(text)
        self.assertEqual(anchors, {1: (0.0, 0.0, 0.48), 2: (4.8, 0.0, 0.48), 12: (1.5, 2.5, 3.5)})
        self.assertEqual(bias, {3: 0.043})

    def test_node_input_guards(self) -> None:
        sim = F.vibrating_flight(duration_s=6.0, seed=51)
        imu_ns, acc, gyro, att_ns, q, ranges = recorded_session(sim, math.radians(70.0))
        params, rig = rendered(gui_config())
        cfg = ob.node_config(params, rig)
        ev = ob.node_events(cfg, imu_ns, acc, gyro, att_ns, q, ranges)
        self.assertEqual(ev.origin_ns, int(min(imu_ns[0], att_ns[0], min(v[0][0] for v in ranges.values()))))
        t = np.array([s.t for s in ev.imu])
        self.assertLess(float(np.max(np.abs(t - (imu_ns - ev.origin_ns) * 1e-9))), 1e-12)   # int64 subtraction first
        self.assertTrue(np.allclose([s.acc for s in ev.imu], sim["imu_acc"], atol=1e-12))     # FRD -> FLU via imu_R
        first = ev.attitude[0].R
        self.assertAlmostEqual(math.atan2(first[1, 0], first[0, 0]), cfg.yaw0_rad, places=9)  # offset from yaw0
        self.assertEqual(len(ev.ranges), sum(len(v[0]) for v in ranges.values()))
        unconfirmed = ob.node_config({**params, "configuration_confirmed": False}, rig)
        ev_u = ob.node_events(unconfirmed, imu_ns, acc, gyro, att_ns, q, ranges)
        self.assertEqual(len(ev_u.ranges), 0)                                # the node ignores ranges until confirmed
        self.assertEqual(ev_u.dropped["ranges_unconfirmed"], len(ev.ranges))
        biased_rig = rig + "an3_bias, 0.25\n"
        ev_b = ob.node_events(ob.node_config({**params, "ignore_rig_bias": False}, biased_rig), imu_ns, acc, gyro,
                              att_ns, q, ranges)
        r3 = [s.range_m for s in ev.ranges if s.anchor_id == 3]
        r3b = [s.range_m for s in ev_b.ranges if s.anchor_id == 3]
        self.assertTrue(np.allclose(np.array(r3) - np.array(r3b), 0.25))
        ev_i = ob.node_events(ob.node_config(params, biased_rig), imu_ns, acc, gyro, att_ns, q, ranges)
        self.assertEqual([s.range_m for s in ev_i.ranges], [s.range_m for s in ev.ranges])   # ignore_rig_bias=True
        bad_ranges = {**ranges, 2: (np.r_[ranges[2][0], ranges[2][0][-1], ranges[2][0][-1] + 10],
                                    np.r_[ranges[2][1], 2.0, 150.0]), 42: (ranges[1][0][:3], ranges[1][1][:3])}
        ev_x = ob.node_events(cfg, imu_ns, acc, gyro, att_ns, q, bad_ranges)
        self.assertEqual(ev_x.dropped["ranges_invalid"], 2)                 # repeated stamp and > 100 m
        self.assertEqual(ev_x.dropped["ranges_unknown_anchor"], 3)
        bad_q = q.copy()
        bad_q[5] = 0.0
        ev_q = ob.node_events(cfg, imu_ns, acc, gyro, att_ns, bad_q, ranges)
        self.assertEqual(len(ev_q.attitude), len(ev.attitude) - 1)
        no_att = ob.node_config({**params, "use_px4_attitude": 0}, rig)
        self.assertEqual(len(ob.node_events(no_att, imu_ns, acc, gyro, att_ns, q, ranges).attitude), 0)

    def test_synthetic_session_replays_like_the_core(self) -> None:
        sim = F.vibrating_flight(duration_s=20.0, seed=52)                 # starts at yaw 30 deg (e20 default)
        rig_offset = math.radians(70.0)
        imu_ns, acc, gyro, att_ns, q, ranges = recorded_session(sim, rig_offset)
        params, rig = rendered(gui_config())                                # yaw0_deg = 30 = true start heading
        cfg = ob.node_config(params, rig)
        ev = ob.node_events(cfg, imu_ns, acc, gyro, att_ns, q, ranges)
        wrapped = (ev.rig_yaw_offset_rad - rig_offset + math.pi) % (2 * math.pi) - math.pi
        self.assertLess(abs(wrapped), 1e-6)                                 # offset recovered from yaw0
        res = ob.replay_node(cfg, ev)
        self.assertEqual(res.reinits, 0)
        self.assertLess(res.init_time, 0.2)                                 # external attitude, no static wait
        self.assertGreater(len(res.att_update_t), 500)
        truth = np.column_stack([np.interp(res.t, sim["truth_t"], sim["truth_p"][:, k]) for k in range(3)])
        err = np.linalg.norm(res.p - truth, axis=1)
        self.assertLess(float(np.sqrt(np.mean(err[res.t > 6.0] ** 2))), 0.05)    # reference: 3.8 cm (GUI default Q)
        direct = er.run_eskf_replay(list(cfg.anchors), ev.imu, ev.ranges, np.array(cfg.lever_flu),
                                    cfg.yaw0_rad, cfg.params, attitude=ev.attitude)
        self.assertTrue(np.array_equal(direct.t, res.t))
        self.assertLess(float(np.max(np.abs(direct.p - res.p))), 1e-12)
        with self.assertRaises(ValueError):
            ob.replay_node(ob.node_config({**params, "imu": "none"}, rig), ev)

    def test_track_comparison_and_stream_statistics(self) -> None:
        t = np.arange(0.0, 10.0, 0.01)
        p = np.column_stack([np.sin(t), np.cos(t), 0.1 * t])
        same = ob.compare_tracks(t, p, t, p)
        self.assertEqual(same["n"], len(t))
        self.assertLess(same["max_m"], 1e-12)
        shifted = ob.compare_tracks(t, p + [0.03, 0.0, 0.04], t, p)
        self.assertAlmostEqual(shifted["median_m"], 0.05, places=9)
        holed = (t < 4.0) | (t > 5.0)
        part = ob.compare_tracks(t, p, t[holed], p[holed], max_gap_s=0.1)
        self.assertLess(part["coverage"], 0.92)
        ns = (np.r_[np.arange(0, 1000) * 20, 20_500 + np.arange(0, 100) * 20] * 1_000_000).astype(np.int64)
        ns = np.r_[ns, ns[-1]]                                              # one repeated stamp
        s = ob.stream_stats(ns, gap_s=0.2)
        self.assertEqual(s["samples"], 1101)
        self.assertAlmostEqual(s["rate_hz"], 50.0, places=6)
        self.assertEqual(s["gaps"], 1)
        self.assertAlmostEqual(s["gap_time_s"], 0.52, places=6)
        self.assertEqual(s["non_increasing"], 1)

    def test_opt_in_aided_bias_in_the_onboard_configuration(self) -> None:
        plain = ec.normalize_config(gui_config())
        self.assertNotIn("aided_bias_m", plain)                             # old configs keep their hash
        self.assertEqual(ec.config_hash(gui_config()), ec.config_hash({**gui_config(), "aided_bias_m": []}))
        value = {**gui_config(), "aided_bias_m": [[4, 0.0914], [1, 0.01366], [7, -0.07787]]}
        normalized = ec.normalize_config(value)
        self.assertEqual(normalized["aided_bias_m"], [[1, 0.01366], [4, 0.0914], [7, -0.07787]])
        self.assertNotEqual(ec.config_hash(value), ec.config_hash(gui_config()))
        params, rig = rendered(value)
        self.assertFalse(params["ignore_rig_bias"])
        anchors, bias = ob.parse_rig(rig)
        self.assertEqual(bias, {1: 0.01366, 4: 0.0914, 7: -0.07787})
        self.assertEqual(len(anchors), 8)
        plain_params, plain_rig = rendered(gui_config())
        self.assertTrue(plain_params["ignore_rig_bias"])
        self.assertEqual(ob.parse_rig(plain_rig)[1], {})
        for bad in ([[9, 0.01]], [[1, 0.6]], [[1, 0.01], [1, 0.02]], [[1]], "1:0.01", [[1.0, 0.01]]):
            with self.assertRaises(ValueError, msg=str(bad)):
                ec.normalize_config({**gui_config(), "aided_bias_m": bad})


if __name__ == "__main__":
    unittest.main()
