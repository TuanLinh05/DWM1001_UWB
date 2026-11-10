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
from typing import Any, Mapping

import numpy as np  # noqa: F401

from eskf_replay import EskfParams  # noqa: F401

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
    raise NotImplementedError("E2.3: implement parse_rig")


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
    raise NotImplementedError("E2.3: implement node_config")


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
    raise NotImplementedError("E2.3: implement node_events")


def replay_node(config: NodeConfig, events: NodeEvents):
    """run_eskf_replay(config.anchors, events.imu, events.ranges, lever, yaw0, config.params,
    attitude=events.attitude). ValueError for imu_mode "none" (the UWB-only CV branch is not replayed)."""
    raise NotImplementedError("E2.3: implement replay_node")


def compare_tracks(t_a, p_a, t_b, p_b, max_gap_s: float = 0.1) -> dict[str, float]:
    """Distance between track a and track b interpolated at a's times (inside b's span, not across
    b gaps > max_gap_s). Returns {"n", "coverage" (fraction of a used), "median_m", "p95_m", "max_m"};
    NaN stats and n = 0 when nothing overlaps."""
    raise NotImplementedError("E2.3: implement compare_tracks")


def stream_stats(stamps_ns, gap_s: float = 0.2) -> dict[str, float]:
    """{"samples", "duration_s", "rate_hz" (1 / median positive dt), "dt_ms_p50", "dt_ms_p95",
    "dt_ms_max", "gaps" (dt > gap_s), "gap_time_s", "non_increasing" (dt <= 0)} of an int64 ns stream."""
    raise NotImplementedError("E2.3: implement stream_stats")
