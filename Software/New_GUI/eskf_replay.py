"""Offline ESKF-A replay (E2.0-B) — SKELETON.

Plan: Plan/E2_0_YEU_CAU_HIEU_CHUAN_CO_CAM_BIEN_VA_REPLAY_ESKF_A.md (section 4).
Contract: tests/test_e20_eskf_replay.py. Do not loosen the tests.

Purpose: a Python mirror of the onboard filter
UP4000/estimator/uwb_eskf/include/uwb_eskf/filter.hpp (struct ESKF,
range_update, so3_exp, attitude_from_accel, gauss_newton) and of the node logic
in UP4000/estimator/uwb_eskf/src/uwb_eskf_node.cpp (static attitude init,
Gauss-Newton position init, re-init after consecutive rejections). It replays
logged IMU + UWB ranges deterministically so the onboard filter can be studied,
tuned and later checked for parity against the C++ code.

Frames: rig ENU (z up), body FLU, gravity (0, 0, -9.81). IMU acc is specific
force in body FLU (m/s^2), gyro in rad/s. PX4 FRD -> FLU: diag(1, -1, -1).
State p is the IMU/body origin; the TAG is at p + R @ lever_flu.

Equations: copy filter.hpp exactly (same F, Q, Huber asymmetry, gate, Joseph
form, injection). One intended difference: at initialization the body
position is tag0 - R0 @ lever (the C++ node currently uses tag0 directly;
report this in the change description so the C++ node can be fixed later).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence

import numpy as np

REPLAY_VERSION = "eskf-a-replay-v1"
GRAVITY = np.array([0.0, 0.0, -9.81])


@dataclass(frozen=True)
class EskfParams:
    # filter.hpp Params
    sig_r: float = 0.15                    # range std (m)
    gate: float = 25.0                     # NIS gate (5 sigma)
    k_pos: float = 1.5                     # Huber threshold for positive innovations (range too long, NLOS)
    k_neg: float = 3.0                     # Huber threshold for negative innovations
    na: float = 175e-6 * 9.81              # accel noise density (m/s^2/sqrtHz)
    ng: float = 0.014 * math.pi / 180      # gyro noise density (rad/s/sqrtHz)
    nba: float = 3e-3                      # accel bias random walk
    nbg: float = 1e-4                      # gyro bias random walk
    # node logic
    init_static_s: float = 1.0             # IMU averaging time before attitude init
    init_window_s: float = 0.05            # ranges newer than this are used for the GN init
    init_min_anchors: int = 6              # min(init_min_anchors, enabled anchors) distinct anchors
    reinit_after_rejects: int = 200        # consecutive refused ranges -> drop the filter and re-init
    # ESKF::init defaults
    sig_p0: float = 0.3
    sig_v0: float = 0.5
    sig_th0_deg: tuple[float, float, float] = (5.0, 5.0, 30.0)
    sig_ba0: float = 0.2
    sig_bg0_deg: float = 1.0


@dataclass(frozen=True)
class ImuSample:
    t: float
    acc: tuple[float, float, float]        # specific force, body FLU
    gyro: tuple[float, float, float]       # body FLU


@dataclass(frozen=True)
class RangeSample:
    t: float
    anchor_id: int
    range_m: float                         # already calibrated range (do not subtract a bias again)


@dataclass
class ReplayResult:
    t: np.ndarray               # (M,) output times = IMU sample times after initialization
    p: np.ndarray               # (M, 3) body/IMU position
    v: np.ndarray               # (M, 3)
    R: np.ndarray               # (M, 3, 3) body-to-rig rotation
    tag: np.ndarray             # (M, 3) p + R @ lever
    p_std: np.ndarray           # (M, 3) sqrt of the position covariance diagonal
    update_t: np.ndarray        # (U,) one entry per range offered to an initialized filter
    update_anchor: np.ndarray   # (U,) int
    update_accepted: np.ndarray  # (U,) bool
    update_nis: np.ndarray      # (U,)
    update_innovation_m: np.ndarray  # (U,) measured - predicted range
    init_time: float | None     # time of the FIRST initialization
    reinits: int                # number of drops after reinit_after_rejects consecutive refusals
    version: str = REPLAY_VERSION

    def acceptance_ratio(self) -> float:
        return float(np.mean(self.update_accepted)) if len(self.update_accepted) else 0.0


def skew(v) -> np.ndarray:
    raise NotImplementedError("E2.0-B: skew")


def so3_exp(phi) -> np.ndarray:
    """Rodrigues; I + skew(phi) below 1e-9 rad, as filter.hpp."""
    raise NotImplementedError("E2.0-B: so3_exp")


def attitude_from_accel(f, yaw: float) -> np.ndarray:
    """filter.hpp attitude_from_accel: roll = atan2(f_y, f_z), pitch = atan2(-f_x, hypot(f_y, f_z)),
    R = Rz(yaw) Ry(pitch) Rx(roll)."""
    raise NotImplementedError("E2.0-B: attitude_from_accel")


def gauss_newton(anchors_xyz: np.ndarray, ranges: np.ndarray, p0: np.ndarray, iters: int = 10) -> np.ndarray:
    raise NotImplementedError("E2.0-B: gauss_newton")


def run_eskf_replay(anchors: Sequence[Any], imu: Sequence[ImuSample], ranges: Sequence[RangeSample],
                    lever_flu, yaw0_rad: float, params: EskfParams = EskfParams()) -> ReplayResult:
    """Replay IMU and ranges in time order and return the filter trajectory.

    Input handling: drop samples with non-finite values, ranges of unknown/disabled anchors and
    ranges <= 0.01 m. Merge and sort by (t, IMU before range, anchor_id) so the result does not
    depend on input order.

    IMU event:
      - before attitude init: accumulate acc/gyro; once t - t_first >= init_static_s:
        R0 = attitude_from_accel(mean acc, yaw0), bg0 = mean gyro.
      - when initialized and t > t_prev: predict(acc, gyro, t - t_prev) exactly as ESKF::predict,
        then append one output row (t, p, v, R, tag = p + R lever, p_std).
    Range event:
      - attitude not ready: ignore.
      - not initialized: keep the latest (t, r) per anchor; when >= min(init_min_anchors, n_enabled)
        anchors are newer than init_window_s, tag0 = gauss_newton(from their centroid); reject the
        candidate if non-finite, if rms residual > 3 * sig_r, or if outside the anchor box +- 2 m;
        else init ESKF at p = tag0 - R0 @ lever with R0, bg = bg0 (other states as ESKF::init).
      - initialized: ESKF::update (asymmetric Huber, gate, Joseph form, injection) and log
        (t, anchor, accepted, nis, innovation). After more than reinit_after_rejects consecutive
        refusals: drop the filter, clear the init buffer, reinits += 1.
    """
    raise NotImplementedError("E2.0-B: run_eskf_replay")
