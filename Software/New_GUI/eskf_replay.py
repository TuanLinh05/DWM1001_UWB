"""Offline ESKF-A replay (E2.0-B).

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
from itertools import combinations
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
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def so3_exp(phi) -> np.ndarray:
    """Rodrigues; I + skew(phi) below 1e-9 rad, as filter.hpp."""
    phi = np.asarray(phi, dtype=float)
    angle = float(np.linalg.norm(phi))
    if angle < 1e-9:
        return np.eye(3) + skew(phi)
    k = skew(phi / angle)
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def attitude_from_accel(f, yaw: float) -> np.ndarray:
    """filter.hpp attitude_from_accel: roll = atan2(f_y, f_z), pitch = atan2(-f_x, hypot(f_y, f_z)),
    R = Rz(yaw) Ry(pitch) Rx(roll)."""
    x, y, z = f
    roll, pitch = math.atan2(y, z), math.atan2(-x, math.hypot(y, z))
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]])
    Ry = np.array([[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]])
    Rz = np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]])
    return Rz @ Ry @ Rx


def gauss_newton(anchors_xyz: np.ndarray, ranges: np.ndarray, p0: np.ndarray, iters: int = 10) -> np.ndarray:
    positions, observations = np.asarray(anchors_xyz, dtype=float), np.asarray(ranges, dtype=float)
    p = np.array(p0, dtype=float, copy=True)
    for _ in range(iters):
        delta = p - positions
        distance = np.linalg.norm(delta, axis=1)
        if np.any(distance < 1e-12):
            return np.full(3, np.nan)
        p -= np.linalg.lstsq(delta / distance[:, None], distance - observations, rcond=None)[0]
    return p


def range_update(P, H, innovation, r_var, params):
    """Scalar asymmetric Huber update, gate first, Joseph covariance as filter.hpp.

    Returns (accepted, updated_covariance, error_state, original_NIS).
    Rejected measurements leave the input covariance unchanged.
    """
    PH = P @ H
    HPH = float(H @ PH)
    S = HPH + r_var
    nis = float(innovation * innovation / S)
    if nis > params.gate:
        return False, P, np.zeros(len(H)), nis
    z = math.sqrt(nis)
    threshold = params.k_pos if innovation > 0 else params.k_neg
    if z > threshold:
        r_var = S * z / threshold - HPH
        S = HPH + r_var
    K = PH / S
    dx = K * innovation
    IKH = np.eye(len(H)) - np.outer(K, H)
    updated = IKH @ P @ IKH.T + r_var * np.outer(K, K)
    return True, updated, dx, nis


class ESKF:
    """15-state filter.hpp mirror; body position, velocity, attitude and IMU biases."""

    def __init__(self, p0, R0, bg0, params):
        self.p = np.array(p0, dtype=float, copy=True)
        self.v = np.zeros(3)
        self.R = np.array(R0, dtype=float, copy=True)
        self.ba = np.zeros(3)
        self.bg = np.array(bg0, dtype=float, copy=True)
        variances = np.r_[np.full(3, params.sig_p0 ** 2), np.full(3, params.sig_v0 ** 2),
                           np.square(np.radians(params.sig_th0_deg)), np.full(3, params.sig_ba0 ** 2),
                           np.full(3, math.radians(params.sig_bg0_deg) ** 2)]
        self.P = np.diag(variances)

    def predict(self, acc_measured, gyro_measured, dt, params):
        a, w = np.asarray(acc_measured) - self.ba, np.asarray(gyro_measured) - self.bg
        acceleration = self.R @ a + GRAVITY
        F = np.eye(15)
        F[0:3, 3:6] = dt * np.eye(3)
        F[3:6, 6:9] = -self.R @ skew(a) * dt
        F[3:6, 9:12] = -self.R * dt
        F[6:9, 6:9] = so3_exp(-w * dt)
        F[6:9, 12:15] = -np.eye(3) * dt
        q = np.repeat([0., params.na ** 2 * dt, params.ng ** 2 * dt,
                       params.nba ** 2 * dt, params.nbg ** 2 * dt], 3)
        self.p = self.p + self.v * dt + 0.5 * acceleration * dt * dt
        self.v = self.v + acceleration * dt
        self.R = self.R @ so3_exp(w * dt)
        self.P = F @ self.P @ F.T + np.diag(q)

    def update(self, anchor_xyz, measured_range, lever, params):
        delta = self.p + self.R @ lever - anchor_xyz
        distance = float(np.linalg.norm(delta))
        if not math.isfinite(distance) or distance < 1e-6:
            return False, 0.0, float("nan")
        u = delta / distance
        H = np.zeros(15)
        H[:3] = u
        H[6:9] = u @ (-self.R @ skew(lever))
        innovation = float(measured_range - distance)
        accepted, covariance, dx, nis = range_update(self.P, H, innovation, params.sig_r ** 2, params)
        if accepted:
            self.P = covariance
            self.p += dx[:3]
            self.v += dx[3:6]
            self.R = self.R @ so3_exp(dx[6:9])
            self.ba += dx[9:12]
            self.bg += dx[12:15]
        return accepted, nis, innovation


def _geometry_ok(xyz):
    span = float(np.max(np.ptp(xyz, axis=0)))
    threshold = max(1e-9, span ** 3 * 1e-7)
    return any(abs(np.linalg.det(np.column_stack((xyz[j] - xyz[i], xyz[k] - xyz[i], xyz[l] - xyz[i])))) > threshold
               for i, j, k, l in combinations(range(len(xyz)), 4))


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
    lever = np.asarray(lever_flu, dtype=float)
    if lever.shape != (3,) or not np.all(np.isfinite(lever)) or not math.isfinite(yaw0_rad):
        raise ValueError("Expected a finite FLU lever and initial rig yaw")
    if params.sig_r <= 0 or params.gate <= 0 or params.k_pos <= 0 or params.k_neg <= 0:
        raise ValueError("Range noise, gate and Huber thresholds must be positive")
    anchor_map = {a.anchor_id: np.array((a.x, a.y, a.z), dtype=float) for a in anchors
                  if getattr(a, "enabled", True) and np.all(np.isfinite((a.x, a.y, a.z)))}
    events = []
    for sample in imu:
        acc, gyro = np.asarray(sample.acc, dtype=float), np.asarray(sample.gyro, dtype=float)
        if math.isfinite(sample.t) and acc.shape == (3,) and gyro.shape == (3,) \
                and np.all(np.isfinite(acc)) and np.all(np.isfinite(gyro)):
            events.append((sample.t, 0, 0, tuple(acc) + tuple(gyro), sample))
    for sample in ranges:
        if math.isfinite(sample.t) and math.isfinite(sample.range_m) and sample.range_m > 0.01 and sample.anchor_id in anchor_map:
            events.append((sample.t, 1, sample.anchor_id, (sample.range_m,), sample))
    events.sort(key=lambda e: e[:4])
    filter_ = None
    first_imu, previous_imu, init_time = None, None, None
    acc_sum, gyro_sum, count = np.zeros(3), np.zeros(3), 0
    attitude_ready = False
    R0, bg0 = np.eye(3), np.zeros(3)
    latest = {}
    rejected_run = reinits = 0
    rows, updates = [], []
    enabled_xyz = np.asarray(list(anchor_map.values()), dtype=float).reshape((-1, 3))
    minimum = min(params.init_min_anchors, len(anchor_map))
    for time_, kind, _, _, sample in events:
        if kind == 0:
            if first_imu is None:
                first_imu = time_
            if not attitude_ready:
                acc_sum += sample.acc
                gyro_sum += sample.gyro
                count += 1
                if time_ - first_imu >= params.init_static_s:
                    R0 = attitude_from_accel(acc_sum / count, yaw0_rad)
                    bg0 = gyro_sum / count
                    attitude_ready = True
            elif filter_ is not None and previous_imu is not None and time_ > previous_imu:
                filter_.predict(sample.acc, sample.gyro, time_ - previous_imu, params)
                rows.append((time_, filter_.p.copy(), filter_.v.copy(), filter_.R.copy(),
                             filter_.p + filter_.R @ lever, np.sqrt(np.maximum(np.diag(filter_.P)[:3], 0.0))))
            previous_imu = time_
        elif attitude_ready:
            if filter_ is None:
                latest[sample.anchor_id] = (time_, sample.range_m)
                recent = sorted(a for a, (t, _) in latest.items() if 0 <= time_ - t < params.init_window_s)
                if len(recent) < max(4, minimum):
                    continue
                xyz = np.array([anchor_map[a] for a in recent])
                if not _geometry_ok(xyz):
                    continue
                measured = np.array([latest[a][1] for a in recent])
                tag0 = gauss_newton(xyz, measured, xyz.mean(axis=0))
                rms = float(np.sqrt(np.mean((np.linalg.norm(tag0 - xyz, axis=1) - measured) ** 2)))
                if not np.all(np.isfinite(tag0)) or rms > 3 * params.sig_r \
                        or np.any(tag0 < enabled_xyz.min(axis=0) - 2) or np.any(tag0 > enabled_xyz.max(axis=0) + 2):
                    continue
                filter_ = ESKF(tag0 - R0 @ lever, R0, bg0, params)
                if init_time is None:
                    init_time = float(time_)
                rejected_run = 0
            else:
                accepted, nis, innovation = filter_.update(anchor_map[sample.anchor_id], sample.range_m, lever, params)
                updates.append((time_, sample.anchor_id, accepted, nis, innovation))
                rejected_run = 0 if accepted else rejected_run + 1
                if rejected_run > params.reinit_after_rejects:
                    filter_ = None
                    latest.clear()
                    reinits += 1
                    rejected_run = 0
    return ReplayResult(
        np.array([r[0] for r in rows]), np.array([r[1] for r in rows]).reshape((-1, 3)),
        np.array([r[2] for r in rows]).reshape((-1, 3)), np.array([r[3] for r in rows]).reshape((-1, 3, 3)),
        np.array([r[4] for r in rows]).reshape((-1, 3)), np.array([r[5] for r in rows]).reshape((-1, 3)),
        np.array([u[0] for u in updates]), np.array([u[1] for u in updates], dtype=int),
        np.array([u[2] for u in updates], dtype=bool), np.array([u[3] for u in updates]),
        np.array([u[4] for u in updates]), init_time, reinits)
