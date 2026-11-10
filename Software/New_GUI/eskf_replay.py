"""Offline ESKF-A replay (E2.0-B equations, opt-in E2.1 extensions).

Plan: Plan/E2_0_YEU_CAU_HIEU_CHUAN_CO_CAM_BIEN_VA_REPLAY_ESKF_A.md (section 4).
Contract: tests/test_e20_eskf_replay.py. Do not loosen the tests.

Purpose: a Python mirror of the onboard filter
UP7000/estimator/uwb_eskf/include/uwb_eskf/filter.hpp (struct ESKF,
range_update, so3_exp, attitude_from_accel, gauss_newton) and of the node logic
in UP7000/estimator/uwb_eskf/src/uwb_eskf_node.cpp (static attitude init,
Gauss-Newton position init, re-init after consecutive rejections). It replays
logged IMU + UWB ranges deterministically so the onboard filter can be studied,
tuned and later checked for parity against the C++ code.

Frames: rig ENU (z up), body FLU, gravity (0, 0, -9.81). IMU acc is specific
force in body FLU (m/s^2), gyro in rad/s. PX4 FRD -> FLU: diag(1, -1, -1).
State p is the IMU/body origin; the TAG is at p + R @ lever_flu.

Equations: copy filter.hpp exactly (same F, Q, Huber asymmetry, gate, Joseph
form, injection). Body initialization is tag0 - R0 @ lever. The request
contrasted this with an older node using tag0 directly; the current workspace
node already contains the lever correction. No onboard source is changed.
The requested offline event driver assumes an initially static IMU averaging
window and omits live ROS arrival-age/configuration/publication-time guards.
E2.1 adds per-anchor rate limiting, propagated attitude carry-over on reinit,
and optional body-frame PX4 attitude Joseph updates. Defaults retain the
E2.0 numerical trajectories; version metadata is now v2.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import combinations
import math
from typing import Any, Sequence

import numpy as np

REPLAY_VERSION = "eskf-a-replay-v2"
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
    # Offline E2.1 extensions; defaults retain E2.0 dynamics and event handling.
    min_anchor_interval_s: float = 0.0
    reinit_keep_attitude: bool = False
    att_sig_deg: tuple[float, float, float] = (2.0, 2.0, 5.0)
    att_gate: float = 25.0
    att_min_interval_s: float = 0.02
    imu_average_s: float = 0.0
    attitude_init: str = "static"
    static_check: bool = False
    static_acc_tol: float = 0.8
    static_gyro_tol: float = 0.15
    imu_gap_reset_s: float = math.inf


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


@dataclass(frozen=True)
class AttitudeSample:
    t: float
    R: Any                                 # body FLU -> rig, proper orthonormal 3x3


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
    skipped_ranges: int = 0
    att_update_t: np.ndarray = field(default_factory=lambda: np.empty(0))
    att_accepted: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=bool))
    att_nis: np.ndarray = field(default_factory=lambda: np.empty(0))
    imu_gap_resets: int = 0
    attitude_inits: int = 0

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


def log_so3(R) -> np.ndarray:
    """Principal rotation vector, including stable small-angle and pi branches.

    At exactly pi the axis sign is intrinsically ambiguous; choose a deterministic
    eigenvector sign. On approach to pi use the skew part to retain its sign.
    """
    R = np.asarray(R, dtype=float)
    sine_axis = np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]]) * .5
    sine = float(np.linalg.norm(sine_axis))
    angle = math.atan2(sine, float(np.clip((np.trace(R)-1)*.5, -1., 1.)))
    if angle < 1e-9:
        return sine_axis
    if math.pi - angle < 1e-6:
        _, vectors = np.linalg.eigh((R + R.T) * .5)
        axis = vectors[:, -1]
        if (sine > 1e-14 and axis @ sine_axis < 0) or (sine <= 1e-14 and axis[np.argmax(np.abs(axis))] < 0):
            axis = -axis
        return angle * axis
    return sine_axis * (angle / sine)


def quat_to_matrix(q_wxyz) -> np.ndarray:
    """Normalized Hamilton quaternion, PX4 body FRD -> local NED."""
    q = np.asarray(q_wxyz, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not .5 < np.linalg.norm(q) < 2:
        raise ValueError("Expected a finite Hamilton quaternion with norm in (0.5, 2)")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def px4_yaw_ned(q_wxyz) -> float:
    R = quat_to_matrix(q_wxyz)
    return math.atan2(R[1, 0], R[0, 0])


def px4_attitude_to_rig(q_wxyz, rig_yaw_offset) -> np.ndarray:
    if not math.isfinite(rig_yaw_offset):
        raise ValueError("Expected a finite rig yaw offset")
    angle = rig_yaw_offset - math.pi / 2
    c, s = math.cos(angle), math.sin(angle)
    Rz = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    ned_to_enu = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
    return Rz @ ned_to_enu @ quat_to_matrix(q_wxyz) @ np.diag([1., -1., -1.])


def rig_yaw_offset_from_yaw0(q_wxyz, yaw0) -> float:
    if not math.isfinite(yaw0):
        raise ValueError("Expected a finite initial rig yaw")
    return yaw0 + px4_yaw_ned(q_wxyz)


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

    def update_attitude(self, measured_R, params):
        """Body-frame SO(3) residual and full-state Joseph/injection update."""
        residual = log_so3(self.R.T @ measured_R)
        H = np.zeros((3, 15))
        H[:, 6:9] = np.eye(3)
        Rn = np.diag(np.square(np.radians(params.att_sig_deg)))
        PH = self.P @ H.T
        S = H @ PH + Rn
        nis = float(residual @ np.linalg.solve(S, residual))
        if nis > params.att_gate:
            return False, nis
        K = np.linalg.solve(S, PH.T).T
        dx = K @ residual
        IKH = np.eye(15) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ Rn @ K.T
        self.p += dx[:3]
        self.v += dx[3:6]
        self.R = self.R @ so3_exp(dx[6:9])
        self.ba += dx[9:12]
        self.bg += dx[12:15]
        return True, nis


def _geometry_ok(xyz):
    span = float(np.max(np.ptp(xyz, axis=0)))
    threshold = max(1e-9, span ** 3 * 1e-7)
    return any(abs(np.linalg.det(np.column_stack((xyz[j] - xyz[i], xyz[k] - xyz[i], xyz[l] - xyz[i])))) > threshold
               for i, j, k, l in combinations(range(len(xyz)), 4))


def sorted_events(imu, ranges, attitude, anchor_map):
    """Finite input events ordered by time, IMU/range/attitude, ID and payload."""
    events = []
    for sample in imu:
        acc, gyro = np.asarray(sample.acc, float), np.asarray(sample.gyro, float)
        if math.isfinite(sample.t) and acc.shape == (3,) and gyro.shape == (3,) \
                and np.isfinite(acc).all() and np.isfinite(gyro).all():
            events.append((sample.t, 0, 0, tuple(acc) + tuple(gyro), sample))
    for sample in ranges:
        if math.isfinite(sample.t) and math.isfinite(sample.range_m) and sample.range_m > .01 and sample.anchor_id in anchor_map:
            events.append((sample.t, 1, sample.anchor_id, (sample.range_m,), sample))
    for sample in attitude:
        try:
            R = np.asarray(sample.R, float)
            valid = math.isfinite(sample.t) and R.shape == (3, 3) and np.isfinite(R).all() \
                and np.allclose(R.T @ R, np.eye(3), atol=1e-6, rtol=0) and np.linalg.det(R) > 0
        except (ValueError, TypeError):
            valid = False
        if valid:
            events.append((sample.t, 2, 0, tuple(R.ravel()), AttitudeSample(sample.t, R)))
    return sorted(events, key=lambda e: e[:4])


class EstimatorCore:
    """ROS-free chronological event core shared by replay and C++ parity export.

    Callbacks do not reorder input. Online averaging emits a bin's mean at the
    arrival of the first raw sample in the next bin; no final-bin flush. Rows
    carry the mean sample time, while ranges/attitudes keep their arrival order.
    reset_attitude retains the latest external R but requires a fresh attitude
    callback after at least one raw IMU before external initialization can resume.
    """

    def __init__(self, anchors, lever_flu, yaw0_rad, params=EskfParams()):
        self.lever = np.asarray(lever_flu, float)
        if self.lever.shape != (3,) or not np.isfinite(self.lever).all() or not math.isfinite(yaw0_rad):
            raise ValueError("Expected a finite FLU lever and initial rig yaw")
        if not np.isfinite([params.sig_r, params.gate, params.k_pos, params.k_neg]).all() \
                or min(params.sig_r, params.gate, params.k_pos, params.k_neg) <= 0:
            raise ValueError("Range noise, gate and Huber thresholds must be positive")
        if not math.isfinite(params.min_anchor_interval_s) or params.min_anchor_interval_s < 0:
            raise ValueError("Expected a finite nonnegative per-anchor range interval")
        if not math.isfinite(params.att_min_interval_s) or params.att_min_interval_s < 0 \
                or not math.isfinite(params.att_gate) or params.att_gate <= 0 \
                or np.shape(params.att_sig_deg) != (3,) or not np.isfinite(params.att_sig_deg).all() or min(params.att_sig_deg) <= 0:
            raise ValueError("Expected finite positive attitude sigmas/gate and nonnegative interval")
        if not math.isfinite(params.imu_average_s) or params.imu_average_s < 0:
            raise ValueError("Expected a finite nonnegative IMU averaging interval")
        if params.attitude_init not in ("static", "external"):
            raise ValueError("attitude_init must be static or external")
        if math.isnan(params.imu_gap_reset_s) or params.imu_gap_reset_s <= 0:
            raise ValueError("Expected a positive IMU gap reset threshold (infinity disables it)")
        if not np.isfinite([params.static_acc_tol, params.static_gyro_tol]).all() \
                or min(params.static_acc_tol, params.static_gyro_tol) < 0:
            raise ValueError("Static tolerances must be finite and nonnegative")
        self.params, self.yaw0 = params, yaw0_rad
        self.anchors = {a.anchor_id: np.array((a.x, a.y, a.z), float) for a in anchors
                        if getattr(a, 'enabled', True) and np.isfinite((a.x, a.y, a.z)).all()}
        self.enabled_xyz = np.asarray(list(self.anchors.values()), float).reshape((-1, 3))
        self.minimum = min(params.init_min_anchors, len(self.anchors))
        self.rows, self.updates, self.att_updates = [], [], []
        self.filter = None
        self.init_time = None
        self.reinits = self.skipped_ranges = self.imu_gap_resets = self.attitude_inits = 0
        self.initializations = self.accepted_ranges = self.accepted_attitudes = self.rejected_attitudes = 0
        self.last_range, self.last_attitude = {}, -math.inf
        self.external_R = None
        self.last_update = None
        self._clear_attitude_state()

    def _clear_static(self):
        self.first_imu = None
        self.acc_sum, self.gyro_sum, self.count = np.zeros(3), np.zeros(3), 0

    def _clear_attitude_state(self):
        self.filter = None
        self.attitude_ready = False
        self.R0, self.bg0 = np.eye(3), np.zeros(3)
        self.held_R = self.held_bg = None
        self.raw_previous = self.previous_imu = None
        self.average_origin = self.average_bin = None
        self.average_count = 0
        self.average_t = 0.
        self.average_acc, self.average_gyro = np.zeros(3), np.zeros(3)
        self.latest, self.rejected_run = {}, 0
        self._clear_static()

    def reset_attitude(self, imu_gap=True):
        self._clear_attitude_state()
        self.last_update = None
        if imu_gap:
            self.imu_gap_resets += 1

    def reset_position(self):
        if self.filter is not None and self.params.reinit_keep_attitude:
            self.held_R, self.held_bg = self.filter.R.copy(), self.filter.bg.copy()
        self.filter = None
        self.latest.clear()
        self.rejected_run = 0

    def on_imu(self, t, acc, gyro):
        self.last_update = None
        acc, gyro = np.asarray(acc, float), np.asarray(gyro, float)
        if not math.isfinite(t) or acc.shape != (3,) or gyro.shape != (3,) \
                or not np.isfinite(acc).all() or not np.isfinite(gyro).all():
            return
        if self.raw_previous is not None and t - self.raw_previous > self.params.imu_gap_reset_s:
            self.reset_attitude()
        self.raw_previous = t
        width = self.params.imu_average_s
        if width > 0:
            if self.average_origin is None:
                self.average_origin = t
            bin_ = math.floor((t - self.average_origin) / width + 1e-9)
            if self.average_bin is not None and bin_ != self.average_bin:
                n = self.average_count
                self._process_imu(self.average_t / n, self.average_acc / n, self.average_gyro / n)
                self.average_count, self.average_t = 0, 0.
                self.average_acc, self.average_gyro = np.zeros(3), np.zeros(3)
            self.average_bin = bin_
            self.average_t += t
            self.average_acc += acc
            self.average_gyro += gyro
            self.average_count += 1
        else:
            self._process_imu(t, acc, gyro)

    def _process_imu(self, t, acc, gyro):
        if self.first_imu is None:
            self.first_imu = t
        if not self.attitude_ready:
            if self.params.attitude_init == "static":
                if self.params.static_check and (abs(np.linalg.norm(acc) - 9.81) > self.params.static_acc_tol
                                                 or np.linalg.norm(gyro) > self.params.static_gyro_tol):
                    self._clear_static()
                else:
                    self.acc_sum += acc
                    self.gyro_sum += gyro
                    self.count += 1
                    if t - self.first_imu >= self.params.init_static_s:
                        self.R0 = attitude_from_accel(self.acc_sum / self.count, self.yaw0)
                        self.bg0 = self.gyro_sum / self.count
                        self.attitude_ready = True
                        self.attitude_inits += 1
        elif self.filter is not None and self.previous_imu is not None and t > self.previous_imu:
            self.filter.predict(acc, gyro, t - self.previous_imu, self.params)
            f = self.filter
            self.rows.append((t, f.p.copy(), f.v.copy(), f.R.copy(), f.p + f.R @ self.lever,
                              np.sqrt(np.maximum(np.diag(f.P)[:3], 0.))))
        elif self.held_R is not None and self.previous_imu is not None and t > self.previous_imu:
            self.held_R = self.held_R @ so3_exp((gyro - self.held_bg) * (t - self.previous_imu))
        self.previous_imu = t

    def on_range(self, aid, t, measured):
        self.last_update = None
        if aid not in self.anchors or not math.isfinite(t) or not math.isfinite(measured) or measured <= .01:
            return
        if t - self.last_range.get(aid, -math.inf) < self.params.min_anchor_interval_s - 1e-9:
            self.skipped_ranges += 1
            return
        self.last_range[aid] = t
        if not self.attitude_ready:
            return
        if self.filter is None:
            self.latest[aid] = (t, measured)
            recent = sorted(a for a, (stamp, _) in self.latest.items() if 0 <= t-stamp < self.params.init_window_s)
            if len(recent) < max(4, self.minimum):
                return
            xyz = np.array([self.anchors[a] for a in recent])
            if not _geometry_ok(xyz):
                return
            ranges = np.array([self.latest[a][1] for a in recent])
            tag0 = gauss_newton(xyz, ranges, xyz.mean(axis=0))
            rms = float(np.sqrt(np.mean((np.linalg.norm(tag0-xyz, axis=1)-ranges)**2)))
            if not np.isfinite(tag0).all() or rms > 3*self.params.sig_r \
                    or np.any(tag0 < self.enabled_xyz.min(axis=0)-2) or np.any(tag0 > self.enabled_xyz.max(axis=0)+2):
                return
            if self.params.attitude_init == "external":
                R, bg = self.external_R, self.held_bg if self.held_bg is not None else np.zeros(3)
            else:
                R, bg = (self.held_R, self.held_bg) if self.held_R is not None else (self.R0, self.bg0)
            self.filter = ESKF(tag0-R @ self.lever, R, bg, self.params)
            if self.init_time is None:
                self.init_time = float(t)
            self.initializations += 1
            self.rejected_run = 0
        else:
            accepted, nis, innovation = self.filter.update(self.anchors[aid], measured, self.lever, self.params)
            self.updates.append((t, aid, accepted, nis, innovation))
            self.last_update = (1, t, aid, accepted, nis, innovation)
            self.accepted_ranges += int(accepted)
            self.rejected_run = 0 if accepted else self.rejected_run + 1
            if self.rejected_run > self.params.reinit_after_rejects:
                self.reset_position()
                self.reinits += 1

    def on_attitude(self, t, R):
        self.last_update = None
        R = np.asarray(R, float)
        if not math.isfinite(t) or R.shape != (3, 3) or not np.isfinite(R).all() \
                or not np.allclose(R.T @ R, np.eye(3), atol=1e-6, rtol=0) or np.linalg.det(R) <= 0:
            return
        if self.params.attitude_init == "external":
            self.external_R = R.copy()
            if not self.attitude_ready and self.raw_previous is not None:
                self.attitude_ready = True
                self.attitude_inits += 1
        if self.filter is not None and t - self.last_attitude >= self.params.att_min_interval_s - 1e-9:
            self.last_attitude = t
            accepted, nis = self.filter.update_attitude(R, self.params)
            self.att_updates.append((t, accepted, nis))
            self.last_update = (2, t, 0, accepted, nis, float('nan'))
            self.accepted_attitudes += int(accepted)
            self.rejected_attitudes += int(not accepted)

    def result(self):
        rows, updates, att_updates = self.rows, self.updates, self.att_updates
        return ReplayResult(
            np.array([r[0] for r in rows]), np.array([r[1] for r in rows]).reshape((-1, 3)),
            np.array([r[2] for r in rows]).reshape((-1, 3)), np.array([r[3] for r in rows]).reshape((-1, 3, 3)),
            np.array([r[4] for r in rows]).reshape((-1, 3)), np.array([r[5] for r in rows]).reshape((-1, 3)),
            np.array([u[0] for u in updates]), np.array([u[1] for u in updates], dtype=int),
            np.array([u[2] for u in updates], dtype=bool), np.array([u[3] for u in updates]),
            np.array([u[4] for u in updates]), self.init_time, self.reinits, skipped_ranges=self.skipped_ranges,
            att_update_t=np.array([u[0] for u in att_updates]),
            att_accepted=np.array([u[1] for u in att_updates], dtype=bool), att_nis=np.array([u[2] for u in att_updates]),
            imu_gap_resets=self.imu_gap_resets, attitude_inits=self.attitude_inits)


def run_eskf_replay(anchors: Sequence[Any], imu: Sequence[ImuSample], ranges: Sequence[RangeSample],
                    lever_flu, yaw0_rad: float, params: EskfParams = EskfParams(),
                    attitude: Sequence[AttitudeSample] = ()) -> ReplayResult:
    """Replay sorted events through the same chronological core used by parity.

    Defaults retain E2.0/E2.1 trajectories. Opt-in E2.2 parameters enable online
    bin averaging, the actual static-start rule, external attitude initialization
    and recovery after raw-IMU gaps. The final open averaging bin is not flushed.
    """
    core = EstimatorCore(anchors, lever_flu, yaw0_rad, params)
    for t, kind, _, _, sample in sorted_events(imu, ranges, attitude, core.anchors):
        if kind == 0:
            core.on_imu(t, sample.acc, sample.gyro)
        elif kind == 1:
            core.on_range(sample.anchor_id, t, sample.range_m)
        else:
            core.on_attitude(t, sample.R)
    return core.result()
