"""Robust range-only CV EKF. All times are device seconds, distances metres.

Each radio observation is consumed once. NIS gating uses the common prior
(before the batch update), so anchor order cannot decide which anchor wins.
The filter deliberately does not learn a persistent range bias as white noise.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Sequence

import numpy as np
from radio_quality import nlos_variance_scale


FILTER_VERSION = "robust-range-ekf-v2-radio-quality"


@dataclass(frozen=True)
class EkfSettings:
    acceleration_noise: float = 2.0   # continuous acceleration PSD, m^2/s^3
    sigma_floor_m: float = 0.035
    huber_sigma: float = 2.5
    reject_sigma: float = 5.0
    max_prediction_s: float = 0.35
    reacquire_after_s: float = 0.8
    recovery_s: float = 0.4
    # A full fix needs dimension+1 distinct anchors accepted within this window,
    # not all in one Burst: with heavy packet loss each cycle may carry only
    # one or two ranges, yet sequential updates still observe the position.
    full_fix_window_s: float = 0.2


@dataclass(frozen=True)
class RangeObservation:
    anchor_id: int
    xyz: tuple[float, float, float]
    range_m: float
    sigma_m: float
    nlos_db: float | None = None
    quality_scale: float = 1.0  # extra R inflation only; never loosens NIS gate


class RobustRangeEkf:
    def __init__(self, settings: EkfSettings | None = None) -> None:
        self.settings = settings or EkfSettings()
        self.initialization_count = 0
        self.reset()

    def reset(self) -> None:
        self.x: np.ndarray | None = None
        self.p: np.ndarray | None = None
        self.last_time_s: float | None = None
        self.last_good_s: float | None = None
        self.dimension = 3
        self.fixed_z = 0.0
        self.health: dict[int, float] = {}
        self.innovations_m: dict[int, float] = {}
        self.nis: dict[int, float] = {}
        self.rejected: dict[int, str] = {}
        self.accepted_ids: tuple[int, ...] = ()
        self.last_accept_s: dict[int, float] = {}
        self.recent_ids: tuple[int, ...] = ()
        self.last_dt = 0.0

    def metadata(self) -> dict:
        return {"version": FILTER_VERSION, **asdict(self.settings)}

    def initialize(self, xyz: Sequence[float], time_s: float, dimension: int, fixed_z: float) -> None:
        self.reset()
        self.initialization_count += 1
        self.dimension, self.fixed_z = dimension, fixed_z
        self.x = np.array([*xyz, 0.0, 0.0, 0.0], dtype=float)
        self.p = np.diag([0.08**2] * 3 + [1.0] * 3)
        self.last_time_s = self.last_good_s = time_s
        self._constrain_2d()

    def _constrain_2d(self) -> None:
        if self.dimension == 2 and self.x is not None and self.p is not None:
            self.x[2], self.x[5] = self.fixed_z, 0.0
            self.p[[2, 5], :] = 0.0
            self.p[:, [2, 5]] = 0.0

    def predict(self, time_s: float) -> bool:
        """False means bootstrap is needed; duplicate/old times do not advance state."""
        if self.x is None or self.last_time_s is None:
            return False
        dt = time_s - self.last_time_s
        if dt < -1e-9 or dt > self.settings.reacquire_after_s:
            self.reset()
            return False
        if self.last_good_s is not None and time_s - self.last_good_s > self.settings.reacquire_after_s:
            self.reset()
            return False
        self.last_dt = max(dt, 0.0)
        self.last_time_s = time_s
        if dt <= 0.0:
            return True
        assert self.p is not None
        f = np.eye(6)
        f[:3, 3:] = np.eye(3) * dt
        q = np.zeros((6, 6))
        # Continuous white acceleration model; no dt clamping or host-clock gain.
        density = self.settings.acceleration_noise
        q[:3, :3] = np.eye(3) * density * dt**3 / 3.0
        q[:3, 3:] = q[3:, :3] = np.eye(3) * density * dt**2 / 2.0
        q[3:, 3:] = np.eye(3) * density * dt
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q
        self._constrain_2d()
        return True

    def update(self, observations: Sequence[RangeObservation], time_s: float) -> None:
        assert self.x is not None and self.p is not None
        self.innovations_m, self.nis, self.rejected = {}, {}, {}
        prior = self.x.copy()
        accepted = []
        for obs in observations:
            aid = obs.anchor_id
            if not math.isfinite(obs.range_m) or obs.range_m <= 0.01:
                self.rejected[aid] = "invalid range"
                continue
            delta = prior[:3] - np.asarray(obs.xyz)
            dist = max(float(np.linalg.norm(delta)), 1e-6)
            h = np.zeros(6)
            h[:3] = delta / dist
            if self.dimension == 2:
                h[2] = 0.0
            innovation = obs.range_m - dist
            sigma = max(obs.sigma_m, self.settings.sigma_floor_m)
            nlos_scale = nlos_variance_scale(obs.nlos_db)
            variance = sigma**2 * nlos_scale
            s = max(float(h @ self.p @ h) + variance, 1e-9)
            nis = innovation**2 / s
            self.innovations_m[aid], self.nis[aid] = innovation, nis
            health = self.health.get(aid, 1.0)
            if nis > self.settings.reject_sigma**2:
                self.health[aid] = max(0.15, health * 0.75)
                self.rejected[aid] = "innovation gate"
                continue
            # Health recovers in seconds, not samples. Gate above uses nominal
            # variance so prolonged rejection cannot itself make a bad link pass.
            health += (1.0 - math.exp(-self.last_dt / self.settings.recovery_s)) * (1.0 - health)
            self.health[aid] = health
            weight = min(1.0, self.settings.huber_sigma / max(math.sqrt(nis), 1e-6))
            quality_scale = min(36.0, max(1.0, obs.quality_scale)) if math.isfinite(obs.quality_scale) else 1.0
            accepted.append((obs, h, innovation, variance * quality_scale / max(health * weight**2, 0.01)))

        self.accepted_ids = tuple(sorted(o.anchor_id for o, *_ in accepted))
        if accepted:
            # Sequential *linear* updates about the common prior. Equivalent to
            # one batch update, with scalar inversions and Joseph covariance.
            identity = np.eye(6)
            for _obs, h, innovation, variance in accepted:
                innovation -= float(h @ (self.x - prior))
                ph = self.p @ h
                s = float(h @ ph) + variance
                gain = ph / s
                self.x += gain * innovation
                ikh = identity - np.outer(gain, h)
                self.p = ikh @ self.p @ ikh.T + variance * np.outer(gain, gain)
            self.p = (self.p + self.p.T) * 0.5
            for obs, *_ in accepted:
                self.last_accept_s[obs.anchor_id] = time_s
        window = self.settings.full_fix_window_s
        self.recent_ids = tuple(sorted(
            aid for aid, t in self.last_accept_s.items() if 0.0 <= time_s - t <= window
        ))
        # dimension+1 distinct anchors accepted recently support a full fix.
        if accepted and len(self.recent_ids) >= self.dimension + 1:
            self.last_good_s = time_s
        self._constrain_2d()

    @property
    def prediction_age_s(self) -> float:
        if self.last_time_s is None or self.last_good_s is None:
            return math.inf
        return max(0.0, self.last_time_s - self.last_good_s)

    @property
    def position_std_m(self) -> tuple[float, float, float]:
        if self.p is None:
            return (math.inf,) * 3
        return tuple(float(math.sqrt(max(self.p[i, i], 0.0))) for i in range(3))
