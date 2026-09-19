"""Robust constant-velocity range filter for the UART debugger.

The estimator follows the position/velocity idea used by ``CV_KALMAN_V2`` in
``STM32_UWB`` while keeping reacquisition continuous. Median-3 removes an
isolated radio spike, an alpha-beta update tracks real drone motion without a
large Kalman deadband, and every correction is bounded by the configured
radial-speed envelope. There is deliberately no range ceiling and no snap.
"""

from __future__ import annotations

from dataclasses import dataclass, field


HOST_FILTER_VERSION = "median3-cv-alpha-beta-v3"

# Constant-velocity alpha-beta gains at the nominal 50 Hz update rate.
POSITION_GAIN = 0.30
VELOCITY_GAIN = 0.04
VELOCITY_DAMPING = 0.95

# Weak FPP should add a little smoothing, never freeze the estimator. The old
# scalar Kalman mapped this region to R=10000 with Q=0.05, producing K~=0.002.
WEAK_FPP_DBM = -95.0
WEAK_POSITION_GAIN_SCALE = 0.90

MAX_RADIAL_SPEED_MM_S = 10_000.0
GATE_MARGIN_UP_MM = 100.0
GATE_MARGIN_DOWN_MM = 250.0
QUIET_RESIDUAL_MM = 30.0
QUIET_VELOCITY_DAMPING = 0.80
MIN_RANGE_MM = 1.0

# Six missing 50 Hz samples are enough to make the old velocity unsafe. Reset
# only velocity/median after that gap; position remains continuous.
STALE_RESET_MS = 120
NOMINAL_SAMPLE_MS = 20
MIN_TRACKING_DT_MS = 10
MAX_TRACKING_DT_MS = 100


@dataclass
class Median3:
    values: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    index: int = 0
    count: int = 0

    def reset(self) -> None:
        self.values[:] = (0.0, 0.0, 0.0)
        self.index = 0
        self.count = 0

    def update(self, value: float) -> float:
        self.values[self.index] = value
        self.index = (self.index + 1) % 3
        self.count = min(self.count + 1, 3)
        if self.count < 3:
            return value
        return sorted(self.values)[1]


@dataclass
class HostRangeFilter:
    """Median-3 plus a continuous constant-velocity alpha-beta estimator."""

    x: float = 0.0
    velocity_mm_s: float = 0.0
    initialized: bool = False
    median: Median3 = field(default_factory=Median3)
    last_sample_ms: int | None = None
    synthetic_now_ms: int = 0

    stale_reacquire_count: int = 0
    clipped_innovation_count: int = 0

    @staticmethod
    def _elapsed_ms(now_ms: int, then_ms: int) -> int:
        return (now_ms - then_ms) & 0xFFFFFFFF

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(value, maximum))

    def _resolve_now_ms(self, now_ms: int | None) -> int:
        if now_ms is None:
            self.synthetic_now_ms = (
                self.synthetic_now_ms + NOMINAL_SAMPLE_MS
            ) & 0xFFFFFFFF
        else:
            self.synthetic_now_ms = int(now_ms) & 0xFFFFFFFF
        return self.synthetic_now_ms

    @staticmethod
    def _time_scaled_gain(nominal_gain: float, dt_scale: float) -> float:
        """Keep the same time response when telemetry periods vary."""

        return 1.0 - (1.0 - nominal_gain) ** dt_scale

    def update(
        self, raw_mm: int, fpp_cdbm: int, now_ms: int | None = None
    ) -> int:
        """Consume one range sample and return a bounded, continuous estimate."""

        now = self._resolve_now_ms(now_ms)
        if not self.initialized:
            measurement = self.median.update(float(raw_mm))
            self.x = max(MIN_RANGE_MM, measurement)
            self.velocity_mm_s = 0.0
            self.last_sample_ms = now
            self.initialized = True
            return round(self.x)

        assert self.last_sample_ms is not None
        elapsed_ms = self._elapsed_ms(now, self.last_sample_ms)
        self.last_sample_ms = now

        if elapsed_ms > STALE_RESET_MS:
            # Preserve the published distance across a radio gap. Only the
            # velocity and median history are stale; resetting x would snap.
            self.median.reset()
            self.velocity_mm_s = 0.0
            elapsed_ms = NOMINAL_SAMPLE_MS
            self.stale_reacquire_count += 1
        else:
            elapsed_ms = int(
                self._clamp(
                    elapsed_ms,
                    MIN_TRACKING_DT_MS,
                    MAX_TRACKING_DT_MS,
                )
            )

        measurement = self.median.update(float(raw_mm))
        dt_s = elapsed_ms / 1000.0
        dt_scale = elapsed_ms / NOMINAL_SAMPLE_MS
        previous_x = self.x

        predicted_x = max(
            MIN_RANGE_MM,
            self.x + self.velocity_mm_s * dt_s,
        )
        innovation_mm = measurement - predicted_x
        up_limit_mm = MAX_RADIAL_SPEED_MM_S * dt_s + GATE_MARGIN_UP_MM
        down_limit_mm = MAX_RADIAL_SPEED_MM_S * dt_s + GATE_MARGIN_DOWN_MM
        bounded_innovation_mm = self._clamp(
            innovation_mm,
            -down_limit_mm,
            up_limit_mm,
        )
        if bounded_innovation_mm != innovation_mm:
            self.clipped_innovation_count += 1

        nominal_position_gain = POSITION_GAIN
        if fpp_cdbm / 100.0 <= WEAK_FPP_DBM:
            nominal_position_gain *= WEAK_POSITION_GAIN_SCALE
        position_gain = self._time_scaled_gain(
            nominal_position_gain,
            dt_scale,
        )
        velocity_gain = self._time_scaled_gain(VELOCITY_GAIN, dt_scale)

        proposed_x = predicted_x + position_gain * bounded_innovation_mm
        max_output_delta_mm = MAX_RADIAL_SPEED_MM_S * dt_s
        self.x = self._clamp(
            proposed_x,
            previous_x - max_output_delta_mm,
            previous_x + max_output_delta_mm,
        )
        self.x = max(MIN_RANGE_MM, self.x)

        self.velocity_mm_s *= VELOCITY_DAMPING**dt_scale
        self.velocity_mm_s += (
            velocity_gain / dt_s
        ) * bounded_innovation_mm
        self.velocity_mm_s = self._clamp(
            self.velocity_mm_s,
            -MAX_RADIAL_SPEED_MM_S,
            MAX_RADIAL_SPEED_MM_S,
        )

        if abs(innovation_mm) < QUIET_RESIDUAL_MM:
            self.velocity_mm_s *= QUIET_VELOCITY_DAMPING**dt_scale
        if self.x <= MIN_RANGE_MM and self.velocity_mm_s < 0.0:
            self.velocity_mm_s = 0.0

        return round(self.x)
