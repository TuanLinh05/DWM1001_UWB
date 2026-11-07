"""Conservative radio quality heuristics; scores are NOT LOS probabilities.

Power, first-path amplitudes and CIR power are correlated. Use the worst
channel indicator rather than multiplying them as independent evidence.
Raw diagnostics are slow (normally 10 Hz); only past, valid, same-boot
samples may influence later ranges, and expire on the device clock.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path


QUALITY_VERSION = "radio-quality-v1"


def nlos_variance_scale(gap_db: float | None) -> float:
    if gap_db is None or not math.isfinite(gap_db):
        return 1.0
    return 1.0 + 0.15 * max(0.0, min(gap_db - 6.0, 12.0)) ** 2


@dataclass(frozen=True)
class QualitySettings:
    diagnostic_max_age_s: float = 0.30
    recovery_tau_s: float = 0.25
    fp_warn_dbm: float = -92.0       # corrected DW1000 power scale
    fp_penalty_step_db: float = 4.0
    fp_noise_warn_db: float = 14.0   # amplitude/noise proxy, not calibrated SNR
    clock_step_warn_ppm: float = 3.0
    max_variance_scale: float = 36.0


@dataclass(frozen=True)
class QualityResult:
    score: float | None = None
    label: str = "UNKNOWN"
    variance_scale: float = 1.0
    extra_variance_scale: float = 1.0
    reasons: tuple[str, ...] = ()
    features: tuple[str, ...] = ()
    diagnostic_age_s: float | None = None
    fp_noise_db: float | None = None


class RadioQualityModel:
    def __init__(self, settings: QualitySettings | None = None) -> None:
        self.settings = settings or QualitySettings()
        self.reset()

    def reset(self) -> None:
        self.samples: dict[int, tuple] = {}
        self.clock: dict[int, tuple[float, float]] = {}
        self.recovery: dict[int, tuple[float, float, bool]] = {}

    def metadata(self) -> dict:
        return {"version": QUALITY_VERSION, "score_is_probability": False,
                "bias_correction": False, **asdict(self.settings)}

    def ingest(self, message, current_boot: int | None) -> bool:
        if current_boot is not None and message.boot_id != current_boot:
            return False
        if message.prf_mhz not in (16, 64):
            return False
        t = message.t_us / 1e6
        for sample in message.anchors:
            old = self.samples.get(sample.anchor_id)
            if old is None or old[0] != message.boot_id or t > old[1]:
                self.samples[sample.anchor_id] = (message.boot_id, t, sample)
        return True

    def assess(self, aid: int, boot: int | None, time_s: float, *,
               fp_dbm: float | None, nlos_db: float | None, ci_ppm: float | None,
               corrected_power: bool = True, nlos_enabled: bool = True,
               late: bool = False) -> QualityResult:
        cfg = self.settings
        reasons, features = [], []
        # Old firmware power scale is ~12 dB lower. Without the corrected flag,
        # absolute power thresholds are disabled; RX-FP differences remain valid.
        baseline = nlos_variance_scale(nlos_db) if nlos_enabled else 1.0
        channel = baseline
        if nlos_enabled and nlos_db is not None and math.isfinite(nlos_db):
            features.append("rx_fp_gap")
            if baseline > 1.2:
                reasons.append(f"RX-FP {nlos_db:.1f}dB")

        def weak_power(power, source):
            nonlocal channel
            if not corrected_power or power is None or not math.isfinite(power):
                return
            features.append(source + "_fp")
            penalty = 1.0 + (max(0.0, cfg.fp_warn_dbm - power) / cfg.fp_penalty_step_db) ** 2
            channel = max(channel, penalty)
            if penalty > 1.2:
                reasons.append(f"{source} FP {power:.1f}dBm")

        weak_power(fp_dbm, "tag")
        age, proxy = None, None
        cached = self.samples.get(aid)
        if cached is not None and cached[0] == boot and (cached[2].resp_valid or cached[2].final_valid):
            sample_age = time_s - cached[1]
            if 0.0 <= sample_age <= cfg.diagnostic_max_age_s:
                age = sample_age
                sample = cached[2]
                # Diagnostics supplement channel health with age decay, rather
                # than assigning the same stale registers to every fast sample.
                freshness = 1.0 - age / cfg.diagnostic_max_age_s
                if sample.resp_valid and sample.rxpacc > 0:
                    amps = (sample.fp_ampl1, sample.fp_ampl2, sample.fp_ampl3)
                    if sample.std_noise > 0 and any(amps):
                        proxy = 20 * math.log10(math.sqrt(sum(a*a for a in amps) / 3) / sample.std_noise)
                        penalty = 1.0 + (max(0.0, cfg.fp_noise_warn_db - proxy) / 4.0) ** 2
                        channel = max(channel, 1.0 + (penalty - 1.0) * freshness)
                        features.append("fp_noise_proxy")
                        if penalty > 1.2:
                            reasons.append(f"FP/noise {proxy:.1f}dB")
                    elif not any(amps) and sample.cir_pwr > 0:
                        channel = max(channel, 1.0 + 15 * freshness)
                        features.append("missing_first_path")
                        reasons.append("No first path in valid RX")
                if sample.final_valid:
                    # FINAL power is a different reception, not a duplicate of
                    # the TAG's RESP power. Still use max, not a product.
                    anchor_fp, anchor_rx = sample.anchor_fp_cdbm, sample.anchor_rx_cdbm
                    if anchor_fp != -32768:
                        before = channel
                        weak_power(anchor_fp / 100, "anchor")
                        channel = max(before, 1 + (channel - 1) * freshness)
                    if nlos_enabled and anchor_fp != -32768 and anchor_rx != -32768:
                        gap = (anchor_rx - anchor_fp) / 100
                        penalty = nlos_variance_scale(gap)
                        channel = max(channel, 1 + (penalty - 1) * freshness)
                        features.append("anchor_rx_fp_gap")
                        if penalty > 1.2:
                            reasons.append(f"Anchor RX-FP {gap:.1f}dB")

        clock_scale = 1.0
        if ci_ppm is not None and math.isfinite(ci_ppm):
            old = self.clock.get(aid)
            if old is not None and 0 < time_s - old[0] <= 0.2:
                step = abs(ci_ppm - old[1])
                if step > cfg.clock_step_warn_ppm:
                    clock_scale += min(8.0, ((step - cfg.clock_step_warn_ppm) / 2) ** 2)
                    reasons.append(f"Clock step {step:.1f}ppm")
                    features.append("clock_step")
            if old is None or time_s > old[0]:
                self.clock[aid] = (time_s, ci_ppm)
        if late:
            clock_scale = max(clock_scale, 1.5)
            features.append("late_reply")
            reasons.append("Late reply")
        desired = min(cfg.max_variance_scale, channel * clock_scale)
        old = self.recovery.get(aid)
        scale = desired
        if features and old is not None and time_s >= old[0] and old[2] == nlos_enabled:
            scale = max(desired, desired + (old[1] - desired) * math.exp(-(time_s - old[0]) / cfg.recovery_tau_s))
            if scale > desired * 1.1:
                reasons.append("Recovering")
        if old is None or time_s > old[0]:
            self.recovery[aid] = (time_s, scale, nlos_enabled)
        score = 100 / math.sqrt(scale) if features else None
        label = "UNKNOWN" if score is None else "GOOD" if score >= 75 else "WARN" if score >= 45 else "POOR"
        return QualityResult(score, label, scale, max(1.0, scale / baseline),
                             tuple(reasons), tuple(features), age, proxy)


class QualityReplay:
    """Restore received ordering, including quality frames arriving AFTER Burst.

    No sidecar means no invented raw diagnostics. Old Burst power fields still
    drive quality. Each JSONL event records the count of accepted CSV rows that
    preceded it, avoiding retroactive use of later radio diagnostics.
    """
    def __init__(self, csv_path: Path, enabled: bool = True) -> None:
        self._events = self._read(csv_path.parent / "rx_quality.jsonl" if enabled else None)
        self._next = next(self._events, None)

    @staticmethod
    def _read(path):
        if path is not None and path.is_file():
            with path.open(encoding="utf-8") as handle:
                previous = -1
                for line in handle:
                    event = json.loads(line)
                    index = int(event["after_burst_rows"])
                    if index < previous:
                        raise ValueError("RX_QUALITY events out of order")
                    previous = index
                    yield index, event["message"]

    def feed_before_row(self, engine, row_index: int, received_s: float) -> None:
        import telemetry_protocol as tp
        while self._next is not None and self._next[0] <= row_index:
            data = dict(self._next[1])
            data["anchors"] = tuple(tp.RxQualityAnchor(**a) for a in data["anchors"])
            engine.ingest_message(tp.RxQualityMessage(**data), received_s)
            self._next = next(self._events, None)

    def close(self) -> None:
        self._events.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
