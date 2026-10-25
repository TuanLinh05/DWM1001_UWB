"""ROS-independent bridge between UART messages and calibration-console JSON.

Compact FP/RX/CI belong to every range. Raw quality is sampled separately;
its cycle, timestamp and age always travel with it, never as a new range.
"""
from __future__ import annotations

import telemetry_protocol as tp


def power(value: int) -> float | None:
    return None if value == tp.INT16_MIN else value / 100


def quality_values(a: tp.RxQualityAnchor) -> dict:
    raw = {name: getattr(a, name) if a.resp_valid else None for name in (
        'rxpacc', 'fp_ampl1', 'fp_ampl2', 'fp_ampl3', 'cir_pwr', 'std_noise',
        'fp_index', 'carrier_integrator')}
    raw.update(resp_valid=a.resp_valid, final_valid=a.final_valid,
        fp_dbm=power(a.fp_cdbm) if a.resp_valid else None,
        rx_dbm=power(a.rx_cdbm) if a.resp_valid else None,
        anchor_fp_dbm=power(a.anchor_fp_cdbm) if a.final_valid else None,
        anchor_rx_dbm=power(a.anchor_rx_cdbm) if a.final_valid else None,
        ci_ppm=-a.carrier_integrator * 1791 / 3125000 if a.resp_valid else None)
    fp, rx = raw['fp_dbm'], raw['rx_dbm']
    raw['nlos_db'] = None if fp is None or rx is None else round(rx-fp, 2)
    return raw


class BurstMetadata:
    def __init__(self) -> None:
        self.quality: tp.RxQualityMessage | None = None
        self.diag: tp.DiagSystemMessage | None = None

    def accept(self, message: object) -> None:
        if isinstance(message, tp.RxQualityMessage):
            self.quality = message
        elif isinstance(message, tp.DiagSystemMessage):
            self.diag = message

    def build(self, message: tp.RangeBurstMessage) -> dict:
        q = self.quality
        # No future, previous-boot or stale sample may be attributed to this
        # range. Older TAGs still supply compact powers/CI without any 0x19.
        if q is not None and (q.boot_id != message.boot_id
                or not 0 <= message.t_us-q.t_us <= 500_000):
            q = None
        quality_by_id = {a.anchor_id: a for a in q.anchors} if q else {}
        anchors = []
        for a in message.anchors:
            fp, rx = power(a.fp_cdbm), power(a.rx_cdbm)
            ppm = None if a.ci_ppm_x100 == tp.INT16_MIN else a.ci_ppm_x100/100
            values = dict(anchor_id=a.anchor_id, mode=a.mode_name, mode_code=a.mode,
                status=a.status, flags=a.flags, range_mm=a.range_mm,
                calibrated=bool(a.flags & tp.BURST_FLAG_CAL_OK),
                valid=bool(a.flags & tp.BURST_FLAG_VALID), radio_ok=a.radio_ok,
                fp_dbm=fp, rx_dbm=rx,
                nlos_db=None if fp is None or rx is None else round(rx-fp, 2),
                ci_ppm=ppm, ci=None if ppm is None else round(ppm*1000),
                quality=None)
            if a.anchor_id in quality_by_id:
                values['quality'] = dict(cycle_seq=q.cycle_seq, tag_time_us=q.t_us,
                    age_ms=(message.t_us-q.t_us)/1000, prf_mhz=q.prf_mhz,
                    **quality_values(quality_by_id[a.anchor_id]))
            anchors.append(values)
        temperature, vbat, env_age = None, None, None
        if q:
            temperature, vbat = q.temperature_c, q.vbat_v
            env_age = None if q.env_age_ms is None else q.env_age_ms+((message.time_ms-q.time_ms)&0xFFFFFFFF)
        elif self.diag is not None and self.diag.boot_id == message.boot_id:
            # DIAG uptime and envelope time are ms, unlike the POLL clock.
            age = (message.time_ms-self.diag.time_ms) & 0xFFFFFFFF
            if age <= 2000:
                temperature, vbat = self.diag.temperature_c, self.diag.vbat_v
        return dict(schema=2, boot_id=message.boot_id, cycle_seq=message.cycle_seq,
            meas_seq=message.meas_seq_first, meas_seq_first=message.meas_seq_first,
            tag_time_us=message.t_us, period_us=message.period_us,
            temperature_c=temperature, vbat_v=vbat, env_age_ms=env_age,
            ds_count=sum(a.radio_ok and a.mode_name == 'DS' for a in message.anchors),
            anchors=anchors)
