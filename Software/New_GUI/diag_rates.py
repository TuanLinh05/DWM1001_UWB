"""Per-second rates of the TAG's cumulative DIAG counters, plus a likely cause.

DIAG_SYSTEM (0x15) and DIAG_ANCHOR (0x11) carry counters since boot, about
once per second. Rates use the TAG clock (time_ms). A counter that goes
backwards (reboot or reset) restarts the baseline instead of producing a
negative rate.

Firmware meanings (Firmware/common/ranging/tag_ranging.c):
- resp_timeouts: the anchor was polled and no RESP was decoded in its slot.
- report_timeouts: RESP decoded but no usable Rb next cycle (FINAL not sent
  or not received by the anchor); the CSV then also shows TIMEOUT.
- poll_skipped: the anchor was offline (8 missed RESPs in a row) and was not
  polled; the CSV shows no entry for it in that cycle.
- rx_errors.phy_header/fcs/sync_loss/lde: a frame was detected but corrupted.
- rx_errors.overrun: the TAG serviced RESPs too slowly (double RX buffer).
- rx_errors.filtered: frames from a foreign PAN/address (other UWB traffic).
- burst_final_late: delayed FINAL armed too late; no Rb for the whole cycle.
"""
from __future__ import annotations

from dataclasses import asdict, is_dataclass
from typing import Any, Mapping

SYSTEM_COUNTERS = (
    "cycle_count", "burst_final_late", "ds_report_timeout", "ds_final_tx_timeout",
    "uart_tx_overflow", "meas_queue_drops", "burst_record_drops", "spi_errors",
    "radio_recoveries", "cycle_overruns",
)
ANCHOR_COUNTERS = (
    "success", "resp_timeouts", "report_timeouts", "poll_skipped", "rx_errors",
    "txn_mismatch", "burst_late", "probes", "ds_ok",
)
CORRUPT_CLASSES = ("phy_header", "fcs", "sync_loss", "lde")


def _as_dict(msg: Any) -> dict:
    return asdict(msg) if is_dataclass(msg) else dict(msg)


def counter_rates(prev: Mapping, cur: Mapping, fields, dt_s: float) -> dict[str, float] | None:
    if dt_s <= 0.0:
        return None
    out: dict[str, float] = {}
    for f in fields:
        a, b = prev.get(f), cur.get(f)
        if a is None or b is None:
            continue
        if b < a:
            return None  # reboot/reset: no valid rate for this pair
        out[f] = (b - a) / dt_s
    return out


class DiagTracker:
    """Keeps the first and the latest DIAG of each kind for rates.

    `window` rates compare the latest message with the previous one (about 1 s);
    `session` rates compare with the first message of the same boot.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._sys_first: dict | None = None
        self._sys_prev: dict | None = None
        self._sys_cur: dict | None = None
        self._anc_first: dict[int, dict] = {}
        self._anc_prev: dict[int, dict] = {}
        self._anc_cur: dict[int, dict] = {}

    def ingest(self, msg: Any) -> None:
        d = _as_dict(msg)
        if "anchor_id" in d and "resp_timeouts" in d:
            aid = int(d["anchor_id"])
            cur = self._anc_cur.get(aid)
            if cur is not None and (d["time_ms"] < cur["time_ms"] or d["success"] < cur["success"]):
                self._anc_first.pop(aid, None)
                cur = None
            self._anc_prev[aid] = cur if cur is not None else d
            self._anc_cur[aid] = d
            self._anc_first.setdefault(aid, d)
        elif "rx_errors" in d and "cycle_count" in d:
            cur = self._sys_cur
            if cur is not None and (d["boot_id"] != cur["boot_id"] or d["cycle_count"] < cur["cycle_count"]):
                self._sys_first = cur = None
                self._anc_first.clear()
                self._anc_prev.clear()
                self._anc_cur.clear()
            self._sys_prev = cur if cur is not None else d
            self._sys_cur = d
            if self._sys_first is None:
                self._sys_first = d

    @staticmethod
    def _system_rates(a: dict | None, b: dict | None) -> dict[str, float] | None:
        if a is None or b is None or a is b:
            return None
        dt = (b["time_ms"] - a["time_ms"]) / 1000.0
        rates = counter_rates(a, b, SYSTEM_COUNTERS, dt)
        if rates is None:
            return None
        rx = counter_rates(a["rx_errors"], b["rx_errors"], b["rx_errors"].keys(), dt) or {}
        rates.update({f"rx_{k}": v for k, v in rx.items()})
        rates["dt_s"] = dt
        return rates

    def system_rates(self, session: bool = False) -> dict[str, float] | None:
        return self._system_rates(self._sys_first if session else self._sys_prev, self._sys_cur)

    def anchor_rates(self, session: bool = False) -> dict[int, dict[str, float]]:
        base = self._anc_first if session else self._anc_prev
        out: dict[int, dict[str, float]] = {}
        for aid, b in self._anc_cur.items():
            a = base.get(aid)
            if a is None or a is b:
                continue
            dt = (b["time_ms"] - a["time_ms"]) / 1000.0
            r = counter_rates(a, b, ANCHOR_COUNTERS, dt)
            if r is not None:
                r["dt_s"] = dt
                out[aid] = r
        return out

    def interpret(self, session: bool = False) -> list[str]:
        return interpret_rates(self.system_rates(session), self.anchor_rates(session))


def interpret_rates(sys_r: Mapping[str, float] | None, anc_r: Mapping[int, Mapping[str, float]]) -> list[str]:
    """Plain-language hints; thresholds are engineering heuristics."""
    notes: list[str] = []
    if not sys_r:
        return notes
    cycles = max(sys_r.get("cycle_count", 0.0), 1e-9)
    corrupt = sum(sys_r.get(f"rx_{k}", 0.0) for k in CORRUPT_CLASSES)
    polled = {
        aid: max(
            cycles - r.get("poll_skipped", 0.0),
            r.get("success", 0.0) + r.get("resp_timeouts", 0.0),
            1e-9,
        )
        for aid, r in anc_r.items()
    }
    resp_to = {aid: r.get("resp_timeouts", 0.0) / polled[aid] for aid, r in anc_r.items()}
    if resp_to:
        worst = max(resp_to.values())
        mean = sum(resp_to.values()) / len(resp_to)
        lost_per_s = sum(r.get("resp_timeouts", 0.0) for r in anc_r.values())
        if mean > 0.10:
            if corrupt < 0.2 * lost_per_s:
                notes.append(
                    f"RESP lost in {100 * mean:.0f}% of polls (worst {100 * worst:.0f}%) and mostly never "
                    f"detected ({corrupt:.0f} corrupted frames/s vs {lost_per_s:.0f} lost/s): the anchor "
                    "did not hear the POLL, did not transmit, or the path is blocked/too weak."
                )
            else:
                notes.append(
                    f"RESP lost in {100 * mean:.0f}% of polls with {corrupt:.0f} corrupted frames/s: frames are "
                    "detected but damaged (interference, overlapping slots or low SNR)."
                )
    if sys_r.get("rx_overrun", 0.0) > 0.01 * cycles:
        notes.append(f"RX overrun {sys_r['rx_overrun']:.0f}/s: the TAG reads RESPs too slowly for the slot width.")
    if sys_r.get("rx_filtered", 0.0) > 0.05 * cycles:
        notes.append(f"{sys_r['rx_filtered']:.0f} foreign frames/s: other UWB traffic on this channel/PAN.")
    if sys_r.get("burst_final_late", 0.0) > 0.01 * cycles:
        notes.append(
            f"FINAL late in {100 * sys_r['burst_final_late'] / cycles:.0f}% of cycles: TAG too slow before "
            "the FINAL deadline; every anchor loses that cycle."
        )
    mism = sum(r.get("txn_mismatch", 0.0) for r in anc_r.values())
    if mism > 0.01 * cycles:
        notes.append(f"{mism:.0f} RESP/s answer an older POLL (txn mismatch): anchor timing is behind.")
    skipped = {aid: r.get("poll_skipped", 0.0) / cycles for aid, r in anc_r.items()}
    off = [f"A{aid} {100 * v:.0f}%" for aid, v in sorted(skipped.items()) if v > 0.05]
    if off:
        notes.append("Offline (not polled) share of cycles: " + ", ".join(off) + ".")
    if sys_r.get("uart_tx_overflow", 0.0) > 0 or sys_r.get("meas_queue_drops", 0.0) > 0:
        notes.append("TAG telemetry overflow/drops: UART cannot carry the selected telemetry.")
    return notes


def format_rates(sys_r: Mapping[str, float] | None, anc_r: Mapping[int, Mapping[str, float]]) -> str:
    """Compact multi-line text for the Diagnostics page."""
    if not sys_r:
        return "Waiting for two DIAG_SYSTEM messages (needs DIAG telemetry on)."
    cycles = sys_r.get("cycle_count", 0.0)
    rx = ", ".join(
        f"{k[3:]} {v:.0f}" for k, v in sorted(sys_r.items())
        if k.startswith("rx_") and v >= 0.5
    ) or "none"
    lines = [
        f"Cycles {cycles:.0f}/s | FINAL late {sys_r.get('burst_final_late', 0):.0f}/s | "
        f"Rb missing {sys_r.get('ds_report_timeout', 0):.0f}/s | RX errors/s: {rx}",
        "Anchor   DS ok/s  RESP lost  no Rb  skipped  late  txn",
    ]
    for aid in sorted(anc_r):
        r = anc_r[aid]
        polled = max(
            cycles - r.get("poll_skipped", 0.0),
            r.get("success", 0.0) + r.get("resp_timeouts", 0.0),
            1e-9,
        )
        skip_pct = min(100.0, 100.0 * r.get("poll_skipped", 0.0) / max(cycles, 1e-9))
        lines.append(
            f"A{aid}      {r.get('ds_ok', 0):6.0f}   {100 * r.get('resp_timeouts', 0) / polled:6.0f}%  "
            f"{r.get('report_timeouts', 0):5.0f}  {skip_pct:6.0f}%  "
            f"{r.get('burst_late', 0):4.0f}  {r.get('txn_mismatch', 0):4.0f}"
        )
    return "\n".join(lines)
