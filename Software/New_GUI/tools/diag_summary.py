"""Explain why Burst ranges are missing in a recorded session.

python tools/diag_summary.py logs/<session> [logs/<session> ...]

From the CSV (any schema-5 log): per anchor, the share of cycles in which it
was polled (an empty status means the TAG skipped it as offline), RESP
received when polled (rx_quality.jsonl sample), range success when polled and
the anchor-late flag. A DS range needs RESP in two consecutive cycles plus
the FINAL in between, so range success is roughly RESP success squared.

From diag.jsonl (written by New_GUI since 07/11/2026): the TAG's own counters
over the session: RESP timeouts, missing Rb, skipped polls, txn mismatch,
FINAL late and RX error classes, followed by a plain-language hint.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diag_rates import DiagTracker, format_rates  # noqa: E402


def csv_breakdown(session: Path) -> None:
    path = session / "uwb_fast_log.csv"
    if not path.is_file():
        print("  (no uwb_fast_log.csv)")
        return
    with path.open(newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows or "a1_status_code" not in rows[0]:
        print("  CSV has no per-anchor status column (schema < 5); breakdown skipped.")
        return
    resp_ok = {a: 0 for a in range(1, 9)}
    resp_n = {a: 0 for a in range(1, 9)}
    qpath = session / "rx_quality.jsonl"
    if qpath.is_file():
        for line in qpath.open(encoding="utf-8"):
            for a in json.loads(line)["message"]["anchors"]:
                resp_n[a["anchor_id"]] += 1
                resp_ok[a["anchor_id"]] += a["flags"] & 1
    n = len(rows)
    t0, t1 = int(rows[0]["tag_time_us"]), int(rows[-1]["tag_time_us"])
    per_cycle = sum(sum(1 for a in range(1, 9) if r[f"a{a}_mm"]) for r in rows) / n
    print(f"  {n} cycles in {(t1 - t0) / 1e6:.1f} s, {per_cycle:.2f} ranges per cycle")
    print("  anchor  polled  RESP|polled  range|polled  late")
    for a in range(1, 9):
        polled = [r for r in rows if r[f"a{a}_status_code"] != ""]
        ok = sum(1 for r in polled if r[f"a{a}_mm"])
        late = sum(1 for r in polled if r[f"a{a}_flags"] and int(float(r[f"a{a}_flags"])) & 0x10)
        resp = f"{100 * resp_ok[a] / resp_n[a]:9.0f}%" if resp_n[a] else "       n/a"
        npol = max(len(polled), 1)
        print(f"  A{a}     {100 * len(polled) / n:5.0f}%  {resp}  {100 * ok / npol:11.0f}%  {100 * late / npol:4.0f}%")


def diag_breakdown(session: Path) -> None:
    path = session / "diag.jsonl"
    if not path.is_file():
        print("  (no diag.jsonl: recorded before DIAG logging; TAG counters unavailable)")
        return
    tracker = DiagTracker()
    for line in path.open(encoding="utf-8"):
        tracker.ingest(json.loads(line)["message"])
    sys_r = tracker.system_rates(session=True)
    anc_r = tracker.anchor_rates(session=True)
    if not sys_r:
        print("  diag.jsonl has fewer than two DIAG_SYSTEM messages of one boot.")
        return
    print(f"  TAG counters over {sys_r['dt_s']:.1f} s (per second):")
    for line in format_rates(sys_r, anc_r).splitlines():
        print("  " + line)
    for note in tracker.interpret(session=True):
        print("  -> " + note)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sessions", nargs="+", type=Path)
    args = parser.parse_args()
    for session in args.sessions:
        print(f"== {session.name}")
        csv_breakdown(session)
        diag_breakdown(session)


if __name__ == "__main__":
    main()
