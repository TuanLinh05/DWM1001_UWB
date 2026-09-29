"""Check the sniffer decoder against a synthetic capture (no hardware).

Builds a stream of SNIFFER_FRAME packets describing one DS-TWR exchange with
two anchors, then verifies the tool reconstructs frame types, transaction IDs
and the reply-delay timing.
"""

from __future__ import annotations

import struct
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GUI_DIR = ROOT.parent / "Software" / "UWB_UART_GUI"
sys.path.insert(0, str(GUI_DIR))
sys.path.insert(0, str(ROOT / "tools"))

import telemetry_protocol as tp  # noqa: E402
import uwb_sniffer  # noqa: E402

TICKS_PER_US = uwb_sniffer.TICKS_PER_US


def header(seq: int, dst: int, src: int, func: int) -> bytes:
    return bytes((0x41, 0x88, seq)) + struct.pack("<HHHB", 0xDECA, dst, src, func)


def sniffer_packet(sequence: int, ticks: int, frame: bytes, status: int = 0x01) -> bytes:
    payload = struct.pack("<BIBhhBB", 1, ticks & 0xFFFFFFFF, (ticks >> 32) & 0xFF,
                          -8100, -7400, status, len(frame)) + frame
    return tp.encode_frame(tp.TYPE_SNIFFER_FRAME, sequence, 0, payload)


def build_capture() -> bytes:
    stream = b""
    sequence = 0
    ticks = 1_000_000_000
    reply_ticks = int(1200 * 65536)          # 1200 UUS
    for anchor in (1, 2):
        txn = 0x40 + anchor
        poll = header(sequence, anchor, 0, 0x21) + bytes((2, txn, 0))
        stream += sniffer_packet(sequence, ticks, poll)
        sequence += 1
        resp = (header(sequence, 0, anchor, 0x10)
                + bytes((2, txn)) + struct.pack("<IBB", reply_ticks, 0, 0))
        stream += sniffer_packet(sequence, ticks + reply_ticks, resp)
        sequence += 1
        final = header(sequence, anchor, 0, 0x23) + bytes((2, txn))
        stream += sniffer_packet(sequence, ticks + reply_ticks + 20_000_000, final)
        sequence += 1
        report = (header(sequence, 0, anchor, 0x22) + bytes((2, txn))
                  + struct.pack("<Ihh", 25_000_000, -8300, -7700))
        stream += sniffer_packet(sequence, ticks + reply_ticks + 40_000_000, report)
        sequence += 1
        ticks += 200_000_000                  # next anchor slot
    stream += sniffer_packet(sequence, 0, b"", status=0x02)   # RX error
    sequence += 1
    # Two burst v3 cycles: broadcast POLL, RESPs of A1 (slot 0) and A3
    # (slot 1, late, carrying the previous Rb), broadcast FINAL.
    uus = 65536
    for cycle in range(2):
        start = 3_000_000_000 + cycle * 4500 * 63898
        txn = 0x70 + cycle
        poll = (header(sequence, 0xFFFF, 0, 0x21)
                + bytes((3, txn, 0, 0x05)) + struct.pack("<HHHB", 800, 350, 1500, 0))
        stream += sniffer_packet(sequence, start, poll)
        sequence += 1
        for slot, anchor in enumerate((1, 3)):
            status = 0x08 | (0x10 if anchor == 3 else 0)
            resp = (header(sequence, 0, anchor, 0x10)
                    + bytes((3, txn)) + struct.pack("<IBBIhhB", (800 + 350 * slot) * uus,
                                                    status, txn - 1, 3000 * uus, -8200, -7600, 0))
            stream += sniffer_packet(sequence, start + (800 + 350 * slot) * uus, resp)
            sequence += 1
        final = header(sequence, 0xFFFF, 0, 0x23) + bytes((3, txn))
        stream += sniffer_packet(sequence, start + 1500 * uus, final)
        sequence += 1
    return stream


def main() -> int:
    capture = build_capture()
    # A named file works on Windows even when the runner has restricted
    # permissions for directories created with tempfile's private ACL.
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as temporary:
        temporary.write(capture)
        path = Path(temporary.name)

    try:
        stream_parser = tp.TelemetryStreamParser()
        decoded = [uwb_sniffer.UwbFrame(tp.decode_frame(frame))
                   for frame in stream_parser.feed(capture)]
        assert stream_parser.counters.crc_errors == 0, "sniffer stream CRC"
        assert len(decoded) == 17, f"expected 17 frames, got {len(decoded)}"
        burst = decoded[9:]
        assert [f.name for f in burst[:4]] == ["POLL", "RESP", "RESP", "FINAL"], burst
        assert burst[0].version == 3 and "mask=0x05 base=800 slot=350" in burst[0].payload_note
        assert "Rb=" in burst[2].payload_note and "LATE" in burst[2].payload_note, \
            burst[2].payload_note
        names = [frame.name for frame in decoded]
        assert names[:4] == ["POLL", "RESP", "FINAL", "REPORT"], names
        assert names[8] == "RX_ERROR", names
        assert decoded[0].txn == 0x41 and decoded[1].txn == 0x41, "transaction id"
        assert decoded[1].source == 1 and decoded[1].destination == 0, "addresses"
        assert "Da=" in decoded[1].payload_note, decoded[1].payload_note
        assert "anchor_fp=-83.0dBm" in decoded[3].payload_note, decoded[3].payload_note

        # The tool must run end to end on a file and produce the summary.
        result = subprocess.run(
            [sys.executable, str(ROOT / "tools" / "uwb_sniffer.py"),
             "--file", str(path), "--quiet"],
            capture_output=True, text=True, check=True)
        assert "POLL -> RESP reply delay" in result.stdout, result.stdout
        assert "17 frames decoded" in result.stdout, result.stdout
        assert "burst POLL -> RESP" in result.stdout, result.stdout
        slot1_us = f"{1150 * 65536 / TICKS_PER_US:.1f}"
        assert f"A3: n=    2 median={slot1_us:>8}" in result.stdout, result.stdout
        assert "RESPs flagged LATE (previous slot missed): 2" in result.stdout, result.stdout
        assert "~222 cycles/s" in result.stdout, result.stdout
        expected_reply_us = f"{1200 * 65536 / TICKS_PER_US:.1f}"
        assert expected_reply_us in result.stdout, result.stdout

    finally:
        path.unlink(missing_ok=True)

    print("sniffer tool tests passed (frame decode, timeline, summary)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
