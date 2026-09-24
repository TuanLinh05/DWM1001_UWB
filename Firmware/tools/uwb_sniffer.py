"""Decode the DWM1001 sniffer stream into a UWB exchange timeline.

The Sniffer_DevKit firmware forwards every frame it hears, with the DW1000
receive timestamp (15.65 ps resolution) and the signal powers. This tool turns
that into a readable timeline, so slot timing, reply delays and collisions can
be measured without a logic analyser.

Examples (PowerShell):
    py -3.12 uwb_sniffer.py --port COM9
    py -3.12 uwb_sniffer.py --port COM9 --seconds 10 --summary --csv sniff.csv
    py -3.12 uwb_sniffer.py --file capture.bin --summary
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path

GUI_DIR = Path(__file__).resolve().parents[2] / "Software" / "UWB_UART_GUI"
sys.path.insert(0, str(GUI_DIR))

import telemetry_protocol as tp  # noqa: E402

TICKS_PER_US = 63897.6          # 499.2 MHz * 128
FUNC_NAMES = {0x21: "POLL", 0x10: "RESP", 0x23: "FINAL", 0x22: "REPORT",
              0x30: "CFG", 0x31: "CFG_ACK"}


class UwbFrame:
    """One decoded frame from the air."""

    def __init__(self, message: tp.SnifferFrameMessage) -> None:
        self.timestamp_ticks = message.rx_timestamp
        self.fp_dbm = message.fp_cdbm / 100.0
        self.rx_dbm = message.rx_cdbm / 100.0
        self.error = bool(message.status & 0x02)
        self.raw = message.frame
        self.ok = False
        self.function = None
        self.source = self.destination = None
        self.version = self.txn = None
        self.payload_note = ""
        self._parse()

    def _parse(self) -> None:
        body = self.raw
        if len(body) < 10 or body[0] != 0x41 or body[1] != 0x88:
            return
        self.ok = True
        self.destination = int.from_bytes(body[5:7], "little")
        self.source = int.from_bytes(body[7:9], "little")
        self.function = body[9]
        payload = body[10:]
        if self.function in (0x21, 0x23) and len(payload) >= 2 and payload[0] == 2:
            self.version, self.txn = 2, payload[1]
            if self.function == 0x21 and len(payload) >= 3 and payload[2] & 0x01:
                self.payload_note = "req_info"
        elif self.function == 0x10:
            if len(payload) >= 8 and payload[0] == 2:
                self.version, self.txn = 2, payload[1]
                reply = int.from_bytes(payload[2:6], "little")
                tlv_len = payload[7] if len(payload) > 7 else 0
                self.payload_note = f"Da={reply / TICKS_PER_US:.1f}us"
                if tlv_len:
                    self.payload_note += f" +{tlv_len}B TLV"
            elif len(payload) >= 4:
                self.version = 1
                self.payload_note = f"Da={int.from_bytes(payload[:4], 'little') / TICKS_PER_US:.1f}us"
        elif self.function == 0x22:
            if len(payload) >= 10 and payload[0] == 2:
                self.version, self.txn = 2, payload[1]
                rb = int.from_bytes(payload[2:6], "little")
                fp = int.from_bytes(payload[6:8], "little", signed=True)
                self.payload_note = f"Rb={rb / TICKS_PER_US:.1f}us anchor_fp={fp / 100.0:.1f}dBm"
            elif len(payload) >= 4:
                self.version = 1
                self.payload_note = f"Rb={int.from_bytes(payload[:4], 'little') / TICKS_PER_US:.1f}us"

    @property
    def name(self) -> str:
        if self.error:
            return "RX_ERROR"
        if not self.ok:
            return f"UNKNOWN({len(self.raw)}B)"
        return FUNC_NAMES.get(self.function, f"FUNC_0x{self.function:02X}")

    def describe(self, delta_us: float | None) -> str:
        gap = "      -" if delta_us is None else f"{delta_us:9.1f}"
        if self.error:
            return f"{gap} us  RX_ERROR"
        if not self.ok:
            return f"{gap} us  {self.name}"
        txn = f" txn={self.txn:02X}" if self.txn is not None else ""
        version = f" v{self.version}" if self.version else ""
        return (f"{gap} us  {self.name:<7} {self.source:#06x}->{self.destination:#06x}"
                f"{version}{txn} fp={self.fp_dbm:6.1f} rx={self.rx_dbm:6.1f} "
                f"{self.payload_note}")


def summarise(frames: list[UwbFrame]) -> None:
    counts: dict[str, int] = defaultdict(int)
    reply_delays: dict[int, list[float]] = defaultdict(list)
    exchange_us: dict[int, list[float]] = defaultdict(list)
    poll_time: dict[int, int] = {}
    poll_anchor: dict[int, int] = {}

    for frame in frames:
        counts[frame.name] += 1
        if not frame.ok or frame.error:
            continue
        if frame.function == 0x21:            # POLL
            poll_time[frame.destination] = frame.timestamp_ticks
            poll_anchor[frame.destination] = frame.timestamp_ticks
        elif frame.function == 0x10:          # RESP
            start = poll_time.get(frame.source)
            if start is not None:
                reply_delays[frame.source].append((frame.timestamp_ticks - start) / TICKS_PER_US)
        elif frame.function == 0x22:          # REPORT ends the exchange
            start = poll_anchor.get(frame.source)
            if start is not None:
                exchange_us[frame.source].append((frame.timestamp_ticks - start) / TICKS_PER_US)
                poll_anchor.pop(frame.source, None)

    print("\nframe counts:")
    for name, count in sorted(counts.items(), key=lambda item: -item[1]):
        print(f"  {name:<10} {count}")

    if reply_delays:
        print("\nPOLL -> RESP reply delay (us), per anchor:")
        for anchor in sorted(reply_delays):
            values = reply_delays[anchor]
            print(f"  A{anchor}: n={len(values):5d} median={statistics.median(values):8.1f} "
                  f"min={min(values):8.1f} max={max(values):8.1f}")
    if exchange_us:
        print("\nPOLL -> REPORT exchange duration (us), per anchor:")
        for anchor in sorted(exchange_us):
            values = exchange_us[anchor]
            values_sorted = sorted(values)
            p95 = values_sorted[int(0.95 * (len(values_sorted) - 1))]
            print(f"  A{anchor}: n={len(values):5d} median={statistics.median(values):8.1f} "
                  f"p95={p95:8.1f} max={max(values):8.1f}")
        total = sum(statistics.median(v) for v in exchange_us.values())
        print(f"\n  sum of median exchange durations: {total:.0f} us "
              f"({len(exchange_us)} anchors; a 50 Hz cycle budget is 20000 us)")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="serial port of the Sniffer_DevKit")
    source.add_argument("--file", help="previously captured raw stream")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--seconds", type=float, default=0.0, help="capture duration")
    parser.add_argument("--csv", help="write the decoded timeline to this CSV")
    parser.add_argument("--summary", action="store_true", help="print timing statistics")
    parser.add_argument("--quiet", action="store_true", help="only print the summary")
    parser.add_argument("--save-raw", help="store the raw stream for later analysis")
    args = parser.parse_args(argv)

    stream_parser = tp.TelemetryStreamParser()
    frames: list[UwbFrame] = []
    previous_ticks: int | None = None
    raw_sink = open(args.save_raw, "wb") if args.save_raw else None
    csv_writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["rx_ticks", "delta_us", "type", "src", "dst", "version",
                             "txn", "fp_dbm", "rx_dbm", "note", "hex"])

    def handle(data: bytes) -> None:
        nonlocal previous_ticks
        if raw_sink is not None:
            raw_sink.write(data)
        for frame in stream_parser.feed(data):
            if frame.type != tp.TYPE_SNIFFER_FRAME:
                continue
            try:
                message = tp.decode_frame(frame)
            except tp.ProtocolError:
                continue
            decoded = UwbFrame(message)
            delta = None
            if previous_ticks is not None and not decoded.error:
                delta = ((decoded.timestamp_ticks - previous_ticks) & 0xFFFFFFFFFF) / TICKS_PER_US
            if not decoded.error:
                previous_ticks = decoded.timestamp_ticks
            frames.append(decoded)
            if not args.quiet:
                print(decoded.describe(delta))
            if csv_writer is not None:
                csv_writer.writerow([
                    decoded.timestamp_ticks, f"{delta:.3f}" if delta is not None else "",
                    decoded.name, decoded.source, decoded.destination, decoded.version,
                    decoded.txn, f"{decoded.fp_dbm:.2f}", f"{decoded.rx_dbm:.2f}",
                    decoded.payload_note, decoded.raw.hex(),
                ])

    try:
        if args.file:
            handle(Path(args.file).read_bytes())
        else:
            import serial  # imported lazily so --file works without pyserial

            with serial.Serial(args.port, args.baud, timeout=0.1) as link:
                deadline = time.monotonic() + args.seconds if args.seconds else None
                while deadline is None or time.monotonic() < deadline:
                    handle(link.read(link.in_waiting or 1))
    except KeyboardInterrupt:
        pass
    finally:
        if raw_sink is not None:
            raw_sink.close()
        if csv_file is not None:
            csv_file.close()

    if args.summary or args.quiet:
        summarise(frames)
    print(f"\n{len(frames)} frames decoded, "
          f"{stream_parser.counters.crc_errors} CRC errors on the sniffer link")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
