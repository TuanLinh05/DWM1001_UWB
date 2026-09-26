"""Command-line client for the TAG command channel (Firmware/common/include/uwb_cmd.h).

Examples (PowerShell):
    py -3.12 uwb_command.py --port COM7 ping
    py -3.12 uwb_command.py --port COM7 info
    py -3.12 uwb_command.py --port COM7 get-cal
    py -3.12 uwb_command.py --port COM7 set-cal 1 12.5          # bias 12.5 mm on A1
    py -3.12 uwb_command.py --port COM7 set-cal 1 -12.5 --gui-offset   # GUI "offset" sign
    py -3.12 uwb_command.py --port COM7 set-txpower reference --save
    py -3.12 uwb_command.py --port COM7 set-telemetry --snapshot --meas --diag --save
    py -3.12 uwb_command.py --port COM7 time-sync --count 20

The default baud is 460800 (Tag_DevKit; firmware built before 2026-09-26 ran
1000000). A Tag PCB on a plain USB-UART adapter needs --baud 115200; the
ESP32-C3 gateway ignores the baud.

Radio changes and SAVE need ranging paused; the tool pauses, applies the
command and resumes automatically. Do not use it while the drone is armed:
the flight computer sets LOCK and the TAG rejects configuration commands.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit("PySerial is required: py -3.12 -m pip install -r requirements.txt") from exc

import telemetry_protocol as tp

TX_POWER_MODES = {"legacy": 0, "reference": 1, "smart": 2, "custom": 3}


class TagLink:
    def __init__(self, port: str, baud: int) -> None:
        self.serial = serial.Serial(port, baud, timeout=0.05)
        self.parser = tp.TelemetryStreamParser()
        self.sequence = int(time.time()) & 0xFFFF
        self.pending: list[tp.DecodedMessage] = []

    def close(self) -> None:
        self.serial.close()

    def _read_messages(self) -> list[tp.DecodedMessage]:
        data = self.serial.read(self.serial.in_waiting or 1)
        messages = []
        for frame in self.parser.feed(data):
            try:
                messages.append(tp.decode_frame(frame))
            except tp.ProtocolError:
                continue
        return messages

    def wait_for(self, predicate, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for message in self._read_messages():
                if predicate(message):
                    return message
        return None

    def command(self, command_id: int, arguments: bytes = b"", retries: int = 3,
                timeout_s: float = 0.6) -> tp.CmdAckMessage:
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        sequence = self.sequence
        frame = tp.encode_command(sequence, command_id, arguments)
        for _ in range(retries):
            self.serial.write(frame)
            ack = self.wait_for(
                lambda m: isinstance(m, tp.CmdAckMessage) and m.sequence == sequence,
                timeout_s,
            )
            if ack is not None:
                return ack
        raise TimeoutError(f"no ACK for command 0x{command_id:02X}")

    def pause(self, timeout_s: float = 2.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            ack = self.command(tp.CMD_PAUSE)
            require_ok(ack)
            if ack.data and ack.data[0]:
                return
            time.sleep(0.05)
        raise TimeoutError("ranging did not pause")

    def resume(self) -> None:
        require_ok(self.command(tp.CMD_RESUME))


def require_ok(ack: tp.CmdAckMessage) -> tp.CmdAckMessage:
    if not ack.ok:
        raise RuntimeError(f"command 0x{ack.command_id:02X} rejected: {ack.result_name}")
    return ack


def with_pause(link: TagLink, action) -> tp.CmdAckMessage:
    link.pause()
    try:
        return action()
    finally:
        link.resume()


def print_device_info(message: tp.DeviceInfoMessage) -> None:
    for name, value in vars(message).items():
        if isinstance(value, int) and name in {"git_hash", "device_id", "otp_part_id",
                                               "otp_lot_id", "otp_ldotune",
                                               "tx_power_register"}:
            value = f"0x{value:08X}"
        print(f"{name:>20}: {value}")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=460800,
                        help="460800 for Tag_DevKit (default), 115200 for a Tag PCB "
                             "on a USB-UART adapter")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("ping")
    sub.add_parser("info")
    sub.add_parser("pause")
    sub.add_parser("resume")
    mask = sub.add_parser("set-mask")
    mask.add_argument("mask", type=lambda v: int(v, 0))
    cal = sub.add_parser("set-cal")
    cal.add_argument("anchor", type=int)
    cal.add_argument("value_mm", type=float)
    cal.add_argument("--gui-offset", action="store_true",
                     help="value is the GUI offset (reference - mean_raw); bias = -offset")
    cal.add_argument("--uncalibrated", action="store_true")
    sub.add_parser("get-cal")
    ant = sub.add_parser("set-ant")
    ant.add_argument("tx", type=int)
    ant.add_argument("rx", type=int)
    txp = sub.add_parser("set-txpower")
    txp.add_argument("mode", choices=sorted(TX_POWER_MODES))
    txp.add_argument("value", nargs="?", type=lambda v: int(v, 0), default=0)
    sub.add_parser("save")
    sub.add_parser("factory-reset")
    sub.add_parser("reboot")
    sync = sub.add_parser("time-sync")
    sync.add_argument("--count", type=int, default=10)
    lock = sub.add_parser("lock")
    lock.add_argument("state", choices=("on", "off"))
    telem = sub.add_parser("set-telemetry")
    telem.add_argument("--snapshot", action="store_true")
    telem.add_argument("--meas", action="store_true")
    telem.add_argument("--diag", action="store_true")
    for name in ("set-ant", "set-txpower", "set-cal", "set-mask", "set-telemetry"):
        sub.choices[name].add_argument("--save", action="store_true",
                                       help="also store the settings in flash")
    args = parser.parse_args(argv)

    link = TagLink(args.port, args.baud)
    try:
        if args.command == "ping":
            ack = require_ok(link.command(tp.CMD_PING))
            print(f"PONG uptime={struct.unpack('<I', ack.data)[0]} ms")
        elif args.command == "info":
            require_ok(link.command(tp.CMD_GET_DEVICE_INFO))
            info = link.wait_for(lambda m: isinstance(m, tp.DeviceInfoMessage), 1.0)
            if info is None:
                raise TimeoutError("no DEVICE_INFO frame")
            print_device_info(info)
        elif args.command == "pause":
            link.pause()
            print("paused")
        elif args.command == "resume":
            link.resume()
            print("resumed")
        elif args.command == "set-mask":
            require_ok(link.command(tp.CMD_SET_ANCHOR_MASK, bytes((args.mask & 0xFF,))))
            print(f"anchor mask = 0x{args.mask & 0xFF:02X}")
        elif args.command == "set-cal":
            bias_mm = -args.value_mm if args.gui_offset else args.value_mm
            payload = struct.pack("<HiB", args.anchor, int(round(bias_mm * 1000.0)),
                                  0 if args.uncalibrated else 1)
            require_ok(link.command(tp.CMD_SET_DS_CAL, payload))
            print(f"A{args.anchor}: bias {bias_mm:+.3f} mm "
                  f"({'uncalibrated' if args.uncalibrated else 'calibrated'})")
        elif args.command == "get-cal":
            ack = require_ok(link.command(tp.CMD_GET_DS_CAL))
            for offset in range(0, len(ack.data), 7):
                anchor_id, bias_um, calibrated = struct.unpack_from("<HiB", ack.data, offset)
                print(f"A{anchor_id}: bias {bias_um / 1000.0:+9.3f} mm  "
                      f"{'CAL' if calibrated else '---'}")
        elif args.command == "set-ant":
            with_pause(link, lambda: require_ok(link.command(
                tp.CMD_SET_ANT_DELAY, struct.pack("<HH", args.tx, args.rx))))
            print(f"antenna delay TX={args.tx} RX={args.rx}")
        elif args.command == "set-txpower":
            ack = with_pause(link, lambda: require_ok(link.command(
                tp.CMD_SET_TX_POWER,
                struct.pack("<BI", TX_POWER_MODES[args.mode], args.value))))
            print(f"TX_POWER = 0x{struct.unpack('<I', ack.data)[0]:08X}")
        elif args.command in ("save", "factory-reset"):
            command_id = tp.CMD_SAVE_SETTINGS if args.command == "save" else tp.CMD_FACTORY_RESET
            with_pause(link, lambda: require_ok(link.command(command_id, timeout_s=2.0)))
            print(f"{args.command}: OK")
        elif args.command == "reboot":
            require_ok(link.command(tp.CMD_REBOOT))
            print("rebooting")
        elif args.command == "time-sync":
            samples = []
            for _ in range(args.count):
                t1 = int(time.perf_counter_ns() // 1000)
                ack = require_ok(link.command(tp.CMD_TIME_SYNC, struct.pack("<Q", t1)))
                t4 = int(time.perf_counter_ns() // 1000)
                _, t2, t3 = struct.unpack("<QQQ", ack.data)
                rtt = (t4 - t1) - (t3 - t2)
                offset = ((t2 - t1) + (t3 - t4)) / 2.0
                samples.append((rtt, offset))
            samples.sort()
            best_rtt, best_offset = samples[0]
            print(f"best RTT {best_rtt} us, TAG - host offset {best_offset:.0f} us "
                  f"({len(samples)} samples)")
        elif args.command == "lock":
            require_ok(link.command(tp.CMD_SET_LOCK, bytes((1 if args.state == "on" else 0,))))
            print(f"lock {args.state}")
        elif args.command == "set-telemetry":
            features = ((tp.TELEM_FEATURE_RANGE_SNAPSHOT if args.snapshot else 0)
                        | (tp.TELEM_FEATURE_RANGE_MEAS if args.meas else 0)
                        | (tp.TELEM_FEATURE_DIAG if args.diag else 0))
            ack = require_ok(link.command(tp.CMD_SET_TELEMETRY, bytes((features,))))
            print(f"telemetry features = 0x{ack.data[0]:02X}")

        if getattr(args, "save", False):
            with_pause(link, lambda: require_ok(link.command(tp.CMD_SAVE_SETTINGS, timeout_s=2.0)))
            print("saved to flash")
    except (TimeoutError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
