"""Session folders, the timestamped UWB frame record and serial port discovery.

All files of one flight live in <log_root>/<session>/. Both nodes stamp data
with the UP4000 clocks: CLOCK_REALTIME (wall time, also used by .tlog) and
CLOCK_MONOTONIC (never jumps; use it to align UWB with timesync.csv).
"""
from __future__ import annotations

from datetime import datetime
import glob
import json
import os
from pathlib import Path
import struct
import time
from typing import BinaryIO, Iterator

FRAME_FILE_MAGIC = b"UWBFRAMES1\n"
_RECORD = struct.Struct("<qqH")  # realtime_ns, monotonic_ns, frame length

# /dev/serial/by-id names survive re-plugging; ttyACM numbers do not.
TAG_PORT_PATTERNS = (
    "/dev/uwb_tag",
    "/dev/serial/by-id/*SEGGER*",
    "/dev/serial/by-id/*J-Link*",
)
PX4_PORT_PATTERNS = (
    "/dev/pixhawk",
    "/dev/serial/by-id/*PX4*-if00",
    "/dev/serial/by-id/*Pixhawk*-if00",
    "/dev/serial/by-id/*Holybro*-if00",
    "/dev/serial/by-id/*PX4*",
    "/dev/serial/by-id/*Pixhawk*",
)


def default_session_name() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def session_dir(log_root: str, session: str) -> Path:
    root = Path(os.path.expanduser(log_root or "~/flight_logs"))
    path = root / (session or default_session_name())
    path.mkdir(parents=True, exist_ok=True)
    return path


def find_port(setting: str, patterns: tuple[str, ...]) -> str | None:
    """`setting` is a device path, or "auto" to take the first pattern match."""
    if setting and setting != "auto":
        return setting
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return None


def now_ns() -> tuple[int, int]:
    return time.time_ns(), time.monotonic_ns()


class FrameRecordWriter:
    """uwb_frames.bin: magic, then <int64 realtime_ns><int64 monotonic_ns>
    <uint16 length><bytes>. Each record is one chunk of the TAG UART byte
    stream exactly as read (frames can span records), stamped when read.
    Feed all records in order into one TelemetryStreamParser to get frames;
    a frame's time is that of the record holding its last byte."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._fh: BinaryIO = path.open("wb")
        self._fh.write(FRAME_FILE_MAGIC)
        self.records = 0
        self.bytes = len(FRAME_FILE_MAGIC)

    def write(self, frame: bytes, realtime_ns: int, monotonic_ns: int) -> None:
        self._fh.write(_RECORD.pack(realtime_ns, monotonic_ns, len(frame)))
        self._fh.write(frame)
        self.records += 1
        self.bytes += _RECORD.size + len(frame)

    def flush(self) -> None:
        self._fh.flush()

    def close(self) -> None:
        self._fh.close()


def read_frame_records(path: Path) -> Iterator[tuple[int, int, bytes]]:
    """Yield (realtime_ns, monotonic_ns, frame). A truncated tail (power loss)
    ends the iteration instead of raising."""
    with Path(path).open("rb") as fh:
        if fh.read(len(FRAME_FILE_MAGIC)) != FRAME_FILE_MAGIC:
            raise ValueError(f"{path} is not a UWB frame record file")
        while True:
            head = fh.read(_RECORD.size)
            if len(head) < _RECORD.size:
                return
            realtime_ns, monotonic_ns, length = _RECORD.unpack(head)
            frame = fh.read(length)
            if len(frame) < length:
                return
            yield realtime_ns, monotonic_ns, frame


def write_json(path: Path, data: dict) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")
    os.replace(tmp, path)  # never leave a half-written status file


def describe_port_loss(exc: Exception) -> str:
    """pyserial's wording when another program reads the same tty."""
    text = str(exc)
    if "multiple access" in text:
        text += (" -> another program is probably reading this port (e.g. MicroXRCEAgent, "
                 "MAVROS, QGroundControl); one serial port cannot be shared")
    return text


class LossLog:
    """Log a repeating link-loss message 3 times, then once per 60."""

    def __init__(self, log) -> None:
        self.log = log
        self.seen: dict[str, int] = {}

    def __call__(self, text: str) -> None:
        n = self.seen.get(text, 0) + 1
        self.seen[text] = n
        if n <= 3 or n % 60 == 0:
            self.log(text + (f" (x{n})" if n > 3 else ""))


def other_port_users(device: str, proc_root: str = "/proc") -> list[str]:
    """Other processes whose command line names this serial device (or the
    tty it links to), e.g. `MicroXRCEAgent serial --dev /dev/ttyACM0`.
    A heuristic: works for root processes too, and needs no extra rights."""
    if not device or not os.path.isdir(proc_root):
        return []
    names = {device, os.path.realpath(device)}
    me = os.getpid()
    found = []
    for pid in os.listdir(proc_root):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            with open(os.path.join(proc_root, pid, "cmdline"), "rb") as fh:
                cmd = fh.read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue
        if any(name in cmd.split() or f"={name}" in cmd for name in names):
            found.append(f"pid {pid}: {cmd[:140]}")
    return found


class PortUserWarning:
    """Warn once per newly seen process that names our port."""

    def __init__(self, log, what: str) -> None:
        self.log, self.what, self.seen = log, what, set()

    def check(self, device: str) -> None:
        for entry in other_port_users(device):
            if entry not in self.seen:
                self.seen.add(entry)
                self.log(f"WARNING: {self.what} port {device} is also named by {entry}. "
                         "Two programs cannot share one serial port; stop it or move it to another port.")


class AlreadyRunning(RuntimeError):
    """Another instance of this node (e.g. the systemd service) holds its lock."""


STOP_HINT = "Stop the other instance first, e.g. `sudo systemctl stop uwb-px4-logger`."


def single_instance(name: str, lock_dir: str = "/tmp"):
    """Hold /tmp/uwb_px4_logger.<name>.lock for the life of the process.
    Two loggers would fight over the serial ports and UDP ports (07/11: a
    manual run beside the service lost its threads to EADDRINUSE). Returns
    the open lock file (keep a reference), or None where flock is missing."""
    try:
        import fcntl
    except ImportError:  # not Linux (tests on Windows)
        return None
    path = os.path.join(lock_dir, f"uwb_px4_logger.{name}.lock")
    fh = open(path, "a+")
    try:
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        fh.seek(0)
        owner = fh.read().strip() or "unknown pid"
        fh.close()
        raise AlreadyRunning(f"{name} is already running ({owner}). {STOP_HINT}") from None
    fh.seek(0)
    fh.truncate()
    fh.write(f"pid {os.getpid()}")
    fh.flush()
    return fh


def bind_or_explain(open_socket, port: int, name: str) -> None:
    """Open the UDP socket now, so a busy port fails loudly at start."""
    try:
        open_socket()
    except OSError as exc:
        raise AlreadyRunning(f"{name}: UDP port {port} is busy ({exc}). "
                             f"Another logger may be running. {STOP_HINT}") from None
