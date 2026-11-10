"""UP4000 health + logger status + PX4 values -> one JSON datagram to the ground.

New_GUI's "UP4000" tab listens on UDP 15557 (read only). The state has the
sections below; each is sent as parts of at most 1200 bytes that the GUI
merges (see split_parts):

  host:       CPU, load, temperatures, memory, disk, WiFi, uptime, IPs,
              CPU of the logger processes                     (1 Hz)
  session:    folder and file sizes                           (1 Hz)
  uwb_relay:  uwb_relay status (counters, parser, ground, TAG) (1 Hz)
  px4_logger: px4_logger status (ULog stream, counters)       (1 Hz)
  px4:        latest PX4 values per message type, rates,
              STATUSTEXT, message_types                       (status_hz)

Host values come from /proc and /sys (Linux); paths are parameters so tests
can use a fake tree. Missing sources give None, never an exception.
"""
from __future__ import annotations

import base64
import glob
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import time

STATUS_KIND = "up4000_status"
STATUS_VERSION = 2  # 2: small merged parts (see split_parts)
LOGGER_PROCESSES = ("uwb_relay", "px4_logger", "up4000_status")


def _read(path: str) -> str | None:
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            return fh.read()
    except OSError:
        return None


def _run_cmd(argv: list[str]) -> str | None:
    try:
        return subprocess.run(argv, capture_output=True, text=True, timeout=1.0).stdout
    except (OSError, subprocess.SubprocessError):
        return None


class HostHealth:
    def __init__(self, proc: str = "/proc", sys: str = "/sys", log_dir: str | None = None,
                 run=None) -> None:
        self.proc = proc
        self.sys = sys
        self.log_dir = log_dir
        self._run = run or _run_cmd  # (argv) -> stdout or None; tests pass a fake
        self._cpu_prev: tuple[int, int] | None = None
        self._proc_prev: dict[int, tuple[float, int]] = {}
        self._ips: list[str] = []
        self._ips_time = -1e9
        try:
            self._ticks = os.sysconf("SC_CLK_TCK")
        except (AttributeError, ValueError, OSError):
            self._ticks = 100

    # ------------------------------------------------------------ pieces
    def cpu_percent(self) -> float | None:
        text = _read(os.path.join(self.proc, "stat"))
        if not text:
            return None
        parts = [int(v) for v in text.splitlines()[0].split()[1:]]
        idle = parts[3] + (parts[4] if len(parts) > 4 else 0)  # idle + iowait
        total = sum(parts[:8])
        prev, self._cpu_prev = self._cpu_prev, (total, idle)
        if prev is None or total <= prev[0]:
            return None
        return round(100.0 * (1.0 - (idle - prev[1]) / (total - prev[0])), 1)

    def load(self) -> list[float] | None:
        text = _read(os.path.join(self.proc, "loadavg"))
        return [float(v) for v in text.split()[:3]] if text else None

    def memory(self) -> dict | None:
        text = _read(os.path.join(self.proc, "meminfo"))
        if not text:
            return None
        info = {}
        for line in text.splitlines():
            key, _, rest = line.partition(":")
            if rest.strip():
                info[key] = int(rest.split()[0])  # kB
        total, avail = info.get("MemTotal"), info.get("MemAvailable")
        if not total or avail is None:
            return None
        return {"total_mb": round(total / 1024), "available_mb": round(avail / 1024),
                "used_percent": round(100.0 * (1 - avail / total), 1)}

    def temperatures(self) -> dict[str, float]:
        temps = {}
        for zone in sorted(glob.glob(os.path.join(self.sys, "class", "thermal", "thermal_zone*"))):
            kind, value = _read(os.path.join(zone, "type")), _read(os.path.join(zone, "temp"))
            if kind and value and value.strip().lstrip("-").isdigit():
                name = kind.strip()
                if name in temps:
                    name = f"{name}_{os.path.basename(zone)}"
                temps[name] = round(int(value) / 1000.0, 1)
        return temps

    def wifi(self) -> list[dict]:
        """/proc/net/wireless plus `iw dev <if> link` and power_save.

        The level column is dBm only when negative: rtl8821cu (UP4000 USB
        dongle) prints 52 while iw says -48 dBm, so a positive level is kept
        as signal_level and the dBm value comes from iw. Noise -256 = unknown.
        """
        text = _read(os.path.join(self.proc, "net", "wireless"))
        out = []
        for line in (text or "").splitlines()[2:]:
            iface, _, rest = line.partition(":")
            fields = rest.split()
            if len(fields) < 3:
                continue
            try:
                level = float(fields[2].rstrip("."))
                noise = float(fields[3].rstrip(".")) if len(fields) > 3 else None
                entry = {"interface": iface.strip(), "link_quality": float(fields[1].rstrip(".")),
                         "signal_dbm": level if level < 0 else None,
                         "noise_dbm": noise if noise is not None and -256 < noise < 0 else None}
            except ValueError:
                continue
            if level >= 0:
                entry["signal_level"] = level
            entry.update(self._iw(entry["interface"]))
            out.append(entry)
        return out

    def _iw(self, iface: str) -> dict:
        info: dict = {}
        for line in (self._run(["iw", "dev", iface, "link"]) or "").splitlines():
            key, _, value = line.strip().partition(":")
            words = value.split()
            try:
                if key == "signal" and words:
                    info["signal_dbm"] = float(words[0])
                elif key == "tx bitrate" and words:
                    info["tx_mbit"] = float(words[0])
                elif key == "rx bitrate" and words:
                    info["rx_mbit"] = float(words[0])
                elif key == "freq" and words:
                    info["freq_mhz"] = float(words[0])
                elif key == "SSID":
                    info["ssid"] = value.strip()
            except ValueError:
                continue
        state = (self._run(["iw", "dev", iface, "get", "power_save"]) or "").strip().lower()
        if state.startswith("power save:"):
            info["power_save"] = state.split(":", 1)[1].strip() == "on"
        return info

    def uptime_s(self) -> float | None:
        text = _read(os.path.join(self.proc, "uptime"))
        return float(text.split()[0]) if text else None

    def disk(self) -> dict | None:
        if not self.log_dir:
            return None
        try:
            usage = shutil.disk_usage(self.log_dir)
        except OSError:
            return None
        return {"path": str(self.log_dir), "free_gb": round(usage.free / 1e9, 2),
                "total_gb": round(usage.total / 1e9, 2),
                "used_percent": round(100.0 * usage.used / usage.total, 1) if usage.total else None}

    def ip_addresses(self) -> list[str]:
        if time.monotonic() - self._ips_time < 10.0:
            return self._ips
        self._ips_time = time.monotonic()
        try:
            text = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True,
                                  text=True, timeout=1.0).stdout
            self._ips = [f"{line.split()[1]} {line.split()[3]}" for line in text.splitlines()
                         if len(line.split()) > 3 and not line.split()[1] == "lo"]
        except (OSError, subprocess.SubprocessError):
            pass
        return self._ips

    def logger_processes(self) -> dict[str, dict]:
        """CPU % (of one core) and RSS of this logger's processes."""
        out: dict[str, dict] = {}
        now = time.monotonic()
        seen = set()
        for entry in os.listdir(self.proc) if os.path.isdir(self.proc) else []:
            if not entry.isdigit():
                continue
            cmd = _read(os.path.join(self.proc, entry, "cmdline")) or ""
            name = next((n for n in LOGGER_PROCESSES if f"uwb_px4_logger/{n}" in cmd), None)
            stat = _read(os.path.join(self.proc, entry, "stat")) if name else None
            if not stat:
                continue
            fields = stat.rsplit(")", 1)[-1].split()
            ticks = int(fields[11]) + int(fields[12])        # utime + stime
            rss_mb = int(fields[21]) * 4096 / 1e6             # pages of 4 KiB
            pid = int(entry)
            seen.add(pid)
            prev = self._proc_prev.get(pid)
            self._proc_prev[pid] = (now, ticks)
            cpu = None
            if prev and now > prev[0]:
                cpu = round(100.0 * (ticks - prev[1]) / self._ticks / (now - prev[0]), 1)
            out[name] = {"pid": pid, "cpu_percent": cpu, "rss_mb": round(rss_mb, 1)}
        self._proc_prev = {pid: v for pid, v in self._proc_prev.items() if pid in seen}
        return out

    def sample(self) -> dict:
        temps = self.temperatures()
        cpu_temps = [v for k, v in temps.items() if any(s in k for s in ("x86_pkg", "coretemp", "cpu", "soc"))]
        return {
            "hostname": socket.gethostname(),
            "ip": self.ip_addresses(),
            "uptime_s": self.uptime_s(),
            "cpu_percent": self.cpu_percent(),
            "cpu_count": os.cpu_count(),
            "load": self.load(),
            "cpu_temp_c": max(cpu_temps) if cpu_temps else (max(temps.values()) if temps else None),
            "temperatures_c": temps,
            "memory": self.memory(),
            "disk": self.disk(),
            "wifi": self.wifi(),
            "processes": self.logger_processes(),
        }


def session_files(folder: Path) -> dict[str, int]:
    try:
        return {p.name: p.stat().st_size for p in sorted(Path(folder).iterdir()) if p.is_file()}
    except OSError:
        return {}


MAX_PART_BYTES = 1200  # one UDP datagram without IP fragmentation on WiFi


def _dumps(value) -> str:
    return json.dumps(value, separators=(",", ":"), default=str)


def split_parts(path: str, data, limit: int = MAX_PART_BYTES, measure=None) -> list[tuple[str, dict]]:
    """Split dictionaries using the complete encoded datagram's byte budget.

    Atomic values (lists/strings) are kept intact here; StatusSender fragments
    an oversized atomic part for lossless, bounded reassembly at the receiver.

    On 07/11 an 8 KB status datagram became ~6 IP fragments and only 19 % of
    them survived WiFi. Each part merges into the receiver's state at `path`
    (dict.update), so losing one part only delays that part.
    """
    measure = measure or (lambda path, value: len(_dumps(value).encode("utf-8")))
    if not isinstance(data, dict):
        return [(path, {"value": data})]
    if measure(path, data) <= limit:
        return [(path, data)]
    parts: list[tuple[str, dict]] = []
    nested: list[tuple[str, dict]] = []
    current: dict = {}
    for key, value in data.items():
        if measure(path, {key: value}) > limit and isinstance(value, dict):
            nested.extend(split_parts(f"{path}.{key}", value, limit, measure))
            continue
        if current and measure(path, dict(current, **{key: value})) > limit:
            parts.append((path, current))
            current = {}
        current[key] = value
    if current:
        parts.append((path, current))
    return parts + nested


class StatusSender:
    """Send status sections as small JSON parts; drops silently while no ground.

    Datagram: {"kind", "version": 2, "seq", "time_unix", "part": "px4.messages",
    "data": {...}}. The receiver merges data into its state at that path.
    """

    def __init__(self, port: int = 15557, max_part_bytes: int = MAX_PART_BYTES) -> None:
        self.port = port
        self.max_part_bytes = max_part_bytes
        self.seq = 0
        self.sent = 0
        self.errors = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._last_broadcast = -1e9

    def datagrams(self, sections: dict) -> list[bytes]:
        self.seq += 1
        now = time.time()
        out = []
        def encode(path, data, fragment=None):
            msg = {"kind": STATUS_KIND, "version": STATUS_VERSION, "seq": self.seq,
                   "time_unix": now, "part": path, "data": data}
            if fragment is not None:
                msg["fragment"] = fragment
            return _dumps(msg).encode("utf-8")

        measure = lambda path, data: len(encode(path, data))
        for name, data in sections.items():
            if data is None:
                continue  # that node has not reported yet
            for path, part in split_parts(name, data, self.max_part_bytes, measure):
                datagram = encode(path, part)
                if len(datagram) <= self.max_part_bytes:
                    out.append(datagram)
                    continue
                # A long STATUSTEXT list or scalar cannot be split by dict path.
                # Reserve the full envelope and worst-case fragment counters.
                ident = len(out)
                header = encode(path, "", {"id": ident, "index": 255, "count": 256})
                chunk_bytes = ((self.max_part_bytes - len(header)) // 4) * 3
                if chunk_bytes <= 0:
                    raise ValueError("status datagram limit is smaller than its envelope")
                raw = _dumps(part).encode("utf-8")
                count = (len(raw) + chunk_bytes - 1) // chunk_bytes
                if count > 256:
                    raise ValueError("status part exceeds 256 fragments")
                for index in range(count):
                    chunk = base64.b64encode(raw[index * chunk_bytes:(index + 1) * chunk_bytes]).decode("ascii")
                    out.append(encode(path, chunk, {"id": ident, "index": index, "count": count}))
        return out

    def target(self, ground_host: str, broadcast: str = "", broadcast_period_s: float = 1.0) -> str | None:
        """Unicast to the ground; with no ground known yet, broadcast at most
        once per period so the tab works before the UWB WiFi link is open."""
        if ground_host:
            return ground_host
        if not broadcast or time.monotonic() - self._last_broadcast < broadcast_period_s:
            return None
        self._last_broadcast = time.monotonic()
        return broadcast

    def send(self, sections: dict, ground_host: str, broadcast: str = "",
             broadcast_period_s: float = 1.0, part_gap_s: float = 0.0) -> int:
        """part_gap_s spaces the parts of one cycle: on the 07/11 WiFi a burst of
        8 x 1.1 kB datagrams arrived 72% of the time, 80% with 2 ms gaps."""
        host = self.target(ground_host, broadcast, broadcast_period_s)
        if host is None:
            return 0
        sent = 0
        for index, datagram in enumerate(self.datagrams(sections)):
            if index and part_gap_s > 0:
                time.sleep(part_gap_s)
            try:
                self._sock.sendto(datagram, (host, self.port))
                sent += 1
            except OSError:
                self.errors += 1
        self.sent += sent
        return sent

    def close(self) -> None:
        self._sock.close()
