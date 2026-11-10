"""New_GUI "UP7000" tab: QGroundControl / Mission Planner-style Flight Display,
EKF2 Health Monitor, Companion Logger Inspector & PX4 .ulg Flight Log Replay.

The UP7000 node `up4000_status` sends JSON status updates ~4 times per second
(and a 1 Hz broadcast before it knows this PC): host health, logger counters,
session files and the latest PX4 values. Nothing is ever sent back.
In addition, users can load any PX4 `.ulg` file directly into this tab to
replay and scrub historical flights with the Primary Flight Display (PFD),
Compass / 2D Local NED Trajectory Radar, EKF2 innovation gauges, and tree view.
"""
from __future__ import annotations

import base64
import bisect
import json
import math
import os
from pathlib import Path
import time
from typing import Any

from PyQt6.QtCore import QPointF, QRectF, QObject, QSize, QTimer, Qt, pyqtSignal
from PyQt6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QFontDatabase,
    QLinearGradient,
    QPainter,
    QPainterPath,
    QPen,
    QPolygonF,
)
from PyQt6.QtNetwork import QHostAddress, QUdpSocket
from PyQt6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QSplitter,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

UP4000_STATUS_PORT = 15557
STATUS_KIND = "up4000_status"
STALE_S = 2.0

# PX4 custom_mode: main mode in bits 16-23, sub mode in bits 24-31.
PX4_MAIN_MODES = {
    1: "MANUAL",
    2: "ALTCTL",
    3: "POSCTL",
    4: "AUTO",
    5: "ACRO",
    6: "OFFBOARD",
    7: "STABILIZED",
    8: "RATTITUDE",
    9: "SIMPLE",
    10: "TERMINATION",
}
PX4_AUTO_SUB = {
    1: "READY",
    2: "TAKEOFF",
    3: "LOITER",
    4: "MISSION",
    5: "RTL",
    6: "LAND",
    8: "FOLLOW_TARGET",
    9: "PRECLAND",
    10: "VTOL_TAKEOFF",
}
# Mapping from PX4 ULog vehicle_status.nav_state to MAVLink custom_mode
PX4_NAV_STATE_TO_CUSTOM_MODE = {
    0: 1 << 16,                  # MANUAL
    1: 2 << 16,                  # ALTCTL
    2: 3 << 16,                  # POSCTL
    3: (4 << 16) | (4 << 24),    # AUTO.MISSION
    4: (4 << 16) | (3 << 24),    # AUTO.LOITER
    5: (4 << 16) | (5 << 24),    # AUTO.RTL
    12: 5 << 16,                 # ACRO
    14: 6 << 16,                 # OFFBOARD
    15: 7 << 16,                 # STABILIZED
    16: 8 << 16,                 # RATTITUDE
    17: (4 << 16) | (2 << 24),   # AUTO.TAKEOFF
    18: (4 << 16) | (6 << 24),   # AUTO.LAND
    19: (4 << 16) | (8 << 24),   # AUTO.FOLLOW_TARGET
    20: (4 << 16) | (9 << 24),   # AUTO.PRECLAND
    21: 10 << 16,                # TERMINATION
    22: (4 << 16) | (10 << 24),  # AUTO.VTOL_TAKEOFF
}

# MAVLink ESTIMATOR_STATUS_FLAGS
EKF_FLAGS = (
    (1, "attitude"),
    (2, "vel horiz"),
    (4, "vel vert"),
    (8, "pos horiz rel"),
    (16, "pos horiz abs"),
    (32, "pos vert abs"),
    (64, "pos vert AGL"),
    (128, "const pos mode"),
    (256, "pred pos horiz rel"),
    (512, "pred pos horiz abs"),
    (1024, "GPS glitch"),
    (2048, "accel error"),
)
EKF_BAD_FLAGS = {128, 1024, 2048}
LANDED = {0: "undefined", 1: "on ground", 2: "in air", 3: "taking off", 4: "landing"}
SEVERITY = {
    0: "EMERG",
    1: "ALERT",
    2: "CRIT",
    3: "ERROR",
    4: "WARN",
    5: "NOTICE",
    6: "INFO",
    7: "DEBUG",
}


def px4_mode_name(custom_mode: int) -> str:
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    name = PX4_MAIN_MODES.get(main, f"mode {main}")
    if main == 4:
        name += "." + PX4_AUTO_SUB.get(sub, str(sub))
    return name


def quat_to_euler_deg(q) -> tuple[float, float, float] | None:
    """MAVLink quaternion (w, x, y, z) -> roll, pitch, yaw in degrees."""
    if not q or len(q) < 4 or any(v is None for v in q[:4]):
        return None
    w, x, y, z = q[:4]
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (w, x, y, z)):
        return None
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def ekf_flags(flags: int) -> list[tuple[str, bool, bool]]:
    """[(name, set, is_bad_when_set)]"""
    return [(name, bool(flags & bit), bit in EKF_BAD_FLAGS) for bit, name in EKF_FLAGS]


def _fmt(value: Any, digits: int = 2, unit: str = "") -> str:
    if value is None:
        return "--"
    if isinstance(value, float):
        if not math.isfinite(value):
            return "--"
        return f"{value:.{digits}f}{unit}"
    return f"{value}{unit}"


def _tree_value(value: Any) -> str:
    """Compact text for the all-values tree: 47, 61.5, 0.0123, True."""
    if isinstance(value, float):
        if not math.isfinite(value):
            return str(value)
        if value.is_integer() and abs(value) < 1e15:
            return str(int(value))
        return f"{value:.4f}".rstrip("0").rstrip(".")
    return "--" if value is None else str(value)


def _ratio_html(name: str, value: Any) -> str:
    """EKF innovation test ratio: >1 rejected by EKF2, >0.5 close to it."""
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return f"{name} --"
    color = "#2e9e44" if value < 0.5 else ("#d98200" if value < 1.0 else "#d93025")
    return f'{name} <span style="color:{color}"><b>{value:.2f}</b></span>'


# ---------------------------------------------------------------------------
# PX4 .ulg Flight Log Parser -> UP7000 status snapshots
# ---------------------------------------------------------------------------
def load_ulog_session(path: str | Path, step_s: float = 0.20) -> dict[str, Any]:
    """Parse a PX4 `.ulg` file into time-indexed `up4000_status` snapshots for GUI replay.

    Returns a dictionary with metadata (`path`, `duration_s`, `hw_ver`, `frames`, `ned_trail`,
    `arm_frame_idx`, `max_alt_frame_idx`, `max_ratio_frame_idx`, `summary`).
    """
    from pyulog import ULog

    p = Path(path)
    ulog = ULog(str(p))
    t0_us = int(ulog.start_timestamp)
    t1_us = int(ulog.last_timestamp)
    duration_s = max(0.0, (t1_us - t0_us) / 1e6)

    def _topic(name: str):
        matches = [d for d in ulog.data_list if d.name == name and len(d.data.get("timestamp", [])) > 0]
        if not matches:
            return None
        # Pick the instance with the most samples
        return max(matches, key=lambda d: len(d.data["timestamp"]))

    t_vstat = _topic("vehicle_status")
    t_land = _topic("vehicle_land_detected")
    t_lpos = _topic("vehicle_local_position")
    t_att = _topic("vehicle_attitude")
    t_batt = _topic("battery_status")
    t_est = _topic("estimator_status")
    t_imu = _topic("vehicle_imu_status")
    t_cpu = _topic("cpuload")

    def _idx_at(topic, ts_us: int) -> int | None:
        if topic is None:
            return None
        ts_arr = topic.data["timestamp"]
        pos = bisect.bisect_right(ts_arr, ts_us) - 1
        return max(0, pos) if len(ts_arr) > 0 else None

    def _val(topic, idx: int | None, key: str, default: Any = None) -> Any:
        if topic is None or idx is None or key not in topic.data:
            return default
        v = topic.data[key][idx]
        if hasattr(v, "item"):
            v = v.item()
        if isinstance(v, float) and not math.isfinite(v):
            return default
        return v

    # Compute topic rates
    rates_hz: dict[str, float] = {}
    if duration_s > 0.1:
        if t_lpos is not None:
            rates_hz["ODOMETRY"] = round(len(t_lpos.data["timestamp"]) / duration_s, 1)
        if t_est is not None:
            rates_hz["ESTIMATOR_STATUS"] = round(len(t_est.data["timestamp"]) / duration_s, 1)

    # Full NED trail (decimated to ~400 points max for fast 2D radar rendering)
    ned_trail: list[tuple[float, float, float]] = []
    if t_lpos is not None:
        xs = t_lpos.data.get("x", [])
        ys = t_lpos.data.get("y", [])
        zs = t_lpos.data.get("z", [])
        n_pts = len(xs)
        stride = max(1, n_pts // 450)
        for i in range(0, n_pts, stride):
            x_v, y_v, z_v = float(xs[i]), float(ys[i]), float(zs[i])
            if math.isfinite(x_v) and math.isfinite(y_v) and math.isfinite(z_v):
                ned_trail.append((x_v, y_v, z_v))

    # Logged messages
    raw_msgs = []
    base_unix = time.time() - duration_s
    for m in ulog.logged_messages:
        lvl = m.log_level
        if isinstance(lvl, int) and 48 <= lvl <= 55:
            sev = lvl - 48
        elif isinstance(lvl, str) and lvl.isdigit():
            sev = int(lvl)
        else:
            sev = 6
        rel_s = max(0.0, (int(m.timestamp) - t0_us) / 1e6)
        raw_msgs.append({
            "timestamp_us": int(m.timestamp),
            "time_unix": base_unix + rel_s,
            "severity": sev,
            "text": str(m.message).strip(),
        })

    hw_ver = str(ulog.msg_info_dict.get("ver_hw", "PX4_FMU"))
    sys_name = str(ulog.msg_info_dict.get("sys_name", "PX4"))
    sw_branch = str(ulog.msg_info_dict.get("ver_sw_branch", ""))
    file_size = p.stat().st_size if p.exists() else 0
    total_ulog_msgs = sum(len(d.data.get("timestamp", [])) for d in ulog.data_list)

    n_steps = max(1, int(math.ceil(duration_s / max(0.05, step_s))) + 1)
    frames: list[dict[str, Any]] = []
    arm_frame_idx = 0
    found_arm = False
    max_alt = -1e9
    max_alt_frame_idx = 0
    max_ratio = -1.0
    max_ratio_frame_idx = 0

    for step_idx in range(n_steps):
        rel_s = min(duration_s, step_idx * step_s)
        ts_us = t0_us + int(rel_s * 1e6)

        i_vs = _idx_at(t_vstat, ts_us)
        i_ld = _idx_at(t_land, ts_us)
        i_lp = _idx_at(t_lpos, ts_us)
        i_at = _idx_at(t_att, ts_us)
        i_bt = _idx_at(t_batt, ts_us)
        i_es = _idx_at(t_est, ts_us)
        i_im = _idx_at(t_imu, ts_us)
        i_cp = _idx_at(t_cpu, ts_us)

        arming_state = int(_val(t_vstat, i_vs, "arming_state", 1) or 1)
        armed = arming_state == 2
        if armed and not found_arm:
            found_arm = True
            arm_frame_idx = step_idx
        nav_state = int(_val(t_vstat, i_vs, "nav_state", 0) or 0)
        custom_mode = PX4_NAV_STATE_TO_CUSTOM_MODE.get(nav_state, 1 << 16)
        base_mode = (128 | 1) if armed else 1

        landed_flag = _val(t_land, i_ld, "landed", 1)
        landed_state = 1 if bool(landed_flag) else 2

        volt_v = _val(t_batt, i_bt, "voltage_filtered_v")
        if volt_v is None or volt_v <= 0.1:
            volt_v = _val(t_batt, i_bt, "voltage_v", 0.0)
        volt_mv = int(round(float(volt_v) * 1000)) if volt_v and volt_v > 0.1 else None
        rem_frac = _val(t_batt, i_bt, "remaining")
        rem_pct = int(round(float(rem_frac) * 100)) if rem_frac is not None and rem_frac >= 0 else -1
        curr_a = _val(t_batt, i_bt, "current_filtered_a", 0.0)

        x = _val(t_lpos, i_lp, "x", 0.0)
        y = _val(t_lpos, i_lp, "y", 0.0)
        z = _val(t_lpos, i_lp, "z", 0.0)
        vx = _val(t_lpos, i_lp, "vx", 0.0)
        vy = _val(t_lpos, i_lp, "vy", 0.0)
        vz = _val(t_lpos, i_lp, "vz", 0.0)
        eph = _val(t_lpos, i_lp, "eph")
        epv = _val(t_lpos, i_lp, "epv")

        alt_m = -float(z or 0.0)
        if alt_m > max_alt:
            max_alt = alt_m
            max_alt_frame_idx = step_idx

        q = [
            _val(t_att, i_at, "q[0]", 1.0),
            _val(t_att, i_at, "q[1]", 0.0),
            _val(t_att, i_at, "q[2]", 0.0),
            _val(t_att, i_at, "q[3]", 0.0),
        ]

        vel_r = _val(t_est, i_es, "vel_test_ratio")
        pos_h_r = _val(t_est, i_es, "pos_test_ratio")
        pos_v_r = _val(t_est, i_es, "hgt_test_ratio")
        mag_r = _val(t_est, i_es, "mag_test_ratio")
        hagl_r = _val(t_est, i_es, "hagl_test_ratio")
        tas_r = _val(t_est, i_es, "tas_test_ratio")
        sol_flags = int(_val(t_est, i_es, "solution_status_flags", 0) or 0)

        for r_val in (vel_r, pos_h_r, pos_v_r, mag_r, hagl_r, tas_r):
            if isinstance(r_val, (int, float)) and math.isfinite(r_val) and r_val > max_ratio:
                max_ratio = float(r_val)
                max_ratio_frame_idx = step_idx

        # Vibration from vehicle_imu_status
        var_ax = _val(t_imu, i_im, "var_accel[0]", 0.0) or 0.0
        var_ay = _val(t_imu, i_im, "var_accel[1]", 0.0) or 0.0
        var_az = _val(t_imu, i_im, "var_accel[2]", 0.0) or 0.0
        vib_x = math.sqrt(max(0.0, float(var_ax)))
        vib_y = math.sqrt(max(0.0, float(var_ay)))
        vib_z = math.sqrt(max(0.0, float(var_az)))
        clip_0 = int(_val(t_imu, i_im, "accel_clipping[0]", 0) or 0)
        clip_1 = int(_val(t_imu, i_im, "accel_clipping[1]", 0) or 0)
        clip_2 = int(_val(t_imu, i_im, "accel_clipping[2]", 0) or 0)
        imu_temp = _val(t_imu, i_im, "temperature_accel", 42.0)

        cpu_load = float(_val(t_cpu, i_cp, "load", 0.45) or 0.45) * 100.0
        ram_usage = float(_val(t_cpu, i_cp, "ram_usage", 0.50) or 0.50) * 100.0

        msgs_so_far = [
            {"time_unix": m["time_unix"], "severity": m["severity"], "text": m["text"]}
            for m in raw_msgs
            if m["timestamp_us"] <= ts_us
        ]
        if not msgs_so_far and raw_msgs:
            msgs_so_far = [
                {"time_unix": m["time_unix"], "severity": m["severity"], "text": m["text"]}
                for m in raw_msgs[:3]
            ]

        snap = {
            "kind": STATUS_KIND,
            "version": 1,
            "seq": step_idx + 1,
            "time_unix": base_unix + rel_s,
            "ulog_rel_s": round(rel_s, 2),
            "ulog_duration_s": round(duration_s, 2),
            "host": {
                "hostname": f"{hw_ver} ({sys_name})",
                "cpu_percent": round(cpu_load, 1),
                "cpu_temp_c": round(float(imu_temp or 40.0), 1),
                "cpu_count": 1,
                "load": [round(cpu_load / 100.0, 2)],
                "memory": {
                    "used_percent": round(ram_usage, 1),
                    "available_mb": round((100.0 - ram_usage) * 2.56, 1),
                    "total_mb": 256,
                },
                "disk": {"free_gb": 16.0, "total_gb": 32.0, "used_percent": 50.0},
                "wifi": [{
                    "interface": "ULog Replay",
                    "ssid": p.name,
                    "signal_dbm": None,
                    "signal_level": 100,
                    "link_quality": 100,
                }],
                "processes": {
                    "px4_fmu": {"cpu_percent": round(cpu_load, 1)},
                },
            },
            "session": {
                "folder": str(p.parent),
                "files": {p.name: file_size},
            },
            "uwb_relay": {
                "tag_port": "ULog offline",
                "ground": f"{sys_name} {sw_branch}".strip(),
                "counts": {"tag_bytes": 0, "udp_datagrams": step_idx + 1},
                "parsed": {"frame_0x18": 0},
                "parser": {"crc_errors": 0},
            },
            "px4_logger": {
                "px4_port": hw_ver,
                "ground": p.name,
                "ulog_part": 1,
                "ulog": {"chunks": len(ulog.data_list), "bytes": file_size, "dropped_chunks": len(ulog.dropouts)},
                "counts": {"messages": total_ulog_msgs, "bad_data": len(ulog.dropouts), "timesync_samples": n_steps},
            },
            "px4": {
                "connected": True,
                "port": hw_ver,
                "rates_hz": rates_hz,
                "messages": {
                    "HEARTBEAT": {
                        "age_s": 0.05,
                        "fields": {"base_mode": base_mode, "custom_mode": custom_mode},
                    },
                    "SYS_STATUS": {
                        "age_s": 0.05,
                        "fields": {
                            "voltage_battery": volt_mv,
                            "battery_remaining": rem_pct,
                            "current_battery": int(round(float(curr_a or 0.0) * 100)),
                        },
                    },
                    "EXTENDED_SYS_STATE": {
                        "age_s": 0.05,
                        "fields": {"landed_state": landed_state},
                    },
                    "ODOMETRY": {
                        "age_s": 0.02,
                        "fields": {
                            "x": float(x or 0.0),
                            "y": float(y or 0.0),
                            "z": float(z or 0.0),
                            "vx": float(vx or 0.0),
                            "vy": float(vy or 0.0),
                            "vz": float(vz or 0.0),
                            "q": [float(v) for v in q],
                            "frame_id": 1,
                        },
                    },
                    "ESTIMATOR_STATUS": {
                        "age_s": 0.05,
                        "fields": {
                            "flags": sol_flags,
                            "vel_ratio": vel_r,
                            "pos_horiz_ratio": pos_h_r,
                            "pos_vert_ratio": pos_v_r,
                            "mag_ratio": mag_r,
                            "hagl_ratio": hagl_r,
                            "tas_ratio": tas_r,
                            "pos_horiz_accuracy": eph,
                            "pos_vert_accuracy": epv,
                        },
                    },
                    "VIBRATION": {
                        "age_s": 0.05,
                        "fields": {
                            "vibration_x": round(vib_x, 3),
                            "vibration_y": round(vib_y, 3),
                            "vibration_z": round(vib_z, 3),
                            "clipping_0": clip_0,
                            "clipping_1": clip_1,
                            "clipping_2": clip_2,
                        },
                    },
                },
                "statustext": msgs_so_far[-12:],
            },
        }
        frames.append(snap)

    return {
        "path": str(p),
        "name": p.name,
        "duration_s": duration_s,
        "step_s": step_s,
        "hw_ver": hw_ver,
        "frames": frames,
        "ned_trail": ned_trail,
        "arm_frame_idx": arm_frame_idx,
        "max_alt_frame_idx": max_alt_frame_idx,
        "max_ratio_frame_idx": max_ratio_frame_idx,
        "max_alt_m": round(max_alt, 2),
        "max_ratio": round(max_ratio, 2),
    }


# ---------------------------------------------------------------------------
# UDP Listener
# ---------------------------------------------------------------------------
class Up4000StatusListener(QObject):
    """Receives the status datagrams on the GUI thread (QUdpSocket)."""

    status_received = pyqtSignal(dict)

    def __init__(self, port: int = UP4000_STATUS_PORT, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.port = port
        self.latest: dict | None = None
        self.state: dict = {}
        self.latest_rx: float | None = None
        self.sender: str | None = None
        self.packets = 0
        self.bad = 0
        self._rate_window: list[float] = []
        self._field_rx: dict[str, float] = {}
        self._fragments: dict[tuple, dict] = {}
        self.sock = QUdpSocket(self)
        flags = QUdpSocket.BindFlag.ShareAddress | QUdpSocket.BindFlag.ReuseAddressHint
        self.bound = self.sock.bind(QHostAddress(QHostAddress.SpecialAddress.AnyIPv4), port, flags)
        self.error = None if self.bound else self.sock.errorString()
        if self.bound:
            self.port = self.sock.localPort()
        self.sock.readyRead.connect(self._read)

    def _read(self) -> None:
        while self.sock.hasPendingDatagrams():
            datagram = self.sock.receiveDatagram()
            self.handle_datagram(bytes(datagram.data()), datagram.senderAddress().toString())

    def handle_datagram(self, data: bytes, sender: str) -> None:
        try:
            msg = json.loads(data)
        except ValueError:
            self.bad += 1
            return
        if not isinstance(msg, dict) or msg.get("kind") != STATUS_KIND:
            self.bad += 1
            return
        now = time.monotonic()
        if "fragment" in msg:
            # Never refresh displayed values until every fragment arrives.
            self._fragments = {k: v for k, v in self._fragments.items() if now - v["rx"] < STALE_S}
            try:
                frag = msg["fragment"]
                index, count = int(frag["index"]), int(frag["count"])
                if not 0 <= index < count <= 256:
                    raise ValueError("invalid fragment index/count")
                chunk = base64.b64decode(msg["data"], validate=True)
                key = (sender, msg["seq"], msg["time_unix"], msg["part"], frag["id"])
                if key not in self._fragments:
                    if len(self._fragments) >= 64:
                        self._fragments.pop(next(iter(self._fragments)))
                    self._fragments[key] = {"rx": now, "count": count, "chunks": {}}
                pending = self._fragments[key]
                if pending["count"] != count:
                    raise ValueError("fragment count changed")
                pending["chunks"][index] = chunk
                if sum(map(len, pending["chunks"].values())) > 256 * 1200:
                    del self._fragments[key]
                    raise ValueError("fragment buffer too large")
                if len(pending["chunks"]) != count:
                    return
                del self._fragments[key]
                msg["data"] = json.loads(b"".join(pending["chunks"][i] for i in range(count)))
            except (ValueError, TypeError, KeyError):
                self.bad += 1
                return
        if "part" in msg:  # version 2: small parts merged into one state
            if not isinstance(msg.get("data"), dict):
                self.bad += 1
                return
            # Prevent an idle secondary board (e.g. old UP4000 'dronepif' broadcasting
            # on the same LAN) from overwriting the active UP7000 ('thoa') state.
            if (
                self.sender is not None
                and sender != self.sender
                and self.latest_rx is not None
                and (now - self.latest_rx) < STALE_S
            ):
                cur_host = (self.state.get("host") or {}).get("hostname")
                cur_active = (
                    cur_host == "thoa"
                    or bool((self.state.get("px4") or {}).get("connected"))
                    or bool((self.state.get("uwb_relay") or {}).get("tag_port"))
                )
                inc_data = msg.get("data") or {}
                inc_is_thoa = inc_data.get("hostname") == "thoa" or "/thoa/" in str(inc_data.get("folder", ""))
                if cur_active and not inc_is_thoa:
                    return
                # Switching to a higher-priority sender: reset merged state cleanly
                self.state = {}
                self._field_rx.clear()
            elif (
                (self.state.get("host") or {}).get("hostname") == "thoa"
                and self.section_age_s("host") < STALE_S * 2.5
            ):
                part_name = str(msg.get("part"))
                part_data = msg.get("data") or {}
                if part_name == "host" and part_data.get("hostname") == "dronepif":
                    return
                if part_name == "session" and "/dronepif/" in str(part_data.get("folder", "")):
                    return
                if (
                    part_name == "uwb_relay"
                    and part_data.get("tag_port") is None
                    and (self.state.get("uwb_relay") or {}).get("tag_port") is not None
                    and self.section_age_s("uwb_relay") < STALE_S
                ):
                    return
                if (
                    part_name == "px4_logger"
                    and part_data.get("px4_port") is None
                    and (self.state.get("px4_logger") or {}).get("px4_port") is not None
                    and self.section_age_s("px4_logger") < STALE_S
                ):
                    return
            for key in msg["data"]:
                prefix = str(msg["part"]) + "." + key
                self._field_rx = {
                    k: t
                    for k, t in self._field_rx.items()
                    if k != prefix and not k.startswith(prefix + ".")
                }
            self._merge(str(msg["part"]), msg["data"])
            self._mark_received(str(msg["part"]), msg["data"], now)
        else:  # version 1: the whole state in one datagram
            self.state = msg
            self._field_rx.clear()
            self._mark_received("", msg, now)
        for key in ("kind", "version", "seq", "time_unix"):
            self.state[key] = msg.get(key)
        self.latest, self.latest_rx, self.sender = self.state, now, sender
        self.packets += 1
        self._rate_window = [t for t in self._rate_window if now - t < 5.0] + [now]
        self.status_received.emit(msg)

    def _mark_received(self, path: str, value, now: float) -> None:
        if isinstance(value, dict) and value:
            for key, child in value.items():
                self._mark_received(f"{path}.{key}" if path else key, child, now)
        else:
            self._field_rx[path] = now

    def section_age_s(self, path: str) -> float:
        times = [t for key, t in self._field_rx.items() if key == path or key.startswith(path + ".")]
        return time.monotonic() - min(times) if times else math.inf

    def message_age_s(self, name: str) -> float:
        path = f"px4.messages.{name}"
        msg = ((self.state.get("px4") or {}).get("messages") or {}).get(name) or {}
        elapsed = self.section_age_s(path)
        source_age = msg.get("age_s")
        source_rx = self._field_rx.get(path + ".age_s")
        age = (
            float(source_age) + time.monotonic() - source_rx
            if source_age is not None and source_rx is not None
            else elapsed
        )
        return max(elapsed, age)

    def _merge(self, path: str, data: dict) -> None:
        node = self.state
        for key in path.split("."):
            child = node.get(key)
            if not isinstance(child, dict):
                child = node[key] = {}
            node = child
        node.update(data)
        px4 = self.state.get("px4")
        if path == "px4" and isinstance(px4, dict) and isinstance(px4.get("messages"), dict):
            if px4.get("connected") is False:
                px4["messages"] = {}  # link lost: do not show old values as current
            elif isinstance(px4.get("message_types"), list):
                keep = set(px4["message_types"])
                px4["messages"] = {k: v for k, v in px4["messages"].items() if k in keep}

    def rate_hz(self) -> float:
        now = time.monotonic()
        recent = [t for t in self._rate_window if now - t < 5.0]
        return len(recent) / 5.0

    def close(self) -> None:
        self.sock.close()


# ---------------------------------------------------------------------------
# UI Reusable Widgets: StatCard, FlightHudWidget (PFD + Compass/NED Radar), RatioBar
# ---------------------------------------------------------------------------
class _Card(QFrame):
    """Same look as main_gui.StatCard (shared object names in the QSS)."""

    def __init__(self, title: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("stat_card")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setFixedHeight(88)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 6, 12, 6)
        layout.setSpacing(1)
        t = QLabel(title.upper())
        t.setObjectName("card_title")
        self.value = QLabel("--")
        self.value.setObjectName("metric_value")
        self.sub = QLabel("")
        self.sub.setObjectName("metric_sub")
        self.sub.setTextFormat(Qt.TextFormat.RichText)
        self.sub.setWordWrap(True)
        for w in (t, self.value, self.sub):
            layout.addWidget(w)
        layout.addStretch(1)

    def set(self, value: str, sub: str = "") -> None:
        if self.value.text() != value:
            self.value.setText(value)
        if self.sub.text() != sub:
            self.sub.setText(sub)


class _TelemetryCell(QFrame):
    """Compact digital instrument cell for QGC/Mission Planner quick-glance telemetry."""

    def __init__(self, label: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("sub_card")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setFixedHeight(54)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 4, 10, 4)
        lay.setSpacing(1)
        self.lbl_title = QLabel(label.upper())
        self.lbl_title.setObjectName("card_title")
        self.lbl_title.setStyleSheet("font-size: 10px; font-weight: 700;")
        self.lbl_val = QLabel("--")
        self.lbl_val.setObjectName("mono_readout")
        self.lbl_val.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
        self.lbl_sub = QLabel("")
        self.lbl_sub.setObjectName("metric_sub")
        self.lbl_sub.setStyleSheet("font-size: 10px;")
        lay.addWidget(self.lbl_title)
        lay.addWidget(self.lbl_val)
        lay.addWidget(self.lbl_sub)

    def set_data(self, val: str, sub: str = "", color: str | None = None) -> None:
        if self.lbl_val.text() != val:
            self.lbl_val.setText(val)
        if self.lbl_sub.text() != sub:
            self.lbl_sub.setText(sub)
        style = f"font-weight: 700; font-size: 12px; color: {color};" if color else "font-weight: 700; font-size: 12px;"
        if self.lbl_val.styleSheet() != style:
            self.lbl_val.setStyleSheet(style)


class _RatioBar(QWidget):
    """Visual horizontal gauge bar for EKF2 innovation test ratios (0.0 -> 1.5 scale)."""

    def __init__(self, label: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.label = label
        self.value: float | None = None
        self.dark_mode = False
        self.setMinimumHeight(20)
        self.setMaximumHeight(22)

    def set_value(self, val: float | None) -> None:
        clean = float(val) if isinstance(val, (int, float)) and math.isfinite(val) else None
        if self.value != clean:
            self.value = clean
            self.update()

    def set_dark_mode(self, dark: bool) -> None:
        if self.dark_mode != dark:
            self.dark_mode = dark
            self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        text_col = QColor("#E2E8F0" if self.dark_mode else "#1E293B")
        muted_col = QColor("#94A3B8" if self.dark_mode else "#64748B")
        track_bg = QColor("#1E293B" if self.dark_mode else "#E2E8F0")
        border_col = QColor("#334155" if self.dark_mode else "#CBD5E1")

        font = p.font()
        font.setPixelSize(11)
        font.setBold(True)
        p.setFont(font)
        p.setPen(text_col)
        p.drawText(QRectF(0, 0, 38, h), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self.label)

        bar_x = 40.0
        val_w = 34.0
        bar_w = max(24.0, w - bar_x - val_w - 4.0)
        bar_h = 10.0
        bar_y = (h - bar_h) / 2.0
        track_rect = QRectF(bar_x, bar_y, bar_w, bar_h)

        p.setPen(QPen(border_col, 1.0))
        p.setBrush(QBrush(track_bg))
        p.drawRoundedRect(track_rect, 4.0, 4.0)

        max_scale = 1.5
        if self.value is not None:
            if self.value < 0.5:
                fill_col = QColor("#16A34A")
            elif self.value < 1.0:
                fill_col = QColor("#D97706")
            else:
                fill_col = QColor("#DC2626")
            frac = max(0.02, min(1.0, self.value / max_scale))
            fill_rect = QRectF(bar_x, bar_y, bar_w * frac, bar_h)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(fill_col))
            p.drawRoundedRect(fill_rect, 4.0, 4.0)

            p.setPen(fill_col)
            p.drawText(
                QRectF(bar_x + bar_w + 2.0, 0, val_w, h),
                Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight,
                f"{self.value:.2f}",
            )
        else:
            p.setPen(muted_col)
            p.drawText(
                QRectF(bar_x + bar_w + 2.0, 0, val_w, h),
                Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight,
                "--",
            )

        # Draw threshold ticks at 0.5 (warn) and 1.0 (reject)
        for thresh, col_hex in ((0.5, "#D97706"), (1.0, "#DC2626")):
            tx = bar_x + (thresh / max_scale) * bar_w
            p.setPen(QPen(QColor(col_hex), 1.2, Qt.PenStyle.DashLine))
            p.drawLine(QPointF(tx, bar_y - 1.5), QPointF(tx, bar_y + bar_h + 1.5))


class FlightHudWidget(QWidget):
    """QGroundControl / Mission Planner-style Primary Flight Display (PFD)
    + Compass Heading Rose & 2D Local NED Trajectory Mini-Radar.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(180, 140)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.dark_mode = False

        # Flight state
        self.roll_deg: float = 0.0
        self.pitch_deg: float = 0.0
        self.yaw_deg: float = 0.0
        self.has_attitude: bool = False

        self.x_n: float = 0.0
        self.y_e: float = 0.0
        self.z_d: float = 0.0
        self.vx: float = 0.0
        self.vy: float = 0.0
        self.vz: float = 0.0
        self.has_position: bool = False

        self.armed: bool = False
        self.mode_str: str = "--"
        self.landed_str: str = "--"
        self.battery_str: str = "-- V"
        self.connected: bool = False

        # Trail of (x_north, y_east) in meters
        self.ned_trail: list[tuple[float, float]] = []

    def sizeHint(self) -> QSize:
        return QSize(560, 265)

    def set_dark_mode(self, dark: bool) -> None:
        if self.dark_mode != dark:
            self.dark_mode = dark
            self.update()

    def clear_trail(self) -> None:
        self.ned_trail.clear()
        self.update()

    def set_full_trail(self, pts: list[tuple[float, float, float]] | list[tuple[float, float]]) -> None:
        self.ned_trail = [(float(p[0]), float(p[1])) for p in pts]
        self.update()

    def update_telemetry(
        self,
        *,
        connected: bool,
        armed: bool | None,
        mode: str,
        landed: str,
        battery: str,
        euler: tuple[float, float, float] | None,
        pos_ned: tuple[float | None, float | None, float | None] | None,
        vel_ned: tuple[float | None, float | None, float | None] | None,
        append_trail: bool = True,
    ) -> None:
        self.connected = connected
        self.armed = bool(armed)
        self.mode_str = mode or "--"
        self.landed_str = landed or "--"
        self.battery_str = battery or "-- V"

        if euler is not None:
            self.roll_deg, self.pitch_deg, self.yaw_deg = euler
            self.has_attitude = True
        else:
            self.has_attitude = False

        if pos_ned is not None and all(isinstance(v, (int, float)) and math.isfinite(v) for v in pos_ned):
            self.x_n, self.y_e, self.z_d = float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2])
            self.has_position = True
            if append_trail:
                if not self.ned_trail or math.hypot(
                    self.x_n - self.ned_trail[-1][0], self.y_e - self.ned_trail[-1][1]
                ) >= 0.05:
                    self.ned_trail.append((self.x_n, self.y_e))
                    if len(self.ned_trail) > 600:
                        self.ned_trail = self.ned_trail[-600:]
        else:
            self.has_position = False

        if vel_ned is not None and all(isinstance(v, (int, float)) and math.isfinite(v) for v in vel_ned):
            self.vx, self.vy, self.vz = float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])
        else:
            self.vx = self.vy = self.vz = 0.0

        self.update()

    # ---------------------------------------------------------------- painting
    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w = float(self.width())
        h = float(self.height())
        gap = 8.0
        avail_w = max(100.0, w - gap)
        left_w = avail_w * 0.54
        right_w = avail_w - left_w

        pfd_rect = QRectF(0.0, 0.0, left_w, h)
        nav_rect = QRectF(left_w + gap, 0.0, right_w, h)

        self._paint_pfd(p, pfd_rect)
        self._paint_nav_radar(p, nav_rect)

    def _paint_pfd(self, p: QPainter, rect: QRectF) -> None:
        p.save()
        bezel_path = QPainterPath()
        bezel_path.addRoundedRect(rect.adjusted(1, 1, -1, -1), 10.0, 10.0)
        p.setClipPath(bezel_path)

        cx = rect.center().x()
        cy = rect.center().y()
        radius = max(rect.width(), rect.height()) * 0.9

        # 1. Sky & Ground rotated by roll and shifted by pitch
        p.save()
        p.translate(cx, cy)
        roll = self.roll_deg if self.has_attitude else 0.0
        pitch = max(-65.0, min(65.0, self.pitch_deg if self.has_attitude else 0.0))
        px_per_deg = rect.height() / 75.0
        pitch_offset = pitch * px_per_deg
        p.rotate(-roll)

        sky_grad = QLinearGradient(0, -radius, 0, pitch_offset)
        sky_grad.setColorAt(0.0, QColor("#0C4A6E"))
        sky_grad.setColorAt(1.0, QColor("#0284C7"))
        p.fillRect(QRectF(-radius, -radius, radius * 2, radius + pitch_offset), sky_grad)

        gnd_grad = QLinearGradient(0, pitch_offset, 0, radius)
        gnd_grad.setColorAt(0.0, QColor("#854D0E"))
        gnd_grad.setColorAt(1.0, QColor("#422006"))
        p.fillRect(QRectF(-radius, pitch_offset, radius * 2, radius - pitch_offset), gnd_grad)

        # Horizon line
        p.setPen(QPen(QColor("#F8FAFC"), 2.0))
        p.drawLine(QPointF(-radius, pitch_offset), QPointF(radius, pitch_offset))

        # Pitch ladder (-30 to +30 deg)
        font = QFont("Consolas", 8, QFont.Weight.Bold)
        p.setFont(font)
        max_ladder_half = max(14.0, min(24.0, rect.width() * 0.11))
        for deg in (-30, -20, -15, -10, -5, 5, 10, 15, 20, 30):
            y = pitch_offset - deg * px_per_deg
            if abs(y) > rect.height() * 0.34:
                continue
            half_len = max_ladder_half if deg % 10 == 0 else max_ladder_half * 0.55
            gap_w = 8.0
            pen_style = Qt.PenStyle.SolidLine if deg > 0 else Qt.PenStyle.DashLine
            p.setPen(QPen(QColor(248, 250, 252, 220), 1.4, pen_style))
            p.drawLine(QPointF(-half_len, y), QPointF(-gap_w, y))
            p.drawLine(QPointF(gap_w, y), QPointF(half_len, y))
            # Vertical end ticks
            tick_dy = 4.0 if deg > 0 else -4.0
            p.drawLine(QPointF(-half_len, y), QPointF(-half_len, y + tick_dy))
            p.drawLine(QPointF(half_len, y), QPointF(half_len, y + tick_dy))
            if deg % 10 == 0:
                lbl = f"{abs(deg)}"
                p.drawText(QRectF(-half_len - 20, y - 7, 18, 14), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, lbl)
                p.drawText(QRectF(half_len + 2, y - 7, 18, 14), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, lbl)
        p.restore()

        # 2. Roll Bank Angle Scale Arc at top
        arc_r = min(rect.width(), rect.height()) * 0.34
        p.save()
        p.translate(cx, cy)
        p.setPen(QPen(QColor(248, 250, 252, 180), 1.4))
        for bank_deg in (-60, -45, -30, -15, 0, 15, 30, 45, 60):
            p.save()
            p.rotate(bank_deg)
            tick_len = 8.0 if bank_deg % 30 == 0 else 5.0
            p.drawLine(QPointF(0, -arc_r), QPointF(0, -arc_r - tick_len))
            p.restore()
        # Roll pointer rotated by -roll
        p.save()
        p.rotate(-roll)
        tri = QPolygonF([
            QPointF(0.0, -arc_r + 1.0),
            QPointF(-5.5, -arc_r + 10.0),
            QPointF(5.5, -arc_r + 10.0),
        ])
        p.setBrush(QBrush(QColor("#FACC15")))
        p.setPen(QPen(QColor("#0F172A"), 1.0))
        p.drawPolygon(tri)
        p.restore()
        p.restore()

        # 3. Fixed Center Aircraft Reticle (QGroundControl yellow wings)
        p.save()
        p.translate(cx, cy)
        reticle_pen = QPen(QColor("#FACC15"), 2.8, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
        p.setPen(reticle_pen)
        p.drawLine(QPointF(-36, 0), QPointF(-12, 0))
        p.drawLine(QPointF(-12, 0), QPointF(-12, 6))
        p.drawLine(QPointF(12, 0), QPointF(36, 0))
        p.drawLine(QPointF(12, 0), QPointF(12, 6))
        p.setBrush(QBrush(QColor("#FACC15")))
        p.drawEllipse(QPointF(0, 0), 3.0, 3.0)
        p.restore()

        # 4. Left Speed Tape (Groundspeed Vxy & Climb Rate -Vz)
        tape_w = max(32.0, min(42.0, rect.width() * 0.19))
        tape_top = rect.top() + 28.0
        tape_bot = rect.bottom() - 24.0
        tape_h = max(46.0, tape_bot - tape_top)
        spd_rect = QRectF(rect.left() + 6.0, tape_top, tape_w, tape_h)
        p.setPen(QPen(QColor(255, 255, 255, 60), 1.0))
        p.setBrush(QBrush(QColor(15, 23, 42, 165)))
        p.drawRoundedRect(spd_rect, 5.0, 5.0)

        v_horiz = math.hypot(self.vx, self.vy) if self.has_position else 0.0
        v_up = -self.vz if self.has_position else 0.0
        p.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
        p.setPen(QColor("#94A3B8"))
        p.drawText(QRectF(spd_rect.left(), spd_rect.top() + 3, tape_w, 12), Qt.AlignmentFlag.AlignCenter, "SPD")
        # Speed ticks
        p.setPen(QColor("#E2E8F0"))
        for d_spd in range(-4, 5):
            tick_v = round(v_horiz) + d_spd
            if tick_v < 0:
                continue
            ty = cy - (tick_v - v_horiz) * 14.0
            if spd_rect.top() + 18 <= ty <= spd_rect.bottom() - 18:
                p.drawLine(QPointF(spd_rect.right() - 6, ty), QPointF(spd_rect.right(), ty))
                p.drawText(
                    QRectF(spd_rect.left() + 3, ty - 6, tape_w - 11, 12),
                    Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                    f"{int(tick_v)}",
                )
        # Current speed box
        spd_box = QRectF(spd_rect.left() + 2, cy - 10, tape_w - 4, 20)
        p.setBrush(QBrush(QColor("#0F172A")))
        p.setPen(QPen(QColor("#38BDF8"), 1.3))
        p.drawRoundedRect(spd_box, 3.0, 3.0)
        p.setPen(QColor("#F8FAFC"))
        p.drawText(spd_box, Qt.AlignmentFlag.AlignCenter, f"{v_horiz:.1f}")
        # Climb rate at bottom of speed tape
        vz_col = QColor("#4ADE80" if abs(v_up) < 1.5 else "#FACC15")
        p.setPen(vz_col)
        p.drawText(
            QRectF(spd_rect.left() + 1, spd_rect.bottom() - 15, tape_w - 2, 13),
            Qt.AlignmentFlag.AlignCenter,
            f"Vz{v_up:+.1f}",
        )

        # 5. Right Altitude Tape (Relative Altitude = -Z NED)
        alt_rect = QRectF(rect.right() - tape_w - 6.0, tape_top, tape_w, tape_h)
        p.setPen(QPen(QColor(255, 255, 255, 60), 1.0))
        p.setBrush(QBrush(QColor(15, 23, 42, 165)))
        p.drawRoundedRect(alt_rect, 5.0, 5.0)

        alt_m = -self.z_d if self.has_position else 0.0
        p.setPen(QColor("#94A3B8"))
        p.drawText(QRectF(alt_rect.left(), alt_rect.top() + 3, tape_w, 12), Qt.AlignmentFlag.AlignCenter, "ALT m")
        p.setPen(QColor("#E2E8F0"))
        for d_alt in range(-4, 5):
            tick_a = round(alt_m) + d_alt
            ty = cy - (tick_a - alt_m) * 14.0
            if alt_rect.top() + 18 <= ty <= alt_rect.bottom() - 18:
                p.drawLine(QPointF(alt_rect.left(), ty), QPointF(alt_rect.left() + 6, ty))
                p.drawText(
                    QRectF(alt_rect.left() + 8, ty - 6, tape_w - 11, 12),
                    Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                    f"{int(tick_a)}",
                )
        alt_box = QRectF(alt_rect.left() + 2, cy - 10, tape_w - 4, 20)
        p.setBrush(QBrush(QColor("#0F172A")))
        p.setPen(QPen(QColor("#4ADE80"), 1.3))
        p.drawRoundedRect(alt_box, 3.0, 3.0)
        p.setPen(QColor("#F8FAFC"))
        p.drawText(alt_box, Qt.AlignmentFlag.AlignCenter, f"{alt_m:+.1f}")

        # 6. Top Status Banner (ARMED / Flight Mode / Landed)
        top_bar = QRectF(rect.left(), rect.top(), rect.width(), 24.0)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(QColor(15, 23, 42, 220)))
        p.drawRect(top_bar)

        f_top = QFont("Segoe UI", 8, QFont.Weight.Bold)
        f_top.setPixelSize(9 if rect.width() < 225 else 10)
        p.setFont(f_top)
        fm_top = p.fontMetrics()

        arm_str = "ARMED" if self.armed else "DISARMED"
        arm_w = max(48.0, float(fm_top.horizontalAdvance(arm_str)) + 12.0)
        arm_badge = QRectF(rect.left() + 5.0, rect.top() + 3.0, arm_w, 18.0)
        arm_bg = QColor("#DC2626") if self.armed else QColor("#334155")
        p.setBrush(QBrush(arm_bg))
        p.drawRoundedRect(arm_badge, 4.0, 4.0)
        p.setPen(QColor("#FFFFFF"))
        p.drawText(arm_badge, Qt.AlignmentFlag.AlignCenter, arm_str)

        land_str = self.landed_str.upper()
        land_w = float(fm_top.horizontalAdvance(land_str)) + 6.0
        land_rect = QRectF(rect.right() - land_w - 5.0, rect.top() + 3.0, land_w, 18.0)
        p.setPen(QColor("#CBD5E1"))
        p.drawText(
            land_rect,
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight,
            land_str,
        )

        mode_left = arm_badge.right() + 4.0
        mode_right = land_rect.left() - 4.0
        mode_w = max(24.0, mode_right - mode_left)
        elided_mode = fm_top.elidedText(self.mode_str, Qt.TextElideMode.ElideRight, int(mode_w))
        p.setPen(QColor("#38BDF8"))
        p.drawText(
            QRectF(mode_left, rect.top() + 3.0, mode_w, 18.0),
            Qt.AlignmentFlag.AlignCenter,
            elided_mode,
        )

        # 7. Bottom Digital Attitude Bar
        bot_bar = QRectF(rect.left(), rect.bottom() - 20.0, rect.width(), 20.0)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(QColor(15, 23, 42, 225)))
        p.drawRect(bot_bar)
        f_bot = QFont("Consolas", 8, QFont.Weight.Bold)
        f_bot.setPixelSize(9 if rect.width() < 220 else 10)
        p.setFont(f_bot)
        p.setPen(QColor("#F8FAFC"))
        if self.has_attitude:
            att_str = f"R{self.roll_deg:+5.1f}° P{self.pitch_deg:+5.1f}° Y{self.yaw_deg:+6.1f}°"
        else:
            att_str = "WAITING FOR ODOMETRY"
        p.drawText(bot_bar, Qt.AlignmentFlag.AlignCenter, att_str)

        # Outer border
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor("#334155" if self.dark_mode else "#94A3B8"), 1.5))
        p.drawRoundedRect(rect.adjusted(1, 1, -1, -1), 10.0, 10.0)
        p.restore()

    def _paint_nav_radar(self, p: QPainter, rect: QRectF) -> None:
        p.save()
        bezel = QPainterPath()
        bezel.addRoundedRect(rect.adjusted(1, 1, -1, -1), 10.0, 10.0)
        p.setClipPath(bezel)

        bg_col = QColor("#0B132B" if self.dark_mode else "#0F172A")
        p.fillRect(rect, bg_col)

        # Top bar: Heading readout
        top_bar = QRectF(rect.left(), rect.top(), rect.width(), 24.0)
        p.fillRect(top_bar, QColor(15, 23, 42, 220))
        hdg = (self.yaw_deg % 360.0) if self.has_attitude else 0.0
        f_nav = QFont("Consolas", 8, QFont.Weight.Bold)
        f_nav.setPixelSize(9 if rect.width() < 185 else 10)
        p.setFont(f_nav)
        p.setPen(QColor("#38BDF8"))
        p.drawText(
            top_bar.adjusted(8, 0, -8, 0),
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
            f"HDG {hdg:05.1f}°" if self.has_attitude else "HDG --°",
        )
        p.setPen(QColor("#94A3B8"))
        p.drawText(
            top_bar.adjusted(8, 0, -8, 0),
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight,
            "RADAR" if rect.width() < 175 else "NED RADAR",
        )

        # Center & radius of compass rose
        avail_h = rect.height() - 44.0
        cx = rect.center().x()
        cy = rect.top() + 24.0 + avail_h * 0.5
        r_outer = max(32.0, min(rect.width() * 0.38, avail_h * 0.40))
        r_inner = r_outer * 0.68

        # Radar range rings & crosshairs
        grid_pen = QPen(QColor(56, 189, 248, 45), 1.0, Qt.PenStyle.DashLine)
        p.setPen(grid_pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(cx, cy), r_inner, r_inner)
        p.drawEllipse(QPointF(cx, cy), r_inner * 0.5, r_inner * 0.5)
        p.drawLine(QPointF(cx - r_outer, cy), QPointF(cx + r_outer, cy))
        p.drawLine(QPointF(cx, cy - r_outer), QPointF(cx, cy + r_outer))

        # Outer compass bezel ring
        p.setPen(QPen(QColor("#475569"), 1.5))
        p.drawEllipse(QPointF(cx, cy), r_outer, r_outer)

        # Compass degree ticks & N / E / S / W cardinal labels (North-up map)
        cardinals = {0: "N", 90: "E", 180: "S", 270: "W"}
        for deg in range(0, 360, 15):
            rad = math.radians(deg - 90)
            c_ang, s_ang = math.cos(rad), math.sin(rad)
            is_card = deg in cardinals
            is_30 = deg % 30 == 0
            t_in = r_outer - (7.0 if is_card else (5.0 if is_30 else 3.0))
            p.setPen(QPen(QColor("#F8FAFC" if is_30 else "#64748B"), 1.4 if is_card else 1.0))
            p.drawLine(
                QPointF(cx + c_ang * t_in, cy + s_ang * t_in),
                QPointF(cx + c_ang * r_outer, cy + s_ang * r_outer),
            )
            if is_card:
                lbl = cardinals[deg]
                p.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
                p.setPen(QColor("#F87171" if lbl == "N" else "#E2E8F0"))
                tx = cx + c_ang * (r_outer + 9.0)
                ty = cy + s_ang * (r_outer + 9.0)
                p.drawText(QRectF(tx - 8, ty - 8, 16, 16), Qt.AlignmentFlag.AlignCenter, lbl)

        # Auto-scale 2D Local NED Trajectory (North = -screen_y, East = +screen_x)
        max_extent = 3.0
        for nx, ey in self.ned_trail:
            max_extent = max(max_extent, abs(nx), abs(ey))
        if self.has_position:
            max_extent = max(max_extent, abs(self.x_n), abs(self.y_e))
        scale = (r_inner * 0.90) / max(1.0, max_extent * 1.15)

        def _ned_to_pt(n_m: float, e_m: float) -> QPointF:
            return QPointF(cx + e_m * scale, cy - n_m * scale)

        # Home marker at (0, 0)
        home_pt = _ned_to_pt(0.0, 0.0)
        p.setPen(QPen(QColor("#4ADE80"), 1.2))
        p.setBrush(QBrush(QColor(22, 163, 74, 180)))
        p.drawEllipse(home_pt, 4.0, 4.0)

        # Flight path trail
        if len(self.ned_trail) >= 2:
            path = QPainterPath()
            first = _ned_to_pt(*self.ned_trail[0])
            path.moveTo(first)
            for nx, ey in self.ned_trail[1:]:
                path.lineTo(_ned_to_pt(nx, ey))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(QColor("#38BDF8"), 1.8))
            p.drawPath(path)

        # Drone aircraft chevron at current position (or center if no pos yet)
        drone_pt = _ned_to_pt(self.x_n, self.y_e) if self.has_position else QPointF(cx, cy)
        p.save()
        p.translate(drone_pt)
        p.rotate(hdg)
        chevron = QPolygonF([
            QPointF(0.0, -9.0),
            QPointF(6.5, 7.0),
            QPointF(0.0, 3.5),
            QPointF(-6.5, 7.0),
        ])
        p.setBrush(QBrush(QColor("#FACC15" if self.armed else "#38BDF8")))
        p.setPen(QPen(QColor("#0F172A"), 1.2))
        p.drawPolygon(chevron)
        p.restore()

        # Bottom bar: NED Position & Scale
        bot_bar = QRectF(rect.left(), rect.bottom() - 20.0, rect.width(), 20.0)
        p.fillRect(bot_bar, QColor(15, 23, 42, 210))
        p.setFont(f_nav)
        p.setPen(QColor("#E2E8F0"))
        dist_home = math.hypot(self.x_n, self.y_e) if self.has_position else 0.0
        if self.has_position:
            nav_txt = (
                f"N{self.x_n:+.1f} E{self.y_e:+.1f} |H|{dist_home:.1f}m"
                if rect.width() < 190
                else f"N {self.x_n:+.1f}m  E {self.y_e:+.1f}m  |H| {dist_home:.1f}m"
            )
        else:
            nav_txt = f"Scale ±{max_extent:.0f}m · Trail {len(self.ned_trail)} pts"
        p.drawText(bot_bar, Qt.AlignmentFlag.AlignCenter, nav_txt)

        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor("#334155" if self.dark_mode else "#94A3B8"), 1.5))
        p.drawRoundedRect(rect.adjusted(1, 1, -1, -1), 10.0, 10.0)
        p.restore()


# ---------------------------------------------------------------------------
# Helpers for Panel Layout
# ---------------------------------------------------------------------------
def _panel(title: str, parent: QWidget) -> tuple[QFrame, QVBoxLayout]:
    frame = QFrame(parent)
    frame.setObjectName("panel_card")
    frame.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
    box = QVBoxLayout(frame)
    box.setContentsMargins(12, 10, 12, 10)
    box.setSpacing(6)
    head = QLabel(title)
    head.setObjectName("section_header")
    box.addWidget(head)
    return frame, box


class _WrapLabel(QLabel):
    """Word-wrapped QLabel that keeps a small vertical buffer for HTML <b> tags."""

    def heightForWidth(self, w: int) -> int:
        h = super().heightForWidth(w)
        return h + 4 if h > 0 else h

    def sizeHint(self) -> QSize:
        s = super().sizeHint()
        return QSize(min(320, s.width()), s.height() + 4)

    def minimumSizeHint(self) -> QSize:
        s = super().minimumSizeHint()
        return QSize(min(100, s.width()), s.height() + 4)


def _text_label(parent: QWidget, mono: bool = True, wrap: bool = True) -> QLabel:
    label = _WrapLabel("--", parent)
    label.setObjectName("metric_sub")
    label.setTextFormat(Qt.TextFormat.RichText)
    label.setWordWrap(wrap)
    label.setContentsMargins(0, 1, 0, 2)
    label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
    label.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
    label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    if mono:
        label.setStyleSheet("font-family: 'Cascadia Mono', 'Consolas', monospace; font-size: 11px;")
    return label


# ---------------------------------------------------------------------------
# Main UP7000 Tab Panel
# ---------------------------------------------------------------------------
class Up4000Panel(QWidget):
    """The tab. Call refresh() from the GUI timer; it throttles itself."""

    def __init__(self, listener: Up4000StatusListener, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("workspace_page")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.listener = listener
        self._last_refresh = 0.0
        self._last_seq: int | None = None
        self._last_tree = 0.0
        self._dark_mode = False

        # PX4 .ulg replay state
        self._replay_session: dict[str, Any] | None = None
        self._replay_idx: int = 0
        self._replay_playing: bool = False
        self._replay_speed: float = 1.0
        self._replay_timer = QTimer(self)
        self._replay_timer.setInterval(100)
        self._replay_timer.timeout.connect(self._on_replay_tick)

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        # 1. Source & PX4 .ulg Replay Toolbar
        toolbar_frame = QFrame(self)
        toolbar_frame.setObjectName("panel_card")
        toolbar_frame.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        tb = QHBoxLayout(toolbar_frame)
        tb.setContentsMargins(10, 6, 10, 6)
        tb.setSpacing(8)

        self.btn_live_mode = QPushButton("Live UDP :15557")
        self.btn_live_mode.setObjectName("toggle_btn")
        self.btn_live_mode.setCheckable(True)
        self.btn_live_mode.setChecked(True)
        self.btn_live_mode.clicked.connect(self.switch_to_live_mode)

        self.btn_open_ulog = QPushButton("Open PX4 .ulg Log...")
        self.btn_open_ulog.setObjectName("primary_btn")
        self.btn_open_ulog.clicked.connect(self._on_open_ulog_dialog)

        self.btn_play_pause = QPushButton("Play")
        self.btn_play_pause.setEnabled(False)
        self.btn_play_pause.clicked.connect(self.toggle_replay_play)

        self.cmb_speed = QComboBox()
        self.cmb_speed.addItems(["0.5x", "1x", "2x", "5x", "10x"])
        self.cmb_speed.setCurrentText("1x")
        self.cmb_speed.setEnabled(False)
        self.cmb_speed.currentTextChanged.connect(self._on_speed_changed)

        self.slider_replay = QSlider(Qt.Orientation.Horizontal)
        self.slider_replay.setEnabled(False)
        self.slider_replay.setRange(0, 0)
        self.slider_replay.valueChanged.connect(self._on_slider_scrubbed)

        self.btn_jump_arm = QPushButton("Arm")
        self.btn_jump_arm.setToolTip("Jump to first Armed timestamp in ULog")
        self.btn_jump_arm.setEnabled(False)
        self.btn_jump_arm.clicked.connect(self.jump_to_arm)

        self.btn_jump_alt = QPushButton("Max Alt")
        self.btn_jump_alt.setToolTip("Jump to peak altitude in ULog")
        self.btn_jump_alt.setEnabled(False)
        self.btn_jump_alt.clicked.connect(self.jump_to_max_alt)

        self.btn_jump_ekf = QPushButton("Peak EKF")
        self.btn_jump_ekf.setToolTip("Jump to highest EKF innovation test ratio in ULog")
        self.btn_jump_ekf.setEnabled(False)
        self.btn_jump_ekf.clicked.connect(self.jump_to_max_ratio)

        self.lbl_replay_info = QLabel("Source: Live UDP 15557")
        self.lbl_replay_info.setObjectName("metric_sub")

        tb.addWidget(self.btn_live_mode)
        tb.addWidget(self.btn_open_ulog)
        tb.addWidget(self.btn_play_pause)
        tb.addWidget(self.cmb_speed)
        tb.addWidget(self.slider_replay, 1)
        tb.addWidget(self.btn_jump_arm)
        tb.addWidget(self.btn_jump_alt)
        tb.addWidget(self.btn_jump_ekf)
        tb.addWidget(self.lbl_replay_info)
        root.addWidget(toolbar_frame)

        # 2. Top 5 Summary KPI Cards
        cards = QHBoxLayout()
        cards.setSpacing(8)
        self.card_link = _Card("UP7000 link")
        self.card_cpu = _Card("CPU / temperature")
        self.card_mem = _Card("Memory / disk")
        self.card_wifi = _Card("WiFi")
        self.card_vehicle = _Card("PX4")
        for c in (self.card_link, self.card_cpu, self.card_mem, self.card_wifi, self.card_vehicle):
            cards.addWidget(c, 1)
        root.addLayout(cards)

        # 3. Main 3-Column Splitter
        split = QSplitter(Qt.Orientation.Horizontal, self)

        # --- Column 1: Primary Flight Display (PFD + Compass/NED Radar) & Flight State ---
        ekf_frame, ekf_box = _panel("FLIGHT DISPLAY & PX4 TELEMETRY (LIVE / REPLAY)", split)
        self.hud = FlightHudWidget(ekf_frame)
        ekf_box.addWidget(self.hud, 1)

        # 6-Cell Quick Instrument Readout Grid (2 columns x 3 rows with compact height)
        grid_wrap = QGridLayout()
        grid_wrap.setSpacing(5)
        self.cell_mode = _TelemetryCell("Flight Mode", ekf_frame)
        self.cell_att = _TelemetryCell("Roll / Pitch / Yaw", ekf_frame)
        self.cell_pos = _TelemetryCell("Local NED (X, Y, Z)", ekf_frame)
        self.cell_vel = _TelemetryCell("Velocity (Vx, Vy, Vz)", ekf_frame)
        self.cell_batt = _TelemetryCell("Battery Power", ekf_frame)
        self.cell_vib = _TelemetryCell("IMU Vibration", ekf_frame)
        grid_wrap.addWidget(self.cell_mode, 0, 0)
        grid_wrap.addWidget(self.cell_att, 0, 1)
        grid_wrap.addWidget(self.cell_pos, 1, 0)
        grid_wrap.addWidget(self.cell_vel, 1, 1)
        grid_wrap.addWidget(self.cell_batt, 2, 0)
        grid_wrap.addWidget(self.cell_vib, 2, 1)
        ekf_box.addLayout(grid_wrap)

        # Detailed PX4 state text (kept in memory for test compatibility; hidden on UI
        # so dynamic STALE/ODOMETRY lines never resize the HUD aspect ratio)
        state_card = QFrame(ekf_frame)
        state_card.setObjectName("sub_card")
        state_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        state_card.setVisible(False)
        state_lay = QVBoxLayout(state_card)
        state_lay.setContentsMargins(8, 4, 8, 4)
        state_lay.setSpacing(2)
        self.lbl_state = _text_label(state_card)
        state_lay.addWidget(self.lbl_state)
        ekf_box.addWidget(state_card)

        # --- Column 2: EKF2 Estimator Innovation Gauges, Companion Logger & PX4 Messages ---
        log_frame, log_box = _panel("EKF2 ESTIMATOR HEALTH & UP7000 LOGGER", split)
        log_scroll = QScrollArea(log_frame)
        log_scroll.setFrameShape(QFrame.Shape.NoFrame)
        log_scroll.setWidgetResizable(True)
        log_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        log_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        log_scroll.setStyleSheet("QScrollArea { background: transparent; border: none; }")
        log_inner = QWidget(log_scroll)
        log_inner.setStyleSheet("background: transparent;")
        log_inner_box = QVBoxLayout(log_inner)
        log_inner_box.setContentsMargins(0, 0, 0, 0)
        log_inner_box.setSpacing(6)

        # 6 Visual Innovation Test Ratio Gauge Bars
        ratios_card = QFrame(log_inner)
        ratios_card.setObjectName("sub_card")
        ratios_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        ratios_lay = QVBoxLayout(ratios_card)
        ratios_lay.setContentsMargins(10, 6, 10, 6)
        ratios_lay.setSpacing(3)
        ratios_hdr = QLabel("EKF2 INNOVATION RATIOS (<0.5 OK · <1.0 WARN · ≥1.0 REJ)")
        ratios_hdr.setObjectName("card_title")
        ratios_hdr.setWordWrap(True)
        ratios_lay.addWidget(ratios_hdr)

        self.ratio_bars: dict[str, _RatioBar] = {}
        ratio_grid = QGridLayout()
        ratio_grid.setSpacing(4)
        for idx, (label_txt, field_key) in enumerate((
            ("Vel", "vel_ratio"),
            ("Pos H", "pos_horiz_ratio"),
            ("Pos V", "pos_vert_ratio"),
            ("Mag", "mag_ratio"),
            ("HAGL", "hagl_ratio"),
            ("TAS", "tas_ratio"),
        )):
            bar = _RatioBar(label_txt, ratios_card)
            self.ratio_bars[field_key] = bar
            ratio_grid.addWidget(bar, idx // 2, idx % 2)
        ratios_lay.addLayout(ratio_grid)

        self.lbl_ekf = _text_label(ratios_card)
        self.lbl_flags = _text_label(ratios_card)
        ratios_lay.addWidget(self.lbl_ekf)
        ratios_lay.addWidget(self.lbl_flags)
        log_inner_box.addWidget(ratios_card)

        # Companion Logger & Session Files Card
        logger_card = QFrame(log_inner)
        logger_card.setObjectName("sub_card")
        logger_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        logger_lay = QVBoxLayout(logger_card)
        logger_lay.setContentsMargins(10, 6, 10, 6)
        logger_lay.setSpacing(4)
        self.lbl_logger = _text_label(logger_card)
        self.lbl_files = _text_label(logger_card)
        logger_lay.addWidget(self.lbl_logger)
        logger_lay.addWidget(self.lbl_files)
        log_inner_box.addWidget(logger_card)

        # PX4 STATUSTEXT Console Card
        msg_card = QFrame(log_inner)
        msg_card.setObjectName("sub_card")
        msg_card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        msg_lay = QVBoxLayout(msg_card)
        msg_lay.setContentsMargins(10, 6, 10, 6)
        self.lbl_text = _text_label(msg_card, mono=False, wrap=True)
        msg_lay.addWidget(self.lbl_text)
        log_inner_box.addWidget(msg_card)
        log_inner_box.addStretch(1)
        log_scroll.setWidget(log_inner)
        log_box.addWidget(log_scroll, 1)

        # --- Column 3: Live All-Values Tree Inspector with Filter ---
        tree_frame, tree_box = _panel("ALL TELEMETRY INSPECTOR", split)
        filter_row = QHBoxLayout()
        filter_row.setSpacing(6)
        self.tree_filter = QLineEdit(tree_frame)
        self.tree_filter.setPlaceholderText("Filter telemetry fields (e.g. ODOMETRY, ratio)...")
        self.tree_filter.setClearButtonEnabled(True)
        self.tree_filter.setMinimumWidth(80)
        self.tree_filter.textChanged.connect(self._apply_tree_filter)

        btn_expand = QPushButton("Expand")
        btn_expand.setFixedWidth(64)
        btn_expand.clicked.connect(lambda: self.tree.expandAll())
        btn_collapse = QPushButton("Collapse")
        btn_collapse.setFixedWidth(68)
        btn_collapse.clicked.connect(lambda: self.tree.collapseAll())
        filter_row.addWidget(self.tree_filter, 1)
        filter_row.addWidget(btn_expand)
        filter_row.addWidget(btn_collapse)
        tree_box.addLayout(filter_row)

        self.tree = QTreeWidget(tree_frame)
        self.tree.setHeaderLabels(["Field", "Value"])
        self.tree.setColumnWidth(0, 195)
        self.tree.setUniformRowHeights(True)
        self.tree.setAlternatingRowColors(True)
        tree_box.addWidget(self.tree, 1)

        ekf_frame.setMinimumWidth(400)
        log_frame.setMinimumWidth(330)
        tree_frame.setMinimumWidth(240)
        split.addWidget(ekf_frame)
        split.addWidget(log_frame)
        split.addWidget(tree_frame)
        split.setStretchFactor(0, 5)
        split.setStretchFactor(1, 4)
        split.setStretchFactor(2, 3)
        split.setSizes([540, 430, 340])
        root.addWidget(split, 1)
        self.refresh(force=True)

    # ---------------------------------------------------------------- Theme
    def set_dark_mode(self, dark: bool) -> None:
        self._dark_mode = bool(dark)
        self.hud.set_dark_mode(self._dark_mode)
        for bar in self.ratio_bars.values():
            bar.set_dark_mode(self._dark_mode)

    # ---------------------------------------------------------------- PX4 .ulg Replay
    def _on_open_ulog_dialog(self) -> None:
        default_dir = str(Path.home() / "Downloads")
        if not os.path.isdir(default_dir):
            default_dir = str(Path.cwd())
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Open PX4 ULog File (.ulg)",
            default_dir,
            "PX4 ULog Files (*.ulg);;All Files (*.*)",
        )
        if path:
            self.load_ulog_file(path)

    def load_ulog_file(self, path: str | Path, step_s: float = 0.20) -> dict[str, Any]:
        """Load a PX4 `.ulg` flight log and switch panel into ULog replay mode."""
        self._replay_timer.stop()
        self._replay_playing = False
        self.btn_play_pause.setText("Play")

        session = load_ulog_session(path, step_s=step_s)
        self._replay_session = session
        self._replay_idx = 0

        n_frames = len(session["frames"])
        self.btn_live_mode.setChecked(False)
        self.btn_play_pause.setEnabled(n_frames > 1)
        self.cmb_speed.setEnabled(n_frames > 1)
        self.slider_replay.blockSignals(True)
        self.slider_replay.setEnabled(n_frames > 1)
        self.slider_replay.setRange(0, max(0, n_frames - 1))
        self.slider_replay.setValue(0)
        self.slider_replay.blockSignals(False)

        self.btn_jump_arm.setEnabled(n_frames > 1)
        self.btn_jump_alt.setEnabled(n_frames > 1)
        self.btn_jump_ekf.setEnabled(n_frames > 1)

        self.hud.set_full_trail(session["ned_trail"])
        self._render_replay_frame(0)
        return session

    def switch_to_live_mode(self) -> None:
        """Exit `.ulg` replay mode and return to live UDP 15557 telemetry."""
        self._replay_timer.stop()
        self._replay_playing = False
        self._replay_session = None
        self.btn_live_mode.setChecked(True)
        self.btn_play_pause.setText("Play")
        self.btn_play_pause.setEnabled(False)
        self.cmb_speed.setEnabled(False)
        self.slider_replay.setEnabled(False)
        self.btn_jump_arm.setEnabled(False)
        self.btn_jump_alt.setEnabled(False)
        self.btn_jump_ekf.setEnabled(False)
        self.lbl_replay_info.setText(f"Source: Live UDP {self.listener.port}")
        self.hud.clear_trail()
        self.refresh(force=True)

    def toggle_replay_play(self) -> None:
        if not self._replay_session:
            return
        self._replay_playing = not self._replay_playing
        self.btn_play_pause.setText("Pause" if self._replay_playing else "Play")
        if self._replay_playing:
            if self._replay_idx >= len(self._replay_session["frames"]) - 1:
                self._replay_idx = 0
            self._replay_timer.start()
        else:
            self._replay_timer.stop()

    def _on_speed_changed(self, text: str) -> None:
        try:
            self._replay_speed = float(text.rstrip("x"))
        except ValueError:
            self._replay_speed = 1.0

    def _on_slider_scrubbed(self, idx: int) -> None:
        if not self._replay_session:
            return
        self._render_replay_frame(idx)

    def jump_to_arm(self) -> None:
        if self._replay_session:
            self.set_replay_index(self._replay_session["arm_frame_idx"])

    def jump_to_max_alt(self) -> None:
        if self._replay_session:
            self.set_replay_index(self._replay_session["max_alt_frame_idx"])

    def jump_to_max_ratio(self) -> None:
        if self._replay_session:
            self.set_replay_index(self._replay_session["max_ratio_frame_idx"])

    def set_replay_index(self, idx: int) -> None:
        if not self._replay_session:
            return
        clamped = max(0, min(len(self._replay_session["frames"]) - 1, int(idx)))
        self.slider_replay.blockSignals(True)
        self.slider_replay.setValue(clamped)
        self.slider_replay.blockSignals(False)
        self._render_replay_frame(clamped)

    def _on_replay_tick(self) -> None:
        if not self._replay_session or not self._replay_playing:
            self._replay_timer.stop()
            return
        step_s = self._replay_session.get("step_s", 0.20)
        advance = max(1, int(round((0.10 * self._replay_speed) / max(0.05, step_s))))
        nxt = self._replay_idx + advance
        if nxt >= len(self._replay_session["frames"]):
            nxt = len(self._replay_session["frames"]) - 1
            self._replay_playing = False
            self._replay_timer.stop()
            self.btn_play_pause.setText("Play")
        self.set_replay_index(nxt)

    def _render_replay_frame(self, idx: int) -> None:
        if not self._replay_session:
            return
        frames = self._replay_session["frames"]
        idx = max(0, min(len(frames) - 1, idx))
        self._replay_idx = idx
        snap = frames[idx]
        rel_s = snap.get("ulog_rel_s", 0.0)
        dur_s = snap.get("ulog_duration_s", 0.0)
        name = self._replay_session.get("name", "ulog")

        self.card_link.set(
            "ULOG REPLAY",
            f"{name} · t={rel_s:.1f}s / {dur_s:.1f}s · frame {idx + 1}/{len(frames)}",
        )
        self.lbl_replay_info.setText(
            f"{name} · {rel_s:.1f}s / {dur_s:.1f}s · MaxAlt {self._replay_session['max_alt_m']:.1f}m"
        )
        self._show_host(snap.get("host") or {})
        self._show_px4(snap.get("px4") or {}, is_replay=True)
        self._show_logger(snap)
        self._sync(self.tree.invisibleRootItem(), snap, is_replay=True)
        if self.tree_filter.text().strip():
            self._apply_tree_filter(self.tree_filter.text())

    # ---------------------------------------------------------------- Live View Refresh
    def refresh(self, force: bool = False) -> None:
        # If user is actively replaying a .ulg file and no new live UDP packet just arrived,
        # keep displaying the ULog replay state unless forced while in live mode.
        if self._replay_session is not None:
            return

        now = time.monotonic()
        if not force and now - self._last_refresh < 0.25:
            return
        self._last_refresh = now
        lst = self.listener
        msg = lst.latest
        if not lst.bound:
            self.card_link.set("NO SOCKET", f"UDP {lst.port}: {lst.error}")
            return
        if msg is None:
            self.card_link.set(
                "WAITING",
                f"Listening UDP {lst.port}. Start the logger on the UP7000; "
                "it broadcasts once per second until it knows this PC.",
            )
            return
        age = now - (lst.latest_rx or now)
        state = "ONLINE" if age < STALE_S else f"STALE {age:.0f}s"
        self.card_link.set(state, f"{lst.sender} · {lst.rate_hz():.1f} pkt/s · seq {msg.get('seq')}")
        # Freshness must advance even if only other sections arrive, or no
        # new datagrams arrive at all.
        self._last_seq = lst.packets
        self._show_host(msg.get("host") or {})
        self._show_px4(msg.get("px4") or {}, is_replay=False)
        self._show_logger(msg)
        stale_sections = [
            name
            for name in ("host", "session", "uwb_relay", "px4_logger")
            if lst.section_age_s(name) >= STALE_S
        ]
        if stale_sections:
            self.card_link.sub.setText(self.card_link.sub.text() + " · STALE: " + ", ".join(stale_sections))
        if force or now - self._last_tree >= 0.5:
            self._last_tree = now
            self._sync(self.tree.invisibleRootItem(), msg, is_replay=False)
            if self.tree_filter.text().strip():
                self._apply_tree_filter(self.tree_filter.text())

    def _show_host(self, h: dict) -> None:
        load = h.get("load") or []
        self.card_cpu.set(
            f"{_fmt(h.get('cpu_percent'), 0, '%')} · {_fmt(h.get('cpu_temp_c'), 0, '°C')}",
            f"load {' '.join(f'{v:.2f}' for v in load)} · {h.get('cpu_count') or '?'} cores · "
            f"{h.get('hostname', '')}",
        )
        mem, disk = h.get("memory") or {}, h.get("disk") or {}
        self.card_mem.set(
            f"RAM {_fmt(mem.get('used_percent'), 0, '%')}",
            f"{mem.get('available_mb', '--')} MB free of {mem.get('total_mb', '--')} · "
            f"disk {disk.get('free_gb', '--')} GB free",
        )
        wifi = (h.get("wifi") or [{}])[0]
        if wifi.get("signal_dbm") is None and wifi.get("signal_level") is not None:
            signal = f"level {_fmt(wifi['signal_level'], 0)}"  # driver-relative, not dBm (rtl8821cu)
        else:
            signal = _fmt(wifi.get("signal_dbm"), 0, " dBm")
        link = [wifi.get("ssid") or wifi.get("interface", "--")]
        if wifi.get("freq_mhz"):
            link.append(f"{wifi['freq_mhz'] / 1000:.2f} GHz")
        if wifi.get("tx_mbit") is not None:
            link.append(f"tx {wifi['tx_mbit']:.0f} Mbit/s")
        link.append(f"quality {_fmt(wifi.get('link_quality'), 0)}")
        if wifi.get("power_save"):
            # Power save on the UP4000 dongle loses UDP (07/11: 15-28 % of 1.1 kB datagrams).
            link.append("<span style='color:#d98200'>power save ON</span>")
        self.card_wifi.set(signal, " · ".join(link))

    def _show_px4(self, px4: dict, is_replay: bool = False) -> None:
        all_msgs = px4.get("messages") or {}
        if is_replay:
            stale: dict[str, float] = {}
        else:
            stale = {
                name: self.listener.message_age_s(name)
                for name in all_msgs
                if self.listener.message_age_s(name) >= STALE_S
            }
        msgs = {name: msg for name, msg in all_msgs.items() if name not in stale}
        f = lambda t: (msgs.get(t) or {}).get("fields") or {}  # noqa: E731
        hb, sys_status, ext, odom = f("HEARTBEAT"), f("SYS_STATUS"), f("EXTENDED_SYS_STATE"), f("ODOMETRY")
        est, vib = f("ESTIMATOR_STATUS"), f("VIBRATION")
        armed = bool((hb.get("base_mode") or 0) & 128) if hb else None
        mode = px4_mode_name(hb.get("custom_mode", 0)) if hb else "--"
        volt = sys_status.get("voltage_battery")
        batt = f"{volt / 1000:.2f} V" if volt not in (None, 65535) else "-- V"
        remaining = sys_status.get("battery_remaining")
        batt += f" {remaining}%" if remaining not in (None, -1) else ""
        landed_txt = LANDED.get(ext.get("landed_state"), "--")

        self.card_vehicle.set(
            ("ARMED" if armed else "DISARMED") if hb else ("NO LINK" if not px4.get("connected") else "--"),
            f"{mode} · {landed_txt} · {batt}",
        )
        if "HEARTBEAT" in stale:
            self.card_vehicle.value.setText("STALE HEARTBEAT")

        arm_state = ("armed" if armed else "disarmed") if hb else "--"
        lines: list[str] = []
        euler = None
        pos_ned = None
        vel_ned = None
        if odom:
            euler = quat_to_euler_deg(odom.get("q"))
            att = "roll {:+.1f} pitch {:+.1f} yaw {:+.1f} deg".format(*euler) if euler else "--"
            age = (msgs.get("ODOMETRY") or {}).get("age_s")
            pos_ned = (odom.get("x"), odom.get("y"), odom.get("z"))
            vel_ned = (odom.get("vx"), odom.get("vy"), odom.get("vz"))
            lines += [
                f"<b>Pos</b> x {_fmt(odom.get('x'), 3)} y {_fmt(odom.get('y'), 3)} "
                f"z {_fmt(odom.get('z'), 3)} m · <b>Vel</b> {_fmt(odom.get('vx'))} "
                f"{_fmt(odom.get('vy'))} {_fmt(odom.get('vz'))} m/s",
                f"<b>Att</b> {att} · <b>ODOM</b> {_fmt(age, 2, 's')} "
                f"({(px4.get('rates_hz') or {}).get('ODOMETRY', '--')} Hz)",
            ]
        else:
            lines.append(
                f"<b>Mode</b> {mode} ({arm_state}) · "
                + ("No fresh ODOMETRY" if "ODOMETRY" in stale else "No ODOMETRY yet")
            )
        if stale:
            names = ", ".join(f"{name} ({age:.1f}s)" for name, age in sorted(stale.items()))
            lines.append(f'<span style="color:#d98200"><b>STALE PX4</b>: {names}</span>')
        self.lbl_state.setText("<br>".join(lines))

        # Update Primary Flight Display (PFD) & Compass/NED Radar
        self.hud.update_telemetry(
            connected=bool(px4.get("connected")),
            armed=armed,
            mode=mode,
            landed=landed_txt,
            battery=batt,
            euler=euler,
            pos_ned=pos_ned,
            vel_ned=vel_ned,
            append_trail=not is_replay,
        )

        # Update 6-Cell Quick Readout Grid (compact strings so 3x2 grid never truncates)
        mode_col = "#DC2626" if armed else None
        self.cell_mode.set_data(f"{mode}", f"{arm_state.upper()} · {landed_txt}", color=mode_col)
        if euler:
            self.cell_att.set_data(
                f"{euler[0]:+.1f}° {euler[1]:+.1f}° {euler[2]:+.1f}°",
                "Roll · Pitch · Yaw",
            )
        else:
            self.cell_att.set_data("-- / -- / --", "Waiting for attitude")

        if odom and pos_ned and all(isinstance(v, (int, float)) for v in pos_ned):
            alt_rel = -float(pos_ned[2])
            self.cell_pos.set_data(
                f"{pos_ned[0]:+.2f} {pos_ned[1]:+.2f} {pos_ned[2]:+.2f}m",
                f"Alt: {alt_rel:+.2f}m · {(px4.get('rates_hz') or {}).get('ODOMETRY', '--')}Hz",
            )
        else:
            self.cell_pos.set_data("--, --, -- m", "No fresh ODOMETRY")

        if odom and vel_ned and all(isinstance(v, (int, float)) for v in vel_ned):
            v_xy = math.hypot(float(vel_ned[0]), float(vel_ned[1]))
            self.cell_vel.set_data(
                f"{vel_ned[0]:+.2f} {vel_ned[1]:+.2f} {vel_ned[2]:+.2f}",
                f"Vxy: {v_xy:.2f} m/s",
            )
        else:
            self.cell_vel.set_data("--, --, -- m/s", "Groundspeed: --")

        curr_raw = sys_status.get("current_battery")
        curr_str = f"Curr: {curr_raw / 100:.1f} A" if isinstance(curr_raw, (int, float)) and curr_raw > 0 else "SYS_STATUS"
        self.cell_batt.set_data(batt, curr_str)

        if vib:
            vx_v, vy_v, vz_v = vib.get("vibration_x"), vib.get("vibration_y"), vib.get("vibration_z")
            c0, c1, c2 = vib.get("clipping_0", 0), vib.get("clipping_1", 0), vib.get("clipping_2", 0)
            vib_warn = "#DC2626" if any(isinstance(c, (int, float)) and c > 0 for c in (c0, c1, c2)) else None
            self.cell_vib.set_data(
                f"{_fmt(vx_v, 1)} / {_fmt(vy_v, 1)} / {_fmt(vz_v, 1)}",
                f"m/s² · Clip {c0}/{c1}/{c2}",
                color=vib_warn,
            )
        else:
            self.cell_vib.set_data("-- / -- / --", "Clip: --/--/--")

        # Update EKF2 Innovation Ratio Bars & Labels
        for field_key, bar in self.ratio_bars.items():
            bar.set_value(est.get(field_key) if est else None)

        ratios = " · ".join(
            _ratio_html(n, est.get(k))
            for n, k in (
                ("vel", "vel_ratio"),
                ("pos H", "pos_horiz_ratio"),
                ("pos V", "pos_vert_ratio"),
                ("mag", "mag_ratio"),
                ("HAGL", "hagl_ratio"),
                ("TAS", "tas_ratio"),
            )
        )
        acc = f"accuracy H {_fmt(est.get('pos_horiz_accuracy'))} m · V {_fmt(est.get('pos_vert_accuracy'))} m"
        vib_txt = (
            f"vibration {_fmt(vib.get('vibration_x'))} {_fmt(vib.get('vibration_y'))} "
            f"{_fmt(vib.get('vibration_z'))} m/s² · clipping {vib.get('clipping_0', '--')}/"
            f"{vib.get('clipping_1', '--')}/{vib.get('clipping_2', '--')}"
            if vib
            else "vibration --"
        )
        self.lbl_ekf.setText(f"<b>EKF test ratios</b> {ratios}<br>{acc}<br>{vib_txt}")

        if est:
            items = []
            for name, on, bad in ekf_flags(est.get("flags") or 0):
                color = ("#d93025" if bad else "#2e9e44") if on else "#8a8f98"
                items.append(f'<span style="color:{color}">{"■" if on else "□"} {name}</span>')
            self.lbl_flags.setText("<b>EKF flags</b> " + " &nbsp; ".join(items))
        else:
            self.lbl_flags.setText("<b>EKF flags</b> -- (no ESTIMATOR_STATUS)")

        texts = px4.get("statustext") or []
        shown = []
        for t in texts[-8:]:
            sev = t.get("severity")
            sev_name = SEVERITY.get(sev, str(sev))
            sev_col = "#d93025" if isinstance(sev, int) and sev <= 3 else ("#d98200" if sev == 4 else "#2563eb")
            ts_str = time.strftime("%H:%M:%S", time.localtime(t.get("time_unix", 0)))
            shown.append(f'{ts_str} <span style="color:{sev_col}"><b>{sev_name}</b></span>: {t.get("text")}')
        self.lbl_text.setText("<b>PX4 messages</b><br>" + ("<br>".join(shown) if shown else "none"))

    def _show_logger(self, msg: dict) -> None:
        uwb, px4l = msg.get("uwb_relay") or {}, msg.get("px4_logger") or {}
        uc, parsed, parser = uwb.get("counts") or {}, uwb.get("parsed") or {}, uwb.get("parser") or {}
        pc, ulog = px4l.get("counts") or {}, px4l.get("ulog") or {}
        procs = (msg.get("host") or {}).get("processes") or {}
        cpu = " · ".join(f"{n} {_fmt(p.get('cpu_percent'), 0, '%')}" for n, p in procs.items()) or "--"
        rows = [
            f"<b>UWB relay</b> · TAG {uwb.get('tag_port', '--')} · ground {uwb.get('ground') or 'not found yet'}",
            f"bytes {uc.get('tag_bytes', 0):,} · Burst parsed {parsed.get('frame_0x18', 0):,} · "
            f"CRC {parser.get('crc_errors', 0)}",
            f"UDP sent {uc.get('udp_datagrams', 0):,} · parse drops {uc.get('parse_queue_dropped_chunks', 0)} · "
            f"reopen {uc.get('tag_silent_reopens', 0)} · callback err {parsed.get('callback_errors', 0)}",
            f"<b>PX4 logger</b> · port {px4l.get('px4_port', '--')} · messages {pc.get('messages', 0):,} · "
            f"bad {pc.get('bad_data', 0)}",
            f"ULog part {px4l.get('ulog_part', 0)} · chunks {ulog.get('chunks', 0):,} · "
            f"lost {ulog.get('dropped_chunks', 0)} · {ulog.get('bytes', 0) / 1e6:.1f} MB",
            f"timesync {pc.get('timesync_samples', 0)} · QGC {px4l.get('ground') or '--'} · <b>CPU</b> {cpu}",
        ]
        self.lbl_logger.setText("<br>".join(rows))
        session = msg.get("session") or {}
        files = session.get("files") or {}
        listing = " · ".join(f"{name}: {size / 1e6:.2f} MB" for name, size in files.items())
        self.lbl_files.setText(f"<b>Session</b> {session.get('folder', '--')}<br>{listing or 'no files'}")

    # ---------------------------------------------------------------- Tree & Filter
    def _sync(self, parent: QTreeWidgetItem, data: Any, path: str = "", is_replay: bool = False) -> None:
        """Update the tree in place so expanded branches stay expanded."""
        if isinstance(data, dict):
            entries = [(str(k), v) for k, v in data.items()]
        elif isinstance(data, list) and any(isinstance(v, (dict, list)) for v in data):
            entries = [(f"[{i}]", v) for i, v in enumerate(data)]
        else:
            return
        existing = {parent.child(i).text(0): parent.child(i) for i in range(parent.childCount())}
        keep = set()
        for key, value in entries:
            child_path = f"{path}.{key}" if path else key
            item = existing.get(key)
            if item is None:
                item = QTreeWidgetItem(parent, [key, ""])
            keep.add(key)
            if isinstance(value, (dict, list)) and (
                isinstance(value, dict) or any(isinstance(v, (dict, list)) for v in value)
            ):
                text = f"{len(value)} items"
                self._sync(item, value, child_path, is_replay=is_replay)
            else:
                text = ", ".join(_tree_value(v) for v in value) if isinstance(value, list) else _tree_value(value)
            if not is_replay:
                if path == "px4.messages":
                    age = self.listener.message_age_s(key)
                    if age >= STALE_S:
                        text += f" · STALE {age:.1f}s"
                elif not path and key in ("host", "session", "uwb_relay", "px4_logger"):
                    age = self.listener.section_age_s(key)
                    if age >= STALE_S:
                        text += f" · STALE {age:.1f}s"
            if item.text(1) != text:
                item.setText(1, text)
        for key, item in existing.items():
            if key not in keep:
                parent.removeChild(item)

    def _apply_tree_filter(self, query: str) -> None:
        q = query.strip().lower()

        def _filter_item(item: QTreeWidgetItem) -> bool:
            if not q:
                item.setHidden(False)
                for i in range(item.childCount()):
                    _filter_item(item.child(i))
                return True
            self_match = q in item.text(0).lower() or q in item.text(1).lower()
            child_match = False
            for i in range(item.childCount()):
                if _filter_item(item.child(i)):
                    child_match = True
            visible = self_match or child_match
            item.setHidden(not visible)
            if child_match:
                item.setExpanded(True)
            return visible

        root = self.tree.invisibleRootItem()
        for i in range(root.childCount()):
            _filter_item(root.child(i))
