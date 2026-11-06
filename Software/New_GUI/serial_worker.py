"""Background QThread Serial & Demo Workers + Fast Non-Blocking CSV Logger."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import queue
import random
import struct
import sys
import threading
import time
from typing import Sequence

from PyQt6.QtCore import QThread, pyqtSignal

_OLD_GUI_DIR = Path(__file__).resolve().parent.parent / "UWB_UART_GUI"
if str(_OLD_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(_OLD_GUI_DIR))

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None  # type: ignore[assignment]
    list_ports = None  # type: ignore[assignment]

import telemetry_protocol as tp  # noqa: E402
from core_state import AnchorConfig, DEFAULT_PRESET_BOX8, UwbStateEngine, encode_set_ds_cal_command  # noqa: E402


BURST_PRESETS = {
    "stable50": tp.BurstSettings(1, 2500, 1000, 1200, 1500, 20000),
    "hz100": tp.BurstSettings(1, 600, 350, 1200, 300, 10000),
    "hz200": tp.BurstSettings(1, 550, 350, 450, 250, 4500),
    "sequential": tp.BurstSettings(0, 2500, 1000, 1200, 1500, 20000),
}


def available_serial_ports() -> list[tuple[str, str]]:
    """Return [(device, label)] with Bluetooth virtual serial ports filtered to the bottom."""
    if list_ports is None:
        return []
    ports = list(list_ports.comports())
    result: list[tuple[str, str]] = []
    for p in sorted(ports, key=lambda item: ("bluetooth" in (item.description or "").lower(), item.device)):
        desc = p.description or "Serial Port"
        result.append((p.device, f"{p.device} — {desc}"))
    return result


class FastCsvLogger:
    """Comprehensive non-blocking CSV & JSON session logger for Burst, 8-Anchor & 3D Trajectory data."""

    def __init__(self, base_dir: Path, source_name: str, engine: UwbStateEngine | None = None) -> None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        clean_src = "".join(c if c.isalnum() else "_" for c in source_name)
        self.session_dir = base_dir / f"uwb_fast_{stamp}_{clean_src}"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.session_dir / "uwb_fast_log.csv"
        self.meta_path = self.session_dir / "session_metadata.json"
        self._engine = engine
        self._started_s = time.time()
        self._queue: queue.Queue[tuple | None] = queue.Queue(maxsize=32768)
        self.rows_written = 0
        self.dropped_rows = 0
        self._stop = threading.Event()
        self._write_metadata_snapshot()
        self._thread = threading.Thread(target=self._writer_loop, name="uwb-fast-csv", daemon=True)
        self._thread.start()

    def _write_metadata_snapshot(self) -> None:
        if self._engine is None:
            return
        try:
            eng = self._engine
            di = eng.device_info
            payload = {
                "schema": 3,
                "started_unix_s": self._started_s,
                "updated_unix_s": time.time(),
                "rows_written": self.rows_written,
                "dropped_rows": self.dropped_rows,
                "solver": {
                    "dimension": eng.solver_dimension,
                    "fixed_z_m": eng.fixed_z_m,
                    "allow_uncalibrated_positioning": eng.allow_uncalibrated_positioning,
                    "use_host_filter_for_pos": eng.use_host_filter_for_pos,
                    "use_nlos_downweight": eng.use_nlos_downweight,
                },
                "firmware": (
                    {
                        "git_hash": f"0x{di.git_hash:08X}",
                        "frame_version": di.frame_version,
                        "role": di.role,
                        "node_address": di.node_address,
                        "uart_baud": di.uart_baud,
                    }
                    if di is not None
                    else None
                ),
                "anchors": [
                    {
                        "anchor_id": a.anchor_id,
                        "x": round(a.x, 4),
                        "y": round(a.y, 4),
                        "z": round(a.z, 4),
                        "enabled": a.enabled,
                        "bias_mm": round(a.bias_mm, 2),
                        "sigma_mm": round(a.sigma_mm, 2),
                        "fw_calibrated": a.fw_calibrated,
                        "fw_bias_mm": round(a.fw_bias_mm, 2),
                    }
                    for a in sorted(eng.anchors.values(), key=lambda item: item.anchor_id)
                ],
                "waypoints": [
                    {
                        "index": wp[0],
                        "label": wp[1],
                        "x": round(wp[2], 4),
                        "y": round(wp[3], 4),
                        "z": round(wp[4], 4),
                        "time_s": round(wp[5], 3),
                    }
                    for wp in eng.waypoints
                ],
            }
            self.meta_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        except Exception:
            pass

    def log_burst(
        self,
        msg: tp.RangeBurstMessage,
        pos_xyz: tuple[float, float, float] | None,
        rms_m: float | None,
        engine: UwbStateEngine | None = None,
    ) -> None:
        eng = engine or self._engine
        sol = eng.latest_solution if eng is not None else None
        if pos_xyz is None and sol is not None:
            pos_xyz = (sol.x, sol.y, sol.z)
        if rms_m is None and sol is not None:
            rms_m = sol.rms_m

        by_id = {a.anchor_id: a for a in msg.anchors}
        row: list[object] = [
            f"{time.time():.6f}",
            msg.boot_id,
            msg.cycle_seq,
            msg.t_us,
            msg.period_us,
            "" if pos_xyz is None else f"{pos_xyz[0]:.4f}",
            "" if pos_xyz is None else f"{pos_xyz[1]:.4f}",
            "" if pos_xyz is None else f"{pos_xyz[2]:.4f}",
            "" if rms_m is None else f"{rms_m:.4f}",
        ]
        for aid in range(1, 9):
            a = by_id.get(aid)
            if a is None or not a.radio_ok or a.range_mm is None:
                row.extend(("", "", "", ""))
            else:
                fp = "" if a.fp_cdbm == tp.INT16_MIN else f"{a.fp_cdbm / 100.0:.1f}"
                nlos = (
                    ""
                    if (a.fp_cdbm == tp.INT16_MIN or a.rx_cdbm == tp.INT16_MIN)
                    else f"{(a.rx_cdbm - a.fp_cdbm) / 100.0:.1f}"
                )
                ci = "" if a.ci_ppm_x100 == tp.INT16_MIN else f"{a.ci_ppm_x100 / 100.0:.2f}"
                row.extend((a.range_mm, fp, nlos, ci))

        # Full 3D motion, DOP, stability & waypoint columns
        wp_label = ""
        if eng is not None and eng.pending_waypoint_label:
            wp_label = eng.pending_waypoint_label
            eng.pending_waypoint_label = ""

        if sol is not None:
            used_str = "|".join(f"A{a}" for a in sol.used_anchor_ids)
            excl_str = "|".join(f"A{a}" for a in sol.excluded_anchor_ids)
            row.extend(
                (
                    f"{sol.raw_x:.4f}",
                    f"{sol.raw_y:.4f}",
                    f"{sol.raw_z:.4f}",
                    f"{sol.vx_mps:.3f}",
                    f"{sol.vy_mps:.3f}",
                    f"{sol.vz_mps:.3f}",
                    f"{sol.speed_mps:.3f}",
                    f"{sol.step_jump_mm:.1f}",
                    f"{sol.max_residual_m * 1000.0:.1f}",
                    f"{sol.dop.pdop:.2f}",
                    f"{sol.dop.hdop:.2f}",
                    f"{sol.dop.vdop:.2f}",
                    f"{sol.dop.gdop:.2f}",
                    len(sol.used_anchor_ids),
                    used_str,
                    excl_str,
                    sol.motion_state,
                    sol.stability_state,
                    sol.instability_reason,
                    wp_label,
                )
            )
        else:
            row.extend(("", "", "", "", "", "", "", "", "", "", "", "", "", 0, "", "", "", "", "", wp_label))

        # Per-anchor processed telemetry & active rig coordinates/biases
        for aid in range(1, 9):
            if eng is not None:
                st = eng.live[aid]
                cfg = eng.anchors[aid]
                corr_txt = f"{st.corrected_mm:.1f}" if st.raw_mm > 0 else ""
                filt_txt = f"{st.host_filtered_mm:.1f}" if st.host_filtered_mm is not None and st.raw_mm > 0 else ""
                res_txt = f"{st.residual_mm:+.1f}" if st.residual_mm is not None else ""
                hz_txt = f"{st.rate_hz:.1f}"
                sig_txt = f"{st.noise_sigma_mm:.1f}" if st.noise_sigma_mm is not None else f"{cfg.sigma_mm:.1f}"
                rx_txt = f"{st.rx_dbm:.1f}" if st.rx_dbm is not None else ""
                st_txt = "OK" if st.radio_ok and st.raw_mm > 0 else "OFF"
                row.extend(
                    (
                        corr_txt,
                        filt_txt,
                        res_txt,
                        hz_txt,
                        sig_txt,
                        rx_txt,
                        st_txt,
                        f"{cfg.x:.3f}",
                        f"{cfg.y:.3f}",
                        f"{cfg.z:.3f}",
                        f"{cfg.bias_mm:.1f}",
                    )
                )
            else:
                row.extend(("", "", "", "", "", "", "", "", "", "", ""))

        try:
            self._queue.put_nowait(tuple(row))
        except queue.Full:
            self.dropped_rows += 1

    def _writer_loop(self) -> None:
        headers = [
            "host_time_s", "boot_id", "cycle_seq", "tag_time_us", "period_us",
            "pos_x_m", "pos_y_m", "pos_z_m", "pos_rms_m",
        ]
        for aid in range(1, 9):
            headers.extend((f"a{aid}_mm", f"a{aid}_fp_dbm", f"a{aid}_nlos_db", f"a{aid}_ci_ppm"))

        headers.extend(
            (
                "raw_x_m", "raw_y_m", "raw_z_m",
                "vel_x_mps", "vel_y_mps", "vel_z_mps", "speed_mps",
                "step_jump_mm", "max_res_mm",
                "pdop", "hdop", "vdop", "gdop",
                "used_count", "used_anchors", "excluded_anchors",
                "motion_state", "stability_state", "instability_reason", "waypoint_label",
            )
        )
        for aid in range(1, 9):
            headers.extend(
                (
                    f"a{aid}_corr_mm",
                    f"a{aid}_filt_mm",
                    f"a{aid}_res_mm",
                    f"a{aid}_hz",
                    f"a{aid}_sigma_mm",
                    f"a{aid}_rx_dbm",
                    f"a{aid}_status",
                    f"a{aid}_cfg_x",
                    f"a{aid}_cfg_y",
                    f"a{aid}_cfg_z",
                    f"a{aid}_bias_mm",
                )
            )

        with self.csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(headers)
            last_flush = time.monotonic()
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    item = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                if item is None:
                    break
                writer.writerow(item)
                self.rows_written += 1
                if time.monotonic() - last_flush >= 1.0:
                    handle.flush()
                    last_flush = time.monotonic()
            handle.flush()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        self._thread.join(timeout=2.0)
        self._write_metadata_snapshot()


class SerialTelemetryWorker(QThread):
    """High-speed UART telemetry reader thread.

    Batches decoded messages every ~25 ms so the Qt GUI main thread processes
    at most 40 signal emissions/s even when the TAG streams at 220-300 Hz.
    """

    batch_ready = pyqtSignal(list)          # list[tuple[DecodedMessage, float]]
    link_status = pyqtSignal(str, str)      # (state: "connected"|"online"|"warn"|"disconnected", text)
    uart_metrics = pyqtSignal(int, int, int, float)  # (valid_frames, crc_errors, discarded_bytes, bytes_per_s)
    log_event = pyqtSignal(str)

    def __init__(self, port: str, baud: int = 460800, parent=None) -> None:
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._stop_event = threading.Event()
        self._cmd_queue: queue.Queue[tuple[int, bytes]] = queue.Queue()
        self._seq = int(time.time_ns()) & 0xFFFFFFFF
        self._serial = None

    def send_command(self, cmd_id: int, payload: bytes = b"") -> None:
        self._cmd_queue.put((cmd_id, bytes(payload)))

    def send_set_ds_cal(self, anchor_id: int, bias_mm: float, calibrated: bool = True) -> None:
        bias_um = int(round(float(bias_mm) * 1000.0))
        self.send_command(tp.CMD_SET_DS_CAL, struct.pack("<HiB", int(anchor_id), bias_um, 1 if calibrated else 0))

    def send_save_settings_sequence(self) -> None:
        """Queue PAUSE -> SAVE_SETTINGS -> RESUME."""
        self.send_command(tp.CMD_PAUSE)
        self.send_command(tp.CMD_SAVE_SETTINGS)
        self.send_command(tp.CMD_RESUME)

    def stop(self) -> None:
        self._stop_event.set()
        if self._serial is not None:
            try:
                self._serial.cancel_read()
            except Exception:
                pass
        self.wait(1500)

    def _write_cmd_now(self, ser, cmd_id: int, payload: bytes = b"") -> None:
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        frame = tp.encode_command(self._seq, cmd_id, payload)
        ser.write(frame)

    def run(self) -> None:
        if serial is None:
            self.link_status.emit("disconnected", "PySerial is not installed")
            return
        parser = tp.TelemetryStreamParser()
        batch: list[tuple[object, float]] = []
        last_flush = time.monotonic()
        last_ping = 0.0
        last_stat = time.monotonic()
        bytes_in_sec = 0
        online_seen = False
        device_info_seen = False

        try:
            # Force J-Link VCOM line coding transition cleanly
            ser = serial.Serial(self.port, 115200, timeout=0.02)
            ser.baudrate = self.baud
            ser.reset_input_buffer()
            self._serial = ser
            self.link_status.emit("connected", f"Opened {self.port} @ {self.baud}")
            self.log_event.emit(f"UART opened: {self.port} @ {self.baud} baud")

            self._write_cmd_now(ser, tp.CMD_GET_DEVICE_INFO)

            while not self._stop_event.is_set():
                now = time.monotonic()
                if now - last_ping >= 1.0:
                    self._write_cmd_now(ser, tp.CMD_PING)
                    if not device_info_seen:
                        self._write_cmd_now(ser, tp.CMD_GET_DEVICE_INFO)
                    last_ping = now

                while not self._cmd_queue.empty():
                    try:
                        cmd_id, payload = self._cmd_queue.get_nowait()
                    except queue.Empty:
                        break
                    self._write_cmd_now(ser, cmd_id, payload)
                    if cmd_id == tp.CMD_PAUSE:
                        time.sleep(0.06)

                chunk = ser.read(min(ser.in_waiting or 1, 16384))
                if chunk:
                    bytes_in_sec += len(chunk)
                    rx_time = time.monotonic()
                    for frame in parser.feed(chunk):
                        try:
                            msg = tp.decode_frame(frame)
                        except tp.ProtocolError as exc:
                            parser.counters.decode_errors += 1
                            self.log_event.emit(f"Protocol error: {exc}")
                            continue

                        if not online_seen:
                            online_seen = True
                            self.link_status.emit("online", f"TAG Online · {self.port}")
                        if isinstance(msg, tp.DeviceInfoMessage) and not device_info_seen:
                            device_info_seen = True
                            self._write_cmd_now(ser, tp.CMD_GET_DS_CAL)
                            wanted = tp.TELEM_FEATURE_RANGE_SNAPSHOT | tp.TELEM_FEATURE_DIAG
                            if msg.uart_baud >= tp.TELEM_RANGE_MEAS_MIN_BAUD:
                                wanted |= tp.TELEM_FEATURE_RANGE_MEAS
                            if (msg.telemetry_features & wanted) != wanted:
                                self._write_cmd_now(
                                    ser,
                                    tp.CMD_SET_TELEMETRY,
                                    bytes((msg.telemetry_features | wanted,)),
                                )
                        batch.append((msg, rx_time))

                now = time.monotonic()
                if batch and (now - last_flush >= 0.025 or len(batch) >= 64):
                    self.batch_ready.emit(batch)
                    batch = []
                    last_flush = now

                if now - last_stat >= 1.0:
                    dt = now - last_stat
                    rate_bps = bytes_in_sec / dt
                    c = parser.counters
                    self.uart_metrics.emit(c.valid_frames, c.crc_errors, c.discarded_bytes, rate_bps)
                    bytes_in_sec = 0
                    last_stat = now

        except Exception as exc:
            self.log_event.emit(f"Serial error: {exc}")
            self.link_status.emit("warn", f"Error: {exc}")
        finally:
            if batch:
                self.batch_ready.emit(batch)
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
            self.link_status.emit("disconnected", "Disconnected")


class DemoBurstWorker(QThread):
    """High-rate 8-anchor 3D Burst DS-TWR simulator for testing 2D/3D & Calibration without hardware."""

    batch_ready = pyqtSignal(list)
    link_status = pyqtSignal(str, str)
    uart_metrics = pyqtSignal(int, int, int, float)
    log_event = pyqtSignal(str)

    def __init__(
        self,
        rate_hz: int = 100,
        anchors: Sequence[AnchorConfig] = DEFAULT_PRESET_BOX8,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.rate_hz = max(10, min(int(rate_hz), 300))
        self.anchors = [AnchorConfig(a.anchor_id, a.x, a.y, a.z, a.enabled) for a in anchors]
        # Simulated hardware biases (mm) before TAG calibration
        self.sim_hw_biases_mm: dict[int, float] = {
            1: 35.0, 2: -28.0, 3: 52.0, 4: -44.0,
            5: 22.0, 6: -31.0, 7: 19.0, 8: -25.0,
        }
        self.tag_cal_biases_um: dict[int, int] = {i: 0 for i in range(1, 9)}
        self.tag_cal_mask: int = 0x00
        self.active_mask: int = 0xFF
        self.motion_profile: str = "helix_3d"  # "stationary" | "circle_2d" | "helix_3d"
        self.stationary_xyz: tuple[float, float, float] = (2.5, 2.0, 1.2)
        self._stop_event = threading.Event()
        self._cmd_queue: queue.Queue[tuple[int, bytes]] = queue.Queue()
        self._burst_settings = BURST_PRESETS["hz100"]

    def set_anchors(self, anchors: Sequence[AnchorConfig]) -> None:
        self.anchors = [AnchorConfig(a.anchor_id, a.x, a.y, a.z, a.enabled) for a in anchors]

    def send_command(self, cmd_id: int, payload: bytes = b"") -> None:
        self._cmd_queue.put((cmd_id, bytes(payload)))

    def send_set_ds_cal(self, anchor_id: int, bias_mm: float, calibrated: bool = True) -> None:
        bias_um = int(round(float(bias_mm) * 1000.0))
        self.send_command(tp.CMD_SET_DS_CAL, struct.pack("<HiB", int(anchor_id), bias_um, 1 if calibrated else 0))

    def send_save_settings_sequence(self) -> None:
        self.send_command(tp.CMD_PAUSE)
        self.send_command(tp.CMD_SAVE_SETTINGS)
        self.send_command(tp.CMD_RESUME)

    def stop(self) -> None:
        self._stop_event.set()
        self.wait(1500)

    def _true_tag_xyz(self, elapsed_s: float) -> tuple[float, float, float]:
        if self.motion_profile == "stationary":
            return self.stationary_xyz
        xs = [a.x for a in self.anchors if a.enabled] or [0.0, 5.0]
        ys = [a.y for a in self.anchors if a.enabled] or [0.0, 4.0]
        zs = [a.z for a in self.anchors if a.enabled] or [0.0, 3.0]
        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0
        cz = (min(zs) + max(zs)) / 2.0
        rx = max((max(xs) - min(xs)) * 0.28, 0.12)
        ry = max((max(ys) - min(ys)) * 0.28, 0.12)
        rz = max((max(zs) - min(zs)) * 0.22, 0.10)
        omega = 0.65
        x = cx + rx * math.cos(omega * elapsed_s)
        y = cy + ry * math.sin(omega * elapsed_s)
        if self.motion_profile == "circle_2d":
            return (x, y, self.stationary_xyz[2])
        z = cz + rz * math.sin(0.35 * omega * elapsed_s)
        return (x, y, z)

    def _handle_commands(self, seq_base: int, time_ms: int) -> list[tuple[object, float]]:
        out: list[tuple[object, float]] = []
        now = time.monotonic()
        while not self._cmd_queue.empty():
            try:
                cmd_id, arg = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            seq_base += 1
            if cmd_id == tp.CMD_SET_DS_CAL and len(arg) == 7:
                aid, bias_um, cal_flag = struct.unpack("<HiB", arg)
                if 1 <= aid <= 8:
                    self.tag_cal_biases_um[aid] = bias_um
                    if cal_flag:
                        self.tag_cal_mask |= 1 << (aid - 1)
                    else:
                        self.tag_cal_mask &= ~(1 << (aid - 1))
                out.append((tp.CmdAckMessage(seq_base, time_ms, cmd_id, 0, b""), now))
                out.append((self._make_info_msg(seq_base, time_ms), now))
            elif cmd_id == tp.CMD_GET_DS_CAL:
                buf = bytearray()
                for aid in range(1, 9):
                    cal = 1 if (self.tag_cal_mask & (1 << (aid - 1))) else 0
                    buf.extend(struct.pack("<HiB", aid, self.tag_cal_biases_um[aid], cal))
                out.append((tp.CmdAckMessage(seq_base, time_ms, cmd_id, 0, bytes(buf)), now))
            elif cmd_id == tp.CMD_SET_BURST:
                if len(arg) == 11:
                    self._burst_settings = tp.BurstSettings.decode(arg)
                    if self._burst_settings.period_us > 0:
                        self.rate_hz = max(10, min(int(1e6 / self._burst_settings.period_us), 250))
                out.append((tp.CmdAckMessage(seq_base, time_ms, cmd_id, 0, self._burst_settings.encode()), now))
            elif cmd_id == tp.CMD_SET_ANCHOR_MASK and len(arg) == 1:
                self.active_mask = arg[0]
                out.append((tp.CmdAckMessage(seq_base, time_ms, cmd_id, 0, b""), now))
            elif cmd_id == tp.CMD_SET_TELEMETRY and len(arg) == 1:
                out.append((tp.CmdAckMessage(seq_base, time_ms, cmd_id, 0, arg), now))
            else:
                out.append((tp.CmdAckMessage(seq_base, time_ms, cmd_id, 0, b"\x01"), now))
        return out

    def _make_info_msg(self, seq: int, time_ms: int) -> tp.InfoMessage:
        return tp.InfoMessage(
            sequence=seq,
            time_ms=time_ms,
            schema=2,
            flags=tp.INFO_FLAG_HW_ANTENNA_DELAY | tp.INFO_FLAG_DS_BUILD | tp.INFO_FLAG_FPP_CORRECTED | tp.INFO_FLAG_BURST,
            ranging_mode=1,
            anchor_count=8,
            calibrated_mask=self.tag_cal_mask,
            filter_mode=1,
            phy_profile_id=3,
            spi_clock_mhz=16,
            motion_mode=0,
            global_motion_state=0,
            active_offsets_um=tuple((aid, self.tag_cal_biases_um[aid]) for aid in range(1, 9)),
        )

    def run(self) -> None:
        rng = random.Random(2026)
        self.link_status.emit("online", "DEMO Simulator · Burst DS-TWR")
        self.log_event.emit("Started 8-Anchor 3D Burst DS-TWR Simulator")
        t0 = time.monotonic()
        cycle_seq = 0
        meas_seq = 1
        boot_id = 0xDE05
        batch: list[tuple[object, float]] = []
        last_flush = t0
        last_info = 0.0

        batch.append((self._make_info_msg(1, 0), t0))

        while not self._stop_event.is_set():
            now = time.monotonic()
            elapsed = now - t0
            time_ms = int(elapsed * 1000.0) & 0xFFFFFFFF
            t_us = int(elapsed * 1e6)
            period_us = int(1e6 / max(self.rate_hz, 1))

            batch.extend(self._handle_commands(cycle_seq, time_ms))

            cycle_seq += 1
            tx, ty, tz = self._true_tag_xyz(elapsed)
            entries: list[tp.BurstAnchor] = []
            for a in self.anchors:
                aid = a.anchor_id
                if not (self.active_mask & (1 << (aid - 1))):
                    entries.append(
                        tp.BurstAnchor(aid, tp.STATUS_TIMEOUT, tp.BURST_MODE_NONE, None, tp.INT16_MIN, tp.INT16_MIN, tp.INT16_MIN)
                    )
                    continue
                true_mm = math.dist((tx, ty, tz), (a.x, a.y, a.z)) * 1000.0
                hw_bias_mm = self.sim_hw_biases_mm.get(aid, 0.0)
                noise_mm = rng.gauss(0.0, 14.0)
                raw_mm = true_mm + hw_bias_mm + noise_mm
                cal_ok = bool(self.tag_cal_mask & (1 << (aid - 1)))
                if cal_ok:
                    reported_mm = max(10, round(raw_mm - self.tag_cal_biases_um[aid] / 1000.0))
                    flags = tp.BURST_FLAG_RADIO_OK | tp.BURST_FLAG_CAL_OK | tp.BURST_FLAG_VALID
                    status = 0
                else:
                    reported_mm = max(10, round(raw_mm))
                    flags = tp.BURST_FLAG_RADIO_OK
                    status = tp.STATUS_CALIBRATION_MISSING

                fp_cdbm = int(-7900 - true_mm * 0.15 + rng.gauss(0, 35))
                nlos_extra = 850 if (aid == 6 and int(elapsed) % 18 in (7, 8)) else 320
                rx_cdbm = fp_cdbm + nlos_extra
                ci_ppm_x100 = int(-120 + aid * 18 + 15 * math.sin(elapsed * 0.1))
                entries.append(
                    tp.BurstAnchor(aid, status, flags, reported_mm, fp_cdbm, rx_cdbm, ci_ppm_x100)
                )

            burst = tp.RangeBurstMessage(
                sequence=cycle_seq,
                time_ms=time_ms,
                boot_id=boot_id,
                cycle_seq=cycle_seq,
                meas_seq_first=meas_seq,
                t_us=t_us,
                period_us=period_us,
                anchors=tuple(entries),
            )
            meas_seq += sum(1 for e in entries if e.radio_ok)
            batch.append((burst, now))

            if now - last_info >= 1.0:
                batch.append((self._make_info_msg(cycle_seq, time_ms), now))
                self.uart_metrics.emit(cycle_seq, 0, 0, float(self.rate_hz * 92))
                last_info = now

            if now - last_flush >= 0.025 or len(batch) >= 32:
                self.batch_ready.emit(batch)
                batch = []
                last_flush = now

            sleep_s = max(0.002, (1.0 / self.rate_hz) - (time.monotonic() - now))
            time.sleep(sleep_s)

        self.link_status.emit("disconnected", "Demo Stopped")
