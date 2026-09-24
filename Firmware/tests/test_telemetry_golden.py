"""Decode the firmware golden telemetry file with the GUI decoder.

Usage: py -3 test_telemetry_golden.py <golden.bin>
"""

from __future__ import annotations

import sys
from pathlib import Path

GUI_DIR = Path(__file__).resolve().parents[2] / "Software" / "UWB_UART_GUI"
sys.path.insert(0, str(GUI_DIR))

import telemetry_protocol as tp  # noqa: E402


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main(path: str) -> int:
    data = Path(path).read_bytes()
    parser = tp.TelemetryStreamParser()
    frames = parser.feed(data)
    check(parser.counters.crc_errors == 0, "CRC errors in golden stream")
    messages = [tp.decode_frame(frame) for frame in frames]
    by_type = {type(message).__name__: message for message in messages}
    check(len(messages) == 9, f"expected 9 frames, got {len(messages)}")

    info = by_type["InfoMessage"]
    check(info.flags & tp.INFO_FLAG_FPP_CORRECTED, "INFO must flag corrected FPP")
    check(info.calibrated_mask == 0x01, "runtime calibrated mask")
    check(dict(info.active_offsets_um)[1] == 123456, "A1 bias in INFO")

    rng = by_type["RangeMessage"]
    check(rng.sequence == 77 and rng.samples[0].raw_mm == 5000, "RANGE snapshot")
    check(rng.samples[0].fpp_cdbm == -8123, "RANGE fpp")

    stats = by_type["StatsMessage"]
    check((stats.poll_count, stats.cycle_hz, stats.operation_hz) == (1000, 50, 180), "STATS")

    meas = by_type["RangeMeasMessage"]
    check(meas.boot_id == 0xBEEF and meas.meas_seq == 42, "RANGE_MEAS ids")
    check(meas.meas_time_us == 123456789012, "RANGE_MEAS time")
    check((meas.anchor_id, meas.txn, meas.flags) == (3, 0x5A, 0x000B), "RANGE_MEAS header")
    check((meas.raw_mm, meas.corrected_mm, meas.filtered_mm) == (5100, 5000, 4995), "ranges")
    check((meas.fp_cdbm, meas.rx_cdbm, meas.anchor_fp_cdbm, meas.anchor_rx_cdbm)
          == (-8200, -7600, -8300, -7700), "powers")
    check((meas.std_noise, meas.fp_index, meas.ci_ppm_x100, meas.slot_us)
          == (35, 0x2A40, -57, 3100), "quality")
    check(meas.calibrated and abs(meas.nlos_indicator_db - 6.0) < 1e-9, "helpers")

    diag = by_type["DiagAnchorMessage"]
    check(diag.anchor_id == 2 and diag.active and diag.backed_off and not diag.calibrated,
          "DIAG_ANCHOR flags")
    check((diag.success, diag.probes, diag.resp_streak, diag.ds_streak) == (11, 21, 3, 1),
          "DIAG_ANCHOR counters")
    check((diag.slot_max_us, diag.poll_tx_max_us) == (4100, 330), "DIAG_ANCHOR timing")

    anchor = by_type["AnchorInfoMessage"]
    check(anchor.anchor_id == 4 and anchor.position_mm == (1000, -2000, 2500), "ANCHOR_INFO")
    check(anchor.build_hash == 0x08D28188 and anchor.rx_antenna_delay == 16437, "ANCHOR build")
    check(anchor.build_config_hash == 0xAABBCCDD, "ANCHOR config hash")

    ack = by_type["CmdAckMessage"]
    check(ack.sequence == 77 and ack.command_id == 0x07 and ack.ok, "CMD_ACK")
    check(ack.data == bytes((1, 2, 3)), "CMD_ACK data")

    system = by_type["DiagSystemMessage"]
    check(system.boot_id == 0xBEEF and system.boot_count == 2, "DIAG_SYSTEM boot")
    check(system.radio_recoveries == 4 and system.config_checks == 99, "DIAG_SYSTEM health")
    check(system.locked and system.settings_status == 0x03, "DIAG_SYSTEM cmd/settings")
    check(system.rx_errors["lde"] == 8 and system.rx_errors["soft_resets"] == 9, "RX errors")
    check(system.ds_ok == 1234, "DS totals")
    check(system.temperature_c is not None and abs(system.temperature_c - 34.4) < 0.01,
          "temperature")
    check(system.vbat_v is not None and abs(system.vbat_v - 3.3) < 0.001, "vbat")

    device = by_type["DeviceInfoMessage"]
    check(device.role == 1 and device.frame_version == 2, "DEVICE_INFO role/protocol")
    check(device.otp_part_id == 0xA1B2C3D4 and device.otp_xtal_trim == 0x15, "DEVICE_INFO OTP")
    check(device.device_id == 0x12345678 and device.tx_power_register == 0x1E1E1E1E,
          "DEVICE_INFO radio")
    check(device.calibrated_mask == 0x01 and device.active_mask == 0x0F, "DEVICE_INFO masks")
    check(device.calibration_profile_id != 0, "DEVICE_INFO calibration profile")

    print("telemetry golden decode passed (9 frame types, C encoder == Python decoder)")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1]))
    except (AssertionError, KeyError, tp.ProtocolError) as error:
        print(f"ERROR: {error!r}", file=sys.stderr)
        raise SystemExit(1)
