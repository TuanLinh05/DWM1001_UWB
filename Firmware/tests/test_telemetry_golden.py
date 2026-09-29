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
    check(len(messages) == 11, f"expected 11 frames, got {len(messages)}")

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
    check(diag.burst_late == 22, "DIAG_ANCHOR burst_late")

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
    check(system.burst_mode == 0, "DIAG_SYSTEM burst mode (Tag PCB boots sequential)")
    check((system.burst_base_uus, system.burst_slot_uus, system.burst_final_margin_uus,
           system.burst_gap_us, system.burst_period_us) == (700, 300, 400, 250, 5000),
          "DIAG_SYSTEM burst timing")
    check((system.burst_final_late, system.burst_record_drops) == (23, 24),
          "DIAG_SYSTEM burst counters")

    device = by_type["DeviceInfoMessage"]
    check(device.role == 1 and device.frame_version == 2, "DEVICE_INFO role/protocol")
    check(device.otp_part_id == 0xA1B2C3D4 and device.otp_xtal_trim == 0x15, "DEVICE_INFO OTP")
    check(device.device_id == 0x12345678 and device.tx_power_register == 0x1E1E1E1E,
          "DEVICE_INFO radio")
    check(device.calibrated_mask == 0x01 and device.active_mask == 0x0F, "DEVICE_INFO masks")
    check(device.calibration_profile_id != 0, "DEVICE_INFO calibration profile")

    bursts = [m for m in messages if isinstance(m, tp.RangeBurstMessage)]
    check(len(bursts) == 2, "two RANGE_BURST frames")
    burst, empty = bursts
    check(burst.sequence == 5000 and burst.cycle_seq == 5000, "RANGE_BURST seq")
    check((burst.boot_id, burst.meas_seq_first, burst.t_us, burst.period_us)
          == (0xBEEF, 900, 987654321012, 4480), "RANGE_BURST header")
    check([a.anchor_id for a in burst.anchors] == [1, 5, 8], "RANGE_BURST anchors")
    a1, a5, a8 = burst.anchors
    check((a1.status, a1.range_mm, a1.mode_name) == (0, 4321, "DS"), "A1 range")
    check((a1.fp_cdbm, a1.rx_cdbm, a1.ci_ppm_x100) == (-8200, -7640, -1825),
          "A1 quality codes (0.5 dB, 0.1 dB, 0.25 ppm)")
    check(a5.flags & tp.BURST_FLAG_ANCHOR_LATE and a5.mode_name == "SS_FALLBACK",
          "A5 late + fallback")
    check((a5.range_mm, a5.fp_cdbm, a5.rx_cdbm, a5.ci_ppm_x100)
          == (12345, tp.INT16_MIN, tp.INT16_MIN, tp.INT16_MIN), "A5 unknown quality")
    check(not a8.radio_ok and a8.range_mm is None and a8.mode_name == "NONE", "A8 timeout")
    check(tp.encode_range_burst_payload(burst) == frames[-2].payload, "RANGE_BURST re-encode")
    meas = burst.to_range_meas({1: 123456})
    check([(m.anchor_id, m.meas_seq) for m in meas] == [(1, 900), (5, 901)],
          "RANGE_BURST -> RANGE_MEAS numbering")
    check((meas[0].raw_mm, meas[0].corrected_mm, meas[0].filtered_mm) == (4444, 4321, 4321),
          "raw rebuilt from the INFO offset")
    check(meas[0].flags == (tp.MEAS_FLAG_RADIO_OK | tp.MEAS_FLAG_CAL_OK
                            | tp.MEAS_FLAG_FILTER_OK), "RANGE_MEAS flags")
    check((meas[1].raw_mm, meas[1].flags, meas[1].mode) == (12345, tp.MEAS_FLAG_RADIO_OK, 2),
          "uncalibrated fallback")
    check(empty.period_us == 0xFFFF and empty.anchors == (), "empty RANGE_BURST")

    print("telemetry golden decode passed (10 frame types, C encoder == Python decoder)")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1]))
    except (AssertionError, KeyError, tp.ProtocolError) as error:
        print(f"ERROR: {error!r}", file=sys.stderr)
        raise SystemExit(1)
