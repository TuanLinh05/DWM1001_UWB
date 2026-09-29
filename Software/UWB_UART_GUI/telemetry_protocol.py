"""Decoder for the DWM1001 Tag UART telemetry stream.

The parser is intentionally independent from Tkinter and pyserial so it can be
unit-tested on a PC and reused by logging or calibration tools later.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Mapping, Union


SOF = b"\xAA\x55"
VERSION = 1
HEADER_SIZE = 14
CRC_SIZE = 2
MAX_PAYLOAD = 256

TYPE_INFO = 0x00
TYPE_RANGE = 0x01
TYPE_STATS = 0x02
TYPE_RANGE_MEAS = 0x10
TYPE_DIAG_ANCHOR = 0x11
TYPE_ANCHOR_INFO = 0x12
TYPE_CMD_ACK = 0x13
TYPE_SNIFFER_FRAME = 0x14
TYPE_DIAG_SYSTEM = 0x15
TYPE_DEVICE_INFO = 0x16
TYPE_GATEWAY_HEALTH = 0x17
TYPE_RANGE_BURST = 0x18
TYPE_CMD = 0x20

INFO_FLAG_HW_ANTENNA_DELAY = 0x01
INFO_FLAG_LEGACY_OFFSET = 0x02
INFO_FLAG_DS_BUILD = 0x04
INFO_FLAG_FPP_CORRECTED = 0x20
INFO_FLAG_BURST = 0x40

# Firmware before the RXPACC fix reported first-path power 20*log10(4) dB too
# low. Host code tuned on those logs converts corrected FPP back with this.
FPP_RXPACC_FIX_CDB = 1204

MEAS_FLAG_RADIO_OK = 0x0001
MEAS_FLAG_CAL_OK = 0x0002
MEAS_FLAG_FILTER_OK = 0x0004
MEAS_FLAG_ANCHOR_DIAG = 0x0008

MEAS_MODE_NAMES = {0: "DS", 1: "SS", 2: "SS_FALLBACK"}

# RANGE_BURST (0x18) per-anchor flags: bits 0-1 mode (MEAS_MODE_NAMES, 3 = none).
BURST_FLAG_MODE_MASK = 0x03
BURST_MODE_NONE = 0x03
BURST_FLAG_CAL_OK = 0x04
BURST_FLAG_VALID = 0x08
BURST_FLAG_ANCHOR_LATE = 0x10
BURST_FLAG_RADIO_OK = 0x20
BURST_RANGE_NONE = 0xFFFF
BURST_HEADER_SIZE = 22
BURST_ENTRY_SIZE = 8

# Host -> TAG commands (Firmware/common/include/uwb_cmd.h).
CMD_PING = 0x01
CMD_GET_DEVICE_INFO = 0x02
CMD_PAUSE = 0x03
CMD_RESUME = 0x04
CMD_SET_ANCHOR_MASK = 0x05
CMD_SET_DS_CAL = 0x06
CMD_GET_DS_CAL = 0x07
CMD_SET_ANT_DELAY = 0x08
CMD_SET_TX_POWER = 0x09
CMD_SAVE_SETTINGS = 0x0A
CMD_FACTORY_RESET = 0x0B
CMD_REBOOT = 0x0C
CMD_TIME_SYNC = 0x0D
CMD_SET_LOCK = 0x0E
CMD_SET_TELEMETRY = 0x0F
CMD_SET_BURST = 0x10

CMD_RESULT_NAMES = {
    0x00: "OK",
    0x01: "UNKNOWN_COMMAND",
    0x02: "BAD_LENGTH",
    0x03: "BAD_ARGUMENT",
    0x04: "LOCKED",
    0x05: "NOT_PAUSED",
    0x06: "STORAGE_ERROR",
    0x07: "RADIO_ERROR",
    0x08: "UNSUPPORTED",
}

TELEM_FEATURE_RANGE_SNAPSHOT = 0x01
TELEM_FEATURE_RANGE_MEAS = 0x02
TELEM_FEATURE_DIAG = 0x04

# telemetry.h: the TAG refuses RANGE_MEAS on a UART slower than this.
TELEM_RANGE_MEAS_MIN_BAUD = 460800
# uart_tx.h UART_TX_BUF_SIZE: the TAG TX ring behind DIAG uart_high_water.
TAG_UART_TX_BUFFER_BYTES = 1024

# RANGE_MEAS (TYPE 0x10) payload, see Telem_SendRangeMeas() in telemetry.c.
RANGE_MEAS_FORMAT = "<BHIQHBBHBiiihhhhHHhH"
RANGE_MEAS_PAYLOAD_SIZE = struct.calcsize(RANGE_MEAS_FORMAT)

INT16_MIN = -32768

STATUS_TIMEOUT = 0x01
STATUS_RX_ERROR = 0x02
STATUS_BAD_FRAME = 0x04
STATUS_COMPUTE_ERROR = 0x08
STATUS_DS_FALLBACK = 0x10
STATUS_CALIBRATION_MISSING = 0x20
STATUS_RANGE_REJECT = 0x40
STATUS_FILTER_REACQUIRE = 0x80


class ProtocolError(ValueError):
    """A frame has a valid envelope but an invalid typed payload."""


@dataclass(frozen=True)
class Frame:
    version: int
    type: int
    payload_length: int
    sequence: int
    time_ms: int
    payload: bytes


@dataclass(frozen=True)
class InfoMessage:
    sequence: int
    time_ms: int
    schema: int
    flags: int
    ranging_mode: int
    anchor_count: int
    calibrated_mask: int
    filter_mode: int
    phy_profile_id: int
    spi_clock_mhz: int
    motion_mode: int
    global_motion_state: int
    active_offsets_um: tuple[tuple[int, int], ...]


@dataclass(frozen=True)
class AnchorSample:
    anchor_id: int
    valid: bool
    status: int
    age_ms: int
    raw_mm: int
    filtered_mm: int
    fpp_cdbm: int


@dataclass(frozen=True)
class RangeMessage:
    sequence: int
    time_ms: int
    samples: tuple[AnchorSample, ...]


@dataclass(frozen=True)
class StatsMessage:
    sequence: int
    time_ms: int
    poll_count: int
    response_ok_count: int
    rx_timeout_count: int
    rx_error_count: int
    cycle_overrun_count: int
    uart_overflow_count: int
    cycle_hz: int
    operation_hz: int


@dataclass(frozen=True)
class RangeMeasMessage:
    """One ranging measurement (TYPE 0x10). raw_mm is before any offset."""

    sequence: int
    time_ms: int
    boot_id: int
    meas_seq: int
    meas_time_us: int
    anchor_id: int
    txn: int
    mode: int
    flags: int
    status: int
    raw_mm: int
    corrected_mm: int
    filtered_mm: int
    fp_cdbm: int
    rx_cdbm: int
    anchor_fp_cdbm: int
    anchor_rx_cdbm: int
    std_noise: int
    fp_index: int
    ci_ppm_x100: int
    slot_us: int

    @property
    def calibrated(self) -> bool:
        return bool(self.flags & MEAS_FLAG_CAL_OK)

    @property
    def mode_name(self) -> str:
        return MEAS_MODE_NAMES.get(self.mode, f"0x{self.mode:02X}")

    @property
    def nlos_indicator_db(self) -> float | None:
        """RX power minus first-path power (APS006 Part 3); >6-10 dB suggests NLOS."""
        if INT16_MIN in (self.fp_cdbm, self.rx_cdbm):
            return None
        return (self.rx_cdbm - self.fp_cdbm) / 100.0


@dataclass(frozen=True)
class BurstAnchor:
    """One anchor of one burst cycle, as the TAG packs it (8 bytes)."""

    anchor_id: int
    status: int
    flags: int
    range_mm: int | None        # calibrated with CAL_OK, raw otherwise
    fp_cdbm: int                # INT16_MIN = unknown; 0.5 dB resolution
    rx_cdbm: int                # fp + (RX - FP); INT16_MIN = unknown
    ci_ppm_x100: int            # INT16_MIN = unknown; 0.25 ppm resolution

    @property
    def radio_ok(self) -> bool:
        return bool(self.flags & BURST_FLAG_RADIO_OK)

    @property
    def mode(self) -> int:
        return self.flags & BURST_FLAG_MODE_MASK

    @property
    def mode_name(self) -> str:
        return "NONE" if self.mode == BURST_MODE_NONE else MEAS_MODE_NAMES.get(self.mode, "?")


@dataclass(frozen=True)
class RangeBurstMessage:
    """Every anchor of one burst DS-TWR cycle (TYPE 0x18), one cycle late.

    t_us is the POLL time of that cycle on the TAG clock, period_us the POLL
    to next POLL interval (0 = unknown, 65535 = saturated).
    """

    sequence: int
    time_ms: int
    boot_id: int
    cycle_seq: int
    meas_seq_first: int
    t_us: int
    period_us: int
    anchors: tuple[BurstAnchor, ...]

    def to_range_meas(self, offsets_um: Mapping[int, int] | None = None
                      ) -> tuple[RangeMeasMessage, ...]:
        """The completed exchanges as RANGE_MEAS records, numbered the way
        the TAG numbers them (meas_seq_first, +1, ... over RADIO_OK entries).

        A calibrated range_mm is the corrected value; raw_mm is rebuilt with
        the INFO offsets (corrected = raw - offset) when they are given.
        filtered_mm is the same range with FILTER_OK when the TAG conditioner
        accepted it (the burst record carries no separate filter output).
        Fields a burst record does not carry are 0 or INT16_MIN.
        """

        records = []
        meas_seq = self.meas_seq_first
        for anchor in self.anchors:
            if not anchor.radio_ok or anchor.range_mm is None:
                continue
            flags = MEAS_FLAG_RADIO_OK
            corrected = anchor.range_mm
            raw = anchor.range_mm
            if anchor.flags & BURST_FLAG_CAL_OK:
                flags |= MEAS_FLAG_CAL_OK
                if offsets_um:
                    raw = corrected + round(offsets_um.get(anchor.anchor_id, 0) / 1000.0)
            if anchor.flags & BURST_FLAG_VALID:
                flags |= MEAS_FLAG_FILTER_OK
            records.append(RangeMeasMessage(
                sequence=self.sequence, time_ms=self.time_ms, boot_id=self.boot_id,
                meas_seq=meas_seq & 0xFFFFFFFF, meas_time_us=self.t_us,
                anchor_id=anchor.anchor_id, txn=self.cycle_seq & 0xFF, mode=anchor.mode,
                flags=flags, status=anchor.status, raw_mm=raw, corrected_mm=corrected,
                filtered_mm=corrected if flags & MEAS_FLAG_FILTER_OK else 0,
                fp_cdbm=anchor.fp_cdbm, rx_cdbm=anchor.rx_cdbm,
                anchor_fp_cdbm=INT16_MIN, anchor_rx_cdbm=INT16_MIN, std_noise=0,
                fp_index=0, ci_ppm_x100=anchor.ci_ppm_x100, slot_us=0,
            ))
            meas_seq += 1
        return tuple(records)


@dataclass(frozen=True)
class DiagAnchorMessage:
    sequence: int
    time_ms: int
    anchor_id: int
    active: bool
    backed_off: bool
    calibrated: bool
    success: int
    resp_timeouts: int
    poll_tx_timeouts: int
    rx_errors: int
    poll_skipped: int
    cal_missing: int
    ds_ok: int
    report_timeouts: int
    ds_fallbacks: int
    txn_mismatch: int
    probes: int
    resp_streak: int
    ds_streak: int
    slot_us: int
    slot_max_us: int
    resp_wait_max_us: int
    processing_max_us: int
    poll_tx_max_us: int
    burst_late: int = 0          # RESPs whose anchor missed its burst slot


@dataclass(frozen=True)
class AnchorInfoMessage:
    sequence: int
    time_ms: int
    anchor_id: int
    anchor_status: int
    position_valid: bool
    position_mm: tuple[int, int, int]
    build_hash: int
    build_dirty: bool
    tx_power_mode: int
    tx_antenna_delay: int
    rx_antenna_delay: int
    boot_count: int
    build_config_hash: int


@dataclass(frozen=True)
class CmdAckMessage:
    sequence: int
    time_ms: int
    command_id: int
    result: int
    data: bytes

    @property
    def ok(self) -> bool:
        return self.result == 0

    @property
    def result_name(self) -> str:
        return CMD_RESULT_NAMES.get(self.result, f"0x{self.result:02X}")


@dataclass(frozen=True)
class SnifferFrameMessage:
    sequence: int
    time_ms: int
    rx_timestamp: int
    fp_cdbm: int
    rx_cdbm: int
    status: int
    frame: bytes


@dataclass(frozen=True)
class DiagSystemMessage:
    sequence: int
    time_ms: int
    boot_id: int
    boot_count: int
    reset_cause: int
    uptime_ms: int
    cycle_count: int
    cycle_us: int
    cycle_max_us: int
    cycle_overruns: int
    spi_errors: int
    radio_recoveries: int
    radio_recovery_failures: int
    config_checks: int
    config_mismatches: int
    last_config_mismatch: int
    fault_hold: bool
    last_fault_cause: int
    paused: bool
    active_mask: int
    uart_tx_overflow: int
    uart_rx_overflow: int
    uart_high_water: int
    meas_queue_drops: int
    cmd_ok: int
    cmd_rejected: int
    cmd_crc_errors: int
    locked: bool
    settings_status: int
    rx_errors: dict[str, int]
    ds_ok: int
    ds_fallback: int
    ds_report_timeout: int
    ds_final_tx_timeout: int
    temperature_c: float | None
    vbat_v: float | None
    # Burst DS-TWR (firmware 0.5+; None/0 from older firmware).
    burst_mode: int | None = None
    burst_base_uus: int = 0
    burst_slot_uus: int = 0
    burst_final_margin_uus: int = 0
    burst_gap_us: int = 0
    burst_period_us: int = 0
    burst_final_late: int = 0
    burst_record_drops: int = 0


@dataclass(frozen=True)
class DeviceInfoMessage:
    sequence: int
    time_ms: int
    role: int
    node_address: int
    git_hash: int
    git_dirty: bool
    frame_version: int
    telemetry_features: int
    wait4resp: bool
    device_id: int
    boot_id: int
    boot_count: int
    reset_cause: int
    tx_power_mode: int
    reference_tuning: bool
    tx_antenna_delay: int
    rx_antenna_delay: int
    tx_power_register: int
    spi_clock_mhz: int
    otp_valid: bool
    otp_part_id: int
    otp_lot_id: int
    otp_ldotune: int
    otp_xtal_trim: int
    otp_revision: int
    otp_vbat_cal: int
    otp_vtemp_cal: int
    applied_xtal_trim: int
    ldo_kicked: bool
    settings_status: int
    uart_baud: int
    active_mask: int
    calibrated_mask: int
    build_config_hash: int
    calibration_profile_id: int


@dataclass(frozen=True)
class GatewayHealthMessage:
    """ESP32-C3 gateway health (TYPE 0x17), emitted once per second."""

    sequence: int
    time_ms: int
    uptime_ms: int
    uart_bytes: int
    frames_ok: int
    crc_errors: int
    length_errors: int
    version_errors: int
    usb_forwarded: int
    usb_dropped: int
    usb_rx_bytes: int
    cmd_bytes: int
    uart_fifo_overflow: int
    uart_buffer_full: int
    uart_frame_errors: int
    uart_parity_errors: int


DecodedMessage = Union[
    GatewayHealthMessage,
    InfoMessage, RangeMessage, StatsMessage, RangeMeasMessage, DiagAnchorMessage,
    AnchorInfoMessage, CmdAckMessage, SnifferFrameMessage, DiagSystemMessage,
    DeviceInfoMessage, RangeBurstMessage,
]

RX_ERROR_FIELDS = (
    "phy_header", "fcs", "sync_loss", "frame_timeout", "lde", "overrun",
    "preamble_timeout", "sfd_timeout", "filtered", "incomplete", "soft_resets",
)


@dataclass
class ParserCounters:
    bytes_received: int = 0
    valid_frames: int = 0
    discarded_bytes: int = 0
    crc_errors: int = 0
    length_errors: int = 0
    version_errors: int = 0
    decode_errors: int = 0

    def snapshot(self) -> "ParserCounters":
        return ParserCounters(**vars(self))


def crc16_ccitt(data: bytes | bytearray | memoryview) -> int:
    """CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF."""

    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(frame_type: int, sequence: int, time_ms: int, payload: bytes) -> bytes:
    """Build a protocol frame. Used by tests and GUI demo mode."""

    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload is too large")
    header = SOF + struct.pack("<BBHII", VERSION, frame_type, len(payload), sequence, time_ms)
    crc = crc16_ccitt(header[2:] + payload)
    return header + payload + struct.pack("<H", crc)


class TelemetryStreamParser:
    """Incremental parser that tolerates fragmented frames and arbitrary noise."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.counters = ParserCounters()

    def reset(self) -> None:
        self._buffer.clear()
        self.counters = ParserCounters()

    def feed(self, data: bytes | bytearray | memoryview) -> list[Frame]:
        if not data:
            return []

        incoming = bytes(data)
        self.counters.bytes_received += len(incoming)
        self._buffer.extend(incoming)
        frames: list[Frame] = []

        while True:
            sof_index = self._buffer.find(SOF)
            if sof_index < 0:
                # Retain a trailing 0xAA because it can be the first SOF byte of
                # the next serial read.
                keep = 1 if self._buffer and self._buffer[-1] == SOF[0] else 0
                discarded = len(self._buffer) - keep
                self.counters.discarded_bytes += discarded
                if keep:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break

            if sof_index:
                self.counters.discarded_bytes += sof_index
                del self._buffer[:sof_index]

            if len(self._buffer) < 6:
                break

            payload_length = struct.unpack_from("<H", self._buffer, 4)[0]
            if payload_length > MAX_PAYLOAD:
                self.counters.length_errors += 1
                self.counters.discarded_bytes += 1
                del self._buffer[0]
                continue

            total_length = HEADER_SIZE + payload_length + CRC_SIZE
            if len(self._buffer) < total_length:
                break

            candidate = bytes(self._buffer[:total_length])
            received_crc = struct.unpack_from("<H", candidate, total_length - CRC_SIZE)[0]
            calculated_crc = crc16_ccitt(candidate[2:-CRC_SIZE])
            if received_crc != calculated_crc:
                self.counters.crc_errors += 1
                self.counters.discarded_bytes += 1
                del self._buffer[0]
                continue

            version = candidate[2]
            if version != VERSION:
                self.counters.version_errors += 1
                self.counters.discarded_bytes += 1
                del self._buffer[0]
                continue

            frames.append(
                Frame(
                    version=version,
                    type=candidate[3],
                    payload_length=payload_length,
                    sequence=struct.unpack_from("<I", candidate, 6)[0],
                    time_ms=struct.unpack_from("<I", candidate, 10)[0],
                    payload=candidate[HEADER_SIZE:-CRC_SIZE],
                )
            )
            self.counters.valid_frames += 1
            del self._buffer[:total_length]

        return frames


def decode_frame(frame: Frame) -> DecodedMessage:
    decoder = _DECODERS.get(frame.type)
    if decoder is None:
        raise ProtocolError(f"unknown frame type 0x{frame.type:02X}")
    return decoder(frame)


def encode_command(sequence: int, command_id: int, arguments: bytes = b"") -> bytes:
    """Build a host -> TAG command frame (TYPE 0x20)."""

    return encode_frame(TYPE_CMD, sequence & 0xFFFFFFFF, 0, bytes((command_id,)) + arguments)


def fpp_to_legacy_scale_cdbm(fpp_cdbm: int, info_flags: int | None) -> int:
    """Map first-path power onto the pre-fix scale used by existing thresholds."""

    if info_flags is not None and info_flags & INFO_FLAG_FPP_CORRECTED:
        return fpp_cdbm - FPP_RXPACC_FIX_CDB
    return fpp_cdbm


def _require_schema(frame: Frame, minimum_length: int, name: str) -> None:
    if len(frame.payload) < minimum_length:
        raise ProtocolError(f"invalid {name} payload length {len(frame.payload)}")
    if frame.payload[0] != 1:
        raise ProtocolError(f"unsupported {name} schema {frame.payload[0]}")


def decode_range_meas(frame: Frame) -> RangeMeasMessage:
    if len(frame.payload) != RANGE_MEAS_PAYLOAD_SIZE:
        raise ProtocolError(f"invalid RANGE_MEAS payload length {len(frame.payload)}")
    _require_schema(frame, RANGE_MEAS_PAYLOAD_SIZE, "RANGE_MEAS")
    values = struct.unpack(RANGE_MEAS_FORMAT, frame.payload)
    return RangeMeasMessage(frame.sequence, frame.time_ms, *values[1:])


def encode_range_meas_payload(message: RangeMeasMessage) -> bytes:
    """Inverse of decode_range_meas (GUI demo source and tests)."""

    return struct.pack(
        RANGE_MEAS_FORMAT, 1, message.boot_id, message.meas_seq, message.meas_time_us,
        message.anchor_id, message.txn, message.mode, message.flags, message.status,
        message.raw_mm, message.corrected_mm, message.filtered_mm,
        message.fp_cdbm, message.rx_cdbm, message.anchor_fp_cdbm, message.anchor_rx_cdbm,
        message.std_noise, message.fp_index, message.ci_ppm_x100, message.slot_us,
    )


def _burst_fp_cdbm(code: int) -> int:
    return INT16_MIN if code == 0xFF else -code * 50


def decode_range_burst(frame: Frame) -> RangeBurstMessage:
    payload = frame.payload
    _require_schema(frame, BURST_HEADER_SIZE, "RANGE_BURST")
    boot_id, cycle_seq, meas_seq_first, t_us, period_us, count = struct.unpack_from(
        "<HIIQHB", payload, 1)
    if len(payload) != BURST_HEADER_SIZE + BURST_ENTRY_SIZE * count:
        raise ProtocolError(
            f"invalid RANGE_BURST payload length {len(payload)} for {count} anchors")
    anchors = []
    for offset in range(BURST_HEADER_SIZE, len(payload), BURST_ENTRY_SIZE):
        anchor_id, status, flags, range_mm, fp_code, nlos_code, ci_code = struct.unpack_from(
            "<BBBHBBb", payload, offset)
        fp_cdbm = _burst_fp_cdbm(fp_code)
        rx_cdbm = (INT16_MIN if fp_cdbm == INT16_MIN or nlos_code == 0xFF
                   else fp_cdbm + nlos_code * 10)
        anchors.append(BurstAnchor(
            anchor_id=anchor_id, status=status, flags=flags,
            range_mm=None if range_mm == BURST_RANGE_NONE else range_mm,
            fp_cdbm=fp_cdbm, rx_cdbm=rx_cdbm,
            ci_ppm_x100=INT16_MIN if ci_code == -128 else ci_code * 25,
        ))
    return RangeBurstMessage(frame.sequence, frame.time_ms, boot_id, cycle_seq,
                             meas_seq_first, t_us, period_us, tuple(anchors))


def encode_range_burst_payload(message: RangeBurstMessage) -> bytes:
    """Inverse of decode_range_burst (GUI demo source and tests)."""

    payload = bytearray(struct.pack(
        "<BHIIQHB", 1, message.boot_id, message.cycle_seq, message.meas_seq_first,
        message.t_us, min(message.period_us, 0xFFFF), len(message.anchors)))
    for anchor in message.anchors:
        fp_code = 0xFF if anchor.fp_cdbm == INT16_MIN else max(
            0, min(254, (-anchor.fp_cdbm + 25) // 50))
        nlos_code = 0xFF if INT16_MIN in (anchor.fp_cdbm, anchor.rx_cdbm) else max(
            0, min(254, (anchor.rx_cdbm - anchor.fp_cdbm + 5) // 10))
        ci_code = -128 if anchor.ci_ppm_x100 == INT16_MIN else max(
            -127, min(127, round(anchor.ci_ppm_x100 / 25)))
        payload += struct.pack(
            "<BBBHBBb", anchor.anchor_id, anchor.status, anchor.flags,
            BURST_RANGE_NONE if anchor.range_mm is None else anchor.range_mm,
            fp_code, nlos_code, ci_code)
    return bytes(payload)


def decode_diag_anchor(frame: Frame) -> DiagAnchorMessage:
    _require_schema(frame, 73, "DIAG_ANCHOR")
    v = struct.unpack_from("<BHBB11IHH5I", frame.payload)
    burst_late = struct.unpack_from("<I", frame.payload, 73)[0] \
        if len(frame.payload) >= 77 else 0
    return DiagAnchorMessage(
        frame.sequence, frame.time_ms, v[1], bool(v[2]), bool(v[3] & 0x01),
        bool(v[3] & 0x02), *v[4:], burst_late=burst_late,
    )


def decode_anchor_info(frame: Frame) -> AnchorInfoMessage:
    _require_schema(frame, 29, "ANCHOR_INFO")
    v = struct.unpack_from("<BHBBiiiIBBHHH", frame.payload)
    build_config_hash = struct.unpack_from("<I", frame.payload, 29)[0] \
        if len(frame.payload) >= 33 else 0
    return AnchorInfoMessage(
        frame.sequence, frame.time_ms, v[1], v[2], bool(v[3]), (v[4], v[5], v[6]),
        v[7], bool(v[8]), v[9], v[10], v[11], v[12], build_config_hash,
    )


def decode_cmd_ack(frame: Frame) -> CmdAckMessage:
    if len(frame.payload) < 2:
        raise ProtocolError("invalid CMD_ACK payload")
    return CmdAckMessage(frame.sequence, frame.time_ms, frame.payload[0],
                         frame.payload[1], bytes(frame.payload[2:]))


def decode_sniffer_frame(frame: Frame) -> SnifferFrameMessage:
    _require_schema(frame, 12, "SNIFFER_FRAME")
    ts_low, ts_high, fp_cdbm, rx_cdbm, status, length = struct.unpack_from(
        "<IBhhBB", frame.payload, 1)
    body = frame.payload[12:]
    if len(body) != length:
        raise ProtocolError("SNIFFER_FRAME length mismatch")
    return SnifferFrameMessage(frame.sequence, frame.time_ms, ts_low | (ts_high << 32),
                               fp_cdbm, rx_cdbm, status, bytes(body))


def decode_diag_system(frame: Frame) -> DiagSystemMessage:
    _require_schema(frame, 149, "DIAG_SYSTEM")
    v = struct.unpack_from("<BHHIIIIIIIIIIIIBBBBIIHIIIIBB11IIIIIhH", frame.payload)
    rx_errors = dict(zip(RX_ERROR_FIELDS, v[28:39]))
    temperature = None if v[43] == INT16_MIN else v[43] / 100.0
    vbat = None if v[44] == 0 else v[44] / 1000.0
    burst: dict[str, int | None] = {}
    if len(frame.payload) >= 168:
        b = struct.unpack_from("<B5HII", frame.payload, 149)
        burst = dict(
            burst_mode=b[0], burst_base_uus=b[1], burst_slot_uus=b[2],
            burst_final_margin_uus=b[3], burst_gap_us=b[4], burst_period_us=b[5],
            burst_final_late=b[6], burst_record_drops=b[7],
        )
    return DiagSystemMessage(
        frame.sequence, frame.time_ms,
        boot_id=v[1], boot_count=v[2], reset_cause=v[3], uptime_ms=v[4],
        cycle_count=v[5], cycle_us=v[6], cycle_max_us=v[7], cycle_overruns=v[8],
        spi_errors=v[9], radio_recoveries=v[10], radio_recovery_failures=v[11],
        config_checks=v[12], config_mismatches=v[13], last_config_mismatch=v[14],
        fault_hold=bool(v[15]), last_fault_cause=v[16], paused=bool(v[17]),
        active_mask=v[18], uart_tx_overflow=v[19], uart_rx_overflow=v[20],
        uart_high_water=v[21], meas_queue_drops=v[22], cmd_ok=v[23],
        cmd_rejected=v[24], cmd_crc_errors=v[25], locked=bool(v[26]),
        settings_status=v[27], rx_errors=rx_errors, ds_ok=v[39], ds_fallback=v[40],
        ds_report_timeout=v[41], ds_final_tx_timeout=v[42],
        temperature_c=temperature, vbat_v=vbat, **burst,
    )


def decode_gateway_health(frame: Frame) -> GatewayHealthMessage:
    _require_schema(frame, 57, "GATEWAY_HEALTH")
    values = struct.unpack_from("<B14I", frame.payload)
    return GatewayHealthMessage(frame.sequence, frame.time_ms, *values[1:])


def decode_device_info(frame: Frame) -> DeviceInfoMessage:
    _require_schema(frame, 61, "DEVICE_INFO")
    v = struct.unpack_from("<BBHIBBBBIHHIBBHHIBBIIIBBBBBBBIBB", frame.payload)
    build_config_hash, calibration_profile_id = (0, 0)
    if len(frame.payload) >= 69:
        build_config_hash, calibration_profile_id = struct.unpack_from("<II", frame.payload, 61)
    return DeviceInfoMessage(
        frame.sequence, frame.time_ms,
        role=v[1], node_address=v[2], git_hash=v[3], git_dirty=bool(v[4]),
        frame_version=v[5], telemetry_features=v[6], wait4resp=bool(v[7]),
        device_id=v[8], boot_id=v[9], boot_count=v[10], reset_cause=v[11],
        tx_power_mode=v[12], reference_tuning=bool(v[13]), tx_antenna_delay=v[14],
        rx_antenna_delay=v[15], tx_power_register=v[16], spi_clock_mhz=v[17],
        otp_valid=bool(v[18]), otp_part_id=v[19], otp_lot_id=v[20], otp_ldotune=v[21],
        otp_xtal_trim=v[22], otp_revision=v[23], otp_vbat_cal=v[24],
        otp_vtemp_cal=v[25], applied_xtal_trim=v[26], ldo_kicked=bool(v[27]),
        settings_status=v[28], uart_baud=v[29], active_mask=v[30],
        calibrated_mask=v[31], build_config_hash=build_config_hash,
        calibration_profile_id=calibration_profile_id,
    )


def decode_info(frame: Frame) -> InfoMessage:
    payload = frame.payload
    if len(payload) < 10 or (len(payload) - 10) % 6:
        raise ProtocolError(f"invalid INFO payload length {len(payload)}")
    if payload[0] != 2 or payload[3] != (len(payload) - 10) // 6:
        raise ProtocolError("unsupported INFO schema or inconsistent anchor count")

    offsets: list[tuple[int, int]] = []
    for offset in range(10, len(payload), 6):
        anchor_id, active_offset_um = struct.unpack_from("<Hi", payload, offset)
        offsets.append((anchor_id, active_offset_um))

    return InfoMessage(
        sequence=frame.sequence,
        time_ms=frame.time_ms,
        schema=payload[0],
        flags=payload[1],
        ranging_mode=payload[2],
        anchor_count=payload[3],
        calibrated_mask=payload[4],
        filter_mode=payload[5],
        phy_profile_id=payload[6],
        spi_clock_mhz=payload[7],
        motion_mode=payload[8],
        global_motion_state=payload[9],
        active_offsets_um=tuple(offsets),
    )


def decode_range(frame: Frame) -> RangeMessage:
    payload = frame.payload
    if not payload:
        raise ProtocolError("empty RANGE payload")

    record_count = payload[0]
    expected_length = 1 + record_count * 16
    if len(payload) != expected_length:
        raise ProtocolError(
            f"invalid RANGE payload length {len(payload)} for {record_count} records"
        )

    samples: list[AnchorSample] = []
    for index in range(record_count):
        offset = 1 + index * 16
        anchor_id, valid, status, age_ms, raw_mm, filtered_mm, fpp_cdbm = struct.unpack_from(
            "<HBBHiih", payload, offset
        )
        if valid not in (0, 1) or any(sample.anchor_id == anchor_id for sample in samples):
            raise ProtocolError("invalid validity byte or duplicate anchor ID")
        if valid and status & 0x6F:
            raise ProtocolError("valid measurement carries a failure status")
        samples.append(
            AnchorSample(
                anchor_id=anchor_id,
                valid=bool(valid),
                status=status,
                age_ms=age_ms,
                raw_mm=raw_mm,
                filtered_mm=filtered_mm,
                fpp_cdbm=fpp_cdbm,
            )
        )

    return RangeMessage(frame.sequence, frame.time_ms, tuple(samples))


def decode_stats(frame: Frame) -> StatsMessage:
    if len(frame.payload) != 28:
        raise ProtocolError(f"invalid STATS payload length {len(frame.payload)}")

    values = struct.unpack("<IIIIIIHH", frame.payload)
    return StatsMessage(frame.sequence, frame.time_ms, *values)


def status_names(status: int) -> tuple[str, ...]:
    if status == 0:
        return ("OK",)

    mapping = (
        (STATUS_TIMEOUT, "TIMEOUT"),
        (STATUS_RX_ERROR, "RX_ERROR"),
        (STATUS_BAD_FRAME, "BAD_FRAME"),
        (STATUS_COMPUTE_ERROR, "COMPUTE_ERROR"),
        (STATUS_DS_FALLBACK, "DS_FALLBACK"),
        (STATUS_CALIBRATION_MISSING, "CAL_MISSING"),
        (STATUS_RANGE_REJECT, "RANGE_REJECT"),
        (STATUS_FILTER_REACQUIRE, "REACQUIRE"),
    )
    return tuple(name for bit, name in mapping if status & bit) or (f"0x{status:02X}",)


_DECODERS = {
    TYPE_INFO: decode_info,
    TYPE_RANGE: decode_range,
    TYPE_STATS: decode_stats,
    TYPE_RANGE_MEAS: decode_range_meas,
    TYPE_DIAG_ANCHOR: decode_diag_anchor,
    TYPE_ANCHOR_INFO: decode_anchor_info,
    TYPE_CMD_ACK: decode_cmd_ack,
    TYPE_SNIFFER_FRAME: decode_sniffer_frame,
    TYPE_DIAG_SYSTEM: decode_diag_system,
    TYPE_DEVICE_INFO: decode_device_info,
    TYPE_GATEWAY_HEALTH: decode_gateway_health,
    TYPE_RANGE_BURST: decode_range_burst,
}


@dataclass(frozen=True)
class BurstSettings:
    """SET_BURST (0x10) arguments and CMD_ACK data."""

    mode: int                   # 0 sequential, 1 burst (0xFF = keep, arguments only)
    base_uus: int
    slot_uus: int
    final_margin_uus: int
    gap_us: int
    period_us: int

    def encode(self) -> bytes:
        return struct.pack("<B5H", self.mode, self.base_uus, self.slot_uus,
                           self.final_margin_uus, self.gap_us, self.period_us)

    @classmethod
    def decode(cls, data: bytes) -> "BurstSettings":
        if len(data) != 11:
            raise ProtocolError(f"invalid SET_BURST data length {len(data)}")
        return cls(*struct.unpack("<B5H", data))

    def cycle_estimate_us(self, anchors: int = 8) -> float:
        """Approximate POLL-to-POLL time: POLL preamble, FINAL at base +
        (n - 1) slots + margin after it, FINAL tail, gap and ~150 us of TAG
        work. period_us in RANGE_BURST is the measured value."""
        final_uus = self.base_uus + max(anchors - 1, 0) * self.slot_uus + self.final_margin_uus
        return final_uus * 1.0256 + 150.0 + 30.0 + self.gap_us + 150.0
