"""Decoder for the DWM1001 Tag UART telemetry stream.

The parser is intentionally independent from Tkinter and pyserial so it can be
unit-tested on a PC and reused by logging or calibration tools later.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Union


SOF = b"\xAA\x55"
VERSION = 1
HEADER_SIZE = 14
CRC_SIZE = 2
MAX_PAYLOAD = 256

TYPE_INFO = 0x00
TYPE_RANGE = 0x01
TYPE_STATS = 0x02

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


DecodedMessage = Union[InfoMessage, RangeMessage, StatsMessage]


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
    if frame.type == TYPE_INFO:
        return decode_info(frame)
    if frame.type == TYPE_RANGE:
        return decode_range(frame)
    if frame.type == TYPE_STATS:
        return decode_stats(frame)
    raise ProtocolError(f"unknown frame type 0x{frame.type:02X}")


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
