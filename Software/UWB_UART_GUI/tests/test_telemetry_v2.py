from __future__ import annotations

from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from telemetry_protocol import (  # noqa: E402
    CMD_SET_DS_CAL,
    GatewayHealthMessage,
    TYPE_GATEWAY_HEALTH,
    CmdAckMessage,
    INFO_FLAG_FPP_CORRECTED,
    ProtocolError,
    RangeMeasMessage,
    SnifferFrameMessage,
    TYPE_CMD,
    TYPE_CMD_ACK,
    TYPE_RANGE_MEAS,
    TYPE_SNIFFER_FRAME,
    TelemetryStreamParser,
    decode_frame,
    encode_command,
    encode_frame,
    fpp_to_legacy_scale_cdbm,
)


def _decode(frame_type: int, payload: bytes, sequence: int = 1):
    return decode_frame(TelemetryStreamParser().feed(encode_frame(frame_type, sequence, 0, payload))[0])


class TelemetryV2Tests(unittest.TestCase):
    def test_range_meas(self) -> None:
        payload = struct.pack(
            "<BHIQHBBHBiiihhhhHHhH", 1, 0x1234, 9, 5_000_000, 2, 0x10, 0, 0x0003, 0,
            4210, 4100, 4095, -8100, -7300, -32768, -32768, 20, 0x2000, 150, 3200,
        )
        message = _decode(TYPE_RANGE_MEAS, payload)
        self.assertIsInstance(message, RangeMeasMessage)
        self.assertEqual((message.anchor_id, message.raw_mm, message.corrected_mm), (2, 4210, 4100))
        self.assertTrue(message.calibrated)
        self.assertAlmostEqual(message.nlos_indicator_db, 8.0)
        self.assertEqual(message.ci_ppm_x100, 150)

    def test_range_meas_rejects_bad_length_and_schema(self) -> None:
        with self.assertRaises(ProtocolError):
            _decode(TYPE_RANGE_MEAS, bytes(49))
        with self.assertRaises(ProtocolError):
            _decode(TYPE_RANGE_MEAS, bytes((2,)) + bytes(49))

    def test_cmd_ack(self) -> None:
        message = _decode(TYPE_CMD_ACK, bytes((CMD_SET_DS_CAL, 5)), sequence=33)
        self.assertIsInstance(message, CmdAckMessage)
        self.assertEqual(message.sequence, 33)
        self.assertFalse(message.ok)
        self.assertEqual(message.result_name, "NOT_PAUSED")

    def test_sniffer_frame(self) -> None:
        body = bytes(range(12))
        payload = struct.pack("<BIBhhBB", 1, 0x89ABCDEF, 0x12, -8000, -7400, 1, len(body)) + body
        message = _decode(TYPE_SNIFFER_FRAME, payload)
        self.assertIsInstance(message, SnifferFrameMessage)
        self.assertEqual(message.rx_timestamp, 0x1289ABCDEF)
        self.assertEqual(message.frame, body)

    def test_command_roundtrip(self) -> None:
        data = encode_command(7, CMD_SET_DS_CAL, struct.pack("<HiB", 1, -12500, 1))
        frame = TelemetryStreamParser().feed(data)[0]
        self.assertEqual(frame.type, TYPE_CMD)
        self.assertEqual(frame.sequence, 7)
        self.assertEqual(frame.payload[0], CMD_SET_DS_CAL)
        self.assertEqual(struct.unpack("<HiB", frame.payload[1:]), (1, -12500, 1))

    def test_gateway_health(self) -> None:
        payload = struct.pack("<B14I", 1, 1000, 2000, 30, 1, 0, 0, 29, 1, 64, 64, 2, 3, 4, 5)
        message = _decode(TYPE_GATEWAY_HEALTH, payload)
        self.assertIsInstance(message, GatewayHealthMessage)
        self.assertEqual((message.frames_ok, message.crc_errors), (30, 1))
        self.assertEqual((message.usb_dropped, message.uart_fifo_overflow), (1, 2))
        self.assertEqual((message.uart_buffer_full, message.uart_parity_errors), (3, 5))

    def test_fpp_scale(self) -> None:
        self.assertEqual(fpp_to_legacy_scale_cdbm(-8300, INFO_FLAG_FPP_CORRECTED), -9504)
        self.assertEqual(fpp_to_legacy_scale_cdbm(-9504, 0x05), -9504)
        self.assertEqual(fpp_to_legacy_scale_cdbm(-9504, None), -9504)


if __name__ == "__main__":
    unittest.main()
