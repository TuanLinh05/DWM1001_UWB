from __future__ import annotations

from pathlib import Path
import re
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
    RANGE_MEAS_PAYLOAD_SIZE,
    TAG_UART_TX_BUFFER_BYTES,
    TELEM_FEATURE_DIAG,
    TELEM_FEATURE_RANGE_MEAS,
    TELEM_FEATURE_RANGE_SNAPSHOT,
    TELEM_RANGE_MEAS_MIN_BAUD,
    TelemetryStreamParser,
    decode_frame,
    encode_command,
    encode_frame,
    encode_range_meas_payload,
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

    def test_range_meas_encoder_is_the_inverse_of_the_decoder(self) -> None:
        payload = struct.pack(
            "<BHIQHBBHBiiihhhhHHhH", 1, 0xBEEF, 0xFFFFFFFF, 2**63 + 5, 8, 0xFF, 2, 0x000F,
            0x90, -1, -2147483648, 2147483647, -32768, 32767, -1, 1, 0xFFFF, 0, -32768, 0xFFFF,
        )
        message = _decode(TYPE_RANGE_MEAS, payload, sequence=0xFFFFFFFF)
        self.assertEqual(RANGE_MEAS_PAYLOAD_SIZE, 50)
        self.assertEqual(encode_range_meas_payload(message), payload)
        self.assertEqual(message.mode_name, "SS_FALLBACK")
        self.assertEqual(_decode(TYPE_RANGE_MEAS, encode_range_meas_payload(message)).meas_seq,
                         0xFFFFFFFF)

    def test_range_meas_rejects_bad_length_and_schema(self) -> None:
        with self.assertRaises(ProtocolError):
            _decode(TYPE_RANGE_MEAS, bytes(49))
        with self.assertRaises(ProtocolError):
            _decode(TYPE_RANGE_MEAS, bytes((2,)) + bytes(49))

    def test_host_constants_match_firmware_headers(self) -> None:
        include = Path(__file__).resolve().parents[3] / "Firmware" / "common" / "include"
        if not include.is_dir():
            self.skipTest("firmware sources are not next to the GUI")

        def define(header: str, name: str) -> int:
            text = (include / header).read_text(encoding="utf-8")
            match = re.search(rf"^#define\s+{name}\s+(0x[0-9A-Fa-f]+|\d+)U\b", text, re.MULTILINE)
            self.assertIsNotNone(match, f"{name} not found in {header}")
            return int(match.group(1), 0)

        self.assertEqual(define("telemetry.h", "TELEM_RANGE_MEAS_MIN_BAUD"),
                         TELEM_RANGE_MEAS_MIN_BAUD)
        self.assertEqual(define("telemetry.h", "TELEM_FEATURE_RANGE_SNAPSHOT"),
                         TELEM_FEATURE_RANGE_SNAPSHOT)
        self.assertEqual(define("telemetry.h", "TELEM_FEATURE_RANGE_MEAS"),
                         TELEM_FEATURE_RANGE_MEAS)
        self.assertEqual(define("telemetry.h", "TELEM_FEATURE_DIAG"), TELEM_FEATURE_DIAG)
        self.assertEqual(define("uart_tx.h", "UART_TX_BUF_SIZE"), TAG_UART_TX_BUFFER_BYTES)

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
