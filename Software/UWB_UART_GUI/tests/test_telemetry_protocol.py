from __future__ import annotations

from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from telemetry_protocol import (  # noqa: E402
    InfoMessage,
    ProtocolError,
    RangeMessage,
    StatsMessage,
    STATUS_CALIBRATION_MISSING,
    STATUS_DS_FALLBACK,
    TelemetryStreamParser,
    TYPE_INFO,
    TYPE_RANGE,
    TYPE_STATS,
    crc16_ccitt,
    decode_frame,
    encode_frame,
    status_names,
)


class TelemetryProtocolTests(unittest.TestCase):
    def test_reject_inconsistent_info_count(self) -> None:
        data = encode_frame(TYPE_INFO, 1, 1, bytes((2, 5, 1, 4, 0, 0, 1, 8, 0, 0)))
        with self.assertRaises(ProtocolError):
            decode_frame(TelemetryStreamParser().feed(data)[0])

    def test_reject_valid_calibration_missing(self) -> None:
        data = encode_frame(TYPE_RANGE, 1, 1,
            bytes((1,)) + struct.pack('<HBBHiih', 1, 1, 0x20, 0, 2000, 2000, -8000))
        with self.assertRaises(ProtocolError):
            decode_frame(TelemetryStreamParser().feed(data)[0])

    def test_crc_known_vector(self) -> None:
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_fragmented_uncalibrated_range(self) -> None:
        payload = bytes((1,)) + struct.pack(
            "<HBBHiih", 1, 0, STATUS_CALIBRATION_MISSING, 0, 2375, 0, -8125
        )
        encoded = encode_frame(TYPE_RANGE, 42, 1234, payload)
        parser = TelemetryStreamParser()

        self.assertEqual(parser.feed(encoded[:3]), [])
        self.assertEqual(parser.feed(encoded[3:11]), [])
        frames = parser.feed(encoded[11:])

        self.assertEqual(len(frames), 1)
        message = decode_frame(frames[0])
        self.assertIsInstance(message, RangeMessage)
        self.assertEqual(message.sequence, 42)
        self.assertEqual(message.samples[0].raw_mm, 2375)
        self.assertEqual(message.samples[0].fpp_cdbm, -8125)
        self.assertFalse(message.samples[0].valid)
        self.assertEqual(status_names(message.samples[0].status), ("CAL_MISSING",))

    def test_four_anchor_packet_and_combined_status_bits(self) -> None:
        payload = bytearray((4,))
        for anchor_id in range(1, 5):
            status = (
                STATUS_CALIBRATION_MISSING | STATUS_DS_FALLBACK
                if anchor_id == 1
                else STATUS_CALIBRATION_MISSING
            )
            payload.extend(
                struct.pack(
                    "<HBBHiih",
                    anchor_id,
                    0,
                    status,
                    0,
                    2000 + anchor_id * 100,
                    0,
                    -8000 - anchor_id,
                )
            )

        encoded = encode_frame(TYPE_RANGE, 0xFFFFFFFF, 9876, bytes(payload))
        self.assertEqual(len(encoded), 81)
        parser = TelemetryStreamParser()
        message = decode_frame(parser.feed(encoded)[0])

        self.assertIsInstance(message, RangeMessage)
        self.assertEqual(len(message.samples), 4)
        self.assertEqual(message.samples[3].anchor_id, 4)
        self.assertEqual(
            status_names(message.samples[0].status),
            ("DS_FALLBACK", "CAL_MISSING"),
        )

    def test_noise_bad_crc_then_valid_frame_recovers(self) -> None:
        payload = bytes((0,))
        bad = bytearray(encode_frame(TYPE_RANGE, 1, 10, payload))
        bad[-1] ^= 0x80
        good = encode_frame(TYPE_RANGE, 2, 20, payload)
        parser = TelemetryStreamParser()

        frames = parser.feed(b"boot log\r\n" + bad + b"noise" + good)

        self.assertEqual([frame.sequence for frame in frames], [2])
        self.assertEqual(parser.counters.crc_errors, 1)
        self.assertGreater(parser.counters.discarded_bytes, 0)

    def test_info_schema_two(self) -> None:
        payload = bytes((2, 0x05, 1, 1, 0, 0, 1, 8, 0, 0))
        payload += struct.pack("<Hi", 1, -12500)
        parser = TelemetryStreamParser()
        frame = parser.feed(encode_frame(TYPE_INFO, 3, 30, payload))[0]
        message = decode_frame(frame)

        self.assertIsInstance(message, InfoMessage)
        self.assertEqual(message.ranging_mode, 1)
        self.assertEqual(message.calibrated_mask, 0)
        self.assertEqual(message.active_offsets_um, ((1, -12500),))

    def test_stats(self) -> None:
        payload = struct.pack("<IIIIIIHH", 100, 95, 4, 1, 0, 0, 50, 95)
        parser = TelemetryStreamParser()
        frame = parser.feed(encode_frame(TYPE_STATS, 4, 40, payload))[0]
        message = decode_frame(frame)

        self.assertIsInstance(message, StatsMessage)
        self.assertEqual(message.response_ok_count, 95)
        self.assertEqual(message.cycle_hz, 50)
        self.assertEqual(message.operation_hz, 95)


if __name__ == "__main__":
    unittest.main()
