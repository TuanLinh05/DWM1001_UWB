"""RANGE_BURST (0x18) and SET_BURST (0x10): decoder, recorder and GUI path."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import re
import shutil
import struct
import sys
import time
import tkinter as tk
import unittest
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from session_recorder import BURST_COLUMNS, SessionRecorder  # noqa: E402
from telemetry_protocol import (  # noqa: E402
    BURST_FLAG_ANCHOR_LATE,
    BURST_FLAG_CAL_OK,
    BURST_FLAG_RADIO_OK,
    BURST_FLAG_VALID,
    BURST_MODE_NONE,
    CMD_SET_BURST,
    INFO_FLAG_BURST,
    INT16_MIN,
    MEAS_FLAG_CAL_OK,
    MEAS_FLAG_FILTER_OK,
    MEAS_FLAG_RADIO_OK,
    TYPE_RANGE_BURST,
    BurstAnchor,
    BurstSettings,
    CmdAckMessage,
    InfoMessage,
    ProtocolError,
    RangeBurstMessage,
    TelemetryStreamParser,
    decode_frame,
    encode_frame,
    encode_range_burst_payload,
)

INCLUDE = Path(__file__).resolve().parents[3] / "Firmware" / "common" / "include"


def _decode(payload: bytes, sequence: int = 1, time_ms: int = 0):
    frame = encode_frame(TYPE_RANGE_BURST, sequence, time_ms, payload)
    return decode_frame(TelemetryStreamParser().feed(frame)[0])


def make_burst(cycle_seq: int, meas_seq_first: int, boot_id: int = 0x1234,
               ranges: dict[int, int | None] | None = None) -> RangeBurstMessage:
    """Anchors 1..8; None = no RESP (timeout), A3 calibrated, A6 late."""
    ranges = ranges if ranges is not None else {
        anchor: 1000 * anchor + 7 for anchor in range(1, 9)}
    anchors = []
    for anchor_id, range_mm in sorted(ranges.items()):
        if range_mm is None:
            anchors.append(BurstAnchor(anchor_id, 0x01, BURST_MODE_NONE, None,
                                       INT16_MIN, INT16_MIN, INT16_MIN))
            continue
        flags = BURST_FLAG_RADIO_OK | BURST_FLAG_VALID
        if anchor_id == 3:
            flags |= BURST_FLAG_CAL_OK
        if anchor_id == 6:
            flags |= BURST_FLAG_ANCHOR_LATE | 2          # SS fallback
        anchors.append(BurstAnchor(anchor_id, 0x10 if anchor_id == 6 else 0, flags, range_mm,
                                   -8000 - 50 * anchor_id, -7500 - 50 * anchor_id,
                                   -2000 + 25 * anchor_id))
    return RangeBurstMessage(cycle_seq, 5000 + cycle_seq, boot_id, cycle_seq, meas_seq_first,
                             1_000_000 + 4500 * cycle_seq, 4480, tuple(anchors))


class RangeBurstProtocolTests(unittest.TestCase):
    def test_round_trip_through_a_frame(self) -> None:
        message = make_burst(7, 100, ranges={1: 1234, 2: None, 6: 65534})
        decoded = _decode(encode_range_burst_payload(message), sequence=7,
                          time_ms=message.time_ms)
        self.assertEqual(decoded, message)
        self.assertEqual(len(encode_range_burst_payload(message)), 22 + 8 * 3)
        a1, a2, a6 = decoded.anchors
        self.assertEqual((a1.mode_name, a2.mode_name, a6.mode_name), ("DS", "NONE", "SS_FALLBACK"))
        self.assertFalse(a2.radio_ok)
        self.assertIsNone(a2.range_mm)

    def test_quality_codes_are_rounded_and_clamped(self) -> None:
        anchor = BurstAnchor(1, 0, BURST_FLAG_RADIO_OK, 5000, -8212, -7650, -1837)
        message = RangeBurstMessage(1, 0, 1, 1, 1, 0, 0, (anchor,))
        decoded = _decode(encode_range_burst_payload(message)).anchors[0]
        self.assertEqual((decoded.fp_cdbm, decoded.rx_cdbm, decoded.ci_ppm_x100),
                         (-8200, -7640, -1825))
        extreme = BurstAnchor(2, 0, BURST_FLAG_RADIO_OK, 5000, -20000, 10000, 9000)
        decoded = _decode(encode_range_burst_payload(
            RangeBurstMessage(1, 0, 1, 1, 1, 0, 0, (extreme,)))).anchors[0]
        self.assertEqual((decoded.fp_cdbm, decoded.ci_ppm_x100), (-12700, 3175))
        self.assertEqual(decoded.rx_cdbm, -12700 + 2540)

    def test_rejects_bad_length_and_schema(self) -> None:
        payload = encode_range_burst_payload(make_burst(1, 1, ranges={1: 1000}))
        with self.assertRaises(ProtocolError):
            _decode(payload[:-1])
        with self.assertRaises(ProtocolError):
            _decode(payload + bytes(8))
        with self.assertRaises(ProtocolError):
            _decode(bytes((2,)) + payload[1:])

    def test_records_follow_the_tag_numbering(self) -> None:
        message = make_burst(9, 500, ranges={1: 1111, 2: None, 3: 3333, 6: 6666})
        records = message.to_range_meas({3: 45_600})
        self.assertEqual([(r.anchor_id, r.meas_seq) for r in records],
                         [(1, 500), (3, 501), (6, 502)])
        a1, a3, a6 = records
        self.assertEqual((a1.raw_mm, a1.corrected_mm, a1.filtered_mm), (1111, 1111, 1111))
        self.assertEqual(a1.flags, MEAS_FLAG_RADIO_OK | MEAS_FLAG_FILTER_OK)
        self.assertEqual((a3.raw_mm, a3.corrected_mm), (3379, 3333))    # raw = corr + bias
        self.assertEqual(a3.flags, MEAS_FLAG_RADIO_OK | MEAS_FLAG_CAL_OK | MEAS_FLAG_FILTER_OK)
        self.assertEqual((a6.mode_name, a6.status), ("SS_FALLBACK", 0x10))
        self.assertTrue(all(r.meas_time_us == message.t_us for r in records))
        self.assertEqual((a1.anchor_fp_cdbm, a1.slot_us), (INT16_MIN, 0))
        self.assertAlmostEqual(a1.nlos_indicator_db, 5.0)
        # Without the INFO offsets the calibrated value stands in for raw.
        self.assertEqual(message.to_range_meas()[1].raw_mm, 3333)

    def test_burst_settings(self) -> None:
        defaults = BurstSettings(1, 800, 350, 450, 500, 0)
        self.assertEqual(BurstSettings.decode(defaults.encode()), defaults)
        self.assertEqual(len(defaults.encode()), 11)
        # Eight anchors at the firmware defaults: ~4.6 ms, i.e. > 200 Hz.
        self.assertLess(defaults.cycle_estimate_us(), 5000.0)
        self.assertGreater(1e6 / defaults.cycle_estimate_us(), 200.0)
        with self.assertRaises(ProtocolError):
            BurstSettings.decode(bytes(10))

    def test_constants_match_firmware_headers(self) -> None:
        if not INCLUDE.is_dir():
            self.skipTest("firmware sources are not next to the GUI")

        def define(header: str, name: str) -> int:
            text = (INCLUDE / header).read_text(encoding="utf-8")
            match = re.search(rf"^#define\s+{name}\s+(0x[0-9A-Fa-f]+|\d+)U?\b", text, re.MULTILINE)
            self.assertIsNotNone(match, f"{name} not found in {header}")
            return int(match.group(1), 0)

        self.assertEqual(define("telemetry_frame.h", "TELEM_TYPE_RANGE_BURST"), TYPE_RANGE_BURST)
        self.assertEqual(define("telemetry.h", "TELEM_INFO_FLAG_BURST"), INFO_FLAG_BURST)
        self.assertEqual(define("uwb_cmd.h", "UWB_CMD_SET_BURST"), CMD_SET_BURST)
        self.assertEqual(define("tag_ranging.h", "TAG_BURST_REC_MODE_NONE"), BURST_MODE_NONE)
        self.assertEqual(define("tag_ranging.h", "TAG_BURST_REC_CAL_OK"), BURST_FLAG_CAL_OK)
        self.assertEqual(define("tag_ranging.h", "TAG_BURST_REC_VALID"), BURST_FLAG_VALID)
        self.assertEqual(define("tag_ranging.h", "TAG_BURST_REC_ANCHOR_LATE"),
                         BURST_FLAG_ANCHOR_LATE)
        self.assertEqual(define("tag_ranging.h", "TAG_BURST_REC_RADIO_OK"), BURST_FLAG_RADIO_OK)
        firmware_defaults = BurstSettings(
            1, define("tag_ranging.h", "UWB_BURST_BASE_UUS"),
            define("tag_ranging.h", "UWB_BURST_SLOT_UUS"),
            define("tag_ranging.h", "UWB_BURST_FINAL_MARGIN_UUS"),
            define("tag_ranging.h", "UWB_BURST_GAP_US"),
            define("tag_ranging.h", "UWB_BURST_PERIOD_US"))
        self.assertGreater(1e6 / firmware_defaults.cycle_estimate_us(), 200.0)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


class RangeBurstRecorderTests(unittest.TestCase):
    def test_burst_csv_and_meas_rows(self) -> None:
        temporary = Path(__file__).resolve().parents[1] / f".burst-test-{uuid.uuid4().hex}"
        temporary.mkdir()
        self.addCleanup(shutil.rmtree, temporary, True)
        recorder = SessionRecorder(temporary, "COM14", 460800)
        try:
            meas_seq = 1
            for cycle in (1, 2, 4):                    # cycle 3 lost on the UART
                ranges = {anchor: 1000 * anchor + cycle for anchor in range(1, 9)}
                ranges[8] = None                       # A8 never answers
                message = make_burst(cycle, meas_seq, ranges=ranges)
                records = message.to_range_meas({3: 50_000})
                recorder.record_burst(message, records)
                meas_seq += len(records) * (2 if cycle == 2 else 1)
        finally:
            recorder.stop()
        self.assertIsNone(recorder.error)

        session = recorder.session_directory
        rows = read_csv(session / "burst.csv")
        self.assertEqual(len(rows), 3)
        self.assertEqual(list(rows[0]), list(BURST_COLUMNS))
        first = rows[0]
        self.assertEqual((first["cycle_seq"], first["polled"], first["ranged"]), ("1", "8", "7"))
        self.assertEqual((first["a1_mode"], first["a1_range_mm"], first["a1_valid"]),
                         ("DS", "1001", "1"))
        self.assertEqual((first["a3_cal"], first["a6_late"], first["a6_mode"]),
                         ("1", "1", "SS_FALLBACK"))
        self.assertEqual((first["a8_mode"], first["a8_range_mm"], first["a8_status_hex"]),
                         ("NONE", "", "0x01"))
        self.assertEqual((first["a1_fp_dbm"], first["a1_nlos_db"], first["a1_ci_ppm"]),
                         ("-80.5", "5.0", "-19.75"))

        meas = read_csv(session / "meas.csv")
        self.assertEqual(len(meas), 21)
        a3 = next(row for row in meas if row["anchor_id"] == "3")
        self.assertEqual((a3["raw_mm"], a3["corrected_mm"]), ("3051", "3001"))
        self.assertEqual(a3["anchor_fp_cdbm"], "")

        metadata = json.loads((session / "session.json").read_text(encoding="utf-8"))
        self.assertEqual((metadata["burst_cycles"], metadata["meas_records"]), (3, 21))
        self.assertEqual(metadata["burst_cycles_expected"], 4)
        self.assertEqual(metadata["burst_delivered_pct"], 75.0)
        self.assertEqual(metadata["meas_delivered_pct"], 75.0)
        summary = {row["anchor_id"]: row for row in read_csv(session / "summary.csv")}
        self.assertEqual(summary["1"]["meas_count"], "3")


class RangeBurstGuiTests(unittest.TestCase):
    def test_burst_cycles_feed_the_range_meas_tab(self) -> None:
        from uwb_uart_gui import UwbGui

        root = tk.Tk()
        root.withdraw()
        try:
            app = UwbGui(root)
            app.events.put(("message", InfoMessage(
                0, 0, 2, INFO_FLAG_BURST, 1, 8, 0x04, 1, 3, 8, 0, 0,
                tuple((anchor, 20_000 if anchor == 3 else 0) for anchor in range(1, 9)))))
            meas_seq = 1
            for cycle in range(1, 41):
                message = make_burst(cycle, meas_seq)
                meas_seq += 8
                app.events.put(("message", message))
            app.events.put(("message", CmdAckMessage(
                3, 0, CMD_SET_BURST, 0, BurstSettings(1, 700, 300, 400, 250, 0).encode())))
            app._poll_events_once()

            self.assertIn("Burst DS-TWR", app.info_var.get())
            tracker = app.meas_panel.tracker
            self.assertEqual((tracker.total, tracker.lost), (320, 0))
            self.assertEqual(sorted(tracker.latest), list(range(1, 9)))
            self.assertEqual(tracker.latest[3].raw_mm, 3007 + 20)
            app.meas_panel.refresh(time.monotonic(), draw=True)
            values = app.meas_panel.tree.item(app.meas_panel.rows[6], "values")
            self.assertEqual(values[2], "SS_FALLBACK")
            log = app.log_text.get("1.0", tk.END)
            self.assertIn("BURST mode=burst base=700 slot=300", log)
        finally:
            for callback in root.tk.call("after", "info"):
                root.after_cancel(callback)
            root.destroy()


if __name__ == "__main__":
    unittest.main()
