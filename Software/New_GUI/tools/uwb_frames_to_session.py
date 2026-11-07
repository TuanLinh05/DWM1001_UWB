"""Turn a UP4000 flight log (uwb_frames.bin) into a New_GUI log session.

python tools/uwb_frames_to_session.py <flight folder or uwb_frames.bin> [--anchors rig.json|rig.txt] [--out logs]

The UP4000 records every TAG frame; the copy received over WiFi by the ground
GUI can have gaps. This replays the complete record through the same engine
as live use (default filter: Robust Adaptive EKF) and writes the usual
uwb_fast_log.csv, rx_quality.jsonl and diag.jsonl, ready for the Replay page
and tools/diag_summary.py. Host time is the UP4000 CLOCK_MONOTONIC.
Anchors default to last_rig_config.json (the layout the GUI last used).
"""
from __future__ import annotations

import argparse
from pathlib import Path
import struct
import sys

NEW_GUI_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NEW_GUI_DIR))
sys.path.insert(0, str(NEW_GUI_DIR.parent / "UWB_UART_GUI"))

import telemetry_protocol as tp  # noqa: E402
from core_state import UwbStateEngine, load_anchor_layout_json, load_anchors_rig_txt  # noqa: E402
from serial_worker import FastCsvLogger  # noqa: E402

FRAME_FILE_MAGIC = b"UWBFRAMES1\n"   # UP4000/.../session.py FrameRecordWriter
_RECORD = struct.Struct("<qqH")


def read_frame_records(path: Path):
    with path.open("rb") as fh:
        if fh.read(len(FRAME_FILE_MAGIC)) != FRAME_FILE_MAGIC:
            raise SystemExit(f"{path} is not a UP4000 uwb_frames.bin")
        while True:
            head = fh.read(_RECORD.size)
            if len(head) < _RECORD.size:
                return
            realtime_ns, monotonic_ns, length = _RECORD.unpack(head)
            frame = fh.read(length)
            if len(frame) < length:
                return  # truncated by power loss
            yield realtime_ns, monotonic_ns, frame


def load_anchors(path: Path):
    return load_anchors_rig_txt(path) if path.suffix.lower() == ".txt" else load_anchor_layout_json(path)


def convert(frames_path: Path, out_root: Path, anchors_path: Path, filter_mode: str = "robust_ekf") -> Path:
    engine = UwbStateEngine(load_anchors(anchors_path))
    engine.set_position_filter_mode(filter_mode)
    flight = frames_path.parent.name
    logger = FastCsvLogger(out_root, f"UP4000_{flight}", engine=engine)
    parser = tp.TelemetryStreamParser()
    try:
        for _realtime_ns, monotonic_ns, blob in read_frame_records(frames_path):
            host_s = monotonic_ns / 1e9
            for frame in parser.feed(blob):
                try:
                    msg = tp.decode_frame(frame)
                except tp.ProtocolError:
                    continue
                engine.ingest_message(msg, host_s)
                if isinstance(msg, tp.RangeBurstMessage):
                    sol = engine.latest_solution
                    logger.log_burst(msg, pos_xyz=(sol.x, sol.y, sol.z) if sol else None,
                                     rms_m=sol.rms_m if sol else None, engine=engine)
                    logger.wait_for_space()
                elif isinstance(msg, tp.RxQualityMessage):
                    logger.log_rx_quality(msg)
                elif isinstance(msg, (tp.DiagSystemMessage, tp.DiagAnchorMessage, tp.StatsMessage)):
                    logger.log_diag(msg, host_s)
    finally:
        logger.stop(timeout_s=60.0)
    return logger.session_dir


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", type=Path, help="UP4000 flight folder or its uwb_frames.bin")
    ap.add_argument("--anchors", type=Path, default=NEW_GUI_DIR / "last_rig_config.json")
    ap.add_argument("--out", type=Path, default=NEW_GUI_DIR / "logs")
    ap.add_argument("--filter", choices=("robust_ekf", "legacy"), default="robust_ekf")
    args = ap.parse_args()
    frames = args.source / "uwb_frames.bin" if args.source.is_dir() else args.source
    session = convert(frames, args.out, args.anchors, args.filter)
    print(f"New_GUI session: {session}")


if __name__ == "__main__":
    main()
