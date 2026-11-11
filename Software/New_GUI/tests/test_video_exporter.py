"""Unit & integration tests for UWB Replay-to-MP4 Video Export Studio (`video_exporter_gui.py`)."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
if os.name == "nt":
    os.environ.setdefault("QT_QPA_FONTDIR", r"C:\Windows\Fonts")

NEW_GUI_DIR = Path(__file__).resolve().parents[1]
if str(NEW_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(NEW_GUI_DIR))
if str(NEW_GUI_DIR.parent / "UWB_UART_GUI") not in sys.path:
    sys.path.insert(0, str(NEW_GUI_DIR.parent / "UWB_UART_GUI"))

import cv2  # type: ignore  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

from core_state import (  # noqa: E402
    DEFAULT_PRESET_ROOM_66_36,
    InstabilitySegment,
    LoadedLogSession,
    ReplayAnchorSnap,
    ReplayFrame,
)
from main_gui import UwbFastGuiWindow  # noqa: E402
from video_exporter_gui import (  # noqa: E402
    ReplayVideoFrameRenderer,
    VideoExportStudioWindow,
    export_session_to_mp4,
    select_video_frame_indices,
)


def _make_synthetic_session(num_frames: int = 60, duration_s: float = 6.0) -> LoadedLogSession:
    frames: list[ReplayFrame] = []
    map_path = []
    for i in range(num_frames):
        t = (i / max(1, num_frames - 1)) * duration_s
        ang = 2.0 * 3.14159 * (i / max(1, num_frames))
        x = 2.5 + 1.2 * (i / max(1, num_frames))
        y = 1.8 + 0.6 * (1.0 if i % 2 == 0 else -1.0) * 0.2
        z = 0.8 + 0.1 * (i / max(1, num_frames))
        stab = "UNSTABLE" if 20 <= i <= 25 else ("WARN" if 10 <= i <= 15 else "STABLE")
        rms = 135.0 if stab == "UNSTABLE" else (78.0 if stab == "WARN" else 32.0)
        snaps = {
            aid: ReplayAnchorSnap(
                anchor_id=aid,
                raw_mm=3200.0 + aid * 50.0,
                corr_mm=3190.0 + aid * 50.0,
                filt_mm=3192.0 + aid * 50.0,
                res_mm=(-12.0 + aid * 3.0),
                rate_hz=245.0,
                sigma_mm=16.5,
                fp_dbm=-81.0,
                rx_dbm=-78.0,
                nlos_db=4.2,
                status_str="OK",
            )
            for aid in range(1, 9)
        }
        frames.append(
            ReplayFrame(
                frame_index=i,
                host_time_s=1000.0 + t,
                rel_time_s=t,
                cycle_seq=i + 1,
                x=x,
                y=y,
                z=z,
                raw_x=x,
                raw_y=y,
                raw_z=z,
                speed_mps=0.45,
                rms_mm=rms,
                max_res_mm=rms * 1.3,
                step_jump_mm=6.0,
                pdop=1.35,
                hdop=0.82,
                vdop=1.08,
                used_anchor_ids=(1, 2, 3, 4, 5, 6, 7, 8),
                excluded_anchor_ids=(),
                motion_state="MOVING",
                stability_state=stab,
                instability_reason="High RMS" if stab != "STABLE" else "",
                waypoint_label="",
                anchors=snaps,
            )
        )
        map_path.append((x, y, z, t, stab, rms))

    segs = [
        InstabilitySegment(
            segment_id=1,
            severity="UNSTABLE",
            start_frame_idx=20,
            peak_frame_idx=22,
            end_frame_idx=25,
            start_rel_s=frames[20].rel_time_s,
            end_rel_s=frames[25].rel_time_s,
            duration_s=frames[25].rel_time_s - frames[20].rel_time_s,
            xyz_m=(frames[22].x, frames[22].y, frames[22].z),
            max_rms_mm=135.0,
            max_jump_mm=24.0,
            min_anchors=8,
            used_anchors_str="A1..A8",
            cause_summary="High RMS 135mm",
        )
    ]
    return LoadedLogSession(
        csv_path=NEW_GUI_DIR / "logs" / "synthetic_test" / "uwb_fast_log.csv",
        anchors=list(DEFAULT_PRESET_ROOM_66_36),
        frames=frames,
        map_path=map_path,
        waypoints=[],
        unstable_segments=segs,
        duration_s=duration_s,
        path_length_2d_m=12.4,
        path_length_3d_m=14.8,
        stable_pct=80.0,
        warn_pct=10.0,
        unstable_pct=10.0,
        total_csv_rows=num_frames * 8,
    )


class VideoExporterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_select_video_frame_indices_respects_speed_and_fps(self) -> None:
        session = _make_synthetic_session(num_frames=100, duration_s=10.0)
        # 10s log at 2x speed and 10 FPS -> ~5s video -> ~51 frames
        idx_list = select_video_frame_indices(session.frames, 0.0, 10.0, playback_speed=2.0, fps=10.0)
        self.assertGreaterEqual(len(idx_list), 50)
        self.assertLessEqual(len(idx_list), 52)
        self.assertEqual(idx_list[0], 0)
        self.assertEqual(idx_list[-1], 99)

    def test_renderer_produces_valid_qimage_in_all_modes(self) -> None:
        session = _make_synthetic_session(num_frames=30, duration_s=3.0)
        renderer = ReplayVideoFrameRenderer()
        renderer.set_session(session)
        for layout in ("DASHBOARD", "MAP_ONLY"):
            for trail in ("PROGRESSIVE_GHOST", "PROGRESSIVE_ONLY", "FULL"):
                for dark in (False, True):
                    renderer.layout_mode = layout
                    renderer.canvas.trail_mode = trail
                    renderer.is_dark = dark
                    qimg = renderer.render_qimage(15, 1280, 720)
                    self.assertEqual(qimg.width(), 1280)
                    self.assertEqual(qimg.height(), 720)

    def test_export_session_to_mp4_creates_playable_video(self) -> None:
        session = _make_synthetic_session(num_frames=40, duration_s=4.0)
        with tempfile.TemporaryDirectory() as tmp:
            out_mp4 = Path(tmp) / "test_replay.mp4"
            res = export_session_to_mp4(
                session,
                out_mp4,
                width=640,
                height=360,
                fps=15.0,
                playback_speed=2.0,
                layout_mode="DASHBOARD",
                trail_mode="PROGRESSIVE_GHOST",
            )
            self.assertTrue(out_mp4.is_file())
            self.assertGreater(out_mp4.stat().st_size, 4096)
            self.assertGreaterEqual(res["frames_rendered"], 25)

            cap = cv2.VideoCapture(str(out_mp4))
            try:
                self.assertTrue(cap.isOpened())
                ok, frame = cap.read()
                self.assertTrue(ok)
                self.assertEqual(frame.shape[1], 640)
                self.assertEqual(frame.shape[0], 360)
            finally:
                cap.release()

    def test_studio_window_and_main_gui_export_button(self) -> None:
        studio = VideoExportStudioWindow(auto_load=False)
        try:
            self.assertIsNotNone(studio.preview_widget)
            self.assertIsNotNone(studio.btn_export)
        finally:
            studio.close()

        win = UwbFastGuiWindow(start_demo=False)
        try:
            self.assertTrue(hasattr(win, "btn_export_mp4"))
            self.assertEqual(win.btn_export_mp4.text(), "Export MP4 Video...")
        finally:
            win.close()


if __name__ == "__main__":
    unittest.main()
