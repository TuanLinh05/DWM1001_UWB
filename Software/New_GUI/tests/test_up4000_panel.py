"""UP4000 tab: decoding helpers, UDP listener, QGC-style Flight HUD, EKF2 gauges,
and PX4 .ulg flight log replay verification."""
from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import sys
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
NEW_GUI_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NEW_GUI_DIR))
sys.path.insert(0, str(NEW_GUI_DIR.parent / "UWB_UART_GUI"))

from PyQt6.QtWidgets import QApplication  # noqa: E402
from design_tokens import DARK_QSS, LIGHT_QSS  # noqa: E402
from up4000_panel import (  # noqa: E402
    FlightHudWidget,
    Up4000Panel,
    Up4000StatusListener,
    ekf_flags,
    load_ulog_session,
    px4_mode_name,
    quat_to_euler_deg,
)

_APP = QApplication.instance() or QApplication([])

ULOG_07_05_39 = Path(r"c:\Users\vutua\Downloads\07_05_39.ulg")
ULOG_03_31_22 = Path(r"c:\Users\vutua\Downloads\03_31_22.ulg")


def sample_status(seq: int = 7) -> dict:
    """Shape of what up4000_status sends (values from the 07/11 drone runs)."""
    return {
        "kind": "up4000_status", "version": 1, "seq": seq, "time_unix": time.time(),
        "host": {"hostname": "dronepif", "ip": ["wlx30169df9a262 192.168.10.23/24"], "uptime_s": 13000.0,
                 "cpu_percent": 47.0, "cpu_count": 2, "load": [2.04, 3.01, 3.45], "cpu_temp_c": 61.5,
                 "temperatures_c": {"x86_pkg_temp": 61.5}, "memory": {"total_mb": 1842, "available_mb": 686,
                                                                      "used_percent": 62.8},
                 "disk": {"free_gb": 16.0, "total_gb": 26.0, "used_percent": 35.0},
                 "wifi": [{"interface": "wlx30169df9a262", "link_quality": 58.0, "signal_dbm": -52.0,
                           "noise_dbm": -256.0}],
                 "processes": {"uwb_relay": {"pid": 71616, "cpu_percent": 60.6, "rss_mb": 58.6},
                               "px4_logger": {"pid": 71618, "cpu_percent": 23.8, "rss_mb": 73.7}}},
        "session": {"folder": "/home/dronepif/flight_logs/hwtest10",
                    "files": {"px4.tlog": 4647241, "px4_stream_1.ulg": 2617230, "uwb_frames.bin": 1475950}},
        "uwb_relay": {"tag_port": "/dev/uwb_tag", "ground": "192.168.10.7:15555",
                      "counts": {"tag_bytes": 1476000, "udp_datagrams": 4747},
                      "parsed": {"frame_0x18": 18279}, "parser": {"crc_errors": 17}},
        "px4_logger": {"px4_port": "/dev/pixhawk", "ground": "192.168.10.7:14550", "ulog_part": 1,
                       "ulog": {"chunks": 10512, "bytes": 2617230, "dropped_chunks": 4},
                       "counts": {"messages": 33352, "bad_data": 1, "timesync_samples": 114}},
        "px4": {"connected": True, "port": "/dev/pixhawk",
                "rates_hz": {"ODOMETRY": 49.4, "ESTIMATOR_STATUS": 9.9},
                "messages": {
                    "HEARTBEAT": {"age_s": 0.3, "fields": {"base_mode": 128 | 1, "custom_mode": 3 << 16}},
                    "SYS_STATUS": {"age_s": 0.4, "fields": {"voltage_battery": 16180, "battery_remaining": 87}},
                    "EXTENDED_SYS_STATE": {"age_s": 0.2, "fields": {"landed_state": 1}},
                    "ODOMETRY": {"age_s": 0.02, "fields": {"x": 1.234, "y": -0.5, "z": -0.8, "frame_id": 1,
                                                          "q": [0.7071068, 0.0, 0.0, 0.7071068],
                                                          "vx": 0.1, "vy": 0.0, "vz": 0.0,
                                                          "pose_covariance": [None] * 21}},
                    "ESTIMATOR_STATUS": {"age_s": 0.05, "fields": {"flags": 1 | 2 | 4 | 8 | 1024,
                                                                  "vel_ratio": 0.2, "pos_horiz_ratio": 0.7,
                                                                  "pos_vert_ratio": 1.3, "mag_ratio": 0.1,
                                                                  "hagl_ratio": None, "tas_ratio": 0.0,
                                                                  "pos_horiz_accuracy": 0.05,
                                                                  "pos_vert_accuracy": 0.08}},
                    "VIBRATION": {"age_s": 0.1, "fields": {"vibration_x": 2.1, "vibration_y": 2.4,
                                                          "vibration_z": 3.0, "clipping_0": 0,
                                                          "clipping_1": 0, "clipping_2": 5}},
                },
                "statustext": [{"time_unix": time.time(), "severity": 4, "text": "Preflight Fail: Compass"}]},
    }


def pump_until(cond, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        _APP.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    return False


class HelperTests(unittest.TestCase):
    def test_px4_mode_names(self):
        self.assertEqual(px4_mode_name(3 << 16), "POSCTL")
        self.assertEqual(px4_mode_name((4 << 16) | (4 << 24)), "AUTO.MISSION")
        self.assertEqual(px4_mode_name(6 << 16), "OFFBOARD")
        self.assertEqual(px4_mode_name(7 << 16), "STABILIZED")

    def test_quaternion_to_euler(self):
        self.assertEqual(tuple(round(v, 3) for v in quat_to_euler_deg([1, 0, 0, 0])), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(quat_to_euler_deg([0.7071068, 0, 0, 0.7071068])[2], 90.0, places=3)
        self.assertIsNone(quat_to_euler_deg([None, 0, 0, 0]))
        self.assertIsNone(quat_to_euler_deg([float("nan"), 0, 0, 0]))

    def test_ekf_flags(self):
        flags = {name: (on, bad) for name, on, bad in ekf_flags(1 | 1024)}
        self.assertEqual(flags["attitude"], (True, False))
        self.assertEqual(flags["GPS glitch"], (True, True))
        self.assertEqual(flags["vel horiz"], (False, False))

    def test_tree_widget_qss_defined_for_light_and_dark_themes(self):
        self.assertIn("QTreeWidget, QTreeView", LIGHT_QSS)
        self.assertIn("QTreeWidget::item:selected", LIGHT_QSS)
        self.assertIn("QTreeWidget, QTreeView", DARK_QSS)


class ListenerAndPanelTests(unittest.TestCase):
    def test_selective_message_loss_and_total_silence_show_stale_values(self):
        listener = Up4000StatusListener(port=0)
        panel = Up4000Panel(listener)
        try:
            with patch("up4000_panel.time.monotonic", return_value=10.0):
                status = sample_status()
                listener.handle_datagram(json.dumps(status).encode(), "drone")
            with patch("up4000_panel.time.monotonic", return_value=12.5):
                # Other traffic stays online; only HEARTBEAT is refreshed.
                packet = {"kind": "up4000_status", "version": 2, "seq": 8,
                          "part": "px4.messages", "data": {"HEARTBEAT": status["px4"]["messages"]["HEARTBEAT"]}}
                listener.handle_datagram(json.dumps(packet).encode(), "drone")
                panel.refresh(force=True)
                self.assertEqual(panel.card_link.value.text(), "ONLINE")
                self.assertEqual(panel.card_vehicle.value.text(), "ARMED")
                self.assertIn("STALE PX4", panel.lbl_state.text())
                self.assertIn("ODOMETRY (", panel.lbl_state.text())
                self.assertNotIn("x 1.234", panel.lbl_state.text())
                self.assertIn("STALE: host", panel.card_link.sub.text())
            with patch("up4000_panel.time.monotonic", return_value=15.0):
                # Refreshing without a new packet must still expire HEARTBEAT.
                panel.refresh()
                self.assertEqual(panel.card_vehicle.value.text(), "STALE HEARTBEAT")
                self.assertTrue(panel.card_link.value.text().startswith("STALE"))
        finally:
            listener.close()

    def test_source_age_and_partially_missing_message_fields(self):
        listener = Up4000StatusListener(port=0)
        try:
            with patch("up4000_panel.time.monotonic", return_value=10.0):
                status = sample_status()
                status["px4"]["messages"]["VIBRATION"]["age_s"] = 8.0
                listener.handle_datagram(json.dumps(status).encode(), "drone")
                self.assertGreaterEqual(listener.message_age_s("VIBRATION"), 8)
            with patch("up4000_panel.time.monotonic", return_value=13.0):
                listener.handle_datagram(json.dumps({"kind": "up4000_status", "version": 2,
                    "part": "px4.messages.ODOMETRY", "data": {"age_s": 0.01}}).encode(), "drone")
                self.assertGreaterEqual(listener.message_age_s("ODOMETRY"), 3)
        finally:
            listener.close()

    def test_long_atomic_value_reassembles_out_of_order_and_loss_stays_stale(self):
        sys.path.insert(0, str(NEW_GUI_DIR.parents[1] / "UP7000" / "ros2_ws" / "src" / "uwb_px4_logger"))
        from uwb_px4_logger.status_reporter import StatusSender
        listener = Up4000StatusListener(port=0)
        sender = StatusSender(max_part_bytes=400)
        expected = {"statustext": [{"text": "cảm biến 📡" * 90}] * 4}
        packets = sender.datagrams({"px4": expected})
        try:
            self.assertTrue(all(len(p) <= 400 for p in packets))
            for p in reversed(packets[1:]):
                listener.handle_datagram(p, "drone")
            self.assertIsNone(listener.latest)
            listener.handle_datagram(packets[0], "drone")
            self.assertEqual(listener.state["px4"], expected)
            self.assertEqual(listener.bad, 0)
            self.assertFalse(listener._fragments)
        finally:
            sender.close()
            listener.close()

    def test_listener_receives_udp_status(self):
        listener = Up4000StatusListener(port=0)
        try:
            self.assertTrue(listener.bound)
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            tx.sendto(b"not json", ("127.0.0.1", listener.port))
            tx.sendto(json.dumps({"kind": "something_else"}).encode(), ("127.0.0.1", listener.port))
            tx.sendto(json.dumps(sample_status()).encode(), ("127.0.0.1", listener.port))
            tx.close()
            self.assertTrue(pump_until(lambda: listener.latest is not None))
            self.assertEqual(listener.latest["seq"], 7)
            self.assertIn("127.0.0.1", listener.sender)
            self.assertTrue(pump_until(lambda: listener.bad == 2))
        finally:
            listener.close()

    def test_panel_shows_health_ekf_logger_and_all_values(self):
        listener = Up4000StatusListener(port=0)
        panel = Up4000Panel(listener)
        try:
            panel.refresh(force=True)
            self.assertEqual(panel.card_link.value.text(), "WAITING")
            listener.handle_datagram(json.dumps(sample_status()).encode(), "192.168.10.23")
            panel.refresh(force=True)
            self.assertEqual(panel.card_link.value.text(), "ONLINE")
            self.assertIn("47%", panel.card_cpu.value.text())
            self.assertIn("62°C", panel.card_cpu.value.text())
            self.assertEqual(panel.card_wifi.value.text(), "-52 dBm")
            self.assertNotIn("power save", panel.card_wifi.sub.text())
            self.assertEqual(panel.card_vehicle.value.text(), "ARMED")
            self.assertIn("POSCTL", panel.card_vehicle.sub.text())
            self.assertIn("16.18 V", panel.card_vehicle.sub.text())
            state = panel.lbl_state.text()
            self.assertIn("x 1.234", state)
            self.assertIn("yaw +90.0", state)
            ekf = panel.lbl_ekf.text()
            self.assertIn("#d93025", ekf)          # pos V ratio 1.3 > 1: rejected (red)
            self.assertIn("#d98200", ekf)          # pos H ratio 0.7: close (orange)
            self.assertAlmostEqual(panel.ratio_bars["pos_vert_ratio"].value, 1.3)
            self.assertAlmostEqual(panel.ratio_bars["pos_horiz_ratio"].value, 0.7)
            self.assertIsNone(panel.ratio_bars["hagl_ratio"].value)
            self.assertIn("GPS glitch", panel.lbl_flags.text())
            self.assertIn("Preflight Fail: Compass", panel.lbl_text.text())
            self.assertIn("18,279", panel.lbl_logger.text())
            self.assertIn("lost 4", panel.lbl_logger.text())
            self.assertIn("uwb_relay 61%", panel.lbl_logger.text())
            self.assertIn("px4_stream_1.ulg: 2.62 MB", panel.lbl_files.text())
            # HUD and quick telemetry cells are updated
            self.assertTrue(panel.hud.has_attitude)
            self.assertTrue(panel.hud.has_position)
            self.assertAlmostEqual(panel.hud.yaw_deg, 90.0, places=2)
            self.assertIn("POSCTL", panel.cell_mode.lbl_val.text())
            # Every field is in the tree, e.g. host > memory > used_percent.
            root = panel.tree.invisibleRootItem()
            top = {root.child(i).text(0): root.child(i) for i in range(root.childCount())}
            host = {top["host"].child(i).text(0): top["host"].child(i) for i in range(top["host"].childCount())}
            memory = {host["memory"].child(i).text(0): host["memory"].child(i).text(1)
                      for i in range(host["memory"].childCount())}
            self.assertEqual(memory["used_percent"], "62.8")
            # A new packet updates the tree in place (expanded items keep their state).
            top["host"].setExpanded(True)
            changed = sample_status(seq=8)
            changed["host"]["memory"]["used_percent"] = 70.0
            listener.handle_datagram(json.dumps(changed).encode(), "192.168.10.23")
            panel.refresh(force=True)
            self.assertTrue(top["host"].isExpanded())
            self.assertEqual(host["memory"].child(2).text(1), "70")
            # rtl8821cu: positive /proc level is not dBm; iw adds link details and power save.
            changed["seq"] = 9
            changed["host"]["wifi"] = [{"interface": "wlx30169df9a262", "link_quality": 95.0,
                                        "signal_dbm": None, "signal_level": 52.0, "ssid": "PIF_CLUB_5G",
                                        "freq_mhz": 5500.0, "tx_mbit": 434.0, "power_save": True}]
            listener.handle_datagram(json.dumps(changed).encode(), "192.168.10.23")
            panel.refresh(force=True)
            self.assertEqual(panel.card_wifi.value.text(), "level 52")
            self.assertIn("PIF_CLUB_5G · 5.50 GHz · tx 434 Mbit/s", panel.card_wifi.sub.text())
            self.assertIn("power save ON", panel.card_wifi.sub.text())
        finally:
            listener.close()

    def test_tree_search_filter_and_hud_rendering(self):
        listener = Up4000StatusListener(port=0)
        panel = Up4000Panel(listener)
        panel.resize(1360, 820)
        try:
            listener.handle_datagram(json.dumps(sample_status()).encode(), "192.168.10.23")
            panel.refresh(force=True)
            root = panel.tree.invisibleRootItem()
            top = {root.child(i).text(0): root.child(i) for i in range(root.childCount())}

            panel.tree_filter.setText("ODOMETRY")
            _APP.processEvents()
            self.assertFalse(top["px4"].isHidden())
            self.assertTrue(top["uwb_relay"].isHidden())

            panel.tree_filter.clear()
            _APP.processEvents()
            self.assertFalse(top["uwb_relay"].isHidden())

            # Test offscreen rendering in both Light and Dark modes
            pix_light = panel.grab()
            self.assertFalse(pix_light.isNull())
            panel.set_dark_mode(True)
            pix_dark = panel.grab()
            self.assertFalse(pix_dark.isNull())
        finally:
            listener.close()

    def test_parts_from_the_up4000_sender_merge_into_the_panel(self):
        sys.path.insert(0, str(NEW_GUI_DIR.parents[1] / "UP7000" / "ros2_ws" / "src" / "uwb_px4_logger"))
        from uwb_px4_logger.status_reporter import StatusSender

        status = sample_status()
        sections = {k: status[k] for k in ("host", "session", "uwb_relay", "px4_logger", "px4")}
        sections["px4"]["message_types"] = sorted(sections["px4"]["messages"])
        sender = StatusSender(max_part_bytes=400)  # force many parts
        datagrams = sender.datagrams(sections)
        sender.close()
        self.assertGreater(len(datagrams), 6)
        listener = Up4000StatusListener(port=0)
        panel = Up4000Panel(listener)
        try:
            for d in datagrams:
                listener.handle_datagram(d, "192.168.10.23")
            panel.refresh(force=True)
            self.assertEqual(panel.card_vehicle.value.text(), "ARMED")
            self.assertIn("x 1.234", panel.lbl_state.text())
            self.assertIn("18,279", panel.lbl_logger.text())
            self.assertEqual(listener.latest["px4"]["messages"]["VIBRATION"]["fields"]["clipping_2"], 5)
            # Next cycle: PX4 stopped sending VIBRATION -> pruned, not shown as current.
            sections["px4"]["messages"].pop("VIBRATION")
            sections["px4"]["message_types"] = sorted(sections["px4"]["messages"])
            sender = StatusSender(max_part_bytes=400)
            for d in sender.datagrams({"px4": sections["px4"]}):
                listener.handle_datagram(d, "192.168.10.23")
            sender.close()
            self.assertNotIn("VIBRATION", listener.latest["px4"]["messages"])
            panel.refresh(force=True)
            self.assertIn("vibration --", panel.lbl_ekf.text())
        finally:
            listener.close()

    def test_main_window_has_up4000_page(self):
        from main_gui import UwbFastGuiWindow

        # Use an ephemeral port: live diagnostics may already own UDP 15557.
        with patch("main_gui.Up4000StatusListener",
                   side_effect=lambda parent: Up4000StatusListener(port=0, parent=parent)):
            win = UwbFastGuiWindow(start_demo=False)
        try:
            self.assertIs(win.pages.widget(4), win.up4000_panel)
            win.pages.setCurrentIndex(4)
            win.up4000_listener.handle_datagram(json.dumps(sample_status()).encode(), "192.168.10.23")
            win.up4000_panel._last_refresh = 0.0  # refresh() is throttled to 4 Hz
            win._on_ui_refresh_tick()
            _APP.processEvents()
            self.assertEqual(win.up4000_panel.card_vehicle.value.text(), "ARMED")
            labels = [b.text() for b in win.nav_group.buttons()]
            self.assertIn("5. UP7000 Onboard", labels)
        finally:
            win.close()

    def test_hud_geometry_locked_and_secondary_sender_ignored(self):
        listener = Up4000StatusListener(port=0)
        panel = Up4000Panel(listener)
        panel.resize(1280, 720)
        panel.show()
        _APP.processEvents()
        try:
            initial_hud_size = panel.hud.size()
            # 1. Primary UP7000 ('thoa') sends active telemetry with 2-line WiFi info
            st_thoa = sample_status(seq=100)
            st_thoa["host"]["hostname"] = "thoa"
            st_thoa["host"]["wifi"] = [{
                "interface": "wlx30169df9a262", "ssid": "PIF_CLUB_5G",
                "freq_mhz": 5500.0, "tx_mbit": 351.0, "link_quality": 70.0, "signal_dbm": -30.0,
            }]
            listener.handle_datagram(
                json.dumps({"kind": "up4000_status", "version": 2, "seq": 100,
                            "time_unix": time.time(), "part": "host", "data": st_thoa["host"]}).encode(),
                "100.123.216.14",
            )
            listener.handle_datagram(
                json.dumps({"kind": "up4000_status", "version": 2, "seq": 100,
                            "time_unix": time.time(), "part": "px4", "data": st_thoa["px4"]}).encode(),
                "100.123.216.14",
            )
            panel.refresh(force=True)
            _APP.processEvents()
            self.assertEqual(panel.hud.size(), initial_hud_size)
            self.assertIn("thoa", panel.card_cpu.sub.text())

            # 2. Idle UP4000 ('dronepif') broadcasts empty host/uwb_relay on the same network
            listener.handle_datagram(
                json.dumps({"kind": "up4000_status", "version": 2, "seq": 50,
                            "time_unix": time.time(), "part": "host",
                            "data": {"hostname": "dronepif", "cpu_percent": 5.0, "wifi": [{}]}}).encode(),
                "192.168.10.23",
            )
            panel.refresh(force=True)
            _APP.processEvents()
            # Must still show 'thoa' and HUD size must remain 100% locked
            self.assertIn("thoa", panel.card_cpu.sub.text())
            self.assertEqual(panel.hud.size(), initial_hud_size)
        finally:
            panel.close()
            listener.close()


class Px4UlogReplayTests(unittest.TestCase):
    """Integration tests loading the user's real PX4 .ulg files into Up4000Panel."""

    @unittest.skipUnless(ULOG_07_05_39.exists(), "07_05_39.ulg not present on disk")
    def test_replay_07_05_39_ulog_posctl_flight(self):
        listener = Up4000StatusListener(port=0)
        panel = Up4000Panel(listener)
        panel.resize(1380, 840)
        try:
            session = panel.load_ulog_file(ULOG_07_05_39, step_s=0.25)
            self.assertAlmostEqual(session["duration_s"], 43.0, delta=1.0)
            self.assertEqual(session["hw_ver"], "PX4_FMU_V6C")
            self.assertGreater(len(session["ned_trail"]), 50)
            self.assertEqual(panel.card_link.value.text(), "ULOG REPLAY")
            self.assertIn("07_05_39.ulg", panel.card_link.sub.text())

            # Frame 0 starts armed in POSCTL (on ground before takeoff)
            panel.jump_to_arm()
            self.assertEqual(panel.card_vehicle.value.text(), "ARMED")
            self.assertIn("POSCTL", panel.card_vehicle.sub.text())
            self.assertIn("on ground", panel.card_vehicle.sub.text())
            self.assertIn("Armed by RC switch", panel.lbl_text.text())

            # Jump to Max Altitude (~6.2m, z ~ -6.22m)
            panel.jump_to_max_alt()
            self.assertEqual(panel.card_vehicle.value.text(), "ARMED")
            self.assertIn("in air", panel.card_vehicle.sub.text())
            self.assertLess(panel.hud.z_d, -5.0)
            self.assertTrue(panel.hud.has_attitude)
            self.assertTrue(panel.hud.has_position)
            self.assertIn("Takeoff detected", panel.lbl_text.text())
            self.assertIn("Accel 1 clipping", panel.lbl_text.text())

            # Scrub to the end of the flight -> DISARMED after landing
            panel.set_replay_index(len(session["frames"]) - 1)
            self.assertEqual(panel.card_vehicle.value.text(), "DISARMED")
            self.assertIn("on ground", panel.card_vehicle.sub.text())

            # Verify HUD grab works cleanly
            pix = panel.grab()
            self.assertFalse(pix.isNull())
        finally:
            listener.close()

    @unittest.skipUnless(ULOG_03_31_22.exists(), "03_31_22.ulg not present on disk")
    def test_replay_03_31_22_ulog_stabilized_flight_and_ekf_spike(self):
        listener = Up4000StatusListener(port=0)
        panel = Up4000Panel(listener)
        panel.resize(1380, 840)
        try:
            session = panel.load_ulog_file(ULOG_03_31_22, step_s=0.50)
            self.assertAlmostEqual(session["duration_s"], 406.7, delta=1.5)
            self.assertEqual(session["hw_ver"], "PX4_FMU_V6C")
            self.assertGreater(session["max_alt_m"], 9.5)
            self.assertGreater(session["max_ratio"], 1.5)

            # Jump to Arm -> ARMED in STABILIZED mode
            panel.jump_to_arm()
            self.assertEqual(panel.card_vehicle.value.text(), "ARMED")
            self.assertIn("STABILIZED", panel.card_vehicle.sub.text())

            # Jump to Peak EKF Innovation Ratio -> vel_ratio > 1.0 (rejected, red #d93025)
            panel.jump_to_max_ratio()
            self.assertGreater(panel.ratio_bars["vel_ratio"].value, 1.0)
            self.assertIn("#d93025", panel.lbl_ekf.text())
            self.assertIn("Avionics Power low", panel.lbl_text.text())

            # Switch back to Live UDP mode
            panel.switch_to_live_mode()
            self.assertIsNone(panel._replay_session)
            self.assertEqual(panel.card_link.value.text(), "WAITING")
        finally:
            listener.close()


if __name__ == "__main__":
    unittest.main()
