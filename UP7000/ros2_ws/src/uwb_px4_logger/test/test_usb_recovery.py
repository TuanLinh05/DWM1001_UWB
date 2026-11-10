"""USB recovery selects only SEGGER and resumes the service on failure."""
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

spec = importlib.util.spec_from_file_location(
    "reset_tag_usb", Path(__file__).resolve().parents[4] / "scripts" / "reset_tag_usb.py")
recovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recovery)


class UsbRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def device(self, name, vendor, product, serial):
        device = self.root / name
        device.mkdir()
        for key, value in {"idVendor": vendor, "product": product, "serial": serial,
                           "busnum": "1", "devnum": "5"}.items():
            (device / key).write_text(value)
        return device

    def test_never_selects_pixhawk_and_requires_unambiguous_jlink(self):
        self.device("1-5", "3185", "PX4 FMU v6C.x", "0")
        with self.assertRaises(RuntimeError):
            recovery.find_jlink(self.root)
        first = self.device("1-6", "1366", "J-Link", "123")
        self.assertEqual(recovery.find_jlink(self.root), first)
        self.device("1-7", "1366", "J-Link", "456")
        with self.assertRaises(RuntimeError):
            recovery.find_jlink(self.root)
        self.assertEqual(recovery.find_jlink(self.root, "123"), first)

    def test_logger_restarts_even_if_usb_reset_fails(self):
        device = self.device("1-6", "1366", "J-Link", "123")
        ioctl = Mock(side_effect=OSError("USB reset failed"))
        with patch.dict(sys.modules, {"fcntl": SimpleNamespace(ioctl=ioctl)}), \
                patch.object(sys, "argv", ["reset_tag_usb.py"]), \
                patch.object(recovery, "find_jlink", return_value=device), \
                patch.object(recovery.os, "geteuid", return_value=0, create=True), \
                patch.object(recovery.os, "open", return_value=7), \
                patch.object(recovery.os, "close") as close, \
                patch.object(recovery.subprocess, "run", return_value=SimpleNamespace(returncode=0)) as run:
            with self.assertRaisesRegex(OSError, "USB reset failed"):
                recovery.main()
            self.assertIn(call(["systemctl", "stop", "uwb-px4-logger"], check=True), run.call_args_list)
            self.assertEqual(run.call_args_list[-1], call(["systemctl", "start", "uwb-px4-logger"], check=True))
            close.assert_called_once_with(7)
