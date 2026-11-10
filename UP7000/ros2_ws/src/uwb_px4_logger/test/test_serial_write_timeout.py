"""A non-reading serial peer must not permanently stop the relay thread."""
import os
from pathlib import Path
import subprocess
import sys
import unittest


@unittest.skipUnless(os.name == 'posix', 'requires a Linux pseudo-terminal')
class SerialWriteTimeoutTests(unittest.TestCase):
    def test_unresponsive_peer_releases_writer(self):
        source = str(Path(__file__).resolve().parents[1])
        code = '''
import os, pty, sys, time
import serial
sys.path.insert(0, sys.argv[1])
from uwb_px4_logger.uwb_relay import _open_serial
master, slave = pty.openpty()
ser = _open_serial(os.ttyname(slave), 460800)
started = time.monotonic()
try:
    ser.write(b'x' * 1048576)
except serial.SerialTimeoutException:
    assert time.monotonic() - started < 1.0
    print('blocked serial write timed out and returned', flush=True)
else:
    raise AssertionError('non-reading serial peer did not exhaust its queue')
finally:
    ser.close()
    os.close(master)
    os.close(slave)
'''
        result = subprocess.run([sys.executable, '-B', '-c', code, source],
                                capture_output=True, text=True, timeout=3)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('timed out and returned', result.stdout)
