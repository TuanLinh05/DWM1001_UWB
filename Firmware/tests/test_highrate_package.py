"""Exercise HEX validation without a compiler, image or connected device."""
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from verify_highrate_package import parse_image

class StringImage:
    """Path-like input so validation tests need no temporary file access."""
    name = "fixture.hex"

    def __init__(self, text):
        self.text = text

    def read_text(self, encoding="ascii"):
        return self.text


def record(kind, offset, data=b""):
    payload = bytes((len(data), offset >> 8, offset & 255, kind)) + data
    return ":" + (payload + bytes((-sum(payload) & 255,))).hex().upper()


class HexValidationTests(unittest.TestCase):
    def image(self, stm32, *, bad_reset=False, bad_sp=False, outside=False):
        base = 0x08000000 if stm32 else 0
        sp = 0x20001000 if not bad_sp else 0x30000000
        reset = base + (0x10001 if bad_reset else 9)
        lines = [record(4, 0, (base >> 16).to_bytes(2, "big")),
                 record(0, 0, struct.pack("<II", sp, reset) + b"\x00\xbf")]
        if outside:
            extra_base = base + (0x10000 if stm32 else 0x80000)
            lines += [record(4, 0, (extra_base >> 16).to_bytes(2, "big")), record(0, 0, b"\x00")]
        lines.append(record(1, 0))
        return StringImage("\n".join(lines) + "\n")

    def test_valid_vectors_for_both_boards(self):
        for stm32 in (False, True):
            result = parse_image(self.image(stm32), stm32)
            self.assertEqual(result["programmed_bytes"], 10)

    def test_flash_bounds(self):
        for stm32 in (False, True):
            with self.assertRaisesRegex(ValueError, "outside flash"):
                parse_image(self.image(stm32, outside=True), stm32)

    def test_reject_invalid_sp_and_reset(self):
        for option in ("bad_sp", "bad_reset"):
            with self.assertRaisesRegex(ValueError, "invalid SP|invalid reset"):
                parse_image(self.image(True, **{option: True}), True)

    def test_corrupt_checksum(self):
        path = self.image(True)
        lines = path.read_text().splitlines()
        lines[1] = lines[1][:-2] + "00"  # damage the data record checksum
        path.text = "\n".join(lines) + "\n"
        with self.assertRaisesRegex(ValueError, "HEX length/checksum"):
            parse_image(path, True)


if __name__ == "__main__":
    unittest.main()
