"""Exercise the STM32 Intel HEX release gate with synthetic images."""

from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from verify_stm32_images import ValidationError, parse_hex  # noqa: E402


def record(kind: int, offset: int, data: bytes = b"") -> str:
    payload = bytes((len(data), offset >> 8, offset & 0xFF, kind)) + data
    return ":" + (payload + bytes((-sum(payload) & 0xFF,))).hex().upper()


class HexValidationTest(unittest.TestCase):
    def setUp(self) -> None:
        scratch = Path(__file__).resolve().parent / "tmp"
        scratch.mkdir(exist_ok=True)
        self.path = scratch / f"stm32_image_{id(self):x}.hex"
        self.addCleanup(self.path.unlink, missing_ok=True)
        self.vectors = struct.pack("<II", 0x20005000, 0x08000009) + b"\x00\xBF"

    def image(self, *, payload: bytes | None = None, base: int = 0x0800,
              checksum_error: bool = False) -> None:
        lines = [record(4, 0, base.to_bytes(2, "big")),
                 record(0, 0, self.vectors if payload is None else payload),
                 record(1, 0)]
        if checksum_error:
            lines[1] = lines[1][:-2] + "00"
        self.path.write_text("\n".join(lines) + "\n", encoding="ascii")

    def test_valid_f103_image(self) -> None:
        self.image()
        info = parse_hex(self.path)
        self.assertEqual(info.initial_sp, 0x20005000)
        self.assertEqual(info.reset_vector, 0x08000009)

    def test_checksum_rejected(self) -> None:
        self.image(checksum_error=True)
        with self.assertRaisesRegex(ValidationError, "checksum"):
            parse_hex(self.path)

    def test_address_outside_flash_rejected(self) -> None:
        self.image(base=0x0801)
        with self.assertRaisesRegex(ValidationError, "outside STM32F103C8 flash"):
            parse_hex(self.path)

    def test_bad_stack_rejected(self) -> None:
        self.image(payload=struct.pack("<II", 0x20008000, 0x08000009) + b"\x00\xBF")
        with self.assertRaisesRegex(ValidationError, "initial SP"):
            parse_hex(self.path)

    def test_non_thumb_reset_rejected(self) -> None:
        self.image(payload=struct.pack("<II", 0x20005000, 0x08000008) + b"\x00\xBF")
        with self.assertRaisesRegex(ValidationError, "Thumb reset"):
            parse_hex(self.path)


if __name__ == "__main__":
    unittest.main()
