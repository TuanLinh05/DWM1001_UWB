"""Check the four STM32F103 anchor images before programming them.

The source check is intentionally usable before a Zephyr build.  The full
check parses every Intel HEX record, checks the Cortex-M vector table and the
64 KiB STM32F103C8 flash boundary, and detects accidentally duplicated images.
It cannot prove that the firmware works over the air; that needs a real board.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import struct
from dataclasses import dataclass
from pathlib import Path


FIRMWARE = Path(__file__).resolve().parents[1]
FLASH_BASE = 0x08000000
FLASH_END = 0x08010000  # STM32F103C8: 64 KiB
RAM_BASE = 0x20000000
RAM_END = 0x20005000  # STM32F103C8: 20 KiB
ANCHOR_IDS = range(5, 9)
PHY_DEFINES = (
    "UWB_USE_DS_TWR",
    "UWB_USE_HW_ANTENNA_DELAY",
    "UWB_USE_LEGACY_OFFSET",
    "UWB_USE_CLOCK_CORRECTION",
    "UWB_TX_POWER_MODE",
    "UWB_DW_REFERENCE_TUNING",
)


class ValidationError(Exception):
    """A source configuration or image is unsafe to assign to a board."""


@dataclass(frozen=True)
class ImageInfo:
    path: Path
    size: int
    last_address: int
    initial_sp: int
    reset_vector: int
    sha256: str


def _defines(path: Path) -> dict[str, str]:
    if not path.is_file():
        raise ValidationError(f"missing configuration: {path}")
    result = {}
    for name, value in re.findall(r"^\s*#define\s+(\w+)\s+([^\r\n/]+)",
                                  path.read_text(encoding="utf-8"), re.MULTILINE):
        result[name] = value.strip()
    return result


def _integer(text: str) -> int:
    # Accept the project's C literal forms, including ((uint16_t)5U).
    value = text.strip().replace("((uint16_t)", "(")
    value = re.sub(r"\b([0-9]+|0[xX][0-9a-fA-F]+)[uUlL]+\b", r"\1", value)
    value = value.strip("() ")
    try:
        return int(value, 0)
    except ValueError as exc:
        raise ValidationError(f"ID is not a fixed integer: {text!r}") from exc


def verify_sources(firmware: Path = FIRMWARE) -> None:
    """Guard board/ID assignment and radio-profile parity with A1–A4."""
    reference = _defines(firmware / "Anchor_1/include/uwb_app_config.h")
    for anchor_id in ANCHOR_IDS:
        project = firmware / f"STM32_Anchor_{anchor_id}"
        config = _defines(project / "include/uwb_app_config.h")
        address = config.get("ANCHOR_ADDR", "")
        if address == "((uint16_t)UWB_ANCHOR_ID)":
            address = config.get("UWB_ANCHOR_ID", "")
        if _integer(address) != anchor_id:
            raise ValidationError(f"{project.name}: ANCHOR_ADDR must be {anchor_id}")
        if "UWB_ANCHOR_ID" in config and _integer(config["UWB_ANCHOR_ID"]) != anchor_id:
            raise ValidationError(f"{project.name}: UWB_ANCHOR_ID must be {anchor_id}")
        if _integer(config.get("TAG_ADDR", "")) != 0:
            raise ValidationError(f"{project.name}: TAG_ADDR must be 0")
        for name in PHY_DEFINES:
            if config.get(name, reference.get(name)) != reference.get(name):
                raise ValidationError(
                    f"{project.name}: {name} differs from DWM1001C A1 configuration"
                )
        if config.get("UWB_DS_CALIBRATED_MASK") != "0U":
            raise ValidationError(f"{project.name}: calibration must remain fail-closed")
        if not (project / "app.overlay").is_file():
            raise ValidationError(f"{project.name}: missing STM32 board overlay")
        if not (project / "CMakeLists.txt").is_file():
            raise ValidationError(f"{project.name}: missing Zephyr project")
        if not (project / "prj.conf").is_file():
            raise ValidationError(f"{project.name}: missing Zephyr configuration")


def parse_hex(path: Path) -> ImageInfo:
    """Validate Intel HEX checksums, bounds, and boot vectors without deps."""
    if not path.is_file():
        raise ValidationError(f"missing image: {path}")

    memory: dict[int, int] = {}
    base = 0
    eof = False
    lines = path.read_text(encoding="ascii").splitlines()
    for number, line in enumerate(lines, start=1):
        if eof:
            raise ValidationError(f"{path}:{number}: record after EOF")
        if not line.startswith(":"):
            raise ValidationError(f"{path}:{number}: invalid Intel HEX record")
        try:
            record = bytes.fromhex(line[1:])
        except ValueError as exc:
            raise ValidationError(f"{path}:{number}: non-hexadecimal record") from exc
        if len(record) < 5 or len(record) != record[0] + 5:
            raise ValidationError(f"{path}:{number}: incorrect record length")
        if sum(record) & 0xFF:
            raise ValidationError(f"{path}:{number}: checksum error")
        count = record[0]
        offset = (record[1] << 8) | record[2]
        kind = record[3]
        data = record[4:-1]

        if kind == 0:  # Data
            for index, value in enumerate(data):
                address = base + offset + index
                if not FLASH_BASE <= address < FLASH_END:
                    raise ValidationError(
                        f"{path}:{number}: 0x{address:08X} outside STM32F103C8 flash"
                    )
                if address in memory and memory[address] != value:
                    raise ValidationError(f"{path}:{number}: conflicting overlapping data")
                memory[address] = value
        elif kind == 1:  # EOF
            if count != 0 or offset != 0:
                raise ValidationError(f"{path}:{number}: invalid EOF")
            eof = True
        elif kind == 2:  # Extended segment address
            if count != 2 or offset != 0:
                raise ValidationError(f"{path}:{number}: invalid segment address")
            base = int.from_bytes(data, "big") << 4
        elif kind == 4:  # Extended linear address
            if count != 2 or offset != 0:
                raise ValidationError(f"{path}:{number}: invalid linear address")
            base = int.from_bytes(data, "big") << 16
        elif kind in (3, 5):  # Optional entry point; vector table is authority
            if count != 4 or offset != 0:
                raise ValidationError(f"{path}:{number}: invalid entry point")
        else:
            raise ValidationError(f"{path}:{number}: unsupported record type {kind}")

    if not eof or not memory:
        raise ValidationError(f"{path}: missing EOF or empty image")
    if min(memory) != FLASH_BASE:
        raise ValidationError(f"{path}: vector table does not start at 0x{FLASH_BASE:08X}")
    try:
        vectors = bytes(memory[FLASH_BASE + index] for index in range(8))
    except KeyError as exc:
        raise ValidationError(f"{path}: incomplete Cortex-M vector table") from exc
    initial_sp, reset_vector = struct.unpack("<II", vectors)
    if not RAM_BASE < initial_sp <= RAM_END or initial_sp & 7:
        raise ValidationError(f"{path}: invalid initial SP 0x{initial_sp:08X}")
    reset_code = reset_vector & ~1
    if not (reset_vector & 1) or not FLASH_BASE <= reset_code < FLASH_END:
        raise ValidationError(f"{path}: invalid Thumb reset vector 0x{reset_vector:08X}")
    if any(reset_code + index not in memory for index in range(2)):
        raise ValidationError(f"{path}: reset handler is absent from image")

    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return ImageInfo(path, len(memory), max(memory), initial_sp, reset_vector, digest)


def verify_images(firmware: Path = FIRMWARE) -> dict[int, ImageInfo]:
    verify_sources(firmware)
    result = {}
    for anchor_id in ANCHOR_IDS:
        path = firmware / "stm32_anchor/dist" / f"anchor_{anchor_id}_stm32f103.hex"
        result[anchor_id] = parse_hex(path)
    digests = [info.sha256 for info in result.values()]
    if len(set(digests)) != len(digests):
        raise ValidationError("two STM32 anchor HEX files are byte-identical")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-only", action="store_true",
                        help="check project configuration before compiling")
    parser.add_argument("--firmware", type=Path, default=FIRMWARE,
                        help="Firmware directory, for testing or alternate checkout")
    args = parser.parse_args()
    try:
        if args.source_only:
            verify_sources(args.firmware)
            print("STM32 A5-A8 source and profile checks passed")
        else:
            images = verify_images(args.firmware)
            for anchor_id, info in images.items():
                print(f"A{anchor_id}: {info.path} | {info.size} bytes | "
                      f"last 0x{info.last_address:08X} | SHA-256 {info.sha256}")
            print("STM32 A5-A8 source and HEX checks passed")
    except (OSError, UnicodeError, ValidationError) as exc:
        parser.exit(1, f"ERROR: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
