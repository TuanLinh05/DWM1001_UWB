"""Verify all nine packaged images and their build configuration.

Checks file checksums, vectors/flash limits, equality to build outputs,
config identity, strict DS, pinctrl, speed optimisation and SPI/FPU selection.
This cannot confirm MCU execution time, RF success rate or hardware Hz.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import struct

from uwb_layout import node_dir

FIRMWARE = Path(__file__).resolve().parents[1]
NODES = [("TAG", "Tag_DevKit", "TAG_DWM1001_DEV.hex")]
NODES += [(f"A{i}", f"Anchor_{i}", f"A{i}_DWM1001C_nRF52832.hex") for i in range(1, 5)]
NODES += [(f"A{i}", f"STM32_Anchor_{i}", f"A{i}_STM32F103C8.hex") for i in range(5, 9)]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def parse_image(path: Path, stm32: bool) -> dict:
    flash_base = 0x08000000 if stm32 else 0
    flash_end = flash_base + (65536 if stm32 else 524288)
    ram_end = 0x20000000 + (20480 if stm32 else 65536)
    memory = {}
    base = 0
    eof = False
    for number, line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        require(not eof and line.startswith(":"), f"{path.name}:{number}: invalid record/EOF")
        record = bytes.fromhex(line[1:])
        require(len(record) >= 5 and len(record) == record[0] + 5 and sum(record) % 256 == 0,
                f"{path.name}:{number}: HEX length/checksum")
        count, offset, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
        payload = record[4:-1]
        if kind == 0:
            for index, value in enumerate(payload):
                address = base + offset + index
                require(flash_base <= address < flash_end, f"{path.name}: address outside flash")
                require(address not in memory or memory[address] == value,
                        f"{path.name}: conflicting records")
                memory[address] = value
        elif kind == 1:
            require(count == 0 and offset == 0, f"{path.name}: invalid EOF")
            eof = True
        elif kind in (2, 4):
            require(count == 2 and offset == 0, f"{path.name}: invalid base")
            base = int.from_bytes(payload, "big") << (4 if kind == 2 else 16)
        elif kind in (3, 5):
            require(count == 4 and offset == 0, f"{path.name}: invalid entry point")
        else:
            raise ValueError(f"{path.name}: unsupported record type {kind}")
    require(eof and bool(memory) and min(memory) == flash_base, f"{path.name}: missing vectors/EOF")
    initial_sp, reset_vector = struct.unpack("<II", bytes(memory[flash_base + i] for i in range(8)))
    reset_code = reset_vector & ~1
    require(0x20000000 < initial_sp <= ram_end and initial_sp % 8 == 0,
            f"{path.name}: invalid SP")
    require(bool(reset_vector & 1) and flash_base <= reset_code < flash_end
            and reset_code in memory and reset_code + 1 in memory, f"{path.name}: invalid reset")
    return dict(programmed_bytes=len(memory), flash_limit_bytes=flash_end-flash_base,
                last_address=f"0x{max(memory):08X}", initial_sp=f"0x{initial_sp:08X}",
                reset_vector=f"0x{reset_vector:08X}")


def verify(firmware: Path, package: Path) -> list[dict]:
    with (package / "manifest.csv").open(encoding="utf-8-sig", newline="") as stream:
        manifest = list(csv.DictReader(stream))
    require(len(manifest) == 9 and {row["File"] for row in manifest} == {n[2] for n in NODES},
            "Manifest must contain exactly TAG and the eight physical anchors")
    rows = []
    for node, project, filename in NODES:
        stm32 = project.startswith("STM32_")
        path = package / filename
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        row = next(r for r in manifest if r["File"] == filename)
        require(digest == row["SHA256"].lower() and path.stat().st_size == int(row["Bytes"]),
                f"{node}: manifest mismatch")
        is_300 = package.name == "highrate_300"
        build_dir_name = "build_highrate_300" if is_300 else "build_highrate"
        build = node_dir(project, firmware) / build_dir_name
        require(path.read_bytes() == (build / "zephyr/zephyr.hex").read_bytes(),
                f"{node}: package differs from build")
        ninja = (build / "build.ninja").read_text(encoding="utf-8")
        config = (build / "zephyr/.config").read_text(encoding="utf-8")
        config_ids = set(re.findall(r"UWB_BUILD_CONFIG_HASH=(0x[0-9a-fA-F]{8})", ninja))
        require(config_ids == {row["ConfigID"]}, f"{node}: config identity mismatch")
        for option in ("PINCTRL", "SPEED_OPTIMIZATIONS", "WATCHDOG", "TIMING_FUNCTIONS"):
            require(f"CONFIG_{option}=y" in config, f"{node}: {option} disabled")
        required_defines = ["UWB_HIGH_RATE=1", "UWB_BURST_REQUIRE_DS=1"]
        if is_300:
            required_defines += ["UWB_HIGH_RATE_300=1", "UWB_PHY_PREAMBLE=64U"]
        for define in required_defines:
            require(define in ninja, f"{node}: {define} absent")
        require("-ffast-math" not in ninja, f"{node}: unsafe fast-math")
        speed = 16000000 if stm32 else 8000000
        require(int(row["SPIHz"]) == speed, f"{node}: manifest SPI speed")
        if stm32:
            require("UWB_DIRECT_STM32_SPI=1" in ninja and "DW_SPI_FAST_MHZ=16U" in ninja,
                    f"{node}: STM32 backend absent")
            require("CONFIG_PINCTRL_STM32=y" in config, f"{node}: STM32 pinctrl absent")
        else:
            require("UWB_DIRECT_SPIM=1" in ninja and "CONFIG_NRFX_SPIM=y" in config,
                    f"{node}: nRF SPIM/pinctrl support absent")
            require("CONFIG_FPU=y" in config and "CONFIG_FPU_SHARING=y" in config
                    and "-mfpu=fpv4-sp-d16" in ninja and "-mfloat-abi=hard" in ninja,
                    f"{node}: nRF FPU configuration absent")
        dts = (build / "zephyr/zephyr.dts").read_text(encoding="utf-8")
        bus_address = "40013000" if stm32 else "40023000"
        bus = re.search(rf"spi@{bus_address}\s*\{{(.*?)ieee802154:", dts, re.S)
        require(bus is not None and 'status = "disabled"' in bus[1],
                f"{node}: Zephyr SPI still owns the radio bus")
        radio = re.search(r"ieee802154:\s*dw1000@0\s*\{(.*?)\};", dts, re.S)
        require(radio is not None, f"{node}: missing radio node")
        frequency = re.search(r"spi-max-frequency\s*=\s*<\s*(0x[0-9a-f]+|\d+)\s*>", radio[1])
        require(frequency is not None and int(frequency[1], 0) == speed,
                f"{node}: radio speed mismatch")
        image = parse_image(path, stm32)
        rows.append(dict(node=node, project=project, file=filename, config_id=row["ConfigID"],
                         sha256=digest, spi_hz=speed, **image))
    require(len({r["sha256"] for r in rows}) == 9, "Two different nodes have identical images")
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware", type=Path, default=FIRMWARE)
    parser.add_argument("--package", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    rows = verify(args.firmware, args.package or args.firmware / "release" / "highrate_200")
    result = dict(status="PASS", hardware_frequency_verified=False, nodes=rows)
    if args.json:
        args.json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    for row in rows:
        print(f"{row['node']}: {row['config_id']} {row['programmed_bytes']} bytes, "
              f"SPI {row['spi_hz']//1000000} MHz, HEX/vector/build/config PASS")
    print("9/9 images verified. Hardware ranging/200 Hz remains to be measured.")


if __name__ == "__main__":
    main()
