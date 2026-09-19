"""Static integration checks for the standalone A1..A8 Zephyr projects."""

from __future__ import annotations

import hashlib
import re
import sys
from pathlib import Path


FIRMWARE_ROOT = Path(__file__).resolve().parents[1]
COMMON_FILES = (
    "app.overlay",
    "include/anchor_ranging.h",
    "include/dw1000_hw.h",
    "include/uwb_calibration.h",
    "include/uwb_platform.h",
    "src/main.c",
    "src/drivers/dw1000.c",
    "src/platform/uwb_platform_zephyr.c",
    "src/ranging/anchor_ranging.c",
)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def validate_anchor(anchor_id: int, reference_hashes: dict[str, str]) -> None:
    project = FIRMWARE_ROOT / f"Anchor_{anchor_id}"
    require(project.is_dir(), f"Missing project: {project.name}")

    config = read(project / "include/uwb_app_config.h")
    addresses = re.findall(
        r"^#define\s+ANCHOR_ADDR\s+\(\(uint16_t\)(\d+)U\)",
        config,
        flags=re.MULTILINE,
    )
    require(addresses == [str(anchor_id)],
            f"{project.name}: ANCHOR_ADDR is not uniquely set to {anchor_id}U")

    cmake = read(project / "CMakeLists.txt")
    require(f"project(dwm1001_anchor_{anchor_id})" in cmake,
            f"{project.name}: CMake project identity is wrong")

    build_script = read(project / "scripts/build.ps1")
    flash_script = read(project / "scripts/flash.ps1")
    require(f"'Anchor_{anchor_id}'" in build_script,
            f"{project.name}: build script maps the wrong project")
    require(f"'Anchor_{anchor_id}\\build'" in flash_script,
            f"{project.name}: flash script maps the wrong project")

    for relative_path, expected_hash in reference_hashes.items():
        candidate = project / relative_path
        require(candidate.is_file(), f"{project.name}: missing {relative_path}")
        require(digest(candidate) == expected_hash,
                f"{project.name}: shared source diverged: {relative_path}")


def validate_tag() -> None:
    header = read(FIRMWARE_ROOT / "Tag/include/tag_ranging.h")
    source = read(FIRMWARE_ROOT / "Tag/src/ranging/tag_ranging.c")
    calibration = read(FIRMWARE_ROOT / "Tag/include/uwb_calibration.h")

    require(re.search(r"^#define\s+TAG_NUM_ANCHORS\s+8U$", header, re.MULTILINE) is not None,
            "TAG_NUM_ANCHORS must be 8U")
    require(re.search(r"^#define\s+TAG_CYCLE_MS\s+20U$", header, re.MULTILINE) is not None,
            "TAG_CYCLE_MS must be 20U (50 Hz)")

    configured_ids = [int(value, 16) for value in re.findall(
        r"\{\s*0x([0-9A-Fa-f]{4})U,\s*UWB_SS_CAL_A\d_BIT\s*\}", source
    )]
    require(configured_ids == list(range(1, 9)),
            f"TAG anchor table must be A1..A8, got {configured_ids}")

    for anchor_id in range(1, 9):
        require(f"UWB_DS_CAL_A{anchor_id}_BIT" in calibration,
                f"Missing DS calibration bit A{anchor_id}")
        require(f"UWB_SS_CAL_A{anchor_id}_BIT" in calibration,
                f"Missing SS calibration bit A{anchor_id}")
        require(f"UWB_DS_OFFSET_A{anchor_id}_M" in calibration,
                f"Missing DS offset A{anchor_id}")


def main() -> int:
    reference = FIRMWARE_ROOT / "Anchor_1"
    reference_hashes = {path: digest(reference / path) for path in COMMON_FILES}
    for anchor_id in range(1, 9):
        validate_anchor(anchor_id, reference_hashes)
    validate_tag()
    print("A1..A8 identity, source-parity and TAG topology checks passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
