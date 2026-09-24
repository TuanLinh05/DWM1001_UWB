"""Static checks on the node projects and the shared source layout.

Sources live in Firmware/common and every node directory only carries its
configuration, so these checks replace the old file-hash parity test: what
they now guard is that no node re-introduces a private copy of the sources,
that the roles/addresses are unique and that the build scripts stay usable.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

FIRMWARE_ROOT = Path(__file__).resolve().parents[1]
COMMON = FIRMWARE_ROOT / "common"
ANCHOR_COUNT = 8
NODES = (
    ("Tag", "TAG", "dwm1001_tag"),
    ("Tag_DevKit", "TAG", "dwm1001_dev_tag"),
    ("Sniffer_DevKit", "SNIFFER", "dwm1001_dev_sniffer"),
) + tuple((f"Anchor_{n}", "ANCHOR", f"dwm1001_anchor_{n}") for n in range(1, ANCHOR_COUNT + 1))

# 'Tag\build', 'Anchor_3', ... as the PowerShell helpers spell them;
# the longest names come first so "Tag" cannot swallow "Tag_DevKit".
PROJECT_TOKEN = re.compile(
    r"'(" + "|".join(sorted((name for name, _, _ in NODES), key=len, reverse=True))
    + r")(?:\\[^']*)?'")

REQUIRED_COMMON_SOURCES = (
    "src/drivers/dw1000.c",
    "src/platform/uwb_platform_zephyr.c",
    "src/ranging/tag_ranging.c",
    "src/ranging/anchor_ranging.c",
    "src/ranging/uwb_frame.c",
    "src/app/main_tag.c",
    "src/app/main_anchor.c",
    "src/app/main_sniffer.c",
    "src/app/uwb_health.c",
    "src/app/uwb_cmd.c",
    "src/app/uwb_settings.c",
    "src/telemetry/telemetry.c",
    "src/telemetry/telemetry_frame.c",
    "src/telemetry/uart_tx_zephyr.c",
    "cmake/uwb_node.cmake",
)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def validate_common() -> None:
    for relative in REQUIRED_COMMON_SOURCES:
        require((COMMON / relative).is_file(), f"missing shared source: common/{relative}")

    cmake = read(COMMON / "cmake/uwb_node.cmake")
    for role in ("TAG", "ANCHOR", "SNIFFER"):
        require(f'role STREQUAL "{role}"' in cmake, f"uwb_node.cmake has no {role} role")
    require("UWB_BUILD_GIT_HASH" in cmake, "build identity is not compiled in")
    require("--untracked-files=normal" in cmake,
            "build dirty flag must include untracked source files")
    require("UWB_BUILD_CONFIG_HASH" in cmake,
            "node-local configuration identity is not compiled in")


def validate_node(name: str, role: str, project: str) -> None:
    node = FIRMWARE_ROOT / name
    require(node.is_dir(), f"missing project: {name}")

    for required in ("CMakeLists.txt", "prj.conf", "app.overlay",
                     "include/uwb_app_config.h", "scripts/build.ps1", "scripts/flash.ps1"):
        require((node / required).is_file(), f"{name}: missing {required}")

    cmake = read(node / "CMakeLists.txt")
    require(f"project({project})" in cmake, f"{name}: CMake project identity is wrong")
    require(f"uwb_node_setup({role})" in cmake, f"{name}: role must be {role}")
    require("../common/cmake/uwb_node.cmake" in cmake, f"{name}: does not use the shared build")

    # No node may keep its own copy of the sources or shared headers.
    require(not (node / "src").exists(), f"{name}: private src/ directory reappeared")
    strays = [path.name for path in (node / "include").iterdir()
              if path.name != "uwb_app_config.h"]
    require(not strays, f"{name}: private copies of shared headers: {strays}")

    # A script may name its project literally or derive it from the directory;
    # what must never happen is a script pointing at a different node.
    for script_name in ("scripts/build.ps1", "scripts/flash.ps1"):
        script = read(node / script_name)
        referenced = set(PROJECT_TOKEN.findall(script))
        derived = "Split-Path $projectRoot -Leaf" in script
        require(derived or referenced == {name},
                f"{name}: {script_name} does not map to the project")
        require(not referenced - {name},
                f"{name}: {script_name} also references {sorted(referenced - {name})}")
    require("UWB_REPO_ROOT" in read(node / "scripts/build.ps1"),
            f"{name}: build script must export UWB_REPO_ROOT for the build id")

    config = read(node / "include/uwb_app_config.h")
    if role == "ANCHOR":
        anchor_id = int(name.split("_")[1])
        addresses = re.findall(r"^#define\s+ANCHOR_ADDR\s+\(\(uint16_t\)(\d+)U\)",
                               config, flags=re.MULTILINE)
        require(addresses == [str(anchor_id)],
                f"{name}: ANCHOR_ADDR is not uniquely set to {anchor_id}U")
        require("CONFIG_HWINFO=y" in read(node / "prj.conf"),
                f"{name}: needs CONFIG_HWINFO for the reset cause")
    if role == "TAG":
        require(re.search(r"^#define\s+TAG_ADDR\s+\(\(uint16_t\)0U\)", config, re.MULTILINE),
                f"{name}: TAG_ADDR must be 0")
        prj = read(node / "prj.conf")
        for option in ("CONFIG_SETTINGS=y", "CONFIG_NVS=y", "CONFIG_FLASH_MAP=y",
                       "CONFIG_HWINFO=y", "CONFIG_REBOOT=y"):
            require(option in prj, f"{name}: prj.conf is missing {option}")


def validate_topology() -> None:
    header = read(COMMON / "include/tag_ranging.h")
    source = read(COMMON / "src/ranging/tag_ranging.c")
    calibration = read(COMMON / "include/uwb_calibration.h")

    require(re.search(r"^#define\s+TAG_NUM_ANCHORS\s+8U$", header, re.MULTILINE),
            "TAG_NUM_ANCHORS must be 8U")
    require(re.search(r"^#define\s+TAG_CYCLE_MS\s+20U$", header, re.MULTILINE),
            "TAG_CYCLE_MS must be 20U (50 Hz)")

    configured = [int(value, 16) for value in re.findall(
        r"\{\s*0x([0-9A-Fa-f]{4})U,\s*UWB_SS_CAL_A\d_BIT,\s*UWB_DS_CAL_A\d_BIT\s*\}", source)]
    require(configured == list(range(1, ANCHOR_COUNT + 1)),
            f"TAG anchor table must be A1..A8, got {configured}")

    for anchor_id in range(1, ANCHOR_COUNT + 1):
        for macro in (f"UWB_DS_CAL_A{anchor_id}_BIT", f"UWB_SS_CAL_A{anchor_id}_BIT",
                      f"UWB_DS_OFFSET_A{anchor_id}_M"):
            require(macro in calibration, f"Missing {macro}")

    # Calibration stays fail-closed in the repository.
    tag_config = read(FIRMWARE_ROOT / "Tag/include/uwb_app_config.h")
    require(re.search(r"^#define\s+UWB_DS_CALIBRATED_MASK\s+0U", tag_config, re.MULTILINE),
            "UWB_DS_CALIBRATED_MASK must stay 0 until the hardware is measured")


def main() -> int:
    validate_common()
    for name, role, project in NODES:
        validate_node(name, role, project)
    validate_topology()
    print(f"node project checks passed ({len(NODES)} projects, shared sources, topology)")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
