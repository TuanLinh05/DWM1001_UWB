"""Static checks on the node projects and the shared source layout.

Sources live in Firmware/common/<module>/ and every node directory under
Firmware/nodes/ only carries its configuration, so these checks replace the
old file-hash parity test: what they now guard is that no node re-introduces
a private copy of the sources, that the roles/addresses are unique, that the
node registries (scripts/uwb_nodes.ps1, tools/uwb_layout.py) agree with the
directories and that the build scripts stay usable.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from uwb_layout import COMMON, COMMON_MODULES, FIRMWARE as FIRMWARE_ROOT  # noqa: E402
from uwb_layout import NODE_PATHS, node_dir  # noqa: E402

ANCHOR_COUNT = 8
NODES = (
    ("Tag", "TAG", "dwm1001_tag"),
    ("Tag_DevKit", "TAG", "dwm1001_dev_tag"),
    ("Sniffer_DevKit", "SNIFFER", "dwm1001_dev_sniffer"),
) + tuple((f"Anchor_{n}", "ANCHOR", f"dwm1001_anchor_{n}") for n in range(1, ANCHOR_COUNT + 1))
STM32_ANCHORS = tuple(f"STM32_Anchor_{n}" for n in range(5, ANCHOR_COUNT + 1))

REQUIRED_COMMON_SOURCES = (
    "drivers/dw1000.c",
    "platform/uwb_platform_zephyr.c",
    "ranging/tag_ranging.c",
    "ranging/anchor_ranging.c",
    "ranging/uwb_frame.c",
    "app/main_tag.c",
    "app/main_anchor.c",
    "app/main_sniffer.c",
    "services/uwb_health.c",
    "services/uwb_cmd.c",
    "services/uwb_settings.c",
    "telemetry/telemetry.c",
    "telemetry/telemetry_frame.c",
    "telemetry/uart_tx_zephyr.c",
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

    # Every source/header sits in a module directory; no flat include/ or src/.
    for legacy in ("include", "src"):
        require(not (COMMON / legacy).exists(), f"common/{legacy}/ reappeared")
    for path in COMMON.rglob("*"):
        if path.suffix in (".c", ".h"):
            parts = path.relative_to(COMMON).parts
            require(len(parts) == 2 and parts[0] in COMMON_MODULES,
                    f"common/{'/'.join(parts)} is outside a module directory")

    cmake = read(COMMON / "cmake/uwb_node.cmake")
    modules = re.search(r"set\(UWB_COMMON_MODULES ([^)]*)\)", cmake)
    require(modules is not None and tuple(modules[1].split()) == COMMON_MODULES,
            "uwb_node.cmake UWB_COMMON_MODULES differs from tools/uwb_layout.py")
    for role in ("TAG", "ANCHOR", "SNIFFER"):
        require(f'role STREQUAL "{role}"' in cmake, f"uwb_node.cmake has no {role} role")
    require("UWB_BUILD_GIT_HASH" in cmake, "build identity is not compiled in")
    require("--untracked-files=normal" in cmake,
            "build dirty flag must include untracked source files")
    require("UWB_BUILD_CONFIG_HASH" in cmake,
            "node-local configuration identity is not compiled in")


def validate_registries() -> None:
    """scripts/uwb_nodes.ps1 and tools/uwb_layout.py must name the same paths."""
    script = read(FIRMWARE_ROOT / "scripts/uwb_nodes.ps1")
    registered = dict(re.findall(r"'(\w+)'\s*=\s*@\{\s*Path = '([^']+)'", script))
    anchor_rows = re.findall(r'Path = "nodes\\anchors\\(nrf|stm32)\\(\w+?)_\$n"', script)
    require(sorted(anchor_rows) == [("nrf", "Anchor"), ("stm32", "STM32_Anchor")],
            "uwb_nodes.ps1 anchor paths changed; update tools/uwb_layout.py too")
    for n in range(1, ANCHOR_COUNT + 1):
        registered[f"Anchor_{n}"] = f"nodes\\anchors\\nrf\\Anchor_{n}"
    for name in STM32_ANCHORS:
        registered[name] = f"nodes\\anchors\\stm32\\{name}"
    require({k: v.replace("\\", "/") for k, v in registered.items()} == NODE_PATHS,
            "scripts/uwb_nodes.ps1 and tools/uwb_layout.py disagree on node paths")

    for name in NODE_PATHS:
        require(node_dir(name).is_dir(), f"registered node {name} has no directory")
    found = {path.parent.name for path in (FIRMWARE_ROOT / "nodes").rglob("CMakeLists.txt")}
    require(found == set(NODE_PATHS),
            f"node projects missing from the registries: {sorted(found - set(NODE_PATHS))}")


def validate_shared_build(name: str, node: Path, role: str) -> None:
    cmake = read(node / "CMakeLists.txt")
    require(f"uwb_node_setup({role})" in cmake, f"{name}: role must be {role}")
    include = re.search(r"include\(\$\{CMAKE_CURRENT_SOURCE_DIR\}/([^)]*uwb_node\.cmake)\)", cmake)
    require(include is not None, f"{name}: does not use the shared build")
    require((node / include[1]).resolve() == (COMMON / "cmake/uwb_node.cmake").resolve(),
            f"{name}: shared build include does not resolve to common/cmake/uwb_node.cmake")

    # No node may keep its own copy of the sources or shared headers.
    require(not (node / "src").exists(), f"{name}: private src/ directory reappeared")
    strays = [path.name for path in (node / "include").iterdir()
              if path.name != "uwb_app_config.h"]
    require(not strays, f"{name}: private copies of shared headers: {strays}")


def validate_anchor_config(name: str, node: Path) -> None:
    config = read(node / "include/uwb_app_config.h")
    anchor_id = int(name.split("_")[-1])
    addresses = re.findall(r"^#define\s+ANCHOR_ADDR\s+\(\(uint16_t\)(\d+)U\)",
                           config, flags=re.MULTILINE)
    require(addresses == [str(anchor_id)],
            f"{name}: ANCHOR_ADDR is not uniquely set to {anchor_id}U")
    require("CONFIG_HWINFO=y" in read(node / "prj.conf"),
            f"{name}: needs CONFIG_HWINFO for the reset cause")


def validate_stm32_node(name: str) -> None:
    node = node_dir(name)
    for required in ("CMakeLists.txt", "prj.conf", "app.overlay", "include/uwb_app_config.h"):
        require((node / required).is_file(), f"{name}: missing {required}")
    validate_shared_build(name, node, "ANCHOR")


def validate_node(name: str, role: str, project: str) -> None:
    node = node_dir(name)
    require(node.is_dir(), f"missing project: {name}")

    for required in ("CMakeLists.txt", "prj.conf", "app.overlay",
                     "include/uwb_app_config.h", "scripts/build.ps1", "scripts/flash.ps1"):
        require((node / required).is_file(), f"{name}: missing {required}")

    cmake = read(node / "CMakeLists.txt")
    require(f"project({project})" in cmake, f"{name}: CMake project identity is wrong")
    validate_shared_build(name, node, role)

    # Node scripts are thin wrappers for the VS Code tasks: the project is
    # derived from the node directory and scripts/node.ps1 does the work
    # (it exports UWB_REPO_ROOT for the build id through Invoke-InUwbDrive).
    for script_name, action in (("scripts/build.ps1", "Build"), ("scripts/flash.ps1", "Flash")):
        script = read(node / script_name)
        require("-Project (Split-Path $nodeDir -Leaf)" in script
                and f"-Action {action}" in script and "scripts\\node.ps1" in script,
                f"{name}: {script_name} must forward to scripts/node.ps1 -Action {action}")
    require(not (node / "scripts/ncs_env.ps1").exists(),
            f"{name}: private ncs_env.ps1 copy reappeared (use scripts/ncs_env.ps1)")

    config = read(node / "include/uwb_app_config.h")
    if role == "ANCHOR":
        validate_anchor_config(name, node)
    if role == "TAG":
        require(re.search(r"^#define\s+TAG_ADDR\s+\(\(uint16_t\)0U\)", config, re.MULTILINE),
                f"{name}: TAG_ADDR must be 0")
        prj = read(node / "prj.conf")
        for option in ("CONFIG_SETTINGS=y", "CONFIG_NVS=y", "CONFIG_FLASH_MAP=y",
                       "CONFIG_HWINFO=y", "CONFIG_REBOOT=y"):
            require(option in prj, f"{name}: prj.conf is missing {option}")
        validate_tag_uart(name, node, config)


def validate_scripts() -> None:
    nodes_script = read(FIRMWARE_ROOT / "scripts/uwb_nodes.ps1")
    require("UWB_REPO_ROOT" in nodes_script,
            "scripts/uwb_nodes.ps1 must export UWB_REPO_ROOT for the build id")
    require(not list((FIRMWARE_ROOT / "nodes").rglob("ncs_env.ps1")),
            "use the single scripts/ncs_env.ps1")


def validate_tag_uart(name: str, node: Path, config: str) -> None:
    """The telemetry defaults must fit the UART the overlay configures.

    Below TELEM_RANGE_MEAS_MIN_BAUD the firmware drops a RANGE_MEAS default
    without an error, and above 115200 only the EasyDMA UARTE keeps the CPU
    out of one interrupt per byte.
    """
    overlay = read(node / "app.overlay")
    speeds = re.findall(r"current-speed\s*=\s*<(\d+)>", overlay)
    require(len(speeds) == 1, f"{name}: app.overlay must set exactly one UART current-speed")
    baud = int(speeds[0])

    telemetry = read(COMMON / "telemetry/telemetry.h")
    minimum = re.search(r"^#define\s+TELEM_RANGE_MEAS_MIN_BAUD\s+(\d+)U", telemetry, re.MULTILINE)
    require(minimum is not None, "telemetry.h: TELEM_RANGE_MEAS_MIN_BAUD not found")
    default = re.search(r"^#define\s+UWB_TELEM_DEFAULT_FEATURES((?:[^\n]*\\\n)*[^\n]*)",
                        config, re.MULTILINE)
    if default is not None and "TELEM_FEATURE_RANGE_MEAS" in default.group(1):
        require(baud >= int(minimum.group(1)),
                f"{name}: RANGE_MEAS is on by default but the UART runs at {baud} baud")
    if baud > 115200:
        require('compatible = "nordic,nrf-uarte";' in overlay,
                f"{name}: {baud} baud needs the EasyDMA UARTE (nordic,nrf-uarte)")


def validate_topology() -> None:
    header = read(COMMON / "ranging/tag_ranging.h")
    source = read(COMMON / "ranging/tag_ranging.c")
    calibration = read(COMMON / "config/uwb_calibration.h")

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
    tag_config = read(node_dir("Tag") / "include/uwb_app_config.h")
    require(re.search(r"^#define\s+UWB_DS_CALIBRATED_MASK\s+0U", tag_config, re.MULTILINE),
            "UWB_DS_CALIBRATED_MASK must stay 0 until the hardware is measured")


def main() -> int:
    validate_common()
    validate_registries()
    validate_scripts()
    for name, role, project in NODES:
        validate_node(name, role, project)
    for name in STM32_ANCHORS:
        validate_stm32_node(name)
    validate_topology()
    print(f"node project checks passed ({len(NODES) + len(STM32_ANCHORS)} projects, "
          "registries, shared sources, topology)")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
