"""Firmware directory layout for the Python tools and tests.

Mirror of scripts/uwb_nodes.ps1 (tests/test_node_projects.py checks that the
two registries agree). Import it instead of hard-coding node paths:

    from uwb_layout import FIRMWARE, node_dir
    config = node_dir("Anchor_1") / "include/uwb_app_config.h"
"""

from __future__ import annotations

from pathlib import Path

FIRMWARE = Path(__file__).resolve().parents[1]
COMMON = FIRMWARE / "common"
RELEASE = FIRMWARE / "release"

# Shared source modules under common/ (each holds its .c and .h files).
COMMON_MODULES = ("app", "drivers", "platform", "ranging", "filters",
                  "telemetry", "services", "config")

NODE_PATHS: dict[str, str] = {
    "Tag": "nodes/tag/Tag",
    "Tag_DevKit": "nodes/tag/Tag_DevKit",
    "Sniffer_DevKit": "nodes/sniffer/Sniffer_DevKit",
    **{f"Anchor_{n}": f"nodes/anchors/nrf/Anchor_{n}" for n in range(1, 9)},
    **{f"STM32_Anchor_{n}": f"nodes/anchors/stm32/STM32_Anchor_{n}" for n in range(5, 9)},
}


def node_dir(name: str, firmware: Path = FIRMWARE) -> Path:
    """Absolute directory of a node project, e.g. node_dir("Tag_DevKit")."""
    try:
        return firmware / NODE_PATHS[name]
    except KeyError:
        raise KeyError(f"unknown node {name!r}; known: {', '.join(NODE_PATHS)}") from None
