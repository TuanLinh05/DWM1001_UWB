#!/usr/bin/env bash
# Show what the logger will find: stable names, by-id links and USB IDs.
echo "== Stable names"; ls -l /dev/uwb_tag /dev/pixhawk 2>/dev/null || echo "(none yet: install udev rules)"
echo "== /dev/serial/by-id"; ls -l /dev/serial/by-id/ 2>/dev/null || echo "(no USB serial devices)"
echo "== USB"; lsusb | grep -Ei "segger|1366|3185|26ac|3162|px4|auterion|holybro" || lsusb
