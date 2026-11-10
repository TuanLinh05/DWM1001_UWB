#!/usr/bin/env bash
# After copying new code: rebuild, refresh the systemd unit, restart the service.
# Run as the normal user (it calls sudo): bash UP7000/scripts/update_service.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$HERE")/ros2_ws"
USER_NAME="${SUDO_USER:-$USER}"
set +u
source /opt/ros/humble/setup.bash
set -u
(cd "$WS" && colcon build --symlink-install)
sed -e "s|@USER@|$USER_NAME|g" -e "s|@WS@|$WS|g" "$HERE/uwb-px4-logger.service" \
  | sudo tee /etc/systemd/system/uwb-px4-logger.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl restart uwb-px4-logger
sleep 3
systemctl --no-pager --lines=0 status uwb-px4-logger | head -4
journalctl -u uwb-px4-logger --since "-10s" --no-pager | grep -E "ERROR|WARNING|already|busy" || echo "no errors in the first seconds"
