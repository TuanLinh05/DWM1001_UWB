#!/usr/bin/env bash
# Install the UP4000 flight logger on Ubuntu 22.04 + ROS 2 Humble.
# Run as the normal user (it calls sudo): bash UP7000/scripts/install.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$HERE")/ros2_ws"
USER_NAME="${SUDO_USER:-$USER}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble not found in /opt/ros/humble" >&2
  exit 1
fi

echo "== Packages"
sudo apt-get update
sudo apt-get install -y python3-serial python3-pip python3-colcon-common-extensions \
  ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-launch-ros
# Keep onboard dependencies on the internal disk even if ~/.local/lib is a
# symlink to a removable drive (as on the migrated UP7000).
python3 -m pip install --no-cache-dir --upgrade --target "$HOME/drone_tools/python" "pymavlink>=2.4.40"

echo "== Serial access and stable device names (/dev/uwb_tag, /dev/pixhawk)"
sudo usermod -aG dialout "$USER_NAME"
sudo install -m 644 "$HERE/99-uwb-px4.rules" /etc/udev/rules.d/99-uwb-px4.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "== Build"
set +u
source /opt/ros/humble/setup.bash
set -u
(cd "$WS" && colcon build --symlink-install)

echo "== systemd service (not enabled automatically)"
sed -e "s|@USER@|$USER_NAME|g" -e "s|@WS@|$WS|g" "$HERE/uwb-px4-logger.service" \
  | sudo tee /etc/systemd/system/uwb-px4-logger.service >/dev/null
if [ ! -f /etc/default/uwb-px4-logger ]; then
  echo 'LAUNCH_ARGS=""' | sudo tee /etc/default/uwb-px4-logger >/dev/null
fi
sudo systemctl daemon-reload

cat <<MSG

Done. Log out and in again once so the dialout group applies.
  Test in a terminal : bash $HERE/run.sh
  Start the service  : sudo systemctl start uwb-px4-logger
  Start at every boot: sudo systemctl enable uwb-px4-logger
  Follow its output  : journalctl -u uwb-px4-logger -f
Logs are written to ~/flight_logs/<date_time>/
MSG
