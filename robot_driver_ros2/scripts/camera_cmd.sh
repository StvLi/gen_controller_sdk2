#!/usr/bin/env bash
# Wrapper: requires `source install/setup.bash` (same semantics as ROS1 camera_cmd.sh).
set -euo pipefail
if ! command -v ros2 &>/dev/null; then
  echo "ros2 not found; source your ROS2 Foxy setup.bash and workspace install/setup.bash"
  exit 1
fi
exec ros2 run robot_driver_ros2 camera_calib_cli -- "$@"
