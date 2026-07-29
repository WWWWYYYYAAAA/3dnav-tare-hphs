#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "ROS setup file not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

cd "${ROOT_DIR}"
catkin_make
