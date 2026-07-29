#!/usr/bin/env bash
set -euo pipefail

patterns=(
  "/opt/ros/noetic/bin/roslaunch"
  "/opt/ros/noetic/bin/rosmaster"
  "roscore"
  "gzserver"
  "gzclient"
  "spawn_model"
  "controller_manager/spawner"
  "a1_rl_policy_driver.py"
  "standing_a1_driver.py"
  "cmu_a1_bridge.py"
)

echo "Stopping ROS/Gazebo processes in this container..."

for pattern in "${patterns[@]}"; do
  pkill -INT -f "${pattern}" 2>/dev/null || true
done

sleep 2

for pattern in "${patterns[@]}"; do
  pkill -TERM -f "${pattern}" 2>/dev/null || true
done

sleep 1

for pattern in "${patterns[@]}"; do
  pkill -KILL -f "${pattern}" 2>/dev/null || true
done

echo "Remaining matching processes:"
pgrep -af "roslaunch|rosmaster|roscore|gzserver|gzclient|spawn_model|controller_manager/spawner|a1_rl_policy_driver|standing_a1_driver|cmu_a1_bridge" || true
echo "Cleanup done."
