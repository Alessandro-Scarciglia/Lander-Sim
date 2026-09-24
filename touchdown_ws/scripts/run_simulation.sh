#!/usr/bin/env bash
# Run one deterministic simulation using single_simulation.yaml.
# The configuration selects headless or interactive Gazebo mode.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_FILE="${TOUCHDOWN_CONFIG_FILE:-$WS_ROOT/cfg/single_simulation.yaml}"

export TOUCHDOWN_WS_ROOT="$WS_ROOT"
export TOUCHDOWN_CONFIG_FILE="$CONFIG_FILE"

source /opt/ros/humble/setup.bash
colcon build
source "$WS_ROOT/install/setup.bash"

HEADLESS="$(python3 -c 'import os, yaml; config = yaml.safe_load(open(os.environ["TOUCHDOWN_CONFIG_FILE"], "r")); print(str(config.get("simulation", {}).get("headless", True)).lower())')"

if [ "$HEADLESS" = "true" ]; then
  echo "[RUN] Single simulation mode: headless (automatic start)"
  ros2 launch "$WS_ROOT/launch/headless.launch.py"
else
  echo "[RUN] Single simulation mode: GUI (paused; start manually in Gazebo)"
  ros2 launch "$WS_ROOT/launch/simulation.launch.py"
fi
