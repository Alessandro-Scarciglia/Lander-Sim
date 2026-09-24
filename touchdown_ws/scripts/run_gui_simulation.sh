#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_FILE="${TOUCHDOWN_CONFIG_FILE:-$WS_ROOT/cfg/simulation.yaml}"

export TOUCHDOWN_WS_ROOT="$WS_ROOT"
export TOUCHDOWN_CONFIG_FILE="$CONFIG_FILE"

source /opt/ros/humble/setup.bash
colcon build
source "$WS_ROOT/install/setup.bash"

ros2 launch "$WS_ROOT/launch/simulation.launch.py"