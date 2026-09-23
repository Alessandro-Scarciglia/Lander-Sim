#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BASE_CONFIG_FILE="${TOUCHDOWN_BASE_CONFIG_FILE:-$WS_ROOT/cfg/experiment_base.yaml}"
CONFIG_FILE="${TOUCHDOWN_CONFIG_FILE:-$WS_ROOT/cfg/simulation.yaml}"

export TOUCHDOWN_WS_ROOT="$WS_ROOT"
export TOUCHDOWN_BASE_CONFIG_FILE="$BASE_CONFIG_FILE"
export TOUCHDOWN_CONFIG_FILE="$CONFIG_FILE"

source /opt/ros/humble/setup.bash
colcon build
source "$WS_ROOT/install/setup.bash"

python3 "$WS_ROOT/tools/experiments/run_morris.py"
