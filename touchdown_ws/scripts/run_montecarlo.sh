#!/usr/bin/env bash
# Run the configured number of sampled, headless simulations.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_FILE="${MC_CONFIG_FILE:-$WS_ROOT/cfg/multiple_simulations.yaml}"

export TOUCHDOWN_WS_ROOT="$WS_ROOT"
export TOUCHDOWN_CONFIG_FILE="$CONFIG_FILE"
export MC_CONFIG_FILE="$CONFIG_FILE"

source /opt/ros/humble/setup.bash
colcon build
source "$WS_ROOT/install/setup.bash"

python3 "$WS_ROOT/tools/experiments/run_monte_carlo.py"
