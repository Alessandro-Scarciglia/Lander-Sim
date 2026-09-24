# Touchdown Simulator

This repository contains a ROS 2 and Gazebo lunar lander touchdown simulator.
The instructions below cover simulation runs that produce CSV output. Analysis
and plotting commands are intentionally not included.

## Requirements

- Docker Desktop with Docker Engine `28.4.0`.
- Docker Compose.
- An X11 server if Gazebo GUI output is required. Headless runs do not require
  GUI forwarding.

Verify Docker before starting:

```bash
docker version
docker compose version
```

## Start the container

Run these commands from the repository root:

```bash
docker compose build
docker compose up -d
docker compose exec touchdown-sim bash
```

Inside the container, build the mounted ROS workspace. The source bind mount
replaces the image copy of the workspace, so this build is required after a
fresh checkout or source change:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/touchdown_ws
colcon build
source install/setup.bash
```

The files in `touchdown_ws/scripts/` are Linux shell scripts for the ROS
container. Do not run them directly from Windows PowerShell. From the Windows
repository root, run them through Docker with `docker compose exec`.

The repository uses the following simulation folders:

```text
touchdown_ws/
├── cfg/
│   ├── simulation.yaml
│   ├── simulation_backup.yaml
│   └── experiment_base.yaml
├── scripts/
│   ├── run_single_simulation.sh
│   ├── run_monte_carlo.sh
│   ├── run_morris.sh
│   └── run_sobol.sh
└── data/results/
    └── montecarlo_summary.csv
```

## Single simulation

The single-run script uses `cfg/simulation.yaml` and launches the headless
simulation. It generates one simulation result in
`data/results/montecarlo_summary.csv`.

```bash
cd /workspace/touchdown_ws
bash scripts/run_single_simulation.sh
```

Equivalent command from Windows PowerShell:

```powershell
docker compose exec touchdown-sim bash -lc "bash /workspace/touchdown_ws/scripts/run_single_simulation.sh"
```

The GUI launch file is available when display forwarding is configured:

```bash
ros2 launch /workspace/touchdown_ws/launch/simulation.launch.py
```

For Docker Desktop on Windows, install and start VcXsrv or Xming with display
number `0` and access control disabled, then run this from the repository root:

```powershell
docker compose build
docker compose up -d
docker compose exec touchdown-sim bash -lc "bash /workspace/touchdown_ws/scripts/run_gui_simulation.sh"
```

Compose defaults `DISPLAY` to `host.docker.internal:0.0`, so PowerShell does
not need a manually configured `DISPLAY` variable. The GUI launch starts the
simulation immediately; Gazebo's Play button is not required.

## Monte Carlo simulation

The Monte Carlo script uses `cfg/simulation.yaml`, runs the configured number
of independent simulations, and appends one CSV row per completed run.

```bash
cd /workspace/touchdown_ws
bash scripts/run_monte_carlo.sh
```

From Windows PowerShell:

```powershell
docker compose exec touchdown-sim bash -lc "bash /workspace/touchdown_ws/scripts/run_monte_carlo.sh"
```

Output:

```text
/workspace/touchdown_ws/data/results/montecarlo_summary.csv
```

## Morris simulation

The Morris script uses:

- `cfg/experiment_base.yaml` as the sampling baseline.
- `cfg/simulation.yaml` as the per-run working configuration.

It generates the Morris samples and executes the required simulations
sequentially.

```bash
cd /workspace/touchdown_ws
bash scripts/run_morris.sh
```

From Windows PowerShell:

```powershell
docker compose exec touchdown-sim bash -lc "bash /workspace/touchdown_ws/scripts/run_morris.sh"
```

The simulation rows are written to:

```text
/workspace/touchdown_ws/data/results/montecarlo_summary.csv
```

## Sobol simulation

The Sobol script uses:

- `cfg/experiment_base.yaml` as the sampling baseline.
- `cfg/simulation.yaml` as the per-run working configuration.

It generates the Sobol samples and executes the required simulations
sequentially.

```bash
cd /workspace/touchdown_ws
bash scripts/run_sobol.sh
```

From Windows PowerShell:

```powershell
docker compose exec touchdown-sim bash -lc "bash /workspace/touchdown_ws/scripts/run_sobol.sh"
```

The simulation rows are written to:

```text
/workspace/touchdown_ws/data/results/montecarlo_summary.csv
```

## Configuration selection

Each wrapper exports the configuration path explicitly through
`TOUCHDOWN_CONFIG_FILE`. The ROS nodes and generators use that path, so a
different checked-in configuration can be tested without changing source
code:

```bash
export TOUCHDOWN_CONFIG_FILE=/workspace/touchdown_ws/cfg/simulation_backup.yaml
bash scripts/run_single_simulation.sh
```

Experiment runners additionally use `TOUCHDOWN_BASE_CONFIG_FILE` for their
sampling baseline. Keep Monte Carlo, Morris, and Sobol runs sequential because
they update the shared working configuration and CSV output.
