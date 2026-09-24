# Touchdown Simulator

Touchdown Simulator is a ROS 2 and Gazebo Classic simulation of a lunar lander touchdown. It combines a configurable lander model, deterministic or randomized initial conditions, landing-leg shock absorbers, terrain contact behavior, and data logging.

The project is intended for two main workflows:

1. **Single simulation:** one reproducible scenario using fixed values from `single_simulation.yaml`.
2. **Monte Carlo simulation:** many independent scenarios sampled from ranges in `multiple_simulations.yaml`.

The simulation uses lunar gravity (`1.625 m/s^2`) and Gazebo's ODE physics engine.

## How It Works

Before Gazebo starts, the launch process generates the model files and landing world:

```text
Configuration YAML
        |
        +--> generate_body.py      -> body.sdf, passenger.sdf
        +--> generate_legs.py      -> leg1.sdf ... leg4.sdf, lander.sdf
        +--> generate_scenario.py  -> landing_scenario.world, terrain_cropped.stl
        |
        v
Gazebo + ROS 2 nodes
        |
        +--> thrusters and disturbances
        +--> shock absorber logging
        +--> telemetry and touchdown results
        |
        v
touchdown_ws/data/results/
```

### Main folders

```text
Touchdown-Simulator/
├── Dockerfile                         Container definition
├── docker-compose.yml                 Container and volume configuration
├── CAD/                               CAD and source geometry
├── README.md
└── touchdown_ws/
    ├── cfg/                           User-facing YAML configurations
    ├── launch/                        ROS 2 launch files
    ├── scripts/                       Main shell entry points
    ├── src/
    │   ├── lander_py/                 Python generators, controllers, loggers
    │   ├── lander_sim/                C++ Gazebo plugins
    │   └── worlds/                    SDF models, terrain, world templates
    ├── tools/experiments/             Monte Carlo and sensitivity runners
    ├── data/results/                  CSV simulation results
    └── figures/                       Generated plots and dashboards
```

## Requirements

The recommended environment is Docker because the project depends on ROS 2 Humble, Gazebo Classic, Python packages, and Linux shell scripts.

- Docker Desktop with Docker Compose
- A working Docker Engine
- An X server for GUI mode on Windows, such as VcXsrv or Xming
- Git, for cloning the repository

Verify Docker from PowerShell:

```powershell
docker version
docker compose version
```

Headless simulations do not require an X server. GUI simulations require display forwarding from the container to the host.

## Clone And Build

Clone the repository and enter it:

```bash
git clone <repository-url>
cd Touchdown-Simulator
```

Build and start the container:

```bash
docker compose build
docker compose up -d
```

Open a shell inside the container:

```bash
docker compose exec touchdown-sim bash
```

Inside the container, source ROS 2 and build the mounted workspace:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/touchdown_ws
colcon build
source install/setup.bash
```

The runner scripts build the workspace automatically as well, so this manual build is mainly useful after a fresh checkout or when diagnosing build problems.

The files in `touchdown_ws/scripts/` are Linux shell scripts for the ROS container. Run them inside the container or through `docker compose exec`, not directly from Windows PowerShell.

## Configuration

### Single simulation

Edit [single_simulation.yaml](touchdown_ws/cfg/single_simulation.yaml). Its scenario values are punctual values, not ranges:

```yaml
simulation:
  headless: true

landing_scenario:
  pos_x: 0.0
  pos_y: 0.0
  altitude: 0.255
  roll_deg: 0.0
  pitch_deg: 0.0
  vel_z: -1.1
```

Set `headless` as follows:

```yaml
headless: true   # automatic simulation, no GUI
headless: false  # Gazebo GUI opens paused
```

With `headless: false`, press Gazebo's Play button to start physics manually.

### Monte Carlo simulation

Edit [multiple_simulations.yaml](touchdown_ws/cfg/multiple_simulations.yaml). Its fields use ranges, and one scenario is sampled for each run:

```yaml
monte_carlo:
  num_runs: 5

landing_scenario:
  vel_z_range: [-2.0, -0.2]
  roll_range_deg: [-5.0, 5.0]
```

The Monte Carlo runner is always headless and starts physics automatically. Change `num_runs` to control how many simulations are scheduled.

### Terrain selection

Both configurations select the terrain with the same simple section:

```yaml
terrain:
  mesh_file: terrain_vis_light.stl
  crop_radius: 25.0
```

The STL must be located in:

```text
touchdown_ws/src/worlds/models/terrain_model/meshes/
```

For each run, `generate_scenario.py` crops the selected mesh around the landing area and writes `terrain_cropped.stl`, which is then loaded by Gazebo.

## Run One Simulation

From inside the container:

```bash
cd /workspace/touchdown_ws
bash scripts/run_simulation.sh
```

The legacy command remains available:

```bash
bash scripts/run_single_simulation.sh
```

It delegates to `run_simulation.sh`.

From Windows PowerShell, run the same workflow through Docker:

```powershell
docker compose exec touchdown-sim bash -lc "bash /workspace/touchdown_ws/scripts/run_simulation.sh"
```

### Single-run behavior

With `headless: true`:

- Gazebo runs without the GUI.
- Physics starts automatically.
- The command waits until the simulation and logger finish.

With `headless: false`:

- Gazebo GUI opens.
- Physics starts paused.
- Press the Play button to begin the simulation.

### Single-run output

Generated model and world files are written under:

```text
touchdown_ws/src/worlds/models/lander_model/
touchdown_ws/src/worlds/landing_scenario.world
touchdown_ws/src/worlds/models/terrain_model/meshes/terrain_cropped.stl
```

Telemetry and touchdown results are written under:

```text
touchdown_ws/data/results/
```

The single-run CSV is named with its timestamp:

```text
single_run_YYYYMMDD_HHMMSS.csv
```

Generated figures are stored under `touchdown_ws/figures/` when enabled.

## Run Monte Carlo

From inside the container:

```bash
cd /workspace/touchdown_ws
bash scripts/run_montecarlo.sh
```

From Windows PowerShell:

```powershell
docker compose exec touchdown-sim bash -lc "bash /workspace/touchdown_ws/scripts/run_montecarlo.sh"
```

The compatibility command with an underscore is also available:

```bash
bash scripts/run_monte_carlo.sh
```

The runner reads `monte_carlo.num_runs` from [multiple_simulations.yaml](touchdown_ws/cfg/multiple_simulations.yaml), assigns a reproducible run ID to each simulation, and executes the runs sequentially.

### Monte Carlo output

The console reports progress in this form:

```text
[RUN 1/5] Starting simulation cycle...
[RUN 1] Execution completed in 42.3 seconds.
...
MONTE CARLO ORCHESTRATOR COMPLETED!
Total scheduled runs: 5
```

The complete Monte Carlo batch writes one timestamped CSV. Each simulation is
one row in that file:

```text
montecarlo_run_5_YYYYMMDD_HHMMSS.csv
...
```

Additional generated logs and figures remain in `data/` and `figures/`.

## Useful Launch Files

These launch files are normally called by the runners:

- `launch/headless.launch.py`: Gazebo server without GUI, physics running automatically.
- `launch/simulation.launch.py`: Gazebo GUI with physics initially paused.

To run a launch file directly, first export the desired configuration:

```bash
export TOUCHDOWN_WS_ROOT=/workspace/touchdown_ws
export TOUCHDOWN_CONFIG_FILE=/workspace/touchdown_ws/cfg/single_simulation.yaml
source /opt/ros/humble/setup.bash
source /workspace/touchdown_ws/install/setup.bash
ros2 launch /workspace/touchdown_ws/launch/simulation.launch.py
```

## Troubleshooting

### The selected terrain is not found

Check the filename and location:

```bash
ls /workspace/touchdown_ws/src/worlds/models/terrain_model/meshes/*.stl
grep -A3 '^terrain:' /workspace/touchdown_ws/cfg/single_simulation.yaml
```

### The GUI does not open

Confirm that an X server is running on the host and that Docker Compose exposes the display:

```bash
echo "$DISPLAY"
```

The default Compose value is `host.docker.internal:0.0`. On Windows, the X server must allow connections from Docker.

### A configuration change is not visible

Rebuild and source the workspace:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/touchdown_ws
colcon build
source install/setup.bash
```

The runner scripts perform this build automatically before launching.

## Other Experiment Runners

The project also contains scripts for Morris and Sobol sensitivity experiments:

```bash
bash scripts/run_morris.sh
bash scripts/run_sobol.sh
```

These workflows modify shared configuration and result files. Run them sequentially, not in parallel with Monte Carlo or another simulation.
