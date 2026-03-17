# Purpose
This repository contains the project of a Lunar Lander Simulator, specifically focused on the touchdown phase (i.e. last meters of landing and lander-terrain interaction). 

# Requirements
Docker version 28.4.0, build d8eb465

# Environment set-up
Clone the repository (SSH address)
```
git clone git@github.com:Alessandro-Scarciglia/Touchdown-Simulator.git
```
Build the docker container (this step might take several minutes).
```
cd Touchdown-Simulator
docker compose build
```
Allow the container to employ display for visualization.
```
xhost +local:root
```

# Run the simulation
Start-up of the docker container (-d releases the terminal).
```
docker compose up -d
```
Get inside the container.
```
docker compose exec touchdown-sim bash
```
Launch the .world file in Gazebo.
```
gazebo src/worlds/landing_site.world
```
(Additional Info) If new terrain models (.stl) have been included in the CAD folder, they shall be copied inside the container. The placeholder <name> in the source path shall be substituted by the filename (e.g. QLSx.stl).
```
cp /root/CAD/<name>.stl /root/touchdown_ws/src/worlds/models/terrain_model/meshes/
```
