# Touchdown-Simulator
Build of docker container (in the folder where docker-compose.yml is located).
Add the flag --no-cache to build from scratch.
```
docker compose build
```
Start-up of the docker container (-d releases the terminal):
```
docker compose up -d
```
Allow the container to employ display for visualization:
```
xhost +local:root
```
Get inside the container:
```
docker compose exec touchdown bash
```
Copy a file.stl from host CAD folder to container mesh folder:
```
cp /root/CAD/<name>.stl /root/touchdown_ws/src/worlds/models/terrain_model/meshes/
```
