"""Crop a source terrain mesh to a configured rectangular analysis region."""

# File, configuration, and mesh imports
import os
import yaml
import trimesh


WS_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

MESH_DIR = os.path.join(WS_ROOT, "src", "worlds", "models", "terrain_model", "meshes")

CONFIG_PATH = os.path.join(WS_ROOT, "cfg", "simulation.yaml")
with open(CONFIG_PATH, "r") as file:
	config = yaml.safe_load(file)

terrain_cfg = config.get("terrain", {})
configured_mesh = terrain_cfg.get("mesh_file", "terrain_col.stl")
MESH_PATH = configured_mesh if os.path.isabs(configured_mesh) else os.path.join(MESH_DIR, configured_mesh)
OUTPUT_MESH = os.path.join(MESH_DIR, "terrain_analysis_crop.stl")

POS_X_RANGE = [0.0, 0.0]
POS_Y_RANGE = [0.0, 0.0]
CROP_RADIUS = terrain_cfg.get("crop_radius", 25.0)

min_x = POS_X_RANGE[0] - CROP_RADIUS
max_x = POS_X_RANGE[1] + CROP_RADIUS
min_y = POS_Y_RANGE[0] - CROP_RADIUS
max_y = POS_Y_RANGE[1] + CROP_RADIUS

print(f"Loading original mesh from: {MESH_PATH}...")
mesh = trimesh.load(MESH_PATH)

print(f"Cutting from: X[{min_x}, {max_x}], Y[{min_y}, {max_y}]...")

# Cut
mesh = mesh.slice_plane(plane_origin=[max_x, 0, 0], plane_normal=[-1, 0, 0])
mesh = mesh.slice_plane(plane_origin=[min_x, 0, 0], plane_normal=[1, 0, 0])
mesh = mesh.slice_plane(plane_origin=[0, max_y, 0], plane_normal=[0, -1, 0])
mesh = mesh.slice_plane(plane_origin=[0, min_y, 0], plane_normal=[0, 1, 0])

# Export
mesh.export(OUTPUT_MESH)
print(f"New mesh saved in: {OUTPUT_MESH}")