import os
import yaml
import trimesh


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if "install" in CURRENT_DIR:
    WS_ROOT = CURRENT_DIR.split("install")[0]
else:
    WS_ROOT = CURRENT_DIR.split("src")[0]

MESH_DIR = os.path.join(WS_ROOT, "src", "worlds", "models", "terrain_model", "meshes")
MESH_PATH = os.path.join(MESH_DIR, "terrain_col.stl")
OUTPUT_MESH = os.path.join(MESH_DIR, "terrain_col_light.stl")

POS_X_RANGE = [0.0, 0.0]
POS_Y_RANGE = [0.0, 0.0]
CROP_RADIUS = 100.0 

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