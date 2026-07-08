import random
import os
import math
import numpy as np
import trimesh
import yaml


# Path definition

# Find the absolute path where this script is located
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
# Determine the workspace root based on where the code is running
if "install" in CURRENT_DIR:
    # If the file runs from install/lander_py/
    WS_ROOT = CURRENT_DIR.split("install")[0]
else:
    # If the file runs from src/lander_py/
    WS_ROOT = CURRENT_DIR.split("src")[0]
MESH_DIR = os.path.join(WS_ROOT, "src", "worlds", "models", "terrain_model", "meshes")
MESH_PATH = os.path.join(MESH_DIR, "moon_0.stl")
WORLD_DIR = os.path.join(WS_ROOT, "src", "worlds")
TEMPLATE_FILE = os.path.join(WORLD_DIR, "landing_scenario_template.world")
OUTPUT_WORLD = os.path.join(WORLD_DIR, "landing_scenario.world")
CONFIG_PATH = os.path.join(WS_ROOT, "config.yaml")

try:
    with open(CONFIG_PATH, 'r') as file:
        config = yaml.safe_load(file)
    print("Config.yaml file successfully loaded!")
except FileNotFoundError:
    print(f"Error: Unable to find the config.yaml file at the path: {CONFIG_PATH}")
    exit(1)

scenario_cfg = config["landing_scenario"]

# Randomization Parameters

# Position Range (m, degrees)
POS_X_RANGE = scenario_cfg["pos_x_range"]
POS_Y_RANGE = scenario_cfg["pos_y_range"]
ALTITUDE_RANGE = scenario_cfg["altitude_range"]  
ROLL_RANGE = (math.radians(scenario_cfg["roll_range_deg"][0]), math.radians(scenario_cfg["roll_range_deg"][1]))
PITCH_RANGE = (math.radians(scenario_cfg["pitch_range_deg"][0]), math.radians(scenario_cfg["pitch_range_deg"][1]))
YAW_RANGE = (math.radians(scenario_cfg["yaw_range_deg"][0]), math.radians(scenario_cfg["yaw_range_deg"][1]))

# Initial Linear & Angular Velocity Range (m/s, rad/s)
VEL_X_RANGE = scenario_cfg["vel_x_range"]
VEL_Y_RANGE = scenario_cfg["vel_y_range"]
VEL_Z_RANGE = scenario_cfg["vel_z_range"]
ANG_X_RANGE = scenario_cfg["ang_x_range"]
ANG_Y_RANGE = scenario_cfg["ang_y_range"]
ANG_Z_RANGE = scenario_cfg["ang_z_range"]

# Plugin parameters
kp_surface = scenario_cfg["kp_surface"]
kp_rock = scenario_cfg["kp_rock"]
kd_surface = scenario_cfg["kd_surface"]
kd_rock = scenario_cfg["kd_rock"]
dust_thickness = scenario_cfg["dust_thickness"]
mu_surface = scenario_cfg["mu_surface"]
mu_rock = scenario_cfg["mu_rock"]

# Mesh loading 
print(f"Loading mesh from: {MESH_PATH}...")
try:
    mesh = trimesh.load(MESH_PATH)
    print("Mesh loaded successfully!\n")
except FileNotFoundError:
    print(f"ERROR: Cannot find mesh at path {MESH_PATH}")
    exit(1)

def get_z_height(x, y, max_z=200.0):
    # Casts a ray downwards to find the terrain intersection
    ray_origins = np.array([[x, y, max_z]])
    ray_directions = np.array([[0, 0, -1]])    
    locations, index_ray, index_tri = mesh.ray.intersects_location(ray_origins=ray_origins, ray_directions=ray_directions)
    if len(locations) > 0:
        return np.max(locations[:, 2])
    return None

def generate_kinematics():
    # Extract Horizontal Position
    pos_x = random.uniform(*POS_X_RANGE)
    pos_y = random.uniform(*POS_Y_RANGE)
    
    # Calculate Z based on real terrain
    z_terrain = get_z_height(pos_x, pos_y)
    
    if z_terrain is None:
        print(f"WARNING: Point X={pos_x:.2f}, Y={pos_y:.2f} is outside the mesh! Using Terrain Z = 0.0")
        z_terrain = 0.0
        
    altitude = random.uniform(*ALTITUDE_RANGE)
    pos_z = z_terrain + altitude  
    
    # Extract Attitude and Velocity
    roll = random.uniform(*ROLL_RANGE)
    pitch = random.uniform(*PITCH_RANGE)
    yaw = random.uniform(*YAW_RANGE)
    vel_x = random.uniform(*VEL_X_RANGE)
    vel_y = random.uniform(*VEL_Y_RANGE)
    vel_z = random.uniform(*VEL_Z_RANGE)
    ang_x = random.uniform(*ANG_X_RANGE)
    ang_y = random.uniform(*ANG_Y_RANGE)
    ang_z = random.uniform(*ANG_Z_RANGE)

    print(f"--- Initial Scenario Generation ---")
    print(f"Position   : X={pos_x:.2f}, Y={pos_y:.2f}, Z={pos_z:.2f}")
    print(f"             (Measured Terrain Z: {z_terrain:.2f} m | Spawn altitude: {altitude:.2f} m)")
    print(f"Attitude   : R={math.degrees(roll):.2f}°, P={math.degrees(pitch):.2f}°, Y={math.degrees(yaw):.2f}°")
    print(f"Linear Vel : Vx={vel_x:.2f}, Vy={vel_y:.2f}, Vz={vel_z:.2f}")
    print(f"Angular Vel: Wx={ang_x:.3f}, Wy={ang_y:.3f}, Wz={ang_z:.3f}")
    
    # Write world file
    try:
        with open(TEMPLATE_FILE, "r") as file:
            template_data = file.read()
            
        new_world = template_data.replace("{POS_X}", f"{pos_x:.5f}")
        new_world = new_world.replace("{POS_Y}", f"{pos_y:.5f}")
        new_world = new_world.replace("{POS_Z}", f"{pos_z:.5f}")
        new_world = new_world.replace("{ROLL}", f"{roll:.5f}")
        new_world = new_world.replace("{PITCH}", f"{pitch:.5f}")
        new_world = new_world.replace("{YAW}", f"{yaw:.5f}")        
        new_world = new_world.replace("{VEL_X}", f"{vel_x:.5f}")
        new_world = new_world.replace("{VEL_Y}", f"{vel_y:.5f}")
        new_world = new_world.replace("{VEL_Z}", f"{vel_z:.5f}")
        new_world = new_world.replace("{ANG_X}", f"{ang_x:.5f}")
        new_world = new_world.replace("{ANG_Y}", f"{ang_y:.5f}")
        new_world = new_world.replace("{ANG_Z}", f"{ang_z:.5f}")
        new_world = new_world.replace("{KP_S}", f"{kp_surface:.5f}")
        new_world = new_world.replace("{KP_R}", f"{kp_rock:.5f}")
        new_world = new_world.replace("{KD_S}", f"{kd_surface:.5f}")
        new_world = new_world.replace("{KD_R}", f"{kd_rock:.5f}")
        new_world = new_world.replace("{MU_S}", f"{mu_surface:.5f}")
        new_world = new_world.replace("{MU_R}", f"{mu_rock:.5f}")
        new_world = new_world.replace("{THICK}", f"{dust_thickness:.5f}")

        with open(OUTPUT_WORLD, "w") as file:
            file.write(new_world)
        print(f"\n[OK] File '{OUTPUT_WORLD}' successfully generated.")
        
    except FileNotFoundError:
        print(f"\nERROR: Cannot find template file: {TEMPLATE_FILE}")


if __name__ == "__main__":
    generate_kinematics()