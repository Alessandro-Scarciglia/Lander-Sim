import random
import os
import math
import numpy as np
import trimesh
import yaml

run_id = os.environ.get('SIMULATION_RUN_ID')
if run_id is not None:
    random.seed(int(run_id))

# Path definition
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if "install" in CURRENT_DIR:
    WS_ROOT = CURRENT_DIR.split("install")[0]
else:
    WS_ROOT = CURRENT_DIR.split("src")[0]

MESH_DIR = os.path.join(WS_ROOT, "src", "worlds", "models", "terrain_model", "meshes")
MESH_PATH = os.path.join(MESH_DIR, "terrain_col_light.stl")
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
leg_cfg = config.get("leg_generation", {})

# Randomization Parameters
POS_X_RANGE = scenario_cfg["pos_x_range"]
POS_Y_RANGE = scenario_cfg["pos_y_range"]
ALTITUDE_RANGE = scenario_cfg["altitude_range"]  
ROLL_RANGE = (math.radians(scenario_cfg["roll_range_deg"][0]), math.radians(scenario_cfg["roll_range_deg"][1]))
PITCH_RANGE = (math.radians(scenario_cfg["pitch_range_deg"][0]), math.radians(scenario_cfg["pitch_range_deg"][1]))
YAW_RANGE = (math.radians(scenario_cfg["yaw_range_deg"][0]), math.radians(scenario_cfg["yaw_range_deg"][1]))

VEL_X_RANGE = scenario_cfg["vel_x_range"]
VEL_Y_RANGE = scenario_cfg["vel_y_range"]
VEL_Z_RANGE = scenario_cfg["vel_z_range"]
ANG_X_RANGE = scenario_cfg["ang_x_range"]
ANG_Y_RANGE = scenario_cfg["ang_y_range"]
ANG_Z_RANGE = scenario_cfg["ang_z_range"]

CROP_RADIUS = 25.0

# Plugin parameters
kp_surface = scenario_cfg["kp_surface"]
kp_rock = scenario_cfg["kp_rock"]
kd_surface = scenario_cfg["kd_surface"]
kd_rock = scenario_cfg["kd_rock"]
dust_thickness = scenario_cfg["dust_thickness"]
mu_surface = scenario_cfg["mu_surface"]
mu_rock = scenario_cfg["mu_rock"]

# Foot coordinate extraction
z_legs = leg_cfg.get("z_legs", 0.25)
point_F = leg_cfg.get("point_F", [1.0, 0.5, -0.8])
r_octagon = leg_cfg.get("octagon_radius", 2.0)
c1 = leg_cfg.get("base_points", {}).get("C1", [0.0, 0.0, 0.0])
c2 = leg_cfg.get("base_points", {}).get("C2", [0.0, 1.0, 0.0])

dist_c1_c2 = math.sqrt((c2[0]-c1[0])**2 + (c2[1]-c1[1])**2 + (c2[2]-c1[2])**2)

x_leg_base = r_octagon
y_leg_base = -dist_c1_c2 / 2.0
x_foot_local = point_F[0]
y_foot_local = point_F[1]

feet_offsets_local = []
for i in range(4):
    yaw_leg = i * (math.pi / 2.0)
    xl_g = x_leg_base * math.cos(yaw_leg) - y_leg_base * math.sin(yaw_leg)
    yl_g = x_leg_base * math.sin(yaw_leg) + y_leg_base * math.cos(yaw_leg)
    xf_g = x_foot_local * math.cos(yaw_leg) - y_foot_local * math.sin(yaw_leg)
    yf_g = x_foot_local * math.sin(yaw_leg) + y_foot_local * math.cos(yaw_leg)
    feet_offsets_local.append((xl_g + xf_g, yl_g + yf_g))

z_foot_joint_local = z_legs + point_F[2]


print(f"Loading mesh from: {MESH_PATH}...")
try:
    mesh = trimesh.load(MESH_PATH)
    print("Mesh loaded successfully!\n")
except FileNotFoundError:
    print(f"ERROR: Cannot find mesh at path {MESH_PATH}")
    exit(1)

def get_z_height(x, y, max_z=200.0):
    ray_origins = np.array([[x, y, max_z]])
    ray_directions = np.array([[0, 0, -1]])    
    locations, index_ray, index_tri = mesh.ray.intersects_location(ray_origins=ray_origins, ray_directions=ray_directions)
    if len(locations) > 0:
        return np.max(locations[:, 2])
    return None

def get_rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    r11, r12, r13 = cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy
    r21, r22, r23 = cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy
    r31, r32, r33 = -sp,   sr*cp,            cr*cp
    return (r11, r12, r13, r21, r22, r23, r31, r32, r33)

def rotate_point(x, y, z, R_mat):
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = R_mat
    return (x*r11 + y*r12 + z*r13, x*r21 + y*r22 + z*r23, x*r31 + y*r32 + z*r33)

def generate_kinematics():
    pos_x = random.uniform(*POS_X_RANGE)
    pos_y = random.uniform(*POS_Y_RANGE)

    # Mesh cutting
    print(f"\nCropping the mesh within a {CROP_RADIUS}m radius around X={pos_x:.2f}, Y={pos_y:.2f}...")   
    
    cropped_mesh = mesh.copy()
    
    # Cuts
    cropped_mesh = cropped_mesh.slice_plane(plane_origin=[pos_x + CROP_RADIUS, 0, 0], plane_normal=[-1, 0, 0])
    cropped_mesh = cropped_mesh.slice_plane(plane_origin=[pos_x - CROP_RADIUS, 0, 0], plane_normal=[1, 0, 0])    
    cropped_mesh = cropped_mesh.slice_plane(plane_origin=[0, pos_y + CROP_RADIUS, 0], plane_normal=[0, -1, 0])
    cropped_mesh = cropped_mesh.slice_plane(plane_origin=[0, pos_y - CROP_RADIUS, 0], plane_normal=[0, 1, 0])
    
    # New mesh
    CROPPED_MESH_PATH = os.path.join(MESH_DIR, "terrain_cropped.stl")
    cropped_mesh.export(CROPPED_MESH_PATH)
    print(f"Mesh successfully cropped and saved to: {CROPPED_MESH_PATH}\n")
    
    roll = random.uniform(*ROLL_RANGE)
    pitch = random.uniform(*PITCH_RANGE)
    yaw = random.uniform(*YAW_RANGE)

    # Rotational matrix
    R_mat = get_rotation_matrix(roll, pitch, yaw)

    # Calculation of the footpad edge extension
    tilt_cos = math.cos(roll) * math.cos(pitch)
    tilt_sin = math.sqrt(1.0 - tilt_cos**2) if tilt_cos < 1.0 else 0.0
    footpad_radius = 0.25      
    footpad_half_height = 0.025    
    foot_disc_drop = -(footpad_half_height * tilt_cos + footpad_radius * tilt_sin)
    
    max_required_pos_z = -9999.0 
    
    for dx, dy in feet_offsets_local:
        # Apply the 3D rotation
        rx, ry, rz = rotate_point(dx, dy, z_foot_joint_local, R_mat)
        
        # New global coordinate of the foot
        foot_x_global = pos_x + rx
        foot_y_global = pos_y + ry
        
        # Find the lowest point of the foot
        z_foot_lowest_rotated = rz + foot_disc_drop
        
        # Find the height of the mesh under the foot
        z_foot_terrain = get_z_height(foot_x_global, foot_y_global)
        if z_foot_terrain is None:
            z_foot_terrain = 0.0  
            
        # Required body Z so that the foot's edge exactly touches the ground
        required_pos_z = z_foot_terrain - z_foot_lowest_rotated
        
        # Update the global maximum Z requirement
        if required_pos_z > max_required_pos_z:
            max_required_pos_z = required_pos_z

    if max_required_pos_z == -9999.0:
        max_required_pos_z = 0.0 

    
    altitude = random.uniform(*ALTITUDE_RANGE)
    pos_z = max_required_pos_z + altitude 
    
    vel_x = random.uniform(*VEL_X_RANGE)
    vel_y = random.uniform(*VEL_Y_RANGE)
    vel_z = random.uniform(*VEL_Z_RANGE)
    ang_x = random.uniform(*ANG_X_RANGE)
    ang_y = random.uniform(*ANG_Y_RANGE)
    ang_z = random.uniform(*ANG_Z_RANGE)

    print(f"--- Initial Scenario Generation ---")
    print(f"Minimum Safe Body Z (0 altitude) : {max_required_pos_z:.3f} m")
    print(f"Spawn Altitude added             : {altitude:.3f} m")
    print(f"Position   : X={pos_x:.2f}, Y={pos_y:.2f}, Z={pos_z:.2f}")
    print(f"Attitude   : R={math.degrees(roll):.2f}°, P={math.degrees(pitch):.2f}°, Y={math.degrees(yaw):.2f}°")
    print(f"Linear Vel : Vx={vel_x:.2f}, Vy={vel_y:.2f}, Vz={vel_z:.2f}")
    print(f"Angular Vel: Wx={ang_x:.3f}, Wy={ang_y:.3f}, Wz={ang_z:.3f}")
    
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