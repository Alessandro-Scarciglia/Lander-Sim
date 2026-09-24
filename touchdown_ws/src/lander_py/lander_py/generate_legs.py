"""Generate the four landing-leg SDF models and the assembled lander model.

The generator converts configured leg geometry and absorber parameters into
link poses, inertias, joints, and Gazebo plugin settings before writing the
individual leg files and their parent lander SDF.
"""

# Geometry, configuration, and numerical imports
import math
import os
import yaml
from lander_py.paths import simulation_config_path, workspace_root
import random
import numpy as np

# Workspace paths and template locations
WS_ROOT = str(workspace_root())

# Base directory
MODELS_DIR = os.path.join(WS_ROOT, "src", "worlds", "models", "lander_model")

LEG_TEMPLATE_FILE = os.path.join(MODELS_DIR, "leg_template.sdf")
LANDER_TEMPLATE_FILE = os.path.join(MODELS_DIR, "lander_template.sdf")
LANDER_OUTPUT_FILE = os.path.join(MODELS_DIR, "lander.sdf")
CONFIG_PATH = str(simulation_config_path())

def steiner_transport(Icg, mass, old_pos, new_pos):    
    d = np.array(old_pos) - np.array(new_pos)
    return np.array(Icg) + mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

def rotate_inertia(I, R):    
    return R @ I @ R.T

def get_rotation_matrix(pitch, yaw):
    # Roll = 0
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry

def calculate_cylinder_inertia(mass, radius, length):    
    ixx = (1.0 / 12.0) * mass * (3 * radius**2 + length**2)
    izz = 0.5 * mass * radius**2
    return ixx, ixx, izz 

def calculate_sphere_inertia(mass, radius):    
    i = (2.0 / 5.0) * mass * (radius ** 2)
    return i, i, i

def calculate_link_kinematics(p_start, p_end, mass, radius):
    vx = p_end[0] - p_start[0]
    vy = p_end[1] - p_start[1]
    vz = p_end[2] - p_start[2]
    L = math.sqrt(vx**2 + vy**2 + vz**2)
    cx = (p_start[0] + p_end[0]) / 2.0
    cy = (p_start[1] + p_end[1]) / 2.0
    cz = (p_start[2] + p_end[2]) / 2.0
    yaw = math.atan2(vy, vx)
    pitch = math.acos(vz / L) if L != 0 else 0
    ixx, iyy, izz = calculate_cylinder_inertia(mass, radius, L)

    R = get_rotation_matrix(pitch, yaw)
    I_local = np.diag([ixx, iyy, izz])
    I_rot = rotate_inertia(I_local, R)
    
    return {
        "L": L, "pose_link": f"{cx:.4f} {cy:.4f} {cz:.4f} 0.0000 {pitch:.4f} {yaw:.4f}",
        "ixx": ixx, "iyy": iyy, "izz": izz,
        "cog": np.array([cx, cy, cz]),
        "I": I_rot
    }

def calculate_piston_kinematics(p_start, p_end, mass_strut, rad_strut, mass_piston, rad_piston):
    vx, vy, vz = p_start[0] - p_end[0], p_start[1] - p_end[1], p_start[2] - p_end[2]
    L_total = math.sqrt(vx**2 + vy**2 + vz**2)
    yaw = math.atan2(vy, vx)
    pitch = math.acos(vz / L_total) if L_total != 0 else 0
    L_half = L_total * 0.5

    v = np.array(p_end) - np.array(p_start)
    cog_strut = np.array(p_start) + 0.25 * v
    cog_piston = np.array(p_start) + 0.75 * v
    
    ixx_s, iyy_s, izz_s = calculate_cylinder_inertia(mass_strut, rad_strut, L_half)
    ixx_p, iyy_p, izz_p = calculate_cylinder_inertia(mass_piston, rad_piston, L_half)

    R = get_rotation_matrix(pitch, yaw)
    I_s_rot = rotate_inertia(np.diag([ixx_s, iyy_s, izz_s]), R)
    I_p_rot = rotate_inertia(np.diag([ixx_p, iyy_p, izz_p]), R)

    return {
        "pose": f"{p_start[0]:.4f} {p_start[1]:.4f} {p_start[2]:.4f} 0.0000 {pitch:.4f} {yaw:.4f}",
        "pitch": pitch, "L_half": L_half, "vis_z": -L_half / 2.0, "rel_z": -L_half,
        "strut": {"mass": mass_strut, "rad": rad_strut, "ixx": ixx_s, "iyy": iyy_s, "izz": izz_s, "cog": cog_strut, "I": I_s_rot},
        "piston": {"mass": mass_piston, "rad": rad_piston, "ixx": ixx_p, "iyy": iyy_p, "izz": izz_p, "cog": cog_piston, "I": I_p_rot}
    }

def generate_legs(template_path, output_dir, config_parameters=None):
    # Default parameters
    cfg = {
        "truss_mass": 25.0, "truss_radius": 0.04,
        "point_D": (0.6, 0.5, 0.3), "point_F": (1.0, 0.5, -0.8),
        "point_B1": (0.0, -0.2, 0.3), "point_B2": (0.0, 1.2, 0.3),
        "base_points": {
            "C1": (0.0, 0.0, 0.0), "C2": (0.0, 1.0, 0.0),
            "A1": (0.0, 0.3, 1.0), "A2": (0.0, 0.7, 1.0)
        },
        "strut_DF_mass": 25.0, "strut_DF_radius": 0.08, "strut_DF_L": 0.6,
        "piston_DF_mass": 20.0, "piston_DF_radius": 0.05, "piston_DF_L": 0.7,
        "b_strut_mass": 15.0, "b_strut_radius": 0.05,
        "b_piston_mass": 10.0, "b_piston_radius": 0.035,
        "df_k": 100000.0, "df_c": 18000.0, "df_fric": 5000.0, "df_limit": 0.45,
        "b_k": 100000.0,  "b_c": 1000.0, "b_fric": 3000.0, "b_limit": 0.4,
        "foot_k": 100.0,  "foot_c": 10.0, "df_crush_force_stage1": 12000.0, "df_rebound_damping": 20000.0,
        "df_crush_force_stage2": 30000.0, "df_stroke_limit_stage_1": 0.254, "df_stroke_limit_stage_2": 0.8128,
        "be_crush_force": 5000.0, "be_rebound_damping": 20000.0, "df_bottomout_stiffness": 10000000,
        "mass_D": 10.0, "mass_E": 10.0, "mass_foot": 5.0
    }
    if config_parameters:
        cfg.update(config_parameters)

    try:
        with open(template_path, "r") as file:
            template_sdf = file.read()
    except FileNotFoundError:
        print(f"Error: Template file {template_path} not found!")
        return cfg 

    t_E = (0.0 - cfg["point_D"][2]) / (cfg["point_F"][2] - cfg["point_D"][2])
    point_E = (
        cfg["point_D"][0] + t_E * (cfg["point_F"][0] - cfg["point_D"][0]),
        cfg["point_D"][1] + t_E * (cfg["point_F"][1] - cfg["point_D"][1]), 0.0
    )

    trusses = {}
    for name, coord in cfg["base_points"].items():
        trusses[name] = calculate_link_kinematics(coord, cfg["point_D"], cfg["truss_mass"], cfg["truss_radius"])

    pistons = {
        "DF": calculate_piston_kinematics(
            cfg["point_D"], cfg["point_F"], 
            cfg["strut_DF_mass"], cfg["strut_DF_radius"], cfg["piston_DF_mass"], cfg["piston_DF_radius"]
        ),
        "B1": calculate_piston_kinematics(
            cfg["point_B1"], point_E, 
            cfg["b_strut_mass"], cfg["b_strut_radius"], cfg["b_piston_mass"], cfg["b_piston_radius"]
        ),
        "B2": calculate_piston_kinematics(
            cfg["point_B2"], point_E, 
            cfg["b_strut_mass"], cfg["b_strut_radius"], cfg["b_piston_mass"], cfg["b_piston_radius"]
        )
    }

    # Total  mass, cog, inertia calculation
    parts = []
    for node in ["C1", "C2", "A1", "A2"]:
        parts.append({"mass": cfg["truss_mass"], "cog": trusses[node]["cog"], "I": trusses[node]["I"]})

    for p_name in ["DF", "B1", "B2"]:
        parts.append({"mass": pistons[p_name]["strut"]["mass"], "cog": pistons[p_name]["strut"]["cog"], "I": pistons[p_name]["strut"]["I"]})
        parts.append({"mass": pistons[p_name]["piston"]["mass"], "cog": pistons[p_name]["piston"]["cog"], "I": pistons[p_name]["piston"]["I"]})

    ixx_d, iyy_d, izz_d = calculate_sphere_inertia(cfg["mass_D"], 0.1) # r_D = 0.1
    ixx_e, iyy_e, izz_e = calculate_sphere_inertia(cfg["mass_E"], 0.1) # r_E = 0.1
    ixx_f, iyy_f, izz_f = calculate_cylinder_inertia(cfg["mass_foot"], 0.25, 0.05) # r = 0.25, h = 0.05

    parts.append({"mass": cfg["mass_D"], "cog": np.array(cfg["point_D"]), "I": np.diag([ixx_d, iyy_d, izz_d])})
    parts.append({"mass": cfg["mass_E"], "cog": np.array(point_E), "I": np.diag([ixx_e, iyy_e, izz_e])})
    df_pitch = -pistons["DF"]["pitch"]
    R_foot = get_rotation_matrix(df_pitch, 0)
    I_foot_rot = rotate_inertia(np.diag([ixx_f, iyy_f, izz_f]), R_foot)
    parts.append({"mass": cfg["mass_foot"], "cog": np.array(cfg["point_F"]), "I": I_foot_rot})

    M_tot = sum(p["mass"] for p in parts)
    COG_tot = sum(p["mass"] * p["cog"] for p in parts) / M_tot
    
    I_tot = np.zeros((3, 3))
    for p in parts:
        I_tot += steiner_transport(p["I"], p["mass"], p["cog"], COG_tot)

    cfg["leg_physics_metadata"] = {
        "mass": M_tot,
        "cog": COG_tot,
        "I": I_tot
    }

    # Sdf writing
    for i in range(1, 5):
        leg_model = template_sdf
        
        leg_model = leg_model.replace("{MODEL_NAME}", f"leg{i}")
        leg_model = leg_model.replace("{TRUSS_MASS}", f"{cfg['truss_mass']:.1f}")
        leg_model = leg_model.replace("{TRUSS_RAD}", f"{cfg['truss_radius']:.3f}")
        leg_model = leg_model.replace("{MASS_D}", f"{cfg['mass_D']:.1f}")
        leg_model = leg_model.replace("{IXX_D}", f"{ixx_d:.6f}")
        leg_model = leg_model.replace("{IYY_D}", f"{iyy_d:.6f}")
        leg_model = leg_model.replace("{IZZ_D}", f"{izz_d:.6f}")        
        leg_model = leg_model.replace("{MASS_E}", f"{cfg['mass_E']:.1f}")
        leg_model = leg_model.replace("{IXX_E}", f"{ixx_e:.6f}")
        leg_model = leg_model.replace("{IYY_E}", f"{iyy_e:.6f}")
        leg_model = leg_model.replace("{IZZ_E}", f"{izz_e:.6f}")        
        leg_model = leg_model.replace("{MASS_FOOT}", f"{cfg['mass_foot']:.1f}")
        leg_model = leg_model.replace("{IXX_FOOT}", f"{ixx_f:.6f}")
        leg_model = leg_model.replace("{IYY_FOOT}", f"{iyy_f:.6f}")
        leg_model = leg_model.replace("{IZZ_FOOT}", f"{izz_f:.6f}")        
        leg_model = leg_model.replace("{POSE_D}", f"{cfg['point_D'][0]:.4f} {cfg['point_D'][1]:.4f} {cfg['point_D'][2]:.4f} 0 0 0")
        leg_model = leg_model.replace("{POSE_E}", f"{point_E[0]:.4f} {point_E[1]:.4f} {point_E[2]:.4f} 0 0 0")

        # C1, C2, A1, A2
        for node in ["C1", "C2", "A1", "A2"]:
            leg_model = leg_model.replace(f"{{POSE_{node}D}}", trusses[node]["pose_link"])
            leg_model = leg_model.replace(f"{{L_{node}D}}", f"{trusses[node]['L']:.4f}")
            leg_model = leg_model.replace(f"{{IXX_{node}D}}", f"{trusses[node]['ixx']:.4f}")
            leg_model = leg_model.replace(f"{{IYY_{node}D}}", f"{trusses[node]['iyy']:.4f}")
            leg_model = leg_model.replace(f"{{IZZ_{node}D}}", f"{trusses[node]['izz']:.4f}")

        # DF
        df = pistons["DF"]
        leg_model = leg_model.replace("{PITCH_FOOT}", f"{-df['pitch']:.4f}")
        leg_model = leg_model.replace("{Z_FOOT_POS}", f"{df['rel_z']:.4f}")
        leg_model = leg_model.replace("{Z_PISTON_START}", f"{df['rel_z']:.4f}")
        
        for part in ["strut", "piston"]:
            prefix = "DF_STRUT" if part == "strut" else "DF_PISTON"
            leg_model = leg_model.replace(f"{{POSE_{prefix}}}", df["pose"]) if part == "strut" else leg_model
            leg_model = leg_model.replace(f"{{MASS_{prefix}}}", f"{df[part]['mass']:.1f}")
            leg_model = leg_model.replace(f"{{RAD_{prefix}}}", f"{df[part]['rad']:.3f}")
            leg_model = leg_model.replace(f"{{L_{prefix}}}", f"{df['L_half']:.4f}")
            leg_model = leg_model.replace(f"{{VIS_Z_{prefix}}}", f"{df['vis_z']:.4f}")
            for ax in ["ixx", "iyy", "izz"]:
                leg_model = leg_model.replace(f"{{{ax.upper()}_{prefix}}}", f"{df[part][ax]:.4f}")

        # B1, B2
        for b_name in ["B1", "B2"]:
            b = pistons[b_name]
            leg_model = leg_model.replace(f"{{POSE_{b_name}_STRUT}}", b["pose"])
            leg_model = leg_model.replace(f"{{L_{b_name}_STRUT}}", f"{b['L_half']:.4f}")
            leg_model = leg_model.replace(f"{{VIS_Z_{b_name}_STRUT}}", f"{b['vis_z']:.4f}")
            leg_model = leg_model.replace(f"{{MASS_B_STRUT}}", f"{b['strut']['mass']:.1f}")
            leg_model = leg_model.replace(f"{{RAD_B_STRUT}}", f"{b['strut']['rad']:.3f}")
            
            leg_model = leg_model.replace(f"{{REL_Z_{b_name}_PISTON}}", f"{b['rel_z']:.4f}")
            leg_model = leg_model.replace(f"{{L_{b_name}_PISTON}}", f"{b['L_half']:.4f}")
            leg_model = leg_model.replace(f"{{VIS_Z_{b_name}_PISTON}}", f"{b['vis_z']:.4f}")
            leg_model = leg_model.replace(f"{{MASS_B_PISTON}}", f"{b['piston']['mass']:.1f}")
            leg_model = leg_model.replace(f"{{RAD_B_PISTON}}", f"{b['piston']['rad']:.3f}")
            
            for ax in ["ixx", "iyy", "izz"]:
                leg_model = leg_model.replace(f"{{{ax.upper()}_{b_name}_STRUT}}", f"{b['strut'][ax]:.4f}")
                leg_model = leg_model.replace(f"{{{ax.upper()}_{b_name}_PISTON}}", f"{b['piston'][ax]:.4f}")

        leg_model = leg_model.replace("{DF_K}", f"{cfg['df_k']:.1f}")
        leg_model = leg_model.replace("{DF_C}", f"{cfg['df_c']:.1f}")
        leg_model = leg_model.replace("{DF_fric}", f"{cfg['df_fric']:.1f}")
        leg_model = leg_model.replace("{DF_LIMIT}", f"{cfg['df_limit']:.2f}")
        leg_model = leg_model.replace("{DF_CF_1}", f"{cfg['df_crush_force_stage1']:.1f}")
        leg_model = leg_model.replace("{DF_CF_2}", f"{cfg['df_crush_force_stage2']:.1f}")
        leg_model = leg_model.replace("{DF_SL_1}", f"{cfg['df_stroke_limit_stage_1']:.3f}")
        leg_model = leg_model.replace("{DF_SL_2}", f"{cfg['df_stroke_limit_stage_2']:.3f}")
        leg_model = leg_model.replace("{DF_BS}", f"{cfg['df_bottomout_stiffness']:.1f}")
        leg_model = leg_model.replace("{DF_RD}", f"{cfg['df_rebound_damping']:.1f}")
        
        leg_model = leg_model.replace("{B_K}", f"{cfg['b_k']:.1f}")
        leg_model = leg_model.replace("{B_C}", f"{cfg['b_c']:.1f}")
        leg_model = leg_model.replace("{B_fric}", f"{cfg['b_fric']:.1f}")
        leg_model = leg_model.replace("{B_LIMIT}", f"{cfg['b_limit']:.2f}")
        leg_model = leg_model.replace("{BE_CF}", f"{cfg['be_crush_force']:.1f}")
        leg_model = leg_model.replace("{BE_RD}", f"{cfg['be_rebound_damping']:.1f}")
        
        leg_model = leg_model.replace("{FOOT_K}", f"{cfg['foot_k']:.1f}")
        leg_model = leg_model.replace("{FOOT_C}", f"{cfg['foot_c']:.1f}")

        leg_dir = os.path.join(output_dir, f"leg{i}")
        os.makedirs(leg_dir, exist_ok=True)
        out_file = os.path.join(leg_dir, f"leg{i}.sdf")
        
        with open(out_file, "w") as f:
            f.write(leg_model)
        print(f"Successfully generated: {out_file}")
    
    return cfg 


def calculate_lander_legs_poses(c1, c2, octagon_radius=2.0, z_offset=0.2):
    # Calculate the distance between C1 and C2
    dist_c1_c2 = math.sqrt((c2[0]-c1[0])**2 + (c2[1]-c1[1])**2 + (c2[2]-c1[2])**2)
    
    # Define the local coordinates of the first leg    
    x_local = octagon_radius
    y_local = -dist_c1_c2 / 2.0
    
    legs_poses = {}
    
    # Rotate by 90 degrees for each subsequent leg
    for i in range(4):
        yaw = i * (math.pi / 2.0)          
        # 2D rotation matrix (Yaw around Z axis)
        x_global = x_local * math.cos(yaw) - y_local * math.sin(yaw)
        y_global = x_local * math.sin(yaw) + y_local * math.cos(yaw)
        
        # Save the string
        legs_poses[f"leg{i+1}"] = f"{x_global:.4f} {y_global:.4f} {z_offset:.4f} 0.0000 0.0000 {yaw:.4f}"
        
    return legs_poses

def generate_lander(template_path, output_file, legs_cfg, octagon_radius=2.0, z_legs=0.2):    
    try:
        with open(template_path, "r") as file:
            lander_sdf = file.read()
    except FileNotFoundError:
        print(f"Error: Template file {template_path} not found!")
        return
        
    c1 = legs_cfg["base_points"]["C1"]
    c2 = legs_cfg["base_points"]["C2"]
    
    # Calculates the 4 poses
    legs_poses = calculate_lander_legs_poses(c1, c2, octagon_radius, z_legs)
    
    # Performs substitutions
    for i in range(1, 5):
        tag = f"{{POSE_LEG_{i}}}"
        lander_sdf = lander_sdf.replace(tag, legs_poses[f"leg{i}"])
        
    # Saves the file    
    with open(output_file, "w") as f:
        f.write(lander_sdf)
    print(f"Successfully generated: {output_file}")



def main():
    print(f"--- Starting Legs & Lander generation ---")

    run_id = os.environ.get('SIMULATION_RUN_ID')
    if run_id is not None:
        random.seed(int(run_id))
    
    # Read from the config file
    try:
        with open(CONFIG_PATH, 'r') as file:
            total_config = yaml.safe_load(file)
        print("Simulation config successfully loaded!")
    except FileNotFoundError:
        print(f"Error: Unable to find the simulation config at the path: {CONFIG_PATH}")
        return
        
    # Extracts the section related to the legs
    if "leg_generation" not in total_config:
        print("Error: Section 'leg_generation' not found in the simulation config!")
        return
        
    cfg_yaml = total_config["leg_generation"]

    params_with_ranges = [
        "foot_k", "foot_c",
        "be_crush_force", "be_rebound_damping",
        "b_k", "b_c",
        "df_bottomout_stiffness", "df_rebound_damping",
        "df_crush_force_stage1", "df_crush_force_stage2",
        "df_k", "df_c"
    ]
    extracted_params = {}

    for param in params_with_ranges:        
        if param in cfg_yaml and isinstance(cfg_yaml[param], list) and len(cfg_yaml[param]) == 2:
            min_val, max_val = cfg_yaml[param]
            val = random.uniform(min_val, max_val)
        elif param in cfg_yaml:
            val = cfg_yaml[param]
        else:
            print(f"Warning: {param} with a [min, max] format not found in the simulation config. Check your parameters.")
            continue

        cfg_yaml[param] = val
        extracted_params[param] = val

    
    print("\n--- Selected Leg Parameters ---")
    for key, value in extracted_params.items():
        print(f" > {key}: {value:.2f}")
    print("---------------------------------------\n")

    # Extracts radius and z_offset
    r_octagon = cfg_yaml.get("octagon_radius", 2.0)
    z_offset = cfg_yaml.get("z_legs", 0.25)
    
    # Generate the legs
    used_config = generate_legs(LEG_TEMPLATE_FILE, MODELS_DIR, config_parameters=cfg_yaml)
    
    # Generate the lander
    if used_config:
        generate_lander(LANDER_TEMPLATE_FILE, LANDER_OUTPUT_FILE, used_config, octagon_radius=r_octagon, z_legs=z_offset)

    meta_params_str = "<!-- METADATA_LEG_PARAMS: " + "; ".join([f"{k}={round(v, 2)}" for k, v in extracted_params.items()]) + " -->"
    
    phys = used_config["leg_physics_metadata"]
    I = phys["I"]
    COG = phys["cog"]
    meta_physics_str = f"""
<!-- METADATA_LEG_PROPERTIES:
mass={phys['mass']:.4f}
cog_x={COG[0]:.4f}
cog_y={COG[1]:.4f}
cog_z={COG[2]:.4f}
ixx={I[0,0]:.4f}
iyy={I[1,1]:.4f}
izz={I[2,2]:.4f}
ixy={I[0,1]:.4f}
ixz={I[0,2]:.4f}
iyz={I[1,2]:.4f}
frame=C1
-->
"""
    leg1_sdf_path = os.path.join(MODELS_DIR, "leg1", "leg1.sdf")
    try:
        with open(leg1_sdf_path, 'a') as f:
            f.write("\n" + meta_params_str + "\n" + meta_physics_str)
        print("Metadata successfully appended to leg1.sdf")
    except Exception as e:
        print(f"Warning: error in  the metadata writing: {e}")
    
    print(f"--- Legs & Lander generation completed ---\n")

if __name__ == "__main__":
    main()