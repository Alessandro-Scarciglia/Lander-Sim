import math
import os
import yaml

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

if "install" in CURRENT_DIR:
    WS_ROOT = CURRENT_DIR.split("install")[0]
else:
    WS_ROOT = CURRENT_DIR.split("src")[0]

# Base directory
MODELS_DIR = os.path.join(WS_ROOT, "src", "worlds", "models", "lander_model")

LEG_TEMPLATE_FILE = os.path.join(MODELS_DIR, "leg_template.sdf")
LANDER_TEMPLATE_FILE = os.path.join(MODELS_DIR, "lander_template.sdf")
LANDER_OUTPUT_FILE = os.path.join(MODELS_DIR, "lander.sdf")
CONFIG_PATH = os.path.join(WS_ROOT, "config.yaml")

def calculate_cylinder_inertia(mass, radius, length):    
    ixx = (1.0 / 12.0) * mass * (3 * radius**2 + length**2)
    izz = 0.5 * mass * radius**2
    return ixx, ixx, izz 

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
    
    return {
        "L": L, "pose_link": f"{cx:.4f} {cy:.4f} {cz:.4f} 0.0000 {pitch:.4f} {yaw:.4f}",
        "ixx": ixx, "iyy": iyy, "izz": izz
    }

def calculate_piston_kinematics(p_start, p_end, mass_strut, rad_strut, mass_piston, rad_piston):
    vx, vy, vz = p_start[0] - p_end[0], p_start[1] - p_end[1], p_start[2] - p_end[2]
    L_total = math.sqrt(vx**2 + vy**2 + vz**2)
    yaw = math.atan2(vy, vx)
    pitch = math.acos(vz / L_total) if L_total != 0 else 0
    L_half = L_total * 0.5
    
    ixx_s, iyy_s, izz_s = calculate_cylinder_inertia(mass_strut, rad_strut, L_half)
    ixx_p, iyy_p, izz_p = calculate_cylinder_inertia(mass_piston, rad_piston, L_half)

    return {
        "pose": f"{p_start[0]:.4f} {p_start[1]:.4f} {p_start[2]:.4f} 0.0000 {pitch:.4f} {yaw:.4f}",
        "pitch": pitch, "L_half": L_half, "vis_z": -L_half / 2.0, "rel_z": -L_half,
        "strut": {"mass": mass_strut, "rad": rad_strut, "ixx": ixx_s, "iyy": iyy_s, "izz": izz_s},
        "piston": {"mass": mass_piston, "rad": rad_piston, "ixx": ixx_p, "iyy": iyy_p, "izz": izz_p}
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
        "df_k": 100000.0, "df_c": 18000.0, "df_limit": 0.45,
        "b_k": 100000.0,  "b_c": 1000.0, "b_limit": 0.4,
        "foot_k": 100.0,  "foot_c": 10.0
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

    for i in range(1, 5):
        leg_model = template_sdf
        
        leg_model = leg_model.replace("{MODEL_NAME}", f"leg{i}")
        leg_model = leg_model.replace("{TRUSS_MASS}", f"{cfg['truss_mass']:.1f}")
        leg_model = leg_model.replace("{TRUSS_RAD}", f"{cfg['truss_radius']:.3f}")
        
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
        leg_model = leg_model.replace("{DF_LIMIT}", f"{cfg['df_limit']:.2f}")
        
        leg_model = leg_model.replace("{B_K}", f"{cfg['b_k']:.1f}")
        leg_model = leg_model.replace("{B_C}", f"{cfg['b_c']:.1f}")
        leg_model = leg_model.replace("{B_LIMIT}", f"{cfg['b_limit']:.2f}")
        
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
    
    # Read from the config file
    try:
        with open(CONFIG_PATH, 'r') as file:
            total_config = yaml.safe_load(file)
        print("Config.yaml file successfully loaded!")
    except FileNotFoundError:
        print(f"Error: Unable to find the config.yaml file at the path: {CONFIG_PATH}")
        return
        
    # Extracts the section related to the legs
    if "leg_generation" not in total_config:
        print("Error: Section 'leg_generation' not found in config.yaml!")
        return
        
    cfg_yaml = total_config["leg_generation"]
    
    # Extracts radius and z_offset
    r_octagon = cfg_yaml.get("octagon_radius", 2.0)
    z_offset = cfg_yaml.get("z_legs", 0.25)
    
    # Generate the legs
    used_config = generate_legs(LEG_TEMPLATE_FILE, MODELS_DIR, config_parameters=cfg_yaml)
    
    # Generate the lander
    if used_config:
        generate_lander(LANDER_TEMPLATE_FILE, LANDER_OUTPUT_FILE, used_config, octagon_radius=r_octagon, z_legs=z_offset)
    
    print(f"--- Legs & Lander generation completed ---\n")

if __name__ == "__main__":
    main()