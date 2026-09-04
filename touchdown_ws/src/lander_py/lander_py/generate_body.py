import numpy as np
import random
import os
import re
import yaml

run_id = os.environ.get('SIMULATION_RUN_ID')
if run_id is not None:
    random.seed(int(run_id))

# Definition of the SDF file path
# Find the absolute path where this script is located
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

# Determine the workspace root based on where the code is running
if "install" in CURRENT_DIR:
    # If the file runs from install/lander_py/
    WS_ROOT = CURRENT_DIR.split("install")[0]
else:
    # If the file runs from src/lander_py/
    WS_ROOT = CURRENT_DIR.split("src")[0]

BASE_DIR = os.path.join(WS_ROOT, "src", "worlds", "models", "lander_model", "body")
TEMPLATE_FILE = os.path.join(BASE_DIR, "body_template.sdf")
OUTPUT_FILE = os.path.join(BASE_DIR, "body.sdf")

PASSENGER_DIR = os.path.join(WS_ROOT, "src", "worlds", "models", "lander_model", "passenger")
PASSENGER_TEMPLATE_FILE = os.path.join(PASSENGER_DIR, "passenger_template.sdf")
PASSENGER_OUTPUT_FILE = os.path.join(PASSENGER_DIR, "passenger.sdf")

CONFIG_PATH = os.path.join(WS_ROOT, "config.yaml")
try:
    with open(CONFIG_PATH, 'r') as file:
        config = yaml.safe_load(file)
    print("Config.yaml file successfully loaded!")
except FileNotFoundError:
    print(f"Error: Unable to find the config.yaml file at the path: {CONFIG_PATH}")
    exit(1)

# Body parameters:
body_cfg = config["body_generation"]
# Landing Mass interval
M1L, M2L = body_cfg["landing_mass_range"]   

# Lander dry
DRY_SCENARIOS = body_cfg["dry_scenarios"]

# Passenger
PASSENGER_SCENARIOS = body_cfg["passenger_scenarios"]

# Tanks
tank_cfg = body_cfg["tank_properties"]

Rcyl = tank_cfg["Rcyl"]
Lcyl_MON = tank_cfg["Lcyl_MON"]
Lcyl_MMH = tank_cfg["Lcyl_MMH"]
Hdome = tank_cfg["Hdome"]
POS_TANK_1 = tank_cfg["pos_tank_1"] # MMH
POS_TANK_2 = tank_cfg["pos_tank_2"] # MON
POS_TANK_3 = tank_cfg["pos_tank_3"] # MON
POS_TANK_4 = tank_cfg["pos_tank_4"] # MMH
rho_MMH = tank_cfg["rho_MMH"]
rho_MON = tank_cfg["rho_MON"]
ratio_MON_MMH = tank_cfg["ratio_MON_MMH"]
EPSILON_CONFIG = float(tank_cfg.get("epsilon_body", 1e-6))

# Huygens-Steiner theorem
def steiner_transport(I_cg, mass, old_pos, new_pos):
    d = np.array(old_pos) - np.array(new_pos)
    d_sq = np.dot(d, d)
    outer_d = np.outer(d, d)
    return np.array(I_cg) + mass * (d_sq * np.eye(3) - outer_d)

def calculate_tank_properties(mass_target, current_rho , current_Lcyl, epsilon, nz=4000):       
    # Formula for the radius of a generating sphere: Rs = (a² + h²)/(2h)    
    Rsphere = (Rcyl**2 + Hdome**2) / (2.0 * Hdome)

    # Distance between the generating sphere center and the junction plane
    dcap = Rsphere - Hdome

    # Junction planes position
    z1 = -current_Lcyl / 2.0
    z2 = +current_Lcyl / 2.0

    # Bottom/top tank
    z_bottom = z1 - Hdome
    z_top    = z2 + Hdome

    # Bottom sphere center
    zc_bottom = z1 + dcap

    # Top sphere center
    zc_top = z2 - dcap
    
    # Target volume
    V_target = mass_target / current_rho
    
    # Cylinder volume
    V_cyl = np.pi * Rcyl**2 * current_Lcyl

    # Volume of a spherical cap (V = π h² (3R-h)/3):    
    V_cap = (
        np.pi * Hdome**2 *
        (3.0 * Rsphere - Hdome)
    ) / 3.0

    # Total volume 
    V_total = V_cyl + 2.0 * V_cap

    if V_target > V_total:
        raise ValueError("Tank overfilled")
   

    def radius_at_z(z):        
        # Bottom cap
        if z < z1:
            val = Rsphere**2 - (z - zc_bottom)**2
            return np.sqrt(max(val, 0.0))
        
        # Cylinder
        elif z1 <= z <= z2:
            return Rcyl

        # Top cap
        else:
            val = Rsphere**2 - (z - zc_top)**2
            return np.sqrt(max(val, 0.0))

    
    def volume_until_height(h):
        z = np.linspace(z_bottom, h, nz)
        area = np.zeros_like(z)
        for i, zi in enumerate(z):
            r = radius_at_z(zi)
            area[i] = np.pi * r**2
        return np.trapezoid(area, z)
    
    
    max_iter = 100
    iter_count = 0
    error = float('inf') # Initialize the error to an infinite value
    
    # Propellant height (bisection method)
    z_low = z_bottom
    z_high = z_top

    while error > epsilon and iter_count < max_iter:
        z_mid = 0.5 * (z_low + z_high)
        v_mid = volume_until_height(z_mid)
        error = abs(V_target - v_mid)

        if v_mid < V_target:
            z_low = z_mid
        else:
            z_high = z_mid

        iter_count += 1

    h_fill = z_mid
    if iter_count == max_iter:
        print(f"Warning: Bisection interrupted at the limit of {max_iter} iterations. Residual error: {error}")
    else: 
        print(f"Convergence at: {iter_count} iterations")
        

    # Calculation of properties
    z = np.linspace(z_bottom, h_fill, nz)
    dz = z[1] - z[0]
    rloc = np.zeros_like(z)

    for i, zi in enumerate(z):
        rloc[i] = radius_at_z(zi)

    # Sectional area
    area = np.pi * rloc**2

    # Infinitesimal volume
    dV = area * dz

    # Infinitesimal mass
    dm = current_rho * dV
    total_mass = np.sum(dm)
    
    # Cog
    z_cg_local = np.sum(z * dm) / total_mass

    # Inertias
    Ixx = 0.0
    Iyy = 0.0
    Izz = 0.0

    for zi, ri, dmi in zip(z, rloc, dm):
        # Infinitesimal disk
        dIzz = 0.5 * dmi * ri**2
        dIxy = 0.25 * dmi * ri**2
        dz_cg = zi - z_cg_local

        dIxx = dIxy + dmi * dz_cg**2
        dIyy = dIxy + dmi * dz_cg**2

        Ixx += dIxx
        Iyy += dIyy
        Izz += dIzz

    return (
        z_cg_local,
        Ixx,
        Iyy,
        Izz
    )


def generate_simulation_files():
    while True:
        # Landing Mass    
        M_L = random.uniform(M1L, M2L)

        # Dry scenario
        scenario = random.choice(DRY_SCENARIOS)
        M_dry = scenario["mass"]
        COG_DRY = np.array(scenario["cog"])
        I_DRY = np.array(scenario["I"]) 

        # Passenger scenario
        passenger_scenario = random.choice(PASSENGER_SCENARIOS)
        M_PASSENGER = passenger_scenario["mass"]
        COG_PASS = np.array(passenger_scenario["cog"])
        I_PASS = np.array(passenger_scenario["I"])   
        
        # Propellant calculation 
        delta_M_P = M_L - M_dry - M_PASSENGER

        if delta_M_P > 0:
            break 

    # Propellant mass calculation
    frazione_MON = ratio_MON_MMH / (1.0 + ratio_MON_MMH)
    frazione_MMH = 1.0 - frazione_MON

    mass_tot_MON = delta_M_P * frazione_MON
    mass_tot_MMH = delta_M_P * frazione_MMH
    m_tank_MON = mass_tot_MON / 2.0
    m_tank_MMH = mass_tot_MMH / 2.0

    # Calculation of properties for tank 1 and tank 4 (MMH)
    z_cg_MMH, ixx_MMH, iyy_MMH, izz_MMH = calculate_tank_properties(m_tank_MMH, rho_MMH, Lcyl_MMH, EPSILON_CONFIG)
    I_tank_MMH = [[ixx_MMH, 0, 0], [0, iyy_MMH, 0], [0, 0, izz_MMH]]
    cog_tank1 = np.array(POS_TANK_1) + np.array([0.0, 0.0, z_cg_MMH])
    cog_tank4 = np.array(POS_TANK_4) + np.array([0.0, 0.0, z_cg_MMH])

    # Calculation of properties for tank 2 and tank 3 (MON)
    z_cg_MON, ixx_MON, iyy_MON, izz_MON = calculate_tank_properties(m_tank_MON, rho_MON, Lcyl_MON, EPSILON_CONFIG)
    I_tank_MON = [[ixx_MON, 0, 0], [0, iyy_MON, 0], [0, 0, izz_MON]]    
    cog_tank2 = np.array(POS_TANK_2) + np.array([0.0, 0.0, z_cg_MON])
    cog_tank3 = np.array(POS_TANK_3) + np.array([0.0, 0.0, z_cg_MON])
    
    # Body link mass (Lander Dry + Fuel)
    M_body_link = M_dry + delta_M_P

    # CoG and Inertias only for the body link
    cog_body = (M_dry * np.array(COG_DRY) + m_tank_MON * cog_tank2 + m_tank_MON * cog_tank3 + m_tank_MMH * cog_tank1 + m_tank_MMH * cog_tank4) / M_body_link

    I_dry_trans = steiner_transport(I_DRY, M_dry, COG_DRY, cog_body)
    I_t1_trans = steiner_transport(I_tank_MMH, m_tank_MMH, cog_tank1, cog_body)
    I_t4_trans = steiner_transport(I_tank_MMH, m_tank_MMH, cog_tank4, cog_body)
    I_t2_trans = steiner_transport(I_tank_MON, m_tank_MON, cog_tank2, cog_body)
    I_t3_trans = steiner_transport(I_tank_MON, m_tank_MON, cog_tank3, cog_body)

    I_body_totale = I_dry_trans + I_t1_trans + I_t4_trans + I_t2_trans + I_t3_trans

    print(f"--- Generation of new simulation ---")
    print(f"M_L (Landing Mass)        : {M_L:.2f} kg")
    print(f"M_dry (Dry Mass)          : {M_dry:.2f} kg")
    print(f"Passenger mass            : {M_PASSENGER:.2f} kg")
    print(f"Residual propellant       : {delta_M_P:.2f} kg")
    print(f"Mass assigned to the body : {M_body_link:.2f} kg")
    
    # SDF file writing
    try:
        with open(TEMPLATE_FILE, "r") as file:
            template_data = file.read()
    except FileNotFoundError:
        print(f"CRITICAL Error: Template file not found!")
        return
    
    # Generate body.sdf
    new_sdf = template_data.replace("{MASS_WET}", f"{M_body_link:.5f}")
    new_sdf = new_sdf.replace("{COG_X}", f"{cog_body[0]:.5f}")
    new_sdf = new_sdf.replace("{COG_Y}", f"{cog_body[1]:.5f}")
    new_sdf = new_sdf.replace("{COG_Z}", f"{cog_body[2]:.5f}")    
    new_sdf = new_sdf.replace("{IXX}", f"{I_body_totale[0,0]:.5f}")
    new_sdf = new_sdf.replace("{IYY}", f"{I_body_totale[1,1]:.5f}")
    new_sdf = new_sdf.replace("{IZZ}", f"{I_body_totale[2,2]:.5f}")
    new_sdf = new_sdf.replace("{IXY}", f"{I_body_totale[0,1]:.5f}")
    new_sdf = new_sdf.replace("{IXZ}", f"{I_body_totale[0,2]:.5f}")
    new_sdf = new_sdf.replace("{IYZ}", f"{I_body_totale[1,2]:.5f}")

    new_sdf += f"\n<!-- METADATA_DRY_MASS: {M_dry:.5f} -->\n"
    new_sdf += f"\n<!-- METADATA_WET_MASS: {M_body_link:.5f} -->\n"
    new_sdf += (
    f"<!-- METADATA_BODY_COG: "
    f"{cog_body[0]:.8f};"
    f"{cog_body[1]:.8f};"
    f"{cog_body[2]:.8f} -->\n"
    )
    new_sdf += (
        f"<!-- METADATA_BODY_INERTIA: "
        f"{I_body_totale[0,0]:.8f};"
        f"{I_body_totale[0,1]:.8f};"
        f"{I_body_totale[0,2]:.8f};"
        f"{I_body_totale[1,0]:.8f};"
        f"{I_body_totale[1,1]:.8f};"
        f"{I_body_totale[1,2]:.8f};"
        f"{I_body_totale[2,0]:.8f};"
        f"{I_body_totale[2,1]:.8f};"
        f"{I_body_totale[2,2]:.8f} -->\n"
    )
    
    try:
        with open(OUTPUT_FILE, "w") as file:
            file.write(new_sdf)
        print(f"'body.sdf' file generated successfully!\n")
    except Exception as e:
        print(f"Error: {e}")
    
    # Generate passenger.sdf   
    try:
        with open(PASSENGER_TEMPLATE_FILE, "r") as file:
            passenger_template = file.read()

    except FileNotFoundError:
        print("CRITICAL Error: Template file not found!")
        return
    new_passenger_sdf = passenger_template
    new_passenger_sdf = new_passenger_sdf.replace("{PASS_MASS}", f"{M_PASSENGER:.5f}")
    new_passenger_sdf = new_passenger_sdf.replace("{PASS_COG_X}", f"{COG_PASS[0]:.5f}")
    new_passenger_sdf = new_passenger_sdf.replace("{PASS_COG_Y}", f"{COG_PASS[1]:.5f}")
    new_passenger_sdf = new_passenger_sdf.replace("{PASS_COG_Z}", f"{COG_PASS[2]:.5f}")
    new_passenger_sdf = new_passenger_sdf.replace("{PASS_IXX}", f"{I_PASS[0,0]:.5f}")
    new_passenger_sdf = new_passenger_sdf.replace("{PASS_IYY}", f"{I_PASS[1,1]:.5f}")
    new_passenger_sdf = new_passenger_sdf.replace("{PASS_IZZ}", f"{I_PASS[2,2]:.5f}")
    new_passenger_sdf = new_passenger_sdf.replace("{PASS_IXY}", f"{I_PASS[0,1]:.5f}")
    new_passenger_sdf = new_passenger_sdf.replace("{PASS_IXZ}", f"{I_PASS[0,2]:.5f}")
    new_passenger_sdf = new_passenger_sdf.replace("{PASS_IYZ}", f"{I_PASS[1,2]:.5f}")
    new_passenger_sdf += (
    f"\n<!-- METADATA_PASSENGER_MASS: {M_PASSENGER:.8f} -->\n"
    )

    new_passenger_sdf += (
        f"<!-- METADATA_PASSENGER_COG: "
        f"{COG_PASS[0]:.8f};"
        f"{COG_PASS[1]:.8f};"
        f"{COG_PASS[2]:.8f} -->\n"
    )

    new_passenger_sdf += (
        f"<!-- METADATA_PASSENGER_INERTIA: "
        f"{I_PASS[0,0]:.8f};"
        f"{I_PASS[0,1]:.8f};"
        f"{I_PASS[0,2]:.8f};"
        f"{I_PASS[1,0]:.8f};"
        f"{I_PASS[1,1]:.8f};"
        f"{I_PASS[1,2]:.8f};"
        f"{I_PASS[2,0]:.8f};"
        f"{I_PASS[2,1]:.8f};"
        f"{I_PASS[2,2]:.8f} -->\n"
    )
        

    with open(PASSENGER_OUTPUT_FILE, "w") as file:
        file.write(new_passenger_sdf)
    print("'passenger.sdf' file generated successfully!")

if __name__ == "__main__":
    generate_simulation_files()