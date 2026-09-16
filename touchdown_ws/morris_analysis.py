import os
import subprocess
import time
import yaml
import copy
from SALib.sample import morris

# Problem definition
problem = {
    'num_vars': 25, 
    'names': [        
        'altitude', 'roll', 'pitch', 'yaw',
        'vel_x', 'vel_y', 'vel_z',
        'ang_x', 'ang_y', 'ang_z',
        
        'dry_scenario_idx',   
        'pass_scenario_idx',  
        'propellant_mass',   
        
        'df_k', 'df_c', 'df_crush_force_stage1', 'df_crush_force_stage2',
        'df_rebound_damping', 'df_bottomout_stiffness', 
        'b_k', 'b_c', 'be_crush_force', 'be_rebound_damping',
        'foot_k', 'foot_c'
    ],
    'bounds': [        
        [0.3, 1.2], [-8.0, 8.0], [-8.0, 8.0], [-8.0, 8.0], 
        [-0.8, 0.8], [-0.8, 0.8], [-2.0, -0.3],
        [-0.1, 0.1], [-0.1, 0.1], [-0.1, 0.1], 
        
        [0.0, 2.99],          
        [0.0, 2.99],          
        [50.0, 1200.0],       
        
        [40000.0, 160000.0], [16000.0, 40000.0], [2400.0, 6400.0], [12000.0, 18000.0], 
        [12000.0, 18000.0], [8000000.0, 12000000.0],  
        [40000.0, 160000.0], [16000.0, 40000.0], [4800.0, 7200.0], [12000.0, 18000.0],
        [160.0, 240.0], [16.0, 24.0]
    ]
}

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_CONFIG_PATH = os.path.join(CURRENT_DIR, "config_base.yaml") 
WORK_CONFIG_PATH = os.path.join(CURRENT_DIR, "config.yaml")      
LAUNCH_FILE_PATH = os.path.join(CURRENT_DIR, "headless_launch.py")
LAUNCH_COMMAND = ['ros2', 'launch', LAUNCH_FILE_PATH]

def update_config_for_run(base_config, param_dict):
    
    config = copy.deepcopy(base_config)
    
    # Positions, Attitude, Linear and Angular Velocities
    config['landing_scenario']['pos_x_range'] = [66.00, 66.00]
    config['landing_scenario']['pos_y_range'] = [-26.00, -26.00]
    
    config['landing_scenario']['altitude_range'] = [float(param_dict['altitude'])] * 2
    config['landing_scenario']['roll_range_deg'] = [float(param_dict['roll'])] * 2
    config['landing_scenario']['pitch_range_deg'] = [float(param_dict['pitch'])] * 2
    config['landing_scenario']['yaw_range_deg'] = [float(param_dict['yaw'])] * 2
    
    config['landing_scenario']['vel_x_range'] = [float(param_dict['vel_x'])] * 2
    config['landing_scenario']['vel_y_range'] = [float(param_dict['vel_y'])] * 2
    config['landing_scenario']['vel_z_range'] = [float(param_dict['vel_z'])] * 2
    
    config['landing_scenario']['ang_x_range'] = [float(param_dict['ang_x'])] * 2
    config['landing_scenario']['ang_y_range'] = [float(param_dict['ang_y'])] * 2
    config['landing_scenario']['ang_z_range'] = [float(param_dict['ang_z'])] * 2
    
    # Masses and Scenarios
    dry_len = len(base_config['body_generation']['dry_scenarios'])
    pass_len = len(base_config['body_generation']['passenger_scenarios'])
    
    dry_idx = min(max(int(param_dict['dry_scenario_idx']), 0), dry_len - 1)
    pass_idx = min(max(int(param_dict['pass_scenario_idx']), 0), pass_len - 1)
    
    chosen_dry = base_config['body_generation']['dry_scenarios'][dry_idx]
    chosen_pass = base_config['body_generation']['passenger_scenarios'][pass_idx]
    
    config['body_generation']['dry_scenarios'] = [ chosen_dry ]
    config['body_generation']['passenger_scenarios'] = [ chosen_pass ]
    
    # Calculate the exact landing mass as a sum
    propellant_mass = float(param_dict['propellant_mass'])
    valid_landing_mass = chosen_dry['mass'] + chosen_pass['mass'] + propellant_mass
    config['body_generation']['landing_mass_range'] = [valid_landing_mass, valid_landing_mass]
    
    # Legs parameters
    config['leg_generation']['df_k'] = [float(param_dict['df_k'])] * 2
    config['leg_generation']['df_c'] = [float(param_dict['df_c'])] * 2
    config['leg_generation']['df_crush_force_stage1'] = [float(param_dict['df_crush_force_stage1'])] * 2
    config['leg_generation']['df_crush_force_stage2'] = [float(param_dict['df_crush_force_stage2'])] * 2
    config['leg_generation']['df_rebound_damping'] = [float(param_dict['df_rebound_damping'])] * 2
    config['leg_generation']['df_bottomout_stiffness'] = [float(param_dict['df_bottomout_stiffness'])] * 2 
    
    config['leg_generation']['b_k'] = [float(param_dict['b_k'])] * 2
    config['leg_generation']['b_c'] = [float(param_dict['b_c'])] * 2
    config['leg_generation']['be_crush_force'] = [float(param_dict['be_crush_force'])] * 2
    config['leg_generation']['be_rebound_damping'] = [float(param_dict['be_rebound_damping'])] * 2
    config['leg_generation']['foot_k'] = [float(param_dict['foot_k'])] * 2
    config['leg_generation']['foot_c'] = [float(param_dict['foot_c'])] * 2
    
    return config

def main():
    print("==================================================================")
    print(" GENERATING MORRIS MATRIX...")
    print("==================================================================")
    
    r_trajectories = 30  
    param_values = morris.sample(problem, N=r_trajectories, num_levels=4, optimal_trajectories=None)
    NUM_RUNS = len(param_values)
    
    print(f" Matrix generated! {NUM_RUNS} simulations expected (N = {r_trajectories} * ({problem['num_vars']} + 1)).")
    print(f" Base Config Path: {BASE_CONFIG_PATH}")
    print("==================================================================")

    if not os.path.exists(BASE_CONFIG_PATH):
        print(f"ERROR: Cannot find {BASE_CONFIG_PATH}. Remember to rename your config.yaml!")
        return

    with open(BASE_CONFIG_PATH, 'r') as f:
        base_config = yaml.safe_load(f)

    os.environ['MC_ENABLE_PLOTS'] = 'False'
    statistics = {"successes": 0, "failures": 0}
    
    absolute_start_time = time.time()

    for run_id, sample in enumerate(param_values, start=1):
        print(f"\n[RUN {run_id}/{NUM_RUNS}] Preparing configuration...")       
        
        if run_id > 1:
            elapsed = time.time() - absolute_start_time
            avg_time = elapsed / (run_id - 1)
            eta_sec = avg_time * (NUM_RUNS - run_id + 1)
            print(f"Estimated time remaining: {eta_sec/3600:.2f} hours")
        
        param_dict = dict(zip(problem['names'], sample))
        
        run_config = update_config_for_run(base_config, param_dict)
        with open(WORK_CONFIG_PATH, 'w') as f:
            yaml.dump(run_config, f, default_flow_style=False)
            
        try:
            os.environ['MC_RUN_ID'] = str(run_id)
            os.environ['SIMULATION_RUN_ID'] = str(run_id)            
            
            print(f"[RUN {run_id}] Background execution (Generation + GzServer)...")
            run_start_time = time.time()             
            
            subprocess.run(LAUNCH_COMMAND, check=True, timeout=120.0)            
            
            run_duration = time.time() - run_start_time
            print(f"[RUN {run_id}] Execution completed in {run_duration:.1f} seconds.")
            statistics["successes"] += 1
            
        except subprocess.TimeoutExpired:
            print(f"[RUN {run_id}] TIMEOUT: Simulation frozen or exploded. Forced shutdown...")
            statistics["failures"] += 1            
            subprocess.run(['pkill', '-9', '-f', 'gzserver'], stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', '-f', 'ros2 launch'], stderr=subprocess.DEVNULL)
            time.sleep(3.0)
            
        except subprocess.CalledProcessError as e:
            print(f"[RUN {run_id}] SOFTWARE ERROR: Failure with exit code {e.returncode}.")
            statistics["failures"] += 1
            subprocess.run(['pkill', '-9', '-f', 'gzserver'], stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', '-f', 'ros2 launch'], stderr=subprocess.DEVNULL)
            
        except Exception as e:
            print(f"[RUN {run_id}] Unexpected script error: {e}")
            statistics["failures"] += 1
            subprocess.run(['pkill', '-9', '-f', 'gzserver'], stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', '-f', 'ros2 launch'], stderr=subprocess.DEVNULL)
            
        time.sleep(2.0)

    total_time = time.time() - absolute_start_time
    print("\n==================================================================")
    print(" MORRIS ANALYSIS COMPLETED!")
    print("==================================================================")
    print(f" Total scheduled runs: {NUM_RUNS}")
    print(f" Completed without crashes: {statistics['successes']}")
    print(f" Simulator crashes/timeouts: {statistics['failures']}")
    print(f" Total time elapsed:  {total_time/3600:.2f} hours")
    print("==================================================================")

if __name__ == "__main__":
    main()