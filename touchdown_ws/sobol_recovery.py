import os
import subprocess
import time
import yaml
import copy
import pandas as pd
try:
    from SALib.sample import sobol
except ImportError:
    print("ERRORE: SALib non trovata. Esegui: pip install SALib")
    exit(1)

# ==============================================================================
# DEFINIZIONE DEL PROBLEMA IDENTICA A IERI
# ==============================================================================
problem = {
    'num_vars': 13, 
    'names': [
        'pos_x', 'pos_y', 'altitude', 'roll', 'pitch', 
        'vel_x', 'vel_y', 'vel_z', 
        'ang_x', 'ang_y', 'ang_z',
        'propellant_mass', 'df_crush_force_stage1'
    ],
    'bounds': [
        [-15.0, 15.0], [-15.0, 15.0], [0.3, 1.2], 
        [-8.0, 8.0], [-8.0, 8.0], 
        [-0.8, 0.8], [-0.8, 0.8], [-2.0, -0.3],
        [-0.1, 0.1], [-0.1, 0.1], [-0.1, 0.1], 
        [50.0, 1200.0], [2400.0, 6400.0]
    ]
}

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_CONFIG_PATH = os.path.join(CURRENT_DIR, "config_base.yaml") 
WORK_CONFIG_PATH = os.path.join(CURRENT_DIR, "config.yaml")      
LAUNCH_FILE_PATH = os.path.join(CURRENT_DIR, "headless_launch.py")
LAUNCH_COMMAND = ['ros2', 'launch', LAUNCH_FILE_PATH]

# Il CSV dove il tuo nodo C++ sta salvando i dati
CSV_PATH = os.path.join(CURRENT_DIR, 'plots', "montecarlo_summary.csv")

def update_config_for_run(base_config, param_dict):
    config = copy.deepcopy(base_config)
    
    # 1. PARAMETRI VARIABILI
    config['landing_scenario']['pos_x_range'] = [float(param_dict['pos_x'])] * 2
    config['landing_scenario']['pos_y_range'] = [float(param_dict['pos_y'])] * 2
    config['landing_scenario']['altitude_range'] = [float(param_dict['altitude'])] * 2
    config['landing_scenario']['roll_range_deg'] = [float(param_dict['roll'])] * 2
    config['landing_scenario']['pitch_range_deg'] = [float(param_dict['pitch'])] * 2
    
    config['landing_scenario']['vel_x_range'] = [float(param_dict['vel_x'])] * 2
    config['landing_scenario']['vel_y_range'] = [float(param_dict['vel_y'])] * 2
    config['landing_scenario']['vel_z_range'] = [float(param_dict['vel_z'])] * 2
    
    config['landing_scenario']['ang_x_range'] = [float(param_dict['ang_x'])] * 2
    config['landing_scenario']['ang_y_range'] = [float(param_dict['ang_y'])] * 2
    config['landing_scenario']['ang_z_range'] = [float(param_dict['ang_z'])] * 2
    
    config['leg_generation']['df_crush_force_stage1'] = [float(param_dict['df_crush_force_stage1'])] * 2
    
    # 2. PARAMETRI FISSI (Valori nominali medi dei 14 scartati da Morris)
    config['landing_scenario']['yaw_range_deg'] = [0.0, 0.0]
    
    chosen_dry = base_config['body_generation']['dry_scenarios'][1]
    chosen_pass = base_config['body_generation']['passenger_scenarios'][1]
    config['body_generation']['dry_scenarios'] = [chosen_dry]
    config['body_generation']['passenger_scenarios'] = [chosen_pass]
    
    propellant_mass = float(param_dict['propellant_mass'])
    valid_landing_mass = chosen_dry['mass'] + chosen_pass['mass'] + propellant_mass
    config['body_generation']['landing_mass_range'] = [valid_landing_mass, valid_landing_mass]
    
    config['leg_generation']['df_k'] = [100000.0, 100000.0]
    config['leg_generation']['df_c'] = [28000.0, 28000.0]
    config['leg_generation']['df_crush_force_stage2'] = [15000.0, 15000.0]
    config['leg_generation']['df_rebound_damping'] = [15000.0, 15000.0]
    config['leg_generation']['df_bottomout_stiffness'] = [10000000.0, 10000000.0]
    config['leg_generation']['b_k'] = [100000.0, 100000.0]
    config['leg_generation']['b_c'] = [28000.0, 28000.0]
    config['leg_generation']['be_crush_force'] = [6000.0, 6000.0]
    config['leg_generation']['be_rebound_damping'] = [15000.0, 15000.0]
    config['leg_generation']['foot_k'] = [200.0, 200.0]
    config['leg_generation']['foot_c'] = [20.0, 20.0]
    
    return config

def main():
    print("==================================================================")
    print(" AVVIO RECOVERY ORCHESTRATOR SOBOL...")
    print("==================================================================")
    
    N_SAMPLES = 256
    param_values = sobol.sample(problem, N_SAMPLES, calc_second_order=False)
    NUM_RUNS = len(param_values)    
    
    completed_runs = set()
    if os.path.exists(CSV_PATH):
        try:
            df = pd.read_csv(CSV_PATH)
            if 'run_id' in df.columns:
                completed_runs = set(df['run_id'].dropna().astype(int).tolist())
                print(f"Trovate {len(completed_runs)} simulazioni già completate nel CSV.")
        except Exception as e:
            print(f"Errore nella lettura del CSV: {e}")
            return
    else:
        print("ATTENZIONE: File CSV non trovato. Sei sicuro di essere nella directory giusta?")
        return

    missing_runs = [i for i in range(1, NUM_RUNS + 1) if i not in completed_runs]
    
    print(f"Run mancanti da eseguire: {len(missing_runs)} su {NUM_RUNS}")
    print("==================================================================")
    
    if len(missing_runs) == 0:
        print("Tutte le simulazioni sono già state completate! Ottimo lavoro.")
        return

    if not os.path.exists(BASE_CONFIG_PATH):
        print(f"ERRORE: Non trovo {BASE_CONFIG_PATH}.")
        return

    with open(BASE_CONFIG_PATH, 'r') as f:
        base_config = yaml.safe_load(f)

    os.environ['MC_ENABLE_PLOTS'] = 'False'
    statistics = {"successes": 0, "failures": 0}
    absolute_start_time = time.time()
    
    
    current_idx = 0

    for run_id, sample in enumerate(param_values, start=1):
        if run_id in completed_runs:
            continue 
            
        current_idx += 1
        print(f"\n[RUN {run_id}/{NUM_RUNS}] (Esecuzione mancante {current_idx}/{len(missing_runs)})")
        
        param_dict = dict(zip(problem['names'], sample))
        run_config = update_config_for_run(base_config, param_dict)
        
        with open(WORK_CONFIG_PATH, 'w') as f:
            yaml.dump(run_config, f, default_flow_style=False)
            
        try:
            os.environ['MC_RUN_ID'] = str(run_id)
            os.environ['SIMULATION_RUN_ID'] = str(run_id)            
            
            run_start_time = time.time()            
            
            subprocess.run(LAUNCH_COMMAND, check=True, timeout=120.0)            
            
            run_duration = time.time() - run_start_time
            print(f"[RUN {run_id}] Completata in {run_duration:.1f} sec.")
            statistics["successes"] += 1
            
        except subprocess.TimeoutExpired:
            print(f"[RUN {run_id}]  TIMEOUT: Chiusura forzata di sicurezza...")
            statistics["failures"] += 1
            subprocess.run(['pkill', '-9', '-f', 'gzserver'], stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', '-f', 'ros2 launch'], stderr=subprocess.DEVNULL)
            time.sleep(3.0)
            
        except subprocess.CalledProcessError as e:
            print(f"[RUN {run_id}] SOFTWARE ERROR (Possibile OOM): Exit code {e.returncode}.")
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
    print(" RECUPERO DI SOBOL COMPLETATO!")
    print("==================================================================")
    print(f" Run mancanti previste: {len(missing_runs)}")
    print(f" Completate ora senza crash: {statistics['successes']}")
    print(f" Simulator crashes/timeouts in questa sessione: {statistics['failures']}")
    print(f" Tempo impiegato per il recupero:  {total_time/3600:.2f} ore")
    print("==================================================================")

if __name__ == "__main__":
    main()