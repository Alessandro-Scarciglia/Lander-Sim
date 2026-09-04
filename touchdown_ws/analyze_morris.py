import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(CURRENT_DIR, 'plots', 'montecarlo_summary_morris15.csv')
MORRIS_DIR = os.path.join(CURRENT_DIR, 'plots', 'morris')


PARAMS = [
    'init_z', 'init_roll', 'init_pitch', 'init_yaw',
    'init_vx', 'init_vy', 'init_vz', 'init_wx', 'init_wy', 'init_wz',
    'dry_mass_kg', 'passenger_mass_kg', 'propellant_mass',    
    'df_k', 'df_c', 'df_crush_force_stage1', 'df_crush_force_stage2',
    'df_rebound_damping', 'df_bottomout_stiffness', 
    'b_k', 'b_c', 'be_crush_force', 'be_rebound_damping',
    'foot_k', 'foot_c'
]


PLOT_LABELS = {
    'init_z': 'Altitude Z',
    'init_roll': 'Roll', 'init_pitch': 'Pitch', 'init_yaw': 'Yaw',
    'init_vx': 'Vel X', 'init_vy': 'Vel Y', 'init_vz': 'Vel Z',
    'init_wx': 'Roll Rate', 'init_wy': 'Pitch Rate', 'init_wz': 'Yaw Rate',
    'dry_mass_kg': 'Dry Mass', 'passenger_mass_kg': 'Pass. Mass', 'propellant_mass': 'Propellant',
    'df_k': 'Leg Stiff. (K)', 'df_c': 'Leg Damping (C)', 
    'df_crush_force_stage1': 'Crush Stage 1', 'df_crush_force_stage2': 'Crush Stage 2',
    'df_rebound_damping': 'Rebound Damping', 'df_bottomout_stiffness': 'Bottom-out Stiff.',
    'b_k': 'Strut Stiff.', 'b_c': 'Strut Damping', 
    'be_crush_force': 'Strut Crush', 'be_rebound_damping': 'Strut Rebound',
    'foot_k': 'Foot Stiff.', 'foot_c': 'Foot Damping'
}

def main():
    print("Starting Custom Morris Analysis (Robust Evaluator)...")
    os.makedirs(MORRIS_DIR, exist_ok=True)
    
    if not os.path.exists(CSV_PATH):
        print(f"ERROR: File {CSV_PATH} not found!")
        return
        
    df = pd.read_csv(CSV_PATH)
    df = df.sort_values('run_id').reset_index(drop=True)
    
    # Recalculate the exact propellant used
    df['propellant_mass'] = df['total_mass_kg'] - df['dry_mass_kg'] - df['passenger_mass_kg'] - df['legs_mass_kg']

    n_runs = len(df)
    if n_runs % 26 != 0:
        print("ERROR: The matrix is not a multiple of 26!")
        return
    n_trajectories = n_runs // 26
    print(f"Found {n_trajectories} perfect trajectories (Total: {n_runs} runs).")

    # Elementary effect method
    df_params = df[PARAMS]
    df_norm = (df_params - df_params.min()) / (df_params.max() - df_params.min())
    
    Y = df['max_dynamic_tilt_deg'].values
    
    # Dictionary to accumulate the Elementary Effects (EE) of each parameter
    EE_dict = {param: [] for param in PARAMS}
    
    for t in range(n_trajectories):        
        for step in range(25):
            idx1 = t * 26 + step
            idx2 = t * 26 + step + 1
            
            # Find which parameter varied the most 
            diffs = df_norm.iloc[idx2] - df_norm.iloc[idx1]
            changed_param = diffs.abs().idxmax()
            delta_x = diffs[changed_param]            
            
            if delta_x == 0:
                continue
                
            delta_y = Y[idx2] - Y[idx1]
            ee = delta_y / delta_x
            
            EE_dict[changed_param].append(ee)

    # Statistical calculation
    results = []
    for param in PARAMS:
        ee_array = np.array(EE_dict[param])
        if len(ee_array) == 0:
            mu_star, sigma = 0.0, 0.0
        else:
            mu_star = np.mean(np.abs(ee_array))
            sigma = np.std(ee_array)
            
        results.append({
            'Parameter': PLOT_LABELS[param],
            'Mu_Star': mu_star,
            'Sigma': sigma
        })
        
    results_df = pd.DataFrame(results).sort_values(by='Mu_Star', ascending=False).reset_index(drop=True)
    
    print("\n" + "="*60)
    print(" MOST CRITICAL PARAMETERS RANKING (MAX DYNAMIC TILT)")
    print("="*60)
    print(results_df.to_string())
    
    # Final plot
    plt.figure(figsize=(12, 8))
    
    mu_stars = results_df['Mu_Star'].values
    sigmas = results_df['Sigma'].values
    labels = results_df['Parameter'].values    
   
    plt.scatter(mu_stars, sigmas, s=120, c='crimson', alpha=0.7, edgecolors='black', zorder=3)
    
    threshold = np.max(mu_stars) * 0.10
    for i, txt in enumerate(labels):
        if mu_stars[i] > threshold:
            plt.annotate(txt, (mu_stars[i], sigmas[i]), xytext=(7, 7), textcoords='offset points', 
                         fontsize=11, fontweight='bold', bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.8))
    
    
    max_mu = np.max(mu_stars) * 1.1 if np.max(mu_stars) > 0 else 1.0
    plt.plot([0, max_mu], [0, max_mu], 'r--', alpha=0.5, label='$\mu^* = \sigma$ (High Interaction/Non-Linearity)', zorder=1)
    plt.plot([0, max_mu], [0, max_mu/2], 'g--', alpha=0.5, label='$\mu^* = 2\sigma$ (Quasi-Linear Effect)', zorder=1)

    plt.title('Morris Sensitivity Analysis: Maximum Dynamic Tilt [deg]', fontsize=16, fontweight='bold')
    plt.xlabel('$\mu^*$ (Absolute Global Importance)', fontsize=14)
    plt.ylabel('$\sigma$ (Non-Linearity or Interactions)', fontsize=14)
    plt.grid(True, linestyle='--', alpha=0.5, zorder=0)
    plt.legend(fontsize=12, loc='upper left')
    plt.xlim(left=0, right=max_mu)
    plt.ylim(bottom=0, top=np.max(sigmas)*1.1 if np.max(sigmas)>0 else 1.0)
    
    plt.tight_layout()
    
    plot_path = os.path.join(MORRIS_DIR, '01_morris_tilt_analysis.png')
    plt.savefig(plot_path, dpi=300)
    print(f"\nPlot successfully saved in: {plot_path}")

if __name__ == "__main__":
    main()