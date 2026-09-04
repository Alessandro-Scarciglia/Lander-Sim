import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import trimesh
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import accuracy_score, classification_report
import numpy as np



CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(CURRENT_DIR, 'plots', 'montecarlo_summary_final.csv')
TERRAIN_MESH_PATH = os.path.join(CURRENT_DIR, 'src', 'worlds', 'models', 'terrain_model','meshes', 'terrain_col_light.stl')

# Cost function weights
W_FORCE = 30   
W_ENERGY = 30  
W_TILT = 40    
PENALTY = 1000 

# Threshold
TILT_THRESHOLD = 30.0             # Degree


# Input/Output metrics
OUTPUT_METRICS = [
    'impact_angle_deg',
    'specific_kin_energy',
    'res_accel_z',
    'max_impact_vz_ms',
    'impact_thrust_N',
    'final_total_tilt_deg',
    'max_force_z_N',
    'settling_time_s',
    'min_belly_clearance_m'
]
INPUT_METRICS = [
    "total_mass_kg",
    "init_x", "init_y", "init_z",
    "init_vx", "init_vy", "init_vz",
    "init_roll", "init_pitch", "init_yaw",
    "init_wx", "init_wy", "init_wz"
]

PLOT_LABELS = {
    # Inputs 
    "total_mass_kg": "Total Mass [kg]",
    "init_x": "Initial X Position [m]",
    "init_y": "Initial Y Position [m]",
    "init_z": "Initial Altitude Z [m]",
    "init_vx": "Initial X Velocity [m/s]",
    "init_vy": "Initial Y Velocity [m/s]",
    "init_vz": "Initial Vertical Velocity [m/s]",
    "init_roll": "Initial Roll [deg]",
    "init_pitch": "Initial Pitch [deg]",
    "init_yaw": "Initial Yaw [deg]",
    "init_wx": "Initial Roll Rate [deg/s]",
    "init_wy": "Initial Pitch Rate [deg/s]",
    "init_wz": "Initial Yaw Rate [deg/s]",

    # Outputs
    "impact_angle_deg": "Impact Angle [deg]",
    "specific_kin_energy": "Specific Kinetic Energy [J/kg]",
    "res_accel_z": "Residual Vertical Acceleration [m/s²]",
    "max_impact_vz_ms": "Vertical Impact Velocity [m/s]",
    "impact_thrust_N": "Impact Thrust [N]",
    "final_total_tilt_deg": "Final Tilt [deg]",
    "max_force_z_N": "Maximum Leg Force [N]",
    "settling_time_s": "Settling Time [s]",
    "min_belly_clearance_m": "Minimum Belly Clearance [m]"
}

SCATTER_PAIRS = [    
    ("init_z", "final_total_tilt_deg"),         
    ("init_y", "final_total_tilt_deg"),          
    ("init_vz", "final_total_tilt_deg"),        
    ("total_mass_kg", "final_total_tilt_deg"),      
    ("init_x", "init_y"),                        
    ("init_z", "init_vz"),                       
    ("total_mass_kg", "init_vz"),                
    ("init_y", "init_vy"),   
    ("settling_time_s", "final_total_tilt_deg"), 
    ("max_impact_vz_ms", "final_total_tilt_deg") 
]

def load_and_enrich_data(csv_path):    
    if not os.path.exists(csv_path):
        print(f"Errore: File {csv_path} non trovato.")
        return None
    df = pd.read_csv(csv_path)
    
    # Calculation of success/failure conditions
    df['is_flipped'] = df['final_total_tilt_deg'] > TILT_THRESHOLD    
    df['flag_hull_crash'] = df['flag_hull_crash'].astype(bool)
    df['flag_thruster_crash'] = df['flag_thruster_crash'].astype(bool)

    df['mission_success'] = ~df['is_flipped']     
    df['mission_success_num'] = df['mission_success'].astype(int)
    
    # Cost function
    norm_force = df['max_force_z_N'] / df['max_force_z_N'].max()
    norm_energy = df['specific_kin_energy'] / df['specific_kin_energy'].max()
    norm_tilt = df['final_total_tilt_deg'] / df['final_total_tilt_deg'].max()
    
    df['base_cost'] = (norm_force * W_FORCE) + (norm_energy * W_ENERGY) + (norm_tilt * W_TILT)
    
    # Penalty
    is_failed = ~df['mission_success']
    df['total_cost'] = df['base_cost'].where(~is_failed, df['base_cost'] + PENALTY)
    
    return df

def print_comprehensive_report(df):    
    total_runs = len(df)
    success_count = df['mission_success'].sum()
    success_rate = (success_count / total_runs) * 100
    
    print("\n" + "="*50)
    print(" MASTER MONTE CARLO ANALYSIS REPORT")
    print("="*50)
    
    print("\nGENERAL STATISTICS")
    print(f"Total Runs:                  {total_runs}")
    print(f"Successes:                   {success_count} ({success_rate:.1f}%)")
    print(f"Failures:                    {total_runs - success_count} ({100 - success_rate:.1f}%)")
    
    print("\nFAILURE DIAGNOSTICS")
    print(f"Tip-over (> {TILT_THRESHOLD}°):            {df['is_flipped'].sum()} runs")             
    print(f"Hull Impact (Crash):         {df['flag_hull_crash'].sum()} runs")
    print(f"Thruster Impact (Crash):     {df['flag_thruster_crash'].sum()} runs")
    
    print("\nPERFORMANCE METRICS")    
    clean_runs = df[~(df['flag_hull_crash'] == True) & ~(df['flag_thruster_crash'] == True)]
    
    if not clean_runs.empty and 'mass_error_pct' in clean_runs.columns:
        avg_mass_error = clean_runs['mass_error_pct'].mean()
        print(f"Average Mass Error (Clean Runs): {avg_mass_error:.2f}%")
    else:
        print("Average Mass Error (Clean Runs): N/A (Nessuna run pulita o colonna mancante)")
    

    print("\nCOST FUNCTION ANALYSIS")    
    df_sorted = df.sort_values(by='total_cost')    
    columns_to_print = ['run_id'] + INPUT_METRICS + ['total_cost']
    print("--- Top 3 perfect landings ---")
    print(df_sorted.head(3)[columns_to_print].to_string(index=False))

    print("\n--- Bottom 3 (Worst landings) ---")
    worst_survivors = df_sorted[df_sorted['total_cost'] < 1000].tail(3)
    print(worst_survivors[columns_to_print].to_string(index=False))
    print("="*50 + "\n")

def generate_plots(df):    
    print("Plots generation...")
    sns.set_theme(style="whitegrid")
    hist_dir = os.path.join(CURRENT_DIR, "plots", "histograms")
    os.makedirs(hist_dir, exist_ok=True)
    corr_dir = os.path.join(CURRENT_DIR, "plots", "correlation")
    os.makedirs(corr_dir, exist_ok=True)
    scatter_dir = os.path.join(CURRENT_DIR, "plots", "scatter")
    os.makedirs(scatter_dir, exist_ok=True)
    box_dir = os.path.join(CURRENT_DIR, "plots", "boxplots")
    os.makedirs(box_dir, exist_ok=True)    

    #  1) Distribution of all output metrics:
    for col in OUTPUT_METRICS:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8,6), gridspec_kw={"height_ratios":[4,1]})
        sns.histplot(df[col], bins=20, kde=True, color="steelblue", edgecolor="black", ax=ax1)
        mean = df[col].mean()
        median = df[col].median()
        ax1.axvline(mean, color="red", linestyle="--", linewidth=2, label=f"Mean = {mean:.2f}")
        ax1.axvline(median, color="green", linestyle="-.", linewidth=2, label=f"Median = {median:.2f}")
        ax1.legend()
        std = df[col].std()
        text = (f"Mean   = {mean:.2f}\n" f"Median = {median:.2f}\n" f"Std = {std:.2f}")
        ax1.text( 0.98, 0.95, text, transform=ax1.transAxes, ha="right", va="top", fontsize=10, bbox=dict(facecolor="white", alpha=0.8))
        ax1.set_title(f"Distribution of {PLOT_LABELS[col]}", fontsize=14, fontweight="bold")
        ax1.set_ylabel("Count")
        sns.boxplot(x=df[col], ax=ax2, color="orange")
        ax2.set_xlabel(PLOT_LABELS[col])
        plt.tight_layout()
        filename = f"01_distribution_{col}.png"
        plt.savefig(os.path.join(hist_dir, filename), dpi=300)
        plt.close()   
    

    # 2) Correlation Heatmaps:
    
    # Correlation between inputs
    plt.figure(figsize=(12, 12))
    corr_inputs = df[INPUT_METRICS].corr()
    corr_inputs.index = [PLOT_LABELS[c] for c in corr_inputs.index]
    corr_inputs.columns = [PLOT_LABELS[c] for c in corr_inputs.columns]
    sns.heatmap(corr_inputs,annot=True, fmt=".2f", cmap="coolwarm", vmin=-1, vmax=1, center=0, square=True, linewidths=0.5, linecolor="white")
    plt.title("Correlation Matrix - Input Variables", fontsize=15, fontweight="bold")
    plt.tight_layout()
    plt.savefig(os.path.join(corr_dir, "01_input_correlation.png"),dpi=300)
    plt.close()

    # Correlation between input and output
    corr_io = df[INPUT_METRICS + OUTPUT_METRICS].corr()
    corr_io = corr_io.loc[INPUT_METRICS, OUTPUT_METRICS]
    corr_io.index = [PLOT_LABELS[c] for c in corr_io.index]
    corr_io.columns = [PLOT_LABELS[c] for c in corr_io.columns]
    plt.figure(figsize=(12, 8))
    sns.heatmap(corr_io, annot=True, fmt=".2f", cmap="coolwarm", vmin=-1, vmax=1, center=0, linewidths=0.5, linecolor="white")
    plt.title("Correlation Matrix - Inputs vs Outputs", fontsize=15, fontweight="bold")
    plt.tight_layout()
    plt.savefig(os.path.join(corr_dir, "02_input_output_correlation.png"), dpi=300)
    plt.close()

    # Correlation between outputs
    plt.figure(figsize=(9, 8))
    corr_outputs = df[OUTPUT_METRICS].corr()
    corr_outputs.index = [PLOT_LABELS[c] for c in corr_outputs.index]
    corr_outputs.columns = [PLOT_LABELS[c] for c in corr_outputs.columns]
    sns.heatmap(corr_outputs, annot=True, fmt=".2f", cmap="coolwarm", vmin=-1, vmax=1, center=0, square=True, linewidths=0.5, linecolor="white")
    plt.title("Correlation Matrix - Output Variables", fontsize=15, fontweight="bold")
    plt.tight_layout()
    plt.savefig(os.path.join(corr_dir, "03_output_correlation.png"), dpi=300)
    plt.close()

    # Sensitivity ranking
    target_outputs = ['max_force_z_N', 'final_total_tilt_deg', 'min_belly_clearance_m', 'total_cost']
    corr_matrix = df[INPUT_METRICS + target_outputs].corr()    
    for target in target_outputs:
        plt.figure(figsize=(8, 5))        
        sensitivities = corr_matrix[target].loc[INPUT_METRICS].abs().sort_values(ascending=False)          
        labels = [PLOT_LABELS.get(idx, idx) for idx in sensitivities.index]        
        sns.barplot(x=sensitivities.values, y=labels, hue=labels, palette="magma", orient='h', legend=False)        
        plt.title(f"Tornado Chart: Main Drivers for\n{PLOT_LABELS.get(target, target)}", fontsize=14, fontweight='bold')
        plt.xlabel("Impact (Absolute Pearson Correlation |r|)")
        plt.grid(axis='x', linestyle='--', alpha=0.7)
        plt.tight_layout()        
        plt.savefig(os.path.join(corr_dir, f"04_tornado_sensitivity_{target}.png"), dpi=300)
        plt.close()  
       
    # 3) Scatter plots:      
    df['outcome_type'] = 'Success'
    df.loc[df['is_flipped'] == True, 'outcome_type'] = 'Tip-Over (Flipped)'

    for i, (x, y) in enumerate(SCATTER_PAIRS, start=1):
        r = df[[x, y]].corr().iloc[0, 1]
        plt.figure(figsize=(7, 6))          
        
        sns.scatterplot(
            data=df, x=x, y=y, hue="outcome_type", 
            hue_order=["Success", "Tip-Over (Flipped)"], 
            palette={"Success": "green", "Tip-Over (Flipped)": "red"}, 
            alpha=0.8, s=60, edgecolor="black" 
        )
        
        sns.regplot(data=df, x=x, y=y, scatter=False, color="black", line_kws={"linewidth": 2, "alpha": 0.8})
        
        
        plt.legend(title="Mission Outcome")
        
        plt.xlabel(PLOT_LABELS[x])
        plt.ylabel(PLOT_LABELS[y])
        plt.title(f"{PLOT_LABELS[x]} vs {PLOT_LABELS[y]}\nPearson r = {r:.2f}", fontsize=14, fontweight="bold")
        plt.grid(True, linestyle="--", alpha=0.5)        
        
        filename = (f"{i:02d}_" f"{PLOT_LABELS[x].split('[')[0].strip()}_to_" f"{PLOT_LABELS[y].split('[')[0].strip()}.png")
        plt.tight_layout()
        plt.savefig(os.path.join(scatter_dir, filename), dpi=300)
        plt.close()  

    # 4) Boxplots    
    for col in OUTPUT_METRICS:
        plt.figure(figsize=(7,5))        
        sns.boxplot(data=df, x="mission_success", y=col, hue="mission_success", palette={True:"green", False:"red", "True":"green", "False":"red"}, legend=False)
        plt.xlabel("Mission Success")
        plt.ylabel(PLOT_LABELS[col])
        plt.title(f"{PLOT_LABELS[col]} vs Mission Outcome")
        plt.tight_layout()
        plt.savefig(os.path.join(box_dir, f"boxplot_{col}.png"), dpi=300)
        plt.close()


    # 5) 2D map total cost distribution    
    map_dir = os.path.join(CURRENT_DIR, "plots", "maps")
    os.makedirs(map_dir, exist_ok=True)    
    plt.figure(figsize=(12, 10))  
    try:        
        mesh = trimesh.load(TERRAIN_MESH_PATH)        
        x = mesh.vertices[:, 0]
        y = mesh.vertices[:, 1]
        z = mesh.vertices[:, 2]
        faces = mesh.faces        
        plt.tripcolor(x, y, faces, z, cmap='gray', shading='gouraud', alpha=0.8, edgecolors='none', antialiased=False)
    except Exception as e:
        print(f"Warning: Unable to load the background mesh. Error: {e}")
        print("The scatter plot will be generated on a white background.")
    
    sc = plt.scatter(df['init_x'], df['init_y'], c=df['total_cost'], cmap='coolwarm', s=60, alpha=0.9, edgecolors='black', linewidths=0.5, zorder=5)     
    cbar = plt.colorbar(sc)
    cbar.set_label('Total Cost (Base Cost + Penalty)', fontsize=12)    
    plt.xlabel(PLOT_LABELS["init_x"], fontsize=12)
    plt.ylabel(PLOT_LABELS["init_y"], fontsize=12)
    plt.title("2D Top-Down View: Terrain Cost Distribution", fontsize=15, fontweight="bold")    
    plt.axis('equal')
    plt.grid(True, linestyle="--", alpha=0.5)    
    plt.tight_layout()
    plt.savefig(os.path.join(map_dir, "01_top_down_cost_map.png"), dpi=300)
    plt.close()               

    print("Plots generated")

def run_svm_analysis(df):
    print("\n" + "="*50)
    print(" SUPPORT VECTOR MACHINE (SVM) - TIP-OVER ANALYSIS")
    print("="*50)

    INPUT_METRICS = [
        'total_mass_kg', 
        'init_z', 
        'init_vx', 
        'init_vy', 
        'init_vz', 
        'init_roll', 
        'init_pitch', 
        'init_yaw', 
        'init_wx', 
        'init_wy', 
        'init_wz'
    ]
    # Data definition of the global model
    X = df[INPUT_METRICS]
    y = df['mission_success_num']
    
    scaler_global = StandardScaler()
    X_scaled = scaler_global.fit_transform(X)
    
    # Model training
    svm_global = SVC(kernel='linear', C=1.0, class_weight='balanced', random_state=42)
    svm_global.fit(X_scaled, y)

    # Model testing
    y_pred = svm_global.predict(X_scaled)
    accuracy = accuracy_score(y, y_pred)
    print(f"Global SVM Model Accuracy: {accuracy * 100:.2f}%\n")
    print("Classification Report:")
    print(classification_report(y, y_pred, target_names=['Tip-Over (0)', 'Success (1)']))

    # Hyperplane Extraction
    weights = svm_global.coef_[0]
    intercept = svm_global.intercept_[0]
    
    print("\nThe safety domain f(inputs) = Success is defined by the inequality:\n")
    equation = ""
    for i, feature in enumerate(INPUT_METRICS):
        weight = weights[i]
        sign = "+" if weight >= 0 else "-"
        equation += f" {sign} {abs(weight):.4f} * {feature}\n"
    sign_b = "+" if intercept >= 0 else "-"
    equation += f" {sign_b} {abs(intercept):.4f} > 0"
    print(equation)    
    
    
    # 2D models
    svm_dir = os.path.join(CURRENT_DIR, "plots", "svm_domain")
    os.makedirs(svm_dir, exist_ok=True)
    
    # Critical couples
    SVM_PAIRS = [
        ("init_vz", "init_z"),             
        ("init_vz", "total_mass_kg"),      
        ("init_vx", "init_pitch")          
    ]
    
    for feature_x, feature_y in SVM_PAIRS:
        print(f"Generating Safe Domain envelope for {feature_x} vs {feature_y}...")
        
        # Data definition
        X_2d = df[[feature_x, feature_y]]
        scaler_2d = StandardScaler()
        X_2d_scaled = scaler_2d.fit_transform(X_2d)
        
        # Model training
        svm_2d = SVC(kernel='rbf', C=1.0, gamma='auto', class_weight='balanced', random_state=42)
        svm_2d.fit(X_2d_scaled, y)
        
        # Plots preaparation
        margin_x = (X_2d[feature_x].max() - X_2d[feature_x].min()) * 0.05
        margin_y = (X_2d[feature_y].max() - X_2d[feature_y].min()) * 0.05
        
        x_min, x_max = X_2d[feature_x].min() - margin_x, X_2d[feature_x].max() + margin_x
        y_min, y_max = X_2d[feature_y].min() - margin_y, X_2d[feature_y].max() + margin_y
        xx, yy = np.meshgrid(np.linspace(x_min, x_max, 300), np.linspace(y_min, y_max, 300))
        
        
        grid_points = np.c_[xx.ravel(), yy.ravel()]
        grid_points_scaled = scaler_2d.transform(grid_points)
        Z = svm_2d.predict(grid_points_scaled).reshape(xx.shape)
        
        # Plot
        plt.figure(figsize=(10, 8))          
        plt.contourf(xx, yy, Z, alpha=0.3, cmap=plt.cm.RdYlGn)        
        sns.scatterplot(
            data=df, x=feature_x, y=feature_y, hue='mission_success', 
            palette={True: "green", False: "red"}, edgecolor='black', zorder=2, s=50
        )          
        
        plt.title(f"Safe Landing intervals\n{PLOT_LABELS.get(feature_x, feature_x)} vs {PLOT_LABELS.get(feature_y, feature_y)}", fontsize=14, fontweight='bold')
        plt.xlabel(PLOT_LABELS.get(feature_x, feature_x))
        plt.ylabel(PLOT_LABELS.get(feature_y, feature_y))        
        handles, labels = plt.gca().get_legend_handles_labels()
        plt.legend(handles, ['Tip-Over', 'Success'], title='Actual Outcome', loc='best')
        
        plt.grid(True, linestyle="--", alpha=0.5)
        plt.tight_layout()        
        
        safe_name_x = feature_x.replace('init_', '')
        safe_name_y = feature_y.replace('init_', '')
        plt.savefig(os.path.join(svm_dir, f"svm_intervals_{safe_name_x}_vs_{safe_name_y}.png"), dpi=300)
        plt.close()

    print("SVM intervals generated successfully.")



if __name__ == "__main__":
    dataset = load_and_enrich_data(CSV_PATH)
    if dataset is not None:
        print_comprehensive_report(dataset)
        generate_plots(dataset)
        run_svm_analysis(dataset)  
        