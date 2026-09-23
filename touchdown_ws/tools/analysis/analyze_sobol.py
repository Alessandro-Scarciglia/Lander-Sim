import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from SALib.analyze import sobol


WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CSV_PATH = os.path.join(WORKSPACE_DIR, 'data', 'results', 'montecarlo_summary_sobol15.csv')
SOBOL_DIR = os.path.join(WORKSPACE_DIR, 'figures', 'sensitivity', 'sobol')
os.makedirs(SOBOL_DIR, exist_ok=True)

problem = {
    'num_vars': 13, 
    'names': [
        'altitude', 'roll', 'pitch', 'yaw',
        'vel_x', 'vel_y', 'vel_z', 
        'ang_x', 'ang_y', 'ang_z',         
        'pass_scenario_idx',
        'propellant_mass',
        'df_crush_force_stage1'
    ],
    'bounds': [
        [0.3, 1.2], [-8.0, 8.0], [-8.0, 8.0], [-8.0, 8.0], 
        [-0.8, 0.8], [-0.8, 0.8], [-2.0, -0.3],
        [-0.1, 0.1], [-0.1, 0.1], [-0.1, 0.1],               
        [0.0, 2.99],
        [50.0, 1200.0],
        [2400.0, 6400.0]
    ]
}

# CSV Loading
print(f"Loading data from {CSV_PATH}...")

try:
    df = pd.read_csv(CSV_PATH)
except FileNotFoundError:
    print(f"\n[ERROR] Unable to find the CSV file at path: {CSV_PATH}")
    exit()

# Sort the dataframe by the run_id column
df = df.sort_values(by='run_id')

# Extract the output column
Y = df['max_dynamic_tilt_deg'].values

# Verify that the number of rows is correct
N = 128
expected_rows = N * (2 * problem['num_vars'] + 2)
if len(Y) != expected_rows:
    print(f"WARNING: The CSV has {len(Y)} rows, but Saltelli expects {expected_rows}!")
else:
    print(f"Found {len(Y)} valid simulations. Proceeding with the analysis...")

# Analysis
Si = sobol.analyze(problem, Y, calc_second_order=True, print_to_console=False)

S1 = Si['S1']  # First-Order Index
ST = Si['ST']  # Total-Order Index
names = problem['names']

print("\n--- SOBOL ANALYSIS RESULTS ---")
print(f"{'Parameter':<22} | {'First-Order (S1)':<18} | {'Total-Order (ST)':<18}")
print("-" * 65)
for i in range(problem['num_vars']):
    print(f"{names[i]:<22} | {S1[i]:>17.4f} | {ST[i]:>17.4f}")

# Second Order
print("\n--- TOP 5 INTERACTIONS (SECOND ORDER S2) ---")
S2 = Si['S2']
interactions = []
for i in range(problem['num_vars']):
    for j in range(i + 1, problem['num_vars']):
        interactions.append((names[i], names[j], S2[i][j]))
interactions.sort(key=lambda x: abs(x[2]) if not np.isnan(x[2]) else 0, reverse=True)

print(f"{'Parameter 1':<20} | {'Parameter 2':<20} | {'S2 Index':<10}")
print("-" * 55)
for inter in interactions[:5]:
    print(f"{inter[0]:<20} | {inter[1]:<20} | {inter[2]:.4f}")

# Plot
x = np.arange(len(names))
width = 0.35

fig, ax = plt.subplots(figsize=(14, 7))

# Bars for First-Order and Total-Order
rects1 = ax.bar(x - width/2, S1, width, label='First-Order (Direct Effect)', color='#2980b9')
rects2 = ax.bar(x + width/2, ST, width, label='Total-Order (With Interactions)', color='#e74c3c')

ax.set_ylabel('Fraction of Explained Variance', fontsize=12)
ax.set_title('Sobol Indices: Parameter impact on max_dynamic_tilt_deg', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels(names, rotation=45, ha='right', fontsize=10)
ax.legend(fontsize=11)
ax.grid(axis='y', linestyle='--', alpha=0.7)

plt.tight_layout()

# Save the plot in the defined folder
plot_filename = os.path.join(SOBOL_DIR, 'sobol_indices_max_tilt.png')
plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
print(f"\nPlot successfully saved in: {plot_filename}")

# Second order heatmap plot
S2_matrix = np.array(Si['S2'])

fig2, ax2 = plt.subplots(figsize=(12, 10))

cax = ax2.imshow(S2_matrix, cmap='Reds', interpolation='nearest', vmin=0)
ax2.set_title('Sobol Second-Order Indices (S2) - Parameter Interactions', fontsize=16, fontweight='bold')

ax2.set_xticks(np.arange(len(names)))
ax2.set_yticks(np.arange(len(names)))
ax2.set_xticklabels(names, rotation=45, ha='right', fontsize=11)
ax2.set_yticklabels(names, fontsize=11)

cbar = fig2.colorbar(cax, ax=ax2, shrink=0.8)
cbar.set_label('Interaction Strength (S2)', fontsize=12)
for i in range(len(names)):
    for j in range(len(names)):
        val = S2_matrix[i, j]        
        if not np.isnan(val) and val > 0.01:            
            text_color = "white" if val > np.nanmax(S2_matrix)*0.6 else "black"
            ax2.text(j, i, f"{val:.2f}", ha="center", va="center", color=text_color, fontsize=9, fontweight='bold')

plt.tight_layout()
heatmap_filename = os.path.join(SOBOL_DIR, '03_sobol_interactions_heatmap.png')
plt.savefig(heatmap_filename, dpi=300, bbox_inches='tight')
print(f"Heatmap (S2) successfully saved in: {heatmap_filename}")