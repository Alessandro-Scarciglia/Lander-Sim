import os
import numpy as np
import matplotlib.pyplot as plt
from SALib.sample import sobol as sobol_sample
from SALib.analyze import sobol

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
SOBOL_DIR = os.path.join(CURRENT_DIR, 'plots', 'sobol_test')
os.makedirs(SOBOL_DIR, exist_ok=True)

problem = {
    'num_vars': 14, 
    'names': [
        'altitude', 'roll', 'pitch', 'yaw',
        'vel_x', 'vel_y', 'vel_z', 
        'ang_x', 'ang_y', 'ang_z', 
        'dry_scenario_idx',
        'pass_scenario_idx',
        'propellant_mass',
        'df_crush_force_stage1'
    ],
    'bounds': [
        [0.3, 1.2], [-8.0, 8.0], [-8.0, 8.0], [-8.0, 8.0], 
        [-0.8, 0.8], [-0.8, 0.8], [-2.0, -0.3],
        [-0.1, 0.1], [-0.1, 0.1], [-0.1, 0.1], 
        [0.0, 2.99],          
        [0.0, 2.99],
        [50.0, 1200.0],
        [2400.0, 6400.0]
    ]
}

print("Generazione del campione Saltelli sintetico...")
N = 128
# 1. Generiamo la matrice X esattamente come se dovessimo simulare in Gazebo
X = sobol_sample.sample(problem, N, calc_second_order=True)

print(f"Generati {X.shape[0]} scenari. Calcolo della Y fittizia...")

# 2. Creiamo i risultati Y basati sulla nostra legge matematica nota
Y = np.zeros(X.shape[0])
for i in range(X.shape[0]):
    altitude = X[i, 0]
    roll = X[i, 1]
    pitch = X[i, 2]
    vel_y = X[i, 5]
    
    # LA NOSTRA LEGGE MATEMATICA DI TEST:
    Y[i] = 50.0 * (vel_y**2) + 5.0 * altitude

# 3. Passiamo i dati artificiali a SALib per l'analisi
Si = sobol.analyze(problem, Y, calc_second_order=True, print_to_console=False)

S1 = Si['S1']  
ST = Si['ST']  
names = problem['names']

print("\n--- SOBOL ANALYSIS RESULTS (SYNTHETIC TEST) ---")
print(f"{'Parameter':<22} | {'First-Order (S1)':<18} | {'Total-Order (ST)':<18}")
print("-" * 65)
for i in range(problem['num_vars']):
    print(f"{names[i]:<22} | {S1[i]:>17.4f} | {ST[i]:>17.4f}")

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

# Plot Grafico S1 e ST
x = np.arange(len(names))
width = 0.35
fig, ax = plt.subplots(figsize=(14, 7))
rects1 = ax.bar(x - width/2, S1, width, label='First-Order (Direct Effect)', color='#2980b9')
rects2 = ax.bar(x + width/2, ST, width, label='Total-Order (With Interactions)', color='#e74c3c')
ax.set_ylabel('Fraction of Explained Variance', fontsize=12)
ax.set_title('TEST MATH: Sobol Indices', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels(names, rotation=45, ha='right', fontsize=10)
ax.legend(fontsize=11)
ax.grid(axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.savefig(os.path.join(SOBOL_DIR, 'test_sobol_bars.png'), dpi=300, bbox_inches='tight')

# Plot Heatmap
S2_matrix = np.array(Si['S2'])
fig2, ax2 = plt.subplots(figsize=(12, 10))
cax = ax2.imshow(S2_matrix, cmap='Reds', interpolation='nearest', vmin=0)
ax2.set_title('TEST MATH: Sobol Interactions', fontsize=16, fontweight='bold')
ax2.set_xticks(np.arange(len(names)))
ax2.set_yticks(np.arange(len(names)))
ax2.set_xticklabels(names, rotation=45, ha='right', fontsize=11)
ax2.set_yticklabels(names, fontsize=11)
cbar = fig2.colorbar(cax, ax=ax2, shrink=0.8)
for i in range(len(names)):
    for j in range(len(names)):
        val = S2_matrix[i, j]        
        if not np.isnan(val) and val > 0.01:            
            text_color = "white" if val > np.nanmax(S2_matrix)*0.6 else "black"
            ax2.text(j, i, f"{val:.2f}", ha="center", va="center", color=text_color, fontsize=9, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(SOBOL_DIR, 'test_sobol_heatmap.png'), dpi=300, bbox_inches='tight')
print("\nTest completato. Controlla le immagini salvate per la verifica visiva!")