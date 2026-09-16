import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_DIR = os.path.join(CURRENT_DIR, 'plots')
OUTPUT_DIR = os.path.join(CURRENT_DIR, 'plots', 'rollover_analysis')
os.makedirs(OUTPUT_DIR, exist_ok=True)

scenarios = {
    '10° Slope': 'montecarlo_summary_sobol10.csv',
    '15° Slope': 'montecarlo_summary_sobol15.csv',
    '20° Slope': 'montecarlo_summary_sobol20.csv'
}

ROLLOVER_THRESHOLD = 30.0 
MASS_ERROR_THRESHOLD = -55.0 

results = {}

print(f"--- EXTRACTING MECHANICALLY CONFIRMED ROLLOVER OCCURRENCES ---")
print(f"Thresholds: Tilt > {ROLLOVER_THRESHOLD}° AND Mass Error < {MASS_ERROR_THRESHOLD}%")
print("-" * 110)
print(f"{'Scenario':<15} | {'Total Runs':<12} | {'Kinematic (>30°)':<18} | {'Confirmed':<12} | {'Percentage':<12} | {'Mean Fail Tilt':<15} | {'Max Tilt'}")
print("-" * 110)

# CSV Loading and Data Extraction
for label, filename in scenarios.items():
    filepath = os.path.join(CSV_DIR, filename)
    try:
        df = pd.read_csv(filepath)
        total_runs = len(df)        
        
        # Kinematic mask for comparison
        kinematic_mask = df['final_total_tilt_deg'] > ROLLOVER_THRESHOLD
        kinematic_count = kinematic_mask.sum()
        
        # Isolate the runs that failed (Double Check: Kinematics + Dynamics)
        failed_mask = (df['final_total_tilt_deg'] > ROLLOVER_THRESHOLD) & (df['mass_error_pct'] < MASS_ERROR_THRESHOLD)
        failed_angles = df.loc[failed_mask, 'final_total_tilt_deg']
        
        rollover_count = failed_angles.count()
        rollover_pct = (rollover_count / total_runs) * 100
        
        # Calculate how far they overshoot (only for confirmed rollovers)
        mean_fail = failed_angles.mean() if rollover_count > 0 else 0.0
        max_fail = failed_angles.max() if rollover_count > 0 else 0.0
        
        results[label] = {
            'total': total_runs,
            'rollovers': rollover_count,
            'percentage': rollover_pct,
            'failed_angles': failed_angles.values,
            'mean_fail': mean_fail,
            'max_fail': max_fail
        }
        
        print(f"{label:<15} | {total_runs:<12} | {kinematic_count:<18} | {rollover_count:<12} | {rollover_pct:<11.2f}% | {mean_fail:<14.2f}° | {max_fail:.2f}°")
        
    except FileNotFoundError:
        print(f"[ERROR] Unable to find the CSV file at path: {filepath}")

if not results:
    print("\n[ERROR] No valid CSV files found. Exiting plot generation.")
    exit()

# Bar Plot Generation
labels = list(results.keys())
percentages = [results[label]['percentage'] for label in labels]
counts = [results[label]['rollovers'] for label in labels]

fig1, ax1 = plt.subplots(figsize=(10, 6))
bar_colors = ['#2ecc71', '#f1c40f', '#e74c3c']
bars = ax1.bar(labels, percentages, width=0.5, color=bar_colors, edgecolor='black', alpha=0.85)

ax1.set_ylabel('Confirmed Rollover Probability (%)', fontsize=12)
ax1.set_title(f'Lander Dynamic Rollover Rate by Slope Angle\n(Confirmed: Tilt > {ROLLOVER_THRESHOLD}° & Mass Error < {MASS_ERROR_THRESHOLD}%)', fontsize=14, fontweight='bold')
max_pct = max(percentages) if percentages else 0
ax1.set_ylim(0, max_pct + (max_pct * 0.15) + 5) 
ax1.grid(axis='y', linestyle='--', alpha=0.7)
ax1.set_axisbelow(True)

for bar, count, pct in zip(bars, counts, percentages):
    height = bar.get_height()
    label_text = f"{pct:.1f}%\n({count} runs)"
    ax1.annotate(label_text,
                xy=(bar.get_x() + bar.get_width() / 2, height),
                xytext=(0, 5),
                textcoords="offset points",
                ha='center', va='bottom', fontsize=11, fontweight='bold')

plt.tight_layout()
plot1_filename = os.path.join(OUTPUT_DIR, 'rollover_comparison_bar.png')
plt.savefig(plot1_filename, dpi=300, bbox_inches='tight')
print(f"\nBar plot successfully saved in: {plot1_filename}")

# Boxplot Generation
fig2, ax2 = plt.subplots(figsize=(10, 6))
plot_data = [results[label]['failed_angles'] for label in labels]
bplot = ax2.boxplot(plot_data, patch_artist=True, labels=labels, zorder=3,
                    boxprops=dict(facecolor='#3498db', color='black', alpha=0.7),
                    medianprops=dict(color='red', linewidth=2),
                    flierprops=dict(marker='o', color='black', alpha=0.3, markersize=4))

ax2.axhline(ROLLOVER_THRESHOLD, color='#e74c3c', linestyle='--', linewidth=2, label=f'Threshold ({ROLLOVER_THRESHOLD}°)', zorder=2)

ax2.set_ylabel('Final Total Tilt (Degrees)', fontsize=12)
ax2.set_title('Distribution of Final Tilt Angles for Confirmed Rollover Events', fontsize=14, fontweight='bold')
ax2.grid(axis='y', linestyle='--', alpha=0.7)
ax2.set_axisbelow(True)
ax2.legend(fontsize=11)

plt.tight_layout()
plot2_filename = os.path.join(OUTPUT_DIR, 'rollover_overshoot_boxplot.png')
plt.savefig(plot2_filename, dpi=300, bbox_inches='tight')
print(f"Boxplot successfully saved in: {plot2_filename}")