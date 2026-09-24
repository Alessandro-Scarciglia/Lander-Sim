"""Find and plot terrain regions whose slope challenges lander stability.

The analysis ray-casts a height map from the terrain mesh, computes local
slopes, filters candidate regions, and saves a map of separated target spots.
"""

# Numerical, mesh, plotting, filtering, and path imports
import numpy as np
import trimesh
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter
import os

# Terrain analysis and visualization
def analyze_terrain(mesh_path, lander_radius=2.5, num_challenging_spots=5, min_dist_between_spots=15.0):
    print(f"Loading mesh: {mesh_path}...")
    mesh = trimesh.load(mesh_path)    
    
    min_bounds, max_bounds = mesh.bounds
    
    # 2D grid
    resolution = 0.2 
    x_grid = np.arange(min_bounds[0], max_bounds[0], resolution)
    y_grid = np.arange(min_bounds[1], max_bounds[1], resolution)
    X, Y = np.meshgrid(x_grid, y_grid)
    
    # Ray-casting
    ray_origins = np.column_stack((X.flatten(), Y.flatten(), np.full(X.size, max_bounds[2] + 10.0)))
    ray_directions = np.tile([0, 0, -1], (X.size, 1))
    
    print("Ray-casting execution for the Heightmap...")
    locations, index_ray, _ = mesh.ray.intersects_location(
        ray_origins=ray_origins, 
        ray_directions=ray_directions
    )
    
    # SAFETY CHECK 1
    if len(locations) == 0:
        print("\nThe laser beams did not intersect anything")
        return None, None

    # Z matrix creation
    Z = np.full(X.shape, np.nan) 
    
    for loc, idx in zip(locations, index_ray):
        row = idx // X.shape[1]
        col = idx % X.shape[1]
        if np.isnan(Z[row, col]) or loc[2] > Z[row, col]:
            Z[row, col] = loc[2]
            
    # SAFETY CHECK 2
    if np.all(np.isnan(Z)):
        print("\nThe Z elevation grid is completely empty.")
        return None, None
            
    # Spatial gradient
    print("Calculation of the spatial gradient of the map...")
    dy, dx = np.gradient(Z, resolution, resolution)    
    slope_rad = np.arctan(np.sqrt(dx**2 + dy**2))
    slope_deg = np.degrees(slope_rad)
    
    # Lander footprint
    slope_deg_safe = np.nan_to_num(slope_deg, nan=90.0)
    
    footprint_pixels = int(lander_radius / resolution)
    avg_slope_deg = uniform_filter(slope_deg_safe, size=footprint_pixels, mode='nearest')
    
    avg_slope_deg[np.isnan(Z)] = np.nan
    
    # Margine di sicurezza dal bordo della mappa
    margin = lander_radius + 2.0 
    valid_area = (X > min_bounds[0] + margin) & (X < max_bounds[0] - margin) & \
                 (Y > min_bounds[1] + margin) & (Y < max_bounds[1] - margin)
    
    
    # Spots searching
    chal_min, chal_max = 8.0, 20.0
    chal_mask = (avg_slope_deg >= chal_min) & (avg_slope_deg <= chal_max) & valid_area
    
    if not np.any(chal_mask):
        # Fallback 
        chal_mask = (avg_slope_deg >= 7.0) & valid_area

    challenging_spots = []
    
    if np.any(chal_mask):        
        valid_rows, valid_cols = np.where(chal_mask)
        valid_X = X[valid_rows, valid_cols]
        valid_Y = Y[valid_rows, valid_cols]
        valid_slopes = avg_slope_deg[valid_rows, valid_cols]      
        
        sort_indices = np.argsort(valid_slopes)[::-1]
        valid_X = valid_X[sort_indices]
        valid_Y = valid_Y[sort_indices]
        valid_slopes = valid_slopes[sort_indices]        
        
        for cx, cy, cslope in zip(valid_X, valid_Y, valid_slopes):
            too_close = False
            for (sx, sy, sslope) in challenging_spots:                
                dist = np.sqrt((cx - sx)**2 + (cy - sy)**2)
                if dist < min_dist_between_spots:
                    too_close = True
                    break
            
            if not too_close:
                challenging_spots.append((cx, cy, cslope))
                if len(challenging_spots) >= num_challenging_spots:
                    break

    print(f"\n--- CHALLENGING SPOTS ---")
    if len(challenging_spots) == 0:
        print("Challenging spots not found.")
    else:
        for i, (cx, cy, cs) in enumerate(challenging_spots):
            print(f"🟠 [C{i+1}]   X, Y : [{cx:.2f}, {cy:.2f}] | Slope: {cs:.2f}°")    
    
    plot_results(X, Y, Z, avg_slope_deg, challenging_spots, lander_radius)
    
    return challenging_spots

def plot_results(X, Y, Z, slope, chal_spots, radius):
    fig, ax = plt.subplots(figsize=(10, 8))    
    
    c = ax.pcolormesh(X, Y, slope, shading='auto', cmap='Reds', vmax=20.0)
    ax.set_title("Slope Map (Spatial Derivative)")
    fig.colorbar(c, ax=ax, label='Slope [Degrees]')    
    
    for i, (cx, cy, cs) in enumerate(chal_spots):
        label = f'Spot {i+1} ({cs:.1f}°)'            
        ax.plot(cx, cy, 'X', color='blue', markersize=10)        
        ax.add_patch(plt.Circle((cx, cy), radius, color='blue', fill=False, linewidth=2, linestyle='--'))        
        ax.text(cx + 3, cy + 3, label, color='blue', fontweight='bold', 
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='blue', pad=1))
        
    if chal_spots:
        ax.legend(['Target Spots'], loc='upper right')
    
    plt.tight_layout()
    save_path = os.path.join(WS_ROOT, "figures", "terrain", "target_spots_analysis.png")
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\nImage successfully saved to: {save_path}")

# Esecution
WS_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

MESH_DIR = os.path.join(WS_ROOT, "src", "worlds", "models", "terrain_model", "meshes")
mesh_path = os.path.join(MESH_DIR, "terrain_col_light.stl")

challenging_spots = analyze_terrain(mesh_path, lander_radius=2.5, num_challenging_spots=10, min_dist_between_spots=15.0)