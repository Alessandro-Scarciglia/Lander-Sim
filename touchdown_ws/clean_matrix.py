import pandas as pd
import os

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
# Assicurati che il nome del file corrisponda a quello che hai nella cartella plots!
CSV_IN_PATH = os.path.join(CURRENT_DIR, 'plots', 'montecarlo_summary.csv')
CSV_OUT_PATH = os.path.join(CURRENT_DIR, 'plots', 'montecarlo_summary_clean.csv')

def main():
    print("Lettura del dataset originale...")
    df = pd.read_csv(CSV_IN_PATH)   
    
    df = df.sort_values('run_id').reset_index(drop=True)    
    
    df['traj_id'] = (df['run_id'] - 1) // 28    
   
    traj_counts = df['traj_id'].value_counts()    
    
    broken_trajs = traj_counts[traj_counts != 28].index.tolist()
    
    print("\n" + "="*50)
    print(f" Trovate {len(broken_trajs)} traiettorie incomplete/rotte: {broken_trajs}")
    print("="*50)    
    
    df_clean = df[~df['traj_id'].isin(broken_trajs)].copy()    
    
    df_clean = df_clean.drop(columns=['traj_id'])
    
    num_trajectories = len(df_clean) // 28
    
    print(f"\nRun originali trovate: {len(df)}")
    print(f"Run rimosse (appartenenti a traiettorie rotte): {len(df) - len(df_clean)}")
    print(f"Run perfette salvate: {len(df_clean)}")
    print(f"Numero di Traiettorie valide per SALib: {num_trajectories} (su 30 iniziali)")    
    
    df_clean.to_csv(CSV_OUT_PATH, index=False)
    print(f"\nDataset pulito salvato con successo in: {CSV_OUT_PATH}")

if __name__ == "__main__":
    main()