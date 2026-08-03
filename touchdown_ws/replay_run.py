import os
import subprocess
import argparse

def main():
    parser = argparse.ArgumentParser(description="Replay a specific Monte Carlo run.")
    parser.add_argument("run_id", type=str, help="Run ID")    
    parser.add_argument("--launch", type=str, default="start_simulation.launch.py", help="ROS 2 launch file name")
    args = parser.parse_args()
    
    os.environ['SIMULATION_RUN_ID'] = str(args.run_id)

    print(f"\n[LAUNCH] >>> Starting Gazebo and nodes for single Run {args.run_id} <<<")
    print(f"[INFO] Random Seed fissato a {args.run_id} per masse e cinematica.\n")
    
    try:            
        subprocess.run(['ros2', 'launch', args.launch], check=True)
    except subprocess.CalledProcessError as e:
        print(f"\n[ERROR] Simulation terminated with error: {e}")
    except KeyboardInterrupt:
        print("\n[INFO] Simulation interrupted by user via keyboard (Ctrl+C).")

if __name__ == "__main__":
    main()

