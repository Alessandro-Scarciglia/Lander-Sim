import os
import subprocess
import time

# Parameters configuration
NUM_RUNS = 100             
LAUNCH_FILE = 'headless.launch.py'

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

CONFIG_PATH = os.environ.get('TOUCHDOWN_CONFIG_FILE', os.path.join(WORKSPACE_DIR, "cfg", "simulation.yaml"))
LAUNCH_FILE_PATH = os.path.join(WORKSPACE_DIR, "launch", LAUNCH_FILE)

LAUNCH_COMMAND = ['ros2', 'launch', LAUNCH_FILE_PATH]


def main():
    print("==================================================================")
    print(f" STARTING AUTOMATED MONTE CARLO ANALYSIS ({NUM_RUNS} RUNS)")
    print("==================================================================")
    print(f" Config Path: {CONFIG_PATH}")
    print(f" Launch Command: {' '.join(LAUNCH_COMMAND)}")
    print("==================================================================")

    os.environ['MC_ENABLE_PLOTS'] = 'False'
    os.environ['TOUCHDOWN_WS_ROOT'] = WORKSPACE_DIR

    statistics = {"successes": 0, "failures": 0}
    absolute_start_time = time.time()

    for run_id in range(1, NUM_RUNS + 1):
        print(f"\n[RUN {run_id}/{NUM_RUNS}] Starting simulation cycle...")
        
        try:
            os.environ['MC_RUN_ID'] = str(run_id)
            os.environ['SIMULATION_RUN_ID'] = str(run_id)            
            print(f"[RUN {run_id}] Background execution (Generation + GzServer)...")
            run_start_time = time.time()            
            subprocess.run(LAUNCH_COMMAND, check=True)            
            run_duration = time.time() - run_start_time
            print(f"[RUN {run_id}] Execution completed in {run_duration:.1f} seconds.")
            statistics["successes"] += 1
            
        except subprocess.CalledProcessError as e:
            print(f"[RUN {run_id}] SOFTWARE ERROR: Failure with exit code {e.returncode}.")
            statistics["failures"] += 1
            
        except Exception as e:
            print(f"[RUN {run_id}] Unexpected script error: {e}")
            statistics["failures"] += 1

        
        time.sleep(2.0)

    # Final calculation
    total_time = time.time() - absolute_start_time
    print("\n==================================================================")
    print(" MONTE CARLO ORCHESTRATOR COMPLETED!")
    print("==================================================================")
    print(f" Total scheduled runs: {NUM_RUNS}")
    print(f" Completed without crashes: {statistics['successes']}")
    print(f" Simulator crashes:    {statistics['failures']}")
    print(f"  Total time elapsed:  {total_time/60:.2f} minutes")
    print("==================================================================")

if __name__ == "__main__":
    main()