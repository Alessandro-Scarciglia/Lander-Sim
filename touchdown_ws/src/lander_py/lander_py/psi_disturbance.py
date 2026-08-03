import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64, Bool
from gazebo_msgs.msg import ModelStates
import math
import os
import signal
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

class PSIDisturbance(Node):
    def __init__(self):
        super().__init__('psi_disturbance_node')
        
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])        
        self.model_name = 'lander_model'

        SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))        
        if "install" in SCRIPT_DIR:
            WS_ROOT = SCRIPT_DIR.split("install")[0]            
        else:
            WS_ROOT = SCRIPT_DIR.split("src")[0]
            
        self.plots_dir = os.path.join(WS_ROOT, "plots")
        os.makedirs(self.plots_dir, exist_ok=True)

        # Mesh loading
        MESH_DIR = os.path.join(WS_ROOT, "src", "worlds", "models", "terrain_model", "meshes")
        MESH_PATH = os.path.join(MESH_DIR, "terrain_cropped.stl")
        
        self.get_logger().info(f"Loading terrain mesh from: {MESH_PATH}...")
        try:
            self.mesh = trimesh.load(MESH_PATH)
            self.get_logger().info("Terrain mesh loaded successfully for relative altitude calculation!")
        except FileNotFoundError:
            self.get_logger().error(f"Cannot find mesh at {MESH_PATH}. Using Z=0.0 for ground.")
            self.mesh = None

        self.last_terrain_z = 0.0
        self.last_calc_x = None
        self.last_calc_y = None
        
        # Publisher
        self.pub_north = self.create_publisher(Float64, '/lander/disturbance/north', 10)
        self.pub_east = self.create_publisher(Float64, '/lander/disturbance/east', 10)
        self.pub_south = self.create_publisher(Float64, '/lander/disturbance/south', 10)
        self.pub_west = self.create_publisher(Float64, '/lander/disturbance/west', 10)
        
        # Subscription
        self.sub_tel = self.create_subscription(ModelStates, '/gazebo/model_states', self.telemetry_callback, 10)
        self.sub_cutoff_left = self.create_subscription(Float64, '/lander/thrusters/left', lambda msg: self.thruster_cb(msg, 'left'), 10)
        self.sub_cutoff_central = self.create_subscription(Float64, '/lander/thrusters/central', lambda msg: self.thruster_cb(msg, 'central'), 10)
        self.sub_stop = self.create_subscription(Bool, '/lander/stop_logging', self.stop_cb, 10)

        self.thruster_states = {
            'left': 0.0,
            'central': 0.0 
        }

        self.thrust_detected = False
        self.start_time = None        
        self.node_start_time = None
        self.relative_cutoff_time = None
        self.omega = 3.0   
        self.main_engine_off = False  
        self.is_recording = True    

        self.time_data = []
        self.n_data = []
        self.e_data = []
        self.s_data = []
        self.w_data = []
        
        self.get_logger().info("Disturbance Generator Node started!")

    def get_terrain_elevation(self, x, y, max_z=200.0):
        if self.mesh is None:
            return 0.0
            
        ray_origins = np.array([[x, y, max_z]])
        ray_directions = np.array([[0, 0, -1]])    
        locations, index_ray, index_tri = self.mesh.ray.intersects_location(
            ray_origins=ray_origins, ray_directions=ray_directions
        )
        if len(locations) > 0:
            return np.max(locations[:, 2])
        return 0.0
    
    def stop_cb(self, msg):
        if msg.data and self.is_recording:
            self.get_logger().info("Automatic stop signal received! Freezing data collection...")
            self.is_recording = False
    
    def thruster_cb(self, msg, thruster_name):
        if not self.is_recording:
            return        
        self.thruster_states[thruster_name] = msg.data        
        total_thrust = sum(self.thruster_states.values())        
        
        if total_thrust > 0.0:
            self.thrust_detected = True            
        
        if self.thrust_detected and total_thrust <= 0.0 and not self.main_engine_off:
            self.main_engine_off = True            
            now = self.get_clock().now().nanoseconds / 1e9
            if self.node_start_time is not None:
                self.relative_cutoff_time = now - self.node_start_time                
            

    def calculate_quadratic_disturbance(self, h):        
        if h >= 10.0:
            return 0.0
            
        # Quadratic interpolation
        a = 2.587
        b = -41.800
        c = 159.333
        
        force = (a * (h ** 2)) + (b * h) + c
        
        # Safety saturation
        return max(0.0, min(force, 500.0))

    def telemetry_callback(self, msg):  
        if not self.is_recording:
            return     
        now = self.get_clock().now()        
        
        if now.nanoseconds == 0:
            return
        
        current_time_sec = now.nanoseconds / 1e9

        if self.node_start_time is None:
            self.node_start_time = current_time_sec

        rel_time = current_time_sec - self.node_start_time

        if self.main_engine_off:
            self.publish_forces(0.0, 0.0, 0.0, 0.0, rel_time)
            return
        
        
        try:
            # Find the lander's index
            index = msg.name.index(self.model_name)

            # Absolute position
            x_pos = msg.pose[index].position.x
            y_pos = msg.pose[index].position.y
            z_abs = msg.pose[index].position.z

            dist_sq = 0.0
            if self.last_calc_x is not None:
                dist_sq = (x_pos - self.last_calc_x)**2 + (y_pos - self.last_calc_y)**2
                
            if self.last_calc_x is None or dist_sq > 0.01:
                self.last_terrain_z = self.get_terrain_elevation(x_pos, y_pos)
                self.last_calc_x = x_pos
                self.last_calc_y = y_pos
            
            # Extract the true altitude
            true_altitude = z_abs - self.last_terrain_z

                        
            # Calculate the total magnitude of the disturbance
            total_force = self.calculate_quadratic_disturbance(true_altitude)                   

            
            if total_force == 0.0:
                self.publish_forces(0.0, 0.0, 0.0, 0.0, rel_time)
                return
            
            if self.start_time is None:
                self.start_time = now
                self.get_logger().info("Simulated Disturbance Profile Started.")

            # Elapsed time
            t_elapsed = (now - self.start_time).nanoseconds / 1e9
            phase = self.omega * t_elapsed
            
            # Sinusoidal distribution
            f_north = total_force * max(0.0, math.sin(phase))
            f_east  = total_force * max(0.0, math.cos(phase))
            f_south = total_force * max(0.0, -math.sin(phase))
            f_west  = total_force * max(0.0, -math.cos(phase))
            
            # Publish
            self.publish_forces(f_north, f_east, f_south, f_west, rel_time)

        except ValueError:            
            pass

    def publish_forces(self, n, e, s, w, rel_time):
        # Log data for plot
        self.time_data.append(rel_time)
        self.n_data.append(float(n))
        self.e_data.append(float(e))
        self.s_data.append(float(s))
        self.w_data.append(float(w))

        msg_n = Float64()
        msg_e = Float64()
        msg_s = Float64()
        msg_w = Float64()
        
        msg_n.data = float(n)
        msg_e.data = float(e)
        msg_s.data = float(s)
        msg_w.data = float(w)
        
        self.pub_north.publish(msg_n)
        self.pub_east.publish(msg_e)
        self.pub_south.publish(msg_s)
        self.pub_west.publish(msg_w)

def main(args=None):
    rclpy.init(args=args)
    node = PSIDisturbance()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        ENABLE_PLOTS = os.environ.get('MC_ENABLE_PLOTS', 'True').lower() == 'true'
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        
        t_data = list(node.time_data)
        n_data = list(node.n_data)
        e_data = list(node.e_data)
        s_data = list(node.s_data)
        w_data = list(node.w_data)
        cutoff_line = node.relative_cutoff_time        
        

        if len(t_data) == 0:
            print("WARNING: No data collected. Skipping plot generation.")
            return

        if ENABLE_PLOTS:
            print("\n[PSIDisturbance Logger] Starting chart generation...")
            # Plot
            fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
            fig.suptitle('Disturbance Forces', fontsize=16, fontweight='bold')

            datasets = [
                (n_data, 'red', 'North Disturbance'),
                (e_data, 'blue', 'East Disturbance'),
                (s_data, 'green', 'South Disturbance'),
                (w_data, 'orange', 'West Disturbance')
            ]

            for i, (data, color, title) in enumerate(datasets):
                axs[i].plot(t_data, data, color=color, linewidth=1.5, label='Force Detected')
                axs[i].set_title(title)
                axs[i].set_ylabel('Force [N]')
                axs[i].grid(True, linestyle=':', alpha=0.7)
                
                # Cut-off line
                if cutoff_line is not None:
                    axs[i].axvline(x=cutoff_line, color='black', linestyle='--', alpha=0.8, label='MECO (Cutoff)' if i==0 else "")
                
                axs[i].legend(loc='upper right', fontsize=9)

            axs[3].set_xlabel('Simulation Time [s]')
            plt.tight_layout(rect=[0, 0.03, 1, 0.96])

            save_path = os.path.join(node.plots_dir, '04_disturbance_profile.png')
            fig.savefig(save_path, dpi=300)

            print(f"Plot saved: {save_path}")
            


if __name__ == '__main__':
    main()