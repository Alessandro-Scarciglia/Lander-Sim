import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from gazebo_msgs.msg import ModelStates, ContactsState
import matplotlib
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import math
from std_msgs.msg import Bool
import re
import signal
import os
import yaml
import csv
from std_msgs.msg import Float64
import numpy as np
import trimesh

class FlightDataLogger(Node):
    def __init__(self, target_model="lander_model"):
        super().__init__('flight_data_logger')

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.target_model = target_model  

        SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))        
        if "install" in SCRIPT_DIR:
            self.ws_root = SCRIPT_DIR.split("install")[0]            
        else:
            self.ws_root = SCRIPT_DIR.split("src")[0]

        CONFIG_PATH = os.path.join(self.ws_root, "config.yaml")
        self.plots_dir = os.path.join(self.ws_root, "plots")
        os.makedirs(self.plots_dir, exist_ok=True)

        try:
            with open(CONFIG_PATH, 'r') as file:
                self.config = yaml.safe_load(file)
            self.get_logger().info("Config.yaml file successfully uploaded to the Data Logger!")
        except FileNotFoundError:
            self.get_logger().error(f"Unable to find the config.yaml file at the path: {CONFIG_PATH}")
            # Fallback
            self.config = {"simulation_stop": {"epsilon": 0.05, "timeout_after_touchdown": 5.0, "steady_req_duration": 1.0}}      
        
        self.times_tel = []
        self.start_time_tel = None
        self.times_forces = []
        self.start_time_forces = None
        self.impact_thrust_total = 0.0        

        self.shutdown_initiated = False
        self.is_recording = True  

        self.pos_x, self.pos_y, self.pos_z = [], [], []
        self.vel_x, self.vel_y, self.vel_z = [], [], []
        self.roll, self.pitch, self.yaw = [], [], []
        self.ang_x, self.ang_y, self.ang_z = [], [], []        
        
        self.legs = ['leg1', 'leg2', 'leg3', 'leg4']
        self.axes = ['x', 'y', 'z']
        self.history_forces = {leg: {axis: [] for axis in self.axes} for leg in self.legs} 
        self.first_leg_touch = None
        self.current_f = {
            leg: {'x':0,'y':0,'z':0}
            for leg in self.legs
        }       
                
        # Shutdown parameters (STEADY STATE & TIMEOUT)
        sim_stop_cfg = self.config.get("simulation_stop", {})
        self.epsilon = sim_stop_cfg.get("epsilon", 0.05)                 
        self.timeout_after_touchdown = sim_stop_cfg.get("timeout_after_touchdown", 5.0) 
        self.steady_req_duration = sim_stop_cfg.get("steady_req_duration", 1.0) 

        self.get_logger().info("=== Configuration ===")
        self.get_logger().info(f" > Epsilon : {self.epsilon}")
        self.get_logger().info(f" > Timeout post-Touchdown : {self.timeout_after_touchdown} s")
        self.get_logger().info(f" > Steady State duration : {self.steady_req_duration} s")
        self.get_logger().info("=========================================")
        
        self.touchdown_time = None           
        self.steady_start_time = None        
        
        # Telemetry Subscription
        self.sub_tel = self.create_subscription(ModelStates, '/gazebo/model_states', self.telemetry_callback, 10)
            
        # Impact forces Subscription
        self.sub_imp = self.create_subscription(ContactsState, '/lander/contact_forces', self.contact_callback, 10)

        # Stop Logging Publisher
        self.pub_stop = self.create_publisher(Bool, '/lander/stop_logging', 10)
        
        # Td time Publisher
        self.pub_touchdown = self.create_publisher(Float64, '/lander/touchdown_time', 10)

        # Thrust Subscription
        self.sub_impact_thrust = self.create_subscription(Float64, '/lander/impact_thrust', self.impact_thrust_callback, 10)
            
        self.get_logger().info(f"Logger Node started. Listening to {self.target_model}...")
        self.get_logger().info(">>> AUTO-STOP ENABLED: The node will stop at steady state or timeout <<<")

    
    def _trigger_exit(self):
        self.get_logger().info("Master node initiating global shutdown sequence...")
        raise SystemExit("Simulation sequence completed.")

    def euler_from_quaternion(self, x, y, z, w):        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z

    def impact_thrust_callback(self, msg):
        if not self.is_recording:
            return            
        self.impact_thrust_total = msg.data
        self.get_logger().info(f"Received impact thrust from ThrustLogger: {self.impact_thrust_total:.2f} N")

    def telemetry_callback(self, msg):
        if not self.is_recording:
            return
        
        if self.target_model in msg.name:
            # Get the model index
            idx = msg.name.index(self.target_model)
            
            now = self.get_clock().now().nanoseconds / 1e9
            if self.start_time_tel is None:
                self.start_time_tel = now
            t = now - self.start_time_tel

            # Linear Data
            self.pos_x.append(msg.pose[idx].position.x)
            self.pos_y.append(msg.pose[idx].position.y)
            self.pos_z.append(msg.pose[idx].position.z)
            
            vx = msg.twist[idx].linear.x
            vy = msg.twist[idx].linear.y
            vz = msg.twist[idx].linear.z
            self.vel_x.append(vx)
            self.vel_y.append(vy)
            self.vel_z.append(vz)
            
            # Angular Data
            q = msg.pose[idx].orientation
            r, p, y = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
            self.roll.append(math.degrees(r))
            self.pitch.append(math.degrees(p))
            self.yaw.append(math.degrees(y))
            
            wx = msg.twist[idx].angular.x
            wy = msg.twist[idx].angular.y
            wz = msg.twist[idx].angular.z
            self.ang_x.append(wx)
            self.ang_y.append(wy)
            self.ang_z.append(wz)
            
            self.times_tel.append(t)
            
            # Steady State and Timeout logic
            if self.touchdown_time is not None:
                
                # Timeout Control
                if (now - self.touchdown_time) > self.timeout_after_touchdown:
                    if not self.shutdown_initiated:
                        self.shutdown_initiated = True
                        self.is_recording = False
                        self.get_logger().info("TIMEOUT reached after touchdown.")
                        msg = Bool()
                        msg.data = True
                        self.pub_stop.publish(msg)                         
                        self.create_timer(1.5, self._trigger_exit)

                # Steady State Control (Speed below epsilon)
                is_steady = (
                    abs(vx) < self.epsilon and abs(vy) < self.epsilon and abs(vz) < self.epsilon and
                    abs(wx) < self.epsilon and abs(wy) < self.epsilon and abs(wz) < self.epsilon
                )

                if is_steady:
                    if self.steady_start_time is None:
                        self.steady_start_time = now 
                    elif (now - self.steady_start_time) > self.steady_req_duration:
                        if not self.shutdown_initiated:
                            self.shutdown_initiated = True
                            self.is_recording = False
                            self.get_logger().info(f"STEADY STATE reached (Variables < {self.epsilon}).")                       
                            msg = Bool()
                            msg.data = True
                            self.pub_stop.publish(msg)                        
                            self.create_timer(1.5, self._trigger_exit)
                else:
                    self.steady_start_time = None 
           

    def contact_callback(self, msg):
        if not self.is_recording:
            return
        
        now = self.get_clock().now().nanoseconds / 1e9

        if self.start_time_forces is None:
            self.start_time_forces = now

        t = now - self.start_time_forces
        self.times_forces.append(t)

        contact_counts = {leg: 0 for leg in self.legs}
        
        new_f = {
            leg: {'x': 0.0, 'y': 0.0, 'z': 0.0}
            for leg in self.legs
        }

        for state in msg.states:
            col_name = state.collision1_name
            for leg in self.legs:
                if leg in col_name:
                    new_f[leg]['x'] += state.total_wrench.force.x
                    new_f[leg]['y'] += state.total_wrench.force.y
                    new_f[leg]['z'] += state.total_wrench.force.z
                    contact_counts[leg] += 1

        for leg in self.legs:
            if contact_counts[leg] > 0:
                self.current_f[leg]['x'] = (new_f[leg]['x'] / contact_counts[leg])
                self.current_f[leg]['y'] = (new_f[leg]['y'] / contact_counts[leg])
                self.current_f[leg]['z'] = (new_f[leg]['z'] / contact_counts[leg])                                

        

        # Save force history
        for leg in self.legs:
            for axis in self.axes:
                self.history_forces[leg][axis].append(self.current_f[leg][axis])

        # Touchdown detection
        if self.touchdown_time is None:
            for leg in self.legs:
                if abs(self.current_f[leg]['z']) > 5.0:
                    self.touchdown_time = now
                    self.first_leg_touch = leg
                    self.get_logger().info(f"TOUCHDOWN DETECTED! ({leg}). Timers started.")
                    msg_td = Float64()
                    msg_td.data = now 
                    self.pub_touchdown.publish(msg_td)
                    break         

       
def get_yaw_rot_matrix(yaw):
    return np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw),  math.cos(yaw), 0],
        [0,             0,             1]
    ])

def steiner_transport(I_cg, mass, pos_cg, new_ref_point):    
    d = np.array(pos_cg) - np.array(new_ref_point)
    d_sq = np.dot(d, d)
    return np.array(I_cg) + mass * (d_sq * np.eye(3) - np.outer(d, d))

def read_body_metadata(filepath):
    mass, cog, I = 0.0, np.zeros(3), np.zeros((3,3))
    try:
        with open(filepath, 'r') as file:
            content = file.read()
            # Mass
            m_match = re.search(r'<!-- METADATA_WET_MASS:\s*([0-9]*\.?[0-9]+)\s*-->', content)
            if m_match: mass = float(m_match.group(1))
            
            # CoG
            cog_match = re.search(r'<!-- METADATA_BODY_COG:\s*([^>]+)-->', content)
            if cog_match:
                vals = [float(x.strip()) for x in cog_match.group(1).split(';') if x.strip()]
                if len(vals) >= 3: cog = np.array(vals[:3])
                
            # Inertia
            i_match = re.search(r'<!-- METADATA_BODY_INERTIA:\s*([^>]+)-->', content)
            if i_match:
                vals = [float(x.strip()) for x in i_match.group(1).split(';') if x.strip()]
                if len(vals) >= 9: I = np.array(vals[:9]).reshape((3,3))
    except FileNotFoundError:
        pass
    return mass, cog, I

def read_passenger_metadata(filepath):
    mass, cog, I = 0.0, np.zeros(3), np.zeros((3,3))
    try:
        with open(filepath, 'r') as file:
            content = file.read()
            # Mass
            m_match = re.search(r'<!-- METADATA_PASSENGER_MASS:\s*([0-9]*\.?[0-9]+)\s*-->', content)
            if m_match: mass = float(m_match.group(1))
            
            # CoG
            cog_match = re.search(r'<!-- METADATA_PASSENGER_COG:\s*([^>]+)-->', content)
            if cog_match:
                vals = [float(x.strip()) for x in cog_match.group(1).split(';') if x.strip()]
                if len(vals) >= 3: cog = np.array(vals[:3])
                
            # Inertia
            i_match = re.search(r'<!-- METADATA_PASSENGER_INERTIA:\s*([^>]+)-->', content)
            if i_match:
                vals = [float(x.strip()) for x in i_match.group(1).split(';') if x.strip()]
                if len(vals) >= 9: I = np.array(vals[:9]).reshape((3,3))
    except FileNotFoundError:
        pass
    return mass, cog, I


def read_leg_physics_metadata(filepath):
    mass, cog, I = 0.0, np.zeros(3), np.zeros((3,3))
    try:
        with open(filepath, 'r') as file:
            content = file.read()
            match = re.search(r'<!-- METADATA_LEG_PROPERTIES:(.+?)-->', content, re.DOTALL)
            if match:
                props = {}
                for line in match.group(1).split('\n'):
                    if '=' in line:
                        k, v = line.split('=')
                        props[k.strip()] = v.strip()
                
                mass = float(props.get('mass', 0.0))
                cog = np.array([float(props.get('cog_x', 0)), float(props.get('cog_y', 0)), float(props.get('cog_z', 0))])
                I = np.array([
                    [float(props.get('ixx', 0)), float(props.get('ixy', 0)), float(props.get('ixz', 0))],
                    [float(props.get('ixy', 0)), float(props.get('iyy', 0)), float(props.get('iyz', 0))],
                    [float(props.get('ixz', 0)), float(props.get('iyz', 0)), float(props.get('izz', 0))]
                ])
    except FileNotFoundError:
        pass
    return mass, cog, I

def read_dry_mass_from_sdf(filepath):        
    try:
        with open(filepath, 'r') as file:
            content = file.read()             
            match = re.search(r'<!-- METADATA_DRY_MASS:\s*([0-9]*\.?[0-9]+)\s*-->', content)
            if match: 
                return float(match.group(1))
    except FileNotFoundError:
        pass
    return 0.0

def read_leg_params_from_sdf(filepath):
    params = {}
    try:
        with open(filepath, 'r') as file:
            content = file.read()            
            match = re.search(r'<!-- METADATA_LEG_PARAMS:\s*(.+?)-->', content)
            if match:
                params_str = match.group(1)                
                for pair in params_str.split(';'):
                    if '=' in pair:
                        k, v = pair.split('=')
                        params[k.strip()] = float(v.strip())
    except FileNotFoundError:
        print(f"[WARN] File {filepath} non trovato per la lettura dei parametri.")
    return params


def main(args=None):
    rclpy.init(args=args)
    node = FlightDataLogger(target_model="lander_model")
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as e:        
        pass    
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        ENABLE_PLOTS = os.environ.get('MC_ENABLE_PLOTS', 'True').lower() == 'true'
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        # Data extraction        
        t_tel = list(node.times_tel)
        t_forces = list(node.times_forces)
        px, py, pz = list(node.pos_x), list(node.pos_y), list(node.pos_z)
        vx, vy, vz = list(node.vel_x), list(node.vel_y), list(node.vel_z)
        ro, pi, ya = list(node.roll), list(node.pitch), list(node.yaw)
        wx, wy, wz = list(node.ang_x), list(node.ang_y), list(node.ang_z)        
        f_imp = {leg: {ax: list(node.history_forces[leg][ax]) for ax in node.axes} for leg in node.legs}

        # Read metadata
        model_dir = os.path.join(node.ws_root, "src", "worlds", "models", "lander_model")
        mass_body, cog_body_local, I_body_local = read_body_metadata(os.path.join(model_dir, "body", "body.sdf"))
        mass_pass, cog_pass_local, I_pass_local = read_passenger_metadata(os.path.join(model_dir, "passenger", "passenger.sdf"))
        mass_leg, cog_leg_local, I_leg_local = read_leg_physics_metadata(os.path.join(model_dir, "leg1", "leg1.sdf"))
        dry_mass = read_dry_mass_from_sdf(os.path.join(model_dir, "body", "body.sdf"))

        # Legs calculation        
        leg_cfg = node.config["leg_generation"]
        r_octagon = leg_cfg.get("octagon_radius", 2.0)
        z_legs = leg_cfg.get("z_legs", 0.05)
        dist_c1_c2 = math.sqrt((leg_cfg["base_points"]["C2"][0]-leg_cfg["base_points"]["C1"][0])**2 + 
                               (leg_cfg["base_points"]["C2"][1]-leg_cfg["base_points"]["C1"][1])**2 + 
                               (leg_cfg["base_points"]["C2"][2]-leg_cfg["base_points"]["C1"][2])**2) if "base_points" in leg_cfg else 1.0

        x_leg_local = r_octagon
        y_leg_local = -dist_c1_c2 / 2.0

        # Offset of RF1 (Body) and RF2 (Passenger) in global frame
        Z_OFFSET_RF1 = 0.0 
        Z_OFFSET_RF2 = 2.464

        parts = []

        # Add Body
        parts.append({
            "mass": mass_body,
            "cog_global": cog_body_local + np.array([0, 0, Z_OFFSET_RF1]),
            "I_global": I_body_local  # No rotation for body
        })
        
        # Add Passenger
        parts.append({
            "mass": mass_pass,
            "cog_global": cog_pass_local + np.array([0, 0, Z_OFFSET_RF2]),
            "I_global": I_pass_local  # No rotation for passenger
        })

        # Add 4 Legs
        mass_legs_tot = 0.0
        for i in range(4):
            yaw_leg = i * (math.pi / 2.0)
            R_leg = get_yaw_rot_matrix(yaw_leg)
            
            x_g = x_leg_local * math.cos(yaw_leg) - y_leg_local * math.sin(yaw_leg)
            y_g = x_leg_local * math.sin(yaw_leg) + y_leg_local * math.cos(yaw_leg)
            pos_origin_global = np.array([x_g, y_g, z_legs])
            
            cog_leg_global = pos_origin_global + R_leg @ cog_leg_local
            I_leg_rot = R_leg @ I_leg_local @ R_leg.T
            
            parts.append({
                "mass": mass_leg,
                "cog_global": cog_leg_global,
                "I_global": I_leg_rot
            })
            mass_legs_tot += mass_leg


        # Calculate Total Mass and Total CoG
        total_mass = sum(p["mass"] for p in parts)
        if total_mass > 0:
            tot_cog = sum(p["mass"] * p["cog_global"] for p in parts) / total_mass
        else:
            tot_cog = np.zeros(3)

        # Calculate Total Inertia 
        tot_I = np.zeros((3, 3))
        for p in parts:
            tot_I += steiner_transport(p["I_global"], p["mass"], p["cog_global"], tot_cog)
        

        # Mass error calculation
        if len(t_forces) > 0:            
            final_fz = sum([f_imp[leg]['z'][-1] for leg in node.legs if len(f_imp[leg]['z']) > 0])            
            gazebo_mass = abs(final_fz) / 1.625                 
            mass_error_kg = gazebo_mass - total_mass
            mass_error_pct = (mass_error_kg / total_mass) * 100.0 if total_mass > 0 else 0.0            
            print("\n" + "="*50)
            print(f"[MASS CHECK] Real total mass: {total_mass:.2f} kg")
            print(f"[MASS CHECK] Gazebo mass: {gazebo_mass:.2f} kg")
            print(f"[MASS CHECK] Steady-State error: {mass_error_kg:.2f} kg ({mass_error_pct:.2f}%)")
            print("="*50 + "\n")
        
        # Leg parameters extraction
        params_names = [
            "foot_k", "foot_c", "be_crush_force", "be_rebound_damping",
            "b_k", "b_c", "df_bottomout_stiffness", "df_rebound_damping",
            "df_crush_force_stage1", "df_crush_force_stage2", "df_k", "df_c"
        ]
        
        legs_sdf_path = os.path.join(model_dir, "leg1", "leg1.sdf") 
        extracted_leg_params = read_leg_params_from_sdf(legs_sdf_path)        
        leg_params_values = [extracted_leg_params.get(p, 0.0) for p in params_names]

        # Relative height calculation
        terrain_path = os.path.join(node.ws_root, "src", "worlds", "models", "terrain_model", "meshes", "terrain_cropped.stl")
        terrain_mesh = None
        pz_rel = list(pz) 

        try:
            terrain_mesh = trimesh.load(terrain_path)
            if len(px) > 0:                
                ray_origins_all = np.column_stack((px, py, np.full(len(px), 100.0)))
                ray_directions_all = np.tile([0, 0, -1], (len(px), 1))
                
                locations, index_ray, _ = terrain_mesh.ray.intersects_location(
                    ray_origins=ray_origins_all,
                    ray_directions=ray_directions_all
                )
                
                # Multiple intersection
                z_terr_dict = {}
                for loc, idx in zip(locations, index_ray):
                    if idx not in z_terr_dict or loc[2] > z_terr_dict[idx]:
                        z_terr_dict[idx] = loc[2]
                
                # Terrain height
                for i in range(len(pz)):
                    if i in z_terr_dict:
                        pz_rel[i] = pz[i] - z_terr_dict[i]
        except Exception as e:
            print(f"Warning: Failed to calculate relative Z from the mesh ({e})")

        min_belly_clearance_m = 0.0

        if ENABLE_PLOTS and len(t_tel) > 0:
            print("\n--- INITIAL VALUES RECORDED (t=0) ---")
            print(f"Position [m]:        X= {px[0]:.4f} | Y= {py[0]:.4f} | Z= {pz_rel[0]:.4f}")
            print(f"Attitude [Degree]:   R= {ro[0]:.4f} | P= {pi[0]:.4f} | Y= {ya[0]:.4f}")
            print(f"Linear Vel. [m/s]:   Vx={vx[0]:.4f} | Vy={vy[0]:.4f} | Vz={vz[0]:.4f}")
            print(f"Angular Vel. [rad/s]: Wx={wx[0]:.4f} | Wy={wy[0]:.4f} | Wz={wz[0]:.4f}")
            print("\n--- FINAL VALUES RECORDED (t=end) ---")
            print(f"Final Height (AGL) [m]: Z(Rel)= {pz_rel[-1]:.4f}")
            print("----------------------------------------\n")
            print("----------------------------------------\n")

        # Simulation parameters and CSV saving   

        # Initial Condition        
        if len(t_tel) > 0:
            init_x, init_y, init_z = px[0], py[0], pz_rel[0]
            init_vx, init_vy, init_vz = vx[0], vy[0], vz[0]
            init_roll, init_pitch, init_yaw = ro[0], pi[0], ya[0]
            init_wx, init_wy, init_wz = wx[0], wy[0], wz[0]
        else:
            init_x = init_y = init_z = init_vx = init_vy = init_vz = 0.0
            init_roll = init_pitch = init_yaw = init_wx = init_wy = init_wz = 0.0     
        
        if len(t_tel) > 0:
            # Find the array index corresponding to the moment of touchdown.
            td_index = 0
            if node.touchdown_time:
                td_time_relative = node.touchdown_time - node.start_time_tel            
                td_index = min(range(len(t_tel)), key=lambda i: abs(t_tel[i] - td_time_relative))
            else:
                td_index = len(t_tel) - 1

            # Impact velocity
            vx_imp = vx[td_index]
            vy_imp = vy[td_index]
            vz_imp = vz[td_index]
            v_abs_imp = math.sqrt(vx_imp**2 + vy_imp**2 + vz_imp**2)
            
            # Specific kinetic energy (J/kg)
            kin_energy_spec = 0.5 * (v_abs_imp**2)
            
            # Verticality (Dot product between: [vx, vy, vz] and [0, 0, -1])        
            cos_theta = abs(vz_imp) / v_abs_imp if v_abs_imp > 0 else 0.0
            cos_theta = max(-1.0, min(1.0, cos_theta))
            impact_angle_deg = math.degrees(math.acos(cos_theta))
            
            # Residual acceleration (Derivative of velocity in the last moments before td)        
            accel_residua_z = 0.0
            if td_index > 5:
                dt = t_tel[td_index] - t_tel[td_index - 5]
                dvz = vz[td_index] - vz[td_index - 5]
                accel_residua_z = dvz / dt if dt > 0 else 0.0

            # First leg to touch the ground
            first_leg = node.first_leg_touch if node.first_leg_touch else "None"

            # Retrieve the id from Environment Variable
            run_id = os.environ.get('MC_RUN_ID', 'Manual_Test')                     
            
            # Max. leg force
            max_leg_force = 0.0
            if len(t_forces) > 0:
                max_leg_force = max([max(f_imp[leg]['z'], default=0.0) for leg in node.legs])
            
            # Settling time
            if node.touchdown_time is not None and node.start_time_tel is not None:
                td_time_relative = node.touchdown_time - node.start_time_tel
                settling_time = t_tel[-1] - td_time_relative
            else:
                settling_time = 0.0

            # Final attitude
            final_roll = abs(ro[-1])
            final_pitch = abs(pi[-1])
            final_yaw = abs(ya[-1])  

            # Tip-over (Dot product between local lander's z-axis and [0, 0, 1]) 
            final_r_rad = math.radians(final_roll)
            final_p_rad = math.radians(final_pitch)            
            dot_product_z = math.cos(final_p_rad) * math.cos(final_r_rad)
            dot_product_z = max(-1.0, min(1.0, dot_product_z))
            total_tilt_deg = math.degrees(math.acos(dot_product_z))

            # Max tilt angle
            max_dynamic_tilt_deg = 0.0
            if td_index < len(t_tel):
                dynamic_tilts = []                
                for i in range(td_index, len(t_tel)):
                    r_rad = math.radians(ro[i])
                    p_rad = math.radians(pi[i])                        
                    dot_z_dyn = math.cos(p_rad) * math.cos(r_rad)
                    dot_z_dyn = max(-1.0, min(1.0, dot_z_dyn))                     
                    tilt_inst_deg = math.degrees(math.acos(dot_z_dyn))
                    dynamic_tilts.append(tilt_inst_deg)
                
                if dynamic_tilts:
                    max_dynamic_tilt_deg = max(dynamic_tilts)

            # Impact thrust
            impact_thrust_to_write = node.impact_thrust_total

            # Belly clearance                     
            try:
                min_belly_clearance_m = 0.0
                flag_hull_crash = False       
                flag_thruster_crash = False                
                if terrain_mesh is not None:                    
                    Z_BELLY = 0.0  
                    THRUSTER_RADIUS = 0.752 / 2.0  
                    THRUSTER_CENTERS = [(0.0, 0.0), (1.5, 0.0), (-1.5, 0.0)]

                    belly_local_points = []
                    point_is_thruster = []
                                        
                    for bx in np.linspace(-2.0, 2.0, 51):
                        for by in np.linspace(-2.0, 2.0, 51):
                            if abs(bx) + abs(by) <= 2.828:                                
                                # Check if the point is in the thruster
                                is_t = False
                                for cx, cy in THRUSTER_CENTERS:                                    
                                    if math.hypot(bx - cx, by - cy) <= THRUSTER_RADIUS:
                                        is_t = True
                                        break                                        
                                belly_local_points.append(np.array([bx, by, Z_BELLY]))
                                point_is_thruster.append(is_t)
                    
                    ray_origins = []
                    ray_directions = []
                    belly_z_globals = []
                    ray_is_thruster = []                
                    
                    start_idx = max(0, td_index - 50) 
                    
                    for i in range(start_idx, len(pz)):
                        r = math.radians(ro[i])
                        p = math.radians(pi[i])
                        y = math.radians(ya[i])
                        
                        cy, sy = math.cos(y), math.sin(y)
                        cp, sp = math.cos(p), math.sin(p)
                        cr, sr = math.cos(r), math.sin(r)                        
                        
                        R_mat = np.array([
                            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                            [-sp,   cp*sr,            cp*cr]
                        ])                        
                        
                        cg_pos = np.array([px[i], py[i], pz[i]])                    
                        
                        # Belly's points location
                        for pt_local, is_t in zip(belly_local_points, point_is_thruster):                            
                            pos_global = cg_pos + R_mat.dot(pt_local)                            
                            belly_z_globals.append(pos_global[2])                             
                            ray_origins.append([pos_global[0], pos_global[1], 100.0])
                            ray_directions.append([0, 0, -1])                            
                            ray_is_thruster.append(is_t)
                    
                    # Ray casting 
                    locations, index_ray, _ = terrain_mesh.ray.intersects_location(
                        ray_origins=ray_origins,
                        ray_directions=ray_directions
                    )
                    
                    clearances = []                    

                    for hit_idx, loc in zip(index_ray, locations):
                        z_terrain = loc[2] 
                        z_belly_pt = belly_z_globals[hit_idx]
                        is_t = ray_is_thruster[hit_idx]                        
                        dist = z_belly_pt - z_terrain
                        clearances.append(dist)
                        if dist <= 0.0:
                            flag_hull_crash = True
                        elif is_t and dist <= 0.117:                            
                            flag_thruster_crash = True
                    
                    if clearances:
                        min_belly_clearance_m = min(clearances)
                    
                        
            except Exception as e:                
                min_belly_clearance_m = 0.0 
                flag_hull_crash = True
                flag_thruster_crash = True         
            

            # Append to CSV file
            csv_path = os.path.join(node.plots_dir, "montecarlo_summary.csv")
            file_exists = os.path.isfile(csv_path)

            with open(csv_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:                    
                    writer.writerow([
                        'run_id', 'total_mass_kg', 'body_mass_kg', 'dry_mass_kg', 'passenger_mass_kg', 'legs_mass_kg', 
                        'tot_cog_x', 'tot_cog_y', 'tot_cog_z',
                        'tot_ixx', 'tot_iyy', 'tot_izz', 'tot_ixy', 'tot_ixz', 'tot_iyz',
                        'init_x', 'init_y', 'init_z',
                        'init_vx', 'init_vy', 'init_vz',
                        'init_roll', 'init_pitch', 'init_yaw',
                        'init_wx', 'init_wy', 'init_wz'
                    ] + params_names + [
                        'first_touch_leg', 'impact_angle_deg', 
                        'specific_kin_energy', 'res_accel_z', 'max_impact_vz_ms', 
                        'impact_thrust_N', 'final_total_tilt_deg', 'max_dynamic_tilt_deg', 'max_force_z_N', 
                        'settling_time_s', 'min_belly_clearance_m',
                        'flag_hull_crash', 'flag_thruster_crash',
                        'perceived_mass_kg', 'mass_error_kg', 'mass_error_pct' 
                    ])
                
                writer.writerow([
                    run_id, round(total_mass, 2), round(mass_body, 2), round(dry_mass, 2), round(mass_pass, 2), round(mass_legs_tot, 2),
                    round(tot_cog[0], 5), round(tot_cog[1], 5), round(tot_cog[2], 5),
                    round(tot_I[0,0], 5), round(tot_I[1,1], 5), round(tot_I[2,2], 5), 
                    round(tot_I[0,1], 5), round(tot_I[0,2], 5), round(tot_I[1,2], 5),
                    round(init_x, 4), round(init_y, 4), round(init_z, 4),
                    round(init_vx, 4), round(init_vy, 4), round(init_vz, 4),
                    round(init_roll, 4), round(init_pitch, 4), round(init_yaw, 4),
                    round(init_wx, 4), round(init_wy, 4), round(init_wz, 4)
                ] + leg_params_values + [
                    first_leg, round(impact_angle_deg, 2), 
                    round(kin_energy_spec, 2), round(accel_residua_z, 2), round(vz_imp, 2), 
                    round(impact_thrust_to_write, 2), round(total_tilt_deg, 2), round(max_dynamic_tilt_deg, 2), round(max_leg_force, 2), 
                    round(settling_time, 2), round(min_belly_clearance_m, 4),
                    flag_hull_crash, flag_thruster_crash,
                    round(gazebo_mass, 2), round(mass_error_kg, 2), round(mass_error_pct, 2) 
                ])
            
            print(f"Mission outcome for Run {run_id} saved to montecarlo_summary.csv")    
        
        
        # Plot generation
        if ENABLE_PLOTS:
            # Plot 1
            if len(t_tel) > 0:
                print(f"Processing {len(t_tel)} telemetry packets...")
                fig1, axs1 = plt.subplots(2, 2, figsize=(16, 10))
                fig1.suptitle('Lander Position and Velocity Dashboard', fontsize=18, fontweight='bold')
                
                axs1[0,0].set_title('Linear Position [m]')
                axs1[0,0].plot(t_tel, px, 'r-', label='X')
                axs1[0,0].plot(t_tel, py, 'g-', label='Y')
                axs1[0,0].plot(t_tel, pz_rel, 'b-', label='Z')
                axs1[0,0].set_ylabel('Meters')
                axs1[0,0].grid(True); axs1[0,0].legend()
                
                axs1[1,0].set_title('Linear Velocity [m/s]')
                axs1[1,0].plot(t_tel, vx, 'r-', label='Vx')
                axs1[1,0].plot(t_tel, vy, 'g-', label='Vy')
                axs1[1,0].plot(t_tel, vz, 'b-', label='Vz')
                axs1[1,0].set_xlabel('Time [s]')
                axs1[1,0].set_ylabel('Velocity [m/s]')
                axs1[1,0].grid(True); axs1[1,0].legend()
                
                axs1[0,1].set_title('Attitude (Euler Angles) [Degrees]')
                axs1[0,1].plot(t_tel, ro, 'r-', label='Roll')
                axs1[0,1].plot(t_tel, pi, 'g-', label='Pitch')
                axs1[0,1].plot(t_tel, ya, 'b-', label='Yaw')
                axs1[0,1].set_ylabel('Degrees [°]')
                axs1[0,1].grid(True); axs1[0,1].legend()
                
                axs1[1,1].set_title('Angular Velocity [rad/s]')
                axs1[1,1].plot(t_tel, wx, 'r-', label='Wx')
                axs1[1,1].plot(t_tel, wy, 'g-', label='Wy')
                axs1[1,1].plot(t_tel, wz, 'b-', label='Wz')
                axs1[1,1].set_xlabel('Time [s]')
                axs1[1,1].set_ylabel('Angular Rate [rad/s]')
                axs1[1,1].grid(True); axs1[1,1].legend()
                
                plt.tight_layout(rect=[0, 0.03, 1, 0.95])
                save_path_1 = os.path.join(node.plots_dir, "01_telemetry_dashboard.png")
                fig1.savefig(save_path_1, dpi=300)
                print(f"Telemetry saved as '{save_path_1}'")
            
            # Plot 2
            if len(t_forces) > 0:
                print(f"Processing {len(t_forces)} force samples...")
                fig2, axs2 = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
                fig2.suptitle('Touchdown Impact Forces', fontsize=16, fontweight='bold')
                
                colors = {'leg1': 'b', 'leg2': 'g', 'leg3': 'm', 'leg4': 'c'}
                titles = ['Lateral Forces (X-Axis)', 'Lateral Forces (Y-Axis)', 'Vertical Forces (Z-Axis)']
                axes_keys = ['x', 'y', 'z']
                
                for i, axis in enumerate(axes_keys):
                    for leg in node.legs:
                        axs2[i].plot(t_forces, f_imp[leg][axis], label=leg.upper(), color=colors[leg])        
                    axs2[i].set_title(titles[i], fontsize=12)
                    axs2[i].set_ylabel('Force (N)', fontsize=10)
                    axs2[i].legend(loc='upper right')
                    axs2[i].grid(True, linestyle=':', alpha=0.7)

                axs2[2].set_xlabel("Simulation Time [s]", fontsize=12)
                plt.tight_layout(rect=[0, 0.03, 1, 0.96])
                
                save_path_2 = os.path.join(node.plots_dir, "02_impact_forces_3D.png")
                fig2.savefig(save_path_2, dpi=300)
                print(f"Impact forces saved as '{save_path_2}'")
            else:
                print("\nThe lander did not touch down. Impact plot not generated.")

if __name__ == '__main__':
    main()