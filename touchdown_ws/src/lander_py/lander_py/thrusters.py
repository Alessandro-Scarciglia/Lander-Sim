import rclpy
import os
import re
from rclpy.node import Node
from std_msgs.msg import Float64
import yaml
from lander_py.paths import simulation_config_path, workspace_root

class Thrusters(Node):
    def __init__(self):
        super().__init__('thrusters_controller')
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)]) 

        # Read masses from sdf files
        
        WS_ROOT = str(workspace_root())

        MODEL_DIR = os.path.join(WS_ROOT, "src", "worlds", "models", "lander_model")
        CONFIG_PATH = str(simulation_config_path())
            
        self.body_sdf_path = os.path.join(MODEL_DIR, "body", "body.sdf")        
        self.passenger_sdf_path = os.path.join(MODEL_DIR, "passenger", "passenger.sdf")        

        try:
            with open(CONFIG_PATH, 'r') as file:
                config = yaml.safe_load(file)
            print("Simulation config successfully loaded!")
        except FileNotFoundError:
            print(f"Error: Unable to find the simulation config at the path: {CONFIG_PATH}")
            exit(1)        
        
        mass_body = self.read_mass_from_sdf(self.body_sdf_path)
        mass_passenger = self.read_mass_from_sdf(self.passenger_sdf_path)
        leg_cfg = config["leg_generation"]
        thruster_cfg = config["thrusters"]
        mass_legs = 4 * self.calculate_leg_mass(leg_cfg)

        # Total mass
        total_mass = mass_body + mass_passenger +  mass_legs

        g = thruster_cfg["lunar_gravity"]                      # Lunar gravity [m/s^2]
        self.num_engines = thruster_cfg["num_engines"]         # Number of main engines
        t_min = thruster_cfg["t_min"]                          # Minimum saturation [N] 
        t_max = thruster_cfg["t_max"]                          # Maximum saturation [N]
        self.engine_max_thrust = thruster_cfg["engine_max_thrust"]

        # Initial thrust calculation
        T0 = total_mass * g 
        
        # Thrust per single engine
        required_thrust_per_engine = T0 / self.num_engines

        # Saturation
        saturated_initial_thrust = max(t_min, min(t_max, required_thrust_per_engine))

        self.initial_throttle = saturated_initial_thrust / self.engine_max_thrust

        # Find t_start        
        if self.initial_throttle >= 0.50:
            self.t_start = (1.0 - self.initial_throttle) * (1.35 / 0.50)
        elif self.initial_throttle >= 0.33:
            self.t_start = 1.35 + (0.50 - self.initial_throttle) * (0.65 / 0.17)
        else:
            self.t_start = 2.0  # Safe offset        
   
        self.get_logger().info("--- THRUST CONTROLLER INITIALIZATION ---")
        self.get_logger().info(f"Body Mass (Dry Body+Fuel): {mass_body:.2f} kg")
        self.get_logger().info(f"Passenger Mass (Payload)  : {mass_passenger:.2f} kg")
        self.get_logger().info(f"Legs Mass (Calculated): {mass_legs:.2f} kg")
        self.get_logger().info(f"Total Flight Mass         : {total_mass:.2f} kg")
        self.get_logger().info(f"Gravity: {g} m/s^2")
        self.get_logger().info(f"Total Required Thrust T0: {T0:.2f} N")
        self.get_logger().info(f"Theoretical Thrust per Engine: {required_thrust_per_engine:.2f} N")
        self.get_logger().info(f"Saturated Thrust (meco_max_thrust): {saturated_initial_thrust:.2f} N")
        self.get_logger().info("------------------------------------")

        # Publishers and timer creation
        self.pub_left = self.create_publisher(Float64, '/lander/thrusters/left', 10)
        self.pub_right = self.create_publisher(Float64, '/lander/thrusters/right', 10)
        self.pub_central = self.create_publisher(Float64, '/lander/thrusters/central', 10)  
        self.pub_tstart = self.create_publisher(Float64, '/lander/t_start', 10)        

        # Timer
        self.start_time = None
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("Main engines started! Waiting for Gazebo...")
    
    def calculate_leg_mass(self, leg_cfg):        
        # 4 Trusses (C1D, C2D, A1D, A2D)
        mass = 4 * leg_cfg.get("truss_mass", 0.0) 

        # DF System
        mass += leg_cfg.get("strut_DF_mass", 0.0)
        mass += leg_cfg.get("piston_DF_mass", 0.0)
        
        # B Systems (B1 e B2)
        mass += 2 * (leg_cfg.get("b_strut_mass", 0.0) + leg_cfg.get("b_piston_mass", 0.0))
        
        # link_D, link_E, link_foot
        mass += 10.0 + 10.0 + 5.0 
        
        return mass

    
    def read_mass_from_sdf(self, filepath):        
        try:
            with open(filepath, 'r') as file:
                content = file.read()                
                match = re.search(r'<mass>\s*([0-9]*\.?[0-9]+)\s*</mass>', content)
                if match:
                    return float(match.group(1))
                else:
                    self.get_logger().warning(f"<mass> tag not found in {filepath}. Returning 0.0 kg")
        except FileNotFoundError:
            self.get_logger().error(f"File not found: {filepath}. Returning 0.0 kg")
        
        return 0.0

    
    def calculate_meco_throttle(self, t_curve):
        t1 = 1.35  
        t2 = 2.0  
        t3 = 2.2
        
        if t_curve <= t1:
            percentuale = 1.0 - (0.50 / t1) * t_curve
        elif t_curve <= t2:
            percentuale = 0.50 - (0.17 / (t2 - t1)) * (t_curve - t1)
        elif t_curve <= t3:
            percentuale = 0.33
        else:
            percentuale = 0.0
            
        return percentuale

    
    def control_loop(self): 
        # Reads the current simulation time from Gazebo     
        now = self.get_clock().now()
        # Initializing the starting time
        if self.start_time is None:
            if now.nanoseconds == 0:
                return            
            self.start_time = now
            self.get_logger().info("Starting Thrust Profile")

        t_elapsed = (now - self.start_time).nanoseconds / 1e9
        t_curve = self.t_start + t_elapsed
        
        if t_elapsed > 3.5:
            msg = Float64()
            msg.data = 0.0
            self.pub_left.publish(msg)
            self.pub_right.publish(msg)
            self.pub_central.publish(msg)
            self.get_logger().info("Sequence completed. Engines off.")
            self.timer.cancel()
            return

        current_throttle = self.calculate_meco_throttle(t_curve)
        current_thrust = current_throttle * self.engine_max_thrust
        
        msg_left = Float64()
        msg_right = Float64()
        msg_central = Float64()

        # Default
        msg_left.data = 0.0
        msg_right.data = 0.0
        msg_central.data = 0.0

        # Thrusters configuration 
        if self.num_engines == 1:
            msg_central.data = float(current_thrust)

        elif self.num_engines == 2:
            msg_left.data = float(current_thrust)
            msg_right.data = float(current_thrust)

        elif self.num_engines == 3:
            msg_left.data = float(current_thrust)
            msg_right.data = float(current_thrust)
            msg_central.data = float(current_thrust)

        # Publish
        msg_tstart = Float64()
        msg_tstart.data = self.t_start
        self.pub_tstart.publish(msg_tstart)
        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)
        self.pub_central.publish(msg_central)

def main(args=None):
    rclpy.init(args=args)
    node = Thrusters()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()