import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates, ContactsState
import matplotlib
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import math
from std_msgs.msg import Bool
import time
import signal
import os
import yaml

class FlightDataLogger(Node):
    def __init__(self, target_model="lander_model"):
        super().__init__('flight_data_logger')

        self.target_model = target_model  

        SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))        
        if "install" in SCRIPT_DIR:
            WS_ROOT = SCRIPT_DIR.split("install")[0]            
        else:
            WS_ROOT = SCRIPT_DIR.split("src")[0]

        CONFIG_PATH = os.path.join(WS_ROOT, "config.yaml")
        self.plots_dir = os.path.join(WS_ROOT, "plots")
        os.makedirs(self.plots_dir, exist_ok=True)

        try:
            with open(CONFIG_PATH, 'r') as file:
                config = yaml.safe_load(file)
            self.get_logger().info("Config.yaml file successfully uploaded to the Data Logger!")
        except FileNotFoundError:
            self.get_logger().error(f"Unable to find the config.yaml file at the path: {CONFIG_PATH}")
            # Fallback
            config = {"simulation_stop": {"epsilon": 0.05, "timeout_after_touchdown": 5.0, "steady_req_duration": 1.0}}      
        
        self.times_tel = []
        self.start_time_tel = None
        self.times_forces = []
        self.start_time_forces = None
        
        self.pos_x, self.pos_y, self.pos_z = [], [], []
        self.vel_x, self.vel_y, self.vel_z = [], [], []
        self.roll, self.pitch, self.yaw = [], [], []
        self.ang_x, self.ang_y, self.ang_z = [], [], []        
        
        self.legs = ['leg1', 'leg2', 'leg3', 'leg4']
        self.axes = ['x', 'y', 'z']
        self.history_forces = {leg: {axis: [] for axis in self.axes} for leg in self.legs}        
        
        # Shutdown parameters (STEADY STATE & TIMEOUT)
        sim_stop_cfg = config.get("simulation_stop", {})
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
            
        self.get_logger().info(f"Logger Node started. Listening to {self.target_model}...")
        self.get_logger().info(">>> AUTO-STOP ENABLED: The node will stop at steady state or timeout <<<")

    
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

    
    def telemetry_callback(self, msg):
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
                    self.get_logger().info("TIMEOUT reached after touchdown.")
                    msg = Bool()
                    msg.data = True
                    self.pub_stop.publish(msg)
                    time.sleep(0.5)
                    raise SystemExit("Simulation Auto-Stop: Timeout")

                # Steady State Control (Speed below epsilon)
                is_steady = (
                    abs(vx) < self.epsilon and abs(vy) < self.epsilon and abs(vz) < self.epsilon and
                    abs(wx) < self.epsilon and abs(wy) < self.epsilon and abs(wz) < self.epsilon
                )

                if is_steady:
                    if self.steady_start_time is None:
                        self.steady_start_time = now 
                    elif (now - self.steady_start_time) > self.steady_req_duration:
                        self.get_logger().info(f"STEADY STATE reached (Variables < {self.epsilon}).")                       
                        msg = Bool()
                        msg.data = True
                        self.pub_stop.publish(msg)
                        time.sleep(0.5)                       
                        raise SystemExit("Simulation Auto-Stop: Steady State")
                else:
                    self.steady_start_time = None 
           

    def contact_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.start_time_forces is None:
            self.start_time_forces = now

        t = now - self.start_time_forces
        self.times_forces.append(t)

        # Reset forces for current sample
        current_f = {leg: {'x': 0.0, 'y': 0.0, 'z': 0.0} for leg in self.legs}

        # Read all active contact states
        for state in msg.states:
            col_name = state.collision1_name
            for leg in self.legs:
                if leg in col_name:
                    current_f[leg]['x'] += state.total_wrench.force.x
                    current_f[leg]['y'] += state.total_wrench.force.y
                    current_f[leg]['z'] += state.total_wrench.force.z

        # Save force history
        for leg in self.legs:
            for axis in self.axes:
                self.history_forces[leg][axis].append(current_f[leg][axis])

        # Touchdown detection
        if self.touchdown_time is None:
            for leg in self.legs:
                if abs(current_f[leg]['z']) > 5.0:
                    self.touchdown_time = now
                    self.get_logger().info(f"TOUCHDOWN DETECTED! ({leg}). Timers started.")
                    break
       

def main(args=None):
    rclpy.init(args=args)
    node = FlightDataLogger(target_model="lander_model")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user! Generating all plots...")
    except SystemExit as e:
        print(f"\n{e}. Generating all plots...") 
    finally:
        # data extraction 
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        t_tel = list(node.times_tel)
        t_forces = list(node.times_forces)
        px, py, pz = list(node.pos_x), list(node.pos_y), list(node.pos_z)
        vx, vy, vz = list(node.vel_x), list(node.vel_y), list(node.vel_z)
        ro, pi, ya = list(node.roll), list(node.pitch), list(node.yaw)
        wx, wy, wz = list(node.ang_x), list(node.ang_y), list(node.ang_z)
        
        f_imp = {leg: {ax: list(node.history_forces[leg][ax]) for ax in node.axes} for leg in node.legs}
        if len(t_tel) > 0:
            print("\n--- INITIAL VALUES RECORDED (t=0) ---")
            print(f"Position [m]:        X= {px[0]:.4f} | Y= {py[0]:.4f} | Z= {pz[0]:.4f}")
            print(f"Attitude [Degree]:   R= {ro[0]:.4f} | P= {pi[0]:.4f} | Y= {ya[0]:.4f}")
            print(f"Linear Vel. [m/s]:   Vx={vx[0]:.4f} | Vy={vy[0]:.4f} | Vz={vz[0]:.4f}")
            print(f"Angular Vel. [rad/s]: Wx={wx[0]:.4f} | Wy={wy[0]:.4f} | Wz={wz[0]:.4f}")
            print("----------------------------------------\n")
        
        # ROS Shutdown
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        # Plot 1
        if len(t_tel) > 0:
            print(f"Processing {len(t_tel)} telemetry packets...")
            fig1, axs1 = plt.subplots(2, 2, figsize=(16, 10))
            fig1.suptitle('Lander Position and Velocity Dashboard', fontsize=18, fontweight='bold')
            
            axs1[0,0].set_title('Linear Position [m]')
            axs1[0,0].plot(t_tel, px, 'r-', label='X')
            axs1[0,0].plot(t_tel, py, 'g-', label='Y')
            axs1[0,0].plot(t_tel, pz, 'b-', label='Z')
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