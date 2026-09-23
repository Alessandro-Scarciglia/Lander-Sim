import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
from lander_py.paths import workspace_root
import signal


class ThrustLogger(Node):

    def __init__(self):

        super().__init__('thrust_logger')

        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        WS_ROOT = str(workspace_root())
            
        self.plots_dir = os.path.join(WS_ROOT, "figures", "simulation")
        os.makedirs(self.plots_dir, exist_ok=True)

        # Thruster Subscribers
        self.sub_left = self.create_subscription(Float64, '/lander/thrusters/left', self.left_cb, 10)
        self.sub_right = self.create_subscription(Float64, '/lander/thrusters/right', self.right_cb, 10)
        self.sub_central = self.create_subscription(Float64, '/lander/thrusters/central', self.central_cb, 10)

        # Td time subscriber
        self.sub_td = self.create_subscription(Float64, '/lander/touchdown_time', self.touchdown_time_cb, 10)

        # Subscriber t_start and stop
        self.sub_tstart = self.create_subscription(Float64, '/lander/t_start', self.tstart_cb, 10)
        self.sub_stop = self.create_subscription(Bool, '/lander/stop_logging', self.stop_cb, 10)

        # Impact Thrust publisher
        self.pub_impact_thrust = self.create_publisher(Float64, '/lander/impact_thrust', 10)

        # Thruster data
        self.left_time = []
        self.left_data = []

        self.right_time = []
        self.right_data = []

        self.central_time = []
        self.central_data = []
        
        self.start_time = None        
        self.t_start = None
        self.is_recording = True

        self.get_logger().info("Thrust Logger STARTED")
        


    def stop_cb(self, msg):
        if msg.data == True:
            self.get_logger().info("Automatic stop signal received! Freezing data collection...")
            self.is_recording = False
        
    def touchdown_time_cb(self, msg):
        absolute_td_time = msg.data        
        
        if self.start_time is None:
            return
            
        # Convert absolute Gazebo time to script relative time
        relative_td_time = absolute_td_time - self.start_time
        
        # Find the index closest to touchdown for each of the three thruster
        idx_l = min(range(len(self.left_time)), key=lambda i: abs(self.left_time[i] - relative_td_time)) if self.left_time else None
        idx_r = min(range(len(self.right_time)), key=lambda i: abs(self.right_time[i] - relative_td_time)) if self.right_time else None
        idx_c = min(range(len(self.central_time)), key=lambda i: abs(self.central_time[i] - relative_td_time)) if self.central_time else None
        
        # Extracts the Newtons of thrust at that instant
        thrust_l = self.left_data[idx_l] if idx_l is not None else 0.0
        thrust_r = self.right_data[idx_r] if idx_r is not None else 0.0
        thrust_c = self.central_data[idx_c] if idx_c is not None else 0.0
        
        total_impact_thrust = thrust_l + thrust_r + thrust_c
        self.get_logger().info(f"Calculated impact thrust: {total_impact_thrust:.2f} N. Publishing back...")
        
        # Send the data to data_logger
        reply_msg = Float64()
        reply_msg.data = total_impact_thrust
        self.pub_impact_thrust.publish(reply_msg)
    
    def tstart_cb(self, msg):
        if not self.is_recording:
            return
        self.t_start = msg.data

    def get_relative_time(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now
        return now - self.start_time

    def left_cb(self, msg):
        if not self.is_recording:
            return
        current_time = self.get_relative_time()
        self.left_time.append(current_time)
        self.left_data.append(msg.data)

    def right_cb(self, msg):
        if not self.is_recording:
            return
        current_time = self.get_relative_time()
        self.right_time.append(current_time)
        self.right_data.append(msg.data)

    def central_cb(self, msg):
        if not self.is_recording:
            return
        current_time = self.get_relative_time()
        self.central_time.append(current_time)
        self.central_data.append(msg.data)



def main(args=None):
    rclpy.init(args=args)
    node = ThrustLogger()
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

        # Copy data
        left_time = list(node.left_time)
        left_data = list(node.left_data)

        right_time = list(node.right_time)
        right_data = list(node.right_data)

        central_time = list(node.central_time)
        central_data = list(node.central_data)

        # Copy t_start
        t_start = node.t_start        
        
        if ENABLE_PLOTS:
            print("\n[ThrustLogger] Starting chart generation...")
            # Calculation of real curve times
            curve_t1 = 1.35
            curve_t2 = 2.0
            curve_t3 = 2.2

            if t_start is not None:

                t1 = curve_t1 - t_start
                t2 = curve_t2 - t_start
                t3 = curve_t3 - t_start

            else:

                print("WARNING: t_start not received!")
                t1, t2, t3 = curve_t1, curve_t2, curve_t3

            # Plot
            fig, axs = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
            fig.suptitle('Individual Motor Thrust Analysis (MECO)', fontsize=16, fontweight='bold')

            datasets = [
                (left_time, left_data, 'red', 'Left Thruster'),
                (right_time, right_data, 'blue', 'Right Thruster'),
                (central_time, central_data, 'green', 'Central Thruster')
            ]

            for i, (t, data, color, title) in enumerate(datasets):

                # STEP PLOT
                axs[i].plot(t, data, color=color, linewidth=1.0, label='Thrust Detected')

                # Horizontal lines
                axs[i].axhline(y=6000, color='gray', linestyle=':', label='Max Nominal (6kN)')
                axs[i].axhline(y=3000, color='orange', linestyle=':', label='Min Saturation (3kN)')

                # Dynamic vertical lines
                if t1 >= 0:
                    axs[i].axvline(x=t1, color='purple', linestyle='-.', alpha=0.7, label='t1' if i == 0 else "")

                if t2 >= 0:
                    axs[i].axvline(x=t2, color='cyan', linestyle='-.', alpha=0.7, label='t2' if i == 0 else "")

                if t3 >= 0:
                    axs[i].axvline(x=t3, color='black', linestyle='--', alpha=0.8, label='MECO' if i == 0 else "")

                axs[i].set_title(title)
                axs[i].set_ylabel('Thrust [N]')

                axs[i].grid(True, linestyle=':', alpha=0.7)

                axs[i].legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=9)

            axs[2].set_xlabel('Flight Time [s]')

            axs[2].set_xlim(-0.1, 2.5)

            plt.tight_layout(rect=[0, 0.03, 0.85, 0.96])

            save_path = os.path.join(node.plots_dir, '03_thrust_profile.png')
            fig.savefig(save_path, dpi=300)

            print(f"Plot saved: {save_path}")
            


if __name__ == '__main__':
    main()