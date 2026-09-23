import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import signal
import matplotlib.pyplot as plt
import numpy as np
import os
from lander_py.paths import workspace_root
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class DFLogger(Node):

    def __init__(self):
        super().__init__("df_logger")

        WS_ROOT = str(workspace_root())

        self.plots_dir = os.path.join(WS_ROOT, "figures", "simulation")
        os.makedirs(self.plots_dir, exist_ok=True)

        self.is_recording = True

        self.data = {
            "leg1": {"pos": [], "force": []},
            "leg2": {"pos": [], "force": []},
            "leg3": {"pos": [], "force": []},
            "leg4": {"pos": [], "force": []},
        }

        self.create_subscription(Point, "/lander/leg1/df_force_displacement", lambda msg: self.callback(msg, "leg1"), 100)
        self.create_subscription(Point, "/lander/leg2/df_force_displacement", lambda msg: self.callback(msg, "leg2"), 100)
        self.create_subscription(Point, "/lander/leg3/df_force_displacement", lambda msg: self.callback(msg, "leg3"), 100)
        self.create_subscription(Point, "/lander/leg4/df_force_displacement", lambda msg: self.callback(msg, "leg4"), 100)
        self.sub_stop = self.create_subscription(Bool, "/lander/stop_logging", self.stop_cb, 10)

        self.get_logger().info("DF Logger started")

    def stop_cb(self, msg):
        if msg.data and self.is_recording:
            self.get_logger().info("Automatic stop signal received! Freezing data collection...")
            self.is_recording = False

    def callback(self, msg, leg):
        if not self.is_recording:
            return
        self.data[leg]["pos"].append(msg.x)
        self.data[leg]["force"].append(msg.y)

    def save_plot(self):
        fig, ax = plt.subplots(figsize=(12, 8))

        for leg in ["leg1", "leg2", "leg3", "leg4"]:

            pos = self.data[leg]["pos"]
            force = self.data[leg]["force"]
            
            if not pos:
                continue
           
            ax.plot(pos, force, linewidth=2, label=leg, alpha=0.8)

        ax.set_title("Primary Strut DF - Force vs Displacement")
        ax.set_xlabel("Displacement [m]")
        ax.set_ylabel("Force [N]")
        ax.grid(True, linestyle=":")
        ax.legend()

        save_path = os.path.join(self.plots_dir, "05_df_force_displacement.png")
        fig.savefig(save_path, dpi=300)
        print("\nPlot saved:")
        print(save_path)
        plt.close(fig)


def main():

    rclpy.init()

    node = DFLogger()

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

        if ENABLE_PLOTS:
            print("\n[DFLogger] Starting chart generation...")
            node.save_plot()        


if __name__ == "__main__":
    main()