"""Launch one automatic, headless landing simulation and its ROS nodes.

The launch description first generates the randomized SDF/world inputs,
starts Gazebo, starts the controllers and loggers, and lets physics run
automatically.
"""

# Launch, process, and ROS action imports
import os
import subprocess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.events import Shutdown

def generate_launch_description():
    # Generate the model and scenario inputs before Gazebo reads them.
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    WS_ROOT = os.path.dirname(SCRIPT_DIR)
    os.environ.setdefault('TOUCHDOWN_WS_ROOT', WS_ROOT)
    
    # Pre-simulation generation phase:
    print("\n[LAUNCH] >>> Generation of model and scenario files in progress... <<<\n")    
    
    subprocess.run(['ros2', 'run', 'lander_py', 'generate_body'], check=True)
    subprocess.run(['ros2', 'run', 'lander_py', 'generate_scenario'], check=True)
    subprocess.run(['ros2', 'run', 'lander_py', 'generate_legs'], check=True)
    
    print("\n[LAUNCH] >>> Generation complete! Starting Gazebo and nodes.... <<<\n")

    # Start Gazebo with physics running for unattended simulations.
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    
    WORLD_DIR = os.path.join(WS_ROOT, "src", "worlds")
    world_path = os.path.join(WORLD_DIR, "landing_scenario.world")
    
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world_path,
            'pause': 'false'
        }.items()
    )

    # Define the runtime controllers and logging nodes.
    
    thrusters_node = Node(
        package='lander_py',
        executable='thrusters_node',
        output='screen'
    )

    psi_disturbance_node = Node(
        package='lander_py',
        executable='psi_disturbance_node',
        output='screen',
        sigterm_timeout='15.0'
    )
    
    data_logger_node = Node(
        package='lander_py',
        executable='data_logger_node',
        output='screen',
        sigterm_timeout='25.0',
        on_exit=[EmitEvent(event=Shutdown())]  
    )

    shock_absorber_logger_node = Node(
        package='lander_py',
        executable='df_logger_node',
        output='screen',
        sigterm_timeout='15.0'  
    )

    thruster_logger_node = Node(
        package='lander_py',
        executable='thruster_logger_node',
        output='screen',
        sigterm_timeout='15.0'
        
    )

    return LaunchDescription([
        gzserver_launch,
        TimerAction(period=0.5, actions=[thrusters_node]),
        TimerAction(period=0.5, actions=[psi_disturbance_node]),
        TimerAction(period=0.5, actions=[data_logger_node]),
        TimerAction(period=0.5, actions=[shock_absorber_logger_node]),
        TimerAction(period=0.5, actions=[thruster_logger_node])
    ])