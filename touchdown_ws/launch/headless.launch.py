import os
import subprocess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.events import Shutdown

def generate_launch_description():
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    WS_ROOT = os.path.dirname(SCRIPT_DIR)
    os.environ.setdefault('TOUCHDOWN_WS_ROOT', WS_ROOT)
    
    # Pre-simulation generation phase:
    print("\n[LAUNCH] >>> Generation of model and scenario files in progress... <<<\n")    
    
    subprocess.run(['ros2', 'run', 'lander_py', 'generate_body'], check=True)
    subprocess.run(['ros2', 'run', 'lander_py', 'generate_scenario'], check=True)
    subprocess.run(['ros2', 'run', 'lander_py', 'generate_legs'], check=True)
    
    print("\n[LAUNCH] >>> Generation complete! Starting Gazebo and nodes.... <<<\n")

    # Starting Gazebo
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    
    WORLD_DIR = os.path.join(WS_ROOT, "src", "worlds")
    world_path = os.path.join(WORLD_DIR, "landing_scenario.world")
    
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world_path,
            'pause': 'true' 
        }.items()
    )

    # Starting ROS 2 nodes
    
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

    unpause_gazebo = TimerAction(
        period=1.5,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/unpause_physics', 'std_srvs/srv/Empty'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gzserver_launch,
        TimerAction(period=0.5, actions=[thrusters_node]),
        TimerAction(period=0.5, actions=[psi_disturbance_node]),
        TimerAction(period=0.5, actions=[data_logger_node]),
        TimerAction(period=0.5, actions=[shock_absorber_logger_node]),
        TimerAction(period=0.5, actions=[thruster_logger_node]),
        unpause_gazebo
    ])