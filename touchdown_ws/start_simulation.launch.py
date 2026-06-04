import os
import subprocess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.events import Shutdown

def generate_launch_description():
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    
    # Pre-simulation generation phase:
    print("\n[LAUNCH] >>> Generation of model and scenario files in progress... <<<\n")    
    
    subprocess.run(['ros2', 'run', 'lander_py', 'generate_body'], check=True)
    subprocess.run(['ros2', 'run', 'lander_py', 'generate_scenario'], check=True)
    
    print("\n[LAUNCH] >>> Generation complete! Starting Gazebo and nodes.... <<<\n")

    # Starting Gazebo
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    
    WORLD_DIR = os.path.join(SCRIPT_DIR, "src", "worlds")
    world_path = os.path.join(WORLD_DIR, "landing_scenario.world")
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')),
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
    
    data_logger_node = Node(
        package='lander_py',
        executable='data_logger_node',
        output='screen',
        sigterm_timeout='15.0'  
    )

    thruster_logger_node = Node(
        package='lander_py',
        executable='thruster_logger_node',
        output='screen',
        sigterm_timeout='15.0',  
        on_exit=[EmitEvent(event=Shutdown())]
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=2.0, actions=[thrusters_node]),
        TimerAction(period=2.0, actions=[data_logger_node]),
        TimerAction(period=2.0, actions=[thruster_logger_node])
    ])