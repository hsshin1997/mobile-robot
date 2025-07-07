from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os

def generate_launch_description():
    world_file = os.path.join(
        os.path.dirname(__file__), '..', 'worlds', 'tb3_empty.world'
    )

    # Start Gazebo (gz sim) with the custom world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file],
        output='screen'
    )

    # Spawn robot state publisher (optional if using full robot_state_publisher)
    # Here we rely on the world’s <model> tag to place the robot

    return LaunchDescription([
        gazebo,
    ])
