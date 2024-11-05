from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the YAML configuration file
    config = os.path.join(
        get_package_share_directory('smb_kinematics'),
        'config',
        'config.yaml'
    )

    # Node configuration
    differential_drive_node = Node(
        package='smb_kinematics',
        executable='differential_drive_controller_node',
        name='differential_drive_controller',
        parameters=[config],
        output='screen',  # Optional: logs output to the console
        emulate_tty=True  # Ensures color logs if supported
    )

    return LaunchDescription([
        differential_drive_node
    ])
