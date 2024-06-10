from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
# import package share
from ament_index_python.packages import get_package_share_directory
import os



current_package = 'ubisense_ros2_port'



def generate_launch_description():

    config_file_path = os.path.join(
        get_package_share_directory(current_package),
        'config',
        'parameters.yaml'
    )
        

    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file_path,
            description='Path to the configuration file'
        ),

        # Node definition
        Node(
            package='ubisense_ros2_port',
            executable='publisher',
            name='ubisense_uwb_publisher',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()