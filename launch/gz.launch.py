import os

from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_path = get_package_share_directory('car_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    action_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            package_path,
            'model',
            'robocross_trial.sdf.xml'
        ])}.items(),
    )

    action_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(package_path, 'config', 'gz_remapings.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(action_gz_sim)
    ld.add_action(action_bridge)
    return ld
