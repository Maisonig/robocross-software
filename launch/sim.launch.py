import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_path = get_package_share_directory('car_bot')
    launch_path = os.path.join(package_path, 'launch')
    param_file = os.path.join(get_package_share_directory("car_bot"), 'config', 'sim.yaml')

    action_simulation_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'gz.launch.py')
    )

    action_rviz_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'state_publisher.launch.py')
    )

    action_front_cloud_to_scan = Node(
        package='car_bot',
        executable='cloud_to_scan.py',
        name='front_cloud_to_scan',
        parameters=[param_file]
    )

    action_mapping = Node(
        package='car_bot',
        executable='mapping.py',
        name='mapping',
        parameters=[param_file]
    )

    action_path_planning = Node(
        package='car_bot',
        executable='path_planning.py',
        name='path_planning',
        parameters=[param_file]
    )

    action_path_controller = Node(
        package='car_bot',
        executable='path_controller.py',
        name='path_controller',
        parameters=[param_file]
    )

    ld = LaunchDescription()
    ld.add_action(action_simulation_launch)
    ld.add_action(action_rviz_launch)
    ld.add_action(action_front_cloud_to_scan)
    ld.add_action(action_mapping)
    # ld.add_action(action_path_planning)
    # ld.add_action(action_path_controller)
    return ld
