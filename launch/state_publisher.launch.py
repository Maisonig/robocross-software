import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('car_bot')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file_name = 'car_bot.urdf.xml'
    urdf = os.path.join(package_path, "model", urdf_file_name)
    with open(urdf, 'r') as urdf_file:
        robot_desc = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d' + os.path.join(package_path, 'rviz', 'car_bot.rviz')]
        ),
        # Node(
        #     package="gazebo_test",
        #     executable="frame",
        #     name = "frame",
        # ),
    ])
