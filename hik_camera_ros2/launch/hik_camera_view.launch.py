import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hik_camera_ros2')
    default_params = os.path.join(share, 'config', 'camera_params.yaml')
    rviz_config = os.path.join(share, 'rviz2', 'hik_camera.rviz')

    params_file = DeclareLaunchArgument('params_file', default_value=default_params)

    camera_node = Node(
        package='hik_camera_ros2', executable='hik_camera_node', name='hik_camera',
        output='screen', parameters=[LaunchConfiguration('params_file')])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='screen')

    return LaunchDescription([params_file, camera_node, rviz_node])
