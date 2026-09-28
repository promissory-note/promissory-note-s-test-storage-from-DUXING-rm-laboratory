import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hik_camera_ros2')
    default_params = os.path.join(pkg_share, 'config', 'camera_params.yaml')

    params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='相机参数 YAML 文件')

    camera_node = Node(
        package='hik_camera_ros2',
        executable='hik_camera_node',
        name='hik_camera',
        output='screen',
        parameters=[LaunchConfiguration('params_file')])

    return LaunchDescription([params_file, camera_node])
