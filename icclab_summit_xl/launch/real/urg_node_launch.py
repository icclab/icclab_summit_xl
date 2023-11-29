import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    urg_node_dir = get_package_share_directory('icclab_summit_xl')
    lidar_position = LaunchConfiguration('lidar_position')
    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            'lidar_position',
            default_value='front',
            description='lidar position, valid values are: front, rear')])

    def expand_param_file_name(context):
        param_file = os.path.join(
                urg_node_dir, 'config',
                'lidar_' + context.launch_configurations['lidar_position'] + '.yaml')
        if os.path.exists(param_file):
            return [SetLaunchConfiguration('param', param_file)]

    param_file_path = OpaqueFunction(function=expand_param_file_name)
    launch_description.add_action(param_file_path)

    hokuyo_node = Node(
        package='urg_node', 
        executable='urg_node_driver', 
        output='screen', 
        name=('lidar_',lidar_position),
        parameters=[LaunchConfiguration('param')],
        remappings=[('scan', (lidar_position, '_laser/scan'))]
        )

    launch_description.add_action(LogInfo(msg=["Lidar Config", " config_param: \n", LaunchConfiguration('param')]))
    launch_description.add_action(hokuyo_node)

    return launch_description
