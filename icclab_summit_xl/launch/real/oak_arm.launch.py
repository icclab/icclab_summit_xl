import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # OAK-D Pro on the arm with the simulation's names: frames as in the URDF (rs_compat),
    # topics /arm_camera/{color,depth}/{image_raw,camera_info}
    oak_launch = os.path.join(
        get_package_share_directory('icclab_summit_xl'), 'launch', 'real', 'oak.camera.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oak_launch),
            launch_arguments={
                'name': 'arm_camera',
                'rs_compat': 'true',
                'depth_image_name': 'image_raw',
                'rgb_camera.color_profile': '1280,720,30',
                'depth_module.depth_profile': '1280,720,30',
            }.items(),
        ),
    ])
