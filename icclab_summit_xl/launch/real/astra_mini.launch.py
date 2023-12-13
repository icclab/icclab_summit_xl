from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():
    params_file = get_package_share_directory(
        "icclab_summit_xl") + "/config/astra_mini_params.yaml"
    with open(params_file, 'r') as file:
        config_params = yaml.safe_load(file)
    container = ComposableNodeContainer(
        name='astra_camera_container',
        namespace='summit/front_rgbd_camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::OBCameraNodeFactory',
                           name='camera',
                           namespace='summit/front_rgbd_camera',
                           parameters=[config_params]),
            ## These use lots of CPU
            #ComposableNode(package='astra_camera',
            #               plugin='astra_camera::PointCloudXyzNode',
            #               namespace='summit/front_rgbd_camera',
            #               name='point_cloud_xyz'),
            #ComposableNode(package='astra_camera',
            #               plugin='astra_camera::PointCloudXyzrgbNode',
            #               namespace='summit/front_rgbd_camera',
            #               name='point_cloud_xyzrgb')
        ],
        output='screen')

    color_compression = Node(
      package='image_transport',
      executable='republish',
      # namespace=namespace,
      # parameters=[params],
      arguments=['raw', 'compressed'],
      remappings=[('in', '/summit/front_rgbd_camera/color/image_raw'),
      ('out/compressed', '/summit/front_rgbd_camera/color/image/compressed')])

    depth_compression = Node(
      package='image_transport',
      executable='republish',
      # namespace=namespace,
      # parameters=[params],
      arguments=['raw', 'compressedDepth'],
      remappings=[('in', '/summit/front_rgbd_camera/depth/image_raw'),
      ('out/compressedDepth', '/summit/front_rgbd_camera/depth/image/compressedDepth')])
    return LaunchDescription([container]) #color_compression, depth_compression -> they flood wg0
