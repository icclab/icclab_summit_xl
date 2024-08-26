from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    ld = LaunchDescription()
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_cli",
        arguments=['-f', 'my_map',
                  '-t', '/summit/map']
    )
 
    ld.add_action(map_saver)
    return ld
