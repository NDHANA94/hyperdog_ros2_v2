import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    this_pkg = 'hyperdog_ros2_pybullet_sim'

    config = os.path.join(get_package_share_directory(this_pkg), 
                          'config', 
                          'hyperdog_pybullet_params.yaml')
    
    node = Node(
        package=this_pkg,
        name='hyperdog_ros2_pybullet_node',
        executable='hyperdog_pybullet_node',
        parameters=[config]
    )

    ld.add_action(node)
    return ld