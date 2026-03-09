import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Buscamos la ruta de nuestro archivo YAML
    config_file = os.path.join(
        get_package_share_directory('img2pcl_ros2'), 
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='img2pcl_ros2',
            executable='Img2Pcl', 
            name='Img2Pcl_node',      
            output='screen',
            parameters=[config_file]  
        )
    ])