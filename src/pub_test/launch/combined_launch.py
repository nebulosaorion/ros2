from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pub_test')
    ros_node_port = 9090  # Porta padrão para WebSocket do nó ROS 2

    return LaunchDescription([
        # Executa o nó publicador_midia usando Node
        Node(
            package='pub_test',
            executable='publicador_midia',
            name='publicador_midia',
            output='screen'
        ),
        # Abre o Foxglove Studio já configurado com a porta do nó
        ExecuteProcess(
            cmd=['firefox', f'https://studio.foxglove.dev/?ds=rosbridge-websocket&host=localhost&port={ros_node_port}'],
            output='screen'
        )
    ])
