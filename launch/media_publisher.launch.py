from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    pkg_share = get_package_share_directory('pub_test')
    ros_node_port = '9090'

    # Configuração do rosbridge_server (WebSocket)
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([  
            os.path.join(
                get_package_share_directory('rosbridge_server'), 
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        ]),
        launch_arguments={
            'port': ros_node_port,
            'address': 'localhost',
        }.items()
    )

    return LaunchDescription([
        # Inicia rosbridge_server
        rosbridge_launch,

        # Nó principal que processa mensagens do display_2
        Node(
            package='pub_test',
            executable='publicador_midia',
            name='publicador_midia',
            output='screen',
            parameters=[{
                'tamanho_tela': [800, 600],  # Parâmetro configurável
            }]
        ),

        # Abre o Foxglove Studio automaticamente
        ExecuteProcess(
            cmd=['xdg-open', f'https://studio.foxglove.dev/?ds=rosbridge-websocket&ds.url=ws://localhost:{ros_node_port}'],
            output='screen'
        )
    ])