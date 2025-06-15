from launch import LaunchDescription #é usada para definir e organizar o conteúdo de um arquivo de lançamento no ROS 2.
from launch.actions import ExecuteProcess, IncludeLaunchDescription ## ações para executar um processo ou incluir outra descrição de lançamento dentro de um lançamento.
from launch_ros.actions import Node # ação Node do launch_ros, que permite definir e lançar nós ROS 2 em um arquivo de lançamento.
from launch.launch_description_sources import AnyLaunchDescriptionSource # permite incluir lançamentos de qualquer fonte, como arquivos XML ou Python.
from ament_index_python.packages import get_package_share_directory #  diretório de compartilhamento de um pacote ROS 2, útil para acessar arquivos dentro de pacotes.
import os #padrão

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
                'tamanho_tela': [800, 600],  
            }]
        ),

        # Abre o Foxglove Studio automaticamente
        ExecuteProcess(
            cmd=['xdg-open', f'https://studio.foxglove.dev/?ds=rosbridge-websocket&ds.url=ws://localhost:{ros_node_port}'],
            output='screen'
        )
    ])