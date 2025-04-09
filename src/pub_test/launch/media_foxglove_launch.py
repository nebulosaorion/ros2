from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pub_test',
            executable='publicador_midia',
            name='publicador_midia',
            output='screen',
            parameters=[]
        )
    ])
