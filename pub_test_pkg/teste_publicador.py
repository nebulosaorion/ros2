#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

class TestePublicador(Node):
    def __init__(self):
        super().__init__('teste_publicador')
        self.publisher_ = self.create_publisher(String, 'topico_exemplo', 10) #
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Nó TestePublicador inicializado.')

    def timer_callback(self):
        msg = String()
        msg.data = 'Olá, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = TestePublicador()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()