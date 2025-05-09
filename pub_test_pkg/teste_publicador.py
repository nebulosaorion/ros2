#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from pub_test.msg import DisplayMessage
import time

class TestPublisherNode(Node):
    def __init__(self):
        super().__init__('test_publisher')
        
        # Cria o publisher para o tópico display_2
        self.publisher = self.create_publisher(
            DisplayMessage,
            'display_2',
            10
        )
        
        # Configura um timer para publicar mensagens diferentes a cada 5 segundos
        self.timer = self.create_timer(5.0, self.publish_test_message)
        self.test_counter = 0
        
        self.get_logger().info("Publicador de teste iniciado!")
        
    def publish_test_message(self):
        msg = DisplayMessage()
        
        # Alterna entre diferentes tipos de mensagens para testar
        if self.test_counter % 4 == 0:
            # Teste de texto
            msg.type = "sentence"
            msg.value = "Este é um teste do sistema Media Publisher para Foxglove. Texto convertido para imagem."
            self.get_logger().info("Publicando mensagem de texto")
            
        elif self.test_counter % 4 == 1:
            # Teste de imagem 
            msg.type = "img"
            msg.value = "/home/evangelista/ros2_ws/arquivos/prato.jpg"  
            self.get_logger().info("Publicando mensagem de imagem")
            
        elif self.test_counter % 4 == 2:
            # Teste de vídeo 
            msg.type = "video"
            msg.value = "/home/evangelista/ros2_ws/arquivos/chimas.mp4"  
            self.get_logger().info("Publicando mensagem de vídeo")
            
        else:
            
            msg.type = "topic"
            msg.value = "/camera/image_raw"  # Altere para um tópico disponível
            self.get_logger().info("Publicando mensagem de tópico")
        
        # Publica a mensagem
        self.publisher.publish(msg)
        self.test_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()