#!/usr/bin/env python3
#coding: utf-8

import rclpy # importo rclpy, biblioteca principal para criar programas Ros2 em Python
from rclpy.node import Node  # importa a classe Node, que herda de Node para ser um nó
from sensor_msgs.msg import Image  # importa msg tipo img
from cv_bridge import CvBridge ##vai converter  img do opencv  e msg do ros, ela traduz basicamente os dados entre os dois formatos
from pub_test.msg import DisplayMessage #permite trabalhar com meng type e value
import cv2 #manipular img, abrir arquivos 
import numpy as np  # matematica, manipulação de arrays
import os

#Biblioteca para maquina de estados
from yasmin import State, StateMachine
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

class MediaPublisherState(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.bridge = CvBridge()
        self.tamanho_tela = [800, 600]
        self.cor_fundo = (255, 255, 255)
        self.cor_texto = (0, 0, 0)
        self.fonte = cv2.FONT_HERSHEY_SIMPLEX
        self.escala_fonte = 1.0
        self.espessura_fonte = 2

    def on_enter(self, blackboard: Blackboard) -> None:
        self.node = blackboard.node
        self.publisher = blackboard.publisher
        if 'tamanho_tela' in blackboard:
            self.tamanho_tela = blackboard.tamanho_tela

class ProcessMessageState(MediaPublisherState):
    def __init__(self):
        super().__init__()
        self.active_subscription = None

    def execute(self, blackboard: Blackboard) -> str:
        try:
            msg = blackboard.display_msg
            self.node.get_logger().info(f"Processando mensagem: type={msg.type}, value={msg.value}")
            
            if msg.type == "topic":
                return self.handle_topic(blackboard, msg.value)
            elif msg.type == "video":
                return self.handle_video(blackboard, msg.value)
            elif msg.type == "img":
                return self.handle_image(blackboard, msg.value)
            elif msg.type == "sentence":
                return self.handle_sentence(blackboard, msg.value)
            else:
                self.node.get_logger().warn(f"Tipo desconhecido: {msg.type}")
                return ABORT
        except Exception as e:
            self.node.get_logger().error(f"Erro ao processar mensagem: {str(e)}")
            return ABORT
        

    def handle_image(self, blackboard: Blackboard, image_path: str) -> str:
        if not os.path.isabs(image_path):
            image_path = os.path.expanduser(image_path)
            
        if not os.path.exists(image_path):
            self.node.get_logger().error(f"Imagem não encontrada: {image_path}")
            return ABORT
            
        try:
            frame = cv2.imread(image_path)
            if frame is not None:
                frame = self.resize_with_aspect(frame)
                self.publish_image(frame)
                return SUCCEED
            else:
                self.node.get_logger().error(f"Falha ao decodificar imagem: {image_path}")
                return ABORT
        except Exception as e:
            self.node.get_logger().error(f"Erro no processamento de imagem: {str(e)}")
            return ABORT

    def handle_sentence(self, blackboard: Blackboard, text: str) -> str:
        try:
            frame = self.create_text_frame(text)
            self.publish_image(frame)
            return SUCCEED
        except Exception as e:
            self.node.get_logger().error(f"Erro ao criar frame de texto: {str(e)}")
            return ABORT

    def create_text_frame(self, text: str) -> np.ndarray:
        frame = np.full((self.tamanho_tela[1], self.tamanho_tela[0], 3), 
                       self.cor_fundo, dtype=np.uint8)
        
        words = text.split()
        lines = []
        current_line = ""
        
        for word in words:
            test_line = f"{current_line} {word}" if current_line else word
            (width, _), _ = cv2.getTextSize(test_line, self.fonte,
                                            self.escala_fonte, self.espessura_fonte)
            
            if width <= self.tamanho_tela[0] - 40:
                current_line = test_line
            else:
                lines.append(current_line)
                current_line = word
                
        if current_line:
            lines.append(current_line)
        
        text_height = len(lines) * (cv2.getTextSize("A", self.fonte,
                                  self.escala_fonte, self.espessura_fonte)[0][1] + 20)
        y_start = (self.tamanho_tela[1] - text_height) // 2
        
        for i, line in enumerate(lines):
            text_size = cv2.getTextSize(line, self.fonte,
                                      self.escala_fonte, self.espessura_fonte)[0]
            x = (self.tamanho_tela[0] - text_size[0]) // 2
            y = y_start + i * (text_size[1] + 20)
            
            cv2.putText(frame, line, (x, y),
                       self.fonte, self.escala_fonte, self.cor_texto,
                       self.espessura_fonte, cv2.LINE_AA)
        
        return frame

    def resize_with_aspect(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        target_w, target_h = self.tamanho_tela
        
        ratio = min(target_w/w, target_h/h)
        new_w, new_h = int(w * ratio), int(h * ratio)
        
        resized = cv2.resize(frame, (new_w, new_h))
        
        result = np.full((target_h, target_w, 3), self.cor_fundo, dtype=np.uint8)
        x_offset = (target_w - new_w) // 2
        y_offset = (target_h - new_h) // 2
        result[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
        
        return result

    def publish_image(self, frame: np.ndarray) -> None:
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
        except Exception as e:
            self.node.get_logger().error(f"Erro ao publicar imagem: {str(e)}")

class MediaPublisherNode:
    def __init__(self):
        self.node = Node("media_publisher")
        self.node.declare_parameter('tamanho_tela', [800, 600])
        
        self.blackboard = Blackboard()
        self.blackboard.node = self.node
        self.blackboard.tamanho_tela = self.node.get_parameter('tamanho_tela').value
        self.blackboard.publisher = self.node.create_publisher(Image, 'ui_display', 10)
        
        # Configuração da máquina de estados
        self.fsm = StateMachine(outcomes=["finished"])
        self.fsm.add_state(
            "PROCESSING",
            ProcessMessageState(),
            transitions={
                SUCCEED: "finished",
                ABORT: "finished"
            }
        )
        
        self.subscription = self.node.create_subscription(
            DisplayMessage,
            'display_2',
            self.message_callback,
            10
        )
        
        self.node.get_logger().info("Media Publisher pronto para receber comandos!")

    def message_callback(self, msg: DisplayMessage):
        self.blackboard.display_msg = msg
        outcome = self.fsm(self.blackboard)
        self.node.get_logger().info(f"Processamento concluído: {outcome}")

def main(args=None):
    rclpy.init(args=args)
    node = MediaPublisherNode()
    
    try:
        rclpy.spin(node.node)
    except KeyboardInterrupt:
        pass
    finally:
        node.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()