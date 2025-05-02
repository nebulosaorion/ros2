#!/usr/bin/env python3
import rclpy# importo rclpy, biblioteca principal para criar programas Ros2 em Python
from rclpy.node import Node # importa a classe Node, que herda de Node para ser um nó
from sensor_msgs.msg import Image # importa msg tipo img
from cv_bridge import CvBridge##vai converter  img do opencv  e msg do ros, ela traduz basicamente os dados entre os dois formatos
from pub_test.msg import DisplayMessage#permite trabalhar com meng type e value
import cv2 #manipular img, abrir arquivos 
import numpy as np # matematica, manipulação de arrays
import os

class MediaPublisher(Node):
    def __init__(self):
        super().__init__('media_publisher')
        
        # Configurações com parâmetros ROS
        self.declare_parameter('tamanho_tela', [800, 600])
        self.tamanho_tela = self.get_parameter('tamanho_tela').value
        self.cor_fundo = (255, 255, 255)  # Branco
        self.cor_texto = (0, 0, 0)  # Preto
        self.fonte = cv2.FONT_HERSHEY_SIMPLEX
        self.escala_fonte = 1.0
        self.espessura_fonte = 2
        
        # Bridge para conversão OpenCV-ROS
        self.bridge = CvBridge()
        
        ### Subscreve ao display_2
        self.subscription = self.create_subscription(
            DisplayMessage,
            'display_2',
            self.process_message,
            10)
        
        ### Publica no ui_display
        self.publisher = self.create_publisher(
            Image, 
            'ui_display', 
            10)
        
        ### Para controle de tópicos dinâmicos
        self.active_subscription = None
        
        self.get_logger().info("Media Publisher pronto para receber comandos!")

    def process_message(self, msg):
            #######Processa mensagens recebidas do displ
        try:
            self.get_logger().info(f"Processando mensagem: type={msg.type}, value={msg.value}")
            
            if msg.type == "topic":
                self.handle_topic(msg.value)
            elif msg.type == "video":
                self.handle_video(msg.value)
            elif msg.type == "img":
                self.handle_image(msg.value)
            elif msg.type == "sentence":
                self.handle_sentence(msg.value)
            else:
                self.get_logger().warn(f"Tipo desconhecido: {msg.type}")
                
        except Exception as e:
            self.get_logger().error(f"Erro ao processar mensagem: {str(e)}")

    def handle_topic(self, topic_name):
             ###Lida com mensagens do tipo 'topic'
        # Cancela assinatura anterior se exis
        if self.active_subscription:
            self.destroy_subscription(self.active_subscription)
             # Cria nova assinatura
        self.active_subscription = self.create_subscription(
            Image,
            topic_name,
            self.forward_image,
            10)
        self.get_logger().info(f"Inscrito no tópico: {topic_name}")

    def forward_image(self, msg):
        try:
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erro ao repassar imagem: {str(e)}")

    def handle_video(self, video_path):
        # Converte path relativo/~/ para absoluto
        if not os.path.isabs(video_path):
            video_path = os.path.expanduser(video_path)
            
        if not os.path.exists(video_path):
            self.get_logger().error(f"Vídeo não encontrado: {video_path}")
            return
            
        try:
            cap = cv2.VideoCapture(video_path)
            if not cap.isOpened():
                self.get_logger().error(f"Falha ao abrir vídeo: {video_path}")
                return
            
            ret, frame = cap.read()
            if ret:
                frame = self.resize_with_aspect(frame)
                self.publish_image(frame)
            else:
                self.get_logger().error(f"Falha ao ler frame do vídeo: {video_path}")
            
            cap.release()
        except Exception as e:
            self.get_logger().error(f"Erro no processamento de vídeo: {str(e)}")

    def handle_image(self, image_path):
        # Converte path relativo/~/ para absoluto
        if not os.path.isabs(image_path):
            image_path = os.path.expanduser(image_path)
            
        if not os.path.exists(image_path):
            self.get_logger().error(f"Imagem não encontrada: {image_path}")
            return
            
        try:
            frame = cv2.imread(image_path)
            if frame is not None:
                frame = self.resize_with_aspect(frame)
                self.publish_image(frame)
            else:
                self.get_logger().error(f"Falha ao decodificar imagem: {image_path}")
        except Exception as e:
            self.get_logger().error(f"Erro no processamento de imagem: {str(e)}")

    def handle_sentence(self, text):
        try:
            frame = self.create_text_frame(text)
            self.publish_image(frame)
        except Exception as e:
            self.get_logger().error(f"Erro ao criar frame de texto: {str(e)}")

    def create_text_frame(self, text):
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

    def resize_with_aspect(self, frame):
             ###Redimensiona mantendo proporção e adiciona bordas se necessário
        h, w = frame.shape[:2]
        target_w, target_h = self.tamanho_tela
              # Calcula novas dimens
        ratio = min(target_w/w, target_h/h)
        new_w, new_h = int(w * ratio), int(h * ratio)
              # Redimensi
        resized = cv2.resize(frame, (new_w, new_h))
              # Cria fundo e centraliza a ima
        result = np.full((target_h, target_w, 3), self.cor_fundo, dtype=np.uint8)
        x_offset = (target_w - new_w) // 2
        y_offset = (target_h - new_h) // 2
        result[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
        
        return result

    def publish_image(self, frame):
            ##  Publica um frame OpenCV no tópico ui_disp
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erro ao publicar imagem: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MediaPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()