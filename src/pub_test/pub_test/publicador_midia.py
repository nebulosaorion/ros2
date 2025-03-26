#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time

class MediaPublisher(Node):
    def __init__(self):
        """
        Inicializa o nó publisher de mídia com tratamento de proporção de vídeo.
        
        Melhorias principais:
        - Mantém proporção original do vídeo
        - Adiciona barras pretas (letterbox) quando necessário
        - Centraliza o vídeo no display
        """
        super().__init__('media_publisher')
        
        # Configurações iniciais
        self.modo = "texto"
        self.publisher_ui = self.create_publisher(Image, '/ui_display', 10)
        self.bridge = CvBridge()
        
        # Mensagens de texto
        self.messages = [
            "Assista ao movimento do braço robótico.",
            "Agora, levante o objeto devagar.",
            "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
            "AAAAAAAAAAA AAAAAAAAAAAAAAAAA AAAAAAAAAAAAAAAAAAA AAAAAAAAAAAAAAAAA"
        ]
        self.current_msg = 0
        
        # Configurações de vídeo
        self.video_path = '/home/evangelista/videos/chimas.mp4'
        self.cap = None
        self.video_aspect_ratio = None  # Será calculado quando abrir o vídeo
        self.running = True
        
        # Configurações de display
        self.display_size = (640, 480)  # Largura, Altura
        self.bg_color = (255, 255, 255)  # Branco (BGR)
        self.text_color = (0, 0, 0)      # Preto (BGR)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.8
        self.font_thickness = 2
        
        # Inicializa timer e thread de entrada
        self.timer = self.create_timer(1.0, self.update_display)  # 1 Hz
        self.thread_input = threading.Thread(target=self.listen_keyboard, daemon=True)
        self.thread_input.start()
        
        self.get_logger().info("Nó de publicação de mídia inicializado com sucesso!")

    def create_text_frame(self, text):
        """Cria um frame com fundo branco e texto preto centralizado."""
        frame = np.full((self.display_size[1], self.display_size[0], 3), 
                       self.bg_color, dtype=np.uint8)
        
        text_size = cv2.getTextSize(text, self.font, self.font_scale, self.font_thickness)[0]
        text_x = (self.display_size[0] - text_size[0]) // 2
        text_y = (self.display_size[1] + text_size[1]) // 2
        
        cv2.putText(frame, text, (text_x, text_y), 
                   self.font, self.font_scale, 
                   self.text_color, self.font_thickness, 
                   cv2.LINE_AA)
        return frame

    def resize_video_frame(self, frame):
        """
        Redimensiona o frame do vídeo mantendo a proporção original,
        adicionando barras pretas se necessário.
        
        Args:
            frame (numpy.ndarray): Frame do vídeo a ser redimensionado
            
        Returns:
            numpy.ndarray: Frame redimensionado com letterbox se necessário
        """
        if self.video_aspect_ratio is None:
            # Calcula a proporção do vídeo na primeira vez
            height, width = frame.shape[:2]
            self.video_aspect_ratio = width / height
            self.get_logger().info(f"Proporção do vídeo detectada: {self.video_aspect_ratio:.2f}")
        
        display_aspect = self.display_size[0] / self.display_size[1]
        
        # Calcula o novo tamanho mantendo a proporção
        if display_aspect > self.video_aspect_ratio:
            # Vídeo mais estreito que o display (barras laterais)
            new_height = self.display_size[1]
            new_width = int(new_height * self.video_aspect_ratio)
        else:
            # Vídeo mais largo que o display (barras superior/inferior)
            new_width = self.display_size[0]
            new_height = int(new_width / self.video_aspect_ratio)
        
        # Redimensiona o vídeo
        resized_frame = cv2.resize(frame, (new_width, new_height))
        
        # Cria fundo branco
        result_frame = np.full((self.display_size[1], self.display_size[0], 3), 
                          self.bg_color, dtype=np.uint8)
        
        # Centraliza o vídeo no display
        x_offset = (self.display_size[0] - new_width) // 2
        y_offset = (self.display_size[1] - new_height) // 2
        
        # Coloca o vídeo redimensionado no centro do display
        result_frame[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = resized_frame
        
        return result_frame

    def update_display(self):
        """Atualiza o display com o conteúdo atual (texto ou vídeo)."""
        try:
            if self.modo == "texto":
                text = self.messages[self.current_msg]
                frame = self.create_text_frame(text)
                self.current_msg = (self.current_msg + 1) % len(self.messages)
                
            elif self.modo == "vídeo":
                if self.cap is None or not self.cap.isOpened():
                    self.cap = cv2.VideoCapture(self.video_path)
                    if not self.cap.isOpened():
                        self.get_logger().error(f"Falha ao abrir vídeo: {self.video_path}")
                        return
                
                ret, frame = self.cap.read()
                if not ret:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()
                
                # Redimensiona mantendo proporção
                frame = self.resize_video_frame(frame)
            
            # Publica o frame
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_ui.publish(msg)
            self.get_logger().info(f'Publicando: {self.modo.upper()}')

        except Exception as e:
            self.get_logger().error(f"Erro ao atualizar display: {str(e)}")

    def listen_keyboard(self):
        """Escuta comandos do teclado para alternar entre modos."""
        self.get_logger().info("Controles:\n1 - Modo Vídeo\n2 - Modo Texto\nCtrl+C - Sair")
        
        while self.running:
            try:
                key = input("Digite 1 (vídeo) ou 2 (texto): ").strip()
                
                if key == '1':
                    self.modo = "vídeo"
                    self.get_logger().info("Modo alterado para: VÍDEO")
                elif key == '2':
                    self.modo = "texto"
                    self.get_logger().info("Modo alterado para: TEXTO")
                else:
                    self.get_logger().warning("Opção inválida! Use 1 ou 2.")
                    
            except Exception as e:
                self.get_logger().error(f"Erro na entrada: {str(e)}")
                break

    def destroy_node(self):
        """Finalização segura do nó."""
        self.get_logger().info("Encerrando nó de mídia...")
        self.running = False
        
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Recursos de vídeo liberados.")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        media_publisher = MediaPublisher()
        rclpy.spin(media_publisher)
        
    except KeyboardInterrupt:
        media_publisher.get_logger().info("Interrupção por teclado detectada.")
    except Exception as e:
        media_publisher.get_logger().fatal(f"Erro fatal: {str(e)}")
    finally:
        media_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()