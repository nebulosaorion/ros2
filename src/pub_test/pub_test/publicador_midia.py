#!/usr/bin/env python3
import rclpy  # Fornece a API básica para criar nós
from rclpy.node import Node  # Classe base para criar nós personalizados
from sensor_msgs.msg import Image  # Usada para publicar frames no tópico /ui_display
from cv_bridge import CvBridge  # Converte entre imagens OpenCV (numpy) e mensagens ROS
import cv2  # Processamento de imagem/vídeo (leitura, redimensionamento, exibição, etc.)
import numpy as np  # Manipulação eficiente de arrays multidimensionais (imagens são arrays numpy)
import threading  # Para executar o loop de entrada do teclado em paralelo
import time

class MediaPublisher(Node):
    def __init__(self):
        super().__init__('media_publisher')
        
        # Configurações iniciais
        self.modo = "texto"
        self.publisher_ui = self.create_publisher(Image, '/ui_display', 10)
        self.bridge = CvBridge()
        
        # Mensagens de texto
        self.messages = [
            "Assista ao movimento robotico.",
            "Agora, levante o objeto devagar.",
            "Pronto! Movimento concluido.",
            "AAAAAAAA AAAAAAAA AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA AAAAAAAAAAAAAAAAAAAAA AAAAAAAAAAAAAAAAAA AAAAAAAAAAAAA AAAAAAAAAAAAA "
        ]
        self.current_msg = 0
        self.text_display_time = 3.0  # 3 segundos por mensagem (mais lento)
        self.last_text_change = time.time()
        
        # Configurações de mídia
        self.video_path = '/home/evangelista/videos/chimas.mp4'
        self.image_path = '/home/evangelista/exemplo/imagem.jpg'
        self.cap = None
        self.video_aspect_ratio = None
        self.image_aspect_ratio = None
        self.running = True
        self.video_speed = 1.5  # Fator de aceleração do vídeo (1.5x mais rápido)
        
        # Configurações de display
        self.display_size = (640, 480)  
        self.bg_color = (255, 255, 255)  
        self.text_color = (0, 0, 0)      
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.8
        self.font_thickness = 2
        
        # Timer mais rápido para vídeo (30Hz)
        self.timer = self.create_timer(1.0/30.0, self.update_display)
        self.thread_input = threading.Thread(target=self.listen_keyboard, daemon=True)
        self.thread_input.start()
        
        self.get_logger().info("Nó de publicação de mídia inicializado com sucesso!")

    def create_text_frame(self, text):
        """Cria um frame com fundo branco e texto preto centralizado com margens."""
        frame = np.full((self.display_size[1], self.display_size[0], 3), 
                       self.bg_color, dtype=np.uint8)
        
        # Configurações de margem (20% da largura/altura)
        margin_x = int(self.display_size[0] * 0.2)
        margin_y = int(self.display_size[1] * 0.2)
        max_width = self.display_size[0] - 2 * margin_x

        # Configurações de texto
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness = 2
        color = (0, 0, 0)  # Preto
        line_spacing = 30  # Espaço entre linhas
        
        # Divide o texto em palavras
        words = text.split()
        lines = []
        current_line = ""

        for word in words:
            # Testa se a palavra cabe na linha atual
            test_line = current_line + " " + word if current_line else word
            (test_width, _), _ = cv2.getTextSize(test_line, font, font_scale, thickness)
            
            if test_width <= max_width:
                current_line = test_line
            else:
                if current_line:  # Adiciona a linha atual se não estiver vazia
                    lines.append(current_line)
                current_line = word
    
        if current_line:  # Adiciona a última linha
            lines.append(current_line)
        
        # Calcula a altura total do texto
        total_height = len(lines) * (cv2.getTextSize("Test", font, font_scale, thickness)[0][1] + line_spacing)
        
        # Posição Y inicial para centralizar verticalmente
        y_start = (self.display_size[1] - total_height) // 2 + margin_y // 2
        
        # Desenha cada linha
        for i, line in enumerate(lines):
            text_size = cv2.getTextSize(line, font, font_scale, thickness)[0]
            text_x = (self.display_size[0] - text_size[0]) // 2
            text_y = y_start + i * (text_size[1] + line_spacing)
            
            cv2.putText(frame, line, (text_x, text_y), 
                      font, font_scale, color, thickness, cv2.LINE_AA)
        
        return frame

    def resize_with_aspect(self, frame, aspect_ratio=None):
        """
        Redimensiona qualquer frame (vídeo ou imagem) mantendo a proporção,
        adicionando barras pretas se necessário.
        
        Args:
            frame (numpy.ndarray): Frame a ser redimensionado
            aspect_ratio (float): Proporção desejada (None para calcular automaticamente)
            
        Returns:
            numpy.ndarray: Frame redimensionado com letterbox se necessário
        """
        if aspect_ratio is None:
            height, width = frame.shape[:2]
            aspect_ratio = width / height
        
        display_aspect = self.display_size[0] / self.display_size[1]
        
        # Calcula o novo tamanho mantendo a proporção
        if display_aspect > aspect_ratio:
            # Conteúdo mais estreito que o display (barras laterais)
            new_height = self.display_size[1]
            new_width = int(new_height * aspect_ratio)
        else:
            # Conteúdo mais largo que o display (barras superior/inferior)
            new_width = self.display_size[0]
            new_height = int(new_width / aspect_ratio)
        
        # Redimensiona
        resized_frame = cv2.resize(frame, (new_width, new_height))
        
        # Cria fundo branco
        result_frame = np.full((self.display_size[1], self.display_size[0], 3), 
                          self.bg_color, dtype=np.uint8)
        
        # Centraliza o conteúdo no display
        x_offset = (self.display_size[0] - new_width) // 2
        y_offset = (self.display_size[1] - new_height) // 2
        
        # Coloca o conteúdo redimensionado no centro
        result_frame[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = resized_frame
        
        return result_frame

    def update_display(self):
        """Atualiza o display com o conteúdo atual (texto, vídeo ou imagem)."""
        try:
            if self.modo == "texto":
                current_time = time.time()
                if current_time - self.last_text_change >= self.text_display_time:
                    self.current_msg = (self.current_msg + 1) % len(self.messages)
                    self.last_text_change = current_time
                
                text = self.messages[self.current_msg]
                frame = self.create_text_frame(text)
                
            elif self.modo == "vídeo":
                if self.cap is None or not self.cap.isOpened():
                    self.cap = cv2.VideoCapture(self.video_path)
                    if not self.cap.isOpened():
                        self.get_logger().error(f"Falha ao abrir vídeo: {self.video_path}")
                        return
                
                # Pula frames para fazer o vídeo mais rápido
                for _ in range(int(self.video_speed)):
                    ret, frame = self.cap.read()
                
                if not ret:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()
                
                frame = self.resize_with_aspect(frame, self.video_aspect_ratio)
            
            elif self.modo == "imagem":
                frame = cv2.imread(self.image_path)
                if frame is None:
                    self.get_logger().error(f"Falha ao carregar imagem: {self.image_path}")
                    return
                
                if self.image_aspect_ratio is None:
                    height, width = frame.shape[:2]
                    self.image_aspect_ratio = width / height
                
                frame = self.resize_with_aspect(frame, self.image_aspect_ratio)
            
            # Publica o frame
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_ui.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Erro ao atualizar display: {str(e)}")

    def listen_keyboard(self):
        """Escuta comandos do teclado para alternar entre modos."""
        self.get_logger().info("Controles:\n1 - Modo Vídeo\n2 - Modo Texto\n3 - Modo Imagem\nCtrl+C - Sair")
        
        while self.running:
            try:
                key = input("Digite 1 (vídeo), 2 (texto) ou 3 (imagem): ").strip()
                
                if key == '1':
                    self.modo = "vídeo"
                    self.get_logger().info("Modo alterado para: VÍDEO")
                elif key == '2':
                    self.modo = "texto"
                    self.get_logger().info("Modo alterado para: TEXTO")
                elif key == '3':
                    self.modo = "imagem"
                    self.get_logger().info("Modo alterado para: IMAGEM")
                else:
                    self.get_logger().warning("Opção inválida! Use 1, 2 ou 3.")
                    
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