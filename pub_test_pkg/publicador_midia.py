#!/usr/bin/env python3
#coding: utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pub_test.msg import DisplayMessage
import cv2
import numpy as np
import os
import threading
import time

from yasmin import State, StateMachine
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

class MediaPublisherState(State):
    def __init__(self, node, publisher):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = node
        self.publisher = publisher
        self.bridge = CvBridge()
        self.tamanho_tela = node.get_parameter('tamanho_tela').value
        self.cor_fundo = tuple(node.get_parameter('cor_fundo').value)
        self.cor_texto = tuple(node.get_parameter('cor_texto').value)
        self.fonte = cv2.FONT_HERSHEY_SIMPLEX
        self.escala_fonte = 1.0
        self.espessura_fonte = 2
        self.video_capture = None
        self.video_thread = None
        self.stop_video_flag = threading.Event()
        self.active_subscription = None

    def stop_current_media(self):
        if self.video_thread and self.video_thread.is_alive():
            self.node.get_logger().info("Parando reprodução de vídeo anterior...")
            self.stop_video_flag.set()
            self.video_thread.join(timeout=1.0)
            if self.video_thread.is_alive():
                self.node.get_logger().warn("Thread de vídeo pode não ter parado corretamente.")
            self.video_thread = None
            self.stop_video_flag.clear()
        if self.video_capture:
            self.video_capture.release()
            self.video_capture = None
        
        if self.active_subscription:
            self.node.destroy_subscription(self.active_subscription)
            self.active_subscription = None
            self.node.get_logger().info("Inscrição de tópico anterior cancelada.")

class ProcessMessageState(MediaPublisherState):
    def __init__(self, node, publisher):
        super().__init__(node, publisher)

    def execute(self, blackboard: Blackboard) -> str:
        try:
            msg = blackboard.display_msg 
            self.node.get_logger().info(f"Processando mensagem: type={msg.type}, value={msg.value}")
            
            self.stop_current_media()

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
        
    def handle_video(self, blackboard: Blackboard, video_path: str) -> str:
        if not os.path.isabs(video_path):
            video_path = os.path.expanduser(video_path)
        
        if not os.path.exists(video_path):
            self.node.get_logger().error(f"Vídeo não encontrado: {video_path}")
            return ABORT

        try:
            self.video_capture = cv2.VideoCapture(video_path)
            if not self.video_capture.isOpened():
                self.node.get_logger().error(f"Falha ao abrir vídeo: {video_path}")
                self.video_capture = None
                return ABORT
            
            self.node.get_logger().info(f"Iniciando reprodução de vídeo: {video_path}")
            
            self.stop_video_flag.clear()
            self.video_thread = threading.Thread(target=self._play_video_loop)
            self.video_thread.daemon = True
            self.video_thread.start()
            
            return SUCCEED
        except Exception as e:
            self.node.get_logger().error(f"Erro no processamento de vídeo: {str(e)}")
            self.video_capture = None
            return ABORT

    def _play_video_loop(self):
        if not self.video_capture:
            self.node.get_logger().error("Nenhuma captura de vídeo disponível para reprodução.")
            return

        fps = self.video_capture.get(cv2.CAP_PROP_FPS)
        delay = 1.0 / fps if fps > 0 else 0.03

        while rclpy.ok() and not self.stop_video_flag.is_set():
            ret, frame = self.video_capture.read()
            if not ret:
                self.node.get_logger().info("Fim do vídeo, reiniciando...")
                self.video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.video_capture.read()
                if not ret:
                    self.node.get_logger().error("Não foi possível reiniciar a reprodução do vídeo.")
                    break
            
            frame = self.resize_with_aspect(frame)
            self.publish_image(frame)
            time.sleep(delay)
        
        self.node.get_logger().info("Reprodução de vídeo encerrada.")
        if self.video_capture:
            self.video_capture.release()
            self.video_capture = None
        self.stop_video_flag.clear()

    def handle_topic(self, blackboard: Blackboard, topic_name: str) -> str:
        if not topic_name.startswith('/'):
            topic_name = '/' + topic_name
        
        try:
            if self.active_subscription:
                self.node.destroy_subscription(self.active_subscription)
                self.active_subscription = None

            self.active_subscription = self.node.create_subscription(
                Image,
                topic_name,
                lambda msg: self._topic_image_callback(msg, blackboard),
                10
            )
            self.node.get_logger().info(f"Inscrito no tópico de imagem: {topic_name}")
            return SUCCEED
        except Exception as e:
            self.node.get_logger().error(f"Erro ao inscrever no tópico: {str(e)}")
            return ABORT
    
    def _topic_image_callback(self, msg: Image, blackboard: Blackboard):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            frame = self.resize_with_aspect(cv_image)
            self.publish_image(frame)
        except Exception as e:
            self.node.get_logger().error(f"Erro ao processar imagem do tópico: {str(e)}")

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
        self.node.declare_parameter('cor_fundo', [255, 255, 255])
        self.node.declare_parameter('cor_texto', [0, 0, 0])
        
        self.publisher = self.node.create_publisher(Image, 'ui_display', 10)
        
        self.fsm = StateMachine(outcomes=["finished"])
        self.fsm.add_state(
            "PROCESSING",
            ProcessMessageState(self.node, self.publisher),
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

    def message_callback(self, msg):
        blackboard = Blackboard()
        blackboard.display_msg = msg
        blackboard.tamanho_tela = self.node.get_parameter('tamanho_tela').value
        
        try:
            outcome = self.fsm(blackboard)
            self.node.get_logger().info(f"Processamento concluído: {outcome}")
        except Exception as e:
            self.node.get_logger().error(f"Erro no callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MediaPublisherNode()
        rclpy.spin(node.node)
    except Exception as e:
        node.node.get_logger().error(f"Erro fatal: {str(e)}")
    finally:
        node.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()