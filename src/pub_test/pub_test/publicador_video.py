import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import imageio
import time
import numpy as np

class PublicadorMidia(Node):
    def __init__(self):
        super().__init__('publicador_midia')
        self.publisher_ = self.create_publisher(Image, 'video_topic', 10)
        self.bridge = CvBridge()

        # Caminho para os arquivos
        self.video_path = '/home/evangelista/videos/chimas.mp4'  
        self.gif_path = '/home/evangelista/exemplo/videogif.mp4'  

        self.mostrar_gif = True  # Alternar entre GIF e vídeo

        if self.mostrar_gif:
            self.publicar_gif()
        else:
            self.iniciar_video()

    def publicar_gif(self):
        """Publica um vídeo no tópico de vídeo, tratando-o como um GIF."""
        self.get_logger().info("Exibindo vídeo como GIF...")

        cap = cv2.VideoCapture(self.gif_path)
        
        if not cap.isOpened():
            self.get_logger().error(f"Não foi possível abrir o vídeo: {self.gif_path}")
            return

        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps <= 0:
            self.get_logger().warn("FPS do vídeo não detectado, usando 10 FPS.")
            fps = 10  # Assume um FPS padrão se não for detectado

        start_time = time.time()
        while time.time() - start_time < 5:  # Exibir por 5 segundos
            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reinicia o vídeo se acabar
                continue

            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info("Publicando frame do vídeo")
            time.sleep(0.5 / fps)

        cap.release()
        self.get_logger().info("Vídeo finalizado. Voltando ao outro vídeo.")
        self.iniciar_video()


    def iniciar_video(self):
        """Inicia a transmissão do vídeo após o GIF."""
        self.cap = cv2.VideoCapture(self.video_path)

        if not self.cap.isOpened():
            self.get_logger().error(f"Não foi possível abrir o vídeo: {self.video_path}")
            return

        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        if self.fps <= 0:
            self.get_logger().warn("FPS do vídeo não detectado, usando 30 FPS.")
            self.fps = 30 

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)
        self.get_logger().info(f"Publicando vídeo a {self.fps} FPS")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info('Publicando frame do vídeo')
        else:
            self.get_logger().info('Fim do vídeo')
            self.cap.release()

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    publicador_midia = PublicadorMidia()
    rclpy.spin(publicador_midia)
    publicador_midia.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
