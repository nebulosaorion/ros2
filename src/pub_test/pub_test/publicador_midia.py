#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import os

class MediaPublisher(Node):
    def __init__(self):
        super().__init__('publicador_midia')
       
        
        self.modo = "texto"
        self.publicador_ui = self.create_publisher(Image, '/ui_display', 10)
        self.ponte = CvBridge()
       
        
        self.mensagens = [
            "Assista ao movimento robótico.",
            "Agora, levante o objeto devagar.",
            "Pronto! Movimento concluído."
        ]
        self.mensagem_atual = 0
        self.tempo_exibicao_texto = 3.0
        self.ultima_mudanca_texto = time.time()
       
        # Configurações de mídia
        self.caminho_video = '/home/evangelista/videos/chimas.mp4' 
        self.caminho_imagem = '/home/evangelista/exemplo/imagem.jpg'  
        self.video_captura = None
        self.executando = True
        self.velocidade_video = 1.5
       
        # Configurações de display
        self.tamanho_tela = (640, 480)
        self.cor_fundo = (255, 255, 255)
        self.cor_texto = (0, 0, 0)
        self.fonte = cv2.FONT_HERSHEY_SIMPLEX
        self.escala_fonte = 0.8
        self.espessura_fonte = 2
       
        # Timer e threads
        self.temporizador = self.create_timer(1.0/30.0, self.atualizar_tela)
        self.thread_entrada = threading.Thread(target=self.escutar_teclado, daemon=True)
        self.thread_entrada.start()
       
        self.get_logger().info("Nó de publicação de mídia inicializado!")
########### NOVA FUNÇÂO ### 
    def sendoDataToFoxGlove(self, str=None, video_path=None, image_path=None):
        if str is not None:
            # passar a string para uma imagem
            frame = self.criar_frame_texto(str)
            # publica no topico ui_display
            self.publicar_frame(frame)
        elif video_path is not None:
            self.caminho_video = video_path
            self.modo = "vídeo"
            self.video_captura = None 
        elif image_path is not None:
            self.caminho_imagem = image_path
            self.modo = "imagem"

    def criar_frame_texto(self, texto):
        frame = np.full((self.tamanho_tela[1], self.tamanho_tela[0], 3),
                       self.cor_fundo, dtype=np.uint8)
       
        margem_x = int(self.tamanho_tela[0] * 0.2)
        margem_y = int(self.tamanho_tela[1] * 0.2)
        largura_maxima = self.tamanho_tela[0] - 2 * margem_x

        palavras = texto.split()
        linhas = []
        linha_atual = ""

        for palavra in palavras:
            linha_teste = linha_atual + " " + palavra if linha_atual else palavra
            (largura_teste, _), _ = cv2.getTextSize(linha_teste, self.fonte,
                                                    self.escala_fonte, self.espessura_fonte)
           
            if largura_teste <= largura_maxima:
                linha_atual = linha_teste
            else:
                if linha_atual:
                    linhas.append(linha_atual)
                linha_atual = palavra
   
        if linha_atual:
            linhas.append(linha_atual)
       
        altura_total = len(linhas) * (cv2.getTextSize("Teste", self.fonte,
                                                      self.escala_fonte, self.espessura_fonte)[0][1] + 30)
       
        y_inicio = (self.tamanho_tela[1] - altura_total) // 2 + margem_y // 2
       
        for i, linha in enumerate(linhas):
            tamanho_texto = cv2.getTextSize(linha, self.fonte,
                                            self.escala_fonte, self.espessura_fonte)[0]
            x_texto = (self.tamanho_tela[0] - tamanho_texto[0]) // 2
            y_texto = y_inicio + i * (tamanho_texto[1] + 30)
           
            cv2.putText(frame, linha, (x_texto, y_texto),
                        self.fonte, self.escala_fonte, self.cor_texto,
                        self.espessura_fonte, cv2.LINE_AA)
       
        return frame

    def redimensionar_com_aspecto(self, frame):
        altura, largura = frame.shape[:2]
        proporcao_aspecto = largura / altura
        proporcao_tela = self.tamanho_tela[0] / self.tamanho_tela[1]
       
        if proporcao_tela > proporcao_aspecto:
            nova_altura = self.tamanho_tela[1]
            nova_largura = int(nova_altura * proporcao_aspecto)
        else:
            nova_largura = self.tamanho_tela[0]
            nova_altura = int(nova_largura / proporcao_aspecto)
       
        frame_redimensionado = cv2.resize(frame, (nova_largura, nova_altura))
        frame_resultado = np.full((self.tamanho_tela[1], self.tamanho_tela[0], 3),
                                  self.cor_fundo, dtype=np.uint8)
       
        deslocamento_x = (self.tamanho_tela[0] - nova_largura) // 2
        deslocamento_y = (self.tamanho_tela[1] - nova_altura) // 2
       
        frame_resultado[deslocamento_y:deslocamento_y+nova_altura, deslocamento_x:deslocamento_x+nova_largura] = frame_redimensionado
       
        return frame_resultado

    def atualizar_tela(self):
        try:
            frame = None
            if self.modo == "texto":
                frame = self.tratar_modo_texto()
            elif self.modo == "vídeo":
                frame = self.tratar_modo_video()
            elif self.modo == "imagem":
                frame = self.tratar_modo_imagem()
           
            if frame is not None:
                self.publicar_frame(frame)

        except Exception as e:
            self.get_logger().error(f"Erro no display: {str(e)}")

    def tratar_modo_texto(self):
        tempo_atual = time.time()
        if tempo_atual - self.ultima_mudanca_texto >= self.tempo_exibicao_texto:
            self.mensagem_atual = (self.mensagem_atual + 1) % len(self.mensagens)
            self.ultima_mudanca_texto = tempo_atual
        return self.criar_frame_texto(self.mensagens[self.mensagem_atual])

    def tratar_modo_video(self):
        if self.video_captura is None or not self.video_captura.isOpened():
            if not os.path.isfile(self.caminho_video):
                self.get_logger().error(f"Arquivo de vídeo não encontrado: {self.caminho_video}")
                return None  
           
            self.video_captura = cv2.VideoCapture(self.caminho_video)
            if not self.video_captura.isOpened():
                self.get_logger().error(f"Falha ao abrir o vídeo: {self.caminho_video}")
                return None  

        for _ in range(int(self.velocidade_video)):
            ret, frame = self.video_captura.read()
       
        if not ret:
            self.video_captura.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.video_captura.read()
       
        return self.redimensionar_com_aspecto(frame) if ret else None

    def tratar_modo_imagem(self):
        if not os.path.isfile(self.caminho_imagem):
            raise FileNotFoundError(f"Imagem não encontrada: {self.caminho_imagem}")
       
        frame = cv2.imread(self.caminho_imagem)
        if frame is None:
            raise RuntimeError("Falha ao carregar imagem")
       
        return self.redimensionar_com_aspecto(frame)

    def publicar_frame(self, frame):
        msg = self.ponte.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publicador_ui.publish(msg)

    def escutar_teclado(self):
        self.get_logger().info("Controles:\n1 - Vídeo\n2 - Texto\n3 - Imagem\nCtrl+C - Sair")
        while self.executando:
            try:
                tecla = input("Digite 1 (vídeo), 2 (texto) ou 3 (imagem): ").strip()
               
                if tecla == '1':
                    self.modo = "vídeo"
                    self.get_logger().info("Modo: VÍDEO")
                elif tecla == '2':
                    self.modo = "texto"
                    self.get_logger().info("Modo: TEXTO")
                elif tecla == '3':
                    self.modo = "imagem"
                    self.get_logger().info("Modo: IMAGEM")
                else:
                    self.get_logger().warning("Opção inválida!")
                   
            except Exception as e:
                self.get_logger().error(f"Erro na entrada: {str(e)}")
                break

    def destruir_no(self):
        self.get_logger().info("Encerrando nó...")
        self.executando = False
       
        if self.video_captura and self.video_captura.isOpened():
            self.video_captura.release()
       
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        no = MediaPublisher()
        # Exemplo de uso:
        no.sendoDataToFoxGlove(video_path='/home/evangelista/videos/chimas.mp4')
        rclpy.spin(no)
    except KeyboardInterrupt:
        no.get_logger().info("Interrompido pelo usuário")
    finally:
        no.destruir_no()
        rclpy.shutdown()

if __name__ == '__main__':
    main()