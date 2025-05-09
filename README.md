# MediaPublisher – Publicador de Mídia para ROS 2

Este nó ROS 2 em Python publica imagens no tópico `/ui_display` com diferentes modos de apresentação: **texto**, **vídeo** ou **imagem**. O conteúdo publicado é formatado como uma mensagem `sensor_msgs/msg/Image`, compatível com visualizadores como o **Foxglove Studio**.

## 📦 Funcionalidades

- Exibe mensagens de texto automaticamente com transições temporizadas.
- Permite exibição de vídeos (`.mp4`) de forma contínua.
- Permite mostrar uma imagem estática.
- Alterna entre os modos via entrada de teclado.
- Pode ser controlado também via função programática `sendoDataToFoxGlove`.

## 🧱 Dependências

Antes de executar o nó, certifique-se de que as seguintes bibliotecas e pacotes estejam instalados:

### Pacotes ROS 2:
- `rclpy`
- `sensor_msgs`
- `cv_bridge`

### Bibliotecas Python:
```bash
sudo apt install python3-opencv
pip install numpy
 ##🚀 Executando o nó

    Adicione o script ao seu pacote ROS 2.

##Torne-o executável:

chmod +x media_publisher.py

##Execute com:

ros2 run pub_test media_publisher

##🎮 Controles em Tempo de Execução

Ao iniciar o nó, você poderá interagir com ele via terminal:

    1 — Modo vídeo

    2 — Modo texto

    3 — Modo imagem

    Ctrl+C — Finaliza o nó

##🧠 Função Auxiliar: sendoDataToFoxGlove

Essa função permite definir o conteúdo da UI de forma programática:

no.sendoDataToFoxGlove(
    str="Nova mensagem de texto",
    video_path="/caminho/para/video.mp4",
    image_path="/caminho/para/imagem.jpg"
)

Apenas um argumento deve ser passado por vez (str, video_path ou image_path).

##🧹 Encerramento

O nó gerencia automaticamente o encerramento de streams de vídeo e o desligamento do ROS. Pressionar Ctrl+C no terminal encerrará o nó com segurança.

##📄 Licença

##Apache License 2.0

##✉ Contato: Miriã Evangelista - evangelista@furg.br 

