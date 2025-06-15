# 🖼️ Publicador de Mídia ROS2 - Sistema de Exibição para Foxglove

Este projeto apresenta um nó ROS2 intermediário projetado para processar texto, imagens, vídeos e outros tópicos ROS, convertendo-os para um formato adequado para exibição no Foxglove Studio.

## 🧠 Conceito

O cerne deste projeto é uma máquina de estados construída com o framework [Yasmin](https://pypi.org/project/yasmin/). O nó `media_publisher` escuta por mensagens no tópico `display_2`. Essas mensagens, do tipo `DisplayMessage.msg`, contêm dois campos chave:

* `type`: Especifica o tipo de mídia (por exemplo, `"sentence"`, `"img"`, `"video"` ou `"topic"`).
* `value`: Contém o conteúdo associado ao `type` (por exemplo, texto, caminho do arquivo ou nome do tópico).

O `media_publisher` processa essas informações e as renderiza como uma imagem, que é então publicada no tópico `ui_display`. Este tópico `ui_display` pode ser visualizado em tempo real dentro do [Foxglove Studio](https://foxglove.dev/).

## 📨 Mensagem Personalizada: `DisplayMessage.msg`

A mensagem personalizada utilizada para os comandos de entrada é definida da seguinte forma:

```text
string type
string value
```

Esta estrutura de mensagem permite um controle flexível sobre a mídia exibida pelo nó media_publisher.

## ⚙️ Nós e Comunicação

Este pacote contém dois nós ROS2 principais:

    media_publisher:
        Propósito: Este é o nó principal responsável por processar as requisições de mídia e publicá-las como imagens. Ele usa uma máquina de estados para gerenciar diferentes tipos de mídia.
        Entrada: Ele assina o tópico display_2, esperando mensagens do tipo DisplayMessage.
        Saída: Ele publica mensagens sensor_msgs/Image para o tópico ui_display, que representa a saída visual para o Foxglove Studio.
        Parâmetros:
            tamanho_tela: Define a resolução da tela de saída (padrão: [800, 600]).
            cor_fundo: Define a cor de fundo (padrão: [255, 255, 255] - branco).
            cor_texto: Define a cor do texto para frases (padrão: [0, 0, 0] - preto).

    teste_publicador:
        Propósito: Este é um nó de teste simples que publica periodicamente uma mensagem std_msgs/String no tópico topico_exemplo. É usado principalmente para demonstrar a capacidade do media_publisher de assinar e exibir conteúdo de outros tópicos ROS.
        Saída: Ele publica mensagens std_msgs/String no tópico topico_exemplo.

**Fluxo de Comunicação:**  
O media_publisher atua como um intermediário. Você envia uma DisplayMessage para display_2. Com base no campo type desta mensagem, o media_publisher irá:

    Exibir uma frase diretamente.
    Carregar e exibir uma imagem de um caminho especificado.
    Reproduzir um vídeo de um caminho especificado.
    Assinar outro tópico ROS (por exemplo, /topico_exemplo do teste_publicador) e retransmitir seu conteúdo de imagem para ui_display.

## 🧠 Explicação da Máquina de Estados (Yasmin)

O nó media_publisher implementa uma máquina de estados usando o framework Yasmin para lidar com diferentes modos de processamento de mídia.

### Estado Base: MediaPublisherState

Esta é uma classe base que fornece funcionalidades comuns para todos os estados de manipulação de mídia. Ela gerencia o CvBridge para converter entre imagens OpenCV e mensagens ROS Image, dimensões da tela, cores e configurações de fonte. Crucialmente, inclui um método stop_current_media() para parar graciosamente qualquer reprodução de mídia em andamento (como threads de vídeo ou assinaturas de tópicos ativas) antes de fazer a transição para um novo estado.

### Estado Principal: ProcessMessageState

Este estado é responsável por interpretar a DisplayMessage de entrada e delegar para a lógica de tratamento de mídia apropriada.

    execute(self, blackboard): Este é o método principal do estado. Ele recupera a DisplayMessage do blackboard do Yasmin, registra a mensagem, chama stop_current_media() para garantir que qualquer mídia anterior seja parada e, em seguida, despacha a mensagem para um dos métodos de tratamento específicos com base em msg.type.

Veja como cada método de tratamento funciona:

    handle_video(self, blackboard, video_path):
        Funcionalidade: Este método lida com a reprodução de arquivos de vídeo.
        Processo: Ele primeiro verifica o caminho do arquivo de vídeo. Se válido, inicializa um objeto VideoCapture do opencv. Os quadros de vídeo são então lidos em uma threading.Thread separada (_play_video_loop) para evitar o bloqueio do loop ROS. Cada quadro é redimensionado para caber na tela e publicado em ui_display. O vídeo é reproduzido em loop indefinidamente até que uma nova mensagem seja recebida ou o nó seja desligado.

    handle_topic(self, blackboard, topic_name):
        Funcionalidade: Este método assina um tópico ROS Image especificado e exibe seu conteúdo.
        Processo: Ele primeiro garante que qualquer assinatura de tópico anterior seja encerrada. Em seguida, cria uma nova assinatura para o topic_name. A função _topic_image_callback é acionada sempre que uma nova Image mensagem é recebida nesse tópico. Este callback converte a Image ROS em uma imagem OpenCV, a redimensiona e a publica em ui_display. Isso permite a exibição de fluxos de imagem ao vivo de outros nós ROS.

    handle_image(self, blackboard, image_path):
        Funcionalidade: Este método exibe um arquivo de imagem estática.
        Processo: Ele carrega a imagem do image_path fornecido usando OpenCV. A imagem é então redimensionada para caber nas dimensões da tela e publicada uma vez em ui_display.

    handle_sentence(self, blackboard, text):
        Funcionalidade: Este método gera uma imagem a partir de uma dada string de texto.
        Processo: Ele cria um quadro de imagem em branco preenchido com a cor_fundo. O text de entrada é então quebrado para caber na largura da tela, e cada linha é desenhada na imagem com a cor_texto. O quadro de imagem resultante é publicado em ui_display.

## 🛠️ Configuração e Instalação

Antes de construir, certifique-se de ter o ROS2 instalado (por exemplo, Humble, Iron, Jazzy).

### Clonar o Repositório:

```bash
git clone https://github.com/nebulosaorion/ros2.git nebulosaorion/ros2
```

### Navegar para o Workspace:

```bash
cd nebulosaorion/ros2
```

### Instalar Dependências:

O projeto depende das seguintes dependências, principalmente para ferramentas de compilação, interfaces ROS e bibliotecas Python:

- ament_cmake  
- ament_cmake_python  
- rosidl_default_generators  
- builtin_interfaces  
- cv_bridge  
- sensor_msgs  
- rclpy  
- python3-opencv (opencv-python)  
- python3-numpy (numpy)  
- yasmin  
- yasmin_ros  

Você pode instalar as dependências relacionadas ao ROS usando rosdep:

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Para as dependências Python, instale manualmente com:

```bash
pip install opencv-python numpy yasmin yasmin-ros
```

### Construir o Pacote:

```bash
colcon build
```

### Configurar o Workspace:

```bash
source install/setup.bash
```

## ▶️ Uso e Comandos de Execução

Após construir e configurar seu workspace, você pode iniciar o media_publisher e testar suas funcionalidades.

### 1. Iniciando o Nó media_publisher

Este arquivo de lançamento iniciará o nó media_publisher e abrirá automaticamente o Foxglove Studio em seu navegador da web, configurado para conectar-se ao rosbridge_server.

```bash
ros2 launch pub_test media_publisher.launch.py
```

Após o lançamento bem-sucedido, você deverá ver o Foxglove Studio aberto. Você precisará adicionar um painel de "Imagem" no Foxglove e assiná-lo ao tópico `/ui_display` para ver a saída do media_publisher.

### 2. Enviando Mensagens para display_2

Use os comandos `ros2 topic pub` para enviar DisplayMessage para o tópico `display_2` e controlar o que é exibido no Foxglove.

#### a) Exibindo uma Frase:

```bash
ros2 topic pub /display_2 pub_test/msg/DisplayMessage "{type: 'sentence', value: 'Olá do Publicador de Mídia ROS2! Esta é uma frase de teste.'}" --once
```

#### b) Exibindo uma Imagem:

```bash
ros2 topic pub /display_2 pub_test/msg/DisplayMessage "{type: 'img', value: '/home/user/Pictures/minha_imagem.jpg'}" --once
```

#### c) Reproduzindo um Vídeo:

```bash
ros2 topic pub /display_2 pub_test/msg/DisplayMessage "{type: 'video', value: '/home/user/Videos/meu_video.mp4'}" --once
```

#### d) Exibindo Conteúdo de Outro Tópico ROS (por exemplo, do teste_publicador):

Em um novo terminal:

```bash
ros2 run pub_test teste_publicador
```

Depois:

```bash
ros2 topic pub /display_2 pub_test/msg/DisplayMessage "{type: 'topic', value: '/topico_exemplo'}" --once
```

> (Nota: `teste_publicador` publica `std_msgs/String`. O `handle_topic` em `publicador_midia.py` espera `sensor_msgs/Image`. Para que este exemplo funcione como esperado, o `teste_publicador` precisaria publicar `sensor_msgs/Image`, ou o `media_publisher` precisaria ser adaptado para processar mensagens `String`.)

Para ver a saída da imagem, certifique-se de ter um painel de Imagem no Foxglove Studio assinado em `/ui_display`.
