# 🖼️ Media Publisher ROS2 - Sistema de Exibição para Foxglove

**Nó intermediário que processa mensagens de texto, imagens, vídeos e tópicos ROS, convertendo-os para exibição no Foxglove Studio.**


# 🧠 Conceito

Este projeto é baseado em uma máquina de estados construída com o framework [Yasmin](https://pypi.org/project/yasmin/). O nó `media_publisher` interpreta mensagens recebidas no tópico `display_2` contendo:

- `type`: tipo de mídia (`"sentence"`, `"img"`, `"video"` ou `"topic"`)
- `value`: conteúdo associado (texto, caminho do arquivo ou nome de tópico)

O resultado é renderizado como imagem e publicado em `ui_display`, que pode ser visualizado em tempo real no [Foxglove Studio](https://foxglove.dev/).

## 📦 Mensagem Personalizada

O tipo de mensagem `DisplayMessage.msg` contém:

```text
string type
string value

## 🧩 💬 Exemplos de mensagens

ros2 topic pub /display_2 pub_test/DisplayMessage "{type: 'sentence', value: 'Olá, mundo!'}"
ros2 topic pub /display_2 pub_test/DisplayMessage "{type: 'img', value: '/caminho/para/imagem.jpg'}"

## 🛠️ Estrutura do Código

  └── pub_test
        ├── CMakeLists.txt
        ├── docs
        │   └── fluxograma.png
        ├── launch
        │   └── media_publisher.launch.py # Configura ROSBridge + nó
        ├── msg
        │   └── DisplayMessage.msg # Define type/value
        ├── package.xml # Dependências
        ├── pub_test_pkg
        │   ├── __init__.py
        │   ├── publicador_midia.py # Lógica principal
        │   ├── __pycache__
        │   └── teste_publicador.py # Teste
        ├── README.md
        ├── resource
        │   └── pub_test
        ├── setup.cfg
        └── setup.py # Configuração Python


## 📦 Dependências
- ROS2 Humble (ou versão compatível)
- Pacotes:
  ```bash
  sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-rosbridge-suite

## 🚀 Como executar

***Pré-requisitos***

    ROS 2 (Humble ou mais recente)

    cv_bridge

    yasmin e yasmin_ros

    Foxglove Studio

## ⚠️ Compile o pacote:
cd ~/ros2_ws
colcon build --packages-select pub_test --symlink-install
source install/setup.bash

## Executar com ROSBridge e Foxglove
ros2 launch pub_test media_publisher.launch.py

→ Isso inicia o rosbridge_websocket na porta 9090

    Roda o nó publicador_midia

    Abre automaticamente o Foxglove Studio no navegador.


## 🧪 Teste
Execute o script de teste:

ros2 run pub_test teste_publicador

## 🛠️ Desenvolvimento

Pacote Python principal: pub_test_pkg

## Scripts:

    publicador_midia.py: nó principal

    teste_publicador.py: testes manuais ou automatizados

## 🐛 Solução de Problemas
"Unknown package 'pub_test'":

rm -rf build install log
colcon build --packages-select pub_test
source install/setup.bash

📄 Licença

Apache License 2.0

✉ Contato: Miriã Evangelista - evangelista@furg.br 