cmake_minimum_required(VERSION 3.5)
project(pub_test)

# Default to C++14 (mesmo que o código seja Python, é um padrão para pacotes ROS)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependências
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yasmin REQUIRED) # Tenta encontrar o yasmin
find_package(yasmin_ros REQUIRED) # Tenta encontrar o yasmin_ros (será resolvido pelo ROS se for um pacote)

# Mensagens customizadas
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DisplayMessage.msg"
  DEPENDENCIES builtin_interfaces
)

# Instalação do pacote Python
ament_python_install_package(pub_test_pkg)

# Executáveis Python - instala cada arquivo separadamente
install(
  PROGRAMS
    pub_test_pkg/publicador_midia.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME publicador_midia
)

install(
  PROGRAMS
    pub_test_pkg/teste_publicador.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teste_publicador
)

# Instala arquivos de launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()