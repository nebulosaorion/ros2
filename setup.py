from setuptools import setup
import os
from glob import glob

package_name = 'pub_test'
python_module_name = 'pub_test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Arquivos básicos do pacote
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        
        # Inclui todas as mensagens customizadas
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        
        # Inclui todos os arquivos de launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Inclui arquivos de parâmetros (se houver)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'yasmin',
        'yasmin_ros'
    ],
    zip_safe=False,
    entry_points={
        'console_scripts': [
            'publicador_midia = pub_test_pkg.publicador_midia:main',
            'teste_publicador = pub_test_pkg.teste_publicador:main',  # Adiciona o script de teste
        ],
    },

    
    author='Evangelista',
    author_email='evangelista@furg.br',
    description='Pacote para processamento de mídia no ROS2',
    license='Apache License 2.0',
    keywords=['ROS2', 'Foxglove', 'Media Processing'],
)