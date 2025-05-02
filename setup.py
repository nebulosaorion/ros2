from setuptools import setup
import os
from glob import glob

package_name = 'pub_test'

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
    install_requires=['setuptools'],
    zip_safe=False,
    entry_points={
        'console_scripts': [
            'publicador_midia = pub_test.publicador_midia:main',
        ],
    },
    # Metadados adicionais
    author='Evangelista',
    author_email='evangelista@todo.todo',
    description='Pacote para processamento de mídia no ROS2',
    license='Apache License 2.0',
    keywords=['ROS2', 'Foxglove', 'Media Processing'],
)