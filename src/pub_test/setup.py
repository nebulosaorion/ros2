from setuptools import find_packages, setup

package_name = 'pub_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evangelista',
    maintainer_email='evangelista@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'publicador_texto = pub_test.publicador_texto:main',
	    'publicador_video = pub_test.publicador_video:main',
        'text_publisher = pub_test.text_publisher:main',
        'publicador_midia = pub_test.publicador_midia:main',  
        ],
    },
)
