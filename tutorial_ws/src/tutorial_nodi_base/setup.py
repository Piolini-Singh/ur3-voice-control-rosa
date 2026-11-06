from setuptools import find_packages, setup

package_name = 'tutorial_nodi_base'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matteo Piolini, Jaspreet Singh',
    maintainer_email='m.piolini001@studenti.unibs.it',
    description='ROS 2 Humble nodes for UR3 joint-space and Cartesian (MoveIt 2) motion, '
                'with a keyboard/voice command interface on /comando_vocale.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        '01_simple_publisher = tutorial_nodi_base.01_simple_publisher:main',
        '02_simple_subscriber = tutorial_nodi_base.02_simple_subscriber:main',
        '03_coordinatore_sorting = tutorial_nodi_base.03_coordinatore_sorting:main',
        '04_muovi_ur3 = tutorial_nodi_base.04_muovi_ur3:main',
        '05_smistatore_ur3 = tutorial_nodi_base.05_smistatore_ur3:main',
        '06_verifica_raggiungibilita = tutorial_nodi_base.06_verifica_raggiungibilita:main',
        '07_esegui_movimento_cartesiano = tutorial_nodi_base.07_esegui_movimento_cartesiano:main',
        '08_interfaccia_continua = tutorial_nodi_base.08_interfaccia_continua:main',
        ],
    },
)
