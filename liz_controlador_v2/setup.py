from setuptools import setup, find_packages

package_name = 'liz_controlador_v2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lizbeth',
    maintainer_email='lizbeth@example.com',
    description='Controlador SeguidorGap y Contador de Vueltas para F1Tenth (mapa Budapest).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'seguidor_gap = liz_controlador_v2.seguidor_gap_node:main',
            'contador_vueltas = liz_controlador_v2.contador_vueltas_node:main',
        ],
    },
)

