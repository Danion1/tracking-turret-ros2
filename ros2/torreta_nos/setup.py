from setuptools import find_packages, setup

package_name = 'torreta_nos'

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
    maintainer='danion',
    maintainer_email='danion.neros@gmail.com',
    description='Nós da torreta de rastreio (escritos pelo Daniel): comando de mira e disparo',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'comando_torreta = torreta_nos.comando_torreta:main',
            'seguir_alvo = torreta_nos.seguir_alvo:main',
            'detectar_alvo = torreta_nos.detectar_alvo:main',
        ],
    },
)
