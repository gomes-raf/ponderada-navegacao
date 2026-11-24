from setuptools import setup

package_name = 'navegacao'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rafael Gomes Ferreira',
    maintainer_email='rafael.gomes@sou.inteli.edu.br',
    description='Navegação autônoma no labirinto do Culling Games',
    license='MIT',
    entry_points={
        'console_scripts': [
            'parte1 = navegacao.parte1:main',
            'parte2 = navegacao.parte2:main',
        ],
    },
)
