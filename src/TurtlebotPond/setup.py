from setuptools import find_packages, setup

package_name = 'TurtlebotPond'

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
    maintainer='caccioro',
    maintainer_email='antonio.guimaraes@sou.inteli.edu.br',
    description='Ponderada robo teleoperado Pt. 1',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtlebot = TurtlebotPond.Ponderada:main"
        ],
    },
)
