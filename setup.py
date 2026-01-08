import os
from glob import glob
from setuptools import setup

package_name = 'obstacle_avoidance_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='md.uddin2@stud.fra-uas.de',
    description='Obstacle avoidance for TurtleBot3 using TEB and DWB planners',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = obstacle_avoidance_tb3.obstacle_avoidance:main',
        ],
    },
)
