from setuptools import setup

package_name = 'obstacle_avoidance_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='your@email.com',
    description='Obstacle avoidance for TurtleBot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = obstacle_avoidance_tb3.obstacle_avoidance:main',
        ],
    },
)

