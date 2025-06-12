from setuptools import setup
import os
from glob import glob

package_name = 'turtle_n_fly'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/resource', glob('resource/*')),

        

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Multi-robot simulation with TurtleBot3 and a DJI 2 Pro.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_controller = turtle_n_fly.drone_controller:main',
            'turtle_node = turtle_n_fly.turtle_node:main',
        ],
    },
)

