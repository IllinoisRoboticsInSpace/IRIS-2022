from setuptools import setup
import os
from glob import glob

package_name = 'basic_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'hook'), glob("hook/*.sh")),
        # Path to the launch file      
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),

        # Path to the world file
        (os.path.join('share', package_name,'worlds/'), glob('./worlds/*')),

        # Path to model files
        (os.path.join('share', package_name,'models/arena_walls'), glob('./models/arena_walls/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iris',
    maintainer_email='iris@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
