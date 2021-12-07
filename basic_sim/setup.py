from setuptools import setup
import os
from glob import glob
from basic_sim import PACKAGE_NAME

def get_data_files():
    """ Returns the list of files that need to be copied to the install folder """
    files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ]
    files.extend(find_dir_files('hook'))
    files.extend(find_dir_files('launch'))
    files.extend(find_dir_files('worlds'))
    files.extend(find_dir_files('models'))
    files.extend(find_dir_files('config'))

    return files

def find_dir_files(dir_name):
    """ Finds and returns a list of files in the given directory RECURSIVELY in the data_files format
    Example list element:  
    (   
        'share/basic_sim/models/arena_walls', 
        ['models/arena_walls/model.config', 'models/arena_walls/model.sdf']
    )
    """
    file_list = []
    for filename in os.listdir(dir_name):
        filepath = os.path.join(dir_name, filename)
        glob_path = filepath
        target_dir = os.path.dirname(filepath)
        if os.path.isdir(filepath):
            file_list.extend(find_dir_files(filepath)) #recursive step
            # glob_path = os.path.join(filepath, '*')
            # target_dir = filepath
        else:
            file_list.append(
                (os.path.join('share', PACKAGE_NAME, target_dir), glob(f"{glob_path}"))
            )
    return file_list

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iris',
    maintainer_email='iris.uiuc@gmail.com',
    description='A basic ros2/gazebo simulation package for the NASA RMC Arena',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_model = basic_sim.ModelSpawner:main',
            'spawn_rock = basic_sim.ModelSpawner:spawn_rock_main'
        ],
    },
)
