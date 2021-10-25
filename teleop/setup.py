from setuptools import setup

package_name = 'teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iris-autonomous',
    maintainer_email='iris@illinois.edu',
    description='Teleop package for IRIS-2022',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = teleop.teleop_node:main', # name = package.file:method
        ],
    },
)
