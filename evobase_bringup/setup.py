from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'evobase_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/evobase_config.yaml']),
        ('share/' + package_name + '/config', ['config/tracked_config.yaml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'ament_index_python', 'pyserial', 'pyyaml'],
    zip_safe=True,
    maintainer='EvoMatrix',
    maintainer_email='you@example.com',
    description='Mobile robot bringup package for differential drive robot with serial communication',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'evobase_cmd_odom = evobase_bringup.evobase_cmd_odom:main',
            'tracked_cmd_odom = evobase_bringup.tracked_cmd_odom:main',
        ],
    },
)