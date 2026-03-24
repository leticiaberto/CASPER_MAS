from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'simulation'

def get_files_recursive(directory):
    paths = []
    for (dirpath, dirnames, filenames) in os.walk(directory):
        for filename in filenames:
            filepath = os.path.join(dirpath, filename)
            install_dir = os.path.join('share', package_name, dirpath)
            paths.append((install_dir, [filepath]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        *get_files_recursive('models'),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leticia Berto',
    maintainer_email='leticia.berto@manchester.ac.uk',
    description='Simulation package for Gazebo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
