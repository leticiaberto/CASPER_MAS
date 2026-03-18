from glob import glob

from setuptools import find_packages, setup

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leticia Berto',
    maintainer_email='leticia.berto@manchester.ac.uk',
    description='Simulation package for Franka Emika Panda robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
