from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'human_adapters'

def get_data_files_recursive(source_dir, install_base):
    """Walk source_dir and return data_files entries for every file found."""
    result = []
    for dirpath, dirnames, filenames in os.walk(source_dir):
        if not filenames:
            continue
        install_dir = os.path.join(
            install_base,
            os.path.relpath(dirpath, os.path.dirname(source_dir))
        )
        files = [os.path.join(dirpath, f) for f in filenames]
        result.append((install_dir, files))
    return result

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        # Recursively install all files under models/ preserving subdirectory
        # structure — needed for 'Walking Actor/model.sdf' and any meshes/textures.
        *get_data_files_recursive('models', os.path.join('share', package_name)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leticia Berto',
    maintainer_email='leticia.berto@manchester.ac.uk',
    description='ROS 2 adapters for CASPER_MAS robots',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'human_adapter = human_adapters.HumanAdapter:main',
            'actor_controller = human_adapters.actor_controller:main',
        ],
    },
)
