from setuptools import find_packages, setup


package_name = 'robot_common'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leticia Berto',
    maintainer_email='leticia.berto@manchester.ac.uk',
    description='Shared utilities for all robot adapters',
    license='Apache-2.0',
)
