from setuptools import setup
from glob import glob

"""
This makes adapters installable when build the ROS workspace (colcon build
"""
package_name = 'ros_adapters'

setup(
    name=package_name,
    version='0.0.1',
    packages=['adapters'],      # This is the Python module folder
    install_requires=['setuptools'],
    zip_safe=True,

    # THIS installs launch files
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
    ],

    author='Leticia Berto',
    author_email='leticia.berto@manchester.ac.uk',
    maintainer='Leticia Berto',
    maintainer_email='leticia.berto@manchester.ac.uk',
    description='ROS 2 adapters for CASPER_MAS robots',
    license='Apache-2.0',
    #entry_points={
     #   'console_scripts': [
            # If want ROS nodes executable, add them here
    #    ],
    #},
)