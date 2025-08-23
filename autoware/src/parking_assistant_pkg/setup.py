from setuptools import setup
import os
from glob import glob

package_name = 'parking_assistant_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    	# required package marker
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install package metadata
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='madhukar',
    maintainer_email='madhukar@todo.todo',
    description='Parking assistant GUI for Autoware',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_assistant_node = parking_assistant_pkg.parking_assistant_node:main',
        ],
    },
)

