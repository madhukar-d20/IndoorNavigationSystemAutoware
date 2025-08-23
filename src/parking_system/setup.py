from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'parking_system'

setup(
    name=package_name,
    version='0.0.0',
    # Use find_packages() to automatically discover your Python code
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Include the web dashboard files
        (os.path.join('share', package_name, 'web'), glob(os.path.join('web', '*'))),
    ],
    install_requires=['setuptools', 'flask'], # Add flask as a dependency
    zip_safe=True,
    maintainer='madhukar',
    maintainer_email='madhukar@todo.todo',
    description='A web dashboard and management system for autonomous parking with Autoware.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This entry point is now correct
            'integrated_dashboard_node = parking_system.integrated_dashboard_node:main',
        ],
    },
)
